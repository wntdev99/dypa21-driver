"""
DYP E09 CAN 초음파 센서 허브 드라이버 (커스텀 CAN 버전)

RS485 인터페이스를 CAN으로 교체한 특주품.
제조사(George) 제공 프로토콜 기준.
"""

import json
import subprocess
import time
from dataclasses import dataclass, field
from typing import Optional
import can


# ──────────────────────────────────────────────────────────────────────────────
# 결과 데이터 클래스
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class DistanceResult:
    """E09 거리 측정 결과 (8포트)

    Attributes:
        ports:            8개 포트 거리값 리스트 (mm). 응답 없는 포트는 None.
        timestamp:        측정 완료 시각 (time.monotonic() 기준)
        elapsed_ms:       명령 전송 후 마지막 패킷 수신까지 소요 시간 (ms)
        packets_received: 수신된 응답 패킷 수 (정상 시 3)
        error:            오류 메시지. 정상 수신 시 None.
    """
    ports:            list
    timestamp:        float
    elapsed_ms:       float
    packets_received: int
    error:            Optional[str] = None

    @property
    def ok(self) -> bool:
        """3개 패킷 모두 정상 수신 여부."""
        return self.error is None and self.packets_received == 3

    def __str__(self) -> str:
        if self.error:
            return f"DistanceResult  [ERROR] {self.error}"

        lines = []
        for i, v in enumerate(self.ports, start=1):
            val = f"{v:5d} mm" if v is not None else "  --- mm"
            lines.append(f"  Port {i}: {val}")
        header = (
            f"DistanceResult  "
            f"[{self.elapsed_ms:6.1f} ms | "
            f"{self.packets_received}/3 packets]\n"
        )
        return header + "\n".join(lines)


# ──────────────────────────────────────────────────────────────────────────────
# E09 드라이버 클래스
# ──────────────────────────────────────────────────────────────────────────────

class E09:
    """DYP E09 커스텀 CAN 초음파 센서 허브 드라이버

    DYP E09 표준 모듈의 RS485(Main Interface 2)를 CAN으로 교체한 특주품.
    초음파 센서 최대 8개를 연결하여 거리 데이터를 CAN 버스로 제공.

    ※ 이 드라이버는 커스텀 CAN 버전 전용이며 표준 E09와 호환되지 않음.

    통신 규격 (제조사 George 제공):
      - Baud rate : 1 Mbps
      - CAN ID    : 0x0520 + slave address  (기본 0x0521)
      - 읽기 명령 : 단일 바이트 0x01
      - 응답 형식 : 3개의 CAN 패킷 (8포트 거리값, mm)
      - 동작 모드 : Mode 1  ─  8포트를 2그룹으로 나누어 순차 측정
                    그룹 A = 포트 1, 2, 3, 8
                    그룹 B = 포트 4, 5, 6, 7

    응답 패킷 구조:
      패킷 1: [0x03][0x01][P1_H][P1_L][P2_H][P2_L][P3_H][P3_L]  → 포트 1~3
      패킷 2: [0x03][0x02][P4_H][P4_L][P5_H][P5_L][P6_H][P6_L]  → 포트 4~6
      패킷 3: [0x03][0x03][P7_H][P7_L][P8_H][P8_L]               → 포트 7~8
      거리(mm) = (High << 8) | Low

    슬레이브 주소 변경 명령:
      [curr_addr][0x06][0x02][0x00][0x00][new_addr]  → CAN ID 0x0520 + new_addr

    폴링 주기 근거 (E09 데이터시트 Mode 1 응답 시간 공식):
      최대 응답 시간 = 28 + 2 × timeout(기본 200ms) = 428ms
      → 안전 폴링 주기: 500ms (2 Hz)

    사용 예시:
        with E09('can0') as hub:
            result = hub.read_distances()
            print(result)

        # 연속 측정
        with E09('can0') as hub:
            for result in hub.continuous_read(count=10):
                print(result)
    """

    CAN_BASE_ID    = 0x0520   # CAN ID 기준값 (+ slave addr)
    CMD_READ       = 0x01     # 거리 읽기 트리거 명령
    FUNC_WRITE     = 0x06     # Write 기능 코드 (슬레이브 주소 변경 시 사용)
    REG_SLAVE_ADDR = 0x0200   # 슬레이브 주소 레지스터

    NUM_PORTS    = 8   # 총 포트 수
    NUM_PACKETS  = 3   # 응답 패킷 수

    # Mode 1 기준 타이밍 (E09 데이터시트, 레지스터 0x0216)
    # 최대 응답시간: 28 + 2×200 = 428ms → RESPONSE_TIMEOUT: 450ms
    RESPONSE_TIMEOUT_S = 0.450   # 패킷 수신 대기 최대 시간 (초)
    POLL_INTERVAL_S    = 0.500   # 권장 최소 폴링 주기 (초)

    def __init__(
        self,
        channel: str = 'can0',
        slave_address: int = 0x01,
        bitrate: int = 1_000_000,
        bustype: str = 'socketcan',
    ):
        """
        Args:
            channel:       CAN 인터페이스 이름  (예: 'can0')
            slave_address: E09 슬레이브 주소    (기본값 0x01, CAN ID = 0x0521)
            bitrate:       CAN 비트레이트       (기본값 1 Mbps)
            bustype:       python-can 버스 타입 (기본값 'socketcan')
        """
        if not 0x01 <= slave_address <= 0xFE:
            raise ValueError(f"슬레이브 주소는 0x01~0xFE 사이여야 합니다. (입력값: {slave_address:#04x})")
        self._channel  = channel
        self._slave    = slave_address
        self._bitrate  = bitrate
        self._bustype  = bustype
        self._bus: Optional[can.BusABC] = None

    # ── 프로퍼티 ───────────────────────────────────────────────────────────────

    @property
    def can_id(self) -> int:
        """현재 슬레이브 주소에 해당하는 CAN ID."""
        return self.CAN_BASE_ID + self._slave

    @property
    def slave_address(self) -> int:
        """현재 슬레이브 주소."""
        return self._slave

    # ── 연결 관리 ──────────────────────────────────────────────────────────────

    @staticmethod
    def setup_interface(
        channel:    str = 'can0',
        bitrate:    int = 1_000_000,
        restart_ms: int = 1_000,
    ) -> None:
        """SocketCAN 인터페이스를 항상 깨끗하게 재설정하고 활성화.

        이미 UP 상태여도 일단 내리고 다시 올림.
        Bus-Off 자동 복구를 위해 restart-ms 옵션을 포함하여 설정.

        Args:
            channel:    CAN 인터페이스 이름 (기본값 'can0')
            bitrate:    CAN 비트레이트 (기본값 1 Mbps)
            restart_ms: Bus-Off 발생 시 자동 재시작 대기 시간 ms
                        (기본값 1000ms. 0이면 자동 재시작 비활성화)

        Raises:
            OSError: 인터페이스가 존재하지 않거나 설정에 실패한 경우.

        수동 실행 참고:
            sudo ip link set can0 down
            sudo ip link set can0 type can bitrate 1000000 restart-ms 1000
            sudo ip link set can0 up
        """
        _MANUAL = (
            f"  수동으로 실행하세요:\n"
            f"    sudo ip link set {channel} down\n"
            f"    sudo ip link set {channel} type can bitrate {bitrate}"
            f" restart-ms {restart_ms}\n"
            f"    sudo ip link set {channel} up"
        )

        # ── 1. 인터페이스 존재 확인 ─────────────────────────────────────────
        ret = subprocess.run(
            ['ip', '-json', 'link', 'show', channel],
            capture_output=True, text=True,
        )
        if ret.returncode != 0 or not ret.stdout.strip():
            raise OSError(
                f"CAN 인터페이스 '{channel}'를 찾을 수 없습니다.\n"
                f"  USB-to-CAN 어댑터가 시스템에 인식되어 있는지 확인하세요:\n"
                f"    ip link show"
            )

        link  = json.loads(ret.stdout)[0]
        flags = link.get('flags', [])

        # ── 2. 인터페이스 비활성화 (UP이면 내려야 재설정 가능) ──────────────
        if 'UP' in flags:
            print(f"[setup] '{channel}' Bus-Off 복구를 위해 재설정합니다...")
            r = subprocess.run(
                ['sudo', 'ip', 'link', 'set', channel, 'down'],
                stderr=subprocess.PIPE, text=True,
            )
            if r.returncode != 0:
                raise OSError(
                    f"인터페이스 비활성화 실패.\n"
                    f"  오류: {r.stderr.strip()}\n"
                    + _MANUAL
                )
        else:
            print(f"[setup] '{channel}' 비활성 상태 → 설정 및 활성화 시도...")

        # ── 3. bitrate + restart-ms 설정 ────────────────────────────────────
        cmd = ['sudo', 'ip', 'link', 'set', channel,
               'type', 'can', 'bitrate', str(bitrate)]
        if restart_ms > 0:
            cmd += ['restart-ms', str(restart_ms)]

        r = subprocess.run(cmd, stderr=subprocess.PIPE, text=True)
        if r.returncode != 0:
            raise OSError(
                f"인터페이스 설정 실패.\n"
                f"  오류: {r.stderr.strip()}\n"
                + _MANUAL
            )

        # ── 4. 인터페이스 활성화 ────────────────────────────────────────────
        r = subprocess.run(
            ['sudo', 'ip', 'link', 'set', channel, 'up'],
            stderr=subprocess.PIPE, text=True,
        )
        if r.returncode != 0:
            raise OSError(
                f"인터페이스 활성화 실패.\n"
                f"  오류: {r.stderr.strip()}\n"
                + _MANUAL
            )

        restart_info = f", restart-ms={restart_ms}ms" if restart_ms > 0 else ""
        print(f"[setup] '{channel}' 활성화 완료 (bitrate={bitrate:,} bps{restart_info})")

    def connect(self, auto_setup: bool = True) -> "E09":
        """CAN 버스 연결.

        with 문 없이 수동 사용 시 호출.

        Args:
            auto_setup: True이면 인터페이스가 DOWN 상태일 때 자동으로
                        설정 및 활성화를 시도함 (sudo 권한 필요, 기본값: True).
        """
        if auto_setup:
            self.setup_interface(self._channel, self._bitrate)

        try:
            self._bus = can.interface.Bus(
                channel=self._channel,
                interface=self._bustype,
                bitrate=self._bitrate,
            )
        except OSError as e:
            raise OSError(
                f"CAN 인터페이스 '{self._channel}' 연결 실패: {e}"
            ) from e
        return self

    def disconnect(self):
        """CAN 버스 연결 해제."""
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    def __enter__(self) -> "E09":
        return self.connect()

    def __exit__(self, *_):
        self.disconnect()

    def _require_connection(self):
        if self._bus is None:
            raise RuntimeError("CAN 버스가 연결되어 있지 않습니다. connect()를 먼저 호출하세요.")

    # ── 거리 읽기 ──────────────────────────────────────────────────────────────

    def read_distances(self) -> DistanceResult:
        """8개 포트 거리값 읽기.

        0x01 명령 전송 → E09가 Mode 1으로 측정 후 3패킷 응답.
        RESPONSE_TIMEOUT_S(450ms) 내에 수신된 패킷만 파싱하여 반환.
        미수신 포트는 None.

        Returns:
            DistanceResult: 8포트 거리(mm) 및 측정 메타데이터.
        """
        self._require_connection()

        msg = can.Message(
            arbitration_id=self.can_id,
            data=[self.CMD_READ],
            is_extended_id=False,
        )
        t_send = time.monotonic()
        try:
            self._bus.send(msg)
        except can.CanOperationError as e:
            # Errno 105 (ENOBUFS): CAN TX 큐 포화 → Bus-Off 상태.
            # E09가 연결/전원 OFF이거나 CAN 배선 문제일 때 발생.
            # setup_interface()가 restart-ms로 자동 복구 예정.
            elapsed_ms = (time.monotonic() - t_send) * 1000
            return DistanceResult(
                ports=[None] * self.NUM_PORTS,
                timestamp=time.monotonic(),
                elapsed_ms=elapsed_ms,
                packets_received=0,
                error=f"CAN 전송 실패 (Bus-Off 또는 배선 문제): {e}",
            )

        ports: list = [None] * self.NUM_PORTS
        received: set = set()
        deadline = t_send + self.RESPONSE_TIMEOUT_S

        while len(received) < self.NUM_PACKETS:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break

            frame = self._bus.recv(timeout=remaining)
            if frame is None:
                break
            if not self._is_response_frame(frame):
                continue

            seq = frame.data[1]
            if seq in received:
                continue

            self._parse_packet(bytes(frame.data), ports)
            received.add(seq)

        elapsed_ms = (time.monotonic() - t_send) * 1000
        return DistanceResult(
            ports=ports,
            timestamp=time.monotonic(),
            elapsed_ms=elapsed_ms,
            packets_received=len(received),
        )

    def _is_response_frame(self, frame: can.Message) -> bool:
        """응답 패킷 유효성 검사.

        조건: 동일 CAN ID, 헤더 바이트 0x03, 시퀀스 번호 1~3.
        """
        data = frame.data
        return (
            frame.arbitration_id == self.can_id
            and len(data) >= 4
            and data[0] == 0x03
            and data[1] in (1, 2, 3)
        )

    @staticmethod
    def _parse_packet(data: bytes, ports: list) -> None:
        """CAN 패킷 1개를 파싱하여 ports 리스트 갱신.

        패킷 시퀀스 → 포트 인덱스 매핑:
          seq=1: 포트 1~3  (인덱스 0~2), 페이로드 6바이트
          seq=2: 포트 4~6  (인덱스 3~5), 페이로드 6바이트
          seq=3: 포트 7~8  (인덱스 6~7), 페이로드 4바이트

        Args:
            data:  CAN 프레임의 data 필드 (bytes)
            ports: 갱신 대상 포트 리스트 (len=8)
        """
        seq     = data[1]
        payload = data[2:]   # 헤더(0x03)와 시퀀스 바이트 이후

        if seq == 1:
            base_idx, count = 0, 3
        elif seq == 2:
            base_idx, count = 3, 3
        else:   # seq == 3
            base_idx, count = 6, 2

        for i in range(count):
            offset = i * 2
            if len(payload) >= offset + 2:
                ports[base_idx + i] = (payload[offset] << 8) | payload[offset + 1]

    # ── 연속 측정 ──────────────────────────────────────────────────────────────

    def continuous_read(
        self,
        count: Optional[int] = None,
        interval: float = POLL_INTERVAL_S,
    ):
        """지정 주기로 거리를 반복 측정하는 제너레이터.

        Args:
            count:    측정 횟수. None이면 KeyboardInterrupt까지 무한 반복.
            interval: 폴링 주기 (초). 기본값 POLL_INTERVAL_S(0.5s).
                      POLL_INTERVAL_S 미만으로 설정 시 경고를 출력함.

        Yields:
            DistanceResult: 각 측정 결과.

        사용 예시:
            for result in hub.continuous_read(count=20):
                print(result)
        """
        self._require_connection()

        if interval < self.POLL_INTERVAL_S:
            print(
                f"[경고] 폴링 주기 {interval*1000:.0f}ms는 권장값 "
                f"{self.POLL_INTERVAL_S*1000:.0f}ms 미만입니다. "
                f"패킷 손실이 발생할 수 있습니다."
            )

        i = 0
        while count is None or i < count:
            t_start = time.monotonic()
            yield self.read_distances()
            i += 1

            elapsed = time.monotonic() - t_start
            sleep_s = interval - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

    # ── 슬레이브 주소 스캔 / 변경 ────────────────────────────────────────────

    def scan(
        self,
        start: int = 0x01,
        end:   int = 0x20,
        timeout_per_addr: float = 0.6,
    ) -> Optional[int]:
        """CAN 버스를 순차 탐색하여 E09 슬레이브 주소 검색.

        각 주소에 거리 읽기 명령(0x01)을 전송하고 유효한 응답 패킷
        (data[0] == 0x03, data[1] in 1~3) 이 돌아오면 해당 주소를 반환.
        George가 확인한 프로토콜(0x01 명령 / 0x03 응답 헤더)만 사용.

        Args:
            start:            스캔 시작 주소 (기본: 0x01)
            end:              스캔 종료 주소 (기본: 0x20)
            timeout_per_addr: 주소당 응답 대기 시간 초 (기본: 0.6s).
                              Mode 1 최대 응답 시간(428ms) 보다 여유 있게 설정.

        Returns:
            발견된 슬레이브 주소 (int). 범위 내에서 응답 없으면 None.
        """
        self._require_connection()

        total = end - start + 1
        for idx, addr in enumerate(range(start, end + 1), start=1):
            test_id = self.CAN_BASE_ID + addr
            print(f"\r  스캔 중... [{idx}/{total}] CAN ID {test_id:#05x} (slave {addr:#04x})",
                  end='', flush=True)

            msg = can.Message(
                arbitration_id=test_id,
                data=[self.CMD_READ],
                is_extended_id=False,
            )
            try:
                self._bus.send(msg)
            except can.CanOperationError:
                continue

            # 첫 번째 유효 응답 패킷이 오면 이 주소가 현재 슬레이브
            deadline = time.monotonic() + timeout_per_addr
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                frame = self._bus.recv(timeout=remaining)
                if frame is None:
                    break
                if (
                    frame.arbitration_id == test_id
                    and len(frame.data) >= 2
                    and frame.data[0] == 0x03
                    and frame.data[1] in (1, 2, 3)
                ):
                    print()   # 줄바꿈
                    return addr

        print()   # 줄바꿈
        return None

    def set_slave_address(self, new_address: int) -> bool:
        """E09 슬레이브 주소(CAN ID) 변경 명령 전송.

        ※ 변경은 E09 모듈 전원을 껐다 켜야 적용됨.
           명령 전송 성공 여부만 반환하며, 즉시 검증하지 않음.

        명령 포맷 (제조사 예시 기준: 01 06 02 00 00 05):
          [curr_addr][0x06][0x02][0x00][0x00][new_addr]

        Args:
            new_address: 새 슬레이브 주소 (0x01~0xFE)

        Returns:
            True:  명령 전송 성공
            False: 전송 실패 (Bus-Off 등)
        """
        self._require_connection()
        if not 0x01 <= new_address <= 0xFE:
            raise ValueError(f"슬레이브 주소는 0x01~0xFE 사이여야 합니다. (입력값: {new_address:#04x})")
        if new_address == self._slave:
            return True

        data = bytes([
            self._slave,
            self.FUNC_WRITE,
            (self.REG_SLAVE_ADDR >> 8) & 0xFF,   # 0x02
            self.REG_SLAVE_ADDR & 0xFF,           # 0x00
            0x00,
            new_address,
        ])
        msg = can.Message(
            arbitration_id=self.can_id,
            data=data,
            is_extended_id=False,
        )
        try:
            self._bus.send(msg)
        except can.CanOperationError as e:
            print(f"[set_slave_address] 전송 실패: {e}")
            return False

        return True

    # ── 생존 확인 ──────────────────────────────────────────────────────────────

    def is_alive(self, timeout: float = POLL_INTERVAL_S) -> bool:
        """E09 모듈 생존 확인.

        거리 읽기 명령(0x01)을 전송하고 timeout 내에 유효한 응답 패킷이
        1개 이상 수신되면 True 반환.

        ※ 전용 Heartbeat 명령이 없어 거리 읽기 명령으로 대체.
           제조사(George)로부터 별도 명령이 제공될 경우 업데이트 필요.

        Args:
            timeout: 응답 대기 최대 시간 (초, 기본값 0.5s)

        Returns:
            True: E09 응답 정상 확인, False: timeout 내 응답 없음
        """
        self._require_connection()

        msg = can.Message(
            arbitration_id=self.can_id,
            data=[self.CMD_READ],
            is_extended_id=False,
        )
        self._bus.send(msg)
        deadline = time.monotonic() + timeout

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            frame = self._bus.recv(timeout=remaining)
            if frame is None:
                return False
            if self._is_response_frame(frame):
                return True

    # ── 상태 정보 ──────────────────────────────────────────────────────────────

    def info(self) -> dict:
        """현재 드라이버 설정 정보를 딕셔너리로 반환."""
        return {
            "channel":              self._channel,
            "bustype":              self._bustype,
            "bitrate_bps":          self._bitrate,
            "slave_address":        hex(self._slave),
            "can_id":               hex(self.can_id),
            "response_timeout_ms":  self.RESPONSE_TIMEOUT_S * 1000,
            "poll_interval_ms":     self.POLL_INTERVAL_S * 1000,
        }


# ──────────────────────────────────────────────────────────────────────────────
# 직접 실행 시 연속 거리 측정 데모
# ──────────────────────────────────────────────────────────────────────────────

def _demo():
    """연속 거리 측정 / CAN ID 변경 테스트 데모 (Ctrl+C로 종료)."""
    import argparse

    parser = argparse.ArgumentParser(description="E09 CAN 드라이버 데모")
    parser.add_argument("--channel",   default="can0",
                        help="CAN 인터페이스 (기본: can0)")
    parser.add_argument("--slave",     type=lambda x: int(x, 0), default=0x01,
                        help="슬레이브 주소 (기본: 0x01, 거리 측정 모드에서 사용)")
    parser.add_argument("--interval",  type=float, default=E09.POLL_INTERVAL_S,
                        help=f"폴링 주기 초 (기본: {E09.POLL_INTERVAL_S}s)")
    parser.add_argument("--count",     type=int, default=None,
                        help="거리 측정 횟수 (기본: 무한)")

    # 스캔 / CAN ID 변경 옵션
    parser.add_argument("--scan",      action="store_true",
                        help="CAN 버스를 스캔하여 현재 슬레이브 주소 탐색 후 종료")
    parser.add_argument("--scan-end",  type=lambda x: int(x, 0), default=0x20,
                        metavar="END_ADDR",
                        help="스캔 종료 주소 (기본: 0x20, --scan / --set-addr 공통)")
    parser.add_argument("--set-addr",  type=lambda x: int(x, 0), default=None,
                        metavar="NEW_ADDR",
                        help="현재 주소를 스캔으로 찾아 변경 후 종료 (예: --set-addr 0x02)")
    args = parser.parse_args()

    print("E09 CAN 드라이버 데모")
    print(f"  채널:     {args.channel}")

    try:
        # 스캔/변경 모드는 초기 --slave 값 무관하게 0x01부터 접속
        init_slave = 0x01 if (args.scan or args.set_addr is not None) else args.slave
        with E09(channel=args.channel, slave_address=init_slave) as hub:

            # ── 스캔 모드 ────────────────────────────────────────────────────
            if args.scan:
                print(f"  모드:     슬레이브 주소 스캔 (0x01 ~ {args.scan_end:#04x})")
                print("-" * 50)
                found = hub.scan(start=0x01, end=args.scan_end)
                if found is not None:
                    print(f"  발견: 슬레이브 주소 {found:#04x}"
                          f"  (CAN ID: {E09.CAN_BASE_ID + found:#05x})")
                else:
                    print(f"  응답 없음: 0x01~{args.scan_end:#04x} 범위에서 E09를 찾지 못했습니다.")
                    print(f"  --scan-end 값을 높이거나 배선/전원을 확인하세요.")
                return

            # ── CAN ID 변경 모드 ─────────────────────────────────────────────
            if args.set_addr is not None:
                new_addr   = args.set_addr
                new_can_id = E09.CAN_BASE_ID + new_addr

                print(f"  모드:     CAN ID 변경 (목표: {new_addr:#04x},"
                      f" CAN ID: {new_can_id:#05x})")
                print("-" * 50)

                # 1단계: 스캔으로 현재 주소 탐색
                print(f"[1/3] 현재 슬레이브 주소 탐색 중 (0x01 ~ {args.scan_end:#04x})...")
                current_addr = hub.scan(start=0x01, end=args.scan_end)
                if current_addr is None:
                    print(f"      응답 없음: E09를 찾지 못했습니다. 배선/전원을 확인하세요.")
                    return
                old_can_id = E09.CAN_BASE_ID + current_addr
                print(f"      현재: {current_addr:#04x}  (CAN ID: {old_can_id:#05x})")

                if current_addr == new_addr:
                    print(f"      이미 목표 주소({new_addr:#04x})와 동일합니다. 변경 불필요.")
                    return

                # 2단계: 발견된 현재 주소로 드라이버 주소 전환 후 변경 명령 전송
                hub._slave = current_addr
                print(f"[2/3] 주소 변경 명령 전송 ({current_addr:#04x} → {new_addr:#04x})...")
                ok = hub.set_slave_address(new_addr)
                if not ok:
                    print("      실패: 명령 전송 중 오류 발생.")
                    return
                print("      전송 완료.")

                # 3단계: 전원 재시작 안내
                print(f"[3/3] E09 모듈의 전원을 껐다 켜주세요.")
                print(f"      재시작 후 새 CAN ID {new_can_id:#05x} 로 통신하세요:")
                print(f"        python3 scripts/e09.py --slave {new_addr:#04x}")
                return

            # ── 연속 거리 측정 모드 ──────────────────────────────────────────
            print(f"  Slave:    {args.slave:#04x}  (CAN ID: {E09.CAN_BASE_ID + args.slave:#05x})")
            print(f"  주기:     {args.interval * 1000:.0f} ms")
            print(f"  횟수:     {'무한' if args.count is None else args.count}")
            print("-" * 50)

            n = 0
            for result in hub.continuous_read(count=args.count, interval=args.interval):
                n += 1
                print(f"[{n:04d}] {result}")
                print()

    except OSError as e:
        print(f"[오류] {e}")
    except KeyboardInterrupt:
        print("\n종료.")


if __name__ == "__main__":
    _demo()
