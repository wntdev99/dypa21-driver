"""
DYP-A21 초음파 센서 드라이버
출처: DYP-A21-Output-Interfaces-Keila.pdf
"""

import struct
import serial
from enum import IntEnum
from typing import Optional


class AngleLevel(IntEnum):
    """지향각 레벨 (datasheet p.10, 레지스터 0x0208)

    Level이 클수록 지향각이 넓고 감도가 높음. 단, 감지 범위는 좁아짐.
    """
    NARROW      = 1   # 단일각 ~50°,  수평 ~50°,  수직 ~65°
    MEDIUM_LOW  = 2   # 단일각 ~55°,  수평 ~55°,  수직 ~70°
    MEDIUM_HIGH = 3   # 단일각 ~65°,  수평 ~60°,  수직 ~75°
    WIDE        = 4   # 단일각 ~70°,  수평 ~65°,  수직 ~90° (기본값)


class ScaleGrade(IntEnum):
    """측정 거리 범위 레벨 (datasheet p.11, 레지스터 0x021F)

    실시간 응답시간 / 처리 응답시간 기준.
    """
    CM_50  = 1   # ~50cm,  실시간 15~80ms,   처리 190~500ms
    CM_150 = 2   # ~150cm, 실시간 20~90ms,   처리 230~550ms
    CM_250 = 3   # ~250cm, 실시간 25~100ms,  처리 250~600ms
    CM_350 = 4   # ~350cm, 실시간 35~110ms,  처리 280~650ms
    CM_500 = 5   # ~500cm, 실시간 40~140ms,  처리 320~750ms (기본값)


class OutputUnit(IntEnum):
    """거리 출력 단위 (datasheet p.10, 레지스터 0x0209)

    UART automatic / UART controlled 모드에서만 유효.
    """
    MM = 0x00   # 밀리미터 (기본값)
    US = 0x01   # 마이크로초 (mm 변환: ÷5.75)


class PowerNoiseLevel(IntEnum):
    """전원 노이즈 감소 레벨 (datasheet p.11, 레지스터 0x021A)

    Level이 높을수록 노이즈 억제 강화. 단, 지향각이 넓어지고
    작은 신호의 물체는 감지 안 될 수 있음.
    """
    BATTERY     = 1   # 배터리 구동 (기본값)
    USB         = 2   # USB 전원 (고주파 노이즈 환경)
    USB_LONG    = 3   # 장거리 USB 전원
    SWITCHING   = 4   # 스위칭 전원
    COMPLEX     = 5   # 복잡한 간섭 환경 (비권장)


class DYPA21:
    """
    DYP-A21 초음파 센서 드라이버

    지원 기능:
      - UART Controlled 트리거 측정 (trigger)
      - Modbus RTU 레지스터 읽기/쓰기
      - 지향각, 측정 거리 범위, 노이즈 레벨 설정
      - 실시간/처리 거리값, 온도, 에코 시간 읽기
      - 스위치 출력 임계값 / 극성 설정
      - Slave 주소 / Baud rate 변경

    ※ UART Controlled 모드에서 Modbus 설정 명령은
       전원 인가 후 500ms 이내에만 유효. (datasheet p.08)

    사용 예시:
        with DYPA21('/dev/ttyUSB0') as sensor:
            print(sensor.trigger())          # 트리거 측정 (mm)
            print(sensor.get_temperature())  # 온도 (°C)
            sensor.set_detection_angle(AngleLevel.NARROW)
    """

    # ── Modbus Read-only 레지스터 ──────────────────────────────────────────
    REG_PROCESSING_VALUE = 0x0100   # 알고리즘 처리 거리 (mm)
    REG_REALTIME_VALUE   = 0x0101   # 실시간 거리 (mm)
    REG_TEMPERATURE      = 0x0102   # 온도 (signed int, 단위 0.1°C)
    REG_ECHO_TIME        = 0x010A   # 에코 시간 (us, ÷5.75 → mm)

    # ── Modbus Read-write 레지스터 ─────────────────────────────────────────
    REG_SLAVE_ADDRESS    = 0x0200   # Slave 주소 (0x01~0xFE)
    REG_BAUD_RATE        = 0x0201   # Baud rate 코드
    REG_SWITCH_POLARITY  = 0x0205   # 스위치 출력 극성
    REG_SWITCH_THRESHOLD = 0x0206   # 스위치 임계값 (mm, 30~5000)
    REG_DETECTION_ANGLE  = 0x0208   # 지향각 레벨 (1~4)
    REG_OUTPUT_UNIT      = 0x0209   # 출력 단위 (0=mm, 1=us)
    REG_POWER_NOISE      = 0x021A   # 전원 노이즈 감소 레벨 (1~5)
    REG_SCALE_GRADE      = 0x021F   # 측정 거리 범위 레벨 (1~5)

    # Baud rate 코드 테이블 (datasheet p.10, 레지스터 0x0201)
    _BAUD_CODE = {
        9600:   0x0002,
        19200:  0x0003,
        38400:  0x0004,
        57600:  0x0007,
        115200: 0x0009,
    }
    _CODE_BAUD = {v: k for k, v in _BAUD_CODE.items()}

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        slave_address: int = 0x01,
        timeout: float = 1.0,
    ):
        """
        Args:
            port:          시리얼 포트 (예: '/dev/ttyUSB0')
            baud:          Baud rate (기본값 115200)
            slave_address: Modbus Slave 주소 (기본값 0x01)
            timeout:       시리얼 읽기 타임아웃 초 (기본값 1.0)
        """
        self._port    = port
        self._baud    = baud
        self._slave   = slave_address
        self._timeout = timeout
        self._ser: Optional[serial.Serial] = None

    # ──────────────────────────────────────────────────────────────────────
    # 연결 관리
    # ──────────────────────────────────────────────────────────────────────

    def connect(self) -> "DYPA21":
        """시리얼 포트 연결. with 문 없이 수동으로 사용할 때 호출."""
        self._ser = serial.Serial(self._port, self._baud, timeout=self._timeout)
        return self

    def disconnect(self):
        """시리얼 포트 해제."""
        if self._ser and self._ser.is_open:
            self._ser.close()

    def __enter__(self) -> "DYPA21":
        return self.connect()

    def __exit__(self, *args):
        self.disconnect()

    def _require_connection(self):
        if self._ser is None or not self._ser.is_open:
            raise RuntimeError("센서가 연결되어 있지 않습니다. connect()를 먼저 호출하세요.")

    # ──────────────────────────────────────────────────────────────────────
    # UART Controlled 트리거 측정
    # ──────────────────────────────────────────────────────────────────────

    def trigger(self) -> Optional[int]:
        """
        트리거(0xFF) 전송 후 거리값 반환 (mm).
        체크섬 오류 또는 응답 없음 시 None 반환.

        트리거 주기는 반드시 150ms 이상이어야 함. (datasheet p.02)
        응답 포맷: [0xFF][Data_H][Data_L][SUM]  (datasheet p.02)
        """
        self._require_connection()
        self._ser.write(b'\xff')
        data = self._ser.read(4)

        if len(data) != 4 or data[0] != 0xFF:
            return None

        distance = (data[1] << 8) + data[2]
        checksum  = (data[0] + data[1] + data[2]) & 0xFF

        return distance if checksum == data[3] else None

    # ──────────────────────────────────────────────────────────────────────
    # Modbus RTU 내부 유틸리티
    # ──────────────────────────────────────────────────────────────────────

    @staticmethod
    def _crc16(data: bytes) -> bytes:
        """Modbus RTU CRC-16 계산 (Low byte first). (datasheet p.09)"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        return struct.pack('<H', crc)

    def _modbus_read(self, register: int, signed: bool = False) -> Optional[int]:
        """
        Modbus RTU Read (Function Code 0x03), 1 레지스터(2바이트) 읽기.
        응답: [addr(1)][0x03(1)][byte_count(1)][data(2)][crc(2)] = 7바이트
        (datasheet p.09)
        """
        self._require_connection()
        payload = struct.pack('>BBHH', self._slave, 0x03, register, 1)
        self._ser.write(payload + self._crc16(payload))

        resp = self._ser.read(7)
        if len(resp) != 7:
            return None
        if resp[5:7] != self._crc16(resp[:5]):
            return None

        fmt = '>h' if signed else '>H'
        return struct.unpack(fmt, resp[3:5])[0]

    def _modbus_write(self, register: int, value: int) -> bool:
        """
        Modbus RTU Write (Function Code 0x06), 1 레지스터 쓰기.
        응답: [addr(1)][0x06(1)][reg(2)][val(2)][crc(2)] = 8바이트 에코
        (datasheet p.09)
        """
        self._require_connection()
        payload = struct.pack('>BBHH', self._slave, 0x06, register, value)
        self._ser.write(payload + self._crc16(payload))

        resp = self._ser.read(8)
        if len(resp) != 8:
            return False
        if resp[6:8] != self._crc16(resp[:6]):
            return False
        return resp[2:6] == payload[2:6]   # 레지스터 주소 + 값 에코 확인

    # ──────────────────────────────────────────────────────────────────────
    # 거리 / 온도 읽기 (Modbus)
    # ──────────────────────────────────────────────────────────────────────

    def get_processing_distance(self) -> Optional[int]:
        """
        알고리즘 처리 거리 읽기 (mm).
        응답 시간 190~750ms (거리 범위에 따라 다름). (datasheet p.10, 0x0100)
        """
        return self._modbus_read(self.REG_PROCESSING_VALUE)

    def get_realtime_distance(self) -> Optional[int]:
        """
        실시간 거리 읽기 (mm).
        응답 시간 15~140ms (거리 범위에 따라 다름). (datasheet p.10, 0x0101)
        """
        return self._modbus_read(self.REG_REALTIME_VALUE)

    def get_temperature(self) -> Optional[float]:
        """
        온도 읽기 (°C). 분해능 0.5°C. (datasheet p.10, 0x0102)
        내부 raw 값에 0.1을 곱하여 반환.
        """
        raw = self._modbus_read(self.REG_TEMPERATURE, signed=True)
        return raw * 0.1 if raw is not None else None

    def get_echo_time_mm(self) -> Optional[float]:
        """
        에코 시간(us)을 읽어 mm 거리로 변환하여 반환.
        변환식: mm = us ÷ 5.75 (datasheet p.10, 0x010A)
        응답 시간 5~140ms.
        """
        us = self._modbus_read(self.REG_ECHO_TIME)
        return us / 5.75 if us is not None else None

    # ──────────────────────────────────────────────────────────────────────
    # 설정: 지향각 (Detection Angle Level)
    # ──────────────────────────────────────────────────────────────────────

    def set_detection_angle(self, level: AngleLevel) -> bool:
        """
        지향각 레벨 설정. (datasheet p.10, 0x0208)

        AngleLevel.NARROW(1) ~ AngleLevel.WIDE(4, 기본값).
        Level이 클수록 지향각이 넓고 감도가 높음.

        ※ UART Controlled 모드: 전원 인가 후 500ms 이내에만 유효.
        """
        level = int(level)
        if not 1 <= level <= 4:
            raise ValueError(f"지향각 레벨은 1~4 사이여야 합니다. (입력값: {level})")
        return self._modbus_write(self.REG_DETECTION_ANGLE, level)

    def get_detection_angle(self) -> Optional[int]:
        """현재 지향각 레벨 읽기. (datasheet p.10, 0x0208)"""
        return self._modbus_read(self.REG_DETECTION_ANGLE)

    # ──────────────────────────────────────────────────────────────────────
    # 설정: 측정 거리 범위 (Scale Grade)
    # ──────────────────────────────────────────────────────────────────────

    def set_scale_grade(self, level: ScaleGrade) -> bool:
        """
        측정 거리 범위 설정. (datasheet p.11, 0x021F)

        ScaleGrade.CM_50(1) ~ ScaleGrade.CM_500(5, 기본값).

        ※ UART Controlled 모드: 전원 인가 후 500ms 이내에만 유효.
        """
        level = int(level)
        if not 1 <= level <= 5:
            raise ValueError(f"Scale grade는 1~5 사이여야 합니다. (입력값: {level})")
        return self._modbus_write(self.REG_SCALE_GRADE, level)

    def get_scale_grade(self) -> Optional[int]:
        """현재 측정 거리 범위 레벨 읽기. (datasheet p.11, 0x021F)"""
        return self._modbus_read(self.REG_SCALE_GRADE)

    # ──────────────────────────────────────────────────────────────────────
    # 설정: 전원 노이즈 감소 레벨 (Power Noise Reduction)
    # ──────────────────────────────────────────────────────────────────────

    def set_power_noise_reduction(self, level: PowerNoiseLevel) -> bool:
        """
        전원 노이즈 감소 레벨 설정. (datasheet p.11, 0x021A)

        PowerNoiseLevel.BATTERY(1, 기본값) ~ PowerNoiseLevel.COMPLEX(5, 비권장).
        Level이 높을수록 노이즈 억제 강화. 단, 지향각이 넓어지고
        작은 반사 신호의 물체는 감지 안 될 수 있음.

        ※ UART Controlled 모드: 전원 인가 후 500ms 이내에만 유효.
        """
        level = int(level)
        if not 1 <= level <= 5:
            raise ValueError(f"노이즈 감소 레벨은 1~5 사이여야 합니다. (입력값: {level})")
        return self._modbus_write(self.REG_POWER_NOISE, level)

    def get_power_noise_reduction(self) -> Optional[int]:
        """현재 전원 노이즈 감소 레벨 읽기. (datasheet p.11, 0x021A)"""
        return self._modbus_read(self.REG_POWER_NOISE)

    # ──────────────────────────────────────────────────────────────────────
    # 설정: 출력 단위 (Output Unit)
    # ──────────────────────────────────────────────────────────────────────

    def set_output_unit(self, unit: OutputUnit) -> bool:
        """
        거리 출력 단위 설정. (datasheet p.10, 0x0209)

        OutputUnit.MM (기본값) 또는 OutputUnit.US (us 단위, ÷5.75 → mm).
        UART automatic / UART controlled 모드에서만 유효.

        ※ UART Controlled 모드: 전원 인가 후 500ms 이내에만 유효.
        """
        return self._modbus_write(self.REG_OUTPUT_UNIT, int(unit))

    def get_output_unit(self) -> Optional[int]:
        """현재 출력 단위 읽기. (0=mm, 1=us) (datasheet p.10, 0x0209)"""
        return self._modbus_read(self.REG_OUTPUT_UNIT)

    # ──────────────────────────────────────────────────────────────────────
    # 설정: Slave 주소 / Baud rate
    # ──────────────────────────────────────────────────────────────────────

    def set_slave_address(self, address: int) -> bool:
        """
        Modbus Slave 주소 변경. (datasheet p.10, 0x0200)
        범위: 0x01~0xFE.

        ※ UART Controlled 모드: 전원 인가 후 500ms 이내에만 유효.
        """
        if not 0x01 <= address <= 0xFE:
            raise ValueError("Slave 주소는 0x01~0xFE 사이여야 합니다.")
        result = self._modbus_write(self.REG_SLAVE_ADDRESS, address)
        if result:
            self._slave = address
        return result

    def set_baud_rate(self, baud: int) -> bool:
        """
        Baud rate 변경. (datasheet p.10, 0x0201)
        지원값: 9600, 19200, 38400, 57600, 115200.

        ※ UART Controlled 모드: 전원 인가 후 500ms 이내에만 유효.
        """
        if baud not in self._BAUD_CODE:
            raise ValueError(f"지원 baud rate: {list(self._BAUD_CODE.keys())}")
        result = self._modbus_write(self.REG_BAUD_RATE, self._BAUD_CODE[baud])
        if result:
            self._ser.baudrate = baud
            self._baud = baud
        return result

    # ──────────────────────────────────────────────────────────────────────
    # 설정: 스위치 출력 (Switch Output 모드 전용)
    # ──────────────────────────────────────────────────────────────────────

    def set_switch_threshold(self, mm: int) -> bool:
        """
        스위치 출력 임계값 설정. (datasheet p.11, 0x0206)
        범위: 30~5000mm. Switch Output 모드에서만 유효.
        """
        if not 30 <= mm <= 5000:
            raise ValueError("임계값은 30~5000mm 사이여야 합니다.")
        return self._modbus_write(self.REG_SWITCH_THRESHOLD, mm)

    def set_switch_polarity(self, positive: bool) -> bool:
        """
        스위치 출력 극성 설정. (datasheet p.10, 0x0205)

        positive=True : 물체 감지 시 HIGH 출력.
        positive=False: 물체 감지 시 LOW 출력 (기본값).
        Switch Output 모드에서만 유효.
        """
        return self._modbus_write(self.REG_SWITCH_POLARITY, 0x01 if positive else 0x00)

    # ──────────────────────────────────────────────────────────────────────
    # 상태 요약
    # ──────────────────────────────────────────────────────────────────────

    def info(self) -> dict:
        """센서의 현재 설정값 전체를 딕셔너리로 반환."""
        unit_raw = self._modbus_read(self.REG_OUTPUT_UNIT)
        baud_raw = self._modbus_read(self.REG_BAUD_RATE)
        addr_raw = self._modbus_read(self.REG_SLAVE_ADDRESS)

        return {
            "slave_address":         hex(addr_raw) if addr_raw is not None else None,
            "baud_rate":             self._CODE_BAUD.get(baud_raw, "unknown"),
            "detection_angle_level": self._modbus_read(self.REG_DETECTION_ANGLE),
            "scale_grade":           self._modbus_read(self.REG_SCALE_GRADE),
            "power_noise_level":     self._modbus_read(self.REG_POWER_NOISE),
            "output_unit":           {0: "mm", 1: "us"}.get(unit_raw),
            "switch_threshold_mm":   self._modbus_read(self.REG_SWITCH_THRESHOLD),
            "temperature_C":         self.get_temperature(),
        }

