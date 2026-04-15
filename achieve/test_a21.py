"""
DYP-A21 초음파 센서 UART 통신 테스트
- Baud rate : 115200bps
- 프레임    : [0xFF][Data_H][Data_L][SUM] (4바이트)
- 거리 단위 : mm
"""

import serial
import time
import sys

PORT = '/dev/ttyUSB0'

# ─────────────────────────────────────────────
# 공통 시리얼 설정
# ─────────────────────────────────────────────
ser = serial.Serial(
    port=PORT,
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

# ─────────────────────────────────────────────
# 프레임 파싱
# ─────────────────────────────────────────────
def parse_frame(data: bytes):
    """4바이트 A21 프레임 파싱. 실패 시 None 반환"""
    if len(data) != 4:
        return None
    if data[0] != 0xFF:
        return None
    checksum = (data[0] + data[1] + data[2]) & 0xFF
    if checksum != data[3]:
        return None
    dist_mm = (data[1] << 8) | data[2]
    return dist_mm


# ─────────────────────────────────────────────
# 0xFF 헤더 동기화 후 4바이트 읽기
# ─────────────────────────────────────────────
def sync_read(timeout_sec=3.0):
    """
    버퍼 중간에서 시작해도 0xFF 헤더를 찾아 프레임 동기화.
    성공 시 4바이트 반환, 타임아웃 시 None 반환.
    """
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == 0xFF:
            rest = ser.read(3)
            if len(rest) == 3:
                return b + rest
    return None


# ─────────────────────────────────────────────
# MODE 1: UART 자동 출력 테스트
#   RX 핀 미연결(HIGH) 상태 → 센서가 자동 연속 출력
# ─────────────────────────────────────────────
def test_auto_mode(count=10):
    print("\n[모드1] UART 자동 출력 테스트")
    print("  RX 핀이 미연결/HIGH 상태여야 합니다")
    print("=" * 45)
    ser.reset_input_buffer()

    ok = 0
    for i in range(count):
        frame = sync_read(timeout_sec=2.0)
        if frame is None:
            print(f"  [{i+1:02d}] 타임아웃 — 데이터 수신 없음")
            continue

        dist = parse_frame(frame)
        if dist is not None:
            print(f"  [{i+1:02d}] 거리: {dist:5d} mm  | raw: {frame.hex()}")
            ok += 1
        else:
            expected_cs = (frame[0] + frame[1] + frame[2]) & 0xFF
            print(f"  [{i+1:02d}] 체크섬 오류 | raw: {frame.hex()} "
                  f"(기대={expected_cs:#04x}, 수신={frame[3]:#04x})")

    print(f"\n  결과: {ok}/{count} 프레임 정상 수신")


# ─────────────────────────────────────────────
# MODE 2: UART 제어 출력 테스트
#   RX 핀으로 트리거 전송 → 센서가 1회 응답
#   트리거 주기는 반드시 >150ms
# ─────────────────────────────────────────────
def test_controlled_mode(count=10):
    print("\n[모드2] UART 제어 출력(트리거) 테스트")
    print("  센서 RX ← FT232 TX 연결 필요")
    print("=" * 45)
    ser.reset_input_buffer()

    ok = 0
    for i in range(count):
        # 임의의 1바이트 전신 → 트리거
        ser.write(b'\x01')
        # TX 에코가 있을 경우를 위한 짧은 대기 후 버퍼 플러시
        time.sleep(0.01)
        ser.reset_input_buffer()

        # 센서 응답 대기 (T1 > T2+15ms, T2 최대 140ms → ~160ms 대기)
        frame = sync_read(timeout_sec=0.5)
        if frame is None:
            print(f"  [{i+1:02d}] 응답 없음")
        else:
            dist = parse_frame(frame)
            if dist is not None:
                print(f"  [{i+1:02d}] 거리: {dist:5d} mm  | raw: {frame.hex()}")
                ok += 1
            else:
                print(f"  [{i+1:02d}] 파싱 실패 | raw: {frame.hex()}")

        # 트리거 주기 >150ms 보장
        time.sleep(0.15)

    print(f"\n  결과: {ok}/{count} 응답 정상 수신")


# ─────────────────────────────────────────────
# 원시 데이터 덤프 (프로토콜 불명확 시 진단용)
# ─────────────────────────────────────────────
def raw_dump(duration_sec=5):
    print(f"\n[진단] {duration_sec}초간 원시 수신 데이터 덤프")
    print("=" * 45)
    ser.reset_input_buffer()
    deadline = time.time() + duration_sec
    buf = b''
    while time.time() < deadline:
        chunk = ser.read(64)
        if chunk:
            buf += chunk
            # 4바이트씩 줄 바꿔 출력
            while len(buf) >= 4:
                blk = buf[:4]
                buf = buf[4:]
                dist = parse_frame(blk)
                tag = f"→ {dist:5d}mm" if dist is not None else "       "
                print(f"  {blk.hex('  ')}  {tag}")
    if buf:
        print(f"  (미완료 잔여: {buf.hex()})")
    print("  덤프 종료")


# ─────────────────────────────────────────────
# 메인
# ─────────────────────────────────────────────
if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'auto'

    print(f"포트  : {PORT}")
    print(f"Baud  : 115200bps")
    print(f"모드  : {mode}")

    try:
        if mode == 'auto':
            test_auto_mode(count=10)
        elif mode == 'ctrl':
            test_controlled_mode(count=10)
        elif mode == 'dump':
            raw_dump(duration_sec=5)
        else:
            print(f"알 수 없는 모드: {mode}")
            print("사용법: python test_a21.py [auto|ctrl|dump]")
    finally:
        ser.close()
        print("\n포트 닫힘")
