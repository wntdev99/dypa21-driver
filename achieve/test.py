import serial

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,         # 문서 기준 기본값
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

def parse_frame(data: bytes) -> list:
    """DYP-E09 18바이트 프레임 파싱"""
    if len(data) != 18:
        raise ValueError(f"프레임 길이 오류: {len(data)} bytes")

    if data[0] != 0xFF:
        raise ValueError(f"헤더 오류: {hex(data[0])}")

    # 체크섬 검증
    checksum = sum(data[:17]) & 0xFF
    if checksum != data[17]:
        raise ValueError(f"체크섬 불일치: 계산={hex(checksum)}, 수신={hex(data[17])}")

    # 8개 센서 거리값 파싱 (단위: mm)
    distances = []
    for i in range(8):
        high = data[1 + i * 2]
        low  = data[2 + i * 2]
        dist_mm = (high << 8) | low
        distances.append(dist_mm)

    return distances

# 수신 루프
while True:
    raw = ser.read(18)
    if raw:
        try:
            distances = parse_frame(raw)
            for idx, d in enumerate(distances, 1):
                print(f"센서 {idx}: {d} mm")
        except ValueError as e:
            print(f"[오류] {e}")