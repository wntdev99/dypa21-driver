"""
DYP-A21 수신 전용 스크립트 (트리거 없음)
근거: DYP-A21-Output-Interfaces-Keila.pdf
  - 프레임: p.01 [0xFF][Data_H][Data_L][SUM] 4바이트
  - Baud  : p.01 115200bps, 8bit, stop 1, parity none
"""

import serial
import time

PORT = '/dev/ttyUSB0'
BAUD = 115200

ser = serial.Serial(
    port=PORT,
    baudrate=BAUD,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.1
)

ser.reset_input_buffer()
print(f"수신 대기 중 ({PORT}, {BAUD}bps) ... Ctrl+C 종료")
print(f"프레임 형식 (p.01): [0xFF][Data_H][Data_L][SUM]")
print("=" * 50)

try:
    while True:
        b = ser.read(1)
        if not b:
            continue

        # raw 바이트 무조건 출력
        ts = time.strftime("%H:%M:%S")
        print(f"[{ts}] RAW: {b.hex()}", end="")

        # 0xFF 헤더 감지 시 나머지 3바이트 읽어 파싱 시도
        if b[0] == 0xFF:
            rest = ser.read(3)
            if rest:
                print(f" {rest.hex()}", end="")
            if len(rest) == 3:
                frame = b + rest
                cs_calc = (frame[0] + frame[1] + frame[2]) & 0xFF
                if cs_calc == frame[3]:
                    dist = (frame[1] << 8) | frame[2]
                    print(f"  →  거리: {dist} mm", end="")
                else:
                    print(f"  →  체크섬 오류 (계산={hex(cs_calc)}, 수신={hex(frame[3])})", end="")

        print()

except KeyboardInterrupt:
    print("\n종료")
finally:
    ser.close()
