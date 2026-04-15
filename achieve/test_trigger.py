"""
DYP-A21 UART 제어 출력 모드 — 1초 주기 트리거 테스트
근거: DYP-A21-Output-Interfaces-Keila.pdf
  - 트리거: p.02 "falling edge or any serial port data"
  - 주기:   p.02 "trigger cycle must be greater than 150ms"
  - 타이밍: p.03 "T1 > T2+15ms, T2=8~140ms" → 응답 최대 155ms
  - 프레임: p.01 [0xFF][Data_H][Data_L][SUM], 4바이트
"""

import serial
import time

PORT     = '/dev/ttyUSB0'
BAUD     = 115200          # p.01 115200bps
TRIGGER  = b'\x01'         # 임의 1바이트 → start bit 하강 에지 발생
INTERVAL = 1.0             # 1초 주기 (> 150ms 조건 충족, p.02)
WAIT     = 0.20            # 응답 대기 200ms (T1 > T2+15ms, T2 최대 140ms, p.03)

ser = serial.Serial(
    port=PORT,
    baudrate=BAUD,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=WAIT
)

print(f"포트  : {PORT} / {BAUD}bps")
print(f"주기  : {INTERVAL}s  |  응답 대기: {WAIT*1000:.0f}ms")
print(f"프레임: [0xFF][Data_H][Data_L][SUM]  (p.01)")
print("=" * 50)
print("[멀티미터 확인 포인트]")
print("  FT232 TX핀 : 트리거 전송 순간 전압 변동 확인")
print("  FT232 RX핀 : 센서 응답 시 전압 변동 확인")
print("=" * 50)

count = 0
try:
    while True:
        count += 1

        # 1. 트리거 전송 (하강 에지 = start bit 발생)
        ser.reset_input_buffer()
        ser.write(TRIGGER)
        ts = time.strftime("%H:%M:%S")
        print(f"\n[{count:03d}] {ts} | 트리거 전송: {TRIGGER.hex()}")

        # 2. TX 에코 제거 (FT232 루프백 방지)
        #    1바이트 × 10bits / 115200bps ≈ 0.087ms
        time.sleep(0.005)
        ser.reset_input_buffer()

        # 3. 센서 응답 대기 및 0xFF 헤더 동기화
        deadline = time.time() + WAIT
        frame = None
        raw_bytes = b''

        while time.time() < deadline:
            b = ser.read(1)
            if not b:
                break
            raw_bytes += b
            if b[0] == 0xFF:
                rest = ser.read(3)
                raw_bytes += rest
                if len(rest) == 3:
                    frame = b + rest
                    break

        # 4. 결과 출력
        if frame:
            cs_calc = (frame[0] + frame[1] + frame[2]) & 0xFF
            if cs_calc == frame[3]:
                dist_mm = (frame[1] << 8) | frame[2]
                print(f"       응답 OK  | raw: {frame.hex()}  | 거리: {dist_mm} mm")
            else:
                print(f"       체크섬 오류 | raw: {frame.hex()}")
                print(f"       계산={hex(cs_calc)}, 수신={hex(frame[3])}")
        elif raw_bytes:
            print(f"       응답 있음(파싱 실패) | raw: {raw_bytes.hex()}")
        else:
            print(f"       응답 없음 ({WAIT*1000:.0f}ms 대기)")

        # 5. 다음 트리거까지 대기
        time.sleep(INTERVAL - WAIT)

except KeyboardInterrupt:
    print("\n\n종료 (Ctrl+C)")
finally:
    ser.close()
    print("포트 닫힘")
