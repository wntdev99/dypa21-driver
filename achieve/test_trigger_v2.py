import serial
import time

# 1. 포트 설정 (ls /dev/ttyUSB* 로 확인한 포트 입력)
PORT = '/dev/ttyUSB0'
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT}")

    while True:
        # 2. 트리거 명령(0xFF) 전송
        ser.write(b'\xff')

        # 3. 데이터 읽기 (4바이트 대기)
        data = ser.read(4)

        if len(data) == 4 and data[0] == 0xff:
            # 4. 거리 계산 (High Byte << 8 + Low Byte)
            distance = (data[1] << 8) + data[2]

            # 5. 체크섬 확인 (선택 사항)
            checksum = (data[0] + data[1] + data[2]) & 0xFF
            if checksum == data[3]:
                print(f"Distance: {distance} mm")
            else:
                print("Checksum Error")

        time.sleep(0.2) # 0.2초 간격 측정

except Exception as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals():
        ser.close()