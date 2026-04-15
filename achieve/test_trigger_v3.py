import serial
import sys
import tty
import termios

PORT = '/dev/ttyUSB0'
BAUD = 115200


def get_key():
    """Enter 없이 단일 키 입력 읽기"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def trigger_and_read(ser):
    """트리거(0xFF) 전송 후 4바이트 응답 수신 및 거리 계산"""
    ser.write(b'\xff')
    data = ser.read(4)

    if len(data) == 4 and data[0] == 0xFF:
        distance = (data[1] << 8) + data[2]
        checksum = (data[0] + data[1] + data[2]) & 0xFF
        if checksum == data[3]:
            return distance, None
        else:
            return None, "Checksum Error"
    return None, f"응답 없음 (수신 바이트: {len(data)})"


try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT}")
    print("아무 키: 거리 측정  |  q: 종료\n")

    while True:
        key = get_key()

        if key.lower() == 'q':
            print("\n종료합니다.")
            break

        distance, error = trigger_and_read(ser)

        if distance is not None:
            print(f"Distance: {distance} mm")
        else:
            print(f"Error: {error}")

except Exception as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals():
        ser.close()
