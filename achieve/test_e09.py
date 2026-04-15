"""
DYP-E09-V1.0 UART 제어 출력 테스트
근거: E09 Datasheet-240222.pdf
  - Baud rate : p.4  "(1) Communication parameters" 9600bps, 8bit, stop 1, parity none
  - 프레임    : p.4-5 "(2) Output protocol format" 18바이트
                [0xFF][D1_H][D1_L]...[D8_H][D8_L][SUM]
  - 체크섬    : p.5  SUM = (header + D1_H + ... + D8_L) & 0x00FF
  - 트리거    : p.6  "(4) UART controlled output" falling edge on RX
  - 응답 대기 : p.6  T1 = T2 + 20ms, default timeout 200ms
"""

import serial
import time

port_path = '/dev/ttyUSB0'

try:
    # p.4 기본값 9600bps이나 현재 E09가 115200bps로 변경된 상태 (p.9 레지스터 0x0201=0x09)
    # p.11 자동 출력 모드 최대 응답시간 1628ms → timeout 2s
    ser = serial.Serial(
        port=port_path,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=2
    )
    print(f"DYP-E09 포트 열림: {port_path}")

    while True:
        # 버퍼 초기화 후 0xFF 헤더 동기화
        # p.4: Frame header fixed to 0xFF
        ser.reset_input_buffer()

        # 0xFF 헤더 탐색
        deadline = time.time() + 2.0
        header = b''
        while time.time() < deadline:
            b = ser.read(1)
            if b and b[0] == 0xFF:
                header = b
                break

        if not header:
            print("수신 실패: 헤더(0xFF) 없음")
            time.sleep(0.75)
            continue

        # 헤더 이후 나머지 17바이트 수신
        # p.4-5: 프레임 = 18바이트
        rest = ser.read(17)
        data = header + rest

        if len(data) != 18:
            print(f"수신 실패: {len(data)}/18 바이트")
        else:
            # p.5: SUM = (header + D1_H + ... + D8_L) & 0x00FF (17바이트 합산)
            checksum = sum(data[:17]) & 0xFF
            if checksum == data[17]:
                # p.4-5: 각 채널 거리 = Data_H × 256 + Data_L
                for ch in range(8):
                    h = data[1 + ch * 2]
                    l = data[2 + ch * 2]
                    dist = (h << 8) | l
                    print(f"  채널 {ch+1}: {dist} mm")
            else:
                print(f"체크섬 오류 (계산={hex(checksum)}, 수신={hex(data[17])})")

        time.sleep(0.75)  # 총 주기 약 1초

except serial.SerialException as e:
    print(f"포트 오류: {e}")
except KeyboardInterrupt:
    print("\n종료")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()