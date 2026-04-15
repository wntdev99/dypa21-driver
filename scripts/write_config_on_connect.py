#!/usr/bin/env python3
"""
DYP-A21 설정값 쓰기 스크립트

전제:
  - FT232는 USB에 항상 연결된 상태 (통신 전용)
  - 센서는 외부 전원으로 구동
  - 외부 전원을 껐다 켜면 500ms Modbus 윈도우가 열림

동작:
  1. /dev/ttyUSB* 장치 감지 (기존 연결 포함)
  2. permission이 해소될 때까지 재시도
  3. 포트 열기 성공 후 각 설정값을 쓰고 검증할 때까지 무한 재시도
     (외부 전원 인가 시 500ms 윈도우 안에 자동 성공)
  4. 결과 출력
"""

import glob
import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dyp_a21 import DYPA21, AngleLevel, ScaleGrade, OutputUnit, PowerNoiseLevel

# ── 설정 ──────────────────────────────────────────────────────────────────────
BAUD           = 115200
MODBUS_TIMEOUT = 0.05    # 50ms: Modbus 1회 read 타임아웃
RETRY_INTERVAL = 0.05    # 50ms: Modbus 실패 후 재시도 간격
PERM_WAIT      = 0.002   # 2ms:  permission denied 재시도 간격

# 쓸 설정값 (현재 설정 기준)
CONFIG_TO_WRITE = {
    "slave_address":         0x1,
    "baud_rate":             115200,
    "detection_angle_level": AngleLevel.WIDE,           # 4
    "scale_grade":           ScaleGrade.CM_500,         # 5
    "power_noise_level":     PowerNoiseLevel.BATTERY,   # 1
    "output_unit":           OutputUnit.MM,             # 0
    "switch_threshold_mm":   1500,
}
# ─────────────────────────────────────────────────────────────────────────────


def find_usb_ports() -> set:
    return set(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))


def get_target_port() -> str:
    """이미 연결된 장치가 있으면 바로 반환. 없으면 새 장치 대기."""
    existing = find_usb_ports()
    if existing:
        port = sorted(existing)[0]
        print(f"기존 연결 감지: {port}")
        return port

    print("FT232 USB 연결 대기 중... (Ctrl+C 종료)")
    known = existing
    while True:
        current = find_usb_ports()
        new_ports = current - known
        if new_ports:
            port = sorted(new_ports)[0]
            print(f"장치 감지: {port}")
            return port
        time.sleep(0.01)


def open_sensor(port: str) -> DYPA21:
    """permission denied가 해소될 때까지 재시도 후 연결된 DYPA21 반환."""
    sensor = DYPA21(port, BAUD, timeout=MODBUS_TIMEOUT)
    attempt = 0
    while True:
        try:
            sensor.connect()
            if attempt > 0:
                print()   # 점 출력 후 줄바꿈
            return sensor
        except serial.SerialException as e:
            if e.errno == 13:   # Permission denied — udev 권한 적용 전 레이스 컨디션
                attempt += 1
                if attempt == 1:
                    print("  permission 대기 중 ", end='', flush=True)
                elif attempt % 10 == 0:
                    print('.', end='', flush=True)
                time.sleep(PERM_WAIT)
            else:
                raise RuntimeError(f"포트 열기 실패: {e}")


def write_config_until_success(sensor: DYPA21) -> tuple:
    """모든 설정값을 쓰고 검증할 때까지 무한 재시도. 반환: (결과 dict, 시도 횟수)"""
    attempt = 0
    results = {}

    while True:
        attempt += 1
        success_count = 0
        sensor._ser.reset_input_buffer()

        # 각 설정값을 쓰고 검증
        # 1. Slave 주소
        if sensor.set_slave_address(CONFIG_TO_WRITE["slave_address"]):
            read_val = sensor.get_detection_angle()  # 다른 항목 읽어서 포트 살아있는지 확인
            if read_val is not None:
                results["slave_address"] = True
                success_count += 1

        # 2. Baud rate
        if sensor.set_baud_rate(CONFIG_TO_WRITE["baud_rate"]):
            read_val = sensor._modbus_read(sensor.REG_BAUD_RATE)
            if read_val == sensor._BAUD_CODE[CONFIG_TO_WRITE["baud_rate"]]:
                results["baud_rate"] = True
                success_count += 1

        # 3. 지향각 레벨
        if sensor.set_detection_angle(CONFIG_TO_WRITE["detection_angle_level"]):
            read_val = sensor.get_detection_angle()
            if read_val == CONFIG_TO_WRITE["detection_angle_level"]:
                results["detection_angle_level"] = True
                success_count += 1

        # 4. 측정 거리 범위
        if sensor.set_scale_grade(CONFIG_TO_WRITE["scale_grade"]):
            read_val = sensor.get_scale_grade()
            if read_val == CONFIG_TO_WRITE["scale_grade"]:
                results["scale_grade"] = True
                success_count += 1

        # 5. 전원 노이즈 레벨
        if sensor.set_power_noise_reduction(CONFIG_TO_WRITE["power_noise_level"]):
            read_val = sensor.get_power_noise_reduction()
            if read_val == CONFIG_TO_WRITE["power_noise_level"]:
                results["power_noise_level"] = True
                success_count += 1

        # 6. 출력 단위
        if sensor.set_output_unit(CONFIG_TO_WRITE["output_unit"]):
            read_val = sensor.get_output_unit()
            if read_val == CONFIG_TO_WRITE["output_unit"]:
                results["output_unit"] = True
                success_count += 1

        # 7. 스위치 임계값
        if sensor.set_switch_threshold(CONFIG_TO_WRITE["switch_threshold_mm"]):
            read_val = sensor._modbus_read(sensor.REG_SWITCH_THRESHOLD)
            if read_val == CONFIG_TO_WRITE["switch_threshold_mm"]:
                results["switch_threshold_mm"] = True
                success_count += 1

        # 모든 항목이 성공하면 반환
        if success_count == 7:
            return results, attempt

        print(f"\r  [{attempt:5d}회] {success_count}/7 항목 쓰기 성공 — 외부 전원 인가 대기...", end='', flush=True)
        time.sleep(RETRY_INTERVAL)


def print_write_result(results: dict, attempt: int):
    """쓰기 결과 출력"""
    success_count = sum(1 for v in results.values() if v)
    print(f"\n쓰기 완료  ({attempt}회 시도 / {success_count}/7 항목 성공)\n")
    print("┌──────────────────────────────────────────────────────┐")
    print("│  DYP-A21 설정값 쓰기 결과                              │")
    print("├──────────────────────────────────────────────────────┤")

    labels = {
        "slave_address":         "Slave 주소              ",
        "baud_rate":             "Baud rate               ",
        "detection_angle_level": "지향각 레벨        (1~4)",
        "scale_grade":           "측정 거리 범위      (1~5)",
        "power_noise_level":     "전원 노이즈 레벨    (1~5)",
        "output_unit":           "출력 단위          (mm/us)",
        "switch_threshold_mm":   "스위치 임계값      (mm)",
    }

    for key, label in labels.items():
        status = "✓ 성공" if results.get(key) else "✗ 실패"
        print(f"│  {label:<30}  :  {status:<8} │")

    print("└──────────────────────────────────────────────────────┘")


def main():
    try:
        port = get_target_port()
        print(f"  ({time.strftime('%H:%M:%S')})")

        print("포트 열기 시도...")
        sensor = open_sensor(port)
        print(f"포트 열기 성공: {port}")

        print("\nModbus 쓰기 시작 — 외부 전원을 인가하면 자동으로 쓰여집니다.\n")

        try:
            results, attempt = write_config_until_success(sensor)
            print_write_result(results, attempt)
        except serial.SerialException:
            print("\n\n포트 연결 끊김.")
        finally:
            sensor.disconnect()

    except KeyboardInterrupt:
        print("\n\n종료합니다.")


if __name__ == "__main__":
    main()
