#!/usr/bin/env python3
"""
DYP-A21 설정값 읽기 스크립트

전제:
  - FT232는 USB에 항상 연결된 상태 (통신 전용)
  - 센서는 외부 전원으로 구동
  - 외부 전원을 껐다 켜면 500ms Modbus 윈도우가 열림

동작:
  1. /dev/ttyUSB* 장치 감지 (기존 연결 포함)
  2. permission이 해소될 때까지 재시도
  3. 포트 열기 성공 후 Modbus 읽기를 성공할 때까지 무한 재시도
     (외부 전원 인가 시 500ms 윈도우 안에 자동 성공)
  4. 결과 출력
"""

import glob
import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dyp_a21 import DYPA21

# ── 설정 ──────────────────────────────────────────────────────────────────────
BAUD           = 115200
MODBUS_TIMEOUT = 0.05    # 50ms: Modbus 1회 read 타임아웃
RETRY_INTERVAL = 0.05    # 50ms: Modbus 실패 후 재시도 간격
PERM_WAIT      = 0.002   # 2ms:  permission denied 재시도 간격
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


def is_valid_cfg(cfg: dict) -> bool:
    """각 레지스터 값이 datasheet 기준 유효 범위에 있는지 검증. (datasheet p.10~11)"""
    checks = [
        cfg.get("baud_rate") not in (None, "unknown"),              # 알려진 baud rate
        cfg.get("detection_angle_level") in (1, 2, 3, 4),          # 1~4
        cfg.get("scale_grade")           in (1, 2, 3, 4, 5),       # 1~5
        cfg.get("power_noise_level")     in (1, 2, 3, 4, 5),       # 1~5
        cfg.get("output_unit")           in ("mm", "us"),           # mm 또는 us
        cfg.get("switch_threshold_mm") is not None
            and 30 <= cfg["switch_threshold_mm"] <= 5000,          # 30~5000mm
        cfg.get("temperature_C") is not None
            and -40 <= cfg["temperature_C"] <= 85,                  # 합리적 온도 범위
    ]
    return all(checks)


def read_until_success(sensor: DYPA21) -> tuple:
    """Modbus 읽기를 성공할 때까지 무한 재시도. 반환: (cfg dict, 시도 횟수)"""
    attempt = 0
    while True:
        attempt += 1

        sensor._ser.reset_input_buffer()   # 이전 수신 데이터 제거
        cfg = sensor.info()

        valid_count = sum(1 for v in cfg.values() if v is not None)
        if is_valid_cfg(cfg):
            return cfg, attempt

        print(f"\r  [{attempt:5d}회] {valid_count}/8 응답 — 외부 전원 인가 대기...", end='', flush=True)
        time.sleep(RETRY_INTERVAL)


def print_config(cfg: dict, attempt: int):
    labels = {
        "slave_address":         "Slave 주소",
        "baud_rate":             "Baud rate",
        "detection_angle_level": "지향각 레벨        (1~4)",
        "scale_grade":           "측정 거리 범위      (1~5)",
        "power_noise_level":     "전원 노이즈 레벨    (1~5)",
        "output_unit":           "출력 단위          (mm/us)",
        "switch_threshold_mm":   "스위치 임계값      (mm)",
        "temperature_C":         "온도               (°C)",
    }
    success_count = sum(1 for v in cfg.values() if v is not None)
    print(f"\n읽기 완료  ({attempt}회 시도 / {success_count}/8 레지스터 성공)\n")
    print("┌──────────────────────────────────────────────────┐")
    print("│  DYP-A21 현재 설정값                              │")
    print("├──────────────────────────────────────────────────┤")
    for key, label in labels.items():
        value = cfg.get(key)
        vstr = str(value) if value is not None else "읽기 실패"
        print(f"│  {label:<30}  :  {vstr:<8} │")
    print("└──────────────────────────────────────────────────┘")


def main():
    try:
        port = get_target_port()
        print(f"  ({time.strftime('%H:%M:%S')})")

        print("포트 열기 시도...")
        sensor = open_sensor(port)
        print(f"포트 열기 성공: {port}")

        print("\nModbus 읽기 시작 — 외부 전원을 인가하면 자동으로 읽힙니다.\n")

        try:
            cfg, attempt = read_until_success(sensor)
            print_config(cfg, attempt)
        except serial.SerialException:
            print("\n\n포트 연결 끊김.")
        finally:
            sensor.disconnect()

    except KeyboardInterrupt:
        print("\n\n종료합니다.")


if __name__ == "__main__":
    main()
