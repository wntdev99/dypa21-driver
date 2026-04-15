#!/usr/bin/env python3
"""
DYP-A21 실시간 거리 측정 스크립트 (UART Controlled Mode)

특징:
  - UART Controlled 모드 (트리거 기반)
  - 실시간 응답 시간: 8~140ms
  - 트리거 주기: 최소 150ms
  - 빠른 응답이 필요한 경우 사용

동작:
  1. /dev/ttyUSB* 장치 감지
  2. 포트 열기 (permission 처리)
  3. trigger(0xFF) 호출로 거리값 수신
  4. 150ms 주기로 반복
"""

import glob
import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dyp_a21 import DYPA21

# ── 설정 ──────────────────────────────────────────────────────────────────────
BAUD             = 115200
MODBUS_TIMEOUT   = 0.05    # 50ms: Modbus 타임아웃
PERM_WAIT        = 0.002   # 2ms: permission denied 재시도 간격
RETRY_INTERVAL   = 0.05    # 50ms: Modbus 실패 후 재시도 간격
# scale_grade에 따른 최소 trigger 주기 (datasheet 응답시간 + 10ms 마진)
TRIGGER_INTERVAL_MAP = {
    1: 0.090,   # CM_50:  80ms + 10ms margin
    2: 0.100,   # CM_150: 90ms + 10ms margin
    3: 0.110,   # CM_250: 100ms + 10ms margin
    4: 0.120,   # CM_350: 110ms + 10ms margin
    5: 0.150,   # CM_500: 140ms + 10ms margin (기본값)
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
                print()
            return sensor
        except serial.SerialException as e:
            if e.errno == 13:  # Permission denied
                attempt += 1
                if attempt == 1:
                    print("  permission 대기 중 ", end='', flush=True)
                elif attempt % 10 == 0:
                    print('.', end='', flush=True)
                time.sleep(PERM_WAIT)
            else:
                raise RuntimeError(f"포트 열기 실패: {e}")


def read_scale_grade_until_success(sensor: DYPA21) -> tuple:
    """Scale grade를 성공적으로 읽을 때까지 재시도. 반환: (scale_grade, 시도 횟수)"""
    attempt = 0
    while True:
        attempt += 1
        sensor._ser.reset_input_buffer()
        scale_grade = sensor.get_scale_grade()

        if scale_grade is not None and scale_grade in TRIGGER_INTERVAL_MAP:
            return scale_grade, attempt

        print(f"\r  [{attempt:5d}회] Scale grade 읽기 중 — 외부 전원 인가 대기...", end='', flush=True)
        time.sleep(RETRY_INTERVAL)

def continuous_distance_measurement(sensor: DYPA21, interval: float):
    """
    실시간 거리값 지속적 측정 및 출력.

    UART Controlled 모드: 트리거(0xFF) 전송 후 거리값 수신.
    응답 시간: 8~140ms (빠름)
    """
    try:
        print(f"{'TIME':<10} {'DISTANCE (mm)':<20} {'STATUS':<10}")
        print("-" * 45)

        while True:
            distance = sensor.trigger()

            timestamp = time.strftime('%H:%M:%S')
            if distance is not None:
                print(f"{timestamp:<10} {distance:<20d} OK")
            else:
                print(f"{timestamp:<10} {'---':<20} FAIL")

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n거리 측정을 종료합니다.")


def main():
    try:
        port = get_target_port()
        print(f"  ({time.strftime('%H:%M:%S')})")

        print("포트 열기 시도...")
        sensor = open_sensor(port)
        print(f"포트 열기 성공: {port}")

        # Scale grade 읽기 (modbus) — 성공할 때까지 재시도
        print("\nModbus 읽기 시작 — 외부 전원을 인가하면 자동으로 읽힙니다.\n")
        try:
            scale_grade, attempt = read_scale_grade_until_success(sensor)
            trigger_interval = TRIGGER_INTERVAL_MAP[scale_grade]
            print(f"\nScale grade 읽기 완료  ({attempt}회 시도)")
            print(f"Scale grade: {scale_grade} → Trigger 주기: {trigger_interval*1000:.0f}ms\n")
        except serial.SerialException:
            print("\n\n포트 연결 끊김.")
            return

        # ── UART Controlled 모드 안정화 ──────────────────────────────────────
        # Modbus 통신 후 센서가 자동으로 UART Controlled 모드로 전환되도록 대기
        # 데이터시트: 전원 인가 후 500ms 이내에만 Modbus 유효
        # → 500ms 이상 경과 후 센서 자동 모드 전환, 추가 500ms 안정화
        print("UART Controlled 모드 안정화 중... ", end='', flush=True)
        time.sleep(1.0)  # 1초 이상 대기 (모드 전환 + 안정화)

        # 입력/출력 버퍼 완전히 초기화
        sensor._ser.reset_input_buffer()
        sensor._ser.reset_output_buffer()
        print("완료\n")
        # ─────────────────────────────────────────────────────────────────────

        continuous_distance_measurement(sensor, trigger_interval)

    except KeyboardInterrupt:
        print("\n\n종료합니다.")
    finally:
        if 'sensor' in locals():
            sensor.disconnect()


if __name__ == "__main__":
    main()
