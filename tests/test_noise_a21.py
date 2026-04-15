#!/usr/bin/env python3
"""
DYP-A21 노이즈 검증 테스트

동일한 고정 벽면을 향해 일정 시간 동안 연속 측정하여
측정값의 안정성(노이즈 수준)을 통계적으로 검증한다.

판정 기준:
  PASS : 표준편차 ≤ STD_THRESHOLD (기본 3mm)
         peak-to-peak ≤ PTP_THRESHOLD (기본 10mm)
  FAIL : 위 기준 중 하나라도 초과

사용법:
  python tests/test_noise_a21.py
  python tests/test_noise_a21.py --duration 60 --std-threshold 5
"""

import argparse
import glob
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from dyp_a21 import DYPA21

# ── 기본 설정 ──────────────────────────────────────────────────────────────────
BAUD            = 115200
MODBUS_TIMEOUT  = 0.05    # 50ms
PERM_WAIT       = 0.002   # 2ms: permission denied 재시도 간격
RETRY_INTERVAL  = 0.05    # 50ms: Modbus 실패 후 재시도 간격
TRIGGER_INTERVAL_MAP = {
    1: 0.090,   # CM_50
    2: 0.100,   # CM_150
    3: 0.110,   # CM_250
    4: 0.120,   # CM_350
    5: 0.150,   # CM_500
}

DEFAULT_DURATION      = 30    # 측정 시간 (초)
DEFAULT_STD_THRESHOLD = 3     # 표준편차 합격 기준 (mm)
DEFAULT_PTP_THRESHOLD = 10    # peak-to-peak 합격 기준 (mm)
# ─────────────────────────────────────────────────────────────────────────────


def find_usb_ports() -> set:
    return set(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))


def get_target_port() -> str:
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
    sensor = DYPA21(port, BAUD, timeout=MODBUS_TIMEOUT)
    attempt = 0
    while True:
        try:
            sensor.connect()
            if attempt > 0:
                print()
            return sensor
        except serial.SerialException as e:
            if e.errno == 13:
                attempt += 1
                if attempt == 1:
                    print("  permission 대기 중 ", end='', flush=True)
                elif attempt % 10 == 0:
                    print('.', end='', flush=True)
                time.sleep(PERM_WAIT)
            else:
                raise RuntimeError(f"포트 열기 실패: {e}")


def read_scale_grade_until_success(sensor: DYPA21) -> tuple:
    attempt = 0
    while True:
        attempt += 1
        sensor._ser.reset_input_buffer()
        scale_grade = sensor.get_scale_grade()
        if scale_grade is not None and scale_grade in TRIGGER_INTERVAL_MAP:
            return scale_grade, attempt
        print(f"\r  [{attempt:5d}회] Scale grade 읽기 중 — 외부 전원 인가 대기...", end='', flush=True)
        time.sleep(RETRY_INTERVAL)


def collect_samples(sensor: DYPA21, interval: float, duration: float) -> list:
    """지정 시간 동안 거리값을 수집하여 반환. None(FAIL)은 제외."""
    samples = []
    fail_count = 0
    total_count = 0
    end_time = time.monotonic() + duration

    print(f"{'#':<6} {'TIME':<10} {'DIST(mm)':<12} {'STATUS'}")
    print("-" * 42)

    while time.monotonic() < end_time:
        distance = sensor.trigger()
        total_count += 1
        timestamp = time.strftime('%H:%M:%S')

        if distance is not None:
            samples.append(distance)
            print(f"{total_count:<6} {timestamp:<10} {distance:<12} OK")
        else:
            fail_count += 1
            print(f"{total_count:<6} {timestamp:<10} {'---':<12} FAIL")

        time.sleep(interval)

    print()
    return samples, fail_count, total_count


def compute_stats(samples: list) -> dict:
    n = len(samples)
    if n == 0:
        return None

    mean = sum(samples) / n
    variance = sum((x - mean) ** 2 for x in samples) / n
    std = variance ** 0.5
    min_v = min(samples)
    max_v = max(samples)
    ptp = max_v - min_v

    return {
        "n":    n,
        "mean": mean,
        "std":  std,
        "min":  min_v,
        "max":  max_v,
        "ptp":  ptp,
    }


def print_result(stats: dict, fail_count: int, total_count: int,
                 std_threshold: float, ptp_threshold: float):
    if stats is None:
        print("=" * 50)
        print("  결과: 유효한 샘플 없음 — FAIL")
        print("=" * 50)
        return

    pass_std = stats["std"] <= std_threshold
    pass_ptp = stats["ptp"] <= ptp_threshold
    overall  = "PASS" if (pass_std and pass_ptp) else "FAIL"

    print("=" * 50)
    print("  [통계 요약]")
    print(f"  총 측정 횟수  : {total_count}")
    print(f"  유효 샘플 수  : {stats['n']}  (FAIL: {fail_count})")
    print(f"  평균 거리     : {stats['mean']:.1f} mm")
    print(f"  표준편차 (σ)  : {stats['std']:.2f} mm  {'✓' if pass_std else '✗'}  (기준: ≤ {std_threshold} mm)")
    print(f"  최소값        : {stats['min']} mm")
    print(f"  최대값        : {stats['max']} mm")
    print(f"  Peak-to-Peak  : {stats['ptp']} mm  {'✓' if pass_ptp else '✗'}  (기준: ≤ {ptp_threshold} mm)")
    print("-" * 50)
    print(f"  최종 판정     : {overall}")
    print("=" * 50)


def parse_args():
    parser = argparse.ArgumentParser(description="DYP-A21 노이즈 검증 테스트")
    parser.add_argument("--duration",      type=float, default=DEFAULT_DURATION,
                        help=f"측정 시간 (초, 기본값: {DEFAULT_DURATION})")
    parser.add_argument("--std-threshold", type=float, default=DEFAULT_STD_THRESHOLD,
                        help=f"표준편차 합격 기준 (mm, 기본값: {DEFAULT_STD_THRESHOLD})")
    parser.add_argument("--ptp-threshold", type=float, default=DEFAULT_PTP_THRESHOLD,
                        help=f"peak-to-peak 합격 기준 (mm, 기본값: {DEFAULT_PTP_THRESHOLD})")
    return parser.parse_args()


def main():
    args = parse_args()

    print("=" * 50)
    print("  DYP-A21 노이즈 검증 테스트")
    print(f"  측정 시간    : {args.duration:.0f}초")
    print(f"  σ 기준       : ≤ {args.std_threshold} mm")
    print(f"  PtP 기준     : ≤ {args.ptp_threshold} mm")
    print("=" * 50)

    sensor = None
    try:
        port = get_target_port()
        print(f"  ({time.strftime('%H:%M:%S')})")

        sensor = open_sensor(port)
        print(f"포트 열기 성공: {port}")

        print("\nModbus 읽기 시작 — 외부 전원을 인가하면 자동으로 읽힙니다.\n")
        scale_grade, attempt = read_scale_grade_until_success(sensor)
        trigger_interval = TRIGGER_INTERVAL_MAP[scale_grade]
        print(f"\nScale grade 읽기 완료  ({attempt}회 시도)")
        print(f"Scale grade: {scale_grade} → Trigger 주기: {trigger_interval*1000:.0f}ms\n")

        # UART Controlled 모드 안정화
        print("UART Controlled 모드 안정화 중... ", end='', flush=True)
        time.sleep(1.0)
        sensor._ser.reset_input_buffer()
        sensor._ser.reset_output_buffer()
        print("완료\n")

        print(f"측정 시작 — 센서를 고정 벽면에 향하게 하세요. ({args.duration:.0f}초)\n")
        samples, fail_count, total_count = collect_samples(
            sensor, trigger_interval, args.duration
        )

        stats = compute_stats(samples)
        print_result(stats, fail_count, total_count,
                     args.std_threshold, args.ptp_threshold)

    except KeyboardInterrupt:
        print("\n\n테스트 중단됨.")
    except serial.SerialException:
        print("\n포트 연결 끊김.")
    finally:
        if sensor:
            sensor.disconnect()


if __name__ == "__main__":
    main()
