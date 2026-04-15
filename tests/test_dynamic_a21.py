#!/usr/bin/env python3
"""
DYP-A21 동적 물체 인식 검증 테스트

FOV 내에서 물체를 움직이면서 급격한 오인식(이상 점프)이 발생하는지 검증한다.
터미널에서 실시간 시계열 ASCII 그래프로 시각화하고,
완료 후 matplotlib PNG로 저장한다.

판정 기준:
  PASS : 이상 점프 비율 ≤ --jump-rate-limit (기본 5%)
  FAIL : 위 기준 초과

  이상 점프 정의: 연속 측정값 간 변화가 --jump-threshold (기본 100mm) 초과

그래프 범례 (터미널):
  *   정상 측정값
  !   이상 점프 발생 지점
  ◄   가장 최근 측정값

사용법:
  python tests/test_dynamic_a21.py
  python tests/test_dynamic_a21.py --duration 60 --jump-threshold 80
  python tests/test_dynamic_a21.py --duration 30 --no-save
"""

import argparse
import curses
import glob
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from dyp_a21 import DYPA21

# ── 기본 설정 ──────────────────────────────────────────────────────────────────
BAUD            = 115200
MODBUS_TIMEOUT  = 0.05
PERM_WAIT       = 0.002
RETRY_INTERVAL  = 0.05
TRIGGER_INTERVAL_MAP = {
    1: 0.090, 2: 0.100, 3: 0.110, 4: 0.120, 5: 0.150,
}

DEFAULT_DURATION        = 30     # 측정 시간 (초)
DEFAULT_JUMP_THRESHOLD  = 100    # 이상 점프 감지 기준 (mm)
DEFAULT_JUMP_RATE_LIMIT = 5.0    # 이상 점프 비율 합격 기준 (%)
# ─────────────────────────────────────────────────────────────────────────────


# ── 센서 연결 유틸리티 ─────────────────────────────────────────────────────────

def find_usb_ports():
    return set(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))


def get_target_port():
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


def open_sensor(port):
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


def read_scale_grade_until_success(sensor):
    attempt = 0
    while True:
        attempt += 1
        sensor._ser.reset_input_buffer()
        scale_grade = sensor.get_scale_grade()
        if scale_grade is not None and scale_grade in TRIGGER_INTERVAL_MAP:
            return scale_grade, attempt
        print(f"\r  [{attempt:5d}회] Scale grade 읽기 중 — 외부 전원 인가 대기...", end='', flush=True)
        time.sleep(RETRY_INTERVAL)


# ── 실시간 시각화 ──────────────────────────────────────────────────────────────

class RealtimeGraph:
    """curses 기반 실시간 시계열 그래프"""

    def __init__(self, stdscr, duration, jump_threshold):
        self.stdscr       = stdscr
        self.duration     = duration
        self.jump_threshold = jump_threshold

        # (elapsed_sec, distance_mm, is_jump)
        self.samples    = []
        self.fail_count = 0
        self.total_count = 0

        curses.curs_set(0)
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN,  -1)   # 정상값
        curses.init_pair(2, curses.COLOR_RED,    -1)   # 이상값
        curses.init_pair(3, curses.COLOR_CYAN,   -1)   # 축/레이블
        curses.init_pair(4, curses.COLOR_YELLOW, -1)   # 경고/도움말
        curses.init_pair(5, curses.COLOR_WHITE,  -1)   # 일반

    def add_sample(self, elapsed, distance):
        self.total_count += 1
        if distance is None:
            self.fail_count += 1
            self.samples.append((elapsed, None, False))
            return

        is_jump = False
        prev_valid = next(
            ((d for _, d, _ in reversed(self.samples) if d is not None)),
            None
        )
        if prev_valid is not None and abs(distance - prev_valid) > self.jump_threshold:
            is_jump = True

        self.samples.append((elapsed, distance, is_jump))

    def render(self, elapsed):
        self.stdscr.erase()
        h, w = self.stdscr.getmaxyx()

        # 레이아웃: 타이틀(1) + 그래프(h-8) + x축(1) + x레이블(1) + 구분선(1) + 통계(3) + 도움말(1)
        graph_h = h - 8
        graph_w = w - 11   # 좌측 Y축 레이블 10자 + 여백 1

        if graph_h < 4 or graph_w < 20:
            self.stdscr.addstr(0, 0, "터미널 창을 더 크게 조정하세요.")
            self.stdscr.refresh()
            return

        # ── 타이틀 ──────────────────────────────────────────
        title = "  DYP-A21 Dynamic Object Test  "
        self.stdscr.addstr(0, max(0, (w - len(title)) // 2),
                           title, curses.color_pair(3) | curses.A_BOLD)

        # ── 유효 샘플 추출 ───────────────────────────────────
        valid = [(t, d, j) for t, d, j in self.samples if d is not None]

        if not valid:
            self.stdscr.addstr(graph_h // 2 + 1, w // 2 - 8, "측정 대기 중...",
                               curses.color_pair(4))
            self.stdscr.refresh()
            return

        # ── Y 범위 계산 (슬라이딩 윈도우) ────────────────────
        window = valid[-graph_w:] if len(valid) > graph_w else valid
        dists  = [d for _, d, _ in window]
        margin = max(30, (max(dists) - min(dists)) * 0.15) if dists else 30
        y_min  = max(0, min(dists) - margin)
        y_max  = max(dists) + margin
        y_rng  = y_max - y_min or 1.0

        # ── Y축 레이블 + 수평 점선 ───────────────────────────
        tick_rows = [1, graph_h // 3, graph_h * 2 // 3, graph_h]
        for row in range(1, graph_h + 1):
            y_val = y_max - (row - 1) / max(graph_h - 1, 1) * y_rng
            if row in tick_rows:
                label = f"{y_val:6.0f} ┤"
                self.stdscr.addstr(row, 0, label, curses.color_pair(3))
                # 수평 점선
                try:
                    self.stdscr.addstr(row, 10, "·" * graph_w, curses.color_pair(3))
                except curses.error:
                    pass
            else:
                self.stdscr.addstr(row, 9, "│", curses.color_pair(3))

        # ── X축 ──────────────────────────────────────────────
        x_axis_row = graph_h + 1
        try:
            self.stdscr.addstr(x_axis_row, 9, "└" + "─" * graph_w, curses.color_pair(3))
        except curses.error:
            pass

        # X축 레이블 (시간)
        label_row = x_axis_row + 1
        t_start = window[0][0] if window else 0
        t_end   = window[-1][0] if window else self.duration
        t_span  = max(t_end - t_start, 1.0)
        for i in range(0, graph_w, max(1, graph_w // 6)):
            t_label = t_start + i / graph_w * t_span
            label = f"{t_label:.0f}s"
            col = 10 + i
            if col + len(label) < w:
                self.stdscr.addstr(label_row, col, label, curses.color_pair(3))

        # ── 데이터 점 그리기 ──────────────────────────────────
        n_window = len(window)
        for idx, (_, dist, is_jump) in enumerate(window):
            col = 10 + int(idx / max(n_window - 1, 1) * (graph_w - 1))
            row = 1 + int((y_max - dist) / y_rng * (graph_h - 1))
            row = max(1, min(graph_h, row))

            if 10 <= col < w - 1:
                if is_jump:
                    try:
                        self.stdscr.addstr(row, col, "!", curses.color_pair(2) | curses.A_BOLD)
                    except curses.error:
                        pass
                else:
                    try:
                        self.stdscr.addstr(row, col, "*", curses.color_pair(1))
                    except curses.error:
                        pass

        # 최신값 마커 ◄
        if valid:
            _, cur_dist, cur_jump = valid[-1]
            cur_row = 1 + int((y_max - cur_dist) / y_rng * (graph_h - 1))
            cur_row = max(1, min(graph_h, cur_row))
            marker_col = min(10 + graph_w, w - 2)
            color = curses.color_pair(2) if cur_jump else curses.color_pair(1)
            try:
                self.stdscr.addstr(cur_row, marker_col, "◄", color | curses.A_BOLD)
            except curses.error:
                pass

        # ── 통계 패널 ─────────────────────────────────────────
        stat_y = label_row + 1
        n      = len(valid)
        mean   = sum(d for _, d, _ in valid) / n
        std    = (sum((d - mean)**2 for _, d, _ in valid) / n) ** 0.5
        jump_n = sum(1 for _, _, j in valid if j)
        jump_r = jump_n / max(self.total_count, 1) * 100
        _, cur_dist, cur_jump = valid[-1]

        # 진행 바
        progress = min(elapsed / self.duration, 1.0)
        bar_w    = max(10, w - 22)
        filled   = int(bar_w * progress)
        bar      = "[" + "█" * filled + "░" * (bar_w - filled) + "]"
        try:
            self.stdscr.addstr(stat_y, 0,
                f" {elapsed:5.1f}s / {self.duration:.0f}s  {bar}",
                curses.color_pair(3))
        except curses.error:
            pass

        # 측정값 통계
        try:
            self.stdscr.addstr(stat_y + 1, 0,
                f" 현재:{cur_dist:5d}mm   평균:{mean:7.1f}mm   σ:{std:6.2f}mm   FAIL:{self.fail_count}회",
                curses.color_pair(1))
        except curses.error:
            pass

        # 이상 점프
        jump_color = curses.color_pair(2) if jump_n > 0 else curses.color_pair(1)
        try:
            self.stdscr.addstr(stat_y + 2, 0,
                f" 이상 점프:{jump_n:4d}회 ({jump_r:5.1f}%)   기준: >±{self.jump_threshold}mm 변화",
                jump_color)
        except curses.error:
            pass

        # 도움말
        try:
            self.stdscr.addstr(h - 1, 0, " Ctrl+C: 조기 종료", curses.color_pair(4))
        except curses.error:
            pass

        self.stdscr.refresh()


def _curses_main(stdscr, sensor, interval, duration, jump_threshold):
    graph = RealtimeGraph(stdscr, duration, jump_threshold)
    start = time.monotonic()
    stdscr.nodelay(True)

    try:
        while True:
            elapsed = time.monotonic() - start
            if elapsed >= duration:
                break

            distance = sensor.trigger()
            graph.add_sample(elapsed, distance)
            graph.render(elapsed)

            time.sleep(interval)
    except KeyboardInterrupt:
        pass

    return graph


# ── 결과 출력 ──────────────────────────────────────────────────────────────────

def print_final_result(graph, jump_rate_limit):
    valid  = [(t, d, j) for t, d, j in graph.samples if d is not None]
    n      = len(valid)
    print("\n" + "=" * 54)
    print("  [동적 물체 인식 검증 결과]")
    print(f"  총 측정 횟수  : {graph.total_count}")
    print(f"  유효 샘플     : {n}  (FAIL: {graph.fail_count})")

    if n > 0:
        dists     = [d for _, d, _ in valid]
        mean      = sum(dists) / n
        std       = (sum((d - mean)**2 for d in dists) / n) ** 0.5
        ptp       = max(dists) - min(dists)
        jump_n    = sum(1 for _, _, j in valid if j)
        jump_rate = jump_n / max(graph.total_count, 1) * 100
        pass_jump = jump_rate <= jump_rate_limit

        print(f"  평균 거리     : {mean:.1f} mm")
        print(f"  표준편차 (σ)  : {std:.2f} mm")
        print(f"  Peak-to-Peak  : {ptp} mm")
        print(f"  이상 점프     : {jump_n}회 ({jump_rate:.1f}%)"
              f"  {'✓' if pass_jump else '✗'}  (기준: ≤ {jump_rate_limit:.1f}%)")
        print("-" * 54)
        print(f"  최종 판정     : {'PASS' if pass_jump else 'FAIL'}")
    else:
        print("  유효한 샘플 없음 — FAIL")

    print("=" * 54)


def save_png(graph, output_path):
    """matplotlib으로 시계열 그래프를 PNG 파일로 저장."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("  (matplotlib 없음 — PNG 저장 건너뜀)")
        return

    valid = [(t, d, j) for t, d, j in graph.samples if d is not None]
    if not valid:
        print("  (유효 샘플 없음 — PNG 저장 건너뜀)")
        return

    ts_all = [t for t, _, _ in graph.samples]
    ds_all = [d for _, d, _ in graph.samples]   # None 포함

    ts_v   = [t for t, d, _ in graph.samples if d is not None]
    ds_v   = [d for _, d, _ in valid]

    jump_pts = [(t, d) for t, d, j in valid if j]

    fig, ax = plt.subplots(figsize=(14, 5))

    # 측정 라인
    ax.plot(ts_v, ds_v, color='steelblue', linewidth=1.0, label='Distance (mm)', zorder=2)
    ax.scatter(ts_v, ds_v, s=8, color='steelblue', zorder=3)

    # FAIL(None) 구간 표시
    for t, d in zip(ts_all, ds_all):
        if d is None:
            ax.axvline(x=t, color='orange', linewidth=0.6, alpha=0.6)

    # 이상 점프 표시
    if jump_pts:
        jt, jd = zip(*jump_pts)
        ax.scatter(jt, jd, s=100, color='red', marker='x', linewidths=2.0,
                   label=f'Jump (n={len(jump_pts)})', zorder=5)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (mm)')
    ax.set_title('DYP-A21 Dynamic Object Test — Time Series')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # 통계 텍스트 박스
    n     = len(ds_v)
    mean  = sum(ds_v) / n
    std   = (sum((d - mean)**2 for d in ds_v) / n) ** 0.5
    info  = (f"n={n}  mean={mean:.1f}mm\n"
             f"σ={std:.2f}mm  jumps={len(jump_pts)}")
    ax.text(0.01, 0.97, info, transform=ax.transAxes,
            verticalalignment='top', fontsize=8,
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    print(f"  그래프 저장됨: {output_path}")


# ── 진입점 ─────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="DYP-A21 동적 물체 인식 검증 테스트",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--duration",        type=float, default=DEFAULT_DURATION,
                        help=f"측정 시간 (초, 기본값: {DEFAULT_DURATION})")
    parser.add_argument("--jump-threshold",  type=int,   default=DEFAULT_JUMP_THRESHOLD,
                        help=f"이상 점프 감지 기준 (mm, 기본값: {DEFAULT_JUMP_THRESHOLD})")
    parser.add_argument("--jump-rate-limit", type=float, default=DEFAULT_JUMP_RATE_LIMIT,
                        help=f"이상 점프 비율 합격 기준 (%%, 기본값: {DEFAULT_JUMP_RATE_LIMIT})")
    parser.add_argument("--no-save",         action="store_true",
                        help="PNG 저장 건너뜀")
    parser.add_argument("--output",          default="test_dynamic_result.png",
                        help="저장할 PNG 파일명 (기본값: test_dynamic_result.png)")
    return parser.parse_args()


def main():
    args = parse_args()

    print("=" * 54)
    print("  DYP-A21 동적 물체 인식 검증 테스트")
    print(f"  측정 시간    : {args.duration:.0f}초")
    print(f"  점프 기준    : >±{args.jump_threshold}mm 변화")
    print(f"  합격 기준    : 이상 점프 비율 ≤ {args.jump_rate_limit:.1f}%")
    print("=" * 54)

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

        print(f"측정 시작 — FOV 내에서 물체를 자유롭게 움직이세요. ({args.duration:.0f}초)\n")
        input("준비되면 Enter를 누르세요...")

        # 실시간 그래프 실행
        graph = curses.wrapper(
            _curses_main,
            sensor,
            trigger_interval,
            args.duration,
            args.jump_threshold,
        )

        # 최종 결과
        print_final_result(graph, args.jump_rate_limit)

        # PNG 저장
        if not args.no_save:
            output = os.path.join(os.path.dirname(os.path.abspath(__file__)), args.output)
            save_png(graph, output)

    except KeyboardInterrupt:
        print("\n\n테스트 중단됨.")
    except serial.SerialException:
        print("\n포트 연결 끊김.")
    finally:
        if sensor:
            sensor.disconnect()


if __name__ == "__main__":
    main()
