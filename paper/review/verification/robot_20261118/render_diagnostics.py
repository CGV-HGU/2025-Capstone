#!/usr/bin/env python3
"""저장된 점검 JSON에서 그림 생성. 원격 접속·ROS 실행·성공 판정 없음."""

import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager


def main():
    root = Path(__file__).resolve().parent
    runs = json.loads((root / "twenty_runs.json").read_text())["runs"]
    font_path = Path("/usr/share/fonts/truetype/nanum/NanumGothic.ttf")
    if font_path.exists():
        font_manager.fontManager.addfont(str(font_path))
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(font_path)).get_name()
    plt.rcParams["axes.unicode_minus"] = False

    x = list(range(1, len(runs) + 1))
    colors = ["#b74732" if i in (8, 12) else "#3e6f89" for i in x]
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, layout="constrained")
    fig.suptitle("기존 20회 rosbag 진단 · 2026-09-21 취득 / 2026-11-18 점검", fontsize=15)

    axes[0].bar(x, [r["end_x_m"] for r in runs], color=colors)
    axes[0].set_ylabel("종료 odom x (m)")
    axes[0].set_title("종료 위치는 대리 지표이며, 목표 도달·접촉·개입 여부를 증명하지 않음", fontsize=10)
    for i in (8, 12):
        r = runs[i - 1]
        axes[0].text(i, r["end_x_m"] + .25, f'{r["end_x_m"]:.2f}', ha="center", fontsize=9)
    axes[0].set_ylim(0, 11)

    axes[1].bar(x, [r["stationary_tail_s"] for r in runs], color=colors)
    axes[1].set_ylabel("끝부분 정지 시간 (초)")
    axes[1].set_title("움직임 기준: odom |vx| > 0.01 m/s 또는 |wz| > 0.02 rad/s", fontsize=10)
    for i in (8, 12):
        r = runs[i - 1]
        axes[1].text(i, r["stationary_tail_s"] + 2, f'{r["stationary_tail_s"]:.1f}초', ha="center", fontsize=9)
    axes[1].set_ylim(0, 110)

    axes[2].plot(x, [r["scan_hz"] for r in runs], "o-", color="#3e6f89", label="scan 발행률")
    axes[2].plot(x, [r["channel_hz"] for r in runs], "s-", color="#ac7224", label="채널 배열 수신률")
    axes[2].set_ylabel("기록 시각 기준 (Hz)")
    axes[2].set_xlabel("주행 기록 번호")
    axes[2].set_ylim(7.8, 10.5)
    axes[2].legend(loc="center right")
    axes[2].set_title("발행률과 새 입력률을 구분 · 영상 획득부터의 지연 측정은 아님", fontsize=10)
    axes[2].set_xticks(x)

    for ax in axes:
        ax.grid(axis="y", alpha=.2)
        ax.set_axisbelow(True)
        ax.spines[["top", "right"]].set_visible(False)

    for suffix in ("png", "pdf"):
        fig.savefig(root / f"twenty_run_diagnostics.{suffix}", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
