"""Plot two identified channel episodes from the arrays distributed in Online Resource 2."""

import argparse
from io import BytesIO
from pathlib import Path
from zipfile import ZipFile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from scan_filter_metrics import valid_ranges


ROOT = Path(__file__).resolve().parents[1]
INK, MUTED, HIGHLIGHT = "#203442", "#637785", "#166968"
EXAMPLES = (
    ("bridge_pilot_1", 205, 28, "(a)  Delayed confirmation", "confirmed"),
    ("bridge_pilot_2", 215, 55, "(b)  Return lost before confirmation", "lost"),
)


def draw_episode(ax, archive, source, onset, channel, title, expected):
    with np.load(BytesIO(archive.read(source + ".npz")), allow_pickle=False) as arrays:
        raw = arrays["bypass_m"][:, channel]
        filtered = arrays["enabled_m"][:, channel]
        time = (arrays["acquisition_elapsed_ns"] - arrays["acquisition_elapsed_ns"][onset]) / 1e6
    raw_valid, filtered_valid = valid_ranges(raw), valid_ranges(filtered)
    if raw_valid[onset-1] or not raw_valid[onset] or filtered_valid[onset]:
        raise ValueError(f"{source}: selected frame is not a withheld onset")
    stop = onset + 1
    while stop < len(raw) and raw_valid[stop] and not filtered_valid[stop]:
        stop += 1
    if stop == len(raw) or ("confirmed" if raw_valid[stop] else "lost") != expected:
        raise ValueError(f"{source}: outcome of the selected episode changed")
    delay = time[stop]
    shown = (time >= -155) & (time <= 635)
    ax.axvspan(0, delay, color="#e3efed", zorder=0)
    for x in (0, delay):
        ax.axvline(x, color="#88aaa6", lw=.65, ls=(0, (2, 2)), zorder=1)
    for y, valid in ((1, raw_valid), (0, filtered_valid)):
        ax.axhline(y, color="#d8dfe1", lw=.65, zorder=1)
        ax.scatter(time[shown & valid], np.full((shown & valid).sum(), y),
                   s=28, c=HIGHLIGHT, zorder=3)
        ax.scatter(time[shown & ~valid], np.full((shown & ~valid).sum(), y),
                   s=22, c=MUTED, marker="x", linewidths=1.0, zorder=3)
    ax.annotate(f"{raw[onset]:.2f} m", (0, 1), xytext=(0, 6), textcoords="offset points",
                ha="center", color=INK, fontsize=8.1)
    if expected == "confirmed":
        ax.annotate(f"{filtered[stop]:.2f} m", (delay, 0), xytext=(0, 6),
                    textcoords="offset points", ha="center", color=INK, fontsize=8.1)
    ax.text(.995, 1.07, f"{delay:.0f} ms to " + ("confirmation" if expected == "confirmed" else "loss"),
            transform=ax.transAxes, ha="right", fontsize=8.1, color=HIGHLIGHT)
    ax.set_title(title, loc="left", color=INK, fontsize=9.2, fontweight="semibold", pad=5)
    ax.set(yticks=[1, 0], yticklabels=["Bypass", "Filtered"], ylim=(-.43, 1.52), xlim=(-155, 635))
    ax.tick_params(axis="y", colors=INK, labelsize=8, length=0, pad=5)
    ax.tick_params(axis="x", colors=MUTED, labelsize=8.1, length=3)
    ax.spines[["left", "right", "top"]].set_visible(False)
    ax.spines["bottom"].set_color("#a4b0b6")
    print(f"{source}: column {channel}, frames {onset} to {stop}, {delay:.3f} ms ({expected})")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--supplement", type=Path, default=ROOT / "analysis/evidence/ESM_2.zip")
    parser.add_argument("--output", type=Path, default=ROOT / "manuscript/figures/filter_examples.pdf")
    args = parser.parse_args()
    with plt.rc_context({"pdf.fonttype": 42, "font.family": "DejaVu Sans"}):
        figure, axes = plt.subplots(2, 1, figsize=(5.13, 2.83), sharex=True)
        figure.subplots_adjust(left=.15, right=.98, bottom=.245, top=.90, hspace=.72)
        with ZipFile(args.supplement) as archive:
            for axis, scene in zip(axes, EXAMPLES):
                draw_episode(axis, archive, *scene)
        axes[0].spines["bottom"].set_visible(False)
        axes[0].tick_params(axis="x", bottom=False)
        axes[1].set_xticks([-100, 0, 100, 200, 300, 400, 500, 600])
        axes[1].set_xlabel("Time from first withheld return (ms)", fontsize=8.2, color=INK, labelpad=3)
        handles = [Line2D([], [], ls="none", marker="o", ms=5, color=HIGHLIGHT,
                          label="Return within scan range"),
                   Line2D([], [], ls="none", marker="x", ms=5, color=MUTED,
                          label="No in-range return")]
        figure.legend(handles=handles, loc="lower center", ncol=2, frameon=False, fontsize=8.1,
                      handlelength=1.0, handletextpad=.35, columnspacing=2.3)
        figure.savefig(args.output, metadata={"Title": "Withheld virtual scan returns", "CreationDate": None})
        plt.close(figure)


if __name__ == "__main__":
    main()
