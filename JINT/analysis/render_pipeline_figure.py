#!/usr/bin/env python3
"""Render the V-LiDAR pipeline figure from one archived RGB frame."""

import argparse
from pathlib import Path
from types import SimpleNamespace

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Patch
import numpy as np
from ultralytics import YOLO

from render_archive_figures import bag_frame
from replay_scan_filter import MODEL_DIR, ReplayScan, RoiChecker, valid_ranges


FIGURE = Path(__file__).resolve().parents[1] / "manuscript/figures/pipeline.pdf"
NAVY = "#182840"
TEAL = "#1b7181"
ORANGE = "#f4a01a"


def replay_projection(rgb):
    model = YOLO(str(MODEL_DIR / "best_openvino_model"), task="segment")
    replay = ReplayScan()
    floor = replay.segment_floor(model, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    # Downstream publication is disconnected in ReplayScan.
    RoiChecker.publish_channel_distances(replay, floor, SimpleNamespace(sec=0, nanosec=0))
    rows = np.where(floor[replay.y_min_scan:] == 0, replay.row_indices, -1).max(axis=0)
    angles = np.deg2rad(np.linspace(-35, 35, replay.num_channels))
    ranges = replay.raw[::-1]  # Same left/right ordering as the published LaserScan.
    finite = valid_ranges(ranges)
    if not floor.any() or not finite.any():
        raise ValueError("choose a frame with a floor mask and a projected obstacle")
    return floor, rows, ranges[finite] * np.sin(angles[finite]), ranges[finite] * np.cos(angles[finite])


def arrow(figure, x0, x1, y):
    figure.add_artist(FancyArrowPatch((x0, y), (x1, y), transform=figure.transFigure,
                                     arrowstyle="-|>", color="#607585", linewidth=.75,
                                     mutation_scale=7, shrinkA=0, shrinkB=0))


def figure_from_frame(rgb, destination):
    floor, rows, lateral, forward = replay_projection(rgb)
    figure = plt.figure(figsize=(5.28, 2.38))
    left = figure.add_axes((.018, .349, .282, .494))
    middle = figure.add_axes((.355, .349, .282, .494))
    scan = figure.add_axes((.754, .392, .224, .411))
    for axis in (left, middle):
        axis.imshow(rgb)
        axis.set_axis_off()

    height, width = rgb.shape[:2]
    overlay = np.empty((height, width, 4), dtype=float)
    overlay[:, :, :3] = matplotlib.colors.to_rgb("#039f9f")
    overlay[:, :, 3] = .36 * cv2.resize(floor, (width, height), interpolation=cv2.INTER_NEAREST)
    middle.imshow(overlay)
    columns = np.flatnonzero(rows >= 0)[::2]
    image_x = (columns + .5) * width / floor.shape[1] - .5
    image_y = (rows[columns] + .5) * height / floor.shape[0] - .5
    middle.scatter(image_x, image_y, c=ORANGE, edgecolors=NAVY, linewidths=.28, s=5.8)

    largest_forward = max(4, int(np.ceil(forward.max() + .27)))
    half_width = max(2, int(np.ceil(np.abs(lateral).max() + .17)))
    scan.set(xlim=(-half_width, half_width), ylim=(-.15, largest_forward))
    scan.set_xticks([-half_width, 0, half_width])
    scan.set_yticks([0, largest_forward / 2, largest_forward])
    scan.set_xlabel("Lateral (m)", color=NAVY, fontsize=7.6, labelpad=2)
    scan.set_ylabel("Forward (m)", color=NAVY, fontsize=7.6, labelpad=2)
    scan.tick_params(labelsize=7, colors=NAVY, length=2, pad=1.5)
    scan.set_facecolor("#f4f8fa")
    scan.grid(lw=.5, color="#d6e1e8", zorder=0)
    scan.spines[["top", "right"]].set_visible(False)
    scan.spines[["bottom", "left"]].set_color("#607585")
    scan.scatter(lateral, forward, c=ORANGE, edgecolors=NAVY, linewidths=.23, s=8.5, zorder=3)
    scan.scatter([0], [0], marker="^", s=19, c=NAVY, zorder=4)

    for x, title in ((.018, "(a) RGB image"), (.355, "(b) Floor boundary"), (.695, "(c) Planar projection")):
        figure.text(x, .92, title, color=NAVY, fontsize=9.2, fontweight="semibold", va="center")
    arrow(figure, .312, .348, .596)
    arrow(figure, .647, .684, .596)

    middle.legend(handles=(
        Patch(facecolor=TEAL, label="Floor mask"),
        Line2D([], [], marker="o", markersize=3.5, markerfacecolor=ORANGE,
               markeredgecolor=NAVY, markeredgewidth=.3, linestyle="none", label="Selected pixels"),
    ), loc="upper center", bbox_to_anchor=(.5, -.052), ncol=2, frameon=False, fontsize=7.2,
        labelcolor=NAVY, handlelength=.8, handletextpad=.35, columnspacing=.8, borderaxespad=0)

    figure.add_artist(FancyBboxPatch((.018, .03), .96, .142, transform=figure.transFigure,
                                     boxstyle="round,pad=0,rounding_size=.016", facecolor="#eaf1f5",
                                     linewidth=0))
    for x, step in ((.168, "Temporal filtering"), (.495, "ROS 2 LaserScan"), (.819, "Nav2 obstacle layer")):
        figure.text(x, .101, step, color=NAVY, fontsize=8.3, fontweight="semibold", ha="center", va="center")
    arrow(figure, .292, .351, .101)
    arrow(figure, .617, .676, .101)

    figure.savefig(destination, dpi=330, metadata={"Title": "V-LiDAR pipeline", "CreationDate": None, "ModDate": None})
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("seconds", type=float, help="recorder offset in seconds")
    parser.add_argument("--output", type=Path, default=FIGURE)
    args = parser.parse_args()
    rgb, _ = bag_frame(args.bag, args.seconds)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure_from_frame(np.asarray(rgb), args.output)


if __name__ == "__main__":
    main()
