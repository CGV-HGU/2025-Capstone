#!/usr/bin/env python3
"""Rebuild two manuscript figures from an archived hallway run and a glare clip."""

import argparse
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np
from ultralytics import YOLO

from render_archive_figures import METADATA, read_archive_frame
from render_pipeline_figure import costmap_in_robot_frame, read_snapshot
from replay_scan_filter import MODEL_DIR, ReplayScan


FIGURES = Path(__file__).resolve().parents[1] / "manuscript/figures"
REFLECTION_STAMP = 1790932901883577839
PATH_COLOR = "#08777b"
FLOOR_COLOR = "#0dbb9f"
REMOVED_COLOR = "#ef5929"


def hallway_figure(bag, destination):
    rgb, scan, costmap, path, odom, transform = read_snapshot(bag, 37)
    x, y, cost, px, py = costmap_in_robot_frame(costmap, path, odom, transform)

    figure, (camera, axis, bar) = plt.subplots(1, 3, figsize=(6.0, 2.85), layout="constrained",
                                              gridspec_kw={"width_ratios": (1.36, 1, .075)})
    camera.imshow(rgb)
    camera.set_axis_off()
    camera.set_title("(a) Recorded camera view", fontsize=9.5)

    colors = LinearSegmentedColormap.from_list("cost", ("#fafbfc", "#d1bde7", "#8a62ae", "#382050"))
    mesh = axis.pcolormesh(x, y, np.ma.masked_less(cost, 0), cmap=colors, vmin=0, vmax=100,
                           shading="flat", edgecolors="face", linewidth=.12)
    axis.plot(px, py, color=PATH_COLOR, lw=1.75, label="Local path")
    axis.scatter(0, 0, marker="^", color="#152b3b", s=31, edgecolor="white", linewidth=.55, label="Robot")
    axis.set(title="(b) Recorded local costmap", xlim=(1.75, -1.75), ylim=(-.28, 3.95), aspect="equal",
             xlabel="Lateral (m)", ylabel="Forward (m)", xticks=(1, 0, -1), yticks=(0, 1, 2, 3))
    axis.set_facecolor("#e4e6e8")
    axis.title.set_fontsize(9.5)
    axis.tick_params(labelsize=8)
    axis.xaxis.label.set_fontsize(8.5)
    axis.yaxis.label.set_fontsize(8.5)
    axis.legend(loc="upper center", bbox_to_anchor=(.5, -.22), ncol=2, fontsize=8, frameon=False)

    colorbar = figure.colorbar(mesh, cax=bar, ticks=(0, 50, 100))
    colorbar.set_label("Cost", size=8.5)
    bar.tick_params(labelsize=8)
    print(f"{bag.name}: {len(scan.ranges)} synchronized scan channels; {len(path.poses)} local path poses")
    figure.savefig(destination, dpi=330, metadata=METADATA)
    plt.close(figure)


def reflection_figure(database, destination):
    rgb = np.array(read_archive_frame(database, REFLECTION_STAMP))
    deployed = ReplayScan()
    bypass = ReplayScan()
    bypass.min_floor_scan_pixels = 0
    model = YOLO(str(MODEL_DIR / "best_openvino_model"), task="segment")
    inferred = deployed.infer(model, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    retained = deployed.floor_mask(inferred)
    removed = (bypass.floor_mask(inferred) > 0) & (retained == 0)
    if not np.any(removed):
        raise ValueError(f"{database}: the selected frame no longer shows a rejected mask")
    print(f"{database.parent.name}: RGB {REFLECTION_STAMP} ns; {removed.sum()} removed pixels on the model grid")

    height, width = rgb.shape[:2]
    keep = cv2.resize(retained, (width, height), interpolation=cv2.INTER_NEAREST).astype(bool)
    reject = cv2.resize(removed.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST).astype(bool)
    figure, axes = plt.subplots(1, 2, figsize=(6.0, 2.65))
    figure.subplots_adjust(left=.01, right=.99, top=.86, bottom=.17, wspace=.05)
    for axis, show_removed, title in zip(axes, (True, False),
                                        ("(a) Lower-band selection bypassed", "(b) Deployed selection")):
        axis.imshow(rgb)
        overlay = np.zeros((height, width, 4), dtype=np.float32)
        overlay[keep] = (*matplotlib.colors.to_rgb(FLOOR_COLOR), .34)
        if show_removed:
            overlay[reject] = (*matplotlib.colors.to_rgb(REMOVED_COLOR), .62)
        axis.imshow(overlay)
        axis.axhline(deployed.y_min_scan * height / deployed.target_h - .5, color="#ffffff", ls="--", lw=1)
        axis.set_title(title, fontsize=9.5)
        axis.set_axis_off()
    figure.legend(handles=(Patch(color=FLOOR_COLOR, label="Retained floor mask"),
                           Patch(color=REMOVED_COLOR, label="Rejected upper mask"),
                           Line2D([], [], color="0.45", ls="--", label="Lower-band boundary")),
                  loc="lower center", bbox_to_anchor=(.5, .03), ncol=3, fontsize=8.5, frameon=False)
    figure.savefig(destination, dpi=330, metadata=METADATA)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hall-bag", type=Path, default=Path.home() / "data/bags/nav_trail_1_26")
    parser.add_argument("--glare-db", type=Path,
                        default=Path.home() / ".local/share/Trash/files/vlidar_run01.4/vlidar_run01_0.db3")
    parser.add_argument("--output", type=Path, default=FIGURES)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    hallway_figure(args.hall_bag, args.output / "archive_nav2_view.pdf")
    reflection_figure(args.glare_db, args.output / "archive_floor_reflection.pdf")


if __name__ == "__main__":
    main()
