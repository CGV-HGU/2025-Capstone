#!/usr/bin/env python3
"""Check the cart's image-boundary channels against the recorded scan and local costmap."""

import argparse
from contextlib import closing
import csv
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Circle
import numpy as np
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from ultralytics import YOLO

from bag_sqlite import bag_messages, bag_metadata
from render_archive_figures import METADATA
from render_pipeline_figure import costmap_in_robot_frame, header_ns, read_snapshot
from replay_scan_filter import MODEL_DIR, ReplayScan


SAMPLES = ((9.8, 1795754055816735954, (52, 62), (349, 405, 390, 411)),
           (12.8, 1795754058748662494, (97, 114), (108, 196, 450, 475)))
RADIUS_M = .25
AMBER = "#b16213"


def scan_offset(bag):
    info = bag_metadata(bag)
    start = info["starting_time"]["nanoseconds_since_epoch"]
    offsets = set()
    with closing(bag_messages(bag, info, ("/tf", "/tf_static"), (start, start + 15_000_000_000))) as records:
        for _, _, payload in records:
            for tf in deserialize_message(payload, TFMessage).transforms:
                if (tf.header.frame_id, tf.child_frame_id) != ("base_link", "lidar_link"):
                    continue
                p, q = tf.transform.translation, tf.transform.rotation
                if (q.x, q.y, q.z, abs(q.w)) != (0., 0., 0., 1.):
                    raise ValueError("expected parallel base and scan axes")
                offsets.add((p.y, p.x))
    if len(offsets) != 1:
        raise ValueError("expected one recorded base-to-scan offset")
    return offsets.pop()


def select_boundary(replay, model, rgb, scan, indices, region):
    floor = replay.segment_floor(model, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    rows = np.where(floor[replay.y_min_scan:] == 0, replay.row_indices, -1).max(axis=0)
    u = np.flatnonzero(rows >= 0)
    v = rows[u]
    bins = len(scan.ranges) - 1 - replay.col_to_ch_lut[u]
    distances = replay.distance_lut[v, u]
    minima = np.full(len(scan.ranges), np.inf)
    np.minimum.at(minima, bins, distances)
    px = (u + .5) * rgb.shape[1] / floor.shape[1] - .5
    py = (v + .5) * rgb.shape[0] / floor.shape[0] - .5
    x0, x1, y0, y1 = region
    inside = (x0 <= px) & (px <= x1) & (y0 <= py) & (py <= y1)
    select = inside & np.isin(bins, indices) & (distances == minima[bins])
    if set(bins[select]) != set(indices):
        raise ValueError("the annotated cart region no longer accounts for all selected LUT bins")
    return px[select], py[select], minima[indices]


def cart_scan_page(bag, output):
    lateral_offset, forward_offset = scan_offset(bag)
    replay = ReplayScan()
    model = YOLO(str(MODEL_DIR / "best_openvino_model"), task="segment")
    fig, axes = plt.subplots(2, 2, figsize=(5.15, 4.1), layout="constrained",
                              gridspec_kw={"width_ratios": (1.45, 1)})
    fig.suptitle("S7. Cart boundary, virtual scan and local costmap", fontsize=11)
    fig.supxlabel("Highlighted image pixels and scan endpoints are amber. The path is teal.", fontsize=7.6)
    color = LinearSegmentedColormap.from_list("cost", ("#faf9fc", "#c6afd6", "#4a2d68"))
    rows = []
    for row, (seconds, stamp, (first, last), region) in enumerate(SAMPLES):
        rgb, scan, costmap, path, odom, transform = read_snapshot(bag, seconds)
        if header_ns(scan) != stamp:
            raise ValueError("unexpected source stamp for the cart annotation")
        indices = np.arange(first, last + 1)
        px, py, lut = select_boundary(replay, model, rgb, scan, indices, region)
        ranges = np.asarray(scan.ranges)[indices]
        if not np.all(np.isfinite(ranges) & (scan.range_min <= ranges) & (ranges <= scan.range_max)):
            raise ValueError("a selected cart angular bin has an invalid recorded range")
        theta = scan.angle_min + scan.angle_increment * indices
        lateral = lateral_offset + ranges * np.sin(theta)
        forward = forward_offset + ranges * np.cos(theta)
        mid_x, mid_y = np.median(lateral), np.median(forward)
        x, y, cost, path_x, path_y = costmap_in_robot_frame(costmap, path, odom, transform)
        cx = (x[:-1, :-1] + x[1:, 1:]) / 2
        cy = (y[:-1, :-1] + y[1:, 1:]) / 2
        patch = np.hypot(cx - mid_x, cy - mid_y) <= RADIUS_M
        costs = cost[patch]
        rows.append({"bag_id": bag.name, "requested_s": seconds, "rgb_scan_acquisition_ns": stamp,
                     "first_bin_index": first, "last_bin_index": last,
                     "recorded_min_m": round(float(ranges.min()), 6),
                     "recorded_max_m": round(float(ranges.max()), 6),
                     "max_abs_difference_to_unfiltered_lut_m": round(float(np.abs(ranges - lut).max()), 6),
                     "median_endpoint_lateral_m": round(float(mid_x), 6),
                     "median_endpoint_forward_m": round(float(mid_y), 6),
                     "radius_m": RADIUS_M, "grid_cells": len(costs),
                     "inscribed_cells": np.count_nonzero(costs == 99),
                     "lethal_cells": np.count_nonzero(costs == 100)})

        image, grid = axes[row]
        image.imshow(rgb)
        image.scatter(px, py, s=6, c="#ffc15a", edgecolor="#392001", linewidth=.3)
        image.set_title(("9.923 s · Cart ahead", "12.865 s · Cart to the left")[row], fontsize=9.4)
        image.set_axis_off()
        grid.pcolormesh(x, y, np.ma.masked_less(cost, 0), cmap=color, vmin=0, vmax=100,
                        shading="flat", rasterized=True)
        grid.plot(path_x, path_y, color="#00797b", lw=1.5)
        grid.scatter([0], [0], color="#182d42", marker="^", s=23)
        grid.scatter(lateral, forward, c="#ffc15a", edgecolor="#392001", linewidth=.25, s=8, zorder=4)
        grid.add_patch(Circle((mid_x, mid_y), RADIUS_M, ec=AMBER, fill=False, lw=1.1, zorder=5))
        grid.set(xlim=(1.55, -1.55), ylim=(-.25, 3.35), aspect="equal", xticks=(1, 0, -1), yticks=(0, 1, 2, 3))
        grid.tick_params(labelsize=7.4)
        grid.set_title("Recorded local map and path", fontsize=9.4)
        grid.set_ylabel("Forward (m)", fontsize=8)
        if row == 1:
            grid.set_xlabel("Lateral (m)", fontsize=8)

    output.mkdir(parents=True, exist_ok=True)
    with (output / "bridge_session_2026-11-27_cart_scan.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    return fig


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, default=Path.home() / "data/bags/nav_c_bridge_02")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "evidence")
    args = parser.parse_args()
    figure = cart_scan_page(args.bag, args.output)
    figure.savefig(args.output / "bridge_session_cart_scan.pdf", dpi=320, metadata=METADATA)
    plt.close(figure)
