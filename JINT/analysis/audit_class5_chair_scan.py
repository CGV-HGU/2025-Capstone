#!/usr/bin/env python3
"""Trace selected image regions below the chair seat into the scan and the published local grid."""

import argparse
import csv
from pathlib import Path

from cv_bridge import CvBridge
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Circle
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from ultralytics import YOLO

from audit_bridge_cart_scan import scan_offset, select_boundary
from bag_sqlite import bag_messages, bag_metadata
from render_archive_figures import METADATA
from render_pipeline_figure import costmap_in_robot_frame, read_snapshot
from replay_scan_filter import MODEL_DIR, ReplayScan
from scan_recorder_timing import IMAGE, SCAN, header_timestamp


JINT = Path(__file__).resolve().parents[1]
# Bag-relative sample targets, exact acquisition stamps, consecutive bins and RGB pixel bounds.
CHAIR_SAMPLES = ((13.5, 1795766760867189938, (68, 75), (290, 330, 400, 415)),
                 (16.2, 1795766763567534985, (119, 127), (39, 88, 409, 430)))
PATCH_RADIUS_M = 0.25


def replay_recorded_sequence(bag, model):
    info = bag_metadata(bag)
    start = info["starting_time"]["nanoseconds_since_epoch"]
    stop = start + 17_000_000_000
    targets = {sample[1] for sample in CHAIR_SAMPLES}
    matched = {header_timestamp(payload) for _, _, payload in
               bag_messages(bag, info, (SCAN,), (start, stop), payload_bytes=12)}
    matched.discard(0)
    bridge, session = CvBridge(), ReplayScan()
    filtered_samples = {}
    count = 0
    for _, _, payload in bag_messages(bag, info, (IMAGE,), (start, stop)):
        stamp = header_timestamp(payload[:12])
        if stamp not in matched:
            continue
        matched.remove(stamp)
        image = deserialize_message(payload, Image)
        _, filtered = session.run(model, bridge.imgmsg_to_cv2(image, "bgr8"), image.header.stamp)
        count += 1
        if stamp in targets:
            filtered_samples[stamp] = (count, filtered[::-1])
    if filtered_samples.keys() != targets:
        raise ValueError("missing a target image from the sequential chair replay")
    return filtered_samples


def draw_chair_trace(bag, figure_path):
    left, forward = scan_offset(bag)
    model = YOLO(str(MODEL_DIR / "best_openvino_model"), task="segment")
    filtered_samples = replay_recorded_sequence(bag, model)
    replay = ReplayScan()
    figure, axes = plt.subplots(2, 2, figsize=(6.15, 4.3), layout="constrained",
                                gridspec_kw={"width_ratios": (1.45, 1)})
    colors = LinearSegmentedColormap.from_list("cost", ("#f8f6fb", "#bc9dd1", "#42295d"))
    evidence = []
    for index, ((seconds, stamp, (first, last), region), (image_axis, grid_axis)) in enumerate(zip(CHAIR_SAMPLES, axes)):
        timing = {}
        rgb, scan, grid, path, odom, transform = read_snapshot(bag, seconds, timing)
        if timing["acquisition_ns"] != stamp:
            raise ValueError("unexpected chair acquisition stamp")
        bins = np.arange(first, last + 1)
        px, py, raw = select_boundary(replay, model, rgb, scan, bins, region)
        recorded = np.asarray(scan.ranges)
        ranges = recorded[bins]
        if not np.all(np.isfinite(ranges) & (scan.range_min <= ranges) & (ranges <= scan.range_max)):
            raise ValueError("chair region contains a bin without a valid recorded range")
        angle = scan.angle_min + bins * scan.angle_increment
        scan_x, scan_y = left + ranges * np.sin(angle), forward + ranges * np.cos(angle)
        center_x, center_y = np.median(scan_x), np.median(scan_y)
        x, y, costs, path_x, path_y = costmap_in_robot_frame(grid, path, odom, transform)
        cells_x = (x[:-1, :-1] + x[1:, 1:]) / 2
        cells_y = (y[:-1, :-1] + y[1:, 1:]) / 2
        patch = costs[np.hypot(cells_x - center_x, cells_y - center_y) <= PATCH_RADIUS_M]
        replay_frame, filtered = filtered_samples[stamp]
        matched = np.isclose(filtered, recorded, rtol=0, atol=0.0001)
        evidence.append({"bag_id": bag.name, **timing,
                         "first_bin": first, "last_bin": last, "region_px": " ".join(map(str, region)),
                         "recorded_range_min_m": round(float(ranges.min()), 6),
                         "recorded_range_max_m": round(float(ranges.max()), 6),
                         "max_delta_to_unfiltered_lut_m": round(float(np.abs(ranges - raw).max()), 6),
                         "sequential_replay_frames": replay_frame,
                         "replay_channels_matching_at_0_1mm": np.count_nonzero(matched),
                         "replay_total_channels": matched.size,
                         "grid_patch_radius_m": PATCH_RADIUS_M, "grid_cells": patch.size,
                         "cost_99": np.count_nonzero(patch == 99), "cost_100": np.count_nonzero(patch == 100)})

        image_axis.imshow(rgb)
        image_axis.scatter(px, py, s=12, c="#ffbe54", edgecolor="#4f2906", linewidth=.45)
        image_axis.set_axis_off()
        image_axis.set_title(f"({chr(97 + 2 * index)}) RGB at {timing['rgb_elapsed_s']:.3f} s", fontsize=9)
        grid_axis.pcolormesh(x, y, np.ma.masked_less(costs, 0), cmap=colors, vmin=0, vmax=100,
                             shading="flat", rasterized=True)
        grid_axis.plot(path_x, path_y, c="#00797b", lw=1.7)
        grid_axis.scatter(0, 0, color="#182d42", marker="^", s=25)
        grid_axis.scatter(scan_x, scan_y, c="#ffbe54", edgecolor="#4f2906", linewidth=.35, s=12)
        grid_axis.add_patch(Circle((center_x, center_y), PATCH_RADIUS_M, ec="#9a590b", fill=False, lw=1.0))
        grid_axis.set(xlim=(1.9, -1.9), ylim=(-.25, 3.45), aspect="equal")
        grid_axis.set_title(f"({chr(98 + 2 * index)}) Local grid and path", fontsize=9)
        grid_axis.set_ylabel("Forward (m)", fontsize=8)
        grid_axis.tick_params(labelsize=7)
        if index == 1:
            grid_axis.set_xlabel("Lateral (m)", fontsize=8)
    figure.savefig(figure_path, dpi=300, metadata=METADATA)
    plt.close(figure)
    return evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, default=Path.home() / "data/bags/nav_class5_20")
    parser.add_argument("--figure", type=Path, default=JINT / "manuscript/figures/class5_chair_scan.pdf")
    parser.add_argument("--output", type=Path, default=JINT / "analysis/evidence/class5_20_chair_scan.csv")
    args = parser.parse_args()
    evidence = draw_chair_trace(args.bag, args.figure)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=evidence[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(evidence)


if __name__ == "__main__":
    main()
