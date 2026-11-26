#!/usr/bin/env python3
"""Render an improved V-LiDAR system architecture figure (Figure 1 v2).

Upgrades over render_pipeline_figure.py:
1. Fixes Left-Right mirror reversal in panels (b) and (c) by inverting the
   ROS +y (left) lateral axis so image left == scan left == costmap left.
2. Uses equal metric aspect ratio in both (b) and (c) and draws the nominal
   +/-31.5 deg FOV and <2.18 m near-field blind arc in (b).
3. Eliminates overlapping badges/labels and adds explicit key parameters
   (YOLO11n-seg cls=0, N=2/EMA alpha=0.6, raytrace_min=2.18 m, 0.25 s watchdog).
"""

import argparse
from collections import defaultdict
import math
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Patch, Polygon, Wedge
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage
from ultralytics import YOLO

from bag_sqlite import bag_messages, bag_metadata
from render_archive_figures import rgb_image
from replay_scan_filter import MODEL_DIR, ReplayScan


FIGURE = Path(__file__).resolve().parents[1] / "manuscript/figures/pipeline.pdf"
TOPICS = {
    "/camera/image_raw": Image,
    "/scan": LaserScan,
    "/local_costmap/costmap": OccupancyGrid,
    "/local_plan": NavPath,
    "/odom": Odometry,
    "/tf": TFMessage,
}

INK, LINE = "#17283d", "#5b7185"
TEAL, BLUE, VIOLET, ORANGE = "#137680", "#175676", "#573885", "#f29e34"
RUST = "#a14c12"
PDF_METADATA = {"Title": "V-LiDAR system architecture (v2)", "CreationDate": None, "ModDate": None}


def header_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def read_snapshot(bag, seconds):
    info = bag_metadata(bag)
    start = info["starting_time"]["nanoseconds_since_epoch"]
    target = start + round(seconds * 1e9)
    messages = defaultdict(list)
    window = (target - 1_000_000_000, target + 1_000_000_000)
    for stamp, topic, payload in bag_messages(bag, info, TOPICS, window):
        messages[topic].append((stamp, deserialize_message(payload, TOPICS[topic])))
    if any(not messages[topic] for topic in TOPICS):
        missing = ", ".join(topic for topic in TOPICS if not messages[topic])
        raise ValueError(f"{bag.name}: missing {missing} near {seconds:g} s")

    scans = {header_ns(scan): scan for _, scan in messages["/scan"]}
    matches = [
        (time, image, scans[header_ns(image)])
        for time, image in messages["/camera/image_raw"]
        if target <= time and header_ns(image) in scans
    ]
    if not matches:
        raise ValueError(f"{bag.name}: no exact RGB/scan acquisition match after {seconds:g} s")
    time, image, scan = matches[0]
    nearby = []
    print(f"{bag.name}: RGB at {(time - start) / 1e9:.3f} s ({time} ns); matching acquisition {header_ns(image)} ns")
    for topic in ("/local_costmap/costmap", "/local_plan", "/odom"):
        stamp, msg = min(messages[topic], key=lambda row: abs(row[0] - time))
        if abs(stamp - time) > 300_000_000:
            raise ValueError(f"{bag.name}: {topic} is more than 300 ms from the RGB recorder time")
        print(f"{topic}: {(stamp - time) / 1e6:+.1f} ms from the RGB recorder time")
        nearby.append(msg)

    costmap, path, odom = nearby
    if path.header.frame_id != costmap.header.frame_id:
        raise ValueError("the local path and costmap must share a frame")
    transform = None
    if odom.header.frame_id != costmap.header.frame_id:
        transforms = [
            tf
            for _, entry in messages["/tf"]
            for tf in entry.transforms
            if tf.header.frame_id == costmap.header.frame_id and tf.child_frame_id == odom.header.frame_id
        ]
        if not transforms:
            raise ValueError("no recorded transform from the odometry frame to the costmap frame")
        transform = min(transforms, key=lambda tf: abs(header_ns(tf) - header_ns(odom))).transform
    return np.asarray(rgb_image(image)), scan, costmap, path, odom, transform


def rounded_box(figure, x, y, width, height, facecolor, label="", *, edgecolor="none",
                text_color=INK, size=8.6, weight="bold", lw=0.8, zorder=0):
    figure.add_artist(
        FancyBboxPatch(
            (x, y),
            width,
            height,
            transform=figure.transFigure,
            boxstyle="round,pad=0,rounding_size=0.012",
            facecolor=facecolor,
            edgecolor=edgecolor,
            linewidth=lw,
            zorder=zorder,
        )
    )
    if label:
        figure.text(
            x + width / 2,
            y + height / 2,
            label,
            ha="center",
            va="center",
            color=text_color,
            fontsize=size,
            fontweight=weight,
            zorder=zorder + 1,
        )


def connector(figure, points, *, dashed=False, lw=1.0):
    for start, stop in zip(points[:-2], points[1:-1]):
        figure.add_artist(
            Line2D(
                (start[0], stop[0]),
                (start[1], stop[1]),
                transform=figure.transFigure,
                color=LINE,
                lw=lw,
                linestyle="--" if dashed else "-",
            )
        )
    figure.add_artist(
        FancyArrowPatch(
            points[-2],
            points[-1],
            transform=figure.transFigure,
            arrowstyle="-|>",
            color=LINE,
            linewidth=lw,
            mutation_scale=8.5,
            linestyle="--" if dashed else "-",
            shrinkA=0,
            shrinkB=0,
        )
    )


def perception_panel(figure, rgb):
    replay = ReplayScan()
    model = YOLO(str(MODEL_DIR / "best_openvino_model"), task="segment")
    floor = replay.segment_floor(model, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    rows = np.where(floor[replay.y_min_scan :] == 0, replay.row_indices, -1).max(axis=0)
    if not floor.any() or not np.any(rows >= 0):
        raise ValueError("choose an RGB frame with a visible floor and boundary")

    height, width = rgb.shape[:2]
    axis = figure.add_axes((0.032, 0.452, 0.272, 0.375))
    axis.imshow(rgb)
    axis.set(xlim=(0, width - 1), ylim=(height - 1, 0.16 * height))
    axis.set_axis_off()

    tint = np.empty((height, width, 4), dtype=np.float32)
    tint[:, :, :3] = matplotlib.colors.to_rgb(TEAL)
    tint[:, :, 3] = 0.38 * cv2.resize(floor, (width, height), interpolation=cv2.INTER_NEAREST)
    axis.imshow(tint)

    # Mark scan-band boundary y_min_scan (row 120 of 256)
    scan_band_y = (replay.y_min_scan / floor.shape[0]) * height
    axis.axhline(scan_band_y, color="white", ls="--", lw=0.75, alpha=0.85)
    axis.text(width * 0.03, scan_band_y - 6, "Scan band (v=120)", color="white",
              fontsize=6.2, fontweight="bold", va="bottom",
              bbox={"facecolor": INK, "edgecolor": "none", "alpha": 0.55, "pad": 1.0})

    columns = np.flatnonzero(rows >= 0)[::3]
    axis.scatter(
        (columns + 0.5) * width / floor.shape[1] - 0.5,
        (rows[columns] + 0.5) * height / floor.shape[0] - 0.5,
        c=ORANGE,
        edgecolors=INK,
        linewidths=0.25,
        s=5.5,
        zorder=4,
    )
    axis.legend(
        handles=(
            Patch(facecolor=TEAL, edgecolor=INK, lw=0.4, label="Floor mask"),
            Line2D([], [], linestyle="none", marker="o", markersize=3.8,
                   markerfacecolor=ORANGE, markeredgecolor=INK, markeredgewidth=0.25, label="Boundary (u, v*)"),
        ),
        loc="upper center",
        bbox_to_anchor=(0.50, -0.032),
        ncol=2,
        frameon=False,
        fontsize=6.9,
        labelcolor=INK,
        handlelength=0.9,
        handletextpad=0.35,
        columnspacing=0.75,
        borderaxespad=0,
    )
    rounded_box(
        figure, 0.032, 0.282, 0.272, 0.082, "#d8ecea",
        "YOLO11n-seg (320×256, cls=0)\n7×7 closing ×2  |  OpenVINO FP16",
        edgecolor="#9ac8c3", size=6.8, weight="normal",
    )


def scan_panel(figure, scan):
    ranges = np.asarray(scan.ranges)
    angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    if not valid.any():
        raise ValueError("the selected LaserScan contains no finite returns")

    # ROS LaserScan: angles > 0 is LEFT (+y_robot), x_robot is forward.
    # Plot y_robot on x-axis and invert x-axis so +y (Left) is on the LEFT side matching panel (a).
    x_fwd = ranges[valid] * np.cos(angles[valid])
    y_lat = ranges[valid] * np.sin(angles[valid])

    axis = figure.add_axes((0.398, 0.448, 0.236, 0.382))
    axis.set_facecolor("white")
    fov_rad = math.radians(31.5)
    near_m, max_fwd = 2.18, max(7.5, math.ceil(x_fwd.max() + 0.3))
    ang = np.linspace(-fov_rad, fov_rad, 50)
    poly_pts = np.vstack(
        [
            np.column_stack([near_m * np.sin(ang), near_m * np.cos(ang)]),
            np.column_stack([max_fwd * np.tan(ang[::-1]), max_fwd * np.ones_like(ang[::-1])]),
        ]
    )
    axis.add_patch(Polygon(poly_pts, closed=True, facecolor="#edf5fb", edgecolor="#adc8df", lw=0.7, zorder=1))
    axis.add_patch(
        Wedge((0.0, 0.0), near_m, 90.0 - 31.5, 90.0 + 31.5, facecolor="#f7e8da",
              edgecolor=RUST, hatch="///", lw=0.55, alpha=0.8, zorder=1)
    )
    axis.text(0.0, 1.10, "<2.18 m\nblind", ha="center", va="center", fontsize=6.0,
              color=RUST, fontweight="bold",
              bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.85, "pad": 0.7}, zorder=4)

    axis.scatter(y_lat, x_fwd, s=9.0, color=BLUE, edgecolors="white", linewidths=0.22, zorder=3)
    axis.scatter([0], [0], marker="^", color=INK, s=22, zorder=5)

    axis.set(xlim=(3.0, -3.0), ylim=(-0.3, max_fwd), aspect="equal",
             xticks=(2.0, 0.0, -2.0), yticks=(0.0, 2.5, 5.0, 7.5))
    axis.set_xticklabels(["+2 (L)", "0", "-2 (R)"])
    axis.grid(color="#e1ebf2", linewidth=0.5, zorder=0)
    axis.set_xlabel("Lateral y (m)", color=INK, fontsize=7.3, labelpad=1.5)
    axis.set_ylabel("Forward x (m)", color=INK, fontsize=7.3, labelpad=1.5)
    axis.tick_params(labelsize=6.5, colors=INK, pad=1.3, length=2)
    axis.spines[["top", "right"]].set_visible(False)
    axis.spines[["bottom", "left"]].set_color(LINE)

    rounded_box(
        figure, 0.365, 0.282, 0.272, 0.082, "#dbe9f6",
        "2D LUT θ(u), r(u,v)  |  0.5° bins\nPersistence (N=2) + EMA (α=0.6) → /scan",
        edgecolor="#a6c4e0", size=6.8, weight="normal",
    )


def yaw(orientation):
    q = orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def navigation_panel(figure, costmap, path, odom, transform):
    if not path.poses:
        raise ValueError("the selected local path is empty")
    pose = odom.pose.pose
    robot_x, robot_y = pose.position.x, pose.position.y
    pose_yaw = yaw(pose.orientation)
    if transform is not None:
        turn = yaw(transform.rotation)
        a, b = math.cos(turn), math.sin(turn)
        robot_x, robot_y = (
            a * robot_x - b * robot_y + transform.translation.x,
            b * robot_x + a * robot_y + transform.translation.y,
        )
        pose_yaw += turn
    c, s = math.cos(pose_yaw), math.sin(pose_yaw)

    # Robot-local coordinates: fwd (+x_robot), lat (+y_robot = Left)
    def to_robot(world_x, world_y):
        dx, dy = world_x - robot_x, world_y - robot_y
        return c * dx + s * dy, -s * dx + c * dy

    axis = figure.add_axes((0.728, 0.448, 0.236, 0.382))
    info = costmap.info
    origin = info.origin
    gx, gy = np.meshgrid(
        np.arange(info.width + 1) * info.resolution,
        np.arange(info.height + 1) * info.resolution,
    )
    a, b = math.cos(yaw(origin.orientation)), math.sin(yaw(origin.orientation))
    fwd, lat = to_robot(origin.position.x + a * gx - b * gy, origin.position.y + b * gx + a * gy)
    cost = np.asarray(costmap.data).reshape(info.height, info.width)
    color_map = LinearSegmentedColormap.from_list("cost", ("#fbfbfc", "#e8e0f2", "#c1a6df", "#654397", "#35204f"))
    axis.pcolormesh(lat, fwd, np.ma.masked_less(cost, 0), cmap=color_map, vmin=0, vmax=100,
                    shading="flat", rasterized=True)

    pfwd, plat = to_robot(
        np.array([p.pose.position.x for p in path.poses]),
        np.array([p.pose.position.y for p in path.poses]),
    )
    axis.plot(plat, pfwd, color=TEAL, lw=1.6)
    axis.scatter([0], [0], marker="^", color=INK, edgecolor="white", linewidths=0.45, s=26, zorder=5)
    # Invert x-axis so +y (Left) sits on the LEFT side matching (a) and (b)
    axis.set(xlim=(1.65, -1.65), ylim=(-0.35, 3.25), aspect="equal",
             xticks=(1.0, 0.0, -1.0), yticks=(0.0, 1.5, 3.0))
    axis.set_xticklabels(["+1 (L)", "0", "-1 (R)"])
    axis.set_facecolor("#f3f1f7")
    axis.tick_params(labelsize=6.5, colors=INK, pad=1.3, length=2)
    axis.set_xlabel("Lateral y (m)", color=INK, fontsize=7.3, labelpad=1.5)
    axis.set_ylabel("Forward x (m)", color=INK, fontsize=7.3, labelpad=1.5)
    axis.spines[["top", "right"]].set_visible(False)
    axis.spines[["bottom", "left"]].set_color("#9f8bb3")

    axis.legend(
        handles=(
            Patch(facecolor="#9471ba", label="Cost"),
            Line2D([], [], color=TEAL, lw=1.4, label="Local path"),
            Line2D([], [], color=INK, marker="^", linestyle="none", markersize=4, label="Robot"),
        ),
        loc="upper center",
        bbox_to_anchor=(0.5, 1.125),
        ncol=3,
        frameon=False,
        fontsize=6.5,
        labelcolor=INK,
        handlelength=0.9,
        handletextpad=0.25,
        columnspacing=0.60,
        borderaxespad=0,
    )
    rounded_box(
        figure, 0.698, 0.282, 0.272, 0.082, "#e9dff2",
        "Obstacle + inflation (0.65 m)\nraytrace_min = 2.18 m  |  0.5 s expiry",
        edgecolor="#c0add4", size=6.8, weight="normal",
    )


def draw(rgb, scan, costmap, path, odom, transform, destination):
    figure = plt.figure(figsize=(6.4, 3.85))
    for x, background, header, label in (
        (0.016, "#edf6f5", "#cfe8e5", "(a) Camera perception"),
        (0.349, "#edf3f9", "#d4e5f4", "(b) Virtual scan (/scan)"),
        (0.682, "#f4eff7", "#e6d9ef", "(c) Nav2 local costmap"),
    ):
        rounded_box(figure, x, 0.268, 0.302, 0.700, background)
        rounded_box(figure, x, 0.900, 0.302, 0.068, header, label, size=8.6)

    perception_panel(figure, rgb)
    scan_panel(figure, scan)
    navigation_panel(figure, costmap, path, odom, transform)

    connector(figure, ((0.320, 0.610), (0.347, 0.610)))
    connector(figure, ((0.653, 0.610), (0.680, 0.610)))
    connector(figure, ((0.834, 0.268), (0.834, 0.236), (0.437, 0.236), (0.437, 0.208)))

    rounded_box(figure, 0.020, 0.120, 0.118, 0.086, "#dce7ee", "Goal\n(10 m, 0)", edgecolor="#aec3d1", size=8.2)
    rounded_box(
        figure, 0.212, 0.120, 0.446, 0.086, "#dce7ee",
        "Nav2 planner and DWB controller", edgecolor="#aec3d1", size=8.6,
    )
    rounded_box(figure, 0.800, 0.120, 0.180, 0.086, "#dce7ee", "OMO-R1 base", edgecolor="#aec3d1", size=8.4)

    connector(figure, ((0.140, 0.163), (0.210, 0.163)))
    connector(figure, ((0.660, 0.163), (0.798, 0.163)))
    figure.text(0.729, 0.184, "/cmd_vel", ha="center", fontsize=7.8, fontweight="bold", color=INK)
    connector(figure, ((0.905, 0.118), (0.905, 0.042), (0.335, 0.042), (0.335, 0.118)), dashed=True)
    figure.text(0.620, 0.066, "Wheel feedback  /odom + TF  (0.25 s timeout watchdog)",
                ha="center", fontsize=7.2, color=INK)

    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=330, metadata=PDF_METADATA)
    plt.close(figure)
    print(f"Saved {destination}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("seconds", type=float, help="recorder offset in seconds")
    parser.add_argument("--output", type=Path, default=FIGURE)
    args = parser.parse_args()
    draw(*read_snapshot(args.bag, args.seconds), args.output)


if __name__ == "__main__":
    main()
