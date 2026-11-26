#!/usr/bin/env python3
"""Render the camera-to-drive architecture with time-aligned archive examples."""

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
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Patch
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
TOPICS = {"/camera/image_raw": Image, "/scan": LaserScan,
          "/local_costmap/costmap": OccupancyGrid, "/local_plan": NavPath,
          "/odom": Odometry, "/tf": TFMessage}
INK, LINE = "#17283d", "#5b7185"
TEAL, BLUE, VIOLET, ORANGE = "#137680", "#205b91", "#573885", "#f29e34"


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
    matches = [(time, image, scans[header_ns(image)])
               for time, image in messages["/camera/image_raw"]
               if target <= time and header_ns(image) in scans]
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
        transforms = [tf for _, entry in messages["/tf"] for tf in entry.transforms
                      if tf.header.frame_id == costmap.header.frame_id and tf.child_frame_id == odom.header.frame_id]
        if not transforms:
            raise ValueError("no recorded transform from the odometry frame to the costmap frame")
        transform = min(transforms, key=lambda tf: abs(header_ns(tf) - header_ns(odom))).transform
    return np.asarray(rgb_image(image)), scan, costmap, path, odom, transform


def box(figure, x, y, width, height, color, label="", *, text_color=INK, size=9):
    figure.add_artist(FancyBboxPatch((x, y), width, height, transform=figure.transFigure,
                                     boxstyle="round,pad=0,rounding_size=0.012",
                                     facecolor=color, edgecolor="none", zorder=0))
    if label:
        figure.text(x + width / 2, y + height / 2, label, ha="center", va="center",
                    color=text_color, fontsize=size, fontweight="semibold")


def connector(figure, points, *, dashed=False):
    for start, stop in zip(points[:-2], points[1:-1]):
        figure.add_artist(Line2D((start[0], stop[0]), (start[1], stop[1]), transform=figure.transFigure,
                                 color=LINE, lw=.9, linestyle="--" if dashed else "-"))
    figure.add_artist(FancyArrowPatch(points[-2], points[-1], transform=figure.transFigure,
                                     arrowstyle="-|>", color=LINE, linewidth=.9, mutation_scale=8.2,
                                     linestyle="--" if dashed else "-", shrinkA=0, shrinkB=0))


def perception(figure, rgb):
    replay = ReplayScan()
    model = YOLO(str(MODEL_DIR / "best_openvino_model"), task="segment")
    floor = replay.segment_floor(model, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    rows = np.where(floor[replay.y_min_scan:] == 0, replay.row_indices, -1).max(axis=0)
    if not floor.any() or not np.any(rows >= 0):
        raise ValueError("choose an RGB frame with a visible floor and boundary")

    image = figure.add_axes((.049, .646, .248, .226))
    overlay = figure.add_axes((.049, .348, .248, .226))
    height, width = rgb.shape[:2]
    for axis in (image, overlay):
        axis.imshow(rgb)
        axis.set(xlim=(0, width - 1), ylim=(height - 1, .22 * height))
        axis.set_axis_off()

    tint = np.empty((height, width, 4), dtype=np.float32)
    tint[:, :, :3] = matplotlib.colors.to_rgb(TEAL)
    tint[:, :, 3] = .37 * cv2.resize(floor, (width, height), interpolation=cv2.INTER_NEAREST)
    overlay.imshow(tint)
    columns = np.flatnonzero(rows >= 0)[::3]
    overlay.scatter((columns + .5) * width / floor.shape[1] - .5,
                    (rows[columns] + .5) * height / floor.shape[0] - .5,
                    c=ORANGE, edgecolors=INK, linewidths=.25, s=5)
    figure.text(.173, .622, "YOLO11-seg + mask closing", color=INK, fontsize=8.2, ha="center")
    connector(figure, ((.173, .608), (.173, .579)))
    overlay.legend(handles=(Patch(facecolor=TEAL, label="Floor mask"),
                            Line2D([], [], linestyle="none", marker="o", markersize=3.6,
                                   markerfacecolor=ORANGE, markeredgecolor=INK,
                                   markeredgewidth=.25, label="Boundary pixels")),
                   loc="upper center", bbox_to_anchor=(.50, -.061), ncol=2, frameon=False,
                   fontsize=8, labelcolor=INK, handlelength=.8, handletextpad=.32,
                   columnspacing=.78, borderaxespad=0)


def scan_panel(figure, scan):
    ranges = np.asarray(scan.ranges)
    angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    if not valid.any():
        raise ValueError("the selected LaserScan contains no finite returns")
    x, y = ranges[valid] * np.sin(angles[valid]), ranges[valid] * np.cos(angles[valid])
    axis = figure.add_axes((.413, .417, .211, .294))
    axis.set_facecolor("white")
    ymax = max(5, math.ceil(y.max() + .2))
    xmax = max(2, math.ceil(np.abs(x).max() + .12))
    axis.set(xlim=(-xmax, xmax), ylim=(-.2, ymax), xticks=(-xmax, 0, xmax),
             yticks=(0, ymax / 2, ymax))
    axis.grid(color="#d8e4ed", linewidth=.5)
    axis.scatter(x, y, s=9, color=BLUE, edgecolors="white", linewidths=.23, zorder=3)
    axis.scatter([0], [0], marker="^", color=INK, s=20, zorder=4)
    axis.set_xlabel("Lateral (m)", color=INK, fontsize=8, labelpad=2)
    axis.set_ylabel("Forward (m)", color=INK, fontsize=8, labelpad=1.6)
    axis.tick_params(labelsize=7.5, colors=INK, pad=1.6, length=2)
    axis.spines[["top", "right"]].set_visible(False)
    axis.spines[["bottom", "left"]].set_color(LINE)
    box(figure, .366, .798, .266, .057, "#dbe9f6", "Range and bearing LUT", size=8.7)
    box(figure, .366, .732, .266, .057, "#dbe9f6", "Persistence + EMA filter", size=8.7)
    figure.text(.515, .308, "ROS 2 LaserScan  /scan", ha="center", fontsize=9.3, fontweight="semibold", color=BLUE)


def yaw(orientation):
    q = orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def navigation(figure, costmap, path, odom, transform):
    if not path.poses:
        raise ValueError("the selected local path is empty")
    pose = odom.pose.pose
    robot_x, robot_y = pose.position.x, pose.position.y
    pose_yaw = yaw(pose.orientation)
    if transform is not None:
        turn = yaw(transform.rotation)
        a, b = math.cos(turn), math.sin(turn)
        robot_x, robot_y = (a * robot_x - b * robot_y + transform.translation.x,
                            b * robot_x + a * robot_y + transform.translation.y)
        pose_yaw += turn
    c, s = math.cos(pose_yaw), math.sin(pose_yaw)

    def relative(world_x, world_y):
        dx, dy = world_x - robot_x, world_y - robot_y
        return -s * dx + c * dy, c * dx + s * dy

    axis = figure.add_axes((.737, .373, .218, .448))
    info = costmap.info
    origin = info.origin
    gx, gy = np.meshgrid(np.arange(info.width + 1) * info.resolution,
                         np.arange(info.height + 1) * info.resolution)
    a, b = math.cos(yaw(origin.orientation)), math.sin(yaw(origin.orientation))
    x, y = relative(origin.position.x + a * gx - b * gy, origin.position.y + b * gx + a * gy)
    cost = np.asarray(costmap.data).reshape(info.height, info.width)
    color_map = LinearSegmentedColormap.from_list(
        "cost", ("#fbfbfc", "#e8e0f2", "#c1a6df", "#654397", "#35204f"))
    axis.pcolormesh(x, y, np.ma.masked_less(cost, 0), cmap=color_map,
                    vmin=0, vmax=100, shading="flat", rasterized=True)
    px, py = relative(np.array([p.pose.position.x for p in path.poses]),
                      np.array([p.pose.position.y for p in path.poses]))
    axis.plot(px, py, color=TEAL, lw=1.5)
    axis.scatter([0], [0], marker="^", color=INK, edgecolor="white", linewidths=.45, s=27, zorder=5)
    axis.set(xlim=(-1.48, 1.48), ylim=(-.42, 3.10), aspect="equal")
    axis.set_facecolor("#f3f1f7")
    axis.set_xticks([])
    axis.set_yticks([])
    axis.spines[:].set_color("#d7cbdf")
    figure.text(.834, .844, "Obstacle + inflation layers", color=VIOLET, fontsize=8.6, ha="center")
    axis.legend(handles=(Patch(facecolor="#9471ba", label="Cost"),
                         Line2D([], [], color=TEAL, lw=1.3, label="Local path"),
                         Line2D([], [], color=INK, marker="^", linestyle="none", markersize=4, label="Robot")),
                loc="upper center", bbox_to_anchor=(.5, -.057), ncol=3, frameon=False,
                fontsize=7.6, labelcolor=INK, handlelength=1, handletextpad=.25,
                columnspacing=.70, borderaxespad=0)


def draw(rgb, scan, costmap, path, odom, transform, destination):
    figure = plt.figure(figsize=(6.0, 3.48))
    for x, background, header, label in (
        (.016, "#edf6f5", "#cfe8e5", "(a) Camera perception"),
        (.349, "#edf3f9", "#d4e5f4", "(b) Virtual scan"),
        (.682, "#f4eff7", "#e6d9ef", "(c) Nav2 costmap"),
    ):
        box(figure, x, .278, .302, .689, background)
        box(figure, x, .895, .302, .072, header, label, size=9)
    perception(figure, rgb)
    scan_panel(figure, scan)
    navigation(figure, costmap, path, odom, transform)
    connector(figure, ((.320, .610), (.346, .610)))
    connector(figure, ((.653, .610), (.680, .610)))
    connector(figure, ((.834, .277), (.834, .248), (.437, .248), (.437, .216)))

    for x, w, label in ((.021, .119, "Goal"), (.218, .438, "Nav2 planner and controller"),
                        (.809, .175, "OMO-R1 base")):
        box(figure, x, .125, w, .089, "#dce7ee", label, size=9)
    connector(figure, ((.14, .169), (.217, .169)))
    connector(figure, ((.658, .161), (.807, .161)))
    figure.text(.739, .184, "/cmd_vel", ha="center", fontsize=8.4, color=INK)
    connector(figure, ((.900, .124), (.900, .046), (.437, .046), (.437, .124)), dashed=True)
    figure.text(.675, .068, "Wheel feedback  /odom + TF", ha="center", fontsize=8.0, color=INK)

    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=330, metadata={"Title": "V-LiDAR system architecture", "CreationDate": None})
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("seconds", type=float, help="recorder offset in seconds")
    parser.add_argument("--output", type=Path, default=FIGURE)
    args = parser.parse_args()
    draw(*read_snapshot(args.bag, args.seconds), args.output)


if __name__ == "__main__":
    main()
