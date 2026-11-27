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
from matplotlib import patheffects
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
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
INK, LINE, BORDER = "#18283a", "#52677a", "#bdcad3"
TEAL, BLUE, ORANGE = "#116f73", "#23618d", "#f6a52c"
FIGURE_SIZE = (5.13, 2.55)
PANEL_X = (.021, .266, .511, .756)
PANEL_WIDTH = .222
PANEL_BOTTOM = .515
PANEL_HEIGHT = .75 * PANEL_WIDTH * FIGURE_SIZE[0] / FIGURE_SIZE[1]


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


def box(figure, x, y, width, height, color, label, *, text_color=INK):
    figure.add_artist(FancyBboxPatch((x, y), width, height, transform=figure.transFigure,
                                     boxstyle="round,pad=0,rounding_size=0.012",
                                     facecolor=color, edgecolor="none", zorder=0))
    figure.text(x + width / 2, y + height / 2, label, ha="center", va="center",
                color=text_color, fontsize=9.4, fontweight="semibold")


def connector(figure, points, *, dashed=False):
    for start, stop in zip(points[:-2], points[1:-1]):
        figure.add_artist(Line2D((start[0], stop[0]), (start[1], stop[1]), transform=figure.transFigure,
                                 color=LINE, lw=1.05, linestyle="--" if dashed else "-"))
    figure.add_artist(FancyArrowPatch(points[-2], points[-1], transform=figure.transFigure,
                                     arrowstyle="-|>", color=LINE, linewidth=1.05, mutation_scale=8.5,
                                     linestyle="--" if dashed else "-", shrinkA=0, shrinkB=0))


def perception(image, overlay, rgb):
    replay = ReplayScan()
    model = YOLO(str(MODEL_DIR / "best_openvino_model"), task="segment")
    floor = replay.segment_floor(model, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    rows = np.where(floor[replay.y_min_scan:] == 0, replay.row_indices, -1).max(axis=0)
    if not floor.any() or not np.any(rows >= 0):
        raise ValueError("choose an RGB frame with a visible floor and boundary")

    height, width = rgb.shape[:2]
    for axis in (image, overlay):
        axis.imshow(rgb)

    tint = np.empty((height, width, 4), dtype=np.float32)
    tint[:, :, :3] = matplotlib.colors.to_rgb(TEAL)
    tint[:, :, 3] = .43 * cv2.resize(floor, (width, height), interpolation=cv2.INTER_NEAREST)
    overlay.imshow(tint)
    columns = np.flatnonzero(rows >= 0)[::2]
    overlay.scatter((columns + .5) * width / floor.shape[1] - .5,
                    (rows[columns] + .5) * height / floor.shape[0] - .5,
                    c=ORANGE, edgecolors=INK, linewidths=.22, s=3.3)


def scan_panel(axis, scan):
    ranges = np.asarray(scan.ranges)
    angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    if not valid.any():
        raise ValueError("the selected LaserScan contains no finite returns")
    x, y = ranges[valid] * np.sin(angles[valid]), ranges[valid] * np.cos(angles[valid])
    ymax = max(5, math.ceil(y.max() + .2))
    ymin = -.75
    xmax = max(np.abs(x).max() + .25, 2 * (ymax - ymin) / 3)
    # In ROS, positive lateral coordinates are to the robot's left.
    axis.set(xlim=(xmax, -xmax), ylim=(ymin, ymax), aspect="equal", facecolor="#f4f8fb")
    axis.axvline(0, color="#d4e0ea", linewidth=.65)
    axis.scatter(x, y, s=7.5, color=BLUE, linewidths=0, zorder=3)
    axis.scatter([0], [0], marker="^", color=INK, s=20, zorder=4)
    scale_left, scale_y = xmax - .65, ymax - .65
    axis.plot((scale_left, scale_left - 2), (scale_y, scale_y), color=LINE, linewidth=1)
    axis.annotate("2 m", (scale_left, scale_y), xytext=(0, -3), textcoords="offset points",
                  ha="left", va="top", fontsize=7.7, color=INK)


def yaw(orientation):
    q = orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def costmap_in_robot_frame(costmap, path, odom, transform):
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

    info = costmap.info
    origin = info.origin
    gx, gy = np.meshgrid(np.arange(info.width + 1) * info.resolution,
                         np.arange(info.height + 1) * info.resolution)
    a, b = math.cos(yaw(origin.orientation)), math.sin(yaw(origin.orientation))
    x, y = relative(origin.position.x + a * gx - b * gy, origin.position.y + b * gx + a * gy)
    cost = np.asarray(costmap.data).reshape(info.height, info.width)
    px, py = relative(np.array([p.pose.position.x for p in path.poses]),
                      np.array([p.pose.position.y for p in path.poses]))
    return x, y, cost, px, py


def navigation(axis, costmap, path, odom, transform):
    x, y, cost, px, py = costmap_in_robot_frame(costmap, path, odom, transform)
    color_map = LinearSegmentedColormap.from_list(
        "cost", ("#fbfafa", "#e6deed", "#af98c7", "#624780", "#3b294f"))
    axis.pcolormesh(x, y, np.ma.masked_less(cost, 0), cmap=color_map,
                    vmin=0, vmax=100, shading="flat", rasterized=True)
    axis.plot(px, py, color=TEAL, lw=1.65,
              path_effects=[patheffects.withStroke(linewidth=3.05, foreground="white")])
    axis.scatter([0], [0], marker="^", color=INK, edgecolor="white", linewidths=.45, s=27, zorder=5)
    axis.set(xlim=(2.2, -2.2), ylim=(-.35, 2.95), aspect="equal")
    axis.set_facecolor("#f3f1f7")


def draw(rgb, scan, costmap, path, odom, transform, destination):
    figure = plt.figure(figsize=FIGURE_SIZE)
    axes = []
    for x, title, detail in zip(PANEL_X,
                                ("(a) RGB input", "(b) Floor mask", "(c) Virtual scan", "(d) Costmap"),
                                ("", "YOLO11-seg", "LUT + temporal filter", "Obstacle + inflation")):
        center = x + PANEL_WIDTH / 2
        figure.text(center, .925, title, ha="center", va="center", color=INK,
                    fontsize=9.5, fontweight="semibold")
        if detail:
            figure.text(center, .458, detail, ha="center", va="center", fontsize=8.2, color=INK)
        axis = figure.add_axes((x, PANEL_BOTTOM, PANEL_WIDTH, PANEL_HEIGHT))
        axis.set(xticks=[], yticks=[])
        for spine in axis.spines.values():
            spine.set(color=BORDER, linewidth=.55)
        axes.append(axis)
    perception(axes[0], axes[1], rgb)
    scan_panel(axes[2], scan)
    navigation(axes[3], costmap, path, odom, transform)
    middle = PANEL_BOTTOM + PANEL_HEIGHT / 2
    for left, right in zip(PANEL_X, PANEL_X[1:]):
        connector(figure, ((left + PANEL_WIDTH + .003, middle), (right - .003, middle)))

    box(figure, .021, .242, .245, .107, "#e6f1ef", "OMO-R1 base")
    box(figure, .562, .242, .416, .107, "#e9eef5", "Nav2 planner + controller")
    box(figure, .846, .046, .132, .096, "#f0f3f6", "Goal")
    connector(figure, ((.867, .417), (.867, .351)))
    connector(figure, ((.558, .295), (.270, .295)))
    figure.text(.414, .317, "/cmd_vel", ha="center", va="bottom", fontsize=8.5, color=INK)
    connector(figure, ((.912, .146), (.912, .240)))
    connector(figure, ((.143, .239), (.143, .096), (.678, .096), (.678, .240)), dashed=True)
    figure.text(.412, .116, "Wheel odometry + TF", ha="center", va="bottom", fontsize=8.5, color=INK)

    destination.parent.mkdir(parents=True, exist_ok=True)
    with plt.rc_context({"pdf.fonttype": 42}):
        figure.savefig(destination, dpi=330,
                       metadata={"Title": "V-LiDAR system architecture", "CreationDate": None})
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
