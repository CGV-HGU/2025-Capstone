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
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle
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
INK, LINE, BORDER = "#14283a", "#506879", "#c9d7db"
TEAL, BLUE, AMBER, DARK_AMBER = "#0f766e", "#316381", "#ffc15a", "#8b450f"
FIGURE_SIZE = (5.13, 3.64)
SCENE = (.028, .526, .527, .387)
GEOMETRY = (.028, .261, .527, .205)
SCAN = (.632, .637, .340, .276)
COSTMAP = (.632, .267, .340, .238)


def header_ns(message):
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def read_snapshot(bag, seconds, timing=None):
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
    elapsed = (time - start) / 1e9
    print(f"{bag.name}: RGB at {elapsed:.3f} s ({time} ns); matching acquisition {header_ns(image)} ns")
    if timing is not None:
        timing.update(rgb_elapsed_s=round(elapsed, 6), acquisition_ns=header_ns(image))
    for topic in ("/local_costmap/costmap", "/local_plan", "/odom"):
        stamp, msg = min(messages[topic], key=lambda row: abs(row[0] - time))
        if abs(stamp - time) > 300_000_000:
            raise ValueError(f"{bag.name}: {topic} is more than 300 ms from the RGB recorder time")
        offset_ms = (stamp - time) / 1e6
        if timing is not None:
            timing[topic + "_minus_rgb_ms"] = round(offset_ms, 3)
        print(f"{topic}: {offset_ms:+.1f} ms from the RGB recorder time")
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


def box(figure, x, y, width, height, color, label=""):
    figure.add_artist(FancyBboxPatch((x, y), width, height, transform=figure.transFigure,
                                     boxstyle="round,pad=0,rounding_size=0.012",
                                     facecolor=color, edgecolor="none", zorder=0))
    if label:
        figure.text(x + width / 2, y + height / 2, label, ha="center", va="center",
                    color=INK, fontsize=8.3, fontweight="semibold")


def connector(figure, points, *, dashed=False, color=LINE):
    for start, stop in zip(points[:-2], points[1:-1]):
        figure.add_artist(Line2D((start[0], stop[0]), (start[1], stop[1]), transform=figure.transFigure,
                                 color=color, lw=1.05, linestyle="--" if dashed else "-"))
    figure.add_artist(FancyArrowPatch(points[-2], points[-1], transform=figure.transFigure,
                                     arrowstyle="-|>", color=color, linewidth=1.05, mutation_scale=8.5,
                                     linestyle="--" if dashed else "-", shrinkA=0, shrinkB=0))


def boundary_sample(rgb, scan):
    replay = ReplayScan()
    model = YOLO(str(MODEL_DIR / "best_openvino_model"), task="segment")
    floor = replay.segment_floor(model, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    rows = np.where(floor[replay.y_min_scan:] == 0, replay.row_indices, -1).max(axis=0)
    columns = np.flatnonzero(rows >= 0)
    ranges = np.asarray(scan.ranges)
    if ranges.size != replay.num_channels:
        raise ValueError("the recorded scan and deployed lookup table have different channel counts")
    # The ROS LaserScan reverses the LUT channels so that positive bearing is left.
    bins = ranges.size - 1 - replay.col_to_ch_lut[columns]
    distances = replay.distance_lut[rows[columns], columns]
    minimum = np.full(ranges.size, np.inf)
    np.minimum.at(minimum, bins, distances)
    eligible = (np.isfinite(distances) & (distances == minimum[bins]) &
                (ranges[bins] >= scan.range_min) & (ranges[bins] <= scan.range_max))
    visible = np.flatnonzero(eligible)
    if not floor.any() or not visible.size:
        raise ValueError("choose an image with a visible floor and a boundary contributing to the recorded scan")
    central = visible[np.abs(columns[visible] - floor.shape[1] / 2) <= .17 * floor.shape[1]]
    region = central if central.size else visible
    selected = region[np.argmin(distances[region])]
    u, v, channel = int(columns[selected]), int(rows[columns[selected]]), int(bins[selected])
    raw = float(distances[selected])
    print(f"LUT pixel ({u}, {v}) -> scan bin {channel}: {raw:.3f} m before filtering; {ranges[channel]:.3f} m recorded")
    return floor, rows, replay.y_min_scan, u, v, channel, raw


def tile(figure, bounds, facecolor="white"):
    axis = figure.add_axes(bounds, facecolor=facecolor)
    axis.set(xticks=[], yticks=[])
    for spine in axis.spines.values():
        spine.set(color=BORDER, linewidth=.65)
    return axis


def scene(figure, rgb, floor, rows, band, u, v):
    axis = tile(figure, SCENE)

    height, width = rgb.shape[:2]
    axis.imshow(rgb)

    tint = np.empty((height, width, 4), dtype=np.float32)
    tint[:, :, :3] = matplotlib.colors.to_rgb(TEAL)
    tint[:, :, 3] = .49 * cv2.resize(floor, (width, height), interpolation=cv2.INTER_NEAREST)
    axis.imshow(tint)
    scale_x, scale_y = width / floor.shape[1], height / floor.shape[0]
    columns = np.flatnonzero(rows >= 0)[::3]
    axis.scatter((columns + .5) * scale_x - .5, (rows[columns] + .5) * scale_y - .5,
                 c=AMBER, edgecolors=INK, linewidths=.24, s=4.2, zorder=3)
    px, py = (u + .5) * scale_x - .5, (v + .5) * scale_y - .5
    axis.axhline(band * scale_y, color="white", alpha=.63, lw=.8, ls=(0, (3, 3)))
    axis.plot((px, px), (band * scale_y, height - 1), color="white", lw=.95, ls="--",
              path_effects=[patheffects.withStroke(linewidth=1.8, foreground=INK)])
    axis.scatter([px], [py], color=AMBER, edgecolors=INK, linewidths=1.0, s=47, zorder=5)
    axis.annotate(r"$(u,v)$", (px, py), xytext=(16, 13), textcoords="offset points",
                  color=INK, fontsize=9, ha="left", va="center", zorder=6,
                  bbox={"facecolor": "#fff4dc", "edgecolor": "none", "boxstyle": "round,pad=.24"},
                  arrowprops={"arrowstyle": "-", "color": AMBER, "lw": 1.4, "shrinkB": 5})
    axis.text(.033, .072, "Floor mask", transform=axis.transAxes, va="bottom", fontsize=7.7,
              color="white", bbox={"facecolor": "#104640", "edgecolor": "none", "boxstyle": "round,pad=.26"})
    axis.text(.968, .918, "YOLO11-seg", transform=axis.transAxes, ha="right", va="top", fontsize=7.7,
              color="white", bbox={"facecolor": INK, "edgecolor": "none", "boxstyle": "round,pad=.26"})
    crop = width * SCENE[3] * FIGURE_SIZE[1] / (SCENE[2] * FIGURE_SIZE[0])
    axis.set(xlim=(-.5, width - .5), ylim=(height - .5, height - .5 - crop))

    inset = tile(figure, (.044, .753, .145, .153))
    inset.imshow(rgb)
    for spine in inset.spines.values():
        spine.set(color="white", linewidth=1.0)
    inset.text(.07, .08, "RGB input", transform=inset.transAxes, fontsize=7.0, color="white",
               bbox={"facecolor": INK, "edgecolor": "none", "boxstyle": "round,pad=.21"})


def projection(figure, raw, bearing):
    axis = tile(figure, GEOMETRY, "#eff7f5")
    axis.set(xlim=(0, 1), ylim=(0, 1))
    floor, lens_x, lens_y, contact_x = .28, .11, .73, .73
    axis.fill_between((.06, .94), .10, floor, color="#e0eee9")
    axis.plot((.06, .94), (floor, floor), color="#74978e", lw=.9)
    axis.add_patch(Rectangle((lens_x - .035, lens_y - .056), .04, .112, facecolor=INK, edgecolor="none"))
    axis.plot((lens_x, .49), (lens_y, lens_y), color="#829b9a", lw=.9, ls=(0, (3, 2)))
    axis.plot((lens_x, contact_x), (lens_y, floor), color=DARK_AMBER, lw=1.45)
    axis.scatter([contact_x], [floor], color=AMBER, edgecolor=DARK_AMBER, lw=.85, s=26, zorder=5)
    aspect = GEOMETRY[2] * FIGURE_SIZE[0] / (GEOMETRY[3] * FIGURE_SIZE[1])
    angle = math.atan2(floor - lens_y, (contact_x - lens_x) * aspect)
    arc = np.linspace(angle, 0, 20)
    axis.plot(lens_x + .17 * np.cos(arc), lens_y + .17 * aspect * np.sin(arc),
              lw=.8, color=DARK_AMBER)
    axis.annotate("", (lens_x, floor), (lens_x, lens_y),
                  arrowprops={"arrowstyle": "<->", "color": TEAL, "lw": .85, "shrinkA": 0, "shrinkB": 0})
    axis.text(.055, .505, r"$h$", color=TEAL, fontsize=8.5, va="center", ha="center")
    axis.text(.30, .89, r"$\phi(v)+\gamma$", fontsize=8.1, color=DARK_AMBER,
              ha="center", va="center")
    axis.annotate("", (contact_x, .16), (lens_x, .16),
                  arrowprops={"arrowstyle": "<->", "color": TEAL, "lw": .8, "shrinkA": 0, "shrinkB": 0})
    axis.text(.42, .16, r"$x(v)$", color=TEAL, fontsize=8.2, ha="center", va="center",
              bbox={"facecolor": "#e0eee9", "edgecolor": "none", "pad": 2})
    bearing_label = f"{bearing:+.1f}°".replace("-", "−")
    axis.text(.965, .85, f"{raw:.2f} m  /  {bearing_label}", ha="right", va="center",
              fontsize=8.3, color=DARK_AMBER, fontweight="semibold")
    axis.text(.965, .65, "LUT range / bin bearing", ha="right", va="center", fontsize=7.4, color=INK)


def scan_panel(axis, scan, channel):
    ranges = np.asarray(scan.ranges)
    angles = scan.angle_min + np.arange(ranges.size) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    if not valid.any():
        raise ValueError("the selected LaserScan contains no finite returns")
    x, y = ranges[valid] * np.sin(angles[valid]), ranges[valid] * np.cos(angles[valid])
    ray = ranges[channel] * np.array((np.sin(angles[channel]), np.cos(angles[channel])))
    ymax = max(5, math.ceil(y.max() + .35))
    ymin = -.75
    aspect = SCAN[2] * FIGURE_SIZE[0] / (SCAN[3] * FIGURE_SIZE[1])
    xmax = .5 * (ymax - ymin) * aspect
    # In ROS, positive lateral coordinates are to the robot's left.
    axis.set(xlim=(xmax, -xmax), ylim=(ymin, ymax), aspect="equal", facecolor="#f4f8fb")
    left, right = ymax * np.tan((scan.angle_max, scan.angle_min))
    axis.fill((0, left, right), (0, ymax, ymax), color="#e5eff4", zorder=0)
    axis.axvline(0, color="#b9ccd6", linewidth=.72, ls=(0, (3, 3)))
    axis.scatter(x, y, s=8, color=BLUE, linewidths=0, zorder=3)
    axis.scatter([0], [0], marker="^", color=INK, s=20, zorder=4)
    axis.plot((0, ray[0]), (0, ray[1]), color=DARK_AMBER, linewidth=1.5, zorder=4)
    axis.scatter(*ray, s=39, color=AMBER, edgecolor=DARK_AMBER, linewidth=.85, zorder=5)
    axis.annotate(f"{ranges[channel]:.2f} m", ray, xytext=(-14, 9), textcoords="offset points",
                  ha="right", va="center", fontsize=8.2, color=DARK_AMBER,
                  bbox={"facecolor": "#fff4dc", "edgecolor": "none", "boxstyle": "round,pad=.23"},
                  arrowprops={"arrowstyle": "-", "color": DARK_AMBER, "lw": .75, "shrinkB": 4})
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
    forward = (-.38, 3.0)
    aspect = COSTMAP[2] * FIGURE_SIZE[0] / (COSTMAP[3] * FIGURE_SIZE[1])
    lateral = .5 * (forward[1] - forward[0]) * aspect
    axis.set(xlim=(lateral, -lateral), ylim=forward, aspect="equal")
    axis.set_facecolor("#f3f1f7")
    axis.annotate("Local path", (px[len(px) // 2], py[len(py) // 2]), xytext=(14, -4),
                  textcoords="offset points", va="center", fontsize=7.2, color=TEAL,
                  bbox={"facecolor": "white", "edgecolor": "none", "boxstyle": "round,pad=.2"},
                  arrowprops={"arrowstyle": "-", "color": TEAL, "lw": .75})


def heading(figure, x, y, text):
    figure.text(x, y, text, va="center", color=INK, fontsize=10.5, fontweight="semibold")


def draw(rgb, scan, costmap, path, odom, transform, destination):
    figure = plt.figure(figsize=FIGURE_SIZE)
    floor, rows, band, u, v, channel, raw = boundary_sample(rgb, scan)
    bearing = math.degrees(scan.angle_min + channel * scan.angle_increment)

    heading(figure, SCENE[0], .952, "(a) From image to ground")
    heading(figure, SCAN[0], .952, "(b) Virtual scan")
    heading(figure, COSTMAP[0], .545, "(c) Nav2 costmap")
    scene(figure, rgb, floor, rows, band, u, v)
    figure.text(SCENE[0], .495, "Calibrated ground projection", va="center", fontsize=8.5,
                color=TEAL, fontweight="semibold")
    projection(figure, raw, bearing)
    scan_panel(tile(figure, SCAN), scan, channel)
    navigation(tile(figure, COSTMAP), costmap, path, odom, transform)
    connector(figure, ((.558, .363), (.589, .363), (.589, .755), (.628, .755)), color=DARK_AMBER)
    figure.text(.634, .602, "Binning + temporal filter", va="center", fontsize=7.9, color=BLUE)
    connector(figure, ((.949, .631), (.949, .510)))

    heading(figure, .028, .219, "(d) Drive and feedback")
    box(figure, .028, .022, .944, .158, "#f1f5f6")
    box(figure, .048, .109, .232, .051, "#dcece8", "OMO-R1 base")
    box(figure, .470, .109, .348, .051, "#e1eaf0", "Planner + controller")
    box(figure, .860, .109, .090, .051, "white", "Goal")
    connector(figure, ((.763, .262), (.763, .163)))
    connector(figure, ((.855, .134), (.823, .134)))
    connector(figure, ((.464, .134), (.286, .134)))
    figure.text(.373, .141, "/cmd_vel", ha="center", va="bottom", fontsize=7.7, color=INK)
    connector(figure, ((.164, .103), (.164, .051), (.644, .051), (.644, .105)), dashed=True)
    figure.text(.403, .062, "Wheel odometry + TF", ha="center", va="bottom", fontsize=7.9, color=INK)

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
