"""Trace a fixed local-costmap patch around archived virtual-scan returns."""

import csv

import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from bag_sqlite import bag_messages, bag_metadata
from render_archive_figures import bag_frame
from render_pipeline_figure import header_ns, yaw


ANCHOR_STAMP = 1795676540140095896
ANCHOR_CHANNELS = slice(52, 57)
PATCH_RADIUS_M = .25
WINDOW = (19.0, 27.0)


def planar_offset(transforms, parent, child):
    variants = set()
    for entry in transforms:
        if (entry.header.frame_id, entry.child_frame_id) == (parent, child):
            t, q = entry.transform.translation, entry.transform.rotation
            if (q.x, q.y, q.z) != (0, 0, 0) or abs(q.w) != 1:
                raise ValueError(f"Expected an unrotated {parent} to {child} transform")
            variants.add((t.x, t.y))
    if len(variants) != 1:
        raise ValueError(f"Expected one recorded planar offset for {parent} to {child}")
    return np.asarray(variants.pop())


def trace(bag):
    info = bag_metadata(bag)
    start = info["starting_time"]["nanoseconds_since_epoch"]
    kinds = {"/scan": LaserScan, "/odom": Odometry, "/tf": TFMessage, "/tf_static": TFMessage,
             "/local_costmap/costmap": OccupancyGrid}
    series = {topic: [] for topic in kinds}
    for receipt, topic, payload in bag_messages(bag, info, kinds):
        elapsed = (receipt - start) / 1e9
        series[topic].append((elapsed, deserialize_message(payload, kinds[topic])))
    transforms = [entry for topic in ("/tf", "/tf_static") for _, message in series[topic]
                  for entry in message.transforms]
    if np.any(planar_offset(transforms, "map", "odom")):
        raise ValueError(f"{bag}: expected map and odom to coincide")
    offset = planar_offset(transforms, "base_footprint", "base_link") + planar_offset(
        transforms, "base_link", "lidar_link")
    poses = [(header_ns(odom), odom.pose.pose) for _, odom in series["/odom"]
             if (odom.header.frame_id, odom.child_frame_id) == ("odom", "base_footprint")]

    def project(scan):
        stamp = header_ns(scan)
        pose_time, pose = min(poses, key=lambda entry: abs(entry[0] - stamp))
        heading = yaw(pose.orientation)
        c, s = np.cos(heading), np.sin(heading)
        rotation = np.array(((c, -s), (s, c)))
        sensor = np.asarray((pose.position.x, pose.position.y)) + rotation @ offset
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min + scan.angle_increment * np.arange(ranges.size)
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        local = ranges[valid, None] * np.column_stack((np.cos(angles[valid]), np.sin(angles[valid])))
        points = np.full((len(ranges), 2), np.nan)
        points[valid] = local @ rotation.T + sensor
        return points, sensor, (pose_time - stamp) / 1e6

    anchor = next(scan for _, scan in series["/scan"] if header_ns(scan) == ANCHOR_STAMP)
    anchor_returns, _, _ = project(anchor)
    reference = np.median(anchor_returns[ANCHOR_CHANNELS], axis=0)
    if not np.all(np.isfinite(reference)):
        raise ValueError(f"{bag}: invalid reference returns")

    scan_rows = []
    grid_rows = []
    for elapsed, scan in series["/scan"]:
        if WINDOW[0] <= elapsed <= WINDOW[1]:
            points, sensor, skew = project(scan)
            distances = np.linalg.norm(points - reference, axis=1)
            scan_rows.append({"elapsed_s": elapsed, "stamp_ns": header_ns(scan), "type": "scan",
                              "scan_returns_in_patch": int((distances <= PATCH_RADIUS_M).sum()),
                              "odom_offset_ms": skew, "sensor_to_reference_m": np.linalg.norm(sensor - reference)})
    for elapsed, grid in series["/local_costmap/costmap"]:
        if WINDOW[0] <= elapsed <= WINDOW[1]:
            msg = grid.info
            if grid.header.frame_id != "map" or abs(msg.origin.orientation.w) != 1:
                raise ValueError(f"{bag}: expected local costmap in the axis-aligned map frame")
            dx = msg.origin.position.x + (np.arange(msg.width) + .5) * msg.resolution - reference[0]
            dy = msg.origin.position.y + (np.arange(msg.height) + .5) * msg.resolution - reference[1]
            mask = dy[:, None]**2 + dx[None, :]**2 <= PATCH_RADIUS_M**2
            cost = np.asarray(grid.data).reshape(msg.height, msg.width)[mask]
            grid_rows.append({"elapsed_s": elapsed, "stamp_ns": header_ns(grid), "type": "local_costmap",
                              "grid_cells_in_patch": cost.size, "cost_99_or_100": int((cost >= 99).sum()),
                              "cost_100": int((cost == 100).sum()), "max_cost": int(cost.max())})
    return reference, scan_rows, grid_rows


def near_field_page(bag, evidence):
    reference, scan, grids = trace(bag)
    rows = sorted(scan + grids, key=lambda row: row["elapsed_s"])
    columns = list(dict.fromkeys(key for row in rows for key in row))
    with (evidence / "pilot_near_field_trace.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

    last_scan = max(row["elapsed_s"] for row in scan if row["scan_returns_in_patch"] > 0)
    last_high = max(row["elapsed_s"] for row in grids if row["cost_99_or_100"] > 0)
    print(f"{bag.name}: patch center {reference}; last scan return {last_scan:.3f} s; last high cost {last_high:.3f} s")

    figure = plt.figure(figsize=(7.1, 5.8))
    grid = figure.add_gridspec(3, 2, height_ratios=(1.45, 1, 1), left=.11, right=.97,
                              top=.85, bottom=.15, hspace=.49, wspace=.13)
    figure.suptitle("S6. Local cost after nearby scan returns disappear", y=.97, fontsize=14)
    figure.text(.5, .908, "Second bridge pilot. Counts use a fixed 0.25 m radius around earlier projected returns.",
                ha="center", fontsize=9.5)
    for column, time in enumerate((19.05, 23.0)):
        image, _ = bag_frame(bag, time)
        axis = figure.add_subplot(grid[0, column])
        axis.imshow(image)
        axis.set_title("(a) Bin in view" if not column else "(b) Ground contact outside image", fontsize=9.5)
        axis.set_axis_off()
    scan_axis = figure.add_subplot(grid[1, :])
    map_axis = figure.add_subplot(grid[2, :], sharex=scan_axis)
    scan_axis.plot([row["elapsed_s"] for row in scan], [row["scan_returns_in_patch"] for row in scan],
                   ".-", ms=3, lw=.95, color="#175676")
    scan_axis.set_ylabel("Scan returns", fontsize=9.5)
    for key, color, label in (("cost_99_or_100", "#643989", "Costs 99 and 100"),
                              ("cost_100", "#aa5016", "Cost 100")):
        map_axis.step([row["elapsed_s"] for row in grids], [row[key] for row in grids],
                      where="post", lw=1.2, color=color, label=label)
    map_axis.set(xlabel="Time from recording start (s)", ylabel="Costmap cells")
    map_axis.legend(loc="upper right", ncol=2, fontsize=8.5, frameon=False)
    for axis in (scan_axis, map_axis):
        axis.axvspan(last_scan, last_high, color="#eee5f2", zorder=0)
        axis.set(xlim=WINDOW, ylim=(0, None))
        axis.grid(axis="y", color=".91")
    scan_axis.tick_params(labelbottom=False)
    figure.text(.5, .047, f"The shaded span runs from the last scan return ({last_scan:.2f} s) "
                f"to the last recorded cost of 99 or 100 ({last_high:.2f} s).", ha="center", fontsize=9)
    return figure
