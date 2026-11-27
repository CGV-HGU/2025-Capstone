#!/usr/bin/env python3
"""Compare recorded commands, odometry, paths, and goal costs near bridge trial endings."""

import argparse
from collections import defaultdict
import csv
from itertools import pairwise
import json
import math
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

from bag_sqlite import bag_messages, bag_metadata
from summarize_reset_diagnostics import yaw


TOPICS = {"/cmd_vel": Twist, "/odom": Odometry, "/plan": NavPath, "/local_plan": NavPath,
          "/local_costmap/costmap": OccupancyGrid, "/global_costmap/costmap": OccupancyGrid,
          "/tf": TFMessage, "/tf_static": TFMessage}
GOAL_X, GOAL_Y, GOAL_RADIUS = 10.0, 0.0, .25
WINDOW_S = 5


def measure_costmap(series, end):
    if not series:
        return None, None, None
    time, grid = series[-1]
    if grid.header.frame_id != "map" or abs(grid.info.origin.orientation.w) != 1:
        raise ValueError("expected a map-frame costmap with an axis-aligned origin")
    origin = grid.info.origin.position
    resolution = grid.info.resolution
    cells = []
    for y in range(grid.info.height):
        dy = origin.y + (y + .5) * resolution - GOAL_Y
        if abs(dy) > GOAL_RADIUS:
            continue
        for x in range(grid.info.width):
            dx = origin.x + (x + .5) * resolution - GOAL_X
            if dx * dx + dy * dy <= GOAL_RADIUS**2:
                cells.append(grid.data[y * grid.info.width + x])
    return (end - time) / 1e9, len(cells), sum(value >= 99 for value in cells)


def path_length(path):
    poses = [entry.pose.position for entry in path.poses]
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in pairwise(poses))


def audit(bag, result):
    if (result["goal_rel_x_m"], result["goal_rel_y_m"]) != (GOAL_X, GOAL_Y):
        raise ValueError(f"{bag}: expected the bridge session's 10 m goal")
    info = bag_metadata(bag)
    origin = info["starting_time"]["nanoseconds_since_epoch"]
    start = result["goal_accepted_time_ns"]
    end = result["result_or_cancel_time_ns"]
    tail = end - WINDOW_S * 1_000_000_000
    samples = defaultdict(list)
    tf_total, tf_identity = 0, 0
    start_pose = None
    for time, topic, payload in bag_messages(bag, info, TOPICS, (origin, end + 1)):
        if topic in ("/tf", "/tf_static"):
            message = deserialize_message(payload, TFMessage)
            for entry in message.transforms:
                if (entry.header.frame_id, entry.child_frame_id) != ("map", "odom"):
                    continue
                v, q = entry.transform.translation, entry.transform.rotation
                tf_total += 1
                tf_identity += (v.x, v.y, v.z, q.x, q.y, q.z, abs(q.w)) == (0, 0, 0, 0, 0, 0, 1)
        elif topic == "/odom" and start_pose is None and time >= start:
            start_pose = deserialize_message(payload, Odometry).pose.pose
        if time >= tail and topic not in ("/tf", "/tf_static"):
            samples[topic].append((time, deserialize_message(payload, TOPICS[topic])))
    if not samples["/odom"] or start_pose is None:
        raise ValueError(f"{bag}: missing odometry near the goal interval")

    pose0 = samples["/odom"][0][1].pose.pose
    pose1 = samples["/odom"][-1][1].pose.pose
    end_span = max(math.hypot(msg.pose.pose.position.x - pose0.position.x,
                             msg.pose.pose.position.y - pose0.position.y) for _, msg in samples["/odom"])
    global_age, global_cells, global_high = measure_costmap(samples["/global_costmap/costmap"], end)
    local_age, local_cells, local_high = measure_costmap(samples["/local_costmap/costmap"], end)
    plans = samples["/plan"]
    return {
        "trial_id": result["trial_id"], "end_window_s": WINDOW_S,
        "first_odom_after_accept_x_m": start_pose.position.x,
        "first_odom_after_accept_y_m": start_pose.position.y,
        "first_odom_after_accept_yaw_deg": math.degrees(yaw(start_pose.orientation)),
        "map_to_odom_transforms": tf_total, "map_to_odom_identity_transforms": tf_identity,
        "goal_distance_at_window_start_m": math.hypot(pose0.position.x-GOAL_X, pose0.position.y-GOAL_Y),
        "goal_distance_at_last_sample_m": math.hypot(pose1.position.x-GOAL_X, pose1.position.y-GOAL_Y),
        "odom_xy_max_from_window_start_m": end_span,
        "cmd_samples": len(samples["/cmd_vel"]),
        "cmd_samples_abs_vx_over_0_02": sum(abs(m.linear.x) > .02 for _, m in samples["/cmd_vel"]),
        "cmd_samples_abs_wz_over_0_02": sum(abs(m.angular.z) > .02 for _, m in samples["/cmd_vel"]),
        "global_plan_samples": len(plans), "local_plan_samples": len(samples["/local_plan"]),
        "last_global_plan_length_m": path_length(plans[-1][1]) if plans else None,
        "last_global_costmap_age_s": global_age, "global_goal_disk_cells": global_cells,
        "global_goal_disk_inscribed_or_lethal_cells": global_high,
        "last_local_costmap_age_s": local_age, "local_goal_disk_cells": local_cells,
        "local_goal_disk_inscribed_or_lethal_cells": local_high,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", type=Path, default=Path.home() / "data/bags")
    parser.add_argument("--outcomes", type=Path, default=Path.home() / "data/trial_outcomes")
    parser.add_argument("--recovered-bags", type=Path, default=Path.home() / "data/jint_recovered")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = []
    for file in sorted(args.outcomes.glob("c_bridge_[0-9][0-9].json")):
        result = json.loads(file.read_text())
        name = f"nav_{result['trial_id']}"
        recovered = args.recovered_bags / name
        bag = recovered if (recovered / "metadata.yaml").is_file() else args.bags / name
        rows.append(audit(bag, result))
    if not rows:
        parser.error("no bridge outcomes found")
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0], lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: round(value, 6) if isinstance(value, float) else value
                             for key, value in row.items()})


if __name__ == "__main__":
    main()
