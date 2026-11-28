#!/usr/bin/env python3
"""Measure recorded local goal costs and motion around Fifth Reading Room run 20 recoveries."""

import argparse
import csv
import json
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message

from audit_class5 import GOAL, GOAL_RADIUS
from bag_sqlite import bag_messages, bag_metadata


GRID = "/local_costmap/costmap"
TYPES = {GRID: OccupancyGrid, "/cmd_vel": Twist, "/odom": Odometry, "/rosout": Log}
CLEAR_MESSAGE = "Received request to clear entirely the local_costmap"
MOTION_THRESHOLD_MPS = 0.02
WINDOW_NS = 2_000_000_000


def goal_disk(grid):
    origin = grid.info.origin
    q = origin.orientation
    if grid.header.frame_id not in ("map", "odom") or (q.x, q.y, q.z, abs(q.w)) != (0, 0, 0, 1):
        raise ValueError("expected an axis-aligned local grid in the recorded map/odom frame")
    x = origin.position.x + (np.arange(grid.info.width) + 0.5) * grid.info.resolution
    y = origin.position.y + (np.arange(grid.info.height) + 0.5) * grid.info.resolution
    inside = (x[None, :] - GOAL[0])**2 + (y[:, None] - GOAL[1])**2 <= GOAL_RADIUS**2
    return np.asarray(grid.data).reshape(grid.info.height, grid.info.width)[inside]


def audit(bag, result):
    # audit_class5 verifies the logged absolute goal and identity map-to-odom transform for this recording.
    start, end = result["goal_accepted_time_ns"], result["result_or_cancel_time_ns"]
    streams = {topic: [] for topic in TYPES}
    for t, topic, payload in bag_messages(bag, bag_metadata(bag), TYPES, (start, end + 1)):
        streams[topic].append((t, deserialize_message(payload, TYPES[topic])))
    for t, log in streams["/rosout"]:
        if log.name != "local_costmap.local_costmap" or log.msg != CLEAR_MESSAGE:
            continue
        row = {"trial_id": result["trial_id"], "clear_from_goal_acceptance_s": round((t - start) / 1e9, 6)}
        before = [sample for sample in streams[GRID] if t - WINDOW_NS <= sample[0] < t]
        after = [sample for sample in streams[GRID] if t <= sample[0] <= t + WINDOW_NS]
        if not before or not after:
            raise ValueError(f"{bag}: missing local grid within two seconds of the clear")
        for label, (stamp, grid) in (("before", before[-1]), ("after", after[0])):
            values = goal_disk(grid)
            row.update({f"{label}_grid_ms": round((stamp - t) / 1e6, 3),
                        f"{label}_goal_cells": values.size,
                        f"{label}_goal_cells_99_100": int(np.count_nonzero((values == 99) | (values == 100))),
                        f"{label}_goal_cells_unknown": int(np.count_nonzero(values < 0))})
        command = next(stamp for stamp, msg in streams["/cmd_vel"] if t <= stamp <= t + WINDOW_NS
                       and msg.linear.x > MOTION_THRESHOLD_MPS)
        odom = next(stamp for stamp, msg in streams["/odom"] if t <= stamp <= t + WINDOW_NS
                    and msg.twist.twist.linear.x > MOTION_THRESHOLD_MPS)
        row["next_forward_command_ms"] = round((command - t) / 1e6, 3)
        row["next_forward_odom_ms"] = round((odom - t) / 1e6, 3)
        yield row


def main():
    data = Path.home() / "data"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, default=data / "bags/nav_class5_20")
    parser.add_argument("--outcome", type=Path, default=data / "trial_outcomes/class5_20.json")
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("evidence") / "class5_20_recovery_costmap.csv")
    args = parser.parse_args()
    rows = list(audit(args.bag, json.loads(args.outcome.read_text(encoding="utf-8"))))
    if not rows:
        parser.error("no documented local-costmap clear found")
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
