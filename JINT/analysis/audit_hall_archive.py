#!/usr/bin/env python3
"""Audit endpoints in the September single-hall trials from recorded wheel odometry."""

import argparse
from contextlib import closing
import csv
from datetime import datetime
import math
from pathlib import Path
from zoneinfo import ZoneInfo

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

from bag_sqlite import bag_messages, bag_metadata


KST = ZoneInfo("Asia/Seoul")
GOAL_DISTANCE_M = 10.0
GOAL_RADIUS_M = 0.50


def pose(payload):
    odom = deserialize_message(payload, Odometry)
    p, q = odom.pose.pose.position, odom.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return p.x, p.y, yaw


def audit(bag):
    info = bag_metadata(bag)
    bag_start = info["starting_time"]["nanoseconds_since_epoch"]
    topics = {entry["topic_metadata"]["name"]: entry["message_count"]
              for entry in info["topics_with_message_count"]}
    with closing(bag_messages(bag, info, ("/odom",))) as messages:
        first = next(messages, None)
        if first is None:
            raise ValueError(f"{bag}: no recorded odometry")
        start_x, start_y, yaw = pose(first[2])
        goal_x = start_x + GOAL_DISTANCE_M * math.cos(yaw)
        goal_y = start_y + GOAL_DISTANCE_M * math.sin(yaw)
        first_entry_s = None
        final_x, final_y = start_x, start_y
        samples = 1
        for stamp, _, payload in messages:
            final_x, final_y, _ = pose(payload)
            samples += 1
            if first_entry_s is None and math.hypot(goal_x - final_x, goal_y - final_y) <= GOAL_RADIUS_M:
                first_entry_s = (stamp - bag_start) / 1e9
    if samples < 2 or samples != topics["/odom"]:
        raise ValueError(f"{bag}: incomplete odometry stream")
    final_distance = math.hypot(goal_x - final_x, goal_y - final_y)
    return {
        "bag_id": bag.name,
        "start_kst": datetime.fromtimestamp(bag_start // 10**9, KST).isoformat(),
        "odom_samples": samples,
        "scan_samples": topics.get("/scan", 0),
        "rgb_samples": topics.get("/camera/image_raw", 0),
        "nominal_goal_x_m": round(goal_x, 6),
        "nominal_goal_y_m": round(goal_y, 6),
        "final_odom_x_m": round(final_x, 6),
        "final_odom_y_m": round(final_y, 6),
        "final_odom_distance_to_nominal_goal_m": round(final_distance, 6),
        "first_odom_entry_50cm_from_bag_start_s": round(first_entry_s, 6) if first_entry_s is not None else "",
        "terminal_odom_within_50cm": int(final_distance <= GOAL_RADIUS_M),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", type=Path, default=Path.home() / "data/bags")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).with_name("evidence") / "hall_session_2026-09-21.csv")
    args = parser.parse_args()
    rows = [audit(args.bags / f"minimal_A_scen1_run{run:02d}") for run in range(1, 21)]
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    within = sum(row["terminal_odom_within_50cm"] for row in rows)
    print(f"{within}/{len(rows)} endpoints within the nominal odometry radius: {args.output}")


if __name__ == "__main__":
    main()
