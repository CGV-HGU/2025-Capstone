#!/usr/bin/env python3
"""Summarize odometry, commands, wheel motion, and goal status from a reset diagnostic bag."""

import argparse
from collections import defaultdict
from itertools import pairwise
import json
import math
from pathlib import Path

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState

from bag_sqlite import bag_messages, bag_metadata


STATUS_TOPIC = "/navigate_to_pose/_action/status"
MESSAGE_TYPES = {"/odom": Odometry, "/cmd_vel": Twist, "/joint_states": JointState,
                 STATUS_TOPIC: GoalStatusArray}
STATUS_NAMES = dict(enumerate(("UNKNOWN", "ACCEPTED", "EXECUTING", "CANCELING",
                               "SUCCEEDED", "CANCELED", "ABORTED")))


def yaw(orientation):
    q = orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def motion(samples):
    if not samples:
        return None
    twists = [twist for _, twist in samples]
    return {"last_sample_s": samples[-1][0],
            "max_abs_vx_mps": max(abs(twist.linear.x) for twist in twists),
            "max_abs_wz_radps": max(abs(twist.angular.z) for twist in twists)}


def odometry(samples):
    result = motion([(seconds, msg.twist.twist) for seconds, msg in samples])
    if result is None:
        return None
    poses = [msg.pose.pose for _, msg in samples]
    stamps = [msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec for _, msg in samples]
    headings = [yaw(pose.orientation) for pose in poses]
    change = 0.0
    max_change = 0.0
    for before, after in pairwise(headings):
        change += math.remainder(after - before, math.tau)
        max_change = max(max_change, abs(change))
    start = poses[0].position
    result.update({
        "max_recorder_gap_s": max((after[0] - before[0] for before, after in pairwise(samples)), default=None),
        "nonincreasing_header_stamps": sum(after <= before for before, after in pairwise(stamps)),
        "max_xy_change_from_first_m": max(math.hypot(pose.position.x - start.x, pose.position.y - start.y)
                                         for pose in poses),
        "max_yaw_change_from_first_deg": math.degrees(max_change),
    })
    return result


def goal_transitions(samples):
    latest = {}
    changes = []
    for seconds, message in samples:
        for item in message.status_list:
            goal_id = bytes(item.goal_info.goal_id.uuid).hex()
            status = item.status
            if latest.get(goal_id) != status:
                changes.append({"seconds": seconds, "goal_id": goal_id,
                                "status": STATUS_NAMES.get(status, str(status))})
                latest[goal_id] = status
    return changes


def summarize(bag, window):
    info = bag_metadata(bag)
    start = info["starting_time"]["nanoseconds_since_epoch"]
    selection = tuple(start + round(seconds * 1e9) for seconds in window) if window else None
    samples = defaultdict(list)
    for stamp, topic, payload in bag_messages(bag, info, MESSAGE_TYPES, selection):
        samples[topic].append(((stamp - start) / 1e9, deserialize_message(payload, MESSAGE_TYPES[topic])))
    wheels = defaultdict(list)
    for _, msg in samples["/joint_states"]:
        for name, position in zip(msg.name, msg.position):
            wheels[name].append(position)
    return {
        "bag": bag.name,
        "window_s": window,
        "recorded_samples_in_window": {topic: len(samples[topic]) for topic in MESSAGE_TYPES},
        "odom": odometry(samples["/odom"]),
        "cmd_vel": motion(samples["/cmd_vel"]),
        "joint_position_span_rad": {name: max(values) - min(values) if len(values) > 1 else None
                                    for name, values in sorted(wheels.items())},
        "goal_status_changes": goal_transitions(samples[STATUS_TOPIC]),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--window", nargs=2, type=float, metavar=("START", "STOP"),
                        help="inspect only this recorder-time interval in seconds from bag start")
    args = parser.parse_args()
    if args.window and (not all(map(math.isfinite, args.window)) or not 0 <= args.window[0] < args.window[1]):
        parser.error("use finite window endpoints with 0 <= START < STOP")
    try:
        print(json.dumps(summarize(args.bag, args.window), indent=2))
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
