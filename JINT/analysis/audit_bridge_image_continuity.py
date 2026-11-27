#!/usr/bin/env python3
"""Check for reused RGB payloads and repeated odometry twist in the reported bridge intervals."""

import argparse
from collections import Counter
import csv
import hashlib
from itertools import pairwise
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

from bag_sqlite import bag_messages, bag_metadata


INTERVALS = (("01", 1795753719989508496, 1795753729978593651),
             ("03", 1795754184506550770, 1795754189606593541),
             ("06", 1795754687302048242, 1795754689037472914),
             ("07", 1795755278821437106, 1795755279943435377))
TOPICS = ("/camera/image_raw", "/odom", "/cmd_vel")
OUTPUT = Path(__file__).resolve().parent / "evidence/bridge_session_2026-11-27_image_continuity.csv"


def measure(bag, start, end):
    info = bag_metadata(bag)
    origin = info["starting_time"]["nanoseconds_since_epoch"]
    stamps, rgb_hashes = [], set()
    odom = Counter()
    commands = set()
    command_samples = 0
    for _, topic, payload in bag_messages(bag, info, TOPICS, (start, end + 1)):
        if topic == "/camera/image_raw":
            image = deserialize_message(payload, Image)
            stamp = image.header.stamp
            stamps.append(stamp.sec * 1_000_000_000 + stamp.nanosec)
            rgb_hashes.add(hashlib.blake2s(image.data).digest())
        elif topic == "/odom":
            twist = deserialize_message(payload, Odometry).twist.twist
            odom[(twist.linear.x, twist.angular.z)] += 1
        else:
            command = deserialize_message(payload, Twist)
            commands.add((round(command.linear.x, 7), round(command.angular.z, 7)))
            command_samples += 1

    if len(stamps) < 2 or not odom or not commands:
        raise ValueError(f"{bag.name}: incomplete RGB, odometry or command data in the selected interval")

    ((vx, wz), repeated), = odom.most_common(1)
    gaps = [b - a for a, b in pairwise(stamps)]
    return {"bag_id": bag.name, "rgb_first_recorder_ns": start, "rgb_last_recorder_ns": end,
            "start_s": round((start - origin) / 1e9, 6), "stop_s": round((end - origin) / 1e9, 6),
            "rgb_messages": len(stamps), "distinct_acquisition_stamps": len(set(stamps)),
            "distinct_rgb_pixel_payloads": len(rgb_hashes), "max_rgb_acquisition_gap_s": round(max(gaps) / 1e9, 6),
            "odom_twist_messages": odom.total(), "exact_equal_twist_messages": repeated,
            "modal_odom_vx_mps": round(vx, 8), "modal_odom_wz_radps": round(wz, 8),
            "command_messages": command_samples, "distinct_rounded_command_components": len(commands)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", type=Path, default=Path.home() / "data/bags")
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    rows = [measure(args.bags / f"nav_c_bridge_{suffix}", start, end) for suffix, start, end in INTERVALS]
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
