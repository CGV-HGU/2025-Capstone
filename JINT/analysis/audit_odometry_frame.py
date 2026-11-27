#!/usr/bin/env python3
"""Measure the recorded map-to-odom transform throughout each pilot bag."""

import argparse
import csv
import math
from pathlib import Path

from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage

from bag_sqlite import bag_messages, bag_metadata


def measure(bag):
    info = bag_metadata(bag)
    transforms = []
    odom_frames = set()
    for _, topic, payload in bag_messages(bag, info, ("/tf", "/tf_static", "/odom")):
        if topic == "/odom":
            message = deserialize_message(payload, Odometry)
            odom_frames.add((message.header.frame_id, message.child_frame_id))
            continue
        for entry in deserialize_message(payload, TFMessage).transforms:
            if entry.header.frame_id == "map" and entry.child_frame_id == "odom":
                transforms.append(entry.transform)

    translations = [math.hypot(t.translation.x, t.translation.y, t.translation.z) for t in transforms]
    rotations = [math.degrees(2 * math.atan2(math.hypot(t.rotation.x, t.rotation.y, t.rotation.z),
                                              abs(t.rotation.w))) for t in transforms]
    identity = sum(position == angle == 0 and abs(transform.rotation.w) == 1
                   for position, angle, transform in zip(translations, rotations, transforms))
    return {"bag_id": bag.name, "map_to_odom_samples": len(transforms),
            "identity_samples": identity, "max_translation_m": max(translations, default=""),
            "max_rotation_deg": max(rotations, default=""),
            "odom_frames": "; ".join(f"{parent} -> {child}" for parent, child in sorted(odom_frames))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("bags", type=Path, nargs="+")
    args = parser.parse_args()
    rows = [measure(bag) for bag in args.bags]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
