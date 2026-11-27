#!/usr/bin/env python3
"""Check which saved model class reproduces the virtual scans in an archived bag."""

import argparse
import csv
from pathlib import Path

from cv_bridge import CvBridge
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, LaserScan
from ultralytics import YOLO

from bag_sqlite import bag_messages, bag_metadata
from replay_scan_filter import MODEL_DIR, ReplayScan, valid_ranges
from scan_recorder_timing import IMAGE, SCAN, header_timestamp


def compare_to_recording(bag, model):
    info = bag_metadata(bag)
    scans = {}
    for _, _, payload in bag_messages(bag, info, (SCAN,)):
        stamp = header_timestamp(payload[:12])
        if stamp > 0 and stamp not in scans:
            message = deserialize_message(payload, LaserScan)
            scans[stamp] = np.asarray(message.ranges, dtype=np.float32)

    bridge = CvBridge()
    replays = [ReplayScan(class_id) for class_id in (0, 1)]
    recorded, replayed = [], [[], []]
    first_stamp = last_stamp = None
    for _, _, payload in bag_messages(bag, info, (IMAGE,)):
        stamp = header_timestamp(payload[:12])
        expected = scans.pop(stamp, None)
        if expected is None:
            continue
        if last_stamp is not None and stamp <= last_stamp:
            raise ValueError(f"{bag}: matched RGB frames must be ordered by acquisition time")
        last_stamp = stamp
        if first_stamp is None:
            first_stamp = stamp
        warmed_up = stamp - first_stamp >= 1_000_000_000
        image = deserialize_message(payload, Image)
        result = replays[0].infer(model, bridge.imgmsg_to_cv2(image, "bgr8"))
        for session, ranges in zip(replays, replayed):
            _, filtered = session.project(result, image.header.stamp)
            if warmed_up:
                # The deployed publisher reverses internal LUT channel order.
                ranges.append(filtered[::-1])
        if warmed_up:
            recorded.append(expected)

    if not recorded:
        raise ValueError(f"{bag}: no matched scans after the one-second warmup")
    expected = np.stack(recorded)
    expected_valid = valid_ranges(expected)
    for session, ranges in zip(replays, replayed):
        candidate = np.stack(ranges)
        if candidate.shape != expected.shape:
            raise ValueError(f"{bag}: recorded and replayed scan lengths differ")
        exact = np.isclose(expected, candidate, rtol=0, atol=1e-4)
        candidate_valid = valid_ranges(candidate)
        common = expected_valid & candidate_valid
        error = np.abs(expected[common] - candidate[common])
        yield {
            "bag_id": bag.name,
            "floor_class_id": session.floor_class_id,
            "matched_frames": len(expected),
            "all_channels": expected.size,
            "matching_channels_at_0_1mm": int(exact.sum()),
            "fully_matching_frames_at_0_1mm": int(np.all(exact, axis=1).sum()),
            "recorded_valid_channels": int(expected_valid.sum()),
            "replay_valid_channels": int(candidate_valid.sum()),
            "common_valid_channels": int(common.sum()),
            "common_valid_error_median_m": float(np.median(error)) if error.size else "",
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("bags", type=Path, nargs="+")
    args = parser.parse_args()
    model = YOLO(str(MODEL_DIR / "best_openvino_model"), task="segment")
    rows = [row for bag in args.bags for row in compare_to_recording(bag, model)]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
