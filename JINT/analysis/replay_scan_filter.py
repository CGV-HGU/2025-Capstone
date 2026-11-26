#!/usr/bin/env python3
"""Compare the existing temporal scan filter with a bypass on archived RGB."""

import argparse
import csv
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from ultralytics import YOLO

from bag_sqlite import bag_messages, bag_metadata


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/freespace_detection"))
from freespace_detection.floor_detector import RoiChecker


MODEL_DIR = ROOT / "src/freespace_detection/scripts"
FRAME_PERIOD_NS = 100_000_000
RANGE_MIN, RANGE_MAX = 0.1, 10.0


class ReplayScan:
    """Run the deployed mask and scan functions with publication disconnected."""

    def __init__(self):
        self.target_w, self.target_h = 320, 256
        self.floor_class_id, self.y_min_scan, self.min_floor_scan_pixels = 0, 120, 50
        self.row_indices = np.arange(self.y_min_scan, self.target_h)[:, None]
        self.num_channels = 141
        self.range_max = float("inf")
        self.col_to_ch_lut = np.load(MODEL_DIR / "col_to_ch_lut.npy")
        self.distance_lut = np.load(MODEL_DIR / "distance_lut_2d.npy")
        self.is_2d_lut = True
        self.scan_pub = None
        self.channel_pub = SimpleNamespace(publish=lambda _: None)
        self.alpha, self.obs_persist_frames, self.clear_persist_frames = 0.6, 2, 2
        self.obs_counter = np.zeros(self.num_channels, dtype=int)
        self.inf_counter = np.zeros(self.num_channels, dtype=int)
        self.filtered_dist = np.full(self.num_channels, self.range_max, dtype=np.float32)
        self.raw, self.filtered = None, None

    def apply_temporal_filter(self, raw):
        self.raw = raw
        self.filtered = RoiChecker.apply_temporal_filter(self, raw)
        return self.filtered

    def segment_floor(self, model, image):
        small = cv2.resize(image, (self.target_w, self.target_h), interpolation=cv2.INTER_LINEAR)
        result = model(small, stream=False, conf=0.15, device="cpu", verbose=False)
        floor = RoiChecker._extract_floor_mask(self, result)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        return cv2.morphologyEx(floor, cv2.MORPH_CLOSE, kernel, iterations=2)

    def run(self, model, image, stamp):
        floor = self.segment_floor(model, image)
        RoiChecker.publish_channel_distances(self, floor, stamp)
        return self.raw, self.filtered


def valid_ranges(values):
    return np.isfinite(values) & (values >= RANGE_MIN) & (values <= RANGE_MAX)


def flicker_and_delta(raw, filtered):
    raw_valid, filtered_valid = valid_ranges(raw), valid_ranges(filtered)
    raw_flips = raw_valid[1:] != raw_valid[:-1]
    filtered_flips = filtered_valid[1:] != filtered_valid[:-1]
    common = raw_valid[1:] & raw_valid[:-1] & filtered_valid[1:] & filtered_valid[:-1]
    raw_delta = np.abs(raw[1:][common] - raw[:-1][common])
    filtered_delta = np.abs(filtered[1:][common] - filtered[:-1][common])
    return (
        int(raw_flips.sum()), int(filtered_flips.sum()), int(raw_flips.size),
        int(common.sum()), np.mean(raw_delta) if raw_delta.size else "",
        np.mean(filtered_delta) if filtered_delta.size else "",
        np.mean(raw_valid), np.mean(filtered_valid),
    )


def replay(directory, model, start_s, sample_s):
    info = bag_metadata(directory)
    origin = info["starting_time"]["nanoseconds_since_epoch"]
    start_ns = origin + round(start_s * 1e9)
    stop_ns = start_ns + round(sample_s * 1e9)
    window = (start_ns, stop_ns)
    session = ReplayScan()
    bridge = CvBridge()
    raws, filtered, pose = [], [], []
    bucket = -1
    for stamp, topic, data in bag_messages(directory, info, ("/odom", "/camera/image_raw"), window):
        if topic == "/odom":
            odom = deserialize_message(data, Odometry)
            point = odom.pose.pose.position
            twist = odom.twist.twist
            pose.append((point.x, point.y, abs(twist.linear.x), abs(twist.angular.z)))
        else:
            next_bucket = (stamp - start_ns) // FRAME_PERIOD_NS
            if next_bucket == bucket:
                continue
            bucket = next_bucket
            image = deserialize_message(data, Image)
            bgr = bridge.imgmsg_to_cv2(image, "bgr8")
            raw, kept = session.run(model, bgr, image.header.stamp)
            if stamp >= start_ns + 1_000_000_000:
                raws.append(raw)
                filtered.append(kept)
    if len(raws) < 2 or not pose:
        raise ValueError(f"{directory}: insufficient RGB or odometry in the selected interval")
    x0, y0 = pose[0][:2]
    return {
        "bag_path": str(directory),
        "elapsed_start_s": start_s,
        "elapsed_stop_s": start_s + sample_s,
        "replay_frames": len(raws),
        "max_odom_translation_m": max(math.hypot(x - x0, y - y0) for x, y, _, _ in pose),
        "max_odom_abs_vx_mps": max(row[2] for row in pose),
        "max_odom_abs_wz_radps": max(row[3] for row in pose),
        **dict(zip((
            "raw_validity_flips", "filtered_validity_flips", "all_channel_transitions",
            "common_valid_transitions", "raw_mean_abs_range_step_m",
            "filtered_mean_abs_range_step_m", "raw_valid_fraction", "filtered_valid_fraction",
        ), flicker_and_delta(np.asarray(raws), np.asarray(filtered)))),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", type=float, default=0.0, help="start in bag-relative seconds")
    parser.add_argument("--seconds", type=float, default=12.0, help="window duration in seconds")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("bags", type=Path, nargs="+")
    args = parser.parse_args()
    if args.start < 0 or args.seconds <= 1:
        parser.error("select a nonnegative start and a window longer than the one-second warmup")
    model = YOLO(str(MODEL_DIR / "best_openvino_model"), task="segment")
    rows = [replay(bag, model, args.start, args.seconds) for bag in args.bags]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
