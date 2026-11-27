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
from scan_filter_metrics import flicker_and_delta, onset_timing
from scan_recorder_timing import IMAGE as RGB_TOPIC, SCAN as SCAN_TOPIC, header_timestamp


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/freespace_detection"))
from freespace_detection.floor_detector import RoiChecker


MODEL_DIR = ROOT / "src/freespace_detection/scripts"
FRAME_PERIOD_NS = 100_000_000


class ReplayScan:
    """Run the deployed mask and scan functions with publication disconnected."""

    def __init__(self, floor_class_id=0):
        self.target_w, self.target_h = 320, 256
        self.floor_class_id = floor_class_id
        self.y_min_scan, self.min_floor_scan_pixels = 120, 50
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

    def infer(self, model, image):
        small = cv2.resize(image, (self.target_w, self.target_h), interpolation=cv2.INTER_LINEAR)
        return model(small, stream=False, conf=0.15, device="cpu", verbose=False)

    def floor_mask(self, result):
        floor = RoiChecker._extract_floor_mask(self, result)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        return cv2.morphologyEx(floor, cv2.MORPH_CLOSE, kernel, iterations=2)

    def segment_floor(self, model, image):
        return self.floor_mask(self.infer(model, image))

    def project(self, result, stamp):
        RoiChecker.publish_channel_distances(self, self.floor_mask(result), stamp)
        return self.raw, self.filtered

    def run(self, model, image, stamp):
        return self.project(self.infer(model, image), stamp)


def replay(directory, model, start_s, sample_s, scan_stamped=False, sample_output=None):
    info = bag_metadata(directory)
    origin = info["starting_time"]["nanoseconds_since_epoch"]
    start_ns = origin + round(start_s * 1e9)
    stop_ns = start_ns + round(sample_s * 1e9)
    window = (start_ns, stop_ns)
    session = ReplayScan()
    bridge = CvBridge()
    raws, filtered, pose, stamps = [], [], [], []
    scan_sources = set()
    if scan_stamped:
        scan_sources = {header_timestamp(prefix) for _, _, prefix in
                        bag_messages(directory, info, (SCAN_TOPIC,), payload_bytes=12)}
        scan_sources.discard(0)
    bucket = -1
    for stamp, topic, data in bag_messages(directory, info, ("/odom", RGB_TOPIC), window):
        if topic == "/odom":
            odom = deserialize_message(data, Odometry)
            point = odom.pose.pose.position
            twist = odom.twist.twist
            pose.append((point.x, point.y, abs(twist.linear.x), abs(twist.angular.z)))
        else:
            if scan_stamped:
                acquisition = header_timestamp(data[:12])
                if acquisition not in scan_sources:
                    continue
                scan_sources.remove(acquisition)
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
                if scan_stamped:
                    stamps.append(acquisition)
    if len(raws) < 2 or not pose:
        raise ValueError(f"{directory}: insufficient RGB or odometry in the selected interval")
    if scan_stamped and (len(stamps) < 3 or np.any(np.diff(stamps) <= 0)):
        raise ValueError(f"{directory}: expected at least three strictly ordered matched acquisition stamps")
    raw_ranges, filtered_ranges = np.asarray(raws), np.asarray(filtered)
    if sample_output is not None:
        stamps_ns = np.asarray(stamps, dtype=np.int64)
        if stamps_ns.size:
            stamps_ns -= stamps_ns[0]
        np.savez_compressed(sample_output, bypass_m=raw_ranges, enabled_m=filtered_ranges,
                            acquisition_elapsed_ns=stamps_ns)
    x0, y0 = pose[0][:2]
    return {
        "bag_path": str(directory),
        "elapsed_start_s": start_s,
        "elapsed_stop_s": start_s + sample_s,
        "replay_frames": len(raws),
        "max_odom_translation_m": max(math.hypot(x - x0, y - y0) for x, y, _, _ in pose),
        "max_odom_abs_vx_mps": max(row[2] for row in pose),
        "max_odom_abs_wz_radps": max(row[3] for row in pose),
        **flicker_and_delta(raw_ranges, filtered_ranges),
        **(onset_timing(raw_ranges, filtered_ranges, stamps) if scan_stamped else {}),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", type=float, default=0.0, help="start in bag-relative seconds")
    parser.add_argument("--seconds", type=float, default=12.0, help="window duration in seconds")
    parser.add_argument("--scan-stamped", action="store_true", help="replay only RGB frames with matching scan stamps")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("bags", type=Path, nargs="+")
    args = parser.parse_args()
    if args.start < 0 or args.seconds <= 1:
        parser.error("select a nonnegative start and a window longer than the one-second warmup")
    model = YOLO(str(MODEL_DIR / "best_openvino_model"), task="segment")
    rows = [replay(bag, model, args.start, args.seconds, args.scan_stamped) for bag in args.bags]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
