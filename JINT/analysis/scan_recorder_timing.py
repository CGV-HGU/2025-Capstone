#!/usr/bin/env python3
"""Measure camera-to-scan recorder timing and source-stamp update cadence."""

import argparse
from collections import Counter
import csv
from pathlib import Path
import struct

import numpy as np

from bag_sqlite import bag_messages, bag_metadata


IMAGE = "/camera/image_raw"
SCAN = "/scan"
TOPIC_TYPES = {IMAGE: "sensor_msgs/msg/Image", SCAN: "sensor_msgs/msg/LaserScan"}
CDR_ENDIAN = {b"\x00\x00": ">", b"\x00\x01": "<"}


def header_timestamp(prefix):
    # Both ROS types start with std_msgs/Header.stamp after CDR encapsulation.
    endian = CDR_ENDIAN.get(prefix[:2])
    if endian is None or len(prefix) < 12:
        raise ValueError("unsupported or truncated ROS 2 CDR header")
    seconds, nanoseconds = struct.unpack_from(f"{endian}iI", prefix, offset=4)
    if nanoseconds >= 1_000_000_000:
        raise ValueError(f"invalid ROS 2 timestamp nanoseconds: {nanoseconds}")
    return seconds * 1_000_000_000 + nanoseconds


def quantiles(values):
    return tuple(round(float(value), 3) for value in np.percentile(values, [50, 95])) if len(values) else ("", "")


def measure(directory):
    info = bag_metadata(directory)
    types = {
        entry["topic_metadata"]["name"]: entry["topic_metadata"]
        for entry in info["topics_with_message_count"]
    }
    for topic, expected in TOPIC_TYPES.items():
        meta = types.get(topic, {})
        if meta.get("serialization_format") != "cdr" or meta.get("type") != expected:
            raise ValueError(f"{directory}: expected {expected} with CDR encoding on {topic}")

    data = {IMAGE: [], SCAN: []}
    for receipt, topic, prefix in bag_messages(directory, info, (IMAGE, SCAN), payload_bytes=12):
        data[topic].append((header_timestamp(prefix), receipt))
    scan_receipt = {}
    for stamp, receipt in data[SCAN]:
        if stamp > 0:
            scan_receipt[stamp] = min(scan_receipt.get(stamp, receipt), receipt)
    image_count = Counter(stamp for stamp, _ in data[IMAGE] if stamp > 0)
    images = {stamp: receipt for stamp, receipt in data[IMAGE] if image_count[stamp] == 1}
    pairs = scan_receipt.keys() & images.keys()
    delays = [(scan_receipt[stamp] - images[stamp]) / 1e6 for stamp in pairs]
    gaps_ms = np.diff(np.array(sorted(scan_receipt), dtype=np.int64)) / 1e6
    p50_scan, p95_scan = quantiles(gaps_ms)
    p50_delay, p95_delay = quantiles(delays)
    p50_rgb_age, p95_rgb_age = quantiles([(images[stamp] - stamp) / 1e6 for stamp in pairs])
    p50_scan_age, p95_scan_age = quantiles([(scan_receipt[stamp] - stamp) / 1e6 for stamp in pairs])
    nonzero_scan = sum(stamp > 0 for stamp, _ in data[SCAN])
    return {
        "bag_id": directory.name,
        "rgb_messages": len(data[IMAGE]),
        "scan_messages": len(data[SCAN]),
        "unique_nonzero_scan_stamps": len(scan_receipt),
        "duplicate_nonzero_scan_messages": nonzero_scan - len(scan_receipt),
        "unique_camera_scan_matches": len(delays),
        "match_fraction_of_unique_scan_stamps": round(len(delays) / len(scan_receipt), 6) if scan_receipt else "",
        "negative_recorder_deltas": sum(value < 0 for value in delays),
        "source_update_mean_hz": round(1e3 / float(np.mean(gaps_ms)), 3) if gaps_ms.size else "",
        "source_update_gap_median_ms": p50_scan,
        "source_update_gap_p95_ms": p95_scan,
        "camera_to_scan_recorder_delta_median_ms": p50_delay,
        "camera_to_scan_recorder_delta_p95_ms": p95_delay,
        "image_header_to_recorder_median_ms": p50_rgb_age,
        "image_header_to_recorder_p95_ms": p95_rgb_age,
        "scan_header_to_recorder_median_ms": p50_scan_age,
        "scan_header_to_recorder_p95_ms": p95_scan_age,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("bags", type=Path, nargs="+")
    args = parser.parse_args()
    rows = [measure(bag) for bag in args.bags]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
