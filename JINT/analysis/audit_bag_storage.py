#!/usr/bin/env python3
"""Measure archived bag sizes and estimate storage for sequential trial recording."""

import argparse
import csv
import math
import os
from pathlib import Path
import subprocess

from bag_sqlite import bag_metadata


GB = 10**9


def database_size(file):
    if file.suffix != ".zstd":
        return file.stat().st_size
    command = ["zstd", "-dqc", "--", str(file)]
    with subprocess.Popen(command, stdout=subprocess.PIPE) as process:
        total = sum(len(chunk) for chunk in iter(lambda: process.stdout.read(2**20), b""))
        if process.wait() != 0:
            raise ValueError(f"Cannot decode compressed storage in {file}")
    return total


def measure(bag):
    info = bag_metadata(bag)
    duration = info["duration"]["nanoseconds"] / 1e9
    files = [bag / file for file in info["relative_file_paths"]]
    if not files or duration <= 0:
        raise ValueError(f"{bag}: expected a completed nonempty bag")
    rgb_messages = sum(item["message_count"] for item in info["topics_with_message_count"]
                       if item["topic_metadata"]["name"] == "/camera/image_raw")
    stored = sum(file.stat().st_size for file in files)
    raw = sum(database_size(file) for file in files)
    return {"bag_id": bag.name, "duration_s": duration, "rgb_messages": rgb_messages,
            "storage_files": len(files), "stored_bytes": stored, "uncompressed_db_bytes": raw,
            "stored_mb_per_s": stored / duration / 1e6, "uncompressed_mb_per_s": raw / duration / 1e6}


def estimate(label, n, stored_bytes, raw_bytes, free_bytes):
    final = n * stored_bytes
    peak = final + raw_bytes
    fits = max(0, math.floor((free_bytes - raw_bytes) / stored_bytes))
    print(f"{label}: {final / GB:.1f} GB archived, {peak / GB:.1f} GB at compression peak, "
          f"room for about {fits} runs under these assumptions")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", type=Path, required=True, help="existing directory on the recording filesystem")
    parser.add_argument("--trials", type=int, required=True)
    parser.add_argument("--recording-seconds", type=float, help="also estimate storage at this total bag duration")
    parser.add_argument("--output", type=Path, help="write the measurements to CSV")
    parser.add_argument("bags", type=Path, nargs="+")
    args = parser.parse_args()
    if args.trials <= 0:
        parser.error("--trials must be positive")
    seconds = args.recording_seconds
    if seconds is not None and (not math.isfinite(seconds) or seconds <= 0):
        parser.error("--recording-seconds must be finite and positive")

    rows = [measure(bag) for bag in args.bags]
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0], lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
    stats = os.statvfs(args.target)
    free = stats.f_bavail * stats.f_frsize
    stored = [row["stored_bytes"] for row in rows]
    raw = [row["uncompressed_db_bytes"] for row in rows]
    print(f"Available on {args.target}: {free / GB:.1f} GB. Base set: {len(rows)} bags.")
    print(f"Observed single-run sizes: {min(stored) / GB:.2f}–{max(stored) / GB:.2f} GB stored, "
          f"{min(raw) / GB:.2f}–{max(raw) / GB:.2f} GB uncompressed.")
    estimate("Observed mean bag size", args.trials, sum(stored) / len(rows), max(raw), free)
    estimate("Largest observed bag size", args.trials, max(stored), max(raw), free)
    if seconds is not None:
        total_duration = sum(row["duration_s"] for row in rows)
        largest_stored_rate = max(row["stored_mb_per_s"] for row in rows) * 1e6
        largest_raw_rate = max(row["uncompressed_mb_per_s"] for row in rows) * 1e6
        estimate(f"{seconds:g} s per bag at pooled rate", args.trials,
                 sum(stored) / total_duration * seconds, largest_raw_rate * seconds, free)
        estimate(f"{seconds:g} s per bag at largest observed rate", args.trials,
                 largest_stored_rate * seconds, largest_raw_rate * seconds, free)


if __name__ == "__main__":
    main()
