#!/usr/bin/env python3
"""Recompute numerical results in Online Resource 2 with NumPy."""

from collections import defaultdict
import csv
from itertools import combinations
from pathlib import Path

import numpy as np

from scan_filter_metrics import flicker_and_delta, onset_timing
from scan_recorder_timing import IMAGE, SCAN, summarize_timestamps


def csv_rows(path):
    with path.open(encoding="utf-8", newline="") as stream:
        yield from csv.DictReader(stream)


def write_rows(path, rows):
    columns = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def cpu_timing(path):
    values = defaultdict(list)
    for row in csv_rows(path):
        values[row["method"]].append(float(row["latency_ms"]))
    return [{"method": method, "frames": len(samples), "mean_ms": np.mean(samples),
             "p95_ms": np.percentile(samples, 95)} for method, samples in values.items()]


def scan_timing(path):
    data = defaultdict(lambda: {IMAGE: [], SCAN: []})
    for row in csv_rows(path):
        data[row["bag_id"]][row["topic"]].append((int(row["source_ns"] or 0), int(row["recorder_ns"])))
    return [summarize_timestamps(bag, topics) for bag, topics in data.items()]


def filter_timing(directory):
    results = []
    for row in csv_rows(directory / "filter_windows.csv"):
        with np.load(directory / row["samples"], allow_pickle=False) as arrays:
            raw = arrays["bypass_m"]
            filtered = arrays["enabled_m"]
            stamps = arrays["acquisition_elapsed_ns"]
        result = {"clip": row["clip"], "replay_frames": len(raw), **flicker_and_delta(raw, filtered)}
        if stamps.size:
            result.update(onset_timing(raw, filtered, stamps))
        results.append(result)
    return results


def box_approach_order(path):
    clips = defaultdict(list)
    for row in csv_rows(path):
        clips[row["clip"]].append(row)
    results = []
    for clip, rows in clips.items():
        for margin in (0, 4, 8):
            pairs = [(a, b) for a, b in combinations(rows, 2)
                     if abs(int(b["width_px"]) - int(a["width_px"])) > margin]
            for cue in ("vl_inverse_range", "midas_disparity", "da_disparity"):
                signs = [(int(b["width_px"]) - int(a["width_px"])) * (float(b[cue]) - float(a[cue]))
                         for a, b in pairs if a[cue] and b[cue]]
                results.append({"clip": clip, "frames": len(rows), "width_margin_px": margin,
                                "cue": cue, "eligible_pairs": len(pairs), "finite_pairs": len(signs),
                                "matched_order": sum(value > 0 for value in signs),
                                "tied_output": sum(value == 0 for value in signs)})
    return results


def main():
    directory = Path(__file__).resolve().parent
    write_rows(directory / "recomputed_box_approach.csv", box_approach_order(directory / "box_approach_cues.csv"))
    write_rows(directory / "recomputed_cpu_timing.csv", cpu_timing(directory / "cpu_frames.csv"))
    write_rows(directory / "recomputed_scan_timing.csv", scan_timing(directory / "scan_messages.csv"))
    write_rows(directory / "recomputed_filter_timing.csv", filter_timing(directory))


if __name__ == "__main__":
    main()
