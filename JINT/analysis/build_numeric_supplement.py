#!/usr/bin/env python3
"""Export numerical evidence for the JINT CPU, scan-timing, and filter results."""

import argparse
from pathlib import Path
import tempfile
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo

from ultralytics import YOLO

from recompute_numeric_supplement import csv_rows, write_rows
from render_evidence_figures import (
    EVIDENCE, SUPPLEMENT_AFFILIATION, SUPPLEMENT_AUTHORS, SUPPLEMENT_JOURNAL,
    SUPPLEMENT_TITLE, CORRESPONDING_EMAIL,
)
from replay_scan_filter import MODEL_DIR, replay
from scan_recorder_timing import read_timestamps


ANALYSIS = Path(__file__).resolve().parent
METHODS = {
    "V-LiDAR, PyTorch": "vl_latency_ms",
    "V-LiDAR, OpenVINO FP16": "vl_openvino_latency_ms",
    "MiDaS v2.1 Small, PyTorch": "midas_latency_ms",
    "Depth Anything V2 Small, PyTorch": "da_latency_ms",
}
TIMING_BAGS = (
    "nav_test_11_26", "nav_trail_1_26", "nav_bridge_01", "nav_bridge_02",
    "nav_bridge_03", "nav_bridge_04", "nav_bridge_05", "vlidar_run01", "vlidar_run02",
)


def source_timing_rows(bags):
    rows = []
    for bag in TIMING_BAGS:
        topics = read_timestamps(bags / bag)
        # Keep valid stamps positive so a zero acquisition stamp stays distinguishable.
        positive_stamps = (stamp for samples in topics.values() for pair in samples for stamp in pair if stamp > 0)
        origin = min(positive_stamps) - 1
        for topic, samples in topics.items():
            for source, recorder in samples:
                rows.append({"bag_id": bag, "topic": topic, "source_ns": source - origin if source else "",
                             "recorder_ns": recorder - origin})
    return rows


def build(destination, bags, precheck):
    cpu_frames = []
    for run in ("run01", "run02"):
        source = EVIDENCE / f"pytorch_comparison_{run}.csv"
        for frame in list(csv_rows(source))[1:]:
            for method, column in METHODS.items():
                cpu_frames.append({"clip": source.stem, "frame": frame["frame"],
                                   "method": method, "latency_ms": frame[column]})

    clips = (
        ("bridge_precheck", precheck, 0.0, 12.0, False),
        ("reflective_hall", bags / "vlidar_run01", 0.0, 12.0, False),
        ("bridge_pilot_1", bags / "nav_bridge_01", 11.078, 62.062, True),
        ("bridge_pilot_2", bags / "nav_bridge_02", 6.296, 45.940, True),
    )
    model = YOLO(str(MODEL_DIR / "best_openvino_model"), task="segment")
    with tempfile.TemporaryDirectory(prefix="jint-numeric-") as temporary:
        directory = Path(temporary)
        index = []
        for label, bag, start, duration, scan_stamped in clips:
            filename = f"{label}.npz"
            replay(bag, model, start, duration, scan_stamped, sample_output=directory / filename)
            index.append({"clip": label, "bag_id": bag.name, "start_s": start,
                          "stop_s": start + duration, "samples": filename})
        write_rows(directory / "filter_windows.csv", index)
        write_rows(directory / "cpu_frames.csv", cpu_frames)
        write_rows(directory / "scan_messages.csv", source_timing_rows(bags))
        files = list(directory.iterdir()) + [ANALYSIS / name for name in (
            "recompute_numeric_supplement.py", "scan_filter_metrics.py", "scan_recorder_timing.py",
        )]
        destination.parent.mkdir(parents=True, exist_ok=True)
        with ZipFile(destination, "w", compression=ZIP_DEFLATED) as archive:
            archive.writestr(ZipInfo("README.txt"), readme().encode("utf-8"), compress_type=ZIP_DEFLATED)
            for file in sorted(files, key=lambda item: item.name):
                archive.writestr(ZipInfo(file.name), file.read_bytes(), compress_type=ZIP_DEFLATED)


def readme():
    return f"""{' '.join(SUPPLEMENT_TITLE.splitlines())}
{SUPPLEMENT_JOURNAL} | Online Resource 2
{SUPPLEMENT_AUTHORS}
{' '.join(SUPPLEMENT_AFFILIATION.splitlines())}
Corresponding author: Sung Soo Hwang ({CORRESPONDING_EMAIL})

Numerical data for the offline CPU timing, recorded scan cadence, and paired
temporal-filter results. The scan and CPU measurements come from prior recordings.

Unzip this archive and run `python3 recompute_numeric_supplement.py` with NumPy
installed. It creates three recomputed_*.csv files beside the script.

cpu_frames.csv contains the retained per-frame processing times in milliseconds.
The first sample from each of the two recordings was excluded before export.
Frame numbers identify rows within their original clip.

scan_messages.csv contains relative nanosecond timestamps. The same origin was
subtracted from both clocks within each recording. Blank source_ns denotes a
zero acquisition stamp. For the November cadence table use the seven nav_* IDs.
Hall test and Hall rerun correspond to nav_test_11_26 and nav_trail_1_26. The two
vlidar_run records support the historical source-stamp comparison in the text.

filter_windows.csv gives the replay windows from recorder start, including the
one-second initialization period. Each NPZ contains paired bypass_m and enabled_m
float arrays with frames as rows and internal scan channels as columns. Bounds
for valid observations are 0.1 to 10 m. acquisition_elapsed_ns contains elapsed
image acquisition times for the moving clips, where frames match recorded scan
source stamps. It is empty for the fixed-scene clips sampled by recorder time.

This archive supports recalculation of the reported scan and timing statistics.
Raw RGB, robot control logs, and adjudicated outcomes are separate records.
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", type=Path, default=Path.home() / "data/bags")
    parser.add_argument("--bridge-precheck", type=Path,
                        default=Path.home() / "data/jint_sources/bridge_static_20261126")
    parser.add_argument("--output", type=Path, default=EVIDENCE / "ESM_2.zip")
    args = parser.parse_args()
    build(args.output, args.bags, args.bridge_precheck)


if __name__ == "__main__":
    main()
