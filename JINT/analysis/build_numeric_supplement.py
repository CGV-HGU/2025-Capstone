#!/usr/bin/env python3
"""Export numerical evidence for the JINT timing, filter, and trial results."""

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
TRIAL_CSVS = (
    "hall_session_2026-09-21.csv", "hall_session_2026-09-21_nav_logs.csv", "bridge_session_2026-11-27_audit.csv",
    "bridge_session_2026-11-27_rgb_odom_pairs.csv", "bridge_session_2026-11-27_image_continuity.csv",
    "class5_session_2026-11-27.csv", "class5_20_recovery_costmap.csv", "class5_20_chair_scan.csv",
    "class5_session_2026-11-27_progress.csv", "bridge_session_2026-11-27_end_states.csv",
    "bridge_session_2026-11-27_cart_scan.csv", "followup_rgb_samples.csv", "followup_rgb_review.csv",
    "box_approach_cues.csv", "hall_goal_paths.csv", "hall_goal_excursions.csv",
    "floor_class_scan_replay.csv",
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
        files.extend(EVIDENCE / name for name in TRIAL_CSVS)
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

Numerical data for offline CPU timing, recorded scan cadence, paired temporal
filter results, and the recorded navigation diagnostics.

Unzip this archive and run `python3 recompute_numeric_supplement.py` with NumPy
installed. It creates four recomputed_*.csv files beside the script.

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
For each initially withheld bin with a preceding and a following sample,
recomputed_filter_timing.csv counts confirmation while the bypass remains valid,
loss of the bypass return before confirmation, and right censoring at clip end.
Episode delays use only confirmed returns. An episode occupies one angular bin.

floor_class_scan_replay.csv compares the scans reproduced using each candidate
floor-class ID with recorded scans at matching acquisition times in two bags.
Agreement measures reproducibility of the deployed output.

This archive supports recalculation of the reported scan and timing statistics.
hall_session_2026-09-21.csv gives the 20 historical hall endpoints relative to
nominal goals reconstructed from the first recorded odometry pose. Its entry
times are relative to recording start. The corresponding hall nav_logs CSV
joins goal client and navigator console results to the 20 bag intervals. Its
result duration starts at acceptance. The bridge session CSVs contain client
outcomes and RGB-odometry comparisons. The Fifth Reading Room session CSV
contains client outcomes, odometric goal distances, and recovery events. The
run-20 costmap CSV gives pre/post grid counts and forward-motion timing. The
chair scan CSV gives sample selection, recorder offsets, projected ranges and
nearby grid costs for the two chair views. The corridor progress CSV covers all
logged failures. The event times use goal acceptance; command and grid offsets
use the local-clear time when present and the failure time otherwise. Blank
entries indicate observations outside the stated time windows. Trial
times for the bridge audit and Fifth Reading Room are relative to goal acceptance
unless the column specifies otherwise. These trial tables contain recorded
navigation diagnostics. External field assessments are separate.

hall_goal_paths.csv gives every wheel x and y sample in the active hall goal
interval, with nanoseconds from the logged client acceptance. The excursions
CSV gives peak absolute lateral position, linearly interpolated first forward
crossings of x=4.0 and 4.5 m, and the last active-goal pose. Blank crossings
indicate the station was never reached. Lateral position is relative to the
nominal centerline.

followup_rgb_samples.csv lists the recorder timestamps of the sampled forward
views. followup_rgb_review.csv describes the initial object and last view of
every retained follow-up recording. Operator motives use a field report only
where available. These annotations do not determine clearance or physical arrival.

box_approach_cues.csv includes recorder timestamps and horizontal box annotations
on 640 by 480 RGB frames. Pixel width is right_x_px minus left_x_px. The V-LiDAR
cue is the inverse of median finite range over the inner half of the box width.
Depth models report median relative disparity over a three-row strip. The
recomputation counts within-clip pairs with strictly different widths above the
specified margin and with increasing cue for the larger image. Pairs share
frames. The resulting counts describe these two approaches.

The underlying sensor recordings are held by the authors.
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
