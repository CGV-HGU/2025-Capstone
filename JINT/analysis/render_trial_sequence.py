#!/usr/bin/env python3
"""Make a manuscript panel from one field-log entry and its RGB/odometry bag."""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from bag_sqlite import bag_metadata
from plot_pilot_odometry import align_to_start, read_poses
from render_archive_figures import METADATA, bag_frame
from summarize_trials import classify


def field_record(log, record_id):
    with log.open(encoding="utf-8-sig", newline="") as stream:
        rows = [row for row in csv.DictReader(stream) if (row.get("record_id") or "").strip() == record_id]
    if len(rows) != 1:
        raise ValueError(f"{log}: expected a single entry for record {record_id}")
    return {key: (value or "").strip() for key, value in rows[0].items() if key is not None}


def time_window(row, info, requested):
    origin = info["starting_time"]["nanoseconds_since_epoch"]
    last = origin + info["duration"]["nanoseconds"]
    if requested is not None:
        start, stop = (origin + round(second * 1e9) for second in requested)
    elif row.get("goal_accepted_time_ns") and row.get("result_or_cancel_time_ns"):
        start, stop = (int(row[key]) for key in ("goal_accepted_time_ns", "result_or_cancel_time_ns"))
    else:
        start, stop = origin, last
    if not origin <= start < stop <= last:
        raise ValueError("path window is outside the recording or empty; specify --window in bag-relative seconds")
    return origin, start, stop


def render_sequence(row, bag, seconds, window, output):
    origin, start, stop = time_window(row, bag_metadata(bag), window)
    if sorted(set(seconds)) != seconds or not all(start <= origin + round(time * 1e9) <= stop for time in seconds):
        raise ValueError("select three increasing RGB times inside the path window")
    poses = [pose for pose in read_poses(bag) if start <= pose[0] <= stop]
    if len(poses) < 2:
        raise ValueError(f"{bag}: fewer than two /odom samples inside the path window")
    aligned = align_to_start(poses)
    frames = [bag_frame(bag, time) for time in seconds]
    frame_stamps = [stamp for _, stamp in frames]
    if len(set(frame_stamps)) != 3 or frame_stamps[-1] > stop:
        raise ValueError(f"{bag}: selected times do not resolve to three distinct frames inside the path window")

    blue = "#175676"
    figure = plt.figure(figsize=(5.7, 3.6), layout="constrained")
    grid = figure.add_gridspec(2, 3)
    path = figure.add_subplot(grid[1, :])
    forward, lateral = zip(*aligned)
    path.plot(forward, lateral, color=blue, lw=1.4)
    path.scatter(forward[0], lateral[0], s=19, marker="s", color="0.50", label="First pose")
    path.scatter(forward[-1], lateral[-1], s=19, marker="D", color=blue, label="Last pose")

    for column, (label, (image, stamp)) in enumerate(zip("abc", frames)):
        index = min(range(len(poses)), key=lambda i: abs(poses[i][0] - stamp))
        recorder_offset = (poses[index][0] - stamp) / 1e6
        print(f"({label}) RGB {stamp} ns, odom {poses[index][0]} ns ({recorder_offset:+.1f} ms)")
        image_axis = figure.add_subplot(grid[0, column])
        image_axis.imshow(image)
        image_axis.set_title(f"({label}) {(stamp - origin) / 1e9:.2f} s", fontsize=9)
        image_axis.set_axis_off()
        xy = aligned[index]
        path.scatter(*xy, s=21, facecolor="white", edgecolor=blue, zorder=3)
        path.annotate(f"({label})", xy, xytext=(0, -13), textcoords="offset points",
                      ha="center", fontsize=8, color=blue)

    phase = row["phase"].lower()
    description = classify(row).replace("_", " ") if phase == "main" else phase
    figure.suptitle(f"{row['record_id']}  ·  {description.capitalize()}", fontsize=10)
    path.set(xlabel="Forward displacement (m)", ylabel="Lateral displacement (m)")
    path.margins(x=.06, y=.28)
    path.grid(color=".90", linewidth=.55)
    path.legend(loc="upper left", ncol=2, fontsize=8, frameon=False)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=320, metadata=METADATA)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("field_log", type=Path)
    parser.add_argument("record_id")
    parser.add_argument("--bags", type=Path, default=Path.home() / "data/bags")
    parser.add_argument("--frames", type=float, nargs=3, required=True, metavar=("A", "B", "C"),
                        help="RGB times in seconds from the start of recording")
    parser.add_argument("--window", type=float, nargs=2, metavar=("START", "STOP"),
                        help="override the path window using seconds from the start of recording")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        row = field_record(args.field_log, args.record_id)
        bag_name = row.get("bag_asset_id")
        if not bag_name or Path(bag_name).name != bag_name or bag_name == "..":
            raise ValueError(f"{args.record_id}: record a bag directory name in bag_asset_id")
        render_sequence(row, args.bags / bag_name, args.frames, args.window, args.output)
    except (OSError, ValueError, KeyError, OverflowError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
