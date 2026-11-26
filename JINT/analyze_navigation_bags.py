#!/usr/bin/env python3
"""Extract pilot controller events and wheel-odometry diagnostics from ROS 2 bags."""

import argparse
import csv
from itertools import groupby
import math
from pathlib import Path
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan

from bag_sqlite import bag_messages, bag_metadata


TOPICS = ("/odom", "/cmd_vel", "/scan", "/rosout")
EVENT = re.compile(
    r"begin navigating|goal (?:reached|succeeded|failed|canceled)|preempt|"
    r"cancel(?:ing|ling|led|ed)|failed to (?:create|find|generate).{0,16}(?:path|plan)|"
    r"planner failed|valid (?:path|plan)|failed to make progress",
    re.IGNORECASE,
)


def longest_constant_turn(odometry):
    # An exactly repeated nonzero turn in the recorded twist, not a stale-driver verdict.
    turns = []
    for (vx, wz), samples in groupby(odometry, key=lambda sample: sample[1:]):
        if abs(wz) > 0.02:
            times = [sample[0] for sample in samples]
            turns.append((times[-1] - times[0], times[0], times[-1], vx, wz))
    return max(turns, default=None)


def read_trace(directory):
    info = bag_metadata(directory)
    start = info["starting_time"]["nanoseconds_since_epoch"]
    metadata_counts = {
        entry["topic_metadata"]["name"]: entry["message_count"]
        for entry in info["topics_with_message_count"]
    }
    odometry = []
    commands = []
    central_ranges = []
    events = []
    for stamp, topic, data in bag_messages(directory, info, TOPICS):
        elapsed = (stamp - start) / 1e9
        if topic == "/odom":
            twist = deserialize_message(data, Odometry).twist.twist
            odometry.append((elapsed, twist.linear.x, twist.angular.z))
        elif topic == "/cmd_vel":
            twist = deserialize_message(data, Twist)
            commands.append((elapsed, twist.linear.x, twist.angular.z))
        elif topic == "/scan":
            scan = deserialize_message(data, LaserScan)
            sector = [
                distance for i, distance in enumerate(scan.ranges)
                if abs(scan.angle_min + i * scan.angle_increment) <= math.radians(10) + 1e-6
                and math.isfinite(distance) and scan.range_min <= distance <= scan.range_max
            ]
            central_ranges.append((elapsed, min(sector, default=float("nan"))))
        else:
            log = deserialize_message(data, Log)
            if EVENT.search(log.msg):
                events.append((directory.name, round(elapsed, 3), log.name, log.msg))
    repeated = longest_constant_turn(odometry)
    summary = [directory.name, round(info["duration"]["nanoseconds"] / 1e9, 3),
               metadata_counts.get("/camera/image_raw", 0), len(central_ranges),
               len(odometry), len(commands), len(events)]
    if repeated:
        summary.extend(round(value, 6) for value in repeated)
    else:
        summary.extend([""] * 5)
    return summary, events, (odometry, commands, central_ranges, repeated)


def write_csv(path, header, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(header)
        writer.writerows(rows)


def plot_traces(path, selected):
    fig, axes = plt.subplots(3, len(selected), squeeze=False, sharex="col",
                             sharey="row", figsize=(7.2, 4.7), layout="constrained")
    for column, (name, (odometry, commands, ranges, repeated)) in enumerate(selected):
        for series, color, label in [(odometry, "#0072b2", "Wheel odom"),
                                     (commands, "#d55e00", "Command")]:
            if series:
                time, vx, wz = zip(*series)
                axes[0, column].plot(time, vx, color=color, label=label, linewidth=0.85)
                axes[1, column].plot(time, wz, color=color, linewidth=0.85)
        if ranges:
            time, distance = zip(*ranges)
            axes[2, column].plot(time, distance, color="#007f5f", linewidth=0.85)
        if repeated and repeated[0] >= 1.0:
            for ax in (axes[0, column], axes[1, column], axes[2, column]):
                ax.axvspan(repeated[1], repeated[2], color="#cab2d6", alpha=0.40, zorder=0)
        axes[0, column].set_title(name)
        axes[2, column].set_xlabel("Time from bag start (s)")
        for ax in axes[:, column]:
            ax.grid(color="0.91", linewidth=0.55)
    axes[0, 0].legend(frameon=False, fontsize=8, loc="upper right")
    axes[0, 0].set_ylabel("Forward speed (m/s)")
    axes[1, 0].set_ylabel("Yaw rate (rad/s)")
    axes[2, 0].set_ylabel("Center scan min. (m)")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--plot", type=Path)
    parser.add_argument("--plot-bags", nargs=2, metavar=("BAG_A", "BAG_B"))
    parser.add_argument("bags", type=Path, nargs="+")
    args = parser.parse_args()
    if bool(args.plot) != bool(args.plot_bags):
        parser.error("--plot and --plot-bags must be used together")

    summaries, events, selected = [], [], {}
    for bag in args.bags:
        summary, bag_events, trace = read_trace(bag)
        summaries.append(summary)
        events.extend(bag_events)
        if args.plot_bags and bag.name in args.plot_bags:
            selected[bag.name] = trace
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "pilot_trace_summary.csv",
              ["bag_id", "bag_duration_s", "rgb_frames_metadata", "scan_samples", "odom_samples",
               "command_samples", "selected_rosout_events", "repeated_turn_duration_s",
               "repeated_turn_start_s", "repeated_turn_end_s", "repeated_vx_mps",
               "repeated_wz_radps"], summaries)
    write_csv(args.output_dir / "pilot_nav_events.csv",
              ["bag_id", "bag_elapsed_s", "node", "message"], events)
    if args.plot:
        if set(args.plot_bags) - selected.keys():
            parser.error("each --plot-bags name must match an input bag directory")
        plot_traces(args.plot, [(name, selected[name]) for name in args.plot_bags])


if __name__ == "__main__":
    main()
