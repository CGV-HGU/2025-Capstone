#!/usr/bin/env python3
"""Audit the saved Fifth Reading Room pilot trials and plot a recovery example."""

import argparse
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from contextlib import closing
import csv
from itertools import groupby, pairwise
import json
import math
from pathlib import Path
import re

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage

from bag_sqlite import bag_metadata, open_databases


ROOT = Path(__file__).resolve().parents[1]
GOAL = (10.0, 0.0)
GOAL_RADIUS = 0.50
RECOVERY_FRAME_TIMES = (4, 12, 20)
QUERY = ("SELECT timestamp, data FROM messages WHERE topic_id = ? "
         "AND timestamp BETWEEN ? AND ? ORDER BY timestamp")
TARGET_LOG = re.compile(r"Target Map Pose \(x=([-\d.]+), y=([-\d.]+),")


def frames_from_database(db, ids, topic, message_type, start, end):
    if topic in ids:
        for stamp, payload in db.execute(QUERY, (ids[topic], start, end)):
            yield stamp, deserialize_message(payload, message_type)


def odom_row(stamp, msg, accepted):
    pose, twist = msg.pose.pose.position, msg.twist.twist
    return (stamp - accepted) / 1e9, pose.x, pose.y, twist.linear.x, twist.angular.z


def longest_zero_commands(commands):
    """Measure contiguous effectively zero commands with up to 0.25 s between samples."""
    start = previous = None
    longest = None
    for seconds, vx, wz in commands:
        if abs(vx) > 0.002 or abs(wz) > 0.002:
            start = previous = None
            continue
        if previous is None or seconds - previous > 0.25:
            start = seconds
        if longest is None or seconds - start > longest[1] - longest[0]:
            longest = start, seconds
        previous = seconds
    return longest


def audit_trial(result_path, bags):
    result = json.loads(result_path.read_text(encoding="utf-8"))
    run = result_path.stem
    if result["trial_id"] != run:
        raise ValueError(f"{result_path}: trial ID mismatch")
    if (result["goal_rel_x_m"], result["goal_rel_y_m"], result["goal_rel_yaw_deg"]) != (*GOAL, 0):
        raise ValueError(f"{result_path}: expected the 10 m straight-goal pilot")
    bag = bags / f"nav_{run}"
    metadata = bag_metadata(bag)
    bag_start = metadata["starting_time"]["nanoseconds_since_epoch"]
    start, end = result["goal_accepted_time_ns"], result["result_or_cancel_time_ns"]
    if not bag_start <= start < end <= bag_start + metadata["duration"]["nanoseconds"]:
        raise ValueError(f"{bag}: goal interval is outside the recording")
    odom, commands, scan_times = [], [], set()
    recovery_frames = {}
    counts = defaultdict(int)
    progress_failures, local_clears = [], []
    targets = set()
    with closing(open_databases(bag, metadata)) as databases:
        for db in databases:
            ids = {name: topic_id for topic_id, name in db.execute("SELECT id, name FROM topics")}
            messages = lambda name, kind, begin=start: frames_from_database(db, ids, name, kind, begin, end)
            odom.extend(odom_row(t, od, start) for t, od in messages("/odom", Odometry))
            commands.extend(((t-start)/1e9, v.linear.x, v.angular.z)
                            for t, v in messages("/cmd_vel", Twist))
            for _, scan in messages("/scan", LaserScan):
                stamp = scan.header.stamp
                nanoseconds = stamp.sec * 1_000_000_000 + stamp.nanosec
                if nanoseconds:
                    scan_times.add(nanoseconds)
            if next(messages("/camera/image_raw", Image, bag_start), None):
                counts["readable_rgb_splits"] += 1
            if run == "class5_20":
                for seconds in RECOVERY_FRAME_TIMES:
                    target = start + seconds * 1_000_000_000
                    candidates = frames_from_database(db, ids, "/camera/image_raw", Image,
                                                     target - 350_000_000, target + 350_000_000)
                    nearest = min(candidates, key=lambda sample: abs(sample[0] - target), default=None)
                    if nearest is not None:
                        delta = abs(nearest[0] - target)
                        previous = recovery_frames.get(seconds)
                        if previous is None or delta < abs(previous[0] - target):
                            recovery_frames[seconds] = nearest
            for t, event in messages("/rosout", Log, bag_start):
                match = TARGET_LOG.search(event.msg)
                if event.name == "avoidance_goal_sender" and match:
                    targets.add(tuple(map(float, match.groups())))
                if event.msg == "Failed to make progress" and t >= start:
                    progress_failures.append((t-start)/1e9)
                if (event.name == "local_costmap.local_costmap" and
                        "Received request to clear entirely the local_costmap" in event.msg and t >= start):
                    local_clears.append((t-start)/1e9)
            for frame in ("/tf", "/tf_static"):
                for _, message in messages(frame, TFMessage, bag_start):
                    for tf in message.transforms:
                        if (tf.header.frame_id, tf.child_frame_id) != ("map", "odom"):
                            continue
                        counts["map_to_odom"] += 1
                        p, q = tf.transform.translation, tf.transform.rotation
                        if (p.x, p.y, p.z, q.x, q.y, q.z, abs(q.w)) != (0, 0, 0, 0, 0, 0, 1):
                            raise ValueError(f"{bag}: nonidentity map-to-odom transform")

    if targets != {GOAL} or not counts["map_to_odom"]:
        raise ValueError(f"{bag}: cannot verify the recorded map goal or odometry frame (targets={targets})")
    if not (len(odom) >= 2 and commands and len(scan_times) >= 2 and counts["readable_rgb_splits"]):
        raise ValueError(f"{bag}: an essential during-trial stream is empty")
    odom.sort()
    commands.sort()
    relative_goal_distances = [(sample[0], math.hypot(sample[1]-GOAL[0], sample[2]-GOAL[1])) for sample in odom]
    first_entry = next((t for t, dist in relative_goal_distances if dist <= GOAL_RADIUS), None)
    closest_t, closest = min(relative_goal_distances, key=lambda row: row[1])
    stop_start, stop_end = longest_zero_commands(commands) or (None, None)
    stop_odom = [(x, y) for t, x, y, _, _ in odom if stop_start is not None and stop_start <= t <= stop_end]
    stop_displacement = max((math.dist(stop_odom[0], p) for p in stop_odom), default=None)
    fixed_twist = (0.0, None, None, None, 0, None)
    for twist, group in groupby(odom, key=lambda row: row[3:]):
        rows = list(group)
        span = rows[-1][0] - rows[0][0]
        if max(map(abs, twist)) > 0.02 and span > fixed_twist[0]:
            fixed_twist = (span, rows[0][0], *twist, len(rows), math.dist(rows[0][1:3], rows[-1][1:3]))
    scan_times = sorted(scan_times)
    scan_gaps = [(b-a)/1e9 for a, b in pairwise(scan_times)]
    topics = {row["topic_metadata"]["name"]: row["message_count"] for row in metadata["topics_with_message_count"]}
    row = {
        "trial_id": run, "timeout_s": result["timeout_sec"], "nav_status": result["nav_status"],
        "terminal_event": result["goal_end_event"], "goal_to_terminal_s": (end-start)/1e9,
        "odom_span_s": odom[-1][0]-odom[0][0], "first_odom_50cm_s": first_entry,
        "closest_goal_m": closest, "closest_goal_s": closest_t,
        "closest_goal_through_90s_m": min(d for t, d in relative_goal_distances if t <= 90),
        "last_goal_m": relative_goal_distances[-1][1],
        "odom_end_x_m": odom[-1][1], "odom_end_y_m": odom[-1][2],
        "odom_max_message_gap_s": max(b[0]-a[0] for a, b in pairwise(odom)),
        "longest_zero_cmd_start_s": stop_start, "longest_zero_cmd_end_s": stop_end,
        "longest_zero_cmd_s": None if stop_start is None else stop_end-stop_start,
        "odom_displacement_during_zero_cmd_m": stop_displacement,
        "longest_fixed_nonzero_odom_s": fixed_twist[0], "fixed_odom_start_s": fixed_twist[1],
        "fixed_odom_vx_mps": fixed_twist[2], "fixed_odom_wz_radps": fixed_twist[3],
        "fixed_odom_samples": fixed_twist[4], "fixed_odom_displacement_m": fixed_twist[5],
        "progress_failure_events_s": ";".join(f"{t:.3f}" for t in progress_failures),
        "local_clear_events_s": ";".join(f"{t:.3f}" for t in local_clears),
        "scan_rate_hz": (len(scan_times)-1) * 1e9/(scan_times[-1]-scan_times[0]),
        "scan_gap_p95_s": float(np.percentile(scan_gaps, 95)), "scan_gap_max_s": max(scan_gaps),
        "rgb_count": topics["/camera/image_raw"], "scan_count": topics["/scan"],
        "odom_count": topics["/odom"], "cmd_count": topics["/cmd_vel"],
        "map_to_odom_transforms": counts["map_to_odom"],
    }
    frames = [(seconds, recovery_frames[seconds][1]) for seconds in RECOVERY_FRAME_TIMES] if recovery_frames else []
    return row, relative_goal_distances, odom, commands, local_clears, frames


def render_recovery(metrics, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from render_archive_figures import rgb_image

    row, distance, odom, commands, clears, frames = metrics
    if len(frames) != len(RECOVERY_FRAME_TIMES):
        raise ValueError("run 20: missing recovery figure RGB frames")
    plt.rcParams.update({"font.size": 8, "axes.spines.top": False, "axes.spines.right": False,
                         "font.family": "DejaVu Sans", "pdf.fonttype": 42})
    fig, panels = plt.subplot_mosaic([["first", "middle", "last"], ["distance"] * 3, ["speed"] * 3],
                                    figsize=(6.45, 5.1), layout="constrained",
                                    gridspec_kw={"height_ratios": (1.2, 1, .85)})
    for key, (seconds, message) in zip(("first", "middle", "last"), frames):
        photo = panels[key]
        photo.imshow(rgb_image(message))
        photo.set_title(f"(a) {seconds} s" if key == "first" else f"{seconds} s", fontsize=8)
        photo.set_axis_off()
    ax, speed = panels["distance"], panels["speed"]
    ax.sharex(speed)
    ax.tick_params(axis="x", labelbottom=False)
    ax.plot(*zip(*distance), color="#185a76", linewidth=1.4, label="Wheel-odom distance")
    ax.axhline(GOAL_RADIUS, color="#555d64", linewidth=0.9, linestyle="--", label="0.50 m radius")
    entry = row["first_odom_50cm_s"]
    ax.scatter(entry, GOAL_RADIUS, s=22, color="#185a76", zorder=5, label=f"First entry ({entry:.1f} s)")
    ax.set(ylabel="Goal distance (m)", ylim=(0, 10.6), title="(b) Approach to the nominal goal")
    ax.legend(loc="upper right", frameon=True, framealpha=1, edgecolor="none", ncol=2, fontsize=7.3)
    speed.plot([t for t, *_ in commands], [v for _, v, _ in commands], color="#b45c1d",
               linewidth=1.1, label="Commanded forward speed")
    speed.plot([t for t, *_ in odom], [v for _, _, _, v, _ in odom], color="#185a76",
               linewidth=.85, linestyle="--", label="Wheel-odom forward speed")
    speed.set(xlabel="Time from goal acceptance (s)", ylabel="Forward speed (m/s)", ylim=(-.06, .43),
              title="(c) Velocity commands and feedback")
    speed.legend(loc="upper left", frameon=True, framealpha=1, edgecolor="none", fontsize=7.3, ncol=2)
    for axis in (ax, speed):
        axis.axvline(90, linewidth=1.0, linestyle=":", color="#6e6e6e")
        for t in clears:
            axis.axvline(t, linewidth=1.0, linestyle="--", color="#6f477c", alpha=.8)
        axis.grid(axis="y", color="#dedede", linewidth=.55)
    speed.text(89, -.047, "90 s", color="#505050", ha="right", va="bottom", fontsize=7)
    speed.set_xlim(0, math.ceil(row["goal_to_terminal_s"] / 5) * 5 + 1)
    fig.savefig(output, metadata={"CreationDate": None, "ModDate": None})
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", type=Path, default=Path.home() / "data/bags")
    parser.add_argument("--outcomes", type=Path, default=Path.home() / "data/trial_outcomes")
    parser.add_argument("--output", type=Path, default=ROOT / "analysis/evidence/class5_session_2026-11-27.csv")
    parser.add_argument("--figure", type=Path, default=ROOT / "manuscript/figures/class5_pilot_recovery.pdf")
    args = parser.parse_args()
    inputs = sorted(args.outcomes.glob("class5_[0-9][0-9].json"))
    if not inputs:
        parser.error("no class5 trials found")
    with ThreadPoolExecutor(max_workers=3) as pool:
        metrics = list(pool.map(lambda path: audit_trial(path, args.bags), inputs))
    rows = [group[0] for group in metrics]
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        output = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator="\n")
        output.writeheader()
        for row in rows:
            output.writerow({key: round(value, 6) if isinstance(value, float) else value
                             for key, value in row.items()})
    render_recovery(next(group for group in metrics if group[0]["trial_id"] == "class5_20"), args.figure)
    print(f"Audited {len(rows)} recordings: {args.output}")


if __name__ == "__main__":
    main()
