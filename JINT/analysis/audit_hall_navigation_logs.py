#!/usr/bin/env python3
"""Join the September hall bags with the corresponding ROS goal and result logs."""

import argparse
from bisect import bisect_right
from collections import defaultdict
import csv
from pathlib import Path
import re

from bag_sqlite import bag_metadata


NANOSECONDS = 1_000_000_000
TARGET = "Target Map Pose (x=10.00, y=0.00,"
ACCEPTED = re.compile(r"Goal accepted! .* \(Timeout: ([\d.]+)s\)\.\.\.")
LINE = re.compile(r"^\[\w+\] \[(\d+)\.(\d{9})\] \[([^]]+)\]: (.*)$")
SERVER_RESULTS = {"Goal succeeded": "SUCCEEDED", "Goal canceled": "CANCELED"}


def log_events(log_dir, runs):
    starts = [start for _, start, _ in runs]
    events = defaultdict(list)
    for pattern in ("python3_*.log", "component_container_isolated_*.log"):
        for path in sorted(log_dir.glob(pattern)):
            started_ms = path.stem.rsplit("_", 1)[-1]
            if not started_ms.isdecimal():
                continue
            started = int(started_ms) * 1_000_000
            if not starts[0] - 3_600 * NANOSECONDS <= started <= runs[-1][2]:
                continue
            with path.open(encoding="utf-8", errors="replace") as stream:
                for number, line in enumerate(stream, 1):
                    match = LINE.match(line)
                    if not match:
                        continue
                    seconds, nanoseconds, node, message = match.groups()
                    stamp = int(seconds) * NANOSECONDS + int(nanoseconds)
                    index = bisect_right(starts, stamp) - 1
                    if index >= 0 and stamp <= runs[index][2]:
                        events[runs[index][0]].append((stamp, node, message, f"{path.name}:{number}"))
    return events


def exactly_one(matches, bag_id, event):
    if len(matches) != 1:
        raise ValueError(f"{bag_id}: expected one {event}, found {len(matches)}")
    return matches[0]


def audit_bag(bag_id, start, events):
    client = [row for row in events if row[1] == "avoidance_goal_sender"]
    server = [row for row in events if row[1] == "bt_navigator"]
    sent = exactly_one([row for row in client if TARGET in row[2]], bag_id, "logged 10 m goal")
    accepted = exactly_one([row for row in client if ACCEPTED.fullmatch(row[2])], bag_id, "client goal acceptance")
    started = exactly_one([row for row in server if row[2].startswith("Begin navigating from")
                           and row[2].endswith("to (10.00, 0.00)")], bag_id, "navigator start")
    client_end = exactly_one([row for row in client if row[2].startswith(("Navigation Succeeded!",
                                                                         "Navigation TIMED OUT after"))],
                             bag_id, "client result")
    server_end = exactly_one([row for row in server if row[2] in SERVER_RESULTS], bag_id, "navigator result")
    client_status = "SUCCEEDED" if client_end[2].startswith("Navigation Succeeded!") else "TIMEOUT"
    server_status = SERVER_RESULTS[server_end[2]]
    consistent = ((client_status, server_status) in {("SUCCEEDED", "SUCCEEDED"), ("TIMEOUT", "CANCELED")})
    if (not consistent or not sent[0] <= min(started[0], accepted[0]) < client_end[0] or
            abs(accepted[0] - started[0]) > 500_000_000 or abs(client_end[0] - server_end[0]) > 500_000_000):
        raise ValueError(f"{bag_id}: client and navigator logs do not agree")
    return {
        "bag_id": bag_id,
        "client_status": client_status,
        "navigator_status": server_status,
        "client_deadline_s": float(ACCEPTED.fullmatch(accepted[2])[1]),
        "goal_acceptance_from_bag_start_s": round((accepted[0] - start) / NANOSECONDS, 6),
        "acceptance_to_client_result_s": round((client_end[0] - accepted[0]) / NANOSECONDS, 6),
        "navigator_result_minus_client_result_ms": round((server_end[0] - client_end[0]) / 1e6, 3),
        "client_goal_log": sent[3],
        "client_acceptance_log": accepted[3],
        "client_result_log": client_end[3],
        "navigator_start_log": started[3],
        "navigator_result_log": server_end[3],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", type=Path, default=Path.home() / "data/bags")
    parser.add_argument("--logs", type=Path, default=Path.home() / ".ros/log")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).with_name("evidence") / "hall_session_2026-09-21_nav_logs.csv")
    args = parser.parse_args()
    runs = []
    for index in range(1, 21):
        bag_id = f"minimal_A_scen1_run{index:02d}"
        info = bag_metadata(args.bags / bag_id)
        start = info["starting_time"]["nanoseconds_since_epoch"]
        runs.append((bag_id, start, start + info["duration"]["nanoseconds"]))
    runs.sort(key=lambda trial: trial[1])
    events = log_events(args.logs, runs)
    rows = [audit_bag(bag_id, start, events[bag_id]) for bag_id, start, _ in runs]
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
