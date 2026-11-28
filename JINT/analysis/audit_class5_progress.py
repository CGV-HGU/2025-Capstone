#!/usr/bin/env python3
"""Audit every logged progress failure in the retained Fifth Reading Room session."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import json
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message

from audit_class5 import GOAL, GOAL_RADIUS
from audit_class5_recovery_costmap import CLEAR_MESSAGE, GRID, MOTION_THRESHOLD_MPS, TYPES, WINDOW_NS, goal_disk
from bag_sqlite import bag_messages, bag_metadata


PROGRESS_FAILURE = "Failed to make progress"
PAIR_WINDOW_NS = 1_000_000_000
LEAD_WINDOW_NS = 500_000_000


def grid_fields(sample, anchor, label):
    if sample is None:
        return {label + suffix: None for suffix in ("_grid_ms", "_full_disk", "_goal_cells", "_cost_99_100", "_unknown")}
    time, grid = sample
    origin, info = grid.info.origin.position, grid.info
    gx, gy = GOAL
    full_disk = (origin.x <= gx - GOAL_RADIUS and origin.y <= gy - GOAL_RADIUS and
                 origin.x + info.width * info.resolution >= gx + GOAL_RADIUS and
                 origin.y + info.height * info.resolution >= gy + GOAL_RADIUS)
    values = goal_disk(grid)
    return {
        f"{label}_grid_ms": round((time - anchor) / 1e6, 3),
        f"{label}_full_disk": int(full_disk),
        f"{label}_goal_cells": values.size,
        f"{label}_cost_99_100": np.count_nonzero((values == 99) | (values == 100)),
        f"{label}_unknown": np.count_nonzero(values < 0),
    }


def audit_run(bag, result, expected_failures):
    # The session audit verified an identity map-to-odom transform and absolute 10 m goals.
    start, end = result["goal_accepted_time_ns"], result["result_or_cancel_time_ns"]
    streams = {topic: [] for topic in TYPES}
    for stamp, topic, payload in bag_messages(bag, bag_metadata(bag), TYPES, (start, end + 1)):
        streams[topic].append((stamp, deserialize_message(payload, TYPES[topic])))
    logs = streams["/rosout"]
    failures = [stamp for stamp, message in logs if message.msg == PROGRESS_FAILURE]
    if [f"{(stamp - start) / 1e9:.3f}" for stamp in failures] != expected_failures:
        raise ValueError(f"{bag}: event times differ from the full-session audit")
    clears = [stamp for stamp, message in logs
              if message.name == "local_costmap.local_costmap" and message.msg == CLEAR_MESSAGE]
    for index, failure in enumerate(failures):
        next_failure = failures[index + 1] if index + 1 < len(failures) else end + 1
        clear = next((stamp for stamp in clears
                      if failure <= stamp < min(failure + PAIR_WINDOW_NS, next_failure)), None)
        anchor = clear if clear is not None else failure
        before = [(t, grid) for t, grid in streams[GRID] if anchor - WINDOW_NS <= t < anchor]
        after = next((sample for sample in streams[GRID] if anchor <= sample[0] <= anchor + WINDOW_NS), None)
        observed_until = min(end, anchor + WINDOW_NS, next_failure - 1)
        commands = streams["/cmd_vel"]
        lead = [msg.linear.x for t, msg in commands if failure - LEAD_WINDOW_NS <= t < failure]
        next_cmd = next((t for t, msg in commands
                         if anchor <= t <= observed_until and msg.linear.x > MOTION_THRESHOLD_MPS), None)
        next_odom = next((t for t, msg in streams["/odom"]
                          if anchor <= t <= observed_until and msg.twist.twist.linear.x > MOTION_THRESHOLD_MPS), None)
        yield {
            "trial_id": result["trial_id"],
            "nav_status": result["nav_status"],
            "failure_from_goal_acceptance_s": round((failure - start) / 1e9, 6),
            "local_clear_from_failure_ms": round((clear - failure) / 1e6, 3) if clear is not None else None,
            "post_anchor_observation_ms": round((observed_until - anchor) / 1e6, 3),
            "pre_failure_cmd_samples": len(lead),
            "pre_failure_forward_cmd_samples": sum(v > MOTION_THRESHOLD_MPS for v in lead),
            "next_forward_cmd_from_anchor_ms": round((next_cmd - anchor) / 1e6, 3) if next_cmd is not None else None,
            "next_forward_odom_from_anchor_ms": round((next_odom - anchor) / 1e6, 3) if next_odom is not None else None,
            **grid_fields(before[-1] if before else None, anchor, "before"),
            **grid_fields(after, anchor, "after"),
        }


def main():
    data = Path.home() / "data"
    evidence = Path(__file__).with_name("evidence")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", type=Path, default=data / "bags")
    parser.add_argument("--outcomes", type=Path, default=data / "trial_outcomes")
    parser.add_argument("--session-summary", type=Path, default=evidence / "class5_session_2026-11-27.csv")
    parser.add_argument("--output", type=Path, default=evidence / "class5_session_2026-11-27_progress.csv")
    args = parser.parse_args()

    with args.session_summary.open(encoding="utf-8") as stream:
        cases = [row for row in csv.DictReader(stream) if row["progress_failure_events_s"]]
    if not cases:
        parser.error("the session does not contain any recorded progress failures")

    def analyze(case):
        trial_id = case["trial_id"]
        result = json.loads((args.outcomes / f"{trial_id}.json").read_text(encoding="utf-8"))
        return list(audit_run(args.bags / f"nav_{trial_id}", result, case["progress_failure_events_s"].split(";")))

    with ThreadPoolExecutor(max_workers=2) as pool:
        rows = [row for run in pool.map(analyze, cases) for row in run]
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
