#!/usr/bin/env python3
"""Extract the wheel paths inside the 20 matched September goal intervals."""

import argparse
from contextlib import closing
import csv
from itertools import pairwise
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from audit_hall_archive import pose
from audit_hall_navigation_logs import LINE, NANOSECONDS
from bag_sqlite import bag_messages, bag_metadata


def event_stamp(logs, reference):
    filename, number = reference.rsplit(":", 1)
    with (logs / filename).open() as stream:
        line = next(line for i, line in enumerate(stream, 1) if i == int(number))
    match = LINE.match(line)
    if match is None or match[3] != "avoidance_goal_sender":
        raise ValueError(f"{reference}: invalid client event")
    return int(match[1]) * NANOSECONDS + int(match[2])


def first_crossing(points, station):
    for (_, x0, y0), (_, x1, y1) in pairwise(points):
        if x0 < station <= x1:
            return y0 + (y1-y0) * (station-x0) / (x1-x0)
    return None


def load_run(bags, logs, row):
    bag = bags / row["bag_id"]
    start = event_stamp(logs, row["client_acceptance_log"])
    end = event_stamp(logs, row["client_result_log"])
    if abs((end-start)/NANOSECONDS - float(row["acceptance_to_client_result_s"])) > 1e-6:
        raise ValueError(f"{bag.name}: result time disagrees with the audit")
    points = []
    with closing(bag_messages(bag, bag_metadata(bag), ["/odom"], (start, end+1))) as messages:
        for ts, _, payload in messages:
            x, y, yaw = pose(payload)
            if not points and max(abs(x), abs(y), abs(yaw)) > 1e-5:
                raise ValueError(f"{bag.name}: odometry axes are not aligned with the nominal route")
            if not all(math.isfinite(v) for v in (x, y)):
                raise ValueError(f"{bag.name}: nonfinite pose")
            points.append((ts-start, x, y))
    if len(points) < 2:
        raise ValueError(f"{bag.name}: insufficient odometry")
    return points


def render(runs, destination):
    color, timeout = "#397b8d", "#aa4d28"
    fig, ax = plt.subplots(figsize=(5.1, 2.42))
    fig.subplots_adjust(left=.105, right=.986, bottom=.20, top=.80)
    ax.axvspan(4, 4.5, color="#dfdacf", alpha=.70, lw=0)
    for trial, status, points in runs:
        _, xs, ys = zip(*points)
        if status == "SUCCEEDED":
            ax.plot(xs, ys, color=color, alpha=.57, lw=.85)
        else:
            ax.plot(xs, ys, color=timeout, ls="--", lw=1.25, zorder=4)
            ax.plot(xs[-1], ys[-1], "x", ms=5.4, color=timeout, zorder=5)
            offset = (2, 7) if trial.endswith("08") else (-8, 8)
            ax.annotate(trial[-2:], (xs[-1],ys[-1]), xytext=offset, textcoords="offset points",
                        ha="center", fontsize=7, color="#793317")
    ax.plot(10, 0, "*", color="#113d46", ms=8, zorder=6)
    ax.set(xlim=(-.2,10.35), ylim=(-.28,2.9), xlabel="Forward position in wheel odometry (m)",
           ylabel="Lateral position (m)", xticks=range(0,11,2), yticks=(0,1,2))
    ax.set_aspect("equal")
    ax.spines[["top","right"]].set_visible(False)
    ax.tick_params(labelsize=7)
    ax.grid(axis="y", lw=.4, color=".89")
    handles = [Line2D([],[],color=color,label="Completed (18)"),
               Line2D([],[],color=timeout, ls="--", marker="x", label="Timed out (2)"),
               Patch(color="#dfdacf", label="Reported box x"),
               Line2D([],[],color="#113d46",ls="",marker="*",label="Goal")]
    fig.legend(handles=handles, ncol=4, loc="upper center", fontsize=7.3, frameon=False,
               columnspacing=1.25, handlelength=1.7)
    fig.savefig(destination, metadata={"CreationDate":None, "ModDate":None})
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    paper = Path(__file__).resolve().parents[1]
    parser.add_argument("--bags", type=Path, default=Path.home()/"data/bags")
    parser.add_argument("--logs", type=Path, default=Path.home()/".ros/log")
    parser.add_argument("--evidence", type=Path, default=paper/"analysis/evidence")
    parser.add_argument("--figure", type=Path, default=paper/"manuscript/figures/hall_goal_paths.pdf")
    args = parser.parse_args()
    runs = []
    with (args.evidence/"hall_session_2026-09-21_nav_logs.csv").open() as stream:
        for row in csv.DictReader(stream):
            runs.append((row["bag_id"],row["client_status"],load_run(args.bags,args.logs,row)))
    with (args.evidence/"hall_goal_paths.csv").open("w") as data, (args.evidence/"hall_goal_excursions.csv").open("w") as summaries:
        samples = csv.writer(data, lineterminator="\n")
        metrics = csv.writer(summaries, lineterminator="\n")
        samples.writerow(("bag_id","from_accept_ns","odom_x_m","odom_y_m"))
        metrics.writerow(("bag_id","client_status","samples","peak_abs_y_m","first_x4_y_m","first_x4p5_y_m","last_x_m","last_y_m"))
        for trial, status, points in runs:
            for ns, x, y in points:
                samples.writerow((trial,ns,x,y))
            metrics.writerow((trial,status,len(points),max(abs(y) for _,_,y in points),
                              first_crossing(points,4),first_crossing(points,4.5),*points[-1][1:]))
    plt.rcParams.update({"font.size":8})
    render(runs,args.figure)


if __name__ == "__main__":
    main()
