#!/usr/bin/env python3
"""Make the calibration, bridge sequence, and supplementary evidence figures."""

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.ticker import MultipleLocator
import numpy as np

from audit_bridge_cart_scan import cart_scan_page
from audit_near_field_costmap import near_field_page
from audit_planner_costmaps import planner_snapshot_page
from bag_sqlite import bag_metadata
from plot_pilot_odometry import align_to_start, read_poses
from render_archive_figures import bag_frame, read_archive_frame


JINT = Path(__file__).resolve().parents[1]
EVIDENCE = JINT / "analysis/evidence"
FIGURES = JINT / "manuscript/figures"
PDF_METADATA = {"CreationDate": None, "ModDate": None}
BLUE, ORANGE = "#175676", "#a14c12"

# Match the title page in manuscript/main.tex.
SUPPLEMENT_TITLE = ("V-LiDAR: Monocular Virtual Scans for Reactive\n"
                    "Obstacle Avoidance in Structured Indoor Environments")
SUPPLEMENT_AUTHORS = "Hyunwoo Gu, Gunmin Yoo, Hyunseo Lee, Hyunmo Kang, Minseok Lee, and Sung Soo Hwang"
SUPPLEMENT_JOURNAL = "Journal of Intelligent & Robotic Systems"
SUPPLEMENT_AFFILIATION = ("School of Artificial Intelligence, Computer and Electrical Engineering,\n"
                          "Handong Global University, Pohang 37554, Republic of Korea")
CORRESPONDING_EMAIL = "sshwang@handong.edu"

ARCHIVE_FRAMES = (
    ("nav_debug_avoidance", 1789917441448965025, "(a) Office chair in the hall"),
    ("vlidar_run01.3", 1790854533842965842, "(b) Metal cart with thin legs"),
    ("vlidar_run01.4", 1790932912384048917, "(c) Box with window glare and reflections"),
    ("vlidar_run01.5", 1790933826258576115, "(d) Fixed dispenser in a narrow corridor"),
)


def visibility_figure():
    geometry = json.loads((EVIDENCE / "scan_lookup_geometry.json").read_text())
    height = geometry["analytic_check"]["assumed_height_m"]
    pitch = geometry["analytic_check"]["assumed_pitch_degrees"]
    center = geometry["centerline_near_forward_m"]
    angles = geometry["populated_laser_bin_span_degrees"]
    figure, (side, plan) = plt.subplots(1, 2, figsize=(5.25, 2.48), layout="constrained")

    side.axhline(0, color="0.25", lw=1)
    side.axvspan(0, center, ymin=0, ymax=0.126, facecolor="#f0dfce", edgecolor=ORANGE, hatch="///", lw=0.4)
    side.plot([0, center], [height, 0], color=BLUE, lw=1.4)
    side.plot([0, 4.6], [height, height - 4.6 * math.tan(math.radians(pitch))], "--", color="0.45", lw=1)
    side.scatter([0, center], [height, 0], color=BLUE, s=15, zorder=3)
    side.annotate(f"Optical axis ({pitch:g}°)", (2.65, 1.02), fontsize=8)
    side.annotate("Bottom-row ray", (.83, .65), xytext=(1.22, .69), fontsize=8, color=BLUE,
                  arrowprops={"arrowstyle": "-", "color": BLUE, "linewidth": .6})
    side.annotate(f"{center:.2f} m", (center, 0), xytext=(center + .1, .18), fontsize=8)
    side.text(.10, -.12, "Near-field gap", fontsize=7, color=ORANGE, va="center",
              bbox={"facecolor": "white", "edgecolor": "none", "alpha": .9, "pad": 1})
    side.set(xlim=(-.15, 4.6), ylim=(-.19, 1.35), xlabel="Forward ground distance (m)",
             ylabel="Height (m)", title="(a) Centerline cross-section")

    lower, upper = map(math.radians, angles)
    nominal_edge_y = 4.6 * math.tan(math.radians(35))
    active_edge_y = 4.6 * math.tan(upper)
    plan.plot([4.6, 0, 4.6], [-nominal_edge_y, 0, nominal_edge_y], ":", color="0.45", lw=1)
    plan.fill([center, 4.6, 4.6, center],
              [center * math.tan(lower), -active_edge_y, active_edge_y, center * math.tan(upper)],
              color="#deedf4", edgecolor=BLUE, lw=1)
    plan.plot([0, center], [0, center * math.tan(lower)], "--", color=BLUE, lw=.65)
    plan.plot([0, center], [0, center * math.tan(upper)], "--", color=BLUE, lw=.65)
    plan.scatter([0], [0], color=BLUE, s=12)
    plan.text(3.5, 0, f"{geometry['populated_bins']} bins\n±{angles[1]:g}°", ha="center", va="center", fontsize=8)
    plan.text(4.68, nominal_edge_y - .09, "35°", fontsize=7, ha="right", va="top")
    plan.set(xlim=(-.15, 4.75), ylim=(-3.4, 3.4), xlabel="Forward ground distance (m)",
             ylabel="Lateral distance (m)", title="(b) Nominal angular support")
    for axis in (side, plan):
        axis.title.set_fontsize(9)
        axis.xaxis.label.set_fontsize(8)
        axis.yaxis.label.set_fontsize(8)
        axis.tick_params(labelsize=7)
        axis.spines[["top", "right"]].set_visible(False)
    figure.savefig(FIGURES / "scan_visibility.pdf", metadata=PDF_METADATA)
    plt.close(figure)


def bridge_sequence(bags):
    first, second = bags / "nav_bridge_01", bags / "nav_bridge_02"
    figure = plt.figure(figsize=(5.7, 3.65), layout="constrained")
    grid = figure.add_gridspec(2, 3, height_ratios=(1, 1.0))
    path_axis = figure.add_subplot(grid[1, :])
    path_axis.axvline(10, color="0.45", linestyle=":", linewidth=1, label="Nominal goal x")
    for bag, color in ((first, "#72838e"), (second, BLUE)):
        poses = read_poses(bag)
        aligned = align_to_start(poses)
        forward, lateral = zip(*aligned)
        path_axis.plot(forward, lateral, color=color, lw=1.35, label=bag.name)
        path_axis.scatter(forward[-1], lateral[-1], s=10, color=color, zorder=3)

    for index, (seconds, phase) in enumerate(((10, "Approach"), (25, "Passing"), (35, "Past the bin"))):
        image, timestamp = bag_frame(second, seconds)
        axis = figure.add_subplot(grid[0, index])
        axis.imshow(image)
        axis.set_axis_off()
        label = chr(ord("a") + index)
        axis.set_title(f"({label}) {phase}", fontsize=9)
        nearest = min(range(len(poses)), key=lambda i: abs(poses[i][0] - timestamp))
        skew = abs(poses[nearest][0] - timestamp) / 1e6
        if skew > 100.0:
            raise ValueError(f"{second.name}: RGB/odom recorder separation {skew:.1f} ms at {seconds} s")
        print(f"Panel ({label}): nearest odometry recorder time is {skew:.1f} ms away")
        x, y = aligned[nearest]
        path_axis.scatter(x, y, s=21, facecolor="white", edgecolor=BLUE, zorder=4)
        path_axis.annotate(f"({label})", (x, y), xytext=(0, -14), textcoords="offset points",
                           ha="center", fontsize=8, color=BLUE)

    path_axis.set(xlabel="Forward displacement (m)", ylabel="Lateral displacement (m)",
                  ylim=(-.96, .56), xlim=(-.32, 10.55))
    path_axis.yaxis.set_major_locator(MultipleLocator(.5))
    path_axis.grid(color="0.90", linewidth=.55)
    path_axis.legend(loc="upper left", ncol=3, fontsize=8, frameon=False)
    figure.savefig(FIGURES / "bridge_pilot_odom.pdf", dpi=320, metadata=PDF_METADATA)
    plt.close(figure)


def bridge_feedback_figure(bag):
    with (EVIDENCE / "pilot_trace_summary.csv").open(newline="") as stream:
        trace = next(row for row in csv.DictReader(stream) if row["bag_id"] == bag.name)
    with (EVIDENCE / "pilot_nav_events.csv").open(newline="") as stream:
        preemption = next(float(row["bag_elapsed_s"]) for row in csv.DictReader(stream)
                          if row["bag_id"] == bag.name and row["message"] == "Received goal preemption request")

    frames = [bag_frame(bag, second) for second in (2.9, preemption, 14.0)]
    poses = read_poses(bag)
    start = bag_metadata(bag)["starting_time"]["nanoseconds_since_epoch"]
    elapsed = np.array([(stamp - start) / 1e9 for stamp, *_ in poses])
    selected = [min(range(len(poses)), key=lambda i: abs(poses[i][0] - stamp)) for _, stamp in frames]
    yaw = np.unwrap([row[3] for row in poses])
    relative_yaw = np.rad2deg(yaw - yaw[selected[0]])

    first, last = (cv2.cvtColor(np.asarray(frames[i][0]), cv2.COLOR_RGB2GRAY) for i in (0, -1))
    corners = cv2.goodFeaturesToTrack(first, maxCorners=250, qualityLevel=.01, minDistance=9)
    tracked, status, _ = cv2.calcOpticalFlowPyrLK(first, last, corners, None)
    returned, back_status, _ = cv2.calcOpticalFlowPyrLK(last, first, tracked, None)
    round_trip_error = np.linalg.norm(corners[:, 0] - returned[:, 0], axis=1)
    valid = (status[:, 0] == 1) & (back_status[:, 0] == 1) & (round_trip_error < 1)
    displacement = np.linalg.norm(corners[valid, 0] - tracked[valid, 0], axis=1)
    median = float(np.median(displacement))
    print(f"{bag.name}: {valid.sum()}/{len(corners)} tracked corners; median {median:.3f} px")
    for (_, stamp), index in zip(frames, selected):
        print(f"  RGB {stamp} ns; odometry {poses[index][0]} ns; relative yaw {relative_yaw[index]:.2f} deg")

    figure = plt.figure(figsize=(7.1, 4.9))
    grid = figure.add_gridspec(2, 3, height_ratios=(1, 1.18), left=.13, right=.97,
                              bottom=.22, top=.84, hspace=.45, wspace=.10)
    figure.suptitle("S4. Camera views during repeated wheel feedback", y=.97, fontsize=14)
    figure.text(.5, .895, "Fifth bridge pilot. Times are measured from the start of the recording.",
                ha="center", fontsize=9.5)
    for column, ((image, stamp), label) in enumerate(zip(frames, "abc")):
        image_axis = figure.add_subplot(grid[0, column])
        image_axis.imshow(image)
        image_axis.set_title(f"({label}) {(stamp - start) / 1e9:.2f} s", fontsize=9)
        image_axis.set_axis_off()

    axis = figure.add_subplot(grid[1, :])
    axis.axvspan(float(trace["repeated_turn_start_s"]), float(trace["repeated_turn_end_s"]),
                 color="#e8e0eb", label="Identical nonzero /odom twist")
    axis.axvline(preemption, color="0.42", ls="--", lw=1, label="Goal preemption")
    axis.plot(elapsed, relative_yaw, color=BLUE, lw=1.5)
    axis.scatter(elapsed[selected], relative_yaw[selected], s=20, color=BLUE, zorder=3)
    for index, label in zip(selected, "abc"):
        axis.annotate(f"({label})", (elapsed[index], relative_yaw[index]), xytext=(5, 3),
                      textcoords="offset points", color=BLUE, fontsize=9)
    axis.set(xlim=(0, 17.5), ylim=(-265, 85), xticks=(0, 4, 8, 12, 16),
             yticks=(0, -90, -180), xlabel="Time from recording start (s)",
             ylabel="Relative odom yaw (°)")
    axis.grid(axis="y", color=".90")
    axis.set_axisbelow(True)
    axis.legend(loc="upper right", fontsize=8, ncol=2)

    figure.text(.13, .080,
                f"Between (a) and (c), odometry yaw changes by {abs(relative_yaw[selected[-1]]):.0f}°.\n"
                f"Median image displacement is {median:.2f} px over {valid.sum()} tracked corners "
                f"({first.shape[1]} × {first.shape[0]} px frames).",
                va="top", fontsize=9)
    return figure


def supplementary_pages(archive, bags):
    metadata = {**PDF_METADATA, "Title": " ".join(SUPPLEMENT_TITLE.splitlines()),
                "Author": SUPPLEMENT_AUTHORS, "Subject": f"{SUPPLEMENT_JOURNAL} — Online Resource 1"}
    with PdfPages(EVIDENCE / "ESM_1.pdf", metadata=metadata) as pdf:
        figure, axes = plt.subplots(2, 2, figsize=(7.1, 7.65))
        figure.subplots_adjust(top=.600, bottom=.04, left=.04, right=.96, wspace=.08, hspace=.36)
        figure.text(.5, .97, f"{SUPPLEMENT_JOURNAL}  |  Online Resource 1",
                    ha="center", va="top", fontsize=9, color=BLUE)
        figure.text(.5, .932, SUPPLEMENT_TITLE, ha="center", va="top", fontsize=14, fontweight="bold")
        figure.text(.5, .862, SUPPLEMENT_AUTHORS, ha="center", va="top", fontsize=9)
        figure.text(.5, .834, SUPPLEMENT_AFFILIATION, ha="center", va="top", fontsize=8.7)
        figure.text(.5, .788, f"Correspondence: Sung Soo Hwang ({CORRESPONDING_EMAIL})",
                    ha="center", va="top", fontsize=8.7, color=BLUE)
        figure.add_artist(plt.Line2D((.08, .92), (.755, .755), transform=figure.transFigure,
                                    color="#a8bac4", linewidth=.6))
        figure.suptitle("S1. Recorded obstacles and environmental conditions", y=.731, fontsize=12)
        figure.text(.5, .683, "Selected archived RGB frames illustrate visual conditions.\n"
                    "They were acquired under different protocols from the formal evaluation.",
                    ha="center", va="top", fontsize=9)
        for axis, (name, timestamp, label) in zip(axes.flat, ARCHIVE_FRAMES):
            db, = sorted((archive / name).glob("*.db3"))
            axis.imshow(read_archive_frame(db, timestamp))
            axis.set_title(label, fontsize=10)
            axis.set_axis_off()
        pdf.savefig(figure, dpi=300)
        plt.close(figure)

        figure, axis = plt.subplots(figsize=(7.1, 4.9))
        figure.subplots_adjust(top=.78, bottom=.15, left=.15, right=.94)
        figure.suptitle("S2. CPU processing time across archived input frames", y=.97, fontsize=14)
        figure.text(.5, .865, "Two recordings, 198 frames per implementation after warm-up.\n"
                    "The horizontal scale is logarithmic. Frame measurements within a recording are correlated.",
                    ha="center", fontsize=10)
        runs = []
        for path in sorted(EVIDENCE.glob("pytorch_comparison_run*.csv")):
            with path.open(newline="") as stream:
                runs.extend(list(csv.DictReader(stream))[1:])
        for column, label, style in (
            ("vl_openvino_latency_ms", "V-LiDAR (OpenVINO FP16)", "-"),
            ("vl_latency_ms", "V-LiDAR (PyTorch)", "--"),
            ("midas_latency_ms", "MiDaS v2.1 Small", "-."),
            ("da_latency_ms", "Depth Anything V2 Small", ":"),
        ):
            values = np.sort([float(row[column]) for row in runs])
            assert len(values) == 198 and np.all(np.isfinite(values) & (values > 0))
            axis.step(values, np.arange(1, len(values) + 1) / len(values), where="post", label=label, ls=style)
        axis.set(xscale="log", ylim=(0, 1.05), xlabel="Processing time per RGB frame (ms)",
                 ylabel="Fraction of frames at or below time")
        axis.grid(color=".90")
        axis.legend(loc="lower right", fontsize=9)
        pdf.savefig(figure)
        plt.close(figure)

        runs = json.loads((JINT.parent / "paper/review/verification/robot_20261118/twenty_runs.json").read_text())["runs"]
        figure, (position, dwell) = plt.subplots(2, 1, sharex=True, figsize=(7.1, 5.4))
        figure.subplots_adjust(top=.78, bottom=.14, left=.14, right=.96, hspace=.20)
        figure.suptitle("S3. Odometry diagnostics from 20 archived runs", y=.97, fontsize=14)
        figure.text(.5, .865, "The September recordings have wheel odometry and scans.\n"
                    "Independent goal-arrival and contact labels are unavailable.", ha="center", fontsize=10)
        indices = [int(run["bag"].rsplit("run", 1)[1]) for run in runs]
        position.bar(indices, [run["end_x_m"] for run in runs], color=BLUE)
        position.set_ylabel("Terminal odom x (m)")
        dwell.bar(indices, [run["stationary_tail_s"] for run in runs], color=ORANGE)
        dwell.set(xlabel="Archived run number", ylabel="Stationary tail (s)", xticks=indices)
        for axis in (position, dwell):
            axis.grid(axis="y", color=".9")
            axis.set_axisbelow(True)
        pdf.savefig(figure)
        plt.close(figure)

        figure = bridge_feedback_figure(bags / "nav_bridge_05")
        pdf.savefig(figure, dpi=300)
        plt.close(figure)

        figure = planner_snapshot_page(bags, EVIDENCE)
        pdf.savefig(figure, dpi=300)
        plt.close(figure)

        figure = near_field_page(bags / "nav_bridge_02", EVIDENCE)
        pdf.savefig(figure, dpi=300)
        plt.close(figure)

        figure = cart_scan_page(bags / "nav_c_bridge_02", EVIDENCE)
        pdf.savefig(figure, dpi=320)
        plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", type=Path, default=Path.home() / "data/bags")
    parser.add_argument("--archive", type=Path, default=Path.home() / ".local/share/Trash/files")
    args = parser.parse_args()
    visibility_figure()
    bridge_sequence(args.bags)
    supplementary_pages(args.archive, args.bags)


if __name__ == "__main__":
    main()
