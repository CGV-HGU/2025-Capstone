#!/usr/bin/env python3
"""Render publication-grade candidate figures in JINT/manuscript/figures/candidates/.

Every plot uses verified repository artifacts (JSON/CSV/NPY) and avoids modifying
the main manuscript's active figures. Also provides --field-log to render
Condition x Layout Wilson-CI success-rate plots once formal trials are logged.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle, Wedge
import numpy as np

from summarize_trials import (
    CONDITION_LABELS,
    DEFAULT_CONDITION_IDS,
    DEFAULT_LAYOUT_IDS,
    LAYOUT_LABELS,
    read_main_trials,
    report,
)

ROOT = Path(__file__).resolve().parents[2]
JINT = ROOT / "JINT"
EVIDENCE = JINT / "analysis" / "evidence"
LOGS = ROOT / "experiments" / "logs"
PC_LOGS = LOGS / "pc_extracted"
SCRIPTS_DIR = ROOT / "src" / "freespace_detection" / "scripts"
CANDIDATES_DIR = JINT / "manuscript" / "figures" / "candidates"
PREVIEWS_DIR = CANDIDATES_DIR / "previews"
PDF_METADATA = {"CreationDate": None, "ModDate": None}

# Consistent Springer-ready palette aligned with render_evidence_figures.py / render_pipeline_figure.py
NAVY = "#182840"
BLUE = "#175676"
TEAL = "#1b7181"
ORANGE = "#a14c12"
AMBER = "#d98218"
SLATE = "#607585"
MUTED = "#72838e"
PLUM = "#6b4c7a"
LIGHT_BLUE = "#deedf4"
LIGHT_BG = "#f4f8fa"
LIGHT_ORANGE = "#f0dfce"

plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "axes.titlesize": 8.5,
        "axes.titleweight": "bold",
        "axes.edgecolor": SLATE,
        "axes.linewidth": 0.75,
        "grid.color": "0.90",
        "grid.linewidth": 0.55,
    }
)


def _save_candidate(fig: plt.Figure, stem: str, title: str, dpi: int = 330) -> None:
    CANDIDATES_DIR.mkdir(parents=True, exist_ok=True)
    PREVIEWS_DIR.mkdir(parents=True, exist_ok=True)
    pdf_path = CANDIDATES_DIR / f"{stem}.pdf"
    png_path = PREVIEWS_DIR / f"{stem}.png"
    fig.savefig(pdf_path, dpi=dpi, metadata={**PDF_METADATA, "Title": title})
    fig.savefig(png_path, dpi=220)
    plt.close(fig)
    disp_path = pdf_path.relative_to(ROOT) if pdf_path.is_relative_to(ROOT) else pdf_path
    print(f"Saved {disp_path} and preview")


def _style_axis(ax: plt.Axes, title_size: float = 8.6, label_size: float = 8.0, tick_size: float = 7.2) -> None:
    ax.title.set_fontsize(title_size)
    ax.title.set_color(NAVY)
    ax.xaxis.label.set_fontsize(label_size)
    ax.yaxis.label.set_fontsize(label_size)
    ax.xaxis.label.set_color(NAVY)
    ax.yaxis.label.set_color(NAVY)
    ax.tick_params(labelsize=tick_size, colors=NAVY)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


# ---------------------------------------------------------------------------
# Candidate 1: Evaluation protocol top-down layouts (S1 & S2, 10 m route)
# ---------------------------------------------------------------------------
def render_cand_01_protocol_layouts() -> None:
    geom = json.loads((EVIDENCE / "scan_lookup_geometry.json").read_text(encoding="utf-8"))
    near_m = float(geom["centerline_near_forward_m"])
    fov_deg = float(geom["populated_laser_bin_span_degrees"][1])

    fig, axes = plt.subplots(2, 1, figsize=(5.6, 3.4), sharex=True, sharey=True, layout="constrained")
    layouts = [
        ("(a) Layout S1: Single box", [(4.5, 0.0, 0.44, 0.44, "Box\n(4.5, 0.0)", ORANGE)]),
        (
            "(b) Layout S2: Staggered box + chair",
            [
                (3.5, 0.35, 0.44, 0.44, "Box\n(3.5, +0.35)", ORANGE),
                (7.0, -0.35, 0.50, 0.50, "Chair\n(7.0, -0.35)", PLUM),
            ],
        ),
    ]

    for ax, (title, obstacles) in zip(axes, layouts):
        ax.set_facecolor(LIGHT_BG)
        ax.axhline(0.0, color="0.72", ls="--", lw=0.8, zorder=1)
        ax.axvline(0.0, color="0.72", ls=":", lw=0.7, zorder=1)
        ax.axvline(10.0, color=TEAL, ls=":", lw=1.0, zorder=1)

        ang = np.linspace(-math.radians(fov_deg), math.radians(fov_deg), 60)
        poly_pts = np.vstack(
            [
                np.column_stack([near_m * np.cos(ang), near_m * np.sin(ang)]),
                np.column_stack([5.2 * np.ones_like(ang[::-1]), 5.2 * np.tan(ang[::-1])]),
            ]
        )
        ax.add_patch(Polygon(poly_pts, closed=True, facecolor=LIGHT_BLUE, edgecolor=BLUE, lw=0.8, alpha=0.6, zorder=2))
        blind = Wedge((0.0, 0.0), near_m, -fov_deg, fov_deg, facecolor=LIGHT_ORANGE, edgecolor=ORANGE,
                      hatch="///", lw=0.65, alpha=0.75, zorder=2)
        ax.add_patch(blind)
        ax.text(5.1, -1.25, f"FOV ±{fov_deg:g}°", ha="right", va="bottom", fontsize=6.5, color=BLUE)
        ax.text(1.12, 0.0, f"Blind\n<{near_m:.2f} m", ha="center", va="center", fontsize=6.6,
                color=ORANGE, fontweight="bold",
                bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.88, "pad": 1.0}, zorder=5)

        ax.add_patch(Rectangle((-0.25, -0.22), 0.50, 0.44, facecolor=NAVY, edgecolor="white", lw=0.8, zorder=4))
        ax.scatter([-0.55], [0.0], marker="^", s=26, color=AMBER, edgecolor=NAVY, lw=0.5, zorder=5)
        ax.annotate("lidar_link\n(-0.55 m)", (-0.55, 0.0), xytext=(-0.62, -0.95), fontsize=6.5, color=NAVY,
                    ha="center", arrowprops={"arrowstyle": "-", "color": SLATE, "lw": 0.6})
        ax.text(0.0, 0.52, "Start\nbase_link (0, 0)", fontsize=6.6, color=NAVY, ha="center", va="bottom")

        for ox, oy, ow, oh, label, col in obstacles:
            ax.add_patch(Rectangle((ox - ow / 2, oy - oh / 2), ow, oh, facecolor=col, edgecolor=NAVY,
                                   lw=0.8, alpha=0.88, zorder=4))
            y_text = oy + oh / 2 + 0.18 if oy >= 0 else oy - oh / 2 - 0.18
            ax.text(ox, y_text, label, ha="center", va="bottom" if oy >= 0 else "top",
                    fontsize=6.8, color=NAVY, fontweight="bold")

        goal_circ = plt.Circle((10.0, 0.0), 0.25, facecolor="#d6efe8", edgecolor=TEAL, lw=1.1, zorder=3)
        ax.add_patch(goal_circ)
        ax.scatter([10.0], [0.0], s=18, color=TEAL, zorder=4)
        ax.text(10.0, 0.42, "Goal (10.0, 0)\nr = 0.25 m", ha="center", va="bottom", fontsize=6.7, color=TEAL)

        ax.set_aspect("equal", adjustable="box")
        ax.set_xlim(-1.25, 11.15)
        ax.set_ylim(-1.48, 1.48)
        ax.set_yticks([-1.0, 0.0, 1.0])
        ax.grid(True, color="0.88", lw=0.45, zorder=0)
        _style_axis(ax)
        ax.set_title(title, loc="left", fontweight="bold", pad=4)

    axes[1].set_xlabel("Forward axis x (m)")
    axes[0].set_ylabel("Lateral y (m)")
    axes[1].set_ylabel("Lateral y (m)")
    _save_candidate(fig, "cand_01_protocol_layouts", "Proposed 10 m protocol layouts S1 and S2")


# ---------------------------------------------------------------------------
# Candidate 2: Camera mounting sensitivity & LUT cosine correction
# ---------------------------------------------------------------------------
def render_cand_02_mount_sensitivity_and_lut() -> None:
    geom = json.loads((EVIDENCE / "scan_lookup_geometry.json").read_text(encoding="utf-8"))
    intr = geom["scaled_intrinsics"]
    fy, cy = float(intr["fy"]), float(intr["cy"])
    H = int(geom["lookup_width_height"][1])
    phi_bottom = math.atan2((H - 1) - cy, fy)

    dist_1d = np.load(SCRIPTS_DIR / "distance_lut.npy")
    dist_2d = np.load(SCRIPTS_DIR / "distance_lut_2d.npy")
    rows = np.arange(H)
    valid = np.isfinite(dist_1d) & (dist_1d <= 6.5)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(5.7, 2.55), layout="constrained")

    gammas = np.linspace(0.5, 14.0, 200)
    heights = [(0.65, MUTED, ":"), (0.85, TEAL, "--"), (1.05, BLUE, "-"), (1.25, ORANGE, "-.")]
    for h_val, col, ls in heights:
        d_min = h_val / np.tan(phi_bottom + np.deg2rad(gammas))
        lw = 1.65 if abs(h_val - 1.05) < 1e-6 else 1.05
        ax1.plot(gammas, d_min, color=col, ls=ls, lw=lw, label=f"h = {h_val:.2f} m")

    dep_d = 1.05 / math.tan(phi_bottom + math.radians(2.0))
    ax1.scatter([2.0], [dep_d], color=ORANGE, edgecolor=NAVY, s=34, zorder=5)
    ax1.annotate(f"LUT config\n(2.0°, {dep_d:.2f} m)", (2.0, dep_d), xytext=(4.8, dep_d + 0.36),
                 fontsize=6.9, color=NAVY,
                 arrowprops={"arrowstyle": "-|>", "color": ORANGE, "lw": 0.8})
    ax1.set(xlim=(0.5, 14.0), ylim=(0.4, 3.2), xlabel="Downward pitch γ (deg)",
            ylabel="Centerline blind gap (m)")
    ax1.grid(True)
    ax1.legend(frameon=False, fontsize=6.7, loc="upper right")
    _style_axis(ax1)
    ax1.set_title("(a) Mounting sensitivity $D_{\\min}(h,\\gamma)$", loc="left", fontweight="bold")

    edge_left = dist_2d[valid, 0]
    center_x = dist_1d[valid]
    r_idx = rows[valid]
    ax2.plot(r_idx, center_x, color=BLUE, lw=1.5, label="Centerline $x(v)$")
    ax2.plot(r_idx, edge_left, color=ORANGE, ls="--", lw=1.4, label="Edge column $r(u_0, v)$")
    ax2.fill_between(r_idx, center_x, edge_left, color=LIGHT_ORANGE, alpha=0.55, label="Cosine correction")
    ax2.axhline(2.18, color=TEAL, ls=":", lw=0.8)
    ax2.annotate(f"v=255: {center_x[-1]:.2f} vs {edge_left[-1]:.2f} m\n(+{(edge_left[-1]/center_x[-1]-1)*100:.1f}% radial)",
                 (r_idx[-1], edge_left[-1]), xytext=(149, 2.75), fontsize=6.6, color=NAVY,
                 bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.75, "pad": 1.0},
                 arrowprops={"arrowstyle": "-|>", "color": ORANGE, "lw": 0.75})
    ax2.set(xlim=(145, 258), ylim=(2.0, 6.5), xlabel="Image row index v (px)",
            ylabel="Mapped ground range (m)")
    ax2.grid(True)
    ax2.legend(frameon=False, fontsize=6.7, loc="upper right")
    _style_axis(ax2)
    ax2.set_title("(b) Exact LUT row-to-range profile", loc="left", fontweight="bold")

    _save_candidate(fig, "cand_02_mount_sensitivity_and_lut", "Camera mounting sensitivity and exact LUT range profile")


# ---------------------------------------------------------------------------
# Candidate 3: YOLO11n-seg 100-epoch training & validation metrics
# ---------------------------------------------------------------------------
def render_cand_03_yolo_training_curves() -> None:
    csv_path = PC_LOGS / "yolo11n_seg_train_results.csv"
    epochs, tr_seg, va_seg, tr_box, va_box, p_m, r_m, map50, map5095 = ([] for _ in range(9))
    with csv_path.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            epochs.append(int(row["epoch"]))
            tr_seg.append(float(row["train/seg_loss"]))
            va_seg.append(float(row["val/seg_loss"]))
            tr_box.append(float(row["train/box_loss"]))
            va_box.append(float(row["val/box_loss"]))
            p_m.append(float(row["metrics/precision(M)"]))
            r_m.append(float(row["metrics/recall(M)"]))
            map50.append(float(row["metrics/mAP50(M)"]))
            map5095.append(float(row["metrics/mAP50-95(M)"]))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(5.8, 2.55), layout="constrained")

    ax1.plot(epochs, tr_seg, color=BLUE, lw=1.4, label="Train seg loss")
    ax1.plot(epochs, va_seg, color=ORANGE, lw=1.4, label="Val seg loss")
    ax1.plot(epochs, tr_box, color=TEAL, ls="--", lw=1.1, label="Train box loss")
    ax1.plot(epochs, va_box, color=MUTED, ls=":", lw=1.2, label="Val box loss")
    ax1.set(xlim=(1, 100), ylim=(0.5, 4.2), xlabel="Training epoch", ylabel="Loss")
    ax1.grid(True)
    ax1.legend(frameon=False, fontsize=6.7, loc="upper right")
    _style_axis(ax1)
    ax1.set_title("(a) Segmentation and box loss", loc="left", fontweight="bold")

    ax2.plot(epochs, map50, color=BLUE, lw=1.5, label=f"mAP@0.50 ({map50[-1]:.3f})")
    ax2.plot(epochs, map5095, color=ORANGE, lw=1.5, label=f"mAP@0.50:0.95 ({map5095[-1]:.3f})")
    ax2.plot(epochs, p_m, color=TEAL, ls="--", lw=1.05, label=f"Precision ({p_m[-1]:.3f})")
    ax2.plot(epochs, r_m, color=MUTED, ls=":", lw=1.15, label=f"Recall ({r_m[-1]:.3f})")
    ax2.scatter([100, 100], [map50[-1], map5095[-1]], color=[BLUE, ORANGE], s=18, zorder=5)
    ax2.set(xlim=(1, 102), ylim=(0.0, 1.02), xlabel="Training epoch",
            ylabel="Validation metric (2-class)")
    ax2.grid(True)
    ax2.legend(frameon=False, fontsize=6.4, loc="lower right")
    _style_axis(ax2)
    ax2.set_title("(b) Validation mask metrics", loc="left", fontweight="bold")

    _save_candidate(fig, "cand_03_yolo_training_curves", "YOLO11n-seg 100-epoch checkpoint curves")


# ---------------------------------------------------------------------------
# Candidate 4: Temporal filter ablation & pilot scan update cadence
# ---------------------------------------------------------------------------
def render_cand_04_temporal_filter_and_cadence() -> None:
    filt_rows = list(csv.DictReader((EVIDENCE / "temporal_filter_replay.csv").open(encoding="utf-8", newline="")))
    timing_rows = list(csv.DictReader((EVIDENCE / "scan_recorder_timing.csv").open(encoding="utf-8", newline="")))
    pilot_rows = [r for r in timing_rows if r["bag_id"].startswith("nav_")]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(5.8, 2.8), layout="constrained")

    clip_labels = ["Bridge precheck\n(13,835 pairs)", "Reflective hall\n(8,945 pairs)"]
    raw_cm = [float(r["raw_mean_abs_range_step_m"]) * 100.0 for r in filt_rows]
    fil_cm = [float(r["filtered_mean_abs_range_step_m"]) * 100.0 for r in filt_rows]
    raw_flips = [int(r["raw_validity_flips"]) for r in filt_rows]
    fil_flips = [int(r["filtered_validity_flips"]) for r in filt_rows]

    x = np.arange(len(clip_labels))
    w = 0.34
    ax1.bar(x - w / 2 - 0.03, raw_cm, w, color=LIGHT_ORANGE, edgecolor=ORANGE, lw=0.9, label="Bypass filter")
    ax1.bar(x + w / 2 + 0.03, fil_cm, w, color=BLUE, edgecolor=NAVY, lw=0.9, label="Temporal filter")
    for idx in range(len(clip_labels)):
        pct = 100.0 * (1.0 - fil_cm[idx] / raw_cm[idx])
        ax1.text(x[idx] - w / 2 - 0.04, raw_cm[idx] + 0.18, f"{raw_cm[idx]:.2f} cm\n({raw_flips[idx]} flips)",
                 ha="center", va="bottom", fontsize=6.1, color=ORANGE)
        ax1.text(x[idx] + w / 2 + 0.05, fil_cm[idx] + 0.18, f"{fil_cm[idx]:.2f} cm\n({fil_flips[idx]} flips,\n-{pct:.0f}%)",
                 ha="center", va="bottom", fontsize=6.1, color=NAVY)
    ax1.set_xticks(x, clip_labels)
    ax1.set_xlim(-0.58, 1.62)
    ax1.set_ylim(0.0, 8.2)
    ax1.set_ylabel("Mean adjacent range step (cm)")
    ax1.grid(axis="y")
    ax1.set_axisbelow(True)
    ax1.legend(frameon=False, fontsize=6.8, loc="upper left")
    _style_axis(ax1)
    ax1.set_title("(a) Fixed-scene filter ablation", loc="left", fontweight="bold")

    short_names = {
        "nav_test_11_26": "Hall test",
        "nav_trail_1_26": "Hall rerun",
        "nav_bridge_01": "Bridge 01",
        "nav_bridge_02": "Bridge 02",
        "nav_bridge_03": "Bridge 03",
        "nav_bridge_04": "Bridge 04",
        "nav_bridge_05": "Bridge 05",
    }
    labels_b = [short_names.get(r["bag_id"], r["bag_id"]) for r in pilot_rows]
    rates = [float(r["source_update_mean_hz"]) for r in pilot_rows]
    gap_p95 = [float(r["source_update_gap_p95_ms"]) for r in pilot_rows]
    y = np.arange(len(pilot_rows))
    colors = [ORANGE if r["bag_id"] == "nav_bridge_03" else BLUE for r in pilot_rows]

    ax2.barh(y, rates, height=0.56, color=colors, edgecolor=NAVY, lw=0.6)
    ax2.axvline(10.0, color=MUTED, ls="--", lw=0.9)
    ax2.text(10.15, -0.58, "10 Hz cap", fontsize=6.2, color=MUTED, va="bottom", ha="left")
    for yi, rate, gp in zip(y, rates, gap_p95):
        ax2.text(rate + 0.18, yi, f"{rate:.2f} Hz (p95 {gp:.0f} ms)", va="center", ha="left",
                 fontsize=6.3, color=ORANGE if gp > 200 else NAVY)
    ax2.set_yticks(y, labels_b)
    ax2.set_ylim(-0.85, len(pilot_rows) - 0.35)
    ax2.invert_yaxis()
    ax2.set_xlim(0.0, 15.2)
    ax2.set_xlabel("Mean scan update rate (Hz)")
    ax2.grid(axis="x")
    ax2.set_axisbelow(True)
    _style_axis(ax2)
    ax2.set_title("(b) Pilot scan cadence & p95 gaps", loc="left", fontweight="bold")

    _save_candidate(fig, "cand_04_temporal_filter_and_cadence", "Temporal filter ablation and pilot scan cadence")


# ---------------------------------------------------------------------------
# Candidate 5: CPU inference timing summary + multi-scene optical audit
# ---------------------------------------------------------------------------
def render_cand_05_cpu_latency_and_glare_audit() -> None:
    cpu_rows = list(csv.DictReader((EVIDENCE / "pytorch_comparison_summary.csv").open(encoding="utf-8", newline="")))
    audit_rows = list(csv.DictReader((LOGS / "glare_backlight_failure_audit.csv").open(encoding="utf-8", newline="")))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(5.85, 2.85), layout="constrained")

    order = [
        ("V-LiDAR (OpenVINO)", "V-LiDAR\nOpenVINO FP16", TEAL),
        ("V-LiDAR (PyTorch)", "V-LiDAR\nPyTorch", BLUE),
        ("MiDaS v2.1 Small (Baseline 1)", "MiDaS v2.1\nSmall", AMBER),
        ("Depth Anything V2 Small (Baseline 2)", "Depth Anything\nV2 Small", ORANGE),
    ]
    by_name = {r["name"]: r for r in cpu_rows}
    y = np.arange(len(order))
    for yi, (key, disp, col) in enumerate(order):
        r = by_name[key]
        mean_ms = float(r["steady_mean_latency_ms"])
        p50 = float(r["steady_p50_latency_ms"])
        p95 = float(r["steady_p95_latency_ms"])
        fps = float(r["steady_fps"])
        ax1.barh(yi, mean_ms, height=0.50, color=col, edgecolor=NAVY, lw=0.65, alpha=0.92)
        ax1.plot([p50, p95], [yi, yi], color=NAVY, lw=1.4, marker="|", markersize=6)
        if mean_ms < 200:
            ax1.text(p95 * 1.15, yi, f"{mean_ms:.1f} ms ({fps:.1f} Hz)", va="center", ha="left",
                     fontsize=6.5, color=NAVY, fontweight="bold")
        else:
            ax1.text(14.0, yi, f"{mean_ms:.1f} ms ({fps:.1f} Hz)", va="center", ha="left",
                     fontsize=6.5, color="white", fontweight="bold")

    ax1.axvline(33.3, color=SLATE, ls="--", lw=0.85)
    ax1.text(31.5, -0.45, "30 Hz\n(33 ms)", fontsize=6.0, color=SLATE, ha="right", va="bottom")
    ax1.axvline(100.0, color=MUTED, ls=":", lw=0.85)
    ax1.text(108.0, -0.45, "10 Hz\n(100 ms)", fontsize=6.0, color=MUTED, ha="left", va="bottom")
    ax1.set_xscale("log")
    ax1.set_xlim(5.0, 1200.0)
    ax1.set_ylim(-0.85, 3.5)
    ax1.set_yticks(y, [item[1] for item in order])
    ax1.invert_yaxis()
    ax1.set_xlabel("Steady-state CPU latency (ms, log)")
    ax1.grid(axis="x")
    ax1.set_axisbelow(True)
    _style_axis(ax1)
    ax1.set_title("(a) Onboard CPU latency", loc="left", fontweight="bold")

    scene_map = {
        "vlidar_run01": "Hall + box 1",
        "vlidar_run02": "Hall + box 2",
        "nav_debug_avoidance": "Swivel chair",
        "vlidar_run01.3": "Metal cart",
        "vlidar_run01.4": "Window glare",
        "vlidar_run01.5": "Narrow corridor",
    }
    ya = np.arange(len(audit_rows))
    roi_pct = [float(r["roi_clear_pct"]) for r in audit_rows]
    area_pct = [float(r["floor_area_pct_mean"]) for r in audit_rows]
    fp_drop = [int(r["upper_fp_dropped"]) for r in audit_rows]
    s_labels = [scene_map.get(r["bag"], r["bag"]) for r in audit_rows]

    h = 0.36
    ax2.barh(ya - h / 2, roi_pct, height=h, color=BLUE, edgecolor=NAVY, lw=0.5, label="ROI ≥95% pass (%)")
    ax2.barh(ya + h / 2, area_pct, height=h, color=LIGHT_BLUE, edgecolor=TEAL, lw=0.6, label="Mean floor area (%)")
    for yi, roi, fp in zip(ya, roi_pct, fp_drop):
        if fp >= 10:
            ax2.text(roi + 2.0, yi - h / 2, f"{fp} glare FPs\ndropped", va="center", ha="left",
                     fontsize=6.1, color=ORANGE, fontweight="bold")
        else:
            ax2.text(roi + 1.8, yi - h / 2, f"{roi:.0f}%", va="center", ha="left", fontsize=6.3, color=NAVY)
    ax2.set_yticks(ya, s_labels)
    ax2.invert_yaxis()
    ax2.set_xlim(0.0, 130.0)
    ax2.set_xlabel("Sampled frames (%)")
    ax2.grid(axis="x")
    ax2.set_axisbelow(True)
    ax2.legend(frameon=False, fontsize=6.3, loc="lower right")
    _style_axis(ax2)
    ax2.set_title("(b) Archived scene mask audit", loc="left", fontweight="bold")

    _save_candidate(fig, "cand_05_cpu_latency_and_glare_audit", "Onboard CPU latency and multi-scene optical audit")


# ---------------------------------------------------------------------------
# Candidate 6: Reflective-hall trajectories & frame-origin range sensitivity
# ---------------------------------------------------------------------------
def render_cand_06_reflective_hall_and_origin_sensitivity() -> None:
    traj1 = list(csv.DictReader((PC_LOGS / "trajectories" / "active_vlidar_run01.csv").open(encoding="utf-8", newline="")))
    traj2 = list(csv.DictReader((PC_LOGS / "trajectories" / "active_vlidar_run02.csv").open(encoding="utf-8", newline="")))
    cmp1 = list(csv.DictReader((EVIDENCE / "pytorch_comparison_run01.csv").open(encoding="utf-8", newline="")))
    cmp2 = list(csv.DictReader((EVIDENCE / "pytorch_comparison_run02.csv").open(encoding="utf-8", newline="")))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(5.85, 2.7), layout="constrained")

    for rows, col, label in ((traj1, BLUE, "vlidar_run01"), (traj2, ORANGE, "vlidar_run02")):
        xs = [float(r["x_m"]) for r in rows]
        ys = [float(r["y_m"]) for r in rows]
        ax1.plot(xs, ys, color=col, lw=1.45, label=label)
        ax1.scatter([xs[-1]], [ys[-1]], color=col, s=16, zorder=4)

    ax1.axvspan(4.4, 4.7, color=LIGHT_ORANGE, alpha=0.55, label="Nominal obs x (4.4–4.7 m)")
    ax1.scatter([4.4, 4.7], [0.0, 0.0], marker="s", s=34, color=[BLUE, ORANGE], edgecolor=NAVY, lw=0.6, zorder=5)
    ax1.axvline(9.77, color=MUTED, ls=":", lw=0.8)
    ax1.set(xlim=(-0.3, 10.3), ylim=(-1.08, 0.42), xlabel="Odometry x (m)", ylabel="Odometry y (m)")
    ax1.grid(True)
    ax1.legend(frameon=False, fontsize=6.3, loc="lower left")
    _style_axis(ax1)
    ax1.set_title("(a) Reflective-hall avoidance traces", loc="left", fontweight="bold")

    in_fov = [r for r in cmp1 + cmp2 if int(r["in_fov"]) == 1 and 2.18 <= float(r["gt_distance"]) <= 5.0]
    gt = np.array([float(r["gt_distance"]) for r in in_fov])
    vl = np.array([float(r["vl_openvino_dist"]) for r in in_fov])
    mi = np.array([float(r["midas_dist"]) for r in in_fov])
    da = np.array([float(r["da_dist"]) for r in in_fov])

    g_line = np.linspace(2.05, 4.95, 50)
    ax2.plot(g_line, g_line, color=NAVY, lw=1.1, ls="-", label="Identity (zero-offset)")
    ax2.fill_between(g_line, g_line, g_line + 0.55, color=LIGHT_BLUE, alpha=0.55,
                     label="Origin shift (+0.51–0.55 m)")
    ax2.scatter(gt, vl, s=14, color=BLUE, edgecolor="white", lw=0.3, label="V-LiDAR (MAE 0.152 m*)")
    ax2.scatter(gt, da, s=12, marker="^", color=TEAL, alpha=0.85, label="DA-V2 OOS (0.519 m*)")
    ax2.scatter(gt, mi, s=12, marker="x", color=ORANGE, alpha=0.85, label="MiDaS OOS (0.811 m*)")
    ax2.set(xlim=(2.05, 4.9), ylim=(2.05, 7.05),
            xlabel="Nominal unverified reference (m)",
            ylabel="Estimated range (m)")
    ax2.grid(True)
    ax2.legend(frameon=False, fontsize=5.8, loc="upper left", ncol=1)
    _style_axis(ax2)
    ax2.set_title("(b) Nominal reference comparison", loc="left", fontweight="bold")

    _save_candidate(fig, "cand_06_reflective_hall_and_origin_sensitivity",
                    "Reflective-hall trajectories and reference-origin sensitivity")


# ---------------------------------------------------------------------------
# Optional formal field-log plotter (for formal 4x2 trials)
# ---------------------------------------------------------------------------
def render_formal_outcomes_from_csv(field_log: Path, config: str | None = None) -> None:
    trials = read_main_trials(field_log)
    if not trials:
        raise ValueError(f"{field_log}: no main trials found")
    configs = sorted({k[0] for k in trials})
    if config is None:
        if len(configs) != 1:
            raise ValueError(f"specify --configuration among: {', '.join(configs)}")
        config = configs[0]

    fig, ax = plt.subplots(figsize=(5.6, 2.9), layout="constrained")
    x = np.arange(len(DEFAULT_CONDITION_IDS))
    w = 0.34
    for offset, layout_id, layout_label, col in (
        (-w / 2, DEFAULT_LAYOUT_IDS[0], LAYOUT_LABELS[0], BLUE),
        (+w / 2, DEFAULT_LAYOUT_IDS[1], LAYOUT_LABELS[1], ORANGE),
    ):
        rates, err_lo, err_hi, texts = [], [], [], []
        for cond_id in DEFAULT_CONDITION_IDS:
            counts = trials.get((config, cond_id, layout_id))
            if not counts:
                rates.append(0.0)
                err_lo.append(0.0)
                err_hi.append(0.0)
                texts.append("0/0")
                continue
            _, evaluated, successes, _, _, _, interval = report(counts)
            if not evaluated:
                rates.append(0.0)
                err_lo.append(0.0)
                err_hi.append(0.0)
                texts.append("0/0")
                continue
            lo_s, hi_s = interval.rstrip("%").split("-")
            p_pct = 100.0 * successes / evaluated
            rates.append(p_pct)
            err_lo.append(max(0.0, p_pct - float(lo_s)))
            err_hi.append(max(0.0, float(hi_s) - p_pct))
            texts.append(f"{successes}/{evaluated}")
        ax.bar(x + offset, rates, w, color=col, edgecolor=NAVY, lw=0.7, label=layout_label)
        ax.errorbar(x + offset, rates, yerr=[err_lo, err_hi], fmt="none", ecolor=NAVY, elinewidth=1.0, capsize=3)
        for xi, r_val, hi_val, txt in zip(x + offset, rates, err_hi, texts):
            ax.text(xi, min(108.0, r_val + hi_val + 2.5), txt, ha="center", va="bottom", fontsize=7.0, color=NAVY)

    ax.set_xticks(x, [lbl.replace(" (", "\n(") for lbl in CONDITION_LABELS])
    ax.set_ylim(0, 126)
    ax.set_ylabel("Physical success rate (%)")
    ax.grid(axis="y")
    ax.set_axisbelow(True)
    ax.legend(frameon=False, fontsize=7.5, loc="upper right", ncol=2)
    _style_axis(ax)
    ax.set_title(f"Formal navigation success rates with 95% Wilson intervals ({config})",
                 loc="left", fontweight="bold")
    _save_candidate(fig, "cand_formal_outcomes_wilson", "Formal navigation outcomes with 95% Wilson CI")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--field-log", type=Path, default=None,
                        help="Optional formal field_log.csv to also render cand_formal_outcomes_wilson.pdf")
    parser.add_argument("--configuration", default=None, help="Configuration ID for --field-log")
    args = parser.parse_args()

    render_cand_01_protocol_layouts()
    render_cand_02_mount_sensitivity_and_lut()
    render_cand_03_yolo_training_curves()
    render_cand_04_temporal_filter_and_cadence()
    render_cand_05_cpu_latency_and_glare_audit()
    render_cand_06_reflective_hall_and_origin_sensitivity()

    if args.field_log is not None:
        render_formal_outcomes_from_csv(args.field_log, args.configuration)


if __name__ == "__main__":
    main()
