#!/usr/bin/env python3
"""
Dual Rosbag Baseline Benchmark Runner
Evaluates V-LiDAR vs MiDaS vs Depth Anything V2 across vlidar_run01 & vlidar_run02:
- Isolated per-model inference loops (eliminating cross-model CPU cache thrashing)
- Strict 2-fold out-of-sample affine disparity calibration for relative MDE baselines
- Ex-warmup steady-state latency and yaw/extrinsic-corrected ranging accuracy
"""

import argparse
import csv
import os
import sys

base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(base_dir, "experiments"))

from benchmark_depth_baselines import (
    DepthAnythingV2Wrapper,
    MiDaSWrapper,
    VLiDARWrapper,
    _fmt_num,
    apply_affine_disparity_calibration,
    export_latex_table,
    export_markdown_report,
    fit_affine_disparity_calibration,
    print_header,
    run_rosbag_dynamic_benchmark,
    summarize_dynamic_records,
)


def _rewrite_csv(path, records):
    if not records:
        return
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)


def main():
    parser = argparse.ArgumentParser(description="Dual-Rosbag Baseline Benchmark Runner")
    parser.add_argument("--use-openvino", action="store_true", default=True, help="Use OpenVINO FP16 for V-LiDAR")
    parser.add_argument("--no-openvino", action="store_false", dest="use_openvino", help="Use PyTorch .pt for V-LiDAR")
    parser.add_argument("--sensor-offset-x", type=float, default=0.0, help="Sensor X offset relative to odom origin (m)")
    parser.add_argument("--max-frames", type=int, default=100, help="Max sampled frames per bag")
    args = parser.parse_args()

    scripts_dir = os.path.join(base_dir, "src", "freespace_detection", "scripts")
    logs_dir = os.path.join(base_dir, "experiments", "logs")
    os.makedirs(logs_dir, exist_ok=True)

    print_header("Dual-Rosbag Baseline Benchmark: Isolated Loops & 2-Fold MDE Calibration")
    print("  • Target Hardware: On-board Embedded CPU (Intel Core Ultra 7 155H)")
    print("  • Datasets: vlidar_run01 & vlidar_run02")

    print("\n📦 [1/4] Initializing Perception Models...")
    vl_wrapper = VLiDARWrapper(scripts_dir, device="cpu", use_openvino=args.use_openvino)
    midas_wrapper = MiDaSWrapper(device="cpu")
    da_wrapper = DepthAnythingV2Wrapper(device="cpu")

    bag01 = os.path.expanduser("~/data/bags/vlidar_run01")
    bag02 = os.path.expanduser("~/data/bags/vlidar_run02")

    print("\n📂 [2/4] Running Isolated Benchmark Loops on vlidar_run01 (Obs X=4.4m)...")
    csv01 = os.path.join(logs_dir, "benchmark_run01.csv")
    records01 = run_rosbag_dynamic_benchmark(
        bag01,
        vl_wrapper,
        midas_wrapper,
        da_wrapper,
        obs_x=4.4,
        obs_y=0.0,
        max_frames=args.max_frames,
        output_csv=csv01,
        sensor_offset_x=args.sensor_offset_x,
    ) or []

    print("\n📂 [3/4] Running Isolated Benchmark Loops on vlidar_run02 (Obs X=4.7m)...")
    csv02 = os.path.join(logs_dir, "benchmark_run02.csv")
    records02 = run_rosbag_dynamic_benchmark(
        bag02,
        vl_wrapper,
        midas_wrapper,
        da_wrapper,
        obs_x=4.7,
        obs_y=0.0,
        max_frames=args.max_frames,
        output_csv=csv02,
        sensor_offset_x=args.sensor_offset_x,
    ) or []

    if not records01 or not records02:
        print("❌ Error: Missing records from one or both bags.")
        return

    # 2-fold out-of-sample affine disparity calibration:
    # Fit (s, t) strictly on session A -> apply to session B, and vice versa.
    for disp_key, dist_key, label in [
        ("midas_disp", "midas_dist", "MiDaS Small"),
        ("da_disp", "da_dist", "Depth Anything V2 Small"),
    ]:
        cal_1to2 = fit_affine_disparity_calibration(records01, disp_key)
        cal_2to1 = fit_affine_disparity_calibration(records02, disp_key)
        if cal_1to2:
            apply_affine_disparity_calibration(records02, disp_key, dist_key, *cal_1to2)
            print(f"  📐 {label} fit(run01->run02): s={cal_1to2[0]:.5f}, t={cal_1to2[1]:.5f}")
        if cal_2to1:
            apply_affine_disparity_calibration(records01, disp_key, dist_key, *cal_2to1)
            print(f"  📐 {label} fit(run02->run01): s={cal_2to1[0]:.5f}, t={cal_2to1[1]:.5f}")

    _rewrite_csv(csv01, records01)
    _rewrite_csv(csv02, records02)

    print("\n📊 [4/4] Aggregating Metrics Across Both Sessions...")
    all_records = records01 + records02
    results = [
        summarize_dynamic_records(all_records, vl_wrapper.name, "vl_dist", "vl_latency_ms", vl_wrapper.count_parameters()),
        summarize_dynamic_records(all_records, midas_wrapper.name, "midas_dist", "midas_latency_ms", midas_wrapper.count_parameters()),
        summarize_dynamic_records(all_records, da_wrapper.name, "da_dist", "da_latency_ms", da_wrapper.count_parameters()),
    ]

    latex_path = os.path.join(logs_dir, "baseline_latex_table.tex")
    md_path = os.path.join(logs_dir, "baseline_benchmark_report.md")
    export_latex_table(results, latex_path)
    export_markdown_report(results, md_path)

    print("\n" + "=" * 116)
    print("🏆 DUAL-ROSBAG BENCHMARK SUMMARY (Isolated Model Loops + 2-Fold OOS Calibrated MDE)")
    print("=" * 116)
    print(f"{'Model / Architecture':<38} | {'Params':<8} | {'Steady Lat (ms)':<16} | {'P95 (ms)':<8} | {'FPS':<6} | {'MAE (m)':<8} | {'Valid %':<8}")
    print("-" * 116)
    for r in results:
        lat_str = (
            f"{r['steady_mean_latency_ms']:.2f} ± {r['steady_std_latency_ms']:.2f}"
            if r.get("steady_mean_latency_ms") is not None
            else "not_measured"
        )
        print(
            f"{r['name']:<38} | {r['params_m']:<8.2f} | {lat_str:<16} | "
            f"{_fmt_num(r.get('steady_p95_latency_ms'), '.1f'):<8} | "
            f"{_fmt_num(r.get('steady_fps'), '.1f'):<6} | {_fmt_num(r['mae_m'], '.3f'):<8} | "
            f"{_fmt_num(r['detection_rate_pct'], '.1f'):<8}"
        )
    print("=" * 116)
    print(f"\n✅ Generated Artifacts:\n  • LaTeX Table : {latex_path}\n  • MD Report   : {md_path}\n  • Run 01 CSV  : {csv01}\n  • Run 02 CSV  : {csv02}")


if __name__ == "__main__":
    main()
