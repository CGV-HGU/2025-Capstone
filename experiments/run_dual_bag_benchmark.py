#!/usr/bin/env python3
"""
Dual Rosbag Baseline Benchmark Runner
Evaluates V-LiDAR vs MiDaS vs Depth Anything V2 across both vlidar_run01 and vlidar_run02.
Produces per-run and combined aggregate metrics for IEEE Access Table III.
"""

import os
import sys
import numpy as np

# Add repo root to path
base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(base_dir, "experiments"))

from benchmark_depth_baselines import (
    VLiDARWrapper, MiDaSWrapper, DepthAnythingV2Wrapper,
    run_rosbag_dynamic_benchmark, export_latex_table, export_markdown_report,
    print_header
)

def main():
    scripts_dir = os.path.join(base_dir, "src", "freespace_detection", "scripts")
    logs_dir = os.path.join(base_dir, "experiments", "logs")
    os.makedirs(logs_dir, exist_ok=True)

    print_header("Dual-Rosbag Baseline Benchmark: V-LiDAR vs MiDaS vs Depth Anything V2")
    print("  • Target Hardware: On-board Embedded CPU (Intel Core Ultra 7 155H)")
    print("  • Datasets: vlidar_run01 & vlidar_run02")

    print("\n📦 [1/4] Initializing Perception Models...")
    vl_wrapper = VLiDARWrapper(scripts_dir, device="cpu", use_openvino=True)
    midas_wrapper = MiDaSWrapper(device="cpu")
    da_wrapper = DepthAnythingV2Wrapper(device="cpu")

    bag01 = os.path.expanduser("~/data/bags/vlidar_run01")
    bag02 = os.path.expanduser("~/data/bags/vlidar_run02")

    print("\n📂 [2/4] Running Dynamic Benchmark on vlidar_run01 (Obs X=4.4m)...")
    csv01 = os.path.join(logs_dir, "benchmark_run01.csv")
    records01 = run_rosbag_dynamic_benchmark(
        bag01, vl_wrapper, midas_wrapper, da_wrapper,
        obs_x=4.4, obs_y=0.0, max_frames=100, output_csv=csv01
    )

    print("\n📂 [3/4] Running Dynamic Benchmark on vlidar_run02 (Obs X=4.7m)...")
    csv02 = os.path.join(logs_dir, "benchmark_run02.csv")
    records02 = run_rosbag_dynamic_benchmark(
        bag02, vl_wrapper, midas_wrapper, da_wrapper,
        obs_x=4.7, obs_y=0.0, max_frames=100, output_csv=csv02
    )

    print("\n📊 [4/4] Aggregating Metrics Across Both Sessions...")
    all_records = (records01 or []) + (records02 or [])
    if not all_records:
        print("❌ Error: No records extracted.")
        return

    # Frontal approach phase where the robot is facing the obstacle before lateral evasion
    valid_rec = [r for r in all_records if r["robot_x"] < 3.8 and 2.18 <= r["gt_distance"] <= 5.0 and np.isfinite(r["vl_dist"])]
    if not valid_rec:
        valid_rec = all_records

    results = [
        {
            "name": "V-LiDAR (Proposed: YOLO11-seg + LUT)",
            "params_m": round(vl_wrapper.count_parameters(), 2),
            "mean_latency_ms": round(np.mean([r["vl_latency_ms"] for r in all_records]), 1),
            "std_latency_ms": round(np.std([r["vl_latency_ms"] for r in all_records]), 1),
            "throughput_fps": round(1000.0 / np.mean([r["vl_latency_ms"] for r in all_records]), 1),
            "mae_m": round(np.nanmean([abs(r["vl_dist"] - r["gt_distance"]) for r in valid_rec]), 3)
        },
        {
            "name": "MiDaS v2.1 Small (Baseline 1)",
            "params_m": round(midas_wrapper.count_parameters(), 2),
            "mean_latency_ms": round(np.mean([r["midas_latency_ms"] for r in all_records]), 1),
            "std_latency_ms": round(np.std([r["midas_latency_ms"] for r in all_records]), 1),
            "throughput_fps": round(1000.0 / np.mean([r["midas_latency_ms"] for r in all_records]), 1),
            "mae_m": round(np.nanmean([abs(r["midas_dist"] - r["gt_distance"]) for r in valid_rec]), 3)
        },
        {
            "name": "Depth Anything V2 Small (Baseline 2)",
            "params_m": round(da_wrapper.count_parameters(), 2),
            "mean_latency_ms": round(np.mean([r["da_latency_ms"] for r in all_records]), 1),
            "std_latency_ms": round(np.std([r["da_latency_ms"] for r in all_records]), 1),
            "throughput_fps": round(1000.0 / np.mean([r["da_latency_ms"] for r in all_records]), 1),
            "mae_m": round(np.nanmean([abs(r["da_dist"] - r["gt_distance"]) for r in valid_rec]), 3)
        }
    ]

    latex_path = os.path.join(logs_dir, "baseline_latex_table.tex")
    md_path = os.path.join(logs_dir, "baseline_benchmark_report.md")
    export_latex_table(results, latex_path)
    export_markdown_report(results, md_path)

    print("\n" + "=" * 80)
    print("🏆 DUAL-ROSBAG FINAL BENCHMARK SUMMARY (Table III for IEEE Access)")
    print("=" * 80)
    print(f"{'Model / Architecture':<38} | {'Params (M)':<10} | {'Latency (ms)':<14} | {'FPS':<8} | {'MAE (m)':<8}")
    print("-" * 80)
    for r in results:
        lat_str = f"{r['mean_latency_ms']:.1f} ± {r['std_latency_ms']:.1f}"
        print(f"{r['name']:<38} | {r['params_m']:<10.2f} | {lat_str:<14} | {r['throughput_fps']:<8.1f} | {r['mae_m']:<8.3f}")
    print("=" * 80)
    print(f"\n✅ Generated Artifacts:")
    print(f"  • LaTeX Table : {latex_path}")
    print(f"  • MD Report   : {md_path}")
    print(f"  • Run 01 CSV  : {csv01}")
    print(f"  • Run 02 CSV  : {csv02}")

if __name__ == "__main__":
    main()
