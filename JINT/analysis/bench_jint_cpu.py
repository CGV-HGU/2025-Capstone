#!/usr/bin/env python3
"""Repeat the archived CPU comparison and preserve per-frame measurements."""

import argparse
import csv
from importlib.metadata import version
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "experiments"))
from benchmark_depth_baselines import (
    DepthAnythingV2Wrapper,
    MiDaSWrapper,
    VLiDARWrapper,
    apply_affine_disparity_calibration,
    fit_affine_disparity_calibration,
    run_rosbag_dynamic_benchmark,
    summarize_dynamic_records,
)


def calibrate_cross_session(first, second, disparity, distance):
    for source, target in ((first, second), (second, first)):
        calibration = fit_affine_disparity_calibration(source, disparity)
        if calibration is None:
            raise ValueError(f"cannot fit {disparity} calibration")
        apply_affine_disparity_calibration(target, disparity, distance, *calibration)


def write_rows(path, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def run_session(db, obstacle_x, frames, vl_torch, vl_ov, midas, depth):
    pytorch_rows = run_rosbag_dynamic_benchmark(
        str(db), vl_torch, midas, depth, obs_x=obstacle_x, max_frames=frames
    )
    deployment_rows = run_rosbag_dynamic_benchmark(
        str(db), vl_ov, None, None, obs_x=obstacle_x, max_frames=frames
    )
    if not pytorch_rows or not deployment_rows:
        raise ValueError(f"{db}: missing benchmark output")
    matched = zip(pytorch_rows, deployment_rows, strict=True)
    for row, deployment in matched:
        if row["timestamp"] != deployment["timestamp"]:
            raise ValueError(f"{db}: the backend replays selected different frames")
        row["vl_openvino_dist"] = deployment["vl_dist"]
        row["vl_openvino_latency_ms"] = deployment["vl_latency_ms"]
    return pytorch_rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("run01", type=Path)
    parser.add_argument("run02", type=Path)
    args = parser.parse_args()

    for bag in (args.run01, args.run02):
        if not bag.is_file() or bag.suffix != ".db3":
            parser.error(f"{bag}: supply the uncompressed SQLite recording")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    cv2.setNumThreads(args.threads)
    assets = str(ROOT / "src/freespace_detection/scripts")
    vl_torch = VLiDARWrapper(assets, device="cpu", use_openvino=False)
    vl_ov = VLiDARWrapper(assets, device="cpu", use_openvino=True)
    midas = MiDaSWrapper(device="cpu")
    depth = DepthAnythingV2Wrapper(device="cpu")
    if not (midas.is_loaded and depth.is_loaded):
        raise RuntimeError("both depth baselines must load before measuring")

    probe = np.zeros((480, 640, 3), dtype=np.uint8)
    metadata = {
        "pytorch_version": str(torch.__version__),
        "ultralytics_version": version("ultralytics"),
        "openvino_version": version("openvino"),
        "transformers_version": version("transformers"),
        "pytorch_cpu_threads": torch.get_num_threads(),
        "pytorch_interop_threads": torch.get_num_interop_threads(),
        "opencv_cpu_threads": cv2.getNumThreads(),
        "v_lidar_input_resize_hw": [vl_torch.target_h, vl_torch.target_w],
        "midas_model_input_hw_for_480x640": list(midas.transform(probe).shape[-2:]),
        "depth_anything_model_input_hw_for_480x640": list(
            depth.processor(images=probe, return_tensors="pt").pixel_values.shape[-2:]
        ),
        "recordings": [str(args.run01), str(args.run02)],
        "range_reference_positions_x_m": [4.4, 4.7],
    }

    first = run_session(args.run01, 4.4, 100, vl_torch, vl_ov, midas, depth)
    second = run_session(args.run02, 4.7, 100, vl_torch, vl_ov, midas, depth)
    v_lidar_probe = cv2.resize(probe, (vl_torch.target_w, vl_torch.target_h))
    metadata["v_lidar_pytorch_model_input_hw"] = list(
        vl_torch.model.predictor.preprocess([v_lidar_probe]).shape[-2:]
    )
    metadata["v_lidar_openvino_model_input_hw"] = list(
        vl_ov.model.predictor.preprocess([v_lidar_probe]).shape[-2:]
    )
    for name in ("midas", "da"):
        calibrate_cross_session(first, second, f"{name}_disp", f"{name}_dist")

    models = [
        ("V-LiDAR (PyTorch)", vl_torch, "vl_dist", "vl_latency_ms"),
        ("V-LiDAR (OpenVINO)", vl_ov, "vl_openvino_dist", "vl_openvino_latency_ms"),
        (midas.name, midas, "midas_dist", "midas_latency_ms"),
        (depth.name, depth, "da_dist", "da_latency_ms"),
    ]
    results = [
        summarize_dynamic_records(first + second, label, distance, timing, model.count_parameters())
        for label, model, distance, timing in models
    ]
    for row in results:
        row["valid_range_output_pct"] = row.pop("detection_rate_pct")
    write_rows(args.output_dir / "pytorch_comparison_run01.csv", first)
    write_rows(args.output_dir / "pytorch_comparison_run02.csv", second)
    write_rows(args.output_dir / "pytorch_comparison_summary.csv", results)
    (args.output_dir / "pytorch_comparison_environment.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    for row in results:
        print(row["name"], row["status"], row["steady_mean_latency_ms"], row["mae_m"])


if __name__ == "__main__":
    main()
