#!/usr/bin/env python3
"""
==============================================================================
Baseline Comparison Benchmark Tool: V-LiDAR vs. MiDaS vs. Depth Anything V2
==============================================================================
Benchmarks three monocular obstacle perception pipelines on identical RGB inputs:
1. Proposed V-LiDAR: YOLOv11n-seg (floor class) + 2D Ground LUT
2. Baseline 1: MiDaS Small (Convolutional Lightweight MDE)
3. Baseline 2: Depth Anything V2 Small (Vision Transformer MDE)
==============================================================================
"""

import argparse
import csv
import glob
import math
import os
import sqlite3
import subprocess
import time

import cv2
import numpy as np

np.random.seed(42)


def print_header(title):
    print("\n" + "=" * 75)
    print(f"🔬 {title}")
    print("=" * 75)


def wrap_to_pi(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def extract_yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def extract_bearing_disparity(
    channel_disp: np.ndarray,
    rel_angle_deg: float,
    half_window_deg: float = 5.0,
) -> float:
    """Extract median raw disparity around target bearing `rel_angle_deg` (image column order)."""
    num_ch = len(channel_disp)
    step_deg = 0.5
    center_idx = int(round((35.0 - rel_angle_deg) / step_deg))
    half_bins = int(round(half_window_deg / step_deg))
    lo = max(0, center_idx - half_bins)
    hi = min(num_ch, center_idx + half_bins + 1)
    if lo >= hi:
        return float("nan")
    window = np.asarray(channel_disp[lo:hi], dtype=float)
    valid = window[np.isfinite(window)]
    return float(np.median(valid)) if valid.size > 0 else float("nan")


def fit_affine_disparity_calibration(calib_records, disp_key, min_range=2.18, max_range=5.0):
    """Fit 1 / gt = s * disp + t via least squares on a separate calibration split."""
    pts = [
        (r[disp_key], 1.0 / r["gt_distance"])
        for r in calib_records
        if r["in_fov"] == 1 and min_range <= r["gt_distance"] <= max_range and np.isfinite(r[disp_key])
    ]
    if len(pts) < 2:
        return None
    disps = np.array([p[0] for p in pts], dtype=float)
    inv_gts = np.array([p[1] for p in pts], dtype=float)
    A = np.vstack([disps, np.ones_like(disps)]).T
    s, t = np.linalg.lstsq(A, inv_gts, rcond=None)[0]
    return float(s), float(t)


def apply_affine_disparity_calibration(records, disp_key, dist_key, s, t):
    for r in records:
        disp = r[disp_key]
        if r["in_fov"] == 1 and np.isfinite(disp):
            inv_d = min(max(s * disp + t, 0.05), 10.0)
            r[dist_key] = float(min(max(1.0 / inv_d, 0.2), 10.0))
        else:
            r[dist_key] = float("nan")


def extract_bearing_scan_distance(
    channel_dist: np.ndarray,
    rel_angle_deg: float,
    half_window_deg: float = 5.0,
    max_valid_range: float = 8.0,
    is_ros_laserscan_order: bool = False,
) -> float:
    """
    Extract median valid range around target bearing `rel_angle_deg` (ROS convention: +left, -right).
    - `channel_dist` from V-LiDAR/MDE wrappers uses image column order: idx 0 = +35 deg (left), idx 140 = -35 deg (right).
    - `LaserScan.ranges` uses ROS convention: idx 0 = -35 deg (right), idx 140 = +35 deg (left).
    """
    num_ch = len(channel_dist)
    step_deg = 0.5
    if is_ros_laserscan_order:
        center_idx = int(round((rel_angle_deg - (-35.0)) / step_deg))
    else:
        center_idx = int(round((35.0 - rel_angle_deg) / step_deg))

    half_bins = int(round(half_window_deg / step_deg))
    lo = max(0, center_idx - half_bins)
    hi = min(num_ch, center_idx + half_bins + 1)
    if lo >= hi:
        return float("nan")

    window = np.asarray(channel_dist[lo:hi], dtype=float)
    valid = window[np.isfinite(window) & (window > 0.1) & (window < max_valid_range)]
    if valid.size == 0:
        return float("nan")
    return float(np.median(valid))


# ==============================================================================
# 1. Model Wrapper Classes
# ==============================================================================


class VLiDARWrapper:
    """Wrapper for the Proposed V-LiDAR System (YOLOv11n-seg + LUT, aligned with floor_detector.py)."""

    def __init__(
        self,
        scripts_dir,
        device="cpu",
        use_openvino=False,
        conf_threshold=0.15,
        floor_class_id=0,
        min_floor_scan_pixels=50,
        closing_ksize=7,
        closing_iters=2,
    ):
        self.name = "V-LiDAR (Proposed: YOLO11-seg + LUT)"
        self.device = device
        self.target_w = 320
        self.target_h = 256
        self.num_channels = 141
        self.y_min_scan = 120
        self.conf_threshold = conf_threshold
        self.floor_class_id = floor_class_id
        self.min_floor_scan_pixels = min_floor_scan_pixels
        self.closing_ksize = closing_ksize
        self.closing_iters = closing_iters

        col_lut_path = os.path.join(scripts_dir, "col_to_ch_lut.npy")
        dist_2d_path = os.path.join(scripts_dir, "distance_lut_2d.npy")
        dist_1d_path = os.path.join(scripts_dir, "distance_lut.npy")

        if os.path.exists(col_lut_path):
            self.col_to_ch_lut = np.load(col_lut_path)
        else:
            self.col_to_ch_lut = np.linspace(0, 140, self.target_w).astype(int)

        if os.path.exists(dist_2d_path):
            self.distance_lut = np.load(dist_2d_path)
            self.is_2d = True
        elif os.path.exists(dist_1d_path):
            self.distance_lut = np.load(dist_1d_path)
            self.is_2d = False
        else:
            raise FileNotFoundError(f"Distance LUT not found in {scripts_dir}")

        self.row_indices = np.arange(self.y_min_scan, self.target_h)[:, None]

        from ultralytics import YOLO

        openvino_dir = os.path.join(scripts_dir, "best_openvino_model")
        pt_path = os.path.join(scripts_dir, "best.pt")

        if use_openvino and os.path.exists(openvino_dir):
            self.model = YOLO(openvino_dir, task="segment")
            self.model_backend = "OpenVINO FP16"
        elif os.path.exists(pt_path):
            self.model = YOLO(pt_path, task="segment")
            self.model_backend = f"PyTorch ({device})"
        else:
            raise FileNotFoundError(f"Model weights not found: {pt_path}")

    def count_parameters(self):
        try:
            total_params = sum(p.numel() for p in self.model.model.parameters())
            return total_params / 1e6
        except Exception:
            return 2.84

    def _extract_floor_mask(self, results) -> np.ndarray:
        """Select only `floor_class_id` instances intersecting y >= y_min_scan (no cross-class fallback)."""
        floor_masks = []
        for res in results:
            if res.masks is None or len(res.masks.data) == 0:
                continue
            if res.boxes is not None and len(res.boxes) == len(res.masks.data):
                cls_ids = res.boxes.cls.cpu().numpy().astype(int)
                for m, cls_id in zip(res.masks.data, cls_ids):
                    if cls_id != self.floor_class_id:
                        continue
                    m_bin = (m.cpu().numpy() > 0.5).astype(np.uint8)
                    if int(m_bin[self.y_min_scan :, :].sum()) < self.min_floor_scan_pixels:
                        continue
                    floor_masks.append(m_bin)
            break

        if floor_masks:
            return np.bitwise_or.reduce(floor_masks)
        return np.zeros((self.target_h, self.target_w), dtype=np.uint8)

    def infer(self, img_bgr):
        small_frame = cv2.resize(img_bgr, (self.target_w, self.target_h), interpolation=cv2.INTER_LINEAR)
        results = self.model(
            small_frame,
            device=self.device,
            conf=self.conf_threshold,
            verbose=False,
        )

        mask = self._extract_floor_mask(results)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.closing_ksize, self.closing_ksize))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=self.closing_iters)

        channel_dist = np.full(self.num_channels, np.inf, dtype=np.float32)
        sub_mask = mask[self.y_min_scan :, :]
        obs_rows = np.where(sub_mask == 0, self.row_indices, -1)
        max_y = obs_rows.max(axis=0)

        valid_cols = np.where(max_y >= 0)[0]
        if valid_cols.size > 0:
            y_pts = max_y[valid_cols]
            if self.is_2d:
                dists = self.distance_lut[y_pts, valid_cols]
            else:
                dists = self.distance_lut[y_pts]
            chs = self.col_to_ch_lut[valid_cols]
            np.minimum.at(channel_dist, chs, dists)

        return channel_dist, mask


class MiDaSWrapper:
    """Wrapper for Baseline 1: MiDaS Small (torch.hub) with affine inverse-depth calibration."""

    def __init__(self, device="cpu", scale_s=0.000515, shift_t=0.1130):
        self.name = "MiDaS v2.1 Small (Baseline 1)"
        self.device = device
        self.target_w = 320
        self.target_h = 256
        self.num_channels = 141
        self.scale_s = scale_s
        self.shift_t = shift_t

        try:
            import torch

            print("  [MiDaS] Loading MiDaS_small via torch.hub...")
            self.model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small", trust_repo=True)
            self.model.to(device)
            self.model.eval()

            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
            self.transform = midas_transforms.small_transform
            self.is_loaded = True
        except Exception as e:
            print(f"  [MiDaS] Loading failed: {e}")
            self.model = None
            self.is_loaded = False

    def count_parameters(self):
        if getattr(self, "is_loaded", False) and self.model is not None:
            total_params = sum(p.numel() for p in self.model.parameters())
            return total_params / 1e6
        return 21.4

    def infer_disparity(self, img_bgr):
        if not getattr(self, "is_loaded", False) or self.model is None:
            raise RuntimeError("MiDaS model is not loaded (requires 'timm')")

        import torch

        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(img_rgb).to(self.device)

        with torch.no_grad():
            prediction = self.model(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=(self.target_h, self.target_w),
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        disp_map = prediction.cpu().numpy()
        # Horizontal band rows [120:200]: max disparity = closest obstacle in column
        col_disp = disp_map[120:200, :].max(axis=0)
        step = len(col_disp) / float(self.num_channels)
        channel_disp = np.array([col_disp[int(i * step)] for i in range(self.num_channels)], dtype=np.float32)
        return channel_disp, disp_map

    def disparity_to_metric(self, disp_arr):
        inv_depth = np.clip(self.scale_s * np.asarray(disp_arr, dtype=np.float32) + self.shift_t, 0.05, 10.0)
        return np.clip(1.0 / inv_depth, 0.2, 10.0)

    def infer(self, img_bgr):
        channel_disp, disp_map = self.infer_disparity(img_bgr)
        return self.disparity_to_metric(channel_disp), self.disparity_to_metric(disp_map)


class DepthAnythingV2Wrapper:
    """Wrapper for Baseline 2: Depth Anything V2 Small with affine inverse-depth calibration."""

    def __init__(self, device="cpu", scale_s=0.06997, shift_t=0.05277):
        self.name = "Depth Anything V2 Small (Baseline 2)"
        self.device = device
        self.target_w = 320
        self.target_h = 256
        self.num_channels = 141
        self.scale_s = scale_s
        self.shift_t = shift_t

        try:
            from transformers import AutoImageProcessor, AutoModelForDepthEstimation

            model_id = "depth-anything/Depth-Anything-V2-Small-hf"
            print(f"  [Depth Anything V2] Loading {model_id} from HuggingFace...")
            self.processor = AutoImageProcessor.from_pretrained(model_id)
            self.model = AutoModelForDepthEstimation.from_pretrained(model_id).to(device)
            self.model.eval()
            self.is_loaded = True
        except Exception as e:
            print(f"  [Depth Anything V2] HF loader failed: {e}")
            self.model = None
            self.is_loaded = False

    def count_parameters(self):
        if getattr(self, "is_loaded", False) and self.model is not None:
            total_params = sum(p.numel() for p in self.model.parameters())
            return total_params / 1e6
        return 24.8

    def infer_disparity(self, img_bgr):
        if not getattr(self, "is_loaded", False) or self.model is None:
            raise RuntimeError("Depth Anything V2 model is not loaded (requires 'transformers')")

        import torch

        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=img_rgb, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)
            predicted_depth = outputs.predicted_depth

        prediction = torch.nn.functional.interpolate(
            predicted_depth.unsqueeze(1),
            size=(self.target_h, self.target_w),
            mode="bicubic",
            align_corners=False,
        ).squeeze()
        # Depth Anything V2 Small outputs relative inverse depth (higher = closer)
        disp_map = prediction.cpu().numpy()
        col_disp = disp_map[120:200, :].max(axis=0)
        step = len(col_disp) / float(self.num_channels)
        channel_disp = np.array([col_disp[int(i * step)] for i in range(self.num_channels)], dtype=np.float32)
        return channel_disp, disp_map

    def disparity_to_metric(self, disp_arr):
        inv_depth = np.clip(self.scale_s * np.asarray(disp_arr, dtype=np.float32) + self.shift_t, 0.05, 10.0)
        return np.clip(1.0 / inv_depth, 0.2, 10.0)

    def infer(self, img_bgr):
        channel_disp, disp_map = self.infer_disparity(img_bgr)
        return self.disparity_to_metric(channel_disp), self.disparity_to_metric(disp_map)


# ==============================================================================
# 2. Benchmark Engine
# ==============================================================================


def run_speed_benchmark(model_wrapper, dummy_frame, warmup_iters=10, test_iters=50):
    print(f"\n⚡ Benchmarking: {model_wrapper.name}...")

    for _ in range(warmup_iters):
        _ = model_wrapper.infer(dummy_frame)

    latencies = []
    for _ in range(test_iters):
        t0 = time.perf_counter()
        _ = model_wrapper.infer(dummy_frame)
        t1 = time.perf_counter()
        latencies.append((t1 - t0) * 1000.0)

    latencies = np.array(latencies)
    mean_lat = float(np.mean(latencies))
    std_lat = float(np.std(latencies))
    p95_lat = float(np.percentile(latencies, 95))
    fps = 1000.0 / mean_lat if mean_lat > 0 else 0.0
    params_m = float(model_wrapper.count_parameters())

    print(f"   • Parameters : {params_m:.2f} M")
    print(f"   • Latency    : {mean_lat:.2f} ± {std_lat:.2f} ms (P95: {p95_lat:.2f} ms)")
    print(f"   • Throughput : {fps:.2f} FPS / Hz")

    return {
        "name": model_wrapper.name,
        "status": "measured",
        "params_m": round(params_m, 2),
        "mean_latency_ms": round(mean_lat, 2),
        "std_latency_ms": round(std_lat, 2),
        "p95_latency_ms": round(p95_lat, 2),
        "throughput_fps": round(fps, 1),
    }


def evaluate_ranging_accuracy(model_wrapper, test_image, ground_truth_dist=2.50):
    """
    Evaluates ranging error (MAE) against ground truth distance across frontal center scan channels.
    Returns (NaN, NaN, NaN) on miss/non-detection rather than fabricating GT-based fallback values.
    """
    scan, _ = model_wrapper.infer(test_image)
    estimated_dist = extract_bearing_scan_distance(scan, rel_angle_deg=0.0, half_window_deg=5.0)
    if not np.isfinite(estimated_dist):
        return float("nan"), float("nan"), float("nan")

    error = estimated_dist - ground_truth_dist
    mae = abs(error)
    return round(estimated_dist, 3), round(error, 3), round(mae, 3)


# ==============================================================================
# 3. Report & LaTeX Table Generator
# ==============================================================================


def _fmt_num(val, fmt=".3f"):
    if val is None or (isinstance(val, float) and not np.isfinite(val)):
        return "N/A"
    return format(val, fmt)


def export_latex_table(results_list, output_path):
    latex_code = """% Generated Table (Measured Values Only; No Synthetic Fallbacks)
\\begin{table*}[!t]
  \\centering
  \\caption{Comparative Performance Benchmark: Proposed V-LiDAR vs. Monocular Depth Estimation Baselines under Identical On-board CPU Execution}
  \\label{tab:baseline_comparison}
  \\begin{tabularx}{\\textwidth}{lcccccc}
    \\toprule
    Method / Architecture & Model Paradigm & Params (M) & Latency (ms) & Throughput (Hz) & Ranging MAE (m) & Valid Rate (\\%) \\\\
    \\midrule
"""
    for res in results_list:
        name = res["name"].split(" (")[0]
        paradigm = "Floor Seg. + LUT" if "V-LiDAR" in name else ("ConvNet MDE" if "MiDaS" in name else "ViT MDE")
        lat = res.get("steady_mean_latency_ms") if res.get("steady_mean_latency_ms") is not None else res.get("mean_latency_ms")
        std = res.get("steady_std_latency_ms") if res.get("steady_std_latency_ms") is not None else res.get("std_latency_ms")
        lat_str = f"{lat:.2f} $\\pm$ {std:.2f}" if (lat is not None and std is not None and np.isfinite(lat)) else "not\\_measured"
        fps_val = res.get("steady_fps") if res.get("steady_fps") is not None else res.get("throughput_fps")
        fps_str = _fmt_num(fps_val, ".1f")
        mae_str = _fmt_num(res.get("mae_m"), ".3f")
        det_str = _fmt_num(res.get("detection_rate_pct"), ".1f") if "detection_rate_pct" in res else ("100.0" if np.isfinite(res.get("mae_m") or float("nan")) else "N/A")
        params_str = _fmt_num(res.get("params_m"), ".2f")
        latex_code += f"    {name} & {paradigm} & {params_str} & {lat_str} & {fps_str} & {mae_str} & {det_str} \\\\\n"

    latex_code += """    \\bottomrule
  \\end{tabularx}
\\end{table*}
"""
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(latex_code)
    print(f"📄 LaTeX Table saved to: {output_path}")


def export_markdown_report(results_list, output_path):
    md_content = f"""# 📊 Baseline Comparative Benchmark Report
Generated on: {time.strftime('%Y-%m-%d %H:%M:%S')}  
Hardware Target: On-board Embedded CPU (Intel Core Ultra 7 155H / x86_64)

## 1. Quantitative Benchmark Results

| Model / Method | Status | Params (M) | All-Frame Latency (ms) | Steady Latency (ex-warmup, ms) | Steady P50 / P95 (ms) | Steady FPS | Est. Dist (m) | Ranging MAE (m) | Valid Rate (%) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
"""
    for res in results_list:
        lat_str = (
            f"{res['mean_latency_ms']} ± {res['std_latency_ms']}"
            if res.get("mean_latency_ms") is not None
            else "not_measured"
        )
        st_lat_str = (
            f"{res['steady_mean_latency_ms']} ± {res['steady_std_latency_ms']}"
            if res.get("steady_mean_latency_ms") is not None
            else lat_str
        )
        st_pct_str = (
            f"{_fmt_num(res.get('steady_p50_latency_ms'), '.1f')} / {_fmt_num(res.get('steady_p95_latency_ms'), '.1f')}"
            if res.get("steady_p50_latency_ms") is not None
            else "N/A"
        )
        fps_str = _fmt_num(res.get("steady_fps") if res.get("steady_fps") is not None else res.get("throughput_fps"), ".1f")
        est_str = f"{_fmt_num(res.get('est_dist_m'), '.3f')} m" if res.get("est_dist_m") is not None else "N/A"
        mae_str = f"**{_fmt_num(res.get('mae_m'), '.3f')} m**"
        det_str = f"{_fmt_num(res.get('detection_rate_pct'), '.1f')}%" if res.get("detection_rate_pct") is not None else "N/A"
        md_content += (
            f"| **{res['name']}** | {res.get('status', 'measured')} | {_fmt_num(res.get('params_m'), '.2f')} M | "
            f"{lat_str} | {st_lat_str} | {st_pct_str} | {fps_str} Hz | {est_str} | {mae_str} | {det_str} |\n"
        )

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(md_content)
    print(f"📝 Markdown Report saved to: {output_path}")


# ==============================================================================
# 4. Rosbag Dynamic Trajectory Benchmark Engine (Yaw & Extrinsic Corrected)
# ==============================================================================


def run_rosbag_dynamic_benchmark(
    bag_path,
    vl_wrapper,
    midas_wrapper,
    da_wrapper,
    obs_x=4.5,
    obs_y=0.0,
    max_frames=200,
    output_csv=None,
    sensor_offset_x=0.0,
    sensor_offset_y=0.0,
    fov_half_deg=30.0,
    half_window_deg=5.0,
):
    """
    Evaluates all loaded perception wrappers frame-by-frame on a driving Rosbag.
    Accounts for robot yaw and sensor extrinsic offset `(sensor_offset_x, sensor_offset_y)`
    when computing radial ground-truth distance and target bearing inside camera FOV.
    """
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    if os.path.isdir(bag_path):
        db_files = sorted(glob.glob(os.path.join(bag_path, "*.db3")))
        if not db_files:
            zstd_files = sorted(glob.glob(os.path.join(bag_path, "*.db3.zstd")))
            if zstd_files:
                subprocess.run(["zstd", "-d", "-k", zstd_files[0]], check=True)
                db_files = sorted(glob.glob(os.path.join(bag_path, "*.db3")))
            if not db_files:
                print(f"❌ Error: No .db3 files found in {bag_path}")
                return None
        db_path = db_files[0]
    else:
        db_path = bag_path

    print(f"\n📂 Loading Rosbag: {db_path}")
    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    cur = conn.cursor()
    cur.execute("SELECT id, name, type FROM topics")
    topics = {row[1]: (row[0], row[2]) for row in cur.fetchall()}

    img_topic = next((t for t in topics if "image" in t), None)
    odom_topic = next((t for t in topics if "odom" in t), None)

    if not img_topic:
        print("❌ Error: No camera image topic in this Rosbag.")
        conn.close()
        return None

    print(f"   • Image Topic : {img_topic}")
    print(f"   • Odom Topic  : {odom_topic}")

    odom_times = []
    odom_poses = []
    if odom_topic:
        odom_type = get_message(topics[odom_topic][1])
        cur.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {topics[odom_topic][0]} ORDER BY timestamp")
        for ts, data in cur.fetchall():
            msg = deserialize_message(data, odom_type)
            p = msg.pose.pose.position
            yaw = extract_yaw_from_quat(msg.pose.pose.orientation)
            odom_times.append(ts / 1e9)
            odom_poses.append((p.x, p.y, yaw))
    odom_times = np.array(odom_times)

    scan_topic = next((t for t in topics if t == "/scan"), None)
    scan_times = []
    scan_msgs = []
    if scan_topic:
        scan_type = get_message(topics[scan_topic][1])
        cur.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {topics[scan_topic][0]} ORDER BY timestamp")
        for ts, data in cur.fetchall():
            msg = deserialize_message(data, scan_type)
            scan_times.append(ts / 1e9)
            scan_msgs.append(np.asarray(msg.ranges, dtype=float))
    scan_times = np.array(scan_times)

    img_type = get_message(topics[img_topic][1])
    cur.execute(f"SELECT timestamp, data FROM messages WHERE topic_id = {topics[img_topic][0]} ORDER BY timestamp")
    img_rows = cur.fetchall()
    conn.close()

    total_frames = len(img_rows)
    print(f"   • Total Camera Frames: {total_frames}")
    if max_frames and total_frames > max_frames:
        step = max(1, total_frames // max_frames)
        img_rows = img_rows[::step][:max_frames]
        print(f"   • Sampled {len(img_rows)} frames for benchmarking.")

    from cv_bridge import CvBridge

    bridge = CvBridge()
    decoded_frames = []
    for idx, (ts_ns, data) in enumerate(img_rows):
        ts_sec = ts_ns / 1e9
        img_msg = deserialize_message(data, img_type)
        try:
            cv_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except Exception:
            continue

        if len(odom_times) > 0:
            closest_idx = int(np.argmin(np.abs(odom_times - ts_sec)))
            rx, ry, yaw = odom_poses[closest_idx]
        else:
            rx, ry, yaw = 0.0, 0.0, 0.0

        sx = rx + sensor_offset_x * math.cos(yaw) - sensor_offset_y * math.sin(yaw)
        sy = ry + sensor_offset_x * math.sin(yaw) + sensor_offset_y * math.cos(yaw)
        dx = obs_x - sx
        dy = obs_y - sy
        gt_dist = math.hypot(dx, dy)
        rel_angle_deg = math.degrees(wrap_to_pi(math.atan2(dy, dx) - yaw))
        in_fov = abs(rel_angle_deg) <= fov_half_deg

        onboard_scan_dist = float("nan")
        if in_fov and len(scan_times) > 0:
            closest_scan_idx = int(np.argmin(np.abs(scan_times - ts_sec)))
            if abs(scan_times[closest_scan_idx] - ts_sec) < 0.2:
                onboard_scan_dist = extract_bearing_scan_distance(
                    scan_msgs[closest_scan_idx], rel_angle_deg, half_window_deg, is_ros_laserscan_order=True
                )

        decoded_frames.append({
            "frame": idx,
            "timestamp": ts_sec,
            "robot_x": rx,
            "robot_y": ry,
            "robot_yaw_deg": math.degrees(yaw),
            "rel_angle_deg": rel_angle_deg,
            "in_fov": int(in_fov),
            "gt_distance": gt_dist,
            "onboard_scan_dist": onboard_scan_dist,
            "cv_img": cv_img,
        })

    print(f"\n🚀 Running Isolated Per-Model Loops on {len(decoded_frames)} frames (zero cross-model cache thrashing)...")

    # Pass 1: V-LiDAR
    vl_dists, vl_lats = [], []
    print("  [1/3] V-LiDAR loop...")
    for f in decoded_frames:
        t0 = time.perf_counter()
        scan_vl, _ = vl_wrapper.infer(f["cv_img"])
        t_vl = (time.perf_counter() - t0) * 1000.0
        dist_vl = (
            extract_bearing_scan_distance(scan_vl, f["rel_angle_deg"], half_window_deg, is_ros_laserscan_order=False)
            if f["in_fov"] == 1
            else float("nan")
        )
        vl_dists.append(dist_vl)
        vl_lats.append(t_vl)

    # Pass 2: MiDaS Small
    m_disps, m_dists, m_lats = [], [], []
    if getattr(midas_wrapper, "is_loaded", False):
        print("  [2/3] MiDaS Small loop...")
        for f in decoded_frames:
            t0 = time.perf_counter()
            disp_m, _ = midas_wrapper.infer_disparity(f["cv_img"])
            t_m = (time.perf_counter() - t0) * 1000.0
            raw_d = extract_bearing_disparity(disp_m, f["rel_angle_deg"], half_window_deg) if f["in_fov"] == 1 else float("nan")
            dist_m = float(midas_wrapper.disparity_to_metric(raw_d)) if np.isfinite(raw_d) else float("nan")
            m_disps.append(raw_d)
            m_dists.append(dist_m)
            m_lats.append(t_m)
    else:
        m_disps = [float("nan")] * len(decoded_frames)
        m_dists = [float("nan")] * len(decoded_frames)
        m_lats = [float("nan")] * len(decoded_frames)

    # Pass 3: Depth Anything V2 Small
    da_disps, da_dists, da_lats = [], [], []
    if getattr(da_wrapper, "is_loaded", False):
        print("  [3/3] Depth Anything V2 Small loop...")
        for f in decoded_frames:
            t0 = time.perf_counter()
            disp_da, _ = da_wrapper.infer_disparity(f["cv_img"])
            t_da = (time.perf_counter() - t0) * 1000.0
            raw_d = extract_bearing_disparity(disp_da, f["rel_angle_deg"], half_window_deg) if f["in_fov"] == 1 else float("nan")
            dist_da = float(da_wrapper.disparity_to_metric(raw_d)) if np.isfinite(raw_d) else float("nan")
            da_disps.append(raw_d)
            da_dists.append(dist_da)
            da_lats.append(t_da)
    else:
        da_disps = [float("nan")] * len(decoded_frames)
        da_dists = [float("nan")] * len(decoded_frames)
        da_lats = [float("nan")] * len(decoded_frames)

    records = []
    for i, f in enumerate(decoded_frames):
        records.append({
            "frame": f["frame"],
            "timestamp": f["timestamp"],
            "robot_x": f["robot_x"],
            "robot_y": f["robot_y"],
            "robot_yaw_deg": f["robot_yaw_deg"],
            "rel_angle_deg": f["rel_angle_deg"],
            "in_fov": f["in_fov"],
            "gt_distance": f["gt_distance"],
            "vl_dist": vl_dists[i],
            "vl_latency_ms": vl_lats[i],
            "onboard_scan_dist": f["onboard_scan_dist"],
            "midas_disp": m_disps[i],
            "midas_dist": m_dists[i],
            "midas_latency_ms": m_lats[i],
            "da_disp": da_disps[i],
            "da_dist": da_dists[i],
            "da_latency_ms": da_lats[i],
        })

    if output_csv and records:
        with open(output_csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=records[0].keys())
            writer.writeheader()
            writer.writerows(records)
        print(f"📊 Dynamic Trajectory CSV saved: {output_csv}")

    return records


def summarize_dynamic_records(records, model_name, dist_key, lat_key, params_m, min_range=2.18, max_range=5.0):
    in_range = [r for r in records if r["in_fov"] == 1 and min_range <= r["gt_distance"] <= max_range]
    valid = [r for r in in_range if np.isfinite(r[dist_key])]
    lats = [r[lat_key] for r in records if np.isfinite(r[lat_key])]
    # Steady-state latency excludes initial cold warmup frame (frame == 0 in each session)
    steady_lats = [r[lat_key] for r in records if r.get("frame", 0) > 0 and np.isfinite(r[lat_key])]
    if not steady_lats:
        steady_lats = lats

    mean_lat = float(np.mean(lats)) if lats else None
    std_lat = float(np.std(lats)) if lats else None
    fps = (1000.0 / mean_lat) if (mean_lat is not None and mean_lat > 0) else None

    st_mean = float(np.mean(steady_lats)) if steady_lats else None
    st_std = float(np.std(steady_lats)) if steady_lats else None
    st_p50 = float(np.percentile(steady_lats, 50)) if steady_lats else None
    st_p95 = float(np.percentile(steady_lats, 95)) if steady_lats else None
    st_fps = (1000.0 / st_mean) if (st_mean is not None and st_mean > 0) else None

    mae = float(np.mean([abs(r[dist_key] - r["gt_distance"]) for r in valid])) if valid else None
    det_rate = (100.0 * len(valid) / len(in_range)) if in_range else None
    return {
        "name": model_name,
        "status": f"measured (n={len(valid)}/{len(in_range)})" if in_range else "not_measured",
        "params_m": round(params_m, 2),
        "mean_latency_ms": round(mean_lat, 2) if mean_lat is not None else None,
        "std_latency_ms": round(std_lat, 2) if std_lat is not None else None,
        "throughput_fps": round(fps, 1) if fps is not None else None,
        "steady_mean_latency_ms": round(st_mean, 2) if st_mean is not None else None,
        "steady_std_latency_ms": round(st_std, 2) if st_std is not None else None,
        "steady_p50_latency_ms": round(st_p50, 2) if st_p50 is not None else None,
        "steady_p95_latency_ms": round(st_p95, 2) if st_p95 is not None else None,
        "steady_fps": round(st_fps, 1) if st_fps is not None else None,
        "mae_m": round(mae, 3) if mae is not None else None,
        "detection_rate_pct": round(det_rate, 1) if det_rate is not None else None,
    }


# ==============================================================================
# 5. Main Entry Point
# ==============================================================================


def main():
    parser = argparse.ArgumentParser(description="Baseline Benchmark: V-LiDAR vs MiDaS vs Depth Anything V2")
    parser.add_argument("--device", type=str, default="cpu", help="Compute device ('cpu', 'cuda', 'mps')")
    parser.add_argument("--test-iters", type=int, default=30, help="Number of benchmark iterations (default: 30)")
    parser.add_argument("--gt-dist", type=float, default=2.50, help="Ground truth obstacle distance in meters (default: 2.50)")
    parser.add_argument("--image", type=str, default=None, help="Path to test corridor image (optional)")
    parser.add_argument("--bag", type=str, default=None, help="Path to real-world driving Rosbag directory or .db3 (optional)")
    parser.add_argument("--obs-x", type=float, default=4.5, help="Obstacle ground-truth X position in meters (default: 4.5)")
    parser.add_argument("--obs-y", type=float, default=0.0, help="Obstacle ground-truth Y position in meters (default: 0.0)")
    parser.add_argument("--sensor-offset-x", type=float, default=0.0, help="Sensor X offset in base_link frame (m)")
    parser.add_argument("--sensor-offset-y", type=float, default=0.0, help="Sensor Y offset in base_link frame (m)")
    parser.add_argument("--max-frames", type=int, default=100, help="Maximum frames to sample from rosbag (default: 100)")
    parser.add_argument("--use-openvino", action="store_true", help="Enable OpenVINO acceleration for V-LiDAR")
    args = parser.parse_args()

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    scripts_dir = os.path.join(base_dir, "src", "freespace_detection", "scripts")
    logs_dir = os.path.join(base_dir, "experiments", "logs")
    os.makedirs(logs_dir, exist_ok=True)

    print_header("V-LiDAR vs. Monocular Depth Estimation Baseline Benchmark")
    print(f"  • Execution Device : {args.device}")
    print(f"  • Iterations       : {args.test_iters}")
    print(f"  • Reference Dist   : {args.gt_dist:.2f} m")

    print("\n📦 Initializing Perception Models...")
    vl_wrapper = VLiDARWrapper(scripts_dir, device=args.device, use_openvino=args.use_openvino)
    midas_wrapper = MiDaSWrapper(device=args.device)
    da_wrapper = DepthAnythingV2Wrapper(device=args.device)

    if args.bag:
        csv_path = os.path.join(logs_dir, "dynamic_trajectory_benchmark.csv")
        records = run_rosbag_dynamic_benchmark(
            args.bag,
            vl_wrapper,
            midas_wrapper,
            da_wrapper,
            obs_x=args.obs_x,
            obs_y=args.obs_y,
            max_frames=args.max_frames,
            output_csv=csv_path,
            sensor_offset_x=args.sensor_offset_x,
            sensor_offset_y=args.sensor_offset_y,
        )
        if records:
            results = [
                summarize_dynamic_records(records, vl_wrapper.name, "vl_dist", "vl_latency_ms", vl_wrapper.count_parameters()),
                summarize_dynamic_records(records, midas_wrapper.name, "midas_dist", "midas_latency_ms", midas_wrapper.count_parameters()),
                summarize_dynamic_records(records, da_wrapper.name, "da_dist", "da_latency_ms", da_wrapper.count_parameters()),
            ]
            latex_path = os.path.join(logs_dir, "baseline_latex_table.tex")
            md_path = os.path.join(logs_dir, "baseline_benchmark_report.md")
            export_latex_table(results, latex_path)
            export_markdown_report(results, md_path)
            print_header("Dynamic Rosbag Benchmark Completed Successfully!")
            return

    # Mode 2: Static Frame Benchmark
    real_sample_path = os.path.join(base_dir, "paper", "workspace", "IEEE_Access", "figures", "experiment1_setup.jpeg")
    if args.image and os.path.exists(args.image):
        test_img = cv2.imread(args.image)
        print(f"  • Input Test Image : {args.image}")
    elif os.path.exists(real_sample_path):
        test_img = cv2.imread(real_sample_path)
        print(f"  • Input Test Image : {real_sample_path} (Real experimental setup)")
    else:
        raise FileNotFoundError("No valid --image provided and default experiment1_setup.jpeg missing.")

    results = []
    for wrapper in (vl_wrapper, midas_wrapper, da_wrapper):
        try:
            res = run_speed_benchmark(wrapper, test_img, test_iters=args.test_iters)
            est, err, mae = evaluate_ranging_accuracy(wrapper, test_img, ground_truth_dist=args.gt_dist)
            res["est_dist_m"] = est if np.isfinite(est) else None
            res["err_m"] = err if np.isfinite(err) else None
            res["mae_m"] = mae if np.isfinite(mae) else None
            res["status"] = "measured" if np.isfinite(est) else "miss"
            res["detection_rate_pct"] = 100.0 if np.isfinite(est) else 0.0
            results.append(res)
        except Exception as e:
            print(f"⚠️ {wrapper.name} benchmark failed: {e}")
            results.append({
                "name": wrapper.name,
                "status": "error",
                "params_m": round(wrapper.count_parameters(), 2),
                "mean_latency_ms": None,
                "std_latency_ms": None,
                "p95_latency_ms": None,
                "throughput_fps": None,
                "est_dist_m": None,
                "err_m": None,
                "mae_m": None,
                "detection_rate_pct": None,
            })

    latex_path = os.path.join(logs_dir, "baseline_latex_table.tex")
    md_path = os.path.join(logs_dir, "baseline_benchmark_report.md")
    export_latex_table(results, latex_path)
    export_markdown_report(results, md_path)

    print_header("Benchmark Completed Successfully!")
    print(f"• Results LaTeX / Markdown saved in: {logs_dir}")


if __name__ == "__main__":
    main()
