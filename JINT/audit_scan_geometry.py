#!/usr/bin/env python3
"""Check the archived scan geometry against the checkout's LUT generator."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage
import yaml

from bag_sqlite import bag_messages, bag_metadata


ROOT = Path(__file__).resolve().parents[1]
LUT_DIR = ROOT / "src/freespace_detection/scripts"
LUT_FILES = ("col_to_ch_lut.npy", "distance_lut.npy", "distance_lut_2d.npy")


def audit_tables(calibration_path):
    with TemporaryDirectory(prefix="jint-lut-") as temporary:
        completed = subprocess.run(
            [sys.executable, str(LUT_DIR / "generate_luts.py")],
            cwd=temporary, check=True, capture_output=True, text=True
        )
        checks = {}
        for name in LUT_FILES:
            stored = (LUT_DIR / name).read_bytes()
            regenerated = (Path(temporary) / name).read_bytes()
            checks[name] = {
                "byte_identical": stored == regenerated,
                "sha256": hashlib.sha256(stored).hexdigest(),
            }
        if not all(check["byte_identical"] for check in checks.values()):
            raise ValueError("generated and stored lookup-table bytes differ")
    camera = yaml.safe_load(calibration_path.read_text(encoding="utf-8"))["Camera"]
    channel = np.load(LUT_DIR / LUT_FILES[0])
    forward = np.load(LUT_DIR / LUT_FILES[1])
    radial = np.load(LUT_DIR / LUT_FILES[2])
    height, width = radial.shape
    occupied = np.unique(channel)
    bin_degrees = -35.0 + 0.5 * occupied

    u, v = np.meshgrid(np.arange(width), np.arange(height))
    fx, cx = (round(camera[key] * width / camera["cols"], 4) for key in ("fx", "cx"))
    fy, cy = (round(camera[key] * height / camera["rows"], 4) for key in ("fy", "cy"))
    mount_height, pitch = 1.05, np.deg2rad(2.0)
    lateral = (u - cx) / fx
    down = (v - cy) / fy
    world_forward = np.cos(pitch) - down * np.sin(pitch)
    world_down = np.sin(pitch) + down * np.cos(pitch)
    with np.errstate(divide="ignore", invalid="ignore"):
        joint_projection = mount_height * np.hypot(world_forward, lateral) / world_down
    # Match the paper's evaluation interval. The reference is theoretical.
    valid = (v >= 120) & (world_down > 0) & (radial >= 2.18) & (radial <= 5.0)
    projection_diff = np.abs(radial[valid] - joint_projection[valid])
    yaw_diff = np.abs(np.rad2deg(np.arctan2(lateral[valid], world_forward[valid]) -
                                np.arctan(lateral[valid])))
    return {
        "generator_stdout": completed.stdout.splitlines(),
        "regenerated_assets": checks,
        "calibration_file": str(calibration_path),
        "calibration_image_width_height": [camera["cols"], camera["rows"]],
        "lookup_width_height": [width, height],
        "scaled_intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
        "unpopulated_nominal_bins": 141 - len(occupied),
        "populated_bins": len(occupied),
        "populated_laser_bin_span_degrees": [float(bin_degrees[0]), float(bin_degrees[-1])],
        "centerline_near_forward_m": float(forward[-1]),
        "edge_near_radial_m": [float(radial[-1, 0]), float(radial[-1, -1])],
        "analytic_check": {
            "assumed_height_m": mount_height,
            "assumed_pitch_degrees": float(np.rad2deg(pitch)),
            "lookup_range_interval_m": [2.18, 5.0],
            "samples": int(valid.sum()),
            "max_abs_radial_difference_m": float(projection_diff.max()),
            "median_abs_radial_difference_m": float(np.median(projection_diff)),
            "max_abs_unquantized_bearing_difference_deg": float(yaw_diff.max()),
        },
    }


def bag_frames(directory):
    info = bag_metadata(directory)
    start = info["starting_time"]["nanoseconds_since_epoch"]
    image_frame, scan_frame, width, height, scan_bins = "", "", "", "", ""
    transforms = {}
    topics = ("/tf", "/tf_static", "/camera/image_raw", "/scan")
    for _, topic, data in bag_messages(directory, info, topics, (start, start + 3_000_000_000)):
        if topic == "/camera/image_raw":
            if not width:
                message = deserialize_message(data, Image)
                image_frame = message.header.frame_id
                width, height = message.width, message.height
        elif topic == "/scan":
            if not scan_bins:
                message = deserialize_message(data, LaserScan)
                scan_frame = message.header.frame_id
                scan_bins = len(message.ranges)
        else:
            for tf in deserialize_message(data, TFMessage).transforms:
                if tf.header.frame_id == "base_link" and tf.child_frame_id in {"camera_link", "lidar_link"}:
                    t = tf.transform.translation
                    q = tf.transform.rotation
                    components = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
                    transforms.setdefault(tf.child_frame_id, set()).add(components)

    row = {"bag_id": directory.name, "sample_window_from_bag_start_s": 3,
           "rgb_frame_id": image_frame, "rgb_width_px": width, "rgb_height_px": height,
           "scan_frame_id": scan_frame, "scan_bins": scan_bins}
    for name in ("camera_link", "lidar_link"):
        values = transforms.get(name, set())
        row[f"{name}_tf_variants"] = len(values)
        row[f"{name}_tf_xyz_qxyzw"] = json.dumps(sorted(values), separators=(",", ":")) if values else ""
    return row


def reference_origin_audit(directory, frames):
    """Change the nominal coordinate convention without rescoring predictions."""
    environment = json.loads((directory / "pytorch_comparison_environment.json").read_text())
    records = []
    for index, obstacle_x in enumerate(environment["range_reference_positions_x_m"], 1):
        run = f"{index:02}"
        tf = next(row for row in frames if row["bag_id"] == f"vlidar_run{run}")
        transforms = json.loads(tf["lidar_link_tf_xyz_qxyzw"])
        if len(transforms) != 1 or transforms[0][3:] != [0.0, 0.0, 0.0, 1.0]:
            raise ValueError("expected one scan transform without relative rotation")
        tx, ty = transforms[0][:2]
        changes, retained = [], 0
        with (directory / f"pytorch_comparison_run{run}.csv").open(newline="") as stream:
            for row in csv.DictReader(stream):
                old_reference = float(row["gt_distance"])
                if row["in_fov"] != "1" or not 2.18 <= old_reference <= 5.0:
                    continue
                x, y = (float(row[key]) for key in ("robot_x", "robot_y"))
                yaw = math.radians(float(row["robot_yaw_deg"]))
                c, s = math.cos(yaw), math.sin(yaw)
                dx = obstacle_x - (x + c * tx - s * ty)
                dy = -(y + s * tx + c * ty)
                new_reference = math.hypot(dx, dy)
                bearing = math.degrees(math.remainder(math.atan2(dy, dx) - yaw, 2 * math.pi))
                changes.append(new_reference - old_reference)
                retained += 2.18 <= new_reference <= 5.0 and abs(bearing) <= 30.0
        records.append({
            "bag_id": tf["bag_id"],
            "nominal_samples": len(changes),
            "retained_samples_with_logged_scan_origin": retained,
            "min_reference_change_m": min(changes),
            "median_reference_change_m": float(np.median(changes)),
            "max_reference_change_m": max(changes),
        })
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--comparison-dir", type=Path)
    parser.add_argument("bags", type=Path, nargs="+")
    args = parser.parse_args()
    model = audit_tables(args.calibration)
    rows = [bag_frames(bag) for bag in args.bags]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "scan_lookup_geometry.json").write_text(
        json.dumps(model, indent=2) + "\n", encoding="utf-8"
    )
    outputs = {"scan_frame_transforms.csv": rows}
    if args.comparison_dir:
        outputs["scan_origin_reference_sensitivity.csv"] = reference_origin_audit(args.comparison_dir, rows)
    for filename, records in outputs.items():
        with (args.output_dir / filename).open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=records[0], lineterminator="\n")
            writer.writeheader()
            writer.writerows(records)


if __name__ == "__main__":
    main()
