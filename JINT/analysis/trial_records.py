#!/usr/bin/env python3
"""Record the checkout used for JINT trials and fill field-log provenance."""

import argparse
import csv
from datetime import datetime, timezone
import fcntl
import hashlib
from importlib.util import find_spec
import io
import json
from pathlib import Path
import sqlite3
import subprocess

from bag_sqlite import bag_metadata, first_topic_payloads


ROOT = Path(__file__).resolve().parents[2]
FIELD_TEMPLATE = ROOT / "paper/review/planning/templates/field_log.csv"
FSD_SCRIPTS = "src/freespace_detection/scripts"
LUT_FILES = [
    f"{FSD_SCRIPTS}/col_to_ch_lut.npy",
    f"{FSD_SCRIPTS}/distance_lut_2d.npy",
    f"{FSD_SCRIPTS}/distance_lut.npy",
]
TRIAL_FILES = [
    "start_all.sh",
    "scripts/run_trial.sh",
    "scripts/record_rosbag.sh",
    "scripts/reset_trial.sh",
    "scripts/test_avoidance_goal.py",
    "src/robot_launch_package/launch/multi_node_launch.py",
    "src/omo_r1_bringup/launch/omo_r1_bringup.launch.py",
    "src/omo_r1_bringup/omo_r1_bringup/scripts/omo_r1_mcu_node.py",
    "src/omo_r1_navigation2/param/omo_r1.yaml",
    "src/freespace_detection/launch/freespace_detection_launch.py",
    "src/freespace_detection/freespace_detection/floor_detector.py",
    "src/fake_lidar_with_tf/fake_lidar_with_tf/fake_lidar_with_tf.py",
]
MODEL_FILES = {
    "openvino": [f"{FSD_SCRIPTS}/best_openvino_model/best.bin",
                 f"{FSD_SCRIPTS}/best_openvino_model/best.xml"],
    "pytorch": [f"{FSD_SCRIPTS}/best.pt"],
}


def file_digest(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def installed_artifacts():
    """Resolve the files the sourced ROS overlay would use on a fresh launch."""
    from ament_index_python.packages import get_package_prefix, get_package_share_directory

    share_files = {
        "robot_launch_package": ("launch/multi_node_launch.py",),
        "omo_r1_bringup": ("launch/omo_r1_bringup.launch.py", "launch/omo_r1_mcu.launch.py",
                           "param/omo_r1_mcu.yaml"),
        "omo_r1_navigation2": ("launch/navigation2.launch.py", "param/omo_r1.yaml"),
        "freespace_detection": ("launch/freespace_detection_launch.py",),
    }
    pairs = {}
    for package, names in share_files.items():
        share = Path(get_package_share_directory(package))
        pairs.update({f"src/{package}/{name}": share / name for name in names})
    modules = {
        "omo_r1_bringup.scripts.omo_r1_mcu_node": "src/omo_r1_bringup/omo_r1_bringup/scripts/omo_r1_mcu_node.py",
        "omo_r1_bringup.scripts.omo_packet_handler": "src/omo_r1_bringup/omo_r1_bringup/scripts/omo_packet_handler.py",
        "freespace_detection.floor_detector": "src/freespace_detection/freespace_detection/floor_detector.py",
        "fake_lidar_with_tf.fake_lidar_with_tf": "src/fake_lidar_with_tf/fake_lidar_with_tf/fake_lidar_with_tf.py",
    }
    for module, source in modules.items():
        spec = find_spec(module)
        if spec is None or spec.origin is None:
            raise ValueError(f"Cannot resolve {module} in the sourced ROS setup")
        pairs[source] = Path(spec.origin)

    artifacts = {}
    for source, installed in pairs.items():
        checksum = file_digest(installed)
        artifacts[source] = {"lookup_path": str(installed), "resolved_path": str(installed.resolve()),
                             "sha256": checksum, "matches_checkout": checksum == file_digest(ROOT / source)}
    pose_converter = Path(get_package_prefix("pose_converter")) / "lib/pose_converter/pose_converter_node"
    artifacts["pose_converter executable"] = {"lookup_path": str(pose_converter),
                                               "resolved_path": str(pose_converter.resolve()),
                                               "sha256": file_digest(pose_converter),
                                               "checkout_source_sha256": file_digest(
                                                   ROOT / "src/pose_converter/src/pose_converter_node.cpp")}
    return artifacts


def snapshot(args):
    model_files = MODEL_FILES[args.backend]
    paths = TRIAL_FILES + LUT_FILES + model_files
    artifacts = {name: file_digest(ROOT / name) for name in paths}
    if args.camera_calibration:
        artifacts["camera_calibration"] = file_digest(args.camera_calibration)

    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    status = subprocess.check_output(["git", "status", "--short", "--", *paths],
                                     cwd=ROOT, text=True).splitlines()
    definition = {
        "code_commit": commit,
        "intended_backend": args.backend,
        "intended_device": args.device,
        "camera_calibration_path": str(args.camera_calibration) if args.camera_calibration else None,
        "file_sha256": artifacts,
        "model_sha256": artifacts[model_files[0]],
        "lut_sha256_manifest": {name: artifacts[name] for name in LUT_FILES},
    }
    if args.installed:
        definition["installed_files"] = installed_artifacts()
    canonical = json.dumps(definition, ensure_ascii=False, sort_keys=True).encode()
    manifest = {
        "configuration_id": hashlib.sha256(canonical).hexdigest()[:16],
        "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "relevant_git_status": status,
        **definition,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(f"{manifest['configuration_id']} {args.output}")


def append_record(filename, row):
    with FIELD_TEMPLATE.open(newline="", encoding="utf-8") as stream:
        columns = next(csv.reader(stream))
    filename.parent.mkdir(parents=True, exist_ok=True)
    with filename.open("a+", encoding="utf-8", newline="") as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        stream.seek(0)
        contents = stream.read()
        reader = csv.DictReader(io.StringIO(contents.removeprefix("\ufeff"), newline=""))
        if reader.fieldnames:
            if reader.fieldnames != columns:
                raise ValueError(f"{filename}: unexpected field log header")
            if any(previous["record_id"] == row["record_id"] for previous in reader):
                raise ValueError(f"{filename}: record {row['record_id']} already exists")
            if not contents.endswith(("\n", "\r")):
                stream.write("\n")
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        if not reader.fieldnames:
            writer.writeheader()
        writer.writerow(row)
        stream.flush()


def add_trial(args):
    manifest = json.loads(args.configuration.read_text(encoding="utf-8")) if args.configuration else {}
    result = json.loads(args.result.read_text(encoding="utf-8")) if args.result else {}
    if result.get("trial_id") not in (None, args.trial_id):
        raise ValueError(f"{args.result}: JSON trial_id differs from {args.trial_id}")
    # Explicit paths allow remaining historical JSON/bag pairs with blank trial_id.
    start = ""
    notes = [f"outcome JSON {args.result}"] if args.result else []
    if result.get("goal_end_event"):
        notes.append(f"terminal event {result['goal_end_event']}")
    accepted_ns = result.get("goal_accepted_time_ns")
    end_ns = result.get("result_or_cancel_time_ns")
    elapsed_s = ""
    if type(accepted_ns) is int and type(end_ns) is int and 0 < accepted_ns <= end_ns:
        elapsed_s = round((end_ns - accepted_ns) / 1e9, 6)
    if args.bag:
        if not args.bag.is_dir():
            raise FileNotFoundError(f"{args.bag}: bag directory not found")
        if (args.bag / "metadata.yaml").is_file():
            info = bag_metadata(args.bag)
            start_ns = info["starting_time"]["nanoseconds_since_epoch"]
            start = datetime.fromtimestamp(start_ns / 1e9, timezone.utc).astimezone().isoformat(timespec="seconds")
        else:
            notes.append("bag metadata missing")
    row = {
        "record_id": args.trial_id,
        "phase": args.phase,
        "environment_id": args.environment,
        "scenario_id": args.layout,
        "start_time_iso8601_with_offset": start,
        "configuration_id": manifest.get("configuration_id", ""),
        "code_commit": manifest.get("code_commit", ""),
        "model_sha256": manifest.get("model_sha256", ""),
        "lut_sha256_manifest": str(args.configuration) if args.configuration else "",
        "goal_accepted_time_ns": accepted_ns,
        "result_or_cancel_time_ns": end_ns,
        "elapsed_s": elapsed_s,
        "nav2_status": result.get("nav_status", ""),
        "bag_asset_id": args.bag.name if args.bag else "",
        "notes": "; ".join(notes),
    }
    append_record(args.field_log, row)
    print(f"{args.trial_id} {args.field_log}")


def check_recording(args):
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, LaserScan

    messages = {"/camera/image_raw": Image, "/scan": LaserScan, "/odom": Odometry}
    types = {"/camera/image_raw": "sensor_msgs/msg/Image", "/scan": "sensor_msgs/msg/LaserScan",
             "/odom": "nav_msgs/msg/Odometry"}
    info = bag_metadata(args.bag)
    topics = {entry["topic_metadata"]["name"]: entry for entry in info["topics_with_message_count"]}
    for topic, expected in types.items():
        entry = topics.get(topic, {})
        metadata = entry.get("topic_metadata", {})
        if not entry.get("message_count") or metadata.get("type") != expected or metadata.get("serialization_format") != "cdr":
            raise ValueError(f"{args.bag}: expected recorded CDR messages of type {expected} on {topic}")

    samples = first_topic_payloads(args.bag, info, messages)
    for topic, message in messages.items():
        if topic not in samples:
            raise ValueError(f"{args.bag}: {topic} appears in metadata but has no readable database rows")
        try:
            decoded = deserialize_message(samples[topic], message)
        except (TypeError, ValueError, RuntimeError) as error:
            raise ValueError(f"{args.bag}: cannot decode {topic}: {error}") from error
        if isinstance(decoded, Image):
            if not (decoded.width and decoded.height and decoded.step) or len(decoded.data) != decoded.step * decoded.height:
                raise ValueError(f"{args.bag}: incomplete image data on {topic}")
        if isinstance(decoded, LaserScan) and not decoded.ranges:
            raise ValueError(f"{args.bag}: empty range array on {topic}")

    report = {"bag": args.bag.name,
              "duration_seconds": info["duration"]["nanoseconds"] / 1e9,
              "decoded_samples": list(messages),
              "recorded_topic_counts": {topic: topics.get(topic, {}).get("message_count", 0)
                                        for topic in (*messages, "/cmd_vel", "/tf", "/plan",
                                                      "/local_plan", "/local_costmap/costmap")}}
    if args.result:
        result = json.loads(args.result.read_text(encoding="utf-8"))
        if args.trial_id and result.get("trial_id") != args.trial_id:
            raise ValueError(f"{args.result}: JSON trial_id differs from {args.trial_id}")
        report["outcome_record"] = {key: result.get(key) for key in ("trial_id", "nav_status", "goal_end_event")}
    print(json.dumps(report, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(required=True)

    capture = commands.add_parser("snapshot", help="hash checkout and optionally installed artifacts")
    capture.add_argument("--output", type=Path, required=True)
    capture.add_argument("--backend", choices=MODEL_FILES, default="openvino")
    capture.add_argument("--device", default="CPU")
    capture.add_argument("--camera-calibration", type=Path)
    capture.add_argument("--installed", action="store_true",
                         help="also hash files resolved from the currently sourced ROS workspace")
    capture.set_defaults(handler=snapshot)

    trial = commands.add_parser("add", help="add bag and outcome provenance to the field CSV")
    trial.add_argument("--field-log", type=Path, required=True)
    trial.add_argument("--trial-id", required=True)
    trial.add_argument("--phase", choices=("pilot", "main"), required=True)
    trial.add_argument("--environment", required=True)
    trial.add_argument("--layout", required=True)
    trial.add_argument("--configuration", type=Path)
    trial.add_argument("--bag", type=Path)
    trial.add_argument("--result", type=Path)
    trial.set_defaults(handler=add_trial)

    check = commands.add_parser("check", help="read and decode representative samples from a completed ROS 2 bag")
    check.add_argument("--bag", type=Path, required=True)
    check.add_argument("--result", type=Path)
    check.add_argument("--trial-id", help="require the supplied result JSON to match this trial ID")
    check.set_defaults(handler=check_recording)

    args = parser.parse_args()
    if getattr(args, "trial_id", None) is not None and not args.trial_id.strip():
        parser.error("trial ID cannot be blank")
    if args.handler is check_recording and args.trial_id and not args.result:
        parser.error("--trial-id requires --result for check")
    try:
        args.handler(args)
    except (ImportError, OSError, ValueError, KeyError, sqlite3.Error, subprocess.CalledProcessError) as error:
        parser.exit(2, f"{error}\n")


if __name__ == "__main__":
    main()
