#!/usr/bin/env python3
"""Record the checkout used for JINT trials and fill field-log provenance."""

import argparse
import csv
from datetime import datetime, timezone
import fcntl
import hashlib
import io
import json
from pathlib import Path
import subprocess

from bag_sqlite import bag_metadata


ROOT = Path(__file__).resolve().parents[1]
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(required=True)

    capture = commands.add_parser("snapshot", help="hash available checkout artifacts")
    capture.add_argument("--output", type=Path, required=True)
    capture.add_argument("--backend", choices=MODEL_FILES, default="openvino")
    capture.add_argument("--device", default="CPU")
    capture.add_argument("--camera-calibration", type=Path)
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

    args = parser.parse_args()
    if hasattr(args, "trial_id") and not args.trial_id.strip():
        parser.error("trial ID cannot be blank")
    try:
        args.handler(args)
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        parser.exit(2, f"{error}\n")


if __name__ == "__main__":
    main()
