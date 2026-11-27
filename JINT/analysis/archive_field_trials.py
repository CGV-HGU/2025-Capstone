#!/usr/bin/env python3
"""Copy a condition's completed bags and field records into a verified archive."""

import argparse
import csv
import fcntl
import json
from pathlib import Path
import shutil
from tempfile import TemporaryDirectory

from bag_sqlite import bag_metadata
from trial_records import OUTCOME_NOTE, file_digest


def relative_path(value):
    path = Path(value)
    if not path.parts or path.is_absolute() or ".." in path.parts or any(char in value for char in "\n\r\\"):
        raise ValueError(f"Invalid relative archive path: {value!r}")
    return path


def gather(log, environment, phase, bags, results):
    with log.open(encoding="utf-8-sig", newline="") as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_SH)
        reader = csv.DictReader(stream)
        columns = reader.fieldnames
        if not columns or not {"record_id", "phase", "environment_id", "bag_asset_id"}.issubset(columns):
            raise ValueError(f"{log}: missing field log columns")
        rows = [row for row in reader if (row["environment_id"] or "").strip() == environment
                and (row["phase"] or "").strip().lower() == phase]
    if not rows:
        raise ValueError(f"{log}: no {phase} rows for {environment}")
    files = {}
    record_ids = set()

    for row in rows:
        record = relative_path((row["record_id"] or "").strip())
        if len(record.parts) != 1 or record in record_ids:
            raise ValueError(f"Invalid or duplicate record ID: {record}")
        record_ids.add(record)

        if name := (row["bag_asset_id"] or "").strip():
            bag_id = relative_path(name)
            if len(bag_id.parts) != 1:
                raise ValueError(f"{record}: expected a bag directory name")
            bag = (bags / bag_id).resolve()
            info = bag_metadata(bag)
            if not info["relative_file_paths"]:
                raise ValueError(f"{bag}: no completed storage files in metadata")
            for name in ["metadata.yaml", *info["relative_file_paths"]]:
                relative = relative_path(name)
                source = (bag / relative).resolve()
                if not source.is_relative_to(bag) or not source.is_file():
                    raise ValueError(f"{bag}: invalid or missing storage file {name}")
                files[Path("bags") / bag_id / relative] = source

        notes = (row.get("notes") or "").split("; ")
        recorded = next((note[len(OUTCOME_NOTE):] for note in notes if note.startswith(OUTCOME_NOTE)), "")
        result = results / f"{record}.json"
        if recorded:
            declared = Path(recorded)
            source = declared if declared.is_absolute() else log.parent / declared
            result = source if source.is_file() else results / declared.name
        if result.is_file():
            outcome = json.loads(result.read_text(encoding="utf-8"))
            if outcome.get("trial_id") not in (None, "", str(record)):
                raise ValueError(f"{result}: trial ID differs from {record}")
            relative = Path("outcomes") / f"{record}.json"
            files[relative] = result
            if "notes" in columns:
                row["notes"] = "; ".join([OUTCOME_NOTE + relative.as_posix(),
                                          *(note for note in notes if note and not note.startswith(OUTCOME_NOTE))])
        elif recorded or (row.get("nav2_status") or "").strip():
            raise FileNotFoundError(f"{record}: could not locate its outcome JSON at {result}")

        if name := (row.get("lut_sha256_manifest") or "").strip():
            manifest = Path(name)
            if not manifest.is_absolute() and (log.parent / manifest).is_file():
                manifest = log.parent / manifest
            configuration = json.loads(manifest.read_text(encoding="utf-8"))
            configuration_id = (row.get("configuration_id") or "").strip()
            if configuration_id and configuration.get("configuration_id") != configuration_id:
                raise ValueError(f"{manifest}: configuration ID differs from {record}")
            relative = Path("configurations") / f"{file_digest(manifest)}.json"
            files[relative] = manifest
            row["lut_sha256_manifest"] = relative.as_posix()
    return columns, rows, files


def archive(destination, columns, rows, files):
    if destination.exists():
        raise FileExistsError(f"{destination}: archive directory already exists")
    needed = sum(source.stat().st_size for source in files.values())
    if shutil.disk_usage(destination.parent).free < needed:
        raise OSError(f"{destination.parent}: insufficient space to copy {needed} bytes")
    # Stage on the destination filesystem so publishing the verified archive is one rename.
    with TemporaryDirectory(prefix=".jint-field-", dir=destination.parent) as temporary:
        staging = Path(temporary)
        checksums = []
        for relative, source in sorted(files.items()):
            checksum = file_digest(source)
            copy = staging / relative
            copy.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, copy)
            if file_digest(copy) != checksum:
                raise ValueError(f"Hash mismatch when copying {source}")
            checksums.append(f"{checksum}  {relative.as_posix()}\n")

        field = staging / "field_log.csv"
        with field.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)
        checksums.append(f"{file_digest(field)}  field_log.csv\n")
        (staging / "SHA256SUMS").write_text("".join(checksums), encoding="utf-8")
        if destination.exists():
            raise FileExistsError(f"{destination}: archive directory already exists")
        staging.rename(destination)
    print(f"{destination}: {len(rows)} records, {len(checksums)} SHA-256 verified files")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("field_log", type=Path)
    parser.add_argument("--environment", required=True)
    parser.add_argument("--phase", choices=("main", "pilot"), default="main")
    parser.add_argument("--bags", type=Path, default=Path.home() / "data/bags")
    parser.add_argument("--results", type=Path, default=Path.home() / "data/trial_outcomes")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        columns, rows, files = gather(args.field_log, args.environment, args.phase, args.bags, args.results)
        archive(args.output, columns, rows, files)
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
