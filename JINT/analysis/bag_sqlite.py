"""Read SQLite rosbag2 data without writing into the recording directory."""

from contextlib import closing
from pathlib import Path
import sqlite3
import subprocess
import tempfile

import yaml


def bag_metadata(directory):
    metadata = yaml.safe_load((directory / "metadata.yaml").read_text(encoding="utf-8"))
    info = metadata["rosbag2_bagfile_information"]
    if info["storage_identifier"] != "sqlite3":
        raise ValueError(f"{directory}: expected SQLite bag storage")
    return info


def open_databases(directory, info):
    for filename in info["relative_file_paths"]:
        stored = directory / filename
        with tempfile.TemporaryDirectory(prefix="jint-bag-") as temporary:
            db_path = stored
            if stored.suffix == ".zstd":
                db_path = Path(temporary) / stored.stem
                subprocess.run(["zstd", "-dq", str(stored), "-o", str(db_path)], check=True)
            with closing(sqlite3.connect(db_path.resolve().as_uri() + "?mode=ro", uri=True)) as db:
                yield db


def bag_messages(directory, info, topic_names, window=None, payload_bytes=None):
    placeholders = ", ".join("?" for _ in topic_names)
    payload = "messages.data" if payload_bytes is None else "substr(messages.data, 1, ?)"
    query = f"""SELECT messages.timestamp, topics.name, {payload}
                FROM messages JOIN topics ON messages.topic_id = topics.id
                WHERE topics.name IN ({placeholders})"""
    parameters = (() if payload_bytes is None else (payload_bytes,)) + tuple(topic_names)
    if window is not None:
        query += " AND messages.timestamp >= ? AND messages.timestamp < ?"
        parameters += tuple(window)
    query += " ORDER BY messages.timestamp"
    with closing(open_databases(directory, info)) as databases:
        for db in databases:
            yield from db.execute(query, parameters)


def first_topic_payloads(directory, info, topic_names):
    """Read a full sample of each requested topic across the bag's split files."""
    samples = {}
    query = """SELECT messages.data FROM messages JOIN topics ON messages.topic_id = topics.id
               WHERE topics.name = ? ORDER BY messages.timestamp LIMIT 1"""
    with closing(open_databases(directory, info)) as databases:
        for db in databases:
            for topic in topic_names:
                if topic not in samples and (row := db.execute(query, (topic,)).fetchone()) is not None:
                    samples[topic] = row[0]
            if len(samples) == len(topic_names):
                break
    return samples
