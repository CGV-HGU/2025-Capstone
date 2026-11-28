#!/usr/bin/env python3
"""Create fixed-time forward-camera review sheets for the 37 follow-up trials."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import json
from pathlib import Path

from PIL import Image, ImageDraw

from bag_sqlite import bag_metadata, open_databases
from render_archive_figures import decode_rgb


def review_sheet(trial, args):
    outcome = json.loads((args.outcomes / f"{trial}.json").read_text())
    start = outcome["goal_accepted_time_ns"]
    end = outcome["result_or_cancel_time_ns"]
    bag = args.bags / f"nav_{trial}"
    if trial == "c_bridge_13":
        bag = args.recovered / bag.name
    targets = {start + s * 10**9 for s in range(0, 33, 4) if start + s * 10**9 <= end}
    targets.update([max(start, end - 5 * 10**9), end])
    nearest = {}
    for db in open_databases(bag, bag_metadata(bag)):
        topic, = db.execute("SELECT id FROM topics WHERE name = '/camera/image_raw'").fetchone()
        samples = db.execute("SELECT timestamp, id FROM messages WHERE topic_id=? "
                             "AND timestamp BETWEEN ? AND ? ORDER BY timestamp", (topic, start, end)).fetchall()
        if not samples:
            continue
        for target in sorted(targets):
            stamp, message_id = min(samples, key=lambda row: (abs(row[0] - target), row[0]))
            if target in nearest and abs(nearest[target][0] - target) <= abs(stamp - target):
                continue
            data, = db.execute("SELECT data FROM messages WHERE id=?", (message_id,)).fetchone()
            nearest[target] = stamp, decode_rgb(data)
    if len(nearest) != len(targets):
        raise ValueError(f"{trial}: missing RGB in the goal interval")
    sheet = Image.new("RGB", (4 * 240, ((len(targets) + 3) // 4) * 202), "white")
    draw = ImageDraw.Draw(sheet)
    manifest = []
    for index, (target, (stamp, frame)) in enumerate(sorted(nearest.items())):
        x, y = index % 4 * 240, index // 4 * 202
        frame.thumbnail((240, 180))
        sheet.paste(frame, (x, y))
        elapsed = (stamp - start) / 1e9
        draw.text((x + 2, y + 181), f"{trial}   {elapsed:.2f} s", fill="black")
        manifest.append({"trial_id": trial, "requested_s": round((target-start)/1e9, 6),
                         "rgb_recorder_ns": stamp, "from_acceptance_s": round(elapsed, 6)})
    sheet.save(args.output / f"{trial}.jpg", quality=91)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    data = Path.home() / "data"
    parser.add_argument("--bags", type=Path, default=data / "bags")
    parser.add_argument("--recovered", type=Path, default=data / "jint_recovered")
    parser.add_argument("--outcomes", type=Path, default=data / "trial_outcomes")
    args = parser.parse_args()
    evidence = Path(__file__).with_name("evidence")
    trials = []
    for filename in ("bridge_session_2026-11-27_audit.csv", "class5_session_2026-11-27.csv"):
        with (evidence / filename).open() as stream:
            trials.extend(row["trial_id"] for row in csv.DictReader(stream))
    args.output.mkdir(parents=True, exist_ok=True)
    with ThreadPoolExecutor(max_workers=2) as pool, (args.output / "rgb_samples.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=["trial_id", "requested_s", "rgb_recorder_ns",
                                                    "from_acceptance_s"], lineterminator="\n")
        writer.writeheader()
        for rows in pool.map(lambda trial: review_sheet(trial, args), trials):
            writer.writerows(rows)


if __name__ == "__main__":
    main()
