#!/usr/bin/env python3
"""Render manuscript panels from recorded RGB frames."""

import argparse
from contextlib import closing
from pathlib import Path
import sqlite3

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image

from bag_sqlite import bag_messages, bag_metadata


METADATA = {"CreationDate": None, "ModDate": None}


def rgb_image(message):
    modes = {"rgb8": "RGB", "bgr8": "BGR"}
    return Image.frombytes("RGB", (message.width, message.height), bytes(message.data),
                           "raw", modes[message.encoding], message.step, 1)


def decode_rgb(payload):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image as RosImage

    return rgb_image(deserialize_message(payload, RosImage))


def read_archive_frame(db, timestamp):
    """Read one exact RGB frame from an older SQLite archive without metadata.yaml."""
    with closing(sqlite3.connect(db.resolve().as_uri() + "?mode=ro", uri=True)) as database:
        row = database.execute("""SELECT messages.data FROM messages JOIN topics
                                  ON messages.topic_id = topics.id
                                  WHERE topics.name = '/camera/image_raw' AND messages.timestamp = ?""",
                               (timestamp,)).fetchone()
    if row is None:
        raise ValueError(f"{db}: no archived RGB frame at {timestamp}")
    return decode_rgb(row[0])


def bag_frame(bag, seconds):
    info = bag_metadata(bag)
    start = info["starting_time"]["nanoseconds_since_epoch"]
    first = start + round(seconds * 1e9)
    window = (first, first + 1_000_000_000)
    with closing(bag_messages(bag, info, ("/camera/image_raw",), window)) as messages:
        row = next(messages, None)
    if row is None:
        raise ValueError(f"{bag}: no RGB frame within one second of {seconds:g} s")
    timestamp, _, payload = row
    image = decode_rgb(payload)
    print(f"{bag.name}: RGB frame at {(timestamp - start) / 1e9:.3f} s ({timestamp} ns)")
    return image, timestamp


def save_panels(images, destination):
    rows = (len(images) + 1) // 2
    figure, axes = plt.subplots(rows, 2, figsize=(4.18, 1.79 * rows), squeeze=False)
    figure.subplots_adjust(left=0.006, right=0.994, top=0.992, bottom=0.132 / rows,
                           wspace=0.035, hspace=0.20)
    for axis in axes.flat:
        axis.set_axis_off()
        axis.set_box_aspect(3 / 4)
    for index, (axis, image) in enumerate(zip(axes.flat, images)):
        if image is None:
            axis.add_patch(Rectangle((0, 0), 1, 1, transform=axis.transAxes, clip_on=False,
                                     facecolor="#f7f8fa", edgecolor="#a3adb8", linewidth=0.55))
            axis.text(0.5, 0.5, "TBD", transform=axis.transAxes,
                      ha="center", va="center", fontsize=10, color="#596776")
        else:
            axis.imshow(image)
        axis.text(0.5, -0.045, f"({chr(97 + index)})", transform=axis.transAxes,
                  ha="center", va="top", fontsize=9)
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=330, metadata=METADATA)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group(required=True)
    inputs.add_argument("--frame", action="append", nargs=2, metavar=("BAG", "SECONDS"),
                        help="use the first recorded RGB frame at or after this time from bag start")
    inputs.add_argument("--placeholder", action="store_true", help="render four empty RGB panels marked TBD")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.placeholder:
        if args.output.name == "formal_sites.pdf":
            parser.error("formal_sites.pdf is reserved for recorded RGB frames")
        save_panels([None] * 4, args.output)
        return
    if not 2 <= len(args.frame) <= 4:
        parser.error("provide two to four --frame entries in panel order")
    if args.output.name == "formal_sites.pdf" and len(args.frame) != 4:
        parser.error("formal_sites.pdf requires four panels in the manuscript caption order")
    try:
        frames = [bag_frame(Path(bag), float(seconds))[0] for bag, seconds in args.frame]
    except (ImportError, OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    save_panels(frames, args.output)


if __name__ == "__main__":
    main()
