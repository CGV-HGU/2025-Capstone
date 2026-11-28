#!/usr/bin/env python3
"""Render manuscript panels from recorded RGB frames."""

import argparse
from contextlib import closing
import json
from pathlib import Path
import sqlite3

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image, ImageOps

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


def load_layout(layout):
    panels = json.loads(layout.read_text(encoding="utf-8"))
    if not isinstance(panels, list) or len(panels) not in (3, 4):
        raise ValueError("the site layout requires three or four panels in manuscript order")
    images = []
    for panel in panels:
        if panel is None:
            images.append(None)
            continue
        if "photo" in panel:
            with Image.open(layout.parent / Path(panel["photo"]).expanduser()) as photo:
                images.append(ImageOps.exif_transpose(photo).convert("RGB"))
        elif "bag" in panel:
            bag = layout.parent / Path(panel["bag"]).expanduser()
            images.append(bag_frame(bag, panel["seconds"])[0])
        else:
            images.append(None)
    return images, panels


def show_photo(axis, image, panel):
    left, top, right, bottom = panel.get("crop", (0, 0, *image.size))
    if not (0 <= left < right <= image.width and 0 <= top < bottom <= image.height):
        raise ValueError("panel crop must fit inside the RGB frame")
    if (right - left) * 3 != (bottom - top) * 4:
        raise ValueError("panel crop must have a 4:3 aspect ratio")
    axis.imshow(image.crop((left, top, right, bottom)))
    if callout := panel.get("callout"):
        target_x, target_y = callout["target"]
        text_x, text_y = callout["position"]
        if not (left < target_x < right and top < target_y < bottom and
                left < text_x < right and top < text_y < bottom):
            raise ValueError("obstacle callout must fit inside the panel crop")
        axis.annotate(callout["text"], (target_x - left, target_y - top),
                      xytext=(text_x - left, text_y - top), ha="center", va="center",
                      fontsize=7.2, color="white",
                      bbox={"facecolor": "#293746", "edgecolor": "none", "boxstyle": "round,pad=.23"},
                      arrowprops={"arrowstyle": "-|>", "color": "white", "lw": .78, "shrinkB": 1.5})


def save_panels(images, destination, panels=None):
    columns = 3 if panels is not None and len(images) == 3 else 2
    rows = (len(images) + columns - 1) // columns
    figure, axes = plt.subplots(rows, columns, figsize=(2.09 * columns, 1.79 * rows), squeeze=False)
    figure.subplots_adjust(left=0.006, right=0.994, top=0.992, bottom=0.132 / rows,
                           wspace=0.035, hspace=0.20)
    for axis in axes.flat:
        axis.set_axis_off()
        axis.set_box_aspect(3 / 4)
    for index, (axis, image) in enumerate(zip(axes.flat, images)):
        panel = panels[index] if panels else None
        if image is None:
            axis.add_patch(Rectangle((0, 0), 1, 1, transform=axis.transAxes, clip_on=False,
                                     facecolor="#f7f8fa", edgecolor="#a3adb8", linewidth=0.55))
            axis.text(0.5, 0.5, "TBD", transform=axis.transAxes,
                      ha="center", va="center", fontsize=10, color="#596776")
        elif panel is None:
            axis.imshow(image)
        else:
            show_photo(axis, image, panel)
        label = f"({chr(97 + index)})"
        if panel and panel.get("label"):
            label += f" {panel['label']}"
        axis.text(0.5, -0.045, label, transform=axis.transAxes,
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
    inputs.add_argument("--layout", type=Path, help="render site panels from a JSON layout")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.placeholder:
        if args.output.name == "formal_sites.pdf":
            parser.error("formal_sites.pdf is reserved for recorded RGB frames")
        save_panels([None] * 4, args.output)
        return
    try:
        if args.layout:
            if args.output.name == "formal_sites.pdf":
                parser.error("formal_sites.pdf requires four recorded --frame entries")
            frames, panels = load_layout(args.layout)
        else:
            if not 2 <= len(args.frame) <= 4:
                parser.error("provide two to four --frame entries in panel order")
            if args.output.name == "formal_sites.pdf" and len(args.frame) != 4:
                parser.error("formal_sites.pdf requires four panels in the manuscript caption order")
            frames = [bag_frame(Path(bag), float(seconds))[0] for bag, seconds in args.frame]
            panels = None
        save_panels(frames, args.output, panels)
    except (ImportError, OSError, ValueError, KeyError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
