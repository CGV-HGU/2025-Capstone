#!/usr/bin/env python3
"""Render manuscript panels from bag RGB frames and optional archived author figures."""

import argparse
from contextlib import closing
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image, ImageOps
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image as RosImage

from bag_sqlite import bag_messages, bag_metadata


ROOT = Path(__file__).resolve().parents[2]
ARCHIVE = ROOT / "paper/workspace/IEEE_Access/figures"
OUTPUT = ROOT / "JINT/manuscript/figures"
METADATA = {"CreationDate": None, "ModDate": None}


def archived_image(name):
    with Image.open(ARCHIVE / name) as image:
        return ImageOps.exif_transpose(image).convert("RGB")


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
    message = deserialize_message(payload, RosImage)
    modes = {"rgb8": "RGB", "bgr8": "BGR"}
    image = Image.frombytes("RGB", (message.width, message.height), bytes(message.data),
                            "raw", modes[message.encoding], message.step, 1)
    print(f"{bag.name}: RGB frame at {(timestamp - start) / 1e9:.3f} s ({timestamp} ns)")
    return image


def save_panels(images, destination):
    rows = (len(images) + 1) // 2
    figure, axes = plt.subplots(rows, 2, figsize=(4.18, 1.79 * rows), squeeze=False)
    figure.subplots_adjust(left=0.006, right=0.994, top=0.992, bottom=0.132 / rows,
                           wspace=0.035, hspace=0.20)
    for axis in axes.flat:
        axis.set_axis_off()
    for index, (axis, image) in enumerate(zip(axes.flat, images)):
        axis.imshow(image)
        axis.text(0.5, -0.045, f"({chr(97 + index)})", transform=axis.transAxes,
                  ha="center", va="top", fontsize=9)
    destination.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(destination, dpi=330, metadata=METADATA)
    plt.close(figure)


def save_archived_views():
    sequence = archived_image("nav2_obstacle_avoidance_sequence.png")
    second_view = sequence.crop((3316, 0, 6632, 1788))
    rgb = second_view.crop((550, 130, 1650, 900))
    costmap = second_view.crop((1770, 475, 2870, 1245))
    save_panels((rgb, costmap), OUTPUT / "archive_nav2_view.pdf")

    reflection = archived_image("segmentation_artifact.png").crop((380, 180, 1140, 709))
    figure = plt.figure(figsize=(2.33, 2.33 * reflection.height / reflection.width))
    axis = figure.add_axes((0, 0, 1, 1))
    axis.imshow(reflection)
    axis.set_axis_off()
    figure.savefig(OUTPUT / "archive_floor_reflection.pdf", dpi=330, metadata=METADATA)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frame", action="append", nargs=2, required=True, metavar=("BAG", "SECONDS"),
                        help="use the first recorded RGB frame at or after this time from bag start")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--archive-views", action="store_true", help="also render legacy RViz and floor-reflection views")
    args = parser.parse_args()
    if not 2 <= len(args.frame) <= 4:
        parser.error("provide two to four --frame entries in panel order")
    if args.output.name == "formal_sites.pdf" and len(args.frame) != 4:
        parser.error("formal_sites.pdf requires four panels in the manuscript caption order")
    try:
        frames = [bag_frame(Path(bag), float(seconds)) for bag, seconds in args.frame]
    except (OSError, ValueError, KeyError) as error:
        parser.error(str(error))
    save_panels(frames, args.output)
    if args.archive_views:
        save_archived_views()


if __name__ == "__main__":
    main()
