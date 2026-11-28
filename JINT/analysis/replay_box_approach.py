#!/usr/bin/env python3
"""Read obstacle-local cues from three models on the two archived box approaches."""

import argparse
import csv
from pathlib import Path
import sys

import cv2
import numpy as np
import torch

from render_archive_figures import read_archive_frame

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "experiments"))
from benchmark_depth_baselines import DepthAnythingV2Wrapper, MiDaSWrapper, VLiDARWrapper

EVIDENCE = Path(__file__).with_name("evidence")


def sample_box(row, bags, vlidar, midas, depth):
    clip, stamp = row["clip"], int(row["rgb_recorder_ns"])
    image = read_archive_frame(bags / clip / f"{clip}_0.db3", stamp)
    bgr = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
    left, right, y = (int(row[key]) for key in ("left_x_px", "right_x_px", "row_y_px"))
    width = right - left
    # Keep central-half columns whose pixel centers map inside the annotation.
    x_original = (np.arange(vlidar.target_w) + .5) * image.width / vlidar.target_w - .5
    cols = np.flatnonzero((x_original >= left + width / 4) & (x_original <= right - width / 4))
    scan, _ = vlidar.infer(bgr)
    bins = np.unique(vlidar.col_to_ch_lut[cols])
    ranges = scan[bins]
    valid = ranges[np.isfinite(ranges) & (ranges > .1) & (ranges <= 10)]
    result = {**row, "width_px": width, "valid_scan_bins": len(valid), "scan_bins": len(bins),
              "vl_inverse_range": 1 / float(np.median(valid)) if len(valid) else ""}
    for name, model in (("midas_disparity", midas), ("da_disparity", depth)):
        _, disparity = model.infer_disparity(bgr)
        if disparity.shape != (vlidar.target_h, vlidar.target_w):
            raise ValueError(f"{name}: unexpected map dimensions {disparity.shape}")
        center = round((y + .5) * disparity.shape[0] / image.height - .5)
        patch = disparity[center-1:center+2, cols]
        finite = patch[np.isfinite(patch)]
        result[name] = float(np.median(finite)) if finite.size else ""
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", type=Path, default=Path.home() / "data/bags")
    parser.add_argument("--output", type=Path, default=EVIDENCE / "box_approach_cues.csv")
    args = parser.parse_args()
    torch.set_num_threads(4)
    torch.set_num_interop_threads(1)
    cv2.setNumThreads(1)
    vlidar = VLiDARWrapper(str(ROOT / "src/freespace_detection/scripts"), device="cpu", use_openvino=True)
    midas, depth = MiDaSWrapper(device="cpu"), DepthAnythingV2Wrapper(device="cpu")
    if not (midas.is_loaded and depth.is_loaded):
        raise RuntimeError("Both depth models must load to compare the same frames")
    with (EVIDENCE / "box_approach_annotations.csv").open() as stream:
        rows = [sample_box(row, args.bags, vlidar, midas, depth) for row in csv.DictReader(stream)]
    with args.output.open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0], lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
