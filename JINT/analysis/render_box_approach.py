#!/usr/bin/env python3
"""Illustrate the annotated box and the three obstacle cues on the archived frames."""

import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from render_archive_figures import METADATA, read_archive_frame


def ranks(values):
    values = np.asarray(values)
    return [1 + (values < value).sum() + ((values == value).sum()-1)/2 for value in values]


def main():
    root = Path(__file__).resolve().parents[1]
    with (root / "analysis/evidence/box_approach_cues.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    fig = plt.figure(figsize=(5.1, 4))
    grid = fig.add_gridspec(2, 2, height_ratios=(.83, 1), left=.105, right=.976,
                            top=.93, bottom=.165, hspace=.52, wspace=.34)
    for column, clip in enumerate(("vlidar_run01", "vlidar_run02")):
        points = [r for r in rows if r["clip"] == clip]
        example = points[len(points)//2]
        db = Path.home() / "data/bags" / clip / f"{clip}_0.db3"
        im = read_archive_frame(db, int(example["rgb_recorder_ns"]))
        left, right, y = (int(example[k]) for k in ("left_x_px", "right_x_px", "row_y_px"))
        center = (left+right)//2
        photo = fig.add_subplot(grid[0,column])
        photo.imshow(im)
        photo.set(xlim=(center-105,center+105), ylim=(y+57,y-57),
                  title=f"({'ab'[column]}) Approach {column+1}, frame {example['frame']}")
        photo.plot([left,right], [y,y], color="#63f2ee", lw=1)
        photo.plot([left,right], [y,y], "|", color="#63f2ee", ms=7)
        photo.set_axis_off()
        ax = fig.add_subplot(grid[1,column])
        widths = [int(r["width_px"]) for r in points]
        for key, name, color, marker in (("vl_inverse_range", "V-LiDAR", "#145b6b", "o"),
                                          ("midas_disparity", "MiDaS", "#b14e23", "x"),
                                          ("da_disparity", "Depth Anything V2", "#615680", "+")):
            values = [float(r[key]) for r in points]
            ax.plot(widths, ranks(values), ls="none", marker=marker, mfc="none", ms=5.1,
                    color=color, label=name)
        ax.set(xlabel="Apparent box width (px)", title=f"({'cd'[column]}) {len(points)} frames")
        ax.set_yticks([1,5,10] if column == 0 else [1,5,10,15,17])
        ax.grid(c="0.91", lw=.6)
        if column == 0:
            ax.set_ylabel("Within-clip cue rank")
            handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(.5,-.005),
               ncol=3, frameon=False, fontsize=8)
    fig.savefig(root / "manuscript/figures/box_approach.pdf", dpi=260, metadata=METADATA)
    plt.close(fig)


if __name__ == "__main__":
    plt.rcParams.update({"font.size": 8, "axes.titlesize": 8.5})
    main()
