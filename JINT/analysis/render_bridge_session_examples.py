#!/usr/bin/env python3
"""Draw the cart and bin examples from the November 27 bridge session."""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from bag_sqlite import bag_metadata
from plot_pilot_odometry import align_to_start, read_poses
from render_archive_figures import METADATA, bag_frame


EXAMPLES = (("c_bridge_02", "Open-frame cart", "#195a77", (9.8, 15.78, 33.73)),
            ("c_bridge_08", "Blue bin", "#924916", (15.50, 26.40, 40.90)))
FIGURE = Path(__file__).resolve().parents[1] / "manuscript/figures/bridge_session_examples.pdf"


def render(bags, outcomes, output):
    fig = plt.figure(figsize=(5.12, 3.85))
    layout = fig.add_gridspec(3, 3, left=.115, right=.987, bottom=.136, top=.920,
                              height_ratios=(1, 1, .94), wspace=.077, hspace=.31)
    path = fig.add_subplot(layout[2, :])
    path.axhline(0, color=".70", ls=":", lw=1)
    path.axvline(10, color=".50", ls="--", lw=.8)
    path.text(9.96, .97, "Nominal goal", transform=path.get_xaxis_transform(),
              ha="right", va="top", fontsize=7.6, color=".3")

    for row, (trial, label, color, offsets) in enumerate(EXAMPLES):
        report = json.loads((outcomes / f"{trial}.json").read_text())
        if report["trial_id"] != trial or report["nav_status"] != "SUCCEEDED":
            raise ValueError(f"{trial}: expected a matching Nav2 completion")
        accepted, ended = (report[field] for field in ("goal_accepted_time_ns", "result_or_cancel_time_ns"))
        bag = bags / f"nav_{trial}"
        origin = bag_metadata(bag)["starting_time"]["nanoseconds_since_epoch"]
        poses = [pose for pose in read_poses(bag) if accepted <= pose[0] <= ended]
        aligned = align_to_start(poses)
        forward, lateral = zip(*aligned)
        path.plot(forward, lateral, lw=1.5, color=color, label=label)
        path.scatter(*aligned[-1], s=25, marker="D", color=color, zorder=4)

        for column, offset in enumerate(offsets):
            frame, stamp = bag_frame(bag, offset)
            if not accepted <= stamp <= ended:
                raise ValueError(f"{trial}: RGB outside the goal interval")
            index = min(range(len(poses)), key=lambda n: abs(poses[n][0] - stamp))
            letter = chr(97 + 3 * row + column)
            axis = fig.add_subplot(layout[row, column])
            axis.imshow(frame)
            axis.set_axis_off()
            inset = f"({letter})  {(stamp - origin) / 1e9:.2f} s"
            axis.text(.04, .93, inset, transform=axis.transAxes, va="top", color="white", fontsize=7.4,
                      bbox={"facecolor": color, "edgecolor": "none", "boxstyle": "round,pad=.24"})
            if row == 0:
                axis.set_title(("Approach", "Passing", "Beyond obstacle")[column], fontsize=8.5, pad=5)
            if column == 0:
                axis.text(-.085, .5, f"{label} · {trial[-2:]}", transform=axis.transAxes,
                          ha="center", va="center", rotation=90, fontsize=8, color=color)
            x, y = aligned[index]
            path.scatter(x, y, s=19, facecolor="white", edgecolor=color, zorder=3)
            path.annotate(letter, (x, y), xytext=(0, 6 if row == 0 else -10),
                          textcoords="offset points", ha="center", color=color, fontsize=7.8)
            print(f"{trial} ({letter}) RGB {stamp} ns; odom offset {(poses[index][0]-stamp)/1e6:+.1f} ms")

    path.set(xlim=(-.3, 10.4), ylim=(-.85, .66), xlabel="Forward displacement (m)",
             ylabel="Lateral (m)", yticks=(-.5, 0, .5))
    path.tick_params(labelsize=8)
    path.xaxis.label.set_size(8.5)
    path.yaxis.label.set_size(8.5)
    path.spines[["top", "right"]].set_visible(False)
    path.grid(axis="y", color=".90", lw=.6)
    path.legend(loc="upper left", ncol=2, fontsize=7.8, frameon=False)
    fig.savefig(output, dpi=320, metadata=METADATA)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags", type=Path, default=Path.home() / "data/bags")
    parser.add_argument("--outcomes", type=Path, default=Path.home() / "data/trial_outcomes")
    parser.add_argument("--output", type=Path, default=FIGURE)
    arguments = parser.parse_args()
    render(arguments.bags, arguments.outcomes, arguments.output)
