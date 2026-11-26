#!/usr/bin/env python3
"""Draw an odometry-only figure from selected ROS 2 bag directories."""

import argparse
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message

from bag_sqlite import bag_messages, bag_metadata


def read_poses(bag_directory):
    poses = []
    info = bag_metadata(bag_directory)
    for stamp, _, data in bag_messages(bag_directory, info, ("/odom",)):
        pose = deserialize_message(data, Odometry).pose.pose
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        poses.append((stamp, pose.position.x, pose.position.y, yaw))
    if not poses:
        raise ValueError(f"{bag_directory} contains no odometry poses")
    return sorted(poses)


def align_to_start(poses):
    _, x0, y0, yaw0 = poses[0]
    c, s = math.cos(yaw0), math.sin(yaw0)
    return [
        (c * (x - x0) + s * (y - y0), -s * (x - x0) + c * (y - y0))
        for _, x, y, _ in poses
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("bags", type=Path, nargs="+")
    args = parser.parse_args()

    fig, ax = plt.subplots(figsize=(6.8, 2.8), layout="constrained")
    ax.axvline(10, color="0.50", linestyle=":", linewidth=1.0, label="Nominal goal x")
    for bag in args.bags:
        poses = read_poses(bag)
        x, y = zip(*align_to_start(poses))
        line, = ax.plot(x, y, linewidth=1.55, label=bag.name)
        ax.scatter(x[-1], y[-1], s=14, color=line.get_color(), zorder=3)
        print(f"{bag.name}: {len(poses)} poses; terminal ({x[-1]:.3f}, {y[-1]:.3f}) m; "
              f"max |y| {max(map(abs, y)):.3f} m")
    ax.set_xlabel("Forward displacement (m)")
    ax.set_ylabel("Lateral displacement (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(color="0.90", linewidth=0.6)
    ax.legend(loc="upper left", ncol=3, fontsize=8, frameon=False)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output)


if __name__ == "__main__":
    main()
