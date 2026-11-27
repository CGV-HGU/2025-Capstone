"""Compare published global costmaps and plan events around archived failures."""

import csv
from dataclasses import dataclass
import re

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Rectangle
from nav_msgs.msg import OccupancyGrid, Path as NavPath
import numpy as np
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message

from bag_sqlite import bag_messages, bag_metadata


GLOBAL_GRID = "/global_costmap/costmap"
FAILURE = re.compile(r"Planning algorithm GridBased failed to generate a valid path to \(([-\d.]+), ([-\d.]+)\)")
COLORS = ("#9da4ad", "#f9fafb", "#ccb7de", "#78469a", "#42164f")


@dataclass(frozen=True)
class PlannerCase:
    bag_id: str
    origin_ns: int
    failure_ns: int
    goal: tuple[float, float]
    tolerance_m: float
    before: tuple[int, OccupancyGrid] | None
    after: tuple[int, OccupancyGrid] | None
    clear_ns: int | None
    next_path: tuple[int, NavPath] | None


def read_cases(bag):
    info = bag_metadata(bag)
    origin = info["starting_time"]["nanoseconds_since_epoch"]
    types = {GLOBAL_GRID: OccupancyGrid, "/rosout": Log, "/plan": NavPath}
    samples = {topic: [] for topic in types}
    for stamp, topic, payload in bag_messages(bag, info, types):
        samples[topic].append((stamp, deserialize_message(payload, types[topic])))

    tolerance = None
    for stamp, message in samples["/rosout"]:
        if message.name != "planner_server":
            continue
        if match := re.fullmatch(r"GridBased: failed to create plan with tolerance ([\d.]+)\.", message.msg):
            tolerance = float(match[1])
            continue
        match = FAILURE.fullmatch(message.msg)
        if not match or tolerance is None:
            continue
        goal = (float(match[1]), float(match[2]))
        before = next((sample for sample in reversed(samples[GLOBAL_GRID]) if sample[0] <= stamp), None)
        after = next((sample for sample in samples[GLOBAL_GRID] if sample[0] > stamp), None)
        path = next((sample for sample in samples["/plan"] if sample[0] > stamp and sample[1].poses), None)
        end = path[0] if path else float("inf")
        clear = next((time for time, event in samples["/rosout"]
                      if stamp < time < end and event.name == "global_costmap.global_costmap"
                      and event.msg == "Received request to clear entirely the global_costmap"), None)
        yield PlannerCase(bag.name, origin, stamp, goal, tolerance, before, after, clear, path)
        tolerance = None


def grid_region(case, message):
    info = message.info
    if message.header.frame_id != "map" or abs(info.origin.orientation.w) != 1:
        raise ValueError(f"{case.bag_id}: expected an axis-aligned global grid in map")
    cost = np.asarray(message.data).reshape(info.height, info.width)
    gx, gy = case.goal
    xmin = info.origin.position.x - gx
    ymin = info.origin.position.y - gy
    r = info.resolution
    extent = (xmin, xmin + r * info.width, ymin, ymin + r * info.height)
    x = xmin + (np.arange(info.width) + 0.5) * r
    y = ymin + (np.arange(info.height) + 0.5) * r
    region = cost[np.ix_(np.abs(y) <= case.tolerance_m, np.abs(x) <= case.tolerance_m)]
    return cost, extent, region


def describe_case(case):
    def relative_ms(stamp):
        return round((stamp - case.failure_ns) / 1e6, 3) if stamp is not None else ""

    row = {"bag_id": case.bag_id, "failure_elapsed_s": round((case.failure_ns - case.origin_ns) / 1e9, 6),
           "goal_x_m": case.goal[0], "goal_y_m": case.goal[1], "tolerance_m": case.tolerance_m,
           "clear_request_ms": relative_ms(case.clear_ns)}
    record_time, path = case.next_path or (None, None)
    if path and path.header.frame_id != "map":
        raise ValueError(f"{case.bag_id}: expected the recorded global path in map")
    end = path.poses[-1].pose.position if path else None
    row.update(next_path_ms=relative_ms(record_time), next_path_poses=len(path.poses) if path else "",
               next_path_end_x_m=end.x if end else "", next_path_end_y_m=end.y if end else "")
    for name, sample in (("before", case.before), ("after", case.after)):
        stamp, grid = sample or (None, None)
        if grid is None:
            region = np.array([], dtype=int)
        else:
            _, _, region = grid_region(case, grid)
        row[f"{name}_grid_ms"] = relative_ms(stamp)
        row[f"{name}_region_cells"] = region.size
        row[f"{name}_cost_0"] = int((region == 0).sum())
        row[f"{name}_cost_99"] = int((region == 99).sum())
        row[f"{name}_cost_100"] = int((region == 100).sum())
    return row


def planner_snapshot_page(bags, evidence):
    cases = [case for index in (3, 4, 5) for case in read_cases(bags / f"nav_bridge_{index:02d}")]
    rows = [describe_case(case) for case in cases]
    with (evidence / "pilot_planner_costmaps.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

    case = [case for case in cases if case.bag_id == "nav_bridge_04"][-1]
    row = describe_case(case)
    if not (case.before and case.after and case.clear_ns and case.next_path):
        raise ValueError(f"{case.bag_id}: incomplete costmap or recovery evidence")
    figure, axes = plt.subplots(1, 2, sharex=True, sharey=True, figsize=(7.1, 4.35))
    figure.subplots_adjust(left=.105, right=.98, top=.758, bottom=.29, wspace=.12)
    figure.suptitle("S5. Global costmap around a planning failure", y=.97, fontsize=14)
    figure.text(.5, .885, f"Fourth bridge pilot. Time zero is the failure log at {row['failure_elapsed_s']:.3f} s.",
                ha="center", fontsize=9.5)
    handles = [Patch(facecolor=COLORS[index], label=label) for index, label in (
        (2, "Cost 1–98"), (3, "Inscribed (99)"), (4, "Lethal (100)"),
    )]
    handles += [Line2D([], [], color="#203242", ls="--", lw=.9, label="Tolerance"),
                Line2D([], [], color="#067a7b", lw=1.3, label="Next plan")]
    figure.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, .128), ncol=5,
                  fontsize=8.5, frameon=False)

    for axis, (key, snapshot, label) in zip(axes, (
        ("before", case.before, "(a) Last grid before failure"),
        ("after", case.after, "(b) First grid after failure"),
    )):
        _, message = snapshot
        cost, extent, region = grid_region(case, message)
        edges_x = np.linspace(extent[0], extent[1], message.info.width + 1)
        edges_y = np.linspace(extent[2], extent[3], message.info.height + 1)
        cols = np.flatnonzero((edges_x[:-1] < 1.55) & (edges_x[1:] > -1.55))
        rows = np.flatnonzero((edges_y[:-1] < 1.35) & (edges_y[1:] > -1.35))
        classes = np.digitize(cost[np.ix_(rows, cols)], (0, 1, 99, 100))
        axis.pcolormesh(edges_x[cols[0]:cols[-1] + 2], edges_y[rows[0]:rows[-1] + 2], classes,
                        cmap=ListedColormap(COLORS), vmin=0, vmax=4, shading="flat")
        tolerance = case.tolerance_m
        axis.add_patch(Rectangle((-tolerance, -tolerance), 2 * tolerance, 2 * tolerance,
                                 fill=False, edgecolor="#203242", linewidth=.9, linestyle="--"))
        axis.scatter(0, 0, marker="+", color="#203242", linewidths=1.25, s=36, zorder=5)
        axis.set_title(f"{label}  ({row[f'{key}_grid_ms']:+.0f} ms)\n"
                       f"{(region >= 99).sum()}/{region.size} cells at cost 99–100", fontsize=9.5)
        axis.set(xlim=(-1.55, 1.55), ylim=(-1.35, 1.35), aspect="equal")
    axes[0].set_ylabel("Map y relative to goal (m)")
    figure.supxlabel("Map x relative to goal (m)", x=.54, y=.204, fontsize=10)
    _, path = case.next_path
    axes[1].plot([pose.pose.position.x - case.goal[0] for pose in path.poses],
                 [pose.pose.position.y - case.goal[1] for pose in path.poses], color="#067a7b", lw=1.6)
    figure.text(.5, .062, f"Clear request at {row['clear_request_ms']:+.0f} ms. "
                f"Next plan published at {row['next_path_ms']:+.0f} ms.", ha="center", fontsize=9.5)
    return figure
