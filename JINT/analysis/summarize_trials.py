#!/usr/bin/env python3
"""Summarize the existing V-LiDAR field log for the JINT results table."""

import argparse
from collections import Counter, defaultdict
import csv
import math
from pathlib import Path
import re


FAILURES = {"collision", "timeout", "nav_abort", "manual_intervention"}
COLUMNS = {
    "record_id", "phase", "environment_id", "scenario_id", "outcome",
    "contact_observed", "manual_intervention", "evaluation_included",
    "invalid_reason", "external_goal_check",
}
CONDITION_LABELS = (
    "Reference hall (day)",
    "Reference hall (night)",
    "Artificial-light interior",
    "Daylit bridge",
)
DEFAULT_CONDITION_IDS = ("original_day", "original_night", "indoor_artificial", "bridge_daylight")
LAYOUT_LABELS = ("Box", "Box + chair")
DEFAULT_LAYOUT_IDS = ("S1", "S2")


def classify(row):
    outcome = row["outcome"].lower()
    included = row["evaluation_included"].lower()
    if outcome == "invalid":
        return "invalid" if included == "false" and row["invalid_reason"] else "unresolved"
    if included != "true":
        return "unresolved"
    if outcome in FAILURES:
        return outcome
    if (
        outcome == "success"
        and row["contact_observed"].lower() == "false"
        and row["manual_intervention"].lower() == "false"
        and row["external_goal_check"]
    ):
        return "success"
    return "unresolved"


def read_main_trials(filename):
    with filename.open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = COLUMNS - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"Missing field log columns: {', '.join(sorted(missing))}")
        seen = set()
        trials = defaultdict(Counter)
        for line_number, raw_row in enumerate(reader, start=2):
            row = {column: (raw_row[column] or "").strip() for column in COLUMNS}
            if row["phase"].lower() != "main":
                continue
            record_id = row["record_id"]
            if not record_id or record_id in seen:
                raise ValueError(f"Line {line_number}: missing or duplicate main record_id")
            seen.add(record_id)
            config = (raw_row.get("configuration_id") or "").strip() or "(unrecorded)"
            group = (config, row["environment_id"] or "(missing)", row["scenario_id"] or "(missing)")
            trials[group][classify(row)] += 1
    return trials


def wilson_bounds(successes, evaluated):
    z = 1.959964
    p = successes / evaluated
    scale = 1 + z**2 / evaluated
    center = (p + z**2 / (2 * evaluated)) / scale
    margin = z * math.sqrt(p * (1 - p) / evaluated + z**2 / (4 * evaluated**2)) / scale
    return 100 * max(0, center - margin), 100 * min(1, center + margin)


def report(counts):
    failures = sum(counts[failure] for failure in FAILURES)
    successes = counts["success"]
    evaluated = successes + failures
    interval = "{:.1f}-{:.1f}%".format(*wilson_bounds(successes, evaluated)) if evaluated else "-"
    return (sum(counts.values()), evaluated, successes, failures, counts["invalid"],
            counts["unresolved"], interval)


def select_outcomes(trials, config, condition_ids, layout_ids):
    configurations = sorted({key[0] for key in trials})
    if config is None:
        if len(configurations) != 1:
            options = ", ".join(configurations)
            raise ValueError(f"select one configuration for the formal results (available: {options or 'none'})")
        config = configurations[0]
    if config not in configurations:
        raise ValueError(f"no main trials for configuration {config}")
    if len(set(condition_ids)) != len(CONDITION_LABELS) or len(set(layout_ids)) != len(LAYOUT_LABELS):
        raise ValueError("use distinct environment IDs and distinct layout IDs")

    selected = {(condition, layout): counts
                for (name, condition, layout), counts in trials.items() if name == config}
    expected = {(condition, layout) for condition in condition_ids for layout in layout_ids}
    if set(selected) - expected:
        raise ValueError(f"unmapped main-trial conditions or layouts: {sorted(set(selected) - expected)}")
    return config, selected


def export_latex(selected, output, condition_ids, layout_ids, figure_name=None):
    lines = [rf"\gdef\JINTFormalOutcomesFigure{{{figure_name}}}%"] if figure_name else []
    for condition, condition_label in zip(condition_ids, CONDITION_LABELS):
        for layout, layout_label in zip(layout_ids, LAYOUT_LABELS):
            counts = selected.get((condition, layout))
            if counts is None:
                numbers = ("--",) * 5
            else:
                attempts, evaluated, successes, _, invalid, unresolved, interval = report(counts)
                rate = (
                    f"{successes}/{evaluated} ({100 * successes / evaluated:.1f}\\%)"
                    if evaluated else "--"
                )
                numbers = (attempts, rate, interval.replace("-", "--").replace("%", r"\%"),
                           invalid, unresolved)
            fields = (condition_label, layout_label, *numbers)
            lines.append(" & ".join(map(str, fields)) + r" \\")
    lines.append(r"\bottomrule")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_outcomes(selected, condition_ids, layout_ids):
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    colours = ("#175676", "#a14c12")
    markers = ("o", "s")
    figure, axis = plt.subplots(figsize=(5.25, 2.45))
    figure.subplots_adjust(left=.35, right=.87, bottom=.23, top=.79)
    centers = list(range(len(condition_ids) - 1, -1, -1))
    for y, condition in zip(centers, condition_ids):
        if y % 2:
            axis.axhspan(y - .5, y + .5, color="#f2f6f8", lw=0)
        for offset, layout, colour, marker in zip((.17, -.17), layout_ids, colours, markers):
            y_plot = y + offset
            counts = selected.get((condition, layout))
            label = "n/a"
            if counts is not None:
                _, evaluated, successes, *_ = report(counts)
                if evaluated:
                    estimate = 100 * successes / evaluated
                    lower, upper = wilson_bounds(successes, evaluated)
                    axis.errorbar(estimate, y_plot, xerr=[[max(0, estimate - lower)], [max(0, upper - estimate)]],
                                  color=colour, fmt=marker, markersize=4, capsize=2, linewidth=1.1,
                                  clip_on=False, zorder=3)
                    label = f"{successes}/{evaluated}"
                else:
                    label = "0 eval."
            axis.text(1.045, y_plot, label, transform=axis.get_yaxis_transform(), va="center",
                      color=colour, fontsize=8)

    axis.set(xlim=(0, 100), ylim=(-.5, len(condition_ids) - .5), xticks=(0, 25, 50, 75, 100), yticks=centers)
    axis.set_yticklabels(CONDITION_LABELS, fontsize=8)
    axis.tick_params(axis="x", labelsize=8)
    axis.tick_params(axis="y", length=0, pad=7)
    axis.set_xlabel("Physical success rate (%)", fontsize=9)
    axis.grid(axis="x", color="#cbd8df", linewidth=.65)
    axis.set_axisbelow(True)
    axis.spines[["top", "left", "right"]].set_visible(False)
    axis.legend(handles=[Line2D([], [], marker=marker, color=colour, linestyle="none", markersize=5, label=label)
                         for marker, colour, label in zip(markers, colours, LAYOUT_LABELS)],
                bbox_to_anchor=(-.02, 1.03), loc="lower left", ncol=2, fontsize=8, frameon=False,
                columnspacing=1, handletextpad=.3, borderaxespad=0)
    return figure


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("field_log", type=Path)
    parser.add_argument("--latex-output", type=Path, help="write rows for the JINT formal-results table")
    parser.add_argument("--figure-output", type=Path, help="write the matching physical-success plot as PDF")
    parser.add_argument("--configuration", help="configuration ID for the LaTeX table")
    parser.add_argument("--condition-ids", nargs=4, default=DEFAULT_CONDITION_IDS,
                        metavar=("ORIGINAL_DAY", "ORIGINAL_NIGHT", "ARTIFICIAL", "BRIDGE"))
    parser.add_argument("--layout-ids", nargs=2, default=DEFAULT_LAYOUT_IDS, metavar=("BOX", "BOX_CHAIR"))
    args = parser.parse_args()
    try:
        trials = read_main_trials(args.field_log)
        if args.latex_output or args.figure_output:
            config, selected = select_outcomes(trials, args.configuration, args.condition_ids, args.layout_ids)
        if args.figure_output:
            if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*\.pdf", args.figure_output.name):
                raise ValueError("give the results figure a LaTeX-compatible PDF filename")
            import matplotlib
            matplotlib.use("Agg")
            figure = plot_outcomes(selected, args.condition_ids, args.layout_ids)
            args.figure_output.parent.mkdir(parents=True, exist_ok=True)
            figure.savefig(args.figure_output, metadata={"CreationDate": None, "ModDate": None})
            print(f"Outcomes figure for {config}: {args.figure_output}\n")
        if args.latex_output:
            figure_name = args.figure_output.name if args.figure_output else None
            export_latex(selected, args.latex_output, args.condition_ids, args.layout_ids, figure_name)
            print(f"LaTeX rows for {config}: {args.latex_output}\n")
    except (ImportError, OSError, ValueError) as error:
        parser.exit(2, f"{error}\n")

    print("| Configuration | Environment | Layout | Attempts | Evaluated | Successes | Failures | Invalid | Unresolved | 95% Wilson interval |")
    print("| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |")
    totals = defaultdict(Counter)
    for (config, environment, layout), counts in sorted(trials.items()):
        totals[config].update(counts)
        fields = (config, environment, layout, *report(counts))
        print("| " + " | ".join(map(str, fields)) + " |")
    for config, total in sorted(totals.items()):
        print("| " + " | ".join(map(str, (config, "All", "All", *report(total)))) + " |")
    for config, total in sorted(totals.items()):
        print(f"\nFailure counts ({config}):", ", ".join(f"{key}={total[key]}" for key in sorted(FAILURES)))
    if not trials:
        print("\nNo main trials recorded.")


if __name__ == "__main__":
    main()
