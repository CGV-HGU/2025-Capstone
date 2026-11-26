#!/usr/bin/env python3
"""Summarize the existing V-LiDAR field log for the JINT results table."""

import argparse
from collections import Counter, defaultdict
import csv
import math
from pathlib import Path


FAILURES = {"collision", "timeout", "nav_abort", "manual_intervention"}
COLUMNS = {
    "record_id", "phase", "environment_id", "scenario_id", "outcome",
    "contact_observed", "manual_intervention", "evaluation_included",
    "invalid_reason", "external_goal_check",
}
CONDITION_LABELS = (
    "Original corridor (day)",
    "Original corridor (night)",
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


def wilson_interval(successes, evaluated):
    if not evaluated:
        return "-"
    z = 1.959964
    p = successes / evaluated
    scale = 1 + z**2 / evaluated
    center = (p + z**2 / (2 * evaluated)) / scale
    margin = z * math.sqrt(p * (1 - p) / evaluated + z**2 / (4 * evaluated**2)) / scale
    lower, upper = max(0, center - margin), min(1, center + margin)
    return f"{100 * lower:.1f}-{100 * upper:.1f}%"


def report(counts):
    failures = sum(counts[failure] for failure in FAILURES)
    successes = counts["success"]
    evaluated = successes + failures
    interval = wilson_interval(successes, evaluated)
    return (sum(counts.values()), evaluated, successes, failures, counts["invalid"],
            counts["unresolved"], interval)


def export_latex(trials, output, config, condition_ids, layout_ids):
    configurations = sorted({key[0] for key in trials})
    if config is None:
        if len(configurations) != 1:
            options = ", ".join(configurations)
            raise ValueError(f"select one configuration for the LaTeX table (available: {options or 'none'})")
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

    lines = []
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
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("field_log", type=Path)
    parser.add_argument("--latex-output", type=Path, help="write rows for the JINT formal-results table")
    parser.add_argument("--configuration", help="configuration ID for the LaTeX table")
    parser.add_argument("--condition-ids", nargs=4, default=DEFAULT_CONDITION_IDS,
                        metavar=("ORIGINAL_DAY", "ORIGINAL_NIGHT", "ARTIFICIAL", "BRIDGE"))
    parser.add_argument("--layout-ids", nargs=2, default=DEFAULT_LAYOUT_IDS, metavar=("BOX", "BOX_CHAIR"))
    args = parser.parse_args()
    try:
        trials = read_main_trials(args.field_log)
        if args.latex_output:
            config = export_latex(trials, args.latex_output, args.configuration,
                                  args.condition_ids, args.layout_ids)
            print(f"LaTeX rows for {config}: {args.latex_output}\n")
    except (OSError, ValueError) as error:
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
