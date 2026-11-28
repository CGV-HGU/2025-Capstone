"""Build manuscript tables from the prospective paired navigation field sheet."""

import argparse
from collections import Counter, defaultdict
import csv
import math
from pathlib import Path
from statistics import median


SITES = {"palgakjeong": "Reference hall", "class5": "Reading-room corridor"}
PUBLIC_COLUMNS = (
    "trial_id", "pair_id", "site", "planned_order", "planned_method", "actual_method",
    "configuration_id", "obstacle_passed", "contact", "manual_intervention",
    "end_distance_measured_m", "physical_outcome", "nav2_status", "nav2_elapsed_s",
)
DEPARTURE_FIELDS = (
    "end_time", "physical_outcome", "obstacle_passed", "contact", "manual_intervention",
    "end_distance_measured_m", "nav2_status", "nav2_elapsed_s", "end_reason", "result_json", "bag_path",
)


def nonnegative(value):
    if not value:
        return None
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise ValueError(f"expected a finite nonnegative measurement, got {value!r}")
    return number


def check_attempt(row):
    for key in ("obstacle_passed", "contact", "manual_intervention"):
        if row[key] not in {"yes", "no", "unknown", ""}:
            raise ValueError(f"{key} must be yes, no, or unknown")
    distance = nonnegative(row["end_distance_measured_m"])
    nonnegative(row["nav2_elapsed_s"])
    known_failure = (row["obstacle_passed"] == "no" or row["contact"] == "yes"
                     or row["manual_intervention"] == "yes" or (distance is not None and distance > 0.50))
    known_success = (row["obstacle_passed"] == "yes" and row["contact"] == "no"
                     and row["manual_intervention"] == "no" and distance is not None and distance <= 0.50)
    verdict = row["physical_outcome"]
    if not row["actual_method"] or row["actual_method"].casefold() == "comparator":
        raise ValueError("enter the name of the method actually used")
    witnessed = row["observer"] or row["video_file"]
    if verdict in {"success", "failure"}:
        if not witnessed:
            raise ValueError("an adjudicated outcome needs an observer or external video reference")
        if not row["end_time"]:
            raise ValueError("an adjudicated outcome needs an end time")
        if (verdict == "success" and not known_success) or (verdict == "failure" and not known_failure):
            raise ValueError("field observations do not support the stated physical outcome")
    elif verdict == "unassessable" and witnessed and (known_success or known_failure):
        raise ValueError("field observations determine an outcome; reconcile the unassessable verdict")
    elif verdict not in {"", "unassessable"}:
        raise ValueError("unknown physical_outcome")


def read_attempts(path):
    attempts, seen = [], set()
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        required = set(PUBLIC_COLUMNS) | set(DEPARTURE_FIELDS) | {"start_time", "observer", "video_file"}
        columns = reader.fieldnames or []
        if len(set(columns)) != len(columns):
            raise ValueError("duplicate CSV column names")
        missing = required.difference(columns)
        if missing:
            raise ValueError("missing CSV columns: " + ", ".join(sorted(missing)))
        for row_number, raw in enumerate(reader, 2):
            if None in raw or any(value is None for value in raw.values()):
                raise ValueError(f"line {row_number}: number of cells does not match the header")
            row = {k: (raw[k] or "").strip() for k in required}
            if not any(row.values()):
                continue
            trial = row["trial_id"]
            if not trial or trial in seen:
                raise ValueError(f"line {row_number}: empty or duplicate trial ID {trial!r}")
            seen.add(trial)
            if not row["start_time"]:
                if any(row[k] for k in DEPARTURE_FIELDS):
                    raise ValueError(f"{trial}: outcome or run data present without a start time")
                continue
            try:
                if row["site"] not in SITES or not row["pair_id"]:
                    raise ValueError("expected a planned site and pair ID")
                check_attempt(row)
            except ValueError as error:
                raise ValueError(f"{trial}: {error}") from error
            attempts.append(row)
    if not any(row["physical_outcome"] in {"success", "failure"} for row in attempts):
        raise ValueError("no externally adjudicated attempts yet; build the archived manuscript without --paired-log")
    return attempts


def tex(value):
    escapes = {"\\": r"\textbackslash{}", "~": r"\textasciitilde{}", "^": r"\textasciicircum{}"}
    return "".join(escapes.get(ch, "\\" + ch if ch in "#$%&_{}" else ch) for ch in value)


def metric(rows, key, digits):
    values = [float(row[key]) for row in rows if row[key]]
    if not values:
        return "---"
    return f"{len(values)}: {median(values):.{digits}f} [{min(values):.{digits}f}, {max(values):.{digits}f}]"


def make_tex(attempts):
    groups, pairs = defaultdict(list), defaultdict(dict)
    names = {}
    for row in attempts:
        method, site = row["actual_method"], row["site"]
        arm = "V-LiDAR" if method.casefold() == "v-lidar" else "Comparator"
        if arm == "Comparator":
            names[method.casefold()] = method
        pair = pairs[row["pair_id"]]
        if pair and next(iter(pair.values()))["site"] != site:
            raise ValueError(f"{row['pair_id']}: paired attempts have different sites")
        if arm in pair:
            raise ValueError(f"{row['pair_id']}: repeated {arm} in the same pair")
        pair[arm] = row
        groups[site, arm].append(row)
    if len(names) > 1:
        raise ValueError("multiple comparator methods were used; report these configurations separately")
    comparator = tex(next(iter(names.values()))) if names else "Comparator (not yet run)"
    outcomes, measures, pair_rows = [], [], []
    for site, label in SITES.items():
        site_pairs = [p for p in pairs.values() if next(iter(p.values()))["site"] == site]
        if not site_pairs:
            continue
        counts = Counter()
        for pair in site_pairs:
            key = tuple(pair.get(arm, {}).get("physical_outcome") for arm in ("V-LiDAR", "Comparator"))
            counts[key] += 1
        resolved_counts = [counts[k] for k in (("success", "success"), ("success", "failure"),
                                               ("failure", "success"), ("failure", "failure"))]
        pair_rows.append([label, len(site_pairs), *resolved_counts, len(site_pairs) - sum(resolved_counts)])
        for arm in ("V-LiDAR", "Comparator"):
            rows = groups.get((site, arm), [])
            if not rows:
                continue
            successes = [row for row in rows if row["physical_outcome"] == "success"]
            failures = sum(row["physical_outcome"] == "failure" for row in rows)
            observed = []
            for key in ("contact", "manual_intervention"):
                known = [r[key] for r in rows if r[key] in {"yes", "no"}]
                observed.append(f"{known.count('yes')}/{len(known)}" if known else "---")
            outcomes.append([label, arm, len(rows), len(successes), failures,
                             len(rows) - failures - len(successes), *observed])
            measures.append([label, arm, metric(rows, "end_distance_measured_m", 2),
                             metric(successes, "nav2_elapsed_s", 1)])
    arm_summaries = []
    for arm, label in (("V-LiDAR", "V-LiDAR"), ("Comparator", comparator)):
        runs = [r for (_, method), group in groups.items() if method == arm for r in group]
        if runs:
            success = sum(r["physical_outcome"] == "success" for r in runs)
            noun = "attempt" if len(runs) == 1 else "attempts"
            arm_summaries.append(f"{label} in {success} of {len(runs)} {noun}")
    summary = " Separate field observations documented unassisted, contact-free passage and goal arrival for "
    summary += " and ".join(arm_summaries) + "."
    open_runs = sum(r["physical_outcome"] not in {"success", "failure"} for r in attempts)
    if open_runs:
        noun = "outcome" if open_runs == 1 else "outcomes"
        summary += f" The field records contained {open_runs} unresolved {noun}."
    macros = [r"\newcommand{\JINTComparator}{" + comparator + "}",
              r"\newcommand{\JINTPairedAbstract}{" + summary + "}"]
    for name, content in (("Outcomes", outcomes), ("Measures", measures), ("Pairs", pair_rows)):
        lines = [" & ".join(map(str, row)) + r" \\" for row in content]
        macros.append(r"\newcommand{\JINTPaired" + name + "}{%\n" + "\n".join(lines) + "\n}")
    return "\n".join(macros) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("field_log", type=Path)
    parser.add_argument("--tex", type=Path, required=True)
    parser.add_argument("--public-csv", type=Path, required=True)
    args = parser.parse_args()
    try:
        attempts = read_attempts(args.field_log)
        tables = make_tex(attempts)
    except (ValueError, OSError) as error:
        parser.exit(1, f"{error}\n")
    args.tex.write_text(tables, encoding="utf-8")
    with args.public_csv.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, PUBLIC_COLUMNS, extrasaction="ignore", lineterminator="\n")
        writer.writeheader()
        writer.writerows(attempts)


if __name__ == "__main__":
    main()
