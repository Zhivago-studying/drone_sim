#!/usr/bin/env python3
"""Summarize DGO relative-velocity source traces against Gazebo ground truth."""

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


AXES = ("x", "y", "z")


def discover(inputs):
    paths = []
    for value in inputs:
        path = Path(value)
        if path.is_dir():
            paths.extend(path.rglob("iris_*_dgo_velocity_source_trace.csv"))
        elif path.is_file():
            paths.append(path)
    return sorted(set(paths))


def finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def summarize(rows, key):
    errors = np.asarray([[row["err_" + axis] for axis in AXES] for row in rows])
    norms = np.linalg.norm(errors, axis=1)
    pair_dt = np.abs(np.asarray([row["pair_dt"] for row in rows]))
    result = dict(zip(("self_id", "target_id", "stage"), key))
    result["samples"] = len(rows)
    for index, axis in enumerate(AXES):
        result["mean_err_" + axis] = float(np.mean(errors[:, index]))
        result["rmse_err_" + axis] = float(np.sqrt(np.mean(errors[:, index] ** 2)))
    result["rmse_norm"] = float(np.sqrt(np.mean(norms ** 2)))
    result["p95_norm"] = float(np.percentile(norms, 95))
    result["mean_abs_pair_dt"] = float(np.mean(pair_dt))
    result["p95_abs_pair_dt"] = float(np.percentile(pair_dt, 95))
    result["max_abs_pair_dt"] = float(np.max(pair_dt))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", help="Trace CSV files or directories")
    parser.add_argument("--output", type=Path,
                        help="Write summary CSV instead of only printing it")
    args = parser.parse_args()

    groups = defaultdict(list)
    paths = discover(args.inputs)
    for path in paths:
        with path.open(newline="", encoding="utf-8") as stream:
            for raw in csv.DictReader(stream):
                self_id = finite(raw.get("self_id"))
                target_id = finite(raw.get("target_id"))
                stage = finite(raw.get("stage"))
                pair_dt = finite(raw.get("pair_dt"))
                errors = [finite(raw.get("err_v" + axis)) for axis in AXES]
                if None in [self_id, target_id, stage, pair_dt] + errors:
                    continue
                row = {"pair_dt": pair_dt}
                row.update({"err_" + axis: value for axis, value in zip(AXES, errors)})
                groups[(int(self_id), int(target_id), int(stage))].append(row)

    summaries = [summarize(groups[key], key) for key in sorted(groups)]
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if summaries:
            with args.output.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
                writer.writeheader()
                writer.writerows(summaries)

    print("trace files: {}".format(len(paths)))
    print("valid samples: {}".format(sum(row["samples"] for row in summaries)))
    for row in summaries:
        print(
            "{}-{} stage {} n={} mean=({:.3f},{:.3f},{:.3f}) "
            "rmse_norm={:.3f} p95_norm={:.3f} pair_dt_p95={:.4f}s".format(
                row["target_id"], row["self_id"], row["stage"], row["samples"],
                row["mean_err_x"], row["mean_err_y"], row["mean_err_z"],
                row["rmse_norm"], row["p95_norm"], row["p95_abs_pair_dt"],
            )
        )


if __name__ == "__main__":
    main()
