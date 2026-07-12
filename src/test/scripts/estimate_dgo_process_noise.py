#!/usr/bin/env python3
"""Estimate DGO measurement noise and CV-model acceleration noise from runs."""

import argparse
import csv
import json
import math
import re
from collections import defaultdict
from pathlib import Path

import numpy as np


AXES = ("x", "y", "z")
STAGE_GROUPS = {
    1: "takeoff",
    2: "low_dynamic",
    3: "high_dynamic",
    4: "high_dynamic",
    5: "high_dynamic",
    6: "landing",
}
PAIR_RE = re.compile(r"iris_(\d+)_relative_to_iris_(\d+)_dgo_error\.csv$")


def parse_runs(spec):
    runs = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            lo, hi = (int(value) for value in part.split("-", 1))
            runs.extend(range(lo, hi + 1))
        else:
            runs.append(int(part))
    return sorted(set(runs))


def sample_std(values):
    values = np.asarray(values, dtype=float)
    return float(np.std(values, ddof=1)) if values.size >= 2 else math.nan


def rms(values):
    values = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(np.square(values)))) if values.size else math.nan


def read_csv(path, required):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = sorted(set(required) - set(reader.fieldnames or []))
        if missing:
            raise ValueError("{} missing columns: {}".format(path, ", ".join(missing)))
        return list(reader)


def load_dgo(path):
    required = [
        "timestamp", "mission_stage", "recording_active",
        "x_gt", "y_gt", "z_gt", "x_est", "y_est", "z_est",
        "err_x", "err_y", "err_z",
    ]
    samples = []
    for raw in read_csv(path, required):
        try:
            row = {key: float(raw[key]) for key in required}
        except ValueError:
            continue
        if not all(math.isfinite(value) for value in row.values()):
            continue
        stage = int(row["mission_stage"])
        if int(row["recording_active"]) != 1 or stage not in STAGE_GROUPS:
            continue
        row["mission_stage"] = stage
        row["timestamp"] = float(row["timestamp"])
        for axis in AXES:
            row["e_" + axis] = row[axis + "_est"] - row[axis + "_gt"]
        row["e_norm"] = math.sqrt(sum(row["e_" + axis] ** 2 for axis in AXES))
        samples.append(row)
    # Keep the final row at duplicate timestamps.
    return list({row["timestamp"]: row for row in sorted(samples, key=lambda x: x["timestamp"])}.values())


def position_stats(samples, run, pair, key, value):
    result = {"run": run, "pair": pair, key: value, "n_samples": len(samples)}
    for axis in AXES:
        errors = np.asarray([row["e_" + axis] for row in samples])
        result["_raw_e_" + axis] = errors
        result["mean_" + axis] = float(np.mean(errors))
        result["std_" + axis] = sample_std(errors)
        result["rmse_" + axis] = rms(errors)
    norms = np.asarray([row["e_norm"] for row in samples])
    result["_raw_e_norm"] = norms
    result["rmse_norm"] = rms(norms)
    result["p95_norm"] = float(np.percentile(norms, 95))
    result["max_norm"] = float(np.max(norms))
    return result


def load_velocity(path):
    required = ["timestamp", "gt_vx", "gt_vy", "gt_vz"]
    samples = {}
    for raw in read_csv(path, required):
        try:
            timestamp = float(raw["timestamp"])
            velocity = np.asarray([float(raw["gt_v" + axis]) for axis in AXES])
        except ValueError:
            continue
        if math.isfinite(timestamp) and np.isfinite(velocity).all():
            samples[timestamp] = velocity
    timestamps = np.asarray(sorted(samples), dtype=float)
    velocities = np.asarray([samples[timestamp] for timestamp in timestamps], dtype=float)
    return timestamps, velocities


def nearest_velocity(query_times, velocity_series, tolerance):
    timestamps, velocities = velocity_series
    result = np.full((len(query_times), 3), np.nan)
    offsets = np.full(len(query_times), np.nan)
    if timestamps.size == 0:
        return result, offsets
    right = np.searchsorted(timestamps, query_times, side="left")
    left = np.maximum(right - 1, 0)
    right = np.minimum(right, timestamps.size - 1)
    left_dt = np.abs(timestamps[left] - query_times)
    right_dt = np.abs(timestamps[right] - query_times)
    indices = np.where(left_dt <= right_dt, left, right)
    nearest_dt = np.abs(timestamps[indices] - query_times)
    good = nearest_dt <= tolerance
    result[good] = velocities[indices[good]]
    offsets[good] = nearest_dt[good]
    return result, offsets


def acceleration_samples(dgo, target_series, reference_series, tolerance,
                         min_dt, max_dt):
    timestamps = np.asarray([row["timestamp"] for row in dgo])
    stages = np.asarray([row["mission_stage"] for row in dgo], dtype=int)
    positions = np.asarray([[row[axis + "_gt"] for axis in AXES] for row in dgo])
    target_velocity, target_dt = nearest_velocity(timestamps, target_series, tolerance)
    reference_velocity, reference_dt = nearest_velocity(timestamps, reference_series, tolerance)
    relative_velocity = target_velocity - reference_velocity
    aligned = np.isfinite(relative_velocity).all(axis=1)
    dt = np.diff(timestamps, prepend=np.nan)
    same_stage = np.zeros(len(dgo), dtype=bool)
    same_stage[1:] = stages[1:] == stages[:-1]
    valid_dt = (dt >= min_dt) & (dt <= max_dt)
    valid = same_stage & valid_dt & aligned
    valid[1:] &= aligned[:-1]
    valid[0] = False

    samples = []
    for index in np.flatnonzero(valid):
        rv = relative_velocity[index] - relative_velocity[index - 1]
        rp = positions[index] - positions[index - 1] - relative_velocity[index - 1] * dt[index]
        row = {"mission_stage": int(stages[index]), "dt": float(dt[index])}
        for axis_index, axis in enumerate(AXES):
            row["av_" + axis] = float(rv[axis_index] / dt[index])
            row["ap_" + axis] = float(2.0 * rp[axis_index] / (dt[index] ** 2))
        samples.append(row)
    diagnostics = {
        "velocity_aligned_rows": int(np.count_nonzero(aligned)),
        "valid_accel_rows": len(samples),
        "rejected_stage_boundary": int(np.count_nonzero(~same_stage)),
        "rejected_dt": int(np.count_nonzero(~valid_dt)),
        "target_align_p95_s": float(np.percentile(target_dt[np.isfinite(target_dt)], 95)),
        "target_align_max_s": float(np.nanmax(target_dt)),
        "reference_align_p95_s": float(np.percentile(reference_dt[np.isfinite(reference_dt)], 95)),
        "reference_align_max_s": float(np.nanmax(reference_dt)),
    }
    return samples, diagnostics


def acceleration_stats(samples, run, pair, key, value):
    result = {
        "run": run, "pair": pair, key: value, "n_samples": len(samples),
        "mean_dt": float(np.mean([row["dt"] for row in samples])),
    }
    result["_raw_dt"] = np.asarray([row["dt"] for row in samples])
    for method in ("av", "ap"):
        suffix = "from_v" if method == "av" else "from_p"
        for axis in AXES:
            values = np.asarray([row[method + "_" + axis] for row in samples])
            result["_raw_" + method + "_" + axis] = values
            result["mean_a" + axis + "_" + suffix] = float(np.mean(values))
            result["std_a" + axis + "_" + suffix] = sample_std(values)
            result["rms_a" + axis + "_" + suffix] = rms(values)
    return result


def split_by(samples, key_function):
    groups = defaultdict(list)
    for row in samples:
        groups[key_function(row)].append(row)
    return groups


def pooled_summary(detail, group_columns, value_columns):
    groups = split_by(detail, lambda row: tuple(row[column] for column in group_columns))
    rows = []
    for keys in sorted(groups, key=lambda value: tuple(str(item) for item in value)):
        group = groups[keys]
        row = dict(zip(group_columns, keys))
        row["n_runs"] = len(set(item["run"] for item in group))
        row["n_samples"] = sum(item["n_samples"] for item in group)
        for column in value_columns:
            values = np.asarray([item[column] for item in group], dtype=float)
            weights = np.asarray([item["n_samples"] for item in group], dtype=float)
            good = np.isfinite(values) & (weights > 0)
            row[column + "_run_mean"] = float(np.mean(values[good])) if good.any() else math.nan
            raw_key = None
            operation = None
            position_match = re.fullmatch(r"(mean|std|rmse)_([xyz])", column)
            accel_match = re.fullmatch(r"(mean|std|rms)_a([xyz])_from_([vp])", column)
            if position_match:
                operation, axis = position_match.groups()
                raw_key = "_raw_e_" + axis
            elif column in ("rmse_norm", "p95_norm", "max_norm"):
                raw_key = "_raw_e_norm"
                operation = column.split("_", 1)[0]
            elif column == "mean_dt":
                raw_key = "_raw_dt"
                operation = "mean"
            elif accel_match:
                operation, axis, source = accel_match.groups()
                raw_key = "_raw_a{}_{}".format(source, axis)

            if raw_key and all(raw_key in item for item in group):
                raw = np.concatenate([item[raw_key] for item in group])
                if operation == "mean":
                    row[column] = float(np.mean(raw))
                elif operation == "std":
                    row[column] = sample_std(raw)
                elif operation in ("rmse", "rms"):
                    row[column] = rms(raw)
                elif operation == "p95":
                    row[column] = float(np.percentile(raw, 95))
                elif operation == "max":
                    row[column] = float(np.max(raw))
            else:
                row[column] = float(np.average(values[good], weights=weights[good])) if good.any() else math.nan
            row[column + "_run_sd"] = sample_std(values[np.isfinite(values)])
        rows.append(row)
    return rows


def add_bootstrap(summary, detail, group_columns, value_columns, iterations, seed):
    rng = np.random.default_rng(seed)
    groups = split_by(detail, lambda row: tuple(row[column] for column in group_columns))
    indexed_summary = {tuple(row[column] for column in group_columns): row for row in summary}
    for keys, group in groups.items():
        output = indexed_summary[keys]
        for column in value_columns:
            per_run = defaultdict(list)
            for item in group:
                if math.isfinite(item[column]):
                    per_run[item["run"]].append(item[column])
            values = np.asarray([np.mean(values) for values in per_run.values()])
            if values.size < 2 or iterations <= 0:
                low = high = math.nan
            else:
                indices = rng.integers(0, values.size, size=(iterations, values.size))
                low, high = (float(value) for value in np.percentile(values[indices].mean(axis=1), [2.5, 97.5]))
            output[column + "_run_mean_ci95_low"] = low
            output[column + "_run_mean_ci95_high"] = high


def detect_dgo_outlier_runs(position_group_detail):
    scores = defaultdict(list)
    for row in position_group_detail:
        if row["stage_group"] == "high_dynamic":
            scores[row["run"]].append(max(row["std_x"], row["std_y"]))
    run_scores = {run: max(values) for run, values in scores.items()}
    values = np.asarray(list(run_scores.values()), dtype=float)
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    threshold = median + 4.0 * 1.4826 * mad
    rows = []
    outliers = set()
    for run, score in sorted(run_scores.items()):
        flagged = score > threshold
        if flagged:
            outliers.add(run)
        rows.append({
            "run": run,
            "high_dynamic_max_horizontal_std": score,
            "robust_median": median,
            "robust_mad": mad,
            "outlier_threshold": threshold,
            "dgo_drift_outlier": int(flagged),
        })
    return rows, outliers


def ceil_step(value, step=0.01):
    return math.ceil((value - 1.0e-12) / step) * step


def write_csv(path, rows):
    if not rows:
        path.write_text("", encoding="ascii")
        return
    columns = []
    for row in rows:
        for column in row:
            if not column.startswith("_") and column not in columns:
                columns.append(column)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, default=Path("run_data"))
    parser.add_argument("--runs", default="89-112")
    parser.add_argument("--output", type=Path,
                        default=Path("run_data/noise_calibration/run_89_112"))
    parser.add_argument("--velocity-align-tolerance", type=float, default=0.03)
    parser.add_argument("--min-dt", type=float, default=0.05)
    parser.add_argument("--max-dt", type=float, default=0.15)
    parser.add_argument("--bootstrap-runs", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=20260712)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    runs = parse_runs(args.runs)
    manifest = []
    position_stage_detail = []
    position_group_detail = []
    acceleration_stage_detail = []
    acceleration_group_detail = []
    alignment_report = []

    for run in runs:
        run_dir = args.run_root / ("run_{}".format(run))
        dgo_paths = sorted((run_dir / "ekf_dgo_test").glob("*_dgo_error.csv"))
        if not dgo_paths:
            manifest.append({"run": run, "status": "missing_dgo", "path": str(run_dir)})
            continue
        velocities = {}
        for uav in range(4):
            path = run_dir / "ins_eskf_test" / ("iris_{}_vel_error.csv".format(uav))
            if path.exists():
                velocities[uav] = load_velocity(path)

        for path in dgo_paths:
            match = PAIR_RE.search(path.name)
            if not match:
                continue
            target, reference = (int(value) for value in match.groups())
            pair = "{}-{}".format(target, reference)
            dgo = load_dgo(path)
            sign_delta = max(
                max(abs(row["err_" + axis] + row["e_" + axis]) for row in dgo)
                for axis in AXES
            )
            manifest.append({
                "run": run, "pair": pair, "status": "ok", "path": str(path),
                "n_samples": len(dgo),
                "stages": " ".join(str(stage) for stage in sorted(set(row["mission_stage"] for row in dgo))),
                "legacy_error_sign_max_delta": sign_delta,
            })
            for stage, samples in split_by(dgo, lambda row: row["mission_stage"]).items():
                position_stage_detail.append(position_stats(samples, run, pair, "stage", stage))
            for group, samples in split_by(dgo, lambda row: STAGE_GROUPS[row["mission_stage"]]).items():
                position_group_detail.append(position_stats(samples, run, pair, "stage_group", group))

            if target not in velocities or reference not in velocities:
                alignment_report.append({"run": run, "pair": pair, "dgo_rows": len(dgo),
                                         "valid_accel_rows": 0, "status": "missing_velocity"})
                continue
            accel, diagnostics = acceleration_samples(
                dgo, velocities[target], velocities[reference],
                args.velocity_align_tolerance, args.min_dt, args.max_dt,
            )
            diagnostics.update({"run": run, "pair": pair, "dgo_rows": len(dgo), "status": "ok"})
            alignment_report.append(diagnostics)
            for stage, samples in split_by(accel, lambda row: row["mission_stage"]).items():
                acceleration_stage_detail.append(acceleration_stats(samples, run, pair, "stage", stage))
            for group, samples in split_by(accel, lambda row: STAGE_GROUPS[row["mission_stage"]]).items():
                acceleration_group_detail.append(acceleration_stats(samples, run, pair, "stage_group", group))

    write_csv(args.output / "input_manifest.csv", manifest)
    write_csv(args.output / "alignment_report.csv", alignment_report)
    write_csv(args.output / "sigma_p_run_stage_pair.csv", position_stage_detail)
    write_csv(args.output / "sigma_p_run_stage_group_pair.csv", position_group_detail)
    write_csv(args.output / "sigma_a_run_stage_pair.csv", acceleration_stage_detail)
    write_csv(args.output / "sigma_a_run_stage_group_pair.csv", acceleration_group_detail)
    quality_rows, dgo_outlier_runs = detect_dgo_outlier_runs(position_group_detail)
    write_csv(args.output / "run_quality_flags.csv", quality_rows)

    position_values = (["mean_" + axis for axis in AXES]
                       + ["std_" + axis for axis in AXES]
                       + ["rmse_norm", "p95_norm", "max_norm"])
    acceleration_values = ["mean_dt"] + [
        metric + "_a" + axis + "_" + suffix
        for metric in ("mean", "std", "rms")
        for suffix in ("from_v", "from_p")
        for axis in AXES
    ]
    summary_specs = [
        (position_stage_detail, ["stage", "pair"], position_values, "sigma_p_stage_pair.csv"),
        (position_group_detail, ["stage_group", "pair"], position_values, "sigma_p_stage_group_pair.csv"),
        (acceleration_stage_detail, ["stage", "pair"], acceleration_values, "sigma_a_stage_pair.csv"),
        (acceleration_group_detail, ["stage_group", "pair"], acceleration_values, "sigma_a_stage_group_pair.csv"),
    ]
    summaries = {}
    for detail, group_columns, values, filename in summary_specs:
        summary = pooled_summary(detail, group_columns, values)
        add_bootstrap(summary, detail, group_columns,
                      [value for value in values if value.startswith("std_")],
                      args.bootstrap_runs, args.seed)
        write_csv(args.output / filename, summary)
        summaries[filename] = summary

    recommended = {"source_runs": args.runs, "empirical_pooled": {}}
    p_summary = summaries["sigma_p_stage_group_pair.csv"]
    a_summary = summaries["sigma_a_stage_group_pair.csv"]
    robust_position_detail = [
        row for row in position_group_detail if row["run"] not in dgo_outlier_runs
    ]
    robust_p_summary = pooled_summary(
        robust_position_detail, ["stage_group", "pair"], position_values
    )
    write_csv(args.output / "sigma_p_stage_group_pair_robust.csv", robust_p_summary)
    recommended["dgo_drift_outlier_runs"] = sorted(dgo_outlier_runs)
    recommended["filter_recommendation"] = {"stage_groups": {}}
    for stage_group in ("takeoff", "low_dynamic", "high_dynamic", "landing"):
        p_rows = [row for row in p_summary if row["stage_group"] == stage_group]
        a_rows = [row for row in a_summary if row["stage_group"] == stage_group]
        if not p_rows or not a_rows:
            continue
        recommended["empirical_pooled"][stage_group] = {
            "dgo_position_noise_std_xyz": [max(row["std_" + axis] for row in p_rows) for axis in AXES],
            "accel_noise_std_xyz_from_v": [max(row["std_a" + axis + "_from_v"] for row in a_rows) for axis in AXES],
            "accel_noise_std_xyz_from_p": [max(row["std_a" + axis + "_from_p"] for row in a_rows) for axis in AXES],
        }
        robust_p_rows = [
            row for row in robust_p_summary if row["stage_group"] == stage_group
        ]
        recommended["filter_recommendation"]["stage_groups"][stage_group] = {
            "dgo_position_noise_std_xyz": [
                ceil_step(1.5 * max(row["std_" + axis] for row in robust_p_rows))
                for axis in AXES
            ],
            "accel_noise_std_xyz": [
                ceil_step(1.1 * max(row["std_a" + axis + "_from_v"] for row in a_rows))
                for axis in AXES
            ],
        }
    filter_groups = recommended["filter_recommendation"]["stage_groups"]
    recommended["filter_recommendation"]["isotropic_current_interface"] = {
        "dgo_position_noise_std": max(
            max(values["dgo_position_noise_std_xyz"])
            for values in filter_groups.values()
        ),
        "accel_noise_std": max(
            max(values["accel_noise_std_xyz"])
            for values in filter_groups.values()
        ),
    }
    (args.output / "recommended_noise.yaml").write_text(
        json.dumps(recommended, indent=2, sort_keys=True) + "\n", encoding="ascii"
    )

    print("runs requested: {}".format(len(runs)))
    print("DGO files processed: {}".format(sum(row["status"] == "ok" for row in manifest)))
    print("output: {}".format(args.output.resolve()))


if __name__ == "__main__":
    main()
