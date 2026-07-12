#!/usr/bin/env python3
"""
Plot DEKF vs DGO vs EKF mean relative position error to iris_0.

Inputs (run_data/run_X/):
  dekf/  — iris_{1,2,3}_relative_to_iris_0_dekf_error.csv
  ekf_dgo_test/ — iris_{1,2,3}_relative_to_iris_0_dgo_error.csv
  ins_eskf_test/ — iris_{1,2,3}_relative_to_iris_0_ekf_error.csv

Output:
  mean_relative_error_dekf_dgo_ekf.png
"""

import argparse
import glob
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
OLD_DEKF_LOG_DIR = os.path.join(REPO_ROOT, "src", "test", "logs")
OLD_DGO_LOG_DIR = os.path.join(REPO_ROOT, "src", "test", "logs")
OLD_EKF_LOG_DIR = os.path.join(REPO_ROOT, "src", "data_process", "logs")
RUN_LOG_BASE = os.path.join(REPO_ROOT, "run_data")
FIGURE_BASE = os.path.join(RUN_LOG_BASE, "figure")


REFERENCE = "iris_0"
DRONES = ["iris_1", "iris_2", "iris_3"]

# 飞行窗口过滤
STAGES_EXCLUDE = {0, 7}   # WAIT, DONE — 始终排除
PRE_FLIGHT = -1           # DGO 启动之前的时间戳

# 方法配置
METHODS = ["dekf", "dgo", "ekf"]
METHOD_COLORS = {"dekf": "#2ca02c", "dgo": "#d62728", "ekf": "#1f77b4"}
METHOD_LABELS = {"dekf": "DEKF", "dgo": "DGO", "ekf": "EKF"}
METHOD_CATEGORIES = {
    "dekf": "dekf",
    "dgo": "ekf_dgo_test",
    "ekf": "ins_eskf_test",
}
METHOD_OLD_DEFAULTS = {
    "dekf": OLD_DEKF_LOG_DIR,
    "dgo": OLD_DGO_LOG_DIR,
    "ekf": OLD_EKF_LOG_DIR,
}


def load_stage_map(log_dir):
    """从 DGO sync_diag 加载阶段映射.

    返回 {stamp, stage} dict, 退化返回 None.
    """
    path = os.path.join(log_dir, "sensor_sync_logs",
                        f"{REFERENCE}_dgo_sync_diag.csv")
    if not os.path.exists(path):
        return None

    try:
        df = pd.read_csv(path)
    except Exception:
        return None

    if "mission_stage" not in df.columns or len(df) < 2:
        return None

    ss = df[["stamp", "mission_stage"]].dropna().sort_values("stamp")
    if ss.empty:
        return None
    return {"stamp": ss["stamp"].to_numpy(dtype=float),
            "stage": ss["mission_stage"].to_numpy(dtype=int)}


def forward_fill_stage(timestamps, stage_map):
    """Forward-fill: 每个 timestamp → 最近前一个 stage. Pre-flight = -1."""
    if stage_map is None:
        return None
    idx = np.searchsorted(stage_map["stamp"], timestamps, side="right") - 1
    return np.where(idx >= 0, stage_map["stage"][idx], PRE_FLIGHT)


def filter_flight_window(df, stage_map, time_col="timestamp"):
    """过滤 DataFrame, 仅保留飞行窗口内的行."""
    if stage_map is None:
        return df
    tt = df[time_col].to_numpy(dtype=float)
    stages = forward_fill_stage(tt, stage_map)
    if stages is None:
        return df
    mask = np.array([int(s) not in STAGES_EXCLUDE and int(s) != PRE_FLIGHT
                     for s in stages])
    kept = mask.sum()
    if kept == len(df):
        return df
    if kept == 0:
        print(f"    flight-window: 0 samples kept, returning empty df")
        return df.iloc[:0].copy()
    print(f"    flight-window: {kept}/{len(df)} samples kept")
    return df[mask].copy()


def resolve_log_dir(run_id, category, old_default):
    if not run_id or run_id == "auto":
        run_dirs = sorted(
            glob.glob(os.path.join(RUN_LOG_BASE, "run_*")),
            key=lambda d: int(os.path.basename(d).split("_")[1]))
        if run_dirs:
            path = os.path.join(run_dirs[-1], category)
            if os.path.isdir(path):
                return path
        return old_default
    rid = str(run_id).replace("run_", "")
    path = os.path.join(RUN_LOG_BASE, f"run_{rid}", category)
    if os.path.isdir(path):
        return path
    print(f"  Warning: {path} not found, falling back to {old_default}")
    return old_default


def load_relative_csv(log_dir, drone, method):
    path = os.path.join(log_dir, f"{drone}_relative_to_{REFERENCE}_{method}_error.csv")
    if not os.path.exists(path):
        print(f"  Skip missing {method.upper()}: {path}")
        return None

    df = pd.read_csv(path)
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.dropna(subset=["timestamp", "error_norm_m"])
    df = df.sort_values("timestamp").drop_duplicates("timestamp")
    if df.empty:
        print(f"  Skip empty {method.upper()}: {path}")
        return None
    return df


def load_dekf_pairwise_csv(log_dir, drone):
    """Load bidirectional pairwise DEKF error CSV.

    Reads from dekf/pairwise/iris_0_{drone}_bidirectional_dekf_error.csv
    and renames err_sym_norm -> error_norm_m for downstream compatibility.
    """
    pairwise_dir = os.path.join(log_dir, "pairwise")
    path = os.path.join(
        pairwise_dir,
        f"{REFERENCE}_{drone}_bidirectional_dekf_error.csv"
    )
    if not os.path.exists(path):
        print(f"  Skip missing DEKF(pairwise): {path}")
        return None

    df = pd.read_csv(path)
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.dropna(subset=["timestamp", "err_sym_norm"])
    df = df.rename(columns={"err_sym_norm": "error_norm_m"})
    df = df.sort_values("timestamp").drop_duplicates("timestamp")
    if df.empty:
        print(f"  Skip empty DEKF(pairwise): {path}")
        return None
    return df


def method_label(method, dekf_pairwise=False):
    """Return display label for a method, respecting pairwise mode for DEKF."""
    if method == "dekf" and dekf_pairwise:
        return "DEKF(pairwise)"
    return METHOD_LABELS[method]


def load_method(log_dir, method, stage_map=None, dekf_pairwise=False):
    result = {}
    for drone in DRONES:
        if method == "dekf" and dekf_pairwise:
            df = load_dekf_pairwise_csv(log_dir, drone)
        else:
            df = load_relative_csv(log_dir, drone, method)
        if df is not None:
            if stage_map is not None:
                df = filter_flight_window(df, stage_map, time_col="timestamp")
            result[drone] = df
            label = method_label(method, dekf_pairwise)
            print(f"  {label} {drone}: {len(df)} samples")
    return result


def build_mean_error(drone_dfs):
    valid = []
    for drone in DRONES:
        df = drone_dfs.get(drone)
        if df is not None and len(df) >= 2:
            valid.append((drone, df))

    if not valid:
        return None

    t_start = max(df["timestamp"].iloc[0] for _, df in valid)
    t_end = min(df["timestamp"].iloc[-1] for _, df in valid)
    if t_end <= t_start:
        return None

    time_axis = np.unique(np.concatenate([
        df.loc[(df["timestamp"] >= t_start) & (df["timestamp"] <= t_end),
               "timestamp"].to_numpy()
        for _, df in valid
    ]))
    if len(time_axis) == 0:
        return None

    errors = []
    for _, df in valid:
        errors.append(np.interp(time_axis,
                                df["timestamp"].to_numpy(),
                                df["error_norm_m"].to_numpy()))
    errors = np.vstack(errors)
    mean_error = errors.mean(axis=0)
    rms_error = np.sqrt((errors ** 2).mean(axis=0))
    return {
        "time": time_axis,
        "mean": mean_error,
        "rms": rms_error,
        "drones": [drone for drone, _ in valid],
    }


def align_methods_window(method_data):
    """Resample all available methods onto a common time axis.

    Input: {"dekf": dekf_mean, "dgo": dgo_mean, "ekf": ekf_mean}
    Returns same dict, with None for missing/unalignable data.
    """
    available = {k: v for k, v in method_data.items() if v is not None}
    if len(available) < 1:
        return method_data

    t_start = max(v["time"][0] for v in available.values())
    t_end = min(v["time"][-1] for v in available.values())
    if t_end <= t_start:
        print("  Warning: methods have no overlapping time window")
        return {k: None for k in method_data}

    time_axis = np.unique(np.concatenate([
        v["time"][(v["time"] >= t_start) & (v["time"] <= t_end)]
        for v in available.values()
    ]))
    if len(time_axis) < 2:
        print("  Warning: too few aligned timestamps")
        return {k: None for k in method_data}

    def resample(data):
        return {
            "time": time_axis,
            "mean": np.interp(time_axis, data["time"], data["mean"]),
            "rms": np.interp(time_axis, data["time"], data["rms"]),
            "drones": data["drones"],
        }

    print(f"  Fair comparison window: {t_start:.3f}s -> {t_end:.3f}s "
          f"({len(time_axis)} aligned timestamps, "
          f"{len(available)} methods)")
    result = {}
    for k in method_data:
        if k in available:
            result[k] = resample(available[k])
        else:
            result[k] = None
    return result


def stats(values):
    values = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(values ** 2))),
        "p95": float(np.quantile(values, 0.95)),
        "max": float(np.max(values)),
    }


def print_summary(name, mean_data, label=None):
    if mean_data is None:
        print(f"{name}: no valid data")
        return

    mean_stats = stats(mean_data["mean"])
    rms_stats = stats(mean_data["rms"])
    tag = f" [{label}]" if label else ""
    print(f"\n{name} relative error summary{tag}")
    if "drones" in mean_data:
        print(f"  runs: {', '.join(str(d) for d in mean_data['drones'])}")
    print(f"  time: {mean_data['time'][0]:.3f}s -> {mean_data['time'][-1]:.3f}s")
    print(f"  mean(norm): mean={mean_stats['mean']:.4f}m "
          f"rmse={mean_stats['rmse']:.4f}m p95={mean_stats['p95']:.4f}m "
          f"max={mean_stats['max']:.4f}m")
    print(f"  rms(norm):  mean={rms_stats['mean']:.4f}m "
          f"rmse={rms_stats['rmse']:.4f}m p95={rms_stats['p95']:.4f}m "
          f"max={rms_stats['max']:.4f}m")


def print_comparison_3way(method_means, dekf_pairwise=False):
    """Print RMSE comparison table across all available methods."""
    rmse = {}
    for m in METHODS:
        data = method_means.get(m)
        if data is not None:
            rmse[m] = stats(data["mean"])["rmse"]

    if len(rmse) < 2:
        print("\nComparison: insufficient methods with aligned data")
        return

    dekf_mode = "bidirectional pairwise" if dekf_pairwise else "anchor-based"
    print(f"\n=== DEKF vs DGO vs EKF fair-window comparison (DEKF mode: {dekf_mode}) ===")
    for m in sorted(rmse, key=lambda x: rmse[x]):
        label = method_label(m, dekf_pairwise)
        print(f"  {label:16s}  mean-relative RMSE: {rmse[m]:.4f}m")

    best = min(rmse, key=lambda x: rmse[x])
    best_label = method_label(best, dekf_pairwise)
    print(f"\n  Best method: {best_label} ({rmse[best]:.4f}m)")

    # Pairwise ratios
    pairs = [("dekf", "dgo"), ("dekf", "ekf"), ("dgo", "ekf")]
    for a, b in pairs:
        if a in rmse and b in rmse and rmse[b] > 0:
            ratio = rmse[a] / rmse[b]
            a_label = method_label(a, dekf_pairwise)
            b_label = method_label(b, dekf_pairwise)
            print(f"  {a_label}/{b_label} ratio: {ratio:.3f}")


# ──────────────────────────────────────────────
#  Plotting
# ──────────────────────────────────────────────

def plot_mean_error_3way(method_means, out_dir, show, dekf_pairwise=False):
    """Single-plot: three methods overlaid."""
    fig, ax = plt.subplots(1, 1, figsize=(14, 5.5))
    dekf_label = method_label("dekf", dekf_pairwise)
    title = f"Mean Relative Position Error to iris_0: {dekf_label} vs DGO vs EKF"
    ax.set_title(title, fontsize=13)

    for m in METHODS:
        data = method_means.get(m)
        if data is None:
            continue
        s = stats(data["mean"])
        color = METHOD_COLORS[m]
        label_name = method_label(m, dekf_pairwise)
        ax.plot(data["time"], data["mean"],
                color=color, linewidth=1.2,
                label=f"{label_name} mean norm  "
                      f"RMSE={s['rmse']:.3f}m  p95={s['p95']:.3f}m")
        ax.plot(data["time"], data["rms"],
                color=color, linewidth=0.7, linestyle="--",
                alpha=0.6, label=f"{label_name} per-time RMS")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Average Relative Position Error (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, ncol=2)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "mean_relative_error_dekf_dgo_ekf.png")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"\nSaved {path}")
    if not show:
        plt.close(fig)


def plot_multi_run_error_3way(method_vars, out_dir, show, run_ids,
                              dekf_pairwise=False):
    """Multi-run overlay: thin per-run lines + bold avg ± 1σ shaded band."""
    fig, ax = plt.subplots(1, 1, figsize=(14, 5.5))
    runs_str = f"run_{run_ids[0]}–run_{run_ids[-1]}"
    dekf_label = method_label("dekf", dekf_pairwise)
    ax.set_title(f"Mean Relative Position Error to iris_0 — {runs_str}",
                 fontsize=13)

    for m in METHODS:
        var = method_vars.get(m)
        if var is None:
            continue
        color = METHOD_COLORS[m]
        label_name = method_label(m, dekf_pairwise)

        # Per-run thin lines
        for i, (rid, curve) in enumerate(var["individual"]):
            alpha = 0.20 + 0.15 * (i / max(len(var["individual"]) - 1, 1))
            ax.plot(var["time"], curve, color=color,
                    linewidth=0.5, alpha=alpha, zorder=1,
                    label=f"{label_name} run_{rid}" if i == 0 else None)

        # ±1σ shaded band
        ax.fill_between(var["time"],
                         var["mean"] - var["std"],
                         var["mean"] + var["std"],
                         color=color, alpha=0.10, zorder=2,
                         label=f"{label_name} ±1σ" if m == "dekf" else None)

        # Average bold line
        rmse = np.sqrt(np.mean(var["mean"] ** 2))
        ax.plot(var["time"], var["mean"], color=color,
                linewidth=2.0, zorder=3,
                label=f"{label_name} avg (RMSE={rmse:.3f}m, "
                      f"N={var['n_runs']} runs)")

    ax.set_xlabel("Elapsed Time (s)")
    ax.set_ylabel("Average Relative Position Error (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=2)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "mean_relative_error_dekf_dgo_ekf.png")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"\nSaved {path}")
    if not show:
        plt.close(fig)


# ──────────────────────────────────────────────
#  Run loading (single-run)
# ──────────────────────────────────────────────

def load_run(run_id, flight_only=False, dekf_pairwise=False):
    """Load DEKF, DGO, EKF for a single run.

    Returns (dekf_mean, dgo_mean, ekf_mean) aligned to common time window.
    """
    log_dirs = {}
    for m in METHODS:
        log_dirs[m] = resolve_log_dir(run_id, METHOD_CATEGORIES[m],
                                      METHOD_OLD_DEFAULTS[m])

    stage_map = None
    if flight_only:
        dgo_debug_dir = resolve_log_dir(run_id, "dgo", "")
        if os.path.isdir(dgo_debug_dir):
            stage_map = load_stage_map(dgo_debug_dir)
            if stage_map is None:
                print("  [Warning] sync_diag empty/unavailable, "
                      "flight-only filter disabled")
        else:
            print("  [Warning] DGO debug dir not found, "
                  "flight-only filter disabled")

    print(f"\n=== run_{run_id} ===")
    method_dfs = {}
    for m in METHODS:
        label = method_label(m, dekf_pairwise)
        print(f"Loading {label} relative CSVs from {log_dirs[m]}")
        method_dfs[m] = load_method(log_dirs[m], m, stage_map=stage_map,
                                    dekf_pairwise=dekf_pairwise)

    method_means = {}
    for m in METHODS:
        method_means[m] = build_mean_error(method_dfs[m])

    # Align to common window
    aligned = align_methods_window(method_means)

    for m in METHODS:
        label = method_label(m, dekf_pairwise)
        print_summary(label, aligned[m])
    print_comparison_3way(aligned, dekf_pairwise=dekf_pairwise)

    return aligned["dekf"], aligned["dgo"], aligned["ekf"]


# ──────────────────────────────────────────────
#  Multi-run aggregation
# ──────────────────────────────────────────────

def build_aggregate_variation(run_results, method):
    """Return per-run aligned series + mean ± 1σ for multi-run plotting."""
    method_idx = METHODS.index(method)
    series = []
    for run_id, result in run_results:
        mean_data = result[method_idx]
        if mean_data is None or len(mean_data["time"]) < 2:
            continue
        duration = mean_data["time"][-1] - mean_data["time"][0]
        if duration < 1.0 or len(mean_data["mean"]) < 10:
            print(f"  Skip {method.upper()} run_{run_id}: "
                  f"too short ({duration:.2f}s, {len(mean_data['mean'])} samples)")
            continue
        elapsed = mean_data["time"] - mean_data["time"][0]
        series.append((run_id, elapsed, mean_data["mean"]))

    if not series:
        return None

    end_time = min(item[1][-1] for item in series)
    if end_time <= 0.0:
        return None

    time_axis = np.unique(np.concatenate([
        elapsed[elapsed <= end_time] for _, elapsed, _ in series
    ]))
    if len(time_axis) < 2:
        return None

    values = np.vstack([
        np.interp(time_axis, elapsed, means)
        for _, elapsed, means in series
    ])
    return {
        "time": time_axis,
        "mean": values.mean(axis=0),
        "std": values.std(axis=0),
        "individual": [
            (run_id, np.interp(time_axis, elapsed, means))
            for run_id, elapsed, means in series
        ],
        "n_runs": len(series),
    }


def aggregate_runs(run_results, method):
    """Average several runs on a common elapsed-time axis."""
    method_idx = METHODS.index(method)
    series = []
    for run_id, result in run_results:
        mean_data = result[method_idx]
        if mean_data is None or len(mean_data["time"]) < 2:
            continue
        duration = mean_data["time"][-1] - mean_data["time"][0]
        if duration < 1.0 or len(mean_data["mean"]) < 10:
            print(f"  Skip {method.upper()} run_{run_id} in aggregate: "
                  f"too short ({duration:.2f}s, {len(mean_data['mean'])} samples)")
            continue
        elapsed = mean_data["time"] - mean_data["time"][0]
        series.append((run_id, elapsed, mean_data["mean"], mean_data["rms"]))

    if not series:
        return None

    end_time = min(item[1][-1] for item in series)
    if end_time <= 0.0:
        return None

    time_axis = np.unique(np.concatenate([
        elapsed[elapsed <= end_time] for _, elapsed, _, _ in series
    ]))
    if len(time_axis) < 2:
        return None

    mean_values = np.vstack([
        np.interp(time_axis, elapsed, mean_values)
        for _, elapsed, mean_values, _ in series
    ])
    rms_values = np.vstack([
        np.interp(time_axis, elapsed, rms_values)
        for _, elapsed, _, rms_values in series
    ])
    return {
        "time": time_axis,
        "mean": mean_values.mean(axis=0),
        "rms": rms_values.mean(axis=0),
        "drones": [f"run_{run_id}" for run_id, _, _, _ in series],
    }


# ──────────────────────────────────────────────
#  main
# ──────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Plot DEKF/DGO/EKF mean relative position error to iris_0.")
    parser.add_argument("--run-id", type=str, default="auto",
                        help="Run ID (number or 'run_N'; 'auto'=latest; "
                             "empty=old defaults)")
    parser.add_argument("--single", type=int, default=None,
                        help="Plot one run ID; equivalent to --run-id N.")
    parser.add_argument("--start-id", type=int, default=None,
                        help="First run ID for multi-run averaging.")
    parser.add_argument("--end-id", type=int, default=None,
                        help="Last run ID for multi-run averaging (inclusive).")
    parser.add_argument("--dekf-log-dir", default=None,
                        help="Override DEKF relative error CSV directory")
    parser.add_argument("--dgo-log-dir", default=None,
                        help="Override DGO relative error CSV directory")
    parser.add_argument("--ekf-log-dir", default=None,
                        help="Override EKF relative error CSV directory")
    parser.add_argument("--out-dir", default=None,
                        help="Output directory for generated figures.")
    parser.add_argument("--no-show", action="store_true",
                        help="Save figures only, do not display windows.")
    parser.add_argument("--flight-only", dest="flight_only",
                        action="store_true", default=True,
                        help="[默认] 仅分析飞行窗口 (排除 WAIT/DONE 阶段)")
    parser.add_argument("--no-flight-only", dest="flight_only",
                        action="store_false",
                        help="使用全部数据 (不排除 WAIT/DONE)")
    parser.add_argument("--dekf-pairwise", action="store_true",
                        help="Use bidirectional pairwise DEKF error CSVs "
                             "instead of legacy anchor-based DEKF CSVs.")
    args = parser.parse_args()

    if (args.start_id is None) != (args.end_id is None):
        parser.error("--start-id and --end-id must be specified together")
    if args.start_id is not None and args.start_id > args.end_id:
        parser.error("--start-id must be <= --end-id")
    if args.single is not None and args.start_id is not None:
        parser.error("--single cannot be combined with --start-id/--end-id")
    if args.start_id is not None and (args.dekf_log_dir or args.dgo_log_dir
                                      or args.ekf_log_dir):
        parser.error("log directory overrides are only supported "
                     "for a single run")

    # ── Multi-run mode ──
    if args.start_id is not None:
        run_results = []
        range_run_ids = []
        for run_id in range(args.start_id, args.end_id + 1):
            run_dir = os.path.join(RUN_LOG_BASE, f"run_{run_id}")
            if not os.path.isdir(run_dir):
                print(f"\nSkip missing run directory: {run_dir}")
                continue
            range_run_ids.append(run_id)
            run_results.append(
                (run_id, load_run(run_id, flight_only=args.flight_only,
                                  dekf_pairwise=args.dekf_pairwise)))

        if not run_results:
            print("No valid runs loaded. Exiting.")
            return

        # 1) Multi-run aggregate plot with variation bands
        method_vars = {}
        for m in METHODS:
            method_vars[m] = build_aggregate_variation(run_results, m)
        if all(v is None for v in method_vars.values()):
            print("No alignable data for multi-run plot. Exiting.")
            return

        # Print per-run summaries
        for rid, (dekf_m, dgo_m, ekf_m) in run_results:
            for m, data in zip(METHODS, [dekf_m, dgo_m, ekf_m]):
                print_summary(method_label(m, args.dekf_pairwise),
                              data, label=f"run_{rid}")

        # Print aggregated summary
        method_agg = {}
        for m in METHODS:
            method_agg[m] = aggregate_runs(run_results, m)
        for m in METHODS:
            print_summary(method_label(m, args.dekf_pairwise),
                          method_agg[m], label="aggregate")
        print_comparison_3way(method_agg, dekf_pairwise=args.dekf_pairwise)

        plot_tag = "dekf_pairwise_dgo_ekf_plot" if args.dekf_pairwise \
                   else "dekf_dgo_ekf_plot"
        run_tag = f"runs_{args.start_id}_to_{args.end_id}"
        out_dir = os.path.abspath(args.out_dir) if args.out_dir else \
                  os.path.join(FIGURE_BASE, run_tag, plot_tag)

        # Multi-run variation plot
        plot_multi_run_error_3way(method_vars, out_dir,
                                  show=not args.no_show,
                                  run_ids=range_run_ids,
                                  dekf_pairwise=args.dekf_pairwise)

        # 2) Individual per-run plots (side effect)
        for rid, _ in run_results:
            dekf_m, dgo_m, ekf_m = load_run(rid,
                                            flight_only=args.flight_only,
                                            dekf_pairwise=args.dekf_pairwise)
            if all(x is None for x in [dekf_m, dgo_m, ekf_m]):
                continue
            run_out = os.path.join(FIGURE_BASE, f"run_{rid}", plot_tag)
            aligned = align_methods_window(
                {"dekf": dekf_m, "dgo": dgo_m, "ekf": ekf_m})
            plot_mean_error_3way(aligned, run_out, show=False,
                                 dekf_pairwise=args.dekf_pairwise)

        if not args.no_show:
            plt.show()
        return

    # ── Single-run mode ──
    selected_run_id = str(args.single) if args.single is not None \
                      else args.run_id

    log_dirs = {}
    log_dirs["dekf"] = os.path.abspath(args.dekf_log_dir) \
        if args.dekf_log_dir else \
        resolve_log_dir(selected_run_id, "dekf", OLD_DEKF_LOG_DIR)
    log_dirs["dgo"] = os.path.abspath(args.dgo_log_dir) \
        if args.dgo_log_dir else \
        resolve_log_dir(selected_run_id, "ekf_dgo_test", OLD_DGO_LOG_DIR)
    log_dirs["ekf"] = os.path.abspath(args.ekf_log_dir) \
        if args.ekf_log_dir else \
        resolve_log_dir(selected_run_id, "ins_eskf_test", OLD_EKF_LOG_DIR)

    # Flight-only: 加载 stage_map
    stage_map = None
    if args.flight_only:
        dgo_debug_dir = resolve_log_dir(selected_run_id, "dgo", "")
        if os.path.isdir(dgo_debug_dir):
            stage_map = load_stage_map(dgo_debug_dir)
            if stage_map is None:
                print("  [Warning] sync_diag empty/unavailable, "
                      "flight-only filter disabled")
        else:
            print("  [Warning] DGO debug dir not found, "
                  "flight-only filter disabled")

    _tag = selected_run_id if selected_run_id and selected_run_id != "auto" \
           else os.path.basename(os.path.dirname(log_dirs["dgo"]))
    run_tag = f"run_{_tag}" if str(_tag).isdigit() else str(_tag)
    fig_base = os.path.join(FIGURE_BASE, run_tag)
    plot_tag = "dekf_pairwise_dgo_ekf_plot" if args.dekf_pairwise \
               else "dekf_dgo_ekf_plot"
    out_dir = os.path.abspath(args.out_dir) if args.out_dir else \
              os.path.join(fig_base, plot_tag)

    method_dfs = {}
    for m in METHODS:
        label = method_label(m, args.dekf_pairwise)
        print(f"Loading {label} relative CSVs from {log_dirs[m]}")
        method_dfs[m] = load_method(log_dirs[m], m, stage_map=stage_map,
                                    dekf_pairwise=args.dekf_pairwise)

    method_means = {}
    for m in METHODS:
        method_means[m] = build_mean_error(method_dfs[m])

    aligned = align_methods_window(method_means)

    for m in METHODS:
        print_summary(method_label(m, args.dekf_pairwise), aligned[m])
    print_comparison_3way(aligned, dekf_pairwise=args.dekf_pairwise)

    if all(v is None for v in aligned.values()):
        print("No valid data loaded. Exiting.")
        return

    plot_mean_error_3way(aligned, out_dir, show=not args.no_show,
                         dekf_pairwise=args.dekf_pairwise)

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
