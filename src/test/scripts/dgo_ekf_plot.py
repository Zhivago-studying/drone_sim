#!/usr/bin/env python3
"""
Plot DGO vs EKF mean relative position error to iris_0.

Inputs:
  - src/test/logs/{drone}_relative_to_iris_0_dgo_error.csv
  - src/data_process/logs/{drone}_relative_to_iris_0_ekf_error.csv

Output:
  - mean_relative_error_dgo_vs_ekf.png
"""

import argparse
import glob
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
OLD_DGO_LOG_DIR = os.path.join(REPO_ROOT, "src", "test", "logs")
OLD_EKF_LOG_DIR = os.path.join(REPO_ROOT, "src", "data_process", "logs")
RUN_LOG_BASE = os.path.join(REPO_ROOT, "run_data")
FIGURE_BASE = os.path.join(RUN_LOG_BASE, "figure")


REFERENCE = "iris_0"
DRONES = ["iris_1", "iris_2", "iris_3"]

# 飞行窗口过滤
STAGES_EXCLUDE = {0, 7}   # WAIT, DONE — 始终排除
PRE_FLIGHT = -1           # DGO 启动之前的时间戳


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


def load_method(log_dir, method, stage_map=None):
    result = {}
    for drone in DRONES:
        df = load_relative_csv(log_dir, drone, method)
        if df is not None:
            if stage_map is not None:
                df = filter_flight_window(df, stage_map, time_col="timestamp")
            result[drone] = df
            print(f"  {method.upper()} {drone}: {len(df)} samples")
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


def align_method_window(dgo_mean, ekf_mean):
    """Resample DGO and EKF onto the same timestamps for a fair comparison."""
    if dgo_mean is None or ekf_mean is None:
        return dgo_mean, ekf_mean

    t_start = max(dgo_mean["time"][0], ekf_mean["time"][0])
    t_end = min(dgo_mean["time"][-1], ekf_mean["time"][-1])
    if t_end <= t_start:
        print("  Warning: DGO and EKF have no overlapping time window")
        return None, None

    time_axis = np.unique(np.concatenate([
        dgo_mean["time"][
            (dgo_mean["time"] >= t_start) & (dgo_mean["time"] <= t_end)],
        ekf_mean["time"][
            (ekf_mean["time"] >= t_start) & (ekf_mean["time"] <= t_end)],
    ]))
    if len(time_axis) < 2:
        return None, None

    def resample(data):
        return {
            "time": time_axis,
            "mean": np.interp(time_axis, data["time"], data["mean"]),
            "rms": np.interp(time_axis, data["time"], data["rms"]),
            "drones": data["drones"],
        }

    print(f"  Fair comparison window: {t_start:.3f}s -> {t_end:.3f}s "
          f"({len(time_axis)} aligned timestamps)")
    return resample(dgo_mean), resample(ekf_mean)


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


def print_comparison(dgo_mean, ekf_mean):
    if dgo_mean is None or ekf_mean is None:
        print("\nDGO vs EKF: unavailable (no common aligned data)")
        return

    dgo_rmse = stats(dgo_mean["mean"])["rmse"]
    ekf_rmse = stats(ekf_mean["mean"])["rmse"]
    ratio = dgo_rmse / ekf_rmse if ekf_rmse > 0.0 else float("inf")
    result = "PASS" if ratio < 1.0 else "FAIL"
    print("\nDGO vs EKF fair-window acceptance")
    print(f"  DGO mean-relative RMSE: {dgo_rmse:.4f}m")
    print(f"  EKF mean-relative RMSE: {ekf_rmse:.4f}m")
    print(f"  ratio DGO/EKF:          {ratio:.3f}")
    print(f"  result:                  {result} (required ratio < 1.0)")


def plot_multi_run_error(dgo_var, ekf_var, out_dir, show, run_ids):
    """Multi-run overlay: thin per-run lines + bold avg ± 1σ shaded band."""
    fig, ax = plt.subplots(1, 1, figsize=(14, 5.5))
    runs_str = f"run_{run_ids[0]}–run_{run_ids[-1]}"
    ax.set_title(f"Mean Relative Position Error to iris_0 — {runs_str}",
                 fontsize=13)

    colors = {"dgo": "#d62728", "ekf": "#1f77b4"}
    labels = {"dgo": "DGO", "ekf": "EKF"}

    for method, var, color, label in [
        ("dgo", dgo_var, colors["dgo"], labels["dgo"]),
        ("ekf", ekf_var, colors["ekf"], labels["ekf"]),
    ]:
        if var is None:
            continue

        # Per-run thin lines
        for i, (rid, curve) in enumerate(var["individual"]):
            alpha = 0.25 + 0.15 * (i / max(len(var["individual"]) - 1, 1))
            ax.plot(var["time"], curve, color=color,
                    linewidth=0.6, alpha=alpha, zorder=1,
                    label=f"{label} run_{rid}" if i == 0 else None)

        # ±1σ shaded band
        ax.fill_between(var["time"],
                         var["mean"] - var["std"],
                         var["mean"] + var["std"],
                         color=color, alpha=0.12, zorder=2,
                         label=f"{label} ±1σ" if method == "dgo" else None)

        # Average bold line
        rmse = np.sqrt(np.mean(var["mean"] ** 2))
        ax.plot(var["time"], var["mean"], color=color,
                linewidth=2.0, zorder=3,
                label=f"{label} avg (RMSE={rmse:.3f}m, N={var['n_runs']} runs)")

    ax.set_xlabel("Elapsed Time (s)")
    ax.set_ylabel("Average Relative Position Error (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, ncol=2)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "mean_relative_error_dgo_vs_ekf.png")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"\nSaved {path}")
    if not show:
        plt.close(fig)


def plot_mean_error(dgo_mean, ekf_mean, out_dir, show):
    fig, ax = plt.subplots(1, 1, figsize=(14, 5))
    ax.set_title("Mean Relative Position Error to iris_0: DGO vs EKF",
                 fontsize=13)

    if dgo_mean is not None:
        s = stats(dgo_mean["mean"])
        ax.plot(dgo_mean["time"], dgo_mean["mean"],
                color="#d62728", linewidth=1.0,
                label=f"DGO mean norm  RMSE={s['rmse']:.3f}m  p95={s['p95']:.3f}m")
        ax.plot(dgo_mean["time"], dgo_mean["rms"],
                color="#d62728", linewidth=0.8, linestyle="--",
                alpha=0.75, label="DGO per-time RMS")

    if ekf_mean is not None:
        s = stats(ekf_mean["mean"])
        ax.plot(ekf_mean["time"], ekf_mean["mean"],
                color="#1f77b4", linewidth=1.0,
                label=f"EKF mean norm  RMSE={s['rmse']:.3f}m  p95={s['p95']:.3f}m")
        ax.plot(ekf_mean["time"], ekf_mean["rms"],
                color="#1f77b4", linewidth=0.8, linestyle="--",
                alpha=0.75, label="EKF per-time RMS")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Average Relative Position Error (m)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "mean_relative_error_dgo_vs_ekf.png")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"\nSaved {path}")
    if not show:
        plt.close(fig)


def load_run(run_id, flight_only=False):
    dgo_log_dir = resolve_log_dir(run_id, "ekf_dgo_test", OLD_DGO_LOG_DIR)
    ekf_log_dir = resolve_log_dir(run_id, "ins_eskf_test", OLD_EKF_LOG_DIR)

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
    print(f"Loading DGO relative CSVs from {dgo_log_dir}")
    dgo_dfs = load_method(dgo_log_dir, "dgo", stage_map=stage_map)
    print(f"Loading EKF relative CSVs from {ekf_log_dir}")
    ekf_dfs = load_method(ekf_log_dir, "ekf", stage_map=stage_map)

    dgo_mean, ekf_mean = align_method_window(
        build_mean_error(dgo_dfs),
        build_mean_error(ekf_dfs))
    print_summary("DGO", dgo_mean)
    print_summary("EKF", ekf_mean)
    print_comparison(dgo_mean, ekf_mean)
    return dgo_mean, ekf_mean


def build_aggregate_variation(run_results, method):
    """Return per-run aligned series + mean ± 1σ for multi-run plotting."""
    series = []
    for run_id, result in run_results:
        mean_data = result[0] if method == "dgo" else result[1]
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
    series = []
    for run_id, result in run_results:
        mean_data = result[0] if method == "dgo" else result[1]
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


def main():
    parser = argparse.ArgumentParser(
        description="Plot DGO/EKF mean relative position error to iris_0.")
    parser.add_argument("--run-id", type=str, default="auto",
                        help="Run ID (number or 'run_N'; 'auto'=latest; empty=old defaults)")
    parser.add_argument("--single", type=int, default=None,
                        help="Plot one run ID; equivalent to --run-id N.")
    parser.add_argument("--start-id", type=int, default=None,
                        help="First run ID for multi-run averaging.")
    parser.add_argument("--end-id", type=int, default=None,
                        help="Last run ID for multi-run averaging (inclusive).")
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
    args = parser.parse_args()

    if (args.start_id is None) != (args.end_id is None):
        parser.error("--start-id and --end-id must be specified together")
    if args.start_id is not None and args.start_id > args.end_id:
        parser.error("--start-id must be <= --end-id")
    if args.single is not None and args.start_id is not None:
        parser.error("--single cannot be combined with --start-id/--end-id")
    if args.start_id is not None and (args.dgo_log_dir or args.ekf_log_dir):
        parser.error("log directory overrides are only supported for a single run")

    if args.start_id is not None:
        run_results = []
        range_run_ids = []
        for run_id in range(args.start_id, args.end_id + 1):
            run_dir = os.path.join(RUN_LOG_BASE, f"run_{run_id}")
            if not os.path.isdir(run_dir):
                print(f"\nSkip missing run directory: {run_dir}")
                continue
            range_run_ids.append(run_id)
            run_results.append((run_id, load_run(run_id, flight_only=args.flight_only)))

        if not run_results:
            print("No valid runs loaded. Exiting.")
            return

        # 1) Multi-run aggregate plot with variation bands
        dgo_var = build_aggregate_variation(run_results, "dgo")
        ekf_var = build_aggregate_variation(run_results, "ekf")
        if dgo_var is None and ekf_var is None:
            print("No alignable data for multi-run plot. Exiting.")
            return

        # Print individual run summaries
        for rid, (dgo_m, ekf_m) in run_results:
            print_summary("DGO", dgo_m, label=f"run_{rid}")
            print_summary("EKF", ekf_m, label=f"run_{rid}")

        # Print aggregated summary
        dgo_mean = aggregate_runs(run_results, "dgo")
        ekf_mean = aggregate_runs(run_results, "ekf")
        print_summary("DGO", dgo_mean, label="aggregate")
        print_summary("EKF", ekf_mean, label="aggregate")
        print_comparison(dgo_mean, ekf_mean)

        run_tag = f"runs_{args.start_id}_to_{args.end_id}"
        out_dir = os.path.abspath(args.out_dir) if args.out_dir else os.path.join(
            FIGURE_BASE, run_tag, "dgo_ekf_plot")

        # Multi-run variation plot
        plot_multi_run_error(dgo_var, ekf_var, out_dir,
                             show=not args.no_show, run_ids=range_run_ids)

        # 2) Individual per-run plots (side effect)
        for rid, _ in run_results:
            dgo_m, ekf_m = load_run(rid, flight_only=args.flight_only)
            if dgo_m is None and ekf_m is None:
                continue
            run_out = os.path.join(FIGURE_BASE, f"run_{rid}", "dgo_ekf_plot")
            plot_mean_error(dgo_m, ekf_m, run_out, show=False)

        if not args.no_show:
            plt.show()
        return

    selected_run_id = str(args.single) if args.single is not None else args.run_id
    dgo_log_dir = os.path.abspath(args.dgo_log_dir) if args.dgo_log_dir else \
                  resolve_log_dir(selected_run_id, "ekf_dgo_test", OLD_DGO_LOG_DIR)
    ekf_log_dir = os.path.abspath(args.ekf_log_dir) if args.ekf_log_dir else \
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
           else os.path.basename(os.path.dirname(dgo_log_dir))
    run_tag = f"run_{_tag}" if _tag.isdigit() else _tag
    fig_base = os.path.join(FIGURE_BASE, run_tag)
    out_dir = os.path.abspath(args.out_dir) if args.out_dir else \
              os.path.join(fig_base, "dgo_ekf_plot")

    print(f"Loading DGO relative CSVs from {dgo_log_dir}")
    dgo_dfs = load_method(dgo_log_dir, "dgo", stage_map=stage_map)
    print(f"\nLoading EKF relative CSVs from {ekf_log_dir}")
    ekf_dfs = load_method(ekf_log_dir, "ekf", stage_map=stage_map)

    dgo_mean, ekf_mean = align_method_window(
        build_mean_error(dgo_dfs),
        build_mean_error(ekf_dfs))
    print_summary("DGO", dgo_mean)
    print_summary("EKF", ekf_mean)
    print_comparison(dgo_mean, ekf_mean)

    if dgo_mean is None and ekf_mean is None:
        print("No valid DGO or EKF data loaded. Exiting.")
        return

    plot_mean_error(dgo_mean, ekf_mean, out_dir, show=not args.no_show)

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
