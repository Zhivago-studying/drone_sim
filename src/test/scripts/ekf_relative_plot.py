#!/usr/bin/env python3
"""
Plot EKF relative position error from ins_eskf_test output.

Reads {drone}_relative_to_iris_0_ekf_error.csv from data_process/logs/.
Generates:
  1. 3D error norm over time (all drones overlaid)
  2. Per-axis error over time (subplots, all drones overlaid)
  3. Cumulative RMSE progression
  4. GT vs EST relative position (X/Y/Z, per-drone)
  5. Mean relative position error over time (iris_1/2/3 average)
"""

import argparse
import glob
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
OLD_DEFAULT_LOG_DIR = os.path.abspath(
    os.path.join(SCRIPT_DIR, "..", "..", "data_process", "logs"))
RUN_LOG_BASE = os.path.join(REPO_ROOT, "run_data")
FIGURE_BASE = os.path.join(RUN_LOG_BASE, "figure")
REFERENCE = "iris_0"
DRONES = ["iris_1", "iris_2", "iris_3"]


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
COLORS = {"iris_1": "#ff7f0e", "iris_2": "#2ca02c", "iris_3": "#d62728"}
LABELS = {"iris_1": "iris_1 → iris_0",
          "iris_2": "iris_2 → iris_0",
          "iris_3": "iris_3 → iris_0"}


def load_csv(log_dir, drone):
    path = os.path.join(log_dir, f"{drone}_relative_to_{REFERENCE}_ekf_error.csv")
    if not os.path.exists(path):
        print(f"  Skip: {path} not found")
        return None
    df = pd.read_csv(path)
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.dropna(subset=["timestamp", "error_norm_m"])
    return df


def compute_stats(df):
    rmse = np.sqrt((df["error_norm_m"] ** 2).mean())
    mean = df["error_norm_m"].mean()
    median = df["error_norm_m"].median()
    p95 = df["error_norm_m"].quantile(0.95)
    p99 = df["error_norm_m"].quantile(0.99)
    max_err = df["error_norm_m"].max()
    return rmse, mean, median, p95, p99, max_err


def build_average_error(drone_dfs):
    """Interpolate all available drone error norms onto one time axis."""
    valid = []
    for drone in DRONES:
        df = drone_dfs.get(drone)
        if df is None or df.empty:
            continue
        df = df.dropna(subset=["timestamp", "error_norm_m"])
        df = df.sort_values("timestamp").drop_duplicates("timestamp")
        if len(df) < 2:
            continue
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

    interpolated = []
    for _, df in valid:
        interpolated.append(np.interp(time_axis,
                                      df["timestamp"].to_numpy(),
                                      df["error_norm_m"].to_numpy()))

    errors = np.vstack(interpolated)
    mean_error = errors.mean(axis=0)
    rms_error = np.sqrt((errors ** 2).mean(axis=0))
    return time_axis, mean_error, rms_error, [drone for drone, _ in valid]


def plot_error_norm(drone_dfs, out_dir, show):
    """Figure 1: 3D error norm over time, all drones overlaid."""
    fig, ax = plt.subplots(1, 1, figsize=(14, 5))
    ax.set_title("EKF Relative Position Error (3D norm) — reference iris_0",
                 fontsize=13)

    for drone in DRONES:
        df = drone_dfs.get(drone)
        if df is None or df.empty:
            continue
        rmse, mean, median, p95, _, _ = compute_stats(df)
        ax.plot(df["timestamp"], df["error_norm_m"],
                color=COLORS[drone], linewidth=0.6,
                label=f"{LABELS[drone]}  "
                      f"RMSE={rmse:.3f}m  mean={mean:.3f}m  p95={p95:.3f}m")
        ax.axhline(rmse, color=COLORS[drone], linestyle=":", linewidth=0.8, alpha=0.5)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Relative Position Error (m)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(out_dir, "relative_error_norm.png")
    fig.savefig(path, dpi=150)
    print(f"  Saved {path}")
    if not show:
        plt.close(fig)


def plot_average_error(drone_dfs, out_dir, show):
    """Figure 2: Mean relative position error over time."""
    avg = build_average_error(drone_dfs)
    if avg is None:
        return

    time_axis, mean_error, rms_error, valid_drones = avg
    fig, ax = plt.subplots(1, 1, figsize=(14, 5))
    ax.set_title("Mean EKF Relative Position Error vs Time — reference iris_0",
                 fontsize=13)

    mean_rmse = np.sqrt((mean_error ** 2).mean())
    rms_mean = rms_error.mean()
    rms_p95 = np.quantile(rms_error, 0.95)
    ax.plot(time_axis, mean_error, color="#1f77b4", linewidth=1.0,
            label=f"Mean norm error ({', '.join(valid_drones)})  "
                  f"RMSE={mean_rmse:.3f}m")
    ax.plot(time_axis, rms_error, color="#9467bd", linewidth=0.8,
            linestyle="--",
            label=f"Per-time RMS error  mean={rms_mean:.3f}m  p95={rms_p95:.3f}m")
    ax.axhline(mean_error.mean(), color="#1f77b4", linestyle=":",
               linewidth=0.8, alpha=0.6)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Average Relative Position Error (m)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(out_dir, "relative_average_error.png")
    fig.savefig(path, dpi=150)
    print(f"  Saved {path}")
    if not show:
        plt.close(fig)


def plot_axis_error(drone_dfs, out_dir, show):
    """Figure 3: Per-axis error (err_x, err_y, err_z) with subplots."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle("EKF Relative Position Error by Axis — reference iris_0",
                 fontsize=13)

    axis_names = ["x", "y", "z"]
    for col, axis in enumerate(axis_names):
        ax = axes[col]
        ax.axhline(0, color="black", linewidth=0.5, alpha=0.4)

        for drone in DRONES:
            df = drone_dfs.get(drone)
            if df is None or df.empty:
                continue
            err_col = f"err_{axis}"
            rmse = np.sqrt((df[err_col] ** 2).mean())
            ax.plot(df["timestamp"], df[err_col],
                    color=COLORS[drone], linewidth=0.6,
                    label=f"{LABELS[drone]}  RMSE={rmse:.3f}m")

        ax.set_title(f"{axis.upper()} axis error")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error (m)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(out_dir, "relative_axis_error.png")
    fig.savefig(path, dpi=150)
    print(f"  Saved {path}")
    if not show:
        plt.close(fig)


def plot_cumulative_rmse(drone_dfs, out_dir, show):
    """Figure 4: Cumulative RMSE progression over time."""
    fig, ax = plt.subplots(1, 1, figsize=(14, 5))
    ax.set_title("EKF Cumulative RMSE over Time — reference iris_0",
                 fontsize=13)

    for drone in DRONES:
        df = drone_dfs.get(drone)
        if df is None or df.empty:
            continue
        ax.plot(df["timestamp"], df["cumulative_rmse_m"],
                color=COLORS[drone], linewidth=0.8,
                label=f"{LABELS[drone]}  final={df['cumulative_rmse_m'].iloc[-1]:.3f}m")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Cumulative RMSE (m)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(out_dir, "relative_cumulative_rmse.png")
    fig.savefig(path, dpi=150)
    print(f"  Saved {path}")
    if not show:
        plt.close(fig)


def plot_gt_vs_est(drone_dfs, out_dir, show):
    """Figure 5: GT vs EST relative position (X/Y/Z), one subplot per drone."""
    n_drones = len([d for d in DRONES if d in drone_dfs and
                    drone_dfs[d] is not None and not drone_dfs[d].empty])
    if n_drones == 0:
        return

    fig, axes = plt.subplots(n_drones, 3, figsize=(18, 5 * n_drones))
    if n_drones == 1:
        axes = axes.reshape(1, -1)
    fig.suptitle("Relative Position: GT vs EKF Estimate — reference iris_0",
                 fontsize=14)

    axis_names = ["x", "y", "z"]
    colors_gt = "#1f77b4"
    colors_est = "#d62728"

    for row, drone in enumerate([d for d in DRONES if d in drone_dfs]):
        df = drone_dfs[drone]
        if df is None or df.empty:
            continue
        for col, axis in enumerate(axis_names):
            ax = axes[row, col]
            gt_col = f"{axis}_gt"
            est_col = f"{axis}_est"
            ax.plot(df["timestamp"], df[gt_col], "-",
                    color=colors_gt, linewidth=0.8, label="GT")
            ax.plot(df["timestamp"], df[est_col], "--",
                    color=colors_est, linewidth=0.8, label="EST")
            ax.set_title(f"{drone} {axis.upper()}")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Position (m)")
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(out_dir, "relative_gt_vs_est.png")
    fig.savefig(path, dpi=150)
    print(f"  Saved {path}")
    if not show:
        plt.close(fig)


def print_summary(drone_dfs):
    """Print per-drone relative error summary."""
    print("\n=============== EKF Relative Position Error Summary ===============")
    print(f"{'Drone':<12} {'Samples':<8} {'RMSE':<10} {'Mean':<10} "
          f"{'Median':<10} {'p95':<10} {'p99':<10} {'Max':<10}")
    print("-" * 80)
    for drone in DRONES:
        df = drone_dfs.get(drone)
        if df is None or df.empty:
            print(f"{drone:<12} no data")
            continue
        rmse, mean, median, p95, p99, max_err = compute_stats(df)
        print(f"{drone:<12} {len(df):<8} {rmse:<10.4f} {mean:<10.4f} "
              f"{median:<10.4f} {p95:<10.4f} {p99:<10.4f} {max_err:<10.4f}")
    print("=" * 80)
    print()

    # Per-axis RMSE
    print("Per-Axis RMSE (m):")
    print(f"{'Drone':<12} {'err_x':<12} {'err_y':<12} {'err_z':<12}")
    print("-" * 48)
    for drone in DRONES:
        df = drone_dfs.get(drone)
        if df is None or df.empty:
            continue
        rx = np.sqrt((df["err_x"] ** 2).mean())
        ry = np.sqrt((df["err_y"] ** 2).mean())
        rz = np.sqrt((df["err_z"] ** 2).mean())
        print(f"{drone:<12} {rx:<12.4f} {ry:<12.4f} {rz:<12.4f}")
    print("-" * 48)
    print(f"{'Ideal':<12} {'0.0000':<12} {'0.0000':<12} {'0.0000':<12}")


def main():
    parser = argparse.ArgumentParser(
        description="Plot EKF relative position error from ins_eskf_test.")
    parser.add_argument("--run-id", type=str, default="auto",
                        help="Run ID (number or 'run_N'; 'auto'=latest; empty=old defaults)")
    parser.add_argument("--log-dir", type=str, default=None,
                        help="Override log directory")
    parser.add_argument("--out-dir", type=str, default=None,
                        help="Output directory for figures.")
    parser.add_argument("--no-show", action="store_true",
                        help="Save figures only, do not display windows.")
    args = parser.parse_args()

    log_dir = os.path.abspath(args.log_dir) if args.log_dir else \
              resolve_log_dir(args.run_id, "ins_eskf_test", OLD_DEFAULT_LOG_DIR)
    _tag = args.run_id if args.run_id and args.run_id != "auto" \
           else os.path.basename(os.path.dirname(log_dir))
    run_tag = f"run_{_tag}" if _tag.isdigit() else _tag
    fig_base = os.path.join(FIGURE_BASE, run_tag)
    out_dir = os.path.abspath(args.out_dir) if args.out_dir else \
              os.path.join(fig_base, "ekf_relative_plot")
    os.makedirs(out_dir, exist_ok=True)

    print(f"Loading relative EKF error CSVs from {log_dir}")
    drone_dfs = {}
    for drone in DRONES:
        df = load_csv(log_dir, drone)
        if df is not None:
            drone_dfs[drone] = df
            print(f"  {drone}: {len(df)} samples")

    if not drone_dfs:
        print("No data loaded. Exiting.")
        return

    show = not args.no_show
    print_summary(drone_dfs)

    plot_error_norm(drone_dfs, out_dir, show)
    plot_average_error(drone_dfs, out_dir, show)
    plot_axis_error(drone_dfs, out_dir, show)
    plot_cumulative_rmse(drone_dfs, out_dir, show)
    plot_gt_vs_est(drone_dfs, out_dir, show)

    if show:
        plt.show()


if __name__ == "__main__":
    main()
