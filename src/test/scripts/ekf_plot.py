#!/usr/bin/env python3
"""
Plot EKF (INS) evaluation results from ins_eskf_test output.

Reads {drone}_gt_traj.csv, {drone}_est_traj.csv from data_process/logs/.
For each drone generates two figures:
  1. GT vs EST position — X / Y / Z / 3D distance
  2. Position error — X / Y / Z / 3D, with RMSE statistics
"""

import argparse
import glob
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
OLD_DEFAULT_LOG_DIR = os.path.abspath(
    os.path.join(SCRIPT_DIR, "..", "..", "data_process", "logs"))
RUN_LOG_BASE = os.path.expanduser("~/swarm_localization/logs")
DEFAULT_DRONES = ["iris_0", "iris_1", "iris_2", "iris_3"]
COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]


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


def load_traj(path):
    """Load a TUM-format trajectory file (space-separated: t x y z qx qy qz qw)."""
    if not os.path.exists(path):
        print(f"  Skip: {path} not found")
        return None
    df = pd.read_csv(path, sep=r"\s+", header=None,
                     names=["t", "x", "y", "z", "qx", "qy", "qz", "qw"],
                     engine="python")
    for col in ["t", "x", "y", "z"]:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df.dropna(subset=["t", "x", "y", "z"])


def align_trajectories(est, gt, max_dt=0.05):
    """Nearest-neighbor alignment: for each EST row find the closest GT row."""
    if est is None or gt is None:
        return None

    gt_times = gt["t"].values
    gt_pos = gt[["x", "y", "z"]].values

    aligned = []
    gt_idx = 0

    for _, row in est.iterrows():
        t = row["t"]
        best_dt = 1e9
        best_j = gt_idx

        for j in range(gt_idx, len(gt_times)):
            dt = abs(t - gt_times[j])
            if dt < best_dt:
                best_dt = dt
                best_j = j
                gt_idx = j
            elif gt_times[j] > t and dt > best_dt:
                break

        if best_dt < max_dt:
            aligned.append({
                "t": t,
                "est_x": row["x"], "est_y": row["y"], "est_z": row["z"],
                "gt_x": gt_pos[best_j, 0],
                "gt_y": gt_pos[best_j, 1],
                "gt_z": gt_pos[best_j, 2],
            })

    if not aligned:
        return None
    return pd.DataFrame(aligned)


def load_and_align(log_dir, drone):
    """Load GT and EST trajectories for a drone, return aligned DataFrame."""
    est_path = os.path.join(log_dir, f"{drone}_est_traj.csv")
    gt_path = os.path.join(log_dir, f"{drone}_gt_traj.csv")

    est = load_traj(est_path)
    gt = load_traj(gt_path)

    if est is None or gt is None:
        return None

    aligned = align_trajectories(est, gt)
    if aligned is None or aligned.empty:
        print(f"  {drone}: no aligned samples")
        return None

    # compute errors
    aligned["err_x"] = aligned["est_x"] - aligned["gt_x"]
    aligned["err_y"] = aligned["est_y"] - aligned["gt_y"]
    aligned["err_z"] = aligned["est_z"] - aligned["gt_z"]
    aligned["err_3d"] = np.sqrt(
        aligned["err_x"] ** 2 + aligned["err_y"] ** 2 + aligned["err_z"] ** 2
    )

    # 3D distance from origin (relative ENU, both start near zero)
    aligned["gt_3d"] = np.sqrt(
        aligned["gt_x"] ** 2 + aligned["gt_y"] ** 2 + aligned["gt_z"] ** 2
    )
    aligned["est_3d"] = np.sqrt(
        aligned["est_x"] ** 2 + aligned["est_y"] ** 2 + aligned["est_z"] ** 2
    )

    return aligned


def plot_traj(drone, df, out_dir, show):
    """Figure 1: GT vs EST position (X, Y, Z, 3D distance)."""
    fig, axes = plt.subplots(1, 4, figsize=(22, 5))
    fig.suptitle(f"{drone} — EKF Estimate vs Ground Truth", fontsize=13)

    axis_names = ["x", "y", "z"]
    for col, axis in enumerate(axis_names):
        ax = axes[col]
        ax.plot(df["t"], df[f"gt_{axis}"], "-", color=COLORS[0],
                linewidth=1.0, label="GT")
        ax.plot(df["t"], df[f"est_{axis}"], "--", color=COLORS[1],
                linewidth=1.0, label="EST")
        err = (df[f"est_{axis}"] - df[f"gt_{axis}"]).abs()
        ax.set_title(f"{axis.upper()}  (max|err|={err.max():.3f}m)")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("m")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    # 3D distance from origin
    ax = axes[3]
    ax.plot(df["t"], df["gt_3d"], "-", color=COLORS[0],
            linewidth=1.0, label="GT")
    ax.plot(df["t"], df["est_3d"], "--", color=COLORS[1],
            linewidth=1.0, label="EST")
    err_3d = (df["est_3d"] - df["gt_3d"]).abs()
    ax.set_title(f"3D distance from origin  (max|err|={err_3d.max():.3f}m)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("m")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(out_dir, f"{drone}_ekf_traj.png")
    fig.savefig(path, dpi=150)
    print(f"  Saved {path}")
    if not show:
        plt.close(fig)


def plot_error(drone, df, out_dir, show):
    """Figure 2: Position error curves (X, Y, Z, 3D) with RMSE statistics."""
    fig, axes = plt.subplots(1, 4, figsize=(22, 5))
    fig.suptitle(f"{drone} — EKF Position Error", fontsize=13)

    axis_names = ["x", "y", "z"]
    for col, axis in enumerate(axis_names):
        ax = axes[col]
        err_col = f"err_{axis}"
        ax.plot(df["t"], df[err_col], "-", color=COLORS[2],
                linewidth=0.8)
        ax.axhline(0, color="black", linewidth=0.5, alpha=0.4)
        rmse = np.sqrt((df[err_col] ** 2).mean())
        mean_err = df[err_col].mean()
        std_err = df[err_col].std()
        ax.set_title(f"{axis.upper()} error  "
                     f"mean={mean_err:.3f}m  std={std_err:.3f}m  RMSE={rmse:.3f}m")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Error (m)")
        ax.grid(True, alpha=0.3)

    # 3D error
    ax = axes[3]
    ax.plot(df["t"], df["err_3d"], "-", color=COLORS[2],
            linewidth=0.8)
    ax.axhline(0, color="black", linewidth=0.5, alpha=0.4)
    rmse_3d = np.sqrt((df["err_3d"] ** 2).mean())
    mean_3d = df["err_3d"].mean()
    p50_3d = df["err_3d"].median()
    p95_3d = df["err_3d"].quantile(0.95)
    ax.set_title(f"3D error  "
                 f"mean={mean_3d:.3f}m  median={p50_3d:.3f}m  "
                 f"p95={p95_3d:.3f}m  RMSE={rmse_3d:.3f}m")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (m)")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(out_dir, f"{drone}_ekf_error.png")
    fig.savefig(path, dpi=150)
    print(f"  Saved {path}")
    if not show:
        plt.close(fig)


def print_summary(drone, df):
    """Print per-drone summary statistics."""
    if df is None or df.empty:
        print(f"{drone}: no data")
        return

    duration = df["t"].iloc[-1] - df["t"].iloc[0]
    ate = np.sqrt((df["err_3d"] ** 2).mean())

    print(f"\n{drone}")
    print(f"  samples: {len(df)} | duration: {duration:.1f}s")
    print(f"  ATE (RMSE):     {ate:.4f} m")
    print(f"  err_3d mean:    {df['err_3d'].mean():.4f} m")
    print(f"  err_3d median:  {df['err_3d'].median():.4f} m")
    print(f"  err_3d p95:     {df['err_3d'].quantile(0.95):.4f} m")
    print(f"  err_3d max:     {df['err_3d'].max():.4f} m")
    print(f"  err_x RMSE:     {np.sqrt((df['err_x']**2).mean()):.4f} m")
    print(f"  err_y RMSE:     {np.sqrt((df['err_y']**2).mean()):.4f} m")
    print(f"  err_z RMSE:     {np.sqrt((df['err_z']**2).mean()):.4f} m")


def main():
    parser = argparse.ArgumentParser(
        description="Plot EKF evaluation results from ins_eskf_test.")
    parser.add_argument("--drone", type=str, default=None,
                        help="Single drone, e.g. iris_0. Omit for all.")
    parser.add_argument("--run-id", type=str, default="auto",
                        help="Run ID (number or 'run_N'; 'auto'=latest; empty=old defaults)")
    parser.add_argument("--log-dir", type=str, default=None,
                        help="Override log directory")
    parser.add_argument("--out-dir", type=str, default=None,
                        help="Output directory for figures.")
    parser.add_argument("--no-show", action="store_true",
                        help="Save figures only, do not display windows.")
    parser.add_argument("--summary-only", action="store_true",
                        help="Print statistics without generating figures.")
    args = parser.parse_args()

    log_dir = os.path.abspath(args.log_dir) if args.log_dir else \
              resolve_log_dir(args.run_id, "ins_eskf_test", OLD_DEFAULT_LOG_DIR)
    _tag = args.run_id if args.run_id and args.run_id != "auto" \
           else os.path.basename(os.path.dirname(log_dir))
    run_tag = f"run_{_tag}" if _tag.isdigit() else _tag
    fig_base = os.path.join(os.path.expanduser("~/swarm_localization/logs/figures"), run_tag)
    out_dir = os.path.abspath(args.out_dir) if args.out_dir \
        else os.path.join(fig_base, "ekf_plot")
    os.makedirs(out_dir, exist_ok=True)

    drones = [args.drone] if args.drone else DEFAULT_DRONES
    show = not args.no_show

    for drone in drones:
        df = load_and_align(log_dir, drone)
        if df is None:
            continue
        print_summary(drone, df)
        if args.summary_only:
            continue
        plot_traj(drone, df, out_dir, show)
        plot_error(drone, df, out_dir, show)

    if show and not args.summary_only:
        plt.show()


if __name__ == "__main__":
    main()
