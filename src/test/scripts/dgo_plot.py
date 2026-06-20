#!/usr/bin/env python3
"""
Plot DGO residual debug CSV.

Reads {drone}_dgo_residual_debug.csv, creates per-target figures:
  1. rel_gt vs rel_pred (X/Y/Z, phase=post)
  2. uwb_meas vs uwb_pred
  3. uwb_residual
  4. cost_uwb
  5. cost_angle
  6. camera_dt
  7. com_dt
  8. communication position vs GT
"""

import argparse
import glob
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
OLD_DEFAULT_LOG_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "logs"))
RUN_LOG_BASE = os.path.expanduser("~/swarm_localization/logs")
DEFAULT_DRONES = ["iris_0", "iris_1", "iris_2", "iris_3"]
COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]


def resolve_log_dir(run_id, category, old_default):
    """If run_id given, look in ~/swarm_localization/logs/run_X/category/.
    If 'auto', pick the latest run directory.  Falls back to old_default."""
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


def load_csv(log_dir, drone):
    path = os.path.join(log_dir, f"{drone}_dgo_residual_debug.csv")
    if not os.path.exists(path):
        print(f"Skip {drone}: {path} not found")
        return None
    df = pd.read_csv(path)
    for col in ["stamp", "target_id", "self_id"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    # Drop fully-NaN rows from empty pre/post writes
    num_cols = [c for c in df.columns if c not in ("phase", "self_id", "target_id", "stamp")]
    df = df.dropna(subset=num_cols, how="all")
    return df


def load_comm_csv(log_dir, drone):
    path = os.path.join(log_dir, f"{drone}_comm_debug.csv")
    if not os.path.exists(path):
        return None
    df = pd.read_csv(path)
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def load_sync_csv(log_dir, drone):
    path = os.path.join(
        log_dir, "sensor_sync_logs", f"{drone}_dgo_sync_diag.csv")
    if not os.path.exists(path):
        return None
    df = pd.read_csv(path)
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def print_camera_constraint_summary(drone, df):
    required = [
        "camera_msg_fresh",
        "camera_valid_target",
        "camera_angle_constraints",
        "camera_xy_constraints",
        "camera_used_in_cost",
    ]
    if df is None or df.empty or any(col not in df.columns for col in required):
        print(f"  {drone}: camera constraint diagnostics unavailable")
        return

    rows = len(df)
    fresh = int(df["camera_msg_fresh"].fillna(0).sum())
    used = int(df["camera_used_in_cost"].fillna(0).sum())
    angle_total = int(df["camera_angle_constraints"].fillna(0).sum())
    xy_total = int(df["camera_xy_constraints"].fillna(0).sum())
    valid_total = int(df["camera_valid_target"].fillna(0).sum())

    print(
        f"  camera constraints: fresh={fresh}/{rows} "
        f"({100.0 * fresh / rows:.1f}%), used={used}/{rows} "
        f"({100.0 * used / rows:.1f}%), valid_targets={valid_total}, "
        f"angle={angle_total}, xy={xy_total}")

    if "mission_stage" in df.columns:
        for stage, group in df.groupby("mission_stage", dropna=True):
            stage_rows = len(group)
            stage_used = int(group["camera_used_in_cost"].fillna(0).sum())
            print(
                f"    stage={int(stage)}: used={stage_used}/{stage_rows} "
                f"({100.0 * stage_used / stage_rows:.1f}%), "
                f"angle={int(group['camera_angle_constraints'].fillna(0).sum())}, "
                f"xy={int(group['camera_xy_constraints'].fillna(0).sum())}")


def plot_rel_gt_vs_pred(drone, df, out_dir, show):
    """rel_gt_x/y/z vs rel_pred_x/y/z, phase=post, per target"""
    post = df[df["phase"] == "post"]
    if post.empty:
        print(f"  {drone}: no post-optimization data, skip rel_gt vs rel_pred")
        return

    targets = sorted(post["target_id"].dropna().unique())
    fig, axes = plt.subplots(len(targets), 3, figsize=(18, 4 * max(len(targets), 1)), squeeze=False)
    fig.suptitle(f"{drone} — rel_gt vs rel_pred (phase=post)", fontsize=13)

    for row, tid in enumerate(targets):
        sub = post[post["target_id"] == tid].sort_values("stamp")
        for col, axis in enumerate(["x", "y", "z"]):
            ax = axes[row, col]
            gt_col = f"rel_gt_{axis}"
            pr_col = f"rel_pred_{axis}"
            if gt_col in sub.columns and pr_col in sub.columns:
                ax.plot(sub["stamp"], sub[gt_col], "-", color=COLORS[0], linewidth=1.0, label="gt")
                ax.plot(sub["stamp"], sub[pr_col], "--", color=COLORS[1], linewidth=1.0, label="pred")
                diff = (sub[gt_col] - sub[pr_col]).abs()
                ax.set_title(f"target={int(tid)}  {axis}  max|err|={diff.max():.3f}m")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("m")
            ax.legend(fontsize=7)
            ax.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(out_dir, f"{drone}_rel_gt_vs_pred.png")
    fig.savefig(path, dpi=150)
    print(f"  Saved {path}")
    if not show:
        plt.close(fig)


def plot_sensor_comparisons(drone, df, out_dir, show):
    """uwb_meas/pred, uwb_residual, cost_uwb, cost_angle, camera_dt, com_dt — per target"""
    targets = sorted(df["target_id"].dropna().unique())
    if not targets:
        return

    fig, axes = plt.subplots(len(targets), 6, figsize=(24, 3.5 * max(len(targets), 1)), squeeze=False)
    fig.suptitle(f"{drone} — sensor debug (pre=thin, post=bold)", fontsize=13)
    plot_names = ["uwb_meas/pred", "uwb_residual", "cost_uwb", "cost_angle",
                  "camera_dt", "com_dt"]

    for row, tid in enumerate(targets):
        sub = df[df["target_id"] == tid].sort_values("stamp")
        pre = sub[sub["phase"] == "pre"]
        post = sub[sub["phase"] == "post"]

        # col 0: uwb_meas vs uwb_pred
        ax = axes[row, 0]
        if "uwb_meas" in sub.columns:
            valid = sub.dropna(subset=["uwb_meas"])
            ax.plot(valid["stamp"], valid["uwb_meas"], "-", color=COLORS[0], linewidth=0.8, alpha=0.7, label="meas")
            ax.plot(valid["stamp"], valid["uwb_pred"], "--", color=COLORS[1], linewidth=0.8, label="pred")
        ax.set_title(f"t={int(tid)} — uwb meas/pred")
        ax.set_ylabel("m"); ax.legend(fontsize=6); ax.grid(True, alpha=0.3)

        # col 1: uwb_residual
        ax = axes[row, 1]
        if "uwb_residual" in sub.columns:
            r = sub.dropna(subset=["uwb_residual"])
            ax.plot(r["stamp"], r["uwb_residual"], "-", color=COLORS[2], linewidth=0.8)
            ax.axhline(0, color="black", linewidth=0.5, alpha=0.4)
        ax.set_title(f"t={int(tid)} — uwb_residual"); ax.set_ylabel("m"); ax.grid(True, alpha=0.3)

        # col 2: cost_uwb
        ax = axes[row, 2]
        for phase, style, alpha in [("pre", "-", 0.4), ("post", "-", 1.0)]:
            ph = sub[sub["phase"] == phase].dropna(subset=["cost_uwb"])
            if not ph.empty:
                ax.plot(ph["stamp"], ph["cost_uwb"], style, color=COLORS[2], linewidth=0.8, alpha=alpha, label=phase)
        ax.set_title(f"t={int(tid)} — cost_uwb"); ax.legend(fontsize=6); ax.grid(True, alpha=0.3)

        # col 3: cost_angle
        ax = axes[row, 3]
        for phase, style, alpha in [("pre", "-", 0.4), ("post", "-", 1.0)]:
            ph = sub[sub["phase"] == phase].dropna(subset=["cost_angle"])
            if not ph.empty:
                ax.plot(ph["stamp"], ph["cost_angle"], style, color=COLORS[3], linewidth=0.8, alpha=alpha, label=phase)
        ax.set_title(f"t={int(tid)} — cost_angle"); ax.legend(fontsize=6); ax.grid(True, alpha=0.3)

        # col 4: camera_dt
        ax = axes[row, 4]
        if "camera_dt" in sub.columns:
            cam = sub.dropna(subset=["camera_dt"])
            ax.plot(cam["stamp"], cam["camera_dt"], "o", color=COLORS[4], markersize=1.5)
        ax.set_title(f"t={int(tid)} — camera_dt"); ax.set_ylabel("s"); ax.grid(True, alpha=0.3)

        # col 5: com_dt
        ax = axes[row, 5]
        if "com_dt" in sub.columns:
            com = sub.dropna(subset=["com_dt"])
            ax.plot(com["stamp"], com["com_dt"], "o", color=COLORS[0], markersize=1.5)
            ax.axhline(0, color="black", linewidth=0.5, alpha=0.4)
        ax.set_title(f"t={int(tid)} — com_dt"); ax.set_ylabel("s"); ax.grid(True, alpha=0.3)

        for col in range(6):
            axes[row, col].set_xlabel("Time (s)")

    fig.tight_layout()
    path = os.path.join(out_dir, f"{drone}_sensor_debug.png")
    fig.savefig(path, dpi=150)
    print(f"  Saved {path}")
    if not show:
        plt.close(fig)


def plot_comm_debug(drone, df, out_dir, show):
    if df is None or df.empty:
        print(f"  {drone}: no comm debug CSV, skip communication plot")
        return

    targets = sorted(df["target_id"].dropna().unique())
    if not targets:
        return

    fig, axes = plt.subplots(len(targets), 5, figsize=(22, 3.5 * max(len(targets), 1)), squeeze=False)
    fig.suptitle(f"{drone} — communication vs Gazebo GT", fontsize=13)

    for row, tid in enumerate(targets):
        sub = df[df["target_id"] == tid].sort_values("stamp")

        for col, axis in enumerate(["x", "y", "z"]):
            ax = axes[row, col]
            com_col = f"com_global_{axis}"
            gt_col = f"gt_{axis}"
            if com_col in sub.columns and gt_col in sub.columns:
                ax.plot(sub["stamp"], sub[com_col], "-", color=COLORS[0], linewidth=0.8, label="comm")
                ax.plot(sub["stamp"], sub[gt_col], "--", color=COLORS[1], linewidth=0.8, label="gt")
            ax.set_title(f"target={int(tid)} {axis}")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("m")
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=6)

        ax = axes[row, 3]
        if "com_error_norm" in sub.columns:
            ax.plot(sub["stamp"], sub["com_error_norm"], "-", color=COLORS[3], linewidth=0.8)
        ax.set_title(f"target={int(tid)} error_norm")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("m")
        ax.grid(True, alpha=0.3)

        ax = axes[row, 4]
        if "com_stamp_dt" in sub.columns:
            ax.plot(sub["stamp"], sub["com_stamp_dt"], "o", color=COLORS[4], markersize=1.5)
            ax.axhline(0, color="black", linewidth=0.5, alpha=0.4)
        ax.set_title(f"target={int(tid)} com_stamp_dt")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("s")
        ax.grid(True, alpha=0.3)

    fig.tight_layout()
    path = os.path.join(out_dir, f"{drone}_comm_debug.png")
    fig.savefig(path, dpi=150)
    print(f"  Saved {path}")
    if not show:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Plot DGO residual debug CSV")
    parser.add_argument("--drone", type=str, default=None, help="Single drone, e.g. iris_0")
    parser.add_argument("--run-id", type=str, default="auto",
                        help="Run ID (number or 'run_N'; 'auto'=latest; empty=old default path)")
    parser.add_argument("--log-dir", type=str, default=None, help="Override log directory")
    parser.add_argument("--no-show", action="store_true", help="Save only, don't display")
    args = parser.parse_args()

    if args.log_dir:
        log_dir = args.log_dir
    else:
        log_dir = resolve_log_dir(args.run_id, "dgo", OLD_DEFAULT_LOG_DIR)

    # 图片存入独立目录, 不与数据混合
    _tag = args.run_id if args.run_id and args.run_id != "auto" \
           else os.path.basename(os.path.dirname(log_dir))
    run_tag = f"run_{_tag}" if _tag.isdigit() else _tag
    fig_base = os.path.join(os.path.expanduser("~/swarm_localization/logs/figures"), run_tag)
    out_dir = os.path.join(fig_base, "dgo_plot")
    os.makedirs(out_dir, exist_ok=True)
    print(f"DGO plots — data: {log_dir}  figures: {out_dir}")

    drones = [args.drone] if args.drone else DEFAULT_DRONES
    show = not args.no_show

    for drone in drones:
        df = load_csv(log_dir, drone)
        if df is None:
            continue
        print(f"{drone}: {len(df)} rows")
        plot_rel_gt_vs_pred(drone, df, out_dir, show)
        plot_sensor_comparisons(drone, df, out_dir, show)
        comm_df = load_comm_csv(log_dir, drone)
        plot_comm_debug(drone, comm_df, out_dir, show)
        sync_df = load_sync_csv(log_dir, drone)
        print_camera_constraint_summary(drone, sync_df)

    if show:
        plt.show()


if __name__ == "__main__":
    main()
