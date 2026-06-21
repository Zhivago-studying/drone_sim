#!/usr/bin/env python3
"""
Plot UWB simulation logs.

New logs contain:
  raw CSV:       seq,timestamp,self_id,target_id,distance,gt_distance
  processed CSV: seq,timestamp,self_id,target_id,filtered_distance,gt_distance

The script also accepts old logs without gt_distance. In that case it only plots
raw vs processed and reports raw-filtered difference, not true measurement error.
"""

import argparse
import glob
import os
from typing import Iterable, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
OLD_DEFAULT_LOG_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "logs", "uwb_test"))
RUN_LOG_BASE = os.path.join(REPO_ROOT, "run_data")
FIGURE_BASE = os.path.join(RUN_LOG_BASE, "figure")
DEFAULT_DRONES = ["iris_0", "iris_1", "iris_2", "iris_3"]
COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]


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


def load_data(log_dir: str, drone_name: str) -> Tuple[Optional[pd.DataFrame], Optional[pd.DataFrame]]:
    raw_path = os.path.join(log_dir, f"{drone_name}_uwb_raw.csv")
    proc_path = os.path.join(log_dir, f"{drone_name}_uwb_processed.csv")
    if not os.path.exists(raw_path) or not os.path.exists(proc_path):
        print(f"Skip {drone_name}: missing {raw_path} or {proc_path}")
        return None, None

    raw = pd.read_csv(raw_path)
    proc = pd.read_csv(proc_path)
    raw = normalize_columns(raw, "raw", drone_name)
    proc = normalize_columns(proc, "processed", drone_name)
    return raw, proc


def normalize_columns(df: pd.DataFrame, kind: str, drone_name: str) -> pd.DataFrame:
    required = {"seq", "timestamp", "self_id", "target_id"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"{drone_name} {kind} CSV missing columns: {sorted(missing)}")

    if kind == "raw" and "distance" not in df.columns:
        raise ValueError(f"{drone_name} raw CSV missing distance column")
    if kind == "processed" and "filtered_distance" not in df.columns:
        if "distance" in df.columns:
            df = df.rename(columns={"distance": "filtered_distance"})
        else:
            raise ValueError(f"{drone_name} processed CSV missing filtered_distance column")

    for col in ["seq", "timestamp", "self_id", "target_id", "distance", "filtered_distance", "gt_distance"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    df = df.dropna(subset=["seq", "timestamp", "self_id", "target_id"])
    df["seq"] = df["seq"].astype(int)
    df["self_id"] = df["self_id"].astype(int)
    df["target_id"] = df["target_id"].astype(int)
    return df.sort_values(["target_id", "timestamp"]).reset_index(drop=True)


def has_truth(raw: pd.DataFrame, proc: pd.DataFrame) -> bool:
    return (
        "gt_distance" in raw.columns
        and "gt_distance" in proc.columns
        and raw["gt_distance"].notna().any()
        and proc["gt_distance"].notna().any()
    )


def build_merged(raw: pd.DataFrame, proc: pd.DataFrame) -> pd.DataFrame:
    keys = ["seq", "timestamp", "self_id", "target_id"]
    merged = pd.merge(raw, proc, on=keys, how="inner", suffixes=("_raw", "_proc"))
    if "gt_distance_raw" in merged.columns:
        merged["gt_distance"] = merged["gt_distance_raw"]
    elif "gt_distance_proc" in merged.columns:
        merged["gt_distance"] = merged["gt_distance_proc"]

    merged["raw_minus_filtered"] = merged["distance"] - merged["filtered_distance"]
    if "gt_distance" in merged.columns and merged["gt_distance"].notna().any():
        merged["raw_error"] = merged["distance"] - merged["gt_distance"]
        merged["filtered_error"] = merged["filtered_distance"] - merged["gt_distance"]
    return merged


def rmse(series: pd.Series) -> float:
    values = series.dropna().to_numpy(dtype=float)
    if values.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(values * values)))


def target_ids(raw: pd.DataFrame, proc: pd.DataFrame) -> Iterable[int]:
    ids = set(raw["target_id"].dropna().astype(int).tolist())
    ids.update(proc["target_id"].dropna().astype(int).tolist())
    return sorted(ids)


def plot_compare(drone_name: str, raw: pd.DataFrame, proc: pd.DataFrame, out_dir: str, show: bool) -> None:
    fig, ax = plt.subplots(figsize=(14, 5))
    truth_available = has_truth(raw, proc)
    title = "UWB true/raw/processed" if truth_available else "UWB raw/processed"
    fig.suptitle(f"{drone_name} - {title}", fontsize=13)

    for idx, tid in enumerate(target_ids(raw, proc)):
        color = COLORS[idx % len(COLORS)]
        r = raw[raw["target_id"] == tid]
        p = proc[proc["target_id"] == tid]

        if truth_available and "gt_distance" in r.columns:
            ax.plot(
                r["timestamp"],
                r["gt_distance"],
                linestyle="-",
                color=color,
                linewidth=1.4,
                alpha=0.95,
                label=f"gt target={tid}",
            )
        ax.plot(
            r["timestamp"],
            r["distance"],
            linestyle="--",
            color=color,
            linewidth=0.7,
            alpha=0.65,
            label=f"raw target={tid}",
        )
        ax.plot(
            p["timestamp"],
            p["filtered_distance"],
            linestyle=":",
            color=color,
            linewidth=1.4,
            label=f"filtered target={tid}",
        )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (m)")
    ax.legend(loc="upper right", fontsize=7, ncol=3 if truth_available else 2)
    ax.grid(True, alpha=0.3)

    save_figure(fig, out_dir, f"{drone_name}_uwb_compare.png", show)


def plot_error(drone_name: str, raw: pd.DataFrame, proc: pd.DataFrame, out_dir: str, show: bool) -> None:
    merged = build_merged(raw, proc)
    if merged.empty:
        print(f"Skip {drone_name} error plot: no aligned raw/processed rows")
        return

    truth_available = "raw_error" in merged.columns and "filtered_error" in merged.columns
    fig, ax = plt.subplots(figsize=(14, 5))
    if truth_available:
        fig.suptitle(f"{drone_name} - UWB error to Gazebo truth", fontsize=13)
    else:
        fig.suptitle(f"{drone_name} - raw minus processed distance", fontsize=13)

    for idx, tid in enumerate(sorted(merged["target_id"].unique())):
        color = COLORS[idx % len(COLORS)]
        sub = merged[merged["target_id"] == tid]
        if truth_available:
            ax.plot(
                sub["timestamp"],
                sub["raw_error"],
                linestyle="--",
                color=color,
                linewidth=0.7,
                alpha=0.65,
                label=f"raw error target={tid}",
            )
            ax.plot(
                sub["timestamp"],
                sub["filtered_error"],
                linestyle="-",
                color=color,
                linewidth=1.1,
                label=f"filtered error target={tid}",
            )
        else:
            ax.plot(
                sub["timestamp"],
                sub["raw_minus_filtered"],
                linestyle="-",
                color=color,
                linewidth=0.9,
                label=f"target={tid}",
            )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Error (m)")
    ax.legend(loc="upper right", fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    if truth_available:
        text = (
            f"raw_RMSE={rmse(merged['raw_error']):.4f} m | "
            f"filtered_RMSE={rmse(merged['filtered_error']):.4f} m | "
            f"accept_rate={len(proc) / max(len(raw), 1) * 100:.1f}%"
        )
    else:
        text = (
            f"raw-filtered RMSE={rmse(merged['raw_minus_filtered']):.4f} m | "
            f"max_abs={merged['raw_minus_filtered'].abs().max():.4f} m | "
            f"accept_rate={len(proc) / max(len(raw), 1) * 100:.1f}%"
        )
    ax.text(
        0.99,
        0.01,
        text,
        transform=ax.transAxes,
        ha="right",
        va="bottom",
        fontsize=8,
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
    )

    save_figure(fig, out_dir, f"{drone_name}_uwb_error.png", show)


def print_summary(drone_name: str, raw: pd.DataFrame, proc: pd.DataFrame) -> None:
    merged = build_merged(raw, proc)
    print(f"\n{drone_name}")
    print(f"  raw rows: {len(raw)} | processed rows: {len(proc)} | aligned rows: {len(merged)}")
    print(f"  targets: {list(target_ids(raw, proc))}")
    print(f"  accept_rate: {len(proc) / max(len(raw), 1) * 100:.1f}%")
    if "raw_error" in merged.columns:
        print(f"  raw error RMSE: {rmse(merged['raw_error']):.4f} m")
        print(f"  filtered error RMSE: {rmse(merged['filtered_error']):.4f} m")
    elif not merged.empty:
        print(f"  raw-filtered RMSE: {rmse(merged['raw_minus_filtered']):.4f} m")
        print("  note: CSV has no gt_distance; this is not true UWB error.")


def save_figure(fig: plt.Figure, out_dir: str, filename: str, show: bool) -> None:
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, filename)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"Saved {path}")
    if not show:
        plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot UWB raw, processed, and Gazebo truth curves.")
    parser.add_argument("--drone", type=str, default=None, help="Drone name, e.g. iris_0. Omit for all.")
    parser.add_argument("--run-id", type=str, default="auto",
                        help="Run ID (number or 'run_N'; 'auto'=latest; empty=old defaults)")
    parser.add_argument("--log-dir", type=str, default=None, help="Override log directory")
    parser.add_argument("--out-dir", type=str, default=None, help="Output directory for figures.")
    parser.add_argument("--no-show", action="store_true", help="Save figures only, do not display windows.")
    parser.add_argument("--summary-only", action="store_true", help="Print statistics without generating figures.")
    args = parser.parse_args()

    log_dir = os.path.abspath(args.log_dir) if args.log_dir else \
              resolve_log_dir(args.run_id, "uwb_zero_score", OLD_DEFAULT_LOG_DIR)
    _tag = args.run_id if args.run_id and args.run_id != "auto" \
           else os.path.basename(os.path.dirname(log_dir))
    run_tag = f"run_{_tag}" if _tag.isdigit() else _tag
    fig_base = os.path.join(FIGURE_BASE, run_tag)
    out_dir = os.path.abspath(args.out_dir) if args.out_dir else \
              os.path.join(fig_base, "uwb_plot")
    drones = [args.drone] if args.drone else DEFAULT_DRONES

    for drone in drones:
        raw, proc = load_data(log_dir, drone)
        if raw is None or proc is None:
            continue
        print_summary(drone, raw, proc)
        if args.summary_only:
            continue
        plot_compare(drone, raw, proc, out_dir, show=not args.no_show)
        plot_error(drone, raw, proc, out_dir, show=not args.no_show)

    if not args.no_show and not args.summary_only:
        plt.show()


if __name__ == "__main__":
    main()
