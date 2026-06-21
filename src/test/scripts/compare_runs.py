#!/usr/bin/env python3
"""
compare_runs.py — 跨 run 飞行窗口 RMSE 对比

Usage:
  python3 compare_runs.py 21 22 23 24 25
  python3 compare_runs.py --run-id-range 19 21
  python3 compare_runs.py 21          # 单 run 也支持
  python3 compare_runs.py 18          # sync_diag 降级测试

对每个 run:
  1. 读取 DGO sync_diag → 提取飞行窗口 (排除 stage 0=WAIT, 7=DONE)
  2. Forward-fill 对齐 EKF/DGO 时间戳
  3. 重算窗口内 DGO/EKF RMSE
  4. 输出三栏对比表
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
RUN_LOG_BASE = os.path.join(REPO_ROOT, "run_data")

REFERENCE = "iris_0"
DRONES = ["iris_1", "iris_2", "iris_3"]

# 语义固定的阶段定义
STAGES_EXCLUDE = {0, 7}   # WAIT, DONE — 始终排除
PRE_FLIGHT = -1           # DGO 启动之前的时间戳


# ── 阶段对齐工具 ────────────────────────────────────────────────

def load_stage_map(run_id, drone=REFERENCE):
    """加载 sync_diag, 返回 (stamp, mission_stage) DataFrame.

    退化返回 None (文件不存在 / 空 / 仅有 header).
    """
    rid = str(run_id).replace("run_", "")
    path = os.path.join(RUN_LOG_BASE, f"run_{rid}",
                        "dgo", "sensor_sync_logs",
                        f"{drone}_dgo_sync_diag.csv")
    if not os.path.exists(path):
        return None

    try:
        with open(path) as f:
            reader = csv.DictReader(f)
            rows = [(float(r["stamp"]), int(float(r["mission_stage"])))
                    for r in reader
                    if r.get("stamp") and r.get("mission_stage")]
    except (ValueError, KeyError) as e:
        print(f"  [Warning] run_{rid} sync_diag parse failed: {e}")
        return None

    if not rows:
        print(f"  [Warning] run_{rid} sync_diag empty (仅 header)")
        return None

    stamps, stages = zip(*rows)
    return {"stamp": np.array(stamps, dtype=float),
            "stage": np.array(stages, dtype=int)}


def forward_fill_stage(timestamps, stage_map):
    """Forward-fill: 对每个 timestamp 赋予最近的前一个 stage.

    早于第一条记录的返回 PRE_FLIGHT (-1).
    """
    if stage_map is None:
        return None

    idx = np.searchsorted(stage_map["stamp"], timestamps, side="right") - 1
    result = np.where(idx >= 0, stage_map["stage"][idx], PRE_FLIGHT)
    return result


def is_flight(stage):
    """True 如果 stage 在飞行窗口内 (1-6)."""
    return int(stage) not in STAGES_EXCLUDE and int(stage) != PRE_FLIGHT


# ── 数据加载 ────────────────────────────────────────────────────

def resolve_log_dir(run_id, category, old_default=""):
    """若 run_data/run_{rid}/{category}/ 存在则返回, 否则返回 old_default."""
    rid = str(run_id).replace("run_", "")
    path = os.path.join(RUN_LOG_BASE, f"run_{rid}", category)
    if os.path.isdir(path):
        return path
    return old_default


def load_relative_csv(log_dir, drone, method):
    """加载单机 DGO/EKF 相对误差 CSV."""
    path = os.path.join(log_dir,
                        f"{drone}_relative_to_{REFERENCE}_{method}_error.csv")
    if not os.path.exists(path):
        return None

    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                t = float(r["timestamp"])
                e = float(r["error_norm_m"])
            except (ValueError, KeyError):
                continue
            rows.append((t, e))

    if not rows:
        return None

    rows.sort(key=lambda x: x[0])
    timestamps, errors = zip(*rows)
    return {"timestamp": np.array(timestamps, dtype=float),
            "error_norm": np.array(errors, dtype=float)}


def load_method(log_dir, method, stage_map=None):
    """加载同一 method 的全部无人机, 可选 flight-window 过滤."""
    result = {}
    for drone in DRONES:
        data = load_relative_csv(log_dir, drone, method)
        if data is None:
            continue

        if stage_map is not None:
            stages = forward_fill_stage(data["timestamp"], stage_map)
            if stages is not None:
                mask = np.array([is_flight(s) for s in stages])
                if mask.sum() > 0:
                    data = {"timestamp": data["timestamp"][mask],
                            "error_norm": data["error_norm"][mask]}
                else:
                    print(f"    {drone}: flight window empty, skip")
                    continue

        result[drone] = data
    return result


# ── RMSE 计算 ───────────────────────────────────────────────────

def compute_rmse(data):
    """计算 error_norm 的 RMSE."""
    if data is None or len(data["error_norm"]) == 0:
        return None
    return float(np.sqrt(np.mean(data["error_norm"] ** 2)))


def per_drone_rmse(dfs_dict):
    """返回 {drone: rmse} 字典."""
    return {drone: compute_rmse(data)
            for drone, data in dfs_dict.items()}


def aggregate_rmse(per_drone):
    """(mean, worst_drone, mean_value)."""
    values = [v for v in per_drone.values() if v is not None]
    if not values:
        return (None, None, None)
    mean_val = float(np.mean(values))
    worst_idx = int(np.argmax(values))
    worst_drone = [d for d in per_drone if per_drone[d] is not None][worst_idx]
    return (mean_val, worst_drone, float(values[worst_idx]))


# ── 主逻辑 ──────────────────────────────────────────────────────

def analyze_run(run_id, flight_only=True):
    """对单个 run 执行飞行窗口分析, 返回结果字典或 None."""
    rid = str(run_id).replace("run_", "")
    dgo_dir = resolve_log_dir(run_id, "ekf_dgo_test")
    ekf_dir = resolve_log_dir(run_id, "ins_eskf_test")

    if not dgo_dir and not ekf_dir:
        print(f"  Skip run_{rid}: no DGO or EKF data")
        return None

    stage_map = None
    flight_dur = None
    if flight_only:
        stage_map = load_stage_map(run_id)
        if stage_map is None:
            print(f"  [Warning] run_{rid} sync_diag unavailable, "
                  f"flight-only filter disabled")
        else:
            flight_stamps = stage_map["stamp"][
                np.array([is_flight(s) for s in stage_map["stage"]])]
            if len(flight_stamps) >= 2:
                flight_dur = float(flight_stamps[-1] - flight_stamps[0])

    dgo_dfs = load_method(dgo_dir, "dgo", stage_map) if dgo_dir else {}
    ekf_dfs = load_method(ekf_dir, "ekf", stage_map) if ekf_dir else {}

    if not dgo_dfs and not ekf_dfs:
        print(f"  Skip run_{rid}: no valid DGO or EKF data loaded")
        return None

    dgo_per = per_drone_rmse(dgo_dfs)
    ekf_per = per_drone_rmse(ekf_dfs)
    dgo_avg, dgo_worst, dgo_worst_v = aggregate_rmse(dgo_per)
    ekf_avg, ekf_worst, ekf_worst_v = aggregate_rmse(ekf_per)

    # 裁决
    ratio = None
    verdict = "N/A"
    if dgo_avg is not None and ekf_avg is not None and ekf_avg > 0:
        ratio = dgo_avg / ekf_avg
        if dgo_avg > 1.0 and ekf_avg > 1.0:
            verdict = "both poor"
        elif ratio < 0.9:
            verdict = "DGO better"
        elif ratio > 1.1:
            verdict = "EKF better"
        else:
            verdict = "comparable"

    n_dgo = sum(len(d["error_norm"]) for d in dgo_dfs.values())
    n_ekf = sum(len(d["error_norm"]) for d in ekf_dfs.values())

    return {
        "run_id": rid,
        "flight_dur": flight_dur,
        "n_dgo": n_dgo,
        "n_ekf": n_ekf,
        "dgo_mean_rmse": dgo_avg,
        "dgo_worst_drone": dgo_worst,
        "dgo_worst_rmse": dgo_worst_v,
        "dgo_per_drone": dgo_per,
        "ekf_mean_rmse": ekf_avg,
        "ekf_worst_drone": ekf_worst,
        "ekf_worst_rmse": ekf_worst_v,
        "ekf_per_drone": ekf_per,
        "ratio": ratio,
        "verdict": verdict,
    }


def print_table(results):
    """输出三栏对比表."""
    if not results:
        print("(no results)")
        return

    # 表头
    header = f"{'run':>5s}  {'dur(s)':>7s}  {'DGO_RMSE':>9s}  {'EKF_RMSE':>9s}  {'DGO/EKF':>8s}  {'verdict':>14s}"
    sep = "-" * len(header)
    print("\n" + sep)
    print(header)
    print(sep)

    for r in results:
        dur = f"{r['flight_dur']:.1f}" if r['flight_dur'] is not None else "N/A"
        dgo = f"{r['dgo_mean_rmse']:.4f}" if r['dgo_mean_rmse'] is not None else "N/A"
        ekf = f"{r['ekf_mean_rmse']:.4f}" if r['ekf_mean_rmse'] is not None else "N/A"
        ratio = f"{r['ratio']:.2f}" if r['ratio'] is not None else "N/A"
        print(f"  {r['run_id']:>3s}  {dur:>7s}  {dgo:>9s}  {ekf:>9s}  {ratio:>8s}  {r['verdict']:>14s}")

    print(sep)

    # 详细: 每架无人机的 RMSE
    print("\n━━━ 单机详情 ━━━━━━━━━━━━")
    for r in results:
        print(f"\n  run_{r['run_id']}:")
        for d in DRONES:
            dgo_v = r['dgo_per_drone'].get(d)
            ekf_v = r['ekf_per_drone'].get(d)
            dgo_s = f"{dgo_v:.4f}" if dgo_v is not None else "N/A"
            ekf_s = f"{ekf_v:.4f}" if ekf_v is not None else "N/A"
            print(f"    {d:>8s}  DGO={dgo_s}  EKF={ekf_s}", end="")
            if dgo_v is not None and ekf_v is not None and ekf_v > 0:
                r2 = dgo_v / ekf_v
                print(f"  ratio={r2:.2f}", end="")
            print()


def main():
    parser = argparse.ArgumentParser(
        description="跨 run 飞行窗口 RMSE 对比工具")
    parser.add_argument("run_ids", nargs="*", type=str,
                        help="Run IDs (e.g. 21 22 23)")
    parser.add_argument("--run-id-range", nargs=2, type=int, metavar=("START", "END"),
                        help="Run ID 范围 (含首尾)")
    parser.add_argument("--no-flight-only", dest="flight_only",
                        action="store_false", default=True,
                        help="禁用飞行窗口过滤 (使用全部数据)")
    args = parser.parse_args()

    # 解析 run ID 列表
    if args.run_id_range:
        run_ids = [str(i) for i in range(args.run_id_range[0],
                                          args.run_id_range[1] + 1)]
    elif args.run_ids:
        run_ids = args.run_ids
    else:
        # 默认: 所有有数据的 run
        dirs = sorted(
            glob.glob(os.path.join(RUN_LOG_BASE, "run_*")),
            key=lambda d: int(os.path.basename(d).split("_")[1]))
        run_ids = [os.path.basename(d).split("_")[1] for d in dirs]

    # 对每个 run 执行分析
    tag = "flight-window" if args.flight_only else "full-timeline"
    print(f"模式: {tag}")
    print(f"共 {len(run_ids)} 个 run")

    results = []
    for rid in run_ids:
        try:
            r = analyze_run(rid, flight_only=args.flight_only)
            if r is not None:
                results.append(r)
        except Exception as e:
            print(f"  [Error] run_{rid}: {e}")

    print_table(results)


if __name__ == "__main__":
    main()
