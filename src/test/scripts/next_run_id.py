#!/usr/bin/env python3
"""
自动递增 run_id:
  扫描 ~/swarm_localization/logs/run_* 目录,
  取最大编号 +1 作为本次 run_id。
  无历史目录时返回 1。
"""
import os, glob, re

LOG_DIR = os.path.expanduser("~/swarm_localization/logs")
os.makedirs(LOG_DIR, exist_ok=True)

max_id = 0
for d in glob.glob(os.path.join(LOG_DIR, "run_*")):
    if not os.path.isdir(d):
        continue
    m = re.search(r"run_(\d+)$", d)
    if m:
        max_id = max(max_id, int(m.group(1)))

print(max_id + 1)
