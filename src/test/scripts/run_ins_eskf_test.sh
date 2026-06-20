#!/bin/bash
# 自动递增 run_id 启动 ins_eskf_test
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_ID=$(python3 "$SCRIPT_DIR/next_run_id.py")
echo "▶ run_id=$RUN_ID"
roslaunch data_process ins_eskf_test.launch run_id:="$RUN_ID" "$@"
