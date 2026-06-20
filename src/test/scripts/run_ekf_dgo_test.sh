#!/bin/bash
# 自动递增 run_id 启动 ekf_dgo_test
# 用法:  ./run_ekf_dgo_test.sh [额外 roslaunch 参数]
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_ID=$(python3 "$SCRIPT_DIR/next_run_id.py")
echo "▶ run_id=$RUN_ID"
roslaunch test ekf_dgo_test.launch run_id:="$RUN_ID" "$@"
