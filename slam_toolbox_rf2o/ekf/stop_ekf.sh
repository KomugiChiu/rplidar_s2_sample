#!/bin/bash
# 停止 start_ekf.sh 啟動的行程。
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="${RUN_DIR:-/tmp/slam_toolbox_rf2o_ekf}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"
[ -f "$PID_FILE" ] || { echo "(nothing running)"; exit 0; }
while read -r pid name; do
  [ -n "$pid" ] || continue
  if kill -0 "$pid" 2>/dev/null; then
    echo "stopping $name ($pid)..."
    kill "$pid" 2>/dev/null || true
  fi
done < "$PID_FILE"
sleep 2
# 保險：按名字再掃一次
pkill -f "[r]ealsense2_camera_node" 2>/dev/null || true
pkill -f "[e]kf_node" 2>/dev/null || true
pkill -f "static_transform_publisher.*camera_link" 2>/dev/null || true
rm -f "$PID_FILE"
echo "== ekf stopped =="
