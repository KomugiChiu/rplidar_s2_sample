#!/bin/bash
# 停止 start_ekf.sh 啟動的行程。
#
# ros2 launch / ekf_node 收到 SIGTERM 後常常卡在 futex_wait 不會退出，
# 所以這裡先送 TERM、等幾秒，還活著就對整個 process group 送 SIGKILL。
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="${RUN_DIR:-/tmp/slam_toolbox_rf2o_ekf}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"

kill_group() {
  local sig="$1" pid="$2"
  kill "-$sig" -- "-$pid" 2>/dev/null || kill "-$sig" "$pid" 2>/dev/null || true
}

stop_pid() {
  local pid="$1" name="$2"
  kill_group TERM "$pid"
  for _ in 1 2 3 4 5 6 7 8; do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.5
  done
  echo "  $name ($pid) ignored SIGTERM, sending SIGKILL"
  kill_group KILL "$pid"
}

if [ -f "$PID_FILE" ]; then
  while read -r pid name; do
    [ -n "${pid:-}" ] || continue
    if kill -0 "$pid" 2>/dev/null; then
      echo "stopping $name ($pid)..."
      stop_pid "$pid" "$name"
    fi
  done < "$PID_FILE"
  rm -f "$PID_FILE"
else
  echo "(nothing running)"
fi

# 保險：按名字再掃一次（launch 的子行程可能已被 reparent 到 init）
sweep() {
  pkill -f "[r]ealsense2_camera_node" 2>/dev/null || true
  pkill -f "[e]kf_node" 2>/dev/null || true
  pkill -f "static_transform_publisher.*camera_link" 2>/dev/null || true
}
sweep
sleep 2
# 還活著就 KILL
pkill -KILL -f "[r]ealsense2_camera_node" 2>/dev/null || true
pkill -KILL -f "[e]kf_node" 2>/dev/null || true
pkill -KILL -f "static_transform_publisher.*camera_link" 2>/dev/null || true
echo "== ekf stopped =="
