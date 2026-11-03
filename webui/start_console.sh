#!/usr/bin/env bash
# Start the robot web console on :8080 (background, like the other start_*.sh).
#
#   ./start_console.sh                 # C++ gateway (cross-compiled binary in gateway/)
#   ./start_console_py.sh              # Python gateway instead (slower, no build needed)
#   ADDR=:8090 ./start_console.sh
#   STACK_DIR=/nvt_ros2_example/slam_toolbox_rf2o ./start_console.sh
#
# The C++ gateway is NOT built here: compiling it on the board takes too long
# and too much memory.  Stop with ./stop_console.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd)"
ADDR="${ADDR:-:8080}"
STACK_DIR="${STACK_DIR:-/nvt_ros2_example/slam_toolbox_rf2o}"
GATEWAY="${GATEWAY:-$SCRIPT_DIR/gateway/run_gateway.sh}"
RUN_DIR="${RUN_DIR:-/tmp/robot_console}"
PID_FILE="$RUN_DIR/pid"
LOG_FILE="$RUN_DIR/console.log"
BIN="$SCRIPT_DIR/console/robot-console"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

mkdir -p "$RUN_DIR"
if [ -s "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
  echo "robot-console already running (pid $(cat "$PID_FILE")); stop it with $SCRIPT_DIR/stop_console.sh"
  exit 0
fi

if [ "$GATEWAY" = "$SCRIPT_DIR/gateway/run_gateway.sh" ] && \
   [ ! -x "$SCRIPT_DIR/gateway/robot_console_gateway" ] && [ ! -x "$SCRIPT_DIR/gateway/build/robot_console_gateway" ]; then
  echo "ERROR: C++ gateway binary not found."
  echo "       put the cross-compiled binary at $SCRIPT_DIR/gateway/robot_console_gateway"
  echo "       or use the Python gateway: $SCRIPT_DIR/start_console_py.sh"
  exit 1
fi

# Go build: one package at a time, low priority, memory capped (no swap on the board)
if [ ! -x "$BIN" ] || [ -n "$(find "$SCRIPT_DIR/console" -newer "$BIN" \( -name '*.go' -o -path '*/web/*' \) -print -quit)" ]; then
  echo "building robot-console (Go) ..."
  CAP=()
  if command -v systemd-run >/dev/null 2>&1 && grep -qw memory /sys/fs/cgroup/cgroup.controllers 2>/dev/null; then
    CAP=(systemd-run --scope --quiet -p MemoryMax=900M -p MemorySwapMax=0)
  fi
  (cd "$SCRIPT_DIR/console" && "${CAP[@]}" env GOFLAGS=-mod=mod GOTOOLCHAIN=local nice -n 19 go build -p 1 -o robot-console .)
fi

setsid "$BIN" -addr "$ADDR" -stack-dir "$STACK_DIR" -gateway "$GATEWAY" </dev/null >"$LOG_FILE" 2>&1 &
echo "$!" > "$PID_FILE"
sleep 1
if ! kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
  echo "ERROR: robot-console did not start"
  tail -n 20 "$LOG_FILE"
  exit 1
fi
IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "robot-console started (pid $(cat "$PID_FILE"))"
echo "  gateway: $GATEWAY"
echo "  open   : http://${IP:-<board-ip>}${ADDR}"
echo "  log    : $LOG_FILE"
echo "  stop   : $SCRIPT_DIR/stop_console.sh"
