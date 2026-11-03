#!/usr/bin/env bash
# Start the robot web console on :8080 (background, like the other start_*.sh).
#
#   ./start_console.sh                 # http://<board-ip>:8080
#   ADDR=:8090 ./start_console.sh
#   STACK_DIR=/nvt_ros2_example/slam_toolbox_rf2o ./start_console.sh
#
# Builds what is missing first (one job at a time, memory capped: the board
# has no swap).  Stop with ./stop_console.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd)"
ADDR="${ADDR:-:8080}"
STACK_DIR="${STACK_DIR:-/nvt_ros2_example/slam_toolbox_rf2o}"
RUN_DIR="${RUN_DIR:-/tmp/robot_console}"
PID_FILE="$RUN_DIR/pid"
LOG_FILE="$RUN_DIR/console.log"
BIN="$SCRIPT_DIR/console/robot-console"
GW_BIN="$SCRIPT_DIR/gateway/build/robot_console_gateway"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

mkdir -p "$RUN_DIR"
if [ -s "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
  echo "robot-console already running (pid $(cat "$PID_FILE")); stop it with $SCRIPT_DIR/stop_console.sh"
  exit 0
fi

if [ ! -x "$GW_BIN" ]; then
  echo "building the ROS gateway (C++, takes ~15 min on the board) ..."
  "$SCRIPT_DIR/gateway/build.sh"
fi
if [ ! -x "$BIN" ] || [ -n "$(find "$SCRIPT_DIR/console" -newer "$BIN" \( -name '*.go' -o -path '*/web/*' \) -print -quit)" ]; then
  echo "building robot-console (Go) ..."
  (cd "$SCRIPT_DIR/console" && GOFLAGS=-mod=mod GOTOOLCHAIN=local nice -n 19 go build -p 1 -o robot-console .)
fi

setsid "$BIN" -addr "$ADDR" -stack-dir "$STACK_DIR" </dev/null >"$LOG_FILE" 2>&1 &
echo "$!" > "$PID_FILE"
sleep 1
if ! kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
  echo "ERROR: robot-console did not start"
  tail -n 20 "$LOG_FILE"
  exit 1
fi
IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "robot-console started (pid $(cat "$PID_FILE"))"
echo "  open : http://${IP:-<board-ip>}${ADDR}"
echo "  log  : $LOG_FILE"
echo "  stop : $SCRIPT_DIR/stop_console.sh"
