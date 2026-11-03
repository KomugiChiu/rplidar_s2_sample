#!/usr/bin/env bash
# Stop the robot web console (the ROS gateway exits with it).
# Does NOT touch the ROS stack, even if it was started from the console.
set -u
RUN_DIR="${RUN_DIR:-/tmp/robot_console}"
PID_FILE="$RUN_DIR/pid"

if [ -s "$PID_FILE" ]; then
  pid="$(cat "$PID_FILE")"
  if kill -0 "$pid" 2>/dev/null; then
    echo "stopping robot-console (pid $pid)"
    kill -TERM "$pid" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
      kill -0 "$pid" 2>/dev/null || break
      sleep 1
    done
    kill -KILL "$pid" 2>/dev/null || true
  fi
  rm -f "$PID_FILE"
fi
# leftovers (console started by hand)
pkill -TERM -f '[c]onsole/robot-console' 2>/dev/null || true
pkill -TERM -f '[b]uild/robot_console_gateway' 2>/dev/null || true
echo "robot-console stopped"
