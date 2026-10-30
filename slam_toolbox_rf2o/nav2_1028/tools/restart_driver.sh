#!/usr/bin/env bash
# Restart only the chassis driver (Mcnamu_driver) so edited constants at the top
# of Mcnamu_driver.py (STRAIGHT_TRIM_*, MOTOR_GAIN, ...) take effect.
# Copies the source file into the install tree first (that copy is what runs).
#   ./restart_driver.sh
set -eo pipefail
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
ROBOT_WS="${ROBOT_WS:-/root/raspybot/yahboomcar_ws}"
LIBRARY_WS="${LIBRARY_WS:-/root/raspybot/library_ws/py_install}"
RUN_DIR="${RUN_DIR:-/tmp/raspybot_nav2}"
PID_FILE="$RUN_DIR/pids"; LOG="$RUN_DIR/logs/yahboomcar.log"
SRC="$ROBOT_WS/src/yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver.py"
DST="$ROBOT_WS/install/yahboomcar_bringup/lib/python3.12/site-packages/yahboomcar_bringup/Mcnamu_driver.py"
python3 -m py_compile "$SRC"
cp "$SRC" "$DST"
PAT="yahboomcar_bringup [M]cnamu_driver --ros-args|lib/yahboomcar_bringup/[M]cnamu_driver --ros-args"
pkill -TERM -f "$PAT" || true; sleep 1; pkill -KILL -f "$PAT" || true
mkdir -p "$RUN_DIR/logs"
setsid bash -c '
  source /opt/ros/jazzy/setup.bash
  source "'"$ROBOT_WS"'/install/local_setup.bash"
  export PYTHONPATH="'"$LIBRARY_WS"':${PYTHONPATH:-}"
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; unset ROS_LOCALHOST_ONLY
  exec ros2 run yahboomcar_bringup Mcnamu_driver --ros-args -r cmd_vel:=/cmd_vel
' </dev/null >"$LOG" 2>&1 &
pid=$!
touch "$PID_FILE"; sed -i '/ yahboomcar$/d' "$PID_FILE"; echo "$pid yahboomcar" >> "$PID_FILE"
for _ in $(seq 1 10); do grep -q 'wheel trim' "$LOG" 2>/dev/null && break; sleep 1; done
grep 'wheel trim' "$LOG" | tail -1 || { echo "driver did not start, see $LOG"; exit 1; }
