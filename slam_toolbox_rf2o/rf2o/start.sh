#!/usr/bin/env bash
# Start rf2o laser odometry only.
#
# Requires:
#   /scan                 (rplidar_node, started by the slam_toolbox scripts)
#   base_footprint -> laser static TF
#
# It does NOT start the lidar, the static TFs, slam_toolbox, EKF, or Nav2.
#
# Usage:
#   ./start.sh
#   RF2O_PARAMS=/path/other.yaml ./start.sh
#
# Stop with: ./stop.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
RUN_DIR="${RUN_DIR:-/tmp/rf2o_only}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"
LOG_DIR="${LOG_DIR:-$RUN_DIR/logs}"
LOG_FILE="$LOG_DIR/rf2o.log"
RF2O_PARAMS="${RF2O_PARAMS:-$SCRIPT_DIR/rf2o_params.yaml}"

# shellcheck disable=SC1090
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1090
source "$SCRIPT_DIR/install/local_setup.bash"

mkdir -p "$RUN_DIR" "$LOG_DIR"
[ -f "$RF2O_PARAMS" ] || { echo "ERROR: params not found: $RF2O_PARAMS" >&2; exit 1; }

if pgrep -f '[r]f2o_laser_odometry_node' >/dev/null 2>&1; then
  echo "ERROR: rf2o_laser_odometry_node is already running; stop it first." >&2
  exit 1
fi

if [ -s "$PID_FILE" ]; then
  while read -r pid name; do
    if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
      echo "ERROR: $name is already running (pid $pid); run ./stop.sh first" >&2
      exit 1
    fi
  done < "$PID_FILE"
fi

if ! ros2 topic info /scan 2>/dev/null | grep -qE 'Publisher count: [1-9]'; then
  echo "ERROR: /scan has no publisher. Start the slam_toolbox/lidar script first." >&2
  exit 1
fi

# rf2o reads the lidar mounting TF once, on the first scan.
for _ in $(seq 1 10); do
  if ( timeout 3 ros2 run tf2_ros tf2_echo base_footprint laser 2>&1 || true ) |
       grep -q "Translation"; then
    break
  fi
  sleep 1
done

echo "starting rf2o: $RF2O_PARAMS"
setsid ros2 run rf2o_laser_odometry rf2o_laser_odometry_node \
  --ros-args --params-file "$RF2O_PARAMS" \
  </dev/null >"$LOG_FILE" 2>&1 &
PID=$!
echo "$PID rf2o" > "$PID_FILE"

sleep 2
if ! kill -0 "$PID" 2>/dev/null; then
  echo "ERROR: rf2o died; see $LOG_FILE" >&2
  tail -n 30 "$LOG_FILE" >&2 || true
  rm -f "$PID_FILE"
  exit 1
fi

echo "rf2o started pid=$PID"
echo "  odom : /odom_rf2o"
echo "  log  : $LOG_FILE"
echo "  stop : $SCRIPT_DIR/stop.sh"
