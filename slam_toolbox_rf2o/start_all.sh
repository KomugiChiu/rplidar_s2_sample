#!/usr/bin/env bash
# Start the whole slam_toolbox_rf2o + EKF + Nav2 (nav2_1028) stack.
#
#   ./start_all.sh
#   PARAMS_FILE=.../nav2_params_omni.yaml ./start_all.sh
#   RECORD=0 ./start_all.sh                 # do not start the cmd_vel logger
#
# Order: lidar+rf2o+slam_toolbox -> RealSense+EKF -> Nav2+speed_floor+driver.
set -eo pipefail

# Whole stack runs on one DDS domain (override with ROS_DOMAIN_ID=...).
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]:-$0}" 2>/dev/null || echo "${BASH_SOURCE[0]:-$0}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"

PARAMS_FILE="${PARAMS_FILE:-$SCRIPT_DIR/nav2_1028/nav2_params_omni_rotation.yaml}"
RECORD="${RECORD:-1}"
LOG_DIR="${LOG_DIR:-/tmp/nav2_logs}"
RECORDER="$SCRIPT_DIR/nav2_1028/tools/record_cmd_vel.py"

echo "== start_all (ROS_DOMAIN_ID=$ROS_DOMAIN_ID) =="
echo "   params: $PARAMS_FILE"

# --- 1. lidar + rf2o + slam_toolbox -----------------------------------------
echo "[1/3] lidar + rf2o + slam_toolbox"
"$SCRIPT_DIR/start_slam_toolbox_rf2o.sh"

# --- 2. RealSense IMU + EKF -------------------------------------------------
echo "[2/3] RealSense IMU + EKF"
"$SCRIPT_DIR/ekf/start_ekf.sh"

# --- wait until SLAM/EKF are actually visible -------------------------------
source /opt/ros/jazzy/setup.bash
echo "      waiting for /map, /odometry/filtered and TF map -> base_link ..."
READY=0
for _ in $(seq 1 30); do
    MAP_INFO="$(timeout 8 ros2 topic info /map 2>/dev/null || true)"
    ODOM_INFO="$(timeout 8 ros2 topic info /odometry/filtered 2>/dev/null || true)"
    TF_OUT="$(timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 || true)"
    if [[ "$MAP_INFO" =~ Publisher\ count:\ [1-9] ]] && \
       [[ "$ODOM_INFO" =~ Publisher\ count:\ [1-9] ]] && \
       [[ "$TF_OUT" == *Translation* ]]; then
        READY=1
        break
    fi
    sleep 2
done
if [ "$READY" != "1" ]; then
    echo "ERROR: SLAM/EKF not ready after 60 s"
    echo "       check /tmp/lidar_slam_toolbox.log and /tmp/slam_toolbox_rf2o_ekf/logs/"
    exit 1
fi
echo "      SLAM/EKF ready"

# --- 3. Nav2 + speed_floor + chassis driver ---------------------------------
echo "[3/3] Nav2 + speed_floor + chassis driver"
PARAMS_FILE="$PARAMS_FILE" "$SCRIPT_DIR/nav2_1028/start_nav2.sh"

# --- optional cmd_vel logger ------------------------------------------------
if [ "$RECORD" = "1" ] && [ -f "$RECORDER" ]; then
    mkdir -p "$LOG_DIR"
    pkill -f '[r]ecord_cmd_vel.py' 2>/dev/null || true
    LOG_FILE="$LOG_DIR/cmd_vel_$(date +%Y%m%d-%H%M%S).csv"
    setsid bash -c "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=$ROS_DOMAIN_ID; exec python3 '$RECORDER' '$LOG_FILE'" \
        </dev/null >"$LOG_DIR/logger.out" 2>&1 &
    echo "$!" > "$LOG_DIR/logger.pid"
    echo "      cmd_vel log: $LOG_FILE (pid $(cat "$LOG_DIR/logger.pid"))"
fi

echo
echo "== all started =="
echo "   domain : $ROS_DOMAIN_ID"
echo "   params : $PARAMS_FILE"
echo "   logs   : /tmp/raspybot_nav2/logs , /tmp/lidar_*.log , $LOG_DIR"
echo "   stop   : $SCRIPT_DIR/stop_all.sh"
