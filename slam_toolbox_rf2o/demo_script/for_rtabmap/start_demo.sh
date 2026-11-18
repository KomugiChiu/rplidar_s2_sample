#!/usr/bin/env bash
# Full demo using RTAB-Map (2D lidar) instead of slam_toolbox.
#
# Starts:
#   RTAB-Map (mapping)  -> /rtabmap/map + map -> odom
#   rf2o + EKF          -> /odometry/filtered + odom -> base_footprint
#   Nav2 + chassis      -> nav2_params_rtabmap.yaml
#   AI + recovery + demo manager
#
# Usage:
#   ./start_demo.sh
#   START_AI=0 ./start_demo.sh
#   RECORD=1 ./start_demo.sh
#
# Stop with: ./stop_demo.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
# shellcheck source=../demo_common.sh
source "$SCRIPT_DIR/../demo_common.sh"

# common computes its own SCRIPT_DIR; restore this folder for print/stop paths.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
RTABMAP_DIR="${RTABMAP_DIR:-/nvt_ros2_example/rtabmap}"
NAV2_PARAMS_FILE="${RTABMAP_NAV2_PARAMS_FILE:-$RTABMAP_DIR/nav2_params_rtabmap.yaml}"
MAP_TOPIC="${MAP_TOPIC:-/rtabmap/map}"
export RTABMAP_DIR NAV2_PARAMS_FILE MAP_TOPIC

rtabmap_preflight() {
  mkdir -p "$LOG_DIR" "$RUN_DIR"
  [ -f "$NAV2_PARAMS_FILE" ] || demo_die "Nav2 params not found: $NAV2_PARAMS_FILE"
  [ -x "$RTABMAP_DIR/start_rtabmap.sh" ] || demo_die "missing $RTABMAP_DIR/start_rtabmap.sh"
  [ -x "$RTABMAP_DIR/stop_rtabmap.sh" ] || demo_die "missing $RTABMAP_DIR/stop_rtabmap.sh"
  [ -x "$RF2O_DIR/start.sh" ] || demo_die "missing $RF2O_DIR/start.sh"
  [ -x "$RF2O_DIR/stop.sh" ] || demo_die "missing $RF2O_DIR/stop.sh"
  [ -x "$EKF_DIR/start_ekf.sh" ] || demo_die "missing $EKF_DIR/start_ekf.sh"
  [ -x "$EKF_DIR/stop_ekf.sh" ] || demo_die "missing $EKF_DIR/stop_ekf.sh"
  [ -x "$NAV2_DIR/start_nav2.sh" ] || demo_die "missing $NAV2_DIR/start_nav2.sh"
  [ -x "$NAV2_DIR/stop_nav2.sh" ] || demo_die "missing $NAV2_DIR/stop_nav2.sh"

  if [ "$START_AI" = "1" ]; then
    [ -f "$REALSENSE_WS/local_setup.bash" ] || [ -f "$REALSENSE_WS/setup.bash" ] \
      || demo_die "RealSense workspace not found: $REALSENSE_WS"
    [ -f "$AI_WS/local_setup.bash" ] || [ -f "$AI_WS/setup.bash" ] \
      || demo_die "AI workspace not found: $AI_WS"
    [ -f "$DEMO_WS/install/local_setup.bash" ] \
      || demo_die "Demo workspace not found: $DEMO_WS/install"
    [ -x "$KIT_DIR/tools/start_ai_tf_only.sh" ] \
      || demo_die "missing $KIT_DIR/tools/start_ai_tf_only.sh"
    [ -x "$RECOVERY_INSTALL/start_person_target_recovery.sh" ] \
      || demo_die "missing $RECOVERY_INSTALL/start_person_target_recovery.sh"
    [ -x "$DEMO_INSTALL/start_demo_only.sh" ] \
      || demo_die "missing $DEMO_INSTALL/start_demo_only.sh"
  fi
}

rtabmap_wait() {
  # shellcheck disable=SC1090
  source /opt/ros/jazzy/setup.bash
  demo_log "      waiting for /rtabmap/map, /odometry/filtered and TF map -> base_link ..."
  local ready=0 map_info odom_info tf_out
  for _ in $(seq 1 30); do
    map_info="$(timeout 8 ros2 topic info /rtabmap/map 2>/dev/null || true)"
    odom_info="$(timeout 8 ros2 topic info /odometry/filtered 2>/dev/null || true)"
    tf_out="$(timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 || true)"
    if [[ "$map_info" =~ Publisher\ count:\ [1-9] ]] &&
       [[ "$odom_info" =~ Publisher\ count:\ [1-9] ]] &&
       [[ "$tf_out" == *Translation* ]]; then
      ready=1
      break
    fi
    sleep 2
  done
  [ "$ready" = "1" ] || demo_die "RTAB-Map/EKF not ready after 60 s"
  demo_log "      RTAB-Map/EKF ready"
}

rtabmap_preflight
demo_check_not_running

echo "== start_demo (RTAB-Map mapping) =="
echo "   domain : $ROS_DOMAIN_ID"
echo "   rtabmap: $RTABMAP_DIR"
echo "   nav2   : $NAV2_PARAMS_FILE"

demo_log "[1/4] RTAB-Map + lidar/TF"
"$RTABMAP_DIR/start_rtabmap.sh"

demo_log "[1/4] rf2o"
"$RF2O_DIR/start.sh"

demo_log "[2/4] RealSense + EKF"
"$EKF_DIR/start_ekf.sh"

rtabmap_wait

demo_log "[3/4] Nav2 + chassis driver"
demo_start_nav2

demo_log "[4/4] AI + person recovery + demo manager"
demo_start_ai

demo_maybe_record
demo_print_done
