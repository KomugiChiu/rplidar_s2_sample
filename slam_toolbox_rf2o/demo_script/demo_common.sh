#!/usr/bin/env bash
# Shared helpers for start_demo.sh and start_demo_static.sh.
# This file is sourced; it is not meant to be executed directly.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-$ROOT_DIR/nav2_1028/nav2_params_plugin.yaml}"
USE_EXTERNAL_SAFETY="${USE_EXTERNAL_SAFETY:-0}"
export USE_EXTERNAL_SAFETY
export USE_LOCALIZATION=0
RECORD="${RECORD:-0}"
START_AI="${START_AI:-1}"
START_REALSENSE="${START_REALSENSE:-0}"
CAMERA_X="${CAMERA_X:-0.0}"
CAMERA_Y="${CAMERA_Y:-0.0}"
CAMERA_Z="${CAMERA_Z:-0.0}"

LOG_DIR="${LOG_DIR:-/tmp/demo_script}"
RUN_DIR="${RUN_DIR:-/tmp/demo_script}"
AI_PID_FILE="${AI_PID_FILE:-$RUN_DIR/ai_pids}"
TOPIC_WAIT="${TOPIC_WAIT:-20}"

REALSENSE_WS="${REALSENSE_WS:-/nvt_ros2_example/realsense-ros}"
AI_WS="${AI_WS:-/nvt_ros2_example/ai_human_detection}"
DEMO_WS="${DEMO_WS:-$HOME/ros2_ws}"
KIT_DIR="${KIT_DIR:-/nvt_ros2_example/demo_manager/fixed_build}"
RECOVERY_INSTALL="${RECOVERY_INSTALL:-/nvt_ros2_example/person_target_recovery/install}"
DEMO_INSTALL="${DEMO_INSTALL:-/nvt_ros2_example/demo_manager/install}"

demo_log()  { echo "[demo] $*"; }
demo_die()  { echo "[demo] ERROR: $*" >&2; exit 1; }

demo_source_if_exists() {
  local path="$1"
  if [ -f "$path/local_setup.bash" ]; then
    # shellcheck disable=SC1090
    source "$path/local_setup.bash" >/dev/null 2>&1 || true
    return 0
  fi
  if [ -f "$path/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "$path/setup.bash" >/dev/null 2>&1 || true
    return 0
  fi
  return 1
}

demo_topic_seen() {
  timeout 5 ros2 topic list 2>/dev/null | grep -qx "$1"
}

demo_wait_topic() {
  local topic="$1" timeout_s="$2" name="$3" i
  for i in $(seq 1 "$timeout_s"); do
    if demo_topic_seen "$topic"; then
      demo_log "$name ready: $topic"
      return 0
    fi
    sleep 1
  done
  demo_log "WARN: $name topic not seen after ${timeout_s}s: $topic"
  return 1
}

demo_start_bg() {
  local name="$1"; shift
  local logfile="$LOG_DIR/$name.log"

  if [ "${DRY_RUN:-0}" = "1" ]; then
    demo_log "DRY_RUN: $name: $*"
    return 0
  fi

  setsid "$@" </dev/null >"$logfile" 2>&1 &
  local pid=$!
  echo "$pid $name" >> "$AI_PID_FILE"
  demo_log "$name started pid=$pid log=$logfile"
}

demo_preflight() {
  mkdir -p "$LOG_DIR" "$RUN_DIR"

  [ -f "$NAV2_PARAMS_FILE" ] || demo_die "Nav2 params not found: $NAV2_PARAMS_FILE"
  [ -x "$ROOT_DIR/start_slam_toolbox_rf2o.sh" ] || demo_die "missing start_slam_toolbox_rf2o.sh"
  [ -x "$ROOT_DIR/ekf/start_ekf.sh" ] || demo_die "missing ekf/start_ekf.sh"
  [ -x "$ROOT_DIR/nav2_1028/start_nav2.sh" ] || demo_die "missing nav2_1028/start_nav2.sh"

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

demo_check_not_running() {
  if [ -s "$AI_PID_FILE" ]; then
    while read -r pid name; do
      [ -n "${pid:-}" ] || continue
      if kill -0 "$pid" 2>/dev/null; then
        demo_die "$name is already running (pid $pid); stop the demo first"
      fi
    done < "$AI_PID_FILE"
    rm -f "$AI_PID_FILE"
  fi
  if pgrep -f '[a]i_human_detection_node' >/dev/null 2>&1; then
    demo_die "ai_human_detection_node is already running; stop the demo first"
  fi
  : > "$AI_PID_FILE"
}

demo_validate_posegraph() {
  local map_file
  map_file="$(sed -n 's/^[[:space:]]*map_file_name:[[:space:]]*"\{0,1\}\([^"]*\)"\{0,1\}[[:space:]]*$/\1/p' \
    "$SLAM_PARAMS" | head -n1)"
  [ -n "$map_file" ] || demo_die "no map_file_name in $SLAM_PARAMS"
  [ -f "${map_file}.posegraph" ] && [ -f "${map_file}.data" ] \
    || demo_die "posegraph not found: ${map_file}.posegraph / .data"
  if ! grep -qE '^[[:space:]]*(map_start_pose|map_start_at_dock):' "$SLAM_PARAMS"; then
    demo_log "WARN: neither map_start_pose nor map_start_at_dock is set in $SLAM_PARAMS"
  fi
}

demo_start_slam() {
  demo_log "[1/4] SLAM (mode=$SLAM_MODE)"
  "$ROOT_DIR/start_slam_toolbox_rf2o.sh"
}

demo_start_ekf() {
  demo_log "[2/4] RealSense + EKF"
  "$ROOT_DIR/ekf/start_ekf.sh"
}

demo_wait_for_slam() {
  # shellcheck disable=SC1090
  source /opt/ros/jazzy/setup.bash
  demo_log "      waiting for /map, /odometry/filtered and TF map -> base_link ..."
  local ready=0 odom_info map_info tf_out
  for _ in $(seq 1 30); do
    odom_info="$(timeout 8 ros2 topic info /odometry/filtered 2>/dev/null || true)"
    map_info="$(timeout 8 ros2 topic info /map 2>/dev/null || true)"
    tf_out="$(timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>&1 || true)"
    if [[ "$map_info" =~ Publisher\ count:\ [1-9] ]] &&
       [[ "$odom_info" =~ Publisher\ count:\ [1-9] ]] &&
       [[ "$tf_out" == *Translation* ]]; then
      ready=1
      break
    fi
    sleep 2
  done
  [ "$ready" = "1" ] || demo_die "SLAM/EKF not ready after 60 s"
  demo_log "      SLAM/EKF ready"
}

demo_start_nav2() {
  demo_log "[3/4] Nav2 + chassis driver"
  PARAMS_FILE="$NAV2_PARAMS_FILE" "$ROOT_DIR/nav2_1028/start_nav2.sh"
}

demo_source_ai_env() {
  # shellcheck disable=SC1090
  source /opt/ros/jazzy/setup.bash
  demo_source_if_exists "$REALSENSE_WS" || true
  demo_source_if_exists "$DEMO_WS/install" || true
  # AI last, so the vendor package wins over the message-only kit package.
  demo_source_if_exists "$AI_WS" || true
  demo_source_if_exists "$RECOVERY_INSTALL" || true
  demo_source_if_exists "$DEMO_INSTALL" || true
}

demo_start_ai() {
  [ "$START_AI" = "1" ] || { demo_log "[4/4] AI disabled (START_AI=0)"; return 0; }

  demo_log "[4/4] AI + person recovery + demo manager"
  demo_source_ai_env

  ros2 pkg prefix ai_human_detection >/dev/null 2>&1 \
    || demo_die "ai_human_detection not found"
  ros2 pkg executables ai_human_detection 2>/dev/null | grep -q 'ai_human_detection_node' \
    || demo_die "ai_human_detection_node not found"
  ros2 pkg prefix person_target_recovery >/dev/null 2>&1 \
    || demo_die "person_target_recovery not found"
  demo_prefix="$(ros2 pkg prefix demo_manager 2>/dev/null || true)"
  case "$demo_prefix" in
    "$DEMO_INSTALL"/*) ;;
    *) demo_die "demo_manager resolves to '${demo_prefix:-<none>}', expected under $DEMO_INSTALL" ;;
  esac

  if [ "$START_REALSENSE" = "1" ] && ! pgrep -f '[r]ealsense2_camera_node' >/dev/null 2>&1; then
    demo_die "START_REALSENSE=1 but no camera is running; start EKF first"
  fi

  if [ ! -e /dev/nvt_gfx0 ]; then
    demo_log "loading nvt_gfx kernel module"
    modprobe nvt_gfx 2>/dev/null || demo_log "WARN: modprobe nvt_gfx failed"
  fi

  demo_start_bg ai_node ros2 run ai_human_detection ai_human_detection_node
  demo_wait_topic /person_infos "$TOPIC_WAIT" "AI node" || true

  demo_start_bg ai_tf "$KIT_DIR/tools/start_ai_tf_only.sh" \
    camera_x:="$CAMERA_X" camera_y:="$CAMERA_Y" camera_z:="$CAMERA_Z"

  demo_start_bg person_target_recovery "$RECOVERY_INSTALL/start_person_target_recovery.sh"
  demo_wait_topic /person_infos_stable "$TOPIC_WAIT" "person_target_recovery" || true

  demo_start_bg demo_manager "$DEMO_INSTALL/start_demo_only.sh"
  demo_wait_topic /demo/person_markers "$TOPIC_WAIT" "demo manager" || true
}

demo_maybe_record() {
  local recorder="$ROOT_DIR/nav2_1028/tools/record_cmd_vel.py"
  [ "$RECORD" = "1" ] || return 0
  [ -f "$recorder" ] || { demo_log "WARN: recorder not found: $recorder"; return 0; }

  mkdir -p "$LOG_DIR"
  pkill -f '[r]ecord_cmd_vel.py' 2>/dev/null || true
  local log_file="$LOG_DIR/cmd_vel_$(date +%Y%m%d-%H%M%S).csv"
  setsid bash -c "source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=$ROS_DOMAIN_ID; exec python3 '$recorder' '$log_file'" \
    </dev/null >"$LOG_DIR/logger.out" 2>&1 &
  echo "$!" > "$LOG_DIR/logger.pid"
  demo_log "cmd_vel log: $log_file (pid $(cat "$LOG_DIR/logger.pid"))"
}

demo_print_done() {
  echo
  demo_log "started"
  echo "   domain : $ROS_DOMAIN_ID"
  echo "   nav2   : $NAV2_PARAMS_FILE"
  echo "   logs   : $LOG_DIR"
  echo "   stop   : $ROOT_DIR/stop_demo.sh"
}
