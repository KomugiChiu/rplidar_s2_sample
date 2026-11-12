#!/usr/bin/env bash
# Start RealSense + AI detection + AI TF + DemoManager.
#
# Based on USB disk yuching/start_camera.md:
#   b. RealSense camera
#   c. ai_human_detection_node
#   d. DemoManager / AI sensor integration
#
# The v4.8.16 kit splits the old start_ai_sensor_integration.sh into:
#   tools/start_ai_tf_only.sh
#   tools/start_demo_manager_only.sh
# This script supports both layouts.
#
# Usage:
#   ./start_ai.sh
#   CAMERA_X=0.0 CAMERA_Y=0.0 CAMERA_Z=0.0 ./start_ai.sh
#   KIT_DIR=/path/to/kit ./start_ai.sh
#   START_REALSENSE=1 ./start_ai.sh   # AI only: start the camera here
#   DRY_RUN=1 ./start_ai.sh
#
# Stop with: ./stop_ai.sh

set -eo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"

REALSENSE_WS="${REALSENSE_WS:-/nvt_ros2_example/realsense-ros}"
AI_WS="${AI_WS:-/nvt_ros2_example/ai_human_detection}"
DEMO_WS="${DEMO_WS:-/root/ros2_ws}"
KIT_DIR="${KIT_DIR:-/nvt_ros2_example/demo_manager/fixed_build}"
# Person Follow Release v1.0.2: recovery -> /person_infos_stable -> demo_manager
RECOVERY_INSTALL="${RECOVERY_INSTALL:-/nvt_ros2_example/person_target_recovery/install}"
DEMO_INSTALL="${DEMO_INSTALL:-/nvt_ros2_example/demo_manager/install}"

CAMERA_X="${CAMERA_X:-0.0}"
CAMERA_Y="${CAMERA_Y:-0.0}"
CAMERA_Z="${CAMERA_Z:-0.0}"

LOG_DIR="${LOG_DIR:-/tmp/slam_toolbox_rf2o_ai}"
PID_FILE="${PID_FILE:-$LOG_DIR/pids}"
CAMERA_WAIT="${CAMERA_WAIT:-20}"
TOPIC_WAIT="${TOPIC_WAIT:-20}"
# 0 (default): do NOT start a camera here; reuse the one started by
#   ekf/start_ekf.sh (start_all.sh).  Starting a second node was the
#   source of duplicate RealSense processes.
# 1: start one here (AI-only setup); auto: reuse if a compatible node exists.
START_REALSENSE="${START_REALSENSE:-0}"
DRY_RUN="${DRY_RUN:-0}"

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]:-$0}" 2>/dev/null || echo "${BASH_SOURCE[0]:-$0}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"

log() { echo "[start_ai] $*"; }
die() { echo "[start_ai] ERROR: $*" >&2; exit 1; }

mkdir -p "$LOG_DIR"
log "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# --- preflight -------------------------------------------------------------
[ -f "$REALSENSE_WS/local_setup.bash" ] || [ -f "$REALSENSE_WS/setup.bash" ] \
  || die "RealSense workspace not found: $REALSENSE_WS"
[ -f "$AI_WS/local_setup.bash" ] || [ -f "$AI_WS/setup.bash" ] \
  || die "AI workspace not found: $AI_WS"
[ -f "$DEMO_WS/install/local_setup.bash" ] \
  || die "Demo workspace not found: $DEMO_WS/install"
[ -d "$KIT_DIR/tools" ] \
  || die "Kit tools not found: $KIT_DIR/tools"

# --- avoid double start ----------------------------------------------------
if [ -s "$PID_FILE" ]; then
  while read -r pid name; do
    [ -n "${pid:-}" ] || continue
    if kill -0 "$pid" 2>/dev/null; then
      log "$name is already running (pid $pid)"
      log "run $SCRIPT_DIR/stop_ai.sh first"
      exit 0
    fi
  done < "$PID_FILE"
fi
: > "$PID_FILE"

# --- ROS environment -------------------------------------------------------
# Use local_setup.bash where possible: the ai_human_detection setup.bash has an
# auto-generated apt-get block, local_setup.bash does not.
source /opt/ros/jazzy/setup.bash

if [ -f "$REALSENSE_WS/local_setup.bash" ]; then
  source "$REALSENSE_WS/local_setup.bash"
else
  source "$REALSENSE_WS/setup.bash"
fi

source "$DEMO_WS/install/local_setup.bash"

# Source the vendor AI workspace LAST.  /root/ros2_ws/install/ai_human_detection
# is a message-only package from the kit; it shadows the vendor package if it
# comes first, and "ros2 run ai_human_detection ai_human_detection_node" then
# fails with "No executable found".
if [ -f "$AI_WS/local_setup.bash" ]; then
  source "$AI_WS/local_setup.bash"
else
  source "$AI_WS/setup.bash"
fi

# Person Follow Release v1.0.2 prefixes (source after the SDK workspace so
# these prefixes win over the old demo_manager install).
if [ -f "$RECOVERY_INSTALL/local_setup.bash" ]; then
  source "$RECOVERY_INSTALL/local_setup.bash"
else
  die "person_target_recovery install not found: $RECOVERY_INSTALL"
fi
if [ -f "$DEMO_INSTALL/setup.bash" ]; then
  source "$DEMO_INSTALL/setup.bash"
else
  die "demo_manager install not found: $DEMO_INSTALL"
fi

# --- ensure the Person Follow Release is installed --------------------------
[ -x "$RECOVERY_INSTALL/start_person_target_recovery.sh" ] \
  || die "missing $RECOVERY_INSTALL/start_person_target_recovery.sh"
[ -x "$DEMO_INSTALL/start_demo_only.sh" ] \
  || die "missing $DEMO_INSTALL/start_demo_only.sh"
DEMO_PREFIX="$(ros2 pkg prefix demo_manager 2>/dev/null || true)"
case "$DEMO_PREFIX" in
  "$DEMO_INSTALL"/*) ;;
  *) die "demo_manager resolves to '${DEMO_PREFIX:-<none>}', expected under $DEMO_INSTALL" ;;
esac
ros2 pkg prefix person_target_recovery >/dev/null 2>&1 \
  || die "person_target_recovery not found"

ros2 pkg prefix ai_human_detection >/dev/null 2>&1 \
  || die "ai_human_detection not found"
ros2 pkg executables ai_human_detection 2>/dev/null | grep -q 'ai_human_detection_node' \
  || die "ai_human_detection_node not found (check AMENT_PREFIX_PATH; workspace ai_human_detection may shadow the vendor package)"
ros2 pkg prefix realsense2_camera >/dev/null 2>&1 \
  || die "realsense2_camera not found"

# --- helpers ---------------------------------------------------------------
start_bg() {
  local name="$1"; shift
  local logfile="$LOG_DIR/$name.log"

  if [ "$DRY_RUN" = "1" ]; then
    log "DRY_RUN: $name: $*"
    return 0
  fi

  setsid "$@" </dev/null >"$logfile" 2>&1 &
  local pid=$!
  echo "$pid $name" >> "$PID_FILE"
  log "$name started pid=$pid log=$logfile"
}

topic_seen() {
  timeout 5 ros2 topic list 2>/dev/null | grep -qx "$1"
}

# Auto-detect must tolerate a slow ROS graph on this loaded SoC: use a longer
# timeout and retry a few times before declaring the camera IMU-only.
topic_available() {
  local topic="$1" i
  for i in 1 2 3; do
    if timeout 10 ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_topic() {
  local topic="$1" timeout="$2" name="$3"
  [ "$DRY_RUN" = "1" ] && return 0

  local i
  for i in $(seq 1 "$timeout"); do
    if topic_seen "$topic"; then
      log "$name ready: $topic"
      return 0
    fi
    sleep 1
  done

  log "WARN: $name topic not seen after ${timeout}s: $topic"
  return 1
}

# --- 1. RealSense ----------------------------------------------------------
# auto: reuse an existing combined IMU+RGBD RealSense node; otherwise start one.
if [ "$START_REALSENSE" = "auto" ]; then
  if pgrep -f '[r]ealsense2_camera_node' >/dev/null; then
    log "checking existing RealSense RGBD configuration..."
    if pgrep -af 'ros2 launch realsense2_camera rs_launch.py' | grep -q 'enable_color:=true'; then
      log "existing RealSense launch has color enabled; using it (skip own launch)"
      START_REALSENSE=0
    elif topic_available /camera/camera/color/image_raw || \
         topic_available /camera/camera/color/image_raw/compressed; then
      log "existing RealSense with RGBD topics detected; using it (skip own launch)"
      START_REALSENSE=0
    else
      log "WARN: existing RealSense is IMU-only (no color topic)"
      log "      update ekf/start_ekf.sh to enable RGBD, or set START_REALSENSE=0"
      die "IMU-only RealSense conflict; see message above"
    fi
  else
    START_REALSENSE=1
  fi
fi

if [ "$START_REALSENSE" = "1" ]; then
  if pgrep -f '[r]ealsense2_camera_node' >/dev/null; then
    log "WARN: another RealSense node is already running (EKF IMU-only?)"
    if [ "${ALLOW_REALSENSE_CONFLICT:-0}" != "1" ]; then
      die "set ALLOW_REALSENSE_CONFLICT=1 to start a second RealSense node anyway"
    fi
  fi

  log "starting RealSense (domain $ROS_DOMAIN_ID)"
  start_bg realsense ros2 launch realsense2_camera rs_launch.py \
    enable_gyro:=true enable_accel:=true \
    enable_rgbd:=true \
    enable_sync:=true \
    align_depth.enable:=true \
    enable_color:=true \
    enable_depth:=true \
    rgb_camera.color_profile:=424x240x15 \
    depth_module.depth_profile:=424x240x15

  # The AI node uses the compressed color image and aligned depth.
  if [ "$DRY_RUN" != "1" ]; then
    for i in $(seq 1 "$CAMERA_WAIT"); do
      if ros2 topic list 2>/dev/null | grep -qE '^/camera/camera/color/image_raw(/compressed)?$'; then
        log "RealSense color topic ready"
        break
      fi
      sleep 1
    done
    ros2 topic list 2>/dev/null | grep -qE '^/camera/camera/color/image_raw(/compressed)?$' \
      || log "WARN: RealSense color topic not seen after ${CAMERA_WAIT}s"
  fi
else
  log "START_REALSENSE=0: skipping RealSense launch"
fi

# --- 2. AI detection node --------------------------------------------------
if [ ! -e /dev/nvt_gfx0 ]; then
  log "loading nvt_gfx kernel module"
  modprobe nvt_gfx 2>/dev/null || log "WARN: modprobe nvt_gfx failed"
fi
if [ ! -e /dev/nvt_gfx0 ]; then
  log "WARN: /dev/nvt_gfx0 still missing; ai_human_detection_node may fail"
fi

log "starting ai_human_detection_node"
start_bg ai_node ros2 run ai_human_detection ai_human_detection_node

if [ "$DRY_RUN" != "1" ] && ! pgrep -f '[a]i_human_detection_node' >/dev/null; then
  log "ERROR: ai_human_detection_node is not running; see $LOG_DIR/ai_node.log"
fi

wait_topic /person_infos "$TOPIC_WAIT" "AI node" || true

# --- 3. AI TF + Person Target Recovery + DemoManager -----------------------
[ -x "$KIT_DIR/tools/start_ai_tf_only.sh" ] \
  || die "AI TF tool not found: $KIT_DIR/tools/start_ai_tf_only.sh"

log "starting AI TF (camera offset: $CAMERA_X $CAMERA_Y $CAMERA_Z)"
start_bg ai_tf "$KIT_DIR/tools/start_ai_tf_only.sh" \
  camera_x:="$CAMERA_X" camera_y:="$CAMERA_Y" camera_z:="$CAMERA_Z"

log "starting person_target_recovery (release v1.0.2)"
start_bg person_target_recovery "$RECOVERY_INSTALL/start_person_target_recovery.sh"
wait_topic /person_infos_stable "$TOPIC_WAIT" "person_target_recovery" || true

log "starting demo_manager_node (release v1.0.2, /person_infos_stable)"
DEMO_PARAMS="$DEMO_INSTALL/demo_manager/share/demo_manager/config/demo_manager.yaml"
[ -f "$DEMO_PARAMS" ] || die "demo_manager params not found: $DEMO_PARAMS"
start_bg demo_manager ros2 run demo_manager demo_manager_node \
  --ros-args --params-file "$DEMO_PARAMS"

if [ "$DRY_RUN" != "1" ]; then
  wait_topic /demo/person_markers "$TOPIC_WAIT" "demo manager" || true
fi

echo
log "AI stack started"
log "  logs: $LOG_DIR"
log "  pids: $PID_FILE"
log "  stop: $SCRIPT_DIR/stop_ai.sh"
log "  check: $KIT_DIR/tools/check_full_stack.sh"
