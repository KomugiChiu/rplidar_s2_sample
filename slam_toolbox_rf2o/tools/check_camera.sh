#!/usr/bin/env bash
# check_camera.sh — RealSense D455 capability / link check
#
# Usage:
#   ./check_camera.sh                 # inspect a running /camera/camera node
#   ./check_camera.sh --probe         # launch a temporary camera, check, then stop it
#   ./check_camera.sh --probe --color 424x240x15 --depth 424x240x15
#
# Exit code:
#   0 = requested color/depth profiles supported and active
#   1 = not supported / not active / camera not running (without --probe)
#
# Options:
#   --probe            launch a temporary RealSense node if none is running
#   --color  WxHxFPS   desired color profile   (default 424x240x15)
#   --depth  WxHxFPS   desired depth profile   (default 424x240x15)
#   --domain N         ROS_DOMAIN_ID           (default 99)
#   --node NAME        camera node name        (default /camera/camera)
#   --timeout N        seconds to wait for the probe node (default 30)
#   --no-rates         skip topic hz checks
#   -h, --help
#
# Persistent copy: /temp/script/check_camera.sh

set -uo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
COLOR_PROFILE="424x240x15"
DEPTH_PROFILE="424x240x15"
ROS_DOMAIN="${ROS_DOMAIN_ID:-99}"
CAMERA_NODE="/camera/camera"
PROBE=0
DO_RATES=1
WAIT_SEC=30
HZ_TIMEOUT=8

while [ $# -gt 0 ]; do
  case "$1" in
    --probe) PROBE=1 ;;
    --color) COLOR_PROFILE="$2"; shift ;;
    --depth) DEPTH_PROFILE="$2"; shift ;;
    --domain) ROS_DOMAIN="$2"; shift ;;
    --node) CAMERA_NODE="$2"; shift ;;
    --timeout) WAIT_SEC="$2"; shift ;;
    --no-rates) DO_RATES=0 ;;
    -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
  shift
done

export ROS_DOMAIN_ID="$ROS_DOMAIN"
# ROS setup.bash references unbound vars; disable nounset while sourcing it.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP" >/dev/null 2>&1
set -u

TS="$(date +%Y%m%d-%H%M%S)"
PROBE_LOG="/tmp/check_camera_${TS}.log"
PROBE_PID=""
FAIL=0

say() { printf '%s\n' "$*"; }
ok()   { printf '  [ OK ] %s\n' "$*"; }
bad()  { printf '  [FAIL] %s\n' "$*"; FAIL=1; }
warn() { printf '  [WARN] %s\n' "$*"; }

cleanup() {
  if [ -n "$PROBE_PID" ]; then
    say ""
    say "== stopping temporary camera (pgid $PROBE_PID) =="
    kill -TERM -- "-$PROBE_PID" 2>/dev/null || kill -TERM "$PROBE_PID" 2>/dev/null
    sleep 3
    kill -KILL -- "-$PROBE_PID" 2>/dev/null || true
    PROBE_PID=""
  fi
}
trap cleanup EXIT

say "== check_camera =="
say "time        : $(date '+%Y-%m-%d %H:%M:%S')"
say "host        : $(hostname)"
say "ROS_DOMAIN  : $ROS_DOMAIN_ID"
say "node        : $CAMERA_NODE"
say "requested   : color=$COLOR_PROFILE  depth=$DEPTH_PROFILE"

# ---------------------------------------------------------------------------
# USB link
# ---------------------------------------------------------------------------
say ""
say "== USB link =="
USB_FOUND=0
for dev in /sys/bus/usb/devices/*; do
  [ -r "$dev/idVendor" ] || continue
  [ "$(cat "$dev/idVendor" 2>/dev/null)" = "8086" ] || continue
  product="$(cat "$dev/product" 2>/dev/null || true)"
  case "$product" in
    *RealSense*) ;;
    *) continue ;;
  esac
  USB_FOUND=1
  speed="$(cat "$dev/speed" 2>/dev/null || echo '?')"
  busnum="$(cat "$dev/busnum" 2>/dev/null || echo '?')"
  devnum="$(cat "$dev/devnum" 2>/dev/null || echo '?')"
  vid="$(cat "$dev/idVendor" 2>/dev/null || echo '?')"
  pid="$(cat "$dev/idProduct" 2>/dev/null || echo '?')"
  say "  $product"
  say "  bus=$busnum dev=$devnum  id=$vid:$pid  speed=${speed} Mbps"
  if [ "$speed" = "5000" ] || [ "$speed" = "10000" ] || [ "$speed" = "20000" ]; then
    ok "USB 3 link (${speed} Mbps)"
  else
    warn "USB 2 link (${speed} Mbps): depth 424x240 may be unsupported"
  fi
done
[ "$USB_FOUND" = "1" ] || bad "no Intel RealSense USB device found"

# ---------------------------------------------------------------------------
# Is the camera node running?
# ---------------------------------------------------------------------------
say ""
say "== camera node =="
NODE_RUNNING=0
if ros2 node list 2>/dev/null | grep -qx "$CAMERA_NODE"; then
  NODE_RUNNING=1
  ok "$CAMERA_NODE is running"
else
  warn "$CAMERA_NODE is not running"
  if [ "$PROBE" = "1" ]; then
    say "  launching temporary camera (log: $PROBE_LOG)"
    setsid bash -c "source '$ROS_SETUP' >/dev/null 2>&1; export ROS_DOMAIN_ID='$ROS_DOMAIN'; \
      exec ros2 launch realsense2_camera rs_launch.py \
        enable_gyro:=true enable_accel:=true \
        enable_rgbd:=true enable_sync:=true align_depth.enable:=true \
        enable_color:=true enable_depth:=true \
        enable_infra1:=false enable_infra2:=false \
        rgb_camera.color_profile:='$COLOR_PROFILE' \
        depth_module.depth_profile:='$DEPTH_PROFILE'" \
      >"$PROBE_LOG" 2>&1 </dev/null &
    PROBE_PID=$!
    for _ in $(seq 1 "$WAIT_SEC"); do
      ros2 node list 2>/dev/null | grep -qx "$CAMERA_NODE" && break
      sleep 1
    done
    if ros2 node list 2>/dev/null | grep -qx "$CAMERA_NODE"; then
      NODE_RUNNING=1
      ok "temporary camera is up"
    else
      bad "temporary camera did not come up within ${WAIT_SEC}s"
      say "  --- last log lines ---"
      tail -n 15 "$PROBE_LOG" 2>/dev/null | sed 's/^/  /'
    fi
  else
    say "  use --probe to launch a temporary camera and check profiles"
  fi
fi

# ---------------------------------------------------------------------------
# Supported / active profiles
# ---------------------------------------------------------------------------
get_param() {
  ros2 param get "$CAMERA_NODE" "$1" 2>/dev/null \
    | sed -n 's/.*String value is: //p' | head -n1
}

list_profiles() {
  ros2 param describe "$CAMERA_NODE" "$1" 2>/dev/null \
    | grep -oE '[0-9]+x[0-9]+x[0-9]+' | sort -u
}

if [ "$NODE_RUNNING" = "1" ]; then
  say ""
  say "== supported profiles =="
  COLOR_LIST="$(list_profiles rgb_camera.color_profile)"
  DEPTH_LIST="$(list_profiles depth_module.depth_profile)"

  if [ -n "$COLOR_LIST" ]; then
    if printf '%s\n' "$COLOR_LIST" | grep -qx "$COLOR_PROFILE"; then
      ok "color $COLOR_PROFILE supported"
    else
      bad "color $COLOR_PROFILE NOT supported"
    fi
    say "  color options: $(printf '%s' "$COLOR_LIST" | tr '\n' ' ')"
  else
    warn "could not read color profile list"
  fi

  if [ -n "$DEPTH_LIST" ]; then
    if printf '%s\n' "$DEPTH_LIST" | grep -qx "$DEPTH_PROFILE"; then
      ok "depth $DEPTH_PROFILE supported"
    else
      bad "depth $DEPTH_PROFILE NOT supported"
    fi
    say "  depth options: $(printf '%s' "$DEPTH_LIST" | tr '\n' ' ')"
  else
    warn "could not read depth profile list"
  fi

  say ""
  say "== active profiles =="
  AC_COLOR="$(get_param rgb_camera.color_profile)"
  AC_DEPTH="$(get_param depth_module.depth_profile)"
  say "  color = ${AC_COLOR:-<unknown>}"
  say "  depth = ${AC_DEPTH:-<unknown>}"
  [ "$AC_COLOR" = "$COLOR_PROFILE" ] && ok "active color matches" || bad "active color differs"
  [ "$AC_DEPTH" = "$DEPTH_PROFILE" ] && ok "active depth matches" || bad "active depth differs"

  say ""
  say "== actual open profile (driver log) =="
  NEWEST_LOG="$(ls -t /root/.ros/log/realsense2_camera_node_*.log 2>/dev/null | head -n1 || true)"
  if [ -n "$NEWEST_LOG" ]; then
    grep "Open profile" "$NEWEST_LOG" | tail -n 4 | sed 's/^/  /'
  else
    warn "no realsense driver log found in /root/.ros/log"
  fi

  if [ "$DO_RATES" = "1" ]; then
    say ""
    say "== topic rates =="
    rate_of() {
      timeout -k 2 "$HZ_TIMEOUT" ros2 topic hz "$1" 2>/dev/null \
        | awk '/average rate/ {print $3; exit}'
    }
    for t in /camera/camera/color/image_raw \
             /camera/camera/depth/image_rect_raw \
             /camera/camera/aligned_depth_to_color/image_raw; do
      r="$(rate_of "$t")"
      if [ -n "$r" ]; then
        say "  $t : ${r} Hz"
      else
        warn "$t : no data"
      fi
    done
  fi
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
say ""
if [ "$FAIL" = "0" ] && [ "$NODE_RUNNING" = "1" ]; then
  say "RESULT: PASS  (color=$COLOR_PROFILE depth=$DEPTH_PROFILE)"
  exit 0
else
  say "RESULT: FAIL"
  exit 1
fi
