#!/usr/bin/env bash
# Stop every process started by start_rtabmap_laser.sh.
#
# Why a separate script:
#   start_rtabmap_laser.sh starts the S2 driver, the static TF publisher(s)
#   and (optionally) the bag recorder with `setsid`, so they live in their own
#   session and never receive the terminal's Ctrl+C.  If the mapping script
#   itself dies (Ctrl+C, `kill -TERM %1` on a stopped job, SIGKILL, closed
#   terminal, crash) those children stay behind as orphans: the S2 keeps
#   /dev/ttyUSB0 busy and the next run fails with "device busy".
#
# Usage (on board):
#   ./stop_rtabmap_laser.sh                    # SIGTERM everything, SIGKILL what survives
#   KEEP_DRIVER=1 ./stop_rtabmap_laser.sh      # keep rplidar_node (owned by someone else)
#   DRY_RUN=1 ./stop_rtabmap_laser.sh          # only list what would be stopped
#
# Exit code: 0 when nothing of this flow is left running, 1 otherwise.
set -eo pipefail   # NOTE: no -u (ROS setup files need unset vars)

KEEP_DRIVER="${KEEP_DRIVER:-0}"
DRY_RUN="${DRY_RUN:-0}"
WAIT_SECS="${WAIT_SECS:-10}"

# Command line patterns of this flow (matched with pgrep/pkill -f).
# `ros2 run` and `ros2 launch` fork the real node, so both the wrapper and
# the node executable have to be listed.
PATTERNS=(
  "ros2 launch rtabmap_launch rtabmap.launch.py"   # rtabmap launch wrapper
  "rtabmap_slam/rtabmap"                           # rtabmap node
  "rtabmap_odom/icp_odometry"                      # icp odometry node
  "ros2 bag record"                                # optional bag recorder
  "static_transform_publisher"                     # base_footprint -> base_link -> laser
  "ros2 launch rplidar_ros rplidar_s2_launch.py"   # S2 driver wrapper
  "rplidar_ros/rplidar_node"                       # S2 driver node
)

# PIDs of this flow (deduped, order preserved).
# Ancestors of this script are excluded: a shell whose command line merely
# mentions "rtabmap" (e.g. `bash -c '... rtabmap ...'`) must not be killed.
match_pids() {
  local pat pid out="" skip
  skip=" $(ancestor_pids) "
  for pat in "${PATTERNS[@]}"; do
    if [ "$KEEP_DRIVER" = "1" ]; then
      case "$pat" in *rplidar*) continue ;; esac
    fi
    for pid in $(pgrep -f "$pat" 2>/dev/null || true); do
      case "$skip" in *" $pid "*) continue ;; esac
      case " $out " in *" $pid "*) ;; *) out="$out $pid" ;; esac
    done
  done
  echo "${out# }"
}

# This script and its parents (the shell it was started from, ...).
ancestor_pids() {
  local pid out=""
  pid=$$
  while [ -n "$pid" ] && [ "$pid" != "0" ] && [ "$pid" != "1" ]; do
    out="$out $pid"
    pid="$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')" || break
  done
  echo "${out# }"
}

# Still existing subset of a PID list.
alive_pids() {
  local pid out=""
  for pid in $1; do
    [ -d "/proc/$pid" ] && out="$out $pid"
  done
  echo "${out# }"
}

# Subset of a PID list that is currently stopped (T state).
stopped_pids() {
  local pid st out=""
  for pid in $1; do
    st="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d ' ')" || continue
    case "$st" in
      T*|t*) out="$out $pid" ;;
    esac
  done
  echo "${out# }"
}

echo "== laser-mapping processes =="
PIDS="$(match_pids)"
if [ -z "$PIDS" ]; then
  echo "  (nothing running)"
  exit 0
fi
for pid in $PIDS; do
  printf '  %-8s %s\n' "$pid" \
    "$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | cut -c1-110)"
done

if [ "$DRY_RUN" = "1" ]; then
  echo "DRY_RUN: nothing was killed"
  exit 0
fi

echo "-- sending SIGTERM"
for pid in $PIDS; do kill -TERM "$pid" 2>/dev/null || true; done

end=$((SECONDS + WAIT_SECS))
while [ "$SECONDS" -lt "$end" ]; do
  PIDS="$(alive_pids "$PIDS")"
  [ -z "$PIDS" ] && break
  sleep 0.5
done

PIDS="$(alive_pids "$PIDS")"
if [ -n "$PIDS" ]; then
  STUCK="$(stopped_pids "$PIDS")"
  if [ -n "$STUCK" ]; then
    # A stopped process does not handle SIGTERM until it is resumed.
    echo "-- $STUCK is stopped: SIGCONT so the pending SIGTERM is delivered"
    for pid in $STUCK; do kill -CONT "$pid" 2>/dev/null || true; done
    sleep 2
  fi
  PIDS="$(alive_pids "$PIDS")"
fi

if [ -n "$PIDS" ]; then
  echo "-- sending SIGKILL to $PIDS"
  for pid in $PIDS; do kill -KILL "$pid" 2>/dev/null || true; done
  sleep 1
  PIDS="$(alive_pids "$PIDS")"
fi

if [ -n "$PIDS" ]; then
  echo "STILL RUNNING: $PIDS"
  exit 1
fi

echo "== all stopped =="
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  echo "-- remaining ROS nodes:"
  timeout 10 ros2 node list 2>/dev/null | sed 's/^/   /' || echo "   (ros2 node list failed)"
fi
exit 0
