#!/usr/bin/env bash
# Stop every process started by start_rtabmap_laser.sh.
#
# Two mechanisms, combined:
#   1. PID file (primary, precise): start_rtabmap_laser.sh records every
#      process it starts (name + pid + kill mode) in $PID_FILE.  The setsid'd
#      helpers are session leaders, so their whole process group is killed
#      with `kill -- -<pid>` (same idea as the nav2 start/stop scripts).
#   2. Command line patterns (fallback, complete): catches orphans whose PID
#      file entry is gone (e.g. /tmp cleaned, older script version, driver
#      started by hand).  Only used for the rtabmap family when the PID file
#      exists, so a driver owned by somebody else is not killed.
#
# Usage (on board):
#   ./stop_rtabmap_laser.sh                    # SIGTERM everything, SIGKILL what survives
#   KEEP_DRIVER=1 ./stop_rtabmap_laser.sh      # keep rplidar_node (owned by someone else)
#   DRY_RUN=1 ./stop_rtabmap_laser.sh          # only list what would be stopped
#   PID_FILE=/tmp/rtabmap_laser/pids ./stop_rtabmap_laser.sh
#
# Exit code: 0 when nothing of this flow is left running, 1 otherwise.
set -eo pipefail   # NOTE: no -u (ROS setup files need unset vars)

KEEP_DRIVER="${KEEP_DRIVER:-0}"
DRY_RUN="${DRY_RUN:-0}"
WAIT_SECS="${WAIT_SECS:-10}"
PID_FILE="${PID_FILE:-/tmp/rtabmap_laser/pids}"
# extra PID files of the other flows of this project (e.g. the icp_odom flow)
EXTRA_PID_FILES="${EXTRA_PID_FILES:-/tmp/icp_odom/pids}"

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
# Patterns used when the PID file is present: the driver is only killed when
# we started it ourselves (recorded in the PID file), never by pattern.
PATTERNS_NODRIVER=(
  "ros2 launch rtabmap_launch rtabmap.launch.py"
  "rtabmap_slam/rtabmap"
  "rtabmap_odom/icp_odometry"
  "ros2 bag record"
  "static_transform_publisher"
)

# PIDs of this flow (deduped, order preserved).
match_pids() {
  local pat pid out="" skip
  skip=" $(ancestor_pids) "
  for pat in "$@"; do
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

# Send a signal to one recorded process.
#   mode "group" -> the process is a session leader: kill its process group
#   mode "proc"  -> single process (ros2 launch shuts its children down itself)
signal_entry() {
  local sig="$1" pid="$2" mode="$3"
  if [ "$mode" = "group" ]; then
    kill "-$sig" -- "-$pid" 2>/dev/null || true
  else
    kill "-$sig" "$pid" 2>/dev/null || true
  fi
}

# --- collect the targets -------------------------------------------------
declare -a TARGET_PIDS=()   # pids to signal
declare -a TARGET_MODES=()  # group|proc
declare -a TARGET_NAMES=()  # human readable

add_target() {  # pid mode name
  TARGET_PIDS+=("$1")
  TARGET_MODES+=("$2")
  TARGET_NAMES+=("$3")
}

echo "== laser-mapping processes =="

HAVE_PID_FILE=0
for pf in $PID_FILE $EXTRA_PID_FILES; do
  [ -s "$pf" ] || continue
  HAVE_PID_FILE=1
  while read -r pid name mode; do
    [ -n "${pid:-}" ] || continue
    [ -d "/proc/$pid" ] || continue
    add_target "$pid" "${mode:-proc}" "${name:-pid$pid}"
  done < "$pf"
done

# Pattern sweep: the rtabmap family always, the driver only when there is no
# PID file (then we cannot know whether we started it).
SWEEP_PATTERNS=("${PATTERNS_NODRIVER[@]}")
if [ "$HAVE_PID_FILE" = "0" ]; then
  SWEEP_PATTERNS=("${PATTERNS[@]}")
fi
if [ "$KEEP_DRIVER" = "1" ]; then
  SWEEP_PATTERNS=("${PATTERNS_NODRIVER[@]}")
fi

for pid in $(match_pids "${SWEEP_PATTERNS[@]}"); do
  known=0
  for known_pid in "${TARGET_PIDS[@]:-}"; do
    [ "$known_pid" = "$pid" ] && known=1
  done
  [ "$known" = "1" ] && continue
  # Only a session leader may be killed as a whole group (pgid == pid).
  # The rtabmap/icp_odometry nodes are children of the launch and share its
  # process group, so they have to be signalled individually.
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
  if [ -n "$pgid" ] && [ "$pgid" = "$pid" ]; then
    add_target "$pid" "group" "sweep"
  else
    add_target "$pid" "proc" "sweep"
  fi
done

if [ "${#TARGET_PIDS[@]}" -eq 0 ]; then
  echo "  (nothing running)"
  for pf in $PID_FILE $EXTRA_PID_FILES; do rm -f "$pf"; done
  exit 0
fi

for i in "${!TARGET_PIDS[@]}"; do
  printf '  %-8s %-6s %s\n' "${TARGET_PIDS[$i]}" "${TARGET_MODES[$i]}" \
    "$(tr '\0' ' ' < "/proc/${TARGET_PIDS[$i]}/cmdline" 2>/dev/null | cut -c1-100)"
done

if [ "$DRY_RUN" = "1" ]; then
  echo "DRY_RUN: nothing was killed"
  exit 0
fi

# --- stop ---------------------------------------------------------------
echo "-- sending SIGTERM"
for i in "${!TARGET_PIDS[@]}"; do
  signal_entry TERM "${TARGET_PIDS[$i]}" "${TARGET_MODES[$i]}"
done

end=$((SECONDS + WAIT_SECS))
while [ "$SECONDS" -lt "$end" ]; do
  remaining=""
  for pid in "${TARGET_PIDS[@]}"; do
    [ -d "/proc/$pid" ] && remaining="$remaining $pid"
  done
  [ -z "$remaining" ] && break
  sleep 0.5
done

# still alive?
STUCK=""
for i in "${!TARGET_PIDS[@]}"; do
  [ -d "/proc/${TARGET_PIDS[$i]}" ] && STUCK="$STUCK ${TARGET_PIDS[$i]}"
done
if [ -n "$STUCK" ]; then
  stopped="$(stopped_pids "$STUCK")"
  if [ -n "$stopped" ]; then
    # A stopped process does not handle SIGTERM until it is resumed.
    echo "-- $stopped is stopped: SIGCONT so the pending SIGTERM is delivered"
    for pid in $stopped; do kill -CONT "$pid" 2>/dev/null || true; done
    sleep 2
  fi
fi

STUCK=""
for i in "${!TARGET_PIDS[@]}"; do
  [ -d "/proc/${TARGET_PIDS[$i]}" ] && STUCK="$STUCK ${TARGET_PIDS[$i]}"
done
if [ -n "$STUCK" ]; then
  echo "-- sending SIGKILL to$STUCK"
  for i in "${!TARGET_PIDS[@]}"; do
    signal_entry KILL "${TARGET_PIDS[$i]}" "${TARGET_MODES[$i]}"
  done
  # give the kernel a moment to reap them
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    sleep 0.5
    STUCK=""
    for i in "${!TARGET_PIDS[@]}"; do
      [ -d "/proc/${TARGET_PIDS[$i]}" ] && STUCK="$STUCK ${TARGET_PIDS[$i]}"
    done
    [ -z "$STUCK" ] && break
  done
fi

STUCK=""
for i in "${!TARGET_PIDS[@]}"; do
  [ -d "/proc/${TARGET_PIDS[$i]}" ] && STUCK="$STUCK ${TARGET_PIDS[$i]}"
done
if [ -n "$STUCK" ]; then
  echo "STILL RUNNING:$STUCK"
  exit 1
fi

for pf in $PID_FILE $EXTRA_PID_FILES; do rm -f "$pf"; done
echo "== all stopped =="
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  echo "-- remaining ROS nodes:"
  timeout 10 ros2 node list 2>/dev/null | sed 's/^/   /' || echo "   (ros2 node list failed)"
fi
exit 0
