#!/usr/bin/env bash
# Stop every process started by start_slam_toolbox_rf2o.sh.
#
# Same mechanism as stop_rtabmap_laser.sh:
#   1. PID file (precise): every process the start script launched is recorded
#      as "pid name mode"; setsid'ed helpers are session leaders, so their
#      whole process group is killed with `kill -- -<pid>`
#   2. command line patterns (fallback): catches orphans whose PID file entry
#      is gone; the S2 driver is only matched this way when there is no PID
#      file, so a driver owned by somebody else is not killed by accident.
#
# Usage (on board):
#   ./stop_slam_toolbox_rf2o.sh                    # SIGTERM everything, SIGKILL what survives
#   DRY_RUN=1 ./stop_slam_toolbox_rf2o.sh          # only list what would be stopped
#   KEEP_DRIVER=1 ./stop_slam_toolbox_rf2o.sh      # keep rplidar_node (owned by someone else)
#   PID_FILE=/tmp/slam_toolbox_rf2o/pids ./stop_slam_toolbox_rf2o.sh
set -eo pipefail   # NOTE: no -u (ROS setup files need unset vars)

KEEP_DRIVER="${KEEP_DRIVER:-0}"
DRY_RUN="${DRY_RUN:-0}"
WAIT_SECS="${WAIT_SECS:-10}"
PID_FILE="${PID_FILE:-/tmp/slam_toolbox_rf2o/pids}"

# Command line patterns of this flow (matched with pgrep/pkill -f).
# Keep them specific: a bare "slam_toolbox" would also match the start/stop
# scripts themselves and any other command line containing the word.
PATTERNS=(
  "sync_slam_toolbox_node"                     # slam_toolbox node
  "ros2 launch slam_toolbox"                  # slam_toolbox launch wrapper
  "rf2o_laser_odometry_node"                  # rf2o odometry node
  "static_transform_publisher"                # base_footprint -> base_link -> laser
  "ros2 launch rplidar_ros rplidar_s2_launch.py"   # S2 driver wrapper
  "rplidar_ros/rplidar_node"                  # S2 driver node
)
# Used when the PID file exists: the driver is only killed when we started it.
PATTERNS_NODRIVER=(
  "sync_slam_toolbox_node"
  "ros2 launch slam_toolbox"
  "rf2o_laser_odometry_node"
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
#   mode "proc"  -> single process
signal_entry() {
  local sig="$1" pid="$2" mode="$3"
  if [ "$mode" = "group" ]; then
    kill "-$sig" -- "-$pid" 2>/dev/null || true
  else
    kill "-$sig" "$pid" 2>/dev/null || true
  fi
}

declare -a TARGET_PIDS=()
declare -a TARGET_MODES=()
declare -a TARGET_NAMES=()

add_target() {
  TARGET_PIDS+=("$1")
  TARGET_MODES+=("$2")
  TARGET_NAMES+=("$3")
}

echo "== slam_toolbox + rf2o processes =="

HAVE_PID_FILE=0
if [ -s "$PID_FILE" ]; then
  HAVE_PID_FILE=1
  while read -r pid name mode; do
    [ -n "${pid:-}" ] || continue
    [ -d "/proc/$pid" ] || continue
    add_target "$pid" "${mode:-proc}" "${name:-pid$pid}"
  done < "$PID_FILE"
fi

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
  # NOTE: `|| true` is required: with pipefail a failing ps (the process
  # disappeared between pgrep and ps) would abort the whole script.
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  if [ -n "$pgid" ] && [ "$pgid" = "$pid" ]; then
    add_target "$pid" "group" "sweep"
  else
    add_target "$pid" "proc" "sweep"
  fi
done

if [ "${#TARGET_PIDS[@]}" -eq 0 ]; then
  echo "  (nothing running)"
  [ "$HAVE_PID_FILE" = "1" ] && rm -f "$PID_FILE"
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
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    sleep 0.5
    STUCK=""
    for i in "${!TARGET_PIDS[@]}"; do
      [ -d "/proc/${TARGET_PIDS[$i]}" ] && STUCK="$STUCK ${TARGET_PIDS[$i]}"
    done
    [ -z "$STUCK" ] && break
  done
fi

if [ -n "$STUCK" ]; then
  echo "STILL RUNNING:$STUCK"
  exit 1
fi

rm -f "$PID_FILE"
echo "== all stopped =="
exit 0
