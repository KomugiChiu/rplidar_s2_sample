#!/usr/bin/env bash
# Stop everything started by start_nav2_rf2o.sh.
#
# Same mechanism as the other stop scripts in this repo:
#   1. PID file (precise): "pid name mode", setsid'ed processes are session
#      leaders -> the whole process group is killed
#   2. command line patterns (fallback): catches orphans; the chassis driver
#      is only matched when KEEP_DRIVER is not set
#
# Usage (on board):
#   ./stop_nav2_rf2o.sh
#   DRY_RUN=1 ./stop_nav2_rf2o.sh            # only list what would be stopped
#   KEEP_DRIVER=1 ./stop_nav2_rf2o.sh        # keep Mcnamu_driver + the gate
set -eo pipefail   # NOTE: no -u (ROS setup files need unset vars)

KEEP_DRIVER="${KEEP_DRIVER:-0}"
DRY_RUN="${DRY_RUN:-0}"
WAIT_SECS="${WAIT_SECS:-10}"
PID_FILE="${PID_FILE:-/tmp/slam_toolbox_rf2o_nav2/pids}"

# nav2 nodes we may have started (keep the patterns specific: a bare "nav2"
# would match half of /opt/ros).
PATTERNS=(
  "ros2 launch nav2_bringup bringup_launch.py"
  "nav2_controller/controller_server"
  "nav2_smoother/smoother_server"
  "nav2_planner/planner_server"
  "nav2_behaviors/behavior_server"
  "nav2_bt_navigator/bt_navigator"
  "nav2_waypoint_follower/waypoint_follower"
  "nav2_velocity_smoother/velocity_smoother"
  "nav2_route/route_server"
  "nav2_docking/docking_server"
  "nav2_collision_monitor/collision_monitor"
  "nav2_lifecycle_manager/lifecycle_manager"
)
DRIVER_PATTERNS=(
  "Mcnamu_driver"
  "translation_only.py"
)

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

ancestor_pids() {
  local pid out=""
  pid=$$
  while [ -n "$pid" ] && [ "$pid" != "0" ] && [ "$pid" != "1" ]; do
    out="$out $pid"
    pid="$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')" || break
  done
  echo "${out# }"
}

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

echo "== nav2 (slam_toolbox + rf2o) processes =="

HAVE_PID_FILE=0
if [ -s "$PID_FILE" ]; then
  HAVE_PID_FILE=1
  while read -r pid name mode; do
    [ -n "${pid:-}" ] || continue
    [ -d "/proc/$pid" ] || continue
    add_target "$pid" "${mode:-proc}" "${name:-pid$pid}"
  done < "$PID_FILE"
fi

SWEEP_PATTERNS=("${PATTERNS[@]}")
if [ "$KEEP_DRIVER" != "1" ]; then
  SWEEP_PATTERNS+=("${DRIVER_PATTERNS[@]}")
fi
# With a PID file the driver is only killed if it is in it; the pattern sweep
# is for orphans, so it stays driver-free in that case.
if [ "$HAVE_PID_FILE" = "1" ] && [ "$KEEP_DRIVER" != "1" ]; then
  SWEEP_PATTERNS=("${PATTERNS[@]}")
fi

for pid in $(match_pids "${SWEEP_PATTERNS[@]}"); do
  known=0
  for known_pid in "${TARGET_PIDS[@]:-}"; do
    [ "$known_pid" = "$pid" ] && known=1
  done
  [ "$known" = "1" ] && continue
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
