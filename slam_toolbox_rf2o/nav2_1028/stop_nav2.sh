#!/usr/bin/env bash
set -u

RUN_DIR="${RUN_DIR:-/tmp/raspybot_nav2}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"
KEEP_DRIVER="${KEEP_DRIVER:-0}"

if [ -s "$PID_FILE" ]; then
    tac "$PID_FILE" | while read -r pid name; do
        [ -n "${pid:-}" ] || continue
        if [ "$KEEP_DRIVER" = "1" ] && [ "${name:-}" = "yahboomcar" ]; then
            continue
        fi
        if kill -0 "$pid" 2>/dev/null; then
            echo "Stopping $name (PID $pid)..."
            kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
        fi
    done

    sleep 3

    tac "$PID_FILE" | while read -r pid name; do
        [ -n "${pid:-}" ] || continue
        if [ "$KEEP_DRIVER" = "1" ] && [ "${name:-}" = "yahboomcar" ]; then
            continue
        fi
        if kill -0 "$pid" 2>/dev/null; then
            echo "Force stopping $name (PID $pid)..."
            kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
        fi
    done

    rm -f "$PID_FILE"
else
    echo "No PID file at $PID_FILE; checking for stray Nav2 processes..."
fi

# Fallback sweep in case any child processes survived
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
if [ "$KEEP_DRIVER" != "1" ]; then
    PATTERNS+=(
        "ros2 launch yahboomcar_bringup bringup.launch.py"
        "Mcnamu_driver"
        "speed_floor.py"
        "directional_stop.py"
    )
fi

for pat in "${PATTERNS[@]}"; do
    pkill -TERM -f "$pat" 2>/dev/null || true
done
sleep 1
for pat in "${PATTERNS[@]}"; do
    pkill -KILL -f "$pat" 2>/dev/null || true
done

echo "Nav2 stack stopped."
