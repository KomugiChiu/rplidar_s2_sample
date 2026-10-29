#!/usr/bin/env bash
# Stop everything started by start_all.sh (and clean up leftovers).
#
#   ./stop_all.sh
#   KEEP_DRIVER=1 ./stop_all.sh   # keep the chassis driver + speed_floor
set -u

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]:-$0}" 2>/dev/null || echo "${BASH_SOURCE[0]:-$0}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"

echo "== stop_all =="

# --- Nav2 + speed_floor + chassis driver ------------------------------------
if [ -x "$SCRIPT_DIR/nav2_1028/stop_nav2.sh" ]; then
    "$SCRIPT_DIR/nav2_1028/stop_nav2.sh" || true
fi

# --- EKF + RealSense ---------------------------------------------------------
if [ -x "$SCRIPT_DIR/ekf/stop_ekf.sh" ]; then
    "$SCRIPT_DIR/ekf/stop_ekf.sh" || true
fi

# --- slam_toolbox + rf2o + lidar --------------------------------------------
if [ -x "$SCRIPT_DIR/stop_slam_toolbox_rf2o.sh" ]; then
    "$SCRIPT_DIR/stop_slam_toolbox_rf2o.sh" || true
fi

# --- cmd_vel logger ----------------------------------------------------------
pkill -f '[r]ecord_cmd_vel.py' 2>/dev/null || true

# --- leftovers (old/partial sessions, no PID file) ---------------------------
for pat in '[r]plidar_node' '[s]tatic_transform_publisher' \
           '[r]f2o_laser_odometry_node' '[a]sync_slam_toolbox_node' \
           '[s]peed_floor.py' '[t]ranslation_only.py'; do
    pkill -TERM -f "$pat" 2>/dev/null || true
done
sleep 2
for pat in '[r]plidar_node' '[s]tatic_transform_publisher' \
           '[r]f2o_laser_odometry_node' '[a]sync_slam_toolbox_node' \
           '[s]peed_floor.py' '[t]ranslation_only.py'; do
    pkill -KILL -f "$pat" 2>/dev/null || true
done

echo "== all stopped =="
