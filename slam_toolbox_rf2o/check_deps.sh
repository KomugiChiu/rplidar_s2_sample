#!/usr/bin/env bash
# Dependency check (+auto-install) for the slam_toolbox_rf2o stack.
# Run ON BOARD (the script checks this robot's ROS workspaces and devices).
#
# Covers:
#   rplidar S2 + rf2o + slam_toolbox
#   D455 + robot_localization EKF
#   Nav2 + nav2_deadband_controller plugin
#   AI detection + person_target_recovery + demo_manager
#
# Usage:
#   ./check_deps.sh                 # check, install missing apt packages
#   ./check_deps.sh --check-only    # report only, install nothing
#   ./check_deps.sh --help
#
# Notes:
#   - Only apt packages are installed automatically.  ROS workspaces are
#     checked and reported; they are never rebuilt by this script.
#   - Hardware that is unplugged is reported as WARN (not a hard failure).
#
# Env overrides:
#   ROS_DISTRO, RPLIDAR_WS, RF2O_ROOT, PLUGIN_WS, ROBOT_WS, LIBRARY_WS,
#   REALSENSE_WS, AI_WS, DEMO_WS, DEMO_INSTALL, RECOVERY_INSTALL, KIT_DIR
set -eo pipefail   # NOTE: no -u (ROS setup files need unset vars)

CHECK_ONLY=0
case "${1:-}" in
  --check-only) CHECK_ONLY=1 ;;
  --help|-h)
    sed -n '2,23p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
    ;;
  "") ;;
  *)
    echo "unknown option: $1 (use --check-only or --help)" >&2
    exit 2
    ;;
esac

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

RPLIDAR_WS="${RPLIDAR_WS:-/nvt_ros2_example/rplidar_ros}"
RF2O_ROOT="${RF2O_ROOT:-/nvt_ros2_example/rf2o}"
PLUGIN_WS="${PLUGIN_WS:-/root/nav2_deadband_ws}"
ROBOT_WS="${ROBOT_WS:-/root/raspybot/yahboomcar_ws}"
LIBRARY_WS="${LIBRARY_WS:-/root/raspybot/library_ws/py_install}"
REALSENSE_WS="${REALSENSE_WS:-/nvt_ros2_example/realsense-ros}"
AI_WS="${AI_WS:-/nvt_ros2_example/ai_human_detection}"
DEMO_WS="${DEMO_WS:-/root/ros2_ws}"
DEMO_INSTALL="${DEMO_INSTALL:-/nvt_ros2_example/demo_manager/install}"
RECOVERY_INSTALL="${RECOVERY_INSTALL:-/nvt_ros2_example/person_target_recovery/install}"
KIT_DIR="${KIT_DIR:-/nvt_ros2_example/demo_manager/fixed_build}"

# Runtime packages from apt.  Workspaces are checked separately below.
APT_PKGS=(
  "ros-${ROS_DISTRO}-rclpy"
  "ros-${ROS_DISTRO}-tf2-ros"
  "ros-${ROS_DISTRO}-tf2-geometry-msgs"
  "ros-${ROS_DISTRO}-slam-toolbox"
  "ros-${ROS_DISTRO}-robot-localization"
  "ros-${ROS_DISTRO}-nav2-bringup"
  "ros-${ROS_DISTRO}-nav2-controller"
  "ros-${ROS_DISTRO}-nav2-mppi-controller"
  "ros-${ROS_DISTRO}-nav2-costmap-2d"
  "ros-${ROS_DISTRO}-nav2-lifecycle-manager"
  "ros-${ROS_DISTRO}-nav2-velocity-smoother"
  "ros-${ROS_DISTRO}-nav2-collision-monitor"
  "ros-${ROS_DISTRO}-nav2-bt-navigator"
  "ros-${ROS_DISTRO}-nav2-behavior-tree"
  "ros-${ROS_DISTRO}-nav2-map-server"
  "ros-${ROS_DISTRO}-nav2-waypoint-follower"
  "ros-${ROS_DISTRO}-nav2-core"
  "ros-${ROS_DISTRO}-nav2-util"
  "ros-${ROS_DISTRO}-pluginlib"
  "ros-${ROS_DISTRO}-realsense2-camera"
  "ros-${ROS_DISTRO}-compressed-image-transport"
  "ros-${ROS_DISTRO}-image-transport"
  "ros-${ROS_DISTRO}-rviz2"
)

FAIL=0
WARN=0
say()  { printf '%-52s %s\n' "$1" "$2"; }
ok()   { say "$1" "OK"; }
bad()  { say "$1" "MISSING"; FAIL=1; }
warn() { say "$1" "$2"; WARN=1; }

source_if_exists() {
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

pkg_ok() {
  ros2 pkg prefix "$1" >/dev/null 2>&1
}

exe_ok() {
  ros2 pkg executables "$1" 2>/dev/null | grep -qx "$1 $2"
}

echo "== OS / ROS =="
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  ok "/opt/ros/${ROS_DISTRO}"
else
  bad "/opt/ros/${ROS_DISTRO}/setup.bash (wrong ROS_DISTRO?)"
fi

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash" 2>/dev/null || true
if command -v ros2 >/dev/null 2>&1; then
  ok "ros2 CLI"
else
  bad "ros2 CLI (source /opt/ros/${ROS_DISTRO}/setup.bash)"
fi

echo "== apt packages =="
APT_MISSING=()
for p in "${APT_PKGS[@]}"; do
  if dpkg -s "$p" >/dev/null 2>&1; then
    ok "$p"
  else
    say "$p" "MISSING"
    APT_MISSING+=("$p")
  fi
done

echo "== python modules =="
if python3 -c 'import rclpy' >/dev/null 2>&1; then
  ok "rclpy"
else
  bad "rclpy"
fi
if python3 -c 'import tf2_ros' >/dev/null 2>&1; then
  ok "tf2_ros"
else
  bad "tf2_ros"
fi

echo "== workspaces / files =="
# rplidar
if [ -f "$RPLIDAR_WS/local_setup.bash" ] &&
   [ -x "$RPLIDAR_WS/rplidar_ros/lib/rplidar_ros/rplidar_node" ]; then
  ok "$RPLIDAR_WS"
else
  bad "$RPLIDAR_WS (rplidar_ros workspace / rplidar_node)"
fi

# rf2o
if [ -f "$RF2O_ROOT/local_setup.bash" ] &&
   [ -x "$RF2O_ROOT/rf2o_laser_odometry/lib/rf2o_laser_odometry/rf2o_laser_odometry_node" ]; then
  ok "$RF2O_ROOT"
else
  bad "$RF2O_ROOT (rf2o_laser_odometry install)"
fi

# nav2_deadband_controller
if [ -f "$PLUGIN_WS/install/local_setup.bash" ] &&
   [ -f "$PLUGIN_WS/install/nav2_deadband_controller/lib/libnav2_deadband_controller.so" ]; then
  ok "$PLUGIN_WS"
else
  bad "$PLUGIN_WS (nav2_deadband_controller plugin)"
fi

# chassis driver workspace
if [ -f "$ROBOT_WS/install/local_setup.bash" ]; then
  ok "$ROBOT_WS"
else
  bad "$ROBOT_WS (yahboomcar_bringup)"
fi

# vendor python path used by the chassis driver
if [ -d "$LIBRARY_WS" ]; then
  ok "$LIBRARY_WS"
else
  warn "$LIBRARY_WS" "missing -> PYTHONPATH may be incomplete"
fi

# RealSense workspace
if [ -f "$REALSENSE_WS/local_setup.bash" ] || [ -f "$REALSENSE_WS/setup.bash" ]; then
  ok "$REALSENSE_WS"
else
  bad "$REALSENSE_WS (realsense2_camera workspace)"
fi

# AI workspace
if [ -f "$AI_WS/local_setup.bash" ] || [ -f "$AI_WS/setup.bash" ]; then
  ok "$AI_WS"
else
  bad "$AI_WS (ai_human_detection workspace)"
fi

# demo SDK workspace
if [ -f "$DEMO_WS/install/local_setup.bash" ]; then
  ok "$DEMO_WS/install"
else
  bad "$DEMO_WS/install (demo kit)"
fi

# person follow installs
if [ -x "$DEMO_INSTALL/start_demo_only.sh" ]; then
  ok "$DEMO_INSTALL"
else
  bad "$DEMO_INSTALL/start_demo_only.sh (demo_manager v1.0.4)"
fi
if [ -x "$RECOVERY_INSTALL/start_person_target_recovery.sh" ]; then
  ok "$RECOVERY_INSTALL"
else
  bad "$RECOVERY_INSTALL/start_person_target_recovery.sh (recovery v1.0.4)"
fi
if [ -d "$KIT_DIR/tools" ]; then
  ok "$KIT_DIR/tools"
else
  bad "$KIT_DIR/tools (AI TF helper)"
fi

echo "== ROS packages after sourcing workspaces =="
source_if_exists "$RPLIDAR_WS" || true
source_if_exists "$RF2O_ROOT" || true
source_if_exists "$PLUGIN_WS/install" || true
source_if_exists "$ROBOT_WS/install" || true
source_if_exists "$REALSENSE_WS" || true
source_if_exists "$DEMO_WS/install" || true
# AI must come after the kit workspace so the vendor package wins.
source_if_exists "$AI_WS" || true
source_if_exists "$RECOVERY_INSTALL" || true
source_if_exists "$DEMO_INSTALL" || true

check_pkg() {
  if pkg_ok "$1"; then
    ok "$1"
  else
    bad "$1 (ros2 pkg prefix)"
  fi
}

check_pkg rplidar_ros
check_pkg rf2o_laser_odometry
check_pkg slam_toolbox
check_pkg robot_localization
check_pkg realsense2_camera
check_pkg ai_human_detection
check_pkg demo_manager
check_pkg person_target_recovery
check_pkg nav2_deadband_controller
check_pkg nav2_bringup
check_pkg nav2_mppi_controller
check_pkg yahboomcar_bringup
check_pkg tf2_ros
check_pkg rviz2

if exe_ok ai_human_detection ai_human_detection_node; then
  ok "ai_human_detection_node"
else
  bad "ai_human_detection_node (workspace shadowing?)"
fi
if exe_ok yahboomcar_bringup Mcnamu_driver; then
  ok "Mcnamu_driver"
else
  bad "Mcnamu_driver (yahboomcar_bringup)"
fi

echo "== hardware =="
S2_TTY=""
for dev in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$dev" ] || continue
  if command -v udevadm >/dev/null 2>&1; then
    if udevadm info -a -n "$dev" 2>/dev/null | grep -q 'ATTRS{idVendor}=="10c4"'; then
      S2_TTY="$dev"
    fi
  elif lsusb 2>/dev/null | grep -q '10c4:ea60'; then
    S2_TTY="$dev"
  fi
done
if [ -n "$S2_TTY" ]; then
  ok "RPLIDAR S2 ($S2_TTY)"
else
  warn "RPLIDAR S2" "not found (USB 10c4:ea60 unplugged?)"
fi

if command -v lsusb >/dev/null 2>&1 && lsusb 2>/dev/null | grep -qiE '8086:0b5c|Intel.*RealSense'; then
  ok "RealSense D455 (USB)"
else
  warn "RealSense D455" "not found (USB unplugged?)"
fi

if [ -e /dev/nvt_gfx0 ]; then
  ok "/dev/nvt_gfx0"
else
  warn "/dev/nvt_gfx0" "missing -> run: modprobe nvt_gfx"
fi

if id -nG 2>/dev/null | grep -qw dialout; then
  ok "dialout group"
else
  warn "dialout group" "missing -> sudo usermod -aG dialout $USER (re-login)"
fi

echo
if [ "$CHECK_ONLY" = "1" ]; then
  if [ "${#APT_MISSING[@]}" -eq 0 ] && [ "$FAIL" -eq 0 ]; then
    echo "check-only: OK (warnings: $WARN)"
    exit 0
  fi
  echo "check-only: apt missing=${#APT_MISSING[@]}, failures present, warnings=$WARN"
  exit 1
fi

# --- install missing apt packages ------------------------------------------
if [ "${#APT_MISSING[@]}" -gt 0 ]; then
  if [ "$(id -u)" -eq 0 ]; then
    APT=(apt-get)
  else
    command -v sudo >/dev/null 2>&1 || {
      echo "ERROR: need root or sudo to install apt packages" >&2
      exit 1
    }
    sudo -v
    APT=(sudo apt-get)
  fi
  echo "installing: ${APT_MISSING[*]}"
  "${APT[@]}" install -y "${APT_MISSING[@]}"
  for p in "${APT_MISSING[@]}"; do
    if dpkg -s "$p" >/dev/null 2>&1; then
      ok "$p"
    else
      say "$p" "INSTALL FAILED"
      FAIL=1
    fi
  done
else
  echo "apt: nothing to install"
fi

echo
if [ "$FAIL" -eq 0 ]; then
  echo "deps OK (warnings: $WARN)"
  exit 0
fi

cat <<'EOF'

Some dependencies are still missing.  ROS workspaces are not built by this
script; build them with colcon, for example:

  # rf2o install is deployed at /nvt_ros2_example/rf2o
  cd /root/nav2_deadband_ws              && colcon build --symlink-install
  cd /root/raspybot/yahboomcar_ws        && colcon build --symlink-install
  cd /nvt_ros2_example/realsense-ros     && colcon build --symlink-install
  cd /nvt_ros2_example/ai_human_detection && colcon build --symlink-install

Then re-run ./check_deps.sh --check-only
EOF
exit 1
