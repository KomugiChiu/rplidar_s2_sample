#!/usr/bin/env bash
# Dependency check (+auto-install) for the standalone slam_toolbox + rf2o setup.
# Run ON BOARD.
#
# Checks:
#   apt: slam_toolbox, nav2_map_server, tf2_ros
#   workspaces: rplidar_ros, rf2o/install
#   yaml: slam_toolbox_rf2o.yaml, slam_toolbox_localization.yaml, rf2o_params.yaml
#   scripts: start_mapping.sh, start_localization.sh, stop.sh, save_map.sh,
#            ../rf2o/start.sh, ../rf2o/stop.sh
#   localization posegraph
#   RPLIDAR S2 USB / dialout
#
# Usage:
#   ./check_deps.sh --check-only
#   ./check_deps.sh
set -eo pipefail

CHECK_ONLY=0
case "${1:-}" in
  --check-only) CHECK_ONLY=1 ;;
  --help|-h)
    sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
    ;;
  "") ;;
  *) echo "unknown option: $1" >&2; exit 2 ;;
esac

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
RPLIDAR_WS="${RPLIDAR_WS:-$SCRIPT_DIR/../rplidar_ros}"
RF2O_INSTALL="${RF2O_INSTALL:-$SCRIPT_DIR/../rf2o/install}"
MAPPING_YAML="${MAPPING_YAML:-$SCRIPT_DIR/slam_toolbox_rf2o.yaml}"
LOCALIZATION_YAML="${LOCALIZATION_YAML:-$SCRIPT_DIR/slam_toolbox_localization.yaml}"
RF2O_YAML="${RF2O_YAML:-$SCRIPT_DIR/../rf2o/rf2o_params.yaml}"

APT_PKGS=(
  "ros-${ROS_DISTRO}-slam-toolbox"
  "ros-${ROS_DISTRO}-nav2-map-server"
  "ros-${ROS_DISTRO}-tf2-ros"
)

FAIL=0
WARN=0
say()  { printf '%-44s %s\n' "$1" "$2"; }
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

echo "== OS / ROS =="
[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ] && ok "/opt/ros/${ROS_DISTRO}" || bad "/opt/ros/${ROS_DISTRO}"
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash" 2>/dev/null || true
command -v ros2 >/dev/null 2>&1 && ok "ros2 CLI" || bad "ros2 CLI"

echo "== apt packages =="
APT_MISSING=()
for p in "${APT_PKGS[@]}"; do
  if dpkg -s "$p" >/dev/null 2>&1; then ok "$p"; else say "$p" "MISSING"; APT_MISSING+=("$p"); fi
done

echo "== files / scripts =="
for f in "$MAPPING_YAML" "$LOCALIZATION_YAML" "$RF2O_YAML"; do
  [ -f "$f" ] && ok "${f#$SCRIPT_DIR/}" || bad "${f#$SCRIPT_DIR/}"
done
for f in start_mapping.sh start_localization.sh stop.sh save_map.sh ../rf2o/start.sh ../rf2o/stop.sh; do
  [ -x "$SCRIPT_DIR/$f" ] && ok "$f" || bad "$f (missing/not executable)"
done

echo "== workspaces / packages =="
if [ -f "$RPLIDAR_WS/local_setup.bash" ] &&
   [ -x "$RPLIDAR_WS/rplidar_ros/lib/rplidar_ros/rplidar_node" ]; then
  ok "$RPLIDAR_WS"
else
  bad "$RPLIDAR_WS (rplidar_ros / rplidar_node)"
fi
if [ -f "$RF2O_INSTALL/local_setup.bash" ] &&
   [ -x "$RF2O_INSTALL/rf2o_laser_odometry/lib/rf2o_laser_odometry/rf2o_laser_odometry_node" ]; then
  ok "$RF2O_INSTALL"
else
  bad "$RF2O_INSTALL (rf2o install)"
fi

source_if_exists "$RPLIDAR_WS" || true
source_if_exists "$RF2O_INSTALL" || true

check_pkg() { ros2 pkg prefix "$1" >/dev/null 2>&1 && ok "$1" || bad "$1"; }
check_exe() {
  ros2 pkg executables "$1" 2>/dev/null | grep -qx "$1 $2" && ok "$2" || bad "$2"
}

check_pkg slam_toolbox
check_pkg rplidar_ros
check_pkg rf2o_laser_odometry
check_pkg nav2_map_server
check_pkg tf2_ros
check_exe slam_toolbox async_slam_toolbox_node
check_exe slam_toolbox localization_slam_toolbox_node
check_exe rplidar_ros rplidar_node
check_exe rf2o_laser_odometry rf2o_laser_odometry_node
check_exe nav2_map_server map_saver_cli
check_exe tf2_ros static_transform_publisher

echo "== localization posegraph =="
map_file="$(sed -n 's/^[[:space:]]*map_file_name:[[:space:]]*"\{0,1\}\([^"]*\)"\{0,1\}[[:space:]]*$/\1/p' \
  "$LOCALIZATION_YAML" 2>/dev/null | head -n1)"
if [ -z "$map_file" ]; then
  warn "map_file_name" "not set in $(basename "$LOCALIZATION_YAML")"
elif [ ! -f "${map_file}.posegraph" ] || [ ! -f "${map_file}.data" ]; then
  warn "posegraph" "missing ${map_file}.posegraph / .data"
else
  ok "posegraph"
fi

echo "== hardware =="
S2_TTY=""
for dev in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$dev" ] || continue
  if command -v udevadm >/dev/null 2>&1; then
    if udevadm info -a -n "$dev" 2>/dev/null | grep -q 'ATTRS{idVendor}=="10c4"'; then S2_TTY="$dev"; fi
  elif lsusb 2>/dev/null | grep -q '10c4:ea60'; then
    S2_TTY="$dev"
  fi
done
[ -n "$S2_TTY" ] && ok "RPLIDAR S2 ($S2_TTY)" || warn "RPLIDAR S2" "not found (USB 10c4:ea60 unplugged?)"
id -nG 2>/dev/null | grep -qw dialout && ok "dialout group" || warn "dialout group" "missing -> usermod -aG dialout"

echo
if [ "$CHECK_ONLY" = "1" ]; then
  if [ "${#APT_MISSING[@]}" -eq 0 ] && [ "$FAIL" -eq 0 ]; then
    echo "check-only: OK (warnings: $WARN)"
    exit 0
  fi
  echo "check-only: apt missing=${#APT_MISSING[@]}, failures present, warnings=$WARN"
  exit 1
fi

if [ "${#APT_MISSING[@]}" -gt 0 ]; then
  if [ "$(id -u)" -eq 0 ]; then APT=(apt-get); else sudo -v; APT=(sudo apt-get); fi
  echo "installing: ${APT_MISSING[*]}"
  "${APT[@]}" install -y "${APT_MISSING[@]}"
  for p in "${APT_MISSING[@]}"; do
    dpkg -s "$p" >/dev/null 2>&1 && ok "$p" || { say "$p" "INSTALL FAILED"; FAIL=1; }
  done
else
  echo "apt: nothing to install"
fi

echo
if [ "$FAIL" -eq 0 ]; then
  echo "slam_toolbox deps OK (warnings: $WARN)"
  exit 0
fi
echo "slam_toolbox deps: some checks failed" >&2
exit 1
