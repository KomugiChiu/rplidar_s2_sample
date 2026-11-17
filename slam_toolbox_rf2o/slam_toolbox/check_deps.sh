#!/usr/bin/env bash
# Dependency check (+auto-install) for the slam_toolbox / rf2o / RPLIDAR flow.
# Run ON BOARD.
#
# Usage:
#   ./check_deps.sh                 # check, install missing apt packages
#   ./check_deps.sh --check-only    # report only, install nothing
#   ./check_deps.sh --help
#
# Env overrides:
#   ROS_DISTRO, RPLIDAR_WS, RF2O_WS, SLAM_PARAMS_MAPPING,
#   SLAM_PARAMS_LOCALIZATION, MAPS_DIR
set -eo pipefail   # NOTE: no -u (ROS setup files need unset vars)

CHECK_ONLY=0
case "${1:-}" in
  --check-only) CHECK_ONLY=1 ;;
  --help|-h)
    sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
    ;;
  "") ;;
  *)
    echo "unknown option: $1 (use --check-only or --help)" >&2
    exit 2
    ;;
esac

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

RPLIDAR_WS="${RPLIDAR_WS:-$ROOT_DIR/../rplidar_ros}"
RF2O_WS="${RF2O_WS:-$HOME/rf2o_ws}"
SLAM_PARAMS_MAPPING="${SLAM_PARAMS_MAPPING:-$ROOT_DIR/slam_toolbox_rf2o.yaml}"
SLAM_PARAMS_LOCALIZATION="${SLAM_PARAMS_LOCALIZATION:-$ROOT_DIR/slam_toolbox_localization.yaml}"
MAPS_DIR="${MAPS_DIR:-$ROOT_DIR/maps}"

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
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  ok "/opt/ros/${ROS_DISTRO}"
else
  bad "/opt/ros/${ROS_DISTRO}/setup.bash (wrong ROS_DISTRO?)"
fi
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash" 2>/dev/null || true
if command -v ros2 >/dev/null 2>&1; then ok "ros2 CLI"; else bad "ros2 CLI"; fi

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

echo "== files / workspaces =="
for f in "$SLAM_PARAMS_MAPPING" "$SLAM_PARAMS_LOCALIZATION" "$ROOT_DIR/rf2o_params.yaml"; do
  rel="${f#$ROOT_DIR/}"
  if [ -f "$f" ]; then ok "$rel"; else bad "$rel"; fi
done
if [ -d "$MAPS_DIR" ]; then
  ok "${MAPS_DIR#$ROOT_DIR/}"
else
  warn "${MAPS_DIR#$ROOT_DIR/}" "missing -> save_map.sh will create it"
fi

if [ -f "$RPLIDAR_WS/local_setup.bash" ] &&
   [ -x "$RPLIDAR_WS/rplidar_ros/lib/rplidar_ros/rplidar_node" ]; then
  ok "$RPLIDAR_WS"
else
  bad "$RPLIDAR_WS (rplidar_ros workspace / rplidar_node)"
fi

if [ -f "$RF2O_WS/install/local_setup.bash" ] &&
   [ -x "$RF2O_WS/install/rf2o_laser_odometry/lib/rf2o_laser_odometry/rf2o_laser_odometry_node" ]; then
  ok "$RF2O_WS"
else
  bad "$RF2O_WS (rf2o_laser_odometry build)"
fi

echo "== ROS packages / executables =="
source_if_exists "$RPLIDAR_WS" || true
source_if_exists "$RF2O_WS/install" || true

check_pkg() {
  if ros2 pkg prefix "$1" >/dev/null 2>&1; then ok "$1"; else bad "$1"; fi
}
check_exe() {
  if ros2 pkg executables "$1" 2>/dev/null | grep -qx "$1 $2"; then
    ok "$2"
  else
    bad "$2"
  fi
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
  "$SLAM_PARAMS_LOCALIZATION" 2>/dev/null | head -n1)"
if [ -z "$map_file" ]; then
  warn "map_file_name" "not set in $(basename "$SLAM_PARAMS_LOCALIZATION")"
elif [ ! -f "${map_file}.posegraph" ] || [ ! -f "${map_file}.data" ]; then
  warn "posegraph" "missing ${map_file}.posegraph / .data (mapping still works)"
else
  ok "posegraph"
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
  echo "slam_toolbox deps OK (warnings: $WARN)"
  exit 0
fi
echo "slam_toolbox deps: some checks failed" >&2
exit 1
