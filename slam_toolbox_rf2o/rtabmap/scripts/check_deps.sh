#!/usr/bin/env bash
# Dependency check for the RTAB-Map lidar mapping flow.
# Run ON BOARD.
#
# Usage:
#   ./check_deps.sh --check-only
#   ./check_deps.sh
set -eo pipefail

CHECK_ONLY=0
case "${1:-}" in
  --check-only) CHECK_ONLY=1 ;;
  --help|-h) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
  "") ;;
  *) echo "unknown option: $1" >&2; exit 2 ;;
esac

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
RPLIDAR_WS="${RPLIDAR_WS:-$SCRIPT_DIR/../rplidar_ros}"

APT_PKGS=(
  "ros-${ROS_DISTRO}-rtabmap-launch"
  "ros-${ROS_DISTRO}-rtabmap-slam"
  "ros-${ROS_DISTRO}-rtabmap-util"
  "ros-${ROS_DISTRO}-tf2-ros"
)

FAIL=0
WARN=0
say()  { printf '%-44s %s\n' "$1" "$2"; }
ok()   { say "$1" "OK"; }
bad()  { say "$1" "MISSING"; FAIL=1; }
warn() { say "$1" "$2"; WARN=1; }

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
for f in rtabmap_laser_mapping.ini nav2_params_rtabmap.yaml; do
  [ -f "$SCRIPT_DIR/$f" ] && ok "$f" || bad "$f"
done
for f in start_rtabmap.sh stop_rtabmap.sh check_deps.sh; do
  [ -x "$SCRIPT_DIR/$f" ] && ok "$f" || bad "$f (missing/not executable)"
done

echo "== rplidar =="
if [ -f "$RPLIDAR_WS/local_setup.bash" ] &&
   [ -x "$RPLIDAR_WS/rplidar_ros/lib/rplidar_ros/rplidar_node" ]; then
  ok "$RPLIDAR_WS"
else
  bad "$RPLIDAR_WS (rplidar_ros / rplidar_node)"
fi
# shellcheck disable=SC1090
source "$RPLIDAR_WS/local_setup.bash" 2>/dev/null || true

echo "== ROS packages =="
for p in rtabmap_slam rtabmap_launch rtabmap_util tf2_ros; do
  ros2 pkg prefix "$p" >/dev/null 2>&1 && ok "$p" || bad "$p"
done

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
[ -n "$S2_TTY" ] && ok "RPLIDAR S2 ($S2_TTY)" || warn "RPLIDAR S2" "not found"
id -nG 2>/dev/null | grep -qw dialout && ok "dialout group" || warn "dialout group" "missing"

echo
if [ "$CHECK_ONLY" = "1" ]; then
  if [ "${#APT_MISSING[@]}" -eq 0 ] && [ "$FAIL" -eq 0 ]; then
    echo "check-only: OK (warnings: $WARN)"; exit 0
  fi
  echo "check-only: apt missing=${#APT_MISSING[@]}, failures present, warnings=$WARN"; exit 1
fi

if [ "${#APT_MISSING[@]}" -gt 0 ]; then
  if [ "$(id -u)" -eq 0 ]; then APT=(apt-get); else sudo -v; APT=(sudo apt-get); fi
  "${APT[@]}" install -y "${APT_MISSING[@]}"
  for p in "${APT_MISSING[@]}"; do
    dpkg -s "$p" >/dev/null 2>&1 && ok "$p" || { say "$p" "INSTALL FAILED"; FAIL=1; }
  done
fi

echo
[ "$FAIL" -eq 0 ] && echo "rtabmap deps OK (warnings: $WARN)" && exit 0
echo "rtabmap deps: some checks failed" >&2
exit 1
