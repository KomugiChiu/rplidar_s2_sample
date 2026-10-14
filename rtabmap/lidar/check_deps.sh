#!/usr/bin/env bash
# Dependency check (+auto-install) for the rtabmap + RPLidar S2 flow.
# Run ON BOARD. Password is typed interactively at runtime (never stored).
#
# Usage:
#   ./check_deps.sh              # check; install whatever apt packages miss
#   ./check_deps.sh --check-only # report only, install nothing
# Env:
#   RPLIDAR_WS (default /nvt_ros2_example/rplidar_ros)  SERIAL_PORT (auto-detected
#   by VID:PID 10c4:ea60; explicit env wins only if no S2 tty is found)
set -eo pipefail   # NOTE: no -u (ROS setup files need unset vars)

CHECK_ONLY=0
[ "${1:-}" = "--check-only" ] && CHECK_ONLY=1

APT_PKGS=(ros-jazzy-rtabmap-launch ros-jazzy-rtabmap-odom ros-jazzy-rtabmap-slam
          ros-jazzy-nav2-map-server)
RPLIDAR_WS="${RPLIDAR_WS:-/nvt_ros2_example/rplidar_ros}"
SERIAL_PORT_EXPLICIT=0; [ -n "${SERIAL_PORT+x}" ] && SERIAL_PORT_EXPLICIT=1
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyUSB0}"

FAIL=0
say() { printf '%-28s %s\n' "$1" "$2"; }

echo "== apt packages =="
MISSING=()
for p in "${APT_PKGS[@]}"; do
  if dpkg -s "$p" >/dev/null 2>&1; then say "$p" OK; else say "$p" MISSING; MISSING+=("$p"); fi
done

echo "== rplidar_ros workspace (check only, never auto-built) =="
if [ -f "$RPLIDAR_WS/setup.bash" ] && [ -x "$RPLIDAR_WS/rplidar_ros/lib/rplidar_ros/rplidar_node" ]; then
  say "$RPLIDAR_WS" OK
else
  say "$RPLIDAR_WS" "MISSING -> ERROR (set RPLIDAR_WS to use another one, e.g. ~/test/rplidar_ros)"
  FAIL=1
fi

echo "== rplidar S2 USB tty (VID:PID 10c4:ea60) =="
S2TTY=""
for dev in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$dev" ] || continue
  if udevadm info -a -n "$dev" 2>/dev/null | grep -q 'ATTRS{idVendor}=="10c4"'; then
    S2TTY="$dev"
    say "S2 found at" "$dev"
  fi
done
[ -n "$S2TTY" ] && [ "$SERIAL_PORT_EXPLICIT" = "0" ] && SERIAL_PORT="$S2TTY"
if [ -z "$S2TTY" ]; then say "S2 tty" "not found (unplugged?)"; fi

echo "== serial permission =="
if [ -e "$SERIAL_PORT" ]; then say "$SERIAL_PORT" "exists"; else say "$SERIAL_PORT" "absent"; fi
if id -nG 2>/dev/null | grep -qw dialout; then
  NEED_DIALOUT=0; say "dialout group" OK
else
  NEED_DIALOUT=1; say "dialout group" "MISSING (needs sudo usermod + re-login)"
fi

if [ "$CHECK_ONLY" = "1" ]; then
  echo "check-only: ${#MISSING[@]} apt missing, FAIL=$FAIL"
  [ "${#MISSING[@]}" -eq 0 ] && [ "$FAIL" -eq 0 ] && exit 0 || exit 1
fi

if [ "${#MISSING[@]}" -gt 0 ]; then
  echo "installing: ${MISSING[*]} (sudo password asked once here)"
  sudo -v
  sudo apt-get install -y "${MISSING[@]}"
  for p in "${MISSING[@]}"; do
    dpkg -s "$p" >/dev/null 2>&1 || { echo "FAILED to install $p"; FAIL=1; }
  done
fi
if [ "$NEED_DIALOUT" = "1" ]; then
  echo "adding user to dialout (sudo password asked once here)"
  sudo -v
  sudo usermod -aG dialout "$USER"
  echo "WARN: re-login (or reboot) for dialout to take effect"
fi

echo "done: FAIL=$FAIL"
exit "$FAIL"
