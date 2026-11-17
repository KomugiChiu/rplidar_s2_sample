#!/usr/bin/env bash
# Dependency check (+auto-install) for the D455 + EKF part of the stack.
# Run ON BOARD.
#
# This covers ekf/start_ekf.sh:
#   realsense2_camera  -> D455 gyro / accel / RGBD
#   static_transform_publisher -> base_footprint -> camera_link
#   robot_localization ekf_node -> /odometry/filtered + odom -> base_footprint
#
# EKF uses the apt realsense2_camera package directly; the separate
# realsense-ros workspace is only used by the AI flow when it starts its own
# camera.
#
# Usage:
#   ./check_deps.sh                 # check, install missing apt packages
#   ./check_deps.sh --check-only    # report only, install nothing
#   ./check_deps.sh --help
#
# Env overrides:
#   ROS_DISTRO, EKF_PARAMS_FILE
set -eo pipefail   # NOTE: no -u (ROS setup files need unset vars)

CHECK_ONLY=0
case "${1:-}" in
  --check-only) CHECK_ONLY=1 ;;
  --help|-h)
    sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
    ;;
  "") ;;
  *)
    echo "unknown option: $1 (use --check-only or --help)" >&2
    exit 2
    ;;
esac

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
PARAMS_FILE="${EKF_PARAMS_FILE:-$SCRIPT_DIR/ekf_params.yaml}"
START_SCRIPT="$SCRIPT_DIR/start_ekf.sh"
STOP_SCRIPT="$SCRIPT_DIR/stop_ekf.sh"

APT_PKGS=(
  "ros-${ROS_DISTRO}-robot-localization"
  "ros-${ROS_DISTRO}-realsense2-camera"
  "ros-${ROS_DISTRO}-tf2-ros"
)

FAIL=0
WARN=0
say()  { printf '%-44s %s\n' "$1" "$2"; }
ok()   { say "$1" "OK"; }
bad()  { say "$1" "MISSING"; FAIL=1; }
warn() { say "$1" "$2"; WARN=1; }

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
  bad "ros2 CLI"
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

echo "== ekf files =="
if [ -f "$PARAMS_FILE" ]; then
  ok "${PARAMS_FILE#$SCRIPT_DIR/}"
else
  bad "${PARAMS_FILE#$SCRIPT_DIR/} (EKF params)"
fi

for f in "$START_SCRIPT" "$STOP_SCRIPT"; do
  rel="${f#$SCRIPT_DIR/}"
  if [ -x "$f" ]; then
    ok "$rel"
  else
    bad "$rel (missing or not executable)"
  fi
done

echo "== ekf_params.yaml =="
if [ ! -f "$PARAMS_FILE" ]; then
  bad "config check skipped (no params file)"
elif ! command -v python3 >/dev/null 2>&1; then
  warn "python3" "missing -> config check skipped"
elif ! python3 -c 'import yaml' >/dev/null 2>&1; then
  warn "python3-yaml" "missing -> config check skipped"
else
  if python3 - "$PARAMS_FILE" <<'PY'
import sys
import yaml

path = sys.argv[1]
try:
    doc = yaml.safe_load(open(path, encoding="utf-8"))
except Exception as exc:
    print(f"cannot parse {path}: {exc}")
    raise SystemExit(1)

params = (doc or {}).get("ekf_filter_node", {}).get("ros__parameters", {})
if not params:
    print("missing ekf_filter_node.ros__parameters")
    raise SystemExit(1)

required = {
    "frequency": 20.0,
    "two_d_mode": True,
    "world_frame": "odom",
    "odom_frame": "odom",
    "base_link_frame": "base_footprint",
    "publish_tf": True,
    "odom0": "/odom_rf2o",
    "imu0": "/camera/camera/gyro/sample",
}

bad = []
for key, expected in required.items():
    if key not in params:
        bad.append(f"missing {key}")
    elif params[key] != expected:
        bad.append(f"{key}={params[key]!r} (expected {expected!r})")

for item in bad:
    print(item)
raise SystemExit(1 if bad else 0)
PY
  then
    ok "required EKF keys / values"
  else
    bad "EKF config check (see above)"
  fi
fi

echo "== ROS packages / executables =="
if ros2 pkg prefix robot_localization >/dev/null 2>&1; then
  ok "robot_localization"
else
  bad "robot_localization"
fi
if ros2 pkg prefix realsense2_camera >/dev/null 2>&1; then
  ok "realsense2_camera"
else
  bad "realsense2_camera"
fi
if ros2 pkg prefix tf2_ros >/dev/null 2>&1; then
  ok "tf2_ros"
else
  bad "tf2_ros"
fi

if ros2 pkg executables robot_localization 2>/dev/null | grep -qx "robot_localization ekf_node"; then
  ok "ekf_node"
else
  bad "ekf_node"
fi
if ros2 pkg executables realsense2_camera 2>/dev/null | grep -qx "realsense2_camera realsense2_camera_node"; then
  ok "realsense2_camera_node"
else
  bad "realsense2_camera_node"
fi
if ros2 pkg executables tf2_ros 2>/dev/null | grep -qx "tf2_ros static_transform_publisher"; then
  ok "static_transform_publisher"
else
  bad "static_transform_publisher"
fi

echo "== hardware =="
if command -v lsusb >/dev/null 2>&1 && lsusb 2>/dev/null | grep -qiE '8086:0b5c|Intel.*RealSense'; then
  ok "RealSense D455 (USB)"
else
  warn "RealSense D455" "not found (USB unplugged?)"
fi

if ls /dev/video* >/dev/null 2>&1; then
  ok "/dev/video*"
else
  warn "/dev/video*" "missing -> camera may fail to open"
fi

if id -nG 2>/dev/null | grep -qw video; then
  ok "video group"
else
  warn "video group" "missing -> sudo usermod -aG video $USER (re-login)"
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
  echo "ekf deps OK (warnings: $WARN)"
  exit 0
fi
echo "ekf deps: some checks failed" >&2
exit 1
