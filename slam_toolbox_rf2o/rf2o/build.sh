#!/usr/bin/env bash
# do_compile + do_install (simulated): colcon build with --merge-install, so a
# single prefix ends up with BOTH the compiled node and the files/ overlay
# (start.sh / stop.sh / rf2o_params.yaml):
#
#   install/lib/rf2o_laser_odometry/rf2o_laser_odometry_node
#   install/share/rf2o_laser_odometry/launch
#   install/start.sh  install/stop.sh  install/rf2o_params.yaml
#   install/setup.bash / local_setup.bash
#
# NOTE: the node is compiled for the machine running this script.  The board
# is aarch64, so build there (or cross-compile with a matching toolchain);
# a notebook (x86_64) build is only good for checking the recipe.
#
# Usage:
#   ./fetch.sh && ./build.sh          # or: make all
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SRC="$SCRIPT_DIR/src/rf2o_laser_odometry"

[ -d "$SRC" ] || { echo "ERROR: src/ missing; run ./fetch.sh (or 'make fetch') first" >&2; exit 1; }

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
# /opt/ros setup scripts reference undeclared variables; relax nounset for the
# source, restore afterwards.
set +u
# shellcheck disable=SC1091
source "/opt/ros/$ROS_DISTRO/setup.bash"
set -u

cd "$SCRIPT_DIR"
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "== install/ =="
ls -l install/lib/rf2o_laser_odometry/ 2>/dev/null | grep -v '^total' || true
ls -l install/start.sh install/stop.sh install/rf2o_params.yaml 2>/dev/null || true
