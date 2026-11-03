#!/usr/bin/env bash
# Build the C++ gateway in place (gateway/build/robot_console_gateway).
#
# The board has 2.9 GB RAM and no swap; one g++ process on ROS 2 headers can
# take >1 GB, and an unbounded -j4 build next to the running stack crashed the
# board (2026-11-03).  So: one job, low priority, GCC collects garbage early,
# and the whole build runs in a memory-capped cgroup - if it does not fit,
# only the compiler is killed.
#
#   ./build.sh                    # MEM_MAX=1300M JOBS=1
#   MEM_MAX=1800M ./build.sh      # more headroom when the ROS stack is stopped
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd)"
JOBS="${JOBS:-1}"
MEM_MAX="${MEM_MAX:-1300M}"

if [ "${_IN_SCOPE:-0}" != "1" ] && command -v systemd-run >/dev/null 2>&1 \
   && grep -qw memory /sys/fs/cgroup/cgroup.controllers 2>/dev/null; then
  echo "building in a memory-capped scope (MemoryMax=$MEM_MAX, -j$JOBS)"
  exec systemd-run --scope --quiet -p MemoryMax="$MEM_MAX" -p MemorySwapMax=0 \
    env _IN_SCOPE=1 JOBS="$JOBS" nice -n 19 bash "$SCRIPT_DIR/build.sh"
fi

source /opt/ros/jazzy/setup.bash
cmake -S "$SCRIPT_DIR" -B "$SCRIPT_DIR/build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS_RELEASE="-O2 -DNDEBUG --param ggc-min-expand=20 --param ggc-min-heapsize=32768"
cmake --build "$SCRIPT_DIR/build" -j"$JOBS"
CG="/sys/fs/cgroup$(cut -d: -f3 /proc/self/cgroup)"
if [ -r "$CG/memory.peak" ]; then
  echo "peak memory: $(( $(cat "$CG/memory.peak") / 1048576 )) MB"
fi
echo "built $SCRIPT_DIR/build/robot_console_gateway"
