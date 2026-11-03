#!/usr/bin/env bash
# Stop everything started by start_all_amcl.sh.
#
# Same process tree as the mapping flow (lidar, rf2o, static TF, RealSense+EKF,
# Nav2 launch + driver + cmd_vel logger), so this delegates to stop_all.sh and
# then sweeps the two nodes that only exist in AMCL mode.
#
#   ./stop_all_amcl.sh
#   KEEP_DRIVER=1 ./stop_all_amcl.sh    # keep the chassis driver running
set -u

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]:-$0}" 2>/dev/null || echo "${BASH_SOURCE[0]:-$0}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)"

echo "== stop_all_amcl =="

"$SCRIPT_DIR/stop_all.sh" || true

# Belt and braces: map_server / AMCL live inside the nav2_bringup launch group,
# which stop_all.sh kills, but a crashed launch can leave them behind.
PATTERNS=(
    "nav2_map_server/map_server"
    "nav2_amcl/amcl"
)
for pat in "${PATTERNS[@]}"; do
    pkill -TERM -f "$pat" 2>/dev/null || true
done
sleep 1
for pat in "${PATTERNS[@]}"; do
    pkill -KILL -f "$pat" 2>/dev/null || true
done

REMAIN="$(pgrep -af 'nav2_map_server/map_server|nav2_amcl/amcl' 2>/dev/null || true)"
if [ -n "$REMAIN" ]; then
    echo "WARN: still running:"
    echo "$REMAIN"
else
    echo "map_server / AMCL stopped"
fi
