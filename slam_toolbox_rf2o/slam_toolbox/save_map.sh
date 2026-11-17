#!/usr/bin/env bash
# Save the current slam_toolbox map (pgm/yaml) and posegraph.
#
# IMPORTANT: serialize_map is only available while slam_toolbox is running in
# MAPPING mode. It cannot be called from localization mode; in localization
# mode the posegraph already exists on disk.
#
# Usage:
#   ./save_map.sh                  # maps/slam_map_<timestamp>
#   ./save_map.sh my_map           # maps/my_map
#   MAP_DIR=/path ./save_map.sh x  # /path/x
#   ./save_map.sh /path/x          # explicit prefix
#
# Output:
#   <prefix>.pgm
#   <prefix>.yaml
#   <prefix>.posegraph
#   <prefix>.data
#
# Stop mapping first? No. Keep it running while saving.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
ROOT_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-99}"
MAP_DIR="${MAP_DIR:-$ROOT_DIR/maps}"

NAME="${1:-slam_map_$(date +%Y%m%d-%H%M%S)}"
case "$NAME" in
  */*) PREFIX="$NAME" ;;
  *)   PREFIX="$MAP_DIR/$NAME" ;;
esac
mkdir -p "$(dirname -- "$PREFIX")"

# shellcheck disable=SC1090
source /opt/ros/jazzy/setup.bash

echo "== save map =="
echo "   prefix : $PREFIX"
echo "   domain : $ROS_DOMAIN_ID"

# --- mode check ------------------------------------------------------------
if pgrep -f '[l]ocalization_slam_toolbox_node' >/dev/null 2>&1; then
  echo "ERROR: slam_toolbox is in LOCALIZATION mode." >&2
  echo "       serialize_map is not available in localization mode." >&2
  echo "       Run ./start_mapping.sh first if you need to build a new posegraph." >&2
  exit 1
fi
if ! pgrep -f '[a]sync_slam_toolbox_node' >/dev/null 2>&1 &&
   ! pgrep -f '[s]ync_slam_toolbox_node' >/dev/null 2>&1; then
  echo "ERROR: no slam_toolbox mapping node is running." >&2
  echo "       Start it with ./start_mapping.sh." >&2
  exit 1
fi

# --- map image / yaml ------------------------------------------------------
if ! ros2 pkg executables nav2_map_server 2>/dev/null |
     grep -qx "nav2_map_server map_saver_cli"; then
  echo "ERROR: nav2_map_server/map_saver_cli not found." >&2
  exit 1
fi

MAP_INFO="$(timeout 10 ros2 topic info /map 2>/dev/null || true)"
if ! grep -qE 'Publisher count: [1-9]' <<<"$MAP_INFO"; then
  echo "ERROR: /map has no publisher. Is slam_toolbox running?" >&2
  exit 1
fi

echo "   saving map image / yaml ..."
if ! ros2 run nav2_map_server map_saver_cli -f "$PREFIX"; then
  echo "ERROR: map_saver_cli failed." >&2
  exit 1
fi

MISSING=0
for ext in pgm yaml; do
  if [ ! -s "$PREFIX.$ext" ]; then
    echo "ERROR: missing $PREFIX.$ext" >&2
    MISSING=1
  fi
done
[ "$MISSING" -eq 0 ] || exit 1

# --- posegraph -------------------------------------------------------------
if ! ros2 service list 2>/dev/null | grep -qx "/slam_toolbox/serialize_map"; then
  echo "ERROR: /slam_toolbox/serialize_map service not found." >&2
  exit 1
fi

echo "   saving posegraph ..."
if ! timeout 60 ros2 service call /slam_toolbox/serialize_map \
     slam_toolbox/srv/SerializePoseGraph \
     "{filename: \"$PREFIX\"}"; then
  echo "ERROR: serialize_map service call failed or timed out." >&2
  exit 1
fi

# serialize_map may return result=0 even when it refuses (localization mode).
# The files are the real proof.
for ext in posegraph data; do
  if [ ! -s "$PREFIX.$ext" ]; then
    echo "ERROR: missing $PREFIX.$ext" >&2
    echo "       serialize_map did not write the posegraph." >&2
    exit 1
  fi
done

echo
echo "saved:"
ls -lh "$PREFIX".pgm "$PREFIX".yaml "$PREFIX".posegraph "$PREFIX".data
echo
echo "Use this prefix in a localization yaml:"
echo "  map_file_name: \"$PREFIX\""
