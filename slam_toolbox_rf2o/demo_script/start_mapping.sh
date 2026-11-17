#!/usr/bin/env bash
# Start the mapping stack so a map can be built manually from RViz.
#
# Starts:
#   RPLIDAR S2 + rf2o + slam_toolbox (mapping)
#   RealSense + EKF
#   Nav2 + chassis driver
#
# AI, person_target_recovery and demo_manager are not started.
#
# Workflow:
#   1. ./start_mapping.sh
#   2. Set 2D Goal Pose / NavigateToPose goals in RViz to drive around
#   3. ./record_map.sh <name>        # save pgm/yaml + posegraph
#   4. ./stop_demo.sh
#
# Usage:
#   ./start_mapping.sh
#   SLAM_PARAMS=/path/other_mapping.yaml ./start_mapping.sh
#   RECORD=1 ./start_mapping.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"

# Mapping needs the full SLAM + EKF + Nav2 stack, but no AI.
export START_AI=0

exec "$SCRIPT_DIR/start_demo.sh" "$@"
