#!/usr/bin/env bash
# Start slam_toolbox in LOCALIZATION mode.
#
# Kept as a compatibility wrapper for the unified start.sh:
#   ./start.sh localization
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
exec "$SCRIPT_DIR/start.sh" localization
