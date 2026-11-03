#!/usr/bin/env bash
# Stop the robot web console started with start_console_py.sh (same as stop_console.sh;
# both gateways are cleaned up).  Does NOT touch the ROS stack.
SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd)"
exec "$SCRIPT_DIR/stop_console.sh" "$@"
