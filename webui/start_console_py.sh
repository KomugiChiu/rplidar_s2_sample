#!/usr/bin/env bash
# Start the robot web console with the Python ROS gateway (gateway_py/).
# Use this until the cross-compiled C++ gateway is available in gateway/.
# Python costs more CPU (~2 ms per ROS callback on this board), so it samples
# the high-rate topics instead of watching them continuously.
#
#   ./start_console_py.sh            # http://<board-ip>:8080
#   ./stop_console_py.sh
SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd)"
GATEWAY="$SCRIPT_DIR/gateway_py/run_gateway.sh" exec "$SCRIPT_DIR/start_console.sh" "$@"
