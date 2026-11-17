#!/usr/bin/env bash
# Stop rf2o started by start.sh.
#
# Usage:
#   ./stop.sh
#   DRY_RUN=1 ./stop.sh
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")" && pwd)"
RUN_DIR="${RUN_DIR:-/tmp/rf2o_only}"
PID_FILE="${PID_FILE:-$RUN_DIR/pids}"
DRY_RUN="${DRY_RUN:-0}"

log() { echo "[rf2o] $*"; }

if [ -s "$PID_FILE" ]; then
  while read -r pid name; do
    [ -n "${pid:-}" ] || continue
    if ! kill -0 "$pid" 2>/dev/null; then
      continue
    fi
    log "stopping $name (pid $pid)"
    [ "$DRY_RUN" = "1" ] && continue
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  done < "$PID_FILE"

  if [ "$DRY_RUN" != "1" ]; then
    sleep 2
    while read -r pid name; do
      [ -n "${pid:-}" ] || continue
      if kill -0 "$pid" 2>/dev/null; then
        log "killing $name (pid $pid)"
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
      fi
    done < "$PID_FILE"
    rm -f "$PID_FILE"
  fi
else
  log "no PID file: $PID_FILE"
  if [ "$DRY_RUN" != "1" ]; then
    pkill -TERM -f '[r]f2o_laser_odometry_node' 2>/dev/null || true
  fi
fi

log "done"
