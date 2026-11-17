#!/usr/bin/env bash
# Stop the standalone chassis driver started by start_chassis.sh.
#
# NOTE: this kills ANY Mcnamu_driver on the board, including the one started by
#       nav2_1028/start_nav2.sh.  It does not touch Nav2 itself.
#
# Usage:
#   ./stop_chassis.sh
#   DRY_RUN=1 ./stop_chassis.sh
set -eo pipefail

PID_FILE="${PID_FILE:-/tmp/nav2_chassis/pids}"
DRY_RUN="${DRY_RUN:-0}"

PATTERNS=(
  "ros2 launch yahboomcar_bringup bringup.launch.py"
  "[M]cnamu_driver"
)

collect_pids() {
    local out=""
    for pat in "${PATTERNS[@]}"; do
        for pid in $(pgrep -f "$pat" 2>/dev/null || true); do
            case " $out " in *" $pid "*) ;; *) out="$out $pid" ;; esac
        done
    done
    echo "${out# }"
}

PIDS="$(collect_pids)"

if [ -s "$PID_FILE" ]; then
    while read -r pid name mode; do
        [ -n "${pid:-}" ] || continue
        [ -d "/proc/$pid" ] || continue
        case " $PIDS " in *" $pid "*) ;; *) PIDS="$PIDS $pid" ;; esac
    done < "$PID_FILE"
fi

PIDS="${PIDS# }"

if [ -z "$PIDS" ]; then
    echo "chassis driver is not running"
    rm -f "$PID_FILE"
    exit 0
fi

echo "== chassis processes =="
for pid in $PIDS; do
    printf '  %-8s %s\n' "$pid" \
        "$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | cut -c1-100)"
done

if [ "$DRY_RUN" = "1" ]; then
    echo "DRY_RUN: nothing was killed"
    exit 0
fi

echo "-- sending SIGTERM"
for pid in $PIDS; do
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
done

sleep 3

STUCK=""
for pid in $PIDS; do
    [ -d "/proc/$pid" ] && STUCK="$STUCK $pid"
done

if [ -n "$STUCK" ]; then
    echo "-- sending SIGKILL to$STUCK"
    for pid in $STUCK; do
        kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
    done
    sleep 1
fi

rm -f "$PID_FILE"

if pgrep -f "[M]cnamu_driver" >/dev/null 2>&1; then
    echo "WARN: Mcnamu_driver is still running"
    pgrep -af "[M]cnamu_driver" | head -n 3
    exit 1
fi

echo "chassis driver stopped"
