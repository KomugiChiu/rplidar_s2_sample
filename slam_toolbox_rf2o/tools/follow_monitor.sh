#!/usr/bin/env bash
# follow_monitor.sh — one-command person-follow recording / monitoring
#
# Usage:
#   ./follow_monitor.sh start   [--label NAME] [--dir DIR] [--duration SEC]
#                               [--no-bag] [--no-cpu] [--no-watch] [--no-tails]
#   ./follow_monitor.sh stop    [--dir DIR]
#   ./follow_monitor.sh status  [--dir DIR]
#   ./follow_monitor.sh summary [--dir DIR]
#   ./follow_monitor.sh stack   [--dir DIR]   refresh stack.txt snapshot
#   ./follow_monitor.sh list
#
# start creates:
#   monitor_logs/run_<YYYYmmdd-HHMMSS>_<label>/
#     bag/                       ros2 bag (mcap)
#     bag.log  bag.pid
#     cpu.csv  cpu.log  cpu_procs.csv  cpu.pid
#     follow_target.csv  follow_watch.log  follow_watch.pid
#     demo_manager_tail.log  ai_node_tail.log  recovery_tail.log  (and .pid)
#     stack.txt                  camera / params / node / topic snapshot
#     RUN_INFO.txt
#   monitor_logs/.last_follow_run -> latest run dir
#
# stop sends SIGTERM to every pid file, waits, then SIGKILL if needed.
# The latest run dir is remembered in monitor_logs/.last_follow_run.
#
# Environment overrides:
#   BASE_D       monitor_logs dir
#   ROS_DOMAIN_ID
#   WATCH_PY     follow_watch.py path   (default /temp/script/follow_watch.py)
#   CPU_PY       monitor_cpu.py path
#   AI_LOG_DIR   /tmp/slam_toolbox_rf2o_ai
#
# Persistent install: /temp/script/follow_monitor.sh

set -uo pipefail

BASE_D="${BASE_D:-/nvt_ros2_example/slam_toolbox_rf2o/monitor_logs}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
AI_SETUP="${AI_SETUP:-/nvt_ros2_example/ai_human_detection/local_setup.bash}"
WS_SETUP="${WS_SETUP:-/root/ros2_ws/install/local_setup.bash}"
WATCH_PY="${WATCH_PY:-/temp/script/follow_watch.py}"
CPU_PY="${CPU_PY:-/nvt_ros2_example/slam_toolbox_rf2o/nav2_1028/tools/monitor_cpu.py}"
AI_LOG_DIR="${AI_LOG_DIR:-/tmp/slam_toolbox_rf2o_ai}"
DOMAIN="${ROS_DOMAIN_ID:-99}"
POINTER="$BASE_D/.last_follow_run"

say() { printf '%s\n' "$*"; }
warn() { printf '[WARN] %s\n' "$*" >&2; }
die() { printf '[ERROR] %s\n' "$*" >&2; exit 1; }

usage() { sed -n '2,40p' "$0"; }

ros_env() {
  # ROS setup.bash references unbound vars; disable nounset while sourcing.
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP" >/dev/null 2>&1
  [ -f "$AI_SETUP" ] && source "$AI_SETUP" >/dev/null 2>&1
  [ -f "$WS_SETUP" ] && source "$WS_SETUP" >/dev/null 2>&1
  set -u
  export ROS_DOMAIN_ID="$DOMAIN"
}

alive() { [ -n "${1:-}" ] && kill -0 "$1" 2>/dev/null; }

read_run() {
  local dir="${1:-}"
  if [ -z "$dir" ]; then
    [ -f "$POINTER" ] || die "no run found ($POINTER missing); pass --dir"
    dir="$(cat "$POINTER")"
  fi
  [ -d "$dir" ] || die "run dir not found: $dir"
  printf '%s' "$dir"
}

write_env_file() {
  local run="$1"
  cat > "$run/.env.sh" <<EOF
set +u
source "$ROS_SETUP" >/dev/null 2>&1
[ -f "$AI_SETUP" ] && source "$AI_SETUP" >/dev/null 2>&1
[ -f "$WS_SETUP" ] && source "$WS_SETUP" >/dev/null 2>&1
set -u
export ROS_DOMAIN_ID="$DOMAIN"
EOF
}

# ros2 param get occasionally hits a transient CLI/daemon error; retry.
pget() {
  local node="$1" p="$2" out="" i
  for i in 1 2 3; do
    out="$(timeout -k 2 5 ros2 param get "$node" "$p" 2>&1 | tail -n1)"
    case "$out" in
      *"value is:"*) printf '%s' "$out"; return 0 ;;
    esac
    sleep 1
  done
  printf '%s' "${out:-unavailable}"
}

collect_stack() {
  local run="$1"
  {
    date
    uptime
    echo "--- camera profiles ---"
    for p in rgb_camera.color_profile depth_module.depth_profile; do
      echo "$p = $(pget /camera/camera "$p")"
    done
    echo "--- demo params ---"
    for p in follow_distance goal_update_distance goal_update_angle \
             follow_timer_period_sec person_timeout_sec person_topic; do
      echo "demo $p = $(pget /demo_manager_node "$p")"
    done
    echo "--- recovery params ---"
    for p in reassociation_timeout_sec reassociation_distance reject_invalid_position; do
      echo "recovery $p = $(pget /person_target_recovery_node "$p")"
    done
    echo "--- nodes ---"
    timeout -k 2 8 ros2 node list 2>/dev/null | sort
    echo "--- topics ---"
    timeout -k 2 8 ros2 topic list 2>/dev/null | sort
  } > "$run/stack.txt" 2>&1
}

cmd_start() {
  local label="follow" duration=1800 dir="" do_bag=1 do_cpu=1 do_watch=1 do_tails=1
  while [ $# -gt 0 ]; do
    case "$1" in
      --label) label="$2"; shift ;;
      --dir) dir="$2"; shift ;;
      --duration) duration="$2"; shift ;;
      --no-bag) do_bag=0 ;;
      --no-cpu) do_cpu=0 ;;
      --no-watch) do_watch=0 ;;
      --no-tails) do_tails=0 ;;
      -h|--help) usage; exit 0 ;;
      *) die "unknown option: $1" ;;
    esac
    shift
  done

  mkdir -p "$BASE_D"

  # refuse to start twice
  if [ -f "$POINTER" ]; then
    local last
    last="$(cat "$POINTER")"
    if [ -d "$last" ]; then
      local p live=0
      for p in "$last"/*.pid; do
        [ -f "$p" ] || continue
        alive "$(cat "$p")" && live=1
      done
      if [ "$live" = "1" ]; then
        die "monitor already running: $last (run 'stop' first)"
      fi
    fi
  fi

  local ts
  ts="$(date +%Y%m%d-%H%M%S)"
  local run="${dir:-$BASE_D/run_${ts}_${label}}"
  mkdir -p "$run"
  printf '%s\n' "$run" > "$POINTER"
  write_env_file "$run"

  ros_env

  # preflight: warn (do not abort) when the stack looks incomplete
  local preflight
  preflight="$(timeout -k 2 8 ros2 topic list 2>/dev/null)"
  local need
  for need in /scan /odometry/filtered /person_infos /demo/state; do
    printf '%s\n' "$preflight" | grep -qx "$need" || warn "topic not found: $need"
  done

  # -------------------------------------------------------------------------
  # bag
  # -------------------------------------------------------------------------
  if [ "$do_bag" = "1" ]; then
    local topics=(
      /scan /odometry/filtered /odom_rf2o /cmd_vel /tf /tf_static /map
      /camera/camera/gyro/sample /camera/camera/color/camera_info
      /person_infos /person_infos_stable /human_detections
      /demo/state /demo/selected_tracking_id /demo/person_markers
      /demo/current_goal_marker /demo/robot_path /demo/robot_marker
      /clicked_point /goal_pose
    )
    local avail t
    avail="$(timeout -k 2 8 ros2 topic list 2>/dev/null)"
    for t in /cmd_vel_nav /cmd_vel_safe /cmd_vel_floor; do
      printf '%s\n' "$avail" | grep -qx "$t" && topics+=("$t")
    done

    local bag_cmd="exec ros2 bag record -o $(printf '%q' "$run/bag")"
    for t in "${topics[@]}"; do
      bag_cmd+=" $(printf '%q' "$t")"
    done
    setsid bash -c "source $(printf '%q' "$run/.env.sh"); $bag_cmd" \
      > "$run/bag.log" 2>&1 < /dev/null &
    echo $! > "$run/bag.pid"
    say "bag        : pid $(cat "$run/bag.pid")  (${#topics[@]} topics)"
  fi

  # -------------------------------------------------------------------------
  # CPU monitor
  # -------------------------------------------------------------------------
  if [ "$do_cpu" = "1" ]; then
    if [ -f "$CPU_PY" ]; then
      setsid python3 "$CPU_PY" --interval 2 --duration "$duration" --top 10 \
        --csv "$run/cpu.csv" > "$run/cpu.log" 2>&1 < /dev/null &
      echo $! > "$run/cpu.pid"
      say "cpu        : pid $(cat "$run/cpu.pid")"
    else
      warn "cpu monitor not found: $CPU_PY"
    fi
  fi

  # -------------------------------------------------------------------------
  # follow_watch
  # -------------------------------------------------------------------------
  if [ "$do_watch" = "1" ]; then
    if [ -f "$WATCH_PY" ]; then
      setsid bash -c "source $(printf '%q' "$run/.env.sh"); \
        exec python3 $(printf '%q' "$WATCH_PY") \
          --out $(printf '%q' "$run/follow_target.csv") --duration $duration" \
        > "$run/follow_watch.log" 2>&1 < /dev/null &
      echo $! > "$run/follow_watch.pid"
      say "follow_watch: pid $(cat "$run/follow_watch.pid")"
    else
      warn "follow_watch.py not found: $WATCH_PY"
    fi
  fi

  # -------------------------------------------------------------------------
  # log tails
  # -------------------------------------------------------------------------
  if [ "$do_tails" = "1" ]; then
    local name
    for name in demo_manager ai_node person_target_recovery; do
      setsid bash -c "exec tail -F $(printf '%q' "$AI_LOG_DIR/$name.log")" \
        > "$run/${name}_tail.log" 2>&1 < /dev/null &
      echo $! > "$run/${name}_tail.pid"
    done
    say "tails      : demo_manager / ai_node / person_target_recovery"
  fi

  # -------------------------------------------------------------------------
  # environment snapshot
  # -------------------------------------------------------------------------
  collect_stack "$run"
  {
    echo "run_dir   : $run"
    echo "started   : $(date '+%Y-%m-%d %H:%M:%S')"
    echo "label     : $label"
    echo "duration  : $duration"
    echo "bag       : $do_bag"
    echo "cpu       : $do_cpu"
    echo "watch     : $do_watch"
    echo "tails     : $do_tails"
    echo "domain    : $DOMAIN"
    echo "watch_py  : $WATCH_PY"
    echo "cpu_py    : $CPU_PY"
  } > "$run/RUN_INFO.txt"

  say ""
  say "RUN=$run"
  say "stop with: $0 stop"
  say "status   : $0 status"
  say "summary  : $0 summary"
}

cmd_stop() {
  local dir="" name pf pid
  while [ $# -gt 0 ]; do
    case "$1" in
      --dir) dir="$2"; shift ;;
      -h|--help) usage; exit 0 ;;
      *) die "unknown option: $1" ;;
    esac
    shift
  done
  local run
  run="$(read_run "$dir")"

  say "== stop =="
  say "RUN=$run"

  for pf in "$run"/*.pid; do
    [ -f "$pf" ] || continue
    name="$(basename "$pf" .pid)"
    pid="$(cat "$pf" 2>/dev/null || true)"
    if alive "$pid"; then
      kill -TERM "$pid" 2>/dev/null || true
      say "  TERM $name ($pid)"
    else
      say "  skip $name (not running)"
    fi
  done

  # wait up to 12 s
  local i left
  for i in $(seq 1 12); do
    left=0
    for pf in "$run"/*.pid; do
      [ -f "$pf" ] || continue
      pid="$(cat "$pf" 2>/dev/null || true)"
      alive "$pid" && left=1
    done
    [ "$left" = "0" ] && break
    sleep 1
  done

  for pf in "$run"/*.pid; do
    [ -f "$pf" ] || continue
    name="$(basename "$pf" .pid)"
    pid="$(cat "$pf" 2>/dev/null || true)"
    if alive "$pid"; then
      kill -KILL "$pid" 2>/dev/null || true
      say "  KILL $name ($pid)"
    fi
  done

  say ""
  say "== files =="
  ls -lh "$run" 2>/dev/null | sed 's/^/  /'
  if [ -d "$run/bag" ]; then
    ls -lh "$run/bag" 2>/dev/null | tail -n 3 | sed 's/^/  /'
    [ -f "$run/bag/metadata.yaml" ] || \
      warn "bag has no metadata.yaml; run: ros2 bag reindex $run/bag"
  fi
  say ""
  say "stopped."
}

cmd_status() {
  local dir="" pf name pid
  while [ $# -gt 0 ]; do
    case "$1" in
      --dir) dir="$2"; shift ;;
      -h|--help) usage; exit 0 ;;
      *) die "unknown option: $1" ;;
    esac
    shift
  done
  local run
  run="$(read_run "$dir")"
  say "RUN=$run"
  say ""
  for pf in "$run"/*.pid; do
    [ -f "$pf" ] || continue
    name="$(basename "$pf" .pid)"
    pid="$(cat "$pf" 2>/dev/null || true)"
    if alive "$pid"; then
      printf '  [RUN ] %-22s %s\n' "$name" "$pid"
    else
      printf '  [STOP] %-22s %s\n' "$name" "$pid"
    fi
  done
  say ""
  [ -f "$run/follow_target.csv" ] && \
    say "follow_target.csv: $(wc -l < "$run/follow_target.csv") rows"
  [ -d "$run/bag" ] && \
    say "bag: $(du -sh "$run/bag" 2>/dev/null | awk '{print $1}')"
  [ -f "$run/cpu.csv" ] && \
    say "cpu.csv: $(($(wc -l < "$run/cpu.csv") - 1)) samples"
  say ""
  say "--- recent state transitions ---"
  if [ -f "$run/follow_target.csv" ]; then
    python3 - "$run/follow_target.csv" <<'PY' 2>/dev/null || true
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
if rows:
    t0 = float(rows[0]['t'])
    prev = None
    out = []
    for r in rows:
        if r['kind'] != 'state':
            continue
        if r['note'] == prev:
            continue
        prev = r['note']
        out.append((float(r['t']) - t0, r['note']))
    for t, s in out[-8:]:
        print('  +%7.1f  %s' % (t, s))
PY
  fi
  say ""
  say "--- last cpu sample ---"
  tail -n 1 "$run/cpu.log" 2>/dev/null | sed 's/^/  /' || true
}

cmd_stack() {
  local dir=""
  while [ $# -gt 0 ]; do
    case "$1" in
      --dir) dir="$2"; shift ;;
      -h|--help) usage; exit 0 ;;
      *) die "unknown option: $1" ;;
    esac
    shift
  done
  local run
  run="$(read_run "$dir")"
  ros_env
  collect_stack "$run"
  say "stack.txt updated: $run/stack.txt"
  say ""
  sed -n '1,24p' "$run/stack.txt"
}

cmd_summary() {
  local dir="" pf
  while [ $# -gt 0 ]; do
    case "$1" in
      --dir) dir="$2"; shift ;;
      -h|--help) usage; exit 0 ;;
      *) die "unknown option: $1" ;;
    esac
    shift
  done
  local run
  run="$(read_run "$dir")"
  if [ ! -f "$run/follow_target.csv" ]; then
    die "no follow_target.csv in $run"
  fi
  python3 - "$run/follow_target.csv" "$run/cpu.csv" <<'PY'
import csv, math, sys
from collections import defaultdict
p=sys.argv[1]; cp=sys.argv[2] if len(sys.argv)>2 else ''
rows=list(csv.DictReader(open(p)))
if not rows:
    print('no rows'); raise SystemExit
t0=float(rows[0]['t'])
def rel(t): return float(t)-t0
print('rows %d  span %.1f..%.1f s' % (len(rows), rel(rows[0]['t']), rel(rows[-1]['t'])))
states=[(rel(r['t']),r['note']) for r in rows if r['kind']=='state']
sel=[(rel(r['t']),r['id']) for r in rows if r['kind']=='selected']
print('\nstate transitions:')
prev=None
for t,s in states:
    if s==prev: continue
    print('  +%7.2f  %s' % (t,s))
    prev=s
if sel:
    print('\nselected IDs:')
    pv=None
    for t,i in sel:
        if i==pv: continue
        print('  +%7.2f  id %s' % (t,i))
        pv=i
ids=defaultdict(list)
for r in rows:
    if r['kind']=='person': ids[int(r['id'])].append((rel(r['t']),float(r['z']),float(r['conf'])))
print('\nselected-person tracks (>=5 rows):')
for i,v in sorted(ids.items()):
    if len(v)<5: continue
    ts=[x[0] for x in v]; zs=[x[1] for x in v]
    print('  id %-3d rows %4d  %6.1f..%6.1f s  %5.2f Hz  z %.2f..%.2f' %
          (i,len(v),ts[0],ts[-1],len(v)/max(1e-9,ts[-1]-ts[0]),min(zs),max(zs)))
gs=[(rel(r['t']),float(r['x']),float(r['y'])) for r in rows if r['kind']=='goal']
print('\ngoals: %d' % len(gs))
if len(gs)>1:
    st=[math.hypot(gs[i][1]-gs[i-1][1], gs[i][2]-gs[i-1][2]) for i in range(1,len(gs))]
    dt=[gs[i][0]-gs[i-1][0] for i in range(1,len(gs))]
    print('  step mean %.3f  max %.3f' % (sum(st)/len(st), max(st)))
    print('  dt   median %.2f max %.2f' % (sorted(dt)[len(dt)//2], max(dt)))
if cp:
    try:
        c=list(csv.DictReader(open(cp)))
        v=[float(r['total_busy']) for r in c]
        if v: print('\nCPU: mean %.1f%%  p95 %.1f%%  max %.1f%%' %
                    (sum(v)/len(v), sorted(v)[int(len(v)*.95)], max(v)))
    except Exception as e:
        print('cpu read failed:', e)
PY
}

cmd_list() {
  find "$BASE_D" -maxdepth 1 -type d -name 'run_*' 2>/dev/null | sort | tail -n 30 | sed 's/^/  /'
  [ -f "$POINTER" ] && say "latest: $(cat "$POINTER")"
}

main() {
  local cmd="${1:-}"
  [ $# -gt 0 ] && shift
  case "$cmd" in
    start)   cmd_start "$@" ;;
    stop)    cmd_stop "$@" ;;
    status)  cmd_status "$@" ;;
    summary) cmd_summary "$@" ;;
    stack)   cmd_stack "$@" ;;
    list)    cmd_list ;;
    -h|--help|help|"") usage ;;
    *) die "unknown command: $cmd (use start|stop|status|summary|list)" ;;
  esac
}

main "$@"
