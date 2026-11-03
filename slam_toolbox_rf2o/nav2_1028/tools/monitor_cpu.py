#!/usr/bin/env python3
"""Per-process CPU monitor for the Nav2 board (no external dependencies).

Reads /proc directly so it works on the Jetson/ARM board where tools like
pidstat may not be installed.  CPU percentages are *deltas* over one interval,
not the lifetime average that `ps` reports.

Examples
--------
    ./monitor_cpu.py                          # 2 s interval, run until Ctrl-C
    ./monitor_cpu.py --interval 1 --duration 60
    ./monitor_cpu.py --csv /tmp/nav2_logs/cpu.csv --top 8
    ./monitor_cpu.py --only-ros               # only ROS-related processes

Outputs
-------
* console: one line per sample + the top processes
* <csv>:                t_unix,total_busy,cpu0,cpu1,...
* <csv stem>_procs.csv: t_unix,pid,name,cpu_pct,rss_mb

Notes
-----
The kernel "load average" on this board is dominated by D-state (uninterruptible)
Jetson driver threads and is NOT a useful CPU indicator; use total_busy here.
"""
import argparse
import os
import sys
import time

ROS_HINTS = (
    "rplidar",
    "rf2o",
    "ekf_node",
    "realsense",
    "slam_toolbox",
    "amcl",
    "map_server",
    "nav2",
    "controller_server",
    "planner_server",
    "bt_navigator",
    "behavior_server",
    "smoother_server",
    "velocity_smoother",
    "waypoint_follower",
    "collision_monitor",
    "lifecycle_manager",
    "route_server",
    "docking_server",
    "Mcnamu_driver",
    "record_cmd_vel",
    "ai_human_detect",
)


def read_cpu():
    out = {}
    with open("/proc/stat") as f:
        for line in f:
            if not line.startswith("cpu"):
                continue
            parts = line.split()
            vals = list(map(int, parts[1:]))
            idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
            out[parts[0]] = (sum(vals), idle)
    return out


def read_procs():
    procs = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/stat") as f:
                head, rest = f.read().rsplit(")", 1)
            name = head.split("(", 1)[1]
            fields = rest.split()
            ticks = int(fields[11]) + int(fields[12])  # utime + stime
            rss_kb = 0
            with open(f"/proc/{pid}/status") as f:
                for line in f:
                    if line.startswith("VmRSS:"):
                        rss_kb = int(line.split()[1])
                        break
            procs[pid] = (name, ticks, rss_kb // 1024)
        except (OSError, IndexError, ValueError):
            continue
    return procs


def is_ros(pid, name):
    if any(h.lower() in name.lower() for h in ROS_HINTS):
        return True
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            cmd = f.read().decode("utf-8", "replace").replace("\x00", " ").lower()
    except OSError:
        return False
    return any(h.lower() in cmd for h in ROS_HINTS)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--interval", type=float, default=2.0)
    ap.add_argument("--duration", type=float, default=0.0, help="0 = forever")
    ap.add_argument("--top", type=int, default=8)
    ap.add_argument("--csv", default="", help="summary csv path")
    ap.add_argument("--only-ros", action="store_true")
    ap.add_argument("--quiet", action="store_true", help="csv only")
    args = ap.parse_args()

    hz = os.sysconf("SC_CLK_TCK")
    summary_f = procs_f = None
    if args.csv:
        base = args.csv
        os.makedirs(os.path.dirname(base) or ".", exist_ok=True)
        stem, ext = os.path.splitext(base)
        summary_f = open(base, "w")
        procs_f = open(f"{stem}_procs{ext or '.csv'}", "w")

    s1, p1 = read_cpu(), read_procs()
    t_start = time.time()
    first = True
    try:
        while True:
            time.sleep(args.interval)
            s2, p2 = read_cpu(), read_procs()
            now = time.time()

            tot = s2["cpu"][0] - s1["cpu"][0]
            idle = s2["cpu"][1] - s1["cpu"][1]
            busy = (1.0 - idle / tot) * 100.0 if tot else 0.0
            cores = sorted((k for k in s2 if k != "cpu"), key=lambda k: int(k[3:]))
            core_busy = []
            for c in cores:
                d = s2[c][0] - s1[c][0]
                i = s2[c][1] - s1[c][1]
                core_busy.append((1.0 - i / d) * 100.0 if d else 0.0)

            rows = []
            for pid, (name, ticks, rss) in p2.items():
                if pid not in p1:
                    continue
                dt = ticks - p1[pid][1]
                if dt <= 0:
                    continue
                if args.only_ros and not is_ros(pid, name):
                    continue
                rows.append((dt / hz / args.interval * 100.0, name, int(pid), rss))
            rows.sort(reverse=True)

            if summary_f:
                if first:
                    summary_f.write("t_unix,total_busy," + ",".join(cores) + "\n")
                summary_f.write(
                    f"{now:.3f},{busy:.1f}," + ",".join(f"{v:.1f}" for v in core_busy) + "\n"
                )
                summary_f.flush()
            if procs_f:
                if first:
                    procs_f.write("t_unix,pid,name,cpu_pct,rss_mb\n")
                for pct, name, pid, rss in rows:
                    procs_f.write(f"{now:.3f},{pid},{name},{pct:.1f},{rss}\n")
                procs_f.flush()

            if not args.quiet:
                top = " | ".join(f"{n} {p:.1f}" for p, n, _, _ in rows[:args.top])
                core_txt = " ".join(f"c{i}:{v:4.0f}%" for i, v in enumerate(core_busy))
                print(f"[{now - t_start:6.1f}s] total {busy:5.1f}%  {core_txt}  "
                      f"| {top}", flush=True)

            first = False
            s1, p1 = s2, p2
            if args.duration and now - t_start >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if summary_f:
            summary_f.close()
        if procs_f:
            procs_f.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
