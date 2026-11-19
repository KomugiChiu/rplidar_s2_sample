#!/usr/bin/env python3
"""slam_toolbox mapping + rf2o + Nav2 CPU report (slam map building).

Reads one follow_monitor run directory (cpu.csv / cpu_nodes.csv /
follow_target.csv) and writes a self-contained HTML report in the same style
as report/slam_static_follow_cpu_run_20261119-170751.html.

This run has no /demo/state events: it is a manual / Nav2 map-building
session, so the report uses analysis phases (startup / mapping / late map
update) instead of follow states.

Usage:
  python3 make_slam_map_building_cpu_report.py \
      --run /nvt_ros2_example/slam_toolbox_rf2o/monitor_logs/run_20261119-172648_follow \
      --out slam_map_building_cpu_run_20261119-172648.html
"""
import argparse
import csv
import html
import os
import statistics as st
from collections import defaultdict

BASE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_RUN = "/nvt_ros2_example/slam_toolbox_rf2o/monitor_logs/run_20261119-172648_follow"
DEFAULT_OUT = os.path.join(BASE, "slam_map_building_cpu_run_20261119-172648.html")
NCORE, NOMINAL = 4, 2.0

# Analysis phases (seconds).  There are no ROS state events in this run.
STARTUP_END = 10.0
MAPPING_END = 80.0

LABEL = {
    "/CLaserOdometry2D": "rf2o (CLaserOdometry2D)", "/slam_toolbox": "slam_toolbox",
    "/camera/camera": "camera", "/ai_human_detection": "ai_human_detection",
    "/rplidar_node": "rplidar_node", "/ekf_filter_node": "ekf_node",
    "/controller_server": "controller_server", "/planner_server": "planner_server",
    "/bt_navigator": "bt_navigator", "/collision_monitor": "collision_monitor",
    "/velocity_smoother": "velocity_smoother", "/behavior_server": "behavior_server",
    "/driver_node": "Mcnamu_driver", "/demo_manager_node": "demo_manager_node",
    "/person_target_recovery_node": "person_target_recovery",
    "/keepout_container": "keepout_container", "/smoother_server": "smoother_server",
    "/route_server": "route_server", "/docking_server": "docking_server",
    "/waypoint_follower": "waypoint_follower",
    "/lifecycle_manager_navigation": "lifecycle_manager",
    "/base_to_ai_human_detection_tf": "ai TF publisher",
    "process:static_transfor": "static TF", "process:python3": "python3 (bag/watch/monitor)",
    "process:ros2": "ros2 launch", "process:grep": "grep", "process:tail": "tail",
}

# 17:07 static follow (same board, AI + demo_manager on).
STATIC = {
    "duration": 114.1, "total": 54.6, "p95": 69.2, "max": 76.4,
    "slam": 6.53, "slam_max": 25.35, "rf2o": 1.74, "rf2o_max": 3.35,
    "camera": 10.8, "ai": 6.8, "path": 16.97,
    "cores": [(57.6, 86.7), (51.7, 99.1), (52.4, 95.2), (55.7, 95.0)],
}
# 16:42 dynamic mapping + follow.
DYN = {
    "duration": 81.4, "total": 59.8, "p95": 89.8, "max": 96.5,
    "slam": 8.9, "slam_max": 35.3, "rf2o": 1.7, "rf2o_max": 2.7,
    "camera": 11.0, "ai": 6.6, "path": 16.81,
    "cores": [(62.7, 92.1), (56.5, 100.0), (59.5, 96.7), (60.2, 99.5)],
}
# 15:45 RTAB-Map + ICP.
RTAB = {
    "duration": 58.9, "total": 72.1, "p95": 94.5, "max": 95.1,
    "icp": 13.4, "icp_max": 19.0, "rtab": 4.5, "rtab_max": 9.0,
    "camera": 11.3, "ai": 6.6, "path": 11.18,
    "cores": [(73.2, 91.2), (71.3, 100.0), (71.4, 99.6), (72.3, 99.6)],
}
STATIC_BY_NODE = {
    "camera": STATIC["camera"], "slam_toolbox": STATIC["slam"],
    "python3 (bag/watch/monitor)": 6.78, "ekf_node": 3.64, "rplidar_node": 3.64,
    "ros2 launch": 3.10, "controller_server": 2.25, "Mcnamu_driver": 1.81,
    "collision_monitor": 1.80, "rf2o (CLaserOdometry2D)": STATIC["rf2o"],
    "bt_navigator": 1.36, "demo_manager_node": 0.96, "planner_server": 0.89,
    "velocity_smoother": 0.55, "docking_server": 0.47, "behavior_server": 0.45,
    "static TF": 0.38, "route_server": 0.38, "smoother_server": 0.35,
    "person_target_recovery": 0.33,
}


def read(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def q(vals, p):
    vals = sorted(vals)
    if not vals:
        return 0.0
    return vals[max(0, min(len(vals) - 1, int(round(p / 100 * (len(vals) - 1)))))]


def load(run_dir):
    cpu = read(os.path.join(run_dir, "cpu.csv"))
    nodes = read(os.path.join(run_dir, "cpu_nodes.csv"))
    ft = read(os.path.join(run_dir, "follow_target.csv"))
    ts = [float(r["t_unix"]) for r in cpu]
    gaps = [NOMINAL] + [b - a for a, b in zip(ts, ts[1:])]
    fac = [NOMINAL / g for g in gaps]
    t0 = ts[0]
    series, rss = defaultdict(dict), defaultdict(dict)
    for r in nodes:
        i = ts.index(float(r["t_unix"]))
        key = float(r["t_unix"])
        series[r["node"]][key] = series[r["node"]].get(key, 0.0) + \
            float(r["cpu_pct"]) * fac[i] / NCORE
        rss[r["node"]][key] = max(rss[r["node"]].get(key, 0.0), float(r["rss_mb"]))
    stats = [{"node": n, "label": LABEL.get(n, n),
              "avg": st.mean([s.get(float(r["t_unix"]), 0.0) for r in cpu]),
              "max": max(s.values()),
              "rss": st.mean(list(rss[n].values()))}
             for n, s in series.items()]
    stats.sort(key=lambda s: -s["avg"])
    rob = [(float(r["t"]) - t0, float(r["x"]), float(r["y"]))
           for r in ft if r["kind"] == "robot"]
    path = sum(((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5
               for a, b in zip(rob, rob[1:]))
    net = (((rob[-1][1] - rob[0][1]) ** 2 + (rob[-1][2] - rob[0][2]) ** 2) ** 0.5) if rob else 0.0
    cmds = [(float(r["t"]) - t0, abs(float(r["vx"])), abs(float(r["wz"])))
            for r in ft if r["kind"] == "cmd"]
    return {
        "run": os.path.basename(os.path.normpath(run_dir)),
        "elapsed": [t - t0 for t in ts],
        "busy": [float(r["total_busy"]) for r in cpu],
        "cores": {c: [float(r[c]) for r in cpu] for c in ("cpu0", "cpu1", "cpu2", "cpu3")},
        "stats": stats, "duration": ts[-1] - t0, "n": len(cpu), "t0": t0,
        "path": path, "net": net,
        "maxvx": max((c[1] for c in cmds), default=0.0),
        "maxwz": max((c[2] for c in cmds), default=0.0),
        "series": series,
    }


def phase(run, a, b):
    v = [x for t, x in zip(run["elapsed"], run["busy"]) if a <= t < b]
    return (st.mean(v), q(v, 95), max(v), len(v)) if v else (0.0, 0.0, 0.0, 0)


def esc(s):
    return html.escape(str(s))


def bar(avg, mx, scale):
    return (f'<div class="bar-track"><div class="bar-fill" style="width:{min(100, avg / scale * 100):.1f}%"></div>'
            f'<div class="bar-max" style="left:{min(100, mx / scale * 100):.1f}%"></div></div>')


def node_peak(M, node):
    s = M["series"][node]
    return max(s.items(), key=lambda kv: kv[1])


def build(run_dir, out):
    M = load(run_dir)
    by = {s["node"]: s for s in M["stats"]}
    allp = phase(M, 0.0, M["duration"] + 1)
    start = phase(M, 0.0, STARTUP_END)
    map_p = phase(M, STARTUP_END, MAPPING_END)
    late = phase(M, MAPPING_END, M["duration"] + 1)
    slam = by["/slam_toolbox"]
    rf2o = by["/CLaserOdometry2D"]
    grp_slam = slam["avg"] + rf2o["avg"]
    grp_cam = by["/camera/camera"]["avg"]
    grp_lidar = by["/rplidar_node"]["avg"] + by["/ekf_filter_node"]["avg"]
    nav_excl = {"/CLaserOdometry2D", "/slam_toolbox", "/camera/camera", "/ai_human_detection",
                "/rplidar_node", "/ekf_filter_node", "/demo_manager_node",
                "/person_target_recovery_node", "/base_to_ai_human_detection_tf",
                "process:python3", "process:ros2", "process:static_transfor"}
    grp_nav = sum(s["avg"] for s in M["stats"] if s["node"] not in nav_excl)
    grp_measure = (by.get("process:python3", {}).get("avg", 0.0)
                   + by.get("process:ros2", {}).get("avg", 0.0)
                   + by.get("process:static_transfor", {}).get("avg", 0.0))
    global_peak = max(zip(M["elapsed"], M["busy"]), key=lambda x: x[1])
    slam_t, slam_v = node_peak(M, "/slam_toolbox")
    slam_t -= M["t0"]
    late_peak = max(((t, v) for t, v in zip(M["elapsed"], M["busy"])
                     if t >= MAPPING_END), key=lambda x: x[1])
    scale = max(6.0, M["stats"][0]["avg"] * 1.15)

    h = []
    w = h.append
    w(f"""<!DOCTYPE html>
<html lang="zh-Hant">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>slam_toolbox 建圖 + rf2o + Nav2 CPU 分析 — {M['run']}</title>
<style>
  :root {{ --bg:#f6f7fb; --card:#fff; --ink:#1f2430; --muted:#5b6472; --line:#e3e6ee;
    --accent:#3a6df0; --ai:#2a9d8f; --cam:#6a5acd; --warn:#b26a00; --warn-bg:#fff4e0;
    --bad:#c0392b; --good:#2a9d8f; --code:#f1f3f8; }}
  * {{ box-sizing:border-box; }}
  body {{ margin:0; background:var(--bg); color:var(--ink);
    font:16px/1.65 "Inter","Noto Sans TC","PingFang TC","Microsoft JhengHei",system-ui,sans-serif; }}
  .wrap {{ max-width:1120px; margin:0 auto; padding:36px 22px 72px; }}
  header.hero {{ background:linear-gradient(135deg,#1f2430 0%,#2b3a55 58%,#35507a 100%);
    color:#fff; border-radius:18px; padding:34px 36px 30px; box-shadow:0 12px 32px rgba(31,36,48,.18); }}
  header.hero h1 {{ margin:0 0 8px; font-size:30px; letter-spacing:.4px; }}
  header.hero p {{ margin:0; color:#c9d3e6; font-size:15px; }}
  .pills {{ margin-top:16px; display:flex; flex-wrap:wrap; gap:8px; }}
  .pill {{ display:inline-block; padding:5px 12px; border-radius:999px; font-size:12.5px;
    letter-spacing:.3px; background:rgba(255,255,255,.14); color:#e8eefb; }}
  .pill.good {{ background:rgba(42,157,143,.28); color:#c8f2ea; }}
  .pill.warn {{ background:rgba(217,164,65,.28); color:#ffe6b8; }}
  h2 {{ font-size:21px; margin:44px 0 12px; padding-bottom:8px; border-bottom:2px solid var(--line); }}
  h3 {{ font-size:16.5px; margin:24px 0 8px; }}
  p.lead {{ color:var(--muted); margin-top:0; }}
  .kpis {{ display:grid; gap:14px; margin-top:22px;
    grid-template-columns:repeat(auto-fit,minmax(190px,1fr)); }}
  .kpi {{ background:var(--card); border:1px solid var(--line); border-radius:14px;
    padding:16px 18px; box-shadow:0 2px 8px rgba(31,36,48,.04); border-top:4px solid var(--accent); }}
  .kpi.warn {{ border-top-color:var(--warn); }}
  .kpi.good {{ border-top-color:var(--good); }}
  .kpi .label {{ font-size:12.5px; letter-spacing:.4px; color:var(--muted); text-transform:uppercase; }}
  .kpi .value {{ font-size:30px; font-weight:700; line-height:1.15; margin:4px 0 2px; font-variant-numeric:tabular-nums; }}
  .kpi .sub {{ font-size:12.5px; color:var(--muted); }}
  .card {{ background:var(--card); border:1px solid var(--line); border-radius:14px;
    padding:20px 22px; box-shadow:0 2px 8px rgba(31,36,48,.04); }}
  .callout {{ border-left:4px solid var(--warn); background:var(--warn-bg);
    border-radius:0 10px 10px 0; padding:14px 18px; margin:18px 0; color:#5a4300; font-size:15px; }}
  .callout.good {{ border-left-color:var(--good); background:#eaf7f4; color:#14544b; }}
  .flow {{ background:var(--code); border-radius:10px; padding:14px 16px;
    font:13.5px/1.7 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;
    white-space:pre; overflow-x:auto; color:#2b3445; }}
  table {{ width:100%; border-collapse:collapse; margin:14px 0 6px; background:var(--card);
    border-radius:12px; overflow:hidden; box-shadow:0 2px 8px rgba(31,36,48,.04); font-size:14.5px; }}
  th,td {{ padding:10px 14px; text-align:left; border-bottom:1px solid var(--line); vertical-align:middle; }}
  thead th {{ background:#eef1f8; font-size:13px; letter-spacing:.3px; }}
  tbody tr:nth-child(even) {{ background:#fafbfe; }}
  tbody tr:last-child td {{ border-bottom:none; }}
  td.num {{ font-variant-numeric:tabular-nums; text-align:right; white-space:nowrap; }}
  .bars {{ min-width:170px; }}
  .bar-track {{ position:relative; height:12px; background:#eef1f8; border-radius:6px; overflow:hidden; }}
  .bar-fill {{ position:absolute; left:0; top:0; bottom:0; background:var(--accent); border-radius:6px; }}
  .bar-max {{ position:absolute; top:-2px; bottom:-2px; width:2px; background:rgba(192,57,43,.75); }}
  .legend {{ display:flex; flex-wrap:wrap; gap:16px; font-size:13px; color:var(--muted); margin-top:10px; }}
  .legend i {{ display:inline-block; width:12px; height:12px; border-radius:3px; margin-right:6px; vertical-align:-1px; }}
  .timeline {{ margin:16px 0 6px; }}
  .tl-track {{ display:flex; height:42px; border-radius:10px; overflow:hidden; border:1px solid var(--line); }}
  .tl-seg {{ display:flex; align-items:center; justify-content:center; font-size:11.5px; color:#fff;
    white-space:nowrap; overflow:hidden; }}
  .tl-idle {{ background:#9aa4b6; }} .tl-sel {{ background:#d9a441; }}
  .tl-follow {{ background:var(--accent); }} .tl-home {{ background:#2a9d8f; }}
  .tl-stop {{ background:#6b7484; }}
  .tl-labels {{ display:flex; font-size:12px; color:var(--muted); margin-top:6px; font-variant-numeric:tabular-nums; }}
  .grid2 {{ display:grid; gap:18px; grid-template-columns:repeat(auto-fit,minmax(340px,1fr)); }}
  .chart-box {{ background:var(--card); border:1px solid var(--line); border-radius:14px;
    padding:16px 16px 8px; box-shadow:0 2px 8px rgba(31,36,48,.04); overflow-x:auto; }}
  .chart-box svg {{ display:block; width:100%; height:auto; min-width:640px; }}
  .small {{ font-size:13px; color:var(--muted); }}
  ul.tight {{ margin:8px 0 0; padding-left:22px; }} ul.tight li {{ margin:5px 0; }}
  code {{ background:var(--code); border-radius:5px; padding:1px 6px; font-size:13.5px; }}
  .good {{ color:var(--good); font-weight:600; }}
  footer {{ margin-top:52px; color:var(--muted); font-size:13px; border-top:1px solid var(--line); padding-top:18px; }}
  @media print {{ body {{ background:#fff; }} .wrap {{ max-width:none; padding:0; }}
    header.hero,.card,table,.chart-box {{ box-shadow:none; }}
    h2 {{ page-break-after:avoid; }} table,.chart-box {{ page-break-inside:avoid; }} }}
</style>
</head>
<body>
<div class="wrap">

<header class="hero">
  <h1>slam_toolbox 建圖 + rf2o + Nav2 CPU 分析（slam map building）</h1>
  <p>{M['run']} · 2026-11-19 17:26:55 · {M['duration']:.1f} s · online_async mapping（static map building）· lite bag</p>
  <div class="pills">
    <span class="pill">slam_toolbox async mapping + rf2o</span>
    <span class="pill">Nav2 + 底盤 · 無 AI / demo_manager</span>
    <span class="pill">4 核 · node CPU 已換算四核佔比</span>
    <span class="pill good">整段 mean {allp[0]:.1f}% / p95 {allp[1]:.1f}%</span>
    <span class="pill warn">整段 max {allp[2]:.1f}%（啟動）</span>
  </div>
</header>

<div class="kpis">
  <div class="kpi good">
    <div class="label">整段 mean</div>
    <div class="value">{allp[0]:.1f}%</div>
    <div class="sub">0 ~ {M['duration']:.1f} s（{allp[3]} 筆）</div>
  </div>
  <div class="kpi good">
    <div class="label">整段 p95</div>
    <div class="value">{allp[1]:.1f}%</div>
    <div class="sub">max {allp[2]:.1f}%；建圖段 p95 {map_p[1]:.1f}%、後段 p95 {late[1]:.1f}%</div>
  </div>
  <div class="kpi">
    <div class="label">slam_toolbox（四核）</div>
    <div class="value">{slam['avg']:.1f}%</div>
    <div class="sub">max {slam['max']:.1f}%（+{slam_t:.1f} s）；本次最大節點</div>
  </div>
  <div class="kpi">
    <div class="label">rf2o（四核）</div>
    <div class="value">{rf2o['avg']:.1f}%</div>
    <div class="sub">max {rf2o['max']:.1f}%；odom 不是瓶頸</div>
  </div>
</div>
<p class="small" style="margin-top:12px">total_busy 為四核整體使用率（0~100%）；node / process 已由單核換算四核佔比（單核 ÷ 4，1 核滿載 = 25%），並以 <code>名目 2.0 s ÷ 實際間隔</code> 修正。這次是 <strong>mapping（建立 static map）</strong>，沒有 AI、demo_manager 或 follow 狀態。</p>

<h2>1. Run 概況</h2>
<div class="grid2">
  <div class="card">
    <h3 style="margin-top:0">分析分段（不是 ROS 狀態）</h3>
    <div class="timeline">
      <div class="tl-track">
        <div class="tl-seg tl-idle" style="width:{STARTUP_END/M['duration']*100:.1f}%">STARTUP</div>
        <div class="tl-seg tl-follow" style="width:{(MAPPING_END-STARTUP_END)/M['duration']*100:.1f}%">MAPPING</div>
        <div class="tl-seg tl-sel" style="width:{(M['duration']-MAPPING_END)/M['duration']*100:.1f}%">LATE MAP UPDATE</div>
      </div>
      <div class="tl-labels">
        <span style="width:{STARTUP_END/M['duration']*100:.1f}%;text-align:left">0 s</span>
        <span style="width:{(MAPPING_END-STARTUP_END)/M['duration']*100:.1f}%;text-align:center">{STARTUP_END:.1f} → {MAPPING_END:.1f} s</span>
        <span style="width:{(M['duration']-MAPPING_END)/M['duration']*100:.1f}%;text-align:right">{M['duration']:.1f} s</span>
      </div>
    </div>
    <div class="flow" style="margin-top:14px">0 ~ {STARTUP_END:.1f} s   bag / monitor / DDS discovery
{STARTUP_END:.1f} ~ {MAPPING_END:.1f} s   穩定建圖（主要行駛段）
{MAPPING_END:.1f} ~ {M['duration']:.1f} s   後段 map update / loop closure 尖峰</div>
    <p class="small" style="margin-bottom:0">`follow_target.csv` 沒有 <code>state</code> 或 <code>goal</code> 筆；本次是手動 / Nav2 駕駛建圖，所以用時間分段分析。</p>
  </div>
  <div class="card">
    <h3 style="margin-top:0">建圖統計</h3>
    <table style="margin-top:4px;box-shadow:none">
      <tbody>
        <tr><td>錄製時間</td><td class="num">{M['duration']:.1f} s</td></tr>
        <tr><td>robot path / net</td><td class="num">{M['path']:.2f} m / {M['net']:.2f} m</td></tr>
        <tr><td>max |vx| / |wz|</td><td class="num">{M['maxvx']:.2f} m/s / {M['maxwz']:.2f} rad/s</td></tr>
        <tr><td>Nav2 global costmap</td><td class="num">203 × 332 @ 0.05 m → 10.15 × 16.60 m</td></tr>
        <tr><td>AI / demo_manager</td><td class="num">off（本次沒有啟動）</td></tr>
      </tbody>
    </table>
  </div>
</div>

<h2>2. 整體 CPU</h2>
<div class="chart-box">
  <svg id="cpuChart" viewBox="0 0 900 350" role="img" aria-label="map building CPU 使用率時間序列"></svg>
  <div class="legend">
    <span><i style="background:#3a6df0"></i>total_busy（2 s 取樣）</span>
    <span><i style="background:rgba(58,109,240,.10)"></i>MAPPING {STARTUP_END:.1f} ~ {MAPPING_END:.1f} s</span>
    <span><i style="background:rgba(217,164,65,.12)"></i>LATE MAP UPDATE {MAPPING_END:.1f} ~ {M['duration']:.1f} s</span>
    <span><i style="background:#c0392b"></i>mapping p95 {map_p[1]:.1f}%</span>
  </div>
</div>

<table>
  <thead><tr><th>區間</th><th class="num">筆數</th><th class="num">mean</th><th class="num">p95</th><th class="num">max</th></tr></thead>
  <tbody>
    <tr><td>整段 0 ~ {M['duration']:.1f} s</td><td class="num">{allp[3]}</td><td class="num">{allp[0]:.1f}%</td><td class="num">{allp[1]:.1f}%</td><td class="num">{allp[2]:.1f}%</td></tr>
    <tr><td>啟動 0 ~ {STARTUP_END:.1f} s</td><td class="num">{start[3]}</td><td class="num">{start[0]:.1f}%</td><td class="num">{start[1]:.1f}%</td><td class="num">{start[2]:.1f}%</td></tr>
    <tr><td>建圖 {STARTUP_END:.1f} ~ {MAPPING_END:.1f} s</td><td class="num">{map_p[3]}</td><td class="num">{map_p[0]:.1f}%</td><td class="num"><strong>{map_p[1]:.1f}%</strong></td><td class="num">{map_p[2]:.1f}%</td></tr>
    <tr><td>後段 {MAPPING_END:.1f} ~ {M['duration']:.1f} s</td><td class="num">{late[3]}</td><td class="num">{late[0]:.1f}%</td><td class="num">{late[1]:.1f}%</td><td class="num">{late[2]:.1f}%</td></tr>
  </tbody>
</table>
<p class="small">整段最高 {global_peak[1]:.1f}% 在 +{global_peak[0]:.1f} s（bag / monitor 啟動與 DDS discovery）；建圖主段很穩（mean {map_p[0]:.1f}%、p95 {map_p[1]:.1f}%）；後段在 +{late_peak[0]:.1f} s 出現 {late_peak[1]:.1f}%，對應 slam_toolbox 的 {slam_v:.1f}%（四核）尖峰，較可能是 map update / loop closure。</p>

<h2>3. Per-core</h2>
<p class="lead">本次整段每核 mean {min(st.mean(v) for v in M['cores'].values()):.1f}~{max(st.mean(v) for v in M['cores'].values()):.1f}%，單核 max {min(max(v) for v in M['cores'].values()):.1f}~{max(max(v) for v in M['cores'].values()):.1f}%。</p>
<table>
  <thead><tr><th>Core</th><th class="num">本次 mean</th><th class="num">本次 max</th><th class="num">17:07 static follow</th><th class="num">16:42 動態建圖+follow</th><th class="num">15:45 RTAB-Map</th><th class="bars">本次 mean（每核 0~100%）</th></tr></thead>
  <tbody>
""")
    for i, (c, v) in enumerate(M["cores"].items()):
        w(f'<tr><td>{c}</td><td class="num">{st.mean(v):.1f}%</td><td class="num">{max(v):.1f}%</td>'
          f'<td class="num">{STATIC["cores"][i][0]:.1f}% / {STATIC["cores"][i][1]:.1f}%</td>'
          f'<td class="num">{DYN["cores"][i][0]:.1f}% / {DYN["cores"][i][1]:.1f}%</td>'
          f'<td class="num">{RTAB["cores"][i][0]:.1f}% / {RTAB["cores"][i][1]:.1f}%</td>'
          f'<td class="bars">{bar(st.mean(v), max(v), 100)}</td></tr>')
    w(f"""  </tbody>
</table>
<p class="small">建圖 run 每核 mean 約 35~38%，明顯低於 static follow（52~58%）、動態建圖+follow（56~63%）與 RTAB-Map（71~73%），主要因為這次沒有 AI / demo_manager / follow。單核 max 仍在啟動段碰到 81.6%。</p>

<h2>4. 建圖 vs follow vs RTAB-Map</h2>
<p class="lead">這不是嚴格 A/B：本次是 <strong>mapping only（無 AI / demo）</strong>，其餘三筆都有 AI + demo_manager（follow）。重點是看「建圖本身」的 SLAM 成本，以及加上 AI / follow 後增加的幅度。</p>
<table>
  <thead><tr><th>指標（四核佔比）</th><th class="num">17:26 建圖（本次）</th><th class="num">17:07 static follow</th><th class="num">16:42 動態建圖+follow</th><th class="num">15:45 RTAB-Map + ICP</th></tr></thead>
  <tbody>
    <tr><td>雷射 odom</td><td class="num">rf2o <strong>{rf2o['avg']:.1f}%</strong>（max {rf2o['max']:.1f}）</td><td class="num">rf2o {STATIC['rf2o']:.1f}%</td><td class="num">rf2o {DYN['rf2o']:.1f}%</td><td class="num">ICP <strong>{RTAB['icp']:.1f}%</strong>（max {RTAB['icp_max']:.1f}）</td></tr>
    <tr><td>SLAM 節點</td><td class="num">slam_toolbox <strong>{slam['avg']:.1f}%</strong>（max {slam['max']:.1f}）</td><td class="num">slam_toolbox {STATIC['slam']:.1f}%（max {STATIC['slam_max']:.1f}）</td><td class="num">slam_toolbox {DYN['slam']:.1f}%（max {DYN['slam_max']:.1f}）</td><td class="num">rtabmap {RTAB['rtab']:.1f}%（max {RTAB['rtab_max']:.1f}）</td></tr>
    <tr><td>odom + SLAM 合計</td><td class="num"><strong>{grp_slam:.1f}%</strong></td><td class="num">{STATIC['rf2o']+STATIC['slam']:.1f}%</td><td class="num">{DYN['rf2o']+DYN['slam']:.1f}%</td><td class="num">{RTAB['icp']+RTAB['rtab']:.1f}%</td></tr>
    <tr><td>camera / AI</td><td class="num">{grp_cam:.1f}% / —</td><td class="num">{STATIC['camera']:.1f}% / {STATIC['ai']:.1f}%</td><td class="num">{DYN['camera']:.1f}% / {DYN['ai']:.1f}%</td><td class="num">{RTAB['camera']:.1f}% / {RTAB['ai']:.1f}%</td></tr>
    <tr><td>整段總 CPU mean</td><td class="num"><strong>{allp[0]:.1f}%</strong></td><td class="num">{STATIC['total']:.1f}%</td><td class="num">{DYN['total']:.1f}%</td><td class="num">{RTAB['total']:.1f}%</td></tr>
    <tr><td>整段 p95 / max</td><td class="num">{allp[1]:.1f} / {allp[2]:.1f}%</td><td class="num">{STATIC['p95']:.1f} / {STATIC['max']:.1f}%</td><td class="num">{DYN['p95']:.1f} / {DYN['max']:.1f}%</td><td class="num">{RTAB['p95']:.1f} / {RTAB['max']:.1f}%</td></tr>
    <tr><td>robot path</td><td class="num">{M['path']:.2f} m</td><td class="num">{STATIC['path']:.2f} m</td><td class="num">{DYN['path']:.2f} m</td><td class="num">{RTAB['path']:.2f} m</td></tr>
    <tr><td>run 長度</td><td class="num">{M['duration']:.1f} s</td><td class="num">{STATIC['duration']:.1f} s</td><td class="num">{DYN['duration']:.1f} s</td><td class="num">{RTAB['duration']:.1f} s</td></tr>
  </tbody>
</table>
<div class="callout good">
  <strong>建圖本身只要 {allp[0]:.1f}%（p95 {allp[1]:.1f}%），是這幾種流程裡最低的。</strong>
  SLAM 節點（slam_toolbox {slam['avg']:.1f}% + rf2o {rf2o['avg']:.1f}%）合計 {grp_slam:.1f}%；最大的固定成本是 camera {grp_cam:.1f}% 與 Nav2 / 底盤 {grp_nav:.1f}%。
</div>
<div class="callout">
  <strong>與 follow 的差距主要來自 AI + demo_manager。</strong>static follow 多了 AI {STATIC['ai']:.1f}% + demo_manager 約 1%，總 CPU 從 {allp[0]:.1f}% 拉到 {STATIC['total']:.1f}%；動態建圖+follow 則是 slam_toolbox 本身也較忙（{DYN['slam']:.1f}%），總 CPU {DYN['total']:.1f}%。
</div>

<h2>5. Node CPU — 整段（四核佔比）</h2>
<p class="lead">odom + SLAM 約 {grp_slam:.1f}%（rf2o + slam_toolbox）；camera {grp_cam:.1f}%；Nav2 + 底盤約 {grp_nav:.1f}%；光達 + EKF 約 {grp_lidar:.1f}%；量測本身（python3 + ros2 + static TF）約 {grp_measure:.1f}%。</p>
<div class="grid2" style="margin-bottom:16px">
  <div class="card" style="border-top:4px solid var(--accent)">
    <div class="label small">odom + SLAM</div>
    <div style="font-size:30px;font-weight:700;font-variant-numeric:tabular-nums">約 {grp_slam:.1f}%</div>
    <div class="small">rf2o {rf2o['avg']:.1f} + slam_toolbox {slam['avg']:.1f}（max {slam['max']:.1f}）</div>
  </div>
  <div class="card" style="border-top:4px solid var(--ai)">
    <div class="label small">camera（EKF 用）</div>
    <div style="font-size:30px;font-weight:700;font-variant-numeric:tabular-nums">約 {grp_cam:.1f}%</div>
    <div class="small">RealSense；本次沒有 AI / demo_manager</div>
  </div>
  <div class="card" style="border-top:4px solid #6b7484">
    <div class="label small">Nav2 + 底盤</div>
    <div style="font-size:30px;font-weight:700;font-variant-numeric:tabular-nums">約 {grp_nav:.1f}%</div>
    <div class="small">controller / planner / BT / collision / driver …</div>
  </div>
</div>
<table>
  <thead><tr><th>Node</th><th class="num">mean</th><th class="num">max</th><th class="num">RSS</th><th class="num">17:07 static follow</th><th class="bars">mean 分布（0~{scale:.0f}%）</th></tr></thead>
  <tbody>
""")
    for s in M["stats"]:
        if s["avg"] < 0.3:
            continue
        n = s["node"]
        if (n in ("process:grep", "process:tail", "process:systemd-journal")
                or "kworker" in n or n.startswith(("process:irq/", "process:rcu", "process:ksoftirqd"))
                or n.startswith(("process:hdmi", "process:wifi", "process:avahi", "process:containerd",
                                 "process:dbus", "process:dhclient", "process:polkitd", "process:sshd",
                                 "process:threadmon", "process:log_notify", "process:_NMP", "process:NMP",
                                 "process:nvt_", "process:ise", "process:lme", "process:md_",
                                 "process:trke", "process:dre", "process:graph", "process:ive",
                                 "process:kflow", "process:sde"))):
            continue
        old = STATIC_BY_NODE.get(s["label"])
        w(f'<tr><td><code>{esc(s["label"])}</code></td><td class="num"><strong>{s["avg"]:.1f}%</strong></td>'
          f'<td class="num">{s["max"]:.1f}%</td><td class="num">{s["rss"]:.0f} MB</td>'
          f'<td class="num">{f"{old:.1f}%" if old is not None else "—"}</td>'
          f'<td class="bars">{bar(s["avg"], s["max"], scale)}</td></tr>')
    w(f"""  </tbody>
</table>
<p class="small">長條為整段 mean、紅線為 max；「17:07 static follow」是同名節點在 static follow run 的整段 mean（該 run 有 AI / demo_manager）。本次沒有 <code>ai_human_detection</code>、<code>demo_manager_node</code>、<code>person_target_recovery</code>。</p>

<h2>6. 觀察與建議</h2>
<ul class="tight">
  <li><strong>建圖成本集中在 slam_toolbox。</strong>整段 {slam['avg']:.1f}%、max {slam['max']:.1f}%（+{slam_t:.1f} s）；後段 +{late_peak[0]:.1f} s 的總 CPU {late_peak[1]:.1f}% 與 slam_toolbox {slam_v:.1f}% 同步，較可能是 map update / loop closure（<code>do_loop_closing: true</code>、<code>map_update_interval: 5.0</code>）。若地圖品質正常，這是預期行為；若同時出現 scan drop 或 map→odom 延遲，再調 <code>loop_match_*</code> / <code>map_update_interval</code>。</li>
  <li><strong>rf2o 很便宜。</strong>{rf2o['avg']:.1f}%（max {rf2o['max']:.1f}%）；與 17:07（1.7%）、16:42（1.7%）一致，odom 不是建圖瓶頸。</li>
  <li><strong>camera 是第二大固定成本（{grp_cam:.1f}%）。</strong>它是 EKF 的 RealSense；若建圖不需要視覺/IMU，關掉可省下大部分，但 EKF 的 odom 融合也會受影響。</li>
  <li><strong>量測負擔約 {grp_measure:.1f}%。</strong>python3（bag / follow_watch / monitor）{by.get('process:python3', {}).get('avg', 0):.1f}% + ros2 launch {by.get('process:ros2', {}).get('avg', 0):.1f}%；要更乾淨的基準可用 <code>--no-bag --no-watch</code> 再跑一次。</li>
  <li><strong>啟動尖峰與建圖無關。</strong>整段 max {global_peak[1]:.1f}% 在 +{global_peak[0]:.1f} s（bag/monitor/DDS discovery）；建圖主段 p95 只有 {map_p[1]:.1f}%。</li>
  <li><strong>scan drop 集中在啟動。</strong><code>/tmp/slam_toolbox_only/logs/slam.log</code> 共 9 次 <code>Message Filter dropping</code>，大多在 17:25:56~17:26:12（啟動/前段），後段 map update 尖峰期間沒有再出現。</li>
  <li><strong>地圖尚未存檔。</strong><code>/nvt_ros2_example/maps</code> 最新仍是 11-18 的 <code>slam_map_20261118-0959.*</code>；本次只建圖、沒有新的 <code>.posegraph</code> / <code>.pgm</code>。要保留這次地圖，請跑 <code>record_map.sh</code> / <code>save_map.sh</code>（或 slam_toolbox 的 map_saver）。</li>
</ul>

<h2>7. 紀錄檔</h2>
<div class="flow">/nvt_ros2_example/slam_toolbox_rf2o/monitor_logs/{M['run']}/
  cpu.csv / cpu_nodes.csv / cpu_procs.csv
  follow_target.csv   0 person 筆 · 0 goals（本次是手動建圖）
  RUN_INFO.txt, .env.sh, bag/
slam_toolbox log: /tmp/slam_toolbox_only/logs/slam.log
  （9 次 queue-full drop，集中在啟動/前段）
Nav2 log: /tmp/raspybot_nav2/logs/nav2.log
  （global costmap 203 × 332 @ 0.05 m = 10.15 × 16.60 m）
對照：17:07 static follow · 16:42 動態建圖+follow · 15:45 RTAB-Map + ICP</div>

<footer>
  slam_toolbox map building + rf2o + Nav2 CPU 分析 · {M['run']} · 2026-11-19 ·
  資料來源 <code>cpu.csv</code> / <code>cpu_nodes.csv</code> / <code>follow_target.csv</code> · monitor_cpu.py（--interval 2）
</footer>

</div>
<script>
(function () {{
  var data = {[[round(x, 1), round(v, 1)] for x, v in zip(M['elapsed'], M['busy'])]};
  var startupEnd = {STARTUP_END:.3f}, mappingEnd = {MAPPING_END:.3f}, runEnd = {M['duration']:.3f};
  var p95 = {map_p[1]:.1f}, peakT = {global_peak[0]:.1f}, peakV = {global_peak[1]:.1f};
  var lateT = {late_peak[0]:.1f}, lateV = {late_peak[1]:.1f};
  var W = 900, H = 350, m = {{ l: 54, r: 24, t: 24, b: 42 }};
  var iw = W - m.l - m.r, ih = H - m.t - m.b;
  var xMax = {int(M['duration'] // 10 * 10 + 10)}, yMax = 100;
  function X(t) {{ return m.l + t / xMax * iw; }}
  function Y(v) {{ return m.t + (1 - v / yMax) * ih; }}
  var NS = "http://www.w3.org/2000/svg", svg = document.getElementById("cpuChart");
  function el(name, attrs, text) {{
    var e = document.createElementNS(NS, name);
    for (var k in attrs) e.setAttribute(k, attrs[k]);
    if (text != null) e.textContent = text;
    svg.appendChild(e); return e;
  }}
  el("rect", {{ x: X(startupEnd), y: m.t, width: X(mappingEnd) - X(startupEnd), height: ih, fill: "rgba(58,109,240,.10)" }});
  el("text", {{ x: (X(startupEnd) + X(mappingEnd)) / 2, y: m.t + 14, "text-anchor": "middle",
    "font-size": "11.5", fill: "#3a6df0", "font-weight": "600", "letter-spacing": ".6" }}, "MAPPING");
  el("rect", {{ x: X(mappingEnd), y: m.t, width: X(runEnd) - X(mappingEnd), height: ih, fill: "rgba(217,164,65,.12)" }});
  el("text", {{ x: (X(mappingEnd) + X(runEnd)) / 2, y: m.t + 14, "text-anchor": "middle",
    "font-size": "11.5", fill: "#9a6a00", "font-weight": "600", "letter-spacing": ".6" }}, "LATE MAP UPDATE");
  [0, 25, 50, 75, 100].forEach(function (v) {{
    el("line", {{ x1: m.l, y1: Y(v), x2: m.l + iw, y2: Y(v), stroke: "#e3e6ee", "stroke-width": v === 0 ? "1.4" : "1" }});
    el("text", {{ x: m.l - 9, y: Y(v) + 4, "text-anchor": "end", "font-size": "11", fill: "#7b8494" }}, v + "%");
  }});
  for (var t = 0; t <= xMax; t += 10) {{
    el("line", {{ x1: X(t), y1: m.t + ih, x2: X(t), y2: m.t + ih + 4, stroke: "#c8ceda", "stroke-width": "1" }});
    el("text", {{ x: X(t), y: m.t + ih + 18, "text-anchor": "middle", "font-size": "11", fill: "#7b8494" }}, t + "s");
  }}
  el("text", {{ x: m.l + iw / 2, y: H - 6, "text-anchor": "middle", "font-size": "11", fill: "#7b8494" }}, "run time (s)");
  el("line", {{ x1: X(startupEnd), y1: Y(p95), x2: X(mappingEnd), y2: Y(p95),
    stroke: "#c0392b", "stroke-width": "1.2", "stroke-dasharray": "6 5", opacity: ".75" }});
  el("text", {{ x: X(mappingEnd) - 4, y: Y(p95) - 6, "text-anchor": "end",
    "font-size": "11", fill: "#c0392b", "font-weight": "600" }}, "mapping p95 " + p95.toFixed(1) + "%");
  el("line", {{ x1: X(lateT), y1: m.t, x2: X(lateT), y2: m.t + ih,
    stroke: "#b26a00", "stroke-width": "1", "stroke-dasharray": "3 4", opacity: ".7" }});
  var d = data.map(function (p, i) {{ return (i ? "L" : "M") + X(p[0]).toFixed(2) + " " + Y(p[1]).toFixed(2); }}).join(" ");
  el("path", {{ d: d + " L" + X(data[data.length - 1][0]).toFixed(2) + " " + Y(0) +
    " L" + X(data[0][0]).toFixed(2) + " " + Y(0) + " Z", fill: "rgba(58,109,240,.14)" }});
  el("path", {{ d: d, fill: "none", stroke: "#3a6df0", "stroke-width": "2.2",
    "stroke-linejoin": "round", "stroke-linecap": "round" }});
  el("circle", {{ cx: X(peakT), cy: Y(peakV), r: "4", fill: "#fff", stroke: "#3a6df0", "stroke-width": "2" }});
  el("text", {{ x: X(peakT), y: Y(peakV) - 12, "text-anchor": "middle", "font-size": "11",
    fill: "#1f2430", "font-weight": "600" }}, peakV.toFixed(1) + "%");
  el("circle", {{ cx: X(lateT), cy: Y(lateV), r: "4", fill: "#fff", stroke: "#b26a00", "stroke-width": "2" }});
  el("text", {{ x: X(lateT), y: Y(lateV) - 12, "text-anchor": "middle", "font-size": "11",
    fill: "#8a5200", "font-weight": "600" }}, "late max " + lateV.toFixed(1) + "%");
  data.forEach(function (p) {{
    el("circle", {{ cx: X(p[0]), cy: Y(p[1]), r: "1.8", fill: "#2c5fd6", opacity: ".55" }});
  }});
}})();
</script>
</body>
</html>
""")
    with open(out, "w", encoding="utf-8") as f:
        f.write("".join(h))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default=DEFAULT_RUN)
    ap.add_argument("--out", default=DEFAULT_OUT)
    args = ap.parse_args()
    out = build(args.run, args.out)
    print(out)


if __name__ == "__main__":
    main()
