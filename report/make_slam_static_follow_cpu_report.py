#!/usr/bin/env python3
"""slam_toolbox static-map + rf2o CPU report (static follow).

Reads one follow_monitor run directory (cpu.csv / cpu_nodes.csv /
follow_target.csv) and writes a self-contained HTML report in the same style
as report/slam_rf2o_cpu_run_20261118-164238.html.

Usage:
  python3 make_slam_static_follow_cpu_report.py \
      --run /nvt_ros2_example/slam_toolbox_rf2o/monitor_logs/run_20261119-170751_follow \
      --out slam_static_follow_cpu_run_20261119-170751.html
"""
import argparse
import csv
import html
import os
import statistics as st
from collections import defaultdict

BASE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_RUN = "/nvt_ros2_example/slam_toolbox_rf2o/monitor_logs/run_20261119-170751_follow"
DEFAULT_OUT = os.path.join(BASE, "slam_static_follow_cpu_run_20261119-170751.html")
NCORE, NOMINAL = 4, 2.0

LABEL = {
    "process:icp_odometry": "icp_odometry", "/rtabmap/rtabmap": "rtabmap",
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

# 11-18 16:42: slam_toolbox dynamic mapping + rf2o (same for_rtabmap frame).
DYN = {
    "duration": 81.4, "total": 59.8, "total_p95": 89.8, "total_max": 96.5,
    "follow_mean": 53.0, "follow_p95": 66.0, "follow_max": 69.1,
    "rf2o": 1.7, "rf2o_max": 2.7, "slam": 8.9, "slam_max": 35.3,
    "camera": 11.0, "ai": 6.6, "python3": 7.8, "ros2": 3.9,
    "rplidar": 3.8, "ekf": 3.5, "controller": 2.3, "driver": 2.0,
    "collision": 1.7, "bt": 1.5, "demo": 1.1, "planner": 0.9,
    "velocity": 0.6, "docking": 0.5, "behavior": 0.4, "static_tf": 0.4,
    "recovery": 0.4, "path": 16.81, "cores": [(62.7, 92.1), (56.5, 100.0),
                                               (59.5, 96.7), (60.2, 99.5)],
}

# 11-19 16:51: static localization, short target-selection sample (no movement).
SHORT = {
    "duration": 58.8, "total": 46.6, "total_p95": 91.4, "total_max": 94.1,
    "follow_mean": 38.4, "follow_p95": 44.4, "follow_max": 45.8,
    "rf2o": 1.76, "rf2o_max": 3.40, "slam": 1.15, "slam_max": 3.74,
    "camera": 10.32, "ai": 6.71, "path": 0.09,
}

# 11-18 15:45: RTAB-Map + ICP baseline (from the existing 15:45 report).
RTAB = {
    "duration": 58.9, "total": 72.1, "total_p95": 94.5, "total_max": 95.1,
    "follow_mean": 72.2, "follow_p95": 82.5, "follow_max": 95.1,
    "icp": 13.4, "icp_max": 19.0, "rtab": 4.5, "rtab_max": 9.0,
    "camera": 11.3, "ai": 6.6, "path": 11.18,
    "cores": [(73.2, 91.2), (71.3, 100.0), (71.4, 99.6), (72.3, 99.6)],
}

DYN_BY_NODE = {
    "camera": DYN["camera"], "slam_toolbox": DYN["slam"],
    "python3 (bag/watch/monitor)": DYN["python3"], "ai_human_detection": DYN["ai"],
    "ros2 launch": DYN["ros2"], "rplidar_node": DYN["rplidar"], "ekf_node": DYN["ekf"],
    "controller_server": DYN["controller"], "Mcnamu_driver": DYN["driver"],
    "collision_monitor": DYN["collision"], "rf2o (CLaserOdometry2D)": DYN["rf2o"],
    "bt_navigator": DYN["bt"], "demo_manager_node": DYN["demo"],
    "planner_server": DYN["planner"], "velocity_smoother": DYN["velocity"],
    "docking_server": DYN["docking"], "behavior_server": DYN["behavior"],
    "static TF": DYN["static_tf"], "person_target_recovery": DYN["recovery"],
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
    states = [(float(r["t"]) - t0, r["note"]) for r in ft if r["kind"] == "state"]
    changes = []
    prev = None
    for t, s in states:
        if s != prev:
            changes.append((t, s))
            prev = s
    goals = sum(1 for r in ft if r["kind"] == "goal")
    persons = [r for r in ft if r["kind"] == "person"]
    cmds = [(float(r["t"]) - t0, abs(float(r["vx"])), abs(float(r["wz"])))
            for r in ft if r["kind"] == "cmd"]
    rob = [(float(r["t"]) - t0, float(r["x"]), float(r["y"]))
           for r in ft if r["kind"] == "robot"]
    path = sum(((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5
               for a, b in zip(rob, rob[1:]))
    net = (((rob[-1][1] - rob[0][1]) ** 2 + (rob[-1][2] - rob[0][2]) ** 2) ** 0.5) if rob else 0.0

    def first(state):
        for t, s in states:
            if s == state:
                return t
        return None

    f_t = first("FOLLOWING")
    h_t = first("RETURN_HOME") or (ts[-1] - t0)
    s_t = first("STOPPED") or (ts[-1] - t0)
    for item in stats:
        item["follow"] = st.mean([series[item["node"]].get(float(r["t_unix"]), 0.0)
                                  for r in cpu
                                  if f_t <= float(r["t_unix"]) - t0 < h_t])
    return {
        "run": os.path.basename(os.path.normpath(run_dir)),
        "elapsed": [t - t0 for t in ts],
        "busy": [float(r["total_busy"]) for r in cpu],
        "cores": {c: [float(r[c]) for r in cpu] for c in ("cpu0", "cpu1", "cpu2", "cpu3")},
        "stats": stats, "states": changes, "goals": goals, "persons": persons,
        "follow_t": f_t, "home_t": h_t, "stop_t": s_t,
        "duration": ts[-1] - t0, "n": len(cpu),
        "path": path, "net": net,
        "maxvx": max((c[1] for c in cmds), default=0.0),
        "maxwz": max((c[2] for c in cmds), default=0.0),
    }


def phase(run, a, b):
    v = [x for t, x in zip(run["elapsed"], run["busy"]) if a <= t < b]
    return (st.mean(v), q(v, 95), max(v), len(v)) if v else (0.0, 0.0, 0.0, 0)


def esc(s):
    return html.escape(str(s))


def bar(avg, mx, scale):
    return (f'<div class="bar-track"><div class="bar-fill" style="width:{min(100, avg / scale * 100):.1f}%"></div>'
            f'<div class="bar-max" style="left:{min(100, mx / scale * 100):.1f}%"></div></div>')


def build(run_dir, out):
    M = load(run_dir)
    by = {s["node"]: s for s in M["stats"]}
    fol = phase(M, M["follow_t"], M["home_t"])
    start = phase(M, 0.0, M["follow_t"])
    aft = phase(M, M["home_t"], M["stop_t"])
    allp = phase(M, 0.0, M["duration"])
    grp_odom = by["/CLaserOdometry2D"]["avg"] + by["/slam_toolbox"]["avg"]
    grp_cam = by["/camera/camera"]["avg"] + by["/ai_human_detection"]["avg"]
    grp_lidar = by["/rplidar_node"]["avg"] + by["/ekf_filter_node"]["avg"]
    nav_excl = {"process:icp_odometry", "/rtabmap/rtabmap", "/CLaserOdometry2D", "/slam_toolbox",
                "/camera/camera", "/ai_human_detection", "/rplidar_node", "/ekf_filter_node",
                "/demo_manager_node", "/person_target_recovery_node",
                "/base_to_ai_human_detection_tf", "process:python3", "process:ros2",
                "process:static_transfor"}
    grp_nav = sum(s["avg"] for s in M["stats"] if s["node"] not in nav_excl)
    grp_measure = (by.get("process:python3", {}).get("avg", 0.0)
                   + by.get("process:ros2", {}).get("avg", 0.0)
                   + by.get("process:static_transfor", {}).get("avg", 0.0))
    follow_peak = max(((t, v) for t, v in zip(M["elapsed"], M["busy"])
                       if M["follow_t"] <= t < M["home_t"]), key=lambda x: x[1])
    global_peak = max(zip(M["elapsed"], M["busy"]), key=lambda x: x[1])
    slam = by["/slam_toolbox"]
    slam_peak = max(series_peak(run_dir, "/slam_toolbox"), key=lambda x: x[1])
    scale = max(6.0, M["stats"][0]["avg"] * 1.15)

    h = []
    w = h.append
    w(f"""<!DOCTYPE html>
<html lang="zh-Hant">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>slam_toolbox 靜態地圖 + rf2o CPU 分析 — {M['run']}</title>
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
  <h1>slam_toolbox 靜態地圖 + rf2o CPU 分析（static follow）</h1>
  <p>{M['run']} · 2026-11-19 17:08:00 · {M['duration']:.1f} s · localization（static map）· lite bag · demo_manager 1.1.0+patch</p>
  <div class="pills">
    <span class="pill">slam_toolbox localization + rf2o</span>
    <span class="pill">D455 640×480 @ 15</span>
    <span class="pill">4 核 · node CPU 已換算四核佔比</span>
    <span class="pill good">跟隨 mean {fol[0]:.1f}% / p95 {fol[1]:.1f}%</span>
    <span class="pill warn">整段 max {max(M['busy']):.1f}%（啟動）</span>
  </div>
</header>

<div class="kpis">
  <div class="kpi good">
    <div class="label">跟隨期間 mean</div>
    <div class="value">{fol[0]:.1f}%</div>
    <div class="sub">{M['follow_t']:.1f} ~ {M['home_t']:.1f} s（{fol[3]} 筆）</div>
  </div>
  <div class="kpi good">
    <div class="label">跟隨期間 p95</div>
    <div class="value">{fol[1]:.1f}%</div>
    <div class="sub">max {fol[2]:.1f}%；動態建圖 16:42 p95 66.0%、RTAB-Map 15:45 p95 82.5%</div>
  </div>
  <div class="kpi good">
    <div class="label">rf2o 佔比（四核）</div>
    <div class="value">{by['/CLaserOdometry2D']['avg']:.1f}%</div>
    <div class="sub">slam_toolbox {slam['avg']:.1f}%（跟隨 {slam['follow']:.1f}%）；動態建圖 slam_toolbox 8.9%</div>
  </div>
  <div class="kpi warn">
    <div class="label">整段 max</div>
    <div class="value">{max(M['busy']):.1f}%</div>
    <div class="sub">啟動段 {global_peak[0]:.1f} s；跟隨期間 max {follow_peak[1]:.1f}%（{follow_peak[0]:.1f} s）</div>
  </div>
</div>
<p class="small" style="margin-top:12px">total_busy 為四核整體使用率（0~100%）；node / process 已由單核換算四核佔比（單核 ÷ 4，1 核滿載 = 25%），並以 <code>名目 2.0 s ÷ 實際間隔</code> 修正。這是 <strong>localization（載入既有 posegraph）</strong>，不是動態建圖。</p>

<h2>1. Run 概況</h2>
<div class="grid2">
  <div class="card">
    <h3 style="margin-top:0">狀態時間線</h3>
    <div class="timeline">
      <div class="tl-track">
        <div class="tl-seg tl-sel" style="width:{M['follow_t']/M['duration']*100:.1f}%">SELECT / WAIT</div>
        <div class="tl-seg tl-follow" style="width:{(M['home_t']-M['follow_t'])/M['duration']*100:.1f}%">FOLLOWING</div>
        <div class="tl-seg tl-home" style="width:{(M['duration']-M['home_t'])/M['duration']*100:.1f}%">RETURN_HOME</div>
      </div>
      <div class="tl-labels">
        <span style="width:{M['follow_t']/M['duration']*100:.1f}%;text-align:left">0 s</span>
        <span style="width:{(M['home_t']-M['follow_t'])/M['duration']*100:.1f}%;text-align:center">{M['follow_t']:.1f} → {M['home_t']:.1f} s</span>
        <span style="width:{(M['duration']-M['home_t'])/M['duration']*100:.1f}%;text-align:right">{M['duration']:.1f} s</span>
      </div>
    </div>
    <div class="flow" style="margin-top:14px">+{M['follow_t']:.1f}  FOLLOWING（{M['goals']} 個 goal）
+{M['home_t']:.1f}  RETURN_HOME
+{M['duration']:.1f}  錄製結束（尚未收到 STOPPED）</div>
    <p class="small" style="margin-bottom:0">第一個狀態 <code>TARGET_SELECTED</code> 出現在 +10.4 s；錄影在 RETURN_HOME 途中結束。</p>
  </div>
  <div class="card">
    <h3 style="margin-top:0">跟隨統計</h3>
    <table style="margin-top:4px;box-shadow:none">
      <tbody>
        <tr><td>跟隨時間</td><td class="num">{M['home_t']-M['follow_t']:.1f} s</td></tr>
        <tr><td>goals</td><td class="num">{M['goals']}</td></tr>
        <tr><td>person 訊息</td><td class="num">{len(M['persons'])} 筆</td></tr>
        <tr><td>robot</td><td class="num">net {M['net']:.2f} m / path {M['path']:.2f} m</td></tr>
        <tr><td>max |vx| / |wz|</td><td class="num">{M['maxvx']:.2f} m/s / {M['maxwz']:.2f} rad/s</td></tr>
      </tbody>
    </table>
  </div>
</div>

<h2>2. 整體 CPU</h2>
<div class="chart-box">
  <svg id="cpuChart" viewBox="0 0 900 350" role="img" aria-label="static follow CPU 使用率時間序列"></svg>
  <div class="legend">
    <span><i style="background:#3a6df0"></i>total_busy（2 s 取樣）</span>
    <span><i style="background:rgba(58,109,240,.10)"></i>FOLLOWING {M['follow_t']:.1f} ~ {M['home_t']:.1f} s</span>
    <span><i style="background:rgba(42,157,143,.10)"></i>RETURN_HOME {M['home_t']:.1f} ~ {M['duration']:.1f} s</span>
    <span><i style="background:#c0392b"></i>follow p95 {fol[1]:.1f}%</span>
  </div>
</div>

<table>
  <thead><tr><th>區間</th><th class="num">筆數</th><th class="num">mean</th><th class="num">p95</th><th class="num">max</th></tr></thead>
  <tbody>
    <tr><td>整段 0 ~ {M['duration']:.1f} s</td><td class="num">{allp[3]}</td><td class="num">{allp[0]:.1f}%</td><td class="num">{allp[1]:.1f}%</td><td class="num">{allp[2]:.1f}%</td></tr>
    <tr><td>啟動 / 選人 0 ~ {M['follow_t']:.1f} s</td><td class="num">{start[3]}</td><td class="num">{start[0]:.1f}%</td><td class="num">{start[1]:.1f}%</td><td class="num">{start[2]:.1f}%</td></tr>
    <tr><td>跟隨 {M['follow_t']:.1f} ~ {M['home_t']:.1f} s</td><td class="num">{fol[3]}</td><td class="num">{fol[0]:.1f}%</td><td class="num"><strong>{fol[1]:.1f}%</strong></td><td class="num">{fol[2]:.1f}%</td></tr>
    <tr><td>返回 {M['home_t']:.1f} ~ {M['duration']:.1f} s</td><td class="num">{aft[3]}</td><td class="num">{aft[0]:.1f}%</td><td class="num">{aft[1]:.1f}%</td><td class="num">{aft[2]:.1f}%</td></tr>
  </tbody>
</table>
<p class="small">整段最高 {global_peak[1]:.1f}% 在 +{global_peak[0]:.1f} s（bag / monitor 啟動與 DDS discovery，所有節點同時被喚醒）；跟隨期間最高 {follow_peak[1]:.1f}% 在 +{follow_peak[0]:.1f} s，對應 slam_toolbox 的 {slam_peak[1]:.1f}%（四核）單筆尖峰。返回段相對平穩（mean {aft[0]:.1f}%）。</p>

<h2>3. Per-core</h2>
<p class="lead">本次整段每核 mean {min(st.mean(v) for v in M['cores'].values()):.1f}~{max(st.mean(v) for v in M['cores'].values()):.1f}%，單核 max {min(max(v) for v in M['cores'].values()):.1f}~{max(max(v) for v in M['cores'].values()):.1f}%。</p>
<table>
  <thead><tr><th>Core</th><th class="num">本次 mean</th><th class="num">本次 max</th><th class="num">16:42 動態 mean / max</th><th class="num">15:45 RTAB mean / max</th><th class="bars">本次 mean（每核 0~100%）</th></tr></thead>
  <tbody>
""")
    for i, (c, v) in enumerate(M["cores"].items()):
        dyn = DYN["cores"][i]
        rtab = RTAB["cores"][i]
        w(f'<tr><td>{c}</td><td class="num">{st.mean(v):.1f}%</td><td class="num">{max(v):.1f}%</td>'
          f'<td class="num">{dyn[0]:.1f}% / {dyn[1]:.1f}%</td>'
          f'<td class="num">{rtab[0]:.1f}% / {rtab[1]:.1f}%</td>'
          f'<td class="bars">{bar(st.mean(v), max(v), 100)}</td></tr>')
    w(f"""  </tbody>
</table>
<p class="small">static localization 比 16:42 動態建圖每核低約 5~7 個百分點，比 15:45 RTAB-Map + ICP 低約 18~20 個百分點。單核 max 仍在啟動段碰到 99.1%。</p>

<h2>4. static follow vs 動態建圖 vs RTAB-Map</h2>
<p class="lead">同一個 for_rtabmap / demo 外框（D455、AI、Nav2、lite bag 都相同），差別只在 odom 與 SLAM 模式。17:07 這筆是 <strong>localization（static map）</strong>；16:42 是動態建圖；15:45 是 RTAB-Map + ICP。</p>
<table>
  <thead><tr><th>指標（四核佔比）</th><th class="num">17:07 static（本次）</th><th class="num">16:51 static（短樣本）</th><th class="num">16:42 動態建圖</th><th class="num">15:45 RTAB-Map + ICP</th></tr></thead>
  <tbody>
    <tr><td>雷射 odom</td><td class="num">rf2o <strong>{by['/CLaserOdometry2D']['avg']:.1f}%</strong>（max {by['/CLaserOdometry2D']['max']:.1f}）</td><td class="num">rf2o {SHORT['rf2o']:.1f}%</td><td class="num">rf2o {DYN['rf2o']:.1f}%</td><td class="num">ICP <strong>{RTAB['icp']:.1f}%</strong>（max {RTAB['icp_max']:.1f}）</td></tr>
    <tr><td>SLAM 節點</td><td class="num">slam_toolbox <strong>{slam['avg']:.1f}%</strong>（max {slam['max']:.1f}）</td><td class="num">slam_toolbox {SHORT['slam']:.1f}%（max {SHORT['slam_max']:.1f}）</td><td class="num">slam_toolbox {DYN['slam']:.1f}%（max {DYN['slam_max']:.1f}）</td><td class="num">rtabmap {RTAB['rtab']:.1f}%（max {RTAB['rtab_max']:.1f}）</td></tr>
    <tr><td>odom + SLAM 合計</td><td class="num"><strong>{grp_odom:.1f}%</strong></td><td class="num">{SHORT['rf2o']+SHORT['slam']:.1f}%</td><td class="num">{DYN['rf2o']+DYN['slam']:.1f}%</td><td class="num">{RTAB['icp']+RTAB['rtab']:.1f}%</td></tr>
    <tr><td>整段總 CPU mean</td><td class="num">{allp[0]:.1f}%</td><td class="num">{SHORT['total']:.1f}%</td><td class="num">{DYN['total']:.1f}%</td><td class="num">{RTAB['total']:.1f}%</td></tr>
    <tr><td>整段 p95 / max</td><td class="num">{allp[1]:.1f} / {allp[2]:.1f}%</td><td class="num">{SHORT['total_p95']:.1f} / {SHORT['total_max']:.1f}%</td><td class="num">{DYN['total_p95']:.1f} / {DYN['total_max']:.1f}%</td><td class="num">{RTAB['total_p95']:.1f} / {RTAB['total_max']:.1f}%</td></tr>
    <tr><td>跟隨 mean / p95</td><td class="num"><strong>{fol[0]:.1f} / {fol[1]:.1f}%</strong></td><td class="num">{SHORT['follow_mean']:.1f} / {SHORT['follow_p95']:.1f}%</td><td class="num">{DYN['follow_mean']:.1f} / {DYN['follow_p95']:.1f}%</td><td class="num">{RTAB['follow_mean']:.1f} / {RTAB['follow_p95']:.1f}%</td></tr>
    <tr><td>robot path</td><td class="num">{M['path']:.2f} m</td><td class="num">{SHORT['path']:.2f} m</td><td class="num">{DYN['path']:.2f} m</td><td class="num">{RTAB['path']:.2f} m</td></tr>
    <tr><td>run 長度</td><td class="num">{M['duration']:.1f} s</td><td class="num">{SHORT['duration']:.1f} s</td><td class="num">{DYN['duration']:.1f} s</td><td class="num">{RTAB['duration']:.1f} s</td></tr>
  </tbody>
</table>
<div class="callout good">
  <strong>static follow 的總 CPU 是 54.6%（p95 69.2%），比 16:42 動態建圖低約 5.2 個百分點，比 RTAB-Map + ICP 低約 17.5 個百分點。</strong>
  差距主要來自 SLAM 節點（static 6.5% vs 動態 8.9% vs ICP 13.4%）；rf2o 在三個方案裡都只要 1.7~1.8%。
</div>
<div class="callout">
  <strong>但 static 不是「免費」：</strong>跟隨期間 slam_toolbox 平均 {slam['follow']:.1f}%，比整段 {slam['avg']:.1f}% 更高；+53.1 s 還出現 {slam['max']:.1f}% 的單筆尖峰。16:51 那筆短樣本 slam_toolbox 只有 1.15%，是因為機器人幾乎沒動（path 0.09 m、0 goals），不能當成 static follow 的常態。
</div>

<h2>5. Node CPU — 整段與跟隨（四核佔比）</h2>
<p class="lead">odom + SLAM 約 {grp_odom:.1f}%（rf2o + slam_toolbox）；相機 + AI 約 {grp_cam:.1f}%；Nav2 + 底盤 + demo 約 {grp_nav:.1f}%；光達 + EKF 約 {grp_lidar:.1f}%；量測本身（python3 + ros2 + static TF）約 {grp_measure:.1f}%。</p>
<div class="grid2" style="margin-bottom:16px">
  <div class="card" style="border-top:4px solid var(--accent)">
    <div class="label small">odom + SLAM</div>
    <div style="font-size:30px;font-weight:700;font-variant-numeric:tabular-nums">約 {grp_odom:.1f}%</div>
    <div class="small">rf2o {by['/CLaserOdometry2D']['avg']:.1f} + slam_toolbox {slam['avg']:.1f}（跟隨 {slam['follow']:.1f}）</div>
  </div>
  <div class="card" style="border-top:4px solid var(--ai)">
    <div class="label small">AI + camera</div>
    <div style="font-size:30px;font-weight:700;font-variant-numeric:tabular-nums">約 {grp_cam:.1f}%</div>
    <div class="small">ai {by['/ai_human_detection']['avg']:.1f} + camera {by['/camera/camera']['avg']:.1f} — 最大塊</div>
  </div>
  <div class="card" style="border-top:4px solid #6b7484">
    <div class="label small">Nav2 + 底盤 + demo</div>
    <div style="font-size:30px;font-weight:700;font-variant-numeric:tabular-nums">約 {grp_nav:.1f}%</div>
    <div class="small">controller / planner / BT / collision / driver / demo …</div>
  </div>
</div>
<table>
  <thead><tr><th>Node</th><th class="num">mean</th><th class="num">跟隨 mean</th><th class="num">max</th><th class="num">RSS</th><th class="num">16:42 動態</th><th class="bars">mean 分布（0~{scale:.0f}%）</th></tr></thead>
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
        old = DYN_BY_NODE.get(s["label"])
        w(f'<tr><td><code>{esc(s["label"])}</code></td><td class="num"><strong>{s["avg"]:.1f}%</strong></td>'
          f'<td class="num">{s["follow"]:.1f}%</td><td class="num">{s["max"]:.1f}%</td>'
          f'<td class="num">{s["rss"]:.0f} MB</td>'
          f'<td class="num">{f"{old:.1f}%" if old is not None else "—"}</td>'
          f'<td class="bars">{bar(s["avg"], s["max"], scale)}</td></tr>')
    w(f"""  </tbody>
</table>
<p class="small">長條為整段 mean、紅線為 max；「16:42 動態」是同名節點在動態建圖 run 的整段 mean。python3 是 bag / follow_watch / monitor 的量測負擔。</p>

<h2>6. 觀察與建議</h2>
<ul class="tight">
  <li><strong>slam_toolbox 是 static follow 最值得觀察的節點。</strong>整段 {slam['avg']:.1f}%、跟隨 {slam['follow']:.1f}%、max {slam['max']:.1f}%（+53.1 s）；單筆 raw CPU 106%（單核），node 有 28 個 thread。啟動 log 顯示 <code>options.num_threads: 50 exceeds maximum ... 4</code>；這是 slam_toolbox Ceres solver plugin 內建的 <code>options_.num_threads = 50</code>（<code>solvers/ceres_solver.cpp</code>），不是 yaml 參數，Jazzy 的 Ceres 會 bound 到 4。若尖峰造成 map→odom 延遲，需改 source 或換 solver plugin；先確認 +53.1 s 的尖峰是否與 scan matching / 地圖大小有關。</li>
  <li><strong>rf2o 不是瓶頸。</strong>整段 {by['/CLaserOdometry2D']['avg']:.1f}%、跟隨 {by['/CLaserOdometry2D']['follow']:.1f}%；與 16:42（1.7%）、16:51（1.8%）一致。</li>
  <li><strong>相機 + AI 是最大塊（{grp_cam:.1f}%）。</strong>camera {by['/camera/camera']['avg']:.1f}% + AI {by['/ai_human_detection']['avg']:.1f}%；若不需要 AI，關掉可省下大部分。</li>
  <li><strong>量測負擔約 {grp_measure:.1f}%。</strong>python3（bag / follow_watch / monitor）{by.get('process:python3', {}).get('avg', 0):.1f}% + ros2 launch {by.get('process:ros2', {}).get('avg', 0):.1f}%；要更乾淨的基準可用 <code>--no-bag --no-watch</code> 再跑一次。</li>
  <li><strong>啟動尖峰與跟隨無關。</strong>整段 max {global_peak[1]:.1f}% 在 +{global_peak[0]:.1f} s；跟隨期間最高 {follow_peak[1]:.1f}%（+{follow_peak[0]:.1f} s），返回段最高 {aft[2]:.1f}%。</li>
  <li><strong>static 的取捨。</strong>總 CPU 比動態建圖低 5.2 個百分點，但跟隨期間 slam_toolbox 仍要 {slam['follow']:.1f}%；static 省掉的是地圖成長 / loop closure，不是 scan matching。</li>
</ul>

<h2>7. 紀錄檔</h2>
<div class="flow">/nvt_ros2_example/slam_toolbox_rf2o/monitor_logs/{M['run']}/
  cpu.csv / cpu_nodes.csv / cpu_procs.csv
  follow_target.csv   {len(M['persons'])} person 筆 · {M['goals']} goals
  RUN_INFO.txt, .env.sh, bag/
slam_toolbox log: /tmp/slam_toolbox_only/logs/slam.log
  （本次 session 18 次 queue-full drop，run 期間 1 次在 +4.15 s）
對照：16:51 static 短樣本 · 16:42 動態建圖 · 15:45 RTAB-Map + ICP</div>

<footer>
  slam_toolbox static map + rf2o CPU 分析 · {M['run']} · 2026-11-19 ·
  資料來源 <code>cpu.csv</code> / <code>cpu_nodes.csv</code> / <code>follow_target.csv</code> · monitor_cpu.py（--interval 2）
</footer>

</div>
<script>
(function () {{
  var data = {[[round(x, 1), round(v, 1)] for x, v in zip(M['elapsed'], M['busy'])]};
  var followStart = {M['follow_t']:.3f}, followEnd = {M['home_t']:.3f}, runEnd = {M['duration']:.3f};
  var p95 = {fol[1]:.1f}, peakT = {global_peak[0]:.1f}, peakV = {global_peak[1]:.1f};
  var followPeakT = {follow_peak[0]:.1f}, followPeakV = {follow_peak[1]:.1f};
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
  el("rect", {{ x: X(followStart), y: m.t, width: X(followEnd) - X(followStart), height: ih, fill: "rgba(58,109,240,.10)" }});
  el("text", {{ x: (X(followStart) + X(followEnd)) / 2, y: m.t + 14, "text-anchor": "middle",
    "font-size": "11.5", fill: "#3a6df0", "font-weight": "600", "letter-spacing": ".6" }}, "FOLLOWING");
  el("rect", {{ x: X(followEnd), y: m.t, width: X(runEnd) - X(followEnd), height: ih, fill: "rgba(42,157,143,.10)" }});
  el("text", {{ x: (X(followEnd) + X(runEnd)) / 2, y: m.t + 14, "text-anchor": "middle",
    "font-size": "11.5", fill: "#2a9d8f", "font-weight": "600", "letter-spacing": ".6" }}, "RETURN_HOME");
  [0, 25, 50, 75, 100].forEach(function (v) {{
    el("line", {{ x1: m.l, y1: Y(v), x2: m.l + iw, y2: Y(v), stroke: "#e3e6ee", "stroke-width": v === 0 ? "1.4" : "1" }});
    el("text", {{ x: m.l - 9, y: Y(v) + 4, "text-anchor": "end", "font-size": "11", fill: "#7b8494" }}, v + "%");
  }});
  for (var t = 0; t <= xMax; t += 10) {{
    el("line", {{ x1: X(t), y1: m.t + ih, x2: X(t), y2: m.t + ih + 4, stroke: "#c8ceda", "stroke-width": "1" }});
    el("text", {{ x: X(t), y: m.t + ih + 18, "text-anchor": "middle", "font-size": "11", fill: "#7b8494" }}, t + "s");
  }}
  el("text", {{ x: m.l + iw / 2, y: H - 6, "text-anchor": "middle", "font-size": "11", fill: "#7b8494" }}, "run time (s)");
  el("line", {{ x1: X(followStart), y1: Y(p95), x2: X(followEnd), y2: Y(p95),
    stroke: "#c0392b", "stroke-width": "1.2", "stroke-dasharray": "6 5", opacity: ".75" }});
  el("text", {{ x: X(followEnd) - 4, y: Y(p95) - 6, "text-anchor": "end",
    "font-size": "11", fill: "#c0392b", "font-weight": "600" }}, "follow p95 " + p95.toFixed(1) + "%");
  el("line", {{ x1: X(followPeakT), y1: m.t, x2: X(followPeakT), y2: m.t + ih,
    stroke: "#b26a00", "stroke-width": "1", "stroke-dasharray": "3 4", opacity: ".7" }});
  var d = data.map(function (p, i) {{ return (i ? "L" : "M") + X(p[0]).toFixed(2) + " " + Y(p[1]).toFixed(2); }}).join(" ");
  el("path", {{ d: d + " L" + X(data[data.length - 1][0]).toFixed(2) + " " + Y(0) +
    " L" + X(data[0][0]).toFixed(2) + " " + Y(0) + " Z", fill: "rgba(58,109,240,.14)" }});
  el("path", {{ d: d, fill: "none", stroke: "#3a6df0", "stroke-width": "2.2",
    "stroke-linejoin": "round", "stroke-linecap": "round" }});
  el("circle", {{ cx: X(peakT), cy: Y(peakV), r: "4", fill: "#fff", stroke: "#3a6df0", "stroke-width": "2" }});
  el("text", {{ x: X(peakT), y: Y(peakV) - 12, "text-anchor": "middle", "font-size": "11",
    fill: "#1f2430", "font-weight": "600" }}, peakV.toFixed(1) + "%");
  el("circle", {{ cx: X(followPeakT), cy: Y(followPeakV), r: "4", fill: "#fff", stroke: "#b26a00", "stroke-width": "2" }});
  el("text", {{ x: X(followPeakT), y: Y(followPeakV) - 12, "text-anchor": "middle", "font-size": "11",
    fill: "#8a5200", "font-weight": "600" }}, "follow max " + followPeakV.toFixed(1) + "%");
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


def series_peak(run_dir, node):
    """Return (t_relative, four-core %) for one node, for the spike marker."""
    cpu = read(os.path.join(run_dir, "cpu.csv"))
    nodes = read(os.path.join(run_dir, "cpu_nodes.csv"))
    ts = [float(r["t_unix"]) for r in cpu]
    t0 = ts[0]
    gaps = [NOMINAL] + [b - a for a, b in zip(ts, ts[1:])]
    fac = [NOMINAL / g for g in gaps]
    out = []
    for r in nodes:
        if r["node"] != node:
            continue
        i = ts.index(float(r["t_unix"]))
        out.append((float(r["t_unix"]) - t0, float(r["cpu_pct"]) * fac[i] / NCORE))
    return out or [(0.0, 0.0)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default=DEFAULT_RUN)
    ap.add_argument("--out", default=DEFAULT_OUT)
    args = ap.parse_args()
    out = build(args.run, args.out)
    print(out)


if __name__ == "__main__":
    main()
