#!/usr/bin/env python3
"""slam_toolbox + rf2o CPU report for the 16:42 run, same style as
report/person_follow_cpu_run_*.html."""
import csv
import html
import os
import re
import statistics as st
from collections import Counter, defaultdict

BASE = os.path.dirname(os.path.abspath(__file__))
MAIN = "run_20261118-164238_follow"
CMP = "run_20261118-154507_follow"
ICP_LOG = os.path.join(BASE, MAIN, "icp_odometry.log")  # 本 run 沒有 ICP，僅保留函式
OUT = os.path.join(BASE, "slam_rf2o_cpu_run_20261118-164238.html")
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


def read(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def q(vals, p):
    vals = sorted(vals)
    return vals[max(0, min(len(vals) - 1, int(round(p / 100 * (len(vals) - 1)))))] if vals else 0.0


def load(run):
    d = os.path.join(BASE, run)
    cpu, nodes = read(os.path.join(d, "cpu.csv")), read(os.path.join(d, "cpu_nodes.csv"))
    ft = read(os.path.join(d, "follow_target.csv"))
    ts = [float(r["t_unix"]) for r in cpu]
    gaps = [NOMINAL] + [b - a for a, b in zip(ts, ts[1:])]
    fac = [NOMINAL / g for g in gaps]
    t0 = ts[0]
    series, rss = defaultdict(dict), defaultdict(dict)
    for r in nodes:
        i = ts.index(float(r["t_unix"]))
        series[r["node"]][r["t_unix"]] = series[r["node"]].get(r["t_unix"], 0.0) + \
            float(r["cpu_pct"]) * fac[i] / NCORE
        rss[r["node"]][r["t_unix"]] = max(rss[r["node"]].get(r["t_unix"], 0.0), float(r["rss_mb"]))
    stats = [{"node": n, "label": LABEL.get(n, n),
              "avg": st.mean([s.get(r["t_unix"], 0.0) for r in cpu]),
              "max": max([s.get(r["t_unix"], 0.0) for r in cpu]),
              "rss": st.mean(list(rss[n].values()))} for n, s in series.items()]
    stats.sort(key=lambda s: -s["avg"])
    states = [(float(r["t"]) - t0, r["note"]) for r in ft if r["kind"] == "state"]
    goals = sum(1 for r in ft if r["kind"] == "goal")
    persons = [r for r in ft if r["kind"] == "person"]
    cmds = [(float(r["t"]) - t0, abs(float(r["vx"])), abs(float(r["wz"]))) for r in ft if r["kind"] == "cmd"]
    rob = [(float(r["t"]) - t0, float(r["x"]), float(r["y"])) for r in ft if r["kind"] == "robot"]
    path = sum(((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5 for a, b in zip(rob, rob[1:]))
    net = (((rob[-1][1] - rob[0][1]) ** 2 + (rob[-1][2] - rob[0][2]) ** 2) ** 0.5) if rob else 0.0

    def first(state):
        for t, s in states:
            if s == state:
                return t
        return None

    f_t, f_h = first("FOLLOWING"), first("RETURN_HOME") or (ts[-1] - t0)
    node_follow = {n: st.mean([s.get(r["t_unix"], 0.0) for r in cpu
                               if f_t <= float(r["t_unix"]) - t0 < f_h]) for n, s in series.items()}
    return {
        "run": run, "elapsed": [t - t0 for t in ts], "busy": [float(r["total_busy"]) for r in cpu],
        "node_follow": node_follow,
        "cores": {c: [float(r[c]) for r in cpu] for c in ("cpu0", "cpu1", "cpu2", "cpu3")},
        "stats": stats, "states": states, "goals": goals, "persons": persons,
        "follow_t": first("FOLLOWING"), "home_t": first("RETURN_HOME"),
        "stop_t": first("STOPPED") or (ts[-1] - t0), "duration": ts[-1] - t0, "n": len(cpu),
        "path": path, "net": net,
        "maxvx": max((c[1] for c in cmds), default=0.0),
        "maxwz": max((c[2] for c in cmds), default=0.0),
        "window": lambda a, b: [(t, v) for t, v in zip([x - t0 for x in ts],
                                                       [float(r["total_busy"]) for r in cpu]) if a <= t < b],
    }


def phase(run, a, b):
    v = [x for _, x in run["window"](a, b)]
    return (st.mean(v), q(v, 95), max(v), len(v)) if v else (0.0, 0.0, 0.0, 0)


def icp_metrics(path):
    if not os.path.exists(path):
        return None
    txt = open(path, errors="ignore").read()
    ratio = [float(x) for x in re.findall(r"ratio=([0-9.]+)", txt)]
    upd = [float(x) for x in re.findall(r"update time=([0-9.]+)s", txt)]
    dly = [float(x) for x in re.findall(r"delay=([0-9.]+)", txt)]
    stamps = [float(x) for x in re.findall(r"\[(\d{10}\.\d+)\]", txt)]
    span = max(stamps) - min(stamps) if stamps else 0
    return {
        "n": len(ratio), "span": span, "rate": len(ratio) / span if span else 0,
        "ratio": (min(ratio), q(ratio, 5), q(ratio, 50), q(ratio, 95), max(ratio)),
        "upd": (q(upd, 50), q(upd, 95), max(upd)),
        "dly": (q(dly, 50), q(dly, 95), max(dly)),
        "drops": txt.count("Dropping image/scan"),
        "range_max": (re.findall(r"scan_range_max\s*=\s*([0-9.]+)", txt) or ["?"])[0],
    }


M, C = load(MAIN), load(CMP)
ICP = icp_metrics(ICP_LOG)
by = {s["node"]: s for s in M["stats"]}
cby = {s["node"]: s for s in C["stats"]}

stop_flow = (f"+{M['stop_t']:.1f}  STOPPED\n" if M["stop_t"] < M["duration"] - 0.5 else "")
late = [(t, v) for t, v in zip(M["elapsed"], M["busy"]) if t >= 10.0]
late_max = max(late, key=lambda x: x[1]) if late else (0.0, 0.0)

ph_follow = (M["follow_t"], M["home_t"])
ph_after = (M["home_t"], M["stop_t"] or M["duration"])
fol = phase(M, *ph_follow)
aft = phase(M, *ph_after)
start = phase(M, 0.0, M["follow_t"])

grp_rtab = by["/CLaserOdometry2D"]["avg"] + by["/slam_toolbox"]["avg"]
grp_cam = by["/camera/camera"]["avg"] + by["/ai_human_detection"]["avg"]
grp_lidar = by["/rplidar_node"]["avg"] + by["/ekf_filter_node"]["avg"]
nav_excl = {"process:icp_odometry", "/rtabmap/rtabmap", "/CLaserOdometry2D", "/slam_toolbox",
            "/camera/camera", "/ai_human_detection",
            "/rplidar_node", "/ekf_filter_node", "/demo_manager_node",
            "/person_target_recovery_node", "/base_to_ai_human_detection_tf",
            "process:python3", "process:ros2", "process:static_transfor"}
grp_nav = sum(s["avg"] for s in M["stats"] if s["node"] not in nav_excl)

# ICP baseline measured on the pre-tuning session (14:43-14:51); that log was
# overwritten by the next start, so the values are kept here.
OLD_ICP = {"n": 3550, "span": 480, "rate": 7.4,
           "ratio": (0.0, 0.615, 0.798, 0.825, 0.858),
           "upd": (0.049, 0.129, 1.175), "dly": (0.147, 0.232, 1.273),
           "drops": 35, "range_max": "0（30 m）"}
MID_ICP = {"n": 6852, "span": 703, "rate": 9.74,
           "ratio": (0.0, 0.622, 0.676, 0.781, 0.849),
           "upd": (0.044, 0.069, 0.829), "dly": (0.143, 0.169, 1.045),
           "drops": 7, "range_max": "8"}


def esc(s):
    return html.escape(str(s))


def bar(avg, mx, scale):
    return (f'<div class="bar-track"><div class="bar-fill" style="width:{min(100,avg/scale*100):.1f}%"></div>'
            f'<div class="bar-max" style="left:{min(100,mx/scale*100):.1f}%"></div></div>')


h = []
w = h.append
w(f"""<!DOCTYPE html>
<html lang="zh-Hant">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>slam_toolbox + rf2o CPU 分析 — {MAIN}</title>
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
  <h1>slam_toolbox + rf2o CPU 分析（動態建圖）</h1>
  <p>{MAIN} · 2026-11-18 16:42:43 · {M['duration']:.1f} s · lite bag · 對照 RTAB-Map + ICP（11-18）與 11-13 的 slam run</p>
  <div class="pills">
    <span class="pill">slam_toolbox 動態建圖 + rf2o</span>
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
    <div class="sub">max {fol[2]:.1f}%；RTAB-Map 15:45 p95 82.5%、14:48 p95 84.6%</div>
  </div>
  <div class="kpi good">
    <div class="label">rf2o 佔比（四核）</div>
    <div class="value">{by['/CLaserOdometry2D']['avg']:.1f}%</div>
    <div class="sub">slam_toolbox {by['/slam_toolbox']['avg']:.1f}%；RTAB-Map 的 ICP 是 11.9~14.2%</div>
  </div>
  <div class="kpi warn">
    <div class="label">整段 max</div>
    <div class="value">{max(M['busy']):.1f}%</div>
    <div class="sub">啟動段，跟隨期間 max {fol[2]:.1f}%</div>
  </div>
</div>
<p class="small" style="margin-top:12px">total_busy 為四核整體使用率（0~100%）；node / process 已由單核換算四核佔比（單核 ÷ 4，1 核滿載 = 25%），並以 <code>名目 2.0 s ÷ 實際間隔</code> 修正。</p>

<h2>1. Run 概況</h2>
<div class="grid2">
  <div class="card">
    <h3 style="margin-top:0">狀態時間線</h3>
    <div class="timeline">
      <div class="tl-track">
        <div class="tl-seg tl-sel" style="width:{M['follow_t']/M['duration']*100:.1f}%">SELECT</div>
        <div class="tl-seg tl-follow" style="width:{(M['home_t']-M['follow_t'])/M['duration']*100:.1f}%">FOLLOWING</div>
        <div class="tl-seg tl-home" style="width:{(M['stop_t']-M['home_t'])/M['duration']*100:.1f}%">RETURN_HOME</div>
        <div class="tl-seg tl-stop" style="width:{(M['duration']-M['stop_t'])/M['duration']*100:.1f}%">END</div>
      </div>
      <div class="tl-labels">
        <span style="width:{M['follow_t']/M['duration']*100:.1f}%;text-align:left">0 s</span>
        <span style="width:{(M['home_t']-M['follow_t'])/M['duration']*100:.1f}%;text-align:center">{M['follow_t']:.1f} → {M['home_t']:.1f} s</span>
        <span style="width:{(M['duration']-M['home_t'])/M['duration']*100:.1f}%;text-align:right">{M['duration']:.1f} s</span>
      </div>
    </div>
    <div class="flow" style="margin-top:14px">+{M['follow_t']:.1f}  FOLLOWING（{M['goals']} 個 goal）
+{M['home_t']:.1f}  RETURN_HOME
{stop_flow}+{M['duration']:.1f}  錄製結束</div>
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
  <svg id="cpuChart" viewBox="0 0 900 350" role="img" aria-label="調參後 CPU 使用率時間序列"></svg>
  <div class="legend">
    <span><i style="background:#3a6df0"></i>total_busy（2 s 取樣）</span>
    <span><i style="background:rgba(58,109,240,.10)"></i>FOLLOWING {M['follow_t']:.1f} ~ {M['home_t']:.1f} s</span>
    <span><i style="background:rgba(42,157,143,.10)"></i>RETURN_HOME {M['home_t']:.1f} ~ {M['stop_t']:.1f} s</span>
    <span><i style="background:#c0392b"></i>follow p95 {fol[1]:.1f}%</span>
  </div>
</div>

<table>
  <thead><tr><th>區間</th><th class="num">筆數</th><th class="num">mean</th><th class="num">p95</th><th class="num">max</th></tr></thead>
  <tbody>
    <tr><td>整段 0 ~ {M['duration']:.1f} s</td><td class="num">{M['n']}</td><td class="num">{st.mean(M['busy']):.1f}%</td><td class="num">{q(M['busy'],95):.1f}%</td><td class="num">{max(M['busy']):.1f}%</td></tr>
    <tr><td>啟動 0 ~ {M['follow_t']:.1f} s</td><td class="num">{start[3]}</td><td class="num">{start[0]:.1f}%</td><td class="num">—</td><td class="num">{start[2]:.1f}%</td></tr>
    <tr><td>跟隨 {M['follow_t']:.1f} ~ {M['home_t']:.1f} s</td><td class="num">{fol[3]}</td><td class="num">{fol[0]:.1f}%</td><td class="num"><strong>{fol[1]:.1f}%</strong></td><td class="num">{fol[2]:.1f}%</td></tr>
    <tr><td>返回 {M['home_t']:.1f} ~ {M['stop_t']:.1f} s</td><td class="num">{aft[3]}</td><td class="num">{aft[0]:.1f}%</td><td class="num">{aft[1]:.1f}%</td><td class="num">{aft[2]:.1f}%</td></tr>
  </tbody>
</table>
<p class="small">跟隨期間 mean {fol[0]:.0f}%、p95 {fol[1]:.0f}% 是這條流程「邊走邊跟」的實際負載；96% 的尖峰只出現在前 7 秒（bag discovery + 相機初始化）。回程後另有一段 89~90%（slam_toolbox loop closure / 地圖整理），與跟隨無關。</p>

<h2>3. Per-core</h2>
<p class="lead">整段每核 mean {min(st.mean(v) for v in M['cores'].values()):.0f}~{max(st.mean(v) for v in M['cores'].values()):.0f}%，單核 max {min(max(v) for v in M['cores'].values()):.0f}~{max(max(v) for v in M['cores'].values()):.0f}%。</p>
<div class="grid2">
  <div class="card">
    <h3 style="margin-top:0">本次（16:42）</h3>
    <table style="box-shadow:none;margin-top:4px">
      <thead><tr><th>Core</th><th class="num">mean</th><th class="num">max</th><th class="bars">mean（每核 0~100%）</th></tr></thead>
      <tbody>
""")
for c, v in M["cores"].items():
    w(f'<tr><td>{c}</td><td class="num">{st.mean(v):.1f}%</td><td class="num">{max(v):.1f}%</td>'
      f'<td class="bars">{bar(st.mean(v), max(v), 100)}</td></tr>')
w("""      </tbody>
    </table>
  </div>
  <div class="card">
    <h3 style="margin-top:0">RTAB-Map 15:45</h3>
    <table style="box-shadow:none;margin-top:4px">
      <thead><tr><th>Core</th><th class="num">mean</th><th class="num">max</th><th class="bars">mean（每核 0~100%）</th></tr></thead>
      <tbody>
""")
for c, v in C["cores"].items():
    w(f'<tr><td>{c}</td><td class="num">{st.mean(v):.1f}%</td><td class="num">{max(v):.1f}%</td>'
      f'<td class="bars">{bar(st.mean(v), max(v), 100)}</td></tr>')
w(f"""      </tbody>
    </table>
  </div>
</div>

<h2>4. odom 方案對照（rf2o vs icp_odometry）</h2>
<p class="lead">同一個 for_rtabmap 外框（相機 640×480@15、AI、Nav2、lite bag 都相同），只差 odom 與建圖節點；本報告是 slam_toolbox 動態建圖 + rf2o。</p>
<table>
  <thead><tr><th>項目（四核佔比）</th><th class="num">slam_toolbox + rf2o（本次）</th><th class="num">RTAB-Map + ICP（11-18）</th></tr></thead>
  <tbody>
    <tr><td>雷射 odom 節點</td><td class="num">rf2o <strong>{by['/CLaserOdometry2D']['avg']:.1f}%</strong>（max {by['/CLaserOdometry2D']['max']:.1f}）</td><td class="num">ICP <strong>11.9~14.2%</strong>（max 15~19）</td></tr>
    <tr><td>建圖節點</td><td class="num">slam_toolbox <strong>{by['/slam_toolbox']['avg']:.1f}%</strong>（max {by['/slam_toolbox']['max']:.1f}）</td><td class="num">rtabmap 4.1~4.8%（max 5~9）</td></tr>
    <tr><td>odom + 建圖合計</td><td class="num"><strong>{by['/CLaserOdometry2D']['avg']+by['/slam_toolbox']['avg']:.1f}%</strong></td><td class="num">16.0~19.0%</td></tr>
    <tr><td>整段總 CPU</td><td class="num">{st.mean(M['busy']):.1f}%</td><td class="num">69.6~72.6%</td></tr>
    <tr><td>跟隨 mean / p95</td><td class="num">{fol[0]:.1f}% / {fol[1]:.1f}%</td><td class="num">66.5~72.9% / 71.0~84.6%</td></tr>
  </tbody>
</table>
<p class="small">ICP 數據取自 11-18 14:48 / 15:31 / 15:45 三筆 run。rf2o 只做 2D 掃描匹配、沒有區域地圖與協方差；ICP 用 libpointmatcher point-to-plane 並維持區域地圖，<strong>成本約為 rf2o 的 7~8 倍</strong>。</p>

<h2>5. Node CPU — 整段（四核佔比）</h2>
<p class="lead">odom + 建圖約 {grp_rtab:.1f}%（rf2o + slam_toolbox）；相機 + AI 約 {grp_cam:.1f}%；Nav2 + 底盤約 {grp_nav:.1f}%；光達 + EKF 約 {grp_lidar:.1f}%。</p>
<div class="grid2" style="margin-bottom:16px">
  <div class="card" style="border-top:4px solid var(--accent)">
    <div class="label small">odom + 建圖</div>
    <div style="font-size:30px;font-weight:700;font-variant-numeric:tabular-nums">約 {grp_rtab:.1f}%</div>
    <div class="small">rf2o {by['/CLaserOdometry2D']['avg']:.1f} + slam_toolbox {by['/slam_toolbox']['avg']:.1f}（RTAB-Map 15:45 是 17.9%）</div>
  </div>
  <div class="card" style="border-top:4px solid var(--ai)">
    <div class="label small">AI + camera</div>
    <div style="font-size:30px;font-weight:700;font-variant-numeric:tabular-nums">約 {grp_cam:.1f}%</div>
    <div class="small">ai {by['/ai_human_detection']['avg']:.1f} + camera {by['/camera/camera']['avg']:.1f}</div>
  </div>
  <div class="card" style="border-top:4px solid #6b7484">
    <div class="label small">Nav2 + 底盤 + demo</div>
    <div style="font-size:30px;font-weight:700;font-variant-numeric:tabular-nums">約 {grp_nav:.1f}%</div>
    <div class="small">controller / planner / BT / collision / driver …</div>
  </div>
</div>
<table>
  <thead><tr><th>Node</th><th class="num">mean</th><th class="num">max</th><th class="num">RTAB-Map 15:45</th><th class="bars">mean 分布（0~{max(6.0, M['stats'][0]['avg']*1.15):.0f}%）</th></tr></thead>
  <tbody>
""")
scale = max(6.0, M["stats"][0]["avg"] * 1.15)
for s in M["stats"]:
    if s["avg"] < 0.3:
        continue
    if (s["node"] in ("process:grep", "process:tail", "process:systemd-journal")
            or "kworker" in s["node"]
            or s["node"].startswith(("process:irq/", "process:rcu", "process:ksoftirqd"))):
        continue
    old = cby.get(s["node"], {}).get("avg")
    w(f'<tr><td><code>{esc(s["label"])}</code></td><td class="num"><strong>{s["avg"]:.1f}%</strong></td>'
      f'<td class="num">{s["max"]:.1f}%</td><td class="num">{f"{old:.1f}%" if old is not None else "—"}</td>'
      f'<td class="bars">{bar(s["avg"], s["max"], scale)}</td></tr>')
w(f"""  </tbody>
</table>
<p class="small">長條為 mean、紅線為 max；最後一欄為 RTAB-Map + ICP（15:45）的同名節點，用來對照。</p>

<h2>6. 與其他 run 對照與結論</h2>
<table>
  <thead><tr><th>指標</th><th class="num">11-13 19:50</th><th class="num">11-13 19:17</th><th class="num">16:42（本次）</th><th class="num">RTAB-Map 15:45</th></tr></thead>
  <tbody>
    <tr><td>odom 節點（四核）</td><td class="num">rf2o 1.7%</td><td class="num">rf2o 1.7%</td><td class="num">rf2o {by['/CLaserOdometry2D']['avg']:.1f}%</td><td class="num">ICP 13.4%</td></tr>
    <tr><td>建圖節點</td><td class="num">2.8%</td><td class="num">3.7%</td><td class="num">{by['/slam_toolbox']['avg']:.1f}%</td><td class="num">4.5%</td></tr>
    <tr><td>整段 mean</td><td class="num">58.8%</td><td class="num">55.2%</td><td class="num">{st.mean(M['busy']):.1f}%</td><td class="num">72.1%</td></tr>
    <tr><td>跟隨 mean / p95</td><td class="num">53.5 / 65.4%</td><td class="num">50.5 / 65.2%</td><td class="num">{fol[0]:.1f} / {fol[1]:.1f}%</td><td class="num">72.2 / 82.5%</td></tr>
    <tr><td>robot path</td><td class="num">4.23 m</td><td class="num">—</td><td class="num">{M['path']:.2f} m</td><td class="num">11.18 m</td></tr>
  </tbody>
</table>
<div class="callout good">
  <strong>跟隨 {fol[0]:.1f}%（p95 {fol[1]:.1f}%），是這幾筆裡最低的。</strong>
  比 RTAB-Map + ICP（15:45）的 72.2% / 82.5% 少約 {72.2-fol[0]:.0f} 個百分點，差距幾乎全部來自 odom 節點（rf2o {by['/CLaserOdometry2D']['avg']:.1f}% vs ICP 13.4%）。
</div>
<ul class="tight">
  <li>rf2o 整段 {by['/CLaserOdometry2D']['avg']:.1f}%、跟隨期間 {M['node_follow']['/CLaserOdometry2D']:.1f}%，odom 本身幾乎不吃資源；與 11-13 兩筆完全一致。</li>
  <li>slam_toolbox 這次 {by['/slam_toolbox']['avg']:.1f}%（max {by['/slam_toolbox']['max']:.1f}%），約為 11-13（2.8~3.7%）的 2.5 倍 → 動態建新圖比載入既有圖/定位重。</li>
  <li>回程後（約 68.9 s）slam_toolbox 單筆 37.1%、總 CPU 89%，屬 loop closure / 地圖整理，與跟隨負載無關。</li>
  <li>整段 max {max(M['busy']):.1f}% 在 4.2 s（bag discovery + 相機初始化），跟隨期間最高 {fol[2]:.1f}%。</li>
</ul>

<h2>7. 建議</h2>
<ul class="tight">
  <li>相機（{by['/camera/camera']['avg']:.1f}%）+ AI（{by['/ai_human_detection']['avg']:.1f}%）現在是最大塊；AI 關閉時只留 gyro/accel 可省掉大部分。</li>
  <li>slam_toolbox 在回程的 37% 尖峰若出現在跟隨中，會拖慢 map→odom；可觀察是否與 loop closure 同步。</li>
  <li>rf2o 便宜但只做相鄰掃描匹配，長距離會累積漂移；若地圖出現重影，這是預期中的取捨（ICP 較穩但貴 7 倍）。</li>
  <li>要更乾淨的基準，可用 <code>BAG=0</code> 再跑一次，扣掉量測本身約 8%。</li>
</ul>

<h2>8. 紀錄檔</h2>
<div class="flow">/nvt_ros2_example/slam_toolbox_rf2o/monitor_logs/{MAIN}/
  cpu.csv / cpu_nodes.csv / cpu_procs.csv
  follow_target.csv   {len(M['persons'])} person 筆 · {M['goals']} goals
  RUN_INFO.txt, .env.sh
（本 run 是 slam_toolbox + rf2o，沒有 icp_odometry.log）
對照：RTAB-Map 15:45（{CMP}）· 14:48 · 15:31 · 11-13 兩筆 slam run</div>

<footer>
  slam_toolbox + rf2o CPU 分析 · {MAIN} · 2026-11-18 ·
  資料來源 <code>cpu.csv</code> / <code>cpu_nodes.csv</code> / <code>icp_odometry.log</code> · monitor_cpu.py（--interval 2）
</footer>

</div>
<script>
(function () {{
  var data = {[[round(x, 1), round(v, 1)] for x, v in zip(M['elapsed'], M['busy'])]};
  var followStart = {M['follow_t']:.3f}, followEnd = {M['home_t']:.3f}, stopEnd = {M['stop_t']:.3f};
  var p95 = {fol[1]:.1f}, peakT = {M['elapsed'][max(range(len(M['busy'])), key=lambda i: M['busy'][i])]:.1f}, peakV = {max(M['busy']):.1f};
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
  el("rect", {{ x: X(followEnd), y: m.t, width: X(stopEnd) - X(followEnd), height: ih, fill: "rgba(42,157,143,.10)" }});
  el("text", {{ x: (X(followEnd) + X(stopEnd)) / 2, y: m.t + 14, "text-anchor": "middle",
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
  var d = data.map(function (p, i) {{ return (i ? "L" : "M") + X(p[0]).toFixed(2) + " " + Y(p[1]).toFixed(2); }}).join(" ");
  el("path", {{ d: d + " L" + X(data[data.length - 1][0]).toFixed(2) + " " + Y(0) +
    " L" + X(data[0][0]).toFixed(2) + " " + Y(0) + " Z", fill: "rgba(58,109,240,.14)" }});
  el("path", {{ d: d, fill: "none", stroke: "#3a6df0", "stroke-width": "2.2",
    "stroke-linejoin": "round", "stroke-linecap": "round" }});
  el("circle", {{ cx: X(peakT), cy: Y(peakV), r: "4", fill: "#fff", stroke: "#3a6df0", "stroke-width": "2" }});
  el("text", {{ x: X(peakT), y: Y(peakV) - 12, "text-anchor": "middle", "font-size": "11",
    fill: "#1f2430", "font-weight": "600" }}, peakV.toFixed(1) + "%");
  data.forEach(function (p) {{
    el("circle", {{ cx: X(p[0]), cy: Y(p[1]), r: "1.8", fill: "#2c5fd6", opacity: ".55" }});
  }});
}})();
</script>
</body>
</html>
""")

with open(OUT, "w", encoding="utf-8") as f:
    f.write("".join(h))
print("wrote", OUT, os.path.getsize(OUT))
print(f"follow {fol[0]:.1f}/{fol[1]:.1f}/{fol[2]:.1f} n={fol[3]}; after {aft[0]:.1f}")
print(f"rf2o {by['/CLaserOdometry2D']['avg']:.1f} slam {by['/slam_toolbox']['avg']:.1f} block {grp_rtab:.1f}")

