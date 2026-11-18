#!/usr/bin/env python3
"""RTAB-Map + ICP CPU report in the style of report/person_follow_cpu_run_*.html."""
import csv
import html
import os
import statistics as st
from collections import Counter, defaultdict

BASE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(BASE, "rtabmap_icp_cpu_run_20261118-144839.html")
NCORE = 4
NOMINAL = 2.0

RUN_A = "run_20261118-144216_follow"
RUN_B = "run_20261118-144839_follow"

LABEL = {
    "process:icp_odometry": "icp_odometry",
    "/rtabmap/rtabmap": "rtabmap",
    "/camera/camera": "camera",
    "/ai_human_detection": "ai_human_detection",
    "/rplidar_node": "rplidar_node",
    "/ekf_filter_node": "ekf_node",
    "/controller_server": "controller_server",
    "/planner_server": "planner_server",
    "/bt_navigator": "bt_navigator",
    "/collision_monitor": "collision_monitor",
    "/velocity_smoother": "velocity_smoother",
    "/behavior_server": "behavior_server",
    "/driver_node": "Mcnamu_driver",
    "/demo_manager_node": "demo_manager_node",
    "/person_target_recovery_node": "person_target_recovery",
    "/keepout_container": "keepout_container",
    "/smoother_server": "smoother_server",
    "/route_server": "route_server",
    "/docking_server": "docking_server",
    "/waypoint_follower": "waypoint_follower",
    "/lifecycle_manager_navigation": "lifecycle_manager",
    "/base_to_ai_human_detection_tf": "ai TF publisher",
    "process:static_transfor": "static TF",
    "process:python3": "python3 (bag/watch/monitor)",
    "process:ros2": "ros2 launch",
    "process:grep": "grep (log watcher)",
    "process:tail": "tail (log watcher)",
    "process:systemd-journal": "systemd-journald",
}


def read(path):
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def pctl(vals, p):
    vals = sorted(vals)
    if not vals:
        return 0.0
    return vals[max(0, min(len(vals) - 1, int(round(p / 100 * (len(vals) - 1)))))]


def load(run):
    d = os.path.join(BASE, run)
    cpu = read(os.path.join(d, "cpu.csv"))
    nodes = read(os.path.join(d, "cpu_nodes.csv"))
    ft = read(os.path.join(d, "follow_target.csv"))

    ts = [float(r["t_unix"]) for r in cpu]
    gaps = [NOMINAL] + [b - a for a, b in zip(ts, ts[1:])]
    factor = [NOMINAL / g for g in gaps]
    t0 = ts[0]
    elapsed = [t - t0 for t in ts]

    series = defaultdict(dict)
    rss = defaultdict(dict)
    threads = defaultdict(dict)
    for r in nodes:
        i = ts.index(float(r["t_unix"]))
        series[r["node"]][r["t_unix"]] = series[r["node"]].get(r["t_unix"], 0.0) + \
            float(r["cpu_pct"]) * factor[i] / NCORE
        rss[r["node"]][r["t_unix"]] = max(rss[r["node"]].get(r["t_unix"], 0.0), float(r["rss_mb"]))
        threads[r["node"]][r["t_unix"]] = max(threads[r["node"]].get(r["t_unix"], 0), int(r["threads"]))

    stats = []
    for node, s in series.items():
        vals = [s.get(r["t_unix"], 0.0) for r in cpu]
        stats.append({
            "node": node, "label": LABEL.get(node, node),
            "avg": st.mean(vals), "max": max(vals), "p95": pctl(vals, 95),
            "rss": st.mean(list(rss[node].values())),
            "threads": max(threads[node].values()),
            "vals": vals,
        })
    stats.sort(key=lambda s: -s["avg"])

    busy = [float(r["total_busy"]) for r in cpu]
    cores = {c: [float(r[c]) for r in cpu] for c in ("cpu0", "cpu1", "cpu2", "cpu3")}

    # state machine
    states = [(float(r["t"]) - t0, r["note"]) for r in ft if r["kind"] == "state"]
    goals = [float(r["t"]) - t0 for r in ft if r["kind"] == "goal"]
    persons = [r for r in ft if r["kind"] == "person" and r["id"] not in ("", "-1")]

    def first(state):
        for t, s in states:
            if s == state:
                return t
        return None

    follow_t = first("FOLLOWING")
    home_t = first("RETURN_HOME")
    done_t = first("HOME")
    fail_t = first("NAVIGATION_FAILED")
    stop_t = first("STOPPED")

    # robot motion
    robot = [(float(r["t"]) - t0, float(r["x"]), float(r["y"])) for r in ft if r["kind"] == "robot"]
    path = sum(((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5 for a, b in zip(robot, robot[1:]))
    net = (((robot[-1][1] - robot[0][1]) ** 2 + (robot[-1][2] - robot[0][2]) ** 2) ** 0.5
           if robot else 0.0)
    cmds = [(float(r["t"]) - t0, float(r["vx"]), float(r["wz"])) for r in ft if r["kind"] == "cmd"]
    vx = [abs(c[1]) for c in cmds]
    wz = [abs(c[2]) for c in cmds]

    def window(a, b):
        return [(t, v) for t, v in zip(elapsed, busy) if a is not None and b is not None and a <= t < b]

    return {
        "run": run, "elapsed": elapsed, "busy": busy, "cores": cores, "stats": stats,
        "states": states, "goals": goals, "persons": persons,
        "follow_t": follow_t, "home_t": home_t, "done_t": done_t,
        "fail_t": fail_t, "stop_t": stop_t,
        "duration": elapsed[-1], "samples": len(cpu),
        "path": path, "net": net, "maxvx": max(vx) if vx else 0, "maxwz": max(wz) if wz else 0,
        "persons_t": [(float(r["t"]) - t0, r["id"], float(r["conf"]), float(r["z"])) for r in persons],
        "window": window,
        "factor_note": st.mean(factor),
    }


A = load(RUN_A)
B = load(RUN_B)


def phase_stats(run, a, b):
    v = [x for _, x in run["window"](a, b)]
    return (st.mean(v), pctl(v, 95), max(v), len(v)) if v else (0, 0, 0, 0)


def merged_stats(runs):
    acc = defaultdict(lambda: {"avg": [], "max": 0.0, "rss": [], "p95": [], "threads": 0})
    for r in runs:
        for s in r["stats"]:
            a = acc[s["node"]]
            a["label"] = s["label"]
            a["avg"].append(s["avg"])
            a["max"] = max(a["max"], s["max"])
            a["rss"].append(s["rss"])
            a["p95"].append(s["p95"])
            a["threads"] = max(a["threads"], s["threads"])
    out = []
    for node, a in acc.items():
        out.append({"node": node, "label": a["label"], "avg": st.mean(a["avg"]),
                    "max": a["max"], "p95": max(a["p95"]), "rss": st.mean(a["rss"]),
                    "threads": a["threads"]})
    out.sort(key=lambda s: -s["avg"])
    return out


M = merged_stats([A, B])
by_node = {s["node"]: s for s in M}

all_busy = A["busy"] + B["busy"]
tot_mean = st.mean(all_busy)
tot_p95 = pctl(all_busy, 95)
tot_max = max(all_busy)
sat = sum(1 for v in all_busy if v >= 90)

# run B phases
ph_idle = (8.5, B["follow_t"] or 0.0)
ph_follow = (B["follow_t"], B["home_t"])
ph_home = (B["home_t"], B["done_t"])
ph_idle2 = (B["done_t"], B["duration"] + 1)
idle_stats = phase_stats(B, *ph_idle)
follow_stats = phase_stats(B, *ph_follow)
home_stats = phase_stats(B, *ph_home)
after_stats = phase_stats(B, *ph_idle2)

foll_person = {"n": 0, "hz": 0.0, "ids": "-", "conf": (0.0, 0.0), "z": (0.0, 0.0)}
if B["follow_t"] and B["home_t"]:
    sel = [p for p in B["persons_t"] if B["follow_t"] <= p[0] < B["home_t"]]
    if sel:
        confs = [p[2] for p in sel if 0.0 < p[2] <= 1.0]
        zs = [p[3] for p in sel if -0.2 < p[3] < 2.5]
        cnt = Counter(p[1] for p in sel)
        top_id, top_n = cnt.most_common(1)[0]
        foll_person = {
            "n": len(sel),
            "hz": len(sel) / max(1e-6, B["home_t"] - B["follow_t"]),
            "ids": f"ID {top_id}（{top_n/len(sel)*100:.0f}%），共 {len(cnt)} 個",
            "conf": (pctl(confs, 5), pctl(confs, 95)) if confs else (0.0, 0.0),
            "z": (pctl(zs, 5), pctl(zs, 95)) if zs else (0.0, 0.0),
        }

grp_rtab = by_node["process:icp_odometry"]["avg"] + by_node["/rtabmap/rtabmap"]["avg"]
grp_cam = by_node["/camera/camera"]["avg"] + by_node["/ai_human_detection"]["avg"]
grp_lidar = by_node["/rplidar_node"]["avg"] + by_node["/ekf_filter_node"]["avg"]
nav_excl = {"process:icp_odometry", "/rtabmap/rtabmap", "/camera/camera", "/ai_human_detection",
            "/rplidar_node", "/ekf_filter_node", "/demo_manager_node",
            "/person_target_recovery_node", "/base_to_ai_human_detection_tf",
            "process:python3", "process:ros2", "process:static_transfor",
            "process:grep", "process:tail", "process:systemd-journal"}
grp_nav = sum(s["avg"] for s in M if s["node"] not in nav_excl)
grp_mon = sum(s["avg"] for s in M if s["node"] in
              ("process:python3", "process:grep", "process:tail"))

# ---------------------------------------------------------------- helpers
def esc(s):
    return html.escape(str(s))


def bar(avg, mx, scale):
    w = min(100.0, avg / scale * 100.0)
    m = min(100.0, mx / scale * 100.0)
    return (f'<div class="bar-track"><div class="bar-fill" style="width:{w:.1f}%"></div>'
            f'<div class="bar-max" style="left:{m:.1f}%"></div></div>')


def disp(node):
    return node[len("process:"):] if node.startswith("process:") else node


def row(node, avg, mx, scale, cls=""):
    return (f'<tr><td><code>{esc(disp(node))}</code></td><td class="num"><strong>{avg:.1f}%</strong></td>'
            f'<td class="num">{mx:.1f}%</td><td class="bars">{bar(avg, mx, scale)}</td></tr>')


# ---------------------------------------------------------------- build
h = []
w = h.append
w(f"""<!DOCTYPE html>
<html lang="zh-Hant">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RTAB-Map + ICP CPU 分析 — run_20261118-144839_follow</title>
<style>
  :root {{
    --bg:#f6f7fb; --card:#fff; --ink:#1f2430; --muted:#5b6472; --line:#e3e6ee;
    --accent:#3a6df0; --ai:#2a9d8f; --cam:#6a5acd; --warn:#b26a00; --warn-bg:#fff4e0;
    --bad:#c0392b; --good:#2a9d8f; --code:#f1f3f8;
  }}
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
  .bar-fill.ai {{ background:var(--ai); }}
  .bar-fill.cam {{ background:var(--cam); }}
  .bar-max {{ position:absolute; top:-2px; bottom:-2px; width:2px; background:rgba(192,57,43,.75); }}
  .legend {{ display:flex; flex-wrap:wrap; gap:16px; font-size:13px; color:var(--muted); margin-top:10px; }}
  .legend i {{ display:inline-block; width:12px; height:12px; border-radius:3px; margin-right:6px; vertical-align:-1px; }}
  .timeline {{ margin:16px 0 6px; }}
  .tl-track {{ display:flex; height:42px; border-radius:10px; overflow:hidden; border:1px solid var(--line); }}
  .tl-seg {{ display:flex; align-items:center; justify-content:center; font-size:11.5px; color:#fff;
    white-space:nowrap; overflow:hidden; }}
  .tl-idle {{ background:#9aa4b6; }}
  .tl-sel {{ background:#d9a441; }}
  .tl-follow {{ background:var(--accent); }}
  .tl-home {{ background:#2a9d8f; }}
  .tl-stop {{ background:#6b7484; }}
  .tl-labels {{ display:flex; font-size:12px; color:var(--muted); margin-top:6px; font-variant-numeric:tabular-nums; }}
  .grid2 {{ display:grid; gap:18px; grid-template-columns:repeat(auto-fit,minmax(340px,1fr)); }}
  .chart-box {{ background:var(--card); border:1px solid var(--line); border-radius:14px;
    padding:16px 16px 8px; box-shadow:0 2px 8px rgba(31,36,48,.04); overflow-x:auto; }}
  .chart-box svg {{ display:block; width:100%; height:auto; min-width:640px; }}
  .small {{ font-size:13px; color:var(--muted); }}
  ul.tight {{ margin:8px 0 0; padding-left:22px; }}
  ul.tight li {{ margin:5px 0; }}
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
  <h1>RTAB-Map + ICP CPU 分析</h1>
  <p>run_20261118-144839_follow（主）· run_20261118-144216_follow（對照）· 2026-11-18 · 117.8 s + 36.7 s · lite bag · 未收集 stack</p>
  <div class="pills">
    <span class="pill">for_rtabmap：rtabmap + icp_odometry</span>
    <span class="pill">D455 640×480 @ 15</span>
    <span class="pill">4 核 · node CPU 已換算四核佔比</span>
    <span class="pill good">跟隨 mean {follow_stats[0]:.1f}%</span>
    <span class="pill warn">待機基準 {idle_stats[0]:.1f}%</span>
  </div>
</header>

<div class="kpis">
  <div class="kpi">
    <div class="label">跟隨期間 mean</div>
    <div class="value">{follow_stats[0]:.1f}%</div>
    <div class="sub">{B['follow_t']:.1f} ~ {B['home_t']:.1f} s（{follow_stats[3]} 筆取樣）</div>
  </div>
  <div class="kpi">
    <div class="label">跟隨期間 p95</div>
    <div class="value">{follow_stats[1]:.1f}%</div>
    <div class="sub">max {follow_stats[2]:.1f}%</div>
  </div>
  <div class="kpi warn">
    <div class="label">待機基準（未移動）</div>
    <div class="value">{idle_stats[0]:.1f}%</div>
    <div class="sub">8.5 ~ {B['follow_t']:.1f} s，{idle_stats[3]} 筆</div>
  </div>
  <div class="kpi warn">
    <div class="label">整段 max</div>
    <div class="value">{tot_max:.1f}%</div>
    <div class="sub">{sat} / {len(all_busy)} 筆 ≥ 90%</div>
  </div>
</div>
<p class="small" style="margin-top:12px">total_busy 是四核整體使用率（0~100%）；node / process 數值已由單核換算為四核佔比（單核 ÷ 4，1 核滿載 = 25%）。process 數值另以 <code>名目 2.0 s ÷ 實際取樣間隔</code> 修正，否則加總會比 <code>/proc/stat</code> 高約 6 個百分點。</p>

<h2>1. Run 概況</h2>
<div class="grid2">
  <div class="card">
    <h3 style="margin-top:0">狀態時間線（run_20261118-144839）</h3>
    <div class="timeline">
      <div class="tl-track">
        <div class="tl-seg tl-idle" style="width:{B['follow_t']/B['duration']*100:.1f}%">IDLE / SELECT</div>
        <div class="tl-seg tl-follow" style="width:{(B['home_t']-B['follow_t'])/B['duration']*100:.1f}%">FOLLOWING</div>
        <div class="tl-seg tl-home" style="width:{(B['done_t']-B['home_t'])/B['duration']*100:.1f}%">RETURN_HOME</div>
        <div class="tl-seg tl-stop" style="width:{(B['duration']-B['done_t'])/B['duration']*100:.1f}%">HOME</div>
      </div>
      <div class="tl-labels">
        <span style="width:{B['follow_t']/B['duration']*100:.1f}%;text-align:left">0 s</span>
        <span style="width:{(B['home_t']-B['follow_t'])/B['duration']*100:.1f}%;text-align:center">{B['follow_t']:.1f} → {B['home_t']:.1f} s</span>
        <span style="width:{(B['duration']-B['home_t'])/B['duration']*100:.1f}%;text-align:right">{B['duration']:.1f} s</span>
      </div>
    </div>
    <div class="flow" style="margin-top:14px">+10.9  TARGET_SELECTED
+16.7  FOLLOWING（51 個 goal）
+58.0  RETURN_HOME
+103.1 HOME（錄製仍在進行）
+117.8 錄製結束</div>
  </div>
  <div class="card">
    <h3 style="margin-top:0">跟隨統計</h3>
    <table style="margin-top:4px;box-shadow:none">
      <tbody>
        <tr><td>跟隨時間</td><td class="num">{B['home_t']-B['follow_t']:.1f} s</td></tr>
        <tr><td>goals</td><td class="num">{len(B['goals'])}</td></tr>
        <tr><td>person（跟隨期間）</td><td class="num">{foll_person['n']} 筆 · {foll_person['hz']:.1f} Hz</td></tr>
        <tr><td>person ID</td><td class="num">{foll_person['ids']}</td></tr>
        <tr><td>confidence / z（p5~p95）</td><td class="num">{foll_person['conf'][0]:.2f}~{foll_person['conf'][1]:.2f} / {foll_person['z'][0]:.2f}~{foll_person['z'][1]:.2f} m</td></tr>
        <tr><td>robot（整段）</td><td class="num">net {B['net']:.2f} m / path {B['path']:.2f} m</td></tr>
        <tr><td>max |vx| / |wz|（cmd）</td><td class="num">{B['maxvx']:.2f} m/s / {B['maxwz']:.2f} rad/s</td></tr>
      </tbody>
    </table>
  </div>
</div>

<div class="callout">
  <strong>待機就要 {idle_stats[0]:.1f}%，跟隨期間 {follow_stats[0]:.1f}%。</strong>
  這條流程的固定成本很高：機器人還沒開始跟隨（0 ~ {B['follow_t']:.1f} s）就已經用掉四核約 {idle_stats[0]:.0f}%，
  跟隨只再增加約 {follow_stats[0]-idle_stats[0]:.1f} 個百分點。降 CPU 要從「常駐節點」下手，而不是跟隨邏輯。
</div>

<h2>2. 整體 CPU</h2>
<div class="chart-box">
  <svg id="cpuChart" viewBox="0 0 900 350" role="img" aria-label="run_20261118-144839 CPU 使用率時間序列"></svg>
  <div class="legend">
    <span><i style="background:#3a6df0"></i>total_busy（2 s 取樣）</span>
    <span><i style="background:rgba(58,109,240,.10)"></i>FOLLOWING {B['follow_t']:.1f} ~ {B['home_t']:.1f} s</span>
    <span><i style="background:rgba(42,157,143,.10)"></i>RETURN_HOME {B['home_t']:.1f} ~ {B['done_t']:.1f} s</span>
    <span><i style="background:#d9a441"></i>啟動 0 ~ 8.5 s</span>
    <span><i style="background:#c0392b"></i>follow p95 {follow_stats[1]:.1f}%</span>
  </div>
</div>

<table>
  <thead><tr><th>區間</th><th class="num">筆數</th><th class="num">mean</th><th class="num">p95</th><th class="num">max</th></tr></thead>
  <tbody>
    <tr><td>整段 0 ~ {B['duration']:.1f} s</td><td class="num">{B['samples']}</td><td class="num">{st.mean(B['busy']):.1f}%</td><td class="num">{pctl(B['busy'],95):.1f}%</td><td class="num">{max(B['busy']):.1f}%</td></tr>
    <tr><td>啟動 0 ~ 8.5 s（bag discovery）</td><td class="num">{phase_stats(B,0,8.5)[3]}</td><td class="num">{phase_stats(B,0,8.5)[0]:.1f}%</td><td class="num">—</td><td class="num">{phase_stats(B,0,8.5)[2]:.1f}%</td></tr>
    <tr><td>待機 / 選人 8.5 ~ {B['follow_t']:.1f} s</td><td class="num">{idle_stats[3]}</td><td class="num">{idle_stats[0]:.1f}%</td><td class="num">—</td><td class="num">{idle_stats[2]:.1f}%</td></tr>
    <tr><td>跟隨 {B['follow_t']:.1f} ~ {B['home_t']:.1f} s</td><td class="num">{follow_stats[3]}</td><td class="num">{follow_stats[0]:.1f}%</td><td class="num"><strong>{follow_stats[1]:.1f}%</strong></td><td class="num">{follow_stats[2]:.1f}%</td></tr>
    <tr><td>返回 {B['home_t']:.1f} ~ {B['done_t']:.1f} s</td><td class="num">{home_stats[3]}</td><td class="num">{home_stats[0]:.1f}%</td><td class="num">{home_stats[1]:.1f}%</td><td class="num">{home_stats[2]:.1f}%</td></tr>
    <tr><td>回到原點後 {B['done_t']:.1f} ~ {B['duration']:.1f} s</td><td class="num">{after_stats[3]}</td><td class="num">{after_stats[0]:.1f}%</td><td class="num">—</td><td class="num">{after_stats[2]:.1f}%</td></tr>
  </tbody>
</table>
<p class="small">「待機 / 選人」期間機器人沒有移動，只有 stack 本身在跑，因此可視為這條流程的 CPU 地板。</p>

<div class="callout">
  <strong>整段 max {tot_max:.1f}%（兩次 run 合計）都出現在啟動前 8 秒。</strong>
  bag 訂閱 discovery 讓 <code>ros2 bag record</code>（python3 wrapper）衝高，相機同時初始化，
  加上 RTAB-Map 載入 DB 與建立地圖；跟隨開始後就回到 60 ~ 86%。
</div>

<h2>3. Per-core</h2>
<p class="lead">run_20261118-144839 整段每核 mean {min(st.mean(v) for v in B['cores'].values()):.0f}~{max(st.mean(v) for v in B['cores'].values()):.0f}%、max {min(max(v) for v in B['cores'].values()):.0f}~{max(max(v) for v in B['cores'].values()):.0f}%。（此區為每顆核心各自的使用率，100% = 該核滿載。）</p>
<div class="grid2">
  <div class="card">
    <h3 style="margin-top:0">run_20261118-144839（117.8 s）</h3>
    <table style="box-shadow:none;margin-top:4px">
      <thead><tr><th>Core</th><th class="num">mean</th><th class="num">max</th><th class="bars">mean（每核 0~100%）</th></tr></thead>
      <tbody>
""")
for c, v in B["cores"].items():
    w(f'<tr><td>{c}</td><td class="num">{st.mean(v):.1f}%</td><td class="num">{max(v):.1f}%</td>'
      f'<td class="bars">{bar(st.mean(v), max(v), 100)}</td></tr>')
w(f"""      </tbody>
    </table>
  </div>
  <div class="card">
    <h3 style="margin-top:0">run_20261118-144216（36.7 s）</h3>
    <table style="box-shadow:none;margin-top:4px">
      <thead><tr><th>Core</th><th class="num">mean</th><th class="num">max</th><th class="bars">mean（每核 0~100%）</th></tr></thead>
      <tbody>
""")
for c, v in A["cores"].items():
    w(f'<tr><td>{c}</td><td class="num">{st.mean(v):.1f}%</td><td class="num">{max(v):.1f}%</td>'
      f'<td class="bars">{bar(st.mean(v), max(v), 100)}</td></tr>')
w("""      </tbody>
    </table>
  </div>
</div>

<h2>4. Node CPU — 整段（四核佔比）</h2>
""")
w(f"""<p class="lead">RTAB-Map 本體 + ICP 合計約 {grp_rtab:.1f}%；相機 + AI 約 {grp_cam:.1f}%；Nav2 + 底盤約 {grp_nav:.1f}%。</p>
<p class="small">node / process 數值已由單核換算為<strong>四核總容量佔比</strong>（單核 ÷ 4）；1 核滿載 = 25%。</p>

<div class="grid2" style="margin-bottom:16px">
  <div class="card" style="border-top:4px solid var(--accent)">
    <div class="label small">RTAB-Map + ICP</div>
    <div style="font-size:30px;font-weight:700;font-variant-numeric:tabular-nums">約 {grp_rtab:.1f}%</div>
    <div class="small">icp_odometry {by_node['process:icp_odometry']['avg']:.1f} + rtabmap {by_node['/rtabmap/rtabmap']['avg']:.1f}</div>
  </div>
  <div class="card" style="border-top:4px solid var(--ai)">
    <div class="label small">AI + camera</div>
    <div style="font-size:30px;font-weight:700;font-variant-numeric:tabular-nums">約 {grp_cam:.1f}%</div>
    <div class="small">ai {by_node['/ai_human_detection']['avg']:.1f} + camera {by_node['/camera/camera']['avg']:.1f}</div>
  </div>
  <div class="card" style="border-top:4px solid #6b7484">
    <div class="label small">Nav2 + 底盤 + demo</div>
    <div style="font-size:30px;font-weight:700;font-variant-numeric:tabular-nums">約 {grp_nav:.1f}%</div>
    <div class="small">controller / planner / BT / collision / driver …</div>
  </div>
</div>

<table>
  <thead><tr><th>Node</th><th class="num">mean</th><th class="num">max</th><th class="bars">mean 分布（0~{max(6.0, M[0]['avg']*1.15):.0f}%）</th></tr></thead>
  <tbody>
""")
scale = max(6.0, M[0]["avg"] * 1.15)
for s in M:
    if s["avg"] < 0.3:
        continue
    if (s["node"] in ("process:grep", "process:tail", "process:systemd-journal")
            or "kworker" in s["node"]
            or s["node"].startswith(("process:irq/", "process:rcu", "process:ksoftirqd"))):
        continue  # kernel threads / log watchers -> section 5
    w(row(s["node"], s["avg"], s["max"], scale))
w(f"""  </tbody>
</table>
<p class="small">長條為 mean，紅線為 max；CPU% 是 process 等級的 interval delta，一個 process 內含多個 node 時為合計值。
RTAB-Map 與 ICP 是兩個獨立 process（<code>rtabmap_slam/rtabmap</code>、<code>rtabmap_odom/icp_odometry</code>）。
kernel thread 與 log watcher 不列在此表。</p>

<h2>5. Monitor / 非 node process（四核佔比）</h2>
<p class="lead">量測本身約四核 {grp_mon:.1f}%（<code>python3</code> {by_node['process:python3']['avg']:.1f}% + log watcher）；<code>ros2 launch</code> 的 {by_node['process:ros2']['avg']:.1f}% 屬於堆疊啟動器，不算量測成本。</p>
<table>
  <thead><tr><th>Process</th><th>說明</th><th class="num">mean（四核）</th><th class="num">max</th></tr></thead>
  <tbody>
    <tr><td><code>python3</code></td><td>ros2 bag record + follow_watch.py + monitor_cpu.py</td><td class="num">{by_node['process:python3']['avg']:.1f}%</td><td class="num">{by_node['process:python3']['max']:.1f}%</td></tr>
    <tr><td><code>ros2</code></td><td>ros2 launch wrappers（rtabmap / nav2 / ekf …）— 堆疊成本</td><td class="num">{by_node['process:ros2']['avg']:.1f}%</td><td class="num">{by_node['process:ros2']['max']:.1f}%</td></tr>
    <tr><td><code>static_transfor</code></td><td>static_transform_publisher</td><td class="num">{by_node['process:static_transfor']['avg']:.1f}%</td><td class="num">{by_node['process:static_transfor']['max']:.1f}%</td></tr>
    <tr><td><code>grep</code> / <code>tail</code></td><td>每秒掃 AI log 的 watcher</td><td class="num">{by_node.get('process:grep',{'avg':0})['avg']+by_node.get('process:tail',{'avg':0})['avg']:.1f}%</td><td class="num">{max(by_node.get('process:grep',{'max':0})['max'],by_node.get('process:tail',{'max':0})['max']):.1f}%</td></tr>
    <tr><td><strong>monitor 合計</strong></td><td>python3（bag + follow_watch + cpu monitor）+ grep/tail</td><td class="num"><strong>{grp_mon:.1f}%</strong></td><td class="num">—</td></tr>
  </tbody>
</table>
<p class="small"><code>cpu_procs.csv</code> 只記 process 名稱，多個 python3 行程（bag、follow_watch、monitor）無法拆分，因此合併呈現。</p>

<h2>6. 與另一筆 run（14:42）比較</h2>
<table>
  <thead><tr><th>指標</th><th class="num">14:42（36.7 s）</th><th class="num">14:48（本次）</th><th>說明</th></tr></thead>
  <tbody>
    <tr><td>狀態</td><td class="num">NAVIGATION_FAILED → STOPPED</td><td class="num">FOLLOWING → HOME</td><td>14:42 只有 1 個 goal 就失敗</td></tr>
    <tr><td>整段 mean（四核）</td><td class="num">{st.mean(A['busy']):.1f}%</td><td class="num">{st.mean(B['busy']):.1f}%</td><td>—</td></tr>
    <tr><td>整段 p95</td><td class="num">{pctl(A['busy'],95):.1f}%</td><td class="num">{pctl(B['busy'],95):.1f}%</td><td>都被啟動段拉高</td></tr>
    <tr><td>整段 max</td><td class="num">{max(A['busy']):.1f}%</td><td class="num">{max(B['busy']):.1f}%</td><td>—</td></tr>
    <tr><td>icp_odometry（四核）</td><td class="num">{next(s['avg'] for s in A['stats'] if s['node']=='process:icp_odometry'):.1f}%</td><td class="num">{by_node['process:icp_odometry']['avg']:.1f}%</td><td>最大單一消耗</td></tr>
    <tr><td>rtabmap（四核）</td><td class="num">{next(s['avg'] for s in A['stats'] if s['node']=='/rtabmap/rtabmap'):.1f}%</td><td class="num">{by_node['/rtabmap/rtabmap']['avg']:.1f}%</td><td>—</td></tr>
    <tr><td>camera（四核）</td><td class="num">{next(s['avg'] for s in A['stats'] if s['node']=='/camera/camera'):.1f}%</td><td class="num">{by_node['/camera/camera']['avg']:.1f}%</td><td>持平</td></tr>
    <tr><td>AI（四核）</td><td class="num">{next(s['avg'] for s in A['stats'] if s['node']=='/ai_human_detection'):.1f}%</td><td class="num">{by_node['/ai_human_detection']['avg']:.1f}%</td><td>—</td></tr>
  </tbody>
</table>

<h2>7. 結論</h2>
<div class="callout">
  <strong>跟隨期間 mean {follow_stats[0]:.1f}%、p95 {follow_stats[1]:.1f}%、max {follow_stats[2]:.1f}%。</strong>
  沒有飽和，但<strong>待機地板已達 {idle_stats[0]:.1f}%</strong>，整段平均 {tot_mean:.1f}% 等於四核 2.9 顆。
</div>
<ul class="tight">
  <li>啟動 0~8.5 s 衝到 {phase_stats(B,0,8.5)[2]:.1f}%（bag discovery + 相機初始化 + RTAB-Map 載入 DB），這是整段 max 的來源。</li>
  <li><code>icp_odometry</code> 是最大單一消耗（{by_node['process:icp_odometry']['avg']:.1f}%，max {by_node['process:icp_odometry']['max']:.1f}%），
      約為 rtabmap 本體（{by_node['/rtabmap/rtabmap']['avg']:.1f}%）的 {by_node['process:icp_odometry']['avg']/max(0.1,by_node['/rtabmap/rtabmap']['avg']):.1f} 倍。</li>
  <li>相機（{by_node['/camera/camera']['avg']:.1f}%）+ AI（{by_node['/ai_human_detection']['avg']:.1f}%）是第二大塊；
      純光達建圖其實不需要相機，它只是為了 EKF 的 gyro 與人物偵測。</li>
  <li>Nav2 + 底盤 + demo 合計 {grp_nav:.1f}%，分散在十幾個 process；collision monitor 與 keepout 佔一部分。</li>
  <li>量測本身約 {grp_mon:.1f}%（bag + follow_watch + log watcher），要評估純堆疊請用 <code>BAG=0</code> 再跑一次。</li>
</ul>

<h2>8. 建議</h2>
<ul class="tight">
  <li>先量基準：<code>BAG=0</code> 再跑一次，扣掉約 {grp_mon:.1f}% 的量測成本後，堆疊實際約 {tot_mean-grp_mon:.0f}%。</li>
  <li>降 ICP：調 <code>Icp/MaxCorrespondenceDistance</code>、<code>Icp/CorrespondenceRatio</code>，或限制 scan 角度範圍；
      ICP 每降 1 個百分點，等於省下約 0.04 顆核心。</li>
  <li>關掉不需要的相機串流：若 AI 關閉，只留 gyro/accel（<code>enable_color:=false enable_depth:=false</code>），可省下相機大部分 {by_node['/camera/camera']['avg']:.1f}%。</li>
  <li>關掉未使用的 Nav2 node：<code>docking_server</code>、<code>route_server</code>、<code>smoother_server</code>、<code>waypoint_follower</code>。</li>
  <li>建圖不需要 bag 時不要錄：<code>BAG=0</code> 可省掉 <code>python3</code> 這 {by_node['process:python3']['avg']:.1f}% 的大部分（先前量測 bag 本身約 2~3 個百分點）。</li>
  <li>待機地板 {idle_stats[0]:.0f}% 偏高；若之後要同時跑其他服務，建議把 <code>icp_odometry</code> 與 <code>rtabmap</code> 綁在不同核心（taskset），避免搶同一核。</li>
</ul>

<h2>9. 紀錄檔</h2>
<div class="flow">/nvt_ros2_example/slam_toolbox_rf2o/monitor_logs/
  run_20261118-144216_follow/     36.7 s · 18 samples（NAVIGATION_FAILED）
  run_20261118-144839_follow/    117.8 s · 57 samples（FOLLOWING → HOME）
    bag/                 lite（mcap）
    cpu.csv              total_busy + cpu0..cpu3
    cpu_procs.csv        per-PID CPU / RSS
    cpu_nodes.csv        per-node CPU / RSS / threads
    follow_target.csv    robot / person / goal / state
    RUN_INFO.txt, .env.sh</div>

<p class="small" style="margin-top:14px">
  產生器：<code>monitor_logs/make_report_style.py</code>（讀原始 CSV，無平滑）<br>
  對照報告：<code>report/person_follow_cpu_run_20261113-195027.html</code>（同版型，slam_toolbox + rf2o）
</p>

<footer>
  RTAB-Map + ICP CPU 分析 · run_20261118-144839_follow · 2026-11-18 ·
  資料來源 <code>cpu.csv</code> / <code>cpu_nodes.csv</code> / <code>cpu_procs.csv</code> / <code>follow_target.csv</code> · monitor_cpu.py（--interval 2）
</footer>

</div>
<script>
(function () {{
  var data = {[[round(x, 1), round(v, 1)] for x, v in zip(B['elapsed'], B['busy'])]};
  var followStart = {B['follow_t']:.3f}, followEnd = {B['home_t']:.3f}, homeEnd = {B['done_t']:.3f};
  var p95 = {follow_stats[1]:.1f}, peakT = {B['elapsed'][B['busy'].index(B['busy'])] if False else B['elapsed'][max(range(len(B['busy'])), key=lambda i: B['busy'][i])]:.1f}, peakV = {max(B['busy']):.1f};
  var W = 900, H = 350;
  var m = {{ l: 54, r: 24, t: 24, b: 42 }};
  var iw = W - m.l - m.r, ih = H - m.t - m.b;
  var xMax = {int(B['duration'] // 10 * 10 + 10)}, yMax = 100;
  function X(t) {{ return m.l + t / xMax * iw; }}
  function Y(v) {{ return m.t + (1 - v / yMax) * ih; }}
  var NS = "http://www.w3.org/2000/svg";
  var svg = document.getElementById("cpuChart");
  function el(name, attrs, text) {{
    var e = document.createElementNS(NS, name);
    for (var k in attrs) e.setAttribute(k, attrs[k]);
    if (text != null) e.textContent = text;
    svg.appendChild(e);
    return e;
  }}

  el("rect", {{ x: X(0), y: m.t, width: X(8.5) - X(0), height: ih, fill: "rgba(217,164,65,.10)" }});
  el("line", {{ x1: X(8.5), y1: m.t, x2: X(8.5), y2: m.t + ih, stroke: "#d9a441",
    "stroke-width": "1", "stroke-dasharray": "4 4", opacity: ".8" }});
  el("text", {{ x: X(4.2), y: m.t + 14, "text-anchor": "middle", "font-size": "11",
    fill: "#9a6a12", "font-weight": "600" }}, "啟動");

  el("rect", {{ x: X(followStart), y: m.t, width: X(followEnd) - X(followStart), height: ih, fill: "rgba(58,109,240,.10)" }});
  el("text", {{ x: (X(followStart) + X(followEnd)) / 2, y: m.t + 14, "text-anchor": "middle",
    "font-size": "11.5", fill: "#3a6df0", "font-weight": "600", "letter-spacing": ".6" }}, "FOLLOWING");

  el("rect", {{ x: X(followEnd), y: m.t, width: X(homeEnd) - X(followEnd), height: ih, fill: "rgba(42,157,143,.10)" }});
  el("text", {{ x: (X(followEnd) + X(homeEnd)) / 2, y: m.t + 14, "text-anchor": "middle",
    "font-size": "11.5", fill: "#2a9d8f", "font-weight": "600", "letter-spacing": ".6" }}, "RETURN_HOME");

  [0, 25, 50, 75, 100].forEach(function (v) {{
    el("line", {{ x1: m.l, y1: Y(v), x2: m.l + iw, y2: Y(v), stroke: "#e3e6ee", "stroke-width": v === 0 ? "1.4" : "1" }});
    el("text", {{ x: m.l - 9, y: Y(v) + 4, "text-anchor": "end", "font-size": "11", fill: "#7b8494" }}, v + "%");
  }});

  for (var t = 0; t <= xMax; t += 20) {{
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
print(f"follow mean {follow_stats[0]:.1f} p95 {follow_stats[1]:.1f} max {follow_stats[2]:.1f} n={follow_stats[3]}")
print(f"idle mean {idle_stats[0]:.1f} n={idle_stats[3]} home {home_stats[0]:.1f} after {after_stats[0]:.1f}")
print(f"groups rtab {grp_rtab:.1f} cam {grp_cam:.1f} nav {grp_nav:.1f} mon {grp_mon:.1f}")
