#!/usr/bin/env python3
"""產生 RTAB-Map + ICP 的 Markdown 效能報告（run_20261118-154507）。"""
import csv
import os
import re
import statistics as st
from collections import defaultdict

BASE = os.path.dirname(os.path.abspath(__file__))
RUN = "run_20261118-154507_follow"
OUT = os.path.join(BASE, "rtabmap_icp_cpu_run_20261118-154507.md")

# 前兩次 run 的已知數值（同一個 monitor 格式，已於當次分析取得）
PREV = {
    "14:48（調參前）": dict(run="run_20261118-144839_follow", dur=117.8, n=57, mean=72.6, p95=92.7,
                        mx=94.0, fol_mean=72.9, fol_p95=84.6, fol_max=85.7, icp=13.2,
                        icp_hz=7.4, icp_upd=(49, 129), icp_ratio=(0.615, 0.798), drops=4.4,
                        path=20.81, goals=51),
    "15:31（調參後）": dict(run="run_20261118-153150_follow", dur=65.1, n=32, mean=69.6, p95=92.3,
                        mx=94.5, fol_mean=66.5, fol_p95=71.0, fol_max=83.3, icp=11.9,
                        icp_hz=9.74, icp_upd=(44, 69), icp_ratio=(0.622, 0.676), drops=0.6,
                        path=15.18, goals=48),
}


def read(p):
    with open(p, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def q(v, p):
    v = sorted(v)
    return v[max(0, min(len(v) - 1, int(round(p / 100 * (len(v) - 1)))))] if v else 0.0


def icp_metrics(path):
    if not os.path.exists(path):
        return None
    txt = open(path, errors="ignore").read()
    ratio = [float(x) for x in re.findall(r"ratio=([0-9.]+)", txt)]
    upd = [float(x) for x in re.findall(r"update time=([0-9.]+)s", txt)]
    dly = [float(x) for x in re.findall(r"delay=([0-9.]+)", txt)]
    stamps = [float(x) for x in re.findall(r"\[(\d{10}\.\d+)\]", txt)]
    span = max(stamps) - min(stamps) if stamps else 0
    return dict(n=len(ratio), span=span, hz=len(ratio) / span if span else 0,
                ratio=(q(ratio, 5), q(ratio, 50), max(ratio)),
                upd=(q(upd, 50) * 1000, q(upd, 95) * 1000, max(upd) * 1000),
                dly=(q(dly, 50) * 1000, q(dly, 95) * 1000),
                drops=txt.count("Dropping image/scan"),
                range_max=(re.findall(r"scan_range_max\s*=\s*([0-9.]+)", txt) or ["?"])[0])


d = os.path.join(BASE, RUN)
cpu, nodes = read(os.path.join(d, "cpu.csv")), read(os.path.join(d, "cpu_nodes.csv"))
ft = read(os.path.join(d, "follow_target.csv"))
ts = [float(r["t_unix"]) for r in cpu]
gaps = [2.0] + [b - a for a, b in zip(ts, ts[1:])]
fac = [2.0 / g for g in gaps]
t0 = ts[0]
dur = ts[-1] - t0

series, rss = defaultdict(dict), defaultdict(dict)
for r in nodes:
    i = ts.index(float(r["t_unix"]))
    series[r["node"]][r["t_unix"]] = series[r["node"]].get(r["t_unix"], 0.0) + \
        float(r["cpu_pct"]) * fac[i] / 4
    rss[r["node"]][r["t_unix"]] = max(rss[r["node"]].get(r["t_unix"], 0.0), float(r["rss_mb"]))

stats = [dict(node=n, avg=st.mean([s.get(r["t_unix"], 0.0) for r in cpu]),
              mx=max([s.get(r["t_unix"], 0.0) for r in cpu]),
              rss=st.mean(list(rss[n].values()))) for n, s in series.items()]
stats.sort(key=lambda s: -s["avg"])
avg = {s["node"]: s["avg"] for s in stats}

busy = [float(r["total_busy"]) for r in cpu]
states = [(float(r["t"]) - t0, r["note"]) for r in ft if r["kind"] == "state"]
follow_t = next(t for t, s in states if s == "FOLLOWING")
home_t = next(t for t, s in states if s == "RETURN_HOME")
stop_t = next(t for t, s in states if s == "STOPPED")


def phase(a, b):
    v = [float(r["total_busy"]) for r in cpu if a <= float(r["t_unix"]) - t0 < b]
    return (st.mean(v), q(v, 95), max(v), len(v)) if v else (0.0, 0.0, 0.0, 0)


fol = phase(follow_t, home_t)
aft = phase(home_t, stop_t)
goals = sum(1 for r in ft if r["kind"] == "goal")
persons = sum(1 for r in ft if r["kind"] == "person")
rob = [(float(r["x"]), float(r["y"])) for r in ft if r["kind"] == "robot"]
path = sum(((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5 for a, b in zip(rob, rob[1:]))
net = ((rob[-1][0] - rob[0][0]) ** 2 + (rob[-1][1] - rob[0][1]) ** 2) ** 0.5
cmds = [(abs(float(r["vx"])), abs(float(r["wz"]))) for r in ft if r["kind"] == "cmd"]
ICP = icp_metrics(os.path.join(d, "icp_odometry.log"))

grp = {
    "RTAB-Map + ICP": avg["process:icp_odometry"] + avg["/rtabmap/rtabmap"],
    "相機 + AI": avg["/camera/camera"] + avg["/ai_human_detection"],
    "光達 + EKF": avg["/rplidar_node"] + avg["/ekf_filter_node"],
    "量測（bag + watch + monitor）": sum(v for n, v in avg.items()
                                   if n in ("process:python3", "process:grep", "process:tail")),
    "ros2 launch wrapper": avg["process:ros2"],
}
excl = {"process:icp_odometry", "/rtabmap/rtabmap", "/camera/camera", "/ai_human_detection",
        "/rplidar_node", "/ekf_filter_node", "/demo_manager_node",
        "/person_target_recovery_node", "/base_to_ai_human_detection_tf",
        "process:python3", "process:ros2", "process:static_transfor"}
grp["Nav2 + 底盤 + demo"] = sum(v for n, v in avg.items() if n not in excl)

L = []
w = L.append
w("# RTAB-Map + ICP 效能報告 — run_20261118-154507")
w("")
w("> 調參後第二次 follow run（59 s）：整段 CPU 72.1%、跟隨 72.2%，ICP 13.4%（四核佔比）。")
w(">")
w("> 撰寫日期：2026-11-18 · 板子 `komugi`（Jetson 4 核）· ROS 2 Jazzy · `for_rtabmap` 流程")
w(">")
w("> 數值皆為 **[實測]**：`monitor_logs/run_20261118-154507_follow/` 的 `cpu.csv` / `cpu_nodes.csv` / "
  "`follow_target.csv`，以及同一次啟動的 `icp_odometry.log`。")
w(">")
w("> CPU 一律換算成**四核整體佔比**（100% = 四核全滿，單核滿載 = 25%）；process 數值另以 "
  "`名目 2.0 s ÷ 實際取樣間隔` 修正。")
w("")
w("---")
w("")
w("## 0. TL;DR")
w("")
w(f"- 整段 **{st.mean(busy):.1f}%**（p95 {q(busy,95):.1f}%、max {max(busy):.1f}%），"
  f"{sum(1 for v in busy if v>=90)}/{len(busy)} 筆 ≥ 90%，全部集中在啟動段。")
w(f"- 跟隨期間（{follow_t:.1f}~{home_t:.1f} s）**{fol[0]:.1f}%**，p95 {fol[1]:.1f}%，max {fol[2]:.1f}%。")
w(f"- 最大節點仍是 **icp_odometry {avg['process:icp_odometry']:.1f}%**，"
  f"其次 camera {avg['/camera/camera']:.1f}%、python3（bag/watch）{avg['process:python3']:.1f}%。")
w(f"- ICP 本體：**{ICP['hz']:.2f} Hz**、update time 中位 {ICP['upd'][0]:.0f} ms / p95 {ICP['upd'][1]:.0f} ms、"
  f"ratio 中位 {ICP['ratio'][1]:.3f}、丟幀 {ICP['drops']} 次 / {ICP['span']/60:.1f} 分"
  f"（{ICP['drops']/(ICP['span']/60):.1f} 次/分）。")
w("- 這次比 15:31 那筆差（跟隨 p95 71.0 → 82.5%），但仍在 14:48（調參前）的水準附近；"
  "短 run（59 s、13 筆跟隨取樣）變異很大，不宜用單筆定調。")
w("")
w("## 1. 這次 run 的設定與狀態")
w("")
w("| 項目 | 值 |")
w("|---|---|")
w(f"| run | `{RUN}` |")
w(f"| 時長 / 取樣 | {dur:.1f} s / {len(cpu)} 筆（2 s） |")
w(f"| 流程 | for_rtabmap：rtabmap + icp_odometry + EKF + Nav2 + AI |")
w(f"| ICP 參數 | `RangeMax=8`、`Force4DoF=true`、`CorrespondenceRatio=0.5`、`MaxCorrDist=0.10`、`MaxTranslation=0.20`、`Iterations=20` |")
w(f"| 狀態 | FOLLOWING {follow_t:.1f} s → RETURN_HOME {home_t:.1f} s → STOPPED {stop_t:.1f} s |")
w(f"| goals / person 訊息 | {goals} / {persons} |")
w(f"| robot 移動 | net {net:.2f} m / path {path:.2f} m |")
w(f"| max \\|vx\\| / \\|wz\\| | {max(c[0] for c in cmds):.2f} m/s / {max(c[1] for c in cmds):.2f} rad/s |")
w("")
w("## 2. CPU 總覽")
w("")
w("| 區間 | 筆數 | mean | p95 | max |")
w("|---|---:|---:|---:|---:|")
w(f"| 整段 0~{dur:.1f} s | {len(cpu)} | {st.mean(busy):.1f}% | {q(busy,95):.1f}% | {max(busy):.1f}% |")
w(f"| 啟動 0~{follow_t:.1f} s | {phase(0,follow_t)[3]} | {phase(0,follow_t)[0]:.1f}% | — | {phase(0,follow_t)[2]:.1f}% |")
w(f"| 跟隨 {follow_t:.1f}~{home_t:.1f} s | {fol[3]} | {fol[0]:.1f}% | **{fol[1]:.1f}%** | {fol[2]:.1f}% |")
w(f"| 返回 {home_t:.1f}~{stop_t:.1f} s | {aft[3]} | {aft[0]:.1f}% | {aft[1]:.1f}% | {aft[2]:.1f}% |")
w("")
w("### Per-core（每核 100% = 該核滿載）")
w("")
w("| Core | mean | max |")
w("|---|---:|---:|")
for c in ("cpu0", "cpu1", "cpu2", "cpu3"):
    v = [float(r[c]) for r in cpu]
    w(f"| {c} | {st.mean(v):.1f}% | {max(v):.1f}% |")
w("")
w("## 3. 節點排行（整段，四核佔比）")
w("")
w("| 節點 | mean | max | RSS |")
w("|---|---:|---:|---:|")
for s in stats:
    if s["avg"] < 0.3:
        continue
    if (s["node"] in ("process:grep", "process:tail", "process:systemd-journal")
            or "kworker" in s["node"]
            or s["node"].startswith(("process:irq/", "process:rcu", "process:ksoftirqd"))):
        continue
    name = {"process:icp_odometry": "icp_odometry",
            "process:python3": "python3（bag / watch / monitor）",
            "process:ros2": "ros2 launch",
            "process:static_transfor": "static TF"}.get(
        s["node"], s["node"][len("process:"):] if s["node"].startswith("process:") else s["node"])
    w(f"| `{name}` | {s['avg']:.1f}% | {s['mx']:.1f}% | {s['rss']:.0f} MB |")
w("")
w("### 分組")
w("")
w("| 群組 | mean |")
w("|---|---:|")
for k, v in grp.items():
    w(f"| {k} | {v:.1f}% |")
w("")
w("## 4. 這次調整了哪些 ICP 參數")
w("")
w("全部寫在 `slam_toolbox_rf2o/rtabmap/config/rtabmap_laser_mapping.ini`；`start_rtabmap.sh` 每次啟動會複製到 `$RUN_DIR` 再傳給節點，RTAB-Map 的回寫不會蓋掉原檔。")
w("")
w("| 參數 | 調整前 | 調整後 | 理由 |")
w("|---|---:|---:|---|")
w("| `Icp\\RangeMax` | 0（= 30 m 全收） | **8 m** | 室內地圖僅 5×6 m，8 m 外點稀疏又吵；點數約與距離平方成正比，直接降 CPU |")
w("| `Icp\\Force4DoF` | false | **true** | 2D 平面底盤：鎖住 roll / pitch / z，避免位姿漂移（本版 `Icp/Strategy=1` 支援） |")
w("| `Icp\\CorrespondenceRatio` | 0.1 | **0.5** | 實測 ratio p5 ≈ 0.6，拉到 0.5 可擋掉 ratio=0 的退化匹配又不誤殺好幀 |")
w("| `Icp\\MaxCorrespondenceDistance` | 0.15 | **0.10** | 回到上游預設；室內 5 cm 地圖不需要 15 cm 對應距離 |")
w("| `Icp\\MaxTranslation` | 0.4 | **0.20** | 0.4 m/幀 @10 Hz = 4 m/s，實際最高 0.2 m/s；收緊可擋單幀跳變 |")
w("| `Icp\\Iterations` | 30 | **20** | 有 `GuessMotion` 初值時 20 次足夠，直接省 CPU |")
w("| `Odom\\ResetCountdown` | 未設（0） | **2 s** | 追蹤遺失 2 秒自動重置，避免 EKF 停在最後位姿 |")
w("")
w("**刻意不動**：`Icp\\Strategy=1`（libpointmatcher，Force4DoF 需要它）、`Icp\\PointToPlane=true`、`Icp\\VoxelSize=0.05`、`Icp\\PointToPlaneK=5`；節點端維持 `publish_tf=false`（TF 由 EKF 獨佔）。")
w("")
w("注意：`Icp\\RangeMax`、`Icp\\VoxelSize` 會被節點轉成 ROS 參數 `scan_range_max` / `scan_voxel_size`（log 會印 `Transferring value ...`）；要改 voxel size 得改節點參數，改 ini 沒用。")
w("")
w("## 5. ICP 內部指標")
w("")
w("| 指標 | 14:48（調參前） | 15:31（調參後） | **15:45（本次）** |")
w("|---|---:|---:|---:|")
w(f"| `scan_range_max` | 0（30 m） | 8 m | **{float(ICP['range_max']):g} m** |")
w(f"| 有效更新率 | 7.4 Hz | 9.74 Hz | **{ICP['hz']:.2f} Hz** |")
w(f"| ratio 中位 / p5 | 0.798 / 0.615 | 0.676 / 0.622 | **{ICP['ratio'][1]:.3f} / {ICP['ratio'][0]:.3f}** |")
w(f"| update time 中位 / p95 | 49 / 129 ms | 44 / 69 ms | **{ICP['upd'][0]:.0f} / {ICP['upd'][1]:.0f} ms** |")
w(f"| delay 中位 / p95 | 147 / 232 ms | 143 / 169 ms | **{ICP['dly'][0]:.0f} / {ICP['dly'][1]:.0f} ms** |")
w(f"| 丟幀 | 4.4 次/分 | 0.6 次/分 | **{ICP['drops']/(ICP['span']/60):.1f} 次/分**（{ICP['drops']} 次） |")
w(f"| session 長度 | 8.0 分 | 11.7 分 | {ICP['span']/60:.1f} 分 |")
w("")
w("## 6. 三次 run 對照")
w("")
w("| 指標 | 14:48（調參前） | 15:31（調參後） | 15:45（本次） |")
w("|---|---:|---:|---:|")
w(f"| 時長 / 取樣 | 117.8 s / 57 | 65.1 s / 32 | {dur:.1f} s / {len(cpu)} |")
w(f"| 整段 mean | 72.6% | 69.6% | {st.mean(busy):.1f}% |")
w(f"| 整段 p95 / max | 92.7 / 94.0% | 92.3 / 94.5% | {q(busy,95):.1f} / {max(busy):.1f}% |")
w(f"| 跟隨 mean | 72.9% | 66.5% | {fol[0]:.1f}% |")
w(f"| 跟隨 p95 | 84.6% | 71.0% | {fol[1]:.1f}% |")
w(f"| icp_odometry（四核） | 13.2% | 11.9% | {avg['process:icp_odometry']:.1f}% |")
w(f"| robot path | 20.81 m | 15.18 m | {path:.2f} m |")
w("")
w("## 7. 觀察")
w("")
w(f"1. **ICP 仍可壓在 100 ms 週期內，但餘裕不多**：p95 {ICP['upd'][1]:.0f} ms，離 100 ms 只剩約 "
  f"{max(0,100-ICP['upd'][1]):.0f} ms；丟幀 {ICP['drops']/(ICP['span']/60):.1f} 次/分，比 15:31 高、"
  "比調參前低。")
w(f"2. **跟隨期間的 CPU 變異大**：同一組參數下 66.5%（15:31）→ 72.2%（本次），"
  "p95 71.0 → 82.5%。短 run 的取樣太少（13 筆），單筆不足以判斷好壞。")
w(f"3. **固定成本沒變**：相機 {avg['/camera/camera']:.1f}% + AI {avg['/ai_human_detection']:.1f}% + "
  f"bag/watch {avg['process:python3']:.1f}% ≈ {avg['/camera/camera']+avg['/ai_human_detection']+avg['process:python3']:.1f}%，"
  "和 ICP 調參無關，是下一次要動的地方。")
w(f"4. **ICP RSS 只有 {stats[0]['rss']:.0f} MB**（15:31 是 178 MB），看起來這次跑的圖/關鍵幀較少，"
  "但也可能是啟動後不久就停止；不影響 CPU 結論。")
w(f"5. **ratio 回到 0.776**（15:31 是 0.676）：本次移動較少（{path:.1f} m vs 15.2 m）、"
  "靜止比例較高，重疊自然較好。")
w("")
w("## 8. 建議")
w("")
w("- 若要確認 ICP 調參的真實效益，**用 2~3 分鐘以上的 run**，或直接看 `update time` p95 與丟幀率，"
  "不要只看單筆整體 mean。")
w(f"- ICP p95 若常態 > 80 ms，可再降點數：`Icp\\VoxelSize` 需改節點參數 `scan_voxel_size`（0.05 → 0.08），"
  "ini 裡的 `Icp\\VoxelSize` 會被節點覆蓋。")
w("- 下一個降 CPU 的目標是相機（11.3%）：AI 關閉時只留 gyro/accel，可省掉大部分。")
w("- 要量純堆疊，用 `BAG=0` 再跑一次，扣掉量測本身約 8%。")
w("")
w("## 9. 資料來源")
w("")
w("```text")
w("/nvt_ros2_example/slam_toolbox_rf2o/monitor_logs/run_20261118-154507_follow/")
w("  cpu.csv               total_busy + cpu0..cpu3（2 s 取樣）")
w("  cpu_nodes.csv         per-node CPU / RSS / threads")
w("  cpu_procs.csv         per-PID CPU / RSS")
w("  follow_target.csv     robot / person / goal / state")
w("  RUN_INFO.txt          run metadata")
w("/tmp/rtabmap_only/logs/icp_odometry.log   ICP 指標（同一次啟動，已被下一次啟動覆寫）")
w("```")
w("")
w(f"*產生器：`report/make_rtabmap_icp_md_report.py`（讀原始 CSV，無平滑）*")
w("")

open(OUT, "w", encoding="utf-8").write("\n".join(L))
print("wrote", OUT, os.path.getsize(OUT))
print(f"overall {st.mean(busy):.1f} follow {fol[0]:.1f}/{fol[1]:.1f} icp {avg['process:icp_odometry']:.1f} hz {ICP['hz']:.2f} drops/min {ICP['drops']/(ICP['span']/60):.1f}")
