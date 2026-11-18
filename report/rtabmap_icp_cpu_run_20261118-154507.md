# RTAB-Map + ICP 效能報告 — run_20261118-154507

> 調參後第二次 follow run（59 s）：整段 CPU 72.1%、跟隨 72.2%，ICP 13.4%（四核佔比）。
>
> 撰寫日期：2026-11-18 · 板子 `komugi`（Jetson 4 核）· ROS 2 Jazzy · `for_rtabmap` 流程
>
> 數值皆為 **[實測]**：`monitor_logs/run_20261118-154507_follow/` 的 `cpu.csv` / `cpu_nodes.csv` / `follow_target.csv`，以及同一次啟動的 `icp_odometry.log`。
>
> CPU 一律換算成**四核整體佔比**（100% = 四核全滿，單核滿載 = 25%）；process 數值另以 `名目 2.0 s ÷ 實際取樣間隔` 修正。

---

## 0. TL;DR

- 整段 **72.1%**（p95 94.5%、max 95.1%），4/29 筆 ≥ 90%，全部集中在啟動段。
- 跟隨期間（5.9~32.2 s）**72.2%**，p95 82.5%，max 95.1%。
- 最大節點仍是 **icp_odometry 13.4%**，其次 camera 11.3%、python3（bag/watch）7.9%。
- ICP 本體：**9.32 Hz**、update time 中位 43 ms / p95 90 ms、ratio 中位 0.776、丟幀 15 次 / 5.7 分（2.6 次/分）。
- 這次比 15:31 那筆差（跟隨 p95 71.0 → 82.5%），但仍在 14:48（調參前）的水準附近；短 run（59 s、13 筆跟隨取樣）變異很大，不宜用單筆定調。

## 1. 這次 run 的設定與狀態

| 項目 | 值 |
|---|---|
| run | `run_20261118-154507_follow` |
| 時長 / 取樣 | 58.9 s / 29 筆（2 s） |
| 流程 | for_rtabmap：rtabmap + icp_odometry + EKF + Nav2 + AI |
| ICP 參數 | `RangeMax=8`、`Force4DoF=true`、`CorrespondenceRatio=0.5`、`MaxCorrDist=0.10`、`MaxTranslation=0.20`、`Iterations=20` |
| 狀態 | FOLLOWING 5.9 s → RETURN_HOME 32.2 s → STOPPED 53.6 s |
| goals / person 訊息 | 39 / 626 |
| robot 移動 | net 6.80 m / path 11.18 m |
| max \|vx\| / \|wz\| | 0.14 m/s / 0.90 rad/s |

## 2. CPU 總覽

| 區間 | 筆數 | mean | p95 | max |
|---|---:|---:|---:|---:|
| 整段 0~58.9 s | 29 | 72.1% | 94.5% | 95.1% |
| 啟動 0~5.9 s | 3 | 92.9% | — | 94.5% |
| 跟隨 5.9~32.2 s | 13 | 72.2% | **82.5%** | 95.1% |
| 返回 32.2~53.6 s | 10 | 67.7% | 70.1% | 70.1% |

### Per-core（每核 100% = 該核滿載）

| Core | mean | max |
|---|---:|---:|
| cpu0 | 73.2% | 91.2% |
| cpu1 | 71.3% | 100.0% |
| cpu2 | 71.4% | 99.6% |
| cpu3 | 72.3% | 99.6% |

## 3. 節點排行（整段，四核佔比）

| 節點 | mean | max | RSS |
|---|---:|---:|---:|
| `icp_odometry` | 13.4% | 15.9% | 99 MB |
| `/camera/camera` | 11.3% | 13.4% | 110 MB |
| `python3（bag / watch / monitor）` | 7.9% | 14.3% | 75 MB |
| `/ai_human_detection` | 6.6% | 9.6% | 115 MB |
| `/rtabmap/rtabmap` | 4.5% | 5.2% | 169 MB |
| `ros2 launch` | 4.2% | 17.3% | 83 MB |
| `/rplidar_node` | 4.0% | 5.7% | 37 MB |
| `/ekf_filter_node` | 3.6% | 4.1% | 42 MB |
| `/controller_server` | 2.5% | 4.0% | 60 MB |
| `/driver_node` | 2.1% | 2.6% | 69 MB |
| `/collision_monitor` | 1.9% | 2.7% | 51 MB |
| `/bt_navigator` | 1.7% | 4.0% | 75 MB |
| `/demo_manager_node` | 1.1% | 2.3% | 48 MB |
| `/planner_server` | 1.1% | 2.5% | 62 MB |
| `/velocity_smoother` | 0.6% | 2.0% | 40 MB |
| `/docking_server` | 0.6% | 2.8% | 52 MB |
| `/behavior_server` | 0.5% | 2.2% | 53 MB |
| `static TF` | 0.5% | 4.8% | 37 MB |
| `/route_server` | 0.5% | 2.1% | 53 MB |
| `/smoother_server` | 0.5% | 2.5% | 49 MB |
| `/person_target_recovery_node` | 0.4% | 1.7% | 36 MB |

### 分組

| 群組 | mean |
|---|---:|
| RTAB-Map + ICP | 17.9% |
| 相機 + AI | 17.9% |
| 光達 + EKF | 7.6% |
| 量測（bag + watch + monitor） | 7.9% |
| ros2 launch wrapper | 4.2% |
| Nav2 + 底盤 + demo | 15.6% |

## 4. ICP 內部指標

| 指標 | 14:48（調參前） | 15:31（調參後） | **15:45（本次）** |
|---|---:|---:|---:|
| `scan_range_max` | 0（30 m） | 8 m | **8 m** |
| 有效更新率 | 7.4 Hz | 9.74 Hz | **9.32 Hz** |
| ratio 中位 / p5 | 0.798 / 0.615 | 0.676 / 0.622 | **0.776 / 0.581** |
| update time 中位 / p95 | 49 / 129 ms | 44 / 69 ms | **43 / 90 ms** |
| delay 中位 / p95 | 147 / 232 ms | 143 / 169 ms | **142 / 192 ms** |
| 丟幀 | 4.4 次/分 | 0.6 次/分 | **2.6 次/分**（15 次） |
| session 長度 | 8.0 分 | 11.7 分 | 5.7 分 |

## 5. 三次 run 對照

| 指標 | 14:48（調參前） | 15:31（調參後） | 15:45（本次） |
|---|---:|---:|---:|
| 時長 / 取樣 | 117.8 s / 57 | 65.1 s / 32 | 58.9 s / 29 |
| 整段 mean | 72.6% | 69.6% | 72.1% |
| 整段 p95 / max | 92.7 / 94.0% | 92.3 / 94.5% | 94.5 / 95.1% |
| 跟隨 mean | 72.9% | 66.5% | 72.2% |
| 跟隨 p95 | 84.6% | 71.0% | 82.5% |
| icp_odometry（四核） | 13.2% | 11.9% | 13.4% |
| robot path | 20.81 m | 15.18 m | 11.18 m |

## 6. 觀察

1. **ICP 仍可壓在 100 ms 週期內，但餘裕不多**：p95 90 ms，離 100 ms 只剩約 10 ms；丟幀 2.6 次/分，比 15:31 高、比調參前低。
2. **跟隨期間的 CPU 變異大**：同一組參數下 66.5%（15:31）→ 72.2%（本次），p95 71.0 → 82.5%。短 run 的取樣太少（13 筆），單筆不足以判斷好壞。
3. **固定成本沒變**：相機 11.3% + AI 6.6% + bag/watch 7.9% ≈ 25.8%，和 ICP 調參無關，是下一次要動的地方。
4. **ICP RSS 只有 99 MB**（15:31 是 178 MB），看起來這次跑的圖/關鍵幀較少，但也可能是啟動後不久就停止；不影響 CPU 結論。
5. **ratio 回到 0.776**（15:31 是 0.676）：本次移動較少（11.2 m vs 15.2 m）、靜止比例較高，重疊自然較好。

## 7. 建議

- 若要確認 ICP 調參的真實效益，**用 2~3 分鐘以上的 run**，或直接看 `update time` p95 與丟幀率，不要只看單筆整體 mean。
- ICP p95 若常態 > 80 ms，可再降點數：`Icp\VoxelSize` 需改節點參數 `scan_voxel_size`（0.05 → 0.08），ini 裡的 `Icp\VoxelSize` 會被節點覆蓋。
- 下一個降 CPU 的目標是相機（11.3%）：AI 關閉時只留 gyro/accel，可省掉大部分。
- 要量純堆疊，用 `BAG=0` 再跑一次，扣掉量測本身約 8%。

## 8. 資料來源

```text
/nvt_ros2_example/slam_toolbox_rf2o/monitor_logs/run_20261118-154507_follow/
  cpu.csv               total_busy + cpu0..cpu3（2 s 取樣）
  cpu_nodes.csv         per-node CPU / RSS / threads
  cpu_procs.csv         per-PID CPU / RSS
  follow_target.csv     robot / person / goal / state
  RUN_INFO.txt          run metadata
/tmp/rtabmap_only/logs/icp_odometry.log   ICP 指標（同一次啟動，已被下一次啟動覆寫）
```

*產生器：`report/make_rtabmap_icp_md_report.py`（讀原始 CSV，無平滑）*
