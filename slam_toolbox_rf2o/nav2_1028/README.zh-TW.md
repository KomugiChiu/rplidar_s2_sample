[English](README.md) | 繁體中文

# nav2_1028

這個資料夾是這台車目前使用的 Nav2 設定。重點：

```text
nav2_params_plugin.yaml   ← start_all.sh / start_all_localization.sh 的預設
nav2_params_amcl.yaml     ← start_all_amcl.sh 使用
nav2_params.yaml          ← 基礎版（純 MPPI），直接跑 start_nav2.sh 時的預設
behavior_trees/           ← 自訂 Behavior Tree
yaml_legacy/              ← 舊版、備份、實驗版 yaml
```

本文的「預設 yaml」指的是 `nav2_params.yaml`（純 MPPI 基礎版），
不是 Nav2 上游的 `nav2_bringup` 範例。

---

## 建置配置

原始碼配置：

```text
scripts/                  *.sh, *.py, tools/
config/                   *.yaml, behavior_trees/, yaml_legacy/
```

使用 colcon 建置：

```bash
colcon build --merge-install --packages-select nav2_1028_bringup
```

install 會是平坦結構：

```text
install/start_nav2.sh
install/stop_nav2.sh
install/start_chassis.sh
install/stop_chassis.sh
install/bringup_bond_launch.py
install/keepout_filter_launch.py
install/nav2_params*.yaml
install/behavior_trees/
install/tools/
```

把 `install/.` 的內容複製到：

```text
/nvt_ros2_example/nav2/
```

板上不要再有 `install/` 子資料夾。

## 1. 啟動時用哪一份 yaml

| 入口 | 使用的 yaml | 自訂 BT |
|---|---|---|
| `../start_all.sh` | `nav2_params_plugin.yaml` | ✅ |
| `../start_all_localization.sh` | `nav2_params_plugin.yaml` | ✅ |
| `../start_all_amcl.sh` | `nav2_params_amcl.yaml` | ✅ |
| 直接 `start_nav2.sh` | `nav2_params.yaml` | ❌ 用 Nav2 內建 BT |

`start_all*` 會把 `PARAMS_FILE` 傳給 `start_nav2.sh`。
直接跑 `start_nav2.sh` 而沒有指定 `PARAMS_FILE` 時，會載入 `nav2_params.yaml`。

---

## 2. Behavior Tree XML 與 Nav2 預設的差異

檔案：

```text
behavior_trees/navigate_to_pose_bt.xml
behavior_trees/navigate_through_poses_bt.xml
```

`nav2_params_plugin.yaml` / `nav2_params_amcl.yaml` 會把
`default_nav_to_pose_bt_xml` 與 `default_nav_through_poses_bt_xml`
指到這兩個檔案。

沒有指定時，Nav2 會使用內建的：

```text
nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml
```

### 2.1 差異整理

| 項目 | Nav2 預設 | 這裡的自訂 BT |
|---|---|---|
| recovery 的 `Spin` | 有，`spin_dist=1.57` | **移除** |
| `FollowPath` 外層 | `RecoveryNode` retries 1 | `FollowPathTfLag`，retries 4 |
| TF 卡住 | 一次 TF 失敗就可能結束 goal | 遇 `TF_ERROR(102)` / `CONTROLLER_TIMED_OUT(107)` 時 `Wait 1.0 s` 後重試 |
| `server_timeout` | 使用 action 預設 | `ComputePathToPose`、`ComputePathThroughPoses`、`FollowPath` 設 `20000 ms` |
| `BackUp` error code | `backup_code_id`（`navigate_to_pose`） | 改成 `backup_error_code` |
| 保留下來的 recovery | ClearCostmap、Wait、BackUp | 相同 |

### 2.2 為什麼移除 Spin

這台是開迴路的 mecanum 底盤：

```text
- 沒有輪速 encoder
- 最小有效旋轉約 0.20 rad/s
- 啟動有約 0.23 s 軟啟動
- 沒有主動煞車
```

90 度的 open-loop Spin 會過衝，甚至撞到旁邊的障礙物。
因此 recovery 不再使用 Spin，只保留：

```text
ClearEntireCostmap
Wait
BackUp
```

`behavior_server` 仍然有載入 Spin plugin，只是這個 BT 不會呼叫它。

### 2.3 為什麼加 FollowPathTfLag

CPU 負載高時 `map -> odom` 可能延遲 0.5~7.5 秒。
Nav2 的 `WouldAControllerRecoveryHelp` 不處理：

```text
TF_ERROR (102)
CONTROLLER_TIMED_OUT (107)
```

所以外層多加一層：

```text
FollowPathTfLag (retries=4)
  └─ FollowPath (retries=1, 原本的 recovery)
  └─ WaitOutTfLag
        AreErrorCodesPresent = 102;107
        Wait 1.0 s
```

TF 恢復後會直接重試同一個 goal，不會讓整個導航任務失敗。

---

## 3. `nav2_params_plugin.yaml` 與 `nav2_params.yaml` 的差異

`nav2_params.yaml` 是 DiffDrive + MPPI 的基礎版。
`nav2_params_plugin.yaml` 在其上換成 DeadbandController，並加入實車調校。

### 3.1 `bt_navigator`

```text
+ 自訂 BT 路徑（behavior_trees/）
+ default_nav_to_pose_bt_xml
+ default_nav_through_poses_bt_xml
```

### 3.2 `controller_server`

| 參數 | 基礎版 | plugin 版 |
|---|---|---|
| `FollowPath.plugin` | `nav2_mppi_controller::MPPIController` | `nav2_deadband_controller::DeadbandController` |
| `FollowPath.mode` | — | `simple` |
| `min_y_velocity_threshold` | `0.5` | `0.001` |
| `progress_checker.required_movement_radius` | `0.15` | `0.10` |
| `progress_checker.movement_time_allowance` | `10.0` | `15.0` |
| goal checker | `SimpleGoalChecker`，xy `0.15` | `StoppedGoalChecker`，xy `0.05` |
| goal checker 停止速度 | — | trans `0.03`、rot `0.10` |

`StoppedGoalChecker` 是為了避免底盤在還有速度時就被判定到達，
之後又滑行 10 cm 以上。

### 3.3 Costmap

| 參數 | 基礎版 | plugin 版 |
|---|---|---|
| local `transform_tolerance` | `0.5` | `3.0` |
| global `transform_tolerance` | `0.5` | `1.0` |
| footprint | `robot_radius: 0.22` | 15 × 10 cm 矩形 |
| local `inflation_radius` | `0.35` | `0.10` |
| global `inflation_radius` | `0.30` | `0.10` |
| keepout filter | 無 | local / global 都開 |

footprint：

```yaml
footprint: "[[0.075, 0.05], [0.075, -0.05], [-0.075, -0.05], [-0.075, 0.05]]"
```

`transform_tolerance` 放大是因為 `map -> odom` 在 CPU 忙時會延遲；
local costmap 用 3.0 秒，是為了避免 `isGoalReached()` 因為 TF 太舊而回 `TF_ERROR`。

Keepout 由 `keepout_filter_launch.py` 提供：

```text
keepout_container
  ├─ keepout_filter_mask_server   (nav2_map_server)
  ├─ costmap_filter_info_server   (nav2_map_server)
  └─ lifecycle_manager_keepout
```

### 3.4 `behavior_server`

| 參數 | 基礎版 | plugin 版 |
|---|---|---|
| `max_rotational_vel` | `0.25` | `0.20` |
| `min_rotational_vel` | `0.15` | `0.20` |
| `rotational_acc_lim` | `1.2` | `0.40` |

即使 BT 不用 Spin，behavior_server 還是保留較慢的旋轉參數。

### 3.5 `velocity_smoother`

| 參數 | 基礎版 | plugin 版 |
|---|---|---|
| `max_velocity` | `[0.15, 0.0, 0.25]` | `[0.20, 0.0, 1.0]` |
| `min_velocity` | `[-0.15, 0.0, -0.25]` | `[-0.20, 0.0, -1.0]` |
| `max_accel` | `[0.8, 0.0, 0.6]` | `[5.0, 0.0, 10.0]` |
| `max_decel` | `[-0.8, 0.0, -1.2]` | `[-5.0, 0.0, -10.0]` |

plugin 版幾乎是 pass-through：

```text
controller 已經輸出底盤可以接受的 step
velocity_smoother 不再 ramp / clip
```

這樣才不會讓停車多延遲約 0.2 秒，也不會把 `wz` 壓到死區以下。

### 3.6 MPPI 參數仍然存在

`FollowPath` 區塊除了 DeadbandController 參數外，還保留一整段 MPPI 參數：

```text
time_steps, batch_size, vx_std, critics, CostCritic, ...
motion_model: Omni
TwirlingCritic
```

這些是給 `mode: "mppi"` 用的。
預設 `mode: "simple"` 時 **MPPI 不會被載入**，這些參數不會生效。

### 3.7 沒有改變的區塊

大致上與基礎版相同：

```text
amcl
planner_server（NavfnPlanner）
smoother_server（SimpleSmoother）
waypoint_follower
route_server
docking_server
collision_monitor（只有註解差異）
```

---

## 4. DeadbandController plugin 的行為

### 4.1 架構

```text
process: controller_server
  ├─ node /controller_server
  ├─ pluginlib: nav2_deadband_controller::DeadbandController
  └─ （只有 mode=mppi 時）
       └─ pluginlib: nav2_mppi_controller::MPPIController
```

`DeadbandController` 是 `nav2_core::Controller` 的實作，
用 pluginlib 載入 `controller_server` process，**不是獨立 node**。

### 4.2 兩種模式

```yaml
FollowPath:
  plugin: "nav2_deadband_controller::DeadbandController"
  mode: "simple"
  primary_controller: "nav2_mppi_controller::MPPIController"
```

| 模式 | 行為 |
|---|---|
| `simple`（預設） | 自己算 pursuit / pulse，不載入 MPPI |
| `mppi` | 載入 `primary_controller`，再套用 floor + 方向停止 |

`mode` 不是 `simple` 或 `mppi` 時，configure 會直接丟出錯誤。

### 4.3 `simple` 模式流程

狀態機：

```text
DRIVE → ROTATE → ARRIVED
```

#### DRIVE

- 沿著路徑取 lookahead 點（`lookahead_distance: 0.3 m`）
- 前進速度固定 `drive_speed: 0.14`
- 轉向：

```text
wz = steer_gain * heading_error
   = 0.8 * error
  上限 steer_max = 0.30
```

- 接近 goal 時：
  - 距離 < `cruise_stop_distance: 0.20 m` 或 heading error 太大
  - 先停車並 settle `settle_time: 0.70 s`
  - 再用 timed pulse 補完最後距離

#### Timed pulse

短距離 goal 不使用閉迴路，直接開回路打一個時間脈衝：

```text
T = ff_dead_time + distance / ff_speed
  = 0.28 + distance / 0.80
```

- 只用在距離 < `ff_max_distance: 0.30 m`
- 旋轉 pulse：

```text
T = rot_dead_time + |yaw_error| / rot_rate
  = 0.05 + |error| / 2.0
```

- 單次旋轉上限 `rotate_pulse: 0.30 s`
- 每次 pulse 結束後會用實際移動量更新 `ff_speed` / `rot_rate`
  （`adapt_gain: 0.2`，忽略停滯與離群值）

#### 接近 goal

- 小於 `near_goal_distance: 0.15 m` 時，不會為了位置誤差原地旋轉
- 若橫向誤差 < `0.9 * xy_retry_distance`，直接沿車頭方向脈衝前後
- goal 在後方且距離 < `reverse_distance: 0.25 m` 時，直接倒退

#### ROTATE

- 位置到達後（< `brake_distance: 0.045 m`）才處理最終 heading
- 以 pulse 旋轉到 goal yaw
- 誤差 < `yaw_tolerance: 0.30 rad` 就進入 `ARRIVED`

#### ARRIVED

- 保持不動
- 有新 goal，或被推離 `xy_retry_distance: 0.05 m` 以上，才重新進 `DRIVE`

### 4.4 路徑前處理

`setPlan()` 會先把 planner 產生的密集路徑降採樣：

```text
path_min_distance: 0.10 m
```

保留第一個與最後一個 pose。
這樣可以減少後續 TF 轉換與 lookahead 計算量。

### 4.5 前向安全停止

前進前會檢查 local costmap：

```text
stop_distance: 0.20 m
stop_half_width: 0.07 m
```

- 只檢查 `LETHAL_OBSTACLE`
- 不把 INSCRIBED 儲存格當成停止條件
- 只阻擋「前進」，不阻擋旋轉與後退

超音波：

```text
sonar_topic: /ultrasonic
sonar_stop_distance: 0.20 m
sonar_hold_time: 0.3 s
```

- `/ultrasonic` 單位是 cm
- 距離 < 0.20 m 時，前進被擋 0.3 秒
- 超出 0.02~3.0 m 範圍的值會忽略

### 4.6 Escape recovery

如果前進被持續擋住超過：

```text
escape_after: 3.0 s
```

就會進入 escape：

```text
1. 判斷障礙物在左邊還是右邊（costmap lethal cell 加權）
2. 往反方向轉 escape_angle: 0.5 rad
3. 最多轉 escape_max_turns: 3 次
4. 前方清了就前進 escape_distance: 0.15 m
5. 如果還是被擋，且後方沒障礙，就倒退 0.15 m
6. 前後都 blocked → 放棄，交回 BT recovery
```

這是為了解決 planner 允許、但車身已經貼著 lethal cell 時
`BackUp` 會回報 `Collision Ahead` 而不動的情況。

### 4.7 停止心跳

底盤 MCU 會記住最後一個速度命令，因此 plugin 有 stop timer：

```text
cmd_vel_timeout: 0.5 s
```

如果 `computeVelocityCommands()` 超過 0.5 秒沒有被呼叫
（goal 完成、Nav2 停止、controller 卡住），
就持續發布零速度到 `cmd_vel`，避免車子維持最後一個非零命令。

### 4.8 `mppi` 模式（非預設）

```text
cmd = MPPI.computeVelocityCommands(...)
```

接著：

1. 只阻擋前進（`forwardBlocked`）
2. `vx` / `vy` 套用 floor：

```text
vx_floor: 0.16
vy_floor: 0.0
wz_floor: 0.90
```

3. 距離 goal > `goal_rotate_distance: 0.30 m` 時，強制 `wz = 0`
4. 進入 0.30 m 內才允許原地旋轉，轉速固定 `wz_floor`
5. 轉到 `rotate_stop_angle: 0.35 rad` 內停止旋轉

### 4.9 主要參數

| 參數 | 值 | 說明 |
|---|---:|---|
| `mode` | `simple` | 預設模式 |
| `drive_speed` | `0.14` | 前進速度 |
| `steer_gain` | `0.8` | 轉向增益 |
| `steer_max` | `0.30` | 行進中最大 `wz` |
| `align_angle` | `0.6` | 超過此誤差先停車轉向 |
| `brake_distance` | `0.045` | 停車距離 |
| `xy_retry_distance` | `0.05` | 被推離後重試距離 |
| `yaw_tolerance` | `0.30` | 最終 heading 容差 |
| `settle_time` | `0.70` | 停止後等待時間 |
| `ff_max_distance` | `0.30` | 使用 timed pulse 的最大距離 |
| `ff_dead_time` | `0.28` | pulse 死區時間 |
| `ff_speed` | `0.80` | pulse 初始速度 |
| `adapt_gain` | `0.2` | pulse 自我校正增益 |
| `rotate_pulse` | `0.30` | 單次旋轉上限 |
| `rot_rate` | `2.0` | 旋轉 pulse 速率 |
| `stop_distance` | `0.20` | costmap 前向停止距離 |
| `stop_half_width` | `0.07` | 前向檢查半寬 |
| `sonar_stop_distance` | `0.20` | 超音波停止距離 |
| `escape_after` | `3.0` | 多久沒動就 escape |
| `cmd_vel_timeout` | `0.5` | 停止心跳逾時 |

---

## 5. 快速確認目前設定

```bash
# 目前 BT 檔案
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
ros2 param get /bt_navigator default_nav_through_poses_bt_xml

# 目前 controller plugin
ros2 param get /controller_server FollowPath.plugin
ros2 param get /controller_server FollowPath.mode

# 目前節點
ros2 node list | grep -E "bt_navigator|controller_server|costmap"
```

注意：

```text
always_reload_bt_xml = False
```

BT XML 在 `bt_navigator` configure 時載入，改檔案後要重啟 `bt_navigator`
（或整個 Nav2）才會生效。

---

## 6. 相關檔案（相對路徑）

```text
nav2_params_plugin.yaml
nav2_params_amcl.yaml
nav2_params.yaml
behavior_trees/navigate_to_pose_bt.xml
behavior_trees/navigate_through_poses_bt.xml
keepout_filter_launch.py
bringup_bond_launch.py
../start_all.sh
../start_all_localization.sh
../start_all_amcl.sh
../nav2_deadband_ws/src/nav2_deadband_controller/
```
