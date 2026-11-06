# slam_toolbox + rf2o + EKF + Nav2（RPLIDAR S2 / D455 / Jazzy）

板上這包負責：**光達 → rf2o 雷射 odom → EKF（D455 gyro）→ slam_toolbox → Nav2 → Yahboomcar 底盤**。
所有 script 都會把 `ROS_DOMAIN_ID` 設為 **99**（可用環境變數覆寫），請保持全鏈一致。

> 先看 §1 的三行指令就能跑；要調參數再看 §4 之後。

---

## 1. 最簡單的方式

```bash
cd /nvt_ros2_example/slam_toolbox_rf2o
```

| 目的 | 啟動 | 停止 |
|---|---|---|
| **建圖 + Nav2 + 底盤**（最常用） | `./start_all.sh` | `./stop_all.sh` |
| **靜態地圖 + AMCL**（不建圖） | `./start_all_amcl.sh` | `./stop_all_amcl.sh` |
| **只建圖**（SLAM，不含 Nav2） | `./start_slam_toolbox_rf2o.sh` | `./stop_slam_toolbox_rf2o.sh` |
| **只底盤**（遙控 / 測試） | `./nav2_1028/start_chassis.sh` | `./nav2_1028/stop_chassis.sh` |
| **AI 人員偵測**（RealSense + AI + DemoManager） | `./start_ai.sh` | `./stop_ai.sh` |

`start_all.sh` 會依序做完：

```text
lidar + rf2o + slam_toolbox  →  RealSense + EKF  →  Nav2 + 底盤 driver  →  cmd_vel logger
```

啟動大約 **60~90 秒**（等 `/map`、`/odometry/filtered`、TF 就緒）。停止用 `./stop_all.sh`。

---

## 2. 資料流與 TF

```text
rplidar S2 ──/scan──► rf2o ──/odom_rf2o──► EKF ◄──/camera/camera/gyro/sample
                                             │
                                             ├── TF odom → base_footprint
                                             └── /odometry/filtered

slam_toolbox ◄──/scan + TF──►  /map、TF map → odom
Nav2（global costmap 讀 /map）──/cmd_vel──► Mcnamu_driver
```

```text
map
└── odom                 ← slam_toolbox（或 AMCL）
    └── base_footprint   ← EKF（odom→base）
        ├── base_link    ← static（z 0.10）
        │   └── laser    ← static（z 0.20、yaw 180°）
        └── camera_link  ← static（0.08, 0, 0.10）
```

重點：

- `odom → base_footprint` **只由 EKF 發**（rf2o 的 `publish_tf: false`）。
- `map → odom` 只能有 **一個** publisher：slam_toolbox **或** AMCL，不可同時。
- `laser` 相對 `base_link` 是 **yaw 180°**；改錯會讓 odom／地圖前後左右鏡像。

---

## 3. 目錄結構

```text
slam_toolbox_rf2o/
├── start_all.sh / stop_all.sh              # 建圖 + EKF + Nav2 + driver（plugin 模式）
├── start_all_amcl.sh / stop_all_amcl.sh    # 靜態地圖 + AMCL
├── start_slam_toolbox_rf2o.sh / stop_...   # lidar + rf2o + slam_toolbox
├── start_ai.sh / stop_ai.sh                # RealSense + AI + DemoManager
├── slam_toolbox_rf2o.yaml                  # slam_toolbox 參數
├── rf2o_params.yaml                        # rf2o 參數
├── rviz_slam_toolbox_rf2o.rviz             # 這條流程用的 RViz
├── README.md                               # 本文件
├── README_start_stop.md                    # SLAM start/stop 的細節補充
├── ekf/
│   ├── start_ekf.sh / stop_ekf.sh
│   └── ekf_params.yaml
├── nav2_1028/                              # ★ 正式 Nav2 流程
│   ├── start_nav2.sh / stop_nav2.sh
│   ├── start_chassis.sh / stop_chassis.sh
│   ├── nav2_params.yaml                    # 基礎版（純 MPPI）
│   ├── nav2_params_plugin.yaml             # ★ default：DeadbandController
│   ├── nav2_params_amcl.yaml               # 靜態地圖 + AMCL
│   ├── nav2_params_omni*.yaml              # 舊的 Omni 實驗版
│   ├── nav2_params.yaml.ori                # 原始備份
│   ├── behavior_trees/                     # 無 Spin 的自訂 BT
│   ├── tools/                              # 記錄 / 校正 / legacy 安全鏈
│   └── nav2_1028.md                        # 完整除錯紀錄
├── maps/                                   # 靜態地圖（pgm/yaml，posegraph 不上 git）
└── nav2/                                   # ⚠️ 舊流程（translation gate），不要用
```

---

## 4. 腳本與參數

### 4.1 `start_all.sh` / `stop_all.sh`（建議入口）

啟動：`start_slam_toolbox_rf2o.sh` → `ekf/start_ekf.sh` → 等 preflight → `nav2_1028/start_nav2.sh` → 可選 cmd_vel logger。

| 變數 | 預設 | 說明 |
|---|---|---|
| `ROS_DOMAIN_ID` | `99` | 全鏈 DDS domain |
| `NAV2_PARAMS_FILE` | `nav2_1028/nav2_params_plugin.yaml` | Nav2 參數；也接受舊名 `PARAMS_FILE` |
| `SLAM_MODE` | `mapping` | `mapping` / `localization` / `none`（AMCL 用） |
| `USE_LOCALIZATION` | `0` | `1` = map_server + AMCL |
| `MAP_FILE` | 空 | `USE_LOCALIZATION=1` 時必填 |
| `USE_EXTERNAL_SAFETY` | `0` | `0` = DeadbandController plugin；`1` = 舊的外部 speed_floor/directional_stop |
| `RECORD` | `1` | `0` = 不啟動 `tools/record_cmd_vel.py` |
| `LOG_DIR` | `/tmp/nav2_logs` | cmd_vel CSV 位置 |

範例：

```bash
./start_all.sh
RECORD=0 ./start_all.sh
NAV2_PARAMS_FILE=$PWD/nav2_1028/nav2_params_omni_rotation.yaml ./start_all.sh
USE_LOCALIZATION=1 MAP_FILE=$PWD/maps/slam_map_20261102.yaml ./start_all.sh
```

`stop_all.sh`：

| 變數 | 預設 | 說明 |
|---|---|---|
| `KEEP_DRIVER` | `0` | `1` = 保留底盤 driver（只停 Nav2 / EKF / SLAM） |

停止順序：Nav2 + driver → EKF/RealSense → slam_toolbox/rf2o/lidar → cmd_vel logger → 清孤兒，約 12 秒。

---

### 4.2 `start_all_amcl.sh` / `stop_all_amcl.sh`

靜態地圖 + AMCL，**不啟動 slam_toolbox**（`map_server` 擁有 `/map`、AMCL 擁有 `map→odom`）。

| 變數 | 預設 | 說明 |
|---|---|---|
| `MAP_FILE` | `maps/slam_map_20261102.yaml` | 靜態地圖 |
| `NAV2_PARAMS_FILE` | `nav2_1028/nav2_params_amcl.yaml` | AMCL 參數 |
| `RECORD` / `LOG_DIR` | 同上 | 透傳給 `start_all.sh` |

```bash
./start_all_amcl.sh
MAP_FILE=/path/other_map.yaml ./start_all_amcl.sh
```

**機器人必須放在 `initial_pose` 對應的位置**，否則先在 RViz 用「2D Pose Estimate」重定位，或：

```bash
python3 nav2_1028/tools/set_initial_pose.py X Y YAW_DEG
```

`initial_pose` 目前設在 `nav2_params_amcl.yaml` 的 `amcl.initial_pose`（2026-11-03 實測停車位）。

---

### 4.3 `start_slam_toolbox_rf2o.sh` / `stop_slam_toolbox_rf2o.sh`

只做 SLAM 這一段：lidar + static TF + rf2o + slam_toolbox。**背景執行，腳本立刻返回**。

| 變數 | 預設 | 說明 |
|---|---|---|
| `SLAM_MODE` | `mapping` | `mapping` / `localization` / `none`（`none` = 只起 lidar+rf2o，給 AMCL 用） |
| `SERIAL_PORT` | `/dev/ttyUSB0` | S2 序列埠 |
| `SCAN_MODE` | `DenseBoost` | RPLIDAR S2 scan mode |
| `ANGLE_COMPENSATE` | `true` | 角度補償 |
| `SCAN_TOPIC` | `/scan` | 光達 topic |
| `LASER_FRAME` / `BASE_FRAME` / `BASE_LINK_FRAME` | `laser` / `base_footprint` / `base_link` | frame 名稱 |
| `LASER_YAW_DEG` | `180` | **S2 實際安裝方向，勿改**（改 0 會鏡像） |
| `LASER_Z` / `BASE_LINK_Z` | `0.20` / `0.10` | static TF 高度 |
| `ODOM_TOPIC` / `VO_FRAME_ID` | `/odom_rf2o` / `odom` | rf2o 輸出 |
| `SLAM_PARAMS` | `slam_toolbox_rf2o.yaml` | slam_toolbox 參數 |
| `SKIP_LIDAR` | `0` | `1` = 不重啟 lidar（沿用已在跑的） |
| `QUIET` | `1` | `0` = 顯示 node 輸出 |
| `NO_LOG` | `0` | `1` = 完全不寫 log |
| `LOG_DIR` | `/tmp` | log 位置（`lidar_*.log`） |
| `RUN_DIR` / `PID_FILE` | `/tmp/slam_toolbox_rf2o` | PID 檔 |
| `TF_WAIT` | `30` | 啟動 rf2o 前等 TF 的秒數 |

```bash
./start_slam_toolbox_rf2o.sh
QUIET=0 ./start_slam_toolbox_rf2o.sh
SLAM_MODE=localization ./start_slam_toolbox_rf2o.sh
SLAM_MODE=none ./start_slam_toolbox_rf2o.sh
./stop_slam_toolbox_rf2o.sh
```

停止腳本參數：

| 變數 | 預設 | 說明 |
|---|---|---|
| `KEEP_DRIVER` | `0` | `1` = 不動 `rplidar_node` |
| `DRY_RUN` | `0` | `1` = 只列出會停哪些 |
| `WAIT_SECS` | `10` | SIGTERM 後等多久才 SIGKILL |

---

#### 效能注意：`check_min_dist_and_heading_precisely`

`slam_toolbox_rf2o.yaml` / `slam_toolbox_localization.yaml` 都有這個參數，
**預設是 `false`**（slam_toolbox 上游預設）。

| 設定 | 行為 | CPU |
|---|---|---|
| **`false`（預設）** | `shouldProcessScan()` 只檢查 `minimum_travel_distance`；**原地旋轉（位移≈0）時不處理掃描** | 低 |
| `true` | 改用「距離 + `minimum_travel_heading`」判斷，**純旋轉也會做掃描匹配** | **明顯較高** |

- 設 `true` 的好處：180° 原地轉完後 heading 會即時被修正，不用等往前走
- 代價：
  - CPU 明顯上升（掃描處理頻率變高）
  - 走廊等特徵少的環境，旋轉中的掃描匹配可能產生壞結果 → 地圖/定位反而變差
- 參數在**啟動時讀取**，改完要重啟 `start_slam_toolbox_rf2o.sh` 才生效

> 建議：**保持 `false`**。除非你確定需要「原地旋轉時也更新 heading」且有 CPU 餘裕，
> 再改成 `true` 並實測地圖品質。

---

### 4.4 `ekf/start_ekf.sh` / `ekf/stop_ekf.sh`

一個 RealSense node 同時給 EKF（gyro）與 AI（RGBD）：`enable_rgbd + color + depth + align_depth + sync`，profile 目前 **424×240 @ 5 fps**。

產出：`/odometry/filtered`、TF `odom → base_footprint`、TF `base_footprint → camera_link`。

| 變數 | 預設 | 說明 |
|---|---|---|
| `EKF_PARAMS_FILE` | `ekf/ekf_params.yaml` | EKF 參數（注意不是 `PARAMS_FILE`） |
| `CAM_X` / `CAM_Y` / `CAM_Z` | `0.08` / `0.0` / `0.10` | `base_footprint → camera_link` |
| `RUN_DIR` / `PID_FILE` / `LOG_DIR` | `/tmp/slam_toolbox_rf2o_ekf` | PID / log |
| `ROS_DISTRO` | `jazzy` | |

```bash
./ekf/start_ekf.sh
CAM_X=0.10 ./ekf/start_ekf.sh
./ekf/stop_ekf.sh
```

> 只開 IMU 可省 CPU：把 `start_ekf.sh` 的 `enable_rgbd:=true / enable_color:=true / enable_depth:=true` 改成 `false`（但 AI 就沒有影像）。

---

### 4.5 `nav2_1028/start_nav2.sh` / `stop_nav2.sh`

啟動底盤 driver + `nav2_bringup`（`slam:=False`）。**preflight 會等 `/map` + `/odometry/filtered` + TF**。

| 變數 | 預設 | 說明 |
|---|---|---|
| `PARAMS_FILE` | `nav2_1028/nav2_params.yaml` | ⚠️ 預設是**沒有 DeadbandController** 的基礎版 |
| `USE_LOCALIZATION` | `0` | `1` = map_server + AMCL（需 `MAP_FILE`） |
| `MAP_FILE` | 空 | 靜態地圖 yaml |
| `START_DRIVER` | `true` | `false` = 不動底盤 driver |
| `SKIP_CHECK` | `0` | `1` = 跳過 preflight |
| `USE_EXTERNAL_SAFETY` | `0` | `1` = 舊的外部安全鏈（`tools/legacy/`） |
| `VX_FLOOR` / `WZ_FLOOR` | `0.16` / `0.52` | 僅外部安全鏈使用 |
| `PLUGIN_WS` | `/root/nav2_deadband_ws` | DeadbandController overlay |
| `RUN_DIR` / `PID_FILE` / `LOG_DIR` | `/tmp/raspybot_nav2` | PID / log |

```bash
# 建議：plugin 版（DeadbandController）
PARAMS_FILE=$PWD/nav2_1028/nav2_params_plugin.yaml ./nav2_1028/start_nav2.sh

# 靜態地圖 + AMCL
USE_LOCALIZATION=1 MAP_FILE=$PWD/maps/slam_map_20261102.yaml \
  PARAMS_FILE=$PWD/nav2_1028/nav2_params_amcl.yaml ./nav2_1028/start_nav2.sh

./nav2_1028/stop_nav2.sh
```

`stop_nav2.sh`：

| 變數 | 預設 | 說明 |
|---|---|---|
| `KEEP_DRIVER` | `0` | `1` = 保留底盤 driver |

---

### 4.6 `nav2_1028/start_chassis.sh` / `stop_chassis.sh`

只起 Yahboomcar 底盤 driver（`Mcnamu_driver`），**不啟動 Nav2**，直接訂閱 `/cmd_vel`。

| 變數 | 預設 | 說明 |
|---|---|---|
| `ROBOT_WS` | `/root/raspybot/yahboomcar_ws` | driver workspace |
| `LIBRARY_WS` | `/root/raspybot/library_ws/py_install` | Python 依賴 |
| `RUN_DIR` / `PID_FILE` / `LOG_DIR` | `/tmp/nav2_chassis` | PID / log |

`stop_chassis.sh`：`DRY_RUN=1` 只列出、`PID_FILE` 可覆寫。

```bash
./nav2_1028/start_chassis.sh
DRY_RUN=1 ./nav2_1028/stop_chassis.sh
```

---

### 4.7 `start_ai.sh` / `stop_ai.sh`

RealSense + `ai_human_detection` + AI TF + DemoManager。**不會**停 SLAM / Nav2 / rf2o / lidar。

| 變數 | 預設 | 說明 |
|---|---|---|
| `START_REALSENSE` | `0` | **預設不起相機**（用 `start_all.sh` 起的 node）；`1` = AI-only 時自己起（含 gyro/accel）；`auto` = 偵測現有 node |
| `REALSENSE_WS` | `/nvt_ros2_example/realsense-ros` | |
| `AI_WS` | `/nvt_ros2_example/ai_human_detection` | |
| `DEMO_WS` | `/root/ros2_ws` | DemoManager workspace |
| `KIT_DIR` | `/nvt_ros2_example/demo_manager/fixed_build` | 原廠 kit tools |
| `CAMERA_X/Y/Z` | `0.0` | AI TF（不是 EKF 的 `CAM_*`） |
| `CAMERA_WAIT` / `TOPIC_WAIT` | `20` / `20` | 等 topic 的秒數 |
| `DRY_RUN` | `0` | `1` = 只印步驟 |
| `LOG_DIR` / `PID_FILE` | `/tmp/slam_toolbox_rf2o_ai` | |

```bash
./start_ai.sh
START_REALSENSE=1 ./start_ai.sh
DRY_RUN=1 ./start_ai.sh
./stop_ai.sh
```

---

### 4.8 `nav2/`（⚠️ 舊流程，不要用）

`start_nav2_rf2o.sh` 是早期版本：它會啟動 `translation_only.py` gate，把 `/cmd_vel` remap 成 `/cmd_vel_straight` 並把 `wz` 歸零，與現在的 plugin 流程衝突。

參數（僅供考古）：`TRANSLATION_ONLY`、`VX_SCALE`、`WZ_MAX_OUT`、`ALLOW_STRAFE`、`SONAR_STOP`、`SCAN_STOP`、`USE_ULTRASONIC`、`WATCHDOG_TIMEOUT`、`GATE_TOPIC`、`GATE_INPUT`。

**新流程請用 `start_all.sh` 或 `nav2_1028/start_nav2.sh`。**

---

## 5. 啟動順序（手動）

`start_all.sh` 已經幫你排好；要手動分段時：

```bash
cd /nvt_ros2_example/slam_toolbox_rf2o

# 1. lidar + rf2o + slam_toolbox
./start_slam_toolbox_rf2o.sh

# 2. RealSense + EKF（等 /odometry/filtered 與 TF）
./ekf/start_ekf.sh

# 3. Nav2 + 底盤（記得指定 plugin 版參數）
PARAMS_FILE=$PWD/nav2_1028/nav2_params_plugin.yaml ./nav2_1028/start_nav2.sh
```

停止順序反過來：

```bash
./nav2_1028/stop_nav2.sh
./ekf/stop_ekf.sh
./stop_slam_toolbox_rf2o.sh
```

---

## 6. 參數檔一覽

| 檔案 | 用途 |
|---|---|
| `slam_toolbox_rf2o.yaml` | slam_toolbox（`transform_publish_period 0.1`、`map_update_interval 5.0`、`resolution 0.05`） |
| `rf2o_params.yaml` | rf2o（`freq 10`、`publish_tf false`） |
| `ekf/ekf_params.yaml` | EKF（`frequency 20`、`two_d_mode`、只融合 gyro yaw rate + rf2o x/y） |
| `nav2_1028/nav2_params_plugin.yaml` | ★ 預設：DeadbandController `simple` + 15×10 cm footprint |
| `nav2_1028/nav2_params_amcl.yaml` | 靜態地圖 + AMCL（`map_server` + `amcl`） |
| `nav2_1028/nav2_params.yaml` | 基礎版（純 MPPI，沒有死區補償） |
| `nav2_1028/nav2_params_omni*.yaml` | 2026-10-28 的 Omni 實驗版（保留參考） |
| `nav2_1028/nav2_params.yaml.ori` | 最原始備份 |

---

## 7. 日誌與 PID 檔

| 流程 | 日誌 | PID 檔 |
|---|---|---|
| lidar / rf2o / SLAM | `/tmp/lidar_rplidar.log`、`/tmp/lidar_rf2o.log`、`/tmp/lidar_slam_toolbox.log`、`/tmp/lidar_static_*.log` | `/tmp/slam_toolbox_rf2o/pids` |
| EKF / RealSense | `/tmp/slam_toolbox_rf2o_ekf/logs/` | `/tmp/slam_toolbox_rf2o_ekf/pids` |
| Nav2 / driver | `/tmp/raspybot_nav2/logs/nav2.log`、`yahboomcar.log` | `/tmp/raspybot_nav2/pids` |
| 底盤（獨立） | `/tmp/nav2_chassis/logs/chassis.log` | `/tmp/nav2_chassis/pids` |
| AI | `/tmp/slam_toolbox_rf2o_ai/` | 同左 |
| cmd_vel CSV | `/tmp/nav2_logs/cmd_vel_*.csv` | `/tmp/nav2_logs/logger.pid` |

CPU / 溫度監控：

```bash
python3 nav2_1028/tools/monitor_cpu.py --interval 2 --csv /tmp/nav2_logs/cpu.csv
```

---

## 8. 常見問題

1. **直接跑 `start_nav2.sh` 覺得車子很鈍／沒有死區補償**
   它預設用 `nav2_params.yaml`（純 MPPI）。請指定：
   `PARAMS_FILE=$PWD/nav2_1028/nav2_params_plugin.yaml ./nav2_1028/start_nav2.sh`

2. **`/map` 有兩個 publisher / `map→odom` 打架**
   不要同時跑 `start_all.sh` 與 `start_all_amcl.sh`；也不要在 slam_toolbox 跑著時再起 map_server。

3. **`start_slam_toolbox_rf2o.sh` 跑完就返回，是不是沒啟動？**
   它是背景執行（`setsid`），用 `ros2 topic hz /scan` 或看 `/tmp/lidar_*.log` 確認。

4. **Nav2 說 `Transform data too old`**
   先看 `ros2 run tf2_ros tf2_echo map base_link` 與 `/tmp/lidar_slam_toolbox.log`；`map→odom` 斷超過 1 秒就會 abort goal。

5. **底盤不動**
   死區：`vx < 0.10~0.12`、`wz < ~0.20` 不會動。plugin 模式由 DeadbandController 補償；若用 `nav2_params.yaml` 就不會有。

6. **RealSense 只能有一個 node**
   EKF 與 AI 共用同一個（RGBD + gyro）。`start_ai.sh` 的 `auto` 會偵測：IMU-only 會直接報錯並提示改 `start_ekf.sh`。

7. **改了 camera 解析度沒生效**
   profile 在 `ekf/start_ekf.sh`（目前 `424x240x5`），要重啟相機（`./ekf/stop_ekf.sh` → `./ekf/start_ekf.sh`）才生效。

8. **`ROS_DOMAIN_ID` 不一致**
   所有 script 預設 99；若某個 terminal 自己 export 了別的值，會看不到彼此的 topic。

9. **舊的 `nav2/` 流程**
   不要用；它會 remap `/cmd_vel_straight` 並把 `wz` 歸零，和現在 plugin 流程衝突。
