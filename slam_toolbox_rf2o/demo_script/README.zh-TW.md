[English](README.md) | 繁體中文

# demo_script

完整 person-follow demo 的兩個入口：

```text
start_demo.sh          mapping 模式
start_demo_static.sh   localization 模式（載入既有 posegraph）
```

兩者都會啟動四層：

```text
1. RPLIDAR S2 + rf2o + slam_toolbox
2. RealSense + EKF
3. Nav2 + 底盤 driver
4. AI detection + person_target_recovery + demo_manager
```

它們**不會**呼叫 `../start_all.sh`、`../start_all_localization.sh`
或 `../start_ai.sh`，而是直接組合低階 start script 與 vendor helper。

## 1. 檔案

| 檔案 | 用途 |
|---|---|
| `start_demo.sh` | 完整 demo，slam_toolbox mapping 模式 |
| `start_demo_static.sh` | 完整 demo，slam_toolbox localization 模式 |
| `start_mapping.sh` | 只啟動 mapping 的 SLAM（不含 EKF / Nav2 / AI） |
| `demo_common.sh` | 共用函式（由 `start_demo*` source） |

## 2. Mapping 與 localization 的差別

| | `start_demo.sh` | `start_demo_static.sh` |
|---|---|---|
| SLAM 模式 | `mapping` | `localization` |
| SLAM 參數 | `../slam_toolbox_rf2o.yaml` | `../slam_toolbox_localization.yaml` |
| Posegraph | 不需要 | 必須存在 |
| 建立新 map | 是 | 否 |
| Nav2 參數 | `../nav2_1028/nav2_params_plugin.yaml` | 相同 |
| 啟動 EKF / Nav2 / AI | 是 | 是 |

如果只要 mapping 的 SLAM（RPLIDAR + rf2o + slam_toolbox，
不含 EKF、Nav2、AI）：

```bash
./start_mapping.sh
```

它等同於 `../slam_toolbox/start_mapping.sh`。
停止請用 `../slam_toolbox/stop.sh`。

Localization 會從 localization yaml 的 `map_file_name` 讀取：

```text
<map_file_name>.posegraph
<map_file_name>.data
```

建立 posegraph 的方式（mapping 模式）：

```bash
cd ../slam_toolbox
./start_mapping.sh
./save_map.sh demo_map
./stop.sh
```

接著把 `map_file_name` 設到 `slam_toolbox_localization.yaml`。

## 3. 使用方式

Mapping demo：

```bash
cd demo_script
./start_demo.sh
```

Localization demo：

```bash
cd demo_script
./start_demo_static.sh
```

停止：

```bash
../stop_demo.sh
```

常用變化：

```bash
START_AI=0 ./start_demo.sh          # 只跑 SLAM + EKF + Nav2
RECORD=1 ./start_demo.sh            # 另外記錄 /cmd_vel_nav CSV
SLAM_PARAMS=/path/to/other.yaml ./start_demo_static.sh
NAV2_PARAMS_FILE=../nav2_1028/nav2_params_plugin.yaml ./start_demo.sh
```

## 4. 環境變數

| 變數 | 預設 | 說明 |
|---|---|---|
| `ROS_DOMAIN_ID` | `99` | DDS domain |
| `START_AI` | `1` | 啟動 AI / recovery / demo manager |
| `RECORD` | `0` | 啟動 `nav2_1028/tools/record_cmd_vel.py` |
| `NAV2_PARAMS_FILE` | `../nav2_1028/nav2_params_plugin.yaml` | Nav2 參數 |
| `SLAM_PARAMS` | mapping 或 localization yaml | SLAM 參數 |
| `USE_EXTERNAL_SAFETY` | `0` | 傳給 `start_nav2.sh` |
| `START_REALSENSE` | `0` | 保持 0；相機由 EKF 啟動 |
| `CAMERA_X` / `CAMERA_Y` / `CAMERA_Z` | `0.0` / `0.0` / `0.0` | AI TF 偏移 |
| `LOG_DIR` | 系統暫存目錄 | log 與 PID |
| `RUN_DIR` | 系統暫存目錄 | PID 檔目錄 |
| `TOPIC_WAIT` | `20` | 每個 topic 等待秒數 |

`START_REALSENSE=1` 在 demo 中沒有用途，因為相機由 EKF 啟動。
若目前沒有相機在跑，script 會直接報錯。

## 5. 各層啟動內容

### SLAM

透過 `../start_slam_toolbox_rf2o.sh`：

```text
rplidar_node
rf2o_laser_odometry_node
base_footprint -> base_link -> laser 的 static TF
slam_toolbox（online_async 或 localization）
```

### EKF

透過 `../ekf/start_ekf.sh`：

```text
realsense2_camera
base_footprint -> camera_link static TF
robot_localization ekf_node
```

### Nav2

透過 `../nav2_1028/start_nav2.sh`：

```text
controller_server（DeadbandController plugin）
planner_server / smoother_server / behavior_server
bt_navigator / waypoint_follower / route_server / docking_server
velocity_smoother / collision_monitor
keepout container
yahboomcar Mcnamu_driver
```

### AI

由 script 直接啟動，或透過 vendor helper：

```text
ai_human_detection_node
person_follow_independent_bringup ai_tf_only.launch.py
person_target_recovery_node
demo_manager_node
```

AI 使用 EKF 已經啟動的相機，不需要第二顆 RealSense。

## 6. 啟動順序

```text
start_demo.sh
  ├─ start_slam_toolbox_rf2o.sh
  ├─ ekf/start_ekf.sh
  ├─ 等 /map + /odometry/filtered + TF map -> base_link
  ├─ nav2_1028/start_nav2.sh
  └─ AI node -> AI TF -> recovery -> demo manager
```

SLAM/EKF 最多等 60 秒，逾時就停止，不會硬啟動 Nav2。

## 7. Log 與停止

log 與 PID 會放在 `LOG_DIR`（預設是系統暫存目錄）：

```text
<LOG_DIR>/ai_node.log
<LOG_DIR>/ai_tf.log
<LOG_DIR>/person_target_recovery.log
<LOG_DIR>/demo_manager.log
<LOG_DIR>/ai_pids
```

停止：

```bash
../stop_demo.sh
```

`stop_demo.sh` 會先停 AI，再停 SLAM / Nav2 / EKF。

## 8. 注意

```text
- start_mapping.sh 只啟動 SLAM，不是完整 demo。
- Mapping demo（start_demo.sh）不需要 posegraph。
- Static demo 需要 .posegraph + .data，請在 mapping 模式存。
- 相機由 EKF 擁有，不要開第二顆 RealSense。
- Nav2 預設是 DeadbandController（simple 模式）。
- START_AI=0 可以只起導航，不跑 AI。
- AI helper 會自己 source vendor workspace；這個資料夾只負責組合與 log。
```
