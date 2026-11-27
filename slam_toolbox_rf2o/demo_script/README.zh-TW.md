[English](README.md) | 繁體中文

# demo_script

三個入口：

```text
start_demo.sh          mapping 模式 + AI
start_demo_static.sh   localization 模式 + AI
start_mapping.sh       mapping 模式，不含 AI
```

`start_demo.sh` 與 `start_demo_static.sh` 會啟動四層：

```text
1. RPLIDAR S2 + rf2o + slam_toolbox
2. RealSense + EKF
3. Nav2 + 底盤 driver
4. AI 偵測 + demo_manager runtime（demo_manager_node + person_target_recovery_node）
```

`start_mapping.sh` 只啟動第 1~3 層，讓你可以用 RViz 設 goal 建圖，
再用 `record_map.sh` 儲存。

## 建置方式

來源結構：

```text
scripts/                  start/stop/demo_common + record_map
scripts/for_rtabmap/      RTAB-Map demo scripts + README
```

用 colcon 編譯：

```bash
cd slam_toolbox_rf2o
colcon build --merge-install --packages-select demo_script_bringup
```

install 會是扁平結構：

```text
install/start_demo.sh
install/start_demo_static.sh
install/start_mapping.sh
install/stop_demo.sh
install/record_map.sh
install/demo_common.sh
install/for_rtabmap/start_demo.sh
install/for_rtabmap/stop_demo.sh
install/for_rtabmap/README.md
install/for_rtabmap/README.zh-TW.md
```

把 `install/.` 複製到：

```text
/nvt_ros2_example/demo_script/
```

不要複製 `install/` 子目錄到板子。

## 1. 檔案

| 檔案 | 用途 |
|---|---|
| `scripts/start_demo.sh` | 完整 demo，slam_toolbox mapping 模式 + AI |
| `scripts/start_demo_static.sh` | 完整 demo，slam_toolbox localization 模式 + AI |
| `scripts/start_mapping.sh` | mapping stack：SLAM + EKF + Nav2，不含 AI |
| `scripts/record_map.sh` | 儲存 pgm/yaml + posegraph（呼叫 `$SLAM_DIR/save_map.sh`） |
| `scripts/stop_demo.sh` | 關閉三個 start script 啟動的 stack |
| `scripts/demo_common.sh` | 共用函式（由 start script source） |
| `scripts/for_rtabmap/start_demo.sh` | 完整 RTAB-Map demo + AI |
| `scripts/for_rtabmap/stop_demo.sh` | 停止 RTAB-Map demo |

## 2. Mapping 與 localization 的差別

| | `start_demo.sh` | `start_demo_static.sh` | `start_mapping.sh` |
|---|---|---|---|
| SLAM 模式 | `mapping` | `localization` | `mapping` |
| SLAM 參數 | `$SLAM_DIR/slam_toolbox_rf2o.yaml` | `$SLAM_DIR/slam_toolbox_localization.yaml` | `$SLAM_DIR/slam_toolbox_rf2o.yaml` |
| Posegraph | 不需要 | 必須存在 | 不需要 |
| 建立新 map | 是 | 否 | 是 |
| 啟動 EKF + Nav2 | 是 | 是 | 是 |
| 啟動 AI | 是 | 是 | 否 |

Mapping 流程：

```bash
./start_mapping.sh
# 用 RViz 的 2D Goal Pose / NavigateToPose 移動建圖
./record_map.sh demo_map
./stop_demo.sh
```

`record_map.sh` 會寫出：

```text
maps/demo_map.pgm
maps/demo_map.yaml
maps/demo_map.posegraph
maps/demo_map.data
```

必須在 mapping 模式執行中呼叫。

Localization 會從 localization yaml 的 `map_file_name` 讀取：

```text
<map_file_name>.posegraph
<map_file_name>.data
```

建立 posegraph 的方式（mapping 模式）：

```bash
./start_mapping.sh
./record_map.sh demo_map
./stop_demo.sh
```

接著把 `map_file_name` 設到 `slam_toolbox_localization.yaml`。

## 3. 使用方式

從部署後的 prefix（`/nvt_ros2_example/demo_script`）或來源樹的
`demo_script/scripts` 執行。

Mapping demo：

```bash
cd /nvt_ros2_example/demo_script
./start_demo.sh
```

Localization demo：

```bash
cd /nvt_ros2_example/demo_script
./start_demo_static.sh
```

不含 AI 的 mapping stack：

```bash
cd /nvt_ros2_example/demo_script
./start_mapping.sh
```

mapping 時儲存 map：

```bash
./record_map.sh demo_map
```

停止任何一個：

```bash
./stop_demo.sh
```

常用變化：

```bash
START_AI=0 ./start_demo.sh          # 只跑 SLAM + EKF + Nav2
DRIVER_IMPL=raspbot ./start_demo.sh        # 改用新的 raspbot_driver
DRIVER_IMPL=raspbot ./start_demo_static.sh # 同上，localization 模式
SLAM_PARAMS=/path/to/other.yaml ./start_demo_static.sh
NAV2_PARAMS_FILE=$NAV2_DIR/nav2_params_plugin.yaml ./start_demo.sh
```

`DRIVER_IMPL` 會傳給 `$NAV2_DIR/start_nav2.sh`；預設是 `yahboom`（原廠
`Mcnamu_driver`），保留作為回滾路徑。`raspbot` 會從
`/root/raspybot/raspbot_driver_ws` 啟動 `raspbot_driver`，並維持 legacy
`/ultrasonic` topic 給既有的 DeadbandController。

## 4. 環境變數

| 變數 | 預設 | 說明 |
|---|---|---|
| `ROS_DOMAIN_ID` | `99` | DDS domain |
| `START_AI` | `1` | 啟動 AI / recovery / demo manager |
| `NAV2_PARAMS_FILE` | `$NAV2_DIR/nav2_params_plugin.yaml` | Nav2 參數 |
| `SLAM_PARAMS` | `$SLAM_DIR/...` yaml | SLAM 參數 |
| `SLAM_DIR` | `/nvt_ros2_example/slam_toolbox` | 獨立 SLAM script |
| `RF2O_DIR` | `/nvt_ros2_example/rf2o` | 獨立 rf2o script |
| `NAV2_DIR` | `/nvt_ros2_example/nav2` | 獨立 Nav2 script |
| `EKF_DIR` | `/nvt_ros2_example/ekf` | 獨立 EKF script |
| `START_REALSENSE` | `0` | 保持 0；相機由 EKF 啟動 |
| `CAMERA_X` / `CAMERA_Y` / `CAMERA_Z` | `0.0` / `0.0` / `0.0` | AI TF 偏移 |
| `LOG_DIR` | 系統暫存目錄 | log 與 PID |
| `RUN_DIR` | 系統暫存目錄 | PID 檔目錄 |
| `TOPIC_WAIT` | `20` | 每個 topic 等待秒數 |
| `DEMO_READY_WAIT` | `40` | `/demo/person_markers` 的較長（非致命）等待時間，主要給 localization 模式用 |

`START_REALSENSE=1` 在 demo 中沒有用途，因為相機由 EKF 啟動。
若目前沒有相機在跑，script 會直接報錯。

在 localization 模式（`start_demo_static.sh`）下，slam_toolbox 會載入
posegraph，`map` frame 可能短暫不存在、TF buffer 需要時間補資料。現在 script
會先等 `map -> base_link` 連續 3 次查詢成功才啟動 Nav2/AI，並給 demo_manager
最多 `DEMO_READY_WAIT` 秒發布 `/demo/person_markers`。逾時只是警告，demo 仍會
啟動；該 topic 通常幾秒後就會出現。

## 5. 各層啟動內容

### SLAM

透過 `$SLAM_DIR` + `$RF2O_DIR`：

```text
rplidar_node
rf2o_laser_odometry_node
base_footprint -> base_link -> laser 的 static TF
slam_toolbox（online_async 或 localization）
```

### EKF

透過 `$EKF_DIR/start_ekf.sh`：

```text
realsense2_camera
base_footprint -> camera_link static TF
robot_localization ekf_node
```

### Nav2

透過 `$NAV2_DIR/start_nav2.sh`：

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
$DEMO_INSTALL/start_demo_only.sh
  |- person_target_recovery_node   (config/person_target_recovery.yaml)
  `- demo_manager_node             (config/demo_manager.yaml)
```

AI 使用 EKF 已經啟動的相機，不需要第二顆 RealSense。

demo_manager runtime 1.1.0 起，這兩個 node 都在 `demo_manager` 套件裡，
由 `start_demo_only.sh` 一起啟動、`stop_demo_only.sh` 一起停止；
原本獨立的 `person_target_recovery` prefix 已不再使用。

已安裝版本：`demo_manager` 1.1.0，位於 `$DEMO_INSTALL`
（`/nvt_ros2_example/demo_manager/install`，同步一份在
`/root/ros2_ws/install/demo_manager`）。`stop_demo_only.sh` 會先呼叫
`/demo/stop` 服務，再退回 SIGTERM/SIGKILL。

vendor tarball 的 `setup.bash`/`setup.sh` 會 chain 到打包機器的
`ai_human_detection` 路徑，而且每次 source 都執行 `sudo apt-get install`；
安裝後兩者都已移除（部署紀錄見 repo）。

## 6. 啟動順序

```text
start_demo.sh
  ├─ $SLAM_DIR/start_slam.sh mapping|localization
  ├─ $RF2O_DIR/start.sh
  ├─ $EKF_DIR/start_ekf.sh
  ├─ 等 /map + /odometry/filtered + TF map -> base_link
  ├─ $NAV2_DIR/start_nav2.sh
  └─ AI node -> AI TF -> recovery -> demo manager
```

SLAM/EKF 最多等 60 秒，逾時就停止，不會硬啟動 Nav2。

`start_mapping.sh` 使用相同順序，但停在 Nav2，不會啟動 AI。

## 7. Log 與停止

log 與 PID 會放在 `LOG_DIR`（預設是系統暫存目錄）：

```text
<LOG_DIR>/ai_node.log
<LOG_DIR>/ai_tf.log
<LOG_DIR>/demo_runtime.log        (person_target_recovery_node + demo_manager_node)
<LOG_DIR>/ai_pids
```

停止：

```bash
./stop_demo.sh
```

`stop_demo.sh` 會先停有記錄的 AI process，再停獨立 slam_toolbox / rf2o，最後停 Nav2 / EKF / driver。

## 8. 注意

```text
- start_mapping.sh 啟動 SLAM + EKF + Nav2，但不含 AI。
- Mapping demo（start_demo.sh）與 start_mapping.sh 不需要 posegraph。
- Static demo 需要 .posegraph + .data，請在 mapping 模式存。
- 相機由 EKF 擁有，不要開第二顆 RealSense。
- rf2o 由 `$RF2O_DIR/start.sh` 單獨啟動；slam_toolbox 不會啟動它。
- Nav2 預設是 DeadbandController（simple 模式）。
- START_AI=0 可以只起導航，不跑 AI。
- AI helper 會自己 source vendor workspace；這個資料夾只負責組合與 log。
- colcon package 名稱是 `demo_script_bringup`；部署位置是
  `/nvt_ros2_example/demo_script`。
```
