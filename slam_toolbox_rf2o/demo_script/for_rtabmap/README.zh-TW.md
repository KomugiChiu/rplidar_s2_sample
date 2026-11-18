[English](README.md) | 繁體中文

# for_rtabmap

這是 [demo_script](../README.zh-TW.md) 流程的 **RTAB-Map** 版本：2D 光達建圖改由
RTAB-Map 負責，而不是 slam_toolbox。

兩個進入點：

```text
start_demo.sh   RTAB-Map 建圖 + EKF + Nav2 + AI
stop_demo.sh    停止 start_demo.sh 起的所有東西
```

沒有 localization/static 變體，也沒有存圖步驟：RTAB-Map 一直建自己的地圖，存在
它的資料庫裡。

## 1. Odometry 鏈

```text
/scan ─> rtabmap_odom/icp_odometry (publish_tf=false)
             │
             └─> /odom_icp ──┐
                             ├─> robot_localization/ekf_node
/camera/camera/gyro/sample ──┘        (ekf_params_icp.yaml)
                                      │
                                      ├─> /odometry/filtered
                                      └─> TF odom -> base_footprint

rtabmap_slam/rtabmap (subscribe_scan, odom_topic=/odometry/filtered)
    ├─> /rtabmap/map
    └─> TF map -> odom
```

`odom -> base_footprint` **只有 EKF 發佈**：`icp_odometry` 用 `publish_tf=false`，
而 RTAB-Map 吃的是濾波後的 odom，跟 slam_toolbox 那條線吃 rf2o + EKF 的方式一致。

`ODOM_SOURCE=external` 會把 ICP 換回 rf2o（`/odom_rf2o`），並改用原本的
`ekf_params.yaml`；EKF 之後的鏈路完全相同。詳見 §7。

## 2. 四層

```text
1. RPLIDAR S2 + 靜態 TF + icp_odometry + RTAB-Map
2. RealSense + 相機 TF + EKF
3. Nav2 + 底盤驅動            (nav2_params_rtabmap.yaml)
4. AI 偵測 + person_target_recovery + demo_manager
```

即使 `START_AI=0`，第 2 層的相機還是會啟動，因為 EKF 要融合 D455 的 gyro；
沒有相機就沒有 yaw rate 可用。

## 3. 檔案

| 檔案 | 用途 |
|---|---|
| `start_demo.sh` | 完整 demo：RTAB-Map + EKF + Nav2 + AI |
| `stop_demo.sh` | 停止 `start_demo.sh` 起的堆疊 |
| `README.md` | 英文版 |
| `README.zh-TW.md` | 本檔 |

## 4. 用法

```bash
cd /nvt_ros2_example/demo_script/for_rtabmap
./start_demo.sh
```

變化用法：

```bash
START_AI=0 ./start_demo.sh          # 只起建圖 + EKF + Nav2
RECORD=1 ./start_demo.sh            # 另外把 /cmd_vel_nav 錄成 CSV
DRY_RUN=1 ./stop_demo.sh            # 只印出會停哪些，不動手
ODOM_SOURCE=external ./start_demo.sh    # 改用 rf2o + /odom_rf2o
```

停止：

```bash
./stop_demo.sh
```

rplidar 是獨佔的：要跑這個之前，先停掉 slam_toolbox demo（或任何佔用
`/dev/ttyUSB0` 的流程）。

## 5. 環境變數

| 變數 | 預設 | 說明 |
|---|---|---|
| `ROS_DOMAIN_ID` | `99` | DDS domain |
| `START_AI` | `1` | 是否起 AI / recovery / demo manager |
| `RECORD` | `0` | 把 `/cmd_vel_nav` 錄成 CSV |
| `ODOM_SOURCE` | `icp` | `icp` = icp_odometry，`external` = rf2o |
| `ODOM_TOPIC` | `/odometry/filtered` | RTAB-Map 與 Nav2 吃的 odom |
| `MAP_TOPIC` | `/rtabmap/map` | RTAB-Map 發佈的地圖 topic |
| `RTABMAP_DIR` | `/nvt_ros2_example/rtabmap` | RTAB-Map 套件腳本 |
| `RTABMAP_NAV2_PARAMS_FILE` | `$RTABMAP_DIR/nav2_params_rtabmap.yaml` | Nav2 參數 |
| `EKF_PARAMS_FILE` | `$EKF_DIR/ekf_params_icp.yaml`（`external` 時為 `ekf_params.yaml`） | EKF 參數 |
| `RF2O_DIR` | `/nvt_ros2_example/rf2o` | rf2o 腳本（只有 `external` 用到） |
| `EKF_DIR` | `/nvt_ros2_example/ekf` | EKF 腳本與參數 |
| `NAV2_DIR` | `/nvt_ros2_example/nav2` | Nav2 腳本 |
| `LOG_DIR` / `RUN_DIR` | `/tmp/demo_script` | log 與 PID 檔 |
| `TOPIC_WAIT` | `20` | 等每個 AI topic 的秒數 |

單獨跑 `$RTABMAP_DIR/start_rtabmap.sh` 另外接受：

| 變數 | 預設 | 說明 |
|---|---|---|
| `SKIP_LIDAR` | `0` | `1` = 沿用已在跑的 rplidar_node |
| `SCAN_TOPIC` | `/scan` | 光達 topic |
| `SERIAL_PORT` | `/dev/ttyUSB0` | RPLIDAR S2 序列埠 |
| `DATABASE_PATH` | `$RTABMAP_DIR/rtabmap.db` | RTAB-Map 資料庫 |
| `DELETE_DB_ON_START` | `0` | `1` = 從空資料庫開始 |
| `ICP_TOPIC` | `/odom_icp` | icp_odometry 輸出 |
| `BASE_FRAME` / `LASER_FRAME` | `base_footprint` / `laser` | TF frame |
| `RUN_DIR` / `LOG_DIR` | `/tmp/rtabmap_only` | PID 檔與 log |

## 6. 啟動順序

```text
start_demo.sh
  ├─ $RTABMAP_DIR/start_rtabmap.sh
  │    ├─ rplidar_node
  │    ├─ base_footprint -> base_link -> laser 靜態 TF
  │    ├─ rtabmap_odom/icp_odometry   (publish_tf=false -> /odom_icp)
  │    └─ rtabmap.launch.py           (subscribe_scan, odom_topic=/odometry/filtered)
  ├─ [$RF2O_DIR/start.sh]             (只有 ODOM_SOURCE=external)
  ├─ $EKF_DIR/start_ekf.sh            (EKF_PARAMS_FILE=ekf_params_icp.yaml)
  ├─ 等 /rtabmap/map + /odometry/filtered + TF map -> base_link  (60 s)
  ├─ $NAV2_DIR/start_nav2.sh          (MAP_TOPIC=/rtabmap/map)
  └─ AI node -> AI TF -> recovery -> demo manager
```

## 7. icp_odometry 與 rf2o 的差別

| | `ODOM_SOURCE=icp`（預設） | `ODOM_SOURCE=external` |
|---|---|---|
| odom 來源 | `rtabmap_odom/icp_odometry` | `rf2o_laser_odometry_node` |
| 原始 odom topic | `/odom_icp` | `/odom_rf2o` |
| EKF 參數 | `ekf_params_icp.yaml` | `ekf_params.yaml` |
| TF `odom -> base_footprint` | EKF | EKF |
| EKF 參數要改 odom0 | 要（`/odom_icp`） | 不用 |

這個資料夾不會動到 slam_toolbox + rf2o 用的 `ekf_params.yaml`：ICP 版本放在
另一個檔案，由 `start_demo.sh` 用 `EKF_PARAMS_FILE` 指定。

為什麼 ICP 不能發 TF：`odom -> base_footprint` 已經由 EKF 負責。同一個
transform 有兩個發佈者會讓 TF 樹在兩個來源間跳動，症狀是 Nav2 出現
"transform too old" 中止，以及 RTAB-Map 重算 `map -> odom` 時地圖突然跳一下。

## 8. Log

```text
/tmp/rtabmap_only/logs/rplidar.log
/tmp/rtabmap_only/logs/icp_odometry.log
/tmp/rtabmap_only/logs/rtabmap.log
/tmp/rtabmap_only/logs/tf_base.log
/tmp/rtabmap_only/logs/tf_laser.log
/tmp/rtabmap_only/pids
/tmp/slam_toolbox_rf2o_ekf/logs/
/tmp/raspybot_nav2/logs/
/tmp/demo_script/*.log          (AI、recovery、demo manager)
```

`stop_demo.sh` 的停止順序：AI -> RTAB-Map（光達/TF/ICP）-> rf2o -> Nav2/底盤 ->
EKF/RealSense。`stop_rtabmap.sh` 與 `stop_ekf.sh` 會先對 process group 送
SIGTERM、等待，再送 SIGKILL 並按名稱掃一遍；因為 `rplidar_node`、`ekf_node`、
`realsense2_camera_node` 有時不理 SIGTERM，會一直佔著 `/dev/ttyUSB0`，
讓下一次啟動失敗。

## 9. 注意事項

```text
- 純光達建圖時看到這個警告是正常的，不影響建圖：
  "There is no image subscription, bag-of-words loop closure detection will be
   disabled..."
- 地圖只以 /rtabmap/map 與 rtabmap.db 的形式存在，這條流程沒有 record_map.sh
  的存圖步驟：Nav2 從空圖開始，邊走邊長圖。
- 資料庫會一直變大。要從乾淨的圖開始：
  DELETE_DB_ON_START=1 ./start_rtabmap.sh，或直接刪掉
  $RTABMAP_DIR/rtabmap.db。
- RTAB-Map 發佈 map -> odom，Nav2 要用 /rtabmap/map 與 /odometry/filtered，
  這也是 nav2_params_rtabmap.yaml 裡的設定。
- 同時間只能有一個行程佔用 rplidar 的序列埠。
- 這裡沒有 static/localization 模式；那要用 slam_toolbox 的
  ../start_demo_static.sh。
```
