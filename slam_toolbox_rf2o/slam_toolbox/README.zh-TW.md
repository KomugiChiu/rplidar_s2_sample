[English](README.md) | 繁體中文

# slam_toolbox

獨立 SLAM 前端：

```text
RPLIDAR S2 -> /scan
static TF: base_footprint -> base_link -> laser
slam_toolbox -> /map + map -> odom
```

這裡**不會**啟動 rf2o。rf2o 在：

```text
../rf2o/
  install/              rf2o_laser_odometry install tree
  rf2o_params.yaml
  start.sh / stop.sh
```

完整 mapping demo（含 EKF、Nav2、AI）請用：

```text
../slam_toolbox_rf2o/demo_script/start_mapping.sh
```

## 建置配置

原始碼配置：

```text
scripts/    *.sh
config/     *.yaml
```

使用 colcon 建置：

```bash
colcon build --merge-install --packages-select slam_toolbox_bringup
```

install 會是平坦結構：

```text
install/start_mapping.sh
install/start_localization.sh
install/stop.sh
install/save_map.sh
install/check_deps.sh
install/slam_toolbox_rf2o.yaml
install/slam_toolbox_localization.yaml
```

把 `install/.` 的內容複製到板端：

```text
/nvt_ros2_example/slam_toolbox/
```

板上不要再有 `install/` 子資料夾。

## 1. 檔案

| 檔案 | 用途 |
|---|---|
| `start_mapping.sh` | 啟動 lidar + static TF + slam_toolbox mapping |
| `start_localization.sh` | 啟動 lidar + static TF + slam_toolbox localization |
| `stop.sh` | 停止 lidar / static TF / slam_toolbox |
| `save_map.sh` | 儲存 pgm/yaml + posegraph（只能 mapping） |
| `check_deps.sh` | 檢查 / 安裝依賴 |
| `_start_slam.sh` | 共用實作 |

## 2. Mapping

```bash
cd /nvt_ros2_example/slam_toolbox
./start_mapping.sh
```

會啟動：

```text
rplidar_node
static_transform_publisher（base_footprint -> base_link）
static_transform_publisher（base_link -> laser）
slam_toolbox online_async_launch.py
```

它**不會**啟動 rf2o 或 EKF。
若要能實際建圖與導航，請用 `../slam_toolbox_rf2o/demo_script/start_mapping.sh`，
它會另外啟動 rf2o、EKF、Nav2。

如果要手動分段：

```text
1. ./start_mapping.sh
2. ../rf2o/start.sh
3. ../ekf/start_ekf.sh
4. 用 RViz goal 移動建圖
5. ./save_map.sh demo_map
6. ./stop.sh
   ../rf2o/stop.sh
```

## 3. Localization

```bash
cd /nvt_ros2_example/slam_toolbox
./start_localization.sh
```

`slam_toolbox_localization.yaml` 的 `map_file_name` 必須對應：

```text
<map_file_name>.posegraph
<map_file_name>.data
```

啟動時車子要在 `map_start_pose`，或 `map_start_at_dock: true` 的 dock 上。

## 4. 儲存 map

只能在 mapping 模式執行中：

```bash
./save_map.sh                  # maps/slam_map_<timestamp>
./save_map.sh demo_map         # maps/demo_map
```

輸出：

```text
maps/demo_map.pgm
maps/demo_map.yaml
maps/demo_map.posegraph
maps/demo_map.data
```

`save_map.sh` 會：

```text
1. map_saver_cli -f <prefix>
2. /slam_toolbox/serialize_map
3. 確認四個檔案都存在
```

localization 模式不能用 `serialize_map`；script 會偵測並提早退出。

## 5. rf2o

```bash
cd /nvt_ros2_example/rf2o
./start.sh
./stop.sh
```

`start.sh` 需要 `/scan` 與 `base_footprint -> laser` TF，
這兩者由 slam_toolbox 的 start script 提供。

rf2o 參數：

```text
../rf2o/rf2o_params.yaml
```

輸出：

```text
/odom_rf2o
```

`publish_tf: false`；`odom -> base_footprint` 由 EKF 發。

## 6. 依賴

apt：

```text
ros-jazzy-slam-toolbox
ros-jazzy-nav2-map-server
ros-jazzy-tf2-ros
```

workspace：

```text
../rplidar_ros
../rf2o
```

檢查：

```bash
./check_deps.sh --check-only
./check_deps.sh
```

## 7. 環境變數

| 變數 | 預設 | 說明 |
|---|---|---|
| `ROS_DOMAIN_ID` | `99` | DDS domain |
| `SLAM_PARAMS` | mapping / localization yaml | SLAM 參數 |
| `RPLIDAR_WS` | `../rplidar_ros` | rplidar workspace |
| `SERIAL_PORT` | `/dev/ttyUSB0` | RPLIDAR S2 serial port |
| `SKIP_LIDAR` | `0` | `1` = 沿用已在跑的 rplidar_node |
| `LASER_YAW_DEG` | `180` | S2 安裝方向 |
| `RUN_DIR` | 系統暫存目錄 | PID / logs |
| `MAP_DIR` | `maps/` | `save_map.sh` 輸出 |
| `DRY_RUN` | `0` | `stop.sh` dry run |

## 8. 注意

```text
- rf2o 與 slam_toolbox 是分開的。
- slam_toolbox 負責 map -> odom。
- EKF 負責 odom -> base_footprint。
- 只跑 start_mapping.sh 不會有 odometry；必須有 EKF。
- 完整 mapping stack 請用 ../slam_toolbox_rf2o/demo_script/start_mapping.sh。
```
