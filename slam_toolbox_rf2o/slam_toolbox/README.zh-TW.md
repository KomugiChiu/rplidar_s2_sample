[English](README.md) | 繁體中文

# slam_toolbox

這個資料夾是 RPLIDAR S2 + rf2o + slam_toolbox 這一段的輔助 script：

```text
/scan -> rf2o -> /odom_rf2o
                slam_toolbox -> /map + map -> odom
```

這裡**不會**啟動 EKF、Nav2、AI 或底盤 driver。
完整 demo 請用 `../demo_script/`。

## 1. 檔案

| 檔案 | 用途 |
|---|---|
| `start_mapping.sh` | 啟動 lidar + rf2o + slam_toolbox，mapping 模式 |
| `start_localization.sh` | 啟動同一組 stack，localization 模式，載入 posegraph |
| `stop.sh` | 停止上面兩個 script 啟動的程序 |
| `save_map.sh` | 儲存 `/map` 的 pgm/yaml 與 posegraph |
| `check_deps.sh` | 檢查 / 安裝這條 flow 的依賴 |

## 2. 需要的 package

apt：

```text
ros-jazzy-slam-toolbox
ros-jazzy-nav2-map-server    # map_saver_cli
ros-jazzy-tf2-ros
```

Workspace：

```text
../rplidar_ros               # rplidar_ros（可用 RPLIDAR_WS 覆寫）
$HOME/rf2o_ws                # rf2o_laser_odometry（可用 RF2O_WS 覆寫）
```

launch 也會用到：

```text
../rf2o_params.yaml
../slam_toolbox_rf2o.yaml
../slam_toolbox_localization.yaml
```

## 3. Mapping 模式

```bash
cd slam_toolbox
./start_mapping.sh
```

等價的低階設定：

```text
SLAM_MODE=mapping
SLAM_PARAMS=slam_toolbox_rf2o.yaml
```

會啟動：

```text
rplidar_node
rf2o_laser_odometry_node
兩個 static TF：base_footprint -> base_link -> laser
slam_toolbox online_async_launch.py
```

檢查：

```bash
ros2 topic hz /scan
ros2 topic hz /odom_rf2o
ros2 topic hz /map
timeout 5 ros2 run tf2_ros tf2_echo map base_link
```

log 由低階 script 產生；`QUIET=0` 可以看到更多輸出。

## 4. 儲存目前的 map

請在 **mapping 模式執行中** 使用：

```bash
./save_map.sh                 # maps/slam_map_<timestamp>
./save_map.sh demo_map        # maps/demo_map
```

輸出：

```text
maps/<name>.pgm
maps/<name>.yaml
maps/<name>.posegraph
maps/<name>.data
```

它會做：

```text
1. ros2 run nav2_map_server map_saver_cli -f <prefix>
2. ros2 service call /slam_toolbox/serialize_map ...
3. 確認四個檔案都存在
```

重要：

```text
serialize_map 只能在 mapping 模式使用。
localization 模式下服務雖然存在，但會拒絕：
"Cannot call serialize map in localization mode"。
```

`save_map.sh` 會先偵測目前是哪一種 node，如果是 localization 就直接退出。
它也會檢查 `.posegraph` / `.data`，因為 SerializePoseGraph 即使沒有寫檔，
也可能回傳 `result=0`。

## 5. Localization 模式

```bash
cd slam_toolbox
./start_localization.sh
```

它會從 `slam_toolbox_localization.yaml` 讀 `map_file_name`，並要求：

```text
<map_file_name>.posegraph
<map_file_name>.data
```

`save_map.sh` 最後會印出可用的 prefix：

```text
Use this prefix in a localization yaml:
  map_file_name: "<prefix>"
```

如果 `map_start_pose` / `map_start_at_dock` 都沒設定，
`start_localization.sh` 會印出 warning。

啟動時請把車放在當初儲存 map 的起點（或 dock），
否則 localization 的初始猜測會錯。

## 6. 停止

```bash
./stop.sh
DRY_RUN=1 ./stop.sh
KEEP_DRIVER=1 ./stop.sh    # 保留別人的 rplidar_node
```

## 7. 依賴檢查

```bash
./check_deps.sh --check-only
./check_deps.sh               # 安裝缺少的 apt package
```

檢查項目：

```text
ROS 2 Jazzy / ros2 CLI
ros-jazzy-slam-toolbox
ros-jazzy-nav2-map-server
ros-jazzy-tf2-ros
rplidar_ros workspace / rplidar_node
rf2o_ws / rf2o_laser_odometry_node
slam_toolbox async + localization nodes
nav2_map_server map_saver_cli
tf2_ros static_transform_publisher
slam_toolbox_rf2o.yaml
slam_toolbox_localization.yaml
rf2o_params.yaml
maps/
localization posegraph（只警告）
RPLIDAR S2 USB
dialout group
```

## 8. 環境變數

| 變數 | 預設 | 說明 |
|---|---|---|
| `ROS_DOMAIN_ID` | `99` | DDS domain |
| `SLAM_PARAMS` | mapping / localization yaml | 參數檔覆寫 |
| `RPLIDAR_WS` | `../rplidar_ros` | rplidar workspace |
| `RF2O_WS` | `$HOME/rf2o_ws` | rf2o workspace |
| `MAP_DIR` | `../maps` | `save_map.sh` 輸出目錄 |
| `MAPS_DIR` | `../maps` | `check_deps.sh` 檢查用 |
| `QUIET` | `1` | 傳給低階 SLAM script |
| `NO_LOG` | `0` | 傳給低階 SLAM script |
| `KEEP_DRIVER` | `0` | 停止時保留 `rplidar_node` |
| `DRY_RUN` | `0` | 只列出 stop 會做什麼 |

## 9. 注意

```text
- rf2o 的 publish_tf 必須是 false；odom -> base_footprint 由 EKF 發。
- slam_toolbox 負責 map -> odom。
- 這個資料夾不啟動 RealSense 或 EKF。
- mapping 用 online_async_launch.py；localization 用 localization_launch.py。
- 存 posegraph 是 mapping 模式的操作，不是 localization 模式。
```
