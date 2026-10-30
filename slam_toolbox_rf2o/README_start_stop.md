# slam_toolbox + rf2o（RPLIDAR S2，ROS2 Jazzy）— start / stop 腳本

以 `rtabmap/lidar/` 那套 start/stop 腳本的骨架重寫的 SLAM 流程：
**rf2o 提供 odom、slam_toolbox 提供地圖**。

## 分工

| 元件 | 產出 |
|---|---|
| `rplidar_ros` | `/scan`（frame `laser`） |
| `rf2o_laser_odometry` | `/odom_rf2o` + TF `odom → base_footprint`（動態） |
| `slam_toolbox` | `/map` + TF `map → odom`（動態） |
| `static_transform_publisher` ×2 | `base_footprint → base_link`、`base_link → laser`（static） |

完整 TF 樹：

```
map
└── odom
     └── base_footprint
          └── base_link
               └── laser
```

> `odom → base_footprint` 由 rf2o 動態發，**不要再加 static 版**（TF 衝突）。

## 板上

```bash
# 板上位置
/nvt_ros2_example/slam_toolbox_rf2o/
├── start_slam_toolbox_rf2o.sh     # 起整條流程（PID 檔 + 乾淨輸出選項）
├── stop_slam_toolbox_rf2o.sh      # 收掉整條流程（含孤兒）
├── slam_toolbox_rf2o.yaml         # slam_toolbox 參數
├── rf2o_params.yaml               # rf2o 參數
└── rviz_slam_toolbox_rf2o.rviz    # 這條流程專用的 RViz（map/scan/odom/TF）
```

```bash
cd /nvt_ros2_example/slam_toolbox_rf2o
./start_slam_toolbox_rf2o.sh           # 建圖（背景執行，腳本立刻返回）
QUIET=0 ./start_slam_toolbox_rf2o.sh   # 把 node 輸出顯示在主控台
NO_LOG=1 ./start_slam_toolbox_rf2o.sh  # 完全不產生 log
SLAM_MODE=localization ./start_slam_toolbox_rf2o.sh
./stop_slam_toolbox_rf2o.sh            # 停止
```

與 rtabmap 那套相同的行為：
- 每個啟動的程序都記在 `/tmp/slam_toolbox_rf2o/pids`（`pid name mode`）
- 已經有實例在跑時會拒絕啟動第二份
- **啟動後立即返回，不佔住 console**（`setsid` + 背景），要停請跑 stop script
- 可從任何工作目錄執行（yaml 與 stop script 都相對於腳本本身解析）
- 只有啟動過程失敗（process 死掉）時，EXIT trap 才會把已起的東西收回來

## 重要參數

| 參數 | 預設 | 說明 |
|---|---|---|
| `LASER_YAW_DEG` | **180** | S2 實際安裝方向；與 `start_rtabmap_laser.sh` 相同（實測：往前推 20 cm，odom x 由 -0.150 → +0.386）。設 0 會讓 odom／地圖左右前後鏡像 |
| `BASE_LINK_Z` / `LASER_Z` | 0.10 / 0.20 | `base_footprint → base_link → laser` 的高度 |
| `QUIET` | 1 | 1 = 主控台只留摘要，node 輸出寫檔 |
| `SLAM_MODE` | mapping | `mapping` = `online_async_launch.py`；`localization` = `localization_launch.py` |

slam_toolbox 參數（`slam_toolbox_rf2o.yaml`）刻意調低負載：
`online_async`（同步模式在這塊 4 核 ARM 板上會吃到 220% CPU）、
`transform_publish_period: 0.1`、`minimum_travel_distance: 0.2`、
`minimum_travel_heading: 0.2`、`minimum_time_interval: 0.5`
（原本 0.0 = 站著不動也每張掃描都進 SLAM）。調整後 slam_toolbox 約 3% CPU。

## rf2o_laser_odometry 安裝（板上）

```bash
mkdir -p /root/rf2o_ws/src && cd /root/rf2o_ws/src
git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git
source /opt/ros/jazzy/setup.bash
cd /root/rf2o_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

（板上已建好，`/root/rf2o_ws/install/rf2o_laser_odometry`；編譯約 3 分 40 秒，
依賴 rclcpp / sensor_msgs / nav_msgs / tf2 / Eigen / Boost 板上已有，可跳過 rosdep。）

## 三個踩過的坑

1. **不要用空值參數**：`ros2 run ... -p init_pose_from_topic:=` 會被 rcl 的參數解析器
   拒絕（`RCLInvalidROSArgsError`）→ node 直接 abort。改成用 `--params-file`。
2. **`set -o pipefail` + 會失敗的 `ps`**：stop script 裡
   `pgid="$(ps -o pgid= -p "$pid" | tr -d ' ')"` 在行程剛好消失時會讓整支腳本提早結束
   → 必須加 `|| true`。
3. **rf2o 只在第一張掃描時讀一次 TF**：static TF 還沒起來就啟動的話，log 會出現
   `"base_footprint" passed to lookupTransform argument target_frame does not exist`，
   然後它默默假設 `laser == base`，odom 就整個轉了 180°（往前推 x 變小）。
   → start script 現在會先等 `tf2_echo base_footprint laser` 成功才啟動 rf2o。
   驗證方式：rf2o log 第一行應該是
   `Laser odom [x,y,yaw]=[0.000000 0.000000 3.141593]` +
   `Robot-base odom [x,y,yaw]=[0.000000 0.000000 0.000000]`。

## 驗收

```bash
ros2 topic hz /scan          # ~10 Hz
ros2 topic hz /odom_rf2o     # ~10 Hz
ros2 topic echo /map --once  # 有 width/height/resolution
ros2 run tf2_ros tf2_echo map laser   # z = 0.20
```
