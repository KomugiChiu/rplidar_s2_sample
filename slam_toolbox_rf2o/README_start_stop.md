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
└── rf2o_params.yaml               # rf2o 參數
```

```bash
cd /nvt_ros2_example/slam_toolbox_rf2o
./start_slam_toolbox_rf2o.sh          # 建圖
QUIET=1 ./start_slam_toolbox_rf2o.sh   # 主控台乾淨，node 輸出寫檔
NO_LOG=1 ./start_slam_toolbox_rf2o.sh  # 完全不產生 log
SLAM_MODE=localization ./start_slam_toolbox_rf2o.sh
./stop_slam_toolbox_rf2o.sh           # 停止
```

與 rtabmap 那套相同的行為：
- 每個啟動的程序都記在 `/tmp/slam_toolbox_rf2o/pids`（`pid name mode`）
- 已經有實例在跑時會拒絕啟動第二份
- `Ctrl+C` 會觸發 trap → 呼叫 stop script → 全部收乾淨
- 可從任何工作目錄執行（ini/yaml 與 stop script 都相對於腳本本身解析）

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

## 兩個踩過的坑

1. **不要用空值參數**：`ros2 run ... -p init_pose_from_topic:=` 會被 rcl 的參數解析器
   拒絕（`RCLInvalidROSArgsError`）→ node 直接 abort。改成用 `--params-file`。
2. **`set -o pipefail` + 會失敗的 `ps`**：stop script 裡
   `pgid="$(ps -o pgid= -p "$pid" | tr -d ' ')"` 在行程剛好消失時會讓整支腳本提早結束
   → 必須加 `|| true`。

## 驗收

```bash
ros2 topic hz /scan          # ~10 Hz
ros2 topic hz /odom_rf2o     # ~10 Hz
ros2 topic echo /map --once  # 有 width/height/resolution
ros2 run tf2_ros tf2_echo map laser   # z = 0.20
```
