[English](README.md) | 繁體中文

# ekf

這個資料夾負責 D455 IMU 與雷射里程計的融合，輸出 Nav2 使用的
`/odometry/filtered` 與 `odom -> base_footprint` TF。

```text
rf2o (/odom_rf2o) ─┐
                   ├─ ekf_node ──► /odometry/filtered
D455 gyro ─────────┘                TF odom -> base_footprint
```

## 1. 檔案

```text
ekf/
├── ekf_params.yaml     # robot_localization 參數
├── start_ekf.sh        # 啟動 D455 + static TF + ekf_node
├── stop_ekf.sh         # 停止上述程序
└── check_deps.sh       # 依賴檢查 / apt 安裝
```

## 2. 需要的 package

全部由 apt 提供，**不需要手動編譯**。

| Package | 提供的東西 | 用途 |
|---|---|---|
| `ros-jazzy-robot-localization` | `ekf_node` | 融合 rf2o 與 gyro |
| `ros-jazzy-realsense2-camera` | `realsense2_camera_node` | D455 gyro / accel / RGBD |
| `ros-jazzy-tf2-ros` | `static_transform_publisher` | `base_footprint -> camera_link` |

`check_deps.sh` 另外會用到：

```text
python3-yaml    # 檢查 ekf_params.yaml 的 key / value
usbutils        # lsusb 檢查 D455
```

EKF 直接使用 apt 的 `realsense2_camera`。
`../realsense-ros` 這個 workspace 是 AI 流程自己啟動相機時才使用，
不是 `start_ekf.sh` 的來源。

## 3. 目前融合的資料

`ekf_params.yaml` 的設定：

```yaml
frequency: 20.0
two_d_mode: true
world_frame: odom
publish_tf: true
```

### 3.1 `odom0`：rf2o

```yaml
odom0: /odom_rf2o
odom0_config: [true,  true,  false,
               false, false, true,
               false, false, false,
               false, false, false,
               false, false, false]
odom0_differential: true
```

只使用：

```text
x
y
yaw
```

不使用 rf2o 的 twist（`vx / vy / vyaw`）。

原因：

```text
雷達以 180 度安裝，rf2o 的 twist 方向是反的
前進時 rf2o 可能回報 vx ≈ -0.4
直接把 twist 融合進去會和 pose 互相拉扯
```

`odom0_differential: true` 會讓 EKF 把連續的 pose 差值轉成速度，
停車時速度可以歸零，避免 filtered pose 在停車後繼續往前爬。

### 3.2 `imu0`：D455 gyro

```yaml
imu0: /camera/camera/gyro/sample
imu0_config: [false, false, false,
              false, false, false,
              false, false, false,
              false, false, true,
              false, false, false]
```

只使用：

```text
yaw 角速度（vyaw）
```

不使用 IMU 的 orientation、roll、pitch 或加速度。
IMU frame 會透過 TF 轉到 `base_footprint`。

## 4. 沒有融合的資料

```text
rf2o 的 twist（vx / vy / vyaw）
D455 的 orientation / roll / pitch
D455 的加速度
輪速 odometry（這台車沒有 encoder）
GPS / navsat
```

## 5. 輸出

```text
/odometry/filtered
TF odom -> base_footprint
```

TF 分工：

```text
slam_toolbox : map -> odom
ekf_node     : odom -> base_footprint
static TF    : base_footprint -> base_link -> laser / camera_link
```

`rf2o` 的 `publish_tf` 必須是 `false`，否則會和 EKF 搶
`odom -> base_footprint`。設定在 `../rf2o_params.yaml`。

## 6. 主要參數

| 參數 | 值 | 說明 |
|---|---:|---|
| `frequency` | `20.0` | 輸出頻率 |
| `sensor_timeout` | `0.3` | 感測器逾時 |
| `two_d_mode` | `true` | 只估平面 x / y / yaw |
| `publish_tf` | `true` | 發 `odom -> base_footprint` |
| `world_frame` | `odom` | 世界座標使用 odom |
| `odom_frame` | `odom` | |
| `base_link_frame` | `base_footprint` | |
| `map_frame` | `map` | 只作為參考，`map -> odom` 由 slam_toolbox 發 |
| `odom0` | `/odom_rf2o` | rf2o |
| `imu0` | `/camera/camera/gyro/sample` | D455 gyro |
| `use_control` | `false` | 不使用 cmd_vel 當輸入 |

## 7. 啟動

前提：SLAM / rf2o 已經啟動，且 `/odom_rf2o` 有資料。

```bash
cd ekf
./start_ekf.sh
```

會啟動：

```text
realsense2_camera（gyro + accel + RGBD）
static_transform_publisher（base_footprint -> camera_link）
robot_localization ekf_node
```

停止：

```bash
./stop_ekf.sh
```

可用環境變數：

| 變數 | 預設 | 說明 |
|---|---|---|
| `EKF_PARAMS_FILE` | `ekf_params.yaml` | 參數檔 |
| `CAM_X` / `CAM_Y` / `CAM_Z` | `0.08` / `0.0` / `0.10` | 相機相對 `base_footprint` |
| `ROS_DISTRO` | `jazzy` | |
| `RUN_DIR` / `PID_FILE` / `LOG_DIR` | 系統暫存目錄 | PID / log |

## 8. 依賴檢查

```bash
cd ekf
./check_deps.sh --check-only    # 只檢查
./check_deps.sh                 # 缺少 apt package 就安裝
```

會檢查：

```text
ROS 2 Jazzy / ros2 CLI
ros-jazzy-robot-localization
ros-jazzy-realsense2-camera
ros-jazzy-tf2-ros
ekf_params.yaml / start_ekf.sh / stop_ekf.sh
ekf_params.yaml 的必要 key 與值
ekf_node / realsense2_camera_node / static_transform_publisher
RealSense D455 USB
/dev/video*
video group
```

目前板端結果：

```text
check-only: OK (warnings: 1)
```

唯一 warning 是 `root` 不在 `video` group；以 root 執行時不影響。

## 9. 驗證

```bash
# EKF 輸出頻率
ros2 topic hz /odometry/filtered

# 看一筆 odometry
ros2 topic echo /odometry/filtered --once

# 看 TF
timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint

# 輸入是否存在
ros2 topic hz /odom_rf2o
ros2 topic hz /camera/camera/gyro/sample
```

正常預期：

```text
/odometry/filtered         約 20 Hz
/odom_rf2o                 有資料
/camera/camera/gyro/sample 有資料
odom -> base_footprint     持續更新
```

## 10. 常見問題

### `/odom_rf2o` 沒有資料

先確認 SLAM / rf2o 已啟動：

```bash
ros2 topic info /odom_rf2o
ros2 topic hz /odom_rf2o
```

### gyro 沒有資料

確認 D455 已啟動，且 launch 有開：

```text
enable_gyro:=true
enable_accel:=true
```

### 同時有兩個 RealSense

`start_ekf.sh` 會啟動一顆 D455。
`start_ai.sh` 預設 `START_REALSENSE=0`，會沿用這一顆；
若在 AI 流程又開第二顆，會發生衝突。

### 停車後 odometry 還在漂

確認：

```yaml
odom0_differential: true
```

以及 rf2o 的 `publish_tf: false`。
