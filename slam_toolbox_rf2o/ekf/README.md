[English] | [繁體中文](README.zh-TW.md)

# ekf

This folder fuses the D455 IMU with laser odometry and publishes the
`/odometry/filtered` topic and the `odom -> base_footprint` TF used by Nav2.

```text
rf2o (/odom_rf2o) ─┐
                   ├─ ekf_node ──► /odometry/filtered
D455 gyro ─────────┘                TF odom -> base_footprint
```

## Build layout

Source layout:

```text
scripts/    *.sh
config/     ekf_params.yaml
```

Build with colcon:

```bash
colcon build --merge-install --packages-select ekf_bringup
```

The install tree is flat:

```text
install/start_ekf.sh
install/stop_ekf.sh
install/check_deps.sh
install/ekf_params.yaml
```

Copy `install/.` to:

```text
/nvt_ros2_example/ekf/
```

Do not copy an `install/` subdirectory to the board.

## 1. Files

```text
ekf/
├── ekf_params.yaml     # robot_localization parameters
├── start_ekf.sh        # starts D455 + static TF + ekf_node
├── stop_ekf.sh         # stops the processes above
└── check_deps.sh       # dependency check / apt install
```

## 2. Required packages

All of them come from apt. **No manual build is required.**

| Package | Provides | Purpose |
|---|---|---|
| `ros-jazzy-robot-localization` | `ekf_node` | Fuses rf2o and gyro |
| `ros-jazzy-realsense2-camera` | `realsense2_camera_node` | D455 gyro / accel / RGBD |
| `ros-jazzy-tf2-ros` | `static_transform_publisher` | `base_footprint -> camera_link` |

`check_deps.sh` also uses:

```text
python3-yaml    # validate keys / values in ekf_params.yaml
usbutils        # lsusb check for the D455
```

EKF uses the apt `realsense2_camera` package directly.
The `../realsense-ros` workspace is only used by the AI flow when it starts
its own camera; it is not the source used by `start_ekf.sh`.

## 3. Currently fused data

Settings in `ekf_params.yaml`:

```yaml
frequency: 20.0
two_d_mode: true
world_frame: odom
publish_tf: true
```

### 3.1 `odom0`: rf2o

```yaml
odom0: /odom_rf2o
odom0_config: [true,  true,  false,
               false, false, true,
               false, false, false,
               false, false, false,
               false, false, false]
odom0_differential: true
```

Only these fields are used:

```text
x
y
yaw
```

rf2o twist (`vx / vy / vyaw`) is **not** used.

Reason:

```text
The lidar is mounted at 180 degrees, so rf2o twist has the opposite sign.
While driving forward, rf2o may report vx ≈ -0.4.
Fusing that twist would fight the pose estimate.
```

`odom0_differential: true` makes EKF turn successive pose differences into
velocity, so the velocity can drop to zero when the robot stops. This prevents
the filtered pose from creeping forward after a stop.

### 3.2 `imu0`: D455 gyro

```yaml
imu0: /camera/camera/gyro/sample
imu0_config: [false, false, false,
              false, false, false,
              false, false, false,
              false, false, true,
              false, false, false]
```

Only this field is used:

```text
yaw angular velocity (vyaw)
```

IMU orientation, roll, pitch, and acceleration are not used.
The IMU frame is transformed into `base_footprint` through TF.

## 4. Data that is not fused

```text
rf2o twist (vx / vy / vyaw)
D455 orientation / roll / pitch
D455 acceleration
Wheel odometry (this robot has no encoders)
GPS / navsat
```

## 5. Outputs

```text
/odometry/filtered
TF odom -> base_footprint
```

TF responsibilities:

```text
slam_toolbox : map -> odom
ekf_node     : odom -> base_footprint
static TF    : base_footprint -> base_link -> laser / camera_link
```

rf2o's `publish_tf` must be `false`; otherwise it competes with EKF for
`odom -> base_footprint`. This is configured in `../rf2o_params.yaml`.

## 6. Key parameters

| Parameter | Value | Description |
|---|---:|---|
| `frequency` | `20.0` | Output rate |
| `sensor_timeout` | `0.3` | Sensor timeout |
| `two_d_mode` | `true` | Estimate planar x / y / yaw only |
| `publish_tf` | `true` | Publish `odom -> base_footprint` |
| `world_frame` | `odom` | World frame is odom |
| `odom_frame` | `odom` | |
| `base_link_frame` | `base_footprint` | |
| `map_frame` | `map` | Reference only; `map -> odom` is published by slam_toolbox |
| `odom0` | `/odom_rf2o` | rf2o |
| `imu0` | `/camera/camera/gyro/sample` | D455 gyro |
| `use_control` | `false` | Does not use cmd_vel as an input |

## 7. Start / stop

Prerequisite: SLAM / rf2o is already running and `/odom_rf2o` is publishing.

```bash
cd ekf
./start_ekf.sh
```

It starts:

```text
realsense2_camera (gyro + accel + RGBD)
static_transform_publisher (base_footprint -> camera_link)
robot_localization ekf_node
```

Stop:

```bash
./stop_ekf.sh
```

Environment variables:

| Variable | Default | Description |
|---|---|---|
| `EKF_PARAMS_FILE` | `ekf_params.yaml` | Parameter file |
| `CAM_X` / `CAM_Y` / `CAM_Z` | `0.08` / `0.0` / `0.10` | Camera offset from `base_footprint` |
| `ROS_DISTRO` | `jazzy` | |
| `RUN_DIR` / `PID_FILE` / `LOG_DIR` | system temp directory | PID / log files |

## 8. Dependency check

```bash
cd ekf
./check_deps.sh --check-only    # check only
./check_deps.sh                 # install missing apt packages
```

It checks:

```text
ROS 2 Jazzy / ros2 CLI
ros-jazzy-robot-localization
ros-jazzy-realsense2-camera
ros-jazzy-tf2-ros
ekf_params.yaml / start_ekf.sh / stop_ekf.sh
required keys and values in ekf_params.yaml
ekf_node / realsense2_camera_node / static_transform_publisher
RealSense D455 USB
/dev/video*
video group
```

Current board result:

```text
check-only: OK (warnings: 1)
```

The only warning is that `root` is not in the `video` group. This does not
matter when running as root.

## 9. Verification

```bash
# EKF output rate
ros2 topic hz /odometry/filtered

# Inspect one odometry message
ros2 topic echo /odometry/filtered --once

# Inspect TF
timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint

# Check inputs
ros2 topic hz /odom_rf2o
ros2 topic hz /camera/camera/gyro/sample
```

Expected:

```text
/odometry/filtered         about 20 Hz
/odom_rf2o                 publishing
/camera/camera/gyro/sample publishing
odom -> base_footprint     continuously updated
```

## 10. Troubleshooting

### `/odom_rf2o` has no data

Make sure SLAM / rf2o is running:

```bash
ros2 topic info /odom_rf2o
ros2 topic hz /odom_rf2o
```

### No gyro data

Make sure the D455 is running with:

```text
enable_gyro:=true
enable_accel:=true
```

### Two RealSense nodes at the same time

`start_ekf.sh` starts one D455.
`start_ai.sh` defaults to `START_REALSENSE=0` and reuses this camera.
Starting a second camera in the AI flow will cause a conflict.

### Odometry drifts after stopping

Confirm:

```yaml
odom0_differential: true
```

and that rf2o has `publish_tf: false`.
