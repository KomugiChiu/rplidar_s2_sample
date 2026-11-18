[English] | [繁體中文](README.zh-TW.md)

# slam_toolbox

Standalone SLAM front-end:

```text
RPLIDAR S2 -> /scan
static TF: base_footprint -> base_link -> laser
slam_toolbox -> /map + map -> odom
```

rf2o is **not** started here. It lives in:

```text
../rf2o/
  install/              rf2o_laser_odometry install tree
  rf2o_params.yaml
  start.sh / stop.sh
```

For the complete mapping demo with EKF, Nav2 and AI, use:

```text
../slam_toolbox_rf2o/demo_script/start_mapping.sh
```

## Build layout

Source layout:

```text
scripts/    *.sh
config/     *.yaml
```

Build with colcon:

```bash
colcon build --merge-install --packages-select slam_toolbox_bringup
```

The install tree is flat:

```text
install/start_slam.sh
install/stop_slam.sh
install/save_map.sh
install/check_deps.sh
install/slam_toolbox_rf2o.yaml
install/slam_toolbox_localization.yaml
```

Copy `install/.` to the board directory:

```text
/nvt_ros2_example/slam_toolbox/
```

Do not copy an `install/` subdirectory to the board.

## 1. Files

| File | Purpose |
|---|---|
| `start_slam.sh` | Unified start: `mapping` or `localization` |
| `stop_slam.sh` | Stop lidar / static TFs / slam_toolbox |
| `save_map.sh` | Save pgm/yaml + posegraph (mapping only) |
| `check_deps.sh` | Check / install dependencies |


## 2. Mapping

```bash
cd /nvt_ros2_example/slam_toolbox
./start_slam.sh mapping
```

Starts:

```text
rplidar_node
static_transform_publisher (base_footprint -> base_link)
static_transform_publisher (base_link -> laser)
slam_toolbox online_async_launch.py
```

It does **not** start rf2o or EKF.
For a usable map + Nav2, use `../slam_toolbox_rf2o/demo_script/start_mapping.sh`, which also
starts rf2o, EKF and Nav2.

If you start components manually:

```text
1. ./start_slam.sh mapping
2. ../rf2o/start.sh
3. ../ekf/start_ekf.sh
4. drive with RViz goals
5. ./save_map.sh demo_map
6. ./stop_slam.sh
   ../rf2o/stop.sh
```

## 3. Localization

```bash
cd /nvt_ros2_example/slam_toolbox
./start_slam.sh localization
```

`map_file_name` in `slam_toolbox_localization.yaml` must point to:

```text
<map_file_name>.posegraph
<map_file_name>.data
```

The robot must start at `map_start_pose` or at the dock when
`map_start_at_dock: true`.

## 4. Save the map

Only while mapping mode is running:

```bash
./save_map.sh                  # maps/slam_map_<timestamp>
./save_map.sh demo_map         # maps/demo_map
```

Output:

```text
maps/demo_map.pgm
maps/demo_map.yaml
maps/demo_map.posegraph
maps/demo_map.data
```

`save_map.sh`:

```text
1. map_saver_cli -f <prefix>
2. /slam_toolbox/serialize_map
3. verifies all four files
```

`serialize_map` is not available in localization mode; the script detects
that and exits early.

## 5. rf2o

```bash
cd /nvt_ros2_example/rf2o
./start.sh
./stop.sh
```

`start.sh` expects `/scan` and the `base_footprint -> laser` TF, which are
provided by the slam_toolbox start script.

rf2o parameters:

```text
../rf2o/rf2o_params.yaml
```

publishes:

```text
/odom_rf2o
```

with `publish_tf: false`; EKF owns `odom -> base_footprint`.

## 6. Dependencies

apt:

```text
ros-jazzy-slam-toolbox
ros-jazzy-nav2-map-server
ros-jazzy-tf2-ros
```

workspaces:

```text
../rplidar_ros
../rf2o
```

Check:

```bash
./check_deps.sh --check-only
./check_deps.sh
```

## 7. Environment variables

| Variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `99` | DDS domain |
| `SLAM_PARAMS` | `slam_toolbox_rf2o.yaml` or localization yaml | SLAM params |
| `RPLIDAR_WS` | `../rplidar_ros` | rplidar workspace |
| `SERIAL_PORT` | `/dev/ttyUSB0` | RPLIDAR S2 serial port |
| `SKIP_LIDAR` | `0` | `1` = reuse an existing rplidar_node |
| `LASER_YAW_DEG` | `180` | S2 mounting orientation |
| `RUN_DIR` | temp directory | PID / logs |
| `MAP_DIR` | `maps/` | `save_map.sh` output |
| `DRY_RUN` | `0` | `stop_slam.sh` dry run |

## 8. Notes

```text
- rf2o is separate from slam_toolbox.
- slam_toolbox owns map -> odom.
- EKF owns odom -> base_footprint.
- start_slam.sh mapping alone does not provide odometry; EKF must be running.
- Use ../slam_toolbox_rf2o/demo_script/start_mapping.sh for the complete mapping stack.
```
