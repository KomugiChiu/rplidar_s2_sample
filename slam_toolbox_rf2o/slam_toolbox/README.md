[English] | [繁體中文](README.zh-TW.md)

# slam_toolbox

Convenience scripts for the RPLIDAR S2 + rf2o + slam_toolbox part of the
stack:

```text
/scan -> rf2o -> /odom_rf2o
                slam_toolbox -> /map + map -> odom
```

This folder does **not** start EKF, Nav2, AI, or the chassis driver.
Use `../demo_script/` for the complete demo.

## 1. Files

| File | Purpose |
|---|---|
| `start_mapping.sh` | Start lidar + rf2o + slam_toolbox in mapping mode |
| `start_localization.sh` | Start the same stack in localization mode; loads a posegraph |
| `stop.sh` | Stop the processes started by either start script |
| `save_map.sh` | Save `/map` as pgm/yaml and save the posegraph |
| `check_deps.sh` | Check / install dependencies for this flow |

## 2. Required packages

apt:

```text
ros-jazzy-slam-toolbox
ros-jazzy-nav2-map-server    # map_saver_cli
ros-jazzy-tf2-ros
```

Workspaces:

```text
../rplidar_ros               # rplidar_ros (or set RPLIDAR_WS)
$HOME/rf2o_ws                # rf2o_laser_odometry (or set RF2O_WS)
```

The launch script also uses:

```text
../rf2o_params.yaml
../slam_toolbox_rf2o.yaml
../slam_toolbox_localization.yaml
```

## 3. Mapping mode

```bash
cd slam_toolbox
./start_mapping.sh
```

Equivalent lower-level call:

```text
SLAM_MODE=mapping
SLAM_PARAMS=slam_toolbox_rf2o.yaml
```

What it starts:

```text
rplidar_node
rf2o_laser_odometry_node
two static transforms: base_footprint -> base_link -> laser
slam_toolbox online_async_launch.py
```

Check:

```bash
ros2 topic hz /scan
ros2 topic hz /odom_rf2o
ros2 topic hz /map
timeout 5 ros2 run tf2_ros tf2_echo map base_link
```

Logs are written by the lower-level script; use `QUIET=0` to see more output.

## 4. Save the current map

Run this **while mapping mode is running**:

```bash
./save_map.sh                 # maps/slam_map_<timestamp>
./save_map.sh demo_map        # maps/demo_map
```

Output:

```text
maps/<name>.pgm
maps/<name>.yaml
maps/<name>.posegraph
maps/<name>.data
```

What it does:

```text
1. ros2 run nav2_map_server map_saver_cli -f <prefix>
2. ros2 service call /slam_toolbox/serialize_map ...
3. verify that all four files exist
```

Important:

```text
serialize_map is only available in mapping mode.
In localization mode the service exists but refuses the request
("Cannot call serialize map in localization mode").
```

`save_map.sh` detects the node type and exits early if localization mode is
running. It also verifies the `.posegraph` / `.data` files because the
SerializePoseGraph response can return `result=0` even when it did not write
anything.

## 5. Localization mode

```bash
cd slam_toolbox
./start_localization.sh
```

It reads `map_file_name` from `slam_toolbox_localization.yaml` and requires:

```text
<map_file_name>.posegraph
<map_file_name>.data
```

`save_map.sh` prints the prefix to use:

```text
Use this prefix in a localization yaml:
  map_file_name: "<prefix>"
```

`start_localization.sh` also warns if neither `map_start_pose` nor
`map_start_at_dock` is set.

Start with the robot at the saved starting pose (or at the dock), otherwise
the initial localization guess will be wrong.

## 6. Stop

```bash
./stop.sh
DRY_RUN=1 ./stop.sh
KEEP_DRIVER=1 ./stop.sh    # keep rplidar_node if owned by someone else
```

## 7. Dependency check

```bash
./check_deps.sh --check-only
./check_deps.sh               # install missing apt packages
```

Checks:

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
localization posegraph (warn only)
RPLIDAR S2 USB
dialout group
```

## 8. Environment variables

| Variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `99` | DDS domain |
| `SLAM_PARAMS` | mapping or localization yaml | Parameter file override |
| `RPLIDAR_WS` | `../rplidar_ros` | rplidar workspace |
| `RF2O_WS` | `$HOME/rf2o_ws` | rf2o workspace |
| `MAP_DIR` | `../maps` | Output directory for `save_map.sh` |
| `MAPS_DIR` | `../maps` | Checked by `check_deps.sh` |
| `QUIET` | `1` | Passed to the lower-level SLAM script |
| `NO_LOG` | `0` | Passed to the lower-level SLAM script |
| `KEEP_DRIVER` | `0` | Keep `rplidar_node` when stopping |
| `DRY_RUN` | `0` | List what stop would do |

## 9. Notes

```text
- rf2o must have publish_tf: false; EKF owns odom -> base_footprint.
- slam_toolbox owns map -> odom.
- This folder does not start RealSense or EKF.
- Mapping uses online_async_launch.py; localization uses localization_launch.py.
- Saving a posegraph is a mapping-mode operation, not a localization-mode operation.
```
