[English] | [繁體中文](README.zh-TW.md)

# demo_script

Two entry points for the complete person-follow demo:

```text
start_demo.sh          mapping mode
start_demo_static.sh   localization mode (loads an existing posegraph)
```

Both start the same four layers:

```text
1. RPLIDAR S2 + rf2o + slam_toolbox
2. RealSense + EKF
3. Nav2 + chassis driver
4. AI detection + person_target_recovery + demo_manager
```

They do **not** call `../start_all.sh`, `../start_all_localization.sh`, or
`../start_ai.sh`. They orchestrate the lower-level start scripts and the
vendor helper scripts directly.

## 1. Files

| File | Purpose |
|---|---|
| `start_demo.sh` | Full demo, slam_toolbox mapping mode |
| `start_demo_static.sh` | Full demo, slam_toolbox localization mode |
| `start_mapping.sh` | Mapping-only SLAM (no EKF / Nav2 / AI) |
| `demo_common.sh` | Shared helpers (sourced by `start_demo*`) |

## 2. Mapping vs. localization

| | `start_demo.sh` | `start_demo_static.sh` |
|---|---|---|
| SLAM mode | `mapping` | `localization` |
| SLAM params | `../slam_toolbox_rf2o.yaml` | `../slam_toolbox_localization.yaml` |
| Posegraph | not required | required |
| Builds a new map | yes | no |
| Nav2 params | `../nav2_1028/nav2_params_plugin.yaml` | same |
| Starts EKF / Nav2 / AI | yes | yes |

For mapping-only SLAM (RPLIDAR + rf2o + slam_toolbox, without EKF, Nav2, or AI):

```bash
./start_mapping.sh
```

This is a shortcut for `../slam_toolbox/start_mapping.sh`.
Stop it with `../slam_toolbox/stop.sh`.

For localization, the posegraph is read from `map_file_name` in the
localization yaml:

```text
<map_file_name>.posegraph
<map_file_name>.data
```

Create one while mapping:

```bash
cd ../slam_toolbox
./start_mapping.sh
./save_map.sh demo_map
./stop.sh
```

Then set `map_file_name` in `slam_toolbox_localization.yaml`.

## 3. Usage

Mapping demo:

```bash
cd demo_script
./start_demo.sh
```

Static/localization demo:

```bash
cd demo_script
./start_demo_static.sh
```

Stop either demo:

```bash
../stop_demo.sh
```

Useful variants:

```bash
START_AI=0 ./start_demo.sh          # SLAM + EKF + Nav2 only
RECORD=1 ./start_demo.sh            # also record /cmd_vel_nav to CSV
SLAM_PARAMS=/path/to/other.yaml ./start_demo_static.sh
NAV2_PARAMS_FILE=../nav2_1028/nav2_params_plugin.yaml ./start_demo.sh
```

## 4. Environment variables

| Variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `99` | DDS domain |
| `START_AI` | `1` | Start AI/recovery/demo manager |
| `RECORD` | `0` | Start `nav2_1028/tools/record_cmd_vel.py` |
| `NAV2_PARAMS_FILE` | `../nav2_1028/nav2_params_plugin.yaml` | Nav2 params |
| `SLAM_PARAMS` | mapping or localization yaml | SLAM params |
| `USE_EXTERNAL_SAFETY` | `0` | Passed to `start_nav2.sh` |
| `START_REALSENSE` | `0` | Keep at 0; EKF starts the camera |
| `CAMERA_X` / `CAMERA_Y` / `CAMERA_Z` | `0.0` / `0.0` / `0.0` | AI TF offset |
| `LOG_DIR` | temp directory | Logs and PID files |
| `RUN_DIR` | temp directory | PID file directory |
| `TOPIC_WAIT` | `20` | Seconds to wait for each topic |

`START_REALSENSE=1` is not useful here: the demo starts the camera through EKF.
If no camera is running, the script exits with an error.

## 5. What each layer starts

### SLAM

Called through `../start_slam_toolbox_rf2o.sh`:

```text
rplidar_node
rf2o_laser_odometry_node
base_footprint -> base_link -> laser static TFs
slam_toolbox (online_async or localization)
```

### EKF

Called through `../ekf/start_ekf.sh`:

```text
realsense2_camera
base_footprint -> camera_link static TF
robot_localization ekf_node
```

### Nav2

Called through `../nav2_1028/start_nav2.sh`:

```text
controller_server (DeadbandController plugin)
planner_server / smoother_server / behavior_server
bt_navigator / waypoint_follower / route_server / docking_server
velocity_smoother / collision_monitor
keepout container
yahboomcar Mcnamu_driver
```

### AI

Started directly by the script (or via vendor helpers):

```text
ai_human_detection_node
person_follow_independent_bringup ai_tf_only.launch.py
person_target_recovery_node
demo_manager_node
```

The AI node uses the camera started by EKF, so no second RealSense is needed.

## 6. Startup order

```text
start_demo.sh
  ├─ start_slam_toolbox_rf2o.sh
  ├─ ekf/start_ekf.sh
  ├─ wait /map + /odometry/filtered + TF map -> base_link
  ├─ nav2_1028/start_nav2.sh
  └─ AI node -> AI TF -> recovery -> demo manager
```

The script waits up to 60 seconds for SLAM/EKF before starting Nav2.

## 7. Logs and stop

Logs and PID files go to `LOG_DIR` (default is a temp directory).

```text
<LOG_DIR>/ai_node.log
<LOG_DIR>/ai_tf.log
<LOG_DIR>/person_target_recovery.log
<LOG_DIR>/demo_manager.log
<LOG_DIR>/ai_pids
```

Stop:

```bash
../stop_demo.sh
```

`stop_demo.sh` stops the AI processes first, then the SLAM/Nav2/EKF stack.

## 8. Notes

```text
- `start_mapping.sh` does not start the demo stack; it only starts SLAM.
- Mapping demo (`start_demo.sh`) does not need a posegraph.
- Static demo requires .posegraph + .data; save them in mapping mode.
- The camera is owned by EKF; do not start a second RealSense.
- Nav2 default here is DeadbandController (simple mode).
- START_AI=0 is useful when only bringing up navigation.
- The AI helper scripts source the vendor workspaces themselves; this folder
  only provides the orchestration and logging.
```
