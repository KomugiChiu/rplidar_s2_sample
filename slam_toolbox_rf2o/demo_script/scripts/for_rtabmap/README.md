[English] | [繁體中文](README.zh-TW.md)

# for_rtabmap

The demo_script flow, but with **RTAB-Map** doing the 2D lidar
mapping instead of slam_toolbox.

Two entry points:

```text
start_demo.sh   RTAB-Map mapping + EKF + Nav2 + AI
stop_demo.sh    stop everything start_demo.sh started
```

There is no localization/static variant and no map save step: RTAB-Map always
builds its own map and keeps it in its database.

## 1. Odometry chain

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

Only the EKF publishes `odom -> base_footprint`. `icp_odometry` runs with
`publish_tf=false`, and RTAB-Map consumes the **filtered** odometry, exactly
like the slam_toolbox flow consumes rf2o + EKF.

`ODOM_SOURCE=external` swaps ICP for rf2o (`/odom_rf2o`) and uses the original
`ekf_params.yaml`; the EKF tail is the same. See §7.

## 2. Four layers

```text
1. RPLIDAR S2 + static TFs + icp_odometry + RTAB-Map
2. RealSense + camera TF + EKF
3. Nav2 + chassis driver        (nav2_params_rtabmap.yaml)
4. AI detection + demo_manager runtime (demo_manager_node + person_target_recovery_node)
```

The camera is still started in layer 2 even with `START_AI=0`, because the EKF
fuses the D455 gyro. Without it the EKF would have no yaw rate input.

## 3. Files

| File | Purpose |
|---|---|
| `start_demo.sh` | Full demo: RTAB-Map + EKF + Nav2 + AI |
| `stop_demo.sh` | Stop the stack started by `start_demo.sh` |
| `README.md` | This file |
| `README.zh-TW.md` | 繁體中文版 |

## 4. Usage

From the deployed prefix (`/nvt_ros2_example/demo_script/for_rtabmap`) or
`demo_script/scripts/for_rtabmap` in the source tree:

```bash
cd /nvt_ros2_example/demo_script/for_rtabmap
./start_demo.sh
```

Variants:

```bash
START_AI=0 ./start_demo.sh          # mapping + EKF + Nav2 only
DRY_RUN=1 ./stop_demo.sh            # show what stop_demo.sh would stop
ODOM_SOURCE=external ./start_demo.sh    # rf2o + /odom_rf2o instead of ICP
```

Stop:

```bash
./stop_demo.sh
```

The rplidar is exclusive: stop the slam_toolbox demo (or anything else using
`/dev/ttyUSB0`) before starting this one.

## 5. Environment variables

| Variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `99` | DDS domain |
| `START_AI` | `1` | Start AI / recovery / demo manager |
| `ODOM_SOURCE` | `icp` | `icp` = icp_odometry, `external` = rf2o |
| `ODOM_TOPIC` | `/odometry/filtered` | Odometry RTAB-Map and Nav2 consume |
| `MAP_TOPIC` | `/rtabmap/map` | Map topic RTAB-Map publishes |
| `RTABMAP_DIR` | `/nvt_ros2_example/rtabmap` | RTAB-Map package scripts |
| `RTABMAP_NAV2_PARAMS_FILE` | `$RTABMAP_DIR/nav2_params_rtabmap.yaml` | Nav2 params |
| `EKF_PARAMS_FILE` | `$EKF_DIR/ekf_params_icp.yaml` (`external`: `ekf_params.yaml`) | EKF params |
| `RF2O_DIR` | `/nvt_ros2_example/rf2o` | rf2o scripts (`external` only) |
| `EKF_DIR` | `/nvt_ros2_example/ekf` | EKF scripts and params |
| `NAV2_DIR` | `/nvt_ros2_example/nav2` | Nav2 scripts |
| `LOG_DIR` / `RUN_DIR` | `/tmp/demo_script` | Logs and PID files |
| `TOPIC_WAIT` | `20` | Seconds to wait for each AI topic |

Bare `$RTABMAP_DIR/start_rtabmap.sh` additionally accepts:

| Variable | Default | Description |
|---|---|---|
| `SKIP_LIDAR` | `0` | `1` = reuse an already running rplidar_node |
| `SCAN_TOPIC` | `/scan` | Laser scan topic |
| `SERIAL_PORT` | `/dev/ttyUSB0` | RPLIDAR S2 port |
| `DATABASE_PATH` | `$RTABMAP_DIR/rtabmap.db` | RTAB-Map database |
| `DELETE_DB_ON_START` | `0` | `1` = start from an empty database |
| `ICP_TOPIC` | `/odom_icp` | icp_odometry output |
| `BASE_FRAME` / `LASER_FRAME` | `base_footprint` / `laser` | TF frames |
| `RUN_DIR` / `LOG_DIR` | `/tmp/rtabmap_only` | PID file and logs |

## 6. Startup order

```text
start_demo.sh
  ├─ $RTABMAP_DIR/start_rtabmap.sh
  │    ├─ rplidar_node
  │    ├─ base_footprint -> base_link -> laser static TFs
  │    ├─ rtabmap_odom/icp_odometry   (publish_tf=false -> /odom_icp)
  │    └─ rtabmap.launch.py           (subscribe_scan, odom_topic=/odometry/filtered)
  ├─ [$RF2O_DIR/start.sh]             (ODOM_SOURCE=external only)
  ├─ $EKF_DIR/start_ekf.sh            (EKF_PARAMS_FILE=ekf_params_icp.yaml)
  ├─ wait /rtabmap/map + /odometry/filtered + TF map -> base_link  (60 s)
  ├─ $NAV2_DIR/start_nav2.sh          (MAP_TOPIC=/rtabmap/map)
  └─ AI node -> AI TF -> recovery -> demo manager
```

## 7. icp_odometry vs rf2o

| | `ODOM_SOURCE=icp` (default) | `ODOM_SOURCE=external` |
|---|---|---|
| Odometry source | `rtabmap_odom/icp_odometry` | `rf2o_laser_odometry_node` |
| Raw odom topic | `/odom_icp` | `/odom_rf2o` |
| EKF params | `ekf_params_icp.yaml` | `ekf_params.yaml` |
| TF `odom -> base_footprint` | EKF | EKF |
| Needs `/odom_icp` in EKF config | yes | no |

`ekf_params.yaml` (used by the slam_toolbox + rf2o demo) is never modified by
this folder: the ICP variant lives in a separate file, and `start_demo.sh`
picks it with `EKF_PARAMS_FILE`.

Why ICP must not publish TF: the EKF already owns
`odom -> base_footprint`. Two publishers on the same transform make the TF
tree flip-flop, which shows up as Nav2 "transform too old" aborts and as a map
that jumps when RTAB-Map recomputes `map -> odom`.

## 8. Logs

```text
/tmp/rtabmap_only/logs/rplidar.log
/tmp/rtabmap_only/logs/icp_odometry.log
/tmp/rtabmap_only/logs/rtabmap.log
/tmp/rtabmap_only/logs/tf_base.log
/tmp/rtabmap_only/logs/tf_laser.log
/tmp/rtabmap_only/pids
/tmp/slam_toolbox_rf2o_ekf/logs/
/tmp/raspybot_nav2/logs/
/tmp/demo_script/*.log          (AI, ai_tf, demo_runtime)
```

`stop_demo.sh` stops in this order: AI -> RTAB-Map (lidar/TF/ICP) -> rf2o ->
Nav2/driver -> EKF/RealSense. `stop_rtabmap.sh` and `stop_ekf.sh` send SIGTERM
to the process group, wait, then SIGKILL and sweep by name, because some nodes
(`rplidar_node`, `ekf_node`, `realsense2_camera_node`) ignore SIGTERM and would
otherwise keep `/dev/ttyUSB0` busy for the next run.

## 9. Notes

```text
- For lidar-only mapping expect this warning, it is harmless:
  "There is no image subscription, bag-of-words loop closure detection will be
   disabled..."
- The map is only published as /rtabmap/map (and in rtabmap.db). This flow has
  no record_map.sh equivalent: Nav2 starts from an empty map and grows it.
- The database grows forever. Use DELETE_DB_ON_START=1 ./start_rtabmap.sh (or
  remove $RTABMAP_DIR/rtabmap.db) to start clean. A few minutes of mapping
  already produces tens of MB, so clear it between runs.
- RTAB-Map rewrites its config file on exit (it dumps ~400 parameters into it).
  start_rtabmap.sh therefore copies the shipped ini to
  $RUN_DIR/rtabmap_laser_mapping.ini and passes that copy, so the package ini
  stays short and no stale parameter dump pins defaults.
- RTAB-Map publishes map -> odom, Nav2 must use /rtabmap/map and
  /odometry/filtered; that is what nav2_params_rtabmap.yaml contains.
- Only one process may own the rplidar serial port at a time.
- There is no static/localization mode here; use the top-level
  `start_demo_static.sh` with slam_toolbox for that.
```
