# rtabmap lidar (RPLidar S2 laser-only mapping)

Runs on the board, no camera required. Sync this directory to the board before use.

## Files

| File | Purpose |
|---|---|
| `start_rtabmap_laser.sh` | One-shot mapping: S2 driver → static TF → rtabmap (ICP odometry + laser scan). Optional bag recording |
| `rtabmap_laser_mapping.ini` | rtabmap parameters, loaded via `cfg:=` (INI values are strings, so no type issues) |
| `stop_rtabmap_laser.sh` | Stops every node of this flow (driver, static TF, rtabmap, bag recorder), including leftovers of a killed run |
| `check_deps.sh` | Dependency check (+ auto-install) for the rtabmap + RPLidar S2 flow |
| `set_str_param.py` | Fallback: update string-typed parameters on a running node without restart |
| `maps/` | Saved 2D maps (pgm + yaml) |
| `params/` | Reference parameter YAMLs (`icp_odometry.yaml`, `rplidar_node.yaml`, `rtabmap.yaml`) |

## Usage (on board)

```bash
./start_rtabmap_laser.sh                           # mapping only
BAG=~/bags/lidar_$(date +%Y%m%d) ./start_rtabmap_laser.sh   # mapping + record bag
SKIP_LIDAR=1 ./start_rtabmap_laser.sh              # /scan already published, skip driver
LASER_YAW_DEG=180 ./start_rtabmap_laser.sh         # use when odom arrow is opposite to the S2 printed arrow
QUIET=1 ./start_rtabmap_laser.sh                   # clean console, node output -> log files
NO_LOG=1 ./start_rtabmap_laser.sh                  # no console output and no log files
```

## Logs

| Variable | Default | Effect |
|---|---|---|
| `QUIET` | `0` | `1` = keep the console clean; the rtabmap launch output goes to `$LOG_FILE` instead of the terminal |
| `NO_LOG` | `0` | `1` = no log files at all: every node output is discarded (`/dev/null`), implies `QUIET=1` |
| `LOG_LEVEL` | (launch default `info`) | forwarded to the launch as `log_level:=<level>` (`info`/`warn`/`error`/`debug`); `QUIET=1` defaults it to `warn` |
| `LOG_DIR` | `/tmp` | directory for all log files (created if missing) |
| `LOG_FILE` | `$LOG_DIR/lidar_rtabmap.log` | rtabmap / icp_odometry output |

Log files written: `lidar_rtabmap.log`, `lidar_rplidar.log`,
`lidar_static_base_link.log`, `lidar_static_tf.log`, `lidar_bag.log`.
With `QUIET=1` the script prints where they are; follow the rtabmap one with
`tail -f /tmp/lidar_rtabmap.log`.

Note: `NO_LOG=1` only silences the *nodes'* stdout/stderr. ROS 2 still writes
its own logs to `~/.ros/log/` and rtabmap still keeps its map database in
`~/.ros/rtabmap.db`.

Note: rtabmap rewrites the `.ini` with its full parameter set when it exits,
so the file grows from the 2 hand-written ICP lines to a full dump.

## Stopping

`start_rtabmap_laser.sh` starts the driver / static TF / bag recorder with
`setsid`, so those do not receive the terminal's Ctrl+C. The script's own trap
cleans them up on Ctrl+C, but if the script is killed (`kill -9`, closed
terminal, crash) they stay behind and keep `/dev/ttyUSB0` busy. Use:

```bash
./stop_rtabmap_laser.sh                    # SIGTERM everything, SIGKILL what survives
DRY_RUN=1 ./stop_rtabmap_laser.sh          # only list what would be stopped
KEEP_DRIVER=1 ./stop_rtabmap_laser.sh      # keep rplidar_node (owned by someone else)
```

If the script was stopped with Ctrl+Z, it is suspended and Ctrl+C no longer
reaches it — use `kill -TERM %1` (the stop script also resumes stopped
processes so the pending SIGTERM is delivered), or `fg` and then Ctrl+C.

### How the stop works

Both mechanisms are combined (same idea as the nav2 start/stop scripts):

1. **PID file (precise)** — the start script records every process it starts
   in `$PID_FILE` (`/tmp/rtabmap_laser/pids`) as `pid name mode`:
   - `group` → the process is a `setsid` session leader, the whole process
     group is killed (`kill -- -<pid>`)
   - `proc` → single process (`ros2 launch` stops its own children)
   The start script refuses to start a second instance while a recorded PID
   is still alive.
2. **Command line patterns (fallback)** — catches orphans whose PID file entry
   is gone (e.g. `/tmp` cleaned, older script version). The S2 driver is only
   matched this way when there is no PID file, so a driver owned by somebody
   else is never killed by accident.

Environment overrides: `SCAN_TOPIC` (/scan), `LASER_FRAME` (laser),
`BASE_FRAME` (base_footprint), `BASE_LINK_FRAME` (base_link), `BASE_LINK_Z`
(0.10), `LASER_Z` (0.20), `SERIAL_PORT` (/dev/ttyUSB0), `ANGLE_COMPENSATE`
(true), `SCAN_MODE` (DenseBoost), `CFG`
(`<script dir>/rtabmap_laser_mapping.ini`), `BAG` (empty), `SKIP_LIDAR` (0),
`QUIET` (0), `NO_LOG` (0), `LOG_LEVEL` (launch default), `LOG_DIR` (/tmp),
`RUN_DIR` (/tmp/rtabmap_laser), `PID_FILE` (`$RUN_DIR/pids`),
`ODOM_TOPIC` (/odom), `VO_FRAME_ID` (odom).

## Odom topic and nav2

The rtabmap nodes run in the `/rtabmap` namespace, so a relative topic would
land on `/rtabmap/odom` — but `nav2`'s `controller_server` / `bt_navigator`
subscribe to the **global** `/odom`. The script therefore launches with an
absolute topic name:

```bash
odom_topic:=/odom     # -> published on /odom
vo_frame_id:=odom     # TF frame id stays "odom" (no leading slash)
```

`ODOM_TOPIC` / `VO_FRAME_ID` can be overridden, e.g. `ODOM_TOPIC=/robot/odom`.
The bag records the same topic (`ODOM_TOPIC`).

The script can be started from any working directory: the `.ini` and
`stop_rtabmap_laser.sh` are resolved relative to the script itself (absolute
paths), and the effective config file is printed on startup.

Prerequisite: `rplidar_ros` workspace (sourced by the script), user in the `dialout` group.

## Required packages (on board)

```bash
sudo apt install ros-jazzy-rtabmap-launch ros-jazzy-rtabmap-odom \
  ros-jazzy-rtabmap-slam ros-jazzy-nav2-map-server
# ros-jazzy-tf2-ros / rosbag2 are part of the base Jazzy install in most cases
# rplidar_ros is not from apt: build from source (includes the S2 launch file)
sudo usermod -aG dialout komugi   # for /dev/ttyUSB0 access, re-login to take effect
```

To view the map locally in rviz: set Fixed Frame to `odom` first,
switch to `map` once it is published.
