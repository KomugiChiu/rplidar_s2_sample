# rtabmap lidar (RPLidar S2 laser-only mapping)

Runs on the board, no camera required. Sync this directory to the board before use.

## Files

| File | Purpose |
|---|---|
| `rtabmap_laser_mapping.sh` | One-shot mapping: S2 driver → static TF → rtabmap (ICP odometry + laser scan). Optional bag recording |
| `rtabmap_laser_mapping.ini` | rtabmap parameters, loaded via `cfg:=` (INI values are strings, so no type issues) |
| `check_deps.sh` | Dependency check (+ auto-install) for the rtabmap + RPLidar S2 flow |
| `set_str_param.py` | Fallback: update string-typed parameters on a running node without restart |
| `maps/` | Saved 2D maps (pgm + yaml) |
| `params/` | Reference parameter YAMLs (`icp_odometry.yaml`, `rplidar_node.yaml`, `rtabmap.yaml`) |

## Usage (on board)

```bash
./rtabmap_laser_mapping.sh                           # mapping only
BAG=~/bags/lidar_$(date +%Y%m%d) ./rtabmap_laser_mapping.sh   # mapping + record bag
SKIP_LIDAR=1 ./rtabmap_laser_mapping.sh              # /scan already published, skip driver
LASER_YAW_DEG=180 ./rtabmap_laser_mapping.sh         # use when odom arrow is opposite to the S2 printed arrow
```

Environment overrides: `SCAN_TOPIC` (/scan), `LASER_FRAME` (laser),
`BASE_FRAME` (base_footprint), `LASER_Z` (0.20), `SERIAL_PORT` (/dev/ttyUSB0),
`ANGLE_COMPENSATE` (true), `SCAN_MODE` (DenseBoost), `CFG`
(./rtabmap_laser_mapping.ini), `BAG` (empty), `SKIP_LIDAR` (0).

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
