# nav2_deadband_ws — Nav2 controller plugin for the Yahboomcar chassis

Custom `nav2_core::Controller` plugin used by the `slam_toolbox_rf2o/nav2_1028`
flow. It adds a **minimum command floor** (so commands never sit below the
chassis PWM deadband), a **forward-only directional stop**, and a simple
pursuit mode tuned for the open-loop mecanum base.

## Package

```text
src/nav2_deadband_controller/
├── include/nav2_deadband_controller/deadband_controller.hpp
├── src/deadband_controller.cpp
├── CMakeLists.txt
├── package.xml
└── plugins.xml        # nav2_deadband_controller::DeadbandController
```

- Plugin class: `nav2_deadband_controller::DeadbandController`
- Base class: `nav2_core::Controller`
- Version: 0.1.0

## Build (board)

```bash
mkdir -p ~/nav2_deadband_ws/src
# copy/clone this package into ~/nav2_deadband_ws/src/
cd ~/nav2_deadband_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav2_deadband_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`build/`, `install/`, `log/` are ignored by git.

## Use with Nav2

Source the overlay before launching Nav2:

```bash
source /root/nav2_deadband_ws/install/local_setup.bash
```

`nav2_1028/start_nav2.sh` already sources `PLUGIN_WS=/root/nav2_deadband_ws`
when it exists.

Nav2 params (`slam_toolbox_rf2o/nav2_1028/nav2_params_plugin.yaml`):

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_deadband_controller::DeadbandController"
      mode: "simple"                     # simple | mppi
      primary_controller: "nav2_mppi_controller::MPPIController"
      vx_floor: 0.16
      vy_floor: 0.0
      wz_floor: 0.90
      # simple mode
      drive_speed: 0.14
      steer_gain: 0.8
      steer_max: 0.30
      align_angle: 0.6
      reverse_distance: 0.25
      brake_distance: 0.045
      xy_retry_distance: 0.05
      yaw_tolerance: 0.30
      settle_time: 0.70
      ff_max_distance: 0.30
      ff_dead_time: 0.28
      ff_speed: 0.80
      adapt_gain: 0.2
      cruise_stop_distance: 0.20
      near_goal_distance: 0.15
      rot_dead_time: 0.05
      rot_rate: 2.0
```

## Modes

| Mode | Behaviour |
|---|---|
| `simple` | Timed forward/rotate pulses + closed-loop cruise, calibrated online (`adapt_gain`). Tuned for the open-loop Yahboomcar. |
| `mppi` | Wraps `primary_controller` (MPPI) and applies the deadband floors. |

## Safety

- Deadband floor on `vx` / `wz`.
- Forward-only stop from the local costmap (`stop_distance`, `stop_half_width`).
- `/ultrasonic` stop (`sonar_stop_distance`, `sonar_hold_time`).
- Heartbeat: publishes zero if Nav2 stops calling the controller (`cmd_vel_timeout`).

## Notes

- `wz_floor` in the source default is 0.52; the field-tuned value in
  `nav2_params_plugin.yaml` is 0.90 (0.52/0.70 measured as not turning, 0.80
  marginal).
- The workspace on the board is `/root/nav2_deadband_ws`; this repo keeps only
  the source package (no build/install/log).
