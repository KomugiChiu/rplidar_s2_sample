[English] | [繁體中文](README.zh-TW.md)

# nav2_1028

This folder contains the Nav2 configuration currently used by this robot.

```text
nav2_params_plugin.yaml   ← default for start_all.sh / start_all_localization.sh
nav2_params_amcl.yaml     ← used by start_all_amcl.sh
nav2_params.yaml          ← baseline (pure MPPI); default when start_nav2.sh runs alone
behavior_trees/           ← custom Behavior Trees
yaml_legacy/              ← old versions, backups, experimental yaml files
```

In this document, "baseline yaml" means `nav2_params.yaml` (the pure MPPI
baseline), not the upstream `nav2_bringup` sample.

---

## Build layout

Source layout:

```text
scripts/                  *.sh, *.py, tools/
config/                   *.yaml, behavior_trees/, yaml_legacy/
```

Build with colcon:

```bash
colcon build --merge-install --packages-select nav2_1028_bringup
```

The install tree is flat:

```text
install/start_nav2.sh
install/stop_nav2.sh
install/start_chassis.sh
install/stop_chassis.sh
install/bringup_bond_launch.py
install/keepout_filter_launch.py
install/nav2_params*.yaml
install/behavior_trees/
install/tools/record_cmd_vel.py
```

Only the files above are installed. The other helpers under `scripts/tools/`
(`monitor_cpu.py`, `relocalize.py`, `set_initial_pose.py`, `trim_test.py`,
`restart_driver.sh`, driver patches, `tools/legacy/`) are dev-only and stay in
the source tree.

Copy `install/.` to:

```text
/nvt_ros2_example/nav2/
```

Do not copy an `install/` subdirectory to the board.

## 1. Which yaml is used at startup

| Entry point | Yaml | Custom BT |
|---|---|---|
| `../start_all.sh` | `nav2_params_plugin.yaml` | ✅ |
| `../start_all_localization.sh` | `nav2_params_plugin.yaml` | ✅ |
| `../start_all_amcl.sh` | `nav2_params_amcl.yaml` | ✅ |
| `start_nav2.sh` directly | `nav2_params.yaml` | ❌ uses Nav2 built-in BT |

`start_all*` passes `PARAMS_FILE` to `start_nav2.sh`.
If `start_nav2.sh` runs alone without `PARAMS_FILE`, it loads
`nav2_params.yaml`.

---

## 2. Behavior Tree XML vs. Nav2 defaults

Files:

```text
behavior_trees/navigate_to_pose_bt.xml
behavior_trees/navigate_through_poses_bt.xml
```

`nav2_params_plugin.yaml` / `nav2_params_amcl.yaml` point
`default_nav_to_pose_bt_xml` and `default_nav_through_poses_bt_xml`
to these files.

Without those settings, Nav2 uses its built-in trees:

```text
nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml
```

### 2.1 Difference summary

| Item | Nav2 default | Custom BT here |
|---|---|---|
| `Spin` recovery | Present, `spin_dist=1.57` | **Removed** |
| Outer `FollowPath` | `RecoveryNode`, retries 1 | `FollowPathTfLag`, retries 4 |
| TF stall | One TF failure can end the goal | On `TF_ERROR(102)` / `CONTROLLER_TIMED_OUT(107)`, `Wait 1.0 s` and retry |
| `server_timeout` | Action default | `ComputePathToPose`, `ComputePathThroughPoses`, `FollowPath` set to `20000 ms` |
| `BackUp` error code | `backup_code_id` (`navigate_to_pose`) | Changed to `backup_error_code` |
| Remaining recoveries | ClearCostmap, Wait, BackUp | Same |

### 2.2 Why remove Spin

This is an open-loop mecanum chassis:

```text
- No wheel encoders
- Minimum effective rotation is about 0.20 rad/s
- About 0.23 s soft-start
- No active braking
```

A 90-degree open-loop Spin overshoots and can hit nearby obstacles.
The recovery therefore no longer uses Spin and keeps only:

```text
ClearEntireCostmap
Wait
BackUp
```

`behavior_server` still loads the Spin plugin; this tree simply never calls it.

### 2.3 Why add FollowPathTfLag

Under high CPU load, `map -> odom` can be delayed by 0.5–7.5 seconds.
Nav2's `WouldAControllerRecoveryHelp` does not handle:

```text
TF_ERROR (102)
CONTROLLER_TIMED_OUT (107)
```

So an extra outer layer was added:

```text
FollowPathTfLag (retries=4)
  └─ FollowPath (retries=1, original recovery)
  └─ WaitOutTfLag
        AreErrorCodesPresent = 102;107
        Wait 1.0 s
```

Once TF recovers, the same goal is retried instead of failing the whole
navigation task.

---

## 3. `nav2_params_plugin.yaml` vs. `nav2_params.yaml`

`nav2_params.yaml` is the DiffDrive + MPPI baseline.
`nav2_params_plugin.yaml` replaces the controller with DeadbandController and
adds real-robot tuning on top.

### 3.1 `bt_navigator`

```text
+ custom BT paths (behavior_trees/)
+ default_nav_to_pose_bt_xml
+ default_nav_through_poses_bt_xml
```

### 3.2 `controller_server`

| Parameter | Baseline | Plugin version |
|---|---|---|
| `FollowPath.plugin` | `nav2_mppi_controller::MPPIController` | `nav2_deadband_controller::DeadbandController` |
| `FollowPath.mode` | — | `simple` |
| `min_y_velocity_threshold` | `0.5` | `0.001` |
| `progress_checker.required_movement_radius` | `0.15` | `0.10` |
| `progress_checker.movement_time_allowance` | `10.0` | `15.0` |
| Goal checker | `SimpleGoalChecker`, xy `0.15` | `StoppedGoalChecker`, xy `0.05` |
| Goal checker stop velocity | — | trans `0.03`, rot `0.10` |

`StoppedGoalChecker` avoids declaring the goal reached while the chassis is
still moving and then coasting more than 10 cm past it.

### 3.3 Costmap

| Parameter | Baseline | Plugin version |
|---|---|---|
| local `transform_tolerance` | `0.5` | `3.0` |
| global `transform_tolerance` | `0.5` | `1.0` |
| Footprint | `robot_radius: 0.22` | 15 × 10 cm rectangle |
| local `inflation_radius` | `0.35` | `0.10` |
| global `inflation_radius` | `0.30` | `0.10` |
| Keepout filter | None | Enabled on local and global |

Footprint:

```yaml
footprint: "[[0.075, 0.05], [0.075, -0.05], [-0.075, -0.05], [-0.075, 0.05]]"
```

`transform_tolerance` is larger because `map -> odom` can lag when the CPU is
busy. The local costmap uses 3.0 s so `isGoalReached()` does not return
`TF_ERROR` just because the transform is old.

Keepout is provided by `keepout_filter_launch.py`:

```text
keepout_container
  ├─ keepout_filter_mask_server   (nav2_map_server)
  ├─ costmap_filter_info_server   (nav2_map_server)
  └─ lifecycle_manager_keepout
```

### 3.4 `behavior_server`

| Parameter | Baseline | Plugin version |
|---|---|---|
| `max_rotational_vel` | `0.25` | `0.20` |
| `min_rotational_vel` | `0.15` | `0.20` |
| `rotational_acc_lim` | `1.2` | `0.40` |

Even though the BT does not use Spin, behavior_server keeps slower rotation
parameters.

### 3.5 `velocity_smoother`

| Parameter | Baseline | Plugin version |
|---|---|---|
| `max_velocity` | `[0.15, 0.0, 0.25]` | `[0.20, 0.0, 1.0]` |
| `min_velocity` | `[-0.15, 0.0, -0.25]` | `[-0.20, 0.0, -1.0]` |
| `max_accel` | `[0.8, 0.0, 0.6]` | `[5.0, 0.0, 10.0]` |
| `max_decel` | `[-0.8, 0.0, -1.2]` | `[-5.0, 0.0, -10.0]` |

The plugin version is nearly pass-through:

```text
The controller already outputs chassis-ready steps.
velocity_smoother no longer ramps or clips them.
```

This avoids adding about 0.2 s of delay to every stop and prevents `wz` from
being clipped below the deadband.

### 3.6 MPPI parameters are still present

Besides the DeadbandController parameters, the `FollowPath` block still
contains a full set of MPPI parameters:

```text
time_steps, batch_size, vx_std, critics, CostCritic, ...
motion_model: Omni
TwirlingCritic
```

These are for `mode: "mppi"`.
With the default `mode: "simple"`, **MPPI is not loaded**, so those parameters
have no effect.

### 3.7 Unchanged sections

Most other sections are the same as the baseline:

```text
amcl
planner_server (NavfnPlanner)
smoother_server (SimpleSmoother)
waypoint_follower
route_server
docking_server
collision_monitor (comment-only difference)
```

---

## 4. DeadbandController plugin behavior

### 4.1 Architecture

```text
process: controller_server
  ├─ node /controller_server
  ├─ pluginlib: nav2_deadband_controller::DeadbandController
  └─ (only with mode=mppi)
       └─ pluginlib: nav2_mppi_controller::MPPIController
```

`DeadbandController` implements `nav2_core::Controller` and is loaded into the
`controller_server` process through pluginlib. It is **not a separate node**.

### 4.2 Two modes

```yaml
FollowPath:
  plugin: "nav2_deadband_controller::DeadbandController"
  mode: "simple"
  primary_controller: "nav2_mppi_controller::MPPIController"
```

| Mode | Behavior |
|---|---|
| `simple` (default) | Runs its own pursuit / pulse logic; does not load MPPI |
| `mppi` | Loads `primary_controller`, then applies floors and directional stop |

If `mode` is neither `simple` nor `mppi`, configure throws an error.

### 4.3 `simple` mode flow

State machine:

```text
DRIVE → ROTATE → ARRIVED
```

#### DRIVE

- Picks a lookahead point on the path (`lookahead_distance: 0.3 m`)
- Constant forward speed `drive_speed: 0.14`
- Steering:

```text
wz = steer_gain * heading_error
   = 0.8 * error
  capped at steer_max = 0.30
```

- Near the goal:
  - distance < `cruise_stop_distance: 0.20 m`, or heading error is too large
  - stop and settle for `settle_time: 0.70 s`
  - finish the remaining distance with a timed pulse

#### Timed pulse

Short goals use an open-loop timed pulse instead of closed-loop control:

```text
T = ff_dead_time + distance / ff_speed
  = 0.28 + distance / 0.80
```

- Only used when distance < `ff_max_distance: 0.30 m`
- Rotation pulse:

```text
T = rot_dead_time + |yaw_error| / rot_rate
  = 0.05 + |error| / 2.0
```

- A single rotation is capped by `rotate_pulse: 0.30 s`
- After each pulse, `ff_speed` / `rot_rate` are updated from the measured
  motion (`adapt_gain: 0.2`; stalls and outliers are ignored)

#### Near the goal

- Below `near_goal_distance: 0.15 m`, it never turns in place for position
  error
- If lateral error < `0.9 * xy_retry_distance`, it pulses straight
  forward/backward along the heading
- If the goal is behind and closer than `reverse_distance: 0.25 m`, it
  reverses directly

#### ROTATE

- Final heading is handled only after position is reached
  (< `brake_distance: 0.045 m`)
- Rotates in pulses to the goal yaw
- Enters `ARRIVED` when error < `yaw_tolerance: 0.30 rad`

#### ARRIVED

- Holds still
- Re-enters `DRIVE` when there is a new goal or the robot is pushed more than
  `xy_retry_distance: 0.05 m`

### 4.4 Path preprocessing

`setPlan()` downsamples the dense planner path:

```text
path_min_distance: 0.10 m
```

It keeps the first and last pose.
This reduces TF transforms and lookahead calculations.

### 4.5 Forward safety stop

Before moving forward, it checks the local costmap:

```text
stop_distance: 0.20 m
stop_half_width: 0.07 m
```

- Only `LETHAL_OBSTACLE` cells are checked
- INSCRIBED cells are not used as a stop condition
- Only forward motion is blocked; rotation and reverse are still allowed

Ultrasonic:

```text
sonar_topic: /ultrasonic
sonar_stop_distance: 0.20 m
sonar_hold_time: 0.3 s
```

- `/ultrasonic` uses centimeters
- Forward motion is blocked for 0.3 s when distance < 0.20 m
- Values outside 0.02–3.0 m are ignored

### 4.6 Escape recovery

If forward motion stays blocked for more than:

```text
escape_after: 3.0 s
```

it enters escape:

```text
1. Determine whether the obstacle is on the left or right
   (weighted lethal cells in the costmap)
2. Turn away by escape_angle: 0.5 rad
3. Up to escape_max_turns: 3 turns
4. If the front is clear, drive forward escape_distance: 0.15 m
5. If still blocked and the rear is clear, reverse 0.15 m
6. If both front and rear are blocked, give up and return to BT recovery
```

This handles cases where the planner allows a path but the robot is already
touching lethal cells, so `BackUp` reports `Collision Ahead` and refuses to
move.

### 4.7 Stop heartbeat

The chassis MCU latches the last velocity command, so the plugin has a stop
timer:

```text
cmd_vel_timeout: 0.5 s
```

If `computeVelocityCommands()` has not been called for 0.5 s
(goal finished, Nav2 stopped, controller stuck), it keeps publishing zero
velocity to `cmd_vel` so the robot does not keep the last non-zero command.

### 4.8 `mppi` mode (not default)

```text
cmd = MPPI.computeVelocityCommands(...)
```

Then:

1. Only forward motion is blocked (`forwardBlocked`)
2. `vx` / `vy` get floors:

```text
vx_floor: 0.16
vy_floor: 0.0
wz_floor: 0.90
```

3. When goal distance > `goal_rotate_distance: 0.30 m`, force `wz = 0`
4. Only inside 0.30 m may it rotate in place, at fixed `wz_floor`
5. Stop rotating when within `rotate_stop_angle: 0.35 rad`

### 4.9 Key parameters

| Parameter | Value | Description |
|---|---:|---|
| `mode` | `simple` | Default mode |
| `drive_speed` | `0.14` | Forward speed |
| `steer_gain` | `0.8` | Steering gain |
| `steer_max` | `0.30` | Maximum `wz` while driving |
| `align_angle` | `0.6` | Stop and rotate above this error |
| `brake_distance` | `0.045` | Stop distance |
| `xy_retry_distance` | `0.05` | Re-drive distance after being pushed |
| `yaw_tolerance` | `0.30` | Final heading tolerance |
| `settle_time` | `0.70` | Wait time after stopping |
| `ff_max_distance` | `0.30` | Maximum distance for timed pulse |
| `ff_dead_time` | `0.28` | Pulse dead time |
| `ff_speed` | `0.80` | Initial pulse speed |
| `adapt_gain` | `0.2` | Pulse self-calibration gain |
| `rotate_pulse` | `0.30` | Maximum single rotation pulse |
| `rot_rate` | `2.0` | Rotation pulse rate |
| `stop_distance` | `0.20` | Costmap forward stop distance |
| `stop_half_width` | `0.07` | Forward check half-width |
| `sonar_stop_distance` | `0.20` | Ultrasonic stop distance |
| `escape_after` | `3.0` | Blocked time before escape |
| `cmd_vel_timeout` | `0.5` | Stop heartbeat timeout |

---

## 5. Quick checks

```bash
# Current BT files
ros2 param get /bt_navigator default_nav_to_pose_bt_xml
ros2 param get /bt_navigator default_nav_through_poses_bt_xml

# Current controller plugin
ros2 param get /controller_server FollowPath.plugin
ros2 param get /controller_server FollowPath.mode

# Current nodes
ros2 node list | grep -E "bt_navigator|controller_server|costmap"
```

Note:

```text
always_reload_bt_xml = False
```

The BT XML is loaded when `bt_navigator` is configured. After editing the
file, restart `bt_navigator` (or all of Nav2) for the change to take effect.

---

## 6. Related files (relative paths)

```text
nav2_params_plugin.yaml
nav2_params_amcl.yaml
nav2_params.yaml
behavior_trees/navigate_to_pose_bt.xml
behavior_trees/navigate_through_poses_bt.xml
keepout_filter_launch.py
bringup_bond_launch.py
../start_all.sh
../start_all_localization.sh
../start_all_amcl.sh
../nav2_deadband_ws/src/nav2_deadband_controller/
```
