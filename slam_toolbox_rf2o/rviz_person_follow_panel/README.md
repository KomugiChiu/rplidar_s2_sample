# person_follow_panel

RViz2 panel with four buttons for the person-follow demo:

| Button | Service | Type |
|---|---|---|
| Start Follow | `/demo/start` | `std_srvs/srv/Trigger` |
| Stop | `/demo/stop` | `std_srvs/srv/Trigger` |
| Clear Target | `/demo/clear_target` | `std_srvs/srv/Trigger` |
| Return Home | `/demo/return_home` | `std_srvs/srv/Trigger` |

The panel uses RViz's existing ROS node and creates the service clients in
`onInitialize()`. Calls are asynchronous, so the RViz UI is not blocked. The
result of each call is shown at the bottom of the panel (`OK`, `FAILED`,
`service not available`).

## Build

Build on the board (arm64), not cross-compile.

```bash
mkdir -p ~/rviz_panel_ws/src
ln -sfn /nvt_ros2_example/slam_toolbox_rf2o/rviz_person_follow_panel \
        ~/rviz_panel_ws/src/person_follow_panel

cd ~/rviz_panel_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select person_follow_panel
```

Dependencies:

```text
ros-jazzy-rviz-common
ros-jazzy-pluginlib
ros-jazzy-std-srvs
Qt5 Widgets (installed with rviz2)
```

If `rviz_common` is missing:

```bash
sudo apt install ros-jazzy-rviz-common ros-jazzy-pluginlib ros-jazzy-std-srvs
```

## Run

Source the panel workspace **after** the robot stack so pluginlib can find it:

```bash
source /opt/ros/jazzy/setup.bash
source /nvt_ros2_example/demo_manager/install/local_setup.bash
source /nvt_ros2_example/person_target_recovery/install/local_setup.bash
source /nvt_ros2_example/ai_human_detection/local_setup.bash
source ~/rviz_panel_ws/install/setup.bash
export ROS_DOMAIN_ID=99

rviz2
```

In RViz:

```text
Panels -> Add New Panel -> person_follow_panel -> DemoControlPanel
```

The panel is also available in a saved RViz config with:

```yaml
Panels:
  - Class: person_follow_panel/DemoControlPanel
    Name: Person Follow
```

## Verify services manually

```bash
ros2 service list | grep demo
ros2 service type /demo/start
ros2 service call /demo/start std_srvs/srv/Trigger "{}"
```

## Floating AI debug image

This package also exports `person_follow_panel/FloatImageDisplay`, a subclass of
`rviz_default_plugins/Image`. On startup it detaches its render panel into a
floating window and sizes it to **1/4 of the available screen** (half width x
half height).

The default RViz config uses it as:

```yaml
- Class: person_follow_panel/FloatImageDisplay
  Name: AI Debug Image
  Topic:
    Value: /debug_image/compressed
```

The window title is `AI Debug Image`; it can still be moved or resized by the
window manager.

## Notes

- The services are called asynchronously; the UI is not blocked.
- If a service is not running, the status line shows `service not available`.
- The panel only calls ROS services; it does not run shell commands.
- If the plugin does not appear in the Add New Panel list:
  ```bash
  ros2 pkg prefix person_follow_panel
  ls $(ros2 pkg prefix person_follow_panel)/share/person_follow_panel
  ```
  Make sure `plugin_description.xml` is installed and the workspace is sourced
  in the same shell that starts `rviz2`.
