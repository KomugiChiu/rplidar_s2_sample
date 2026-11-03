# Robot Console — realtime web UI on :8080

Browser console for the person-follow robot: health of every node/topic/TF edge,
"why isn't it moving", Nav2 map / costmaps / plan / goals / waypoints, camera +
AI detections, logs, and manual override.  Works on phones.

```text
browser ──HTTP :8080──► console/robot-console   (Go, stdlib only)
                          ├─ /proc: CPU per core, memory, temp, disk, Wi-Fi, stack processes, PID files
                          ├─ tails /tmp/*.log of the stack scripts (dedup vs /rosout, noise filter)
                          ├─ runs start_all / stop_all / start_ai / stop_ai (whitelist only)
                          └─ stdio ──► gateway/robot_console_gateway   (C++ rclcpp node "robot_console")
                                        ├─ rates / age / bandwidth of monitored topics, TF age/gaps/jumps
                                        ├─ health rules (thresholds from /debug_slam_lidar_ekf_komugi.md)
                                        ├─ map, costmaps, scan, plan, people → only while somebody watches
                                        ├─ Nav2 goal / waypoint / cancel, E-STOP, manual drive (/cmd_vel)
                                        └─ topic inspector (any type, via typesupport introspection)
```

There are two interchangeable gateways with the same stdio protocol and channels:

| Gateway | Where | Start with | CPU (stack running, measured 2026-11-03) |
|---|---|---|---|
| C++ (rclcpp) | `gateway/` — the cross-compiled binary goes to `gateway/robot_console_gateway` | `start_console.sh` | expected a few % |
| Python (rclpy) | `gateway_py/` — no build | `start_console_py.sh` | ~30 % of one core idle, ~46 % with map + both cameras + inspector open |

rclpy costs ~2 ms CPU per callback on this SoC (one 200 Hz subscription = 39 % of a core),
so the Python gateway samples instead of watching everything: gyro, camera_info,
`/odometry/filtered` and rf2o are measured 1 s every 15 s (shown as "sampled"); the
`/cmd_vel_nav` / `/cmd_vel_smoothed` chain is sampled while idle and switches to continuous
as soon as a Nav2 goal or manual drive starts; `/scan` is continuous only while the map is
open; rf2o's per-scan INFO is filtered out of `/rosout` inside DDS.

## Run

```bash
/root/rplidar_s2_sample/webui/start_console_py.sh  # Python gateway → http://<board-ip>:8080
/root/rplidar_s2_sample/webui/stop_console_py.sh

/root/rplidar_s2_sample/webui/start_console.sh     # C++ gateway (needs gateway/robot_console_gateway)
/root/rplidar_s2_sample/webui/stop_console.sh
```

Log: `/tmp/robot_console/console.log`.  The console does not need the ROS stack:
start it first and use **System → Start all** to bring the stack up.

Boot-time start (optional): `systemd/robot-console.service` (see the comment in it;
`KillMode=process` keeps a stack started from the console alive when the console restarts).

## Build

* Go server: `start_console*.sh` rebuilds `console/robot-console` when a `.go` or `web/` file
  is newer (one package at a time, `nice 19`, memory capped at 900 MB).
* C++ gateway: **cross-compile it** (aarch64, ROS 2 Jazzy, sources in `gateway/src`, deps in
  `gateway/package.xml`) and copy the binary to `gateway/robot_console_gateway`;
  `gateway/run_gateway.sh` uses it from there.  Building on the board is not practical:
  `gateway/build.sh` (one job, memory capped at 1.3 GB) did not finish in reasonable time.

**The board has 2.9 GB RAM and no swap.**  An unbounded `-j4` C++ build next to the
running stack crashed it (2026-11-03).  Run one heavy job at a time.

UI development without rebuilding Go: `console/robot-console -web console/web -addr :8090`.

## Tabs

| Tab | What |
|---|---|
| Overview | **Why is it (not) moving** — demo_manager → Nav2 goal → BT → `/cmd_vel_nav` → smoother → collision_monitor → `/cmd_vel` → EKF velocity → sonar, with a verdict and the log lines that explain a stop (costmap stop, collision_monitor invalid source, TF too old…). Below: every check grouped by **owner and team** (Nav2/SLAM team · AI team · shared). |
| Map | `/map`, global/local costmap, scan, global plan, robot + trail, AI people (via TF) and demo_manager markers. Tools: **Goal** (press, drag = heading), **Waypoints** (queue, Go = `follow_waypoints`), Init pose (AMCL mode). Cancel navigation, clear costmaps, mini drive pad. |
| Drive | Camera (color + `/human_detections` boxes, or AI debug image) and the manual drive pad. |
| AI / Demo | Perception chain RealSense → AI node → TF → demo_manager → Nav2 with the first broken stage, both camera views, people table (AI output vs demo_manager markers), AI/demo log incl. `/debug_log`. |
| Topics | All topics (type, pub/sub count, rate for monitored ones, owner). Click → inspector: rate, bandwidth, size, publisher/subscriber QoS, latest message. Click a node → its publishers/subscribers/services. |
| TF | Frame tree and every edge: rate, age since last receipt, stamp lag, gaps > 0.5 s, `map→odom` jumps. |
| System | Start/stop the stack scripts with live output, processes (CPU/RSS), board, Nav2 lifecycle, `/diagnostics`, PID files, lidar serial + USB kernel events, console status. |
| Logs | `/rosout` + `/debug_log` + all stack log files in one stream; filter by level/owner/source/regex; known failure strings get a hint; download. |

## Manual drive and E-STOP

* Holding a direction **cancels every Nav2 goal** (`navigate_to_pose`, `follow_waypoints`,
  `navigate_through_poses`) and calls `/demo/stop` if demo_manager is following, then
  publishes `/cmd_vel` at 20 Hz.  **No obstacle check** (by request).
* Deadman: the browser sends a keepalive every 100 ms; the gateway stops 0.5 s after the
  last one (released button, closed tab, lost Wi-Fi).  The chassis driver's own 0.3 s
  watchdog is the last layer.
* Limits: 0.35 m/s, 1.5 rad/s (`TELEOP_MAX_*` in `gateway/src/gateway.cpp`).
* For ~2 s after cancelling a running goal, collision_monitor still publishes zeros on
  `/cmd_vel` (its `stop_pub_timeout`), so the first moment of an override can stutter.
* **E-STOP** (header, always visible): cancel all goals, `/demo/stop`, zero `/cmd_vel` for 1 s.
  It is a convenience, not a safety device.
* Access is open to anyone on the LAN (decided 2026-11-03).

## Adding a monitored topic / check

`gateway/src/gateway.cpp`: `MONITORS` (topic, type, owner, expected/WARN/ERROR Hz),
`TF_CHECKS`, `LIFECYCLE`, `BLOCKERS` (rosout patterns shown in the motion chain).
Board/process rules: `console/sysmon.go` (`procDefs`, thresholds).  Log hints and noise
filters: `console/logs.go` (`hintRules`, `noiseRules`).  Owner → team mapping for the UI:
`console/web/core.js` (`OWNERS`).
