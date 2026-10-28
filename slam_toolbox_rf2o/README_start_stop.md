# slam_toolbox + rf2o（RPLIDAR S2，ROS2 Jazzy）— start / stop 腳本

以 `rtabmap/lidar/` 那套 start/stop 腳本的骨架重寫的 SLAM 流程：
**rf2o 提供 odom、slam_toolbox 提供地圖**。

## 分工

| 元件 | 產出 |
|---|---|
| `rplidar_ros` | `/scan`（frame `laser`） |
| `rf2o_laser_odometry` | `/odom_rf2o` + TF `odom → base_footprint`（動態） |
| `slam_toolbox` | `/map` + TF `map → odom`（動態） |
| `static_transform_publisher` ×2 | `base_footprint → base_link`、`base_link → laser`（static） |

完整 TF 樹：

```
map
└── odom
     └── base_footprint
          └── base_link
               └── laser
```

> `odom → base_footprint` 由 rf2o 動態發，**不要再加 static 版**（TF 衝突）。

## 板上

```bash
# 板上位置
/nvt_ros2_example/slam_toolbox_rf2o/
├── start_slam_toolbox_rf2o.sh     # 起整條流程（PID 檔 + 乾淨輸出選項）
├── stop_slam_toolbox_rf2o.sh      # 收掉整條流程（含孤兒）
├── slam_toolbox_rf2o.yaml         # slam_toolbox 參數
├── rf2o_params.yaml               # rf2o 參數
├── rviz_slam_toolbox_rf2o.rviz    # 這條流程專用 RViz（map/scan/odom/TF + costmap/plan/footprint）
├── nav2/
│   ├── start_nav2_rf2o.sh         # 起 Nav2（吃這條流程的 /map + /odom_rf2o）
│   ├── stop_nav2_rf2o.sh
│   ├── nav2_params_rf2o.yaml      # odom_topic=/odom_rf2o、static layer 讀 /map
│   ├── translation_only.py        # /cmd_vel → /cmd_vel_straight（只平移不旋轉）
│   └── behavior_trees/navigate_to_pose_bt.xml
└── patches/
    └── rf2o_laser_tf_retry.patch  # rf2o 取 TF 失敗會重試（見下面坑 3）
```

```bash
cd /nvt_ros2_example/slam_toolbox_rf2o
./start_slam_toolbox_rf2o.sh           # 建圖（背景執行，腳本立刻返回）
QUIET=0 ./start_slam_toolbox_rf2o.sh   # 把 node 輸出顯示在主控台
NO_LOG=1 ./start_slam_toolbox_rf2o.sh  # 完全不產生 log
SLAM_MODE=localization ./start_slam_toolbox_rf2o.sh
./stop_slam_toolbox_rf2o.sh            # 停止
```

與 rtabmap 那套相同的行為：
- 每個啟動的程序都記在 `/tmp/slam_toolbox_rf2o/pids`（`pid name mode`）
- 已經有實例在跑時會拒絕啟動第二份
- **啟動後立即返回，不佔住 console**（`setsid` + 背景），要停請跑 stop script
- 可從任何工作目錄執行（yaml 與 stop script 都相對於腳本本身解析）
- 只有啟動過程失敗（process 死掉）時，EXIT trap 才會把已起的東西收回來

## 重要參數

| 參數 | 預設 | 說明 |
|---|---|---|
| `LASER_YAW_DEG` | **180** | S2 實際安裝方向；與 `start_rtabmap_laser.sh` 相同（實測：往前推 20 cm，odom x 由 -0.150 → +0.386）。設 0 會讓 odom／地圖左右前後鏡像 |
| `BASE_LINK_Z` / `LASER_Z` | 0.10 / 0.20 | `base_footprint → base_link → laser` 的高度 |
| `QUIET` | 1 | 1 = 主控台只留摘要，node 輸出寫檔 |
| `SLAM_MODE` | mapping | `mapping` = `online_async_launch.py`；`localization` = `localization_launch.py` |

slam_toolbox 參數（`slam_toolbox_rf2o.yaml`）刻意調低負載：
`online_async`（同步模式在這塊 4 核 ARM 板上會吃到 220% CPU）、
`transform_publish_period: 0.1`、`minimum_travel_distance: 0.2`、
`minimum_travel_heading: 0.2`、`minimum_time_interval: 0.5`
（原本 0.0 = 站著不動也每張掃描都進 SLAM）。調整後 slam_toolbox 約 3% CPU。

## rf2o_laser_odometry 安裝（板上）

```bash
mkdir -p /root/rf2o_ws/src && cd /root/rf2o_ws/src
git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git
source /opt/ros/jazzy/setup.bash
cd /root/rf2o_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

（板上已建好，`/root/rf2o_ws/install/rf2o_laser_odometry`；編譯約 3 分 40 秒，
依賴 rclcpp / sensor_msgs / nav_msgs / tf2 / Eigen / Boost 板上已有，可跳過 rosdep。）

## Nav2（吃這條流程的 topic）

`nav2/start_nav2_rf2o.sh` 起 Nav2，**不吃底圖、也不自己跑 SLAM**：

```bash
cd /nvt_ros2_example/slam_toolbox_rf2o
./start_slam_toolbox_rf2o.sh        # 1. 先有 /map 與 map->odom
./nav2/start_nav2_rf2o.sh           # 2. 再起 Nav2（背景，約 18 秒返回）
./nav2/stop_nav2_rf2o.sh            # 停 Nav2（預設連底盤 driver 一起收）
KEEP_DRIVER=1 ./nav2/stop_nav2_rf2o.sh   # driver 留著
DRY_RUN=1 ./nav2/stop_nav2_rf2o.sh       # 只列出會停哪些程序
```

啟動參數是 `slam:=False use_localization:=False` → 不會起第二份 slam_toolbox，
也不會起 amcl / map_server。Nav2 直接吃這條流程的 topic：

| Nav2 端 | 來源 |
|---|---|
| `odom_topic`（controller_server / bt_navigator / smoother） | `/odometry/filtered`（ekf：rf2o + D455 gyro，見下） |
| global costmap static layer | `/map`（slam_toolbox，transient local） |
| local/global costmap obstacle layer、collision_monitor | `/scan` |
| TF | `map → odom → base_footprint → base_link → laser` |

其他選項：

```bash
START_DRIVER=0 ./nav2/start_nav2_rf2o.sh      # 底盤 driver 已經在跑，不要再起一份
TRANSLATION_ONLY=0 ./nav2/start_nav2_rf2o.sh  # 允許原地旋轉（預設只平移）
SKIP_CHECK=1 ./nav2/start_nav2_rf2o.sh        # 跳過 /map + TF 前置檢查
ALLOW_STRAFE=1 ./nav2/start_nav2_rf2o.sh      # 恢復 mecanum 橫移（不建議，側面沒有感測器）
SONAR_STOP=0.50 SCAN_STOP=0.35 ./nav2/start_nav2_rf2o.sh  # 調安全停止距離
USE_ULTRASONIC=false ./nav2/start_nav2_rf2o.sh      # 排除超音波停止（測試用，預設開）
SCAN_STOP_ENABLED=false ./nav2/start_nav2_rf2o.sh   # 排除雷射扇形停止（測試用，預設開）
WATCHDOG_TIMEOUT=0.1 ./nav2/start_nav2_rf2o.sh       # gate 無命令後歸零超時（預設 0.1 s）
GATE_INPUT=/cmd_vel_smoothed ./nav2/start_nav2_rf2o.sh  # gate 改接 smoother（預設 /cmd_vel）
PARAMS_FILE=.../nav2_params_rf2o_rotation.yaml TRANSLATION_ONLY=0 ./nav2/start_nav2_rf2o.sh  # 旋轉 profile
```

預設 `TRANSLATION_ONLY=1`：`translation_only.py` 把 `/cmd_vel` 的角速度歸零、
**橫移也歸零**（`allow_strafe=False`），再送到 `/cmd_vel_straight`。它同時是最後
一道安全層（也是 cmd_vel watchdog：driver 本身沒有超時，nav2 停發後 gate 補零）：

| 來源 | 預設 | 行為 |
|---|---|---|
| `/ultrasonic`（cm，`USE_ULTRASONIC` 可關） | 0.65 m | 前方障礙 < 0.65 m → 輸出 0（hold 0.5 s） |
| `/scan`（m，前方 ±35°，`SCAN_STOP_ENABLED` 可關） | 0.45 m | 正前方障礙 < 0.45 m → 輸出 0 |
| watchdog（`WATCHDOG_TIMEOUT`） | 0.1 s | 超過這麼久沒收到命令 → 輸出 0 |

兩種停止都只擋前進；`vx < 0` 的後退永遠允許。MPPI 端另外把 `vy_max` 設為 0。
gate 的輸入 topic 可用 `GATE_INPUT` 換（預設 `/cmd_vel`；測試旋轉時用
`/cmd_vel_smoothed` 繞過 collision_monitor——它在這塊板子上只能跑 ~5 Hz，
gate 的 watchdog 會把空隙填零造成抖動）。

實測（0.15 m/s，2026-10-27）：0.5 m 目標走 0.603 m、側偏 0.069 m、yaw −2.4°；
0.3 m 目標走 0.418 m、側偏 0.111 m、yaw +16°。map→odom 只漂 <3 cm/3°，SLAM
是穩的；但底盤開環會走不直，長距離導航需要 IMU/輪編碼器做 yaw 回饋
（ekf 融合已接上，見下）。

## 旋轉 profile 與 ekf yaw 融合（2026-10-28）

底盤是開環 mecanum：driver 把 `wz` 轉成 PWM 的增益只有 `vx` 的 1/8
（`speed_spin = -vz × 31.125` vs `speed_fb = vx × 255`），wz=0.2 只拿到
±6 PWM，剛好壓在馬達啟動閾值上（隨電池電壓浮動）。實測最小可轉是 **0.20**，
0.10/0.15 完全不動；0.30 轉得動但 coast 更大。

`nav2/nav2_params_rf2o_rotation.yaml`（`TRANSLATION_ONLY=0` + `PARAMS_FILE`
指定它）：yaw 容差 0.35 rad、MPPI/ smoother/behavior 的 theta 上限 0.20、
`vy` 仍鎖 0。工作點 **0.20**，`/spin` 實測 overshoot：

| 命令 | 0.20 overshoot | 0.30 overshoot |
|---|---|---|
| ±45° | +8 ~ +13°（容差 20° 內） | +14 ~ +24°（常超標） |

coast 三段（實測停命令後還多轉 ~20°→ 壓到 ~12°）：
`smoother max_decel z 0.6→1.2`、`velocity_smoother.velocity_timeout 1.0→0.1`、
gate `WATCHDOG_TIMEOUT 0.5→0.1`。剩下的約 12° 是 MCU 斷電後的機械滑行，
軟體砍不掉。

D455 IMU 融合（`ekf/`）：robot_localization 的 ekf 吃 `/odom_rf2o`
（x/y/yaw + 速度）+ gyro z 角速度，輸出 `/odometry/filtered`（20 Hz，Nav2 全上高負載時約 15 Hz），
`odom→base_footprint` 改由 ekf 發佈（rf2o 的 `publish_tf` 設 false，
板上 rf2o 重編過，舊 binary 會無視這個參數）。靜態 TF
`base_footprint→camera_link`（0.08, 0, 0.10，鏡頭朝前水平置中）。
Nav2 的三個 `odom_topic` 已改吃 `/odometry/filtered`。
gyro bias 約 −0.19°/s（3 秒轉一次只偏 ~0.6°），長漂由 rf2o 絕對 yaw 壓住。

```bash
cd /nvt_ros2_example/slam_toolbox_rf2o
./ekf/start_ekf.sh   # D455 IMU-only + static TF + ekf（SLAM 之後、Nav2 之前起）
./ekf/stop_ekf.sh
ros2 topic hz /odometry/filtered   # 20 Hz（高負載時約 15 Hz）
```

注意：
- **不要同時跑 rtabmap 那套 Nav2**（`/nvt_ros2_example/nav2/start_nav2_rtabmap.sh`）：
  節點名稱、`/cmd_vel`、costmap topic 都一樣，會互相打架。
- PID 檔 `/tmp/slam_toolbox_rf2o_nav2/pids`，log 在
  `/tmp/slam_toolbox_rf2o_nav2/logs/`。
- `rviz_slam_toolbox_rf2o.rviz` 已含 Nav2 顯示：Global Costmap、Local Costmap、
  Nav2 Plan（`/plan`）、Robot Footprint（`/local_costmap/published_footprint`）、
  Goal Pose（`/goal_pose`），工具列多一個 **2D Goal Pose**。
- 這份 params 沿用 rtabmap 的調參，log 會出現
  `configured inflation radius (0.150) is smaller than the computed inscribed radius (0.225)`
  的 ERROR；那是既有設定（inflation 0.15），不是這條流程新增的問題。

## 三個踩過的坑

1. **不要用空值參數**：`ros2 run ... -p init_pose_from_topic:=` 會被 rcl 的參數解析器
   拒絕（`RCLInvalidROSArgsError`）→ node 直接 abort。改成用 `--params-file`。
2. **`set -o pipefail` + 會失敗的 `ps`**：stop script 裡
   `pgid="$(ps -o pgid= -p "$pid" | tr -d ' ')"` 在行程剛好消失時會讓整支腳本提早結束
   → 必須加 `|| true`。
3. **rf2o 只在第一張掃描時讀一次 TF**：static TF 還沒進到 rf2o 自己的 TF buffer
   （剛開機 DDS discovery 可能慢個 1～2 秒）時，log 會出現
   `"base_footprint" passed to lookupTransform argument target_frame does not exist`；
   原版程式接著默默假設 `laser == base`，odom 就整個轉了 180°（往前推 x 變小）。
   → 只靠 start script 的 `tf2_echo` 等待**不夠**（那是另一支程式的 TF buffer，證明不了
   rf2o 自己的 buffer 已有資料；板子重開後實測還是會踩到）。
   正解是 `patches/rf2o_laser_tf_retry.patch`：`setLaserPoseFromTf()` 失敗就
   `return`，下一張掃描再試，直到成功才初始化。板上已套用並重新編譯。
   驗證方式：rf2o log 會先出現幾行 `retrying on the next scan`，之後
   `Laser odom [x,y,yaw]=[0.000 0.000 ±3.1414]` + `Robot-base odom [x,y,yaw]=[0.000 0.000 ~0]`。
   套用方式（板上）：

   ```bash
   cd /root/rf2o_ws/src/rf2o_laser_odometry
   patch -p1 < /path/to/rf2o_laser_tf_retry.patch
   cd /root/rf2o_ws && source /opt/ros/jazzy/setup.bash
   colcon build --packages-select rf2o_laser_odometry --parallel-workers 1   # 約 1 分 20 秒
   ```

## 驗收

```bash
ros2 topic hz /scan          # ~10 Hz
ros2 topic hz /odom_rf2o     # ~10 Hz
ros2 topic hz /odometry/filtered  # ~20 Hz（高負載時約 15 Hz）
ros2 topic echo /map --once  # 有 width/height/resolution
ros2 run tf2_ros tf2_echo map laser   # z = 0.20

# Nav2 起來後（不用送 goal）
ros2 lifecycle get /controller_server    # active [3]
ros2 param get /controller_server odom_topic   # /odometry/filtered
ros2 topic info /odom_rf2o -v            # ekf 有訂
ros2 topic info /map -v                  # global_costmap 有訂
ros2 topic echo /global_costmap/costmap --once --field info   # 跟 /map 一樣大
```
