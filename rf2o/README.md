# rf2o laser odometry ＋ slam_toolbox（RPLIDAR S2，ROS2 Jazzy，免硬體 odom）

用 S2 掃描即時算出平面 odom（`rf2o_laser_odometry`），餵給 `slam_toolbox`，
取代 `arm64_slam/` 的 static identity 假 odom。
2026-09-18 於 NT98635 ARM 板（`komugi@192.168.50.240`）實測打通：
`/odom_rf2o` 10Hz、toolbox `active [3]`、tf 全樹通，靜態 CPU 約 19%
（rplidar ~8％／rf2o ~6％／toolbox ~3％）。

## 0. 前提（板上）
- Ubuntu 24.04＋ROS2 Jazzy，S2 接 `/dev/ttyUSB0`（baud 固定 `1000000`），
  使用者在 `dialout` 群
- `rplidar_ros` 已裝（`~/test/rplidar_ros/setup.bash` 可 source）
- `ros-jazzy-slam-toolbox`、`ros-jazzy-nav2-map-server` 已裝
- `~/slam_toolbox.yaml`（`odom_frame: odom／base_frame: base_footprint／
  scan_topic: /scan`），並按下述回調

## 1. 安裝（板上，原生編譯，約 4 分鐘）
```bash
mkdir -p ~/rf2o_ws/src && cd ~/rf2o_ws/src
git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git
# 原作者官方 ROS2 版（預設分支即 ros2）；備選 Adlink-ROS/rf2o_laser_odometry（humble-devel）
source /opt/ros/jazzy/setup.bash
cd ~/rf2o_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
註：`rosdep update` 在此板 SSL 失敗且 `sudo` 需密碼；依賴
（rclcpp／sensor_msgs／nav_msgs／tf2／Eigen／Boost）經 `dpkg -l` 確認已齊，
直接跳過 rosdep 編譯。`package.xml` 內的 `cmake_modules` 是 ROS1 殘留，
CMakeLists 實際沒用到，無礙。

## 2. 傳檔上板（x86 執行，本目錄四檔）
```bash
cd rf2o
scp slam_s2_rf2o_headless.launch.py slam_s2_rf2o_start.sh rf2o_params.yaml <board>:~/
```

## 3. 回調 slam_toolbox yaml（板上，有真 odom 後把距離門加回來）
```bash
cp ~/slam_toolbox.yaml ~/slam_toolbox.yaml.hector_bak   # 備份假 odom 版
sed -i "s/minimum_travel_distance: 0.0/minimum_travel_distance: 0.5/; \
s/minimum_time_interval: 0.2/minimum_time_interval: 0.5/" ~/slam_toolbox.yaml
```

## 4. 執行
### 4.1 一鍵全跑（推薦）
```bash
~/slam_s2_rf2o_start.sh
~/slam_s2_rf2o_start.sh laser_height:=0.15
~/slam_s2_rf2o_start.sh serial_port:=/dev/ttyUSB1 angle_compensate:=false
```
`slam_s2_rf2o_start.sh`＝三層 source（jazzy→rplidar→rf2o_ws）＋
`exec ros2 launch ~/slam_s2_rf2o_headless.launch.py "$@"`。
一鍵 launch 內容：rplidar S2＋static `base_footprint→laser`
（z=`laser_height`，預設 0.2）＋rf2o＋toolbox async。
**注意：`odom→base_footprint` 由 rf2o 動態發布，不可再加 static 版（TF 衝突）。**

### 4.2 手動版（4 終端，排錯用）
```bash
# 終端 1：雷達
source /opt/ros/jazzy/setup.bash; source ~/test/rplidar_ros/setup.bash
ros2 launch rplidar_ros rplidar_s2_launch.py

# 終端 2：static tf（只留這一條！odom→base 由 rf2o 發）
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0.2 0 0 0 base_footprint laser

# 終端 3：rf2o（init_pose 留空必須走 yaml 檔，見 §6 坑 1）
source /opt/ros/jazzy/setup.bash; source ~/rf2o_ws/install/setup.bash
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
  --params-file ~/rf2o_params.yaml

# 終端 4：toolbox
source /opt/ros/jazzy/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=~/slam_toolbox.yaml use_sim_time:=false
```

## 5. 驗證＋建圖（板上）
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /scan /odom_rf2o        # 期待各 ~10Hz
ros2 run tf2_ros tf2_echo odom base_footprint
# 走動時 Translation 變化（恆零＝rf2o 沒在算）；靜止接近零正常
ros2 lifecycle get /slam_toolbox      # 期待 active [3]
ros2 run tf2_ros tf2_echo map laser   # map→odom→base_footprint→laser 全通
# 抱板慢走繞一圈回起點後：
ros2 run nav2_map_server map_saver_cli -f ~/map_s2_rf2o_01
ros2 bag record /scan /tf /tf_static /odom_rf2o
```

## 6. 排查（實測踩過）
1. **`Waiting for laser_scans` 永不停**：`init_pose_from_topic` 預設是
   `/base_pose_ground_truth`，訂不到就永不初始化、所有掃描被丟。
   留空必須用 `--params-file` yaml 傳（`""`）；CLI `-p` 傳空字串會炸 rcl 解析。
2. **雙跑衝突**：hector 和 toolbox 都會發 `map→odom`，兩套不可並存，先清舊棧。
   清行程勿用 `pkill -f <長檔名>`（pattern 會匹配自己 ssh 命令列而斷線；
   `pkill -x` 又對 >15 字元名無效）：用
   `ps -eo pid,args | grep "[r]f2o_laser_odometry_node"` 取 PID 再 `kill`，
   且 kill 與重起（含裸檔名的指令）不可放同一次 ssh。
3. **ros2 剛重開看不到 node／topic 數為 0**：daemon 發現落後，
   `ros2 daemon stop` 後重查。
4. 每條 ros2 指令後噴 `sudo: a terminal is required`：`~/.bashrc` 有 sudo 行，待清。

## 7. 資料流
```
[雷達] → /scan 10Hz → [rf2o：dense scan-alignment→/odom_rf2o 10Hz＋odom→base TF]
  → [toolbox：Karto 匹配＋迴環，用真 odom 當初估→/map＋map→odom]
```
