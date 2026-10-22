# slam_toolbox + rf2o 錄影建圖（RPLIDAR S2，ROS2 Jazzy）

板端已有 `rf2o_laser_odometry`（`~/rf2o_ws`）＋`slam_toolbox`，本目錄是整理版：
從 `../rf2o/` 拷入 launch＋start＋params，補上 bag 錄影 script，三檔上板即跑。

## 0. TF 與資料流

```
[雷達] → /scan 10Hz → [rf2o：dense scan-alignment→/odom_rf2o 10Hz＋odom→base TF]
  → [toolbox：Karto 匹配＋迴環，用真 odom 當初估→/map＋map→odom]
鏈：map→odom（toolbox）→ odom→base_footprint（rf2o 動態）→ base_footprint→laser（static）
```

* `odom→base_footprint` 只許 rf2o 發動態，不可再補 static 版，否則 TF 打架。
* `rf2o_params.yaml` 內 `init_pose_from_topic: ""` 必須走 yaml 檔傳（CLI `-p` 傳空字串會炸 rcl 解析，見 `../rf2o/README.md §6`）。

## 1. 傳檔上板（本機）

```bash
cd slam_toolbox_rf2o
scp slam_s2_rf2o_headless.launch.py slam_s2_rf2o_start.sh slam_s2_rf2o_record.sh rf2o_params.yaml <board>:~/
```

## 2. 板上建圖＋錄影（2 終端）

```bash
# T1：全鏈
~/slam_s2_rf2o_start.sh
~/slam_s2_rf2o_start.sh laser_height:=0.15
~/slam_s2_rf2o_start.sh serial_port:=/dev/ttyUSB1 angle_compensate:=false
# T2：錄影（SLAM 起來、/map 出來後再錄）
~/slam_s2_rf2o_record.sh
~/slam_s2_rf2o_record.sh -o ~/rosbag2_toolbox_rf2o_01
# 驗活（T3）：
source /opt/ros/jazzy/setup.bash
ros2 topic hz /scan /odom_rf2o          # 期待各 ~10Hz
ros2 lifecycle get /slam_toolbox        # 期待 active [3]
ros2 run tf2_ros tf2_echo map laser     # map→odom→base_footprint→laser 全通
# 存圖：ros2 run nav2_map_server map_saver_cli -f ~/map_toolbox_rf2o_01
```

## 3. 本機看包

```bash
scp -r <board>:~/rosbag2_toolbox_rf2o_* .
ros2 bag info rosbag2_toolbox_rf2o_01   # 確認有 /scan /tf /tf_static /odom_rf2o /map
```

## 4. 排查

* `rf2o Waiting for laser_scans`：`init_pose_from_topic` 沒留空，見 `../rf2o/README.md §6`。
* `reparenting / multiple authority (odom→base)`：static 的 `odom_to_base` 沒拿掉，本 launch 已拿掉，不要手動再補。
* hector 和 toolbox 都會發 `map→odom`，兩套不可並存，先清舊棧。
* `ros2 剛重開看不到 node`：`ros2 daemon stop` 後重查。
