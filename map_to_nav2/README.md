# SLAM map 作為 Nav2 static map（RPLIDAR S2，ROS2 Jazzy）

`slam_toolbox` 用 `map_saver_cli -f` 存出的 `map_*.yaml + .pgm` 可直接餵給 Nav2，不用轉檔。
格式（`image / resolution / origin / negate / occupied_thresh / free_thresh / mode`）
就是 `nav2_map_server` 的格式，例：`../slam_toolbox_static/map_slamtoolbox_static_02.yaml`。

```yaml
image: map_slamtoolbox_static_02.pgm
mode: trinary
resolution: 0.050
origin: [-8.473, -9.611, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

資料流：`map_server` 發靜態 `/map`，`amcl` 發 `map→odom`，取代掉建圖時的 `slam_toolbox`。

## 0. TF 與資料流（建圖 vs Nav2 對照）

```
建圖：[S2] → /scan → [slam_toolbox：Karto＋迴環 → /map＋map→odom]
Nav2：[S2] → /scan → [amcl：/map＋/scan＋odom → map→odom]，[map_server：yaml/pgm → /map]
鏈（兩者共通後段）：odom→base_footprint（rf2o 動態）→ base_footprint→laser（static，z=雷達高度）
```

* `map→odom` 同一時間只能有一個發布者：進 Nav2 前停掉 `online_async_launch.py`（`slam_toolbox`），否則跟 `amcl` 互搶。
* `odom→base_footprint` 用 rf2o 動態版（`../slam_toolbox_rf2o/slam_s2_rf2o_headless.launch.py` 那顆，
  `publish_tf:=true`）。`slam_toolbox_static` 那種 static identity 假 odom 在 Nav2 下不可用，
  `amcl` 不會動。rf2o 和 static 版不可並存。

## 1. 安裝

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server
```

## 2. 最小驗證：只起 `map_server`

```bash
# yaml 用絕對路徑，pgm 要在同一目錄
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/home/user/Downloads/komugi/lidar/slam_toolbox_static/map_slamtoolbox_static_02.yaml
# 另開終端 activate（map_server 是 lifecycle node）
ros2 service call /map_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
ros2 service call /map_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
ros2 topic echo /map --once  # 有 OccupancyGrid 即成功
```

## 3. 正式：localization / bringup

```bash
# T1：sensor＋tf＋odom（沿用 rf2o 版，但不要起 slam_toolbox；只要驗 odom 用 rf2o_only 即可）
ros2 launch ~/rf2o_only_headless.launch.py
# 或：ros2 launch ~/slam_s2_rf2o_headless.launch.py（起完後手動殺掉 slam_toolbox 那段）

# T2：只做定位（先讓 amcl 收斂）
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/user/Downloads/komugi/lidar/slam_toolbox_static/map_slamtoolbox_static_02.yaml \
  use_sim_time:=false autostart:=true

# T3：全 Nav2（含 planner / controller / bt_navigator）
ros2 launch nav2_bringup bringup_launch.py \
  map:=/home/user/Downloads/komugi/lidar/slam_toolbox_static/map_slamtoolbox_static_02.yaml \
  use_sim_time:=false autostart:=true
```

`bringup_launch.py` 會自己帶 `map_server＋amcl＋lifecycle_manager`，不用手動 `change_state`。
自訂參數時用本目錄 `nav2_params_min.yaml`：`--params-file` 指向它，
重點是 `map_server.yaml_filename`、`amcl.scan_topic / map_topic / odom_frame_id / base_frame_id` 要對上。

## 4. 驗活

```bash
ros2 topic hz /scan                  # 期待 ~10Hz
ros2 tf2 echo map odom                # amcl 發的，必須有
ros2 tf2 echo odom base_footprint     # rf2o 發的，必須有
ros2 topic echo /map --once | grep -E 'width|height|resolution'
```

再在 rviz 用 `2D Pose Estimate` 給初位姿，`amcl` 收斂後 `/map` 與 `/scan` 會重合。

## 5. 排查

* `yaml` 內 `image: xxx.pgm` 是相對路徑：換機器／換目錄起 `map_server` 會找不到圖。
  保險起見改成絕對路徑，或把 `yaml＋pgm` 拷到同一目錄再起。
* `reparenting / multiple authority (map→odom / odom→base)`：`slam_toolbox`＋`amcl`、
  rf2o＋static odom 都不可並存，跑之前 `ps` 清乾淨。
* 手持／無輪速計：Nav2 仍可跑定位＋規劃，但物理移動靠手抱；`odom` 來源就是 rf2o，
  特徵少（空曠少牆）的地方誰都漂。
