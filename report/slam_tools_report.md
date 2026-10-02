# LiDAR SLAM 建圖工具技術報告

> SLAM 是什麼 → map 結構 → 本 repo 6 條鏈對照 → 各工具輸入/特點/安裝 → sample code 搭建 → 額外參數 → 坑速查
>
> 撰寫日期：2026-10-02
> 實測環境：Ubuntu 24.04.5 + ROS 2 Jazzy；ARM 板 `NT98635-Ubuntu`（`192.168.50.240`）+ RPLIDAR S2（`model=113`，`DenseBoost` 10Hz）
> 本文數值標示沿用 `report/rplidar_s2_lidar_report.md`：**[實測]** = 板上跑出來的、**[推導]** = 由程式碼算出的、**[規格]** = 官方定義。本節所有路徑已實讀驗證。

---

## 目錄

- [0. TL;DR](#sec-0)
- [1. SLAM 是什麼](#sec-1)
- [2. map 資料本身的結構](#sec-2)
- [3. 本 repo 建圖工具總覽](#sec-3)
- [4. 各工具需要的資料/Topic/感測器](#sec-4)
- [5. 各工具特點](#sec-5)
- [6. 如何安裝各工具](#sec-6)
- [7. Sample code 如何搭建](#sec-7)
- [8. 額外設定參數](#sec-8)
- [9. 已知坑速查](#sec-9)
- [附錄 A：檔案索引](#sec-app-a) · [附錄 B：指令速查](#sec-app-b)

---

<a id="sec-0"></a>
## 0. TL;DR

| 問題 | 答案 |
|---|---|
| SLAM 是什麼？ | **用 `/scan` 反推「我在哪 + 環境長怎樣」**。前端 scan matching 算位姿增量，後端迴環消漂，輸出 `/map` + `map→odom` |
| map 是什麼？ | 主流是 `nav_msgs/msg/OccupancyGrid`：`resolution/width/height/origin + int8[] data`（`-1`未知/`0`空閒/`100`佔據）。存檔 = `pgm + yaml`；RTAB-Map 另有 `.db` 3D graph |
| 有幾套工具？ | 6 條鏈全在 `lidar/` 下已驗：`slam_toolbox_static`、`slam_toolbox_rf2o`（=`rf2o/`）、`hector_rf2o`、`hector_use_rf2o`、`carto/`、`arm64_slam/rtabmap/` + `rf2o_only/`（純里程計，無圖） |
| 都吃什麼？ | **全部只吃 `/scan`（`LaserScan`，`frame_id=laser`，10Hz）+ TF 樹**。無 IMU、無輪速、無相機（rtabmap 的 `rgbd/fusion` 除外，尚未實測） |
| 差異一句話 | `toolbox_static` 最穩（手持首選）；`hector` 最輕（無迴環）；`carto` 上限最高（pose-graph）；`rtabmap` 唯一 3D/融合；`rf2o` 是里程計不是 SLAM，本環境**低估 7 倍判死刑** |
| 怎麼裝？ | `toolbox/hector/carto/rtabmap/map_server` 全 `apt` 即可；只有 `rplidar_ros` 和 `rf2o` 要 `source` 編 |
| Sample code 結構？ | 全部同一模板：`rplidar_node` + 1~2 條 `static_transform_publisher` + 1 個 SLAM node + 可選 `rf2o`。差異只在 `odom→base` 誰發 + SLAM 參數檔（`yaml` / `lua` /直寫 `Node parameters`） |

---

<a id="sec-1"></a>
## 1. SLAM 是什麼，能用來做什麼

### 1.1 一句話

SLAM（Simultaneous Localization and Mapping）= **在未知環境中，一邊估自身位姿、一邊畫地圖**。2D LiDAR SLAM 的輸入就是「相鄰兩圈 `/scan` 線段的形狀差」，輸出就是「位姿修正量 + 佔位地圖」。

```
前端（每幀都做）：scan matching → t時刻位姿
後端（偶爾做）  ：loop closure / pose-graph 優化 → 消累積漂移
輸出：/map（給人看/給規劃用）+ map→odom TF（給定位用）
```

### 1.2 能做什麼應用

| 應用 | 機制 | 本 repo 對應 |
|---|---|---|
| **2D 佔位建圖** | scan matching 累積 | 全部 6 鏈，`/map` 5cm/格 |
| **雷射里程計（無輪速時）** | dense scan alignment | `rf2o` → `/odom_rf2o` 10Hz |
| **迴環消漂** | 回起點做圖對圖優化 | `toolbox`（`do_loop_closure:true`）、`carto` pose-graph、`rtabmap` 外觀迴環 |
| **離線重放/調參/A-B 比較** | `ros2 bag record /scan /tf /tf_static /map` | `bags/` 11 個包 + 各目錄 `*_record.sh` / `*_replay.sh` |
| **存圖給 Nav2** | `map_saver_cli -f map` | `arm64_slam/*.pgm+yaml` 共 18 張 |
| **2D 定位（無車也能 demo）** | 載圖 + scan-to-map | `toolbox localization` / `amcl`（S2 報告 §10，未實測） |
| **3D/融合建圖預備** | 同一 `/scan` 餵 rtabmap | `rtabmap scan` 已出圖；`rgbd/fusion` 待相機 |

> 本環境是**手持雷達 + 板子，無底盤**，所以以上全是「建圖」，不是「導航」。Nav2 完整堆疊缺 `/odom` 真值 + `cmd_vel` 消費者，見 S2 報告 §10。

---

<a id="sec-2"></a>
## 2. map 資料本身的結構

### 2.1 線上：`OccupancyGrid`（toolbox / hector / carto / rtabmap 共用）

`[規格]` 定義（`nav_msgs/msg/OccupancyGrid`，全文 3 欄）：

```msg
std_msgs/Header header          # frame_id = map
nav_msgs/MapMetaData info       # resolution / width / height / origin
int8[] data                     # row-major，-1=未知，0=空閒，100=佔據
```

`[實測]` 板上行為（`bags/rosbag2_toolbox_static_2026_09_23-14_42_16`，149s）：

| topic | 頻率 | 說明 |
|---|---|---|
| `/map` | 0.19Hz（29 則） | `map_update_interval:5.0` 節流 + 走過 `minimum_travel_*` 才長大；靜止不發是正常的 |
| `/tf` 內 `map→odom` | 20Hz | `transform_publish_period:0.05`，一直送 |
| QoS | `transient_local` latch | 晚訂閱也拿得到 |

### 2.2 落盤：`pgm + yaml`

```yaml
# arm64_slam/map_s2_0922_carto_01.yaml [實測]
image: map_s2_0922_carto_01.pgm
mode: trinary
resolution: 0.050
origin: [-4.665, -9.250, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

`pgm` 是灰階圖（黑=牆、白=空、灰=未知），`yaml` 是解析度 + 原點。`map_saver_cli` 存，`map_server` 載。

### 2.3 RTAB-Map 額外結構

| 輸出 | 型別 | 說明 |
|---|---|---|
| `/rtabmap/map` | `OccupancyGrid` | 優先用它存 pgm（`save_grid_map.sh` 預設 `-t /rtabmap/map`） |
| `/rtabmap/mapData` | rtabmap 自有 | 3D graph + 特徵，rviz 看不到 |
| `~/.ros/rtabmap.db` | sqlite | 3D 資料庫；換圖要 `delete_db_on_start:=true`，否則污染 |

---

<a id="sec-3"></a>
## 3. 本 repo 建圖工具總覽 [實測]

| 目錄 | 組合 | 訂 `/scan` 者 | 產出 | `odom→base` 誰發 | 結果 |
|---|---|---|---|---|---|
| `slam_toolbox_static/` | toolbox + static 假 odom | `/slam_toolbox` | `/map` + `/tf` | static identity | **手持最穩**，`map_slamtoolbox_static_01/02` |
| `rf2o/` = `slam_toolbox_rf2o/` | toolbox + rf2o 真 odom | rf2o + toolbox | `/map` + `/odom_rf2o` | rf2o 動態 | 圖可出，但被 rf2o 拖累 |
| `rf2o_only/` | rf2o 單獨，無 SLAM | rf2o | `/odom_rf2o` 無 `/map` | rf2o 動態 | 8~10m 只量出 1.3m，**低估 7 倍** |
| `hector_rf2o/` | hector 純雷射 + rf2o 並跑記錄 | `hector_mapping` | `/map` + `/odom_rf2o` | rf2o 動態 | 圖可出；rf2o 不餵 hector |
| `hector_use_rf2o/` | hector 吃 rf2o 初估 | `hector_mapping` | 同上 | rf2o 動態 | A/B 對照 `use_tf_pose_start_estimate` true/false |
| `carto/` | cartographer + 可選 rf2o | `cartographer_node` | `/map`（經 `occupancy_grid_node`） | rf2o 或 carto 自發 | **上限最高**，pose-graph 全域優化 |
| `arm64_slam/rtabmap/` | rtabmap scan-only | `icp_odometry` + rtabmap | `/rtabmap/odom` + `/rtabmap/map` | rtabmap | 通過，34% CPU / 420MB；`rgbd/fusion` 無相機未驗 |

---

<a id="sec-4"></a>
## 4. 各工具需要的資料/Topic/感測器

### 4.1 共通輸入（全部鏈都一樣）

```
[RPLIDAR S2] --serial 1Mbaud--> [rplidar_node → /scan 10Hz, frame=laser, 3240 bin]
                                        │
                                        ▼ DDS multicast
                              [任一 SLAM node] + TF樹：
                              map → odom → base_footprint → laser
```

| 需要 | 型別 | 誰提供 | 備註 |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `rplidar_node` | 唯一感測器輸入；`intensities` 全鏈都不用（上游鏡射 bug，見 S2 報告 §7.4） |
| `odom→base_footprint` | TF（`/tf` 或 `/tf_static`） | static 或 rf2o/carto（exactly 一個） | 運動初估；static =「假設沒動」，漂由 `map→odom` 吃掉 |
| `base_footprint→laser` | TF static | `static_transform_publisher`，`z=laser_height` | 安裝高度，本 repo 預設 `0.2` |
| `map→odom` | TF dynamic | 各 SLAM node | 定位修正量；hector/toolbox 不可並存 |

### 4.2 各工具額外輸入

| 工具 | 除 `/scan` 外還吃什麼 | 感測器 |
|---|---|---|
| slam_toolbox | TF 的 `odom→base`（初估，不訂 `/odom_rf2o` topic） | 無 |
| hector | 同上；`use_tf_pose_start_estimate=true` 才看 TF，否則純雷射 | 無 |
| cartographer | `odom:=/odom_rf2o`（`use_odometry=true` 時）；`false` 時不吃 | 無（`use_imu_data=false`） |
| rtabmap scan | `/scan` 經 `icp_odometry` 轉 `/rtabmap/odom` 再進 rtabmap | 無 |
| rtabmap rgbd/fusion | RGB + Depth（對齊）+ CameraInfo | 需 RealSense/Orbbec（**本環境無，未驗**） |
| rf2o | 只吃 `/scan`，輸出 `/odom_rf2o` + TF | 無 |

---

<a id="sec-5"></a>
## 5. 各工具特點

| 工具 | 演算法 | 迴環 | 優點 | 缺點 / 本環境實測 |
|---|---|---|---|---|
| **slam_toolbox**（Karto） | scan matching + Sparse Pose Adjustment | 有（`enable_loop_closure:true`） | 手持側移最穩；`minimum_travel_*=0` 版每幀進圖 | CPU ~3%；`lifecycle` 須 `active` 才工作 |
| **hector_mapping** | Gauss-Newton multi-res 匹配，無 odom 也能動 | 無 | 最輕，不依賴 odom；小場域夠用 | 大場域漂；`map_size` 固定（預設 `2048`=102m @5cm），走出去就沒圖 |
| **cartographer** | 前端相關性匹配 + Ceres + 後端 pose-graph | 最強（全域優化） | 大場域 + 迴環上限最高；`purelaser` 無 odom 也能跑 | 最吃 CPU + 要調 `lua`；`published_frame` 誤設 `odom` 會發 `odom→odom` 自迴圈糊圖 |
| **rtabmap** | ICP odom + 圖優化 + 外觀迴環 | 有（外觀+幾何） | 唯一能融合相機、保留 3D `.db` 的；apt 即裝 | scan-only 無外觀迴環優勢不明顯；34%/420MB 最重 |
| **rf2o**（非 SLAM） | dense range-flow 配準 | — | 免硬體產 10Hz odom + covariance | **本環境低估 7 倍**；`twist` 是差分算的很 noisy；只做儀表板/黑盒子用 |

---

<a id="sec-6"></a>
## 6. 如何安裝各工具

| 工具 | 安裝方式 [實測] | 指令 |
|---|---|---|
| `rplidar_ros`（S2 driver） | **必須 source 編 `-b ros2`**，apt 無 S2 launch | `bash x86_ros2/install_driver.sh`（`x86_ros2/install_driver.sh:16`）；板上有網就同腳本原生編，否則 `arm64_ros2/cross_build_rplidar.sh` |
| `slam_toolbox` | apt 二進位 | `sudo apt install ros-jazzy-slam-toolbox`（`arm64_slam/README.md:17`） |
| `hector_mapping` | 板上 `~/hector_ws` source 編（`hector_slam_humble`）；x86 可 apt | 板：見 `hector_rf2o/README.md:3`；`mapping_default.launch.py`（注意不是 ROS1 的 `.launch`） |
| `rf2o` | source 編（~4min） | `mkdir -p ~/rf2o_ws/src && git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git && colcon build`（`rf2o/README.md:17-24`）；跳過 `rosdep`（板上 SSL 失敗，依賴已齊） |
| `cartographer` | **apt，勿 source 編** | `bash carto/carto_install.sh` = `sudo apt install ros-jazzy-cartographer-ros` + 鋪 `~/carto_config/`（`carto/carto_install.sh:7-13`） |
| `rtabmap` | apt | `sudo apt install ros-jazzy-rtabmap-launch ros-jazzy-nav2-map-server`（`arm64_slam/rtabmap/README.md:45`） |
| `nav2_map_server`（存圖） | apt | `sudo apt install ros-jazzy-nav2-map-server`，用 `map_saver_cli -f ~/map_xxx` |

---

<a id="sec-7"></a>
## 7. Sample code 如何搭建

### 7.1 通用模板（5 個 launch 共用）

```python
# slam_toolbox_static/slam_s2_headless.launch.py:18-88 [實測讀碼]
rplidar = IncludeLaunchDescription(rplidar_s2_launch.py)  # serial_port/baud=1000000/frame=laser/angle_compensate/scan_mode
odom_to_base = Node(static_transform_publisher, 0 0 0 ..., odom, base_footprint)  # rf2o鏈拿掉此行
base_to_laser = Node(static_transform_publisher, 0 0 laser_height ..., base_footprint, laser)
slam = IncludeLaunchDescription(online_async_launch.py)   # 或 hector / carto / rf2o Node
```

啟動一律：`scp *.launch.py *.sh *.yaml/*.lua <board>:~/` → `~/xxx_start.sh`（三層 `source` + `exec ros2 launch`）→ 另終端 `record.sh`（`ros2 bag record /scan /tf /tf_static [/odom_rf2o] /map`）。

### 7.2 各鏈差異

| 鏈 | launch 檔 | SLAM 節點接法 | 參數檔 |
|---|---|---|---|
| toolbox_static | `slam_toolbox_static/slam_s2_headless.launch.py` | `Include online_async_launch.py`，`slam_params_file:=~/slam_toolbox.yaml`，`use_sim_time:=false` | `slam_toolbox_static.yaml:35-43`：`resolution:0.05`，`minimum_travel_*=0` 全 0（每幀進圖），`scan_topic:/scan` |
| toolbox_rf2o | `rf2o/slam_s2_rf2o_headless.launch.py:54-68` | 同上 + `rf2o_node`（`laser_scan_topic:/scan`，`odom_topic:/odom_rf2o`，`publish_tf:True`，`freq:10.0`） | `rf2o/rf2o_params.yaml:1-9`；toolbox yaml 把 `minimum_travel_distance` 調回 `0.5`（有真 odom 才節流） |
| hector_rf2o | `hector_rf2o/hector_s2_rf2o_headless.launch.py:71-85` | `Include mapping_default.launch.py`（`scan_topic/base_frame/odom_frame/pub_map_odom_transform=true`），`use_tf_pose_start_estimate` 寫死 `false` | launch arg 直傳，無 yaml；`map_size:=2048` |
| hector_use_rf2o | `hector_use_rf2o/hector_use_rf2o_headless.launch.py:73-101` | 直寫 `Node(hector_mapping)`，`use_tf_pose_start_estimate` 可切（預設 `true` 吃 rf2o TF 初估） | 同上 + `map_resolution:0.05`，`map_update_distance_thresh:0.4` |
| carto | `carto/carto_s2_headless.launch.py:75-97` | `Node(cartographer_node -configuration_directory ~/carto_config -configuration_basename s2_*.lua)` + `occupancy_grid_node(resolution:0.05)`，`remap scan:=/scan odom:=/odom_rf2o` | `s2_rf2o_2d.lua` vs `s2_purelaser_2d.lua`：差 `use_odometry` + `provide_odom_frame` + `odometry_sampling_ratio:0.5`（繞 rf2o stamp FATAL） |
| rtabmap scan | `arm64_slam/rtabmap/launch/rtabmap_scan.launch.py` via `scripts/run_rtabmap.sh scan` | `icp_odometry`（訂 `/scan`）+ `rtabmap`（`odom_source:=icp/auto/external`） | `config/rtabmap_scan.ini`；`laser_z:=0.2`，`rgbd/fusion` 另需 `rgb_topic/depth_topic/camera_info_topic` |
| rf2o_only | `rf2o_only/rf2o_only_headless.launch.py:48-64` | 只有 `rf2o_node`，無 SLAM、無 `/map` | 同 `rf2o_params.yaml`；`init_pose_from_topic:''` 必須走 yaml（CLI `-p` 傳空字串炸 rcl 解析） |

驗活統一三行 [實測]：`ros2 topic hz /scan`（~10Hz）→ `ros2 lifecycle get /slam_toolbox`（`active [3]`）→ `ros2 run tf2_ros tf2_echo map laser`（全通）。

---

<a id="sec-8"></a>
## 8. 額外設定參數

### 8.1 感測器側（`rplidar_s2_launch.py`，全鏈共用）

| 參數 | 預設 | 說明 |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | CP2102N 映射；`lsusb -d 10c4:ea60` 先確認 |
| `serial_baudrate` | `1000000` | ★ S2 固定，寫 115200 即「連上沒資料」 |
| `frame_id` | `laser` | 覆蓋 node 內建 `laser_frame`；SLAM/TF 全鏈對齊它 |
| `angle_compensate` | `true`（3240 固定格） | 小板吃力設 `false`（1900~3500 浮動，省 40% 流量/CPU） |
| `scan_mode` | `DenseBoost` | 空字串 = 裝置 typical；填錯印出可用模式 |
| `laser_height`（本 repo 自加） | `0.2` | `base_footprint→laser` 的 `z`，手持幾公分填多少；2D SLAM 忽略 z 但 Nav2 避障需要對 |

### 8.2 工具側關鍵參數

| 工具 | 參數 | 值 / 建議 |
|---|---|---|
| toolbox | `resolution:0.05`，`max_laser_range:12.0`，`map_update_interval:5.0`，`transform_publish_period:0.05` | static 版 `minimum_*=0`；rf2o 版調回 `0.5/0.5` |
| hector | `map_resolution:0.05`，`map_size:2048`（=102m），`map_update_distance_thresh:0.4`，`pub_map_odom_transform:true` | 大場地 `map_size:=4096` |
| carto lua | `num_accumulated_range_data:10`，`max_range:12.`，`min_score:0.65`，`odometry_sampling_ratio:0.5` | 迴環接不上先降 `min_score→0.55`；CPU 不夠再加大 `accumulated` |
| rtabmap | `laser_z`，`odom_source`（`icp/rgbd/external/auto`），`delete_db_on_start`，`resolution:0.05` | 低資源用 `rtabmap_low_memory.ini` + 降相機解析度 |
| rf2o | `freq:10.0`（對齊雷達），`publish_tf:true`（命脈，關掉 toolbox 無感），`init_pose_from_topic:""` | `odom_topic` 故意叫 `/odom_rf2o` 不叫 `/odom`，跟未來輪速計區隔 |

---

<a id="sec-9"></a>
## 9. 已知坑速查（全部本專案實測踩過）

| 症狀 | 原因 | 解法 |
|---|---|---|
| `reparenting / multiple authority (odom→base)` | rf2o 動態 + static 同時發 | 只留一個；rf2o 鏈拿掉 `odom_to_base` static |
| `/map` 長不大/凍在 62x77 | 拿 rf2o 版 yaml（`minimum 0.5`）配 static odom | static 必須用 `slam_toolbox_static.yaml`（三個 minimum 全 0） |
| `rf2o Waiting for laser_scans` 永不停 | `init_pose_from_topic` 預設 `/base_pose_ground_truth` 訂不到 | 走 `--params-file` yaml 傳 `""`，勿用 CLI `-p` |
| carto timestamp FATAL | rf2o odom stamp 與 scan 同刻 | `odometry_sampling_ratio:0.5`（已預設） |
| carto 圖糊、`bag /tf` 有 `odom→odom` | lua `published_frame` 誤設 `odom` | 必須 `base_footprint` |
| toolbox 不訂 `/scan`、不發 `/map` | lifecycle 未 activate | 用 `online_async_launch.py`（自動）或手動 `configure` + `activate` |
| hector `FileNotFound` | 寫成 ROS1 `mapping_default.launch` | Jazzy 是 `mapping_default.launch.py` |
| rviz `No map received` | QoS 錯 / 包內無 `/map` | Map display 必須 `Transient Local` + `Reliable`；先 `ros2 topic echo /map --once` |
| `ros2 topic list` 陳舊 | daemon discovery 快取 | `ros2 daemon stop` |

---

<a id="sec-app-a"></a>
## 附錄 A：本 repo 檔案索引

```
lidar/
├── arm64_slam/slam_s2_headless.launch.py  static+toolbox 一鍵模板
├── slam_toolbox_static/  launch + static yaml + record/start + 2 張圖
├── slam_toolbox_rf2o/ + rf2o/  rf2o+toolbox（含 rf2o_params.yaml）
├── rf2o_only/  純里程計驗證（rviz Decay Time 看軌跡 / bag 算低估倍數）
├── hector_rf2o/  純雷射並跑版 + hector_rf2o.rviz + replay.sh
├── hector_use_rf2o/  直寫 Node 吃初估版（A/B 對照）
├── carto/  carto_install.sh + headless.launch.py + s2_{purelaser,rf2o}_2d.lua + start.sh
├── arm64_slam/rtabmap/  launch/ + config/*.ini + scripts/{run,check_topics,save_grid_map,record_bag}.sh
├── arm64_slam/map_*.pgm+yaml  18 張實測圖
├── bags/  11 個 mcap（含 toolbox_static/toolbox_rf2o/hector 兩版）
└── report/rplidar_s2_lidar_report.md  S2 底層（/scan/TF/頻寬）詳解
```

<a id="sec-app-b"></a>
## 附錄 B：指令速查

```bash
# ── 安裝 ──
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server  # toolbox+存圖
sudo apt install ros-jazzy-cartographer-ros                        # carto
sudo apt install ros-jazzy-rtabmap-launch                         # rtabmap
# rf2o：見 §6 source 編；rplidar S2：bash x86_ros2/install_driver.sh

# ── 一鍵建圖（板上）──
ros2 launch ~/slam_s2_headless.launch.py                            # static+toolbox
~/slam_s2_rf2o_start.sh laser_height:=0.15                         # +rf2o
~/carto_s2_start.sh use_odom:=false                                # carto 純激光
./arm64_slam/rtabmap/scripts/run_rtabmap.sh scan laser_z:=0.2 delete_db_on_start:=true

# ── 驗 + 存 ──
ros2 topic hz /scan; ros2 lifecycle get /slam_toolbox; ros2 run tf2_ros tf2_echo map laser
ros2 run nav2_map_server map_saver_cli -f ~/map_01
ros2 bag record /scan /tf /tf_static /odom_rf2o /map
```

### 參考

- 上游：`https://github.com/Slamtec/rplidar_ros`（`-b ros2`，v2.1.4）
- rf2o：`https://github.com/MAPIRlab/rf2o_laser_odometry`（`-b ros2`）
- ROS 2 API：`sensor_msgs/msg/LaserScan`、`nav_msgs/msg/OccupancyGrid`（Jazzy `common_interfaces`）
- 本專案實測記錄：`../README.md`、`../arm64_slam/README.md`、`../arm64_slam/rtabmap/README.md` 及各子目錄 README、`../bags/` metadata
- 底層詳解：`./rplidar_s2_lidar_report.md`（§7 `/scan`、§8 TF/odom、§10 Nav2 缺口）
