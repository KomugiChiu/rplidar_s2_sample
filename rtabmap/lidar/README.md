# rtabmap lidar（RPLidar S2 純雷射建圖）

板上路徑：`~/rtabmap_lidar/`（本目錄同步過去）。跑在板上，不用相機。

## 檔案

| 檔案 | 用途 |
|---|---|
| `laser_mapping.sh` | 一鍵建圖：S2 driver → static TF → rtabmap（icp odom＋scan）。選配錄 bag |
| `laser_mapping.ini` | rtabmap 參數（經 `cfg:=` 載入；ini 值本來就是字串，無型別問題） |
| `set_str_param.py` | 備用：不重啟、線上改跑著節點的字串型參數 |
| `maps/` | 存下的 2D 圖（pgm＋yaml） |

## 用法（板上）

```bash
./laser_mapping.sh                         # 建圖
BAG=~/bags/lidar_$(date +%Y%m%d) ./laser_mapping.sh   # 建圖＋錄影
SKIP_LIDAR=1 ./laser_mapping.sh            # /scan 已有人發，跳過 driver
LASER_YAW_DEG=180 ./laser_mapping.sh       # odom 箭頭跟 S2 機身箭頭反向時用
```

Env：`SCAN_TOPIC`（/scan）、`LASER_FRAME`（laser，環境慣例）、
`BASE_FRAME`（base_footprint）、`LASER_Z`（0.20）、`SERIAL_PORT`（/dev/ttyUSB0）、
`ANGLE_COMPENSATE`（true）、`SCAN_MODE`（DenseBoost）。

前置：`~/test/rplidar_ros` workspace（script 會 source）、使用者在 `dialout` 群。

## 需求套件（板上實測有的）

```bash
sudo apt install ros-jazzy-rtabmap-launch ros-jazzy-rtabmap-odom \
  ros-jazzy-rtabmap-slam ros-jazzy-nav2-map-server uhubctl
# ros-jazzy-tf2-ros / rosbag2 為 Jazzy 基本盤，通常已內建
# rplidar_ros 不走 apt：~/test/rplidar_ros 原始碼編譯（S2 launch 在內）
# uhubctl：雷達 wedge 時 USB 埠斷電 30 秒重啟用（5 秒不夠）
sudo usermod -aG dialout komugi   # /dev/ttyUSB0 權限，重登入生效
```
本地 rviz 看圖：`/tmp/opencode/hector_rtabmap.rviz`（Fixed Frame 先放 `odom`，
有 `map` 後切過去）。

## 實測（2026-10-14，S2＋NT98635）
- `/scan` 10Hz → icp odom 10Hz；慢走可建出 27x35m 走廊＋房間圖（`maps/`）。
- `Icp/MaxTranslation=0.4`＋`MaxCorrespondenceDistance=0.15`（預設 0.2/0.1 會把
  正常步伐判成 out of bounds → 追蹤斷點 → 碎圖）。

## 踩雷（血淚，修完才穩定）
1. `visual_odometry:=false` 只停 odom 節點，rtabmap 本體還會等 camera topics
   （`subscribe_rgbd` 不夠，還要 `depth:=false`＋`subscribe_rgb:=false`＋
   `stereo:=false`），否則永遠 "Did not receive data"、沒圖。
2. `odom_args:=...` 傳多組參數會變成單一 argv token 被忽略；空值更會直接
   炸掉整個 launch（malformed）。調參走 `.ini`＋`cfg:=`。
3. rtabmap 數字參數是 string 型：`ros2 param set/load` 會被 YAML 轉型擋掉，
   用 `set_str_param.py` 繞過（線上微調用）。
4. 同名節點會重複上線搶 topic（odom 變 20Hz、參數讀到舊的）：殺行程一律用
   PID（`ps` 列出、只殺 >500 的、殺完驗零殘留），不要信 pkill pattern。
5. `/scan` 沒資料時先懷疑 lidar 本體：馬達轉但零 frame＝裝置 wedge，
   USB reset 不夠力時用 `uhubctl` 斷電 30 秒（5 秒不夠）再重起。
6. `ros2 topic info` 的 daemon 圖會騙人（鬼 topic、假 0 publisher），
   用 `hz`（實際流量）為準；script 的 preflight 只警告不 abort。
7. script 的 trap 在 `SKIP_LIDAR=1` 時不可殺 rplidar（會殺掉別人的正常節點）。
8. `set -u` 跟 ROS `setup.bash` 衝突（`AMENT_TRACE_SETUP_FILES`），script 不加 `-u`。
