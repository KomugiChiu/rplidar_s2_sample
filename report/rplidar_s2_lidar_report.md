# RPLIDAR S2 雷達技術報告

> 原理說明 → SDK 位置 → ROS 2 node 編譯 → Topic 清單與欄位解讀 → SLAM / Nav2 demo 能做什麼
>
> 撰寫日期：2026-09-29
> 實測環境：Ubuntu 24.04.5 + ROS 2 Jazzy；x86_64 主機；ARM64 板 `NT98635-Ubuntu`（`192.168.50.240`）
> 實機：RPLIDAR S2（serial 版，`model=113` / `fw=1.2` / `hw=18`）+ CP2102N USB 轉接（`10c4:ea60` → `/dev/ttyUSB0`）
> 本文所有數值分三類標示：**[實測]** = 實機跑出來的、**[推導]** = 由程式碼/實測值算出的、**[規格]** = 廠商 datasheet。

---

## 0. TL;DR

| 問題 | 答案 |
|---|---|
| SDK 在哪？ | 內嵌在 `upstream/rplidar_ros/sdk/`（SDK v2.1.0，68 檔），CMake 用 `FILE(GLOB)` 直接編進 `rplidar_node`，**不用另外裝 SDK** |
| ROS 2 node 怎麼編？ | **必須從 source 編**（`-b ros2` branch）。apt 的 `ros-jazzy-rplidar-ros 2.1.0` 沒有 S2 的 launch。指令見 `x86_ros2/install_driver.sh` |
| 雷達能給什麼 topic？ | 只有 **`/scan`（`sensor_msgs/msg/LaserScan`）** + 兩個服務 `/start_motor`、`/stop_motor`（`std_srvs/srv/Empty`）。沒有 PointCloud2、沒有 IMU、沒有 TF |
| Topic 裡的值是什麼？ | 一圈 3240 個 bin 的距離 + 反射強度。實測 `angle` 跨 359°、`range 0.15~30 m`、10 Hz、`inf` = 該方向無回波。逐欄解說見 §7 |
| 在 SLAM demo 能幹嘛？ | 本 repo 已驗 6 條鏈：`slam_toolbox`（static 假 odom / rf2o 真 odom）、`hector_mapping`、`hector+rf2o`、`cartographer`、`rtabmap`（scan-only）、`rf2o` 單獨里程計。皆為**手持建圖**，證據在 `arm64_slam/`、`bags/` |
| Nav2 呢？ | **本 repo 目前只用 `nav2_map_server` 存圖**，完整 Nav2 導航（AMCL / costmap / planner / controller）**尚未實測**，缺輪速/底盤。缺口清單見 §9 |

---

## 1. LiDAR 是什麼

### 1.1 一句話

LiDAR（Light Detection and Ranging）＝ **雷射光束掃過環境量測往返時間，換算出距離與角度，得到 2D/3D 幾何點雲**。它量的是「幾何」，不是「顏色」，所以在無光、弱紋理、強光下都比相機穩。

### 1.2 兩種主流測距原理

| 原理 | 做法 | 代表 | 優缺點 |
|---|---|---|---|
| **三角測量**（Triangulation） | 固定發光點與感測點成一組三角形，從三角形解距離 | RPLIDAR **A1/A2/A3/S1**、Hokuyo | 成本低；但**量程越遠誤差越大**（誤差與距離平方成正比），A1 官方 12 m／實際 6 m 內較準 |
| **ToF 飛行時間**（Time of Flight） | 直接量雷射發出→反射回來的時間 `t`，`d = c·t/2`（c ≈ 3×10⁸ m/s） | RPLIDAR **S2/S3**、Livox | 量程遠、精度均勻；**抗強光能力取決於發光功率與信噪比** |

> **RPLIDAR S2 是 ToF**（廠商頁面明載「DTOF 飛行時間測距技術」），32,000 次/秒取樣，量程半徑 30 m（S2 另有 18 m / 50 m 變體）。

### 1.3 為什麼 SLAM/Nav2 只吃 2D

S2 機械旋轉 + 單線雷射，一次只掃到**一個水平面**，所以單圈是 2D 線段（360 條距離值）。
2D SLAM（slam_toolbox / hector / cartographer 2D）就是拿「相鄰兩圈線段的形狀」做比對推位姿。
Z 軸（雷達離地高度）SLAM 不看，只影響**後續 Nav2 規劃與避障的物理意義**，所以安裝高度一定要對（見 §5.3）。

---

## 2. RPLIDAR S2 硬體與本機實機

### 2.1 規格 **[規格]**

| 項目 | 數值 |
|---|---|
| 測距原理 | ToF 飛行時間 |
| 取樣率 | 32,000 次/秒（32 kHz） |
| 典型掃描頻率 | 10 Hz（600 rpm） |
| 角度解析度 | 10 Hz 下 0.1125°/點（= 3200 點 ÷ 360°） |
| 掃描範圍 | 2D 平面 360° |
| 量程 | 半徑 30 m（`range_min` 0.15 m） |
| 抗光 | 80,000 lux 強光可用 |
| 介面 | TTL UART（serial）/ Ethernet / WiFi USB |
| 供電 | 5 V，啟動 1.5 A、工作 0.5~0.6 A |
| 防護 | IP65 |

### 2.2 本機連接 **[實測]**

```
[ARM 板 / 主機] USB ──> CP2102N (10c4:ea60) ──> /dev/ttyUSB0 ──> 1,000,000 baud ──> S2
```

```bash
lsusb -d 10c4:ea60          # 看到 CP2102N
ls -l /dev/ttyUSB*          # crw-rw---- 1 root dialout ... /dev/ttyUSB0
modinfo cp210x | head -1    # Ubuntu 24.04 核心內建驅動，免裝
```

**S2 serial 波特率固定 1,000,000**。用 A1/A2 範例的 115200 會「連得上但沒資料」——這是本專案踩過最多次的坑。

### 2.3 健康檢查（不起轉，零依賴）**[實測]**

```bash
python3 x86_simple/s2_info_check.py --port /dev/ttyUSB0
# INFO OK: model=113 fw=1.2 hw=18
# HEALTH OK: status=0 error_code=0
# RESULT: PASS
```

- `model=113`（0x71）= S2。SDK 內 `sl_lidar_driver.cpp` 有 `case 7: //model ID of S2`，`113 >> 4 = 7` ✔
- `status=0` = `SL_LIDAR_STATUS_OK`（1=Warning 可用，2=Error 需重開機）

> 這個腳本開頭會先送 `STOP (A5 25)`：前一次掃描若沒正常 stop，雷達還在轉，此時直接問 INFO 拿到的是掃描位元組而不是 descriptor，會誤判（實測踩過）。

### 2.4 不經 ROS 的純掃描 **[實測]**

```bash
python3 -m venv .venv
.venv/bin/pip install -r x86_simple/requirements.txt   # pyrplidarsdk + pyserial
.venv/bin/python x86_simple/s2_scan.py --port /dev/ttyUSB0 --scans 3 --out scan.csv
```

實測：每圈 1900~3500 點、3 圈共 7662 點、室內 range 0.17~4.35 m，樣本存 `x86_simple/scan_sample.csv`（欄位 `scan,angle_deg,range_m,quality`）。

> Ubuntu 24.04 有 **PEP 668**，裸 `pip install` 會被擋，一定要 venv 或 `--break-system-packages`。

---

## 3. SDK 位置

### 3.1 上游來源

| 項目 | 內容 |
|---|---|
| GitHub | `https://github.com/Slamtec/rplidar_ros` |
| Branch | **`ros2`**（⚠️ 不要用 tags `2.1.x` tarball，那是 ROS1 catkin，編譯會報需要 catkin） |
| 版本 | package `2.1.4` / **SDK `2.1.0`** |
| Commit | `24cc9b6dea97e045bda1408eaa867ce730fd3fc3`（2025-04-27） |
| License | BSD-3-Clause |

### 3.2 本機快照（免重抓）

```
/home/user/Downloads/komugi/lidar/upstream/
├── VERSION.md                 來源 / branch / commit / 快照日期
└── rplidar_ros/               109 檔（已去巢狀 .git）
    ├── src/rplidar_node.cpp   node 本體（598 行）
    ├── src/rplidar_client.cpp 測試用 client
    ├── include/rplidar_node.hpp, visibility.h
    ├── launch/                每型號一對：rplidar_<m>_launch.py（headless）＋ view_...（＋rviz）
    │                          S2 = rplidar_s2_launch.py / view_rplidar_s2_launch.py
    ├── rviz/rplidar_ros.rviz  view_ launch 載入的顯示配置
    ├── scripts/rplidar.rules  10c4:ea60 → /dev/rplidar (0777) 的 udev 規則
    └── sdk/                   ★ SDK 本體在這（68 檔）
        ├── include/           12 個公開標頭：sl_lidar_driver.h, sl_lidar_protocol.h,
        │                       rplidar.h, rplidar_cmd.h, rptypes.h ...（共 2,054 行）
        └── src/               11,480 行
            ├── sl_lidar_driver.cpp / rplidar_driver.cpp   指令、掃描、Express/DenseBoost 核心
            ├── sl_lidarprotocol_codec.cpp / sl_async_transceiver.cpp / sl_crc.cpp
            ├── sl_serial_channel.cpp / sl_tcp_channel.cpp / sl_udp_channel.cpp
            ├── dataunpacker/unpacker/  handler_capsules / hqnode / normalnode 三種封包解包器
            ├── hal/           thread / event / locker / byteops（跨平台封裝）
            └── arch/{linux,macOS,win32}/  net_serial.cpp 等平台實作
```

**關鍵設計**：SDK 不是獨立 library，而是被 `CMakeLists.txt` 用 `FILE(GLOB)` 抓進去跟 node 一起編成單一執行檔：

```cmake
set(RPLIDAR_SDK_PATH "${PROJECT_SOURCE_DIR}/sdk/")
FILE(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/*.cpp")
add_executable(rplidar_node src/rplidar_node.cpp ${RPLIDAR_SDK_SRC})
```

→ **好處**：不需要另外安裝 SDK、不會版本打架。**代價**：要改 SDK 行為就得改 `sdk/` 內檔案並重編。

### 3.3 SDK 對上層暴露的三層 API **[實測讀碼]**

```cpp
// 高階（rplidar_driver.h）：C++ 類別介面
RPlidarDriver::startScanExpress(force, modeId, &outMode)   // 選模式開掃
RPlidarDriver::grabScanDataHq(nodes[], &count)             // 取一整圈 HQ 點
ILidarDriver::getAllSupportedScanModes(vector&)            // 問裝置支援哪些模式
ILidarDriver::setMotorSpeed(rpm) / getDeviceInfo() / getHealth()

// 中階（sl_lidar_driver.h）：sl_* C 風格
// 通道層（sl_serial_channel / sl_tcp_channel / sl_udp_channel）
// 協議層（sl_lidarprotocol_codec：指令編解碼 + CRC）
```

節點實際走的是 `grabScanDataHq()` → `ascendScanData()`（依角度排序）→ 填 `LaserScan`。

### 3.4 其他語言的 SDK 選項

| 選項 | 用途 | 狀態 |
|---|---|---|
| `pyrplidarsdk` (Python, nanobind 包 C++ SDK) | 免 ROS 驗證、寫 Python node | **[實測]** 0.1.2 x86_64 與 aarch64 都有 manylinux wheel，免編譯；需 Python ≥ 3.10 |
| `pip install rplidar`（老 rplidar.py） | — | **不要用**：無 DenseBoost，只能降級玩 |

---

## 4. 編譯 ROS 2 node

### 4.1 為什麼不能直接 apt

```bash
apt show ros-jazzy-rplidar-ros        # 2.1.0
dpkg-deb -c <deb> | grep launch
# 只有 rplidar_a1/a2m7/a2m8/a2m12/a3/c1/s1/s1_tcp/s2e/s3/t1
# → 沒有 rplidar_s2_launch.py  ★
```

結論：**S2 一律 source 編**。

### 4.2 方案 A：x86 原生編譯（推薦）**[實測 ~11 秒]**

```bash
# 最快：跑既有腳本
bash /home/user/Downloads/komugi/lidar/x86_ros2/install_driver.sh
```

等價手動步驟：

```bash
sudo apt install -y python3-colcon-common-extensions git
source /opt/ros/jazzy/setup.bash

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git   # ★ -b ros2

cd ~/ros2_ws
sudo rosdep install --from-paths src --ignore-src -r -y   # 失敗可跳過
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash

# 驗收
ls ~/ros2_ws/install/rplidar_ros/share/rplidar_ros/launch/ | grep s2
# rplidar_s2_launch.py, view_rplidar_s2_launch.py
```

> 也可以完全離線用本機快照：
> ```bash
> cp -r /home/user/Downloads/komugi/lidar/upstream/rplidar_ros ~/ros2_ws/src/
> ```

### 4.3 方案 B：ARM 板原生編譯

板子有網路且 CPU 夠時，直接把 4.2 整段在板上跑一次即可（步驟完全相同）。ARM64 板上實測 node 裝在 `~/test/rplidar_ros/`。

### 4.4 方案 C：x86 host cross 編 → 傳到板 **[實測]**

```bash
bash /home/user/Downloads/komugi/lidar/arm64_ros2/cross_build_rplidar.sh
```

流程摘要（腳本已自動化）：

```bash
# 1. apt 下 arm64 版 rclcpp/sensor-msgs/std-srvs/rclcpp-components → ~/ros2_sysroot/debs
# 2. dpkg-deb -x 全部解成 sysroot
# 3. 抓 ros2 branch 原始碼到 ~/lidar_ws_arm64/src
# 4. colcon build --merge-install \
#      --cmake-args -DCMAKE_TOOLCHAIN_FILE=toolchain-aarch64-ros2.cmake \
#                    -DCMAKE_PREFIX_PATH=$SYSROOT/opt/ros/jazzy \
#                    -DAMENT_PREFIX_PATH=$SYSROOT/opt/ros/jazzy \
#                    -DCMAKE_BUILD_TYPE=Release
# 5. 驗產物架構
file ~/lidar_ws_arm64/install/rplidar_ros/lib/rplidar_ros/rplidar_node
# 期待: ELF 64-bit LSB executable, ARM aarch64 ...
```

傳到板子：

```bash
scp -r ~/lidar_ws_arm64/install <board>:~/
# [板] runtime 依賴（與 cross 下載時同名同版）
sudo apt install ros-jazzy-rclcpp ros-jazzy-sensor-msgs \
                 ros-jazzy-std-srvs ros-jazzy-rclcpp-components
source /opt/ros/jazzy/setup.bash
source ~/lidar_ws_arm64/install/setup.bash
```

`toolchain-aarch64-ros2.cmake` 兩個關鍵設定：
- `CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY`：ament/rosidl 的 try-compile 產物不能在 host 執行
- `CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER`：python / colcon 用 host 的，不要 sysroot 的

### 4.5 免 ROS 的 Python node **[實測]**

```bash
# ~/Downloads/komugi/lidar/x86_simple/s2_laserscan_node.py
python3 -m venv .venv && .venv/bin/pip install -r x86_simple/requirements.txt
.venv/bin/python x86_simple/s2_laserscan_node.py --ros-args -p port:=/dev/ttyUSB0 -p bins:=720
```

| | Python node | C++ `rplidar_node` |
|---|---|---|
| `/scan` 頻率 | 10.0 Hz（std ≈ 1.0 ms） | 10.0 Hz（std ≈ 0.9 ms） |
| CPU（top, 5 採樣） | ~9~11 % | ~8~11 %（啟動瞬間 20 %） |
| bin 數 | 固定 720（可參數調） | `angle_compensate=true` 時固定 3240 |
| 結論 | 這顆雷達量級下**無感差異**（瓶頸是 32k pts/s 的 SDK 解包 + DDS） | 長跑 jitter 略小；要 lifecycle/composition 用它 |

### 4.6 編譯期疑難

| 症狀 | 原因 | 解法 |
|---|---|---|
| `找不到 catkin` | 抓到 ROS1 tarball | 一定要 `git clone -b ros2` |
| `ls launch \| grep s2` 沒東西 | 用了 apt 版 | source 編 |
| 一堆 warning | SDK 用 `-Wall -Wextra -Wpedantic` | 無害，本專案實測只有 warning 可編過 |

---

## 5. 執行與參數

### 5.1 啟動

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash          # 板上是 ~/test/rplidar_ros/setup.bash

# Headless（SSH 也能跑）★ 預設即 serial /dev/ttyUSB0 / 1000000 / DenseBoost
ros2 launch rplidar_ros rplidar_s2_launch.py

# 要 rviz（NB 有螢幕時）
ros2 launch rplidar_ros view_rplidar_s2_launch.py

# 改參數
ros2 launch rplidar_ros rplidar_s2_launch.py \
  serial_port:=/dev/ttyUSB1 angle_compensate:=false scan_mode:=DenseBoost
```

**正常啟動應該印出** **[實測]**：

```
RPLidar running on ROS2 package rplidar_ros. RPLIDAR SDK Version:2.1.0
RPLidar S/N: xxxxxxxxxxxxxxxx
Firmware Ver: 1.02
Hardware Rev: 18
RPLidar health status : 0
RPLidar health status : OK.
current scan mode: DenseBoost, sample rate: 32 Khz, max_distance: 30.0 m, scan frequency: 10.0 Hz,
```

### 5.2 參數表

Launch 參數（`rplidar_s2_launch.py`）→ node 參數：

| Launch 參數 | 預設 | 說明 |
|---|---|---|
| `channel_type` | `serial` | `serial` / `tcp` / `udp`（S2 用 serial） |
| `serial_port` | `/dev/ttyUSB0` | |
| `serial_baudrate` | **`1000000`** | ★ S2 固定值，寫錯就是「連上沒資料」 |
| `frame_id` | `laser` | ★ 覆蓋了 node 內建預設 `laser_frame`；SLAM 端要對齊 |
| `inverted` | `false` | 是否鏡射 180° |
| `angle_compensate` | `true` | 角度補償：把不規則原始點複製填進等角度均勻格（詳見 §5.3.2，非內插） |
| `scan_mode` | `DenseBoost` | 空字串 = 用裝置 typical 模式；填錯會印出所有可用模式 |

Node 額外支援但 launch 沒暴露的參數（`ros2 param` 可設）：

| 參數 | 預設 | 說明 |
|---|---|---|
| `topic_name` | `scan` | Topic 名（相對名，實際為 `/scan`） |
| `tcp_ip` / `tcp_port` | `192.168.0.7` / `20108` | Ethernet 版用 |
| `udp_ip` / `udp_port` | `192.168.11.2` / `8089` | |
| `scan_frequency` | `10.0`（udp 時 20.0） | Tof 系列會 `setMotorSpeed(Hz×60)` 實際調轉速 |
| `auto_standby` | `false` | **沒人訂閱就自動停馬達**，省電但會有重啟延遲 |
| `flip_x_axis` | `false` | 翻轉 180° 索引 |

### 5.3 一圈多少點：3200、3240 與 `angle_compensate`

#### 5.3.1 3200 怎麼來的：取樣率 ÷ 轉速

```
一圈點數 = 32,000 (次/秒) ÷ 10 (圈/秒) = 3200 點/圈
```

- **32,000 次/秒**：S2 在 DenseBoost 下的雷射打點速度。SDK 回報 `us_per_sample = 31.25µs`，
  `1,000,000 ÷ 31.25 = 32,000`，node 開機 log 的 `sample rate: 32 Khz` 就是它。
- **10 圈/秒**：馬達 600 RPM（`scan_frequency` 參數，`600 ÷ 60 = 10Hz`），ToF 系列 node 會實際下
  `setMotorSpeed(600)` 去鎖轉速。
- 程式同一算式（`src/rplidar_node.cpp:324`）：
  `points_per_circle = 1000×1000 / us_per_sample / scan_frequency`。
- **3200 是期望值**：轉速會飄，實際每圈 3100~3300 都正常；
  串口驗算也對得上：32000 點 × 2B = 64kB/s，佔 1Mbaud（100kB/s）的 64%。

#### 5.3.2 3240 怎麼來的：固定格子，不是內插

`angle_compensate=true`（預設）時（`src/rplidar_node.cpp:325-326, 491-502`）：

```cpp
angle_compensate_multiple = points_per_circle / 360 + 1;  // 3200/360 = 8 → 9
node_count = 360 * angle_compensate_multiple;             // 3240 格，每格 0.1108°
```

做法是**複製填格（zero-order hold），不是內插**：開 3240 格歸零，每個原始量測往自己的角度位置
往後蓋 9 格，後蓋的覆蓋先蓋的；沒被蓋到的格子維持 0 → 發成 `inf`。
沒有任何平均或加權運算，**每個發佈值都是某個原始量測的精確拷貝，沒有算出新數字**。

推論：

- 資訊量還是物理的 ~3200 點，3240 只是格子數（多 40 格是為了每度湊整數格）。
- 空格的 `inf` 有兩種來源：真無回波方向（大宗）與量化漏格（8.9 點/度 vs 9 格/度偶爾漏一格）——
  下游都當無效處理，實務無差別。
- 好處：均勻解析度，下游（rviz、SLAM、AMCL）拿到固定網格。
  代價：CPU 多一層填格迴圈——**ARM 小板吃力就設 `false`** [實測踩過]。

#### 5.3.3 `false` 時為什麼不是固定

`false` 那條路徑不做重採樣，直接把原始點切片發出去（`src/rplidar_node.cpp:518-531`）：

```cpp
while (nodes[i++].dist_mm_q2 == 0);  start_node = i-1;   // 切掉頭部無效點
i = count - 1;
while (nodes[i--].dist_mm_q2 == 0);  end_node = i+1;     // 切掉尾部無效點
angle_min = getAngle(nodes[start_node]);                 // 該圈首個有效點實測角
angle_max = getAngle(nodes[end_node]);                   // 該圈末個有效點實測角
publish_scan(&nodes[start_node], end_node - start_node + 1, ...);
```

點數 = `end_node - start_node + 1`，三件事每圈都在變：
1. **轉速抖動**：grab 按時間抓一圈，600rpm 飄一下就差幾十上百點；
2. **有無回波**：`dist == 0` 的方向不計入，切掉的頭尾長度每圈不同
  （`false` 曾看到 1900 點，就是頭尾剛好落在無回波區被切掉，不是馬達掉速）；
3. **轉子相位**：每次 grab 起始角度隨機，首末有效點落在哪個角度都不一樣——
   所以 `angle_min/max` 也浮動，不再是 ±180°，**下游必須能接受變長陣列**，
   `angle_increment` 也是每則現算的（`span/(N-1)`，`:253`）。

#### 5.3.4 角度各不同，距離值常重複

`true` 模式下 3240 個 bin 的角度是等差數列（`angle_min + i×angle_increment`），兩兩不同、
−179° 起 +180° 止全跨 359°、沒有繞圈重疊。
但角度不同 ≠ 量到不同東西：同一原始點被複製到相鄰多格是常態
（實測前 10 點：`0.326×3, 0.328×3, 0.329×2, …`）。
寫 scan matching 時相鄰點高度相關是正常的，不要當 3240 個獨立觀測——
有效角解析度就是物理的 ~0.11°，填格不會變出新資訊。

#### 5.3.5 板端實測驗證（2026-09-29，ARM 板，S2 預設啟動）

```
current scan mode: DenseBoost, sample rate: 32 Khz, max_distance: 30.0 m, scan frequency: 10.0 Hz
N_ranges = N_intensities = 3240（= (max-min)/inc + 1 ✔）
valid 2399（74.0%）/ inf 544（16.8%）/ 盲區<0.15m 297（9.2%）
min 0.150m / max 3.812m / mean 0.381m / median 0.256m
hz: 10.0 Hz（std ~0.0005 s）
```

> 注意：`ros2 topic echo` 預設只印前 128 個陣列元素（後面變 `'...'`），
> 要拿完整一圈必須加 `--full-length`，否則會誤判 `N=128`（實測踩過）。

### 5.4 驗收

```bash
ros2 topic hz /scan                          # 期待 ~10.0Hz
ros2 topic echo /scan --once | head -n 20    # 對照 x86_ros2/scan_once_sample.yaml
ros2 topic bw /scan                          # 期待 ~260 kB/s（見 §7.6）
ros2 service list | grep motor                # start_motor / stop_motor
ros2 param get /rplidar_node frame_id         # 期待 "laser"
```

Headless 一鍵驗證：`bash x86_ros2/s2_ros_check.sh /dev/ttyUSB0 12`

---

## 6. Topic 清單

`rplidar_node` 建立的介面**只有這些**（[實測讀碼] `src/rplidar_node.cpp`）：

| 名稱 | 型別 | 方向 | 頻率 | QoS | 說明 |
|---|---|---|---|---|---|
| **`/scan`** | `sensor_msgs/msg/LaserScan` | Pub | 10 Hz | `KeepLast(10)` = **Reliable + Volatile** | ★ 唯一資料輸出。**沒有** PointCloud2 |
| `/start_motor` | `std_srvs/srv/Empty` | Srv | — | — | 呼叫後 `setMotorSpeed()` + 重設 scan mode |
| `/stop_motor` | `std_srvs/srv/Empty` | Srv | — | — | 停掃 + `setMotorSpeed(0)`。`auto_standby:=true` 時**兩個服務都被忽略** |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | Pub | 事件驅動 | — | ROS 標準 |
| `/rosout`、`/rosout_ros` | `rcl_interfaces/msg/Log` | Pub | 事件驅動 | — | ROS 標準 |

**雷達「不提供」的東西**（要就自己加）：

| 想要 | 沒有 | 補法 |
|---|---|---|
| 點雲 `/points`（`PointCloud2`） | ✗ | 轉換：`laser_geometry` 或自己寫（3240 bin × 10 Hz 很小，`pointcloud_to_laserscan` 那套是反向） |
| 里程計 `/odom` | ✗ | `rf2o_laser_odometry`（本 repo `rf2o/`，**[實測]** 10 Hz） |
| IMU | ✗ | S2 內建無 IMU |
| 雷達自我狀態 `/diagnostics` | ✗ | 只有開機時打一次 `getDeviceInfo` / `getHealth`，要持續監控得自己包 `diagnostic_updater` |
| TF | ✗ | 全部要外部發（static_transform_publisher 或 SLAM 節點） |
| 角度補償後的 PointCloud2 | ✗ | — |

服務用法：

```bash
ros2 service call /stop_motor std_srvs/srv/Empty     # 省電／讓別的工具搶串口前先停
ros2 service call /start_motor std_srvs/srv/Empty
```

> QoS 相容性提醒：`/scan` 是 **Reliable**。Reliable publisher 餵 Best-Effort subscriber 是**相容**的（Nav2/SLAM 預設 SensorDataQoS 都能收到），但反過來不行——自己寫 publisher 用 Best-Effort 會讓 reliable 的下游全收不到。實測 bag metadata 確認 `/scan` = `reliability: reliable, durability: volatile` ✔

---

## 7. `/scan` 欄位逐項解說

### 7.1 原始 `.msg` 定義 [實測讀檔]

ROS 2 Jazzy 實際檔案：`/opt/ros/jazzy/share/sensor_msgs/msg/LaserScan.msg`，**全文只有 30 行**：

```msg
# Single scan from a planar laser range-finder
#
# If you have another ranging device with different behavior (e.g. a sonar
# array), please find or create a different message, since applications
# will make fairly laser-specific assumptions about this data

std_msgs/Header header
                              # timestamp in the header is the acquisition time of
                              # the first ray in the scan.
                              #
                              # in frame frame_id, angles are measured around
                              # the positive Z axis (counterclockwise, if Z is up)
                              # with zero angle being forward along the x axis

float32 angle_min            # start angle of the scan [rad]
float32 angle_max            # end angle of the scan [rad]
float32 angle_increment      # angular distance between measurements [rad]

float32 time_increment       # time between measurements [seconds] - if your scanner
                              # is moving, this will be used in interpolating position
                              # of 3d points
float32 scan_time            # time between scans [seconds]

float32 range_min            # minimum range value [m]
float32 range_max            # maximum range value [m]

float32[] ranges             # range data [m]
                              # (Note: values < range_min or > range_max should be discarded)
float32[] intensities        # intensity data [device-specific units]. If your
                              # device does not provide intensities, please leave
                              # the array empty.
```

**設計意圖**（官方註解直接寫在檔案裡）：

1. **整包只有 9 個欄位**，其中 7 個 `float32` 標量 + 2 個 `float32[]`。資訊密度極高——這是刻意設計的，不是偷懶。
2. **座標系約定寫死在註解裡**：`header.frame_id` 這個座標系中，角度是**繞 +Z 軸量測**，**Z 朝上時為逆時針**，
   **0 角 = +X 方向（正前方）**。這一條是整個 ROS 生態的假設，rviz / slam_toolbox / Nav2 costmap 全都靠它。
3. **`header.stamp` 是「第一條光線的量測時刻」**，不是「訊息產出時刻」。
4. **兩個陣列長度必須相同**，index 對齊：`ranges[i]` 對 `intensities[i]` 對 `angle_min + i*angle_increment`。
5. **`ranges` 有明確的無效值約定**：小於 `range_min` 或大於 `range_max` 的值**必須丟棄**；
   `inf` 是「無回波」的慣例表示。
6. **`intensities` 單位是 device-specific**，沒有的話**留空陣列**（不是填 0）——這是本專案踩到的坑，見 §7.4(a)。
7. 註解開頭明講：**這是給雷射專用**，聲納等不同原理的裝置不要用這包（下游會做雷射假設）。

**巢狀型別**（`ros2 interface show` 展開）：

```
std_msgs/Header                    /opt/ros/jazzy/share/std_msgs/msg/Header.msg
├── builtin_interfaces/Time
│   ├── int32  sec
│   └── uint32 nanosec             # 範圍 [0, 1e9)，-1.7 s = {sec: -2, nanosec: 3e8}
└── string frame_id
```

`nanosec` 是 `uint32` 不是 `int32`——負時間靠 `sec` 借位（`-1.7 s` 表示成 `{sec: -2, nanosec: 3e8}`），
所以 `stamp.sec + stamp.nanosec * 1e-9` 這種寫法在負時間會算錯，實務上要轉成 `builtin_interfaces.msg.Time` 用。

### 7.2 欄位定義 + 本機實測值

實測樣本：`x86_ros2/scan_once_sample.yaml`（2026-09-10，x86，S2 DenseBoost）

| 欄位 | 型別 | 實測值 | 單位 | 意義 |
|---|---|---|---|---|
| `header.stamp` | `Time` | `1789038601.346582855` | s | ★ **本圈掃描「開始」的時刻**（node 在 `grabScanDataHq()` 前取時間）。用系統時間，不要在 replay 時開 `use_sim_time` 混著算 |
| `header.frame_id` | `string` | `laser` | — | ★ 這圈資料的座標系。SLAM 靠它查 TF 換到 `base_footprint`；**寫錯等於下游全廢** |
| `angle_min` | `float32` | `-3.1241390705108643` | rad (**-179.0°**) | 第 0 個 bin 的角度 |
| `angle_max` | `float32` | `3.1415927410125732` | rad (**+180.0°**) | 最後一個 bin 的角度 |
| `angle_increment` | `float32` | `0.0019344649044796824` | rad/bin (**0.1108°**) | 角度步長 |
| `scan_time` | `float32` | `0.09930974990129471` | s | ★ **實務上是「阻塞等一整圈花了多久」**，≈ 1/10.07 s。不是裝置回報的掃描週期 |
| `time_increment` | `float32` | `3.066061981371604e-05` | s/bin | `scan_time / (N-1)`。用於把每個 bin 的量測時間插到點上（做 motion compensation 時才會用到） |
| `range_min` | `float32` | `0.15000000596046448` | m | 有效量測下限。**node 裡寫死 0.15**；小於此值視為無效 |
| `range_max` | `float32` | `30.0` | m | ★ 由 `scan_mode` 的 `max_distance` 帶入（S2 DenseBoost = 30 m）。下游用它裁掉不可能的距離 |
| `ranges` | `float32[N]` | `1.141, 1.138, ..., .inf, ...` | m | ★ **本圈每個 bin 的距離**。`inf` = 該方向無回波（超量程／無反射／雷射被遮） |
| `intensities` | `float32[N]` | `0.0, 47.0, 47.0, ...` | 無單位 | 回波強度 = SDK `quality >> 2`（0~63）。`0` = 該 bin 無有效回波 |

**注意：7 個標量欄位全是 `float32`，不是 `float64`** [實測]。`scan_once_sample.yaml` 裡那些
`1.1410000324249268`、`-3.1241390705108643` 這種 17 位長尾數字，是 **`float32 → float64` 提升印出來的副作用**，
不是真有 double 精度。驗證（每個值都能 float32 round-trip 精確還原）：

```python
>>> struct.unpack('f', struct.pack('f', 1.1410000324249268))[0] == 1.1410000324249268
True
```

實際精度：距離 1.141 m 只能到約 **6×10⁻⁸ m**（float32 的 24 位尾數），角度 0.1108° 增量約 1.2×10⁻⁸ rad。
對 1 cm 解析度的 SLAM **綽綽有餘**；但別拿 `angle_min + i*angle_increment` 在 i=3239 的末端做精密角度累積。

**陣列長度 N = 3240** [推導]：

```
N = (angle_max - angle_min) / angle_increment + 1
  = (3.1415927 - (-3.1241391)) / 0.0019344649 + 1
  = 6.2657318 / 0.0019344649 + 1 = 3239 + 1 = 3240   ✔ 與 360×angle_compensate_multiple(9) 一致
```

### 7.3 從數值還原點座標

```python
import math
msg = <LaserScan>
N = len(msg.ranges)
pts = []
for i, r in enumerate(msg.ranges):
    if math.isinf(r) or r < msg.range_min or r > msg.range_max:
        continue                                  # 濾掉無效
    a = msg.angle_min + i * msg.angle_increment   # 雷達座標角度
    pts.append((r * math.cos(a), r * math.sin(a)))# 換成 x, y
```

要把點搬到 `map` 座標系，用 TF：

```bash
ros2 run tf2_ros tf2_echo map laser     # 手動看換算關係
# 程式裡用 tf2_ros buffer.lookupTransform("map", msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp))
```

**時戳很重要**：`lookupTransform` 要用 `msg.header.stamp` 而不是 `now()`，否則雷達在動時會有幾公分的「時間撕裂」誤差。

### 7.4 兩個實作陷阱（讀碼 + 實測佐證）

**(a) `intensities` 與 `ranges` 索引不對齊（上游 bug）**

`src/rplidar_node.cpp:278-282`：

```cpp
float read_value = (float)nodes[i].dist_mm_q2 / 4.0f / 1000;   // 讀 nodes[i]
size_t apply_index = i;
if (reverse_data) apply_index = node_count - 1 - i;             // 鏡射

scan_msg->ranges[apply_index] = read_value;                     // ✔ 用 nodes[i]
scan_msg->intensities[apply_index] = (float)(nodes[apply_index].quality >> 2);  // ✘ 讀 nodes[apply_index]
```

`inverted=false`（預設）時 `reverse_data = (!inverted && reversed) = true`，兩者索引不同。
**實測佐證**：`scan_once_sample.yaml` 前 13 個 bin 的 `ranges` 是連續的 `1.141 / 1.138 / …`（有效回波），但 `intensities[0..12] = 0.0`——對不上，正是這個鏡射造成的。

→ **影響**：本專案所有 SLAM 鏈都沒用到 `intensities`，無實害。
→ **若要做**「用反射強度濾掉玻璃／黑色物體」之類的功能，必須先修此行（改成 `nodes[i].quality >> 2`）重編，或自己寫 node。

**(b) `intensities` 的單位是 `quality >> 2`**

SDK 原始 `quality` 欄位是 8-bit（實測到 188），`intensities[i] = quality_i >> 2`，
對照 `x86_simple/scan_sample.csv` 同場景 `quality=188`：`188 >> 2 = 47` ✔
所以 `intensities` 值域約 0~63.75。要比較跨場景相對強弱可以，
**不要當絕對物理量**（會隨環境光、距離、髒鏡面變動）。

### 7.5 角度原點：訊息 0 rad 對應裝置的哪裡？

[推導] `publish_scan()` 用 `angle_msg = π - angle_device`，`apply_index = N-1-i`，
所以 `angle_msg = 0`（正前方，rviz 的 +X）對應**裝置的 180° 位置**（通常是雷達外殼的接口/背面方向）。

實務影響：裝雷達時外殼朝向會決定機器人「正前方」是哪個物理方向。
**建議開機後拿一塊紙板在機體正前方走一圈，rviz 看光點落點確認**；不對就用 `inverted:=true` 翻 180°。

### 7.6 流量與頻寬估算 [推導]

**DDS 側（單則 `/scan`，CDR 實際佈局）**：

```
std_msgs/Header
  stamp.sec/nanosec   = 4 + 4                =  8 B
  frame_id            = 4(len) + 5("laser")  =  9 B → 對齊補到 12 B
7 × float32                                    = 28 B
ranges      = 4(len) + 3240 × 4               = 12,964 B
intensities = 4(len) + 3240 × 4               = 12,964 B
──────────────────────────────────────────────────────
每則 ≈ 25,976 B ≈ 25.4 KiB  →  × 10 Hz ≈ 260 kB/s ≈ 2.1 Mbit/s（未計 DDS overhead）
```

> `ranges` + `intensities` 佔 **99.7 %** 的訊息體積。
> 換句話說 **`/scan` 的流量幾乎完全由「bin 數 × 2 個 float32 陣列」決定**，header 與 7 個標量可忽略。

**Serial 側**：

```
32,000 樣本/秒 × 2 B/樣本(DenseBoost: 1B dist + 1B quality) ≈ 64 kB/s
1 Mbaud @ 8N1 = 1,000,000 / 10 = 100 kB/s  →  使用率約 64%
```

→ 這就是 README 說「1Mbaud 接近上限、用短線直插」的量化依據。**[實測]** 換長線/USB 供電弱時點數會亂跳、轉速上不去。

**省頻寬的選項**：
- `angle_compensate:=false` → 1900~3500 bin，少 40 % 體積但角度不均勻
- 不要用 `PointCloud2` 中轉（同樣資料量、overhead 更大）

### 7.7 三種尺度的點位對照 [實測]

以下全部取自板上同一圈（2026-09-29，`stamp=1790649729.185`，N=3240）。
心算基準：9 bin = 1°，90 bin = 10°，1080 bin = 120°。

| | 相鄰（稠密區 bin 0→1） | 相鄰（邊界 bin 1000→1001） | 相距 1000（bin 0/1000/2000/3000） | 繞一圈（bin 0→3239） |
|---|---|---|---|---|
| 索引間隔 | 1 | 1 | 1000 | 3239 |
| 角度間隔 | **0.1108°** | **0.1108°** | **110.84°**（約 1/3 圈） | **359°** |
| 0.3 m 處弧長 | 0.58 mm | 0.58 mm | 0.58 m | 1.88 m（周長） |
| 時間間隔 | 29.5 µs | 29.5 µs | 29.5 ms | 95.5 ms（= `scan_time`） |
| 1 m/s 下的位移 | 0.03 mm（可無視） | 0.03 mm | 3 cm（deskew 邊緣） | ~10 cm（必須 deskew） |
| `ranges` | 0.326 → 0.326（相同，複製） | 0.310 → `inf`（斷崖） | 0.326 / 0.310 / 0.594 / 0.208（互不相關） | 2399 有效 + 544 `inf` + 297 盲區 |
| `intensities` | 0 → 0 | 47 → 47 | 0 / 47 / 0 / 47 | 0 與 47 交錯 |
| 兩欄對得上嗎 | ✘（距離有效，強度 0） | ✘（`inf` 配 47） | ✘（隨機） | ✘（471 + 478 處錯位，見 §7.4） |
| 代表意義 | 同一物體表面的拷貝 | 物體邊緣 / 門縫 | 房間不同面的指紋 | 環境完整輪廓（定位用） |

實測值明細：

```
相鄰： (0,-179.00°,0.326) (1,-178.89°,0.326) (2,-178.78°,0.326) (3,-178.67°,0.328)
千格： (0,-179.0°,0.326) (1000,-68.16°,0.310) (2000,+42.67°,0.594) (3000,+153.51°,0.208)
邊界： (1000,-68.16°,0.310) → (1001,-68.05°,inf)；(1999~2001,+42.6°~+42.8°,0.594×3)
圈尾： (3237,179.78°,inf) (3238,179.89°,inf) (3239,180.0°,inf)
```

三個觀察：

1. **`intensities` 在三個尺度全錯位，不是偶發**（稠密區 0、邊界 47、`inf` 配 47 交錯），
   證實是系統性索引鏡射 bug——整圈都不能用。
2. **時間是隱藏的第四維**：`time_increment`（29.5µs）× 索引差 = 兩點真實時間差。
   相鄰點可當同時，整圈差 ~10 cm（1 m/s 時）——走動建圖必須用 `stamp + i×time_increment` 逐點校正；
   靜止手持建圖可無視，這也是本專案 SLAM 鏈沒處理它卻照樣能動的原因。
3. **三尺度對應 SLAM 三層用法**：相鄰點看表面連續性（濾波殺孤點）→
   千格看形狀指紋（scan matching 比對）→ 整圈是完整觀測（迴環、costmap、定位）。

### 7.8 `stamp` 與 `frame_id` 的更新語意

- **`header.stamp` 一圈變一次**：一則 `/scan` = 一圈 = 一個 stamp（第一條光線的時刻），
  3240 bin 共用。Node 先 `start_scan_time = now()` 再阻塞抓一整圈（`:467-469`），
  所以 stamp 以 10Hz 更新、圈內不變。實證：截斷版那圈 `1790649640.175`，
  完整版那圈 `1790649729.185`（不同圈，不同 stamp）。
  各點真實時刻 = `stamp + i × time_increment`。
- **`frame_id` 永遠不變**（執行中）：固定 `"laser"`，是靜態配置（launch 參數），
  只有重起 node 改參數才會變。它是 TF 樹的鑰匙：SLAM 拿 `stamp` 配 `frame_id`
  查 `map→…→laser`，一個管時間、一個管空間。

### 7.9 角度問答：原始角、格子角與真值 [實測讀碼]

#### Q1：原始資料知道每點的角度嗎？

知道。**SDK 每個 HQ 樣本自帶角度**（`angle_z_q14`，解析度 90/16384 ≈ 0.0055°），
比發佈格子（0.1108°）細 20 倍。用 `s2_scan.py` 直接讀 SDK，每點角度就是量到的。
「只知道頭尾」是 `false` 發佈格式的假象，不是原始資料的限制。

#### Q2：發佈後每點角度還準嗎？

| 層級 | 每點角度是什麼 | 誤差 |
|---|---|---|
| SDK 原始 | 真量測 | 0.0055° |
| `true` 發佈 | 格子角（已用真角度分過格，`:493-494`） | ≤ 半格 0.055°（5 m 處 ≈ 4.8 mm） |
| `false` 發佈 | 等距假設（只有頭尾是真的，`:253`） | 圈內轉速抖動級（< 0.1°） |

`false` 中間點的真角度發佈後找不回來；要精確 deskew 就繞過 node 讀 SDK，
或改 node 多發一欄真角度。

#### Q3：`true` 有資料的格子，值就是真值嗎？

**值是真的**（原始量測的精確拷貝），附帶四個條件：

1. **角度標籤有半格誤差**：值是真點的，標的是格心的角，差 ≤ 0.055°。
2. **歸屬可能是隔壁點**：每點往後蓋 9 格、後蓋覆蓋先蓋，格子裡是「附近約 1° 內某點」的真值。
   平滑牆面無差；物體邊緣斷點位置有 1 格不確定性
   （bin 1000 = 0.310 → bin 1001 = `inf` 那道斷崖）。
3. **盲區值照發**：< 0.15 m 的也是真量測，node 不濾，下游自己丟。
4. **同索引的 `intensity` 不是同一點的**（§7.4 錯位 bug）：距離可信、強度不可信，分開看。

#### Q4：3240 到底是不是內插？

不是。判定句：

> 原始每點自帶角度；發佈成 `/scan` 時，按角度分格、複製填入 3240 格，沒分到點的格子填 `inf`。

兩個詞不能錯：「內插」→「複製填入」（無加權平均、無新數值）；
「填滿」→「填格」（板上那圈只填 2696 格，544 格是空的）。
**試金石：看到 `ranges` 裡的 `inf`，就知道它不是內插——內插不會產出 `inf`，只有「有格沒點」才會。**

#### Q5：3240 個角度會重複嗎？

不會。`angle[i] = angle_min + i×increment` 是嚴格等差數列（公差 0.1108° > 0），
−179° 起 +180° 止全跨 359°，沒有繞圈重疊的 bin。
但角度不同 ≠ 值不同：同一原始點被複製到相鄰多格是常態
（`0.326×3, 0.328×3, …`），有效角解析度仍是物理的 ~0.11°。

### 7.10 SDK 原始樣本實例 [實測]

2026-09-29 板上直讀 SDK（`pyrplidarsdk.get_scan_data()`，未經 node 轉發，同顆 S2）：
`DeviceInfo(model=113, firmware=258, hardware=18)`，`health OK`，
這圈 `N=2411`（注意：不是 3200——grab 內容每圈浮動，又一證據）。

#### 相鄰的點：間距不均、值各自獨立

```
raw[0] angle=359.951° range=0.3250m q=188
raw[1] angle=  0.077° range=0.3230m q=188
raw[2] angle=  0.187° range=0.3210m q=188
raw[3] angle=  0.297° range=0.3190m q=188
raw[4] angle=  0.401° range=0.3180m q=188
raw[5] angle=  0.511° range=0.3200m q=188
```

角度差 0.126 / 0.110 / 0.110 / 0.104 / 0.110°——**不固定**（轉速抖動）；
距離 0.3250 → 0.3230 → 0.3210 → 0.3190，**各差 1~2mm、獨立量測**。
對比發佈版 `0.326×3` 連格相同——那是填格拷貝，這裡才是真相。
本圈 `quality` 全 188、`zeros=0`（零無效點；node 的 `intensity 47 = 188>>2`）。

#### 差 90 度的點：隔 565 格，毫不相干

```
raw[0]   angle=359.951° range=0.3250m q=188
raw[565] angle= 89.621° range=0.4620m q=188   (Δ=89.67°)
```

落在第 565 格而不是理論的 ~603 格（2411/4）——間距不均，「數格子找角度」只能估算。

#### 繞完一圈：尾接頭連續，無斷點

```
raw[2408] angle=359.637° range=0.3280m
raw[2409] angle=359.747° range=0.3270m
raw[2410] angle=359.841° range=0.3260m
raw[0]    angle=359.951° range=0.3250m
raw[1]    angle=  0.077° range=0.3230m
raw[2]    angle=  0.187° range=0.3210m
```

跨 0° 連續（值也連續 0.326 → 0.325 → 0.323），「圈」沒有物理斷點；
陣列從 359.95° 開始——**grab 起點是隨機轉子相位**，`flag` 只標記過圈處。

#### 原始 vs 發佈對照

| | SDK 原始（本圈） | `/scan` 發佈（前一圈） |
|---|---|---|
| 點數 | 2411（浮動） | 3240（固定格） |
| 角度間隔 | 0.10~0.13° 不均 | 0.1108° 等距 |
| 相鄰距離值 | 各差 1~2mm（獨立） | 常連格相同（拷貝） |
| 品質 | 全 188（`zeros=0`） | 47 或 0（`>>2` + 錯位，§7.4） |
| 起點 | 隨機相位（這次 359.95°） | 固定 −179° |



---

## 8. 在本專案的 SLAM demo 環境能做的事

### 8.1 全鏈資料流（已實測）

```
[RPLIDAR S2] --serial 1Mbaud--> [rplidar_node : SDK 解包 → /scan 10Hz, frame=laser]
                                       │  DDS multicast
                                       ▼
        ┌──────────────────────────────┼───────────────────────────────┐
        ▼                              ▼                               ▼
 [rf2o_laser_odometry]        [slam_toolbox]                   [hector_mapping]
   dense scan alignment        Karto 配準 + 迴環                  scan matching
   → /odom_rf2o (10Hz)         → /map + map→odom                 → /map + map→odom
   → TF odom→base_footprint                                       (選配 use_tf_pose_start_estimate 吃 rf2o)
        │                              │                               │
        └──────────┬───────────────────┴───────────────────────────────┘
                   ▼
  [cartographer_node] / [rtabmap icp_odometry]  → /map 或 /rtabmap/map
                   ▼
  [nav2_map_server] map_saver_cli -f map   /   ros2 bag record  →  回 x86 重放調參
```

### 8.2 各 demo 鏈對照（全部 **[實測]**，2026-09-16 ~ 09-24，ARM 板）

| 目錄 | 組合 | 訂 `/scan` 的 node | 產出 topic | `odom→base_footprint` 誰發 | 靜態 CPU | 實測結果 |
|---|---|---|---|---|---|---|
| `arm64_slam/` | S2 + static tf + `slam_toolbox` | `/slam_toolbox` | `/map`, `/tf` | static identity | ~12 % | 打通，圖可存 |
| `slam_toolbox_static/` | 同上，static 專用 yaml | `/slam_toolbox` | `/map`, `/tf` | static identity | — | 兩張圖 `map_slamtoolbox_static_01/02` |
| `rf2o_only/` | S2 + static + `rf2o` | `rf2o` | `/odom_rf2o` | rf2o 動態 | — | ⚠️ **8~10 m 只量出 1.3 m（低估 7 倍）→ 本環境 rf2o 判死刑** |
| `rf2o/` | + `slam_toolbox` | rf2o + toolbox | `/map`, `/odom_rf2o` | rf2o 動態 | ~19 % | 圖可出，但受 rf2o 精度拖累 |
| `hector_rf2o/` | + `hector_mapping`（純雷射） | `hector_mapping` | `/map`, `/odom_rf2o` | rf2o 動態 | — | 圖可出；rf2o 只記錄不餵 hector |
| `hector_use_rf2o/` | hector 吃 rf2o 初估 | `hector_mapping` | `/map`, `/odom_rf2o` | rf2o 動態 | — | A/B 對照（`use_tf_pose_start_estimate` true/false） |
| `carto/` | + `cartographer` + `occupancy_grid_node` | `cartographer_node` | `/map`（5 cm） | rf2o 或 carto 自發 | — | 通過，**效果上限最高**（pose-graph 全域最佳化） |
| `arm64_slam/rtabmap/` | + `rtabmap` scan-only | `icp_odometry` + rtabmap | `/rtabmap/odom`, `/rtabmap/map` | rtabmap | 34 % / 420 MB | 通過；rgbd/fusion 因無相機未驗 |

### 8.3 `/scan` 這一個 topic，在 demo 環境能撐出哪些功能

| 功能 | 機制 | 本 repo 對應實作 | 證據 |
|---|---|---|---|
| **2D 佔位地圖建構** | 相鄰圈線段做 scan matching | `slam_toolbox`（Karto）、`hector_mapping` | `/map` OccupancyGrid 5 cm/格 |
| **無里程計雷射里程計** | dense scan alignment 累積 | `rf2o` → `/odom_rf2o` 10 Hz | `bags/rosbag2_2026_09_23-09_26_37` |
| **迴環閉合 / 消除累積漂移** | 回頭重訪時做圖對圖最佳化 | `slam_toolbox` `do_loop_closure: true`；`carto` pose-graph | 路線疊加＋回起點才有效 |
| **TF 驅動的座標鏈維護** | 每個 node 靠 TF 把 `/scan` 換算到自己的 frame | `map→odom→base_footprint→laser` | `ros2 run tf2_ros tf2_echo map laser` |
| **地圖存檔** | `map_saver_cli` 訂 `/map` 存 pgm+yaml | `nav2_map_server` | `arm64_slam/map_*.pgm` 18 張 |
| **錄包 → 離線重放 / 調參 / A/B 比較** | `ros2 bag record` | 全鏈皆有 record script | `bags/` 11 個 bag |
| **視覺化除錯** | rviz LaserScan（Decay Time 疊成軌跡） | `rviz/rplidar_ros.rviz`、`hector_rf2o.rviz` | 開 `view_rplidar_s2_launch.py` |
| **品質檢測**（本專案自創用法） | 統計 `inf` 比例 / `intensities` 分布 | `rf2o_only/` 用 bag 離線算路徑長 vs 淨位移 | rf2o 低估 7 倍的定量判定 |
| **3D / 融合建圖預備** | 同一 `/scan` 餵 `rtabmap`（`odom_source=icp`） | `arm64_slam/rtabmap/` scan 模式 | `/rtabmap/map` 0.23.7 已出圖 |

**Bag 的實際數據**（`bags/rosbag2_toolbox_static_2026_09_23-14_42_16`，149.28 s）：

| topic | 型別 | 訊息數 | 推算頻率 |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | 1468 | **9.83 Hz** |
| `/tf` | `tf2_msgs/msg/TFMessage` | 2932 | 19.6 Hz（toolbox `transform_publish_period: 0.05`） |
| `/map` | `nav_msgs/msg/OccupancyGrid` | 29 | 0.19 Hz（`map_update_interval: 5.0`） |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 2 | 一次（`odom→base` + `base→laser`） |

→ 別被「`/map` 沒發幾次」誤導：**静止時 `/map` 不更新是正常的**，走過 `minimum_travel_distance` 才長大。

### 8.4 對 topic 的「直接」用法（不改 SLAM）

```bash
# 純看掃描品質（不跑任何 SLAM）
ros2 topic hz /scan; ros2 topic bw /scan

# 濾掉機器人自己的機身（laser_filters，接到 /scan_filtered）
#   - 角度過濾：把自己腳下的反射切掉
#   - 距離過濾：range_min=0.35 擋掉支架／人抱著雷達的手臂
#   - 遮蔽過濾：sunlight_rejection（S2 強光下雜訊）

# 轉點雲給 3D 工具（rtabmap / PCL / RViz PointCloud2）
ros2 run tf2_ros tf2_static_transform_publisher ...   # 補齊 frame
# 然後自寫 3240 點的 LaserScan → PointCloud2 轉換（順便修 §7.4(a) 的 intensity 問題）

# 錄原始資料留檔（不重跑 SLAM，x86 慢慢調）
ros2 bag record /scan /tf /tf_static /odom_rf2o /map
ros2 bag play rosbag2_xxx --clock          # 重放時 rviz 要 -p use_sim_time:=true
```

---

## 9. Nav2：現況、缺口、可行的下一步

### 9.1 現況（誠實說明）

**本 repo 目前用到 Nav2 的只有 `nav2_map_server` 的 `map_saver_cli`**（存圖），
`sudo apt install ros-jazzy-nav2-map-server`。
完整的 Nav2 導航堆疊（AMCL / costmap_2d / planner / controller / behavior）**尚未在此環境實測**。

原因是物理限制，不是沒裝：本環境是**手持雷達 + 板子，沒有底盤**——沒有 `/cmd_vel` 消費者、沒有輪速、沒有里程計、沒有 `robot_description`。

### 9.2 要把 `/scan` 接到 Nav2，各元件吃什麼

| Nav2 元件 | 需要 `/scan` 嗎 | 還需要什麼 | 本環境狀態 |
|---|---|---|---|
| `map_server` | ✗ | 已存的 pgm+yaml | ✅ 可直接用（`map_s2_0922_carto_01.yaml` 等 18 張） |
| `amcl`（粒子濾波定位） | ✅ | `/map` + TF `map→odom→base_footprint→laser` + `/initialpose` | ⚠️ TF 鏈有（手持時是 static identity），**缺 odom 與底盤** |
| `slam_toolbox` localization 模式 | ✅ | `/map` + `/scan`（2D 場景通常比 AMCL 輕） | ⚠️ 同上 |
| `nav2_costmap_2d` `obstacle_layer` | ✅ **核心使用者** | `observation_sources: scan`、`robot_radius/footprint`、`resolution`、`rolling_window` | ⚠️ 缺 `robot_radius` 與底盤 |
| `nav2_planner`（Smac / NavFn） | 間接（讀 costmap） | costmap + TF 鏈 + 機器人尺寸 | ⚠️ 同上 |
| `nav2_controller`（MPPI / DWB） | 間接（讀 local costmap） | **真 odom**（TF `odom→base`）+ `cmd_vel` 介面 | ❌ 缺底盤 |
| `nav2_behaviors`（spin / backup） | ✅ 靠 local costmap | 底盤 | ❌ |
| `nav2_velocity_smoother` | ✗ | `/cmd_vel` | ❌ |
| `waypoint_follower` | ✗ | 底盤 + 全域 planner | ❌ |
| `nav2_smoother_server` | ✗ | costmap | ⚠️ |

### 9.3 缺什麼才能跑起來（依優先序）

1. **底盤**：差速或全向車 + `cmd_vel` 訂閱者（自寫或用 `ros2_teleop_keyboard` 接真機）
2. **`/odom`**：輪速計（`diff_drive_controller` 會自發）或 IMU+輪速餵 `robot_localization`（EKF）
   - 現況替代品 `rf2o` 在本環境低估 7 倍，**不可直接當 Nav2 odom**
3. **TF 骨架**（Nav2 硬性要求）：
   ```
   map → odom        (amcl 或 slam_toolbox localization 發)
   odom → base_footprint   (輪速／EKF，dynamic)
   base_footprint → base_link
   base_link → laser       (static，z = 實際安裝高度)
   ```
4. **參數檔**：`nav2_params.yaml`（`robot_radius` 或 `footprint`、`resolution: 0.05`、
   `obstacle_layer.observation_sources: scan`、`raytrace_max_range: 12.0`）
5. **起點**：`/map` 由 `map_server` 載入；要 Nav2 認得 `laser` frame，
   需把 `/scan` remap 成 Nav2 期望的名字（`scan` 或在 `obstacle_layer` 改 `topic: /scan`）

### 9.4 不接底盤也能 demo 的部分（成本最低的示範）

即使沒有車，**這些可以立刻用現有 `/scan` + 已存地圖做**：

```bash
# A. 載圖 + 在圖上算一條路徑（planner 有 costmap 就有輸出，cmd_vel 沒有也無妨）
ros2 launch nav2_bringup launch_bringup.launch.py \
  use_sim_time:=false map:=$HOME/map_s2_0922_carto_01.yaml \
  params_file:=$HOME/nav2_params.yaml
# 沒有 base 就把 controller_server 關掉，只留 planner_server

# B. 用 slam_toolbox 做 2D 定位（比 AMCL 輕，對 2D LiDAR 常更穩）
ros2 launch slam_toolbox localization_launch.py \
  map_file_name:=$HOME/map_s2_0922_carto_01.yaml use_sim_time:=false

# C. 純看雷射在已存地圖上的比對（rviz 疊圖，找 scan-to-map 殘差）
#    Fixed Frame=map，疊 Map(/map) + LaserScan(/scan)
```

---

## 10. 已知坑速查（全部本專案實測踩過）

| 症狀 | 原因 | 解法 |
|---|---|---|
| node 起來但沒資料 | baud 用了 115200/256000 | S2 固定 **1000000** |
| `ls launch \| grep s2` 沒東西 | 用了 apt 版 2.1.0 | source 編 `-b ros2` |
| 編譯報需要 catkin | 抓到 tags `2.1.x` tarball（ROS1） | `git clone -b ros2` |
| `Permission denied /dev/ttyUSB0` | 使用者不在 `dialout` | `sudo usermod -aG dialout $USER`（重登）或臨時 `sudo chmod 666` |
| `Error, cannot bind to the specified serial port` | 已被別的行程佔用（rf2o/舊 node/上次崩潰） | `ps -eo pid,args \| grep "[r]plidar_node"` 取 PID 再 kill |
| 串口突然斷線 | `ModemManager` 搶 ttyUSB | `sudo systemctl disable --now ModemManager` |
| 轉速慢、點數亂跳 | 供電不足（ARM 板載 USB 弱） | 電源 hub / 外供 |
| 首次掃描拿到 descriptor 失敗 | 前次未 stop，雷達還在轉 | 先送 `STOP (A5 25)`（`s2_info_check.py` 已內建） |
| 角度跳動、圖糊 | `angle_compensate` 在小板吃 CPU | `angle_compensate:=false` |
| `Could not get transform` | TF 鏈斷 | `ros2 run tf2_ros tf2_echo map laser` 逐段驗 |
| `reparenting / multiple authority (odom→base)` | rf2o 和 static 兩者都發 | 只留一個發布者 |
| toolbox 不訂 `/scan`、不發 `/map` | lifecycle 沒 activate | 用 `online_async_launch.py`（自動 activate）或手動 `lifecycle set /slam_toolbox configure` + `activate` |
| `ros2 topic list` 看不到東西 | daemon discovery 快取 | `ros2 daemon stop` |
| NB 看不到板上的 `/scan` | 板上 node 根本沒跑 / multicast 被擋 / 混用 discovery server | `ps` 先確認 node；`ros2 multicast receive\|send` 測；**兩邊都別設 `ROS_DISCOVERY_SERVER`** |
| `/map` 長不大 | 拿了 rf2o 版的 yaml（`minimum_travel_distance: 0.5`）配 static odom | static 版必須用 `slam_toolbox_static.yaml`（三個 minimum 全 0） |
| `rf2o Waiting for laser_scans` 永不停 | `init_pose_from_topic` 沒留空 | 必須用 `--params-file` 傳 `init_pose_from_topic: ""`（CLI `-p` 傳空字串會炸 rcl 解析） |
| carto timestamp FATAL | rf2o odom stamp 與 scan 同刻 | `odometry_sampling_ratio: 0.5` 繞過（治本是修 rf2o stamp） |
| carto 圖糊、bag `/tf` 有 `odom→odom` | lua `published_frame` 誤設成 `odom` | 必須是 `base_footprint`（carto 發 `odom→tracking`） |
| pip 裝不動 | Ubuntu 24.04 PEP 668 | 一律 venv |

---

## 附錄 A：本 repo 檔案索引

```
/home/user/Downloads/komugi/lidar/
├── README.md                     總計畫 + 安裝 + 瓶頸 + 實測記錄
├── upstream/VERSION.md           ★ SDK/driver 來源、branch、commit
├── upstream/rplidar_ros/         ★ SDK 在 sdk/、node 在 src/、launch/rplidar_s2_launch.py
├── x86_simple/
│   ├── s2_info_check.py          GET_INFO + GET_HEALTH，不起轉
│   ├── s2_scan.py                pyrplidarsdk 取圈存 csv
│   ├── s2_laserscan_node.py      純 Python ROS2 node（720 bin）
│   ├── requirements.txt          pyrplidarsdk + pyserial
│   └── scan_sample.csv           實測 3 圈 7662 點
├── x86_ros2/
│   ├── install_driver.sh         ★ 官方安裝流程（source 編 S2）
│   ├── s2_ros_check.sh           headless 驗證
│   └── scan_once_sample.yaml     ★ 實測 /scan 樣本
├── arm64_simple/                 同 x86_simple（純 Python）
├── arm64_ros2/
│   ├── cross_build_rplidar.sh    ★ x86 → aarch64 cross 編譯
│   └── toolchain-aarch64-ros2.cmake
├── arm64_slam/                   板上全流程 + 18 張實測地圖 + slam_s2_headless.launch.py
│   └── rtabmap/                  RTAB-Map scan/rgbd/fusion 三模式
├── rf2o/ , rf2o_only/            rf2o 里程計（並跑 / 單獨驗）
├── hector_rf2o/ , hector_use_rf2o/  hector 建圖（並跑 / 吃初估）
├── slam_toolbox_rf2o/ , slam_toolbox_static/  toolbox 兩種 odom 方案
├── carto/                        Cartographer + rf2o（效果上限最高）
├── bags/                         11 個實測 rosbag（mcap）
└── report/                       ★ 本報告
```

## 附錄 B：指令速查

```bash
# ── 一次性安裝（x86）────────────────────────────────
bash ~/Downloads/komugi/lidar/x86_ros2/install_driver.sh

# ── 起雷達 ──────────────────────────────────────────
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch rplidar_ros rplidar_s2_launch.py                    # headless
ros2 launch rplidar_ros view_rplidar_s2_launch.py               # + rviz
ros2 launch rplidar_ros rplidar_s2_launch.py angle_compensate:=false

# ── 檢查 ────────────────────────────────────────────
ros2 topic list | grep -E 'scan|motor'
ros2 topic hz /scan            # ~10.0 Hz
ros2 topic bw /scan            # ~260 kB/s
ros2 topic echo /scan --once | head -n 14
ros2 topic info /scan -v       # 看 QoS
ros2 service call /stop_motor std_srvs/srv/Empty

# ── 參數 ────────────────────────────────────────────
ros2 param dump /rplidar_node
ros2 param set /rplidar_node angle_compensate false   # 需重啟才生效（開機讀一次）

# ── 一鍵建圖（手持）────────────────────────────────
# arm64_slam：static 假 odom + slam_toolbox
ros2 launch ~/slam_s2_headless.launch.py
# rf2o：真雷射里程計 + toolbox
~/slam_s2_rf2o_start.sh
# carto：pose-graph 最佳化
~/carto_s2_start.sh
# 驗 + 存圖
ros2 lifecycle get /slam_toolbox                 # active [3]
ros2 run tf2_ros tf2_echo map laser
ros2 run nav2_map_server map_saver_cli -f ~/map_s2_01
ros2 bag record /scan /tf /tf_static /map

# ── 排錯 ────────────────────────────────────────────
ps -eo pid,args | grep "[r]plidar_node"
ros2 daemon stop && ros2 topic list
ros2 run tf2_ros tf2_echo base_footprint laser
```

## 附錄 C：本文關鍵數據一覽

| 數據 | 值 | 來源 |
|---|---|---|
| SDK 版本 | 2.1.0（68 檔，headers 2,054 行 / src 11,480 行） | `sdk/include/sl_lidar.h` |
| package 版本 / commit | 2.1.4 / `24cc9b6`（2025-04-27） | `upstream/VERSION.md` |
| 裝置資訊 | `model=113 fw=1.2 hw=18` | `s2_info_check.py` **[實測]** |
| scan mode / 取樣率 | `DenseBoost, 32 Khz, max 30.0 m, 10.0 Hz` | node 開機 log **[實測]** |
| `/scan` 頻率 | 9.8~10.0 Hz | `ros2 topic hz`、bag metadata **[實測]** |
| 物理一圈點數 | ~3200（32000 次/秒 ÷ 10 圈/秒，期望值） | §5.3.1 **[推導]** |
| `/scan` bin 數 | 3240（`angle_compensate=true`，固定格）／1900~3500（false，原始切片） | §5.3.2–5.3.3 **[實測]** |
| 3240 的性質 | 複製填格（zero-order hold），非內插；有效角解析度仍 ~0.11° | §5.3.2／§5.3.4 |
| 板端整圈實測 | N=3240，有效 74.0%／inf 16.8%／盲區 9.2%，median 0.256 m，hz 10.0 | §5.3.5 **[實測]** |
| 角度跨度 | −179.0° ~ +180.0°（359°），0.1108°/bin | `scan_once_sample.yaml` **[實測]** |
| 量程 | `range_min` 0.15 m、`range_max` 30.0 m | 同上 **[實測]** |
| `scan_time` | ≈ 0.0993 s（= 等一圈的阻塞時間） | 同上 **[實測]** |
| `time_increment` | 3.066e-5 s/bin | 同上 **[實測]** |
| 單則訊息大小 / 頻寬 | 25.4 KiB / 10 Hz ≈ 260 kB/s | §7.6 **[推導]** |
| Serial 使用率 | 32k 樣本 × 2 B ≈ 64 kB/s / 100 kB/s ≈ 64 % | §7.6 **[推導]** |
| CPU（rplidar node） | ~8~11 %（x86 與 ARM 板同量級） | README §6 **[實測]** |
| rf2o 精度 | 8~10 m 路徑只量出 1.3 m（**低估 7 倍**） | `rf2o_only/README.md` **[實測]** |

---

### 參考

- 上游：`https://github.com/Slamtec/rplidar_ros`（`-b ros2`，v2.1.4）
- 產品頁：Slamtec RPLIDAR S2（ToF / 32k 次每秒 / 30 m / 80k lux / IP65）
- ROS 2 API：`sensor_msgs/msg/LaserScan` 定義於 `ros2/common_interfaces`（Jazzy）
- 本專案實測記錄：`../README.md`、`../arm64_slam/README.md` 及各子目錄 README、`../bags/*.yaml` metadata
