# RPLIDAR S2 (serial) 調查與驗證 — x86 / ARM64, Ubuntu 24.04 + ROS2 Jazzy

實機：S2 serial + CP2102N USB 轉接 (`10c4:ea60` → `/dev/ttyUSB0`)，本機 x86_64 已實測全通。
ARM 板未在手邊，`arm64_simple/` 與 x86 同碼 (純 Python)，wheel 與語法已驗，板上照跑即可。

## 檔案佈局 (都在 `~/Downloads/komugi/lidar/`)

```text
lidar/
├── README.md                  本檔 (計畫 + 安裝 + 瓶頸 + 實測記錄)
├── x86_simple/                x86 簡單驗證版 (免 ROS)
│   ├── s2_info_check.py       健康檢查：GET_INFO + GET_HEALTH，不起轉，零額外依賴
│   ├── s2_scan.py             第一圈掃描：pyrplidarsdk，會轉，存 csv
│   ├── requirements.txt       pyrplidarsdk + pyserial
│   └── scan_sample.csv        實測 3 圈樣本 (7662 點)
├── x86_ros2/                  x86 ROS2 版
│   ├── install_driver.sh      source 編 rplidar_ros (S2 必須，見 §2)
│   ├── s2_ros_check.sh        headless node + /scan once + hz 驗證
│   └── scan_once_sample.yaml  實測 /scan 樣本
└── arm64_simple/              ARM64 簡單驗證版 (與 x86_simple 同碼)
    ├── s2_info_check.py
    ├── s2_scan.py
    ├── requirements.txt
    └── README.md              板上步驟 + ARM 差異
└── arm64_ros2/                ARM64 ROS2 cross 版 (x86 host 上編)
    ├── cross_build_rplidar.sh toolchain-aarch64-ros2.cmake  編出 arm64 rplidar_node
└── upstream/                  上游快照 (免重抓，見 VERSION.md)
    └── rplidar_ros/           Slamtec/rplidar_ros ros2 branch (node+sdk+launch 全套)
```

## 0. `upstream/rplidar_ros` 實際檔案分佈 (ros2 branch, 共 109 檔，不含 .git)

```text
rplidar_ros/
├── src/rplidar_node.cpp      node 本體 (598 行)：開串口 → 設 scan_mode → 發 /scan(LaserScan)
├── src/rplidar_client.cpp    測試 client
├── include/rplidar_node.hpp  node class 定義；visibility.h (動態庫導出宏)
├── launch/                   每型號一對：rplidar_<m>_launch.py (headless) + view_...(＋rviz)
│                             S2 用 rplidar_s2_launch.py / view_rplidar_s2_launch.py
├── rviz/rplidar_ros.rviz     view_ launch 載入的顯示配置
├── scripts/                  rplidar.rules (10c4:ea60→/dev/rplidar, 0777) + 建/刪 udev 腳本
├── sdk/                      內嵌 driver source (CMake glob 全編進 rplidar_node，免另裝 SDK)
│   ├── include/              公開標頭：sl_lidar_driver.h / sl_lidar_protocol.h / rplidar.h 等 12 檔
│   └── src/
│       ├── sl_lidar_driver.cpp / rplidar_driver.cpp   指令、掃描、express/DenseBoost 核心
│       ├── sl_lidarprotocol_codec.cpp / sl_async_transceiver.cpp / sl_crc.cpp  協議編解碼
│       ├── sl_serial_channel.cpp / sl_tcp_channel.cpp / sl_udp_channel.cpp  三種通道 (S2 用 serial)
│       ├── dataunpacker/     封包解包：capsules / hqnode / normalnode 三種 handler
│       └── arch/{linux,macOS,win32}/  各平台串口/socket/thread 實作 (Linux 用 net_serial.cpp)
├── package.xml / CMakeLists.txt  ament_cmake；相依 rclcpp/sensor_msgs/std_srvs/rclcpp_components
└── debian/udev  打包用 udev 目錄佔位
```

## 1. 計畫 (已執行完)


1. 識別：`lsusb` 見 `10c4:ea60 CP2102N` + `/dev/ttyUSB0` → serial 版確定。
2. 健康檢查：pyserial 直送 `A5 50 / A5 52`，確認 1Mbaud 可通、HEALTH OK。
3. 簡單掃描：`pyrplidarsdk` 取 3 圈點雲，確認會轉、有點。
4. ROS2：source 編 `rplidar_ros` (ros2 branch)，起 `rplidar_s2_launch.py`，確認 `/scan` 10Hz。
5. ARM64：同碼移植，wheel/py_compile 已驗，待板上重放 §4 指令。

## 2. 需要額外安裝 (以本機 Ubuntu 24.04 + Jazzy 為準)

| 用途 | 指令 | 本次實測狀態 |
|---|---|---|
| 健康檢查 | 系統已有 `python3-serial` (3.5)，**免裝** | 直接跑通 |
| 掃描 (x86/ARM) | `python3 -m venv .venv && .venv/bin/pip install -r x86_simple/requirements.txt` | venv + `pyrplidarsdk 0.1.2` 實測 OK；注意 Ubuntu 24.04 的 PEP 668：**不可裸 `pip install`**，要用 venv 或 `--break-system-packages` |
| 掃描 (ARM) | 板上同上；`pyrplidarsdk` 有 `manylinux aarch64` wheel (已用 `--platform manylinux_2_27_aarch64` 下載確認 0.1.1)，免編譯；需 Python ≥3.10 (24.04 預設 3.12) | wheel 存在已驗，板上待跑 |
| ROS2 driver | **apt 不夠**：`ros-jazzy-rplidar-ros=2.1.0` 經 `dpkg-deb -c` 確認**無 S2 launch** (只有 A1/A3/S1) → 跑 `x86_ros2/install_driver.sh` 從 source 編 | `/tmp/opencode/lidar_ws` 編譯通過 (~11s，僅 warnings)，S2 launch 預設即 `baud 1000000 + DenseBoost` |
| 串口權限 | 本機使用者已在 `dialout`，**免裝**；他機：`sudo usermod -aG dialout $USER` 或跑包內 `create_udev_rules.sh` | 已確認 |
| CP210x 驅動 | noble kernel 內建 `cp210x`，**免裝** | `driver -> .../cp210x` 已確認 |

> 本次驗證未用 sudo 安裝任何 apt 包 (環境不便輸密碼)：ROS2 用 source 編繞過，Python 用 venv 繞過 PEP 668。

## 3. x86 執行方式與實測結果

```bash
# 健康檢查 (不起轉)
python3 x86_simple/s2_info_check.py --port /dev/ttyUSB0
# 掃描 (會轉)
python3 -m venv .venv && .venv/bin/pip install -r x86_simple/requirements.txt
.venv/bin/python x86_simple/s2_scan.py --port /dev/ttyUSB0 --scans 5 --out scan.csv
# ROS2 (另開 WS，已編好可直接用；正式請跑 install_driver.sh 建 ~/ros2_ws)
source /opt/ros/jazzy/setup.bash
./x86_ros2/s2_ros_check.sh /dev/ttyUSB0 12
```

實測 (2026-09-10, 本機)：

* `s2_info_check.py` → `INFO OK: model=113 fw=1.2 hw=18` + `HEALTH OK: status=0 error_code=0` + `RESULT: PASS`
* `s2_scan.py --scans 3` → `device_info/model=113` 一致，每圈 1900~3500 點、共 7662 點，range 0.17~4.35m (室內)，見 `scan_sample.csv`
* ROS2 `rplidar_s2_launch.py` → node 印 `current scan mode: DenseBoost, sample rate: 32 Khz, max_distance: 30.0 m, scan frequency: 10.0 Hz`；`/scan` (`frame laser`, range 0.15~30m) `hz` 穩定 **~10.0Hz**，見 `scan_once_sample.yaml`
* rviz 可看：`ros2 launch rplidar_ros view_rplidar_s2_launch.py`（= 上面 node + rviz2 帶 `rviz/rplidar_ros.rviz`），本機 `DISPLAY=:0` 實測 rviz2 正常啟起、`/scan` 同步 10Hz；不要畫面時用 `rplidar_s2_launch.py` 即可
* 修正記錄：初版 `s2_scan.py` 漏 `connect()` (報 `Device is not connected`)，已補上；`s2_ros_check.sh` 改用 headless 的 `rplidar_s2_launch.py` (view_ 版要開 rviz，SSH 會卡)
* `s2_info_check.py` 開頭會先送 `STOP (A5 25)`：前一次掃描若未正常 stop，雷達會續轉，此時直接問 INFO 會拿到掃描位元組而非 descriptor（實測踩過，已修）。

## 4. ARM64 執行方式 (板上，Ubuntu 24.04 + Jazzy)

```bash
ls -l /dev/ttyUSB*; lsusb -d 10c4:ea60     # 先看到裝置
python3 arm64_simple/s2_info_check.py --port /dev/ttyUSB0
python3 -m venv .venv && .venv/bin/pip install -r arm64_simple/requirements.txt
.venv/bin/python arm64_simple/s2_scan.py --port /dev/ttyUSB0 --scans 5 --out scan.csv
```

詳見 `arm64_simple/README.md`。本機已驗：兩支 `.py` 經 `py_compile`，`requirements` 版號與 aarch64 wheel 存在性。

### 板上起 node，NB 開 rviz 看（跨機）

```bash
# 板上 (headless)：只起 node
source ~/lidar_ws_arm64/install/setup.bash  # 或 ~/ros2_ws/install/setup.bash
ros2 launch rplidar_ros rplidar_s2_launch.py
# NB (x86)：只開 rviz，加 LaserScan display 訂 /scan，Fixed Frame 選 laser
rviz2
```

前提（缺一即看不到）：
1. 兩邊同網段、multicast 通（多數家用 AP 可；公司網/VPN 常擋——見下）。
2. `ROS_DOMAIN_ID` 一致（預設都是 0，不動最省事；`echo $ROS_DOMAIN_ID` 兩邊對）。
3. 同 Jazzy、同 RMW（預設 Fast DDS，兩邊都不改）。
4. 防火牆放行：`sudo ufw allow in on <網卡>` 或先 `sudo ufw disable` 試。
5. NB 驗：`ros2 topic list | grep scan`、`ros2 topic hz /scan` 看到 ~10Hz 即通。

若 multicast 被擋（跨網段/特定 WiFi）：改用 Discovery Server——板上起
`fastdds discovery -i 0 -p 11811`，兩邊 export
`ROS_DISCOVERY_SERVER="<server-ip>:11811"`；或換 CycloneDDS 配 `CYCLONEXML` 單播 peers。

## 6. Python node vs C++ node 效能實測 (本機 x86, S2 DenseBoost 10Hz)

| | Python (`s2_laserscan_node.py`) | C++ (`rplidar_node`) |
|---|---|---|
| `/scan` 頻率 | 10.0Hz，std ~1.0ms | 10.0Hz，std ~0.9ms |
| CPU (top, 5 採樣) | ~9~11% | ~8~11% (啟動瞬間 20%) |
| 資料路徑 | 同一套 C++ SDK (nanobind 包裝)，解包零差異；Python 只多一層 list 轉換 + 720-bin裝箱迴圈 (~ms 級) | 全 C++，含 `angle_compensate` |
| 結論 | **這顆雷達的量級下無感差異**：瓶頸是 32k pts/s 的 SDK 解包 + DDS，主導 CPU 的是同一份 code。Python 版少了 `angle_compensate`，要的話自己加。大量雷達/高頻率或 ARM 小板吃緊時再考慮 C++；開發速度優先就 Python。 | 長跑穩定、jitter 略小；要接 lifecycle/composition 也是它。 |

> 注意：Python timer 設 0.1s 發佈，若回調偶爾超時會有抖動；實測本機 std 1.0ms vs 0.9ms，同一量級。

## 7. 可能的瓶頸

1. **baud 用錯**：S2 serial 固定 `1000000`；套 A1/A2/S1 範例的 115200/256000 會連上無數據。本包三處預設皆已是 1000000。
2. **apt 版無 S2**：`ros-jazzy-rplidar-ros 2.1.0` 無 S2 launch，S2 必須 source 編 (`install_driver.sh` 已處理)。
3. **供電**：啟動 1.5A / 工作 0.5~0.6A；ARM 板載 USB 更弱，務必帶電源 hub 或外供，否則轉速上不去、點數亂跳。
4. **串口被搶**：`ModemManager` active (本機 `mmcli -L` 尚無 modem，可先不動)；若突然斷線：`sudo systemctl disable --now ModemManager`。
5. **PEP 668**：24.04 裸 `pip install` 會被擋，一律 venv。
6. **頻寬/線材**：32k pts/s @1Mbaud 接近上限，用短線直插；`angle_compensate=true` 在小板吃 CPU，會卡再關。
7. **舊 Python 庫**：`pip install rplidar` (rplidar.py) 無 DenseBoost，只能降級玩，本包不用它。
