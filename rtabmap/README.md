# D456 + RTAB-Map 首板 bringup（`komugi@192.168.50.200`，NT98635 arm64）

板：Ubuntu 24.04 noble / ROS2 Jazzy / 4核A73 / 2.8GB RAM。
相機：`8086:0b5c` D456，ASIC serial `412243060290`，FW `5.17.3.10`，
`rs-enumerate-devices` 回報 IMU `BMI085`，Advanced Mode YES。

## 1. 已安裝（apt，board 上）

```bash
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-jazzy-librealsense2 ros-jazzy-realsense2-camera \
  ros-jazzy-rtabmap-ros
```

實裝版本：`librealsense2 2.58.4`，`realsense2-camera 4.58.4`，
`rtabmap-ros 0.23.13`（含 slam/odom/viz/launch 全套）。
工具在 `/opt/ros/jazzy/bin/rs-*`，跑前必須先 `source /opt/ros/jazzy/setup.bash`
（否則 `librealsense2.so.2.58` 找不到）。

## 2. USB 權限（持久化已寫）

ROS 版 librealsense 不帶 udev rules，原生 `/dev/bus/usb/*` 是 `644 root`
→ `lsusb -v` 報 `Couldn't open device`。已寫
`/etc/udev/rules.d/99-realsense-libusb.rules`：

```text
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", MODE="0666", GROUP="video"
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTR{power/control}="on"
KERNEL=="iio*", MODE="0664", GROUP="video"
```

（第二行禁 USB autosuspend，防 idle 斷流；改規則後只 `reload-rules`，
不要在 node 跑時下 `trigger --action=add`。）

```bash
sudo udevadm control --reload-rules
# 新插入的裝置自動 666；已插著的當下那顆要手動補一次：
sudo chmod 666 /dev/bus/usb/<bus>/<dev>
```

注意：`komugi` 已在 `video` 群，`/dev/video*` 可開；
`/dev/iio:device0/1`（IMU）仍是 root 600，要 `sudo` 讀。

## 3. 相機體質（`rs-enumerate-devices` 實測）

* 當時鏈路是 **USB2（480M，`Usb Type Descriptor 2.1`）**，Bus 002。
  此鏈路下可用：Depth `640x480x30` / `848x480x10` / `1280x720x5`；
  Color `640x480x30` / `1280x800x8`。**強烈建議換到 USB3 空口（Bus 003/005，5000M）再跑圖。**
* IMU 走 `hid-sensor-hub` → `iio:device0 accel_3d` / `iio:device1 gyro_3d`，
  有即時值（accel Z 軸 ~1g，gyro 會動）。`/dev/input/event0` 只是 UVC button，不是 IMU。

## 4. 出圖指令（USB2 保守參數）

**2026-10-12 17:35 實測出圖成功**（`Bus 002 Dev 021`，USB2 高速，連續穩定數分鐘）。
當下 node 仍在跑（log `/tmp/rs_launch3.log`，殺掉用 `pkill -f "[r]ealsense2_camera"`）：

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30 \
  enable_gyro:=false enable_accel:=false \
  align_depth.enable:=true
# 另開終端驗：
ros2 topic list | grep camera
timeout 15 ros2 topic hz /camera/camera/color/image_raw
```

實測：`/camera/camera/color/image_raw` 640x480 rgb8 @**15Hz**（std 0.09ms），
`/camera/camera/depth/image_rect_raw` 640x480 16UC1 @**15Hz**，
`/camera/camera/aligned_depth_to_color/image_raw` 有發；
另有 `depth/camera_info`、`color/camera_info`、`extrinsics/*`、`tf_static`。
注意：要 30 卻只出 15——USB2 頻寬下 node 自動降檔，先求穩。

IMU 說明：`enable_gyro/accel:=true` 曾起不來——此 kernel 無 `hidraw`，
librealsense 改走 IIO sysfs（`scan_elements/*_en`、`trigger/*`），
那些檔是 root 600，komugi 寫不進去。
**已解**：udev 加 `ACTION=="add", SUBSYSTEM=="iio", RUN+="/bin/chmod -R a+rw /sys%p"`，
`trigger --subsystem-match=iio --action=add` 重放即生效（只動 iio，
不断 USB，安全），新插入也自動生效。**2026-10-12 18:3x 實測 IMU 正常**：
`/camera/camera/imu` @**200.4Hz**（std 0.17ms），gyro 靜止毫弧度級噪聲，
accel 某軸 ~9.74 m/s²（≈1g，安裝朝向所致）；另有 `gyro/sample`、`accel/sample`、
`depth_to_accel/gyro` extrinsics。rtabmap 要做 VIO 時把
`imu_topic:=/camera/camera/imu` 餵給 `rgbd_odometry` 即可（`unite_imu_method:=2`）。

**2026-10-13 靜態檢驗通過**（`/tmp/imu_check.py`，600 樣本）：
accel 均值 (+0.10,−9.75,−0.37)、重力模 9.755（誤差 <1%），
gyro 零偏 <3mrad/s、噪聲 ~3mrad/s，dt 均值 5.56ms（標稱 5ms，偶發 319ms 調度毛刺），
frame `camera_imu_optical_frame`；另與 kernel IIO 獨立路徑交叉對過量級一致。
附帶抓到兩個坑：(1) 自寫訂閱者必須用 sensor-data QoS，否則
`incompatible QoS` 零封包；(2) 舊節點跑 15.5h 後 accel 流凍結（200 樣本僅 1 種值，
節點死時卡 `Dl` 在 kernel IIO 路徑），重起節點即恢復（87/200 distinct，
accel@100Hz 本來就會每值重複一次）。accel 再凍就重起節點。
方向軸正負待動態翻轉驗（把重力甩到別軸看跟不跟）。
**2026-10-13 動態驗通過**：翻轉 90° 後重力從 Y 軸 (-9.75) 換到 X 軸 (-9.5)，
模 ~1g，軸映射正確，可餵 VIO（X 軸量值小約 1.5%，係出廠 IMU 內參缺失用預設值，
VIO 可容忍，方向對最重要）。

## 5. 踩雷記錄（2026-10-12）

1. `udevadm trigger --action=add` 趁 node 在跑時下 → 裝置重列舉，
   node 狂刷 `backend-v4l2.cpp xioctl(VIDIOC_QBUF) ... No such device`。
   教訓：改 udev 後**先殺 node 再 trigger**；`pkill -f realsense` 會殺掉自己的 ssh，
   要用 `pkill -f "[r]ealsense2_camera"`。
2. 接著 `dmesg`：`usb 3-1 disconnect (dev 38)` → 重列 `dev 39` SuperSpeed →
   `GET_CUR -32` 連發 → `usb3-port1: Cannot enable. Maybe the USB cable is bad?` →
   徹底掉線。`reboot` + 手動重開後仍未列舉，`lsusb` 無 `8086`，`/dev/video*` 全無。
3. 结论：軟體側全就緒，**卡在實體層**。請檢查：D456 線有無鬆脫、
   換短線直插 **USB3 口**（避開 hub），該口供電要穩（D456 標 720mA）。

補充（17:03 複查）：`dmesg` 從開機起每隔數秒重複
`usb usb3-port1: Cannot enable. Maybe the USB cable is bad?`，
代表線有插在 USB3-port1 上但 link training 一直失敗；
`ps` 無吃 CPU 的 userspace 行程（load 高是 D-state 的 USB 重試造成，會自己退）。
請：整條拔掉等 10 秒 → 換短的 USB3 線 → 直插另一個 USB3 口（Bus 005），
再叫我驗 `lsusb -d 8086:0b5c`。

補充（17:05 複查）：重插後裝置曾短暫回來（`Bus 003 Dev 010`，5000M，
`/dev/video0-5` 全長出來），但 16 秒後又掉線，`dmesg` 回到
`Cannot enable` 循環。反觀之前在 USB2（480M）上它穩了十幾分鐘
（含 `rs-enumerate` 成功）。研判：這條線/這個口的 USB3 SuperSpeed
線路不良（USB2 線路是好的）。請依序試：(a) 另一個 USB3 實體口（Bus 005）；
(b) 換一條短的 USB3 線 / 加供電 hub；(c) 保底：插 USB2 口，
`640x480x30` 在 USB2 下仍可用，先求出圖再求頻寬。
（同效做法：換一條**只有 USB2 的線**直插同一個孔，強制走 480M；
之前 USB2 模式穩了十幾分鐘。）另外 D456 端是 USB-C，
可把線拔掉等 30 秒（放電）、翻面重插，檢查接頭有無入塵。

## 7. 強制 USB2（hub unbind 法，實測有效）

同一個孔一下談 USB2（穩）一下談 USB3（16 秒掉），解法是把 SS hub 驅動解綁，
裝置就只能走 HS（USB2）列舉。Bus 003（5000M）是 SS，Bus 002（480M）是 HS，
同一個 xhci 控制器（`2f0970000.u3hst`）的兩個邏輯 hub：

```bash
# 確認：三個 root-hub 介面都在 hub driver 下
ls /sys/bus/usb/drivers/hub/ | grep -E "^[0-9]"
# 解綁 SS hub（bus 3），可逆，SSH 不受影響（wifi 在 bus 4）
echo komugi | sudo -S sh -c 'echo 3-0:1.0 > /sys/bus/usb/drivers/hub/unbind'
# 驗：3-0:1.0 的 driver symlink 消失，且 dmesg 不再增加 Cannot enable
ls -la /sys/bus/usb/devices/3-0:1.0/driver  # 應不存在
# 恢復（需要時）：echo 3-0:1.0 | sudo tee /sys/bus/usb/drivers/hub/bind
# 注意：重開機後 bind 恢復原狀，需重下；要持久化再寫 systemd oneshot。
```

實測（2026-10-12 18:1x）：解綁後 `Cannot enable` 風暴停止（最後一筆停在 178s）。
但當下相機仍未在 HS 上列舉——SS 反覆跳線可能已把裝置端掛住，需**拔插一次**
（斷電重置相機），之後它只能談上 USB2。另注意中間板子不明原因重開過一次
（`up` 歸零；第二次解綁沒重開，應與 unbind 無關，疑似看門狗或人為）。

補充（18:2x）：做過 bind 還原對照——綁回去後風暴**立刻恢復**（4 秒一次），
證明 xhci 控制器本身健康、裝置電氣上有反應，只是 SS training 永遠失敗；
再解綁後 30 秒零新訊息，風暴確定可控。結論：線/相機的 SS 通路壞，
等效強制 USB2。已保持 `3-0:1.0` 解綁狀態，重插後相機只能走 Bus 002（HS）。

補充（18:3x）：重插後 HS 依然無列舉；bind 還原→風暴秒恢復→再 unbind 壓住。
 pattern 固定為「SS 有反應、HS 完全靜默」，疑 D+/D- 線路或裝置端 USB PHY 卡住。
遠端軟體手段已用盡，轉硬體隔離：(1) 換線（最可能，充電線常缺 SS 對）；
(2) D456 插別台電腦（RealSense Viewer）驗相機死活，同條線也交叉測；
(3) 供電 hub。板端保持 SS 解綁靜置，load 會自己退。

## 8. 相機回來後下一步（rtabmap 接線）

```bash
lsusb -d 8086:0b5c   # 先看到裝置（應在 Bus 002，480M）
lsusb -t             # 應為 480M（已被強制 USB2）
source /opt/ros/jazzy/setup.bash && rs-enumerate-devices | head -20
# 相機節點（IMU 全開；log /tmp/rs_launch5.log；殺：pkill -f "[r]ealsense2_camera"）：
ros2 launch realsense2_camera rs_launch.py \
  depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30 \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2 \
  align_depth.enable:=true
# rtabmap（log /tmp/rtabmap2.log；殺：pkill -f "[r]tabmap"）：
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  frame_id:=camera_link approx_sync:=true rgbd_odometry:=true \
  rtabmap_viz:=false rviz:=false rtabmap_args:=--delete_db_on_start
```

注意：`rtabmap_viz` 是底線不是 `rtabmapviz`（寫錯會靜默忽略，
headless 上起 viz 只會 Qt crash）。`frame_id:=camera_link` 是官方
camera-only 寫法。`--delete_db_on_start` 每次重建，db 在 `~/.ros/rtabmap.db`。
**2026-10-12 18:37 實測建圖管線全通**：`/rtabmap/odom` @6.2Hz，
`/rtabmap/info` ref_id 持續增加（112+），db 32MB 成長中。
板上 rgbd_odometry 吃 ~70% CPU（4 核夠，6Hz odom 可接受）；
rgb-depth 固定差一幀（66ms）的 approx_sync 警告無害。
建圖要**動相機**：靜止只會堆同一定位節點，需手持慢速平移/環視 1-2 分鐘
才會有軌跡、loop closure 與可用地圖。

**血淚教訓（2026-10-13）：不要讓 rtabmap 空跑過夜。**
放著 15.5h（靜態無人動）db 從 32MB 膨脹到 **13.7GB**（`~/.ros/rtabmap.db`），
把小板拖到 echo 都超時。已殺掉清掉（磁碟回 18%）。以後：只在要建圖時起，
建完先備份 db 就停；或設 `Mem/ImageKept:=false`、`Db/SqlInMemory:=true` 限用量。

**CPU/記憶體實測（2026-10-13，靜態場景，640x480）**：皆為單核百分比（板子 4 核），
三次取樣穩定：`realsense2_camera_node` ~20% / ~110MB，
`rgbd_odometry` ~80% / ~185MB（大頭：特徵提取+匹配），
`rtabmap` 本體 ~27% / ~260MB；合計 **~1.3 核 + ~550MB**，2.8GB 夠用。
`odom` 輸出約 6Hz（CPU 瓶頸吃掉幀）。動起來做 loop closure 時 rtabmap 會有瞬間尖峰。
另注意 `uptime` 的 load 39 是 USB 風暴期的 D-state 歷史殘留假象，實際只吃 1.3 核；
db 靜態約 15~18MB/min 成長，測完記得停+刪。

## 9. 錄 bag（2026-10-13，USB2 線重起後）

```bash
mkdir -p ~/bags && cd ~/bags
ros2 bag record -o d456_rgbd_20261013 \
  /camera/camera/color/image_raw /camera/camera/color/camera_info \
  /camera/camera/aligned_depth_to_color/image_raw \
  /camera/camera/aligned_depth_to_color/camera_info \
  /camera/camera/imu /tf /tf_static /rtabmap/odom
# 停：pkill -f "[r]os2 bag record"；驗：ros2 bag info <dir>；看大小：du -sh
```

實測：8 個 topic 全訂上，recorder 吃 ~23% CPU，寫入約 20~25MB/s
（640x480 rgb+depth @15Hz，1 分鐘 1.4GB）。
錄的同時 rtabmap 照跑（建圖+錄包並行）。錄 3 分鐘約 4~5GB，`~/bags` 在 eMMC，
46G 可用。錄完或播包重建圖都用這包（`ros2 bag play` + 同一組 rtabmap 參數）。
注意：錄包只存數據，要長地圖仍須**邊錄邊慢速移動相機**。

**2026-10-13 首次建圖+錄包成果**（`~/bags/d456_rgbd_20261013/`，手持走約 3.4 分鐘）：
`d456_rgbd_20261013_0.mcap` 3.9GB / 206s / 51575 則——color/depth
各 ~2710 幀（~13Hz）、imu 36274 則（~176Hz）、odom 466 則（~2.3Hz，
bag+建圖並行負載下掉到 2~4Hz，堪用但下次可考慮降解析度）、tf 全。
`rtabmap_online.db` 89MB（線上建圖快照）。`map_2d.pgm/yaml`：
`ros2 run nav2_map_server map_saver_cli -f map_2d --ros-args -r map:=/rtabmap/map`
由 `/rtabmap/map` 存下，153x151 @0.05m（7.6x7.5m），格式同 lidar 的 pgm/yaml 可直接比對。
錄完 rtabmap 已停（防 db 膨脹），相機節點留著。

## 10. 本地備份（2026-10-13）

板上 `~/bags/d456_rgbd_20261013/` 的小檔已拉回本地
`lidar/rtabmap/bags/d456_rgbd_20261013/`：`rtabmap_online.db` 89MB +
`map_2d.pgm/yaml` + `metadata.yaml`（板端裝 `rsync`，`expect` 餵密碼）。
`map_2d.pgm` 本地看過：有房間輪廓（牆/障礙/自由區），但雜訊斑點多——
首版堪用，odom 抖動（2~6Hz）+手持晃動所致；下次慢走+光線好會更乾淨。

**mcap 3.9GB 未拉回**：這段 WiFi 實測約 1MB/s，mcap raw 傳完要一小時以上，
`--partial` 續傳到 747MB 時中止，**本地斷點已刪除**；完整 mcap 仍只在板上
`~/bags/d456_rgbd_20261013/`。要取回建議實體隨身碟拷，或改錄短一點/降解析度。
（本地 `metadata.yaml` 是那份 mcap 的，但少了 mcap 本體，此目錄不能直接
`ros2 bag play`。）

## 11. USB3 重試（2026-10-13，板子改走有線 `192.168.50.127`）

板子重開後 IP 從 `.200`（WiFi）變成 `.127`（eth0 有線，另有 `.128` 副 IP），
相機改掛 **Bus 005 USB3 5000M**（之前都只 train 到 USB2 480M）。
本節速率皆為**板端** `ros2 topic hz` 實測。

**參數正名**：`rgb_camera.profile` / `depth_module.profile` **不存在**，
會被忽略跑預設（color 1280x720@30 + depth 848x480@30，彩色只剩 ~4.3Hz）。
正確是 `rgb_camera.color_profile` / `depth_module.depth_profile`
（可用：`rgb_camera.color_format`、`depth_module.depth_format`）。

**解析度×fps 實測矩陣**（皆 `align_depth.enable:=true` 除非註明）：

| 設定 | color | depth/aligned | imu | 結果 |
|---|---|---|---|---|
| 預設 1280x720@30+848x480@30 | ~4.3Hz | — | ~184Hz | 太重，不用 |
| 640x480@30 | ~13Hz | depth 起不來（`Depth stream start failure`） | 200Hz | 不行 |
| 640x480@15 | ~13Hz | **無聲**（align 把 depth 吃掉，0 幀） | 200Hz | 不行 |
| depth-only 640x480@15 | — | **14.99Hz** | — | depth 本體 OK |
| color+depth 640x480@15（無 align 無 IMU） | 12.9Hz | 15.0Hz | — | OK，兇手是 align |
| 424x240@15 + align（無 IMU） | 15.0Hz | 15.0Hz | — | OK |
| A：424x240@15 + align + IMU | 15.0Hz | 15.0Hz | 200Hz | OK，零 `Out of frame` |
| **B：640x480@5 + align + IMU（採用）** | **5.0Hz** | **5.0Hz** | 200Hz | OK，零 `Out of frame` |

結論：這顆 SoC 跑 `align_depth` 上限約 0.3MPix·fps（640x480@15 會
`Out of frame resources` 餓死 depth），B 版 640x480@5 留 CPU 給 rtabmap。
dmesg 偶見 `uvcvideo ... -71 (EPROTO)`＋`GET_CUR -32`，USB3 鏈路品質普通，
單流/降載可穩跑。

**rtabmap 吃壓縮的正確姿勢**：`rgb_image_transport` /
`depth_image_transport` launch 參數**不會**改變訂閱（實測仍訂 raw×3，
compressed 訂閱數 0）——只能走自架 `republish` 解碼。正確 remap 是
`in/compressed`（不是 `in`），搭配 `-p` 參數：

```bash
ros2 run image_transport republish --ros-args -r __node:=relay_rgb \
  -r in/compressed:=/camera/camera/color/image_raw/compressed \
  -r out:=/relay/color/image_raw \
  -p in_transport:=compressed -p out_transport:=raw
ros2 run image_transport republish --ros-args -r __node:=relay_depth \
  -r in/compressedDepth:=/camera/camera/aligned_depth_to_color/image_raw/compressedDepth \
  -r out:=/relay/aligned_depth_to_color/image_raw \
  -p in_transport:=compressedDepth -p out_transport:=raw
# 驗：/relay/color/image_raw ~4.7Hz，/relay/aligned_depth_to_color/image_raw ~5Hz
```

rtabmap 改吃 `/relay/*`（`compressed:=false`，不加 `imu_topic`，見下）：
odom ~1.8Hz，靜態 quality ~500（std ~0.5mm），可建圖。

**IMU 現況**：`unite_imu_method:=2` 的 `/camera/camera/imu` 以 200Hz 正常輸出，
但它是 6 軸無 orientation，rtabmap `imuAsyncCallback` 會以
`IMU received doesn't have orientation set, it is ignored` 跳過並以 200Hz 洗版
（順便把板子 CPU/SSH 拖垮）。線上建圖先拿掉 `imu_topic`（純視覺），
IMU 照錄進 bag 供離線用；要真·VIO 得另加 attitude filter（madgwick）生 orientation。

**進行中**（`~/bags/d456_usb3_cmp_5fps/`）：壓縮 bag 錄影中
（`.../compressed` + `.../compressedDepth` + 雙 `camera_info` + `imu` + `tf`），
124MB 已停錄；本地 rviz2（`/tmp/opencode/rtabmap_local.rviz`）已開看過，現全關。

**YUYV 翻盤（2026-10-13 同日加測）**：彩色改 `rgb_camera.color_format:=YUYV`
（16bpp，USB 頻寬省一半），`640x480@30` + depth + align + IMU **一次全滿速**：
color 29.97Hz / aligned depth 29.97Hz / imu 200Hz，零 `Out of frame`、零硬體錯誤。
相機節點只吃 ~35% 單核 / ~110MB，記憶體夠。注意 ROS 端吐出仍是 rgb8
（wrapper 內轉），省的是 USB＋librealsense 側。後續建圖優先用此配置，
但 rtabmap 端輸入 30fps 會比 5fps 吃約 6 倍 odom CPU，必要時降到 `@15`。

**首次 VIO 繞行實測（`.233`，YUYV 640x480@15，bag `d456_yuyv15_vio_233`）**：
160s / 33574 則 / **136MB**——壓縮有效（同長 raw 約 3.7GB，~27 倍）。
但**建圖失敗**：走動中板子 CPU 飽和，bag 內 color 掉到 4.3Hz、odom 掉到 0.8Hz，
基線拉大→`Not enough inliers 0/20`→全程 176 次 quality=0、87 幀被丟棄，
只剩 4335 點殘片（`cloud_map.ply` 138KB），`/rtabmap/map` 沒發布、2D 存不了。
教訓：640x480 的 odom（update ~0.2~0.5s）在這顆 SoC 上跟不上走動；
下次用 **424x240@15**（已驗全速，像素少 4 倍）重走。
小檔全拉回本地 `bags/d456_yuyv15_vio_233/`（mcap 141MB + db 31MB + ply + metadata）。

**424x240@15 繞行成功（`.233`，depth 用 480x270——注意 depth 不支援 424x240，**
**會靜默退回 640x480 拖慢 align，改 480x270 後 color/depth/imu 全滿 15/15/200）**：
走動中追蹤撐住（quality ~270 為主，仍有 85 幀被丟但不成災），回到原點後存檔：
`bags/map424/`——`cloud_map.ply` 11206 點**含 RGB**（354KB）、
`map_2d.pgm` 131x123 @0.05m（6.5x6m，有房間輪廓＋障礙＋自由區，雜訊多但可用）、
`rtabmap.db` 34MB，全拉回本地。odom 走動中約 6~7Hz，比 640 版快 3 倍。
rtabmap 仍跑著（db 凍結在 34MB），要再走一圈或停，說一聲。

**424 錄影繞行（`bags/map424_bag/`，154s / 68521 則 / 208MB，全拉回本地）**：
bag 本體是好的——color compressed 2062 則（~13Hz 全速）、compressedDepth 2013 則、
imu 28k、odom 548 則（~3.5Hz）。壓縮效益：同長 424-raw 約 1GB → 208MB（~5x；
424 本來就小，倍數不如 640 的 27x）。
但線上追蹤又丟了（234 次 quality=0），只剩 8694 點 ply、無 2D。
此包 sensor 是全速的，適合**離線降速重播**（`--rate 0.5` 給 odom 兩倍時間），
有機會救回地圖——比 640 那包（輸入只剩 4.3Hz）更有救。

**離線半速重播成功（`bags/map_replay/`，`--rate 0.5`，約 5 分鐘播完）**：
1397 個 odom（392 個 quality=0，剩下全 ~250；live 是 234/130，丟失率 64%→28%），
`cloud_map.ply` **24203 點含 RGB**（live 版一半）、`map_2d.pgm` 135x214 @0.05m
（6.7x10.7m，房間輪廓＋走廊結構清楚，可用）、`rtabmap.db` 15MB，全拉回本地。
結論：**錄全速包＋離線半速建圖**是這塊板子的正確 workflow——live 只負責錄，
重活放離線。注意重播 delay 顯示 600+s 純屬 bag 時間戳是舊的，無害。

**IMU 打通（VIO）**：`imu_filter_madgwick`（板上已有 `imu_filter_madgwick_node`）
吃 `/camera/camera/imu`（`use_mag:=false publish_tf:=false`）吐 `/imu/data` @200Hz
並補上 orientation，rtabmap `imu_topic:=/imu/data` 後零
`doesn't have orientation` 錯誤，真正做視覺-慣性融合。
VIO 版 odom ~2.9Hz **quality ~540**（純視覺 ~370），更好。
標準建圖鏈（`.200`，USB2，YUYV 640x480@15）：
相機 → compressed(+) → 雙 relay → yuv2rgb → rtabmap（+madgwick IMU），不錄 bag，
本地 rviz 看 `/rtabmap/cloud_map`。

**換 port 再測 @30（Bus 005 5000M）**：YUYV 640x480@30 + depth@30 重現
`Depth stream start failure`（重開相機節點也一樣），color 19.5Hz、depth 無聲。
之前那次 29.97 滿速是 4 次中的唯一一次——**@30 不可重現，正式放棄**，
`640x480@15` 定為標準配置（USB2/USB3/各 port 全過）。本輪重建整鏈：
odom ~2.9Hz quality ~510，無 type error。

**前一次 USB3（Bus 003 5000M）**：同樣 @30 跑不滿（color 19.5Hz、depth 無聲），
降 `@15` 全滿（14.8/14.6/200Hz），odom ~2Hz quality ~440。另發現 `relay_depth`
（PNG 解碼）在 SoC 上限約 **7~8Hz**（輸入 14.6Hz 只解出一半），
color JPEG 解碼可滿 15Hz；odom 本來就只有 ~2Hz，吃 7.6Hz depth 照算無誤，
建圖不受影響。

**USB2 回測（之前板子無預警重開後，相機掉回 Bus 002 480M）**：YUYV `640x480@30`
在 USB2 跑不滿（color 19.5Hz、aligned depth 無聲，頻寬不夠），降到 `@15`
則 color 15Hz / aligned depth 15Hz / imu 200Hz 全滿、零錯誤。
最終鏈（USB2 實測可跑）：相機 YUYV@15 → compressed/compressedDepth →
relay 解碼（~15Hz）→ yuv2rgb 轉 rgb8（15Hz）→ rtabmap，
odom ~2.8Hz quality ~370，無 type error。

**新鏈 CPU/記憶體實測**（USB2 YUYV 640x480@15 全速跑，靜態場景，單核%）：
相機節點 55% / ~130MB（含 JPEG+PNG 雙壓縮編碼給 relay 吃）、
`relay_rgb` 11% / `relay_depth` 7%、`yuv2rgb`（python 轉碼）13% / ~160MB、
`rgbd_odometry` 52%（啟動累計平均）/`~185MB、`rtabmap` 24% / ~300MB；
合計 **~1.6 核 + ~800MB**，4 核 / 2.9GB 夠用（`uptime` load 39 照例是 D-state 假象）。
注意 odom 是 burst 型：每幀 update time ~0.15~0.2s，top 瞬時可衝到 **~92% 單核**
（相機節點同時 ~75%、轉碼 ~25%、雙 relay 各 ~17%，整機約 30% 即 1.2 核在跑）。
對照舊鏈（§8，RGB8 直吃無 relay）：odom ~80%——新鏈 odom 反而較輕，
因為輸入 15fps 且 JPEG 解碼扔給了 relay 兩顆分攤。

## 12. 問題總覽（2026-10-13 一日排除全記錄，細節見 §11）

**USB/硬體**：(1) 相機多半只 train 上 USB2，USB3 要靠重插運氣（且 wifi dongle
插著時上不了 USB3，後改走有線）；(2) 相機會「衰退」（幾分鐘後 color→1Hz、
depth 死、IMU 200→30Hz），軟體重開/initial_reset 無效，只能物理重插/重開機；
(3) 板子無預警重開多次，IP 跳來跳去（.200→.127→.128→.200→.233），/tmp 腳本全滅；
(4) `load 39` 是 SoC D-state 假象，看負載以 top %CPU 為準。

**參數坑**：(5) `rgb_camera.profile` 不存在被靜默忽略，正解
`color_profile`/`depth_profile`；(6) rtabmap 硬擋 YUYV，要自寫 yuv2rgb 轉 rgb8；
(7) rtabmap 的 `*_image_transport` 參數無效，只能自架 republish relay，
且 remap 要用 `in/compressed` 形式；(8) depth 不支援 424x240（靜默退回 640x480），
改配 480x270；(9) 6 軸 IMU 無 orientation 會被 rtabmap 忽略＋洗版，
用 madgwick 補 orientation 做真 VIO；(10) `align_depth` 上限約 0.3MPix·fps，
RGB8@15 會 `Out of frame` 餓死 depth；(11) PNG（compressedDepth）編解碼上限 ~8Hz。

**效能/建圖**：(12) YUYV 640x480@30 四次僅一次滿速（`Depth stream start failure`），
放棄，@15 為標準；(13) odom 單核瓶頸（瞬時 ~92%）：640 走動掉到 0.8~3.5Hz→
inliers 0/20 跟丟全滅；424 live 仍會丟，**半速離線重播**救回（24k 點＋可用 2D）；
(14) pkill 殺不掉 setsid 孤兒致三代 rtabmap 搶 topic，改 PID kill；
(15) 高負載 SSH 卡死、WiFi 傳檔 ~400KB/s、rviz pgrep 誤判等小插曲。

**定論 workflow**：live 只錄全速壓縮包（424@15，~5x），建圖放離線半速播。
