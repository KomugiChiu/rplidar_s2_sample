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
KERNEL=="iio*", MODE="0664", GROUP="video"
```

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

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30 \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2 \
  align_depth.enable:=true
# 另開終端驗：
ros2 topic list | grep camera
ros2 topic hz /camera/camera/color/image_raw /camera/camera/depth/image_raw
ros2 topic echo /camera/camera/color/image_raw --once
```

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

## 6. 相機回來後下一步

```bash
lsusb -d 8086:0b5c   # 先看到裝置
lsusb -t             # 確認 5000M
source /opt/ros/jazzy/setup.bash && rs-enumerate-devices | head -20
# 再跑 §4，hz 穩定後接 rtabmap：
ros2 launch rtabmap_launch rtabmap.launch.py \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  approx_sync:=true qos:=2 rgbd_odometry:=true
```
