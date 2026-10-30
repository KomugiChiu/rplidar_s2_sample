# Mcnamu_driver final backup（2026-10-30）

這個目錄備份板上最終使用的 Yahboom 底盤 driver。

## 板上實際路徑

```text
/root/raspybot/yahboomcar_ws/src/yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver.py
/root/raspybot/yahboomcar_ws/install/yahboomcar_bringup/lib/python3.12/site-packages/yahboomcar_bringup/Mcnamu_driver.py
```

> 實際執行的是 `install/.../site-packages/.../Mcnamu_driver.py`。
> 修改時兩份都要同步，或重新 build `yahboomcar_bringup`。

## 這個備份包含什麼

`Mcnamu_driver.py` 是最終版 source，包含：

- `STRAIGHT_TRIM_FORWARD = 0.60`
- `STRAIGHT_TRIM_REVERSE = 0.30`
- `MOTOR_GAIN = [1.0, 1.0, 1.0, 1.0]`
- `LINEAR_PWM_PER_MPS = 255.0`
- `SPIN_PWM_PER_RADPS = (117 + 132) / 8`
- `CMD_TIMEOUT_S = 0.3`
- `side_gains()` / `wheel_gains()`
- `cmd_vel_callback()` 內的 forward/reverse trim
- `watchdog()`：0.3 s 沒收到 `/cmd_vel` → 四顆馬達送 0

## 兩種使用方式

### 方式 1：直接複製最終 source

```bash
cp Mcnamu_driver.py \
  /root/raspybot/yahboomcar_ws/src/yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver.py

cp Mcnamu_driver.py \
  /root/raspybot/yahboomcar_ws/install/yahboomcar_bringup/lib/python3.12/site-packages/yahboomcar_bringup/Mcnamu_driver.py
```

### 方式 2：套用完整 patch

Patch 檔案：

```text
../tools/Mcnamu_driver_final.patch
```

Baseline 是原廠 backup：

```text
/root/raspybot/yahboomcar_ws/src/yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver.py.bak.20261021-182215
```

套用：

```bash
cd /root/raspybot/yahboomcar_ws/src/yahboomcar_bringup/yahboomcar_bringup
patch -p1 < /path/to/Mcnamu_driver_final.patch
```

再把修改後的檔案複製到 install 的 site-packages，或重新 build：

```bash
cd /root/raspybot/yahboomcar_ws
colcon build --packages-select yahboomcar_bringup
```

## 重啟 driver

```bash
/nvt_ros2_example/slam_toolbox_rf2o/nav2_1028/tools/restart_driver.sh
```

或重啟 Nav2：

```bash
/nvt_ros2_example/slam_toolbox_rf2o/nav2_1028/stop_nav2.sh
USE_EXTERNAL_SAFETY=0 \
PARAMS_FILE=/nvt_ros2_example/slam_toolbox_rf2o/nav2_1028/nav2_params_plugin.yaml \
  /nvt_ros2_example/slam_toolbox_rf2o/nav2_1028/start_nav2.sh
```

## 備註

- `../tools/Mcnamu_driver_watchdog.patch` 是較早的 watchdog-only patch。
- `Mcnamu_driver_final.patch` 是完整版，已包含 watchdog + trim calibration，**取代** watchdog-only patch。
- 這份 driver 是 2026-10-30 最終調校版本；更換電池、輪子或底盤後，trim 值可能需要重新調整。
