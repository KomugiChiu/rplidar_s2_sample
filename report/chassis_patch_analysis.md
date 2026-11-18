# 底盤（Yahboomcar / Mcnamu driver）Patch 異動分析

> 分析日期：2026-11-18
> 對象：`slam_toolbox_rf2o/nav2_1028/` 底盤 driver 與相關 patch

## 1. 總覽

repo 中共有 3 個 patch，其中 2 個是底盤 driver（Mcnamu），1 個是 rf2o（與底盤無關）。板上實際執行的是 backup 的完整 source `Mcnamu_driver.py`（192 行），patch 是它的來源記錄。

| 檔案 | 對象 | 行數 | 狀態 |
|---|---|---|---|
| `nav2_1028/scripts/tools/Mcnamu_driver_watchdog.patch` | 底盤 | 35 | 舊版，已被取代 |
| `nav2_1028/scripts/tools/Mcnamu_driver_final.patch` | 底盤 | 141 | 現行完整版（2026-10-30） |
| `patches/rf2o_laser_tf_retry.patch` | rf2o odometry | 22 | 非底盤（TF 初始化重試） |

板上使用路徑（`nav2_1028/driver/README.md`）：

```text
/root/raspybot/yahboomcar_ws/src/yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver.py
/root/raspybot/yahboomcar_ws/install/yahboomcar_bringup/lib/python3.12/site-packages/yahboomcar_bringup/Mcnamu_driver.py
```

> 實際執行的是 `install/.../site-packages/...` 那份。修改時兩份都要同步，或重新 `colcon build --packages-select yahboomcar_bringup`。

## 2. 底盤改了什麼（`Mcnamu_driver_final.patch`，現行版）

### 2.1 直線行駛校準（核心改動）— 修「叫它直走卻會偏」

新增常數：

```python
STRAIGHT_TRIM_FORWARD = 0.60
STRAIGHT_TRIM_REVERSE = 0.30
```

原理：

- **只補償較弱的一側**（不降低快的一側），避免慢速側掉進 PWM ~33 死區。
  - `trim > 0` → 加速右輪（車往右偏時用）
  - `trim < 0` → 加速左輪
- 前進/後退分開補償（mecanum 底盤前後漂移不同）。
- **原地旋轉（vx=0）不補償**，保留 Nav2 的旋轉校正（`wz_floor` / `rot_rate`）有效。
- 調校方式：送 `vx=0.14` 0.7 s，讀 `/odom_rf2o` 的 yaw；yaw 為負（右轉）就提高 trim，為正就降低。
- 本車2026-10-30實測：trim 0 → 前進 -0.20 rad / 後退 +0.08 rad（每 22 cm）。

新增函式：

```python
def side_gains(trim):          # 回傳 (left_gain, right_gain)，只補弱側
def wheel_gains(trim):         # 套 MOTOR_GAIN 後回傳 4 輪 [m0,m1,m2,m3]
```

### 2.2 每馬達獨立增益

```python
MOTOR_GAIN = [1.0, 1.0, 1.0, 1.0]
```

疊在 `STRAIGHT_TRIM_*` 之上，可個別微調四顆馬達。

### 2.3 PWM 常數參數化

把原本散落在程式中的魔法數字抽成常數，方便調校：

```python
LINEAR_PWM_PER_MPS = 255.0
SPIN_PWM_PER_RADPS = (117 + 132) / 8
```

### 2.4 PWM 輸出改寫（含安全鉗制）

```python
if speed_fb > 0.0:   gain = self.gain_fwd
elif speed_fb < 0.0: gain = self.gain_rev
else:                gain = self.gain_spin

for i, r in enumerate((r1, r2, r3, r4)):
    pwm = max(-255, min(255, round(r * gain[i])))
    self.car.Ctrl_Muto(i, pwm)
```

依 `speed_fb` 正/負/零選 fwd/rev/spin gain，並加 `-255~255` 上限保護。

### 2.5 Watchdog 停車保護（安全關鍵）

```python
CMD_TIMEOUT_S = 0.3
self.last_cmd_time = time.time()
self.watchdog_timer = self.create_timer(0.1, self.watchdog)

def watchdog(self):
    if time.time() - self.last_cmd_time > CMD_TIMEOUT_S:
        for i in range(4):
            self.car.Ctrl_Muto(i, 0)
```

0.3 秒沒收到 `/cmd_vel` → 四顆馬達送 0。MCU 會閂住最後指令，沒有此機制車會持續衝。

### 2.6 I2C 讀取容錯（穩定性）

超音波與循線讀取前檢查回傳值非空，失敗時 `warn` 並 `return`，避免 `track[0]` / `raw_H[0]` 在空陣列時崩潰：

```python
track = self.car.read_data_array(0x0a, 1)
if not track:
    self.get_logger().warn('I2C read failed (line sensor)')
    return
# 超音波同理（0x1b / 0x1a）
```

## 3. 兩個底盤 patch 的關係

| | watchdog patch（舊） | final patch（現行） |
|---|---|---|
| 時間 | 2026-10-30 13:54 | 2026-10-30 17:18 |
| 內容 | 只有 watchdog（2.5） | watchdog + 直線校準 + 增益 + PWM 常數 + I2C 容錯 |
| 關係 | — | 完全取代 watchdog patch |

`final` 是 `watchdog` 的超集。若先套 `watchdog` 再套 `final` 會衝突；依 `driver/README.md`，**直接用 `final` 即可**。

套用方式：

```bash
cd /root/raspybot/yahboomcar_ws/src/yahboomcar_bringup/yahboomcar_bringup
patch -p1 < /path/to/Mcnamu_driver_final.patch
cd /root/raspybot/yahboomcar_ws && colcon build --packages-select yahboomcar_bringup
```

Baseline 為原廠 backup：`Mcnamu_driver.py.bak.20261021-182215`。

## 4. 與底盤相關的「非 patch」調整（重要脈絡）

由 `nav2_1028.md` 可見，底盤問題的解法不只 patch。因底盤為**純開環、無輪編碼器**，另搭配 Nav2 參數：

- 移除 `translation_only.py` gate → 底盤直接訂閱 `/cmd_vel`。
- `wz_max` / `wz_std` 由 1.9 / 0.4 壓到 **0.25 / 0.15**（開環底盤猛烈旋轉過衝）。
- 關閉 `vy`（開環橫移會偏航，6 s 內 yaw 漂 +26°）。
- DeadbandController 處理 PWM 死區（`vx < 0.12` 會間歇卡住；`vx=0.08` ≈ 20 PWM 低於啟動死區）。
- `yaw_goal_tolerance` 0.25 → 0.35（避免底盤最小起轉步進反覆抖動）。
- 獨立 `start_chassis.sh` / `stop_chassis.sh`；`stop_nav2.sh` 支援 `KEEP_DRIVER=1` 保留底盤。

## 5. 非底盤 patch：`rf2o_laser_tf_retry.patch`

雖非底盤，但同屬車體相關，一併記錄。修改 `rf2o/src/CLaserOdometry2DNode.cpp`：

- 雷達安裝 TF 只在第一幀讀取一次；開機 DDS discovery 慢時 TF 可能還沒進 buffer，原程式會 silently fallback 成「雷達 = base」，導致 odometry 被安裝角旋轉。
- 改為 `setLaserPoseFromTf()` 失敗時以 `RCLCPP_WARN_THROTTLE` 警告並 `return`，下一幀重試。

## 6. 注意事項

1. 板上執行的是 `install/.../site-packages/.../Mcnamu_driver.py`，不是 src 那份 — 修改要同步兩份或重新 build。
2. 這台是 2026-10-30 調校版本；換電池、輪子或底盤後 trim 值需重新調整。
3. `final` patch 已包含 watchdog，勿再套 `watchdog` patch。

## 7. 可引用路徑

| 用途 | 路徑 |
|---|---|
| 現行 driver source | `slam_toolbox_rf2o/nav2_1028/driver/Mcnamu_driver.py` |
| 完整 patch | `slam_toolbox_rf2o/nav2_1028/scripts/tools/Mcnamu_driver_final.patch` |
| 舊 watchdog patch | `slam_toolbox_rf2o/nav2_1028/scripts/tools/Mcnamu_driver_watchdog.patch` |
| driver 說明 | `slam_toolbox_rf2o/nav2_1028/driver/README.md` |
| Nav2 調校紀錄 | `nav2_1028.md` |
