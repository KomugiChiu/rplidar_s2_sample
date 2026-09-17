# ARM64 純板建圖 (console only，不需 x86、不需螢幕)

整條鏈全跑在 ARM 板上：`rplidar_node` + static tf + `slam_toolbox` + 存圖都是
headless，只有 rviz 需要螢幕——盲走建圖，圖存下來拿回 x86 看，或錄 bag 重放。

實測板：`NT98635-Ubuntu`，Ubuntu 24.04 + ROS2 Jazzy，node 裝於 `~/test/rplidar_ros/`。

## 0. 前提 (板上)

```bash
ls -l /dev/ttyUSB*; lsusb -d 10c4:ea60     # 先看到 CP2102N，S2 baud 固定 1000000
id | grep dialout || sudo usermod -aG dialout $USER  # 不在群就加 (重登生效)
# 臨時解 (重開機失效，udev 規則才是永久解)：sudo chmod 666 /dev/ttyUSB0
```

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server
```

`slam_toolbox.yaml` 從 x86 傳上板 (板上只需這一檔，不用整個 workspace)：

```bash
# [x86]
scp ~/Downloads/komugi/ros2_slam/slam_example/config/slam_toolbox.yaml <board>:~/
```

> 網路只用預設 multicast，**兩邊都不要設 `ROS_DISCOVERY_SERVER`**。
> 實測 FastDDS discovery-client 模式在這環境連板子自己都看不到 `/scan`，
> 切回預設 multicast 即正常。若之前設過：`unset ROS_DISCOVERY_SERVER` +
> `ros2 daemon stop` (daemon 會記住舊設定，不重起不生效)。

## 1. 建圖流程 (板上 4 個終端)

```bash
# 終端 1：雷達 (headless，預設 serial /dev/ttyUSB0 / 1000000 / DenseBoost 10Hz)
source /opt/ros/jazzy/setup.bash; source ~/test/rplidar_ros/setup.bash
ros2 launch rplidar_ros rplidar_s2_launch.py
# 期待：health OK + current scan mode: DenseBoost ... scan frequency: 10.0 Hz

# 終端 2、3：static tf (掛著別關，見 §3 為何是固定值)
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint
ros2 run tf2_ros static_transform_publisher 0 0 0.2 0 0 0 base_footprint laser
# 第二個 0.2 = 雷達手持高度 (m)，實際幾公分改多少

# 終端 4：SLAM (Jazzy 這版 async 也是 lifecycle，起來是 unconfigured，要手動開工)
source /opt/ros/jazzy/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file ~/slam_toolbox.yaml
# 另開終端 (或同終端 Ctrl+Z 背景後) 下兩行，重開 node 就要重做：
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
# 期待：兩次都 Transitioning successful，最後 lifecycle get 顯示 active [3]
```

## 2. 無螢幕驗證 (板上)

```bash
ros2 topic hz /scan                        # 期待 ~10Hz (雷達層通了)
ros2 lifecycle get /slam_toolbox           # 期待 active [3]；unconfigured/inactive＝還沒開工
ros2 node info /slam_toolbox | grep -A8 -E 'Subscribers|Publishers'
# 期待 SUBSCRIPTIONS 有 /scan，PUBLISHERS 有 /map + /tf (active 後才有)
ros2 topic list | grep -E '/map|/tf'       # 期待 /map + /tf + /tf_static 都在
ros2 topic echo /map --once                # ROS2 沒有 /map/info，看 /map 裡的 .info.width/.height，變大＝圖在長大
ros2 run tf2_ros tf2_echo map laser        # tf 樹完整才印得出換算值
# 靜止時 /map 沒頻率是正常的 (minimum_travel_distance: 0.5，走半米才更新一次)
```

## 3. tf 說明：高度固定＝固定值

`base_footprint → laser` 描述「雷達怎麼鎖在載具上」，硬鎖不動就是常數，
發一次 latch 全網通用，這就是 `static_transform_publisher` 的用途。
2D SLAM 本來就忽略 z／俯仰／翻滾，大致朝上即可當剛體。

| tf | 性質 | 誰發 |
|---|---|---|
| `base_footprint → laser` | static，安裝幾何 | 手動補 |
| `odom → base_footprint` | static，手持無里程計用 identity 佔位 | 手動補 |
| `map → odom` | **dynamic**，toolbox 每 0.05s 更新 (定位修正量) | toolbox |

樹斷任何一段，toolbox 就報 `Could not get transform` 且不出圖。

## 4. 建圖＋存檔 (板上)

```bash
# 抱著板子＋雷達慢走：平移為主、轉彎放慢、路線多重疊、回到起點 (利於迴環)。
# yaml 內 minimum_travel_distance: 0.5，走半米才更新一次，沒動靜是正常的。
ros2 run nav2_map_server map_saver_cli -f ~/map_s2_01   # 存 pgm + yaml
ros2 bag record /scan /tf /tf_static                    # 留包，回 x86 重放調參
```

圖拿回 x86 看：

```bash
# [x86]
scp <board>:~/map_s2_01.* .    # pgm 直接用看圖軟體開
```

## 5. 資料流向

```
[雷達] → serial 1Mbaud → [rplidar_node：SDK 解包→/scan 10Hz，frame laser]
  → DDS multicast → [toolbox：scan matching＋loop closure，用 tf 把點換算進 map]
  → /map (OccupancyGrid 5cm/格，latch) → map_saver / bag
```

排錯永遠從上游開始：`/scan` 沒有 → 下面全空。`ros2 topic hz` 逐段驗。

## 6. 排查 (實測踩過)

1. **`/scan` 消失**：先 `ps aux | grep rplidar_node` 看 node 活著沒，再 `ros2 daemon stop`
   後重 `topic list` (daemon 會快取舊發現狀態)。
2. **殘留 discovery server**：`ps` 要找 `fast-discovery-server` (不是 `fastdds`)，
   舊的沒清會跟新的搶 11811；本流程不用它，全清掉最乾淨。
3. **小板吃力**：`ros2 launch rplidar_ros rplidar_s2_launch.py angle_compensate:=false`，
   `angle_compensate` 是小板上最吃 CPU 的地方。
4. **串口被搶**：`ModemManager` 佔用時 `sudo systemctl disable --now ModemManager`。
5. **供電**：板載 USB 弱，帶電源 hub 或外供，否則轉速上不去、點數亂跳。
6. **toolbox 沒訂 `/scan`、沒發 `/map`**：`node info` 只有 `/parameter_events` 就是
   `unconfigured`，`lifecycle get` 確認後補 `set ... configure` + `set ... activate`。
