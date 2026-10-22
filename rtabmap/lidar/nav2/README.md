# nav2 + rtabmap（nav2 吃 rtabmap 的地圖）

這組流程讓 **nav2 直接使用 rtabmap 產生的地圖**，不再另外跑 slam_toolbox / amcl。

## 資料流

```
rtabmap (start_rtabmap_laser.sh)
  ├── /rtabmap/map            OccupancyGrid (transient local)
  ├── map → odom              TF (rtabmap SLAM node)
  └── odom → base_footprint  TF (icp_odometry)
        └── base_footprint → base_link → laser   (static)

nav2 (start_nav2_rtabmap.sh, slam:=False use_localization:=False)
  ├── global_costmap 的 static layer 直接訂閱 /rtabmap/map
  └── planner / controller / bt_navigator / costmaps ...
```

`slam:=False use_localization:=False` → nav2 **不會**啟動 slam_toolbox、amcl、map_server，
所以 `map→odom` 與 `/map` 都只有 rtabmap 這一個發布者（不會互搶）。

## 檔案

| 檔案 | 說明 |
|---|---|
| `start_nav2_rtabmap.sh` | 啟動 nav2（背景執行，不佔 console） |
| `nav2_params_rtabmap.yaml` | nav2 參數：static layer 的 `map_topic: /rtabmap/map`，並沿用已調好的 tolerance / inflation / unknown_cost_value 等設定 |
| `behavior_trees/navigate_to_pose_bt.xml` | 自訂 BT：action node 加 `server_timeout="20000"`（單位是**毫秒**） |

## 使用（板上）

```bash
# 1. 起 rtabmap（地圖 + odom），預設 yaw 180°、安靜、背景執行
cd /nvt_ros2_example/rtabmap && ./start_rtabmap_laser.sh

# 2. 起 nav2
cd /nvt_ros2_example/nav2 && ./start_nav2_rtabmap.sh

# 3. 看地圖
rviz2 -d ~/Downloads/komugi/lidar/rtabmap/lidar/nav2_slam_rviz.rviz

# 4. 停（兩個流程共用 stop script）
/nvt_ros2_example/rtabmap/stop_rtabmap_laser.sh
/nvt_ros2_example/nav2/stop_nav2.sh
```

## 驗證結果（2026-10-22 實測）

| 項目 | 結果 |
|---|---|
| `/rtabmap/map` 與 global costmap | 尺寸/原點完全一致（109x113 @0.05 m, origin -1.09,-2.73）→ static layer 成功吃到地圖 |
| nav2 節點 | planner / controller / bt_navigator / 兩個 costmap 全部 `active`，且**沒有** slam_toolbox、amcl、map_server |
| `navigate_to_pose` | `Goal succeeded`（`Received a goal` → `Reached the goal!`），機器人有實際移動 |

## 注意事項

- 地圖是 rtabmap 產生的，**rtabmap 沒在跑就沒有地圖**（此流程不適用於只跑 `start_icp_odom.sh` 的情況）。
- 若要讓 nav2 用靜態地圖 + AMCL 定位，請用原本的 `start_nav2.sh`（`slam:=True` → slam_toolbox）。
- 兩套 SLAM 同時跑會讓 `map→odom` 有兩個發布者 → TF 互搶、機器人位置跳動，**不要同時開**。
