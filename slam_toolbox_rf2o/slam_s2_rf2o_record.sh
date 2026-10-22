#!/bin/bash
# 板上錄影：toolbox+rf2o 全開時錄 5 個回放必需 topic
# 用法（板上，SLAM 起來後另開終端）：
#   ~/slam_s2_rf2o_record.sh
#   ~/slam_s2_rf2o_record.sh -o ~/rosbag2_toolbox_rf2o_01
# 拿回本機：
#   scp -r <board>:~/rosbag2_toolbox_rf2o_* .
set -e
source /opt/ros/jazzy/setup.bash
if [ "$1" = "-o" ] && [ -n "$2" ]; then
  OUT="$2"
else
  OUT="$HOME/rosbag2_toolbox_rf2o_$(date +%Y_%m_%d-%H_%M_%S)"
fi
echo "record -> $OUT"
exec ros2 bag record -o "$OUT" /scan /tf /tf_static /odom_rf2o /map
