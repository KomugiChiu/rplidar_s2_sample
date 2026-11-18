cmake_minimum_required(VERSION 3.8)
project(slam_toolbox_bringup NONE)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/start_slam.sh
  scripts/stop_slam.sh
  scripts/save_map.sh
  scripts/check_deps.sh
  DESTINATION .
)

install(FILES
  config/slam_toolbox_rf2o.yaml
  config/slam_toolbox_localization.yaml
  DESTINATION .
)

ament_package()
