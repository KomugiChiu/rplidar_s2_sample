cmake_minimum_required(VERSION 3.8)
project(rtabmap_bringup NONE)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/start_rtabmap.sh
  scripts/stop_rtabmap.sh
  scripts/check_deps.sh
  DESTINATION .
)

install(FILES
  config/rtabmap_laser_mapping.ini
  config/nav2_params_rtabmap.yaml
  DESTINATION .
)

ament_package()
