cmake_minimum_required(VERSION 3.8)
project(nav2_1028_bringup NONE)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/start_nav2.sh
  scripts/stop_nav2.sh
  scripts/start_chassis.sh
  scripts/stop_chassis.sh
  scripts/bringup_bond_launch.py
  scripts/keepout_filter_launch.py
  DESTINATION .
)

install(FILES
  config/nav2_params.yaml
  config/nav2_params_plugin.yaml
  config/nav2_params_amcl.yaml
  DESTINATION .
)

install(DIRECTORY
  config/behavior_trees
  DESTINATION .
)

install(DIRECTORY
  scripts/tools
  DESTINATION .
)

ament_package()
