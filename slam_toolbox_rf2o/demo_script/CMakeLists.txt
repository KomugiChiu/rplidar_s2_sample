cmake_minimum_required(VERSION 3.8)
project(demo_script_bringup NONE)

find_package(ament_cmake REQUIRED)

# Flat prefix layout: the entry points and demo_common.sh sit next to
# local_setup.bash, exactly like /nvt_ros2_example/demo_script.
install(PROGRAMS
  scripts/start_demo.sh
  scripts/start_demo_static.sh
  scripts/start_mapping.sh
  scripts/stop_demo.sh
  scripts/record_map.sh
  scripts/demo_common.sh
  DESTINATION .
)

install(PROGRAMS
  scripts/for_rtabmap/start_demo.sh
  scripts/for_rtabmap/stop_demo.sh
  DESTINATION for_rtabmap
)

install(FILES
  scripts/for_rtabmap/README.md
  scripts/for_rtabmap/README.zh-TW.md
  DESTINATION for_rtabmap
)

install(FILES
  README.md
  README.zh-TW.md
  DESTINATION .
)

ament_package()
