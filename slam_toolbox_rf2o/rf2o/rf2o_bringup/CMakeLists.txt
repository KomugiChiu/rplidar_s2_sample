cmake_minimum_required(VERSION 3.8)
project(rf2o_bringup NONE)

find_package(ament_cmake REQUIRED)

# The single source of truth for the overlay files is ../files (the "bbappend"
# part of this recipe).  This package only installs them flat into the prefix,
# so the deployed layout keeps start.sh / rf2o_params.yaml next to
# local_setup.bash, exactly like the other /nvt_ros2_example/* packages.
set(OVERLAY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../files")

install(PROGRAMS
  "${OVERLAY_DIR}/start.sh"
  "${OVERLAY_DIR}/stop.sh"
  DESTINATION .)

install(FILES
  "${OVERLAY_DIR}/rf2o_params.yaml"
  DESTINATION .)

ament_package()
