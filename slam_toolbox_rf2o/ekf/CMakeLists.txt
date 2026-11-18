cmake_minimum_required(VERSION 3.8)
project(ekf_bringup NONE)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/start_ekf.sh
  scripts/stop_ekf.sh
  scripts/check_deps.sh
  DESTINATION .
)

install(FILES
  config/ekf_params.yaml
  config/ekf_params_icp.yaml
  DESTINATION .
)

ament_package()
