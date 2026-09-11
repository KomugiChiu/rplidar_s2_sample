# AArch64 cross toolchain for ROS2 Jazzy packages (x86_64 host -> aarch64).
# 用法: colcon build --cmake-args -DCMAKE_TOOLCHAIN_FILE=<此檔> ...
# 前提: ~/ros2_sysroot/sysroot 已照 cross_build_rplidar.sh 萃好；host 已裝 ROS Jazzy 建置工具。
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CROSS_ROOT "$ENV{HOME}/ros2_sysroot/sysroot")
set(CMAKE_SYSROOT "${CROSS_ROOT}")

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# 只在 sysroot 找 lib/header/package；tool programs (python/colcon) 用 host 的
set(CMAKE_FIND_ROOT_PATH "${CROSS_ROOT}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ament / rosidl 的 try-compile 產物不能在 host 執行，改產 static lib 即可
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# pkg-config 指到 sysroot，避免抓到 host amd64 的 .pc
set(ENV{PKG_CONFIG_SYSROOT_DIR} "${CROSS_ROOT}")
set(ENV{PKG_CONFIG_LIBDIR} "${CROSS_ROOT}/usr/lib/aarch64-linux-gnu/pkgconfig:${CROSS_ROOT}/usr/share/pkgconfig")
