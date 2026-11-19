# ROS 2 與 Yocto / OpenEmbedded

## 問題

ROS 2 with Yocto：如果我引用 OpenEmbedded，可以知道裡面包含了哪些 ROS 2 package 的原始碼嗎？

## 回答

先釐清一點：**OpenEmbedded / meta-openembedded 本身不含 ROS 2**。ROS 2 的 recipe 來自另一個 layer：`meta-ros`（`github.com/ros/meta-ros`），裡面再分 `meta-ros-common`、`meta-ros2`、`meta-ros2-humble`（或 jazzy 等，依 distro 一個 layer）。所以要先確認你的 `bblayers.conf` 有加這些 layer。

可以，而且有三個層次。

### 1. Layer 裡「有哪些」recipe（原始碼候選清單）

```bash
bitbake-layers show-recipes | grep -i -E 'ros|rcl|msgs'
bitbake -s | grep -i ros
```

或直接看目錄：`meta-ros2-humble/recipes-ros2/` 下每個子目錄就是一個 ROS package 的 recipe（`rclcpp_git.bb` 之類）。

但 **有 recipe ≠ 你的 image 裡有**。meta-ros 是 superflore 從 rosdistro 自動產生的，清單接近該 distro 的 `distribution.yaml`，但不是每個包都會被你的 image 拉進來。

### 2. Image 實際 build 了哪些 recipe（= 哪些 source code）

```bash
bitbake -g core-image-xxx        # 在 build/ 目錄產生
```

會得到：

- `pn-buildlist`：這次 build 用到的**所有 recipe**（每個 ROS package 一行）
- `task-depends.dot`：完整任務依賴圖

這是最準的「原始碼清單」。

### 3. 每個 recipe 對應的 upstream repo / commit

```bash
bitbake -e rclcpp | grep -E '^(SRC_URI|SRCREV|PV)='
```

`SRC_URI` 就是原始碼位置（通常 `git://github.com/ros2/rclcpp.git`），`SRCREV` 是 commit。實際下載解開的 source 在：

```
build/tmp/work/<arch>/<recipe>/<version>/git/
build/downloads/   (DL_DIR，git mirror)
```

## 已安裝套件的清單

- Image manifest：`tmp/deploy/images/<machine>/<image>.manifest`（runtime package 名）
- License manifest：`tmp/deploy/licenses/<image>-<machine>/license.manifest`（含版本、license，可反推 source）
- Host 端：`oe-pkgdata-util list-pkgs`、`oe-pkgdata-util lookup-recipe <pkg>` 做 package → recipe 對映
- Target 上：`ros2 pkg list` 可交叉比對實際安裝的 ROS package

## 注意事項

- meta-ros 是**從原始碼建置**（不是預編 binary），所以「image 裡有哪些 ROS 2 code」幾乎就等於 `pn-buildlist` 裡的 ROS recipe。
- 版本是 `PV = git` + `SRCREV`，不是語意版本；要對照 upstream 得看 commit。
- 部分 ROS package 在 meta-ros 中未支援或被 blacklist（尤其牽 GUI、大依賴的），所以清單會比 rosdistro 少。
- 第三方依賴（Qt、OpenCV、PCL 等）不在 meta-ros，但會出現在同一份 `pn-buildlist`。

一句話：**`bitbake -g <image>` 的 `pn-buildlist` + 每個 recipe 的 `SRC_URI/SRCREV`，就是你要的「包含哪些 ROS 2 原始碼」答案。**

---

# 以 meta-ros jazzy 為基底新增 rf2o 編譯

## 問題

以 meta-ros jazzy 為基底，如果我要新增 rf2o 的編譯，需要做什麼？

## 回答

以下以 **meta-ros `scarthgap` 分支的 `meta-ros2-jazzy`** 為準。

先講結論：**Jazzy 的 meta-ros 裡沒有 rf2o 的 recipe**。`rf2o_laser_odometry` 沒有 release 進 Jazzy rosdistro（`distribution.yaml` 查不到），所以 meta-ros 的 `generated-recipes` 不會有它。你必須自己補一個 recipe，不能只改 `IMAGE_INSTALL`。

建議放在自己的 layer，不要直接改 meta-ros 的 checkout。

### 1. 建 layer

```
meta-rf2o/
├── conf/layer.conf
└── recipes-ros/rf2o-laser-odometry/rf2o-laser-odometry_git.bb
```

`meta-rf2o/conf/layer.conf`：

```bitbake
BBPATH .= ":${LAYERDIR}"
BBFILES += "${LAYERDIR}/recipes-*/*/*.bb ${LAYERDIR}/recipes-*/*/*.bbappend"

BBFILE_COLLECTIONS += "meta-rf2o"
BBFILE_PATTERN_meta-rf2o = "^${LAYERDIR}/"
BBFILE_PRIORITY_meta-rf2o = "10"

LAYERDEPENDS_meta-rf2o = "ros2-jazzy-layer"
LAYERSERIES_COMPAT_meta-rf2o = "scarthgap"
```

> `LAYERSERIES_COMPAT` 要跟你實際用的 Yocto release 一致；若你用別的分支，改成對應名稱。

### 2. Recipe

`recipes-ros/rf2o-laser-odometry/rf2o-laser-odometry_git.bb`：

```bitbake
SUMMARY = "RF2O planar laser odometry for ROS 2"
HOMEPAGE = "https://github.com/MAPIRlab/rf2o_laser_odometry"
LICENSE = "GPL-3.0-only"
LIC_FILES_CHKSUM = "file://LICENSE;md5=84dcc94da3adb52b53ae4fa38fe49e5d"

SRC_URI = "git://github.com/MAPIRlab/rf2o_laser_odometry.git;branch=ros2;protocol=https"
SRCREV = "b38c68e46387b98845ecbfeb6660292f967a00d3"
PV = "0.1.0+git"
S = "${WORKDIR}/git"

inherit ros_distro_jazzy
inherit ros_component

ROS_CN = "rf2o_laser_odometry"
ROS_BPN = "rf2o_laser_odometry"

ROS_BUILD_DEPENDS = " \
    geometry-msgs \
    nav-msgs \
    rclcpp \
    sensor-msgs \
    std-msgs \
    tf2 \
    tf2-geometry-msgs \
    tf2-ros \
    libeigen \
"

ROS_BUILDTOOL_DEPENDS = " \
    ament-cmake-native \
    eigen3-cmake-module-native \
"

ROS_BUILDTOOL_EXPORT_DEPENDS = " \
    eigen3-cmake-module-native \
"

ROS_EXPORT_DEPENDS = "libeigen"

ROS_EXEC_DEPENDS = " \
    geometry-msgs \
    nav-msgs \
    rclcpp \
    sensor-msgs \
    std-msgs \
    tf2 \
    tf2-geometry-msgs \
    tf2-ros \
"

# boost 是 CMakeLists 裡 find_package(Boost) 要的（原始碼其實沒用到）
DEPENDS = "${ROS_BUILD_DEPENDS} ${ROS_BUILDTOOL_DEPENDS} boost"
DEPENDS += "${ROS_EXPORT_DEPENDS} ${ROS_BUILDTOOL_EXPORT_DEPENDS}"
RDEPENDS:${PN} += "${ROS_EXEC_DEPENDS}"

# Jazzy 的 rclcpp 需要 C++17，覆蓋 package 內的 C++14 預設
EXTRA_OECMAKE += "-DCMAKE_CXX_STANDARD=17"

ROS_BUILD_TYPE = "ament_cmake"
inherit ros_${ROS_BUILD_TYPE}
```

### 3. 加進 image

`conf/local.conf`：

```bitbake
IMAGE_INSTALL:append = " rf2o-laser-odometry"
```

（或加進你的 custom image recipe。）

### 4. 編譯與驗證

```bash
bitbake-layers add-layer /path/to/meta-rf2o
bitbake rf2o-laser-odometry
bitbake <your-image>
```

確認有沒有進 image：

```bash
bitbake -g <your-image>
grep rf2o pn-buildlist
oe-pkgdata-util list-pkgs | grep rf2o
```

在 target 上：

```bash
ros2 pkg prefix rf2o_laser_odometry
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
```

可調參數：`laser_scan_topic`（預設 `/scan`）、`odom_topic`（`/odom_rf2o`）、`base_frame_id`、`odom_frame_id`、`publish_tf`。

### 幾個容易踩的點

- **`nav_msgs` 一定要加**：source 用了 `nav_msgs::msg::Odometry`，但 upstream `package.xml` 漏列；CMakeLists 有 `find_package(nav_msgs)`。
- **Eigen 的 OE recipe 叫 `libeigen`**，不是 `eigen`；`eigen3_cmake_module` 對應 `eigen3-cmake-module-native`。
- **不要用 `cmake_modules`**：那是 ROS 1 遺留依賴，Jazzy 沒這 recipe，CMakeLists 也沒用到。
- **Boost 只是 CMakeLists 要求、程式沒用**；不想拉 boost 可以 patch 掉 `find_package(Boost REQUIRED)`。
- **ros2 分支是 2021 年的舊 code**（Foxy/Galactic 時代），在 Jazzy 上「大概率可編」，但若遇到 API 問題，常見修法是：C++17、`package.xml` 的 `<run_depend>` 改成 `<exec_depend>`、移除 Boost。
- recipe 是 `_git.bb`、`SRCREV` 已釘最新 `ros2` 分支 commit；若要固定可重現性，保留這個 SRCREV。
- 不建議把 recipe 直接丟進 meta-ros checkout，更新 meta-ros 時容易衝突或遺失。
