[English](README.md) | 繁體中文

# rf2o recipe（yocto 模擬）

把 rf2o（上游 `MAPIRlab/rf2o_laser_odometry`）變成「可複製的編譯配方」：
**上游 source 用 git clone 取得、本地新增的檔案用事先 copy 的 overlay 模擬
bbappend、最後 colcon build 出一個同時含 node 與 scripts/yaml 的扁平 prefix**。

## 目錄（= 一個 recipe）

```
rf2o/
├── Makefile                        ← 對外門面：給上一層 makefile $(MAKE) -C rf2o 用
├── src.env                         ← 唯一變數出處（URL / branch / SRCREV / DESTDIR）
├── fetch.sh                        ← do_fetch + do_patch：clone + git apply（冪等）
├── build.sh                        ← do_compile + do_install：colcon build --merge-install
├── patches/rf2o_laser_tf_retry.patch  ← 本地 patch（雷射 TF 未就緒時重試，不退回 laser==base）
├── files/                          ← bbappend 的 file://：scripts + yaml 唯一正本
│   ├── start.sh / stop.sh / rf2o_params.yaml
├── rf2o_bringup/                   ← colcon package：把 files/ 裝進 prefix 根（上游 clone 不動）
└── src/rf2o_laser_odometry/        ← fetch.sh 的 clone 目標（不進 git）
```

上游 clone 永遠保持乾淨：`rf2o_bringup` 只負責安裝 overlay 檔案，所以同一棵樹
隨時可以從零重建。

## yocto 對照

| yocto | 這裡 |
|---|---|
| `SRC_URI = git://...;branch=ros2` | `src.env` 的 `RF2O_GIT_URL` / `RF2O_BRANCH` |
| `SRCREV` | `RF2O_SRCREV`（釘在 `b38c68e`，ros2 branch 當時的 head） |
| `SRC_URI += file://*.patch` | `patches/*.patch`，由 `fetch.sh` 套上 |
| `FILESEXTRAPATHS` / append 補檔案 | `files/` + `rf2o_bringup`（只負責安裝，不改上游） |
| `do_compile` / `do_install` | `build.sh`（colcon build --merge-install） |
| image 部署 | `make install`（DESTDIR） |

## 用法

```bash
make all                     # fetch + compile（重跑安全）
make fetch                   # 只補 source + patch
make compile                 # 只 colcon build
make stage                   # 只把 install/ 複製到 DESTDIR，不 build
make install                 # compile + 部署（既有 DESTDIR 會備份成 *.bak.<ts>）
make install-files           # 只鋪 files/（給已經自己會編 rf2o 的環境）
make check                   # 看 src/ 與產物狀態
make clean / distclean       # 清輸出 / 連 src/ 一起清
```

外層 makefile（`dir1/Makefile`）只要：

```makefile
all clean install:
	$(MAKE) -C rf2o $@
```

或以 SDK 慣例帶 staging：`$(MAKE) -C rf2o install DESTDIR=$(STAGE)/rf2o`。

`fetch` 重跑時完全離線：`src/` 已存在且停在 pin 的版本就不 clone，patch 已套
就跳過。只有在「本地 clone 沒有這個 rev」時（例如改了 `RF2O_SRCREV`）才會上
網抓；`RF2O_SRCREV=<sha> ./fetch.sh` 與 `make fetch RF2O_SRCREV=<sha>` 都可用
（環境變數優先於 `src.env`）。

## 交叉編譯（由外層注入）

`compile` 不寫死 colcon，而是執行 `$(BUILD_CMD)`（預設 `./build.sh`）；
`fetch` 同理是 `$(FETCH_CMD)`。外層（cross 環境）可以把自家的 build/rosdep
wrapper 塞進來，而這個資料夾完全不需要知道它的名字：

```makefile
# 外層 Makefile（wrapper 只存在這一層）
all:
	$(MAKE) -C rf2o fetch FETCH_CMD="<your-fetch-wrapper>"
	$(MAKE) -C rf2o compile BUILD_CMD="<your-build-wrapper> $(CURDIR)/rf2o"
	$(MAKE) -C rf2o stage DESTDIR=$(CURDIR)/stage
```

或由環境變數注入（Make 會沿用已存在的 `BUILD_CMD`）：

```bash
export BUILD_CMD=/path/to/cross-build-wrapper
make -C rf2o compile
```

契約：wrapper 收到「workspace 路徑」，最後要在該路徑留下 **merge-install** 的
`install/`（`lib/rf2o_laser_odometry/...`、`share/...`、`setup.bash`）。
`stage` 只複製 `install/`，不會再觸發預設 colcon；若 wrapper 只編上游、
沒編 `rf2o_bringup`，再補一次 `make install-files` 把 `files/` 疊上去即可。

## ⚠️ 架構限制

`rf2o_laser_odometry_node` 是**編譯產物**，編譯機器 = 執行機器：

- 板子是 **aarch64** → 正式 build 在**板子上**跑（`fetch.sh` 會直接在板上
  clone，板子有網路）
- notebook（x86_64）的 build 只用來驗證 recipe 結構，產物**不能**部署到板子

板上流程：

```bash
# notebook 端把整包 recipe 丢過去（src/build/install/log 都不用帶）
rsync -a --exclude src --exclude build --exclude install --exclude log \
      slam_toolbox_rf2o/rf2o/ root@BOARD:/root/rf2o_build/

# 板上
cd /root/rf2o_build
./fetch.sh                     # clone + patch（冪等）
./build.sh                     # colcon build --merge-install
make install DESTDIR=/nvt_ros2_example/rf2o   # 自動備份舊目錄
```

編譯前先停掉 demo：在滿載的 stack 上再全核編譯，曾把板子操到重開機。

## 產出（install/，與 /nvt_ros2_example/rf2o 同構）

```
install/lib/rf2o_laser_odometry/rf2o_laser_odometry_node   ← 上游編譯產物
install/share/rf2o_laser_odometry/launch
install/start.sh  install/stop.sh  install/rf2o_params.yaml ← files/ overlay
install/setup.bash / local_setup.bash                       ← start.sh 會 source 的
```

`start.sh` 行為不變：只吃 `/scan` 與 `base_footprint→laser` TF，發佈
`/odom_rf2o`（`publish_tf: false`，`odom→base_footprint` 仍由 EKF 擁有）。

## patch 內容

`rf2o_laser_tf_retry.patch`：第一張 scan 初始化時若 static TF 還沒進 buffer，
改為等下一張 scan 重試；原本會 silently 退回 `laser == base`，讓 odom 旋轉
180°（安裝 yaw）。
