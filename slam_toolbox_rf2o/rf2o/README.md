[English] | [繁體中文](README.zh-TW.md)

# rf2o recipe (yocto simulation)

Turns rf2o (upstream `MAPIRlab/rf2o_laser_odometry`) into a **copyable build
recipe**: the upstream source is obtained with `git clone`, local additions are
pre-copied overlay files (simulating a bbappend), and a single `colcon build`
produces a flat prefix that contains both the compiled node and the
scripts/yaml.

## Layout (one recipe)

```
rf2o/
├── Makefile                        <- front-end for an outer makefile: $(MAKE) -C rf2o ...
├── src.env                         <- single source of truth (URL / branch / SRCREV / DESTDIR)
├── fetch.sh                        <- do_fetch + do_patch: clone + git apply (idempotent)
├── build.sh                        <- do_compile + do_install: colcon build --merge-install
├── patches/rf2o_laser_tf_retry.patch  <- local patch (retry laser TF, never fall back to laser==base)
├── files/                          <- the bbappend "file://" part: scripts + yaml (single copy)
│   ├── start.sh / stop.sh / rf2o_params.yaml
├── rf2o_bringup/                   <- colcon package that installs files/ at the prefix root
└── src/rf2o_laser_odometry/        <- clone target of fetch.sh (not tracked by git)
```

The upstream clone is never modified: `rf2o_bringup` only installs the overlay
files, so the same tree can be rebuilt from scratch at any time.

## Yocto mapping

| yocto | here |
|---|---|
| `SRC_URI = git://...;branch=ros2` | `RF2O_GIT_URL` / `RF2O_BRANCH` in `src.env` |
| `SRCREV` | `RF2O_SRCREV` (pinned to `b38c68e`, head of the ros2 branch) |
| `SRC_URI += file://*.patch` | `patches/*.patch`, applied by `fetch.sh` |
| `FILESEXTRAPATHS` / append files | `files/` + `rf2o_bringup` (installs, never patches upstream) |
| `do_compile` / `do_install` | `build.sh` (colcon build --merge-install) |
| image deployment | `make install` (DESTDIR) |

## Usage

```bash
make all                     # fetch + compile (safe to re-run)
make fetch                   # source + patches only
make compile                 # colcon build only
make stage                   # copy install/ to DESTDIR, do NOT build
make install                 # compile + stage (existing DESTDIR is backed up to *.bak.<ts>)
make install-files           # overlay only (scripts + yaml), no build
make check                   # show src/ and artifact state
make clean / distclean       # remove outputs / also remove src/
```

An outer makefile (`dir1/Makefile`) only needs:

```makefile
all clean install:
	$(MAKE) -C rf2o $@
```

or, with SDK-style staging: `$(MAKE) -C rf2o install DESTDIR=$(STAGE)/rf2o`.

`fetch` is offline on re-runs: the clone is skipped when `src/` already exists
at the pinned rev, and patches are skipped when already applied. The network is
only touched when the pinned rev is missing from the local clone (e.g. after
bumping `RF2O_SRCREV`); `RF2O_SRCREV=<sha> ./fetch.sh` and
`make fetch RF2O_SRCREV=<sha>` both work (environment variables win over
`src.env`).

## Cross compilation (injected by the outer layer)

`compile` does not hard-code colcon: it runs `$(BUILD_CMD)` (default
`./build.sh`), and `fetch` runs `$(FETCH_CMD)`. A cross-compile layer can plug
in its own build/rosdep wrapper without this directory knowing its name:

```makefile
# outer Makefile (the wrapper lives only in this layer)
all:
	$(MAKE) -C rf2o fetch FETCH_CMD="<your-fetch-wrapper>"
	$(MAKE) -C rf2o compile BUILD_CMD="<your-build-wrapper> $(CURDIR)/rf2o"
	$(MAKE) -C rf2o stage DESTDIR=$(CURDIR)/stage
```

or through the environment (make keeps an already defined `BUILD_CMD`):

```bash
export BUILD_CMD=/path/to/cross-build-wrapper
make -C rf2o compile
```

Contract: the wrapper receives the workspace path and must leave a
**merge-install** `install/` there (`lib/rf2o_laser_odometry/...`,
`share/...`, `setup.bash`). `stage` only copies `install/` and never triggers
the default colcon; if the wrapper builds only the upstream package, add
`make install-files` to put `files/` on top.

## Architecture constraint

`rf2o_laser_odometry_node` is a compiled artifact, so the build host must match
the run host:

- the board is **aarch64** -> run the real build **on the board** (`fetch.sh`
  clones there; the board has network access)
- a notebook (x86_64) build only validates the recipe and **must not** be
  deployed to the board

Board flow:

```bash
# from the notebook: ship the recipe (no src/build/install/log needed)
rsync -a --exclude src --exclude build --exclude install --exclude log \
      slam_toolbox_rf2o/rf2o/ root@BOARD:/root/rf2o_build/

# on the board
cd /root/rf2o_build
./fetch.sh                     # clone + patch (idempotent)
./build.sh                     # colcon build --merge-install
make install DESTDIR=/nvt_ros2_example/rf2o   # backs up the previous directory
```

Stop the demo before building: a full compile on top of a running stack can
push the board hard enough to reboot it.

## Output (install/, same shape as /nvt_ros2_example/rf2o)

```
install/lib/rf2o_laser_odometry/rf2o_laser_odometry_node   <- upstream build
install/share/rf2o_laser_odometry/launch
install/start.sh  install/stop.sh  install/rf2o_params.yaml <- files/ overlay
install/setup.bash / local_setup.bash                       <- sourced by start.sh
```

`start.sh` behaviour is unchanged: it consumes `/scan` and the
`base_footprint -> laser` TF and publishes `/odom_rf2o`
(`publish_tf: false`; `odom -> base_footprint` stays owned by the EKF).

## Patch

`rf2o_laser_tf_retry.patch`: when the first scan arrives before the static
transforms are in the buffer, wait for the next scan instead of silently
falling back to `laser == base` (which rotated the odometry by the mounting
yaw).
