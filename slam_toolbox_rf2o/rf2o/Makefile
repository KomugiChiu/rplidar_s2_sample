# rf2o recipe - front-end for a parent-level build (e.g. `$(MAKE) -C rf2o all`).
#
# The build engine is colcon (see build.sh); this Makefile only maps
# conventional target names onto fetch.sh / build.sh so an outer SDK-style
# makefile can drive this directory without knowing what is inside.
#
# Targets:
#   all            fetch + compile (default; safe to re-run)
#   fetch          clone/patch upstream source (idempotent)
#   compile        run the build command (default: ./build.sh = plain colcon)
#   stage          copy install/ to DESTDIR, do NOT build (outer build already did)
#   install        compile + stage the flat prefix into DESTDIR (backup if it exists)
#   install-files  stage only files/ (scripts + yaml), no build
#   clean          remove build/ install/ log/ (keeps src/)
#   distclean      also remove src/
#
# Variables (all overridable, none of them name an external tool):
#   DESTDIR        staging path (default: $(RF2O_DESTDIR) from src.env)
#   FETCH_CMD      command that prepares src/          (default: ./fetch.sh)
#   BUILD_CMD      command that produces install/      (default: ./build.sh)
#
# A cross-compile layer can inject its own build/rosdep wrapper without this
# directory ever knowing about it:
#   $(MAKE) -C rf2o compile BUILD_CMD="<path-to-wrapper> <workspace-dir>"
# or export BUILD_CMD in the environment.  Nothing here names that wrapper;
# it only has to leave a merge-install layout in install/.

include src.env

# Let `make fetch RF2O_SRCREV=<sha>` reach the script as an environment value.
export RF2O_GIT_URL RF2O_BRANCH RF2O_SRCREV

DESTDIR ?= $(RF2O_DESTDIR)
FETCH_CMD ?= ./fetch.sh
BUILD_CMD ?= ./build.sh

.PHONY: all fetch compile stage install install-files clean distclean check

# fetch -> compile is a chain; keep it serial even under `make -j` from the
# parent build.
.NOTPARALLEL:

all: fetch compile

fetch:
	$(FETCH_CMD)

compile:
	$(BUILD_CMD)

check:
	@[ -d src/rf2o_laser_odometry ] || { echo "src/ missing: run 'make fetch'"; exit 1; }
	@echo "arch: $$(uname -m)  (board needs aarch64)"
	@[ -x install/lib/rf2o_laser_odometry/rf2o_laser_odometry_node ] \
	  && file install/lib/rf2o_laser_odometry/rf2o_laser_odometry_node \
	  || echo "install/ not built yet: run 'make compile'"

# Copy install/ only.  For an outer build that already produced install/ with
# its own command (no dependency on compile, so the default colcon never runs).
stage:
	@[ -d install ] || { echo "ERROR: install/ missing; build it first" >&2; exit 1; }
	@if [ -d "$(DESTDIR)" ]; then \
	  ts=$$(date +%Y%m%d-%H%M%S); \
	  mv "$(DESTDIR)" "$(DESTDIR).bak.$$ts"; \
	  echo "backup: $(DESTDIR).bak.$$ts"; \
	fi
	mkdir -p "$(DESTDIR)"
	cp -a install/. "$(DESTDIR)/"
	chmod +x "$(DESTDIR)/start.sh" "$(DESTDIR)/stop.sh"
	@echo "staged -> $(DESTDIR)"

# Stage the whole flat prefix.  An existing DESTDIR is moved aside first
# (timestamped backup), matching the deploy convention of the other packages.
install: compile
	@if [ -d "$(DESTDIR)" ]; then \
	  ts=$$(date +%Y%m%d-%H%M%S); \
	  mv "$(DESTDIR)" "$(DESTDIR).bak.$$ts"; \
	  echo "backup: $(DESTDIR).bak.$$ts"; \
	fi
	mkdir -p "$(DESTDIR)"
	cp -a install/. "$(DESTDIR)/"
	chmod +x "$(DESTDIR)/start.sh" "$(DESTDIR)/stop.sh"
	@echo "installed -> $(DESTDIR)"

# Overlay only: for environments that already build rf2o themselves and just
# need the scripts/params on top of their own prefix.
install-files:
	mkdir -p "$(DESTDIR)"
	cp -a files/. "$(DESTDIR)/"
	chmod +x "$(DESTDIR)/start.sh" "$(DESTDIR)/stop.sh"
	@echo "files -> $(DESTDIR)"

clean:
	rm -rf build install log

distclean: clean
	rm -rf src
