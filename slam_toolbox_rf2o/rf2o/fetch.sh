#!/usr/bin/env bash
# do_fetch + do_patch (simulated):
#   1. clone upstream rf2o_laser_odometry at the rev pinned in src.env
#   2. apply patches/*.patch on top of the pristine clone
#
# Idempotent: re-running skips the clone and skips already applied patches.
# A rev change discards the previous checkout (including our patches, which are
# re-applied afterwards).  Local edits made inside src/ are lost on purpose,
# the same way bitbake rebuilds WORKDIR from scratch.
#
# Usage:
#   ./fetch.sh
#   RF2O_SRCREV=<sha> ./fetch.sh      # test another rev
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# src.env is the default; environment variables win (RF2O_SRCREV=<sha> ./fetch.sh).
_env_url="${RF2O_GIT_URL:-}"
_env_branch="${RF2O_BRANCH:-}"
_env_srcrev="${RF2O_SRCREV:-}"
# shellcheck source=src.env
source "$SCRIPT_DIR/src.env"
RF2O_GIT_URL="${_env_url:-$RF2O_GIT_URL}"
RF2O_BRANCH="${_env_branch:-$RF2O_BRANCH}"
SRCREV="${_env_srcrev:-${RF2O_SRCREV:-$RF2O_BRANCH}}"
unset _env_url _env_branch _env_srcrev

SRC="$SCRIPT_DIR/src/rf2o_laser_odometry"

mkdir -p "$SCRIPT_DIR/src"

if [ ! -d "$SRC/.git" ]; then
  echo "clone: $RF2O_GIT_URL (branch $RF2O_BRANCH) -> $SRC"
  git clone --quiet --branch "$RF2O_BRANCH" "$RF2O_GIT_URL" "$SRC"
fi

cd "$SRC"
CURRENT="$(git rev-parse HEAD)"
if [ "$CURRENT" != "$SRCREV" ]; then
  # Only touch the network when the pinned rev is not in the local clone
  # (e.g. after bumping RF2O_SRCREV).  Otherwise a re-run is fully offline.
  if ! git cat-file -e "${SRCREV}^{commit}" 2>/dev/null; then
    echo "fetch:  $RF2O_BRANCH (rev not in the local clone)"
    git fetch --quiet origin "$RF2O_BRANCH" \
      || { echo "ERROR: cannot fetch $RF2O_BRANCH from $RF2O_GIT_URL" >&2; exit 1; }
    git cat-file -e "${SRCREV}^{commit}" 2>/dev/null \
      || { echo "ERROR: rev $SRCREV not found on $RF2O_BRANCH" >&2; exit 1; }
  fi
  echo "checkout: $SRCREV (was ${CURRENT:-none})"
  git checkout --quiet -- .
  git checkout --quiet --detach "$SRCREV" \
    || { echo "ERROR: cannot checkout $SRCREV; run 'make distclean' first" >&2; exit 1; }
fi

for patch in "$SCRIPT_DIR"/patches/*.patch; do
  [ -f "$patch" ] || continue
  name="$(basename "$patch")"
  if git apply --check "$patch" 2>/dev/null; then
    echo "patch:  $name"
    git apply "$patch"
  elif git apply --reverse --check "$patch" 2>/dev/null; then
    echo "patch:  $name (already applied)"
  else
    echo "ERROR: $name does not apply on top of $SRCREV" >&2
    exit 1
  fi
done

echo "source ready: $(git rev-parse --short HEAD), $(git status --short | wc -l) locally modified file(s)"
