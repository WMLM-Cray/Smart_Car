#!/usr/bin/env bash
# Headless load smoke test: run gzserver on a world and fail on load errors
# (missing model resources, mesh load failures, SDF problems).
# Usage: tools/gazebo_load_check.sh [world_file]
# Exit codes: 0 = clean load, 1 = load error detected, 2 = gzserver not found.
set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(dirname "$SCRIPT_DIR")"
WORLD="${1:-$PKG_ROOT/worlds/sensor_test.world}"

if ! command -v gzserver >/dev/null 2>&1; then
  echo 'SKIP: gzserver not found in PATH' >&2
  exit 2
fi
export GAZEBO_MODEL_PATH="$PKG_ROOT/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

LOG="$(mktemp /tmp/gzload.XXXXXX)"
trap 'rm -f "$LOG"' EXIT
timeout -k 5 --signal=INT 15 gzserver --verbose "$WORLD" >"$LOG" 2>&1
RC=$?

# gzserver runs until killed: rc 124 (timeout SIGINT) or 0 means it stayed
# alive; anything else is a crash.
if [ "$RC" != "0" ] && [ "$RC" != "124" ]; then
  echo "FAIL: gzserver exited with code $RC" >&2
  sed 's/^/    /' "$LOG" >&2
  exit 1
fi
if ! grep -Eq 'Gazebo multi-robot simulator|connected to gazebo master' "$LOG"; then
  echo 'FAIL: gzserver did not start (no startup banner in log)' >&2
  sed 's/^/    /' "$LOG" >&2
  exit 1
fi
# A referenced model:// not shipped locally makes Gazebo fall back to the
# ONLINE ModelDatabase ("Getting models from[http://models.gazebosim.org/]").
# That is exactly the "cache/network masks a missing file" case - flag it.
ERROR_RE='\[Err\]|Error [0-9]|ould not|nable to load|does not exist|No such file|nvalid|Getting models from'
if grep -Eq "$ERROR_RE" "$LOG"; then
  echo 'FAIL: error lines detected in gzserver log:' >&2
  grep -En "$ERROR_RE" "$LOG" | sed 's/^/    /' >&2
  exit 1
fi
echo "OK: gzserver loaded $WORLD cleanly (rc=$RC)"
exit 0
