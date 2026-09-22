#!/usr/bin/env bash
# Gazebo load smoke test: positive (sensor_test.world loads cleanly in
# headless gzserver, including the trimesh cone collisions) plus a negative
# control (a world referencing a missing model must be detected as failure).
# Exit codes: 0 pass, 1 fail, 2 skipped (gzserver not available).
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

bash tools/gazebo_load_check.sh
rc=$?
if [ "$rc" -eq 2 ]; then
  echo 'SKIP: gazebo not available on this machine'
  exit 2
elif [ "$rc" -ne 0 ]; then
  echo 'FAIL: positive smoke test failed (see log above)'
  exit 1
fi
echo '[PASS] positive: sensor_test.world loads cleanly in gzserver'

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
sed 's#model://cone_red#model://missing_model_xyz#' worlds/sensor_test.world \
  > "$TMP/broken.world"
if bash tools/gazebo_load_check.sh "$TMP/broken.world" >/dev/null 2>&1; then
  echo 'FAIL: negative control NOT detected (checker missed a missing model)'
  exit 1
fi
echo '[PASS] negative: broken world (missing model) correctly detected'
exit 0
