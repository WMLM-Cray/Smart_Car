#!/usr/bin/env bash
# Open a racecar_test_assets world in Gazebo Classic WITHOUT ROS2 (test-fixture
# preview). Usage: bash tools/preview.sh [world-file] [extra gazebo arguments]
#   world-file defaults to sensor_test.world (e.g. demo_world.world).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(dirname "$SCRIPT_DIR")"
export GAZEBO_MODEL_PATH="$PKG_ROOT/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
WORLD="sensor_test.world"
if [[ "${1:-}" == *.world ]]; then
  WORLD="$1"
  shift || true
fi
exec gazebo --verbose "$PKG_ROOT/worlds/$WORLD" "$@"
