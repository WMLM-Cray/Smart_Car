#!/usr/bin/env bash
# 只启动仿真；不杀进程、不自动启动或使能策略。
set -e
SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [ ! -f "$SIM_ROOT/install/setup.bash" ]; then
  echo "请先在 $SIM_ROOT 执行 colcon build" >&2
  exit 1
fi
source "$SIM_ROOT/install/setup.bash"
export ROS_DOMAIN_ID=43
export GAZEBO_MODEL_DATABASE_URI=""
exec ros2 launch racecar_sim sim_bringup.launch.py "$@"
