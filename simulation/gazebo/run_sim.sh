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
# 仿真域可用环境变量指定,默认 60。共享服务器上队友的仿真占着 43~50、84(2026-09-26 查),
# 实车是 42。同域会串话题:我们的 /cmd_vel 会开动别人的车。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-60}"
if [ "$ROS_DOMAIN_ID" = "42" ]; then
  echo "ROS_DOMAIN_ID=42 是实车域,仿真不能用" >&2
  exit 1
fi
echo "仿真域 ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
# Gazebo 自带模型(ground_plane、sun)的路径在这里;精简时漏了这句,场景里没有地面(2026-09-26)。
# 主端口按域错开(默认 11345 是别人仿真在用的),setup.sh 不覆盖已设的值
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://localhost:$((11300 + ROS_DOMAIN_ID))}"
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_DATABASE_URI=""
echo "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
exec ros2 launch racecar_sim sim_bringup.launch.py "$@"
