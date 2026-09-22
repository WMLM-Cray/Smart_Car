#!/usr/bin/env bash
# 调试 /gazebo/get_entity_state 原始输出(查 verify 的 PARSE_FAIL)
export ROS2CLI_NO_DAEMON=1
source "$(dirname "$0")/_env.sh"

echo "== 服务列表 =="
ros2 service list 2>/dev/null | grep -i gazebo

echo "== call 原始输出 =="
timeout 8 ros2 service call /gazebo/get_entity_state gazebo_msgs/srv/GetEntityState \
  "{name: 'racecar', reference_frame: 'world'}" 2>&1
