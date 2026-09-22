#!/usr/bin/env bash
# =============================================================================
# 驱动链路分诊:/cmd_vel pub → relay 转发 → 控制器订阅 → joint_states
# 前提:仿真+relay 在跑。逐段亮灯,坏在哪一段一目了然。
# 注意:Humble 的 ros2 topic echo 无 -n 参数(参数错误会静默退出),取一条
#       消息用 timeout+head
# =============================================================================
export ROS2CLI_NO_DAEMON=1
source "$(dirname "$0")/_env.sh"

echo "== [A] joint_states 裸 echo(基础可用性) =="
timeout 6 ros2 topic echo /joint_states 2>/dev/null | head -8

echo
echo "== [B] pub /cmd_vel 的同时 echo relay 输出话题 =="
nohup timeout 6 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" >/tmp/pub.log 2>&1 &
sleep 2
timeout 4 ros2 topic echo /ackermann_steering_controller/reference_unstamped 2>/dev/null | head -10

echo
echo "== [C] 控制器 odometry(车动没动) =="
timeout 6 ros2 topic echo /ackermann_steering_controller/odometry 2>/dev/null | grep -E "  x:|  y:|frame_id" | head -8

echo
echo "== [D] TF odom =="
timeout 6 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -m1 -A1 "Translation" || echo "(无 odom TF)"
