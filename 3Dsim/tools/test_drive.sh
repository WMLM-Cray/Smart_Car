#!/usr/bin/env bash
# =============================================================================
# 驱动实测(验收第5步的独立版):/cmd_vel 直行 → TF 位移;转向 → 关节角
# 前提:仿真已在跑且 cmd_vel_relay 已启动(bash tools/restart_sim.sh 先行)
# =============================================================================
export ROS_DOMAIN_ID=42
export ROS2CLI_NO_DAEMON=1
source /opt/ros/humble/setup.bash
source ~/hwcart_sim/hwcart_sim/install/setup.bash

echo "===== 直行 0.5 m/s × 3s ====="
timeout 3 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" >/dev/null 2>&1
sleep 1
echo "-- odom→base_footprint 位移 --"
timeout 6 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | grep -m1 -A1 Translation

echo
echo "===== 转向 0.3 m/s + 0.5 rad/s × 3s ====="
timeout 3 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}" >/dev/null 2>&1
sleep 1
echo "-- joint_states(看 steering 角) --"
timeout 8 ros2 topic echo -n 1 /joint_states 2>/dev/null | sed -n "/name:/,/position:/p" | head -6
