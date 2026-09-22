#!/usr/bin/env bash
# =============================================================================
# 仿真运行时侦查脚本(非验收):查控制器订阅话题/传感器数据流/TF/IMU加载情况
# 用法(WSL 内,仿真已在跑): bash probe_sim.sh
# =============================================================================
export ROS2CLI_NO_DAEMON=1
source "$(dirname "$0")/_env.sh"

echo "===== 1. ackermann 控制器相关话题 ====="
timeout 10 ros2 topic list | grep -i ackermann
echo
echo "===== 2. reference 话题类型 ====="
for t in /ackermann_steering_controller/reference /ackermann_steering_controller/reference_unstamped; do
  echo "--- $t ---"
  timeout 8 ros2 topic info "$t" 2>&1
done
echo
echo "===== 3. 传感器 hz ====="
for t in /scan /IMU_data /camera_sensor/image_raw /joint_states; do
  r=$(timeout 8 ros2 topic hz "$t" 2>&1 | head -1)
  echo "$t : $r"
done
echo
echo "===== 4. TF 抽查 ====="
timeout 6 ros2 run tf2_ros tf2_echo base_footprint laser_link 2>/dev/null | grep -m1 -A1 Translation
timeout 6 ros2 run tf2_ros tf2_echo base_footprint laser_sensor_link 2>/dev/null | grep -m1 -A1 Translation
echo
echo "===== 5. IMU 传感器加载情况(日志) ====="
grep -iE "imu" /tmp/sim_test.log | head -10
