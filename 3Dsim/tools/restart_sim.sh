#!/usr/bin/env bash
# =============================================================================
# 仿真重启+快速体检(开发迭代用,非正式验收)
# 用法(WSL 内,先 cd ~/hwcart_sim): bash tools/restart_sim.sh
# 流程:优雅停 daemon → 清残留 → 启动(headless) → 35s 后输出体检报告
# =============================================================================
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/hwcart_sim/hwcart_sim/install/setup.bash

# 优雅停 daemon(绝不 pkill -9 daemon:强杀留脏 socket,后续 ros2 CLI 报
# xmlrpc Fault !rclpy.ok(),2026-09-14 实测)
ros2 daemon stop >/dev/null 2>&1

# 清残留(模式加字符类防 pkill 自匹配)
pkill -9 -x gzserver 2>/dev/null
pkill -9 -x gzclient 2>/dev/null
pkill -9 -f "sim_bringu[p]" 2>/dev/null
pkill -9 -f "robot_state_publishe[r]" 2>/dev/null
pkill -9 -f "spawne[r]" 2>/dev/null
sleep 2

nohup ros2 launch racecar_sim sim_bringup.launch.py use_gui:=false > /tmp/sim_test.log 2>&1 &
sleep 35

export ROS2CLI_NO_DAEMON=1
echo "=== parser error 计数(应为0) ==="
grep -c "parser error" /tmp/sim_test.log
echo "=== gazebo_ros2_control Loaded(应为1) ==="
grep -c "Loaded gazebo_ros2_control" /tmp/sim_test.log
echo "=== IMU 插件日志(REP145,应出现) ==="
grep -iE "imu_plugin|initial_orientation" /tmp/sim_test.log | head -3
echo "=== 控制器 ==="
timeout 25 ros2 control list_controllers
echo "=== 关键话题 ==="
timeout 10 ros2 topic list | grep -E "/scan$|/image_raw|/IMU_data|/camera_info|reference_unstamped"
echo "=== 传感器 hz ==="
for t in /scan /image_raw /IMU_data; do
  r=$(timeout 8 ros2 topic hz "$t" 2>&1 | head -1)
  echo "$t : $r"
done
