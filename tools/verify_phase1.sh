#!/usr/bin/env bash
# =============================================================================
# 阶段1 验收脚本(headless):构建→控制器→直行/转向→传感器/TF→一键启动
# 用法(WSL 内): bash tools/verify_phase1.sh
# 幂等:结束自动清理 gazebo/ros 进程
# =============================================================================
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/hwcart_sim/hwcart_sim/install/setup.bash
set -u  # 注意:ROS setup.bash 不能在 set -u 下 source
PASS=0; FAIL=0
ok()  { echo "[PASS] $1"; PASS=$((PASS+1)); }
bad() { echo "[FAIL] $1"; FAIL=$((FAIL+1)); }

cleanup() {
  pkill -x gzserver 2>/dev/null; pkill -x gzclient 2>/dev/null
  pkill -f "ros2 launch" 2>/dev/null; pkill -f robot_state_publisher 2>/dev/null
  pkill -f spawn_entity 2>/dev/null; pkill -f controller_manager 2>/dev/null
  sleep 2
}
trap cleanup EXIT
cleanup

echo "===== [1/5] 一键启动(use_gui:=false) ====="
ros2 launch racecar_sim sim_bringup.launch.py use_gui:=false > /tmp/sim_test.log 2>&1 &
sleep 22
if grep -q "SpawnEntity: Successfully spawned entity" /tmp/sim_test.log; then ok "spawn 成功"; else bad "spawn 失败(见 /tmp/sim_test.log)"; fi

echo "===== [2/5] 控制器激活 ====="
CTRL=$(ros2 control list_controllers 2>/dev/null)
echo "$CTRL"
echo "$CTRL" | grep -q "joint_state_broadcaster.*active" && ok "joint_state_broadcaster active" || bad "joint_state_broadcaster 未激活"
echo "$CTRL" | grep -q "ackermann_steering_controller.*active" && ok "ackermann_steering_controller active" || bad "ackermann 未激活"

echo "===== [3/5] 传感器 ====="
HZ_SCAN=$(timeout 8 ros2 topic hz /scan 2>&1 | head -1)
echo "scan: $HZ_SCAN"; echo "$HZ_SCAN" | grep -q "average rate" && ok "/scan 有数据" || bad "/scan 无数据"
HZ_IMG=$(timeout 8 ros2 topic hz /image_raw 2>&1 | head -1)
echo "image: $HZ_IMG"; echo "$HZ_IMG" | grep -q "average rate" && ok "/image_raw 有数据" || bad "/image_raw 无数据"
HZ_IMU=$(timeout 8 ros2 topic hz /IMU_data 2>&1 | head -1)
echo "imu: $HZ_IMU"; echo "$HZ_IMU" | grep -q "average rate" && ok "/IMU_data 有数据" || bad "/IMU_data 无数据"

echo "===== [4/5] TF:厂家 frame 与 物理 sensor frame 并存 ====="
TF_L=$(timeout 6 ros2 run tf2_ros tf2_echo base_footprint laser_link 2>&1 | grep -m1 -A1 Translation)
echo "laser_link:        $TF_L"
echo "$TF_L" | grep -q "z is 0" && ok "厂家 laser_link z=0 保留" || bad "laser_link z≠0"
TF_LS=$(timeout 6 ros2 run tf2_ros tf2_echo base_footprint laser_sensor_link 2>&1 | grep -m1 -A1 Translation)
echo "laser_sensor_link: $TF_LS"
echo "$TF_LS" | grep -q "z is 0.21" && ok "laser_sensor_link z=0.21(物理高度)" || bad "laser_sensor_link z≠0.21"
timeout 4 ros2 topic echo -n 1 /scan --field header.frame_id | grep -q laser_sensor_link && ok "/scan frame_id=laser_sensor_link" || bad "/scan frame_id 不对"
timeout 8 ros2 topic echo -n 1 /IMU_data --field header.frame_id 2>/dev/null | grep -q imu_sensor_link && ok "/IMU_data frame_id=imu_sensor_link" || bad "/IMU_data frame_id 不对"
timeout 8 ros2 topic echo -n 1 /image_raw --field header.frame_id 2>/dev/null | grep -qx "camera" && ok "/image_raw frame_id=camera" || bad "/image_raw frame_id 不对"

echo "===== [5/5] 直行与前轮转向 ====="
echo "-- 直行 0.5m/s × 3s --"
timeout 3 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" > /dev/null 2>&1
sleep 1
POS=$(timeout 6 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -m1 -A1 Translation)
echo "odom→base_footprint: $POS"
X=$(echo "$POS" | grep -oP "x is \K-?[0-9.]+")
awk -v x="${X:-0}" 'BEGIN{exit !(x>1.0)}' && ok "直行位移 x=${X}m>1m" || bad "直行位移不足 x=${X}m"

echo "-- 转向:0.3m/s + 0.5rad/s × 3s --"
timeout 3 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}" > /dev/null 2>&1
sleep 1
JS=$(timeout 6 ros2 topic echo -n 1 /joint_states 2>/dev/null)
echo "$JS" | head -4
STEER=$(echo "$JS" | python3 -c "
import sys, json
d = json.load(sys.stdin)
for n, p in zip(d['name'], d['position']):
    if 'steering' in n: print(abs(p)); break
" 2>/dev/null)
awk -v s="${STEER:-0}" 'BEGIN{exit !(s>0.1)}' && ok "前轮转向角 ${STEER}rad>0.1" || bad "前轮未转向"

echo ""
echo "===== 结果: PASS=$PASS FAIL=$FAIL ====="
[ $FAIL -eq 0 ] && echo "ALL_GREEN" || echo "HAS_FAILURES"
