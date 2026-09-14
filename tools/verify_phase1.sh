#!/usr/bin/env bash
# =============================================================================
# 阶段1 验收脚本(headless):构建→控制器→直行/转向→传感器/TF→一键启动
# 用法(WSL 内): bash ~/hwcart_sim/tools/verify_phase1.sh
# 幂等:结束自动清理 gazebo/ros 进程
# 2026-09-14 按实测对齐:
#   - Humble 的 ros2 topic echo 无 -n 参数,取单条消息用 timeout+awk(--- 分隔)
#   - tf2_echo 输出格式是 "Translation: [x, y, z]" 数组,不是旧文档的 "x is ..."
#   - joint_states 用 YAML 解析(echo 输出 YAML,不是 JSON)
#   - 启动等 35s:spawn→双 spawner→sim_wiring 事件链需 ~30s
# =============================================================================
export ROS_DOMAIN_ID=42
export ROS2CLI_NO_DAEMON=1
source /opt/ros/humble/setup.bash
source ~/hwcart_sim/hwcart_sim/install/setup.bash
PASS=0; FAIL=0
ok()  { echo "[PASS] $1"; PASS=$((PASS+1)); }
bad() { echo "[FAIL] $1"; FAIL=$((FAIL+1)); }

cleanup() {
  # gzserver 对 SIGTERM 不敏感(实测残留导致下轮端口 11345 被占),必须 -9
  pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null
  pkill -9 -f "ros2 launc[h]" 2>/dev/null; pkill -9 -f "robot_state_publishe[r]" 2>/dev/null
  pkill -9 -f "spawn_entit[y]" 2>/dev/null; pkill -9 -f "sim_wirin[g]" 2>/dev/null
  ros2 daemon stop >/dev/null 2>&1
  # 等 gazebo master 端口释放(最多5s)
  for i in 1 2 3 4 5; do
    ss -ltn 2>/dev/null | grep -q 11345 || break
    sleep 1
  done
  sleep 1
}
trap cleanup EXIT
cleanup

echo "===== [1/5] 一键启动(use_gui:=false) ====="
ros2 launch racecar_sim sim_bringup.launch.py use_gui:=false > /tmp/sim_test.log 2>&1 &
sleep 35
if grep -q "SpawnEntity: Successfully spawned entity" /tmp/sim_test.log; then ok "spawn 成功"; else bad "spawn 失败(见 /tmp/sim_test.log)"; fi
grep -q "parser error" /tmp/sim_test.log && bad "gazebo_ros2_control 参数注入错误" || ok "无 parser error(注释剥离生效)"

echo "===== [2/5] 控制器激活 ====="
# timeout 必须:controller_manager 没起来时该命令无限等待(实测挂10min+)
CTRL=$(timeout 20 ros2 control list_controllers 2>/dev/null)
echo "$CTRL"
echo "$CTRL" | grep -q "joint_state_broadcaster.*active" && ok "joint_state_broadcaster active" || bad "joint_state_broadcaster 未激活"
echo "$CTRL" | grep -q "ackermann_steering_controller.*active" && ok "ackermann_steering_controller active" || bad "ackermann 未激活"

echo "===== [3/5] 传感器 ====="
# Gazebo sensor 惰性激活且就绪顺序不定(实测相机/雷达/IMU 谁晚谁早每轮
# 不同),轮询等待而非死 sleep
wait_topic_alive() {
  local t=$1
  for i in $(seq 1 10); do
    timeout 5 ros2 topic hz "$t" 2>&1 | head -1 | grep -q "average rate" && return 0
    sleep 2
  done
  return 1
}
for t in /scan /image_raw /IMU_data; do
  if wait_topic_alive "$t"; then
    r=$(timeout 8 ros2 topic hz "$t" 2>&1 | head -1)
    echo "$t: $r"; ok "$t 有数据"
  else
    echo "$t: 等待超时"; bad "$t 无数据"
  fi
done

echo "===== [4/5] TF:厂家 frame 与 物理 sensor frame 并存 ====="
# tf2_echo 的 Translation 是 "[x, y, z]" 数组,第三元素即 z
TF_L=$(timeout 8 ros2 run tf2_ros tf2_echo base_footprint laser_link 2>/dev/null | grep -m1 "Translation")
echo "laser_link:        $TF_L"
echo "$TF_L" | grep -qP "Translation: \[[^,]+, [^,]+, 0\.0+\]" && ok "厂家 laser_link z=0 保留" || bad "laser_link z≠0"
TF_LS=$(timeout 8 ros2 run tf2_ros tf2_echo base_footprint laser_sensor_link 2>/dev/null | grep -m1 "Translation")
echo "laser_sensor_link: $TF_LS"
echo "$TF_LS" | grep -qP "Translation: \[[^,]+, [^,]+, 0\.21[0-9]*\]" && ok "laser_sensor_link z=0.21(物理高度)" || bad "laser_sensor_link z≠0.21"

# frame_id:echo 流式输出,grep -m1 流内直配(高频话题 stdout 前几行可能混
# 入 QoS 警告 "A message was lost!!!",head -1 会抓错行)
timeout 12 ros2 topic echo /scan --field header.frame_id 2>/dev/null | grep -m1 -q laser_sensor_link && ok "/scan frame_id=laser_sensor_link" || bad "/scan frame_id 不对"
timeout 12 ros2 topic echo /IMU_data --field header.frame_id 2>/dev/null | grep -m1 -q imu_sensor_link && ok "/IMU_data frame_id=imu_sensor_link" || bad "/IMU_data frame_id 不对"
timeout 12 ros2 topic echo /image_raw --field header.frame_id 2>/dev/null | grep -m1 -qx "camera" && ok "/image_raw frame_id=camera" || bad "/image_raw frame_id 不对"

echo "===== [5/5] 直行与前轮转向 ====="
echo "-- 直行 0.5m/s × 3s --"
timeout 3 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" > /dev/null 2>&1
sleep 1
POS=$(timeout 8 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | grep -m1 "Translation")
echo "odom→base_footprint: $POS"
X=$(echo "$POS" | grep -oP "Translation: \[\K-?[0-9.]+")
awk -v x="${X:-0}" 'BEGIN{exit !(x>1.0)}' && ok "直行位移 x=${X}m>1m" || bad "直行位移不足 x=${X}m"

echo "-- 转向:0.3m/s + 0.5rad/s × 3s --"
timeout 3 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.5}}" > /dev/null 2>&1
sleep 1
# 取一条完整 joint_states:name: 行起收集到 ---(三个坑:QoS 警告行混在
# stdout;--- 会粘在警告行尾;分隔行 --- 若被收集,YAML 里是"第二个文档
# 的开始",safe_load 抛异常 → exit 规则必须放在 print 前且不输出 ---)
JS=$(timeout 8 ros2 topic echo /joint_states 2>/dev/null \
     | awk '/^name:/{flag=1} /^---$/{if(flag)exit} flag{print}')
echo "$JS" | head -4
STEER=$(echo "$JS" | python3 -c "
import sys, yaml
d = yaml.safe_load(sys.stdin)
for n, p in zip(d['name'], d['position']):
    if 'steering' in n:
        print(abs(p)); break
" 2>/dev/null)
awk -v s="${STEER:-0}" 'BEGIN{exit !(s>0.1)}' && ok "前轮转向角 ${STEER}rad>0.1" || bad "前轮未转向"

echo ""
echo "===== 结果: PASS=$PASS FAIL=$FAIL ====="
[ $FAIL -eq 0 ] && echo "ALL_GREEN" || echo "HAS_FAILURES"
