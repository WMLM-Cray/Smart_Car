#!/bin/bash
# safety_nav_one.sh —— 低速单点导航安全链启动(2026-09-16,team 版,不碰厂家文件)
#
# 与厂家 nav_one.sh 的三点区别:
#   1. 多一层 cmd_vel_safety(/cmd_vel→检查/限幅/超时停车→/car_cmd_vel);
#   2. 底盘用 Run_car.launch.py(node 版 driver,订 /car_cmd_vel;公式 ×100+1540 已联调验证);
#   3. 不起 Run_nav.launch.py(它捆绑 waypoint_cycle 循环路点执行器,会自动发 /goal_pose,
#      违反"本轮只做单目标");直接起 bringup_launch.py(纯 nav2,无循环途经点)。
#
# 链路:RViz 单目标 → bt_navigator → controller_server(/cmd_vel)
#         → cmd_vel_safety(限幅 0.10m/s / 超时 0.5s 零速) → /car_cmd_vel
#         → racecar_driver_node → /dev/car → 电调/舵机
#
# 用法: bash safety_nav_one.sh [地图yaml]     # 默认 map1
# 停止: Ctrl+C(陷阱收链);失控时第一时间物理断电调开关!
set -u
[ -f /opt/ros/humble/setup.bash ] && { set +u; source /opt/ros/humble/setup.bash; set -u; }
[ -f "$HOME/racecar/install/setup.bash" ] && { set +u; source "$HOME/racecar/install/setup.bash"; set -u; }

TEAM="$HOME/racecar_team"
MAP="${1:-$HOME/racecar/src/racecar/map/map1.yaml}"
PARAMS="$TEAM/nav_safe.yaml"

# gate 版本选择(2026-09-19 增): v1=现役(默认,保留回滚) / v2=候选(修复 B1/B2/B3)。
# ⚠️ v2 未部署车端、未经队友阶段验证前,不得作动力导航前提;动力导航默认仍走 v1。
GATE_VERSION="${GATE_VERSION:-v1}"
case "$GATE_VERSION" in
  v1) GATE_PY="$TEAM/encoder_gate.py" ;;
  v2) GATE_PY="$TEAM/encoder_fix/encoder_gate_v2.py" ;;
  *)  echo "未知 GATE_VERSION=$GATE_VERSION(仅 v1/v2),中止"; exit 2 ;;
esac

for f in "$TEAM/cmd_vel_safety.py" "$PARAMS" "$MAP" "$GATE_PY"; do
  [ -f "$f" ] || { echo "缺少 $f,中止"; exit 2; }
done

if pgrep -f "racecar_driver_node" > /dev/null 2>&1; then
  echo "!! 已有 driver 进程在跑,先收掉旧链再启动(避免双写 /dev/car)"; exit 2
fi

SAFETY_PID=""; GATE_PID=""; CAR_PID=""; NAV_PID=""
terminate() {
  echo "收链..."
  [ -n "$NAV_PID" ]   && kill "$NAV_PID" 2>/dev/null
  [ -n "$SAFETY_PID" ] && kill "$SAFETY_PID" 2>/dev/null
  [ -n "$GATE_PID" ]  && kill "$GATE_PID" 2>/dev/null
  [ -n "$CAR_PID" ]   && kill "$CAR_PID" 2>/dev/null
  sleep 3
  # ros2 launch 子进程会脱离进程组,按名兜底(字符类防自匹配)
  pkill -f "[b]ringup_launch" 2>/dev/null
  pkill -f "[R]un_car.launch" 2>/dev/null
  pkill -f "[c]md_vel_safety" 2>/dev/null
  pkill -f "[e]ncoder_gate" 2>/dev/null
  pkill -f "ros2 launch" 2>/dev/null
  wait 2>/dev/null
  echo "已收链。确认车静止后再断电调开关。"
}
trap terminate SIGINT SIGTERM

echo "=== 安全链启动(地图: $MAP)==="
echo "=== 前提自查:车辆按 POWERON 方案就位?区域清空?手边有电调开关? ==="

python3 "$TEAM/cmd_vel_safety.py" > "$TEAM/safety_last.log" 2>&1 &
SAFETY_PID=$!
echo "[1/3] cmd_vel_safety 已起(日志 $TEAM/safety_last.log)"

# 0918 增:编码器合理性闸门(毛刺→假odom→碰撞已发生一次,见 encoder_gate*.py 头注)
python3 "$GATE_PY" > "$TEAM/encoder_gate_last.log" 2>&1 &
GATE_PID=$!
echo "[1b] encoder_gate($GATE_VERSION) 已起(日志 $TEAM/encoder_gate_last.log)"

setsid ros2 launch racecar Run_car.launch.py > "$TEAM/runcar_last.log" 2>&1 &
CAR_PID=$!
echo "[2/3] 传感器+底盘链(Run_car)已起,等 10s..."
sleep 10

ros2 launch racecar bringup_launch.py map:="$MAP" params_file:="$PARAMS" use_sim_time:=false \
  > "$TEAM/nav_last.log" 2>&1 &
NAV_PID=$!
echo "[3/3] nav2 bringup 已起(无 waypoint_cycle)。等 AMCL 就绪后在 RViz 给初始位姿(2D Pose Estimate),再点单目标。"

wait
