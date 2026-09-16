#!/bin/bash
# teleop_manual.sh —— 实体手柄手动控制链启动(2026-09-17,team 版,不碰厂家文件)
#
# 唯一指令链:
#   实体手柄(/dev/input/js*) → joy_node(/joy, autorepeat 20Hz)
#     → racecar_teleop(物理速度 Twist) ──remap──▶ /cmd_vel
#     → cmd_vel_safety(限幅 0.10/超时 0.5s 零速) → /car_cmd_vel
#     → racecar_driver_node(Run_car 内) → /dev/car → 电调/舵机
#
# 与 safety_nav_one.sh(导航链)的区别:不起 nav2 bringup,改起 joy+teleop。
# ⚠️ 手动模式唯一指令源承诺:本链不与 Nav2 / waypoint_cycle / 键盘控制 /
#    厂家 teleop 并存;厂家直通通道 /teleop_cmd_vel(下划线,绕过安全层)
#    本链不使用——注意与本链 /teleop/cmd_vel(斜杠)仅一字之差,勿混。
#
# 用法:  bash teleop_manual.sh [teleop参数yaml]     # 默认 ~/racecar_team/realcar_teleop.yaml
# 停止:  松开手柄使能键=立即零速;Ctrl+C=收链;
#        失控第一时间物理断电调开关(舵机供电≠电调断电,断电调舵机仍会动)
set -u
[ -f /opt/ros/humble/setup.bash ] && { set +u; source /opt/ros/humble/setup.bash; set -u; }
[ -f "$HOME/racecar/install/setup.bash" ] && { set +u; source "$HOME/racecar/install/setup.bash"; set -u; }
# team 侧 teleop 工作区(colcon build 后才有)
[ -f "$HOME/racecar_team/teleop_ws/install/setup.bash" ] && \
    { set +u; source "$HOME/racecar_team/teleop_ws/install/setup.bash"; set -u; }

TEAM="$HOME/racecar_team"
PARAMS="${1:-$TEAM/realcar_teleop.yaml}"

for f in "$TEAM/cmd_vel_safety.py" "$PARAMS"; do
  [ -f "$f" ] || { echo "缺少 $f,中止"; exit 2; }
done

# 前置自查 -------------------------------------------------------------
if pgrep -f "racecar_driver_node" > /dev/null 2>&1; then
  echo "!! 已有 driver 进程在跑,先收掉旧链再启动(避免双写 /dev/car)"; exit 2
fi
if ! ros2 pkg list 2>/dev/null | grep -q "^racecar_teleop$"; then
  echo "!! racecar_teleop 包未安装:先在 ~/racecar_team/teleop_ws colcon build"; exit 2
fi
if ! ros2 pkg list 2>/dev/null | grep -q "^joy$"; then
  echo "!! joy 包未安装:sudo apt install ros-humble-joy"; exit 2
fi
JSDEV=""
for d in /dev/input/js*; do
  [ -r "$d" ] && JSDEV="$d" && break
done
[ -z "$JSDEV" ] && { echo "!! 未找到可读手柄设备 /dev/input/js*(插好手柄;权限问题查 input 组)"; exit 2; }
echo "手柄设备: $JSDEV(若多个 js 设备,joy_node 自动取第一个;轴/按键务必先标定)"

# 收链 -----------------------------------------------------------------
SAFETY_PID=""; CAR_PID=""; JOY_PID=""; TELEOP_PID=""
terminate() {
  echo "收链..."
  for p in "$TELEOP_PID" "$JOY_PID" "$SAFETY_PID" "$CAR_PID"; do
    [ -n "$p" ] && kill "$p" 2>/dev/null
  done
  sleep 3
  # 兜底按名清理(字符类防 pkill 自匹配;[t]eleop_node 同时防误杀本脚本)
  pkill -f "[t]eleop_node" 2>/dev/null
  pkill -f "[j]oy_node" 2>/dev/null
  pkill -f "[R]un_car.launch" 2>/dev/null
  pkill -f "[r]acecar_driver_node" 2>/dev/null
  pkill -f "[c]md_vel_safety" 2>/dev/null
  pkill -f "ros2 launch" 2>/dev/null
  wait 2>/dev/null
  echo "已收链。确认车静止后再断电调开关。"
}
trap terminate SIGINT SIGTERM

echo "=== 手动链启动(teleop参数: $PARAMS)==="
echo "=== 前提自查:电调断开(首测无动力)?区域清空?手边有电调开关? ==="

python3 "$TEAM/cmd_vel_safety.py" > "$TEAM/safety_last.log" 2>&1 &
SAFETY_PID=$!
echo "[1/4] cmd_vel_safety 已起(日志 $TEAM/safety_last.log)"

setsid ros2 launch racecar Run_car.launch.py > "$TEAM/runcar_last.log" 2>&1 &
CAR_PID=$!
echo "[2/4] 传感器+底盘链(Run_car)已起,等 10s..."
sleep 10

ros2 run joy joy_node -p autorepeat_rate:=20.0 > "$TEAM/joy_last.log" 2>&1 &
JOY_PID=$!
echo "[3/4] joy_node 已起(autorepeat 20Hz,$JSDEV)"

# teleop 输出 remap 到 /cmd_vel 进安全层(物理速度,速度→PWM 只在 driver 一处)
ros2 run racecar_teleop teleop_node --ros-args \
    --params-file "$PARAMS" \
    -r /teleop/cmd_vel:=/cmd_vel > "$TEAM/teleop_last.log" 2>&1 &
TELEOP_PID=$!
echo "[4/4] racecar_teleop 已起(/teleop/cmd_vel → /cmd_vel → 安全层)"

echo
echo "链路自检(另开终端): ros2 topic echo /car_cmd_vel   # 应见限幅后速度"
echo "手柄标定(首次必做): ros2 topic echo /joy           # 逐轴逐键记录索引"
echo "操作: 先松一次使能键(解锁),再按住使能+推油门;松使能=立即零速"

wait
