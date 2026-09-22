#!/bin/bash
# [Nav2调试探针v2] G4亚毫秒规划失败定位:等激活后直接打 /compute_path_to_pose 二分
# 车在出生点(0,0,0):
#   P1 强制穿左U:  goal(-2.30,4.75,0°)   底道西行→BL弧→顶道,G4同款走廊反向
#   P2 右U(参照):  goal( 2.00,4.75,180°) 已知 G2 可规划
#   P3 底道西行:   goal(-2.00,0.00,180°) 纯直道
#   P4 左U中段:    goal(-2.60,2.00,270°)
#   P5 G4原目标:   goal( 0.00,0.00,  0°) 起点=终点附近(车恰在出生点,验同点位姿)
set +u
source /opt/ros/humble/setup.bash
source /home/lzx/hwcart_sim/hwcart_sim/install/setup.bash
export ROS_DOMAIN_ID=42

echo "=== 预检 ==="
pkill -INT -f "navigation_bringup.launch.py" 2>/dev/null; sleep 3
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f spawn_entity 2>/dev/null; sleep 1
pgrep -x gzserver && { echo "gzserver残留,放弃"; exit 1; }
ss -tln 2>/dev/null | grep -q 11345 && { echo "11345占用,放弃"; exit 1; }
echo "预检OK"

Xvfb :95 -screen 0 1400x900x24 >/tmp/xvfb5.log 2>&1 &
XV=$!
sleep 2
DISPLAY=:95 setsid ros2 launch racecar_bringup_sim navigation_bringup.launch.py \
    use_gui:=false world:=trackA.world spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0 \
    nav_rviz:=false >/tmp/plan_probe.log 2>&1 &

echo "=== 等待两侧管理器激活(最长240s) ==="
OK=0
for i in $(seq 1 120); do
  N=$(grep -c "Managed nodes are active" /tmp/plan_probe.log 2>/dev/null || echo 0)
  if [ "$N" -ge 2 ]; then OK=1; echo "第$((i*2))s: 双管理器激活"; break; fi
  sleep 2
done
if [ "$OK" != "1" ]; then
  echo "激活失败,放弃。错误行:"; grep -E "Failed|Aborting" /tmp/plan_probe.log | head -5
  kill -INT $(pgrep -f navigation_bringup.launch.py) 2>/dev/null; sleep 4
  pkill -9 -f gzserver 2>/dev/null; kill $XV 2>/dev/null; exit 1
fi
sleep 5

yaw2quat() {
  python3 -c "import math,sys;a=math.radians(float(sys.argv[1]));print(f'{math.sin(a/2):.6f} {math.cos(a/2):.6f}')" "$1"
}
try_plan() {  # 名 x y yaw度
  local NM="$1" X="$2" Y="$3" YAW="$4"
  read Z W <<< "$(yaw2quat "$YAW")"
  echo "--- $NM goal($X,$Y,${YAW}°) ---"
  local T0=$(date +%s.%N)
  timeout 40 ros2 action send_goal /compute_path_to_pose \
    nav2_msgs/action/ComputePathToPose \
    "{goal: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: $Z, w: $W}}}, planner_id: 'GridBased'}" 2>&1 \
    | tail -4
  local T1=$(date +%s.%N)
  echo "耗时: $(python3 -c "print(f'{$T1-$T0:.2f}s')")"
}

try_plan "P1强制穿左U"   -2.30 4.75 0
try_plan "P2右U参照"      2.00 4.75 180
try_plan "P3底道西行"    -2.00 0.00 180
try_plan "P4左U中段"     -2.60 2.00 270
try_plan "P5出生点0yaw"   0.00 0.00 0

echo "=== 规划器日志(本次探针新增) ==="
grep -E "planner_server-9" /tmp/plan_probe.log | grep -vE "Creating|Waiting|design" | tail -10

kill -INT $(pgrep -f "navigation_bringup.launch.py") 2>/dev/null; sleep 4
pkill -9 -f gzserver 2>/dev/null; kill $XV 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
echo "探针结束"
