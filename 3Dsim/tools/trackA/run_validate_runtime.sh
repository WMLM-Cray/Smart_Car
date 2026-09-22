#!/bin/bash
# trackA 运行时四项验证编排(2026-09-16;在 WSL ubuntu2204-ros2 内运行)
# 用法:bash tools/trackA/run_validate_runtime.sh   (工作目录=~/hwcart_sim)
set +u
source /opt/ros/humble/setup.bash
source /home/lzx/hwcart_sim/hwcart_sim/install/setup.bash
export ROS_DOMAIN_ID=42
cd /home/lzx/hwcart_sim

echo "=== 起仿真(trackA headless)==="
setsid ros2 launch racecar_sim sim_bringup.launch.py \
    use_gui:=false world:=trackA.world \
    spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0 \
    >/tmp/val_runtime_sim.log 2>&1 &
SIM_PGID=$!
echo "sim pgid=$SIM_PGID"

echo "=== 跑四项验证器(内部自带 90s spawn 等待)==="
timeout 300 python3 tools/trackA/validate_trackA_runtime.py
RC=$?

echo "=== 清理 ==="
kill -KILL -- -$SIM_PGID 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
sleep 2
echo "validate_runtime_overall_rc=$RC"
exit $RC
