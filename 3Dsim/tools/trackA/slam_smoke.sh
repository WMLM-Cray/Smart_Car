#!/bin/bash
# SLAM 短距离建图冒烟编排(2026-09-16,阶段3 前置;在 WSL ubuntu2204-ros2 内运行)
# 用法:bash tools/trackA/slam_smoke.sh   (工作目录=~/hwcart_sim)
# 流程:trackA 仿真(headless)→ slam_toolbox online_async → 驱动+验图 → 清理
# 只读验证:不改冻结赛道/车辆几何/任何参数。
set +u
source /opt/ros/humble/setup.bash
source /home/lzx/hwcart_sim/hwcart_sim/install/setup.bash
export ROS_DOMAIN_ID=42
cd /home/lzx/hwcart_sim
mkdir -p docs/track

echo "=== [1/3] 起仿真(trackA headless,setsid 进程组)==="
setsid ros2 launch racecar_sim sim_bringup.launch.py \
    use_gui:=false world:=trackA.world \
    spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0 \
    >/tmp/slam_sim.log 2>&1 &
SIM_PGID=$!
echo "sim pgid=$SIM_PGID"

echo "=== [2/3] 等 60s 让 spawn+控制器就绪,再起 slam_toolbox ==="
sleep 60
setsid ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=true \
    >/tmp/slam_tb.log 2>&1 &
SLAM_PGID=$!
echo "slam pgid=$SLAM_PGID"
sleep 10

echo "=== [3/3] 驱动+验图 ==="
python3 tools/trackA/slam_smoke.py
RC=$?

echo "=== 清理(进程组 KILL;gzserver 不理 SIGTERM)==="
kill -TERM -- -$SLAM_PGID 2>/dev/null
sleep 3
kill -KILL -- -$SIM_PGID 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
sleep 2
echo "slam_smoke_overall_rc=$RC"
exit $RC
