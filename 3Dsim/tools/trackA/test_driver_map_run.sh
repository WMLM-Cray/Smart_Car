#!/bin/bash
# [仿真测试驾驶器] 整圈 + SLAM 建图 + RViz 截图 + map_saver(阶段3 第2/3步合证)
# 用法:bash tools/trackA/test_driver_map_run.sh [full|half]
set +u
ROUTE=${1:-full}
source /opt/ros/humble/setup.bash
source /home/lzx/hwcart_sim/hwcart_sim/install/setup.bash
export ROS_DOMAIN_ID=42
cd /home/lzx/hwcart_sim
mkdir -p docs/track

echo "=== [1/4] Xvfb(:95)==="
Xvfb :95 -screen 0 1400x900x24 >/tmp/xvfb2.log 2>&1 &
XVFB_PID=$!
sleep 2

echo "=== [2/4] slam_bringup(仿真+SLAM+RViz;不含键盘/手柄/Nav2)==="
DISPLAY=:95 setsid ros2 launch racecar_bringup_sim slam_bringup.launch.py \
    use_gui:=false world:=trackA.world \
    spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0 \
    >/tmp/driver_map.log 2>&1 &
DEMO_PGID=$!
echo "pgid=$DEMO_PGID  预热 75s…"
sleep 75

echo "=== [3/4] 仿真测试驾驶器 route=$ROUTE ==="
timeout 480 python3 tools/trackA/test_driver.py --route $ROUTE
RC=$?
import -display :95 -window root docs/track/test_driver_rviz_${ROUTE}.png 2>/dev/null \
  && echo shot_ok

echo "=== [4/4] map_saver 存整圈地图 ==="
timeout 30 ros2 run nav2_map_server map_saver_cli \
    -f /home/lzx/hwcart_sim/docs/track/trackA_map_full \
    && echo map_saved

echo "=== 清理 ==="
kill -KILL -- -$DEMO_PGID 2>/dev/null
kill $XVFB_PID 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
sleep 2
echo "rc=$RC"
ls -la docs/track/ | grep -E "test_driver_rviz|trackA_map_full|test_driver_${ROUTE}"
exit $RC
