#!/bin/bash
# 阶段3 第一步:可见二维建图演示(Xvfb 无头 RViz + 截图 + map_saver 存图)
# 在 WSL ubuntu2204-ros2 内运行:bash tools/trackA/slam_demo_run.sh
# 产物(~/hwcart_sim/docs/track/):slam_demo_rviz_*.png + trackA_map.pgm/.yaml
set +u
source /opt/ros/humble/setup.bash
source /home/lzx/hwcart_sim/hwcart_sim/install/setup.bash
export ROS_DOMAIN_ID=42
cd /home/lzx/hwcart_sim
mkdir -p docs/track

echo "=== [1/4] 起 Xvfb 虚拟屏(:95)==="
Xvfb :95 -screen 0 1400x900x24 >/tmp/xvfb.log 2>&1 &
XVFB_PID=$!
sleep 2

echo "=== [2/4] 起 slam_bringup(仿真+slam_toolbox+RViz@Xvfb)==="
DISPLAY=:95 setsid ros2 launch racecar_bringup_sim slam_bringup.launch.py \
    use_gui:=false world:=trackA.world \
    spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0 \
    >/tmp/slam_demo.log 2>&1 &
DEMO_PGID=$!
echo "demo pgid=$DEMO_PGID  等待 75s(仿真+控制器+SLAM+RViz 就绪)…"
sleep 75

echo "=== [3/4] 行驶前截图 → 驱动(复用 slam_smoke)→ 行驶后截图 ==="
import -display :95 -window root docs/track/slam_demo_rviz_0_start.png 2>/dev/null \
  && echo shot0_ok
python3 tools/trackA/slam_smoke.py | tee /tmp/slam_demo_drive.log
DRIVE_RC=${PIPESTATUS[0]}
import -display :95 -window root docs/track/slam_demo_rviz_1_after.png 2>/dev/null \
  && echo shot1_ok

echo "=== [4/4] map_saver 正式存图 ==="
timeout 30 ros2 run nav2_map_server map_saver_cli \
    -f /home/lzx/hwcart_sim/docs/track/trackA_map \
    && echo map_saved

echo "=== 清理 ==="
kill -KILL -- -$DEMO_PGID 2>/dev/null
kill $XVFB_PID 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
sleep 2
echo "slam_demo_drive_rc=$DRIVE_RC"
ls -la docs/track/ | grep -E 'slam_demo|trackA_map'
exit $DRIVE_RC
