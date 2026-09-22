#!/bin/bash
# [AMCL定位验证] 阶段3 第三步编排:重启场景(无SLAM/无测试驾驶器)
#   → 载入已存地图 trackA_map_full → AMCL(初始位姿先给值) → 验收器
# 用法:bash tools/trackA/amcl_demo_run.sh
set +u
source /opt/ros/humble/setup.bash
source /home/lzx/hwcart_sim/hwcart_sim/install/setup.bash
export ROS_DOMAIN_ID=42
cd /home/lzx/hwcart_sim
mkdir -p docs/track

echo "=== [1/5] Xvfb(:95)==="
Xvfb :95 -screen 0 1400x900x24 >/tmp/xvfb3.log 2>&1 &
XVFB_PID=$!
sleep 2

echo "=== [2/5] localization_bringup(仿真+map_server+AMCL;不含SLAM/测试驾驶器)==="
DISPLAY=:95 setsid ros2 launch racecar_bringup_sim localization_bringup.launch.py \
    use_gui:=false world:=trackA.world \
    spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0 \
    >/tmp/amcl_loc.log 2>&1 &
LOC_PGID=$!
echo "pgid=$LOC_PGID  预热 75s(仿真+控制器+lifecycle 激活 map_server→amcl)…"
sleep 75
echo "--- 生命周期/地图载入关键行 ---"
grep -E "Creating bond|Configuring|Activating|activate|Map info|yaml" \
    /tmp/amcl_loc.log | tail -8

echo "=== [3/5] 初始定位截图(地图+激光+AMCL位姿+粒子云)==="
import -display :95 -window root docs/track/amcl_rviz_0_initial.png 2>/dev/null \
  && echo shot0_ok

echo "=== [4/5] AMCL 定位验收器(静止→前进2.5m→停→倒回→停;误差 vs 真值)==="
timeout 240 python3 tools/trackA/amcl_check.py
RC=$?
import -display :95 -window root docs/track/amcl_rviz_1_after.png 2>/dev/null \
  && echo shot1_ok

echo "=== [5/5] 清理 ==="
kill -KILL -- -$LOC_PGID 2>/dev/null
kill $XVFB_PID 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
sleep 2
echo "rc=$RC"
ls -la docs/track/ | grep -E "amcl_"
exit $RC
