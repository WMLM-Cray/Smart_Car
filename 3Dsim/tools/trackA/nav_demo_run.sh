#!/bin/bash
# [Nav2单点导航验证] 阶段3 第四步编排:重启场景(无SLAM/无测试驾驶器)
#   → 已存地图+AMCL(第三步链) + Nav2(RPP/NavFn/速度平滑/极简BT)
#   → 验收器:短直道→右U弯→连续目标(顶道左行→左U复合回出生点=整圈)
# 用法:bash tools/trackA/nav_demo_run.sh
set +u
source /opt/ros/humble/setup.bash
source /home/lzx/hwcart_sim/hwcart_sim/install/setup.bash
export ROS_DOMAIN_ID=42
cd /home/lzx/hwcart_sim
mkdir -p docs/track docs/track/logs

echo "=== [0/5] 预检:清残留仿真进程 ==="
# Gazebo Master 绑 TCP 11345;上一轮若有孤儿 gzserver 存活,本轮 gzserver
# bind 失败 exit 255 → 车不生成 → AMCL/导航全瘫(2026-09-16 实测定位)。
# setsid 会 fork 新会话(PGID≠记录的 $!),按 PGID 击杀不可靠 → 按进程名清。
pkill -INT -f "navigation_bringup.launch.py" 2>/dev/null
pkill -INT -f "localization_bringup.launch.py" 2>/dev/null
sleep 3
pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null
pkill -9 -f spawn_entity 2>/dev/null
sleep 2
if pgrep -x gzserver >/dev/null; then
  echo "错误:gzserver 未清干净,请从 Windows 侧 wsl --shutdown 后重试"; exit 1
fi
if ss -tln 2>/dev/null | grep -q 11345; then
  echo "错误:11345 端口仍被占用"; exit 1
fi
echo "预检通过:无残留 gzserver,11345 空闲"

echo "=== [1/5] Xvfb(:95)==="
Xvfb :95 -screen 0 1400x900x24 >/tmp/xvfb4.log 2>&1 &
XVFB_PID=$!
sleep 2

echo "=== [2/5] navigation_bringup(仿真+AMCL+Nav2;不含SLAM/测试驾驶器)==="
DISPLAY=:95 setsid ros2 launch racecar_bringup_sim navigation_bringup.launch.py \
    use_gui:=false world:=trackA.world \
    spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0 \
    >/tmp/nav_run.log 2>&1 &
NAV_PGID=$!
echo "pgid=$NAV_PGID  预热(仿真+定位链激活+导航五节点激活+代价地图)…"
OK=0
for i in $(seq 1 90); do
  # grep -c 无匹配时退出码1,`|| echo 0` 会拼出两行"0"导致整数比较报错——
  # 改为先取值再判空
  N=$(grep -c "Managed nodes are active" /tmp/nav_run.log 2>/dev/null)
  [ -z "$N" ] && N=0
  if [ "$N" -ge 2 ]; then OK=1; echo "第$((i*2))s: 双管理器激活"; break; fi
  sleep 2
done
if [ "$OK" != "1" ]; then
  echo "错误:生命周期未双激活(启动竞态或节点失败),提前终止省时间。关键行:"
  grep -E "Failed|Aborting|died" /tmp/nav_run.log | head -6
  pkill -INT -f "navigation_bringup.launch.py" 2>/dev/null; sleep 4
  pkill -9 -f gzserver 2>/dev/null; kill $XVFB_PID 2>/dev/null
  exit 1
fi
sleep 20
echo "--- 生命周期/地图载入关键行 ---"
grep -E "Managed nodes|Failed|Received a [0-9]+ X" /tmp/nav_run.log | tail -6

echo "=== [3/5] 初始截图(地图+双代价地图+粒子云)==="
import -display :95 -window root docs/track/nav_rviz_0_initial.png 2>/dev/null \
  && echo shot0_ok

echo "=== [4/5] Nav2 验收器(短直道→右U→顶道左行→左U复合回出生点;整圈)==="
timeout 900 python3 tools/trackA/nav_check.py
RC=$?
import -display :95 -window root docs/track/nav_rviz_1_after.png 2>/dev/null \
  && echo shot1_ok

echo "=== [5/5] 清理 ==="
# 先 SIGINT 让 ros2 launch 有序收子进程,再点名补杀;不能用 kill -- -PGID
# (setsid fork 后实际 PGID ≠ $!,会扑空留孤儿 → 下轮 gzserver bind 255)
pkill -INT -f "navigation_bringup.launch.py" 2>/dev/null; sleep 5
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
pkill -9 -f spawn_entity 2>/dev/null
kill $XVFB_PID 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
sleep 2
echo "rc=$RC"
echo "--- 残留复查(应为空) ---"
pgrep -af "gzserver|navigation_bringup" | grep -v pgrep || echo "(已清空)"
ls -la docs/track/ | grep -E "nav_"
exit $RC
