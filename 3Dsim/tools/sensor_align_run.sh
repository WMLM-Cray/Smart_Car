#!/bin/bash
# [实车传感器对齐验证编排] 2026-09-16 任务书:仿真配置对齐实车
# (雷达 11.99Hz→仿真目标 12Hz/1667 点;IMU 200→100Hz)后的一次性验证:
#   预检 → trackA headless 仿真 → 稳定后 30s 仿真时间采样验收 → 清理
# 证据落 docs/validation/2026-09-16/(报告+仿真日志),不放 /tmp。
# 用法:bash tools/sensor_align_run.sh   (工作目录=~/hwcart_sim)
set +u
source /opt/ros/humble/setup.bash
source /home/lzx/hwcart_sim/hwcart_sim/install/setup.bash
export ROS_DOMAIN_ID=42
cd /home/lzx/hwcart_sim
OUT=docs/validation/2026-09-16
mkdir -p "$OUT"

echo "=== [0/3] 预检:清残留仿真进程 ==="
# 同 nav_demo_run.sh:孤儿 gzserver 占 11345 会让本轮静默失败
pkill -INT -f "sim_bringup.launch.py" 2>/dev/null
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

echo "=== [1/3] 起仿真(trackA headless,setsid 进程组)==="
setsid ros2 launch racecar_sim sim_bringup.launch.py \
    use_gui:=false world:=trackA.world \
    spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0 \
    >/tmp/sensor_sim.log 2>&1 &
SIM_PGID=$!
echo "sim pgid=$SIM_PGID"

echo "=== [2/3] 等 /scan 首条+再稳 10s(稳定运行后才开始采样)==="
WAITED=0
while [ $WAITED -lt 120 ]; do
  if timeout 3 ros2 topic echo /scan --field header.frame_id 2>/dev/null \
      | grep -m1 -q .; then
    echo "/scan 已有数据(${WAITED}s)"; break
  fi
  sleep 3; WAITED=$((WAITED+3))
done
if [ $WAITED -ge 120 ]; then
  echo "错误:120s 仍无 /scan,中止;/tmp/sensor_sim.log 尾部:"
  tail -5 /tmp/sensor_sim.log
  kill -KILL -- -$SIM_PGID 2>/dev/null
  cp /tmp/sensor_sim.log "$OUT/sensor_sim.log" 2>/dev/null
  exit 2
fi
sleep 10

echo "=== [3/3] 30s 仿真时间采样验收(时间戳频率+接收端频率+实时因子)==="
python3 tools/sensor_align_check.py --report "$OUT/sensor_align_report.json"
RC=$?

echo "=== 清理(进程组 KILL;gzserver 不理 SIGTERM)==="
kill -TERM -- -$SIM_PGID 2>/dev/null
sleep 3
kill -KILL -- -$SIM_PGID 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
sleep 2
cp /tmp/sensor_sim.log "$OUT/sensor_sim.log" 2>/dev/null
echo "sensor_align_overall_rc=$RC(报告:$OUT/sensor_align_report.json)"
exit $RC
