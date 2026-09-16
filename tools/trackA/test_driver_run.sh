#!/bin/bash
# [仿真测试驾驶器] 编排:起 trackA 无头仿真 → 跑指定分段 → 清理
# 用法:bash tools/trackA/test_driver_run.sh [short|u_right|half|full]
set +u
ROUTE=${1:-short}
case $ROUTE in
  short)   TMO=200;; u_right) TMO=260;;
  half)    TMO=320;; full)    TMO=480;; *) echo "bad route"; exit 1;;
esac
source /opt/ros/humble/setup.bash
source /home/lzx/hwcart_sim/hwcart_sim/install/setup.bash
export ROS_DOMAIN_ID=42
cd /home/lzx/hwcart_sim

echo "=== [1/3] 起 trackA 无头仿真(不含 SLAM/RViz/键盘/Nav2)==="
setsid ros2 launch racecar_sim sim_bringup.launch.py \
    use_gui:=false world:=trackA.world \
    spawn_x:=11.0 spawn_y:=0.75 spawn_yaw:=0 \
    >/tmp/test_driver_sim.log 2>&1 &
SIM_PGID=$!
echo "sim pgid=$SIM_PGID  预热 45s(车/控制器就绪)…"
sleep 45

echo "=== [2/3] 仿真测试驾驶器 route=$ROUTE (timeout ${TMO}s) ==="
timeout $TMO python3 tools/trackA/test_driver.py --route $ROUTE
RC=$?
# 收尾零速(驾驶器正常路径已发;兜底)
timeout 5 ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist >/dev/null 2>&1

echo "=== [3/3] 清理 ==="
kill -KILL -- -$SIM_PGID 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
sleep 2
echo "test_driver_rc=$RC"
ls -la docs/track/test_driver_${ROUTE}_report.json 2>/dev/null
exit $RC
