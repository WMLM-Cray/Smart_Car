#!/bin/bash
# 一键重开仿真:清残留 -> 起 Gazebo + 车 + 赛道 -> 起策略 -> 使能。
# 双击 sim_start.bat 会调它。命令行里跑也行:bash 3Dsim/run_sim.sh
#
# 为什么每次都要整个重启而不是复位:Gazebo 的 Reset World(Ctrl+R)会把仿真
# 时间倒回 0,gazebo_ros2_control 从此不再注册 update 回调,车变砖且救不回来
# (详见 teleop/sim_reset.py 的说明)。想在跑坏之后重来一次,用 sim_reset.bat,
# 那个是传送,不动时间,不用重启。
#
# 注意别加 set -u:/opt/ros/humble/setup.bash 里引用了未定义的
# AMENT_TRACE_SETUP_FILES,开了就在 source 那一行直接退出。

WORLD=${WORLD:-$HOME/worlds/track_s3_sim.world}
# 低速 1.5m/s(s3):完赛率最高、几乎不撞锥。2.5m/s 的是 runs/policy_v25.npz,
# 圈速快 8 秒但在这条赛道上三次都没跑完一圈。
POLICY=${POLICY:-runs/policy_s3.npz}

# 仿真单独占一个 ROS 域,和实车(42)分开。默认的 0 是公共域:2026-09-20 车和
# 笔记本同时挂在同一个热点上,局域网里另一台机器的 Gazebo 被 DDS 自动发现,
# 实车 /scan 上凭空多出一个 2000 点的发布者,12.1Hz 的真帧里混进 8.1Hz 的假帧。
# 不报错、不告警,策略照收 —— 只能靠数点数才看得出来。
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43}

source /opt/ros/humble/setup.bash
source ~/hw_ws/install/setup.bash
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_DATABASE_URI="http://127.0.0.1:1/"   # 立刻被拒,不上网找模型
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA            # 不点名会挑中 Intel 核显,跑 Gazebo 掉设备

echo "清理残留…"
killall -9 gzserver gzclient 2>/dev/null
pkill -9 -f sim_bringup.launch.py 2>/dev/null
pkill -9 -f policy_node.py 2>/dev/null
sleep 4

echo "启动仿真:$WORLD"
ros2 launch racecar_sim sim_bringup.launch.py world:="$WORLD" &
launch_pid=$!

# 等 /joint_states 有数 —— 这是 gazebo_ros2_control 的 update 循环真的活着的唯一
# 可靠信号。控制器显示 active 不算数,时间被重置过时它也是 active 但没人调。
echo -n "等控制器就绪"
for _ in $(seq 45); do
    if timeout 3 ros2 topic echo /joint_states --once >/dev/null 2>&1; then
        echo " 好了"
        break
    fi
    echo -n .
    sleep 2
done

echo "启动策略:$POLICY"
cd ~/autocar && python3 teleop/policy_node.py "$POLICY" --sim &
sleep 8
ros2 topic pub --once /policy_enable std_msgs/Bool "{data: true}" >/dev/null 2>&1
echo "已使能。关掉这个窗口就停。"

wait $launch_pid
