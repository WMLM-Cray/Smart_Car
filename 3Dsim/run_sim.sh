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

# 路径解析交给 tools/_env.sh:工作区在 ~/hwcart_sim 还是 ~/hw_ws 两边不一样,
# 写死哪个另一边都跑不起来(2026-09-22 队友反馈)。域也在那里统一成 43。
source "$(dirname "$0")/tools/_env.sh"

# 赛道 world 是生成出来的,不入库(runs/* 被 .gitignore 挡着,只有生成它用的
# cones.csv + gaps.json 在库里)。先找仓库里生成的位置,再退回 ~/worlds。
# 都没有就照 docs/SIM_CAR_MODELS.md「把 fastsim 的赛道喂给 racecar_sim」那三步生成。
if [ -z "${WORLD:-}" ]; then
  for _w in "$REPO/runs/track_s3/track_s3_sim.world" "$HOME/worlds/track_s3_sim.world"; do
    [ -f "$_w" ] && WORLD=$_w && break
  done
fi
[ -n "${WORLD:-}" ] || { echo "找不到赛道 world,用 WORLD=<路径> 指定" >&2; exit 1; }
# 低速 1.5m/s(s3):完赛率最高、几乎不撞锥。2.5m/s 的是 runs/policy_v25.npz,
# 圈速快 8 秒但在这条赛道上三次都没跑完一圈。
POLICY=${POLICY:-runs/policy_s3.npz}

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
cd "$AUTOCAR" && python3 teleop/policy_node.py "$POLICY" --sim &
sleep 8
# /policy_enable 是**心跳**不是开关:policy_node 的 ENABLE_TIMEOUT 是 0.3s,
# 0.3 秒收不到就回中位。原来这里用 `ros2 topic pub --once`,只能让车抖一下
# (hold_enable.py 的文档字符串早写明了,但这里没照做)。2026-09-22 在赛道 A
# 上排查"车不动"时发现。
python3 "$AUTOCAR/teleop/hold_enable.py" &
hold_pid=$!
echo "已使能(心跳持续发)。关掉这个窗口就停。"

wait $launch_pid
kill $hold_pid 2>/dev/null
