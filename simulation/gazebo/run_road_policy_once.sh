#!/usr/bin/env bash
# 单实例:一次 Gazebo 验收(v2 策略节点 road_policy_node.py)。
#
# 与 run_s3_policy_once.sh 共享隔离方案和打分流程,但**策略节点那一行不同,而且
# 不是换个文件名那么简单** —— 两个节点的命令行不通用,见下面 POLICY_SPEED 处的说明。
# 隔离的四样东西一字不改(WORKER 变量决定):
#   1. ROS_DOMAIN_ID      —— 否则节点互相看得见,/cmd_vel 被别的实例收走
#   2. GAZEBO_MASTER_URI 端口 —— 否则第二个 gzserver 直接 bind 失败
#   3. world 文件名        —— 否则两个实例抢着 cp 同一个文件
#   4. /tmp 控制器 yaml    —— 由 launch 按域分文件
#
# 用法: bash simulation/gazebo/run_road_policy_once.sh <world-source> <model> <layout> <output>
set -eo pipefail

if [[ $# -ne 4 ]]; then
  echo "usage: $0 <world-source> <model> <layout> <output>" >&2
  exit 2
fi

WORLD_SOURCE=$1
MODEL=$2
LAYOUT=$3
OUTPUT=$4

if ! grep -q 'gazebo_ros_state' "$WORLD_SOURCE"; then
  echo "world source lacks gazebo_ros_state; use the *_sim.world variant so /model_states is available" >&2
  exit 2
fi

# v2 仿真节点没有防撞:road_runtime 里根本没有 ScanGuard(带 guard 的是队友分支
# feat/road-v2-car-node 上的实车节点 road_car_node.py,还没并进来)。所以 GUARD
# 无处可传。这不是小事 —— 我们所有 Gazebo 验收都是 guard=0.25 口径,而这条路上
# 没有 guard,跑出来的圈数/撞锥数**和 v1 的不可比**。不静默吞掉,设了就报错,
# 免得产出一批看着正常、实则换了口径的数字。
#
# 放在这里而不是起完仿真再判:gzserver 起来要一分多钟,参数错误不该等那么久。
if [[ -n "${GUARD:-}" ]]; then
  echo "v2 仿真节点没有防撞(GUARD=$GUARD 无处可传);" >&2
  echo "本轮的撞锥统计是纯物理碰撞,和 v1 的 guard=0.25 口径不可比。" >&2
  echo "要么不设 GUARD 并接受这个口径,要么改用 run_s3_policy_once.sh。" >&2
  exit 5
fi

REPO=$(cd "$(dirname "$0")/../.." && pwd)
WS="$REPO/simulation/gazebo"
LOG="${OUTPUT%.json}.launch.log"

WORKER=${WORKER:-}
if [[ -n "$WORKER" ]]; then
  ROS_DOMAIN_ID=$((43 + WORKER))
  GAZEBO_PORT=$((11345 + WORKER))
  WORLD_NAME="codex_road_eval_w${WORKER}.world"
else
  ROS_DOMAIN_ID=43
  GAZEBO_PORT=11345
  WORLD_NAME="codex_road_eval.world"
fi
WORLD="$WS/install/racecar_sim/share/racecar_sim/worlds/$WORLD_NAME"

source /opt/ros/humble/setup.bash || true
source "$WS/install/setup.bash" || true
export ROS_DOMAIN_ID
export GAZEBO_MASTER_URI="http://127.0.0.1:${GAZEBO_PORT}"
export GAZEBO_MODEL_DATABASE_URI=http://127.0.0.1:1/

cp "$WORLD_SOURCE" "$WORLD"

# 预检:端口必须空闲。残留的 gzserver 占着它,新实例会 bind 失败(exit 255),
# 而**车仍然会 spawn 进残留实例的 world** —— 跑出来的 JSON 完全像模像样,
# 只是跑的是另一条赛道。这是最贵的失败模式,必须在开跑前挡住。
if timeout 3 bash -c "</dev/tcp/127.0.0.1/${GAZEBO_PORT}" 2>/dev/null; then
  echo "GAZEBO_MASTER_URI 端口 ${GAZEBO_PORT} 已被占用(worker ${WORKER} 有残留实例);先清干净再跑" >&2
  exit 3
fi

# gzserver **不会**随 ros2 launch 一起死。kill $SIM_PID 只杀掉 launch 包装进程,
# gzserver/gzclient 被孤儿化后继续占着 GAZEBO_MASTER_URI 端口和 world 文件。
# 后果不是"多占一点内存",而是**下一批的数据被污染**:
#   下一批 cp 覆盖同一个 world 文件 -> 它的 gzserver bind 端口失败(exit 255)
#   -> 车 spawn 进**残留的旧 gzserver** 的 world 里(日志: Entity [racecar]
#      already exists),而 track_eval 拿新赛道 layout 去评分。
# 实测:arc 批之后,narrow/right/s_bend 全部 12 条跑的都是 arc 的赛道,
# 同 worker 槽位的三条轨迹相距 0.007m(纯采样噪声)。
#
# 按 **world 文件名**杀,不按进程名:WORLD_NAME 含 worker 号,是每个 worker
# 独有的,所以并行时不会误杀同批其他 worker(更不能按 gzserver 通用名杀,
# 那会杀掉队友域 43-50 的实例)。
cleanup() {
  kill "${ENABLE_PID:-}" "${POLICY_PID:-}" "${SIM_PID:-}" 2>/dev/null || true
  wait "${ENABLE_PID:-}" "${POLICY_PID:-}" "${SIM_PID:-}" 2>/dev/null || true
  pkill -f "gzserver.*${WORLD_NAME}" 2>/dev/null || true
  pkill -f "gzclient.*${WORLD_NAME}" 2>/dev/null || true
  # 等端口真正释放:gzserver 收 SIGTERM 到退出要几秒,不等就会把残留
  # 带进下一批。
  for _ in {1..20}; do
    pgrep -f "gzserver.*${WORLD_NAME}" >/dev/null 2>&1 || break
    sleep 1
  done
}
trap cleanup EXIT INT TERM

ros2 launch racecar_sim sim_bringup.launch.py \
  use_gui:=false world:="$WORLD_NAME" \
  spawn_x:=${SPAWN_X:-0} spawn_y:=${SPAWN_Y:-0} spawn_yaw:=${SPAWN_YAW:-0} >"$LOG" 2>&1 &
SIM_PID=$!

for _ in {1..45}; do
  if timeout 3 ros2 topic echo /joint_states --once >/dev/null 2>&1; then
    break
  fi
  sleep 2
done

# ⚠️ 本脚本跑的是 v2 仿真节点 vehicle/road_policy_node.py,它的命令行和 v1 的
# steer_policy_node.py **不通用**,移植时踩过三个坑,别照着 v1 脚本改:
#   * 它**没有 --sim**:节点本身只做仿真,并在启动时强制要求 ROS_DOMAIN_ID
#     已设置且不是实车域 42(下面已经 export 过,所以能过)。
#   * 速度上限的选项名是 **--speed-cap**。写 --speed 也能通,但那是 argparse 的
#     前缀缩写,不是真选项 —— 节点哪天加一个 --speed-xxx 就变成 "ambiguous
#     option" 直接 exit 2,是个会自己坏掉的写法。
#   * 它**没有防撞**,所以上面 GUARD 那段直接 exit 5。
#
# --speed-cap 在 v2 里是**速度上限(cap)**,不是固定油门:策略自己选 0.45~1.0 m/s。
POLICY_SPEED=${POLICY_SPEED:-1.0}
POLICY_LOG="${OUTPUT%.json}.policy.log"
python3 "$REPO/vehicle/road_policy_node.py" "$MODEL" \
  --speed-cap "$POLICY_SPEED" >"$POLICY_LOG" 2>&1 &
POLICY_PID=$!
python3 "$REPO/simulation/gazebo/hold_enable.py" >"/tmp/codex_road_enable_w${WORKER:-0}.log" 2>&1 &
ENABLE_PID=$!
sleep 3

# 策略节点**必须还活着**。它启动时会自己校验 --speed-cap 是否落在训练区间
# 0.45~1.0(road_policy.V_LO/V_HI),不合法就 SystemExit —— 但那是个后台进程,
# 脚本看不见,会照跑 120s 然后产出一份 laps=0 的完整 JSON。那份 JSON 和"模型跑
# 不动"长得一模一样,是最容易被当成模型缺陷的假信号。这里当场判死。
if ! kill -0 "$POLICY_PID" 2>/dev/null; then
  echo "策略节点已退出,本轮作废(不是模型的问题);最后几行日志:" >&2
  tail -5 "$POLICY_LOG" >&2
  exit 6
fi

python3 "$REPO/simulation/gazebo/track_eval.py" "$LAYOUT" \
  --seconds "${RUN_SECONDS:-120}" --out "$OUTPUT"

# 后检:本轮自己的 gzserver 必须活着。它中途死掉的话,track_eval 仍会把
# 已收到的 /model_states 写成一份完整 JSON,肉眼看不出来。
if ! pgrep -f "gzserver.*${WORLD_NAME}" >/dev/null 2>&1; then
  echo "本轮的 gzserver 中途退出,${OUTPUT} 不可信" >&2
  exit 4
fi
