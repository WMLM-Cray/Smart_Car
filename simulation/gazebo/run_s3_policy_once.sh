#!/usr/bin/env bash
# ROS Humble setup.bash reads an optional unset variable.
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

# 这里**没有 --scan-timeout**:v1 的 steer_policy_node.py 不认这个选项,而它的
# 位置参数是 `[a for a in sys.argv if not a.startswith('--')]` 过滤出来的 ——
# 传了它不会报错,但 `0.3` 会混进位置参数里变成多余的 argv。不致命(模型路径在最
# 前面,argv[0] 仍然对),但会让 SCAN_TIMEOUT 环境变量静默失效,以为调了其实没调。
# 真要调雷达超时,得先在节点上把这个选项补出来。
#
# 放在这里而不是起完仿真再判:gzserver 起来要一分多钟,参数错误不该等那么久。
if [[ -n "${SCAN_TIMEOUT:-}" ]]; then
  echo "v1 的 steer_policy_node.py 不认 --scan-timeout(SCAN_TIMEOUT=${SCAN_TIMEOUT} 无处可传);" >&2
  echo "静默忽略会让'调过超时'变成假的,所以这里直接报错。要调先给节点补上这个选项。" >&2
  exit 5
fi

REPO=$(cd "$(dirname "$0")/../.." && pwd)
WS="$REPO/simulation/gazebo"
LOG="${OUTPUT%.json}.launch.log"

# 并行验收:WORKER 给 0/1/2/... 时,每个 worker 用独立的 ROS 域、Gazebo 端口和
# world 文件名,这样多个 gzserver 能同时跑而互不干扰。不设就是原来的单实例行为
# (域 43、默认端口、固定 world 名),本机沿用不受影响。
#
# 隔离的四样东西,少一样就会互相污染:
#   1. ROS_DOMAIN_ID —— 否则节点互相看得见,/cmd_vel 被别的实例收走
#   2. GAZEBO_MASTER_URI 端口 —— 否则第二个 gzserver 直接 bind 失败
#   3. world 文件名 —— 否则两个实例抢着 cp 同一个文件,内容对不上
#   4. /tmp 的控制器 yaml —— 由 launch 按域分文件(见 sim_bringup.launch.py)
WORKER=${WORKER:-}
if [[ -n "$WORKER" ]]; then
  ROS_DOMAIN_ID=$((43 + WORKER))
  GAZEBO_PORT=$((11345 + WORKER))
  WORLD_NAME="codex_s3_eval_w${WORKER}.world"
else
  ROS_DOMAIN_ID=43
  GAZEBO_PORT=11345
  WORLD_NAME="codex_s3_eval.world"
fi
WORLD="$WS/install/racecar_sim/share/racecar_sim/worlds/$WORLD_NAME"

# set -eo pipefail 没有 -u,但 ROS 的 setup.bash 仍可能因内部未定义变量报错;
# 用 || true 兜住,并留在当前 shell(ros2 launch 需要这份环境)。
source /opt/ros/humble/setup.bash || true
source "$WS/install/setup.bash" || true
export ROS_DOMAIN_ID
export GAZEBO_MASTER_URI="http://127.0.0.1:${GAZEBO_PORT}"
export GAZEBO_MODEL_DATABASE_URI=http://127.0.0.1:1/

cp "$WORLD_SOURCE" "$WORLD"

cleanup() {
  kill "${ENABLE_PID:-}" "${POLICY_PID:-}" "${SIM_PID:-}" 2>/dev/null || true
  wait "${ENABLE_PID:-}" "${POLICY_PID:-}" "${SIM_PID:-}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# spawn 取 layout 的 start_pose(maps/ 原坐标,和 track_eval 判过线用同一个起点);
# 显式设 SPAWN_X/Y/YAW 可覆盖。
read -r START_X START_Y START_YAW < <(python3 -c '
import math, sys, yaml
s = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["start_pose"]
print(s["xy"][0], s["xy"][1], math.radians(s["yaw_deg"]))' "$LAYOUT")   || { echo "读不到 $LAYOUT 的 start_pose" >&2; exit 2; }


ros2 launch racecar_sim sim_bringup.launch.py \
  use_gui:=false world:="$WORLD_NAME" \
  spawn_x:=${SPAWN_X:-$START_X} spawn_y:=${SPAWN_Y:-$START_Y} spawn_yaw:=${SPAWN_YAW:-$START_YAW} >"$LOG" 2>&1 &
SIM_PID=$!

for _ in {1..45}; do
  if timeout 3 ros2 topic echo /joint_states --once >/dev/null 2>&1; then
    break
  fi
  sleep 2
done

POLICY_SPEED=${POLICY_SPEED:-0.6}
GUARD_ARG=""
if [[ -n "${GUARD:-}" ]]; then
  GUARD_ARG="--guard ${GUARD}"
fi
POLICY_LOG="${OUTPUT%.json}.policy.log"
python3 "$REPO/vehicle/steer_policy_node.py" "$MODEL" \
  --sim --speed "$POLICY_SPEED" $GUARD_ARG >"$POLICY_LOG" 2>&1 &
POLICY_PID=$!
python3 "$REPO/simulation/gazebo/hold_enable.py" >"/tmp/codex_s3_enable_w${WORKER:-0}.log" 2>&1 &
ENABLE_PID=$!
sleep 3

# 策略节点**必须还活着**。它启动时会自己校验参数(比如 --speed-loop 与 --sim
# 互斥、--guard 只能在仿真里用),不合法就 sys.exit —— 但那是个后台进程,脚本
# 看不见,会照跑 120s 然后产出一份 laps=0 的完整 JSON。那份 JSON 和"模型跑不动"
# 长得一模一样,是最容易被当成模型缺陷的假信号。这里当场判死。
if ! kill -0 "$POLICY_PID" 2>/dev/null; then
  echo "策略节点已退出,本轮作废(不是模型的问题);最后几行日志:" >&2
  tail -5 "$POLICY_LOG" >&2
  exit 6
fi

python3 "$REPO/simulation/gazebo/track_eval.py" "$LAYOUT" \
  --seconds "${RUN_SECONDS:-120}" --out "$OUTPUT"
