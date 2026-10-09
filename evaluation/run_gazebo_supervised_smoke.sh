#!/usr/bin/env bash
# Label 20261005 is an artifact identifier, not a claim about today's date.
#
# 两个环境变量必须显式给出（外部路径不留默认值）：
#   TEAM_REPO  队友 3Dsim 检出目录（dev suite + 打分工具 + 遥操作节点在那里）
#   PYTHON     跑仿真用的解释器（含 ROS humble / gazebo 依赖）
#
# 只用给一个解释器：runner 内部要 VENV_PYTHON（它会按 <root>/bin/python 推回 VENV_ROOT，
# 并校验"当前解释器就是这个 venv 的 python"），这里默认就用外层 PYTHON。
# 要换成别的 venv 时，显式给 VENV_ROOT / VENV_PYTHON 覆盖即可。
#
# 车模/世界用**本仓**的 simulation/gazebo 包（实测几何 0.35/0.145/0.28），不再用队友那份旧几何
# （0.33/0.07/0.21）；两份的 sim_wiring.py 逐字节相同，所以换源只换几何不换接线。
# 需要跑旧几何时把 SIM_SETUP 指到队友的 install/setup.bash 即可。
#
# 若 --policy-entry 指向本仓 arms/corridor_arm.py，还必须给 CORRIDOR_ARM（臂身份，见该文件第 60 行：
# 不设就是 D 臂）。runner 会拒绝这种"静默换臂"，因为它跑出来的数字不属于 arms/VERSIONS.md 的口径。
# 从 evaluation/corridor_arm_run.sh 起跑就不用管：它把第一个参数 export 成 CORRIDOR_ARM。
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd -- "$HERE/.." && pwd)
TEAM_REPO=${TEAM_REPO:?TEAM_REPO 必填：队友 3Dsim 检出目录（本仓不猜同级目录）}
PYTHON=${PYTHON:?PYTHON 必填：跑仿真用的解释器（本仓不带机器路径默认值）}
export VENV_PYTHON=${VENV_PYTHON:-$PYTHON}
SIM_SETUP=${SIM_SETUP:-$ROOT/simulation/gazebo/install/setup.bash}
# 导出给 runner：它据此区分"本仓 sim"与"显式指定的队友旧几何 sim"（没导出就按"必须本仓"判——
# colcon build 失败会留下一个 stub install/setup.bash，这时的整局会静默换成队友旧车模）。
export SIM_SETUP
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1
[[ -x "$PYTHON" ]] || { echo "Specified interpreter missing: $PYTHON" >&2; exit 2; }
[[ -d "$TEAM_REPO" ]] || { echo "TEAM_REPO 不存在: $TEAM_REPO" >&2; exit 2; }
[[ -f "$SIM_SETUP" ]] || { echo "缺少 $SIM_SETUP：先 source /opt/ros/humble/setup.bash && cd simulation/gazebo && colcon build" >&2; exit 2; }
# 注意：build 失败也会留下 stub install/setup.bash（上面那道检查因此拦不住），runner 会再核对
# racecar_sim 的实际来源；这里先给一句可操作的提示，免得看到的是更难懂的报错。
if [[ ! -d "$ROOT/simulation/gazebo/install/racecar_sim" && "$SIM_SETUP" == "$ROOT/simulation/gazebo/install/setup.bash" ]]; then
  echo "本仓 sim 未 build 成功（install/racecar_sim 不存在）：先 source /opt/ros/humble/setup.bash && cd simulation/gazebo && colcon build；要故意跑队友旧几何 sim 就显式 SIM_SETUP=<队友>/install/setup.bash" >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
# 队友那份 install 先 source（提供 hwcart_sim 里的控制器等其余包），再用本仓的 racecar_sim 覆盖车模。
if [[ -f "$TEAM_REPO/3Dsim/hwcart_sim/install/setup.bash" ]]; then
  source "$TEAM_REPO/3Dsim/hwcart_sim/install/setup.bash"
fi
source "$SIM_SETUP"
set -u
exec "$PYTHON" "$HERE/gazebo_supervised_runner.py" --team-repo "$TEAM_REPO" "$@"
