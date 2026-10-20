#!/usr/bin/env bash
# Only this task's Python runner owns cleanup; no name-based process killing.
# TEAM_REPO / PYTHON 必须显式给出（外部路径与解释器不留机器默认值）。
set -eo pipefail
HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd -- "$HERE/.." && pwd)
TEAM_REPO=${TEAM_REPO:?TEAM_REPO 必填：队友 3Dsim 检出目录（本仓不猜同级目录）}
PYTHON=${PYTHON:?PYTHON 必填：跑仿真用的解释器（本仓不带机器路径默认值）}
# 只给一个解释器：探针内部要 VENV_PYTHON（并据此反推 VENV_ROOT 做"当前就在这个 venv 里"的校验）。
export VENV_PYTHON=${VENV_PYTHON:-$PYTHON}
source /opt/ros/humble/setup.bash
source "$TEAM_REPO/3Dsim/hwcart_sim/install/setup.bash"
set -u
export ROS_DOMAIN_ID=63
export GAZEBO_MASTER_URI=http://127.0.0.1:11365
export GAZEBO_MODEL_DATABASE_URI=http://127.0.0.1:1/
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1
cd "$ROOT"
exec "$PYTHON" -u -m evaluation.gazebo_braking_probe "$@"
