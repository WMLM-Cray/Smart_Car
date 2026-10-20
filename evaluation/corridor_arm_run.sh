#!/usr/bin/env bash
# corridor_arm_v1 generic runner:
#   corridor_arm_run.sh <A|B|C|D|E|F|G|H|I|J|K|L|N|P|R|S|T|U|V|W|Y> <cap> <track-name> <reps>
#
# <reps> is either a count ("3" -> reps 1..3) or a range ("2:3" -> reps 2..3), so a
# campaign can resume after a rep has already been banked.
#
# Same orchestration as capladder_run.sh (baseline arm, unmodified production stack), with one
# difference: the policy entry point is arms/corridor_arm.py instead of the production policy
# node.  Everything the arm does is a runtime monkey-patch of the node class -- no production
# file is written to, and vehicle/road_policy_node.py / vehicle/scan_guard.py / common/* keep
# their exact bytes.  Which arm is live is carried in CORRIDOR_ARM, which child_env() inherits
# from os.environ.  Y = W 的层配置 + 仓内宿主（arms/arm_host.py，见该文件文档字符串）。
#
# Output dirs are never reused; an existing dir or log is a hard refuse.
set -uo pipefail
# Repo root = this script's parent dir; nothing below is a machine path.
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd) || exit 9
cd "$ROOT" || exit 9
# 解释器与队友仓检出位置都必须显式给出：仓库不带机器路径默认值。
export PYTHON=${PYTHON:?PYTHON 必填：跑仿真用的解释器（含 ROS humble / gazebo 依赖）}
export TEAM_REPO=${TEAM_REPO:?TEAM_REPO 必填：队友 3Dsim 检出目录（拥有 dev suite 与打分工具）}
ARM=${1:?usage: corridor_arm_run.sh <A|B|C|D|E|F|G|H|I|J|K|L|N|P|R|S|T|U|V|W|Y> <cap> <track> <reps>}
CAP=${2:?}
TRACK_NAME=${3:?}
REPS=${4:?}
case "$ARM" in A|B|C|D|E|F|G|H|I|J|K|L|N|P|R|S|T|U|V|W|Y) ;; *) echo "bad arm: $ARM"; exit 5 ;; esac
# <reps> is a count or a FIRST:LAST range.
case "$REPS" in
  *:*) REP_FIRST=${REPS%%:*}; REP_LAST=${REPS##*:} ;;
  *)   REP_FIRST=1; REP_LAST=$REPS ;;
esac
export CORRIDOR_ARM=$ARM
ENTRY=$ROOT/arms/corridor_arm.py
[ -f "$ENTRY" ] || { echo "no entry: $ENTRY"; exit 6; }
if [ "${TRACK_NAME#/}" != "$TRACK_NAME" ]; then
  # 绝对路径：用于部署验收赛道（本仓 evaluation/panel/track_A..D，或队友仓的原始副本）
  TRACK=$TRACK_NAME
  OUT_TAG=$(basename "$(dirname "$TRACK")")_$(basename "$TRACK")
else
  TRACK=$TEAM_REPO/runs/gen_suite_20260926/$TRACK_NAME
  OUT_TAG=$TRACK_NAME
fi
[ -d "$TRACK" ] || { echo "no such track: $TRACK"; exit 4; }
for REP in $(seq "$REP_FIRST" "$REP_LAST"); do
  # ARM_RUN_TAG（可选，默认空）：给输出目录名加后缀，用于在**同一条白名单赛道**上跑
  # 实验臂而不撞已有基线目录（赛道白名单只认 evaluation/panel 与队友 dev-suite，
  # 所以实验无法靠换赛道目录名错开）。默认空 = 与既有命名逐字相同。
  OUT=$ROOT/runs/gazebo_supervised_smoke_20261005_CORRIDOR_arm${ARM}_cap${CAP}_${OUT_TAG}${ARM_RUN_TAG:-}_r${REP}
  LOG=${OUT}.log
  if [ -e "$OUT" ] || [ -e "$LOG" ]; then echo "REFUSE: $OUT exists"; exit 3; fi
  rc=-1
  {
    printf 'CORRIDOR arm=%s cap=%s track=%s rep=%s START ' "$ARM" "$CAP" "$TRACK_NAME" "$REP"; date --iso-8601=seconds
    bash "$ROOT/evaluation/run_gazebo_supervised_smoke.sh" \
      --arm baseline --cap "$CAP" --track "$TRACK" --out "$OUT" \
      --policy-entry "$ENTRY" --preflight-only
    echo "preflight rc=$?"
    bash "$ROOT/evaluation/run_gazebo_supervised_smoke.sh" \
      --arm baseline --cap "$CAP" --track "$TRACK" --out "$OUT" \
      --policy-entry "$ENTRY"
    rc=$?
    printf 'CORRIDOR arm=%s cap=%s track=%s rep=%s END rc=%s ' "$ARM" "$CAP" "$TRACK_NAME" "$REP"; date --iso-8601=seconds
  } >"$LOG" 2>&1
  echo "rep $REP done rc=$rc"
done
echo "ARM $ARM cap=$CAP track=$TRACK_NAME COMPLETE"
