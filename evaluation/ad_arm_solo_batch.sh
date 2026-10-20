#!/usr/bin/env bash
# ad_arm_solo_batch.sh <ARM> <CAP> [REPS] [tracks...]
#
# 单臂批次:等 gzserver 归零后，在官方 A–D 面板上并发跑一个臂(A 道的 W21 / C 道的 W22)，
# 默认 tracks = A C、REPS = 3。用于"参考臂的读数已经在账上、只缺新臂那一半"的情形
# (例如 cap2.0 上 A 臂已有 4 局认证读数，只缺 N 臂)。
#
# 需要严格两臂交错时用 ad_arm_pair_batch.sh；这里不做交错，所以读数的可比性依赖
# 同一受监督仪器 + 同 cap + 逐局 pairing.certified。
set -uo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd) || exit 9
cd "$ROOT" || exit 9
PYTHON=${PYTHON:?PYTHON 必填：跑仿真用的解释器（仓库不带机器路径默认值）}
ARM=${1:?usage: ad_arm_solo_batch.sh <ARM> <CAP> [REPS] [tracks...]}
CAP=${2:?}
REPS=${3:-3}
shift 3 2>/dev/null || shift $#
TRACKS=${*:-A C}
WAIT_MAX=${WAIT_MAX:-40}
BASE=$ROOT/evaluation/panel
# 判读只报本批次实际跑的道。
TRACK_CSV=$(echo "$TRACKS" | tr ' ' ',')

echo "# 等 gzserver 归零 $(date --iso-8601=seconds)"
ok=0
for i in $(seq 1 "$WAIT_MAX"); do
  n=$(pgrep -c gzserver || true)
  if [ "${n:-0}" -eq 0 ]; then ok=1; break; fi
  echo "  忙 gzserver=${n} 第 ${i}/${WAIT_MAX} 次，15s 后再看 $(date +%H:%M:%S)"
  sleep 15
done
if [ "$ok" -ne 1 ]; then echo "# ABORT: gzserver 一直没归零，不叠局"; exit 7; fi

pids=""
i=0
for T in $TRACKS; do
  i=$((i + 1))
  W=$((20 + i))
  [ "$W" -gt 22 ] && { echo "# 只允许 2 路并发，跳过 track_$T"; continue; }
  echo "# START worker=$W arm=$ARM cap=$CAP track_$T reps=1..$REPS $(date --iso-8601=seconds)"
  WORKER=$W bash "$ROOT/evaluation/corridor_arm_run.sh" "$ARM" "$CAP" "$BASE/track_$T" "1:$REPS" \
    > "runs/_solo_${ARM}_cap${CAP}_track${T}.log" 2>&1 &
  pids="$pids $!"
done
rcs=""
for p in $pids; do wait "$p"; rcs="$rcs $?"; done
echo "# rc:$rcs  $(date --iso-8601=seconds)"
for T in $TRACKS; do
  f="runs/_solo_${ARM}_cap${CAP}_track${T}.log"
  [ -f "$f" ] && { echo "== $f =="; tail -6 "$f"; }
done

echo "== 状态 =="
bash "$ROOT/evaluation/ad_cap_status.sh" "$CAP"
echo "== 判读（A vs $ARM）=="
"$PYTHON" "$ROOT/evaluation/ad_lap_verdict.py" "$CAP" --tracks "$TRACK_CSV" --arms "A,$ARM" || true
echo "# ad_arm_solo_batch arm=$ARM cap=$CAP COMPLETE $(date --iso-8601=seconds)"
