#!/usr/bin/env bash
# 只读：列出某 cap 下 A–D 面板上所有 arm 跑的局与关键读数。
# 用法: ad_cap_status.sh <cap>      例如 ad_cap_status.sh 2.2
# 两批跑批目录都列（局名里带 panel / gz_official，一看就知道是哪把尺子）：
#   *_panel_track_*    = 仓内宿主 + 仓内实测 sim（新尺子）
#   *_gz_official_track_* = 旧队友 sim（旧尺子，只作参照）
# 这是逐行列清单，不做聚合，混列不会算错分母；要聚合判读用 ad_lap_verdict.py --runs。
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
CAP="${1:?用法: ad_cap_status.sh <cap>}"
PY=${PYTHON:-${VENV_PYTHON:?PYTHON 或 VENV_PYTHON 必填：仓库不带机器路径默认值}}

echo "现在 $(date +%H:%M:%S)  gzserver=$(pgrep -c gzserver || true)"
cd "$HERE/../runs" || exit 1
shopt -s nullglob
for d in gazebo_supervised_smoke_*CORRIDOR_arm*_cap${CAP}_*track_*_r*; do
  [ -d "$d" ] || continue          # 同名 .log / .json 也会被 glob 命中，跳过
  case "$d" in *_VOID*) echo "$(basename "$d")   (作废档,不计)"; continue ;; esac
  [ -f "$d/report.json" ] || { echo "$(basename "$d")   (无 report.json)"; continue; }
  "$PY" - "$d" <<'EOF'
import json, os, sys
d = sys.argv[1]
j = json.load(open(os.path.join(d, 'report.json'), encoding='utf-8'))
sr = j.get('scorer_result') or {}
pa = j.get('pairing') or {}
au = j.get('track_progress_audit') or {}
sp = sr.get('lap_splits_s') or []
print('%-72s %-8s cert=%-5s laps=%-3s hits=%-3s off=%-4s steady=%-7s wall=%-7s splits=%s'
      % (os.path.basename(d), j.get('status'), pa.get('certified'),
         sr.get('laps'), sr.get('cone_hits'), sr.get('off_track_samples'),
         ('%.2f' % sorted(sp[1:])[len(sp[1:]) // 2]) if len(sp) > 1 else '-',
         ('%.1f' % sr['elapsed_wall_s']) if sr.get('elapsed_wall_s') else '-',
         ','.join('%.1f' % x for x in sp)))
EOF
done
