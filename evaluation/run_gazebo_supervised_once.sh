#!/usr/bin/env bash
set -euo pipefail
echo 'REFUSED: retired launcher uses unsafe name/domain cleanup and unmatched scoring windows.' >&2
echo 'Use evaluation/run_gazebo_supervised_smoke.sh --arm pair --worker 21 --out <fresh owned runs path>.' >&2
exit 2
