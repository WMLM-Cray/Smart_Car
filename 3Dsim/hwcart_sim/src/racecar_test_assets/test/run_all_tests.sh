#!/usr/bin/env bash
# Run every racecar_test_assets check. Exits non-zero if any check fails.
# The gazebo-dependent smoke test is SKIPped (not failed) when gzserver is
# absent, so the geometry/consistency suite stays runnable anywhere.
# Usage: bash test/run_all_tests.sh
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

PY=''
for c in python3 python; do
  if "$c" -c 'import sys' >/dev/null 2>&1; then PY="$c"; break; fi
done
if [ -z "$PY" ]; then
  echo 'FAIL: no working python3/python interpreter found' >&2
  exit 1
fi
echo "using python: $PY ($("$PY" --version 2>&1))"

FAILED=0
run() {
  local name="$1"
  shift
  echo
  echo "===== $name ====="
  if "$@"; then
    echo "PASS: $name"
  else
    echo "FAIL: $name"
    FAILED=1
  fi
}

run 'model dimension check'    "$PY" tools/check_models.py
run 'world/csv consistency'    "$PY" tools/check_world.py
run 'demo world/csv consistency' \
    "$PY" tools/check_world.py --config config/demo_world.csv \
        --output worlds/demo_world.world --world-name demo_world
run 'layout spec (task book)'  "$PY" test/test_layout_spec.py
run 'input validation errors'  "$PY" test/test_validation_errors.py
run 'determinism'              bash test/test_determinism.sh "$PY"
run 'acceptance validator'     "$PY" tools/validate_assets.py
run 'negative acceptance test' bash test/test_acceptance_negative.sh "$PY"
run 'clean-room install check' bash tools/acceptance_install_check.sh "$PY"

echo
echo '===== gazebo load smoke (positive + negative) ====='
if bash test/test_gazebo_smoke.sh; then
  echo 'PASS: gazebo load smoke'
else
  rc=$?
  if [ "$rc" -eq 2 ]; then
    echo 'SKIP: gazebo load smoke (gzserver not available on this machine)'
  else
    echo 'FAIL: gazebo load smoke'
    FAILED=1
  fi
fi

echo
echo '===== z=0.21 cone cross-section (informational) ====='
"$PY" tools/cone_section.py

echo
if [ "$FAILED" -ne 0 ]; then
  echo 'RESULT: FAILED'
  exit 1
fi
echo 'RESULT: PASSED'
exit 0
