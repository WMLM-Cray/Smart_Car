#!/usr/bin/env bash
# Negative acceptance test (验收第3步): prove the validator is not decorative.
# In a THROWAWAY COPY of the package, introduce one defect at a time, run
# tools/validate_assets.py and require that it
#   (a) exits non-zero, and
#   (b) prints a [FAIL] line naming the defect.
# Defects covered (one fresh copy each, nothing is left broken):
#   1. broken cone height   (frustum mount z 0.05 -> 0.08, total 0.66 m)
#   2. deleted required mesh (cone_blue STL removed)
#   3. wrong cone coordinate  (red_01 x 1.2 -> 1.9 in the world)
#   4. duplicate id           (blue_03 renamed to blue_02 in the world)
# Usage: bash test/test_acceptance_negative.sh [python-cmd]
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PY="${1:-}"
if [ -z "$PY" ]; then
  for c in python3 python; do
    if "$c" -c 'import sys' >/dev/null 2>&1; then PY="$c"; break; fi
  done
fi
[ -n "$PY" ] || { echo 'FAIL: no working python3/python found' >&2; exit 1; }

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

fresh_copy() {
  rm -rf "$TMP/pkg"
  (cd "$HERE/.." && tar -cf - --exclude='__pycache__' --exclude='*.pyc' \
     --exclude='delivery' --exclude='.git' racecar_test_assets) | tar -xf - -C "$TMP"
  mv "$TMP/racecar_test_assets" "$TMP/pkg"
}

run_validate() {
  "$PY" -B "$TMP/pkg/tools/validate_assets.py" >"$TMP/out.txt" 2>&1
}

FAILED=0

expect_reject() {  # label, needle-regex
  local label="$1" needle="$2" rc
  run_validate
  rc=$?
  if [ "$rc" -eq 0 ]; then
    echo "[FAIL] negative ($label): validator exited 0 - defect NOT caught"
    FAILED=1
  else
    echo "[PASS] negative ($label): validator exit code $rc (non-zero)"
  fi
  if grep -qE "$needle" "$TMP/out.txt"; then
    echo "[PASS] negative ($label): validator names the defect:"
    echo "        $(grep -m1 -E "$needle" "$TMP/out.txt")"
  else
    echo "[FAIL] negative ($label): no line matching '$needle' in output:"
    cat "$TMP/out.txt"
    FAILED=1
  fi
}

echo "== negative acceptance test: validator must catch broken assets =="
echo "== (defects are introduced in a throwaway copy only) =="

fresh_copy
run_validate
if [ $? -eq 0 ]; then
  echo '[PASS] baseline: unmodified copy passes (exit 0)'
else
  echo '[FAIL] baseline: unmodified copy does not pass - fix the package first'
  cat "$TMP/out.txt"
  exit 1
fi

# 1) broken cone height: raise the frustum mount 0.05 -> 0.08 (total 0.66 m)
fresh_copy
sed -i 's|<pose>0 0 0.05 0 0 0</pose>|<pose>0 0 0.08 0 0 0</pose>|g' \
  "$TMP/pkg/models/cone_red/model.sdf"
expect_reject 'broken cone height (cone_red total 0.66 m)' \
  'FAIL.*cone_red.*total height'

# 2) deleted required mesh: remove the cone_blue STL
fresh_copy
rm "$TMP/pkg/models/cone_blue/meshes/cone_frustum.stl"
expect_reject 'deleted required mesh (cone_blue STL)' \
  'FAIL.*cone_blue.*cone_frustum'

# 3) wrong cone coordinate: red_01 x 1.2 -> 1.9 in the world
fresh_copy
sed -i 's|<pose>1.2 0.75 0 0 0 0</pose>|<pose>1.9 0.75 0 0 0 0</pose>|' \
  "$TMP/pkg/worlds/sensor_test.world"
expect_reject 'wrong cone coordinate (red_01 x=1.9)' \
  'FAIL.*red_01.*task book'

# 4) duplicate id: rename blue_03 to blue_02 in the world
fresh_copy
sed -i 's|<name>blue_03</name>|<name>blue_02</name>|' \
  "$TMP/pkg/worlds/sensor_test.world"
expect_reject 'duplicate id (blue_02 twice)' \
  'FAIL.*duplicate'

# restore: a fresh (pristine) copy must pass again
fresh_copy
run_validate
if [ $? -eq 0 ]; then
  echo '[PASS] restore: pristine copy passes again (nothing left broken)'
else
  echo '[FAIL] restore: pristine copy does not pass'
  cat "$TMP/out.txt"
  FAILED=1
fi

echo
if [ "$FAILED" -ne 0 ]; then
  echo 'RESULT: FAILED'
  exit 1
fi
echo 'RESULT: PASSED'
exit 0
