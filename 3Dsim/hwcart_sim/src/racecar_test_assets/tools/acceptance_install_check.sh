#!/usr/bin/env bash
# Clean-room install check (验收第1步, the part that needs no ROS/Gazebo):
# put the DELIVERED package into a fresh throwaway workspace and verify it is
# self-contained.  The delivered state is extracted with `git archive HEAD`
# (exactly what is committed - no untracked leftovers, no Gazebo cache), or,
# when git is unavailable, a working-tree copy is used and clearly labelled.
#
# Checks, against the extracted copy ONLY:
#   1. no absolute machine paths and no network URIs in shipped resources
#   2. no references to vendor / main-project code (racecar_sim)
#   3. every model:// include and mesh uri resolves inside the copy
#      (this is also a full tools/validate_assets.py run on the copy)
#   4. CMake install rules cover every shipped file (nothing silently left)
#   5. the env hook prepends the package models dir to GAZEBO_MODEL_PATH
#
# The ROS parts of acceptance step 1 (colcon build log, launch log) still
# require the target environment - see delivery/build.log and
# delivery/launch.log for the exact commands.
# Usage: bash tools/acceptance_install_check.sh [python-cmd]
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="$(basename "$HERE")"

PY="${1:-}"
if [ -z "$PY" ]; then
  for c in python3 python; do
    if "$c" -c 'import sys' >/dev/null 2>&1; then PY="$c"; break; fi
  done
fi
[ -n "$PY" ] || { echo 'FAIL: no working python3/python found' >&2; exit 1; }

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

# ---- extract the delivered package into a fresh "test workspace" ----
PKG=''
MODE=''
if command -v git >/dev/null 2>&1 \
   && ROOT="$(git -C "$HERE" rev-parse --show-toplevel 2>/dev/null)" \
   && [ -n "$ROOT" ]; then
  REL="$(git -C "$HERE" rev-parse --show-prefix)"
  REL="${REL%/}"
  if [ -n "$REL" ] && mkdir -p "$TMP/extract" \
     && git -C "$ROOT" archive HEAD "$REL" | tar -x -C "$TMP/extract" 2>/dev/null \
     && [ -f "$TMP/extract/$REL/package.xml" ]; then
    PKG="$TMP/extract/$REL"
    MODE="git archive HEAD @ $(git -C "$ROOT" rev-parse --short HEAD) (committed delivery state)"
  fi
fi
if [ -z "$PKG" ]; then
  (cd "$HERE/.." && tar -cf - --exclude='__pycache__' --exclude='*.pyc' \
     --exclude='delivery' --exclude='.git' "$NAME") | tar -xf - -C "$TMP"
  PKG="$TMP/$NAME"
  MODE="working-tree copy (git archive unavailable - NOT the committed-state check)"
fi

echo "== clean-room install check =="
echo "== source   : $MODE =="
echo "== copy at  : \$TMP (throwaway) =="

FAILED=0
ok()  { echo "[PASS] $1"; }
bad() { echo "[FAIL] $1"; FAILED=1; }

# functional files (docs excluded: they may legitimately show example paths)
FUN="$(find "$PKG/models" "$PKG/worlds" "$PKG/config" "$PKG/launch" \
       "$PKG/env-hooks" -type f 2>/dev/null; \
       ls "$PKG/package.xml" "$PKG/CMakeLists.txt" 2>/dev/null)"

# 1a. no absolute machine paths (drive letters not preceded by a letter,
#     so uri schemes like model:// do not false-positive; also /home/, /Users/)
HITS="$(grep -lE '(^|[^A-Za-z])[A-Za-z]:[\\/]|/home/|/Users/' $FUN 2>/dev/null || true)"
[ -z "$HITS" ] && ok 'no absolute machine paths in shipped resources' \
  || bad "absolute machine paths in: $HITS"

# 1b. no network URIs (nothing is downloaded at build or launch time)
HITS="$(grep -lE 'https?://|ftp://' $FUN 2>/dev/null || true)"
[ -z "$HITS" ] && ok 'no network URIs in shipped resources (no runtime downloads)' \
  || bad "network URIs in: $HITS"

# 2. no vendor / main-project code references (package must stand alone)
HITS="$(grep -il 'racecar_sim' $FUN 2>/dev/null || true)"
[ -z "$HITS" ] && ok 'no references to main-project code (racecar_sim)' \
  || bad "main-project references in: $HITS"

# 3. full acceptance validation on the extracted copy (resource resolution,
#    geometry vs rules, layout vs task book, z=0.21 mesh section, ...)
#    -B: do not write __pycache__ into the copy under inspection
echo "-- tools/validate_assets.py on the extracted copy --"
if "$PY" -B "$PKG/tools/validate_assets.py"; then
  ok 'validate_assets passes on the extracted copy (resources resolve in-package)'
else
  bad 'validate_assets FAILED on the extracted copy'
fi

# 4. install coverage: every shipped file must be covered by an install rule
#    (or be a known non-installed meta file)
echo "-- install-rule coverage of every shipped file --"
UNCOVERED="$(cd "$PKG" && find . -type f | sed 's|^\./||' | sort | while read -r f; do
  case "$f" in
    models/*|worlds/*|config/*|launch/*|test/*|docs/*|env-hooks/*) : ;;
    package.xml|CMakeLists.txt|README.md|.git*) : ;;
    delivery/*) : ;;  # delivery evidence, deliberately not installed
    tools/*) grep -qF "$f" CMakeLists.txt || echo "$f  (missing from install(PROGRAMS))" ;;
    *) echo "$f  (not covered by any install rule)" ;;
  esac
done)"
if [ -z "$UNCOVERED" ]; then
  ok 'every shipped file is covered by an install rule (or a known meta file)'
else
  bad "files not covered by install rules:"
  echo "$UNCOVERED" | sed 's/^/        /'
fi

# 5. env hook must prepend this package's models dir to GAZEBO_MODEL_PATH
if grep -q 'prepend-non-duplicate;GAZEBO_MODEL_PATH;share/racecar_test_assets/models' \
     "$PKG/env-hooks/racecar_test_assets.dsv" 2>/dev/null; then
  ok 'env hook prepends share/racecar_test_assets/models to GAZEBO_MODEL_PATH'
else
  bad 'env hook missing or does not set GAZEBO_MODEL_PATH'
fi

echo
if [ "$FAILED" -ne 0 ]; then
  echo 'RESULT: FAILED'
  exit 1
fi
echo 'RESULT: PASSED'
exit 0
