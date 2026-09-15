#!/usr/bin/env bash
# Determinism: the same CSV input must regenerate byte-identical artifacts,
# and they must equal the checked-in files (guards against hand-edit drift).
# Usage: test/test_determinism.sh <python-binary>
set -u
PY="${1:?usage: test_determinism.sh <python-binary>}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
ok=0

"$PY" tools/generate_world.py --output "$TMP/a.world" >/dev/null || ok=1
"$PY" tools/generate_world.py --output "$TMP/b.world" >/dev/null || ok=1
if cmp -s "$TMP/a.world" "$TMP/b.world"; then
  echo '[PASS] world: two runs byte-identical'
else
  echo '[FAIL] world: two runs differ'
  ok=1
fi
if cmp -s "$TMP/a.world" worlds/sensor_test.world; then
  echo '[PASS] world: checked-in file matches regeneration'
else
  echo '[FAIL] worlds/sensor_test.world is stale - run: python3 tools/generate_world.py'
  ok=1
fi

"$PY" tools/generate_world.py --config config/demo_world.csv \
    --output "$TMP/demo.world" --world-name demo_world >/dev/null || ok=1
if cmp -s "$TMP/demo.world" worlds/demo_world.world; then
  echo '[PASS] world: checked-in demo_world matches regeneration'
else
  echo '[FAIL] worlds/demo_world.world is stale - run: python3 tools/generate_world.py --config config/demo_world.csv --output worlds/demo_world.world --world-name demo_world'
  ok=1
fi

"$PY" tools/generate_top_view.py --output "$TMP/a.svg" >/dev/null || ok=1
"$PY" tools/generate_top_view.py --output "$TMP/b.svg" >/dev/null || ok=1
if cmp -s "$TMP/a.svg" "$TMP/b.svg"; then
  echo '[PASS] svg: two runs identical'
else
  echo '[FAIL] svg: two runs differ'
  ok=1
fi
if cmp -s "$TMP/a.svg" docs/top_view.svg; then
  echo '[PASS] svg: checked-in file matches regeneration'
else
  echo '[FAIL] docs/top_view.svg is stale - run: python3 tools/generate_top_view.py'
  ok=1
fi

cp models/cone_red/meshes/cone_frustum.stl "$TMP/orig.stl"
"$PY" tools/generate_cone_mesh.py >/dev/null || ok=1
if cmp -s "$TMP/orig.stl" models/cone_red/meshes/cone_frustum.stl; then
  echo '[PASS] stl: mesh regeneration byte-identical (generator stable)'
else
  echo '[FAIL] stl: regeneration differs from checked-in mesh'
  ok=1
fi
if cmp -s models/cone_red/meshes/cone_frustum.stl models/cone_blue/meshes/cone_frustum.stl; then
  echo '[PASS] stl: cone_red and cone_blue meshes are identical'
else
  echo '[FAIL] stl: cone_red/cone_blue meshes differ'
  ok=1
fi

exit "$ok"
