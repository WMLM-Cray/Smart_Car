"""Generate worlds/sensor_test.world from config/sensor_test.csv.

The CSV is the single source of truth for object placement (task book §9).
The renderer is deterministic: same CSV -> byte-identical world file, so the
checked-in world can always be verified against the CSV (see check_world.py
and test/test_determinism.sh).

Validation errors abort with a non-zero exit code and leave the output file
untouched. Covered at minimum: duplicate id, non-finite/invalid coordinate,
unknown model type, malformed header, wrong field count.

Usage:
  python3 tools/generate_world.py                       # regenerate default
  python3 tools/generate_world.py --config X --output Y
"""
import argparse
import csv
import math
import re
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
PKG_ROOT = TOOLS.parent
MODELS_DIR = PKG_ROOT / 'models'
DEFAULT_CONFIG = PKG_ROOT / 'config' / 'sensor_test.csv'
DEFAULT_OUTPUT = PKG_ROOT / 'worlds' / 'sensor_test.world'
COLUMNS = ['id', 'model', 'x', 'y', 'z', 'yaw']
ID_RE = re.compile(r'[A-Za-z][A-Za-z0-9_]*')


class LayoutError(Exception):
    pass


def _known_model(name):
    d = MODELS_DIR / name
    return (d / 'model.config').is_file() and (d / 'model.sdf').is_file()


def load_layout(config_path):
    """Read and validate the layout CSV -> list of object dicts."""
    with open(config_path, encoding='utf-8-sig', newline='') as f:
        rows = [r for r in csv.reader(f)
                if r and not r[0].lstrip().startswith('#')]
    if not rows:
        raise LayoutError('empty config file')
    header = [c.strip() for c in rows[0]]
    if header != COLUMNS:
        raise LayoutError(f'bad header {header}, expected {COLUMNS}')
    objects, seen = [], set()
    for lineno, row in enumerate(rows[1:], start=2):
        if len(row) != len(COLUMNS):
            raise LayoutError(f'line {lineno}: expected {len(COLUMNS)} fields, '
                              f'got {len(row)}: {row}')
        oid, model = row[0].strip(), row[1].strip()
        if not ID_RE.fullmatch(oid):
            raise LayoutError(f'line {lineno}: invalid id {oid!r}')
        if oid in seen:
            raise LayoutError(f'line {lineno}: duplicate id {oid!r}')
        seen.add(oid)
        if not _known_model(model):
            raise LayoutError(f'line {lineno}: unknown model type {model!r} '
                              f'(no models/{model}/ with model.config + model.sdf)')
        try:
            x, y, z, yaw = (float(v) for v in row[2:6])
        except ValueError:
            raise LayoutError(f'line {lineno}: non-numeric coordinate in {row[2:6]}')
        if not all(math.isfinite(v) for v in (x, y, z, yaw)):
            raise LayoutError(f'line {lineno}: non-finite coordinate for {oid!r}')
        objects.append(dict(id=oid, model=model, x=x, y=y, z=z, yaw=yaw))
    if not objects:
        raise LayoutError('config contains no objects')
    return objects


def _fmt(v):
    """Compact fixed formatting; deterministic; no '-0'."""
    s = f'{v:.6f}'
    if s.startswith('-') and float(s) == 0.0:
        s = s[1:]
    s = s.rstrip('0').rstrip('.')
    return s or '0'


def _rel_label(path):
    try:
        return Path(path).resolve().relative_to(PKG_ROOT).as_posix()
    except ValueError:
        return str(path)


def render(objects, config_path, world_name='sensor_test'):
    includes = []
    for o in objects:
        includes.append(
            '    <include>\n'
            f'      <uri>model://{o["model"]}</uri>\n'
            f'      <name>{o["id"]}</name>\n'
            f'      <pose>{_fmt(o["x"])} {_fmt(o["y"])} {_fmt(o["z"])} '
            f'0 0 {_fmt(o["yaw"])}</pose>\n'
            '    </include>')
    body = '\n'.join(includes)
    return f'''<?xml version="1.0" ?>
<!-- {world_name}.world - GENERATED FILE, DO NOT EDIT BY HAND.
     Source of truth : {_rel_label(config_path)}
     Renderer        : tools/generate_world.py (deterministic)
     Regenerate with : python3 tools/generate_world.py, config {_rel_label(config_path)}, output worlds/{world_name}.world
     TEST FIXTURE for sensor validation - NOT an official A/B/C/D track.
     Units: metres and radians, Z up, ground top surface at z=0. -->
<sdf version="1.6">
  <world name="{world_name}">
    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.72 0.75 0.78 1</background>
      <shadows>true</shadows>
    </scene>
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <ambient>0.4 0.4 0.4 1</ambient>
      <direction>-0.45 0.2 -0.87</direction>
    </light>
    <!-- flat ground, top surface at z=0; defined inline (no external model or
         network dependency) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.30 0.30 0.30 1</ambient>
            <diffuse>0.42 0.42 0.42 1</diffuse>
            <specular>0 0 0 1</specular>
          </material>
        </visual>
      </link>
    </model>
    <!-- ==== objects below are generated from the CSV - do not edit ==== -->
{body}
  </world>
</sdf>
'''


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    ap.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    ap.add_argument('--world-name', default='sensor_test',
                    help='world name written into the <world name=...> element')
    args = ap.parse_args()
    try:
        objects = load_layout(args.config)
    except LayoutError as e:
        print(f'ERROR (config {args.config}): {e}', file=sys.stderr)
        return 1
    text = render(objects, args.config, args.world_name)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(text.encode('utf-8'))
    counts = {}
    for o in objects:
        counts[o['model']] = counts.get(o['model'], 0) + 1
    summary = ', '.join(f'{k} x{v}' for k, v in sorted(counts.items()))
    print(f'OK: {len(objects)} objects ({summary})')
    print(f'OK: wrote {_rel_label(args.output)} ({len(text.encode("utf-8"))} bytes)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
