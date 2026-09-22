"""Consistency check: a generated world must match its layout CSV.

Catches hand-edits that drift away from the CSV source of truth. Also
verifies structural requirements: SDF 1.6, world name, inline ground plane
(top surface z=0), sun light, and that every <include> resolves to a model
directory shipped in this package. Exits non-zero on any mismatch.

Run:
  python3 tools/check_world.py                                   # sensor_test
  python3 tools/check_world.py --config config/demo_world.csv \
      --output worlds/demo_world.world --world-name demo_world   # demo scene
"""
import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import generate_world as gw  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--config', type=Path, default=gw.DEFAULT_CONFIG)
    ap.add_argument('--output', type=Path, default=gw.DEFAULT_OUTPUT)
    ap.add_argument('--world-name', default=None,
                    help='expected world name (default: output filename stem)')
    args = ap.parse_args()
    world_name = args.world_name or args.output.stem

    failed = []

    def check(name, ok, detail=''):
        print(f'[{"PASS" if ok else "FAIL"}] {name}' + (f'  ({detail})' if detail else ''))
        if not ok:
            failed.append(name)

    try:
        objects = gw.load_layout(args.config)
    except gw.LayoutError as e:
        print(f'[FAIL] {args.config} is invalid: {e}')
        return 1
    check(f'{args.config.name} valid', True, f'{len(objects)} objects')

    try:
        root = ET.parse(args.output).getroot()
    except (ET.ParseError, OSError) as e:
        print(f'[FAIL] {args.output} does not parse: {e}')
        return 1
    check('world parses as XML, root <sdf>', root.tag == 'sdf')
    check('sdf version 1.6 (Gazebo Classic 11)', root.get('version') == '1.6')
    world = root.find('world')
    check(f'world name = {world_name}',
          world is not None and world.get('name') == world_name)
    check('inline ground_plane model present (top surface z=0)',
          world.find("./model[@name='ground_plane']") is not None)
    check('sun directional light present',
          world.find("./light[@name='sun']") is not None)

    includes = world.findall('include')
    csv_by_id = {o['id']: o for o in objects}
    world_names = [inc.findtext('name') for inc in includes]
    check('no duplicate object names in world',
          len(world_names) == len(set(world_names)))
    check('world object count == CSV object count',
          len(includes) == len(objects), f'{len(includes)} vs {len(objects)}')

    for inc in includes:
        name = inc.findtext('name')
        uri = inc.findtext('uri')
        pose = [float(v) for v in inc.find('pose').text.split()]
        o = csv_by_id.get(name)
        if o is None:
            check(f'{name}: present in CSV', False, 'no such id in CSV')
            continue
        ok = (uri == f'model://{o["model"]}'
              and abs(pose[0] - o['x']) < 1e-9
              and abs(pose[1] - o['y']) < 1e-9
              and abs(pose[2] - o['z']) < 1e-9
              and pose[3] == 0.0 and pose[4] == 0.0
              and abs(pose[5] - o['yaw']) < 1e-9)
        check(f'{name}: model+pose match CSV', ok,
              f'{uri} @ ({pose[0]:g}, {pose[1]:g}, {pose[2]:g}, yaw {pose[5]:g})')
        check(f'{name}: model dir shipped in package',
              (gw.MODELS_DIR / o['model'] / 'model.sdf').is_file())

    missing = set(csv_by_id) - set(world_names)
    check('every CSV object present in world', not missing,
          f'missing: {sorted(missing)}' if missing else 'all present')

    if failed:
        print(f'== FAILED: {len(failed)} problem(s): {failed} ==')
        return 1
    print(f'== PASSED: world matches CSV ({world_name}) ==')
    return 0


if __name__ == '__main__':
    sys.exit(main())
