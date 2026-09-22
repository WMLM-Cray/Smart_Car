"""CSV -> portable SDF prototype. Cone boxes are explicit test proxies.
This generates no official A-D geometry and does not implement visual recognition.
"""
import argparse, csv, json, math, random, re
from pathlib import Path
import xml.etree.ElementTree as ET

def load_cones(path):
    with open(path, encoding='utf-8-sig', newline='') as f:
        rows = list(csv.DictReader(f))
    ids = set()
    for r in rows:
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', r['id']) or r['id'] in ids:
            raise ValueError('invalid or duplicate cone id')
        ids.add(r['id'])
        if r['color'] not in ('red', 'blue'):
            raise ValueError('cone color must be red/blue')
        r['x'], r['y'] = float(r['x']), float(r['y'])
        if not all(math.isfinite(r[k]) for k in ('x', 'y')):
            raise ValueError('nonfinite coordinate')
    return rows

def build(cones, candidates, seed, board_thickness, boards=3):
    """boards: how many blue boards to drop into vetted gaps.

    0 is for fixed official layouts: track A's spec has no boards, and its cone
    spacing (1.027 m on the straights, 0.917 m on E8) leaves only two gaps in
    the [1.08, 1.5) window anyway, so the >=3 rule below cannot be met there.
    """
    if boards < 0:
        raise ValueError('board count must not be negative')
    if not math.isfinite(board_thickness) or not 0 < board_thickness < .8:
        raise ValueError('explicit board thickness in (0, .8) metres required')
    by_id = {c['id']: c for c in cones}
    if len(by_id) != len(cones):
        raise ValueError('duplicate cone id')
    pairs = set()
    for pair in candidates:
        if len(pair) != 2 or pair[0] == pair[1] or any(x not in by_id for x in pair):
            raise ValueError('invalid board endpoints')
        a, b = (by_id[x] for x in pair)
        if a['color'] != b['color']:
            raise ValueError('board must join the same boundary color')
        gap = math.hypot(a['x']-b['x'], a['y']-b['y'])
        if not 1.08 <= gap < 1.5:
            raise ValueError('candidate cannot fit .8 m board between .28 m proxies')
        key = tuple(sorted(pair))
        if key in pairs:
            raise ValueError('duplicate board candidate')
        pairs.add(key)
    if len(pairs) < boards:
        raise ValueError(f'at least {boards} distinct vetted candidate gaps required')
    chosen = random.Random(seed).sample(sorted(pairs), boards)
    sdf = ET.Element('sdf', version='1.6')
    world = ET.SubElement(sdf, 'world', name='prototype')
    ET.SubElement(world, 'gravity').text = '0 0 -9.81'
    physics = ET.SubElement(world, 'physics', name='default', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_update_rate').text = '1000'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'direction').text = '-0.5 0.2 -1'
    ET.SubElement(light, 'diffuse').text = '1 1 1 1'

    def box(name, x, y, z, yaw, size, color):
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = f'{x} {y} {z} 0 0 {yaw}'
        link = ET.SubElement(model, 'link', name='body')
        for tag in ('visual', 'collision'):
            item = ET.SubElement(link, tag, name=tag)
            geom = ET.SubElement(item, 'geometry')
            ET.SubElement(ET.SubElement(geom, 'box'), 'size').text = size
            if tag == 'visual':
                material = ET.SubElement(item, 'material')
                for attr in ('ambient', 'diffuse'):
                    ET.SubElement(material, attr).text = color

    box('ground', 0, 0, -.05, 0, '200 200 0.1', '.35 .35 .35 1')
    for c in cones:
        box('cone_proxy_'+c['id'], c['x'], c['y'], .315, 0, '.28 .28 .63',
            '1 0 0 1' if c['color'] == 'red' else '0 0 1 1')
    for i, (a_id, b_id) in enumerate(chosen):
        a, b = by_id[a_id], by_id[b_id]
        box(f'board_{i}', (a['x']+b['x'])/2, (a['y']+b['y'])/2, .25,
            math.atan2(b['y']-a['y'], b['x']-a['x']),
            f'.8 {board_thickness} .5', '0 0 1 1')
    ET.indent(sdf)
    return ET.tostring(sdf, encoding='unicode'), dict(
        seed=seed, selected_gaps=chosen, board_thickness=board_thickness,
        geometry_status='prototype_box_cones_not_official_track')

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cones', required=True, type=Path)
    p.add_argument('--gaps', required=True, type=Path)
    p.add_argument('--seed', required=True, type=int)
    p.add_argument('--board-thickness', required=True, type=float)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--boards', type=int, default=3,
                   help='blue boards to place (0 for fixed official layouts)')
    args = p.parse_args()
    sdf, manifest = build(load_cones(args.cones), json.loads(args.gaps.read_text()),
                          args.seed, args.board_thickness, args.boards)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(sdf, encoding='utf-8')
    args.output.with_suffix('.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')

if __name__ == '__main__':
    main()
