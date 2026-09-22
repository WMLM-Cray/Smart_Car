"""Acceptance validator - the single delivery-acceptance entry point.

Checks the SHIPPED artifacts directly (SDF + STL meshes + generated world);
never regenerates anything and never trusts the CSV alone:

  [A] every referenced file resolves inside the package (model:// includes,
      mesh URIs, model.config entries); no network URIs, no absolute paths
      from the development machine;
  [B] cone envelope re-measured from the delivered SDF/STL, including
      <scale> and <pose> (final envelope): total height 0.63 m (rule),
      base outer envelope 0.28 x 0.28 m (rule), bottom-face-centre origin,
      grounded, nothing below z=0;
  [C] red/blue cones: identical geometry, colour only;
  [D] blue board: length 0.80 m, height 0.50 m (rule), thickness recorded;
  [E] reference box dimensions;
  [F] layout read from worlds/sensor_test.world and compared with the
      task-book coordinate table (independent of the CSV), plus a
      world == CSV consistency check;
  [G] the z = 0.21 m collision cross-section verified against the ACTUAL
      STL mesh by slicing triangles, not just by formula.

Any failure prints a [FAIL] line naming the object and the problem, and the
script exits non-zero.  Usage:
    python3 tools/validate_assets.py [--pkg-root PATH]
"""
import argparse
import csv
import math
import re
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import asset_params as P    # noqa: E402  rule/assumption contract
import check_models as cm   # noqa: E402  read_stl / bbox helpers

DEFAULT_PKG_ROOT = TOOLS.parent
TOL = 1e-6        # SDF text values are exact
MESH_TOL = 1e-4   # float32 STL quantisation
Z_LASER = 0.21    # main-project laser height above ground (task book C4)

# a windows drive path (C:\ or C:/) whose letter is NOT preceded by another
# letter - so uri scheme separators like the 'l:' in 'model://' do not match
ABS_PATH_RE = re.compile(r'(?<![A-Za-z])[A-Za-z]:[\\/]|/home/|/Users/')
NET_URI_RE = re.compile(r'https?://|ftp://', re.IGNORECASE)

# ---- task-book C2/C3 coordinate table (round-1 test fixture) ----
# Deliberately independent of config/sensor_test.csv: the WORLD file is
# checked against this table, so a wrong CSV cannot mask a wrong layout.
TASKBOOK_CONE_X = [1.2, 2.4, 3.6, 4.8, 6.0]
TASKBOOK_ROW_Y = 0.75
TASKBOOK_LAYOUT = {}
for _i, _x in enumerate(TASKBOOK_CONE_X, start=1):
    TASKBOOK_LAYOUT[f'red_{_i:02d}'] = ('cone_red', _x, +TASKBOOK_ROW_Y, 0.0, 0.0)
    TASKBOOK_LAYOUT[f'blue_{_i:02d}'] = ('cone_blue', _x, -TASKBOOK_ROW_Y, 0.0, 0.0)
TASKBOOK_LAYOUT['board_01'] = ('blue_board', 3.0, -0.75, 0.0, 0.0)
TASKBOOK_LAYOUT['reference_box'] = ('reference_box', 8.0, 0.0, 0.0, 0.0)

_results = []


def check(name, ok, detail=''):
    _results.append(ok)
    line = f'[{"PASS" if ok else "FAIL"}] {name}'
    if detail:
        line += f'  ({detail})'
    print(line)
    return ok


def near(a, b, tol=TOL):
    return abs(a - b) <= tol


def floats(el):
    if el is None or not el.text:
        return None
    return [float(v) for v in el.text.split()]


def resolve_uri(root, uri):
    """model://<name>/<rest> -> Path inside <pkg>/models (raises if not found)."""
    if not uri.startswith('model://'):
        raise ValueError(f'uri is not model:// : {uri!r}')
    f = (root / 'models' / uri[len('model://'):]).resolve()
    base = (root / 'models').resolve()
    if f.parts[:len(base.parts)] != base.parts:
        raise ValueError(f'uri escapes the models dir: {uri!r}')
    if not f.is_file():
        raise FileNotFoundError(f'{uri} -> {f.relative_to(root)}: not shipped')
    return f


def mesh_scale(mesh_el):
    s = floats(mesh_el.find('scale'))
    return s if s else [1.0, 1.0, 1.0]


def functional_files(root):
    """Runtime-relevant files (docs are excluded: they may show examples)."""
    files = []
    for d in ('models', 'worlds', 'config', 'launch', 'env-hooks'):
        p = root / d
        if p.is_dir():
            files += sorted(f for f in p.rglob('*') if f.is_file())
    for f in ('package.xml', 'CMakeLists.txt'):
        if (root / f).is_file():
            files.append(root / f)
    return files


def ring_radius(tris, z_level, tol=1e-5):
    """Max horizontal radius among mesh vertices at z ~= z_level."""
    rs = [math.hypot(p[0], p[1]) for tri in tris for p in tri
          if abs(p[2] - z_level) <= tol]
    return max(rs) if rs else None


def dist_to_segment(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    s = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2)) if l2 else 0.0
    return math.hypot(px - (ax + s * dx), py - (ay + s * dy))


# ---------------------------------------------------------------- section A
def section_resources(root):
    print('\n== [A] resource references resolve inside the package ==')
    model_dirs = sorted(d for d in (root / 'models').iterdir() if d.is_dir())
    for d in model_dirs:
        check(f'models/{d.name}: model.config + model.sdf shipped',
              (d / 'model.config').is_file() and (d / 'model.sdf').is_file())
        if (d / 'model.config').is_file():
            mc = ET.parse(d / 'model.config').getroot()
            check(f'models/{d.name}: model.config name + sdf entry',
                  mc.findtext('name') == d.name and mc.findtext('sdf') == 'model.sdf')
        if (d / 'model.sdf').is_file():
            try:
                r = ET.parse(d / 'model.sdf').getroot()
                meshes = list(r.iter('mesh'))
                if not meshes:
                    check(f'{d.name}: box model (no meshes, nothing to resolve)',
                          True)
                for m in meshes:
                    u = m.findtext('uri')
                    if not u:
                        check(f'{d.name}: <mesh> has a <uri>', False,
                              'uri element missing')
                        continue
                    try:
                        f = resolve_uri(root, u)
                        n, _ = cm.read_stl(f)
                        check(f'{d.name}: mesh uri resolves + STL parses: {u}',
                              True, f'{n} triangles')
                    except (ValueError, OSError, struct.error) as e:
                        check(f'{d.name}: mesh uri resolves + STL parses: {u}',
                              False, str(e))
            except ET.ParseError as e:
                check(f'{d.name}: model.sdf parses', False, str(e))
    try:
        world_root = ET.parse(root / 'worlds' / 'sensor_test.world').getroot()
        for inc in world_root.iter('include'):
            uri = inc.findtext('uri') or ''
            m = uri[len('model://'):] if uri.startswith('model://') else None
            check(f'world include resolves: {uri}',
                  m is not None and (root / 'models' / m / 'model.sdf').is_file())
    except (ET.ParseError, OSError) as e:
        check('worlds/sensor_test.world parses', False, str(e))
    bad_net, bad_abs = [], []
    # binary assets (STL meshes, PNG textures) are not text: skip them for
    # the absolute-path / network-URI scan, which is a textual audit only.
    binary_exts = {'.stl', '.png', '.jpg', '.jpeg', '.gif', '.dae', '.obj'}
    for f in functional_files(root):
        if f.suffix.lower() in binary_exts:
            continue
        text = f.read_text(encoding='utf-8', errors='replace')
        rel = f.relative_to(root).as_posix()
        if NET_URI_RE.search(text):
            bad_net.append(rel)
        if ABS_PATH_RE.search(text):
            bad_abs.append(rel)
    check('no network URIs in shipped resources (no runtime downloads)',
          not bad_net, str(bad_net))
    check('no absolute machine paths in shipped resources', not bad_abs,
          str(bad_abs))


# ---------------------------------------------------------------- section B
def parse_cone(root, name):
    """Read the delivered cone SDF + STL -> measurement dict."""
    sdf_path = root / 'models' / name / 'model.sdf'
    r = ET.parse(sdf_path).getroot()
    model = r.find('model')
    link = model.find("link[@name='base_link']")
    out = {'static': model.findtext('static', '').strip(), 'base': {}, 'frustum': {}}
    for kind in ('collision', 'visual'):
        el = link.find(f"{kind}[@name='base_{kind}']")
        out['base'][kind] = dict(size=floats(el.find('geometry/box/size')),
                                 pose=floats(el.find('pose')))
    for kind in ('collision', 'visual'):
        el = link.find(f"{kind}[@name='frustum_{kind}']")
        mesh = el.find('geometry/mesh')
        out['frustum'][kind] = dict(uri=mesh.findtext('uri'),
                                    scale=mesh_scale(mesh),
                                    pose=floats(el.find('pose')))
    out['diffuse'] = floats(link.find(
        "visual[@name='base_visual']/material/diffuse"))
    stl = resolve_uri(root, out['frustum']['collision']['uri'])
    n, tris = cm.read_stl(stl)
    out['stl_path'], out['tris'], out['ntri'] = stl, tris, n
    out['bbox'] = cm.bbox(tris)
    return out


def section_cone(root, name):
    print(f'\n== [B] cone {name}: rule envelope, measured on delivered files ==')
    try:
        c = parse_cone(root, name)
    except Exception as e:
        check(f'{name}: model parses (SDF + STL readable)', False, str(e))
        return None
    check(f'{name}: <static> = true', c['static'] == 'true')
    for kind in ('collision', 'visual'):
        size, pose = c['base'][kind]['size'], c['base'][kind]['pose']
        check(f'{name}/{kind}: base outer envelope 0.28 x 0.28 m (rule)',
              near(size[0], P.CONE_BASE_SIZE) and near(size[1], P.CONE_BASE_SIZE),
              f'{size[0]:.4f} x {size[1]:.4f} m')
        check(f'{name}/{kind}: base thickness == {P.CONE_BASE_THICKNESS} '
              f'(assumption, on record)', near(size[2], P.CONE_BASE_THICKNESS),
              f'{size[2]:.4f} m')
        check(f'{name}/{kind}: origin = bottom-face centre, pose z = size_z/2 '
              f'(grounded at world z=0)',
              near(pose[2], size[2] / 2) and near(pose[0], 0) and near(pose[1], 0),
              f'pose z {pose[2]:.4f} vs size_z/2 {size[2] / 2:.4f}')
    check(f'{name}: base visual == collision (size + pose)',
          c['base']['collision'] == c['base']['visual'])
    check(f'{name}: frustum visual == collision (uri + scale + pose)',
          c['frustum']['collision'] == c['frustum']['visual'])

    x0, x1, y0, y1, z0, z1 = c['bbox']
    sc = c['frustum']['collision']['scale']
    mount_z = c['frustum']['collision']['pose'][2]
    check(f'{name}: mesh triangle count == 4 x {P.CONE_SEGMENTS}',
          c['ntri'] == 4 * P.CONE_SEGMENTS, str(c['ntri']))
    c['r_bottom'] = ring_radius(c['tris'], z0)
    c['r_top'] = ring_radius(c['tris'], z1)

    # final envelope: mesh own size x SDF scale, then offset by pose
    mesh_bottom_diam = 2 * max(abs(x0), abs(x1), abs(y0), abs(y1)) \
        * max(abs(sc[0]), abs(sc[1]))
    mesh_world_bottom = mount_z + z0 * sc[2]
    mesh_world_top = mount_z + z1 * sc[2]
    base_top = c['base']['collision']['pose'][2] + c['base']['collision']['size'][2] / 2
    box_bottom = c['base']['collision']['pose'][2] - c['base']['collision']['size'][2] / 2
    total_h = max(base_top, mesh_world_top)
    check(f'{name}: frustum bottom diameter (mesh x scale) <= base 0.28 m',
          mesh_bottom_diam <= P.CONE_BASE_SIZE + TOL,
          f'{mesh_bottom_diam:.4f} m')
    check(f'{name}: frustum sits on the base (mesh bottom == base top)',
          near(mesh_world_bottom, base_top, MESH_TOL),
          f'{mesh_world_bottom:.4f} vs {base_top:.4f} m')
    check(f'{name}: total height {P.CONE_TOTAL_HEIGHT} m (rule; measured incl. '
          f'scale + pose)', near(total_h, P.CONE_TOTAL_HEIGHT, MESH_TOL),
          f'{total_h:.4f} m')
    check(f'{name}: no geometry below z=0 (no ground penetration)',
          box_bottom >= -TOL and mesh_world_bottom >= -TOL,
          f'base bottom {box_bottom:.4f}, mesh bottom {mesh_world_bottom:.4f} m')
    c['mount_z'] = mount_z
    c['scale_z'] = sc[2]
    c['frustum_height'] = z1 - z0
    return c


# ---------------------------------------------------------------- section C
def section_identical(red, blue):
    print('\n== [C] red/blue cones: identical geometry, colour only ==')
    check('cone_red/cone_blue: STL files byte-identical',
          red['stl_path'].read_bytes() == blue['stl_path'].read_bytes())

    def geo(c, name):
        return (tuple(c['base']['collision']['size']),
                tuple(c['base']['collision']['pose']),
                tuple(c['frustum']['collision']['pose']),
                tuple(c['frustum']['collision']['scale']),
                c['frustum']['collision']['uri'].replace(f'model://{name}/',
                                                         'model://MODEL/'))

    check('cone_red/cone_blue: SDF geometry identical (box size/pose, mesh '
          'pose/scale/uri)', geo(red, 'cone_red') == geo(blue, 'cone_blue'))
    d_red, d_blue = red['diffuse'], blue['diffuse']
    check('cone_red: diffuse is red (R dominant)',
          d_red[0] > 0.5 and d_red[1] < 0.3 and d_red[2] < 0.3,
          ' '.join(f'{v:.2f}' for v in d_red))
    check('cone_blue: diffuse is blue (B dominant)',
          d_blue[2] > 0.5 and d_blue[0] < 0.3,
          ' '.join(f'{v:.2f}' for v in d_blue))
    check('cone_red/cone_blue: diffuse colours differ (colour is the only '
          'difference)', d_red != d_blue)


# --------------------------------------------------------- sections D and E
def section_box_model(root, name, rule, record_thickness=False):
    """rule: expected (x, y, z) sizes (x=length, y=thickness, z=height)."""
    print(f'\n== [D/E] {name}: dimensions measured on delivered SDF ==')
    try:
        r = ET.parse(root / 'models' / name / 'model.sdf').getroot()
    except (ET.ParseError, OSError) as e:
        check(f'{name}: model.sdf parses', False, str(e))
        return None
    model = r.find('model')
    link = model.find("link[@name='base_link']")
    got = {}
    for kind in ('collision', 'visual'):
        el = link.find(f"{kind}[@name='{kind}']")
        got[kind] = dict(size=floats(el.find('geometry/box/size')),
                         pose=floats(el.find('pose')))
    check(f'{name}: <static> = true', model.findtext('static', '').strip() == 'true')
    check(f'{name}: visual == collision (size + pose)', got['collision'] == got['visual'])
    size, pose = got['collision']['size'], got['collision']['pose']
    check(f'{name}: size {rule[0]} x {rule[1]} x {rule[2]} m '
          f'(x=length, y=thickness, z=height)',
          all(near(a, b) for a, b in zip(size, rule)),
          ' '.join(f'{v:.3f}' for v in size))
    if record_thickness:
        check(f'{name}: thickness recorded (assumption, on record in '
              f'docs/assumptions.md)', 0 < size[1] <= 0.1,
              f'thickness = {size[1]:.3f} m')
    check(f'{name}: origin = bottom-face centre, pose z = size_z/2 '
          f'(grounded at world z=0)',
          near(pose[2], size[2] / 2) and near(pose[0], 0) and near(pose[1], 0),
          f'pose z {pose[2]:.4f} vs size_z/2 {size[2] / 2:.4f}')
    check(f'{name}: no geometry below z=0',
          pose[2] - size[2] / 2 >= -TOL, f'bottom {pose[2] - size[2] / 2:.4f} m')
    d = floats(link.find("visual[@name='visual']/material/diffuse"))
    return dict(size=size, pose=pose, diffuse=d)


# ---------------------------------------------------------------- section F
def section_layout(root, board, refbox, cone_base):
    print('\n== [F] world layout vs task-book coordinate table ==')
    try:
        wroot = ET.parse(root / 'worlds' / 'sensor_test.world').getroot()
    except (ET.ParseError, OSError) as e:
        check('worlds/sensor_test.world parses', False, str(e))
        return
    check('sdf version 1.6 (Gazebo Classic 11)', wroot.get('version') == '1.6')
    world = wroot.find('world')
    check('world name = sensor_test',
          world is not None and world.get('name') == 'sensor_test')
    check('inline ground_plane present (top surface z=0)',
          world.find("./model[@name='ground_plane']") is not None)
    check('sun directional light present',
          world.find("./light[@name='sun']") is not None)

    entries = []
    for inc in world.findall('include'):
        pose = floats(inc.find('pose'))
        if pose is None or len(pose) != 6:
            check(f'{inc.findtext("name")}: <pose> element present (x y z r p y)',
                  False, str(inc.findtext('pose')))
            continue
        entries.append(dict(name=inc.findtext('name'), uri=inc.findtext('uri'),
                            pose=pose))
    names = [e['name'] for e in entries]
    dup = sorted({n for n in names if names.count(n) > 1})
    check('no duplicate object ids in world', not dup,
          f'duplicates: {dup}' if dup else f'{len(names)} unique ids')
    counts = {}
    for e in entries:
        counts[e['uri']] = counts.get(e['uri'], 0) + 1
    n_cones = counts.get('model://cone_red', 0) + counts.get('model://cone_blue', 0)
    check('object counts: 10 cones + 1 board + 1 reference_box (12 includes)',
          n_cones == 10 and counts.get('model://blue_board', 0) == 1
          and counts.get('model://reference_box', 0) == 1 and len(entries) == 12,
          f'cones {n_cones}, boards {counts.get("model://blue_board", 0)}, '
          f'refboxes {counts.get("model://reference_box", 0)}, '
          f'total {len(entries)}')
    missing = sorted(set(TASKBOOK_LAYOUT) - set(names))
    extra = sorted(set(names) - set(TASKBOOK_LAYOUT))
    check('every task-book object present in world (and nothing extra)',
          not missing and not extra,
          f'missing: {missing}, unexpected: {extra}'
          if missing or extra else 'exact id set match')
    for e in entries:
        exp = TASKBOOK_LAYOUT.get(e['name'])
        if exp is None:
            continue
        em, ex, ey, ez, eyaw = exp
        p = e['pose']
        ok = (e['uri'] == f'model://{em}' and near(p[0], ex, 1e-9)
              and near(p[1], ey, 1e-9) and near(p[2], ez, 1e-9)
              and p[3] == 0.0 and p[4] == 0.0 and near(p[5], eyaw, 1e-9))
        check(f'{e["name"]}: pose matches task book '
              f'({em} @ {ex:g}, {ey:g}, yaw {eyaw:g})', ok,
              f'{e["uri"]} @ ({p[0]:g}, {p[1]:g}, {p[2]:g}, yaw {p[5]:g})')

    if board and refbox and cone_base:
        half_board = board['size'][0] / 2
        half_base = cone_base / 2
        gap_l = (3.0 - half_board) - (2.4 + half_base)
        gap_r = (3.6 - half_base) - (3.0 + half_board)
        check('board_01 does not intrude into blue_02/blue_03 bases',
              gap_l > 0 and gap_r > 0,
              f'gaps {gap_l:.2f} m / {gap_r:.2f} m')
        front = 8.0 - refbox['size'][0] / 2
        check('reference_box front (+X) surface at x = 7.75 m, facing the start',
              near(front, 7.75, 1e-9),
              f'x = {front:.3f} m (centre 8.0 - size_x/2 {refbox["size"][0] / 2:g})')

    # world == CSV (single source of truth; CSV read independently here)
    try:
        with open(root / 'config' / 'sensor_test.csv',
                  encoding='utf-8-sig', newline='') as f:
            rows = [r for r in csv.reader(f)
                    if r and not r[0].lstrip().startswith('#')]
        header = [c.strip() for c in rows[0]]
        csv_rows = [dict(id=r[0].strip(), model=r[1].strip(), x=float(r[2]),
                         y=float(r[3]), z=float(r[4]), yaw=float(r[5]))
                    for r in rows[1:]]
    except (OSError, ValueError, IndexError) as e:
        check('config/sensor_test.csv readable', False, str(e))
        return
    check('config/sensor_test.csv header', header == ['id', 'model', 'x', 'y',
                                                      'z', 'yaw'], str(header))
    csv_ok = True
    csv_detail = ''
    by_id = {r['id']: r for r in csv_rows}
    if set(by_id) != set(names):
        csv_ok, csv_detail = False, 'id sets differ between CSV and world'
    else:
        for e in entries:
            r = by_id[e['name']]
            p = e['pose']
            same = (e['uri'] == f"model://{r['model']}"
                    and near(p[0], r['x'], 1e-9) and near(p[1], r['y'], 1e-9)
                    and near(p[2], r['z'], 1e-9) and near(p[5], r['yaw'], 1e-9))
            if not same:
                csv_ok = False
                csv_detail = f'{e["name"]}: world {e["uri"]} @ ({p[0]:g}, ' \
                             f'{p[1]:g}, {p[2]:g}, yaw {p[5]:g}) != CSV ' \
                             f'{r["model"]} @ ({r["x"]:g}, {r["y"]:g}, ' \
                             f'{r["z"]:g}, yaw {r["yaw"]:g})'
                break
    check('world == CSV (single source of truth)', csv_ok, csv_detail)


# ---------------------------------------------------------------- section G
def section_z021(cone):
    print(f'\n== [G] z = {Z_LASER} m collision cross-section, sliced from the '
          f'ACTUAL mesh ==')
    c = cone
    z0 = c['bbox'][4]
    z1 = c['bbox'][5]
    z_local = (Z_LASER - c['mount_z']) / c['scale_z']
    check(f'z={Z_LASER} lies in the frustum part '
          f'({c["mount_z"]:.2f} <= z <= {c["mount_z"] + c["frustum_height"]:.2f} m)',
          z0 < z_local < z1)
    r_bot, r_top = c['r_bottom'], c['r_top']
    h = c['frustum_height']
    t = (z_local - z0) / h
    r_theory = r_bot + (r_top - r_bot) * t

    # slice every triangle with the plane (mesh vertices are only at the
    # two rings, so the plane crosses edges strictly inside segments)
    pts = []
    for a, b, tri_c in c['tris']:
        seg = []
        for p, q in ((a, b), (b, tri_c), (tri_c, a)):
            dz = q[2] - p[2]
            if dz == 0:
                continue
            s = (z_local - p[2]) / dz
            if 0.0 < s < 1.0:
                seg.append((p[0] + s * (q[0] - p[0]),
                            p[1] + s * (q[1] - p[1])))
        if len(seg) == 2:
            pts += seg
    uniq = []
    for p in pts:
        if not any(abs(p[0] - q[0]) < 1e-5 and abs(p[1] - q[1]) < 1e-5
                   for q in uniq):
            uniq.append(p)
    verts = [p for p in uniq if abs(math.hypot(*p) - r_theory) <= 2 * MESH_TOL]
    others = [p for p in uniq if p not in verts]
    r_meas = max((math.hypot(*p) for p in verts), default=float('nan'))
    check(f'mesh slice @ local z={z_local:.3f}: exactly {P.CONE_SEGMENTS} '
          f'polygon vertices at the theoretical radius',
          len(verts) == P.CONE_SEGMENTS,
          f'{len(verts)} found, r_measured={r_meas:.5f} vs '
          f'r_theory={r_theory:.5f} m')
    if len(verts) == P.CONE_SEGMENTS:
        step = 2 * math.pi / P.CONE_SEGMENTS
        angs = sorted(math.atan2(p[1], p[0]) for p in verts)
        gaps = [angs[(i + 1) % P.CONE_SEGMENTS] - angs[i]
                for i in range(P.CONE_SEGMENTS - 1)]
        gaps.append(angs[0] + 2 * math.pi - angs[-1])
        check('slice vertices evenly spaced (regular polygon)',
              all(abs(g - step) < 1e-3 for g in gaps),
              f'max spacing deviation '
              f'{max(abs(g - step) for g in gaps):.2e} rad')
        check(f'mesh slice: exactly {P.CONE_SEGMENTS} on-edge crossing points '
              f'(diagonal edges)', len(others) == P.CONE_SEGMENTS,
              f'{len(others)} found')
        if len(others) == P.CONE_SEGMENTS:
            vs = sorted(verts, key=lambda p: math.atan2(p[1], p[0]))
            max_dev = max(min(dist_to_segment(d, vs[i], vs[(i + 1) % len(vs)])
                              for i in range(len(vs))) for d in others)
            check('all other slice points lie on the polygon edges '
                  '(section is the exact 24-gon)', max_dev <= 2 * MESH_TOL,
                  f'max deviation {max_dev:.2e} m')
        inr = min(dist_to_segment((0.0, 0.0), vs[i], vs[(i + 1) % len(vs)])
                  for i in range(len(vs)))
        check(f'measured circumdiameter {2 * r_theory:.4f} m matches theory',
              near(2 * r_meas, 2 * r_theory, MESH_TOL),
              f'measured {2 * r_meas:.4f} vs theory {2 * r_theory:.4f} m')
        check('measured inradius == r * cos(pi/24) (regular 24-gon)',
              near(inr, r_theory * math.cos(math.pi / P.CONE_SEGMENTS),
                   MESH_TOL),
              f'measured {inr:.5f} vs theory '
              f'{r_theory * math.cos(math.pi / P.CONE_SEGMENTS):.5f} m')
        print('  截面参数（全部实测自交付文件）:')
        print(f'  底座厚度      : {c["mount_z"]:.3f} m（SDF 实测）')
        print(f'  圆台高度      : {h:.3f} m（STL bbox 实测）')
        print(f'  底/顶部半径   : {r_bot:.4f} / {r_top:.4f} m（STL 环顶点实测）')
        print(f'  理论半径 @z={Z_LASER}: {r_theory:.4f} m（线性插值）'
              f' -> 外接圆直径 {2 * r_theory:.4f} m')
        print(f'  实测网格截面  : 外接圆直径 {2 * r_meas:.4f} m（24 顶点），'
              f'内切圆直径 {2 * inr:.4f} m')
        print(f'  多边形近似误差: {(1 - inr / r_meas) * 100:.2f} %'
              f'（24 棱外接 vs 内切）')


# ------------------------------------------------------------- section H
def _png_size(path):
    """Return (width, height) from a PNG IHDR, or None if not a valid PNG."""
    data = path.read_bytes()
    if len(data) < 24 or data[:8] != b'\x89PNG\r\n\x1a\n':
        return None
    if data[12:16] != b'IHDR':
        return None
    return struct.unpack_from('>II', data, 16)


def section_new_models(root):
    """New model families: traffic light, A/B signs, lane line.

    Size/origin/colour checks are measured on the delivered SDF (like sections
    B-E). Sign face textures are checked for being shipped, resolvable via
    their material script, and a valid 512x512 PNG (signature + IHDR size).
    """
    print('\n== [H] new models: traffic_light / sign_a / sign_b / lane_line ==')

    name = 'traffic_light'
    try:
        r = ET.parse(root / 'models' / name / 'model.sdf').getroot()
    except (ET.ParseError, OSError) as e:
        check(f'{name}: model.sdf parses', False, str(e))
    else:
        m = r.find('model')
        check(f'{name}: <static> = true',
              m.findtext('static', '').strip() == 'true')
        link = m.find("link[@name='base_link']")
        hb = floats(link.find("collision[@name='housing_collision']/geometry/box/size"))
        check(f'{name}: housing box {P.TL_HOUSING_SIZE[0]} x '
              f'{P.TL_HOUSING_SIZE[1]} x {P.TL_HOUSING_SIZE[2]} m',
              all(near(a, b) for a, b in zip(hb, P.TL_HOUSING_SIZE)),
              ' '.join(f'{v:.3f}' for v in hb))
        pp = floats(link.find("collision[@name='pole_collision']/pose"))
        check(f'{name}: pole grounded (pose z={P.TL_POLE_HEIGHT / 2})',
              near(pp[2], P.TL_POLE_HEIGHT / 2))
        for colour in ('red', 'yellow', 'green'):
            v = link.find(f"visual[@name='{colour}_lens_visual']")
            check(f'{name}: {colour} lens visual present', v is not None)

    for name, letter in (('sign_a', 'A'), ('sign_b', 'B')):
        try:
            r = ET.parse(root / 'models' / name / 'model.sdf').getroot()
        except (ET.ParseError, OSError) as e:
            check(f'{name}: model.sdf parses', False, str(e))
            continue
        m = r.find('model')
        check(f'{name}: <static> = true',
              m.findtext('static', '').strip() == 'true')
        fv = m.find("link[@name='base_link']/visual[@name='face_visual']")
        script = fv.find('material/script') if fv is not None else None
        uri = script.findtext('uri') if script is not None else None
        try:
            resolve_uri(root, uri)
            check(f'{name}: material script uri resolves in package', True, uri)
        except (ValueError, FileNotFoundError) as e:
            check(f'{name}: material script uri resolves in package', False,
                  str(e))
        png = root / 'models' / name / 'materials' / 'textures' / f'{name}.png'
        check(f'{name}: texture PNG shipped ({letter})', png.is_file(),
              png.relative_to(root).as_posix())
        if png.is_file():
            check(f'{name}: texture PNG is a 512x512 image',
                  _png_size(png) == (512, 512),
                  f'{_png_size(png)}')

    name = 'lane_line'
    try:
        r = ET.parse(root / 'models' / name / 'model.sdf').getroot()
    except (ET.ParseError, OSError) as e:
        check(f'{name}: model.sdf parses', False, str(e))
    else:
        m = r.find('model')
        check(f'{name}: <static> = true',
              m.findtext('static', '').strip() == 'true')
        sz = floats(m.find("link[@name='base_link']/collision[@name='collision']/geometry/box/size"))
        exp = [P.LANE_LINE_LENGTH, P.LANE_LINE_WIDTH, P.LANE_LINE_THICKNESS]
        check(f'{name}: box {exp[0]} x {exp[1]} x {exp[2]} m',
              all(near(a, b) for a, b in zip(sz, exp)),
              ' '.join(f'{v:.3f}' for v in sz))


# ------------------------------------------------------------- section I
def section_demo_world(root):
    """Demo world: includes all five model families and matches its CSV."""
    print('\n== [I] demo_world: five model families, world == CSV ==')
    try:
        wroot = ET.parse(root / 'worlds' / 'demo_world.world').getroot()
    except (ET.ParseError, OSError) as e:
        check('worlds/demo_world.world parses', False, str(e))
        return
    check('sdf version 1.6', wroot.get('version') == '1.6')
    world = wroot.find('world')
    check('demo world name = demo_world',
          world is not None and world.get('name') == 'demo_world')
    entries = []
    for inc in world.findall('include'):
        entries.append(dict(name=inc.findtext('name'), uri=inc.findtext('uri')))
    models = {e['uri'] for e in entries}
    need = {'model://traffic_light', 'model://sign_a', 'model://sign_b',
            'model://lane_line', 'model://cone_red', 'model://cone_blue',
            'model://blue_board'}
    check('demo world includes all five model families (+ cones + board)',
          need <= models,
          f'missing: {sorted(need - models)}' if need - models else 'all present')

    try:
        with open(root / 'config' / 'demo_world.csv',
                  encoding='utf-8-sig', newline='') as f:
            rows = [r for r in csv.reader(f)
                    if r and not r[0].lstrip().startswith('#')]
        csv_rows = [dict(id=r[0].strip(), model=r[1].strip(), x=float(r[2]),
                         y=float(r[3]), z=float(r[4]), yaw=float(r[5]))
                    for r in rows[1:]]
    except (OSError, ValueError, IndexError) as e:
        check('config/demo_world.csv readable', False, str(e))
        return
    by_id = {r['id']: r for r in csv_rows}
    world_ids = {e['name'] for e in entries}
    check('demo world id set == demo CSV id set',
          set(by_id) == world_ids,
          f'csv {len(by_id)} vs world {len(world_ids)}')
    ok = True
    detail = ''
    for e in entries:
        r = by_id.get(e['name'])
        if r is None or e['uri'] != f"model://{r['model']}":
            ok, detail = False, f'{e["name"]}: {e["uri"]}'
            break
    check('demo world model references == demo CSV', ok, detail)


def safe(label, fn):
    try:
        return fn()
    except Exception as e:
        check(f'{label}: section completed without internal error', False,
              repr(e))
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--pkg-root', type=Path, default=DEFAULT_PKG_ROOT,
                    help='package root to validate (default: this copy)')
    args = ap.parse_args()
    root = args.pkg_root.resolve()
    print('== validate_assets: acceptance check on the DELIVERED files ==')
    print(f'== package root: {root} ==')
    safe('[A] resources', lambda: section_resources(root))
    red = safe('[B] cone_red', lambda: section_cone(root, 'cone_red'))
    blue = safe('[B] cone_blue', lambda: section_cone(root, 'cone_blue'))
    if red and blue:
        safe('[C] identical', lambda: section_identical(red, blue))
    board = safe('[D] board', lambda: section_box_model(
        root, 'blue_board',
        [P.BOARD_LENGTH, P.BOARD_THICKNESS, P.BOARD_HEIGHT],
        record_thickness=True))
    refbox = safe('[E] reference_box', lambda: section_box_model(
        root, 'reference_box', list(P.REFBOX_SIZE)))
    cone_base = red['base']['collision']['size'][0] if red else None
    safe('[F] layout', lambda: section_layout(root, board, refbox, cone_base))
    if red:
        safe('[G] z=0.21 section', lambda: section_z021(red))
    safe('[H] new models', lambda: section_new_models(root))
    safe('[I] demo world', lambda: section_demo_world(root))

    n_ok = sum(_results)
    n_bad = len(_results) - n_ok
    print(f'\n== validate_assets: {len(_results)} checks: '
          f'{n_ok} passed, {n_bad} failed ==')
    if n_bad:
        print('RESULT: FAILED')
        return 1
    print('RESULT: PASSED')
    return 0


if __name__ == '__main__':
    sys.exit(main())
