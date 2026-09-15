"""Cross-check shipped models (SDF + STL) against asset_params.py.

Verifies: cone envelope (rule: 0.63 m total height, 0.28 x 0.28 base),
identical red/blue geometry, visual/collision consistency, ground-contact
origins, board and reference box dimensions, static flags, model.config
entries. Prints a pass/fail table; exits non-zero on any failure.

This is the reproducible evidence for task-book items 11.2 (尺寸检查) and
A3 (z=0.21 截面 -> use cone_section.py). Run: python3 tools/check_models.py
"""
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import asset_params as P  # noqa: E402

MODELS = Path(__file__).resolve().parent.parent / 'models'
TOL = 1e-6
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


def read_stl(path):
    """Parse a binary STL -> (triangle count, [(v1, v2, v3), ...])."""
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f'{path.name}: shorter than an STL header')
    (n,) = struct.unpack_from('<I', data, 80)
    if len(data) != 84 + 50 * n:
        raise ValueError(f'{path.name}: not a well-formed binary STL')
    tris = []
    for k in range(n):
        v = struct.unpack_from('<12f', data, 84 + 50 * k)
        tris.append((v[3:6], v[6:9], v[9:12]))
    return n, tris


def bbox(tris):
    pts = [p for tri in tris for p in tri]
    return (min(p[0] for p in pts), max(p[0] for p in pts),
            min(p[1] for p in pts), max(p[1] for p in pts),
            min(p[2] for p in pts), max(p[2] for p in pts))


def floats(el):
    return [float(v) for v in el.text.split()]


def check_model_config(name):
    try:
        root = ET.parse(MODELS / name / 'model.config').getroot()
        ok_name = root.findtext('name') == name
        ok_sdf = root.findtext('sdf') == 'model.sdf'
        check(f'{name}/model.config: name + sdf entry valid', ok_name and ok_sdf)
    except (ET.ParseError, OSError) as e:
        check(f'{name}/model.config: parses', False, str(e))


def check_cone(name, is_red):
    try:
        root = ET.parse(MODELS / name / 'model.sdf').getroot()
    except (ET.ParseError, OSError) as e:
        check(f'{name}/model.sdf: parses', False, str(e))
        return
    model = root.find('model')
    check(f'{name}: <static> = true (test-only, no push/displacement validation)',
          model.findtext('static', '').strip() == 'true')
    link = model.find("link[@name='base_link']")
    bc = link.find("collision[@name='base_collision']")
    bv = link.find("visual[@name='base_visual']")
    fc = link.find("collision[@name='frustum_collision']")
    fv = link.find("visual[@name='frustum_visual']")

    exp_base = [P.CONE_BASE_SIZE, P.CONE_BASE_SIZE, P.CONE_BASE_THICKNESS]
    for el, tag in ((bc, 'base_collision'), (bv, 'base_visual')):
        size = floats(el.find('geometry/box/size'))
        check(f'{name}/{tag}: base box {P.CONE_BASE_SIZE}x{P.CONE_BASE_SIZE}'
              f'x{P.CONE_BASE_THICKNESS}',
              all(near(a, b) for a, b in zip(size, exp_base)),
              ' '.join(f'{v:.3f}' for v in size))
        pose = floats(el.find('pose'))
        check(f'{name}/{tag}: base pose z = {P.CONE_BASE_THICKNESS / 2:.3f} '
              f'(ground contact at z=0)',
              near(pose[2], P.CONE_BASE_THICKNESS / 2) and near(pose[0], 0)
              and near(pose[1], 0))
    check(f'{name}: base visual size == collision size',
          floats(bv.find('geometry/box/size')) == floats(bc.find('geometry/box/size')))

    for el, tag in ((fc, 'frustum_collision'), (fv, 'frustum_visual')):
        pose = floats(el.find('pose'))
        check(f'{name}/{tag}: mesh mounted at z = {P.CONE_BASE_THICKNESS:.2f}',
              near(pose[2], P.CONE_BASE_THICKNESS))
        uri = el.findtext('geometry/mesh/uri')
        check(f'{name}/{tag}: mesh uri model://{name}/meshes/cone_frustum.stl',
              uri == f'model://{name}/meshes/cone_frustum.stl', uri)

    stl = MODELS / name / 'meshes' / 'cone_frustum.stl'
    try:
        n, tris = read_stl(stl)
        check(f'{name}: STL parses as binary STL',
              True, f'{n} triangles')
        check(f'{name}: STL triangle count == 4 x {P.CONE_SEGMENTS} segments',
              n == 4 * P.CONE_SEGMENTS)
        x0, x1, y0, y1, z0, z1 = bbox(tris)
        r0 = P.CONE_FRUSTUM_BOTTOM_DIAM / 2
        check(f'{name}: STL bbox x/y in [-{r0:.2f}, {r0:.2f}] m '
              f'(frustum bottom within {P.CONE_BASE_SIZE} m base)',
              near(x0, -r0, 1e-4) and near(x1, r0, 1e-4)
              and near(y0, -r0, 1e-4) and near(y1, r0, 1e-4)
              and 2 * max(abs(x0), abs(x1)) <= P.CONE_BASE_SIZE + TOL,
              f'x[{x0:.4f},{x1:.4f}] y[{y0:.4f},{y1:.4f}]')
        check(f'{name}: STL bbox z in [0, {P.CONE_FRUSTUM_HEIGHT:.2f}] m',
              near(z0, 0.0, 1e-6) and near(z1, P.CONE_FRUSTUM_HEIGHT, 1e-4),
              f'z[{z0:.4f},{z1:.4f}]')
        check(f'{name}: total height {P.CONE_TOTAL_HEIGHT} m '
              f'(rule) = base {P.CONE_BASE_THICKNESS} + frustum {z1:.4f}',
              near(P.CONE_BASE_THICKNESS + z1, P.CONE_TOTAL_HEIGHT, 1e-4),
              f'{P.CONE_BASE_THICKNESS + z1:.4f}')
    except (ValueError, OSError) as e:
        check(f'{name}: STL readable', False, str(e))

    # material colour (both visuals must share the same diffuse)
    d1 = floats(bv.find('material/diffuse'))
    d2 = floats(fv.find('material/diffuse'))
    if is_red:
        ok = d1[0] > 0.5 and d1[1] < 0.3 and d1[2] < 0.3 and d1 == d2
    else:
        ok = d1[2] > 0.5 and d1[0] < 0.3 and d1[1] < 0.4 and d1 == d2
    check(f'{name}: diffuse is {"red" if is_red else "blue"} '
          f'and identical on both visuals', ok,
          ' '.join(f'{v:.2f}' for v in d1))


def check_box_model(name, size, z_center, colour_test, colour_desc):
    try:
        root = ET.parse(MODELS / name / 'model.sdf').getroot()
    except (ET.ParseError, OSError) as e:
        check(f'{name}/model.sdf: parses', False, str(e))
        return
    model = root.find('model')
    check(f'{name}: <static> = true',
          model.findtext('static', '').strip() == 'true')
    link = model.find("link[@name='base_link']")
    col = link.find("collision[@name='collision']")
    vis = link.find("visual[@name='visual']")
    for el, tag in ((col, 'collision'), (vis, 'visual')):
        got = floats(el.find('geometry/box/size'))
        check(f'{name}/{tag}: box {size[0]} x {size[1]} x {size[2]}',
              all(near(a, b) for a, b in zip(got, size)),
              ' '.join(f'{v:.3f}' for v in got))
        pose = floats(el.find('pose'))
        check(f'{name}/{tag}: pose z = {z_center:.3f} (bottom edge/face at z=0)',
              near(pose[2], z_center) and near(pose[0], 0) and near(pose[1], 0))
    d = floats(vis.find('material/diffuse'))
    check(f'{name}: diffuse {colour_desc}', colour_test(d),
          ' '.join(f'{v:.2f}' for v in d))


def _diffuse(el):
    d = el.find('material/diffuse') if el is not None else None
    return floats(d) if d is not None and d.text else None


def check_traffic_light():
    name = 'traffic_light'
    try:
        root = ET.parse(MODELS / name / 'model.sdf').getroot()
    except (ET.ParseError, OSError) as e:
        check(f'{name}/model.sdf: parses', False, str(e))
        return
    model = root.find('model')
    check(f'{name}: <static> = true',
          model.findtext('static', '').strip() == 'true')
    link = model.find("link[@name='base_link']")

    for kind in ('collision', 'visual'):
        el = link.find(f"{kind}[@name='pole_{kind}']")
        r = floats(el.find('geometry/cylinder/radius'))[0]
        L = floats(el.find('geometry/cylinder/length'))[0]
        pose = floats(el.find('pose'))
        check(f'{name}/{kind}/pole: cylinder r={P.TL_POLE_RADIUS} '
              f'h={P.TL_POLE_HEIGHT}',
              near(r, P.TL_POLE_RADIUS) and near(L, P.TL_POLE_HEIGHT),
              f'r={r:.3f} L={L:.3f}')
        check(f'{name}/{kind}/pole: pose z={P.TL_POLE_HEIGHT / 2:.2f} '
              f'(grounded at z=0)',
              near(pose[2], P.TL_POLE_HEIGHT / 2) and near(pose[0], 0)
              and near(pose[1], 0))

    hx, hy, hz = P.TL_HOUSING_SIZE
    for kind in ('collision', 'visual'):
        el = link.find(f"{kind}[@name='housing_{kind}']")
        size = floats(el.find('geometry/box/size'))
        pose = floats(el.find('pose'))
        check(f'{name}/{kind}/housing: box {hx} x {hy} x {hz}',
              all(near(a, b) for a, b in zip(size, P.TL_HOUSING_SIZE)),
              ' '.join(f'{v:.3f}' for v in size))
        check(f'{name}/{kind}/housing: pose z={P.TL_HOUSING_BOTTOM_Z + hz / 2:.2f} '
              f'(sits on pole top)',
              near(pose[2], P.TL_HOUSING_BOTTOM_Z + hz / 2))

    z_c = P.TL_HOUSING_BOTTOM_Z + hz / 2
    for colour, dz, test in (
        ('red', +P.TL_LENS_DZ,
         lambda d: d[0] > 0.5 and d[1] < 0.3 and d[2] < 0.3),
        ('yellow', 0.0,
         lambda d: d[0] > 0.5 and d[1] > 0.5 and d[2] < 0.3),
        ('green', -P.TL_LENS_DZ,
         lambda d: d[1] > 0.5 and d[0] < 0.3 and d[2] < 0.3),
    ):
        for kind in ('collision', 'visual'):
            el = link.find(f"{kind}[@name='{colour}_lens_{kind}']")
            r = floats(el.find('geometry/sphere/radius'))[0]
            pose = floats(el.find('pose'))
            check(f'{name}/{kind}/{colour}_lens: sphere r={P.TL_LENS_RADIUS}',
                  near(r, P.TL_LENS_RADIUS), f'r={r:.3f}')
            check(f'{name}/{kind}/{colour}_lens: z={z_c + dz:.2f} y={P.TL_LENS_Y}',
                  near(pose[2], z_c + dz) and near(pose[1], P.TL_LENS_Y))
        d = _diffuse(link.find(f"visual[@name='{colour}_lens_visual']"))
        check(f'{name}/{colour}_lens: diffuse is {colour}',
              d is not None and test(d),
              ' '.join(f'{v:.2f}' for v in d) if d else 'missing')


def check_sign(name, letter):
    try:
        root = ET.parse(MODELS / name / 'model.sdf').getroot()
    except (ET.ParseError, OSError) as e:
        check(f'{name}/model.sdf: parses', False, str(e))
        return
    model = root.find('model')
    check(f'{name}: <static> = true',
          model.findtext('static', '').strip() == 'true')
    link = model.find("link[@name='base_link']")

    for kind in ('collision', 'visual'):
        el = link.find(f"{kind}[@name='pole_{kind}']")
        r = floats(el.find('geometry/cylinder/radius'))[0]
        L = floats(el.find('geometry/cylinder/length'))[0]
        pose = floats(el.find('pose'))
        check(f'{name}/{kind}/pole: cylinder r={P.SIGN_POLE_RADIUS} '
              f'h={P.SIGN_POLE_HEIGHT}',
              near(r, P.SIGN_POLE_RADIUS) and near(L, P.SIGN_POLE_HEIGHT),
              f'r={r:.3f} L={L:.3f}')
        check(f'{name}/{kind}/pole: pose z={P.SIGN_POLE_HEIGHT / 2:.2f} '
              f'(grounded at z=0)',
              near(pose[2], P.SIGN_POLE_HEIGHT / 2) and near(pose[0], 0)
              and near(pose[1], 0))

    fx, fy, fz = P.SIGN_FACE_SIZE
    for kind in ('collision', 'visual'):
        el = link.find(f"{kind}[@name='face_{kind}']")
        size = floats(el.find('geometry/box/size'))
        pose = floats(el.find('pose'))
        check(f'{name}/{kind}/face: box {fx} x {fy} x {fz}',
              all(near(a, b) for a, b in zip(size, P.SIGN_FACE_SIZE)),
              ' '.join(f'{v:.3f}' for v in size))
        check(f'{name}/{kind}/face: pose z={P.SIGN_FACE_CENTER_Z}',
              near(pose[2], P.SIGN_FACE_CENTER_Z) and near(pose[0], 0)
              and near(pose[1], 0))

    fv = link.find("visual[@name='face_visual']")
    script = fv.find('material/script')
    uri = script.findtext('uri') if script is not None else None
    mname = script.findtext('name') if script is not None else None
    check(f'{name}: face visual material script uri = '
          f'model://{name}/materials/scripts/{name}.material',
          uri == f'model://{name}/materials/scripts/{name}.material', uri)
    check(f'{name}: material script name = {name}/face',
          mname == f'{name}/face', mname)
    png = MODELS / name / 'materials' / 'textures' / f'{name}.png'
    check(f'{name}: texture PNG shipped ({png.name}, letter {letter!r})',
          png.is_file(), png.relative_to(MODELS).as_posix())


def check_lane_line():
    name = 'lane_line'
    try:
        root = ET.parse(MODELS / name / 'model.sdf').getroot()
    except (ET.ParseError, OSError) as e:
        check(f'{name}/model.sdf: parses', False, str(e))
        return
    model = root.find('model')
    check(f'{name}: <static> = true',
          model.findtext('static', '').strip() == 'true')
    link = model.find("link[@name='base_link']")
    size = [P.LANE_LINE_LENGTH, P.LANE_LINE_WIDTH, P.LANE_LINE_THICKNESS]
    for kind in ('collision', 'visual'):
        el = link.find(f"{kind}[@name='{kind}']")
        got = floats(el.find('geometry/box/size'))
        pose = floats(el.find('pose'))
        check(f'{name}/{kind}: box {size[0]} x {size[1]} x {size[2]}',
              all(near(a, b) for a, b in zip(got, size)),
              ' '.join(f'{v:.3f}' for v in got))
        check(f'{name}/{kind}: pose z={size[2] / 2:.3f} (bottom face on ground)',
              near(pose[2], size[2] / 2) and near(pose[0], 0) and near(pose[1], 0))
    d = _diffuse(link.find("visual[@name='visual']"))
    check(f'{name}: diffuse white',
          d is not None and d[0] > 0.7 and d[1] > 0.7 and d[2] > 0.7,
          ' '.join(f'{v:.2f}' for v in d) if d else 'missing')


def main():
    print(f'== model dimension check (params from asset_params.py, '
          f'tol {TOL:g} m) ==')
    for name in ('cone_red', 'cone_blue', 'blue_board', 'reference_box',
                 'traffic_light', 'sign_a', 'sign_b', 'lane_line'):
        check_model_config(name)
    check_cone('cone_red', is_red=True)
    check_cone('cone_blue', is_red=False)
    check_traffic_light()
    check_sign('sign_a', 'A')
    check_sign('sign_b', 'B')
    check_lane_line()

    red = MODELS / 'cone_red' / 'meshes' / 'cone_frustum.stl'
    blue = MODELS / 'cone_blue' / 'meshes' / 'cone_frustum.stl'
    check('cone_red/cone_blue: STLs byte-identical (shared geometry)',
          red.read_bytes() == blue.read_bytes())

    check_box_model(
        'blue_board', [P.BOARD_LENGTH, P.BOARD_THICKNESS, P.BOARD_HEIGHT],
        P.BOARD_HEIGHT / 2,
        lambda d: d[2] > 0.5 and d[0] < 0.3, 'blue')
    check_box_model(
        'reference_box', list(P.REFBOX_SIZE), P.REFBOX_SIZE[2] / 2,
        lambda d: max(d[:3]) - min(d[:3]) < 0.05, 'grey (achromatic)')

    n_ok = sum(_results)
    n_bad = len(_results) - n_ok
    print(f'== {len(_results)} checks: {n_ok} passed, {n_bad} failed ==')
    return 1 if n_bad else 0


if __name__ == '__main__':
    sys.exit(main())
