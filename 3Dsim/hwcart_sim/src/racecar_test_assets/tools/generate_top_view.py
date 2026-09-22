"""Generate docs/top_view.svg from config/sensor_test.csv (deterministic).

The diagram is derived from the same single source of truth as the world
file, so coordinates can never drift between CSV, world and drawing.

Usage: python3 tools/generate_top_view.py [--config X] [--output Y]
"""
import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import asset_params as P  # noqa: E402
import generate_world as gw  # noqa: E402

PKG_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUTPUT = PKG_ROOT / 'docs' / 'top_view.svg'

# ---- canvas ----
S = 130.0                                   # px per metre
WX0, WX1 = -1.1, 9.4                        # world window (x, metres)
WY0, WY1 = -1.7, 1.7                        # world window (y, metres)
ML, MR, MT, MB, FOOT = 84, 52, 66, 66, 30   # margins / footer band (px)
W = ML + (WX1 - WX0) * S + MR
H = MT + (WY1 - WY0) * S + MB + FOOT

RED, BLUE = '#d94a4a', '#3b66d9'
BOARD_C, GREY, INK, GRID = '#1e3fa0', '#9a9a9a', '#333333', '#e9e9e9'
DASH_C = '#c0392b'


def X(x):
    return ML + (x - WX0) * S


def Y(y):
    return MT + (WY1 - y) * S


def _head(tx, ty, ux, uy, color):
    """Arrowhead polygon with tip at (tx,ty), pointing along (ux,uy)."""
    bx, by = tx - ux * 9, ty - uy * 9
    px, py = -uy * 3.5, ux * 3.5
    return (f'<polygon points="{tx:.1f},{ty:.1f} {bx + px:.1f},{by + py:.1f} '
            f'{bx - px:.1f},{by - py:.1f}" fill="{color}"/>')


def arrow(ax, ay, bx, by, color, both=True):
    """Dimension line (world coords) with explicit arrowheads."""
    x1, y1, x2, y2 = X(ax), Y(ay), X(bx), Y(by)
    dx, dy = x2 - x1, y2 - y1
    L = math.hypot(dx, dy) or 1.0
    ux, uy = dx / L, dy / L
    out = [f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
           f'stroke="{color}" stroke-width="1.4"/>',
           _head(x2, y2, ux, uy, color)]
    if both:
        out.append(_head(x1, y1, -ux, -uy, color))
    return '\n'.join(out)


def text(x, y, s, size=11, fill=INK, anchor='middle', bold=False, rotate=None):
    tr = f' transform="rotate({rotate} {X(x):.1f} {Y(y):.1f})"' if rotate is not None else ''
    fw = ' font-weight="bold"' if bold else ''
    return (f'<text x="{X(x):.1f}" y="{Y(y):.1f}" font-size="{size}" '
            f'fill="{fill}" text-anchor="{anchor}"{fw}{tr}>{s}</text>')


def build(objects):
    a = []
    a.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" '
             f'height="{H:.0f}" viewBox="0 0 {W:.0f} {H:.0f}" '
             f'font-family="sans-serif">')
    a.append(f'<rect x="0" y="0" width="{W:.0f}" height="{H:.0f}" fill="white"/>')

    # title + legend (top margin)
    a.append(f'<text x="{ML}" y="28" font-size="15" font-weight="bold" fill="{INK}">'
             f'racecar_test_assets · sensor_test 顶视图（测试夹具，非官方 A/B/C/D 赛道）</text>')
    lx = ML
    a.append(f'<text x="{lx}" y="51" font-size="11" fill="{INK}">图例:</text>')
    lx += 46
    a.append(f'<rect x="{lx}" y="39" width="12" height="12" fill="{RED}" '
             f'fill-opacity="0.25" stroke="{RED}"/>'
             f'<circle cx="{lx + 6}" cy="45" r="4.5" fill="{RED}"/>')
    a.append(f'<text x="{lx + 18}" y="50" font-size="11" fill="{INK}">红锥桶</text>')
    lx += 96
    a.append(f'<rect x="{lx}" y="39" width="12" height="12" fill="{BLUE}" '
             f'fill-opacity="0.25" stroke="{BLUE}"/>'
             f'<circle cx="{lx + 6}" cy="45" r="4.5" fill="{BLUE}"/>')
    a.append(f'<text x="{lx + 18}" y="50" font-size="11" fill="{INK}">蓝锥桶</text>')
    lx += 96
    a.append(f'<line x1="{lx}" y1="45" x2="{lx + 16}" y2="45" stroke="{BOARD_C}" '
             f'stroke-width="4" stroke-linecap="round"/>')
    a.append(f'<text x="{lx + 22}" y="50" font-size="11" fill="{INK}">蓝色挡板</text>')
    lx += 112
    a.append(f'<rect x="{lx}" y="39" width="12" height="12" fill="{GREY}" '
             f'stroke="#666666"/>')
    a.append(f'<text x="{lx + 18}" y="50" font-size="11" fill="{INK}">测距参考盒</text>')

    # 1 m grid + tick numbers
    for gx in range(int(math.floor(WX0)) + 1, int(math.ceil(WX1))):
        a.append(f'<line x1="{X(gx):.1f}" y1="{Y(WY1):.1f}" x2="{X(gx):.1f}" '
                 f'y2="{Y(WY0):.1f}" stroke="{GRID}" stroke-width="1"/>')
    for gy in range(int(math.floor(WY0)) + 1, int(math.ceil(WY1))):
        a.append(f'<line x1="{X(WX0):.1f}" y1="{Y(gy):.1f}" x2="{X(WX1):.1f}" '
                 f'y2="{Y(gy):.1f}" stroke="{GRID}" stroke-width="1"/>')
    for gx in range(0, 10):
        a.append(f'<text x="{X(gx):.1f}" y="{Y(WY0) + 16:.1f}" font-size="9" '
                 f'fill="#999999" text-anchor="middle">{gx}</text>')
    for gy in (-1, 0, 1):
        a.append(f'<text x="{ML - 8:.1f}" y="{Y(gy) + 3:.1f}" font-size="9" '
                 f'fill="#999999" text-anchor="end">{gy}</text>')

    # axes (+X forward, +Y left), drawn under the objects
    a.append(arrow(0, 0, 9.25, 0, '#555555', both=False))
    a.append(arrow(0, 0, 0, 1.55, '#555555', both=False))
    a.append(text(8.75, 0.42, '+X（车辆前进方向）', size=11, fill='#555555'))
    a.append(text(0.14, 1.47, '+Y（左）', size=11, fill='#555555', anchor='start'))
    a.append(text(0.14, -1.52, '−Y（右）', size=11, fill='#555555', anchor='start'))

    # origin / recommended vehicle start
    a.append(f'<circle cx="{X(0):.1f}" cy="{Y(0):.1f}" r="4.5" fill="white" '
             f'stroke="#222222" stroke-width="1.6"/>')
    a.append(text(-0.12, -0.02, '车辆起点', size=10.5, anchor='end'))
    a.append(text(-0.12, -0.18, 'base_footprint (0, 0, yaw=0)',
                  size=9.5, fill='#777777', anchor='end'))

    # reference box front face (x=7.75) dashed guide
    a.append(f'<line x1="{X(7.75):.1f}" y1="{Y(1.3):.1f}" x2="{X(7.75):.1f}" '
             f'y2="{Y(-1.3):.1f}" stroke="{DASH_C}" stroke-width="1.3" '
             f'stroke-dasharray="7 5"/>')
    a.append(text(7.75, 1.44, 'x = 7.75 m（参考盒前表面）', size=11, fill=DASH_C))

    # dimensions
    a.append(arrow(0.85, 0.75, 0.85, -0.75, '#555555'))
    a.append(text(0.85, 0.0, '1.50 m（锥桶排距）', size=10.5, fill='#555555',
                  rotate=-90))
    a.append(arrow(1.2, 1.05, 2.4, 1.05, '#555555'))
    a.append(text(1.8, 1.13, '1.20 m（同侧相邻间距）', size=10.5, fill='#555555'))
    a.append(arrow(0.0, -1.3, 7.75, -1.3, '#555555'))
    a.append(text(3.875, -1.42, '7.75 m（原点 → 参考盒前表面）',
                  size=10.5, fill='#555555'))

    # objects
    for o in objects:
        if o['model'] == 'cone_red' or o['model'] == 'cone_blue':
            c = RED if o['model'] == 'cone_red' else BLUE
            side = P.CONE_BASE_SIZE * S
            r = P.CONE_FRUSTUM_BOTTOM_DIAM / 2.0 * S
            cx, cy = X(o['x']), Y(o['y'])
            a.append(f'<rect x="{cx - side / 2:.1f}" y="{cy - side / 2:.1f}" '
                     f'width="{side:.1f}" height="{side:.1f}" fill="{c}" '
                     f'fill-opacity="0.25" stroke="{c}" stroke-width="1"/>')
            a.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="{c}"/>')
            a.append(text(o['x'], o['y'] - 0.27, o['id'], size=9.5, fill=INK))
        elif o['model'] == 'blue_board':
            half = P.BOARD_LENGTH / 2.0
            dx = math.cos(o['yaw']) * half
            dy = math.sin(o['yaw']) * half
            a.append(f'<line x1="{X(o["x"] - dx):.1f}" y1="{Y(o["y"] - dy):.1f}" '
                     f'x2="{X(o["x"] + dx):.1f}" y2="{Y(o["y"] + dy):.1f}" '
                     f'stroke="{BOARD_C}" stroke-width="5" stroke-linecap="round"/>')
            a.append(text(o['x'], o['y'] + 0.32, f'挡板 {o["id"]}（0.80×0.50）',
                          size=10, fill=BOARD_C))
        elif o['model'] == 'reference_box':
            w, h = P.REFBOX_SIZE[0] * S, P.REFBOX_SIZE[1] * S
            a.append(f'<rect x="{X(o["x"]) - w / 2:.1f}" y="{Y(o["y"]) - h / 2:.1f}" '
                     f'width="{w:.1f}" height="{h:.1f}" fill="{GREY}" '
                     f'stroke="#666666" stroke-width="1.2"/>')
            a.append(text(o['x'] + 0.33, o['y'] + 0.10, '参考盒', size=10,
                          fill=INK, anchor='start'))
            a.append(text(o['x'] + 0.33, o['y'] - 0.06, 'reference_box', size=9,
                          fill='#777777', anchor='start'))

    # footer notes
    a.append(f'<text x="{ML}" y="{H - 34:.1f}" font-size="10.5" fill="#555555">'
             f'坐标约定：米 / 弧度，Z 向上，地面 z=0；车辆起点朝 +X，+Y 为左；'
             f'锥桶原点=底面中心，挡板原点=底边中心。</text>')
    a.append(f'<text x="{ML}" y="{H - 16:.1f}" font-size="10.5" fill="#555555">'
             f'注：挡板线宽经视觉加粗（实际厚度 0.02 m，假设）；雷达（z=0.21，'
             f'相对 base_footprint x=+0.07）正前方理论距离 7.68 m = 7.75 − 0.07。</text>')
    a.append('</svg>')
    return '\n'.join(a) + '\n'


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--config', type=Path, default=gw.DEFAULT_CONFIG)
    ap.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    args = ap.parse_args()
    try:
        objects = gw.load_layout(args.config)
    except gw.LayoutError as e:
        print(f'ERROR (config {args.config}): {e}', file=sys.stderr)
        return 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(build(objects).encode('utf-8'))
    print(f'OK: {len(objects)} objects -> {gw._rel_label(args.output)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
