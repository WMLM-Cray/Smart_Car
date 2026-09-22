"""Print the cone collision cross-section at a given height.

Default z = 0.21 m, the main project's laser height relative to
base_footprint (task book C4). Values are computed analytically from
asset_params.py - the same parameters the STL is generated from - so this
documents exactly what the lidar should see at that height.

Run: python3 tools/cone_section.py [--z 0.21]
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import asset_params as P  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--z', type=float, default=0.21,
                    help='height above ground in metres (default 0.21)')
    args = ap.parse_args()
    s = P.cone_cross_section(args.z)
    print(f'锥桶碰撞截面 @ z = {args.z:.3f} m （地面 z=0，锥桶总高 '
          f'{P.CONE_TOTAL_HEIGHT:.2f} m）')
    if s['part'] == 'square_base':
        print(f'  部件        : 方形底座 (0 <= z <= {P.CONE_BASE_THICKNESS:.2f} m)')
        print(f'  截面形状    : {s["size"]:.2f} x {s["size"]:.2f} m 正方形'
              f'（底面外廓，规则给定）')
        print(f'  截面面积    : {s["area"]:.4f} m^2')
    elif s['part'] == 'frustum':
        print(f'  部件        : 圆台段 ({P.CONE_BASE_THICKNESS:.2f} <= z <= '
              f'{P.CONE_TOTAL_HEIGHT:.2f} m)')
        print(f'  截面外接圆直径: {s["diameter"]:.4f} m   <- 2D 雷达可见最大宽度')
        print(f'  圆等效截面面积: {s["area"]:.4f} m^2')
        print(f'  {P.CONE_SEGMENTS} 棱多边形近似的内切圆直径: '
              f'{2 * s["polygon_inradius"]:.4f} m '
              f'(比外接圆小 {(1 - s["polygon_inradius"] / s["circumradius"]) * 100:.2f}%)')
    else:
        print(f'  该高度没有碰撞体（{s["note"]}）')
    print('说明: 数值由 asset_params.py 解析计算，与 STL 网格同源；')
    print('      视觉网格与碰撞网格为同一份 STL，两者截面一致。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
