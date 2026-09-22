"""Regenerate the shared cone frustum binary STL (cone_red + cone_blue).

Geometry comes from asset_params.py. The STL origin sits at the bottom-centre
of the frustum (z=0 at the frustum bottom, +z up); model.sdf mounts it at
z=CONE_BASE_THICKNESS so the whole model stands 0.63 m tall on the ground.

Deterministic: fixed header, fixed vertex order, no timestamps. The script
refuses to write a mesh that fails its orientation/watertight self-checks.

Usage: python3 tools/generate_cone_mesh.py
"""
import math
import struct
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import asset_params as P  # noqa: E402

PKG_ROOT = Path(__file__).resolve().parent.parent
OUT_PATHS = [
    PKG_ROOT / 'models' / 'cone_red' / 'meshes' / 'cone_frustum.stl',
    PKG_ROOT / 'models' / 'cone_blue' / 'meshes' / 'cone_frustum.stl',
]
STL_HEADER = b'racecar_test_assets cone frustum - generated, see tools/generate_cone_mesh.py'


def frustum_triangles():
    r0 = P.CONE_FRUSTUM_BOTTOM_DIAM / 2.0
    r1 = P.CONE_FRUSTUM_TOP_DIAM / 2.0
    h = P.CONE_FRUSTUM_HEIGHT
    n = P.CONE_SEGMENTS
    b = [(r0 * math.cos(2 * math.pi * i / n), r0 * math.sin(2 * math.pi * i / n), 0.0)
         for i in range(n)]
    t = [(r1 * math.cos(2 * math.pi * i / n), r1 * math.sin(2 * math.pi * i / n), h)
         for i in range(n)]
    cb, ct = (0.0, 0.0, 0.0), (0.0, 0.0, h)
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [
            (b[i], b[j], t[j]),   # side quad, lower triangle, outward winding
            (b[i], t[j], t[i]),   # side quad, upper triangle, outward winding
            (cb, b[j], b[i]),     # bottom cap, normal -z
            (ct, t[i], t[j]),     # top cap, normal +z
        ]
    return tris


def tri_normal(tri):
    (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = tri
    ux, uy, uz = x2 - x1, y2 - y1, z2 - z1
    vx, vy, vz = x3 - x1, y3 - y1, z3 - z1
    return (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)


def self_check(tris):
    """Outward-oriented, non-degenerate, closed manifold surface."""
    h = P.CONE_FRUSTUM_HEIGHT
    for tri in tris:
        nx, ny, nz = tri_normal(tri)
        norm = math.sqrt(nx * nx + ny * ny + nz * nz)
        assert norm > 1e-9, 'degenerate triangle'
        nx, ny, nz = nx / norm, ny / norm, nz / norm
        zs = [v[2] for v in tri]
        if all(abs(z) < 1e-6 for z in zs):          # bottom cap
            assert nz < 0.99, 'bottom cap normal must point -z'
        elif all(abs(z - h) < 1e-6 for z in zs):    # top cap
            assert nz > 0.99, 'top cap normal must point +z'
        else:                                       # side face: radial outward
            cx = sum(v[0] for v in tri) / 3.0
            cy = sum(v[1] for v in tri) / 3.0
            radial = math.hypot(cx, cy)
            assert radial > 1e-9 and (nx * cx + ny * cy) / radial > 0.9, \
                'side normal not outward'
    edges = Counter()
    for a, b, c in tris:
        for u, v in ((a, b), (b, c), (c, a)):
            edges[(u, v)] += 1
    for (u, v), cnt in edges.items():
        assert cnt == 1, 'duplicate directed edge (non-manifold mesh)'
        assert edges.get((v, u)) == 1, 'missing opposite edge (not watertight)'


def write_binary_stl(path, tris):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'wb') as f:
        f.write(STL_HEADER.ljust(80, b'\0')[:80])
        f.write(struct.pack('<I', len(tris)))
        for tri in tris:
            nx, ny, nz = tri_normal(tri)
            norm = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            f.write(struct.pack('<3f', nx / norm, ny / norm, nz / norm))
            for v in tri:
                f.write(struct.pack('<3f', *v))
            f.write(struct.pack('<H', 0))


def main():
    tris = frustum_triangles()
    self_check(tris)
    xs = [v[0] for tri in tris for v in tri]
    ys = [v[1] for tri in tris for v in tri]
    zs = [v[2] for tri in tris for v in tri]
    print(f'mesh: {len(tris)} triangles ({P.CONE_SEGMENTS} segments)')
    print(f'bbox: x [{min(xs):.4f}, {max(xs):.4f}] m, '
          f'y [{min(ys):.4f}, {max(ys):.4f}] m, z [{min(zs):.4f}, {max(zs):.4f}] m')
    for path in OUT_PATHS:
        write_binary_stl(path, tris)
        print(f'wrote: {path.relative_to(PKG_ROOT).as_posix()} ({path.stat().st_size} bytes)')


if __name__ == '__main__':
    main()
