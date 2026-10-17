"""随机楼层:走廊 + 房间 + 大厅,里面摆椅子、桌子、箱子、柱子,出一张 RealMap(5 cm 栅格)给 RealMapEnv 开。
用于开放场景的强化学习(车在楼里自由探索,没有"路"和终点)。

    python -m simulation.fastsim.floorplan --out runs/floorplan --count 6      # 出几张图看

生成步骤(都在 5 cm 格上):
  走廊   第一条横贯全楼,之后每条从已有走廊上某点垂直伸出(T / L / 十字路口、死胡同自然出现)
  房间   贴着走廊一侧开,中间隔一道墙,墙上开门;少数做成大厅(开阔区)
  墙     地面外扩 WALL 厚的一圈
  家具   椅子 = 4 条 4 cm 腿(雷达只扫到 4 个点,相机看得到椅子下面的地面);桌子 = 4 条腿;
         箱子 / 柜子 = 实心矩形(贴墙放);柱子 = 实心方块(大厅里)。
         摆完检查车能去的区域(地面按车半宽腐蚀后)没被切碎,切碎了就少摆一些重来。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

CELL = 0.05


@dataclass(frozen=True)
class PlanConfig:
    kind: str = 'building'                          # 'building' = 走廊 + 房间;'room' = 一间长方形房间(课程第一步)
    room_box: tuple = ((5.0, 12.0), (4.0, 10.0))    # kind='room' 时房间的长、宽 m
    size: tuple = ((20.0, 36.0), (15.0, 28.0))     # 楼的外框 m(x, y)
    corridors: tuple = (2, 5)                       # 走廊条数
    corridor_w: tuple = (1.2, 3.0)                  # 走廊宽 m
    # 走廊尽头:车不能倒车,掉头要 ~2.6 m 宽(后轴转弯半径 0.88 m + 车身 + 截停余量),窄走廊的死路进去就出不来。
    # 第一版每条走廊两头都顶到外墙,v4g 91% 的局在死路 / 墙角被截停,结局全由出发点决定,RL 学不到东西(2026-10-17)。
    # 现在尽头大多扩成能掉头的门厅,只有 dead_end 的概率保持原样
    dead_end: float = 0.2                           # 走廊尽头保持死路的概率
    lobby: tuple = (3.0, 6.0)                       # 门厅边长 m(沿走廊 / 横向各抽一次)
    rooms: tuple = (3, 9)                           # 房间个数(尝试次数 × 3,放不下的跳过)
    room_size: tuple = (3.0, 8.0)                   # 房间边长 m
    hall_chance: float = 0.25                       # 房间做成大厅(边长 8~12 m、开两个门)的概率
    door_w: tuple = (0.8, 1.6)                      # 门宽 m
    wall: float = 0.10                              # 墙厚 m
    chair_sets: tuple = (2, 8)                      # 桌椅组数(一张桌子 + 2~6 把椅子)
    wall_chairs: tuple = (3, 12)                    # 靠墙单放的椅子
    boxes: tuple = (3, 12)                          # 贴墙的箱子 / 柜子
    pillars: tuple = (0, 4)                         # 大厅柱子
    car_half: float = 0.25                          # 连通检查用的车半宽(含余量)m


def _rect(g, x0, y0, x1, y1, val=True):
    """世界坐标 m 的矩形 -> 格,夹到图内。"""
    i0, i1 = max(0, int(round(x0 / CELL))), min(g.shape[0], int(round(x1 / CELL)))
    j0, j1 = max(0, int(round(y0 / CELL))), min(g.shape[1], int(round(y1 / CELL)))
    if i1 > i0 and j1 > j0:
        g[i0:i1, j0:j1] = val
    return i0, j0, i1, j1


def _corridors(rng, cfg, floor, X, Y):
    segs = []                                       # (方向 0 = 沿 x / 1 = 沿 y, 中线坐标, 起, 止, 宽)
    w = rng.uniform(*cfg.corridor_w)
    if rng.random() < 0.5:
        segs.append((0, rng.uniform(0.3, 0.7) * Y, 1.0, X - 1.0, w))
    else:
        segs.append((1, rng.uniform(0.3, 0.7) * X, 1.0, Y - 1.0, w))
    for _ in range(int(rng.integers(*cfg.corridors)) - 1):
        d, c, a, b, _w = segs[rng.integers(len(segs))]
        at = rng.uniform(a + 1.5, b - 1.5)          # 从已有走廊上这一点垂直伸出
        w = rng.uniform(*cfg.corridor_w)
        span = Y if d == 0 else X
        lo, hi = (1.0, c) if rng.random() < 0.5 else (c, span - 1.0)
        if rng.random() < 0.3:                      # 十字:两边都伸
            lo, hi = 1.0, span - 1.0
        if hi - lo < 3.0:
            continue
        segs.append((1 - d, at, lo, hi, w))
    for d, c, a, b, w in segs:
        if d == 0:
            _rect(floor, a, c - w / 2, b, c + w / 2)
        else:
            _rect(floor, c - w / 2, a, c + w / 2, b)
    for k, (d, c, a, b, w) in enumerate(segs):           # 走廊尽头:大多扩成门厅
        for end, out in ((a, -1), (b, 1)):
            if k > 0 and _attached(segs[:k], d, c, end):
                continue                                    # 这头接在别的走廊上,不是尽头
            if rng.random() < cfg.dead_end:
                continue
            L, D = rng.uniform(*cfg.lobby), rng.uniform(*cfg.lobby)
            s0, s1 = sorted((end, end - out * L))
            if d == 0:
                _rect(floor, max(0.5, s0), max(0.5, c - D / 2), min(X - 0.5, s1), min(Y - 0.5, c + D / 2))
            else:
                _rect(floor, max(0.5, c - D / 2), max(0.5, s0), min(X - 0.5, c + D / 2), min(Y - 0.5, s1))
    return segs


def _attached(prev, d, c, end):
    """沿方向 d、横向位置 c 的走廊在 end 处是不是落在之前某条(垂直的)走廊上。"""
    return any(pd != d and abs(pc - end) < pw / 2 + 1e-6 and pa - 1e-6 <= c <= pb + 1e-6 for pd, pc, pa, pb, pw in prev)


def _rooms(rng, cfg, floor, segs, X, Y):
    """贴走廊开房间:房间和走廊之间留一道墙,墙上开门。返回 [(x0, y0, x1, y1, 是否大厅)]。"""
    from scipy.ndimage import binary_dilation
    rooms, wall = [], cfg.wall
    pad = int(round(wall / CELL))                       # 房间和别的地面之间至少隔这么多格(墙)
    for _ in range(int(rng.integers(*cfg.rooms)) * 8):
        if len(rooms) >= cfg.rooms[1]:
            break
        d, c, a, b, w = segs[rng.integers(len(segs))]
        hall = rng.random() < cfg.hall_chance
        lo, hi = (8.0, 12.0) if hall else cfg.room_size
        L, D = rng.uniform(lo, hi), rng.uniform(lo, hi)     # 沿走廊长、离走廊深
        s0 = rng.uniform(a, max(a, b - L))
        side = 1 if rng.random() < 0.5 else -1
        near = c + side * (w / 2 + wall + CELL)             # 房间靠走廊那条边(多隔一格,免得取整后贴上)
        far = near + side * D
        if d == 0:
            x0, x1, y0, y1 = s0, s0 + L, min(near, far), max(near, far)
        else:
            y0, y1, x0, x1 = s0, s0 + L, min(near, far), max(near, far)
        if x0 < 0.5 or y0 < 0.5 or x1 > X - 0.5 or y1 > Y - 0.5:
            continue
        probe = np.zeros_like(floor)
        _rect(probe, x0, y0, x1, y1)
        if (binary_dilation(probe, iterations=pad) & floor).any():
            continue                                        # 压到别的房间 / 走廊(要留出墙)
        floor |= probe
        for _k in range(2 if hall else 1):                  # 门:穿过走廊和房间之间的墙
            dw = rng.uniform(*cfg.door_w)
            t = rng.uniform(s0 + 0.3, s0 + L - dw - 0.3)
            a0, a1 = sorted((c + side * (w / 2 - 0.05), near + side * 0.05))
            if d == 0:
                _rect(floor, t, a0, t + dw, a1)
            else:
                _rect(floor, a0, t, a1, t + dw)
        rooms.append((x0, y0, x1, y1, hall))
    return rooms


def _furniture(rng, cfg, floor, rooms):
    """返回 (legs 布尔格:细腿 —— 雷达能扫到、相机看得到下面的地面, solid 布尔格:实心家具)。"""
    from scipy.ndimage import distance_transform_edt
    legs = np.zeros_like(floor)
    solid = np.zeros_like(floor)
    dist = distance_transform_edt(floor) * CELL             # 到墙的距离
    fi = np.argwhere(floor)

    def leg(x, y):
        i, j = int(round(x / CELL)), int(round(y / CELL))
        if 0 <= i < floor.shape[0] and 0 <= j < floor.shape[1] and floor[i, j]:
            legs[i, j] = True

    def chair(x, y, yaw, s=None):
        s = s or rng.uniform(0.4, 0.5)
        c, sn = math.cos(yaw), math.sin(yaw)
        for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            leg(x + (a * c - b * sn) * s / 2, y + (a * sn + b * c) * s / 2)

    def free_spot(min_wall, max_wall=99.0):
        for _ in range(50):
            i, j = fi[rng.integers(len(fi))]
            if min_wall <= dist[i, j] <= max_wall:
                return i * CELL, j * CELL
        return None

    # 桌椅组:放在房间 / 大厅里(离墙 >= 1 m)
    spots = [r for r in rooms] or [None]
    for _ in range(int(rng.integers(*cfg.chair_sets))):
        r = spots[rng.integers(len(spots))]
        if r is None:
            p = free_spot(1.0)
        else:
            x0, y0, x1, y1, _h = r
            p = (rng.uniform(x0 + 1.0, max(x0 + 1.0, x1 - 1.0)), rng.uniform(y0 + 1.0, max(y0 + 1.0, y1 - 1.0)))
        if p is None or not floor[int(round(p[0] / CELL)), int(round(p[1] / CELL))]:
            continue
        x, y = p
        yaw = rng.choice([0.0, math.pi / 2]) + rng.normal(0, 0.05)
        tl, tw = rng.uniform(1.0, 1.8), rng.uniform(0.6, 0.8)
        c, s = math.cos(yaw), math.sin(yaw)
        for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            leg(x + (a * c * tl - b * s * tw) / 2 * 0.9, y + (a * s * tl + b * c * tw) / 2 * 0.9)
        for _k in range(int(rng.integers(2, 7))):           # 椅子围着桌子,随手拉开一点、转一点
            side = rng.choice([-1, 1])
            along = rng.uniform(-tl / 2 + 0.25, tl / 2 - 0.25)
            off = tw / 2 + rng.uniform(0.15, 0.5)
            chair(x + c * along - s * side * off, y + s * along + c * side * off, yaw + rng.normal(0, 0.3))
    # 靠墙的椅子
    for _ in range(int(rng.integers(*cfg.wall_chairs))):
        p = free_spot(0.25, 0.45)
        if p:
            chair(*p, rng.uniform(0, 2 * math.pi))
    # 贴墙的箱子 / 柜子
    for _ in range(int(rng.integers(*cfg.boxes))):
        p = free_spot(0.05, 0.15)
        if p:
            a, b = rng.uniform(0.3, 1.2), rng.uniform(0.3, 0.6)
            if rng.random() < 0.5:
                a, b = b, a
            _rect(solid, p[0] - a / 2, p[1] - b / 2, p[0] + a / 2, p[1] + b / 2)
    # 柱子:只在大厅
    halls = [r for r in rooms if r[4]]
    for _ in range(int(rng.integers(*cfg.pillars)) if halls else 0):
        x0, y0, x1, y1, _h = halls[rng.integers(len(halls))]
        x, y, h = rng.uniform(x0 + 1.5, x1 - 1.5), rng.uniform(y0 + 1.5, y1 - 1.5), rng.uniform(0.2, 0.3)
        _rect(solid, x - h, y - h, x + h, y + h)
    solid &= floor
    return legs, solid


def _reach(free, half):
    """车(半宽 half)能开到的格:地面按半宽腐蚀后最大的连通块。"""
    from scipy.ndimage import binary_erosion, label
    r = int(math.ceil(half / CELL))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    core = binary_erosion(free, xx ** 2 + yy ** 2 <= r * r)
    lab, n = label(core)
    if n == 0:
        return core
    return lab == (np.bincount(lab.ravel())[1:].argmax() + 1)


def generate(rng: np.random.Generator, cfg: PlanConfig = PlanConfig(), tries: int = 20):
    """返回 (RealMap, info)。info:floor / legs / solid / reach 布尔格(格坐标从 0 开始)、rooms、corridors。"""
    from scipy.ndimage import binary_dilation
    from simulation.fastsim.real_map import RealMap
    for _ in range(tries):
        if cfg.kind == 'room':
            X, Y = rng.uniform(*cfg.room_box[0]) + 1.0, rng.uniform(*cfg.room_box[1]) + 1.0
            floor = np.zeros((int(X / CELL) + 1, int(Y / CELL) + 1), bool)
            _rect(floor, 0.5, 0.5, X - 0.5, Y - 0.5)
            segs, rooms = [], [(0.5, 0.5, X - 0.5, Y - 0.5, False)]
        else:
            X, Y = rng.uniform(*cfg.size[0]), rng.uniform(*cfg.size[1])
            floor = np.zeros((int(X / CELL) + 1, int(Y / CELL) + 1), bool)
            segs = _corridors(rng, cfg, floor, X, Y)
            rooms = _rooms(rng, cfg, floor, segs, X, Y)
        base = _reach(floor, cfg.car_half).sum()
        for _k in range(4):                                 # 家具把路堵死了就重摆
            legs, solid = _furniture(rng, cfg, floor, rooms)
            hard = binary_dilation(legs, iterations=1) | solid    # 腿按 15 cm 算(车不会从两条腿之间钻)
            reach = _reach(floor & ~hard, cfg.car_half)
            if reach.sum() >= 0.85 * base:
                break
        else:
            continue
        t = max(1, int(round(cfg.wall / CELL)))
        wall = binary_dilation(floor, iterations=t) & ~floor
        occ = wall | legs | solid
        pad = t + 2                                         # 地图四周留几格,墙不贴图边
        P = lambda g: np.argwhere(g) + pad
        world = RealMap(P(occ), CELL, P(floor & ~solid), P(floor | wall), known_pad=0.0)
        info = dict(floor=floor, legs=legs, solid=solid, reach=reach, rooms=rooms, corridors=segs, pad=pad)
        return world, info
    raise RuntimeError(f'{tries} 次都没生成出能开的楼层')


def circle_clear() -> float:
    """满舵绕圈时圆心到障碍至少要多远 m:车身扫过的外半径(前外角)+ 截停净空 + 一格余量。
    出发点要有一侧能这样一直绕圈 —— 保证每一局都"有解",不会一出发就注定被截停
    (第一版只要求车头前 1.5 m 空,why_guard 里好几局出发时离墙太近、怎么打都转不开,2026-10-17)。"""
    from common.params import CarParams
    from vehicle.road_car_node import GUARD_STOP
    p = CarParams()
    R = p.wheelbase / math.tan(p.max_steer)
    return math.hypot(R + p.width / 2, p.length - p.rear_overhang) + GUARD_STOP + CELL


def sample_start(rng, world, info, clear=0.6, ahead=1.5):
    """随机出发位姿 (x, y, yaw):离障碍 >= clear,车头前 ahead 米内没有障碍,
    并且左满舵或右满舵的圆(circle_clear)里没有障碍。"""
    from scipy.ndimage import distance_transform_edt

    from common.params import CarParams
    p = CarParams()
    R, need = p.wheelbase / math.tan(p.max_steer), circle_clear()
    edt = world._edt()
    pad = info['pad']
    ok = info['reach'] & (distance_transform_edt(~(info['legs'] | info['solid']) & info['floor']) * CELL >= clear)
    cand = np.argwhere(ok)
    for _ in range(2000):
        i, j = cand[rng.integers(len(cand))]
        x, y = (i + pad) * CELL, (j + pad) * CELL
        yaw = rng.uniform(-math.pi, math.pi)
        if world.raycast(x, y, np.array([yaw - 0.3, yaw, yaw + 0.3]), ahead + 0.5).min() <= ahead:
            continue
        for side in (1, -1):
            cx, cy = x - side * R * math.sin(yaw), y + side * R * math.cos(yaw)
            ci, cj = int(round(cx / CELL)) - world.i0, int(round(cy / CELL)) - world.j0
            if 0 <= ci < edt.shape[0] and 0 <= cj < edt.shape[1] and edt[ci, cj] >= need:
                return x, y, yaw
    raise RuntimeError('找不到出发点')


def render(cv2, world, info, path=None):
    from simulation.fastsim.map_view import MapView
    h, w = world.occ.shape
    view = MapView(((world.i0) * CELL, (world.j0) * CELL, (world.i0 + h) * CELL, (world.j0 + w) * CELL), CELL, px=3, dot=3)
    img = view.base(cv2, np.argwhere(world.occ) + (world.i0, world.j0), np.argwhere(world.free) + (world.i0, world.j0))
    if path is not None and len(path) > 1:
        view.path(cv2, img, np.asarray(path)[:, :2])
        view.car(cv2, img, path[-1])
    return img, view


def main(argv=None):
    import argparse
    import os

    import cv2
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--count', type=int, default=6)
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    for k in range(a.count):
        rng = np.random.default_rng(a.seed + k)
        world, info = generate(rng)
        x, y, yaw = sample_start(rng, world, info)
        img, view = render(cv2, world, info)
        view.car(cv2, img, (x, y, yaw), color=(0, 0, 220), size=0.5)
        cv2.imwrite(os.path.join(a.out, f'plan_{a.seed + k:03d}.png'), img)
        print(f'plan {a.seed + k}: {world.occ.shape[0] * CELL:.0f} x {world.occ.shape[1] * CELL:.0f} m  '
              f'走廊 {len(info["corridors"])}  房间 {len(info["rooms"])}(大厅 {sum(r[4] for r in info["rooms"])})  '
              f'椅/桌腿 {info["legs"].sum()}  可达 {info["reach"].sum() * CELL ** 2:.0f} m²')


if __name__ == '__main__':
    main()
