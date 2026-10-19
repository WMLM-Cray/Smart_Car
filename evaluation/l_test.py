"""L 型拐角测试:按实车 1014 seg_06 的几何(m5 直冲墙角、截停)在 fastsim 里搭一个左拐的 L,看哪些条件下模型不拐。

    python -m evaluation.l_test models/a.npz models/b.npz [teacher] [--seeds 2]
    python -m evaluation.l_test run --model models/a.npz --cap 0.7 --count 300 --out runs/l_a.json   # v4.5 主指标
    python -m evaluation.l_test compare runs/l_a.json runs/l_b.json

v4.5 主指标(docs/v4.5方案.md §6):`run` 按种子抽加难的 L 弯 —— 左弯 / 右弯、离前墙 5~9 m、路宽 2.5~6 m
(5 m 以上弯内侧敞开)、初始朝向 ±10°,报通过率和**转向起始点**:策略开始一直朝弯打方向(见 STEER_ON)时
车最前端离前墙多远、放行后多少秒(用户 10-19 定;车头转过 TURNED 的位置另报作参考);`compare` 对同一批局配对比较。

实车那段(车的起点坐标系,x 前 y 左):右边是墙(y ≈ -1.9),正前方 7 m 是墙,出口在左边(一大片屋子,家具零散);
车贴右墙往前开了 6.3 m,停在离前墙 0.7 m 处。这里:进口直道 + 尖 90° 左弯(外侧方角)+ 出口直道,
车(后轴)离前墙 FRONT 米、离右墙 RIGHT 米;路宽(= 左边多空)和初始朝向扫一遍。开到出口直道终点算过,
截停 / 撞 / 超时算失败。部署条件同 tee_test:不用 YOLO、地面分割 0.33 s / 0.35 s、按车上的投影 + 补缝(10-19 起;之前是理想地面分割)。
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import itertools
import math
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import numpy as np

FRONT = 7.0                 # 车(后轴)离前墙 m(实车 seg_06)
RIGHT = 1.9                 # 车离右墙 m
WIDTHS = (2.5, 3.5, 4.5)    # 路宽:实车左边是开阔的屋子,扫一个范围
YAWS = (-10.0, 0.0, 10.0)   # 初始朝向偏差 °(左正;实车起步时大致平行右墙)
EXIT = 5.0                  # 出口直道长 m
TURNED = math.radians(15.0) # 车头相对进口路的方向朝弯转过这么多 = 车头开始转(参考;初始朝向只有 ±10°,不会误触发)
STEER_ON = 0.1              # 转向起始点(主指标):从车头转过 TURNED 那一拍往回找,一直"朝弯打方向超过 STEER_ON、
                            # 且车头已经偏向弯那一侧"的这一段的起点。10-19 试过两种不行的:"第一次连续超过阈值"
                            # 被起步纠正初始朝向触发;"转过 45° 前最后一段连续超过 0.3"被老师平缓打方向时的小起伏打断
TEST_SEED0 = 96_000_000     # run 的测试种子段;挑 candidate 用 SELECT_SEED0(和训练 30M/60M、bev_testset 93M、A-D 95M 不重叠)
SELECT_SEED0 = 97_000_000


def ell(width, open_left=False, front=FRONT, right_turn=False):
    """左拐 L:外侧(右墙 -> 前墙)是方角(见 _square_corner)。
    open_left:内侧(左边)不立墙、只是地面边界(EDGE)—— 实车左边是开阔的屋子,墙在 BEV 视野(左右 4 m)外。
    right_turn:镜像成右拐(open_left 这时指弯内侧,也就是右边);front:出发点离前墙多远(决定进口直道长)。"""
    from simulation.fastsim.road import EDGE, WALL, build
    lead = front - width + 1.5
    inner = EDGE if open_left else WALL
    mat = (WALL, inner) if right_turn else (inner, WALL)
    elems = [dict(kind='straight', length=lead, width=width, mat=mat),
             dict(kind='arc', angle=-90.0 if right_turn else 90.0, r_in=0.05, width=width, mat=mat),
             dict(kind='straight', length=EXIT, width=width, mat=mat)]
    road = build(elems)
    _square_corner(road, road.elems[1])
    road.region = (np.array([[-100.0, -100.0], [100.0, -100.0], [100.0, 100.0], [-100.0, 100.0]]),
                   np.array([[500.0, 500.0], [501.0, 500.0], [501.0, 501.0]]))
    return road


def _square_corner(road, e):
    """把弯外侧的圆角墙换成方角(同 road.add_pockets 的做法、口袋深度 0)。不直接用 add_pockets:
    它的"口袋压到别的路段"检查在出口直道紧接弯道时会误拒,结果外侧一直是半径 = 路宽的大圆弧,把车顺着引过去
    (2026-10-17 第一版 l_test 就是这样,全部模型 100% 过)。方角里那块也算地面(road.pockets)。"""
    from simulation.fastsim.road import WALL_STEP, _line_x, _resample
    sign = 1.0 if e['angle'] > 0 else -1.0
    i0, i1 = e['_i0'], e['_i1']
    outer = road.right if sign > 0 else road.left
    d0 = np.array([math.cos(road.heading[i0]), math.sin(road.heading[i0])])
    d1 = np.array([math.cos(road.heading[i1]), math.sin(road.heading[i1])])
    p0, p1 = outer[i0], outer[i1]
    b = _line_x(p0, d0, p1, d1)
    c = road.centre[i0] + sign * (e['r_in'] + e['width'] / 2) * road.normal[i0]
    w = road.walls.reshape(-1, 2) - c
    on_arc = ((np.abs(np.hypot(w[:, 0], w[:, 1]) - (e['r_in'] + e['width'])) < 0.05) & (w @ d0 > 1e-3) & (w @ d1 < -1e-3))
    add = []
    for q0, q1 in ((p0 - WALL_STEP * d0, b), (b, p1 + WALL_STEP * d1)):
        pts, _ = _resample(np.array([q0, q1]), WALL_STEP)
        add.append(np.stack([pts[:-1], pts[1:]], axis=1))
    road.walls = np.concatenate([road.walls[~on_arc.reshape(-1, 2).any(axis=1)]] + add)
    road.pockets = np.array([[p0, b, b, b, p1]], float)


def run_one(args):
    model, width, yaw, seed, open_left = args
    from common.bev_policy import BevPolicy
    from common.road_policy import V_LO, Encoding
    from simulation.fastsim.road_env import Plant
    from training.bev_env import BevEnv
    road = ell(width, open_left)
    pl = replace(Plant.sample(np.random.default_rng(seed), Encoding(3.0, V_LO, 3.0)), cap=0.7)
    if model != 'teacher':
        pl = replace(pl, enc=BevPolicy(model).enc)    # 动作按模型自己的速度契约解码(带 _d 的模型能停车 / 刹车);老模型的契约和这里一样,结果逐位不变
    dist = FRONT - width - 0.05                          # 后轴到弯道起点(弯内角);前墙在弯起点外 r_in + 路宽
    i0 = road.elems[1]['_i0'] - int(round(dist / 0.05))
    off = -width / 2 + RIGHT                             # 横向:左正
    b = BevEnv(road, pl, seed, (i0, off, math.radians(yaw)), noise=True, free=True, seg_period=0.33,
               seg_lag=0.35, yolo=False, guard=True, seg_lut=True, seg_fill=True)     # 地面分割按车上的投影 + 补缝
    if model == 'teacher':
        from common.params import CarParams
        from training.bev_env import SeenTeacher
        from training.road_teacher import R_MIN, plan
        P = plan(road, i0, off, CarParams().width, pl, r_min=R_MIN, a_lat=1.47)
        if isinstance(P, str):
            return 'no_plan', []
        t = SeenTeacher(b, P, 1.47)
        pol = lambda img, sc: t()
    else:
        net = BevPolicy(model)
        pol = net
    why, path = None, []
    while why is None and b.env.t < 30.0:
        why = b.step(pol(*b.observation()))
        path.append(b.pose)
    return why or 'timeout', np.array(path)


def case(seed: int) -> dict:
    """一局加难的 L 弯(run 用):左 / 右弯、离前墙、路宽、朝向都按种子抽;路宽 >= 5 m 时弯内侧敞开(同实车 seg_06)。"""
    rng = np.random.default_rng([seed, 11])
    width = float(rng.choice((2.5, 3.5, 4.5, 5.0, 6.0)))
    return dict(seed=seed, right_turn=bool(rng.random() < 0.5), width=width, open_left=width >= 5.0,
                front=round(float(rng.uniform(5.0, 9.0)), 2), yaw=round(float(rng.uniform(-10.0, 10.0)), 1))


def run_case(args):
    """一局。返回结局、转向起始点(离前墙多远 steer_gap / 放行后几秒 steer_t,没打就 None)、车头转过 TURNED 时
    离前墙多远(onset)、结束时离前墙多远、最后一拍的转向和车速。"""
    model, c, cap = args
    from common.bev_policy import BevPolicy
    from common.params import CarParams
    from common.road_policy import V_LO, Encoding
    from simulation.fastsim.road_env import Plant
    from training.bev_env import BevEnv
    road = ell(c['width'], c['open_left'], c['front'], c['right_turn'])
    sign = -1.0 if c['right_turn'] else 1.0
    pl = replace(Plant.sample(np.random.default_rng(c['seed']), Encoding(3.0, V_LO, 3.0)), cap=cap)
    pol = None
    if model != 'teacher':
        pol = BevPolicy(model)
        pl = replace(pl, enc=pol.enc)                      # 动作按模型自己的速度契约解码
    i0 = road.elems[1]['_i0'] - int(round((c['front'] - c['width'] - 0.05) / 0.05))
    off = sign * (-c['width'] / 2 + RIGHT)                 # 离外侧墙 RIGHT 米
    b = BevEnv(road, pl, c['seed'], (i0, off, math.radians(c['yaw'])), noise=True, free=True, seg_period=0.33,
               seg_lag=0.35, yolo=False, guard=True, seg_lut=True, seg_fill=True)
    cp = CarParams()
    if pol is None:
        from training.bev_env import SeenTeacher
        from training.road_teacher import R_MIN, plan
        P = plan(road, i0, off, cp.width, pl, r_min=R_MIN, a_lat=1.47)
        if isinstance(P, str):
            return dict(c, why='no_plan', onset=None, steer_gap=None, steer_t=None, gap_end=None, steer_end=None, v_end=None)
        t = SeenTeacher(b, P, 1.47)
        pol = lambda img, sc: t()
    h0 = road.heading[i0]
    d0 = np.array([math.cos(h0), math.sin(h0)])
    wall = float(road.pockets[0][1] @ d0)                  # 前墙(方角)在进口方向上的位置
    nose = cp.length - cp.rear_overhang                    # 后轴 -> 车最前端
    car = b.env.car

    def gap():
        return wall - (float(np.array([car.x, car.y]) @ d0) + nose * math.cos(car.yaw - h0))

    def turned():
        return sign * ((car.yaw - h0 + math.pi) % (2 * math.pi) - math.pi)

    why, onset, a = None, None, np.zeros(2)
    steer, run = None, None                                # run:当前这一段朝弯打方向的起点(离前墙, 时刻)
    while why is None and b.env.t < 30.0:
        a = pol(*b.observation())
        if onset is None:
            if sign * float(a[0]) > STEER_ON and turned() >= 0.0:
                run = run or (gap(), b.env.t)
            else:
                run = None
        why = b.step(a)
        if onset is None and turned() > TURNED:
            onset, steer = gap(), run
    return dict(c, why=why or 'timeout', onset=None if onset is None else round(onset, 3),
                steer_gap=None if steer is None else round(steer[0], 3),
                steer_t=None if steer is None else round(steer[1], 2),
                gap_end=round(gap(), 3), steer_end=round(float(a[0]), 3), v_end=round(float(car.v), 3))


def paired(a: list, b: list, n_boot: int = 2000, seed: int = 0) -> dict:
    """同一批局(按种子对齐)的两组结果:通过率差 b - a(配对 bootstrap 95% 区间、McNemar 不一致对),
    转向起始点 steer_gap 和车头转过 TURNED 的位置 onset 的差 b - a(两边都有的局,配对 bootstrap 95% 区间;
    正数 = b 离墙更远就开始转)。"""
    ka, kb = {r['seed']: r for r in a}, {r['seed']: r for r in b}
    common = [s for s in kb if s in ka]
    ok_a = np.array([ka[s]['why'] == 'goal' for s in common], float)
    ok_b = np.array([kb[s]['why'] == 'goal' for s in common], float)
    rng = np.random.default_rng(seed)
    d_ok = (ok_b - ok_a)[rng.integers(0, len(common), (n_boot, len(common)))].mean(1)
    out = dict(n=len(common), pass_a=float(ok_a.mean()), pass_b=float(ok_b.mean()), d_pass=float((ok_b - ok_a).mean()),
               d_pass_ci=[float(np.percentile(d_ok, 2.5)), float(np.percentile(d_ok, 97.5))],
               only_a=int(((ok_a == 1) & (ok_b == 0)).sum()), only_b=int(((ok_a == 0) & (ok_b == 1)).sum()))
    for k in ('steer_gap', 'onset'):
        both = [s for s in common if ka[s].get(k) is not None and kb[s].get(k) is not None]
        out[f'n_{k}'] = len(both)
        if both:
            d = np.array([kb[s][k] - ka[s][k] for s in both])
            bd = d[rng.integers(0, len(d), (n_boot, len(d)))].mean(1)
            out.update({f'{k}_a': float(np.mean([ka[s][k] for s in both])),
                        f'{k}_b': float(np.mean([kb[s][k] for s in both])), f'd_{k}': float(d.mean()),
                        f'd_{k}_ci': [float(np.percentile(bd, 2.5)), float(np.percentile(bd, 97.5))]})
    return out


def _sampled(argv):
    import json
    ap = argparse.ArgumentParser(prog='l_test run / compare')
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('--model', required=True, help='npz 或 teacher')
    r.add_argument('--cap', type=float, default=0.7)
    r.add_argument('--count', type=int, default=300)
    r.add_argument('--select', action='store_true', help='用挑 candidate 的种子段(不碰测试种子)')
    r.add_argument('--workers', type=int, default=8)
    r.add_argument('--out', required=True)
    c = sub.add_parser('compare')
    c.add_argument('a')
    c.add_argument('b')
    a = ap.parse_args(argv)
    if a.cmd == 'run':
        seed0 = SELECT_SEED0 if a.select else TEST_SEED0
        cases = [case(seed0 + i) for i in range(a.count)]
        with ProcessPoolExecutor(a.workers) as ex:
            rows = list(ex.map(run_case, [(a.model, k, a.cap) for k in cases], chunksize=4))
        ok = [r['why'] == 'goal' for r in rows]
        st = [r for r in rows if r['steer_gap'] is not None]
        on = [r['onset'] for r in rows if r['onset'] is not None]
        reasons = dict(Counter(r['why'] for r in rows))
        print(f'{os.path.basename(a.model)} 限速 {a.cap}: 过 {np.mean(ok):.1%} ({sum(ok)}/{len(ok)}) {reasons}  '
              f'转向起始点 离前墙 {np.mean([r["steer_gap"] for r in st]):.2f} m / 放行后 '
              f'{np.mean([r["steer_t"] for r in st]):.2f} s(n={len(st)})  车头转过 15° 离前墙 {np.mean(on):.2f} m')
        with open(a.out, 'w', encoding='utf-8') as f:
            json.dump(dict(model=a.model, cap=a.cap, seed0=seed0, rows=rows), f, ensure_ascii=False)
    else:
        ra, rb = (json.load(open(p, encoding='utf-8')) for p in (a.a, a.b))
        assert ra['cap'] == rb['cap'] and ra['seed0'] == rb['seed0'], '限速 / 种子段不同,不能配对'
        print(json.dumps(paired(ra['rows'], rb['rows']), ensure_ascii=False, indent=1))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('models', nargs='+')
    ap.add_argument('--seeds', type=int, default=2)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--open-left', action='store_true', help='左边不立墙(EDGE),配 --widths 5 6')
    ap.add_argument('--widths', type=float, nargs='+', default=list(WIDTHS))
    import sys
    argv = sys.argv[1:] if argv is None else argv
    if argv and argv[0] in ('run', 'compare'):
        return _sampled(argv)
    a = ap.parse_args(argv)
    grid = list(itertools.product(a.widths, YAWS, range(a.seeds)))
    with ProcessPoolExecutor(a.workers) as ex:
        for m in a.models:
            res = list(ex.map(run_one, [(m, w, y, 7000 + s, a.open_left) for w, y, s in grid]))
            ok = [r[0] == 'goal' for r in res]
            print(f'{os.path.basename(m)}: 过 {np.mean(ok):.0%} ({sum(ok)}/{len(ok)})  {dict(Counter(r[0] for r in res))}')
            for w in a.widths:
                sub = [r[0] for (ww, _, _), r in zip(grid, res) if ww == w]
                print(f'   路宽 {w:.1f} m: 过 {np.mean([x == "goal" for x in sub]):.0%}  {dict(Counter(sub))}')


if __name__ == '__main__':
    main()
