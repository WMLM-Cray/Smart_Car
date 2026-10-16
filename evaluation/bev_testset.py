"""随机路段配对测试集:同一批局(道路 + 车辆参数 + 起点)、同一噪声种子上比较两个模型。

    python -m evaluation.bev_testset build --out runs/testset_93M.pkl              # 4 级 × 400 局
    python -m evaluation.bev_testset run --eps runs/testset_93M.pkl --model models/road_v2.5/candidate_b036.npz \
        --out runs/test_b036.json
    python -m evaluation.bev_testset run --eps runs/testset_93M.pkl --model runs/bev_v1_d/candidate_014.npz \
        --out runs/test_d014.json
    python -m evaluation.bev_testset compare runs/test_b036.json runs/test_d014.json
    # bev_v2 按实车地面分割节奏跑(默认 0.1 s、无延迟 = 训练设定):--seg-period 0.33 --seg-lag 0.35

局的生成:training.road_check.sample_episode(seed, level, enc=road_v2.5 契约, cap_max=1.8),seed = seed0 + i,
每级用同一串 seed(老师 a_lat 用 sample_episode 的默认值;它参与可通性拒绝采样,改了就是另一批局)。
build 打印并存下指纹(道路、车辆参数、起点逐局摘要的 md5):同指纹 = 同一批局,不必比对 pickle 本身。
road_v2.x 模型吃 RoadEnv 的扇区观测,bev_v1/v2 模型套 BevEnv(v2 带假地面分割);两者噪声种子都用局的 seed。
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import hashlib
import json
import math
import pickle
from concurrent.futures import ProcessPoolExecutor
from dataclasses import astuple

import numpy as np

LEVELS = ('L5', 'L6h', 'L4c', 'L6f')
SEED0 = 93_000_000              # 和训练(10M/30M)、挑模型(50M/60M)、RL 池(20M)、A-D(95M)都不重叠
CAP_MAX = 1.8
SCHEMA = 'road_v2.5'


def _one(args):
    from common.road_policy import spec_of
    from training.road_check import sample_episode
    seed, level = args
    road, pl, start, _, _ = sample_episode(seed, level, enc=spec_of(SCHEMA), cap_max=CAP_MAX)
    return dict(seed=seed, level=level, road=road, pl=pl, start=start)


def fingerprint(eps) -> str:
    h = hashlib.md5()
    for e in eps:
        r = e['road']
        h.update(f"{e['seed']} {e['level']} {tuple(e['start'])!r} {astuple(e['pl'])!r}".encode())
        for a in (r.cones, r.left, r.right):
            h.update(np.ascontiguousarray(np.round(a, 6)).tobytes())
    return h.hexdigest()


def build(a):
    jobs = [(a.seed0 + i, L) for L in a.levels.split(',') for i in range(a.per_level)]
    with ProcessPoolExecutor(a.workers) as ex:
        eps = list(ex.map(_one, jobs, chunksize=4))
    with open(a.out, 'wb') as f:
        pickle.dump(eps, f)
    fp = fingerprint(eps)
    with open(a.out + '.md5', 'w') as f:
        f.write(fp + '\n')
    print(f'{len(eps)} 局 -> {a.out}  指纹 {fp}')


_EPS = None


def _init(path):
    global _EPS
    with open(path, 'rb') as f:
        _EPS = pickle.load(f)


def _run(args):
    i, model, seg_period, seg_lag, yolo, guard, seg_lut, seg_fill = args
    ep = _EPS[i]
    with np.load(model) as z:
        schema = str(z['schema'])
    if schema.startswith('bev_'):
        from common.bev_policy import BevPolicy
        from training.bev_env import BevEnv
        pol = BevPolicy(model)
        b = BevEnv(ep['road'], ep['pl'], ep['seed'], ep['start'], noise=True, free=pol.free,
                   seg_period=seg_period, seg_lag=seg_lag, yolo=yolo, guard=guard, seg_lut=seg_lut, seg_fill=seg_fill)
        why = None
        while why is None:
            why = b.step(pol(*b.observation()))
        t = b.env.t
    else:
        from common.road_policy import RoadPolicy
        from simulation.fastsim.road_env import RoadEnv
        pol = RoadPolicy(model)
        env = RoadEnv(ep['road'], ep['pl'], ep['seed'], ep['start'], noise=True)
        why = None
        while why is None:
            why = env.step(pol(env.observation()))
        t = env.t
    return dict(i=i, reason=why, t=t)


def run(a):
    with open(a.eps, 'rb') as f:
        eps = pickle.load(f)
    fp = fingerprint(eps)
    idx = range(0, len(eps), a.every)          # 每 every 局取一局(各级按顺序排,取样后各级比例不变)
    levels = [eps[i]['level'] for i in idx]
    with ProcessPoolExecutor(a.workers, initializer=_init, initargs=(a.eps,)) as ex:
        rows = list(ex.map(_run, [(i, a.model, a.seg_period, a.seg_lag, not a.no_yolo, a.guard, a.seg_lut, a.seg_fill) for i in idx],
                           chunksize=2))
    for L in dict.fromkeys(levels):
        r = [x for x, l in zip(rows, levels) if l == L]
        print(L, 'n=%d ok=%.3f' % (len(r), np.mean([x['reason'] == 'goal' for x in r])))
    print('all %.4f' % np.mean([x['reason'] == 'goal' for x in rows]))
    with open(a.out, 'w') as f:
        json.dump(dict(model=a.model, fingerprint=fp, seg_period=a.seg_period, seg_lag=a.seg_lag, yolo=not a.no_yolo, guard=a.guard, seg_lut=a.seg_lut, seg_fill=a.seg_fill, every=a.every,
                       levels=levels, rows=rows), f)


def mcnemar(a_ok, b_ok):
    """返回 (A 败 B 胜, A 胜 B 败, 双侧精确 p)。"""
    n01 = sum(1 for x, y in zip(a_ok, b_ok) if not x and y)
    n10 = sum(1 for x, y in zip(a_ok, b_ok) if x and not y)
    n = n01 + n10
    p = 1.0 if n == 0 else min(1.0, 2 * sum(math.comb(n, k) for k in range(min(n01, n10) + 1)) / 2 ** n)
    return n01, n10, p


def compare(a):
    A, B = (json.load(open(p)) for p in (a.a, a.b))
    if A['fingerprint'] != B['fingerprint']:
        raise SystemExit('两份结果不是同一批局(指纹不同),不能配对比较')
    common = {r['i'] for r in A['rows']} & {r['i'] for r in B['rows']}     # 一方是 --every 取样时只比共同的局
    ok = [[r['reason'] == 'goal' for r in sorted(X['rows'], key=lambda r: r['i']) if r['i'] in common] for X in (A, B)]
    n01, n10, p = mcnemar(*ok)
    print(f"A {A['model']}: {np.mean(ok[0]):.4f}\nB {B['model']}: {np.mean(ok[1]):.4f}\n"
          f"A 败 B 胜 {n01} / A 胜 B 败 {n10},McNemar p = {p:.4g}(n = {len(ok[0])})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('build')
    b.add_argument('--out', required=True)
    b.add_argument('--per-level', type=int, default=400)
    b.add_argument('--levels', default=','.join(LEVELS), help='无锥桶对照用 LE3,LE4')
    b.add_argument('--seed0', type=int, default=SEED0)
    b.add_argument('--workers', type=int, default=8)
    r = sub.add_parser('run')
    r.add_argument('--eps', required=True)
    r.add_argument('--model', required=True)
    r.add_argument('--out', required=True)
    r.add_argument('--workers', type=int, default=8)
    r.add_argument('--seg-period', type=float, default=0.1, help='bev_v2 假地面分割周期 s(实车 M2F-T 288 ~0.33)')
    r.add_argument('--seg-lag', type=float, default=0.0, help='假地面分割拍摄到登记的延迟 s(实车 ~0.35)')
    r.add_argument('--no-yolo', action='store_true', help='锥桶层恒空(车上没跑 YOLO 时的部署条件)')
    r.add_argument('--guard', action='store_true', help='按车上截停规则判失败(保险杠前 < 0.25 m = guard)')
    r.add_argument('--seg-lut', action='store_true', help='假地面分割按车上的像素投影出点(部署条件)')
    r.add_argument('--seg-fill', action='store_true', help='配 --seg-lut:补行间的缝(车上 --fill)')
    r.add_argument('--every', type=int, default=1,
                   help='每隔几局取一局(5 = 1600 局取 320):区分 80%% 和 90%% 用不着上千局,省时间')
    c = sub.add_parser('compare')
    c.add_argument('a')
    c.add_argument('b')
    a = ap.parse_args()
    {'build': build, 'run': run, 'compare': compare}[a.cmd](a)


if __name__ == '__main__':
    main()
