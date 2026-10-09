"""随机路段配对测试集:同一批局(道路 + 车辆参数 + 起点)、同一噪声种子上比较两个模型。

    python -m evaluation.bev_testset build --out runs/testset_93M.pkl              # 4 级 × 400 局
    python -m evaluation.bev_testset run --eps runs/testset_93M.pkl --model models/road_v2.5/candidate_b036.npz \
        --out runs/test_b036.json
    python -m evaluation.bev_testset run --eps runs/testset_93M.pkl --model runs/bev_v1_d/candidate_014.npz \
        --out runs/test_d014.json
    python -m evaluation.bev_testset compare runs/test_b036.json runs/test_d014.json

局的生成:training.road_check.sample_episode(seed, level, enc=road_v2.5 契约, cap_max=1.8),seed = seed0 + i,
每级用同一串 seed(老师 a_lat 用 sample_episode 的默认值;它参与可通性拒绝采样,改了就是另一批局)。
build 打印并存下指纹(道路、车辆参数、起点逐局摘要的 md5):同指纹 = 同一批局,不必比对 pickle 本身。
road_v2.x 模型吃 RoadEnv 的扇区观测,bev_v1 模型套 BevEnv;两者噪声种子都用局的 seed。
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
    jobs = [(a.seed0 + i, L) for L in LEVELS for i in range(a.per_level)]
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
    i, model = args
    ep = _EPS[i]
    with np.load(model) as z:
        schema = str(z['schema'])
    if schema.startswith('bev_v1'):
        from common.bev_policy import BevPolicy
        from training.bev_env import BevEnv
        pol = BevPolicy(model)
        b = BevEnv(ep['road'], ep['pl'], ep['seed'], ep['start'], noise=True)
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
    levels = [e['level'] for e in eps]
    with ProcessPoolExecutor(a.workers, initializer=_init, initargs=(a.eps,)) as ex:
        rows = list(ex.map(_run, [(i, a.model) for i in range(len(eps))], chunksize=2))
    for L in dict.fromkeys(levels):
        r = [x for x, l in zip(rows, levels) if l == L]
        print(L, 'n=%d ok=%.3f' % (len(r), np.mean([x['reason'] == 'goal' for x in r])))
    print('all %.4f' % np.mean([x['reason'] == 'goal' for x in rows]))
    with open(a.out, 'w') as f:
        json.dump(dict(model=a.model, fingerprint=fp, levels=levels, rows=rows), f)


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
    ok = [[r['reason'] == 'goal' for r in sorted(X['rows'], key=lambda r: r['i'])] for X in (A, B)]
    n01, n10, p = mcnemar(*ok)
    print(f"A {A['model']}: {np.mean(ok[0]):.4f}\nB {B['model']}: {np.mean(ok[1]):.4f}\n"
          f"A 败 B 胜 {n01} / A 胜 B 败 {n10},McNemar p = {p:.4g}(n = {len(ok[0])})")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    b = sub.add_parser('build')
    b.add_argument('--out', required=True)
    b.add_argument('--per-level', type=int, default=400)
    b.add_argument('--seed0', type=int, default=SEED0)
    b.add_argument('--workers', type=int, default=8)
    r = sub.add_parser('run')
    r.add_argument('--eps', required=True)
    r.add_argument('--model', required=True)
    r.add_argument('--out', required=True)
    r.add_argument('--workers', type=int, default=8)
    c = sub.add_parser('compare')
    c.add_argument('a')
    c.add_argument('b')
    a = ap.parse_args()
    {'build': build, 'run': run, 'compare': compare}[a.cmd](a)


if __name__ == '__main__':
    main()
