"""bev_v1 的 DAgger 训练:老师 = SeenTeacher(只用已扫到信息),学生 = BEV 卷积网络。

    python -m training.bev_train --out runs/bev_v1_a --rounds 40 --workers 7

流程和 road_train 一样:每轮按 beta 混合老师/学生采一批局(β 1 -> 0,前几轮基本是老师开 =
行为克隆式预训练),老师逐拍给标签进回放池,训 updates 步,固定种子评估,导出 candidate_XXX.npz。
回放池里图按位压缩(5×80×80 个 0/1 -> 4000 字节),30 万帧约 1.2 GB。
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from common.bev_policy import C, H, N_SCALAR, W, forward
from common.road_policy import V_LO, Encoding
from simulation.fastsim.road import LEVELS
from training.bev_env import BevEnv, SeenTeacher
from training.road_check import sample_episode

TRAIN_SEED0, SELECT_SEED0 = 30_000_000, 60_000_000     # 和 road_train(10M/50M)、测试(90M+)都不重叠
BITS = C * H * W


def run_episode(seed, level, weights=None, beta=0.0, collect=False, enc=None, a_lat=0.45, cap_max=None):
    try:
        road, pl, start, P, _ = sample_episode(seed, level, enc=enc, a_lat=a_lat, cap_max=cap_max)
    except RuntimeError:
        return None
    benv = BevEnv(road, pl, seed, start, noise=True)
    teacher = SeenTeacher(benv, P, a_lat)
    rng = np.random.default_rng(seed + 1)
    imgs, scs, ys = [], [], []
    why, n = None, 0
    while why is None:
        img, sc = benv.observation()
        label = teacher()
        if collect:
            imgs.append(np.packbits(img.astype(bool).ravel()))
            scs.append(sc)
            ys.append(label)
        if weights is None or rng.random() < beta:
            act = label + (rng.normal(0, [0.05, 0.05]) if collect else 0.0)
        else:
            act = forward(weights, img, sc)
        why = benv.step(act)
        n += 1
    row = dict(seed=seed, level=level, reason=why, cap=pl.cap, time=benv.env.t, steps=n)
    if collect:
        return np.asarray(imgs), np.asarray(scs, np.float32), np.asarray(ys, np.float32), row
    return row


def _job(args):
    return run_episode(*args)


class Replay:
    def __init__(self, cap):
        self.img = np.zeros((cap, BITS // 8), np.uint8)
        self.sc = np.zeros((cap, N_SCALAR), np.float32)
        self.y = np.zeros((cap, 2), np.float32)
        self.size = self.ptr = 0

    def add(self, img, sc, y):
        for i in range(len(y)):
            self.img[self.ptr], self.sc[self.ptr], self.y[self.ptr] = img[i], sc[i], y[i]
            self.ptr = (self.ptr + 1) % len(self.y)
            self.size = min(len(self.y), self.size + 1)

    def batch(self, rng, n):
        i = rng.integers(0, self.size, n)
        img = np.unpackbits(self.img[i], axis=1)[:, :BITS].reshape(n, C, H, W).astype(np.float32)
        return img, self.sc[i], self.y[i]


def summarize(rows):
    ok = [r['reason'] == 'goal' for r in rows]
    return dict(n=len(rows), success=float(np.mean(ok)) if rows else 0.0,
                reasons={k: sum(r['reason'] == k for r in rows) for k in {r['reason'] for r in rows}})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--rounds', type=int, default=40)
    ap.add_argument('--episodes-per-round', type=int, default=168)
    ap.add_argument('--updates', type=int, default=1500)
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--replay', type=int, default=300_000)
    ap.add_argument('--ema', type=float, default=0.999)
    ap.add_argument('--workers', type=int, default=7)
    ap.add_argument('--torch-threads', type=int, default=4)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--speed-scale', type=float, default=3.0)
    ap.add_argument('--v-hi', type=float, default=3.0)
    ap.add_argument('--a-lat', type=float, default=0.45)
    ap.add_argument('--cap-max', type=float, default=1.8)
    ap.add_argument('--mix', default='L5:0.2,L5s:0.2,L6h:0.15,L6f:0.15,L4:0.15,L6:0.1,L3:0.05')
    ap.add_argument('--eval-levels', default='L5,L6h,L4c,L6f')
    ap.add_argument('--eval-count', type=int, default=192)
    a = ap.parse_args()
    enc = Encoding(a.speed_scale, V_LO, a.v_hi)

    import torch
    from training.bev_net import export, make_net, weights_of
    torch.set_num_threads(a.torch_threads)
    torch.manual_seed(a.seed)
    rng = np.random.default_rng(a.seed)
    net, ema = make_net(), make_net()
    ema.load_state_dict(net.state_dict())
    opt = torch.optim.Adam(net.parameters(), lr=a.lr)
    a.out.mkdir(parents=True, exist_ok=False)
    cfg = {k: str(v) for k, v in vars(a).items()}
    cfg.update(enc=tuple(enc), git_head=os.popen('git rev-parse HEAD').read().strip())
    (a.out / 'config.json').write_text(json.dumps(cfg, indent=1), encoding='utf-8')
    log = open(a.out / 'train.log', 'a', encoding='utf-8', buffering=1)

    def say(msg):
        line = f'{time.strftime("%H:%M:%S")} {msg}'
        print(line, flush=True)
        log.write(line + '\n')

    mix = [(n, float(p)) for n, p in (x.split(':') for x in a.mix.split(','))]
    probs = np.array([p for _, p in mix]) / sum(p for _, p in mix)
    names = {l.name for l in LEVELS}
    assert all(n in names for n, _ in mix)
    ev_levels = a.eval_levels.split(',')
    replay = Replay(a.replay)
    brng = np.random.default_rng(a.seed + 1)
    wy = torch.tensor([1.0, 0.5])
    seed = TRAIN_SEED0
    with ProcessPoolExecutor(a.workers) as ex:
        for r in range(a.rounds):
            beta = max(0.0, 1.0 - r / 4)
            wts = None if r == 0 else weights_of(ema)
            jobs = []
            for _ in range(a.episodes_per_round):
                jobs.append((seed, mix[int(rng.choice(len(mix), p=probs))][0], wts, beta, True, enc,
                             a.a_lat, a.cap_max))
                seed += 1
            t0 = time.time()
            got = [g for g in ex.map(_job, jobs, chunksize=2) if g is not None]
            for img, sc, y, _ in got:
                replay.add(img, sc, y)
            collect = summarize([g[3] for g in got])
            t_collect = time.time() - t0

            t0 = time.time()
            losses = []
            for _ in range(a.updates):
                img, sc, y = replay.batch(brng, a.batch)
                pred = net(torch.from_numpy(img), torch.from_numpy(sc))
                loss = (((pred - torch.from_numpy(y)) ** 2) * wy).mean()
                opt.zero_grad()
                loss.backward()
                opt.step()
                with torch.no_grad():
                    for pe, pn in zip(ema.parameters(), net.parameters()):
                        pe.mul_(a.ema).add_(pn, alpha=1 - a.ema)
                losses.append(loss.item())
            t_update = time.time() - t0

            t0 = time.time()
            wts = weights_of(ema)
            ev = [e for e in ex.map(_job, [(SELECT_SEED0 + i, ev_levels[i % len(ev_levels)], wts, 0.0, False,
                                             enc, a.a_lat, a.cap_max) for i in range(a.eval_count)],
                                    chunksize=2) if e is not None]
            summ = summarize(ev)
            by = {n: summarize([e for e in ev if e['level'] == n])['success'] for n in ev_levels}
            t_eval = time.time() - t0
            export(ema, a.out / f'candidate_{r:03d}.npz', enc)
            say(f'round {r} beta={beta:.2f} samples={replay.size} loss={np.mean(losses[-200:]):.4f} '
                f'collect={collect["success"]:.3f} eval={summ["success"]:.3f} {by} {summ["reasons"]} '
                f'[{t_collect:.0f}s/{t_update:.0f}s/{t_eval:.0f}s]')
    say('done')


if __name__ == '__main__':
    main()
