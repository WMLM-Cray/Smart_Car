"""路段策略 v2:并行 DAgger + 按几何难度推进的课程。

    python -m training.road_train --out runs/road_v2_a --workers 14
    python -m training.road_train --out runs/road_v2_a --workers 14 --resume

每轮:
  1 采样  workers 个进程并行跑 episodes_per_round 局。每局先过 road_check 的三层可通性检查;
          按 beta 概率逐拍用老师动作(带小噪声)或学生动作开车,每拍都记老师标签
  2 更新  全部数据进回放池(FIFO),MLP 做 updates 次小批量回归
  3 评估  学生独立开 eval_count 局(挑模型用的种子,不和训练重叠),按速度上限分三档统计
  4 课程  当前级别总通过率 >= 0.95 且每档 >= 0.90 就升一级。训练里 70% 当前级、30% 以前各级

速度上限每局在 [0.45, 1.0] 均匀随机,从第一轮就是,不按速度分阶段。
种子:训练 10M 起,挑模型 50M 起,最终测试 90M 起(这里不用)。
"""
from __future__ import annotations

import os

# 每个采样进程只用一个 BLAS 线程:32 个进程 x 32 线程会把机器压到负载 400(2026-09-25 实测)。
# 必须在 import numpy 之前设。
for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from simulation.road import LEVELS
from training.road_check import sample_episode
from simulation.road_env import RoadEnv
from common.road_policy import DIM, FRAMES, SCHEMA, History, forward
from training.road_teacher import Teacher

TRAIN_SEED0, SELECT_SEED0 = 10_000_000, 50_000_000
SPEED_BINS = ((0.45, 0.6), (0.6, 0.8), (0.8, 1.01))
PROMOTE_ALL, PROMOTE_BIN = 0.95, 0.90


def run_episode(seed, level, weights=None, beta=0.0, collect=False):
    road, pl, start, P, rejects = sample_episode(seed, level)
    env = RoadEnv(road, pl, seed, start, noise=True)
    teacher = Teacher(env, P)
    hist = History()
    rng = np.random.default_rng(seed + 1)
    xs, ys = [], []
    why, n = None, 0
    while why is None:
        x = hist.push(env.observation())
        label = teacher()
        if collect:
            xs.append(x)
            ys.append(label)
        if weights is None or rng.random() < beta:
            act = label + (rng.normal(0, [0.05, 0.05]) if collect else 0.0)
        else:
            act = forward(weights[0], weights[1], x)
        why = env.step(act)
        n += 1
    row = dict(seed=seed, level=level, reason=why, cap=pl.cap, v_plateau=pl.v_plateau,
               min_clear=float(env.min_clear), time=env.t, steps=n,
               recovery=start[0] != 0 or start[1] != 0.0, rejects=dict(rejects),
               elems=[e['kind'] for e in road.elems])
    if collect:
        return np.asarray(xs, np.float32), np.asarray(ys, np.float32), row
    return row


def _job(args):
    return run_episode(*args)


def summarize(rows):
    ok = [r['reason'] == 'goal' for r in rows]
    out = dict(episodes=len(rows), success=float(np.mean(ok)) if rows else 0.0,
               reasons={k: sum(r['reason'] == k for r in rows)
                        for k in sorted({r['reason'] for r in rows})})
    bins = {}
    for lo, hi in SPEED_BINS:
        sel = [r['reason'] == 'goal' for r in rows if lo <= r['cap'] < hi]
        bins[f'{lo:.2f}-{min(hi, 1.0):.2f}'] = (float(np.mean(sel)) if sel else None, len(sel))
    out['by_cap'] = bins
    return out


def promoted(summary) -> bool:
    return summary['success'] >= PROMOTE_ALL and all(
        v is None or v >= PROMOTE_BIN for v, _ in summary['by_cap'].values())


class Replay:
    """固定大小的环形回放池。观测存 float16(都在 ±1 内,精度 1e-3 够用),省一半内存。

    以前是 list + 每轮 np.concatenate + 连回放一起 torch.save,3M 帧时峰值十几 GB,
    被 OOM 杀过两次(2026-09-25)。现在不拼接、断点里只存下标,数组单独 np.save。
    """

    def __init__(self, cap):
        self.x = np.zeros((cap, DIM * FRAMES), np.float16)
        self.y = np.zeros((cap, 2), np.float32)
        self.size = self.ptr = 0

    def add(self, x, y):
        for i in range(0, len(x), len(self.x)):
            xs, ys = x[i:i + len(self.x)], y[i:i + len(self.x)]
            idx = (self.ptr + np.arange(len(xs))) % len(self.x)
            self.x[idx], self.y[idx] = xs, ys
            self.ptr = int((self.ptr + len(xs)) % len(self.x))
            self.size = min(len(self.x), self.size + len(xs))

    def batch(self, rng, n):
        i = rng.integers(0, self.size, n)
        return self.x[i].astype(np.float32), self.y[i]

    def save(self, out):
        for name, arr in (('replay_x', self.x), ('replay_y', self.y)):
            with open(out / f'{name}.tmp', 'wb') as f:
                np.save(f, arr[:self.size])
            os.replace(out / f'{name}.tmp', out / f'{name}.npy')
        return dict(size=self.size, ptr=self.ptr)

    def load(self, out, meta):
        x = np.load(out / 'replay_x.npy', mmap_mode='r')
        n = min(len(x), len(self.x))
        self.x[:n] = x[:n]
        self.y[:n] = np.load(out / 'replay_y.npy')[:n]
        self.size, self.ptr = n, meta['ptr'] % len(self.x)


def make_net():
    import torch
    from torch import nn
    return nn.Sequential(nn.Linear(DIM * FRAMES, 256), nn.Tanh(),
                         nn.Linear(256, 256), nn.Tanh(), nn.Linear(256, 2))


def weights_of(net):
    layers = [net[i] for i in (0, 2, 4)]
    return ([l.weight.detach().numpy().copy() for l in layers],
            [l.bias.detach().numpy().copy() for l in layers])


def export(net, path):
    w, b = weights_of(net)
    np.savez(path, schema=SCHEMA, **{f'w{i}': w[i] for i in range(3)},
             **{f'b{i}': b[i] for i in range(3)})


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--rounds', type=int, default=60)
    ap.add_argument('--episodes-per-round', type=int, default=240)
    ap.add_argument('--updates', type=int, default=3000)
    ap.add_argument('--batch', type=int, default=1024)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--replay', type=int, default=3_000_000)
    ap.add_argument('--eval-count', type=int, default=200)
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--torch-threads', type=int, default=8)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--resume', action='store_true')
    ap.add_argument('--init', type=Path, default=None, help='从这个 road_v2 .npz 热启动(新目录,回放从零)')
    ap.add_argument('--start-level', default='L1', choices=[l.name for l in LEVELS])
    ap.add_argument('--mix', default=None,
                    help="固定配比,不走升级:'L6:0.4,L5:0.35,L4:0.15,L3:0.1';评估用 --start-level 那一级")
    ap.add_argument('--eval-levels', default=None,
                    help="评估分摊到这几级(逗号分隔),日志逐级报;默认只评当前级")
    ap.add_argument('--ema', type=float, default=0.0,
                    help='权重滑动平均系数(每次更新);>0 时采样/评估/导出都用平均后的权重。'
                         '单个存档在赛道 A 方角上忽好忽坏(v2_f 第 10 轮 100%%、第 25 轮 0%%)')
    a = ap.parse_args()

    import torch
    torch.set_num_threads(a.torch_threads)
    torch.manual_seed(a.seed)
    rng = np.random.default_rng(a.seed)
    net = make_net()
    opt = torch.optim.Adam(net.parameters(), lr=a.lr)
    ema = make_net()
    ema.load_state_dict(net.state_dict())
    a.out.mkdir(parents=True, exist_ok=True)
    ckpt = a.out / 'latest.pt'
    state = dict(round=0, level=[l.name for l in LEVELS].index(a.start_level),
                 next_seed=TRAIN_SEED0, replay=None, history=[])
    replay = Replay(a.replay)
    batch_rng = np.random.default_rng(a.seed + 1)
    if a.resume:
        s = torch.load(ckpt, map_location='cpu', weights_only=False)
        net.load_state_dict(s['net'])
        ema.load_state_dict(s.get('ema', s['net']))
        opt.load_state_dict(s['opt'])
        rng.bit_generator.state = s['numpy_rng']
        state = s['state']
        if 'replay_x' in state:             # 旧格式断点:回放是 list,转进环形池
            for x, y in zip(state.pop('replay_x'), state.pop('replay_y')):
                replay.add(x, y)
        else:
            replay.load(a.out, state['replay'])
        del s
    elif ckpt.exists():
        raise SystemExit(f'{a.out} 里已经有断点,续训加 --resume,新实验换目录')
    else:
        cfg = {k: str(v) if isinstance(v, Path) else v for k, v in vars(a).items()}
        cfg.update(schema=SCHEMA, levels=[l.name for l in LEVELS],
                   git_head=os.popen('git rev-parse HEAD').read().strip())
        (a.out / 'config.json').write_text(json.dumps(cfg, indent=1), encoding='utf-8')
        if a.init is not None:
            with np.load(a.init) as z:
                if str(z['schema']) != SCHEMA:
                    raise SystemExit(f'{a.init} 不是 {SCHEMA}')
                with torch.no_grad():
                    for i, k in enumerate((0, 2, 4)):
                        net[k].weight.copy_(torch.from_numpy(z[f'w{i}']))
                        net[k].bias.copy_(torch.from_numpy(z[f'b{i}']))
                ema.load_state_dict(net.state_dict())

    log = open(a.out / 'train.log', 'a', encoding='utf-8', buffering=1)

    def say(msg):
        line = f'{time.strftime("%H:%M:%S")} {msg}'
        print(line, flush=True)
        log.write(line + '\n')

    with ProcessPoolExecutor(a.workers) as ex:
        while state['round'] < a.rounds:
            r, lv = state['round'], state['level']
            beta = max(0.0, 1.0 - 0.25 * r)
            pol = ema if a.ema > 0 else net
            wts = None if r == 0 and a.init is None else weights_of(pol)
            jobs = []
            mix = None if a.mix is None else [(n, float(p)) for n, p in
                                              (x.split(':') for x in a.mix.split(','))]
            for _ in range(a.episodes_per_round):
                if mix is not None:
                    name = mix[int(rng.choice(len(mix), p=np.array([p for _, p in mix]) / sum(p for _, p in mix)))][0]
                    level = next(l for l in LEVELS if l.name == name)
                else:
                    level = LEVELS[lv] if lv == 0 or rng.random() < 0.7 else LEVELS[int(rng.integers(lv))]
                jobs.append((state['next_seed'], level.name, wts, beta, True))
                state['next_seed'] += 1
            t0 = time.time()
            rows = []
            for x, y, row in ex.map(_job, jobs):
                replay.add(x, y)
                rows.append(row)
            t_collect = time.time() - t0

            wy = torch.tensor([1.0, 0.5])
            t0 = time.time()
            losses = []
            for _ in range(a.updates):
                xb, yb = replay.batch(batch_rng, a.batch)
                loss = (((net(torch.from_numpy(xb)) - torch.from_numpy(yb)) ** 2) * wy).mean()
                opt.zero_grad()
                loss.backward()
                opt.step()
                if a.ema > 0:
                    with torch.no_grad():
                        for pe, p in zip(ema.parameters(), net.parameters()):
                            pe.mul_(a.ema).add_(p, alpha=1 - a.ema)
                losses.append(loss.item())
            t_update = time.time() - t0

            t0 = time.time()
            wts = weights_of(pol)
            level = LEVELS[lv].name
            ev_levels = a.eval_levels.split(',') if a.eval_levels else [level]
            ev = list(ex.map(_job, [(SELECT_SEED0 + i, ev_levels[i % len(ev_levels)], wts, 0.0, False)
                                    for i in range(a.eval_count)]))
            summ = summarize(ev)
            if a.eval_levels:
                summ['by_level'] = {n: summarize([e for e in ev if e['level'] == n])['success']
                                    for n in ev_levels}
            t_eval = time.time() - t0

            export(pol, a.out / f'candidate_{r:03d}.npz')
            report = dict(round=r, level=level, beta=beta, samples=replay.size,
                          loss=float(np.mean(losses[-200:])), collect=summarize(rows),
                          eval=summ, seconds=dict(collect=t_collect, update=t_update, eval=t_eval),
                          episodes=ev)
            (a.out / f'eval_{r:03d}.json').write_text(json.dumps(report, indent=1), encoding='utf-8')
            say(f'round {r} {level} beta={beta:.2f} samples={replay.size} loss={report["loss"]:.4f} '
                f'eval={summ["success"]:.3f} {summ.get("by_level", "")} {summ["by_cap"]} {summ["reasons"]} '
                f'[{t_collect:.0f}s/{t_update:.0f}s/{t_eval:.0f}s]')
            state['history'].append(dict(round=r, level=level, success=summ['success']))
            if a.mix is None and promoted(summ) and lv + 1 < len(LEVELS):
                state['level'] = lv + 1
                say(f'升级 -> {LEVELS[lv + 1].name}')
            state['round'] = r + 1
            state['replay'] = replay.save(a.out)
            tmp = a.out / 'latest.tmp'
            torch.save(dict(net=net.state_dict(), ema=ema.state_dict(), opt=opt.state_dict(),
                            numpy_rng=rng.bit_generator.state, state=state), tmp)
            os.replace(tmp, ckpt)
    say('done')


if __name__ == '__main__':
    main()
