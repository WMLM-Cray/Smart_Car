"""road_v2.5 的 RL 精修(PPO):从 DAgger 模型出发,在 FastSim 里直接优化"又快又不撞"。

    python -m training.road_rl pool --out runs/rl_pool.pkl --count 4000 --workers 7
    python -m training.road_rl train --pool runs/rl_pool.pkl --init models/road_v2.5/candidate_b036.npz \
        --out runs/road_v2.5_rl --workers 7

为什么要 RL:DAgger 最多和老师一样好,老师过弯保守(横向加速度 0.45)。RL 按奖励自己找更快的开法。
风险:FastSim 的抓地力 0.8g 是假设值(vehicle.py 摩擦圆),RL 会钻它的空子 —— 结果在实车抓地力
测过之前只算实验。三道约束:
  奖励    每拍沿路前进的距离 - 时间代价;撞/出界扣 CRASH;按摩擦圆打滑程度扣 SLIP
  锚定    动作均值偏离初始(DAgger)模型的平方误差 × anchor(逐轮衰减),不许忘掉已经会的
  评估    和 DAgger 同一套:road_train.run_episode 在挑模型种子上跑确定性策略

赛道池:sample_episode 每局要让老师闭环开一遍确认可通(~5s),比学生开一局还慢,所以先生成一批存起来,
RL 反复用(每局车辆参数/噪声种子照样随机)。
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import json
import pickle
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from common.road_policy import DIM, FRAMES, SCHEMA_SPICA, History, spec_of, suffix_of
from simulation.fastsim.road import LEVELS
from simulation.fastsim.road_env import CONTROL_DT, RoadEnv
from training.road_check import sample_episode

SCHEMA = 'road_v2.5'             # 赛道池默认的速度契约(= spica_v3_s3_v0.45-3)
POOL_SEED0 = 20_000_000         # 和 DAgger 训练(10M)、挑模型(50M)、测试(90M)都不重叠
MIX = 'L5:0.2,L5s:0.2,L6h:0.15,L6f:0.15,L4:0.15,L6:0.1,L3:0.05'
TIME_COST = 0.02                # 每拍 m:相当于 0.4 m/s 以下"不划算"
CRASH = 50.0                    # 撞/出界(m 当量)。10 时 RL 拿 ~5% 撞车换 ~14% 提速(2026-09-30 第一轮),用户要稳定优先
SLIP = 0.5                      # 每拍 × 打滑程度(0~1)
MAX_STEPS = 3000


# ---------------- 赛道池 ----------------

def _pool_job(args):
    seed, level, cap_max, schema = args
    road, pl, start, _, _ = sample_episode(seed, level, enc=spec_of(schema), cap_max=cap_max)
    return dict(seed=seed, level=level, road=road, pl=pl, start=start)


def build_pool(a):
    rng = np.random.default_rng(0)
    mix = [(n, float(p)) for n, p in (x.split(':') for x in a.mix.split(','))]
    probs = np.array([p for _, p in mix]) / sum(p for _, p in mix)
    jobs = [(POOL_SEED0 + i, mix[int(rng.choice(len(mix), p=probs))][0], a.cap_max, a.schema)
            for i in range(a.count)]
    t0 = time.time()
    with ProcessPoolExecutor(a.workers) as ex:
        pool = list(ex.map(_pool_job, jobs, chunksize=4))
    Path(a.out).parent.mkdir(parents=True, exist_ok=True)
    with open(a.out, 'wb') as f:
        pickle.dump(pool, f)
    print(f'{len(pool)} 局 -> {a.out}({time.time() - t0:.0f}s)')


# ---------------- 网络(numpy 推理,torch 训练) ----------------

def mlp_np(w, b, x, out_tanh=False):
    for i in range(len(w) - 1):
        x = np.tanh(x @ w[i].T + b[i])
    x = x @ w[-1].T + b[-1]
    return np.tanh(x) if out_tanh else x


_POOL = None


def _init_worker(pool_path):
    global _POOL
    with open(pool_path, 'rb') as f:
        _POOL = pickle.load(f)


def rollout(args):
    """跑一局随机策略。返回逐拍 obs、动作、logp、奖励,以及结束原因。"""
    idx, noise_seed, actor, log_std, crash, time_cost = args
    ep = _POOL[idx]
    env = RoadEnv(ep['road'], ep['pl'], noise_seed, ep['start'], noise=True)
    rng = np.random.default_rng(noise_seed)
    hist = History()
    std = np.exp(log_std)
    xs, acts, logps, rews = [], [], [], []
    why, n = None, 0
    s_prev = env.road.s[env.idx]
    while why is None and n < MAX_STEPS:
        x = hist.push(env.observation())
        mu = mlp_np(actor[0], actor[1], x)
        a = mu + rng.normal(0, 1, 2) * std
        logp = float(-0.5 * np.sum(((a - mu) / std) ** 2) - np.sum(log_std) - np.log(2 * np.pi))
        why = env.step(np.clip(a, -1, 1))
        s_now = env.road.s[env.idx]
        r = (s_now - s_prev) - time_cost - SLIP * float(getattr(env.car, 'slip', 0.0))
        if why in ('collision', 'off_track'):
            r -= crash
        s_prev = s_now
        xs.append(x); acts.append(a.astype(np.float32)); logps.append(logp); rews.append(r)
        n += 1
    return (np.asarray(xs, np.float16), np.asarray(acts, np.float32), np.asarray(logps, np.float32),
            np.asarray(rews, np.float32), why or 'timeout', ep['level'], ep['pl'].cap, env.t)


def gae(rews, vals, gamma, lam, terminal):
    """vals 比 rews 多一个(末态价值,终止局为 0)。"""
    adv = np.zeros_like(rews)
    last = 0.0
    for t in range(len(rews) - 1, -1, -1):
        nv = 0.0 if (t == len(rews) - 1 and terminal) else vals[t + 1]
        delta = rews[t] + gamma * nv - vals[t]
        last = delta + gamma * lam * last
        adv[t] = last
    return adv, adv + vals[:-1]


def train(a):
    import torch
    from torch import nn

    from training.road_teacher import A_LAT
    from training.road_train import SELECT_SEED0, _job, summarize

    torch.set_num_threads(a.torch_threads)
    torch.manual_seed(a.seed)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'config.json').write_text(json.dumps({k: str(v) for k, v in vars(a).items()}, indent=1),
                                     encoding='utf-8')
    D = DIM * FRAMES

    def net(n_out):
        return nn.Sequential(nn.Linear(D, 256), nn.Tanh(), nn.Linear(256, 256), nn.Tanh(),
                             nn.Linear(256, n_out))
    actor, critic = net(2), net(1)
    with np.load(a.init) as z:
        enc = spec_of(str(z['schema']))
        if z['w0'].shape[1] != D:
            raise SystemExit(f'{a.init} 的输入层是 {z["w0"].shape[1]} 列,RL 只支持带 last_action 的 {D} 列')
        ref_w = [z[f'w{i}'] for i in range(3)]
        ref_b = [z[f'b{i}'] for i in range(3)]
    with torch.no_grad():
        for i, k in enumerate((0, 2, 4)):
            actor[k].weight.copy_(torch.from_numpy(ref_w[i]))
            actor[k].bias.copy_(torch.from_numpy(ref_b[i]))
    ref = net(2)
    ref.load_state_dict(actor.state_dict())
    for p_ in ref.parameters():
        p_.requires_grad_(False)
    log_std = torch.full((2,), float(np.log(a.std0)), requires_grad=True)
    opt_a = torch.optim.Adam(list(actor.parameters()) + [log_std], lr=a.lr)
    opt_c = torch.optim.Adam(critic.parameters(), lr=a.lr_critic)
    with open(a.pool, 'rb') as f:
        pool_encs = {ep['pl'].enc for ep in pickle.load(f)}
    if pool_encs != {enc}:
        raise SystemExit(f'赛道池的契约 {pool_encs} 和 {a.init} 的 {enc} 不一致')
    schema = SCHEMA_SPICA + suffix_of(enc)
    with open(a.pool, 'rb') as f:
        n_pool = len(pickle.load(f))
    rng = np.random.default_rng(a.seed)
    log = open(out / 'train.log', 'a', encoding='utf-8', buffering=1)

    def say(msg):
        line = f'{time.strftime("%H:%M:%S")} {msg}'
        print(line, flush=True)
        log.write(line + '\n')

    def weights(m):
        ls = [m[i] for i in (0, 2, 4)]
        return ([l.weight.detach().numpy().copy() for l in ls], [l.bias.detach().numpy().copy() for l in ls])

    with ProcessPoolExecutor(a.workers, initializer=_init_worker, initargs=(a.pool,)) as ex:
        for it in range(a.iters):
            t0 = time.time()
            w = weights(actor)
            jobs = [(int(rng.integers(n_pool)), int(rng.integers(2**31)), w, log_std.detach().numpy().copy(),
                     a.crash, a.time_cost) for _ in range(a.episodes)]
            eps = list(ex.map(rollout, jobs))
            t_roll = time.time() - t0

            X = torch.from_numpy(np.concatenate([e[0] for e in eps]).astype(np.float32))
            A = torch.from_numpy(np.concatenate([e[1] for e in eps]))
            LP = torch.from_numpy(np.concatenate([e[2] for e in eps]))
            with torch.no_grad():
                V = critic(X).squeeze(1).numpy()
            advs, rets, o = [], [], 0
            for e in eps:
                n = len(e[3])
                v = np.r_[V[o:o + n], 0.0]
                ad, rt = gae(e[3], v, a.gamma, a.lam, terminal=e[4] != 'timeout')
                advs.append(ad); rets.append(rt); o += n
            ADV = torch.from_numpy(np.concatenate(advs).astype(np.float32))
            RET = torch.from_numpy(np.concatenate(rets).astype(np.float32))
            ADV = (ADV - ADV.mean()) / (ADV.std() + 1e-8)
            anchor = max(a.anchor_min, a.anchor * (a.anchor_decay ** it))
            train_actor = it >= a.critic_warmup
            N = len(X)
            for _ in range(a.epochs):
                perm = torch.randperm(N)
                for s in range(0, N, a.minibatch):
                    j = perm[s:s + a.minibatch]
                    vloss = ((critic(X[j]).squeeze(1) - RET[j]) ** 2).mean()
                    opt_c.zero_grad(); vloss.backward(); opt_c.step()
                    if not train_actor:
                        continue
                    mu = actor(X[j])
                    ls = log_std.expand_as(mu)
                    logp = (-0.5 * ((A[j] - mu) / ls.exp()) ** 2 - ls - 0.5 * np.log(2 * np.pi)).sum(1)
                    ratio = (logp - LP[j]).exp()
                    pg = -torch.min(ratio * ADV[j], ratio.clamp(1 - a.clip, 1 + a.clip) * ADV[j]).mean()
                    with torch.no_grad():
                        mu_ref = ref(X[j])
                    aloss = pg + anchor * ((mu - mu_ref) ** 2).mean()
                    opt_a.zero_grad(); aloss.backward(); opt_a.step()
                    with torch.no_grad():
                        log_std.clamp_(np.log(0.02), np.log(0.5))
            t_upd = time.time() - t0 - t_roll

            reasons = {}
            for e in eps:
                reasons[e[4]] = reasons.get(e[4], 0) + 1
            ok = [e for e in eps if e[4] == 'goal']
            msg = (f'iter {it} steps={N} goal={len(ok) / len(eps):.3f} {reasons} '
                   f'ret={np.mean([e[3].sum() for e in eps]):.2f} '
                   f'ep_time={np.mean([e[7] for e in ok]) if ok else 0:.1f}s '
                   f'std={np.exp(log_std.detach().numpy()).round(3).tolist()} anchor={anchor:.3f} '
                   f'[{t_roll:.0f}s/{t_upd:.0f}s]')
            if (it + 1) % a.eval_every == 0 or it == a.iters - 1:
                ev_levels = a.eval_levels.split(',')
                ev = list(ex.map(_job, [(SELECT_SEED0 + i, ev_levels[i % len(ev_levels)], weights(actor),
                                         0.0, False, enc, A_LAT, a.cap_max) for i in range(a.eval_count)]))
                ev = [e for e in ev if e is not None]
                summ = summarize(ev, enc)
                by_level = {nm: summarize([e for e in ev if e['level'] == nm], enc)['success']
                            for nm in ev_levels}
                msg += f' | eval={summ["success"]:.3f} {by_level}'
                ww, bb = weights(actor)
                np.savez(out / f'candidate_{it:03d}.npz', schema=schema,
                         **{f'w{i}': ww[i] for i in range(3)}, **{f'b{i}': bb[i] for i in range(3)})
            say(msg)
    say('done')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('pool')
    p.add_argument('--out', required=True)
    p.add_argument('--count', type=int, default=4000)
    p.add_argument('--mix', default=MIX)
    p.add_argument('--cap-max', type=float, default=1.8)
    p.add_argument('--workers', type=int, default=7)
    p.add_argument('--schema', default=SCHEMA, help='赛道池按哪个速度契约抽车辆参数(要和 --init 的一致)')
    t = sub.add_parser('train')
    t.add_argument('--pool', required=True)
    t.add_argument('--init', required=True)
    t.add_argument('--out', required=True)
    t.add_argument('--iters', type=int, default=300)
    t.add_argument('--episodes', type=int, default=112, help='每轮采多少局')
    t.add_argument('--workers', type=int, default=7)
    t.add_argument('--torch-threads', type=int, default=2)
    t.add_argument('--seed', type=int, default=0)
    t.add_argument('--gamma', type=float, default=0.99)
    t.add_argument('--lam', type=float, default=0.95)
    t.add_argument('--clip', type=float, default=0.2)
    t.add_argument('--epochs', type=int, default=4)
    t.add_argument('--minibatch', type=int, default=4096)
    t.add_argument('--lr', type=float, default=1e-4)
    t.add_argument('--lr-critic', type=float, default=1e-3)
    t.add_argument('--std0', type=float, default=0.1, help='探索噪声初值(动作 ±1 刻度)')
    t.add_argument('--anchor', type=float, default=1.0, help='偏离初始模型的惩罚系数')
    t.add_argument('--anchor-decay', type=float, default=0.995)
    t.add_argument('--anchor-min', type=float, default=0.3, help='锚定系数下限,不许离初始模型太远')
    t.add_argument('--crash', type=float, default=CRASH, help='撞/出界扣分(m 当量)')
    t.add_argument('--time-cost', type=float, default=TIME_COST, help='每拍时间代价(m),越小越不急着提速')
    t.add_argument('--critic-warmup', type=int, default=5, help='前几轮只训价值网络')
    t.add_argument('--cap-max', type=float, default=1.8)
    t.add_argument('--eval-every', type=int, default=5)
    t.add_argument('--eval-levels', default='L5,L5s,L6h,L4c,L6f')
    t.add_argument('--eval-count', type=int, default=240)
    a = ap.parse_args()
    build_pool(a) if a.cmd == 'pool' else train(a)


if __name__ == '__main__':
    main()
