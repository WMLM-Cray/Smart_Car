"""开放场景探索的强化学习(PPO):从现有 BEV 模型(比如 v4g)出发,在随机楼层里学"多看到没见过的地面、别被截停"。

    python -m training.explore_rl --init models/v4_bev/001.npz --out runs/explore_rl --workers 30

环境和奖励见 training/explore_env:奖励 = 地面分割新标出的地面面积 m²,截停 / 撞上扣 CRASH。
做法照 training/road_rl:
  锚定   动作均值偏离初始模型的平方误差 × anchor(逐轮衰减到 anchor_min),不许忘掉沿路开
  价值网 同一套卷积结构,卷积层从初始模型抄过来,输出层清零
  评估   每 eval_every 轮在固定的 eval_count 个楼层上跑确定性策略,报新标面积、截停率,存 candidate_NNN.npz
图是 0/1 的,按位打包存(一拍 4.8 kB),一轮几万拍也放得下。
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

from training.road_rl import gae

TRAIN_SEED0 = 30_000_000        # 训练楼层种子;和别的训练 / 测试种子段都不重叠
EVAL_SEED0 = 70_000_000         # 评估楼层(固定)


def mean_raw(params: dict, img, sc) -> np.ndarray:
    """common.bev_policy.forward 不裁 ±1 的版本(PPO 的高斯均值;发给车之前再裁)。"""
    from common.bev_policy import CONVS, _conv_s2
    x = img.astype(np.float32)
    for k in range(len(CONVS)):
        x = np.maximum(_conv_s2(x, params[f'c{k}w'], params[f'c{k}b']), 0.0)
    h = np.concatenate([x.reshape(-1), sc.astype(np.float32)])
    for k in range(2):
        h = np.tanh(params[f'f{k}w'] @ h + params[f'f{k}b'])
    return (params['f2w'] @ h + params['f2b']).astype(np.float32)


def rollout(args):
    """一局随机策略。返回打包的图、标量、动作、logp、奖励、结束原因、新标面积、开过的距离。"""
    seed, noise_seed, params, log_std, enc, duration, level = args
    from training.explore_env import make
    env, _ = make(seed, enc, level)
    env.b.env.budget_s = env.b.env.t + duration
    rng = np.random.default_rng(noise_seed)
    std = np.exp(log_std)
    imgs, scs, acts, logps, rews = [], [], [], [], []
    why = None
    while why is None:
        img, sc = env.observation()
        mu = mean_raw(params, img, sc)
        a = mu + rng.normal(0, 1, 2) * std
        logp = float(-0.5 * np.sum(((a - mu) / std) ** 2) - np.sum(log_std) - np.log(2 * np.pi))
        r, why = env.step(np.clip(a, -1, 1))
        imgs.append(np.packbits(img.astype(bool).ravel()))
        scs.append(sc); acts.append(a.astype(np.float32)); logps.append(logp); rews.append(r)
    return (np.asarray(imgs), np.asarray(scs, np.float32), np.asarray(acts, np.float32),
            np.asarray(logps, np.float32), np.asarray(rews, np.float32), why, env.area - env.area0, env.b.env.dist)


def evaluate(args):
    """确定性策略跑一局:返回 (结束原因, 新标面积, 距离)。"""
    seed, params, enc, duration, level = args
    from training.explore_env import make
    env, _ = make(seed, enc, level)
    env.b.env.budget_s = env.b.env.t + duration
    why = None
    while why is None:
        _, why = env.step(np.clip(mean_raw(params, *env.observation()), -1, 1))
    return why, env.area - env.area0, env.b.env.dist


def train(a):
    import torch

    from common.bev_policy import H, W, channels_of
    from common.road_policy import spec_of
    from training.bev_net import load_into, make_net, weights_of

    torch.set_num_threads(a.torch_threads)
    torch.manual_seed(a.seed)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'config.json').write_text(json.dumps({k: str(v) for k, v in vars(a).items()}, indent=1), encoding='utf-8')
    with np.load(a.init) as z:
        schema = str(z['schema'])
    c_in = channels_of(schema)
    enc = spec_of('spica_v3' + schema[len('bev_v2'):])          # 同 BevPolicy
    actor, critic, ref = make_net(c_in), make_net(c_in), make_net(c_in)
    for net in (actor, critic, ref):
        load_into(net, a.init)
    with torch.no_grad():
        critic.f2.weight.zero_(); critic.f2.bias.zero_()
    for p_ in ref.parameters():
        p_.requires_grad_(False)
    log_std = torch.full((2,), float(np.log(a.std0)), requires_grad=True)
    opt_a = torch.optim.Adam(list(actor.parameters()) + [log_std], lr=a.lr)
    opt_c = torch.optim.Adam(critic.parameters(), lr=a.lr_critic)
    bits = c_in * H * W
    rng = np.random.default_rng(a.seed)
    log = open(out / 'train.log', 'a', encoding='utf-8', buffering=1)

    def say(msg):
        line = f'{time.strftime("%H:%M:%S")} {msg}'
        print(line, flush=True)
        log.write(line + '\n')

    def params_of(net):
        return {k: v.astype(np.float32) for k, v in weights_of(net).items()}

    def unpack(P, j):
        return torch.from_numpy(np.unpackbits(P[j], axis=1)[:, :bits].reshape(len(j), c_in, H, W).astype(np.float32))

    def value(P, S):
        out_ = []
        with torch.no_grad():
            for s in range(0, len(P), 1024):
                j = np.arange(s, min(len(P), s + 1024))
                out_.append(critic(unpack(P, j), S[j])[:, 0].numpy())
        return np.concatenate(out_)

    mix = [(n, float(p_)) for n, p_ in (x.split(':') for x in a.level.split(','))]
    names, probs = [n for n, _ in mix], np.array([p_ for _, p_ in mix]) / sum(p_ for _, p_ in mix)

    def run_eval(ex, it):
        """每个场地各 eval_count 局(同一批种子)。"""
        w = params_of(actor)
        np.savez(out / f'candidate_{it:03d}.npz', schema=np.array(schema), **weights_of(actor))
        msg = []
        for nm in names:
            ev = list(ex.map(evaluate, [(EVAL_SEED0 + i, w, enc, a.duration, nm) for i in range(a.eval_count)]))
            msg.append(f'{nm}: 面积 {np.mean([e[1] for e in ev]):.1f} m² 截停/撞 '
                       f'{np.mean([e[0] in ("guard", "collision") for e in ev]):.0%}')
        return 'eval ' + ' / '.join(msg)

    with ProcessPoolExecutor(a.workers) as ex:
        say(f'初始 {a.init} ' + run_eval(ex, -1))
        for it in range(a.iters):
            t0 = time.time()
            w = params_of(actor)
            jobs = [(TRAIN_SEED0 + int(rng.integers(10**7)), int(rng.integers(2**31)), w,
                     log_std.detach().numpy().copy(), enc, a.duration, names[rng.choice(len(names), p=probs)]) for _ in range(a.episodes)]
            eps = list(ex.map(rollout, jobs))
            t_roll = time.time() - t0
            P = np.concatenate([e[0] for e in eps])
            S = torch.from_numpy(np.concatenate([e[1] for e in eps]))
            A = torch.from_numpy(np.concatenate([e[2] for e in eps]))
            LP = torch.from_numpy(np.concatenate([e[3] for e in eps]))
            V = value(P, S)
            advs, rets, o = [], [], 0
            for e in eps:
                n = len(e[4])
                ad, rt = gae(e[4], np.r_[V[o:o + n], 0.0], a.gamma, a.lam, terminal=e[5] != 'timeout')
                advs.append(ad); rets.append(rt); o += n
            ADV = torch.from_numpy(np.concatenate(advs).astype(np.float32))
            RET = torch.from_numpy(np.concatenate(rets).astype(np.float32))
            ADV = (ADV - ADV.mean()) / (ADV.std() + 1e-8)
            anchor = max(a.anchor_min, a.anchor * (a.anchor_decay ** it))
            N = len(P)
            for _ in range(a.epochs):
                perm = np.random.default_rng(int(rng.integers(2**31))).permutation(N)
                for s in range(0, N, a.minibatch):
                    j = perm[s:s + a.minibatch]
                    X, Sj = unpack(P, j), S[j]
                    vloss = ((critic(X, Sj)[:, 0] - RET[j]) ** 2).mean()
                    opt_c.zero_grad(); vloss.backward(); opt_c.step()
                    if it < a.critic_warmup:
                        continue
                    mu = actor(X, Sj)
                    ls = log_std.expand_as(mu)
                    logp = (-0.5 * ((A[j] - mu) / ls.exp()) ** 2 - ls - 0.5 * np.log(2 * np.pi)).sum(1)
                    ratio = (logp - LP[j]).exp()
                    pg = -torch.min(ratio * ADV[j], ratio.clamp(1 - a.clip, 1 + a.clip) * ADV[j]).mean()
                    with torch.no_grad():
                        mu_ref = ref(X, Sj)
                    aloss = pg + anchor * ((mu - mu_ref) ** 2).mean()
                    opt_a.zero_grad(); aloss.backward(); opt_a.step()
                    with torch.no_grad():
                        log_std.clamp_(np.log(0.02), np.log(0.5))
            t_upd = time.time() - t0 - t_roll
            reasons = {}
            for e in eps:
                reasons[e[5]] = reasons.get(e[5], 0) + 1
            msg = (f'iter {it} steps={N} {reasons} ret={np.mean([e[4].sum() for e in eps]):.1f} '
                   f'面积={np.mean([e[6] for e in eps]):.1f} 距离={np.mean([e[7] for e in eps]):.1f} '
                   f'std={np.exp(log_std.detach().numpy()).round(3).tolist()} anchor={anchor:.3f} '
                   f'[{t_roll:.0f}s/{t_upd:.0f}s]')
            if (it + 1) % a.eval_every == 0 or it == a.iters - 1:
                msg += ' | ' + run_eval(ex, it)
            say(msg)
    say('done')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--init', required=True, help='起点模型(bev_v2 schema)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--iters', type=int, default=200)
    ap.add_argument('--episodes', type=int, default=64, help='每轮局数')
    ap.add_argument('--duration', type=float, default=60.0, help='每局最长 s')
    ap.add_argument('--level', default='room:0.3,room_f:0.7',
                    help='场地混合 名字:比例(training/explore_env.levels:room 空房间、room_f 有家具的房间、building 整栋楼)')
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--torch-threads', type=int, default=8)
    ap.add_argument('--lr', type=float, default=1e-4)
    ap.add_argument('--lr-critic', type=float, default=3e-4)
    ap.add_argument('--std0', type=float, default=0.15)
    ap.add_argument('--gamma', type=float, default=0.99)
    ap.add_argument('--lam', type=float, default=0.95)
    ap.add_argument('--clip', type=float, default=0.2)
    ap.add_argument('--epochs', type=int, default=3)
    ap.add_argument('--minibatch', type=int, default=512)
    ap.add_argument('--anchor', type=float, default=1.0)
    ap.add_argument('--anchor-decay', type=float, default=0.97)
    ap.add_argument('--anchor-min', type=float, default=0.05)
    ap.add_argument('--critic-warmup', type=int, default=2, help='前几轮只训价值网')
    ap.add_argument('--eval-every', type=int, default=5)
    ap.add_argument('--eval-count', type=int, default=32)
    ap.add_argument('--seed', type=int, default=0)
    train(ap.parse_args(argv))


if __name__ == '__main__':
    main()
