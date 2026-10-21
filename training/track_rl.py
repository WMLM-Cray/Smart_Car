"""第 2 段:沿赛道开的强化学习(PPO),从 bev_v2.5 模型(c027)出发。方案见 docs/强化学习方案.md。

    python -m training.track_rl --init runs/Balpha_full_c027.npz --out runs/track_rl --workers 7 --iters 5

和 explore_rl 的区别(都是按《强化学习方案》§1 的教训改的):
  任务   只有沿赛道开:关卡全是桶 / 路沿,没有往返和房间
  奖励   每拍沿中心线前进的弧长;失败(出界 / 截停 / 撞 / 停滞)一次性扣 C0 + C1·v²,四种同价
  停滞   3 s 内沿路前进不到 0.3 m(不看瞬时车速)
  开局   一半的局老师先开到难弯前 4 m 再交给策略(BEV 历史和车速都是真的),样本集中在难弯
  网络   bev_v2.5 双图;冻结两个卷积主干,只训全连接头;价值网吃特权量(非对称)
  噪声   转向 / 速度分开设,速度小
  限速   一半 0.45–2.5(和 c027 训练一样,保住速度泛化),一半 0.7–1.2(车上常用)
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import json
import math
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

from training.road_rl import gae

TRAIN_SEED0 = 45_000_000        # 训练局;和 bev_train(30M / 60M)、explore(30M / 70M)、测试(90M+)都不重叠
EVAL_SEED0 = 75_000_000         # 训练中的固定评估局
MIX = 'L5g:0.25,L5:0.10,L5s:0.10,L6h:0.10,L6f:0.10,L4:0.05,LW3:0.10,LW4:0.10,LWO3:0.05,LWT3c:0.05'
C0, C1 = 10.0, 5.0              # 失败扣分 C0 + C1·v²(m 进度为单位)
STALL_S, STALL_M = 3.0, 0.3     # 这么久沿路前进不到这么多 = 停滞
DS_CLIP = (-0.1, 0.2)           # 单拍弧长增量裁剪(防投影跳变)
HARD_TURN = math.radians(45.0)  # 中心线 1.5 m 内转过这么多算难弯
MERGE_M = 3.0                   # 两段转弯隔不到这么远算同一个弯
HAND_BEFORE = 4.0               # 老师开到难弯前这么多 m 交给策略
EPISODE_S = 40.0
N_PRIV = 6
A_LAT = 1.47                    # 同 c027 训练(--a-lat)
CAR = dict(guard=True, yolo=False, seg_lut=True, seg_fill=True, seg_period=0.33, seg_lag=0.35)
FAIL = ('off_track', 'guard', 'collision', 'stall')


def hard_corners(road, s_min, s_max):
    """难弯的起点弧长:中心线 1.5 m 内转过 HARD_TURN 的位置;前面 MERGE_M 内已经在转的算同一个弯,只取开头。限在 [s_min, s_max]。"""
    h = np.unwrap(road.heading)
    ds = float(np.median(np.diff(road.s)))
    k = max(1, int(round(1.5 / ds)))
    on = np.abs(h[k:] - h[:-k]) > HARD_TURN
    idx = np.where(on)[0]
    starts = [i for n, i in enumerate(idx) if n == 0 or road.s[i] - road.s[idx[n - 1]] > MERGE_M]
    return [float(road.s[i]) for i in starts if s_min <= road.s[i] <= s_max]


class TrackEnv:
    """一局:抽路 + 车辆参数(同 bev_train),可选老师先开到难弯前。step 返回 (奖励, 结束原因或 None)。"""

    def __init__(self, seed, level, enc, cap, hard_start, lag_max=3):
        from training.bev_env import BevEnv, SeenTeacher
        from training.road_check import sample_episode
        road, pl, start, P, _ = sample_episode(seed, level, enc=enc, a_lat=A_LAT, cap_max=cap[1], cap_min=cap[0])
        pl = replace(pl, lag=int(np.random.default_rng(seed + 17).integers(0, lag_max + 1)))
        self.b = BevEnv(road, pl, seed, start, noise=True, free=True, near=True, **CAR)
        self.road, self.env = road, self.b.env
        self.why = None
        self.handed = 0.0
        if hard_start:
            s0 = float(road.s[self.env.idx])
            cs = hard_corners(road, s0 + HAND_BEFORE + 2.0, s0 + 30.0)
            if cs:
                target = cs[np.random.default_rng(seed + 29).integers(len(cs))] - HAND_BEFORE
                teacher = SeenTeacher(self.b, P, A_LAT)
                while self.why is None and road.s[self.env.idx] < target:
                    self.why = self.b.step(teacher())       # 建图在 step 里;不裁图,省时间
                self.handed = float(road.s[self.env.idx] - s0)
        self.env.budget_s = self.env.t + EPISODE_S
        self.s_prev = float(road.s[self.env.idx])
        self.hist = [(self.env.t, self.s_prev)]

    def observation(self):
        return self.b.observation()

    def privileged(self):
        """价值网用的仿真内信息(不上车):限速、车速、横向偏移 / 半宽、航向误差、前方 3 m 最大曲率、剩余时间。"""
        env, road = self.env, self.road
        i, d = road.project((env.car.x, env.car.y), env.idx)
        dh = (env.car.yaw - road.heading[i] + math.pi) % (2 * math.pi) - math.pi
        j = min(len(road.s) - 1, i + int(3.0 / max(1e-6, float(road.s[1] - road.s[0]))))
        h = np.unwrap(road.heading[i:j + 1])
        kmax = float(np.max(np.abs(np.diff(h)) / np.maximum(np.diff(road.s[i:j + 1]), 1e-6))) if j > i else 0.0
        return np.array([env.pl.cap / 2.5, env.car.v / 2.5, d / max(float(road.half[i]), 0.1), dh,
                         min(kmax, 2.0) / 2.0, max(0.0, env.budget_s - env.t) / EPISODE_S], np.float32)

    def step(self, action):
        v = float(self.env.car.v)
        why = self.b.step(action)
        s = float(self.road.s[self.env.idx])
        r = float(np.clip(s - self.s_prev, *DS_CLIP))
        self.s_prev = s
        t = self.env.t
        self.hist.append((t, s))
        while self.hist and self.hist[0][0] < t - STALL_S:
            self.hist.pop(0)
        if why is None and t - self.hist[0][0] >= STALL_S - 0.06 and s - self.hist[0][1] < STALL_M:
            why = 'stall'
        if why in FAIL:
            r -= C0 + C1 * v * v
        return r, why


def mean_raw(p: dict, img, sc, near) -> np.ndarray:
    """common.bev_policy.forward_v25 不裁 ±1 的版本(PPO 的高斯均值;发给车之前再裁)。"""
    from common.bev_policy import CONVS, NEAR_CONVS, _conv_s2
    x, nx = img.astype(np.float32), near.astype(np.float32)
    for k in range(len(CONVS)):
        x = np.maximum(_conv_s2(x, p[f'c{k}w'], p[f'c{k}b']), 0.0)
    for k in range(len(NEAR_CONVS)):
        nx = np.maximum(_conv_s2(nx, p[f'n{k}w'], p[f'n{k}b']), 0.0)
    h = np.concatenate([x.reshape(-1), nx.reshape(-1), sc.astype(np.float32)])
    for k in range(2):
        h = np.tanh(p[f'f{k}w'] @ h + p[f'f{k}b'])
    return (p['f2w'] @ h + p['f2b']).astype(np.float32)


def pack(x):
    return np.packbits(x.astype(bool).ravel())


def rollout(args):
    """一局随机策略。返回打包的主图 / 近场图、标量、特权量、动作、logp、奖励、结束原因、局信息,
    以及到点时最后一拍动作之后的观测(续算价值用;真结束为 None)。老师先开那段已经不在里面。"""
    seed, noise_seed, params, log_std, enc, level, cap, hard = args
    try:
        env = TrackEnv(seed, level, enc, cap, hard)
    except RuntimeError:
        return None
    rng = np.random.default_rng(noise_seed)
    std = np.exp(log_std)
    imgs, nears, scs, privs, acts, logps, rews = [], [], [], [], [], [], []
    why = env.why            # 老师先开时已经出事(极少),这局作废
    if why is not None:
        return None
    while why is None:
        img, sc, near = env.observation()
        privs.append(env.privileged())
        mu = mean_raw(params, img, sc, near)
        a = mu + rng.normal(0, 1, 2) * std
        logps.append(float(-0.5 * np.sum(((a - mu) / std) ** 2) - np.sum(log_std) - np.log(2 * np.pi)))
        r, why = env.step(np.clip(a, -1, 1))
        imgs.append(pack(img)); nears.append(pack(near)); scs.append(sc)
        acts.append(a.astype(np.float32)); rews.append(r)
    last = None
    if why == 'timeout':
        img, sc, near = env.observation()
        last = (pack(img), pack(near), np.asarray(sc, np.float32), env.privileged())
    info = dict(level=level, cap=float(env.env.pl.cap), why=why, hard=bool(env.handed), dist=float(env.s_prev),
                time=float(env.env.t))
    return (np.asarray(imgs), np.asarray(nears), np.asarray(scs, np.float32), np.asarray(privs, np.float32),
            np.asarray(acts, np.float32), np.asarray(logps, np.float32), np.asarray(rews, np.float32), why, info, last)


def evaluate(args):
    """确定性策略跑一局(从起点开,不让老师先开),返回 (结束原因, 一局奖励)。"""
    seed, params, enc, level, cap = args
    try:
        env = TrackEnv(seed, level, enc, cap, False)
    except RuntimeError:
        return None
    why, ret = None, 0.0
    while why is None:
        img, sc, near = env.observation()
        r, why = env.step(np.clip(mean_raw(params, img, sc, near), -1, 1))
        ret += r
    return why, ret


def eval_real(args):
    """实车 A 赛道布局(maps/realA_1020.yaml)按车上条件一局,返回结束原因。"""
    from evaluation.bev_track import run
    path, seed, cap = args
    return run((path, 'realA_1020', seed, cap, True, True, 0.0, 0.0))['reason']


def parse_mix(s):
    mix = [(x.split(':')[0], float(x.split(':')[1])) for x in s.split(',')]
    p = np.array([v for _, v in mix])
    return [n for n, _ in mix], p / p.sum()


def train(a):
    import torch

    from common.bev_policy import C2, H, NEAR_SIZE, W, schema_prefix, SCHEMA_V25
    from common.road_policy import spec_of
    from training.bev_net import load_into, make_net, weights_of
    from training.explore_rl import grad_norm

    torch.set_num_threads(a.torch_threads)
    torch.manual_seed(a.seed)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    ckpt = out / 'ckpt.pt'
    if ckpt.exists() != a.resume:
        raise SystemExit(f'{ckpt} ' + ('已存在:接着训加 --resume,重新训换个 --out' if ckpt.exists() else '不存在,没法 --resume'))
    (out / ('config.json' if not a.resume else f'config_resume_{time.strftime("%m%d_%H%M%S")}.json')).write_text(
        json.dumps({k: str(v) for k, v in vars(a).items()}, indent=1), encoding='utf-8')
    with np.load(a.init) as z:
        schema = str(z['schema'])
    if schema_prefix(schema) != SCHEMA_V25:
        raise SystemExit(f'起点要是 bev_v2.5 模型,给的是 {schema}')
    enc = spec_of('spica_v3' + schema[len(SCHEMA_V25):])        # 同 BevPolicy
    actor, critic, ref = (make_net(C2, near=True) for _ in range(3))
    for net in (actor, critic, ref):
        load_into(net, a.init)
    with torch.no_grad():
        critic.f2.weight.zero_(); critic.f2.bias.zero_()
        old = critic.f0                                         # 价值网多吃 N_PRIV 个特权量,新列从 0 开始
        critic.f0 = torch.nn.Linear(old.in_features + N_PRIV, old.out_features)
        critic.f0.weight.zero_(); critic.f0.weight[:, :old.in_features].copy_(old.weight); critic.f0.bias.copy_(old.bias)
    frozen = [p_ for net in (actor, critic) for m in (net.convs, net.near_convs) for p_ in m.parameters()]
    for p_ in frozen + list(ref.parameters()):                 # 冻结两个卷积主干(方案 §4),参考网络整体不动
        p_.requires_grad_(False)
    dev = torch.device(a.device)
    for net in (actor, critic, ref):
        net.to(dev, memory_format=torch.channels_last)
    log_std = torch.tensor([math.log(a.std_steer), math.log(a.std_speed)], device=dev, requires_grad=True)
    std_lo = torch.tensor([math.log(0.03), math.log(0.02)], device=dev)
    opt_a = torch.optim.Adam([p_ for p_ in actor.parameters() if p_.requires_grad] + [log_std], lr=a.lr)
    opt_c = torch.optim.Adam([p_ for p_ in critic.parameters() if p_.requires_grad], lr=a.lr_critic)
    bits, nbits = C2 * H * W, C2 * NEAR_SIZE * NEAR_SIZE
    rng = np.random.default_rng(a.seed)
    start = 0
    if a.resume:
        z = torch.load(ckpt, map_location=dev, weights_only=False)
        for k, net in (('actor', actor), ('critic', critic), ('ref', ref)):
            net.load_state_dict(z[k])
        with torch.no_grad():
            log_std.copy_(z['log_std'])
        opt_a.load_state_dict(z['opt_a']); opt_c.load_state_dict(z['opt_c'])
        rng.bit_generator.state = z['rng']
        torch.set_rng_state(z['torch_rng'])
        start = z['iter'] + 1

    def save(it):
        tmp = out / 'ckpt.pt.tmp'
        torch.save({'iter': it, 'actor': actor.state_dict(), 'critic': critic.state_dict(), 'ref': ref.state_dict(),
                    'log_std': log_std.detach().clone(), 'opt_a': opt_a.state_dict(), 'opt_c': opt_c.state_dict(),
                    'rng': rng.bit_generator.state, 'torch_rng': torch.get_rng_state(), 'args': vars(a)}, tmp)
        os.replace(tmp, ckpt)

    log = open(out / 'train.log', 'a', encoding='utf-8', buffering=1)
    diag = open(out / 'diag.jsonl', 'a', encoding='utf-8', buffering=1)

    def say(msg):
        line = f'{time.strftime("%H:%M:%S")} {msg}'
        print(line, flush=True)
        log.write(line + '\n')

    def params_of(net):
        return {k: v.astype(np.float32) for k, v in weights_of(net).items()}

    def unpack(P, j, n, size):
        x = np.unpackbits(P[j], axis=1)[:, :n].reshape(len(j), C2, size, size).astype(np.float32)
        return torch.from_numpy(x).to(dev).contiguous(memory_format=torch.channels_last)

    def infer(net, P, PN, S):
        res = []
        with torch.no_grad():
            for s in range(0, len(P), 512):
                j = np.arange(s, min(len(P), s + 512))
                res.append(net(unpack(P, j, bits, H), S[j], unpack(PN, j, nbits, NEAR_SIZE)))
        return torch.cat(res)

    names, probs = parse_mix(a.mix)

    def cap_of():
        return (0.45, 2.5) if rng.random() < 0.5 else (0.7, 1.2)

    def run_eval(ex, it):
        w = params_of(actor)
        path = out / f'candidate_{it:03d}.npz'
        np.savez(path, schema=np.array(schema), **{k: v for k, v in w.items()})
        msg = []
        for nm in names:
            ev = [e for e in ex.map(evaluate, [(EVAL_SEED0 + i, w, enc, nm, (1.2, 1.2)) for i in range(a.eval_count)])
                  if e is not None]
            why = [e[0] for e in ev]
            msg.append(f'{nm} {why.count("goal") / max(1, len(why)):.2f}')
        real = list(ex.map(eval_real, [(str(path), EVAL_SEED0 + i, c) for c in (0.7, 1.2) for i in range(a.eval_count)]))
        n = a.eval_count
        msg.append(f'realA 0.7 {real[:n].count("goal")}/{n} 1.2 {real[n:].count("goal")}/{n}')
        return 'eval ' + ' '.join(msg)

    with ProcessPoolExecutor(a.workers) as ex:
        if a.resume:
            say(f'从 {ckpt} 第 {start} 轮接着训')
        else:
            say(f'初始 {a.init} ' + run_eval(ex, -1))
        for it in range(start, a.iters):
            t0 = time.time()
            w = params_of(actor)
            ls_np = log_std.detach().cpu().numpy().copy()
            jobs = [(TRAIN_SEED0 + int(rng.integers(10**7)), int(rng.integers(2**31)), w, ls_np, enc,
                     names[rng.choice(len(names), p=probs)], cap_of(), bool(rng.random() < a.hard_frac))
                    for _ in range(a.episodes)]
            eps = [e for e in ex.map(rollout, jobs) if e is not None]
            t_roll = time.time() - t0
            P = np.concatenate([e[0] for e in eps]); PN = np.concatenate([e[1] for e in eps])
            S = torch.from_numpy(np.concatenate([e[2] for e in eps])).to(dev)
            SC = torch.cat([S, torch.from_numpy(np.concatenate([e[3] for e in eps])).to(dev)], 1)
            A = torch.from_numpy(np.concatenate([e[4] for e in eps])).to(dev)
            LP = torch.from_numpy(np.concatenate([e[5] for e in eps])).to(dev)
            V = infer(critic, P, PN, SC)[:, 0].cpu().numpy()
            MU_REF = infer(ref, P, PN, S)
            cut = [i for i, e in enumerate(eps) if e[9] is not None]
            V_T = {}
            if cut:
                PT = np.stack([eps[i][9][0] for i in cut]); PNT = np.stack([eps[i][9][1] for i in cut])
                ST = torch.from_numpy(np.stack([np.r_[eps[i][9][2], eps[i][9][3]] for i in cut])).to(dev)
                V_T = dict(zip(cut, infer(critic, PT, PNT, ST)[:, 0].cpu().numpy()))
            advs, rets, o = [], [], 0
            for i, e in enumerate(eps):
                n = len(e[6])
                ad, rt = gae(e[6], np.r_[V[o:o + n], V_T.get(i, 0.0)], a.gamma, a.lam, terminal=e[7] != 'timeout')
                advs.append(ad); rets.append(rt); o += n
            ret_np = np.concatenate(rets)
            ADV = torch.from_numpy(np.concatenate(advs).astype(np.float32)).to(dev)
            RET = torch.from_numpy(ret_np.astype(np.float32)).to(dev)
            ADV = (ADV - ADV.mean()) / (ADV.std() + 1e-8)
            anchor = max(a.anchor_min, a.anchor * (a.anchor_decay ** it))
            N = len(P)
            st = {k: [] for k in ('kl', 'clip', 'gn_a', 'gn_c', 'vloss', 'pg', 'anchor_loss')}
            for _ in range(a.epochs):
                perm = np.random.default_rng(int(rng.integers(2**31))).permutation(N)
                for s in range(0, N, a.minibatch):
                    jn = perm[s:s + a.minibatch]
                    X, XN = unpack(P, jn, bits, H), unpack(PN, jn, nbits, NEAR_SIZE)
                    j = torch.as_tensor(jn, device=dev)
                    vloss = ((critic(X, SC[j], XN)[:, 0] - RET[j]) ** 2).mean()
                    opt_c.zero_grad(); vloss.backward()
                    st['gn_c'].append(grad_norm([p_ for p_ in critic.parameters() if p_.requires_grad]))
                    st['vloss'].append(vloss.item())
                    opt_c.step()
                    if it < a.critic_warmup:
                        continue
                    mu = actor(X, S[j], XN)
                    ls = log_std.expand_as(mu)
                    logp = (-0.5 * ((A[j] - mu) / ls.exp()) ** 2 - ls - 0.5 * np.log(2 * np.pi)).sum(1)
                    ratio = (logp - LP[j]).exp()
                    pg = -torch.min(ratio * ADV[j], ratio.clamp(1 - a.clip, 1 + a.clip) * ADV[j]).mean()
                    an = ((mu - MU_REF[j]) ** 2).mean()
                    opt_a.zero_grad(); (pg + anchor * an).backward()
                    st['gn_a'].append(grad_norm([p_ for p_ in actor.parameters() if p_.requires_grad] + [log_std]))
                    opt_a.step()
                    with torch.no_grad():
                        lr_ = logp - LP[j]
                        st['kl'].append(float(((lr_.exp() - 1) - lr_).mean()))
                        st['clip'].append(float(((lr_.exp() - 1).abs() > a.clip).float().mean()))
                        log_std.copy_(torch.maximum(log_std, std_lo).clamp(max=math.log(0.5)))
                    st['pg'].append(pg.item()); st['anchor_loss'].append(an.item())
            t_upd = time.time() - t0 - t_roll
            mean = lambda x: float(np.mean(x)) if len(x) else float('nan')
            var = float(np.var(ret_np))
            reasons, per = {}, {}
            for e in eps:
                reasons[e[7]] = reasons.get(e[7], 0) + 1
                p_ = per.setdefault(e[8]['level'], {'n': 0, 'goal': 0, 'fail': 0, 'ret': []})
                p_['n'] += 1; p_['goal'] += e[7] == 'goal'; p_['fail'] += e[7] in FAIL
                p_['ret'].append(float(e[6].sum()))
            for p_ in per.values():
                p_['ret'] = round(mean(p_['ret']), 2)
            hard = [e for e in eps if e[8]['hard']]
            d = {'iter': it, **{k: mean(v) for k, v in st.items()},
                 'ev': 1.0 - float(np.var(ret_np - V)) / var if var > 0 else float('nan'),
                 'reasons': reasons, 'levels': per, 'hard_eps': len(hard),
                 'hard_fail': mean([e[7] in FAIL for e in hard]),
                 'std': np.exp(log_std.detach().cpu().numpy()).round(4).tolist(), 'anchor': anchor,
                 'cap_mean': mean([e[8]['cap'] for e in eps]),
                 'caps': {f'<{hi}': {r: sum(e[7] == r for e in g) for r in ('goal',) + FAIL} | {'n': len(g)}
                          for lo, hi in ((0, 0.7), (0.7, 1.2), (1.2, 1.8), (1.8, 9))
                          for g in [[e for e in eps if lo <= e[8]['cap'] < hi]]},
                 'steer_sat': float(np.mean(np.abs(A[:, 0].cpu().numpy()) >= 1.0))}
            diag.write(json.dumps(d, ensure_ascii=False) + '\n')
            msg = (f'iter {it} eps={len(eps)} steps={N} {reasons} ret={mean([e[6].sum() for e in eps]):.1f} '
                   f'难弯失败={d["hard_fail"]:.2f}({len(hard)}) std={d["std"]} anchor={anchor:.2f} '
                   f'kl={d["kl"]:.4f} clip={d["clip"]:.2f} ev={d["ev"]:.2f} [{t_roll:.0f}s/{t_upd:.0f}s]')
            if (it + 1) % a.eval_every == 0 or it == a.iters - 1:
                msg += ' | ' + run_eval(ex, it)
            say(msg)
            save(it)
    say('done')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--init', required=True, help='起点模型(bev_v2.5)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--iters', type=int, default=40)
    ap.add_argument('--episodes', type=int, default=64, help='每轮局数')
    ap.add_argument('--mix', default=MIX)
    ap.add_argument('--hard-frac', type=float, default=0.5, help='老师先开到难弯前再交给策略的局的比例')
    ap.add_argument('--workers', type=int, default=7)
    ap.add_argument('--torch-threads', type=int, default=4)
    ap.add_argument('--device', default='cpu')
    ap.add_argument('--lr', type=float, default=3e-5)
    ap.add_argument('--lr-critic', type=float, default=3e-4)
    ap.add_argument('--std-steer', type=float, default=0.10)
    ap.add_argument('--std-speed', type=float, default=0.05)
    ap.add_argument('--gamma', type=float, default=0.995)
    ap.add_argument('--lam', type=float, default=0.95)
    ap.add_argument('--clip', type=float, default=0.15)
    ap.add_argument('--epochs', type=int, default=2)
    ap.add_argument('--minibatch', type=int, default=512)
    ap.add_argument('--anchor', type=float, default=1.0)
    ap.add_argument('--anchor-decay', type=float, default=0.97)
    ap.add_argument('--anchor-min', type=float, default=0.6)
    ap.add_argument('--critic-warmup', type=int, default=3)
    ap.add_argument('--eval-every', type=int, default=2)
    ap.add_argument('--eval-count', type=int, default=16)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--resume', action='store_true')
    train(ap.parse_args())


if __name__ == '__main__':
    main()
