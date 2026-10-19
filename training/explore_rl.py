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

from training.explore_env import room_level
from training.road_rl import gae

N_PRIV = 4                      # Explore.privileged 的长度
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
    """一局随机策略。返回打包的图、标量、动作、logp、奖励、结束原因、新标面积、开过的距离、特权信息,
    以及最后一拍动作之后的观测(打包图、标量、特权信息;只有到点结束才给,否则 None),到点续算用它的价值。"""
    seed, noise_seed, params, log_std, enc, duration, level = args
    from training.explore_env import make
    env, _ = make(seed, enc, level)
    env.b.env.budget_s = env.b.env.t + duration
    rng = np.random.default_rng(noise_seed)
    std = np.exp(log_std)
    imgs, scs, acts, logps, rews, privs = [], [], [], [], [], []
    why = None
    while why is None:
        img, sc = env.observation()
        privs.append(env.privileged())
        mu = mean_raw(params, img, sc)
        a = mu + rng.normal(0, 1, 2) * std
        logp = float(-0.5 * np.sum(((a - mu) / std) ** 2) - np.sum(log_std) - np.log(2 * np.pi))
        r, why = env.step(np.clip(a, -1, 1))
        imgs.append(np.packbits(img.astype(bool).ravel()))
        scs.append(sc); acts.append(a.astype(np.float32)); logps.append(logp); rews.append(r)
    last = None
    if why == 'timeout':
        img, sc = env.observation()
        last = (np.packbits(img.astype(bool).ravel()), np.asarray(sc, np.float32), np.asarray(env.privileged(), np.float32))
    return (np.asarray(imgs), np.asarray(scs, np.float32), np.asarray(acts, np.float32),
            np.asarray(logps, np.float32), np.asarray(rews, np.float32), why, env.area - env.area0, env.dist,
            np.asarray(privs, np.float32), last)


def evaluate(args):
    """确定性策略跑一局:返回 (结束原因, 新标面积, 距离)。"""
    seed, params, enc, duration, level = args
    from training.explore_env import make
    env, _ = make(seed, enc, level)
    env.b.env.budget_s = env.b.env.t + duration
    why = None
    while why is None:
        _, why = env.step(np.clip(mean_raw(params, *env.observation()), -1, 1))
    return why, env.area - env.area0, env.dist


def speed_head_affine(old, new) -> tuple[float, float]:
    """老的线性速度契约 old 换到带死区的 new:新动作 = k·老动作 + c,在 old 能表达、new 的前进段装得下的车速上
    解出来的目标车速逐位相同(老动作 -1 -> 新前进段起点 new.dead,也就是 v_lo;超过 new.v_hi 的部分裁掉)。
    网络最后一层是线性的,速度那一行乘 k 加 c 就是精确换算,从老模型热启动不用重训速度头。"""
    assert not old.dead and new.dead and not new.log      # 对数刻度不是仿射,换不过去
    span = (new.v_hi - new.v_lo) / (1.0 - new.dead)
    k = (old.v_hi - old.v_lo) / 2.0 / span
    return k, new.dead + (old.v_lo - new.v_lo + (old.v_hi - old.v_lo) / 2.0) / span


def grad_norm(params) -> float:
    """backward 之后、step 之前所有梯度的总 L2 范数。"""
    return float(sum(float((p.grad.detach() ** 2).sum()) for p in params if p.grad is not None) ** 0.5)


def diagnose(it, eps, levels_, V, ret, acts, enc, st) -> dict:
    """一轮的诊断(写 diag.jsonl):PPO 更新幅度(KL、clip fraction、梯度范数、各项损失;critic 预热轮没有策略项,记 nan)、
    价值网拟合(更新前价值对回报的解释方差)、各关卡的局数 / 步数 / 回报 / 结束原因、实际执行的动作分布
    (转向被裁到 ±1 的比例、速度动作落到限速 CAP 的比例和平均目标车速)。"""
    from common.road_policy import action_to_speed
    from training.explore_env import CAP
    mean = lambda x: float(np.mean(x)) if len(x) else float('nan')
    var = float(np.var(ret))
    per = {}
    for e, lv in zip(eps, levels_):
        p_ = per.setdefault(lv, {'episodes': 0, 'steps': 0, 'ret': [], 'why': {}})
        p_['episodes'] += 1; p_['steps'] += len(e[4]); p_['ret'].append(float(e[4].sum()))
        p_['why'][e[5]] = p_['why'].get(e[5], 0) + 1
    for p_ in per.values():
        p_['ret'] = round(float(np.mean(p_['ret'])), 2)
    v_cmd = np.array([action_to_speed(float(x), enc) for x in np.clip(acts[:, 1], -1, 1)])
    fwd = v_cmd > 0
    return {'iter': it, **{k: mean(v) for k, v in st.items()},
            'ev': 1.0 - float(np.var(ret - V)) / var if var > 0 else float('nan'),
            'levels': per,
            'steer_sat': float(np.mean(np.abs(acts[:, 0]) >= 1.0)),
            'speed_at_cap': float(np.mean(v_cmd >= CAP)),
            'speed_cmd': float(np.mean(np.minimum(v_cmd[fwd], CAP))) if fwd.any() else float('nan'),
            'stop_frac': float(np.mean(v_cmd == 0.0)), 'brake_frac': float(np.mean(v_cmd < 0.0))}   # 带 _d 契约才有


def train(a):
    import torch

    from common.bev_policy import H, W, channels_of
    from common.road_policy import spec_of
    from training.bev_net import load_into, make_net, weights_of

    torch.set_num_threads(a.torch_threads)
    torch.manual_seed(a.seed)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    ckpt = out / 'ckpt.pt'
    if ckpt.exists() != a.resume:
        raise SystemExit(f'{ckpt} ' + ('已存在:接着训加 --resume,重新训换个 --out' if ckpt.exists() else '不存在,没法 --resume'))
    (out / 'config.json' if not a.resume else out / f'config_resume_{time.strftime("%m%d_%H%M%S")}.json').write_text(
        json.dumps({k: str(v) for k, v in vars(a).items()}, indent=1), encoding='utf-8')
    with np.load(a.init) as z:
        schema = str(z['schema'])
    c_in = channels_of(schema)
    enc = spec_of('spica_v3' + schema[len('bev_v2'):])          # 同 BevPolicy
    actor, critic, ref = make_net(c_in), make_net(c_in), make_net(c_in)
    for net in (actor, critic, ref):
        load_into(net, a.init)
    if a.contract:                      # 换速度契约(比如加停车 / 刹车死区):速度那一行精确换算,锚定也在新动作空间里比
        new = spec_of('spica_v3' + a.contract)
        k, c = speed_head_affine(enc, new)
        with torch.no_grad():
            for net in (actor, ref):
                net.f2.weight[1] *= k
                net.f2.bias[1] = net.f2.bias[1] * k + c
        schema, enc = 'bev_v2' + a.contract, new
    with torch.no_grad():
        critic.f2.weight.zero_(); critic.f2.bias.zero_()
    if a.priv:                          # 价值网多吃 N_PRIV 个仿真内标量(Explore.privileged),新列从 0 开始
        old = critic.f0
        critic.f0 = torch.nn.Linear(old.in_features + N_PRIV, old.out_features)
        with torch.no_grad():
            critic.f0.weight.zero_()
            critic.f0.weight[:, :old.in_features].copy_(old.weight)
            critic.f0.bias.copy_(old.bias)
    for p_ in ref.parameters():
        p_.requires_grad_(False)
    dev = torch.device(a.device)        # 更新在 GPU 上跑(采样仍在 CPU 进程池;8 核虚拟机上更新占一轮的 2/3)
    for net in (actor, critic, ref):
        net.to(dev, memory_format=torch.channels_last)     # 卷积在 CPU 上快 30~40%(8 核虚拟机实测一批 320 -> 190 ms)
    log_std = torch.full((2,), float(np.log(a.std0)), device=dev, requires_grad=True)
    opt_a = torch.optim.Adam(list(actor.parameters()) + [log_std], lr=a.lr)
    opt_c = torch.optim.Adam(critic.parameters(), lr=a.lr_critic)
    bits = c_in * H * W
    rng = np.random.default_rng(a.seed)
    start = 0
    if a.resume:                        # 完整训练状态:同一条命令加 --resume,和不中断一口气跑完逐位一致
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
        """每轮结束存完整训练状态(先写临时文件再改名,中途断电也不会留下半个文件)。candidate_NNN.npz 只有推理权重,不能续训。"""
        tmp = out / 'ckpt.pt.tmp'
        torch.save({'iter': it, 'actor': actor.state_dict(), 'critic': critic.state_dict(), 'ref': ref.state_dict(),
                    'log_std': log_std.detach().clone(), 'opt_a': opt_a.state_dict(), 'opt_c': opt_c.state_dict(),
                    'rng': rng.bit_generator.state, 'torch_rng': torch.get_rng_state(), 'schema': schema,
                    'args': vars(a)}, tmp)
        os.replace(tmp, ckpt)

    log = open(out / 'train.log', 'a', encoding='utf-8', buffering=1)
    diag = open(out / 'diag.jsonl', 'a', encoding='utf-8', buffering=1)    # 每轮一行,见 diagnose

    def say(msg):
        line = f'{time.strftime("%H:%M:%S")} {msg}'
        print(line, flush=True)
        log.write(line + '\n')

    def params_of(net):
        return {k: v.astype(np.float32) for k, v in weights_of(net).items()}

    def unpack(P, j):
        return torch.from_numpy(np.unpackbits(P[j], axis=1)[:, :bits].reshape(len(j), c_in, H, W).astype(np.float32)).to(dev).contiguous(memory_format=torch.channels_last)

    def infer(net, P, S):
        out_ = []
        with torch.no_grad():
            for s in range(0, len(P), 1024):
                j = np.arange(s, min(len(P), s + 1024))
                out_.append(net(unpack(P, j), S[j]))
        return torch.cat(out_)

    mix = [(x.split(':')[0], float(x.split(':')[1]) if ':' in x else 1.0) for x in a.level.split(',')]
    names, probs = [n for n, _ in mix], np.array([p_ for _, p_ in mix]) / sum(p_ for _, p_ in mix)

    def run_eval(ex, it):
        """每个场地各 eval_count 局(同一批种子)。"""
        w = params_of(actor)
        np.savez(out / f'candidate_{it:03d}.npz', schema=np.array(schema), **weights_of(actor))
        msg = []
        for nm in names:
            ev = list(ex.map(evaluate, [(EVAL_SEED0 + i, w, enc, a.duration, nm) for i in range(a.eval_count)]))
            why = [e[0] for e in ev]
            m = (f'{nm}: 面积 {np.mean([e[1] for e in ev]):.1f} m² 截停/撞 '
                 f'{np.mean([w_ in ("guard", "collision") for w_ in why]):.0%}')
            if room_level(nm) is None:  # 路段关卡:出界、到终点也报(只看截停时看不出往路外开)
                m += f' 出界 {why.count("off_track") / len(why):.0%} 到终点 {why.count("goal") / len(why):.0%}'
            msg.append(m)
        return 'eval ' + ' / '.join(msg)

    with ProcessPoolExecutor(a.workers) as ex:
        if a.resume:
            say(f'从 {ckpt} 第 {start} 轮接着训')
        else:
            say(f'初始 {a.init} ' + run_eval(ex, -1))
        for it in range(start, a.iters):
            t0 = time.time()
            w = params_of(actor)
            jobs = [(TRAIN_SEED0 + int(rng.integers(10**7)), int(rng.integers(2**31)), w,
                     log_std.detach().cpu().numpy().copy(), enc, a.duration, names[rng.choice(len(names), p=probs)]) for _ in range(a.episodes)]
            eps = list(ex.map(rollout, jobs))
            t_roll = time.time() - t0
            P = np.concatenate([e[0] for e in eps])
            S = torch.from_numpy(np.concatenate([e[1] for e in eps])).to(dev)
            A = torch.from_numpy(np.concatenate([e[2] for e in eps])).to(dev)
            LP = torch.from_numpy(np.concatenate([e[3] for e in eps])).to(dev)
            SC = torch.cat([S, torch.from_numpy(np.concatenate([e[8] for e in eps])).to(dev)], 1) if a.priv else S   # 价值网的标量
            V = infer(critic, P, SC)[:, 0].cpu().numpy()
            MU_REF = infer(ref, P, S)           # 参考网络不变,一轮算一次,不用每个 minibatch 再跑
            # 到点(timeout)不是真结束:用最后一拍动作**之后**那个观测的价值接着算(Pardo 2018 partial-episode
            # bootstrapping)。2026-10-18 之前误用了最后一拍动作之前的观测 V(s_{T-1}),C 组"PEB 无效"的对照不作数
            cut = [i for i, e in enumerate(eps) if e[9] is not None]
            V_T = {}
            if cut and not a.no_peb:
                PT = np.stack([eps[i][9][0] for i in cut])
                ST = torch.from_numpy(np.stack([eps[i][9][1] for i in cut])).to(dev)
                if a.priv:
                    ST = torch.cat([ST, torch.from_numpy(np.stack([eps[i][9][2] for i in cut])).to(dev)], 1)
                V_T = dict(zip(cut, infer(critic, PT, ST)[:, 0].cpu().numpy()))
            advs, rets, o = [], [], 0
            for i, e in enumerate(eps):
                n = len(e[4])
                tail = V_T.get(i, 0.0)
                ad, rt = gae(e[4], np.r_[V[o:o + n], tail], a.gamma, a.lam, terminal=e[5] != 'timeout')
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
                    j = perm[s:s + a.minibatch]
                    X = unpack(P, j)
                    j = torch.as_tensor(j, device=dev)
                    Sj = S[j]
                    vloss = ((critic(X, SC[j])[:, 0] - RET[j]) ** 2).mean()
                    opt_c.zero_grad(); vloss.backward()
                    st['gn_c'].append(grad_norm(critic.parameters())); st['vloss'].append(vloss.item())
                    opt_c.step()
                    if it < a.critic_warmup:
                        continue
                    mu = actor(X, Sj)
                    ls = log_std.expand_as(mu)
                    logp = (-0.5 * ((A[j] - mu) / ls.exp()) ** 2 - ls - 0.5 * np.log(2 * np.pi)).sum(1)
                    ratio = (logp - LP[j]).exp()
                    pg = -torch.min(ratio * ADV[j], ratio.clamp(1 - a.clip, 1 + a.clip) * ADV[j]).mean()
                    an = ((mu - MU_REF[j]) ** 2).mean()
                    aloss = pg + anchor * an
                    opt_a.zero_grad(); aloss.backward()
                    st['gn_a'].append(grad_norm(list(actor.parameters()) + [log_std]))
                    opt_a.step()
                    with torch.no_grad():
                        lr_ = logp - LP[j]
                        st['kl'].append(float(((lr_.exp() - 1) - lr_).mean()))      # k3 估计,非负
                        st['clip'].append(float(((lr_.exp() - 1).abs() > a.clip).float().mean()))
                    st['pg'].append(pg.item()); st['anchor_loss'].append(an.item())
                    with torch.no_grad():
                        log_std.clamp_(np.log(0.02), np.log(0.5))
            t_upd = time.time() - t0 - t_roll
            reasons = {}
            for e in eps:
                reasons[e[5]] = reasons.get(e[5], 0) + 1
            d = diagnose(it, eps, [j_[6] for j_ in jobs], V, ret_np, A.cpu().numpy(), enc, st)
            diag.write(json.dumps(d, ensure_ascii=False) + '\n')
            msg = (f'iter {it} steps={N} {reasons} ret={np.mean([e[4].sum() for e in eps]):.1f} '
                   f'面积={np.mean([e[6] for e in eps]):.1f} 距离={np.mean([e[7] for e in eps]):.1f} '
                   f'std={np.exp(log_std.detach().cpu().numpy()).round(3).tolist()} anchor={anchor:.3f} '
                   f'kl={d["kl"]:.4f} clip={d["clip"]:.2f} ev={d["ev"]:.2f} '
                   f'[{t_roll:.0f}s/{t_upd:.0f}s]')
            if (it + 1) % a.eval_every == 0 or it == a.iters - 1:
                msg += ' | ' + run_eval(ex, it)
            say(msg)
            save(it)
    say('done')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--init', required=True, help='起点模型(bev_v2 schema)')
    ap.add_argument('--out', required=True)
    ap.add_argument('--iters', type=int, default=200)
    ap.add_argument('--episodes', type=int, default=64, help='每轮局数')
    ap.add_argument('--duration', type=float, default=60.0, help='每局最长 s')
    ap.add_argument('--level', default='room:0.3,room_f:0.7',
                    help='场地混合 名字:比例(training/explore_env.levels:room 空房间、room_n 墙面有起伏、room_f 再加少量家具、building 整栋楼)')
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--torch-threads', type=int, default=8)
    ap.add_argument('--device', default='cpu', help='更新用的设备:cpu / cuda')
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
    ap.add_argument('--anchor-min', type=float, default=0.3,
                    help='锚定衰减的下限(0.05 时第二轮锚定到底后 45 轮测试集 74%% -> 57%%,2026-10-18 改 0.3)')
    ap.add_argument('--critic-warmup', type=int, default=2, help='前几轮只训价值网')
    ap.add_argument('--eval-every', type=int, default=5)
    ap.add_argument('--eval-count', type=int, default=32)
    ap.add_argument('--priv', action='store_true', help='价值网多吃仿真内信息(非对称 actor-critic,见 Explore.privileged)')
    ap.add_argument('--no-peb', action='store_true', help='到点也按真结束算(2026-10-18 之前的做法,对照用)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--contract', default=None,
                    help='换速度契约后缀,比如 _s3_v0.45-1_d0.2(能停车 / 刹车,见 common/road_policy);起点模型的速度头精确换算过去')
    ap.add_argument('--resume', action='store_true', help='从 --out 里的 ckpt.pt 接着训(命令其余部分照旧;--iters 可以加大)')
    train(ap.parse_args(argv))


if __name__ == '__main__':
    main()
