"""road_v2.5 的 TD3 精修:Twin Delayed DDPG,与 PPO(road_rl)同赛道池、同奖励、同评测口径。

    python -m training.road_td3 pool --config training/config/rl_td3.yaml --out runs/rl_pool.pkl
    python -m training.road_td3 train --config training/config/rl_td3.yaml --pool runs/rl_pool.pkl \
        --init models/road_v2.5/candidate_b036.npz --out runs/road_v2.5_td3

为什么换 TD3:PPO 两轮训练都在 40-60 轮后渐进劣化 —— on-policy 每轮整批更新,
一次坏策略会把整批数据带偏。TD3 的四道保险是另一条路:
  双 Q    取 min(Q1,Q2) 做 target,压制 Q 值高估
  延迟    策略更新频率只有评论家的一半,价值网络先站稳,策略再追
  平滑    target 动作加截断噪声,别把 Q 的尖峰当真
  锚定    actor 偏离初始模型的动作 MSE 惩罚(bc)。冒烟实测:少量数据下 Q 几乎平坦,
          Adam 的步长与梯度大小无关,纯噪声 500 步就能把好策略推出动作域 ——
          必须有锚把无信号方向拉住。PPO 的 anchor 同理。
另外 off-policy 用回放缓冲区,旧的好数据不会被一次坏策略冲掉;
采集与更新按 rounds 交错进行,策略不会在两批数据之间漂移上千步。

可比性设计(与 road_rl 的 PPO 严格对齐):
  同一赛道池 / 同一 RlEnv 奖励(crash/time_cost/slip)/ 同一初始化(candidate_b036)
  / 同一套挑模型种子评估(SELECT_SEED0)/ 同一 candidate npz 导出格式。
  因此报告里三列(基线 / PPO / TD3)的差可以归因到算法本身。

代价:off-policy 每轮要做上万次小批量梯度更新(CPU 上比 PPO 慢),
obs 存 float16 的回放缓冲区占 ~0.5GB 内存。
"""
from __future__ import annotations

import os

for _k in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_k, '1')

import argparse
import pickle
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from common.road_policy import DIM, FRAMES, SCHEMA_SPICA, spec_of, suffix_of
from training.rl_env import RlEnv
from training.road_rl import _apply_config, _git_commit, build_pool, load_config, mlp_np

CONFIG_PATH = str(Path(__file__).parent / 'config' / 'rl_td3.yaml')

_POOL = None


def _init_worker(pool_path):
    global _POOL
    with open(pool_path, 'rb') as f:
        _POOL = pickle.load(f)


def collect(args):
    """用当前确定性策略 + 高斯探索噪声跑一局,返回逐拍转移。

    obs 存 (n+1, D) float16:首行是 s0,第 t+1 行是执行 a_t 后的 s'。
    done[t] = 1 仅当第 t 拍真终止(goal/collision/off_track);
    超时截断的末拍 done=0 —— 评论家要对截断末态 bootstrap,和 PPO 的 GAE 口径一致。

    转向专项(args 末尾 vcfg 非 None):速度分量被 v_plan 剖面接管(与 evaluation.road_lap2
    的 plan 模式同源),策略只学转向 —— vcfg=(a_lat_sup, a_lat_low, kappa_split, a_acc, a_dec)。
    """
    idx, noise_seed, w, b, explore_std, crash, time_cost, slip, max_steps = args[:9]
    vcfg = args[9] if len(args) > 9 else None
    ep = _POOL[idx]
    env = RlEnv(ep['road'], ep['pl'], noise_seed, ep['start'], noise=True,
                crash=crash, time_cost=time_cost, slip=slip, max_steps=max_steps)
    rng = np.random.default_rng(noise_seed)
    vg_s = vg_v = None
    if vcfg is not None:                       # 预计算全程速度剖面(lap_len 传 1.0 → 从 s=0 起算)
        from evaluation.road_lap2 import v_profile
        from common.road_policy import speed_to_action
        vg_s, vg_v = v_profile(ep['road'], 1.0, float(ep['road'].s[-1]), vcfg[0],
                               ep['pl'].cap, ep['pl'].v_top, 0.3, 99.0,
                               vcfg[3], vcfg[4], vcfg[1], vcfg[2])
    xs, acts, rews, dones = [env.reset()], [], [], []
    while True:
        mu = mlp_np(w, b, xs[-1].astype(np.float32))
        a = mu + rng.normal(0, 1, 2) * explore_std
        if vg_s is not None:                   # 速度听规划器,只保留转向探索
            v_now = float(np.interp(env.env.road.s[env.env.idx], vg_s, vg_v))
            a[1] = speed_to_action(v_now, ep['pl'].enc)
        x2, r, term, trunc, info = env.step(a)      # RlEnv 内部 clip 到 ±1
        xs.append(x2)
        acts.append(a.astype(np.float32))
        rews.append(r)
        dones.append(1.0 if term else 0.0)
        if term or trunc:
            break
    return (np.asarray(xs, np.float16), np.asarray(acts, np.float32),
            np.asarray(rews, np.float32), np.asarray(dones, np.float32),
            info['reason'], ep['level'], ep['pl'].cap, info['time'])


class Replay:
    """环形回放缓冲区。obs 用 float16 存(内存减半),采样时转 float32。"""

    def __init__(self, capacity, obs_dim):
        self.cap = int(capacity)
        self.n = 0                                   # 缓冲区内可用条数
        self.i = 0                                   # 下一个写位置
        self.s = np.zeros((self.cap, obs_dim), np.float16)
        self.s2 = np.zeros((self.cap, obs_dim), np.float16)
        self.a = np.zeros((self.cap, 2), np.float32)
        self.r = np.zeros(self.cap, np.float32)
        self.d = np.zeros(self.cap, np.float32)

    def add_episode(self, xs, acts, rews, dones):
        """xs 是 (n+1, D):第 t 拍转移为 (xs[t], acts[t], rews[t], xs[t+1], dones[t])。"""
        for t in range(len(acts)):
            j = self.i
            self.s[j] = xs[t]
            self.s2[j] = xs[t + 1]
            self.a[j] = acts[t]
            self.r[j] = rews[t]
            self.d[j] = dones[t]
            self.i = (self.i + 1) % self.cap
            self.n = min(self.n + 1, self.cap)

    def sample(self, rng, batch):
        j = rng.integers(0, self.n, size=batch)
        return (self.s[j].astype(np.float32), self.s2[j].astype(np.float32),
                self.a[j], self.r[j], self.d[j])


def train(a):
    import torch
    from torch import nn

    from training.road_teacher import A_LAT
    from training.road_train import SELECT_SEED0, _job, summarize

    torch.set_num_threads(a.torch_threads)
    torch.manual_seed(a.seed)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    import json
    meta = {k: str(v) for k, v in vars(a).items()}
    meta['git_commit'] = _git_commit()
    (out / 'config.json').write_text(json.dumps(meta, indent=1), encoding='utf-8')
    D = DIM * FRAMES

    def net(n_in, n_out):
        return nn.Sequential(nn.Linear(n_in, 256), nn.Tanh(), nn.Linear(256, 256), nn.Tanh(),
                             nn.Linear(256, n_out))
    actor = net(D, 2)
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
    schema = SCHEMA_SPICA + suffix_of(enc)
    with open(a.pool, 'rb') as f:
        pool = pickle.load(f)
    if {ep['pl'].enc for ep in pool} != {enc}:
        raise SystemExit(f'赛道池的契约和 {a.init} 的 {enc} 不一致')
    n_pool = len(pool)

    critic1, critic2 = net(D + 2, 1), net(D + 2, 1)
    at, c1t, c2t = net(D, 2), net(D + 2, 1), net(D + 2, 1)
    for t_, src in ((at, actor), (c1t, critic1), (c2t, critic2)):
        t_.load_state_dict(src.state_dict())
        for p_ in t_.parameters():
            p_.requires_grad_(False)
    opt_a = torch.optim.Adam(actor.parameters(), lr=a.lr)
    opt_c = torch.optim.Adam(list(critic1.parameters()) + list(critic2.parameters()), lr=a.lr_critic)
    ref = net(D, 2)                                  # 冻结的初始模型,给 actor 锚定用
    ref.load_state_dict(actor.state_dict())
    for p_ in ref.parameters():
        p_.requires_grad_(False)

    buf = Replay(a.buffer, D)
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
            t0, t_roll = time.time(), 0.0
            n_new, n_upd, q1v, drift = 0, 0, float('nan'), float('nan')
            reasons, all_eps = {}, []
            per_round = max(1, a.episodes // a.rounds)
            budget = a.max_updates
            # 交错式更新:采一小批 -> 按比例更新,循环。整轮采集再集中更新会让策略
            # 在两批数据之间漂移上千步(冒烟实测 2000 次更新足以把好策略彻底摧毁)。
            for r0 in range(0, a.episodes, per_round):
                w, b = weights(actor)
                _vl = getattr(a, 'v_plan_a_lat', None)
                vcfg = (None if _vl is None else
                        (float(_vl), getattr(a, 'v_plan_a_lat_low', None),
                         float(getattr(a, 'v_plan_kappa_split', 0.55)),
                         float(getattr(a, 'v_plan_a_acc', 0.6)),
                         float(getattr(a, 'v_plan_a_dec', 0.55))))
                jobs = [(int(rng.integers(n_pool)), int(rng.integers(2**31)), w, b, a.explore_std,
                         a.crash, a.time_cost, a.slip, a.max_steps, vcfg)
                        for _ in range(min(per_round, a.episodes - r0))]
                ts = time.time()
                eps = list(ex.map(collect, jobs))
                t_roll += time.time() - ts
                for e in eps:
                    buf.add_episode(e[0], e[1], e[2], e[3])
                    reasons[e[4]] = reasons.get(e[4], 0) + 1
                steps = sum(len(e[1]) for e in eps)
                n_new += steps
                all_eps += eps
                if buf.n < a.warmup:
                    continue
                k = min(int(steps * a.utd), budget)
                budget -= k
                for u in range(k):
                    s, s2, ac, rew, dn = buf.sample(rng, a.batch)
                    S, S2, A_, R_, D_ = (torch.from_numpy(x) for x in (s, s2, ac, rew, dn))
                    with torch.no_grad():
                        noise = (torch.randn(a.batch, 2) * a.target_noise).clamp(-a.noise_clip, a.noise_clip)
                        a2 = (at(S2) + noise).clamp(-1, 1)
                        q = torch.min(c1t(torch.cat([S2, a2], 1)).squeeze(1),
                                      c2t(torch.cat([S2, a2], 1)).squeeze(1))
                        y = R_ + a.gamma * (1 - D_) * q
                    q1 = critic1(torch.cat([S, A_], 1)).squeeze(1)
                    q2 = critic2(torch.cat([S, A_], 1)).squeeze(1)
                    closs = ((q1 - y) ** 2).mean() + ((q2 - y) ** 2).mean()
                    opt_c.zero_grad(); closs.backward(); opt_c.step()
                    if (u + 1) % a.policy_delay == 0:
                        # 锚定:Q 平坦时 Adam 的噪声步会把策略推飞(实测 500 步漂出动作域),
                        # bc 项把无信号方向的漂移拉回初始模型,有信号时才允许偏离。
                        mu = actor(S)
                        with torch.no_grad():
                            mu_ref = ref(S)
                        aloss = (-critic1(torch.cat([S, mu], 1)).squeeze(1).mean()
                                 + a.bc * ((mu - mu_ref) ** 2).mean())
                        opt_a.zero_grad(); aloss.backward(); opt_a.step()
                        with torch.no_grad():
                            for t_, src in ((at, actor), (c1t, critic1), (c2t, critic2)):
                                for pt, ps in zip(t_.parameters(), src.parameters()):
                                    pt.mul_(1 - a.tau).add_(a.tau * ps)
                    if q1v != q1v:
                        q1v = float(q1.detach().mean())
                if k:
                    with torch.no_grad():
                        drift = float((actor(S) - ref(S)).abs().max())
                n_upd += k
            t_upd = time.time() - t0 - t_roll

            ok = [e for e in all_eps if e[4] == 'goal']
            msg = (f'iter {it} steps={n_new} buf={buf.n} goal={len(ok) / len(all_eps):.3f} {reasons} '
                   f'ret={np.mean([e[2].sum() for e in all_eps]):.2f} '
                   f'ep_time={np.mean([e[7] for e in ok]) if ok else 0:.1f}s '
                   f'q1={q1v:.1f} drift={drift:.3f} upd={n_upd} [{t_roll:.0f}s/{t_upd:.0f}s]')
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


def make_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('pool')
    p.add_argument('--out', required=True)
    p.add_argument('--config', default=CONFIG_PATH, help='实验配置 yaml(默认 training/config/rl_td3.yaml)')
    p.add_argument('--count', type=int, default=None)
    p.add_argument('--mix', default=None)
    p.add_argument('--cap-max', type=float, default=None)
    p.add_argument('--workers', type=int, default=None)
    p.add_argument('--schema', default=None, help='赛道池按哪个速度契约抽车辆参数(要和 --init 的一致)')
    p.add_argument('--pool-seed0', type=int, default=None,
                   help='赛道池种子起点(默认取配置,与 DAgger 训练/挑模型/测试种子不重叠)')
    t = sub.add_parser('train')
    t.add_argument('--pool', required=True)
    t.add_argument('--init', required=True)
    t.add_argument('--out', required=True)
    t.add_argument('--config', default=CONFIG_PATH, help='实验配置 yaml(默认 training/config/rl_td3.yaml)')
    t.add_argument('--cap-max', type=float, default=None, help='挑模型种子评估的速度抽样上限')
    t.add_argument('--crash', type=float, default=None)
    t.add_argument('--time-cost', type=float, default=None)
    t.add_argument('--slip', type=float, default=None)
    t.add_argument('--max-steps', type=int, default=None)
    t.add_argument('--iters', type=int, default=None)
    t.add_argument('--episodes', type=int, default=None)
    t.add_argument('--workers', type=int, default=None)
    t.add_argument('--torch-threads', type=int, default=None)
    t.add_argument('--seed', type=int, default=None)
    t.add_argument('--gamma', type=float, default=None)
    t.add_argument('--tau', type=float, default=None, help='目标网络软更新系数')
    t.add_argument('--lr', type=float, default=None)
    t.add_argument('--lr-critic', type=float, default=None)
    t.add_argument('--batch', type=int, default=None)
    t.add_argument('--buffer', type=int, default=None, help='回放缓冲区容量(条)')
    t.add_argument('--utd', type=float, default=None, help='每条新数据做几次梯度更新')
    t.add_argument('--max-updates', type=int, default=None, help='每轮梯度更新上限')
    t.add_argument('--warmup', type=int, default=None, help='缓冲区攒够多少条才开始更新')
    t.add_argument('--explore-std', type=float, default=None, help='采集时的高斯探索噪声')
    t.add_argument('--target-noise', type=float, default=None)
    t.add_argument('--noise-clip', type=float, default=None)
    t.add_argument('--policy-delay', type=int, default=None)
    t.add_argument('--rounds', type=int, default=None,
                   help='每轮采集拆成几批交错更新(防止策略在两批数据间漂移过远)')
    t.add_argument('--bc', type=float, default=None,
                   help='actor 锚定系数:偏离初始模型动作的平方误差惩罚')
    t.add_argument('--eval-every', type=int, default=None)
    t.add_argument('--eval-levels', default=None)
    t.add_argument('--eval-count', type=int, default=None)
    t.add_argument('--v-plan-a-lat', type=float, default=None,
                   help='转向专项:采集时速度被 v_plan 剖面接管;普通弯横向加速度假设 m/s²。'
                        'None=不接管(速度由策略自己出)')
    t.add_argument('--v-plan-a-lat-low', type=float, default=None,
                   help='转向专项:急弯(κ≥kappa-split)降档用的横向加速度假设')
    t.add_argument('--v-plan-kappa-split', type=float, default=None,
                   help='转向专项:急弯分档阈值 1/m(默认 0.55)')
    t.add_argument('--v-plan-a-acc', type=float, default=None,
                   help='转向专项:剖面前向加速假设 m/s²(默认 0.6)')
    t.add_argument('--v-plan-a-dec', type=float, default=None,
                   help='转向专项:剖面制动假设 m/s²(默认 0.55)')
    return ap, sub


def main():
    ap, sub = make_parser()
    a = ap.parse_args()
    cfg = load_config(a.config)
    _apply_config(cfg, sub, a.cmd, a, a.config)
    if a.cmd == 'pool':
        build_pool(a)
    else:
        train(a)


if __name__ == '__main__':
    main()
