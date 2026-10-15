"""road_td3 测试:缓冲区语义、采集转移一致性、TD3 更新步可跑通、配置键对得上。

不加载真实赛道池(177MB 太慢):collect 用 monkeypatch 往 _POOL 里塞一局
直接 generate 出来的 L3(测接口,不测可通性,与 test_rl_env 同一思路)。
"""
from __future__ import annotations

import numpy as np
import pytest

from common.road_policy import DIM, FRAMES
from simulation.fastsim.road import LEVELS, generate
from simulation.fastsim.road_env import Plant
from training import road_td3
from training.road_rl import load_config

D = DIM * FRAMES


@pytest.fixture
def one_episode():
    rng = np.random.default_rng(123)
    lv = next(l for l in LEVELS if l.name == 'L3')
    road = generate(rng, lv)
    pl = Plant.sample(rng)
    return dict(road=road, pl=pl, start=(0, 0.0, 0.0), level='L3')


def _weights():
    w = [np.random.default_rng(i).normal(0, 0.1, (256, D if i == 0 else 256)).astype(np.float32)
         for i in range(2)]
    w.append(np.random.default_rng(9).normal(0, 0.1, (2, 256)).astype(np.float32))
    b = [np.zeros((256,), np.float32), np.zeros((256,), np.float32), np.zeros((2,), np.float32)]
    return w, b


# ---------------- Replay ----------------

def test_replay_shapes_and_wraparound():
    buf = road_td3.Replay(capacity=8, obs_dim=D)
    xs = np.arange(5, dtype=np.float16)[:, None].repeat(D, 1)   # 第 t 行值 = t
    acts = np.ones((4, 2), np.float32)
    rews = np.arange(4, dtype=np.float32)
    dones = np.array([0, 0, 0, 1], np.float32)
    buf.add_episode(xs, acts, rews, dones)
    assert buf.n == 4
    s, s2, a, r, d = buf.sample(np.random.default_rng(0), 3)
    assert s.shape == (3, D) and s2.shape == (3, D)
    assert s.dtype == np.float32                      # 采样时升精度
    assert a.shape == (3, 2) and d.shape == (3,)
    # 转移语义:s[t]=xs[t](值 t),s2[t]=xs[t+1](值 t+1)
    assert all(buf.s[k].max() == k and buf.s2[k].max() == k + 1 for k in range(4))
    # 再塞 6 条,容量 8 环绕覆盖
    buf.add_episode(np.zeros((7, D), np.float16), np.ones((6, 2), np.float32),
                    np.zeros(6, np.float32), np.zeros(6, np.float32))
    assert buf.n == 8 and buf.i == 2


def test_replay_done_semantics():
    """terminated 末拍 done=1(不 bootstrap),截断末拍 done=0(要 bootstrap)。"""
    buf = road_td3.Replay(capacity=16, obs_dim=D)
    xs = np.arange(4, dtype=np.float16)[:, None].repeat(D, 1)
    acts = np.zeros((3, 2), np.float32)
    rews = np.zeros(3, np.float32)
    dones = np.array([0, 0, 1], np.float32)          # 第 3 拍撞车终止
    buf.add_episode(xs, acts, rews, dones)
    assert buf.d[2] == 1.0 and buf.d[0] == 0.0 and buf.d[1] == 0.0
    assert np.array_equal(buf.s[2], xs[2]) and np.array_equal(buf.s2[2], xs[3])


# ---------------- collect ----------------

def test_collect_transition_consistency(one_episode, monkeypatch):
    monkeypatch.setattr(road_td3, '_POOL', [one_episode])
    w, b = _weights()
    xs, acts, rews, dones, reason, level, cap, ep_time = road_td3.collect(
        (0, 77, w, b, 0.1, 50.0, 0.02, 0.5, 20))
    n = len(acts)
    assert xs.shape == (n + 1, D) and xs.dtype == np.float16
    assert acts.shape == (n, 2) and rews.shape == (n,) and dones.shape == (n,)
    assert np.isfinite(rews).all()
    assert reason == 'timeout' and dones[-1] == 0.0   # 步数上限截断 -> 末拍 bootstrap
    assert ep_time > 0


def test_collect_reproducible(one_episode, monkeypatch):
    monkeypatch.setattr(road_td3, '_POOL', [one_episode])
    w, b = _weights()
    out1 = road_td3.collect((0, 55, w, b, 0.1, 50.0, 0.02, 0.5, 10))
    out2 = road_td3.collect((0, 55, w, b, 0.1, 50.0, 0.02, 0.5, 10))
    assert np.array_equal(out1[1], out2[1])           # 同噪声种子 -> 动作逐位一致


# ---------------- TD3 更新步 ----------------

def test_td3_update_step(one_episode, monkeypatch):
    """攒几局进缓冲区,跑 3 次 TD3 更新:损失有限、权重真的动了。"""
    torch = pytest.importorskip('torch')
    monkeypatch.setattr(road_td3, '_POOL', [one_episode])
    w, b = _weights()
    eps = [road_td3.collect((0, s, w, b, 0.1, 50.0, 0.02, 0.5, 15)) for s in (1, 2, 3)]
    buf = road_td3.Replay(4096, D)
    for e in eps:
        buf.add_episode(e[0], e[1], e[2], e[3])
    assert buf.n > 0

    from torch import nn
    rng = np.random.default_rng(0)
    actor = nn.Sequential(nn.Linear(D, 256), nn.Tanh(), nn.Linear(256, 256), nn.Tanh(),
                          nn.Linear(256, 2))
    c1 = nn.Sequential(nn.Linear(D + 2, 256), nn.Tanh(), nn.Linear(256, 256), nn.Tanh(),
                       nn.Linear(256, 1))
    c2 = nn.Sequential(nn.Linear(D + 2, 256), nn.Tanh(), nn.Linear(256, 256), nn.Tanh(),
                       nn.Linear(256, 1))
    at_, c1t, c2t = (nn.Sequential(*[m for m in net]) for net in (actor, c1, c2))
    for t_, src in ((at_, actor), (c1t, c1), (c2t, c2)):
        t_.load_state_dict(src.state_dict())
    opt_a = torch.optim.Adam(actor.parameters(), lr=5e-5)
    opt_c = torch.optim.Adam(list(c1.parameters()) + list(c2.parameters()), lr=3e-4)
    w0 = actor[0].weight.detach().numpy().copy()

    tau, gamma, batch = 0.005, 0.99, 32
    for u in range(3):
        s, s2, ac, rew, dn = buf.sample(rng, batch)
        S, S2, A_, R_, D_ = (torch.from_numpy(x) for x in (s, s2, ac, rew, dn))
        with torch.no_grad():
            noise = (torch.randn(batch, 2) * 0.2).clamp(-0.5, 0.5)
            a2 = (at_(S2) + noise).clamp(-1, 1)
            q = torch.min(c1t(torch.cat([S2, a2], 1)).squeeze(1),
                          c2t(torch.cat([S2, a2], 1)).squeeze(1))
            y = R_ + gamma * (1 - D_) * q
        q1, q2 = c1(torch.cat([S, A_], 1)).squeeze(1), c2(torch.cat([S, A_], 1)).squeeze(1)
        closs = ((q1 - y) ** 2).mean() + ((q2 - y) ** 2).mean()
        assert np.isfinite(float(closs))
        opt_c.zero_grad(); closs.backward(); opt_c.step()
        if (u + 1) % 2 == 0:
            aloss = -c1(torch.cat([S, actor(S)], 1)).squeeze(1).mean()
            assert np.isfinite(float(aloss))
            opt_a.zero_grad(); aloss.backward(); opt_a.step()
            with torch.no_grad():
                for t_, src in ((at_, actor), (c1t, c1), (c2t, c2)):
                    for pt, ps in zip(t_.parameters(), src.parameters()):
                        pt.mul_(1 - tau).add_(tau * ps)
    assert not np.array_equal(w0, actor[0].weight.detach().numpy())


# ---------------- 配置与入口 ----------------

def test_config_keys_all_recognized():
    """rl_td3.yaml 里每个键都能被某个子命令认出(打错键 = 立即报错的纪律)。"""
    cfg = load_config(road_td3.CONFIG_PATH)
    ap, sub = road_td3.make_parser()
    union = {act.dest for sp in sub.choices.values() for act in sp._actions}
    extra = set(cfg) - union
    assert not extra, f'配置里有不认识的键: {sorted(extra)}'


def test_config_fills_train_args():
    ap, sub = road_td3.make_parser()
    a = ap.parse_args(['train', '--pool', 'p.pkl', '--init', 'i.npz', '--out', 'o'])
    from training.road_rl import _apply_config
    _apply_config(load_config(road_td3.CONFIG_PATH), sub, 'train', a, road_td3.CONFIG_PATH)
    assert a.iters == 100 and a.gamma == 0.99 and a.policy_delay == 2
    assert a.crash == 50.0 and a.max_steps == 3000    # 奖励与 PPO 完全一致(可比性)
    assert a.rounds == 8 and a.bc == 10.0             # 交错更新 + 锚定(见 road_td3 文档字符串)
