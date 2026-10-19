"""回放从使能后的新观测开始,不能原地预攒地面证据再起步。"""
import numpy as np

from common.road_policy import DEFAULT_ENC
from simulation.fastsim.real_map import RealMap


def test_replay_does_not_warm_up_before_first_action(monkeypatch):
    from common import bev_policy
    from evaluation import real_map_replay
    from training import bev_env
    ij = np.array([(i, j) for i in range(-40, 41) for j in range(-40, 41)])
    walls = ij[(abs(ij[:, 0]) == 40) | (abs(ij[:, 1]) == 40)]
    world = RealMap(walls, free_ij=ij)
    monkeypatch.setattr(real_map_replay, 'load_world', lambda *args: world)
    calls, envs = [], []
    original = bev_env.BevEnv

    def make_env(*args, **kwargs):
        env = original(*args, **kwargs)
        envs.append(env)
        return env

    class Policy:
        enc, free = DEFAULT_ENC, True

        def __init__(self, path):
            pass

        def __call__(self, img, sc):
            calls.append((envs[0].env.t, img[5].sum()))
            return np.zeros(2)

    monkeypatch.setattr(bev_env, 'BevEnv', make_env)
    monkeypatch.setattr(bev_policy, 'BevPolicy', Policy)
    why, dist, elapsed, path = real_map_replay.run_one(
        ('unused', None, None, 'unused', (0, 0, 0), 1000, 0.1, True))
    assert calls[0] == (0.0, 0.0)
    assert envs[0].env.pl.launch_delay <= 0.2
    assert why == 'timeout' and 0.1 <= elapsed <= 0.16
    assert len(path) == len(calls)
