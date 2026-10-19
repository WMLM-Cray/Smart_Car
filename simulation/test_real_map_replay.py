"""回放起步门控:旧契约零动作也前进,首帧 FREE 到达前不能执行策略/动力学。"""
import numpy as np
import pytest

from common.road_policy import action_to_speed, spec_of
from simulation.fastsim.real_map import RealMap


@pytest.fixture
def replay(monkeypatch):
    from common import bev_policy
    from evaluation import real_map_replay
    from simulation.fastsim.real_map_env import MapFloorSeg
    from training import bev_env

    ij = np.array([(i, j) for i in range(-40, 41) for j in range(-40, 41)])
    walls = ij[(abs(ij[:, 0]) == 40) | (abs(ij[:, 1]) == 40)]
    world = RealMap(walls, free_ij=ij)
    monkeypatch.setattr(real_map_replay, 'load_world', lambda *args: world)
    calls, envs, scans = [], [], []
    original = bev_env.BevEnv
    options = dict(free=True, empty=False)

    def detect(self, pose):
        return np.empty((0, 2)) if options['empty'] else np.array([[1.0, 0.0]])

    monkeypatch.setattr(MapFloorSeg, 'detect', detect)

    def make_env(*args, **kwargs):
        env = original(*args, **kwargs)
        envs.append(env)
        scan = env.env._scan

        def capture_scan():
            scans.append(env.env.t)
            scan()

        env.env._scan = capture_scan
        return env

    class Policy:
        enc = spec_of('road_v2.5')          # 老契约:零速度动作解码为正目标速度

        def __init__(self, path):
            self.free = options['free']

        def __call__(self, img, sc):
            env = envs[0]
            calls.append((env.env.t, env.pose, env.env.car.v,
                          img[5].sum() if self.free else None, env.seg_frame))
            return np.zeros(2)

    monkeypatch.setattr(bev_env, 'BevEnv', make_env)
    monkeypatch.setattr(bev_policy, 'BevPolicy', Policy)

    def run(duration, free=True, empty=False, lag=0.35):
        options.update(free=free, empty=empty)
        monkeypatch.setattr(real_map_replay, 'SEG_LAG', lag)
        result = real_map_replay.run_one(('unused', None, None, 'unused', (0, 0, 0), 1000, duration, True))
        return result, calls, envs[0], scans

    return run


def test_no_motion_or_policy_before_first_floor_but_sensors_advance(replay):
    (why, dist, elapsed, path), calls, env, scans = replay(0.3)
    assert action_to_speed(0.0, env.env.pl.enc) > 0
    assert why == 'timeout' and calls == []
    assert dist == 0 and env.env.car.v == 0 and env.env.speed.t == 0
    assert np.all(path == 0) and env.pose == (0, 0, 0)
    assert 0.3 <= elapsed < 0.36 and len(scans) >= 3
    assert env.seg_frame is None and not env.bev.crop_layers(env.pose, elapsed)[5].any()


def test_first_policy_and_movement_only_after_floor_delivery(replay):
    (why, dist, elapsed, path), calls, env, scans = replay(1.0)
    assert why == 'timeout' and dist > 0
    t, pose, speed, floor, frame = calls[0]
    assert t == pytest.approx(0.35) and pose == (0, 0, 0) and speed == 0
    assert frame is not None and frame[2] == 0.0 and floor > 0
    assert np.all(path[:7] == 0) and (path[7:] != 0).any()
    assert calls[-1][2] > 0 and len(scans) > 3


def test_empty_but_fresh_frame_also_releases_gate(replay):
    _, calls, _, _ = replay(0.4, empty=True)
    assert calls and calls[0][0] == pytest.approx(0.35)
    assert calls[0][3] == 0 and calls[0][4] is not None


def test_stale_capture_cannot_release_gate(replay):
    (why, dist, _, _), calls, env, _ = replay(1.3, lag=1.2)
    assert env.seg_frame is not None
    assert why == 'timeout' and calls == [] and dist == 0


@pytest.mark.parametrize('free,lag', [(False, 0.35), (True, 0.0)])
def test_models_without_floor_or_with_immediate_floor_start_at_zero(replay, free, lag):
    _, calls, _, _ = replay(0.1, free=free, lag=lag)
    assert calls and calls[0][0] == 0.0
