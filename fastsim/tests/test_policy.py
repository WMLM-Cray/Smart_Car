"""车上跑的 Policy 必须和训练时的 env+网络逐位一致 —— 不一致就绝不上车。

观测差半度、叠帧顺序反了、归一化漏了,现象都是"车莫名其妙乱开",事后极难查。
所以这里不测"差不多",测的是完全相同的动作序列。
"""
import subprocess
import sys

import numpy as np
import pytest
import torch

from fastsim.curriculum import STAGES
from fastsim.policy import Policy

CKPT = 'runs/dagger_s3_w256.pt'


@pytest.fixture(scope='module')
def npz(tmp_path_factory):
    if not __import__('pathlib').Path(CKPT).exists():
        pytest.skip(f'没有 {CKPT}')
    out = tmp_path_factory.mktemp('p') / 'policy.npz'
    subprocess.run([sys.executable, 'fastsim/legacy/export_policy.py', CKPT, str(out)], check=True)
    return str(out)


def test_numpy_policy_matches_training_pipeline(npz):
    """同一条赛道跑 200 步,Policy 和 Student+env 给出的动作必须完全相同。"""
    from fastsim.legacy.dagger import Student, make_net

    ck = torch.load(CKPT, map_location='cpu')
    dim, frames = int(ck['obs_dim']), int(ck['frames'])
    net = make_net(dim, ck['width'], frames)
    net.load_state_dict(ck['state_dict'])
    student = Student(net, dim, frames)

    stage = STAGES['s3']
    env = stage.make_env(mapping=False)
    obs, _ = env.reset(seed=90000)
    pol = Policy(npz)

    for step in range(200):
        want = student(obs)
        # Policy 只拿车上拿得到的东西:原始测距、雷达角度、编码器、IMU、剩余时间
        got = pol.forward(pol.observe(env.ranges, env.lidar.angles, env.encoder_speed,
                                      env.imu['angular_velocity_z'],
                                      max(0.0, 1.0 - env.time / env.budget)))
        # 网络输出本身不受约束(实测会给到 1.00002),env 和 Policy 都是 clip 后才用
        np.testing.assert_allclose(got, np.clip(want, -1, 1), atol=1e-5,
                                   err_msg=f'第 {step} 步动作对不上')
        obs, _, term, trunc, _ = env.step(want)
        if term or trunc:
            break


def test_sector_mapping_is_resolution_independent(npz):
    """实车 1667 点、队友 Gazebo 2000 点,过同一个 Policy 都得到 48 维。"""
    pol = Policy(npz)
    for n in (1667, 2000):
        angles = -np.pi + np.arange(n) * (2 * np.pi / n)
        o = pol.observe(np.full(n, 3.0), angles, 1.0, 0.0, 1.0)
        assert o.shape == (pol.obs_dim,)
        assert np.allclose(o[:pol.n_sectors], 0.3)


def test_angles_wrap_to_pi(npz):
    """雷达按 0..2π 发角度时也得落对扇区 —— 不折回去只会安静地开错。"""
    n = 1667
    a = -np.pi + np.arange(n) * (2 * np.pi / n)
    r = np.full(n, 9.0)
    r[np.argmin(np.abs(a - np.pi / 4))] = 1.0        # 左前 45° 放一个近点
    want = Policy(npz).observe(r, a, 0.0, 0.0, 1.0)
    got = Policy(npz).observe(r, a % (2 * np.pi), 0.0, 0.0, 1.0)
    np.testing.assert_allclose(got, want)
