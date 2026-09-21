import numpy as np
import torch
import pytest

from fastsim.low_speed_policy import DIM, History, SteeringPolicy, observe
from fastsim.low_speed_train import atomic_save, export, load_state, make_env, make_net


def test_resume_allows_extension_but_rejects_other_changes():
    from fastsim.low_speed_train import validate_resume_config
    old = dict(steps=500000, phase='A', lookahead=1.2)
    new = dict(old, steps=1000000, recovery_starts=True)
    validate_resume_config(old, new)
    with pytest.raises(ValueError):
        validate_resume_config(old, dict(new, lookahead=.8))
    with pytest.raises(ValueError):
        validate_resume_config(old, dict(new, steps=100))


def test_recovery_reset_has_correct_progress_and_no_initial_collision():
    env = make_env(10000402, .6, training=True, recovery_start=True)
    s, _, _ = env.track.project((env.car.x, env.car.y))
    assert env.last_s == s
    assert env.progress == 0
    assert not np.any(env.car.footprint_hits_circles(env.obstacles, env.tp.cone_base_radius))
    _, _, _, _, info = env.step([.4, 0])
    assert abs(info['lap_fraction']) < .01


def test_observation_uses_physical_speed_and_has_no_time():
    angles = np.linspace(-np.pi, np.pi, 1667, endpoint=False)
    a = observe(np.ones(1667), angles, .6, .8, .3, -.4)
    assert a.shape == (DIM,)
    np.testing.assert_allclose(a[-4:], [.4, .8 / 1.5, .1, -.4])
    np.testing.assert_array_equal(a, observe(np.ones(1667), angles + 2*np.pi, .6, .8, .3, -.4))


def test_numpy_export_matches_torch_through_real_observations(tmp_path):
    torch.manual_seed(3)
    net = make_net()
    path = tmp_path / 'candidate.npz'
    export(net, path)
    policy, history = SteeringPolicy(path), History()
    env = make_env(10000001, .6)
    last = 0.
    for _ in range(100):
        obs = observe(env.ranges, env.lidar.angles, env.encoder_speed, .6,
                      env.imu['angular_velocity_z'], last)
        with torch.no_grad():
            expected = float(np.clip(net(torch.from_numpy(history.push(obs))).item(), -1, 1))
        actual = policy(obs)
        np.testing.assert_allclose(actual, expected, atol=1e-6)
        last = actual
        _, _, term, trunc, _ = env.step([.6 / env.v_cmd_fwd, actual])
        if term or trunc:
            break
    assert env.terminate_on_cone
    policy.reset()
    assert not np.any(policy.history.buf)


def test_resume_restores_optimizer_replay_and_rng(tmp_path):
    from fastsim.low_speed_policy import SCHEMA
    torch.manual_seed(7)
    rng = np.random.default_rng(7)
    net = make_net()
    opt = torch.optim.Adam(net.parameters())
    x = torch.randn(2, DIM * 5)
    loss = net(x).square().mean()
    loss.backward()
    opt.step()
    path = tmp_path / 'latest.pt'
    replay = {.6: (x.numpy(), np.zeros((2, 1), np.float32))}
    atomic_save(dict(schema=SCHEMA, net=net.state_dict(), optimizer=opt.state_dict(),
        replay=replay, numpy_rng=rng.bit_generator.state, torch_rng=torch.get_rng_state()), path)
    want_np, want_torch = rng.random(4), torch.rand(4)
    restored = make_net()
    new_opt = torch.optim.Adam(restored.parameters())
    state = load_state(path, restored, new_opt, rng)
    np.testing.assert_array_equal(rng.random(4), want_np)
    torch.testing.assert_close(torch.rand(4), want_torch)
    np.testing.assert_array_equal(state['replay'][.6][0], x.numpy())
    torch.testing.assert_close(restored(x), net(x))
    for model, optimizer in ((net, opt), (restored, new_opt)):
        optimizer.zero_grad()
        model(x).square().mean().backward()
        optimizer.step()
    torch.testing.assert_close(restored(x), net(x))
