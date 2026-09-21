import numpy as np
import pytest

from fastsim.multispeed import SPEED_SCALE, make_env, summary


def test_speed_units_and_target_are_separate():
    slow, fast = make_env(1, .5), make_env(1, 2.5)
    a, _ = slow.reset(seed=1)
    b, _ = fast.reset(seed=1)
    np.testing.assert_array_equal(a[:-1], b[:-1])
    assert a[-1] == pytest.approx(.5 / SPEED_SCALE)
    assert b[-1] == 1
    for env in (slow, fast):
        env.unwrapped.encoder_speed = .4
        assert env.unwrapped._observation()[48] == pytest.approx(.4 / SPEED_SCALE)
    slow.step([1, .2])
    assert slow.unwrapped.v_cmd == pytest.approx(.5)
    np.testing.assert_allclose(slow.unwrapped.last_action, [1, .2])


def test_delayed_command_and_reset_clear_history():
    env = make_env(4, .8)
    env.delay_steps = 2
    env.reset(seed=4)
    env.step([1, .5])
    assert env.unwrapped.v_cmd == 0
    env.step([1, .5])
    assert env.unwrapped.v_cmd == 0
    env.step([1, .5])
    assert env.unwrapped.v_cmd == pytest.approx(.8)
    env.reset(seed=4)
    env.step([0, 0])
    assert env.unwrapped.v_cmd == 0


def test_schedule_is_observable_and_randomization_reproducible():
    a = make_env(5, .5, randomized=True, end_speed=1.5)
    b = make_env(5, .5, randomized=True, end_speed=1.5)
    assert a.unwrapped.cp == b.unwrapped.cp
    assert a.delay_steps == b.delay_steps
    a.reset(seed=5)
    obs, *_ = a.step([0, 0])
    assert .5 < a.target_speed < 1.5
    assert obs[-1] == pytest.approx(a.target_speed / SPEED_SCALE)


def test_failures_are_included_in_collision_metric():
    rows = [dict(speed=.5, termination_reason='off_track', cone_hits=3, lateral_rmse=.8),
            dict(speed=.5, termination_reason='lap', cone_hits=0, lateral_rmse=.1)]
    result = summary(rows)['0.5']
    assert result['total_cone_hits'] == 3
    assert result['clean_laps'] == result['laps'] == 1
