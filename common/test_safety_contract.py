"""Differential parity between DeploymentGuardContract and the deployment tick.

The reference implementation is this repository's own
``vehicle.road_car_node.RoadCarController`` (ROS-free controller, with ``loop``
and ``guard`` injected below). Nothing outside the repository is read, so a
clean checkout can run this file.
"""
from types import SimpleNamespace

import numpy as np
import pytest

from common.params import CarParams
from common.pwm import action_to_steer
from common.safety_contract import (
    GUARD_STOP,
    STUCK_SECONDS,
    DeploymentGuardContract,
)
from vehicle import road_car_node
from vehicle.road_car_node import RoadCarController

STEER_ACTION = 0.2
SCAN = dict(ranges=[10.0] * 64, angle_min=-1.5, angle_increment=3.0 / 64)


class FakePolicy:
    def reset(self):
        pass

    def __call__(self, obs):
        return np.array([STEER_ACTION, 0.4], dtype=np.float32)


class FakeLoop:
    """Encoder loop stand-in: constant PWM, counts resets."""

    def __init__(self, pwm=1650.0):
        self.pwm = float(pwm)
        self.resets = 0

    def reset(self):
        self.resets += 1

    def update(self, target, speed, dt):
        return self.pwm


class FakeGuard:
    """Scan guard stand-in: the gap is whatever the test sets."""

    def __init__(self):
        self.gap_m = None

    def update(self, msg):
        pass

    def gap(self, steer=0.0, now=None):
        return self.gap_m


def rig(cap=1.5):
    params = CarParams()
    controller = RoadCarController(FakePolicy(), cap, params=params,
                                   loop=FakeLoop(), guard=FakeGuard())
    contract = DeploymentGuardContract(params=params)
    steer = action_to_steer(STEER_ACTION, params)
    return controller, contract, params, steer


def refresh(controller, now):
    """Feed one fresh sensor set + enable heartbeat at ``now``."""
    controller.on_enable(True, now)
    controller.on_encoder(0.5, now)
    controller.on_imu(0.0, now)
    controller.on_scan(SimpleNamespace(**SCAN), now)


def guard_event(controller):
    """部署 event 里只取截停那部分;'avoid' / 'escape' 是截停之前的转向事件,契约不管。"""
    return controller.event if controller.event in ('guard', 'stuck') else None


def run_both(sequence, cap=1.5):
    """Drive deployment and contract with the same (now, gap) sequence."""
    controller, contract, params, steer = rig(cap)
    decisions, commands = [], []
    for now, gap in sequence:
        refresh(controller, now)
        controller.guard.gap_m = gap
        command = controller.tick(now)
        decision = contract.step(now, gap, controller.steer_cmd, controller.rt.speed)   # 避障在截停之前,契约吃实打转角
        assert command is not None, 'controller must publish while enabled and fresh'
        assert command[1] == decision.steer, 'steering is published on every tick'
        assert decision.event == guard_event(controller)
        assert contract.block_t == controller.block_t
        assert contract.gap == controller.gap
        if guard_event(controller) is None:
            assert decision.throttle is None, 'throttle on a free tick belongs to SpeedLoop'
        else:
            assert command[0] == params.pwm_mid == decision.throttle
        decisions.append(decision)
        commands.append(command)
    return controller, contract, params, decisions, commands


@pytest.mark.parametrize('sequence', [
    # 放行 → 刚好等于阈值(不截停) → 低于阈值 → 恢复
    [(0.0, 1.00), (0.1, 0.30), (0.2, 0.25), (0.3, 0.24), (0.4, None), (5.0, 0.20)],
    # 截停计时跨过 STUCK_SECONDS（部署用 > 严格大于）
    [(0.0, 0.20), (2.999, 0.20), (3.0, 0.20), (3.001, 0.20), (6.0, 0.20), (6.5, 1.0)],
    # 截停中途净空变化不重置计时；恢复后重新计时
    [(0.0, 0.24), (0.5, 0.05), (1.0, 0.249), (4.0, 0.20), (4.1, 0.30), (4.2, 0.20)],
    # inf / nan / None 都不截停（部署就是一次普通 <）
    [(0.0, float('inf')), (0.1, float('nan')), (0.2, None), (0.3, 0.2)],
])
def test_contract_matches_deployment_tick(sequence):
    controller, contract, _params, decisions, commands = run_both(sequence)
    assert decisions[-1].event == guard_event(controller)
    assert commands  # every tick published something


def test_constants_match_the_deployment_module():
    assert GUARD_STOP == road_car_node.GUARD_STOP == 0.25
    assert STUCK_SECONDS == road_car_node.STUCK_SECONDS == 3.0
    assert DeploymentGuardContract().threshold == road_car_node.GUARD_STOP
    assert DeploymentGuardContract().stuck_seconds == road_car_node.STUCK_SECONDS


def test_blocked_ticks_never_command_reverse():
    """The deployment holds at pwm_mid; nothing may go below neutral."""
    sequence = [(t / 10.0, 0.20) for t in range(0, 61)]
    _c, _k, params, decisions, commands = run_both(sequence)
    blocked = [d for d in decisions if d.event]
    assert len(blocked) == len(sequence)
    assert {d.throttle for d in blocked} == {params.pwm_mid}
    assert all(c[0] >= params.pwm_mid for c in commands), 'no reverse in the guard'
    assert {d.event for d in blocked} == {'guard', 'stuck'}
    assert blocked[-1].blocked_s == pytest.approx(6.0)


def test_release_and_reenable_clear_the_block_timer():
    controller, contract, params, steer = rig()
    refresh(controller, 0.0)
    controller.guard.gap_m = 0.2
    controller.tick(0.0)
    decision = contract.step(0.0, 0.2, steer)
    assert controller.block_t == contract.block_t == 0.0
    assert decision.event == 'guard'

    # 松手 → 重新使能：部署在 False→True 边沿清 block_t（event 保留上次的值，下一拍才改写）
    assert controller.on_enable(False, 0.1) is True
    controller.on_enable(True, 1.0)
    contract.release()
    assert controller.block_t is None and contract.block_t is None
    assert controller.event == contract.event == 'guard', 'release only drops the timer'

    refresh(controller, 2.0)
    controller.guard.gap_m = 0.2
    controller.tick(2.0)
    decision = contract.step(2.0, 0.2, steer)
    assert controller.block_t == contract.block_t == 2.0, 'timer restarts after re-enable'
    assert decision.event == controller.event == 'guard'
