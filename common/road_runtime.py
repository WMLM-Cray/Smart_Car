"""Pure NumPy runtime adapter for the road_v2 simulation policy."""
from __future__ import annotations

import numpy as np

from common.control import to_cmd_vel
from common.params import CarParams
from common.pwm import action_to_steer
from common.road_policy import DEFAULT_ENC, RoadPolicy, action_to_speed, observe


STOP = 'stop'


class RoadRuntime:
    """Sensor freshness, training-contract observations, and physical sim commands."""

    def __init__(self, policy, speed_cap=1.0, sensor_timeout=0.3, enable_timeout=0.3,
                 params=None):
        self.policy = policy
        # 速度契约跟着模型走:RoadPolicy 从 schema 反解(见 common/road_policy.py)。
        # 假策略(测试用)没有 enc,退回默认区间。
        self.enc = getattr(policy, 'enc', DEFAULT_ENC)
        self.speed_cap = float(speed_cap)
        self.sensor_timeout = float(sensor_timeout)
        self.enable_timeout = float(enable_timeout)
        self.params = params or CarParams()
        self.reset()

    def reset(self):
        self.scan = self.angles = None
        self.scan_t = self.odom_t = self.imu_t = None
        self.speed = self.yaw_rate = 0.0
        self.last_action = np.zeros(2, dtype=np.float32)
        self.enabled = False
        self.enable_t = None
        self.policy.reset()

    def on_scan(self, ranges, angle_min, angle_increment, now):
        self.scan = np.asarray(ranges, dtype=np.float32).copy()
        self.angles = float(angle_min) + np.arange(len(self.scan)) * float(angle_increment)
        self.scan_t = float(now)

    def on_odometry(self, speed, now):
        self.speed = float(speed)
        self.odom_t = float(now)

    def on_imu(self, yaw_rate, now):
        self.yaw_rate = float(yaw_rate)
        self.imu_t = float(now)

    def set_enabled(self, enabled, now):
        """Update enable heartbeat; return True when an immediate neutral is needed."""
        now = float(now)
        if not enabled:
            changed = self.enabled
            self.enabled, self.enable_t = False, None
            self._reset_history()
            return changed
        if not self.enabled:
            self._reset_history()
        self.enabled, self.enable_t = True, now
        return False

    def _reset_history(self):
        self.policy.reset()
        self.last_action[:] = 0.0

    def _fresh(self, stamp, now):
        return stamp is not None and 0.0 <= now - stamp <= self.sensor_timeout

    def stale_sensors(self, now):
        """Names of sensors that are missing or older than sensor_timeout."""
        now = float(now)
        return [name for name, t in (('scan', self.scan_t if self.scan is not None else None),
                                     ('odom', self.odom_t), ('imu', self.imu_t))
                if not self._fresh(t, now)]

    def decide(self, now):
        """One control step shared by the sim and car nodes.

        None = disabled, send nothing; STOP = heartbeat or a sensor went stale, send neutral;
        otherwise (steer rad, target speed m/s).
        """
        now = float(now)
        if not self.enabled:
            return None
        if self.enable_t is None or not 0.0 <= now - self.enable_t <= self.enable_timeout:
            self.enabled, self.enable_t = False, None
            self._reset_history()
            return STOP
        if self.stale_sensors(now):
            self._reset_history()
            return STOP

        action = np.clip(np.asarray(self._act(now), dtype=np.float32).reshape(2), -1.0, 1.0)
        self.last_action[:] = action
        steer = action_to_steer(float(action[0]), self.params)
        target_speed = min(action_to_speed(float(action[1]), self.enc), self.speed_cap)
        return steer, target_speed

    def _act(self, now):
        """模型这一拍的原始动作;BevRuntime 换成 BEV 输入。"""
        obs = observe(self.scan, self.angles, self.speed, self.speed_cap,
                      self.yaw_rate, self.last_action, scale=self.enc.scale)
        return self.policy(obs)

    def step(self, now):
        """Simulation output: (linear m/s, yaw rad/s); None means disabled and silent."""
        d = self.decide(now)
        if d is None or d is STOP:
            return None if d is None else (0.0, 0.0)
        steer, target_speed = d
        return to_cmd_vel(target_speed, steer, self.params)
