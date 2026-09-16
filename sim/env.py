"""双边锥桶环形赛道 Gymnasium 环境。

动作 Box(-1,1,(2,)) = [速度, 转角],物理单位:
    速度 = a0 · v_cmd_fwd(前进)或 a0 · v_cmd_rev(倒车),m/s
    转角 = a1 · max_steer,rad,左正
实车 PWM 换算在 pwm.py,只在对接实车时用;info 里附带换算结果便于对照。
观测只来自传感器,不读赛道真值(53 维):
    48 扇区雷达最近距离 / obs_range —— 前 ±90° 每 5° 一个(36),后方每 15° 一个(12)
    + 编码器速度 + IMU 偏航角速度 + 上一步动作(2) + 剩余时间比例
建图(mapping=True)另走完整 1667 点扫描,与观测无关,见 mapping.py。
奖励 = 沿中心线前进米数 - 每步时间 - 撞锥 5(规则:撞锥 +5s)- 出界 10;跑完一圈 +10。

用法:
    from sim.env import ConeTrackEnv
    env = ConeTrackEnv(); obs, info = env.reset(seed=0)
"""
from __future__ import annotations

import math

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from .mapping import OccupancyMap
from .params import CarParams, TrackParams
from .pwm import speed_to_pwm, steer_to_turn
from .sensors import Lidar, encoder, imu
from .track import make_track
from .vehicle import Car

# 扇区边界:后左 6×15° | 前 36×5° | 后右 6×15°
SECTOR_EDGES = np.concatenate([
    np.linspace(-math.pi, -math.pi / 2, 7),
    np.linspace(-math.pi / 2, math.pi / 2, 37)[1:],
    np.linspace(math.pi / 2, math.pi, 7)[1:],
])
SECTORS = len(SECTOR_EDGES) - 1        # 48


class ConeTrackEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, car_params: CarParams = CarParams(), track_params: TrackParams = TrackParams(),
                 control_dt=0.05, physics_dt=0.01, budget=120.0, v_cmd_fwd=None, v_cmd_rev=None,
                 obs_range=10.0, terminate_on_cone=True, noise=True,
                 mapping=False, map_hz=2.0, map_ray_step=4):
        super().__init__()
        self.cp, self.tp = car_params, track_params
        self.control_dt, self.physics_dt, self.budget = control_dt, physics_dt, budget
        self.v_cmd_fwd = car_params.v_cmd_fwd if v_cmd_fwd is None else float(v_cmd_fwd)
        self.v_cmd_rev = car_params.v_cmd_rev if v_cmd_rev is None else float(v_cmd_rev)
        self.obs_range, self.terminate_on_cone, self.noise = obs_range, terminate_on_cone, noise
        self.mapping, self.map_hz, self.map_ray_step = mapping, map_hz, int(map_ray_step)
        self.action_space = spaces.Box(-1.0, 1.0, (2,), dtype=np.float32)
        self.observation_space = spaces.Box(-1.0, 1.0, (SECTORS + 5,), dtype=np.float32)
        self.car = Car(car_params)
        self.lidar = Lidar(car_params)
        # 每条射线落在哪个扇区,只算一次
        self.sector_index = np.clip(
            np.searchsorted(SECTOR_EDGES, self.lidar.angles, side="right") - 1, 0, SECTORS - 1)

    # ---- 动作 -> 物理指令 ----
    def action_to_command(self, action) -> tuple[float, float]:
        a0, a1 = float(np.clip(action[0], -1, 1)), float(np.clip(action[1], -1, 1))
        v = a0 * (self.v_cmd_fwd if a0 >= 0 else self.v_cmd_rev)
        return v, a1 * self.cp.max_steer

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.track = make_track(self.np_random, self.tp)
        start = self.track.centerline[0]
        _, _, heading = self.track.project(start)
        # 后轴放在中心线起点,车头朝切向
        self.car.reset(start[0], start[1], heading)
        self.time = 0.0
        self.progress = 0.0                 # 累计沿中心线前进的米数(可为负)
        self.last_s = self.track.project(start)[0]
        self.cone_hits = 0
        self.hit_ids: set[int] = set()
        self.last_action = np.zeros(2, np.float32)
        self.next_scan = 0.0
        self.v_cmd, self.steer_cmd = 0.0, 0.0
        self.map = OccupancyMap() if self.mapping else None
        self.next_map = 0.0
        self.cone_coverage = 0.0
        self._sense()
        return self._observation(), self._info(None)

    def _rng(self):
        return self.np_random if self.noise else None

    def _sense(self):
        self.ranges = self.lidar.scan(self.car, self.track.cones, self.tp, self._rng())
        self.imu = imu(self.car, self.cp, self._rng())
        self.encoder_speed = encoder(self.car, self.cp, self._rng())
        if self.map is not None and self.time >= self.next_map - 1e-9:
            self.next_map += 1.0 / self.map_hz
            c, s = math.cos(self.car.yaw), math.sin(self.car.yaw)
            origin = (self.car.x + self.cp.lidar_x * c, self.car.y + self.cp.lidar_x * s)
            step = self.map_ray_step
            self.map.integrate(origin, self.lidar.angles[::step] + self.car.yaw,
                               self.ranges[::step], self.obs_range)

    def _observation(self):
        r = np.minimum(self.ranges, self.obs_range)
        sectors = np.full(SECTORS, self.obs_range)
        np.minimum.at(sectors, self.sector_index, r)
        obs = np.concatenate([
            sectors / self.obs_range,
            [self.encoder_speed / self.v_cmd_fwd,
             self.imu["angular_velocity_z"] / 3.0],
            self.last_action,
            [max(0.0, 1.0 - self.time / self.budget)],
        ])
        return np.clip(obs, -1, 1).astype(np.float32)

    def step(self, action):
        action = np.clip(np.asarray(action, np.float32), -1, 1)
        self.v_cmd, self.steer_cmd = self.action_to_command(action)
        new_hits = 0
        steps = max(1, round(self.control_dt / self.physics_dt))
        for _ in range(steps):
            self.car.step(self.v_cmd, self.steer_cmd, self.physics_dt)
            self.time += self.physics_dt
            hits = np.flatnonzero(self.car.footprint_hits_circles(self.track.cones, self.tp.cone_base_radius))
            fresh = set(hits.tolist()) - self.hit_ids
            new_hits += len(fresh)
            self.hit_ids |= fresh
            if self.time >= self.next_scan - 1e-9:        # 雷达按 12Hz 刷新
                self.next_scan += 1.0 / self.cp.lidar_hz
                self._sense()
        self.cone_hits += new_hits
        self.last_action = action

        s, lateral, _ = self.track.project((self.car.x, self.car.y))
        ds = s - self.last_s
        length = self.track.length
        if ds < -length / 2:
            ds += length                                  # 跨过起点
        elif ds > length / 2:
            ds -= length
        self.last_s = s
        self.progress += ds

        reward = ds - 0.01 - 5.0 * new_hits
        reason = None
        if abs(lateral) > self.tp.width / 2 + 0.5:
            reward -= 10.0; reason = "off_track"
        elif new_hits and self.terminate_on_cone:
            reason = "cone"
        elif self.progress >= length:
            reward += 10.0; reason = "lap"
        terminated = reason is not None
        truncated = not terminated and self.time >= self.budget - 1e-9
        if truncated:
            reason = "timeout"
        if (terminated or truncated) and self.map is not None:
            self.cone_coverage = self.map.cone_coverage(self.track.cones, self.tp.cone_base_radius)
        return self._observation(), float(reward), terminated, truncated, self._info(reason, lateral)

    def _info(self, reason, lateral=0.0):
        return dict(time=self.time, progress=self.progress, lap_fraction=self.progress / self.track.length,
                    lateral=lateral, cone_hits=self.cone_hits, speed=self.car.v,
                    v_cmd=self.v_cmd, steer_cmd=self.steer_cmd,
                    cone_coverage=self.cone_coverage,
                    mapped_free_area=self.map.free_area() if (self.map is not None and reason) else 0.0,
                    # 只作对照:实车发这两个数(见 pwm.py),仿真内部不用
                    pwm=speed_to_pwm(self.v_cmd, self.cp), turn=steer_to_turn(self.steer_cmd, self.cp),
                    termination_reason=reason)
