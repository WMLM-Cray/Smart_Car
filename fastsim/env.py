"""双边锥桶环形赛道 Gymnasium 环境。

动作 Box(-2,2,(2,)) = [速度, 转角],进物理前先 clip 到 ±1:
    速度 = a0 · v_cmd_fwd(前进)或 a0 · v_cmd_rev(倒车),m/s
    转角 = 实测舵机曲线(turn_mid + a1·turn_span),rad,左正,见 pwm.action_to_steer
空间开到 ±2 是给 clip 高斯留活动余量,超出 ±1 的部分在奖励里二次罚,见 ACTION_LIMIT。
实车 PWM 换算在 pwm.py,只在对接实车时用;info 里附带换算结果便于对照。
观测只来自传感器,不读赛道真值(53 维):
    48 扇区雷达最近距离 / obs_range —— 前 ±90° 每 5° 一个(36),后方每 15° 一个(12)
    + 编码器速度 + IMU 偏航角速度 + 上一步动作(2) + 剩余时间比例
建图(mapping=True)另走完整 1667 点扫描,与观测无关,见 mapping.py。
奖励 = 沿中心线前进米数 - 每步 0.02 - 撞锥 1.5(不终止)- 贴边二次罚 - 越界动作罚 - 出界 5;一圈 +10。
出界罚压到 5:进度是 +1/米,罚 10 等于要白跑 10m 才回本,摔一次太贵会逼出"不动"这个解。

用法:
    from fastsim.env import ConeTrackEnv
    env = ConeTrackEnv(); obs, info = env.reset(seed=0)
"""
from __future__ import annotations

import math

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from .mapping import OccupancyMap
from .obs import SECTOR_EDGES, SECTORS, build_obs, make_sector_edges, sector_index, sectorize
from .params import CarParams, RuleTrackParams, TrackParams
from .pwm import action_to_steer, speed_to_pwm, steer_to_turn
from .sensors import Lidar, encoder, imu
from .track import make_track
from .track_rules import make_rule_track
from .vehicle import Car

# 动作空间开到 ±2,超出 ±1 的部分照样 clip 到 ±1 才进物理,但要在奖励里罚。
# 原因(2026-09-18 实测):动作空间就是 ±1 时,SB3 的高斯策略均值会一路漂到 +3、
# std 0.3,采样到 a0<1 的概率约 1e-11 —— clip 之外没有梯度把它拉回来。结果 s1~s3
# 四个阶段一律 96%+ 满油门,"减速"这个动作根本试不出来,s3a 的限速课程也就失效了。
# 留出 clip 外的活动空间 + 二次罚,均值会停在 1 附近,低油门始终可采样。
ACTION_LIMIT = 2.0
OVERSHOOT_COST = 0.05          # 每步 · (|a|-1)² 的惩罚系数


class ConeTrackEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, car_params: CarParams = CarParams(), track_params: TrackParams = TrackParams(),
                 control_dt=0.05, physics_dt=0.01, budget=120.0, v_cmd_fwd=None, v_cmd_rev=None,
                 obs_range=10.0, terminate_on_cone=False, noise=True,
                 mapping=False, map_hz=2.0, map_ray_step=4, front_sectors=36):
        super().__init__()
        self.sector_edges = make_sector_edges(front_sectors)
        self.n_sectors = len(self.sector_edges) - 1
        self.cp, self.tp = car_params, track_params
        self.control_dt, self.physics_dt, self.budget = control_dt, physics_dt, budget
        self.v_cmd_fwd = car_params.v_cmd_fwd if v_cmd_fwd is None else float(v_cmd_fwd)
        self.v_cmd_rev = car_params.v_cmd_rev if v_cmd_rev is None else float(v_cmd_rev)
        self.obs_range, self.terminate_on_cone, self.noise = obs_range, terminate_on_cone, noise
        self.mapping, self.map_hz, self.map_ray_step = mapping, map_hz, int(map_ray_step)
        self.action_space = spaces.Box(-ACTION_LIMIT, ACTION_LIMIT, (2,), dtype=np.float32)
        self.observation_space = spaces.Box(-1.0, 1.0, (self.n_sectors + 5,), dtype=np.float32)
        self.car = Car(car_params)
        self.lidar = Lidar(car_params)
        self.sector_index = sector_index(self.lidar.angles, self.sector_edges)   # 只算一次

    # ---- 动作 -> 物理指令 ----
    def action_to_command(self, action) -> tuple[float, float]:
        a0, a1 = float(np.clip(action[0], -1, 1)), float(np.clip(action[1], -1, 1))
        v = a0 * (self.v_cmd_fwd if a0 >= 0 else self.v_cmd_rev)
        return v, action_to_steer(a1, self.cp)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.track = (make_rule_track(self.np_random, self.tp)
                      if isinstance(self.tp, RuleTrackParams) else make_track(self.np_random, self.tp))
        # 锥桶 + 挡板采样点:碰撞和建图覆盖率都按这一组算(挡板不是点目标,按 10cm 采样)
        self.obstacles = np.vstack([self.track.cones, self.track.board_points])
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
        self.ranges = self.lidar.scan(self.car, self.track, self._rng())
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
        return build_obs(
            sectorize(self.ranges, self.sector_index, self.obs_range, self.n_sectors),
            self.encoder_speed, self.imu["angular_velocity_z"], self.last_action,
            max(0.0, 1.0 - self.time / self.budget), self.v_cmd_fwd)

    def step(self, action):
        raw = np.clip(np.asarray(action, np.float32), -ACTION_LIMIT, ACTION_LIMIT)
        overshoot = float(np.sum(np.maximum(np.abs(raw) - 1.0, 0.0) ** 2))
        action = np.clip(raw, -1, 1)
        self.v_cmd, self.steer_cmd = self.action_to_command(action)
        new_hits = 0
        steps = max(1, round(self.control_dt / self.physics_dt))
        for _ in range(steps):
            self.car.step(self.v_cmd, self.steer_cmd, self.physics_dt)
            self.time += self.physics_dt
            hits = np.flatnonzero(self.car.footprint_hits_circles(self.obstacles, self.tp.cone_base_radius))
            fresh = set(hits.tolist()) - self.hit_ids
            new_hits += len(fresh)
            self.hit_ids |= fresh
            if self.time >= self.next_scan - 1e-9:        # 雷达按 12Hz 刷新
                self.next_scan += 1.0 / self.cp.lidar_hz
                self._sense()
        self.cone_hits += new_hits
        self.last_action = action

        s, lateral, _ = self.track.project((self.car.x, self.car.y), near_s=self.last_s)
        ds = s - self.last_s
        length = self.track.length
        if ds < -length / 2:
            ds += length                                  # 跨过起点
        elif ds > length / 2:
            ds -= length
        self.last_s = s
        self.progress += ds

        # 撞锥不再终止:原来撞一下就结束,训练数据被截断在赛道前 1/3(s3 平均一局 10.8s
        # ≈ 16m),S 弯段策略压根没见过。规则是撞锥 +5s,按时间价值只值 -1 左右。
        # 时间罚必须留着(2026-09-18 试过去掉,20/20 全部超时):ds 是按米发的,不按秒,
        # 没有步罚的话原地不动是严格最优 —— 实测策略把速度压到 0.05m/s 磨满 120s。
        # 加倍到 0.02 是给速度选择留出压力:1.5m/s 跑完一圈 -10.8,0.75m/s 要 -21.6。
        # 横向罚是新加的稠密项:出界前唯一的减速信号原来只有终止的 -5,中间一片没有梯度。
        # 二次形式 —— 中线附近可忽略,贴到边上才显著;系数压到 0.03,满速时 ds 仍然打得过它。
        lane = 0.03 * (lateral / (self.tp.width / 2)) ** 2
        reward = ds - 0.02 - 1.5 * new_hits - lane - OVERSHOOT_COST * overshoot
        reason = None
        if abs(lateral) > self.tp.width / 2 + 0.5:
            reward -= 5.0; reason = "off_track"
        elif new_hits and self.terminate_on_cone:
            reason = "cone"
        elif self.progress >= length:
            reward += 10.0; reason = "lap"
        terminated = reason is not None
        truncated = not terminated and self.time >= self.budget - 1e-9
        if truncated:
            reason = "timeout"
        if (terminated or truncated) and self.map is not None:
            self.cone_coverage = self.map.cone_coverage(self.obstacles, self.tp.cone_base_radius)
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
