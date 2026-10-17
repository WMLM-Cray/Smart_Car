"""开放场景探索(强化学习用):车在随机楼层(simulation/fastsim/floorplan)里自由开,没有路、没有终点。

奖励 = 这一拍地面分割新标出来的地面面积(m²,和车上 Logbook 的浅绿层同一个东西);截停 / 撞上扣 CRASH,这局结束。
只算相机地面(~3 m)不算雷达:雷达 10 m,原地转一圈就把整间屋标完了。出发前原地攒 BEV 的 WARM 秒里标的不算。
部署条件同车上:截停、无 YOLO、地面分割按车上的投影 + 补缝(MapFloorSeg)、速度上限 CAP。
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

CAP = 0.7               # 车上网页设的速度上限
WARM = 2.0              # 出发前原地攒 BEV 的时间 s(同 evaluation/real_map_replay)
DURATION = 60.0         # 每局最长 s(不含 WARM)
CRASH = 5.0             # 截停 / 撞上扣的分(= 5 m² 新地面)
SEG_PERIOD, SEG_LAG = 0.33, 0.35


class Explore:
    def __init__(self, world, pl, seed: int, start, duration: float = DURATION, seg=(SEG_PERIOD, SEG_LAG)):
        from training.bev_env import BevEnv
        pl = replace(pl, launch_delay=WARM + 0.5)             # 速度环在 launch_delay 内不动车
        self.b = b = BevEnv(None, pl, seed, start, noise=True, free=True, seg_period=seg[0], seg_lag=seg[1],
                            guard=True, world=world)
        b.env.budget_s = WARM + duration
        self.world = world
        self.marked = np.zeros_like(world.free)
        bev, update = b.bev, b.bev.update_camera

        def update_and_mark(kinds, xy, pose, t):
            update(kinds, xy, pose, t)
            self._mark(xy, pose)

        bev.update_camera = update_and_mark
        while b.env.t < WARM - 1e-9:                          # 原地攒 BEV(动作不影响:车还没放行)
            b.env.step(np.zeros(2))
            b._camera()
        self.area0 = self.area                                # 出发前看到的不算

    def _mark(self, xy, pose):
        if not len(xy):
            return
        x, y, yaw = pose
        c, s = np.cos(yaw), np.sin(yaw)
        w = self.world
        i = np.round((x + c * xy[:, 0] - s * xy[:, 1]) / w.cell).astype(np.int64) - w.i0
        j = np.round((y + s * xy[:, 0] + c * xy[:, 1]) / w.cell).astype(np.int64) - w.j0
        ok = (i >= 0) & (i < self.marked.shape[0]) & (j >= 0) & (j < self.marked.shape[1])
        self.marked[i[ok], j[ok]] = True

    @property
    def area(self) -> float:
        """标出来的地面面积 m²(只算地图上真是地面的格)。"""
        return float((self.marked & self.world.free).sum()) * self.world.cell ** 2

    def observation(self):
        return self.b.observation()

    def step(self, action):
        """一拍(50 ms)。返回 (奖励, 结束原因或 None)。"""
        before = self.area
        why = self.b.step(action)
        r = self.area - before
        if why in ('guard', 'collision'):
            r -= CRASH
        return r, why


def make(seed: int, enc, cfg=None):
    """按种子出一局:随机楼层 + 出发点 + 车辆参数。"""
    from simulation.fastsim.floorplan import PlanConfig, generate, sample_start
    from simulation.fastsim.road_env import Plant
    rng = np.random.default_rng(seed)
    world, info = generate(rng, cfg or PlanConfig())
    start = sample_start(rng, world, info)
    pl = replace(Plant.sample(rng, enc), cap=CAP)
    return Explore(world, pl, seed, start), info
