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

    def privileged(self) -> np.ndarray:
        """只给价值网的仿真内信息(非对称 actor-critic,Pinto 2018;车上不用):周围 2 m / 5 m 内还没标过的地面
        (奖励就是它,BEV 看不全)、离最近障碍多远、这局还剩多少时间(到点截断时价值和剩余时间有关,Pardo 2018)。"""
        w, car, e = self.world, self.b.env.car, self.b.env
        i, j = int(round(car.x / w.cell)) - w.i0, int(round(car.y / w.cell)) - w.j0
        out = []
        for r_m in (2.0, 5.0):
            r = int(r_m / w.cell)
            i0, i1, j0, j1 = max(0, i - r), i + r + 1, max(0, j - r), j + r + 1
            left = (w.free[i0:i1, j0:j1] & ~self.marked[i0:i1, j0:j1]).sum() * w.cell ** 2
            out.append(left / (np.pi * r_m ** 2))
        edt = w._edt()
        clear = edt[i, j] if 0 <= i < edt.shape[0] and 0 <= j < edt.shape[1] else 0.0
        out.append(min(float(clear), 3.0) / 3.0)
        out.append(max(0.0, e.budget_s - e.t) / 60.0)
        return np.asarray(out, np.float32)

    def step(self, action):
        """一拍(50 ms)。返回 (奖励, 结束原因或 None)。"""
        before = self.area
        why = self.b.step(action)
        r = self.area - before
        if why in ('guard', 'collision'):
            r -= CRASH
        return r, why


def levels():
    """课程:空的长方形房间 -> 墙面有浅槽 / 凸起 / 切角的房间 -> 再摆少量家具 -> 整栋楼。
    room_f 的家具按现实里一间屋的量:最多一张桌子配 1~4 把椅子、0~2 把靠墙椅子、0~2 个箱子
    (第一版 1~2 组桌椅、每组 2~6 把,用户说现实里找不出这么多椅子,2026-10-18)。"""
    from simulation.fastsim.floorplan import PlanConfig
    none = dict(chair_sets=(0, 1), wall_chairs=(0, 1), boxes=(0, 1), pillars=(0, 1))
    shape = dict(features=(2, 7), corner_cut=0.3)
    return {'room': PlanConfig(kind='room', **none),
            'room_n': PlanConfig(kind='room', **none, **shape),
            'room_f': PlanConfig(kind='room', **shape, chair_sets=(0, 2), chairs_per_set=(1, 5),
                                 wall_chairs=(0, 3), boxes=(0, 3)),
            'building': PlanConfig()}


def make(seed: int, enc, level: str = 'building'):
    """按种子出一局:随机场地(levels())+ 出发点 + 车辆参数。"""
    from simulation.fastsim.floorplan import generate, sample_start
    from simulation.fastsim.road_env import Plant
    rng = np.random.default_rng(seed)
    for _ in range(20):                 # 小房间摆满家具时可能找不到"能绕圈"的出发点,换一张图
        world, info = generate(rng, levels()[level])
        try:
            start = sample_start(rng, world, info)
            break
        except RuntimeError:
            continue
    else:
        raise RuntimeError(f'{level} 种子 {seed}:20 张图都找不到出发点')
    pl = replace(Plant.sample(rng, enc), cap=CAP)
    return Explore(world, pl, seed, start), info
