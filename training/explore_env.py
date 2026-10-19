"""开放场景探索(强化学习用):车在随机楼层(simulation/fastsim/floorplan)里自由开,没有路、没有终点。

奖励 = 这一拍地面分割新标出来的地面面积(m²,和车上 Logbook 的浅绿层同一个东西);截停 / 撞上 / 停滞扣 CRASH,这局结束。
只算相机地面(~3 m)不算雷达:雷达 10 m,原地转一圈就把整间屋标完了。
放行时 BEV 是空的、车马上就走(同车上:重新使能清空 BEV,common/bev_runtime._reset_history;起步等待见 Plant.sample)。
2026-10-18 之前是出发前原地攒 2 s BEV 再走,车上没有这 2 s。
部署条件同车上:截停、无 YOLO、地面分割按车上的投影 + 补缝(MapFloorSeg)。速度上限:房间 CAP(车上开放场景的限速),
路段关卡按种子在 ROAD_CAP 里抽 —— 0.7 下老师从不刹车,RL 碰不到入弯要减速的场合(docs/v4.5方案.md §5.2)。
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from simulation.fastsim.road_env import CONTROL_DT

CAP = 0.7               # 车上网页设的速度上限
DURATION = 60.0         # 每局最长 s
CRASH = 5.0             # 截停 / 撞上扣的分(= 5 m² 新地面)
PROGRESS = 1.0          # 路段关卡:沿路每前进 1 m 的奖励(0.7 m/s 开 40 s ≈ 28,和房间一局的面积分同一量级)
OFF_ROAD = 20.0         # 路段关卡出界扣的分:按面积给分时出界只扣 5,第二轮 iter 149 测试集出界 27 -> 81 次(2026-10-18)
STALL_V, STALL_S = 0.05, 3.0   # 车速低于 STALL_V 持续 STALL_S 秒 = 停滞,和截停一样结束、扣 CRASH(主动停车不是目标,
                                # 紧急停车归 guard;不罚的话 RL 可能学会停着不动躲截停扣分,2026-10-19 用户定 3 s)
ROAD_CAP = (0.7, 2.0)           # 路段关卡的速度上限抽取范围
SEG_PERIOD, SEG_LAG = 0.33, 0.35


class _RoadGrid:
    """路段关卡没有 RealMap:按路的范围开一张 5 cm 标记网格,地面分割只出路面上的点,全算地面。"""
    def __init__(self, road, cell=0.05, pad=4.0):
        pts = np.vstack([road.centre, road.walls.reshape(-1, 2)]) if len(road.walls) else road.centre
        lo, hi = pts.min(0) - pad, pts.max(0) + pad
        self.cell = cell
        self.i0, self.j0 = int(np.floor(lo[0] / cell)), int(np.floor(lo[1] / cell))
        self.free = np.ones((int((hi[0] - lo[0]) / cell) + 2, int((hi[1] - lo[1]) / cell) + 2), bool)


class Explore:
    def __init__(self, world, pl, seed: int, start, duration: float = DURATION, seg=(SEG_PERIOD, SEG_LAG), road=None):
        """world:RealMap 场地(楼层 / 房间);或者 road:路段关卡(training 的 LW3 等,防止忘掉沿路开,
        Wołczyk 2024 —— 只在楼层里练,第二轮 89 轮后原来的无锥桶测试集 81% -> 76%),start 用路的约定 (i, 横移, 朝向)。"""
        from training.bev_env import BevEnv
        if road is None:
            self.b = b = BevEnv(None, pl, seed, start, noise=True, free=True, seg_period=seg[0], seg_lag=seg[1],
                                guard=True, world=world)
        else:
            self.b = b = BevEnv(road, pl, seed, start, noise=True, free=True, seg_period=seg[0], seg_lag=seg[1],
                                yolo=False, guard=True, seg_lut=True, seg_fill=True)
        b.env.budget_s = duration
        self.world = world
        self.grid = world if road is None else _RoadGrid(road)
        self.marked = np.zeros_like(self.grid.free)
        bev, update = b.bev, b.bev.update_camera

        def update_and_mark(kinds, xy, pose, t):
            update(kinds, xy, pose, t)
            self._mark(xy, pose)

        bev.update_camera = update_and_mark
        self.area0 = self.area                                # 放行时还没标过(0);留着给"新标面积"做差
        self.road = road
        self.s = road.s[b.env.idx] if road is not None else 0.0   # 路段关卡:沿中心线走到哪了 m
        self.dist = 0.0                                       # 放行后开过的路程 m(RoadEnv 没有 dist,自己记)
        self.still = 0.0                                      # 连续低于 STALL_V 的时间 s

    def _mark(self, xy, pose):
        if not len(xy):
            return
        x, y, yaw = pose
        c, s = np.cos(yaw), np.sin(yaw)
        w = self.grid
        i = np.round((x + c * xy[:, 0] - s * xy[:, 1]) / w.cell).astype(np.int64) - w.i0
        j = np.round((y + s * xy[:, 0] + c * xy[:, 1]) / w.cell).astype(np.int64) - w.j0
        ok = (i >= 0) & (i < self.marked.shape[0]) & (j >= 0) & (j < self.marked.shape[1])
        self.marked[i[ok], j[ok]] = True

    @property
    def area(self) -> float:
        """标出来的地面面积 m²(只算地图上真是地面的格)。"""
        return float((self.marked & self.grid.free).sum()) * self.grid.cell ** 2

    def observation(self):
        return self.b.observation()

    def privileged(self) -> np.ndarray:
        """只给价值网的仿真内信息(非对称 actor-critic,Pinto 2018;车上不用):周围 2 m / 5 m 内还没标过的地面
        (奖励就是它,BEV 看不全)、离最近障碍多远、这局还剩多少时间(到点截断时价值和剩余时间有关,Pardo 2018)。"""
        if self.world is None:                                # 路段关卡没有地图,价值网的额外输入给 0
            return np.zeros(4, np.float32)
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
        """一拍(50 ms)。返回 (奖励, 结束原因或 None)。
        房间:新标的地面面积;路段:沿路前进的距离 × PROGRESS(路边敞开的地面也是地面,按面积给分等于奖励开出路,
        第二轮遗忘的几乎全是出界)。截停 / 撞上 / 停滞扣 CRASH,路段出界扣 OFF_ROAD。"""
        before, (x0, y0, _) = self.area, self.b.pose
        why = self.b.step(action)
        e = self.b.env
        self.still = self.still + CONTROL_DT if e.car.v < STALL_V else 0.0
        if why is None and self.still >= STALL_S:
            why = 'stall'
        x1, y1, _ = self.b.pose
        self.dist += float(np.hypot(x1 - x0, y1 - y0))
        if self.road is None:
            r = self.area - before
        else:
            s = self.road.s[self.b.env.idx]
            r, self.s = PROGRESS * (s - self.s), s
        if why == 'off_track' and self.road is not None:
            r -= OFF_ROAD
        elif why in ('guard', 'collision', 'off_track', 'stall'):
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


CRAMPED = '_c'          # 关卡名后缀:局促起步(车头 0.7~1.5 m 就是障碍,离障碍最近 0.4 m;floorplan.sample_start 的 facing)
CRAMPED_FACING, CRAMPED_CLEAR = (0.7, 1.5), 0.4


def room_level(name: str):
    """房间类关卡名 -> (PlanConfig, 是否局促起步);路段关卡返回 None。"""
    lv = levels()
    if name in lv:
        return lv[name], False
    if name.endswith(CRAMPED) and name[:-len(CRAMPED)] in lv:
        return lv[name[:-len(CRAMPED)]], True
    return None


def make(seed: int, enc, level: str = 'building'):
    """按种子出一局:随机场地(levels())+ 出发点 + 车辆参数。level 也可以是路段关卡名(simulation/fastsim/road.LEVELS,
    比如 LW3 / LW4 / LWO3):用 road_check.sample_episode 抽一条老师开得过的路,奖励是沿路前进的距离(见 Explore.step)。
    房间关卡名加 _c(比如 room_n_c)= 同样的房间、局促起步(见 room_level)。"""
    from simulation.fastsim.floorplan import generate, sample_start
    from simulation.fastsim.road_env import Plant
    room = room_level(level)
    if room is None:
        from training.road_check import sample_episode
        road, pl, start, _, _ = sample_episode(seed, level, enc=enc, cap_min=ROAD_CAP[0], cap_max=ROAD_CAP[1])
        return Explore(None, pl, seed, start, road=road), None     # 限速在抽局时就定,老师可通检查用的是同一个限速
    rng = np.random.default_rng(seed)
    for _ in range(20):                 # 小房间摆满家具时可能找不到"能绕圈"的出发点,换一张图
        world, info = generate(rng, room[0])
        try:
            start = (sample_start(rng, world, info, clear=CRAMPED_CLEAR, facing=CRAMPED_FACING) if room[1]
                     else sample_start(rng, world, info))
            break
        except RuntimeError:
            continue
    else:
        raise RuntimeError(f'{level} 种子 {seed}:20 张图都找不到出发点')
    pl = replace(Plant.sample(rng, enc), cap=CAP)
    return Explore(world, pl, seed, start), info
