"""bev_v1 的训练/评估环境:RoadEnv + BEV 累积 + 假 YOLO,以及"只用已扫到信息"的老师。

位姿:用 FastSim 的真位姿(车上用雷达里程计,用户实测漂移很小,先不建漂移模型)。
雷达:每次 RoadEnv 真扫描时,按**扫描那一刻**的位姿登记进 BEV。
摄像头:每 yolo_period 秒检测一次(香橙派上 YOLO 帧率待实测,先按 10 Hz [假设])。
地面分割(bev_v2,free=True):每 seg_period 秒拍一次,seg_lag 秒后(推理延迟)按拍摄那一刻的位姿登记进 FREE 层
(FakeFloorSeg;和车上 BevRuntime.on_free 一样)。默认 0.1 s、无延迟是训练时的设定;实车 M2F-T 288 是 ~0.33 s / ~0.35 s。

老师(SeenTeacher):特权车辆参数(和 road_train 一样);重规划窗口只到"左右边界都被雷达扫到过"
的地方,窗口没到路终点时末端限速 sqrt(a_lat·R_MIN)。它**不是**严格只用学生 BEV 里的信息:
  - 已知前方不足 MIN_WIN 时仍按真值道路规划 MIN_WIN(plan() 至少要 3 个粗网格点)
  - 初始计划是全路计划,重规划失败时沿用旧计划(开局连续失败就会用上它)
  - "扫到过"不过期,学生 BEV 20 s 后会忘
2026-10-09 试过严格版(去掉以上三条,未知就最低速直行):老师自己开成功率 99.4% -> 95.4%
(0.45 m/s²/限速 1.8)、98.4% -> 95.3%(1.47/2.5),L6h/L6f 掉 5~9 个点,所以保留现在的特权。
"""
from __future__ import annotations

import math

import numpy as np
from scipy.spatial import cKDTree

from common.bev import Bev, BevConfig, CellType
from common.bev_policy import bev_image, bev_images, scalars
from simulation.fastsim.camera import FakeFloorSeg, FakeYolo
from simulation.fastsim.road import DS, EDGE
from simulation.fastsim.road_env import CONTROL_DT, RoadEnv
import training.road_teacher as RT
from training.road_teacher import BRAKE_MARGIN, HORIZON, R_MIN, R_REPLAN, SPEED_MARGIN, Teacher, plan

GUARD_MARGIN = 0.1             # SeenTeacher(guard_aware) 净空额外留的余量 m

SEEN_R = 0.8            # 边界点离雷达回波这么近算"看到过"(桶距最大 1.5 m)
FLOOR_SEEN_R = math.sqrt(2) * BevConfig().res  # 地面只容许一格量化误差,不能沿用稀疏锥桶的 0.8 m
MIN_WIN = 0.5           # 重规划窗口至少看这么远 m
YOLO_PERIOD = 0.1       # [假设] 摄像头检测周期 s
SEG_PERIOD = 0.1        # [假设] 地面分割周期 s(NPU 上 SegFormer-B0 待实测)


class BevEnv:
    """包一层 RoadEnv:每拍维护 BEV,observation() 给 (图, 标量),near=True 时追加第三项近场图。"""

    def __init__(self, road, pl, seed, start=(0, 0.0, 0.0), noise=True, robust=False,
                 yolo_period=YOLO_PERIOD, free=False, seg_period=SEG_PERIOD, seg_lag=0.0, aug=(0.0, 0.0),
                 yolo=True, guard=False, seg_lut=False, seg_fill=False, world=None, near=False, no_seg=False):
        # world(RealMap):在车上建的实景地图里开,road 不用,start = (x, y, yaw);没有 YOLO,地面分割按车上的投影 + 补缝
        if world is None:
            self.env = RoadEnv(road, pl, seed, start, noise=noise, robust=robust)
        else:
            from simulation.fastsim.real_map_env import RealMapEnv
            self.env = RealMapEnv(world, pl, seed, start, noise=noise)
            yolo = False
        # 裁图位姿抖动 (平移 m, 转角 rad),只在训练采集时开:BEV 是 5 cm 格,墙跨一格网络输出就跳(v3.5m5 横移 0.5 cm
        # 转角跳 10°,2026-10-15)。每拍按随机亚格偏移裁图,同一几何见过各种量化相位,学成对格子位置不敏感。车上不抖
        self.aug = aug
        self.aug_rng = np.random.default_rng(seed + 13) if aug[0] > 0 or aug[1] > 0 else None   # 独立随机流
        self.bev = Bev(lidar_yaw=pl.lidar_yaw)
        self.cam = FakeYolo(road, seed=seed + 7) if world is None else None
        self.yolo_period, self.next_yolo = yolo_period, 0.0
        # yolo=False:不登记假 YOLO(锥桶/胶带/区域层恒空),和车上一样(车上没跑 YOLO,2026-10-16)。
        # 路上的锥桶照样在,雷达照样扫得到、撞了照样算撞
        self.yolo = yolo
        self.free = free
        self.near = near
        if self.near and not self.free:
            raise ValueError('bev_v2.5 近场输入需要同时启用 6 通道主图')
        # seg_lut:假地面分割按车上的像素投影出点(远处条纹、没确认时只到 2 m),见 FloorSegConfig.lut
        from dataclasses import replace as _replace
        from simulation.fastsim.camera import FloorSegConfig
        seg_cfg = _replace(FloorSegConfig(), lut=(72, 96), fill=seg_fill) if seg_lut else FloorSegConfig()
        self.seg = FakeFloorSeg(road, cfg=seg_cfg, seed=seed + 11) if free and world is None else None    # 独立随机流:不改 bev_v1 的局
        if world is not None and free:
            from simulation.fastsim.real_map_env import MapFloorSeg
            self.seg = MapFloorSeg(world, seed=seed + 11)
        if no_seg:                 # 纯雷达(10-22):不跑地面分割,FREE 层永远空,通道照留(和 bev_v2.5 网络兼容)
            self.seg = None
        self.seg_period, self.seg_lag, self.next_seg = seg_period, seg_lag, 0.0
        self.seg_queue = []                                               # (到达时刻, 拍摄位姿, 点)
        self.seg_hooks = []                                               # 已送到 BEV 的帧:pose, xy, stamp
        self.seg_frame = None
        self.scan_hooks = []
        # guard='escape':指令转角净空不够时先找逃生转角(ScanGuard.escape),有就把转角换过去照常开,都没有才算失败。
        # guard=True:和车上 road_car_node 一样按将要打的转角扫车身矩形,保险杠前净空 < guard_stop(车速) 就算这局失败('guard')。
        # 车上截停后油门回中位、策略不出倒车,基本就卡死了;仿真里原来不管,模型在锥桶方角贴着开也算过(2026-10-16:
        # 无锥桶 r29 在 L6f 16 局里 12 局会触发截停,老师 1 局)
        self.guard = None
        self.guard_escape, self.n_escape = guard == 'escape', 0
        if guard:
            from vehicle.road_car_node import guard_stop
            from vehicle.scan_guard import ScanGuard
            self.guard, self.guard_stop = ScanGuard(), guard_stop       # 同车上:0.7 m/s 以上按车速放大
            self.scan_hooks.append(lambda pose: self._guard_scan())
        env, scan = self.env, self.env._scan

        def scan_and_register():
            scan()
            car = env.car
            self.scan_pose = (car.x, car.y, car.yaw)
            self.bev.update_lidar(env.ranges, env.angles, self.scan_pose, env.t)
            for hook in self.scan_hooks:
                hook(self.scan_pose)

        env._scan = scan_and_register
        self.scan_pose = self.pose                                        # 构造时已扫过的那一帧
        if self.guard is not None:
            self._guard_scan()
        self.bev.update_lidar(env.ranges, env.angles, self.scan_pose, 0.0)
        self._camera()

    @property
    def pose(self):
        c = self.env.car
        return (c.x, c.y, c.yaw)

    def _camera(self):
        if self.yolo and self.env.t >= self.next_yolo:
            kinds, xy = self.cam.detect(self.pose)
            self.bev.update_camera(kinds, xy, self.pose, self.env.t)
            self.next_yolo += self.yolo_period
        if self.seg is not None and self.env.t >= self.next_seg:
            self.seg_queue.append((self.env.t + self.seg_lag, self.pose, self.seg.detect(self.pose)))
            self.next_seg += self.seg_period
        while self.seg_queue and self.env.t >= self.seg_queue[0][0] - 1e-9:
            arrival, pose, xy = self.seg_queue.pop(0)
            stamp = arrival - self.seg_lag
            self.bev.update_camera(np.full(len(xy), int(CellType.FREE)), xy, pose, stamp)
            self.seg_frame = (pose, xy, stamp)
            for hook in self.seg_hooks:
                hook(pose, xy, stamp)

    def observation(self):
        env = self.env
        if self.near:
            img, near_img = self.images()
            enc_speed = env.car.v + (env.rng.normal(0, 0.02) if env.noise else 0.0)
            gyro = env.car.yaw_rate + (env.rng.normal(0, env.cp.gyro_sigma) if env.noise else 0.0)
            return img, scalars(enc_speed, env.pl.cap, gyro, env.pl.enc), near_img
        enc_speed = env.car.v + (env.rng.normal(0, 0.02) if env.noise else 0.0)
        gyro = env.car.yaw_rate + (env.rng.normal(0, env.cp.gyro_sigma) if env.noise else 0.0)
        return self.image(), scalars(enc_speed, env.pl.cap, gyro, env.pl.enc)

    def images(self):
        """同一裁图位姿下的主图和 5 cm 近场图;供 bev_v2.5 和成对增强采集使用。"""
        if not self.near:
            raise ValueError('当前 BevEnv 未启用近场分支')
        env, pose = self.env, self.pose
        if self.aug_rng is not None:
            d, a = self.aug
            pose = (pose[0] + self.aug_rng.uniform(-d, d), pose[1] + self.aug_rng.uniform(-d, d),
                    pose[2] + self.aug_rng.uniform(-a, a))
        return bev_images(self.bev, pose, env.t, env.ranges, env.angles, self.scan_pose)

    def image(self):
        """这一拍的主 BEV 图;开了 aug 就按一个新的随机亚格偏移裁。"""
        env, pose = self.env, self.pose
        if self.aug_rng is not None:
            d, a = self.aug
            pose = (pose[0] + self.aug_rng.uniform(-d, d), pose[1] + self.aug_rng.uniform(-d, d),
                    pose[2] + self.aug_rng.uniform(-a, a))
        return bev_image(self.bev, pose, env.t, env.ranges, env.angles, self.scan_pose, free=self.free)

    def _guard_scan(self):
        """把这一帧雷达按 ScanGuard.update 的规则换成后轴系前方点(车上喂的是同一份 /scan)。"""
        from vehicle.scan_guard import FRONT_HALF_ANGLE, LIDAR_X
        e = self.env
        a = (np.asarray(e.angles, float) + np.pi) % (2 * np.pi) - np.pi
        r = np.asarray(e.ranges, float)
        ok = np.isfinite(r) & (r > 0.05) & (np.abs(a) <= FRONT_HALF_ANGLE)
        x, y = r[ok] * np.cos(a[ok]) + LIDAR_X, r[ok] * np.sin(a[ok])
        self.guard.points = list(zip(x[x > 0], y[x > 0]))

    def step(self, action):
        if self.guard is not None:
            from common.pwm import action_to_steer
            steer = action_to_steer(float(np.clip(action[0], -1, 1)), self.env.cp)
            need = self.guard_stop(self.env.speed.v, self.env.cp)
            if self.guard._clear(steer) < need:
                alt = self.guard.escape(steer, need, self.env.cp.max_steer) if self.guard_escape else None
                if alt is None:
                    return 'guard'
                from common.pwm import steer_to_action
                action = np.array([steer_to_action(alt, self.env.cp), action[1]], np.float32)
                self.n_escape += 1
        why = self.env.step(action)
        self._camera()
        return why


class SeenTeacher(Teacher):
    """重规划窗口只到左右边界都被雷达扫到过的地方。"""

    def __init__(self, benv: BevEnv, P, a_lat: float, guard_aware: bool = False, steady: bool = False):
        super().__init__(benv.env, P, a_lat=a_lat)
        self.steady = steady    # guard_aware 的自洽限速版(见 _steady),2026-10-22 加
        # guard_aware:按车上截停规则限速(要 benv 开了 guard)。按这一拍要打的转角扫净空,扣掉指令生效前
        # 要走的距离,反解 guard_stop 得允许车速;超了就刹车。默认关:老训练配置逐位不变。2026-10-21 加。
        self.benv = benv if guard_aware and benv.guard is not None else None
        road = benv.env.road
        self.nb = len(road.left)
        self.tree = cKDTree(np.vstack([road.left, road.right]))
        self.seen = np.zeros(2 * self.nb, bool)
        # EDGE 只能由实际送到学生 BEV 的地面点确认,不能用理想视场提前登记。
        self.edge = np.concatenate([road.mat_left, road.mat_right]) == EDGE
        self.seg = benv.seg
        self.v_end = math.sqrt(a_lat * R_MIN)
        benv.scan_hooks.append(self._mark)
        self._mark(benv.pose)
        benv.seg_hooks.append(self._mark_floor)
        if benv.seg_frame is not None:
            self._mark_floor(*benv.seg_frame)

    def _mark(self, pose):
        env = self.env
        from common.bev import lidar_points
        pts = lidar_points(env.ranges, env.angles, pose, env.cp, env.pl.lidar_yaw, 10.0)
        for idx in self.tree.query_ball_point(pts, SEEN_R):
            self.seen[idx] |= ~self.edge[idx]

    def _mark_floor(self, pose, xy, stamp):
        if not len(xy) or not self.edge.any():
            return
        from common.bev import body_to_odom
        dist, _ = cKDTree(body_to_odom(xy, pose)).query(self.tree.data[self.edge])
        self.seen[self.edge] |= dist <= FLOOR_SEEN_R

    def __call__(self) -> np.ndarray:
        a = super().__call__()
        if self.benv is None:
            return a
        from common.pwm import action_to_steer
        from common.road_policy import BRAKE_CMD, speed_to_action
        from vehicle.road_car_node import guard_speed
        env, car, pl = self.env, self.env.car, self.pl
        from vehicle.road_car_node import guard_stop
        lead = car.v * (pl.dead_time + CONTROL_DT * (pl.lag + 1)) + GUARD_MARGIN
        # 净空按"这一拍的转角 + 刹停距离内路线上要打的转角"取最小:转角一变扫掠弧就变,净空会突然塌(实测 inf -> 0.7 m)
        P = self.plan
        if self.steady:
            return self._steady(a, P)
        reach = P.s[self.j] + lead + guard_stop(car.v, env.cp)
        ks = P.kappa[self.j:int(np.searchsorted(P.s, reach)) + 1:3]
        steers = [action_to_steer(float(a[0]), env.cp)] + [math.atan(env.cp.wheelbase * k) for k in ks]
        clear = min(self.benv.guard._clear(st) for st in steers)
        if not math.isfinite(clear):
            return a
        v_ok = guard_speed(clear - lead, env.cp)
        if car.v > max(v_ok, self.release) + BRAKE_MARGIN:
            return np.array([a[0], speed_to_action(BRAKE_CMD, env.pl.enc)], np.float32)
        if a[1] > speed_to_action(v_ok, env.pl.enc):
            a = np.array([a[0], speed_to_action(v_ok, env.pl.enc)], np.float32)
        return a

    def _steady(self, a, P):
        """guard_aware 的修正版(2026-10-22,实车 MAD MAX 弯里直道都来回刹)。原版把路线上后面各点的曲率当转角,
        从**车现在的位姿**画圆弧扫:老师正打 +20° 绕桶时,"按路线前面的 0° 从现在直着开"0.6 m 就撞到正在绕开的桶 ->
        刹,弯里几乎每拍都这样(L5g 每 100 m 70~100 次)。这里改成在路线上的未来位姿按那里的转角模拟截停(见下),
        目标车速走滑行曲线、刹车走刹车曲线,两者同源;另外目标车速不高于基础老师的 v_coast。"""
        from common.pwm import action_to_steer
        from common.road_policy import BRAKE_CMD, speed_to_action
        env, car, pl = self.env, self.env.car, self.pl
        k_lead = pl.dead_time + CONTROL_DT * (pl.lag + 1)
        # 基础老师目标车速按"能刹车"的曲线 P.v,刹车却按"只靠滑行"的 v_coast 判:车速在两者之间就刹一拍、
        # 目标又高于车速再给油、再刹(锯齿)。进了滑行来不及的区间就不再要比 v_coast 高的车速,只松油。
        if P.v_coast is not None and a[1] >= 0:
            kv = min(len(P.v) - 1, int(np.searchsorted(P.s, P.s[self.j] + max(car.v, 0.3) * k_lead + SPEED_MARGIN)))
            a = np.array([a[0], min(float(a[1]), speed_to_action(max(float(P.v_coast[kv]), self.release), env.pl.enc))],
                         np.float32)
        # 车上截停每拍按"这一拍转角的圆弧"查 gap >= guard_stop(车速)。在计划路线上往前取点,按车到那里的位姿和
        # 那里的转角(路线曲率)模拟这次检查,得到到那里时允许的车速 u_k;现在的目标车速 = 只靠滑行(A_DEC)能在
        # 那之前降到 u_k 的最大车速(扣指令延迟里走的路,v 和延迟自洽地二分);按实测刹车减速度都来不及才刹。
        # 目标和刹车出自同一条曲线,不来回切。k = 0 是这一拍要打的转角。
        ds, u = self._future_guard(P, action_to_steer(float(a[0]), env.cp))
        if not len(ds):
            return a

        def v_ok(v, dec):
            room = np.maximum(ds - v * k_lead, 0.0)
            room[0] = 0.0                                   # 这一拍的检查车速已经改不了
            return float(np.sqrt(u * u + 2 * dec * room).min())

        if car.v > max(v_ok(car.v, env.cp.guard_brake_decel), self.release) + BRAKE_MARGIN:
            return np.array([a[0], speed_to_action(BRAKE_CMD, env.pl.enc)], np.float32)
        lo, hi = 0.0, max(env.pl.cap, car.v)            # v_ok 随 v 单调不增:二分找 v <= v_ok(v) 的最大 v
        if hi <= v_ok(hi, RT.A_DEC):
            return a
        for _ in range(10):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if mid <= v_ok(mid, RT.A_DEC) else (lo, mid)
        if a[1] > speed_to_action(lo, env.pl.enc):
            a = np.array([a[0], speed_to_action(lo, env.pl.enc)], np.float32)
        return a

    def _future_guard(self, P, steer_now, look=3.0, step=0.15):
        """计划路线上每 step m 一个点(第 0 个是车现在、转角 steer_now):在那里按路线曲率的转角做一次截停检查
        (这一帧前方雷达点,ScanGuard 同款车身扫掠),返回 (离现在的路程, 那里允许的最高车速)。"""
        from vehicle.road_car_node import guard_speed
        from vehicle.scan_guard import FRONT_HALF_ANGLE, LIDAR_X, ScanGuard
        g = self.benv.guard
        if not g.points:
            return np.zeros(0), np.zeros(0)
        car, cp = self.env.car, self.env.cp
        p = np.asarray(g.points, float)
        c, s = math.cos(car.yaw), math.sin(car.yaw)
        w = np.c_[car.x + c * p[:, 0] - s * p[:, 1], car.y + s * p[:, 0] + c * p[:, 1]]
        s0 = P.s[self.j]
        ks = np.searchsorted(P.s, s0 + np.arange(step, look, step))
        ks = ks[ks < len(P.s) - 1]
        out_d, out_u = [0.0], [guard_speed(g._clear(steer_now) - GUARD_MARGIN, cp)]
        probe = ScanGuard()
        for k in ks:
            dx, dy = P.pts[k + 1] - P.pts[k - 1] if k > 0 else P.pts[1] - P.pts[0]
            h = math.atan2(dy, dx)
            ch, sh = math.cos(h), math.sin(h)
            rx, ry = w[:, 0] - P.pts[k, 0], w[:, 1] - P.pts[k, 1]
            bx, by = ch * rx + sh * ry, -sh * rx + ch * ry
            keep = (bx > 0) & (np.abs(np.arctan2(by, bx - LIDAR_X)) <= FRONT_HALF_ANGLE)
            probe.points = list(zip(bx[keep], by[keep]))
            out_d.append(float(P.s[k] - s0))
            out_u.append(guard_speed(probe._clear(math.atan(cp.wheelbase * P.kappa[k])) - GUARD_MARGIN, cp))
        return np.array(out_d), np.array(out_u)

    def known_ahead(self, i):
        both = self.seen[:self.nb] & self.seen[self.nb:]
        j = i
        while j < self.nb - 1 and both[j + 1]:
            j += 1
        return (j - i) * DS

    def _replan(self):
        env, car = self.env, self.env.car
        i, d = env.road.project((car.x, car.y), env.idx)
        win = min(HORIZON, max(MIN_WIN, self.known_ahead(i)))
        P = plan(env.road, i, d, env.cp.width, self.pl, win, R_REPLAN, self.a_lat, self.v_end, self.brake)
        if isinstance(P, str):
            self.fails += 1
            return
        self.plan, self.j = P, 0
