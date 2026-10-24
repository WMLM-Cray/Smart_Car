#!/usr/bin/env python3
"""用 /scan 做的基础防撞:算出保险杠前面沿**将要走的路径**还有多少空间。

给遥控(server.py)和测速(pwm_sweep.py)共用。只管前进方向,倒车不拦 —— 雷达在车头,
背后本来就看不见,拦了反而没法把车倒出来。

雷达装在后轴前 14.5cm,车最前端(前轮)在后轴前 41.5cm(2026-09-30 实测,见下方常量)。内部一律换算到后轴坐标系(base_footprint),
gap() 返回的是**车头**到障碍的距离,所以要减掉 NOSE_AHEAD。

## 为什么走廊要跟着方向盘弯(2026-09-22)

原来只看正前方一条直走廊。赛道 A 的 180° 掉头内径 1.25m,内侧锥桶就落在这条直
走廊里 —— 但车在打方向,实际是从旁边绕过去的,根本不会撞。实测策略跑赛道 A 时
报了 132 次"前方 0.15m,截停",每次车离最近锥桶都还有 0.79m。截停之后油门归零、
转向照发、场景永远不变,车就再也起不来了(策略不出倒车),一次误判 = 这一局结束。

现在按当前前轮转角算后轴的行驶圆弧,只看**扫掠路径**上的东西,距离也沿弧长量。
转角给 0(或不给)就退化成原来的直走廊,老调用方行为不变。

## 改成扫车身矩形(2026-10-10)

原来是"后轴路径两侧各 0.28 m 的带子里、在后轴前面的点"都算,距离 = 沿弧长 - 车长。车身**旁边**的点
(比如平行的墙离车中线 0.25 m,车身外还有 7.5 cm)算出来是负数 -> 截停,擦得过去的也被拦(用户实测)。
现在把车身矩形(后轴系 x ∈ [-REAR_OVERHANG, NOSE_AHEAD]、|y| ≤ 半宽 + SIDE_MARGIN)沿当前转角的路径往前推,
gap = 后轴还能走多远车身才碰到第一个点。已经在车身旁边、不在前进路线上的点不算。
"""
from __future__ import annotations

import math
import time

import numpy as np

# [实测 2026-09-30 用户量] 雷达圆心到车身前端 0.24、后端 0.20 m;前轮比车身前端多伸出 0.03、后轮比后端 0.01;
# 轮径 ~0.13;雷达离地 0.28;车尾里程计展开后再伸出 0.17。推出:最前端是前轮(雷达前 0.27),
# 前轴在雷达前 0.205、后轴在雷达后 0.145(轴距 0.35,旧记录 0.33)。原来按厂家 TF:后轴->雷达 0.07、雷达->车头 0.36。
BUMPER_AHEAD = 0.27        # [实测] 雷达到车最前端(前轮胎面),直行净空只用它
LIDAR_X = 0.145            # [实测推算] 后轴到雷达
NOSE_AHEAD = LIDAR_X + BUMPER_AHEAD    # 0.415,后轴到车最前端
HALF_WIDTH = 0.175         # [实测] 车身半宽
SIDE_MARGIN = 0.04         # 车身两侧余量:雷达噪声 ~1 cm + 实际转角和标定的偏差(原来是 0.105)
HALF_CORRIDOR = HALF_WIDTH + SIDE_MARGIN
REAR_OVERHANG = 0.065      # [实测] 后轴到车尾
FRONT_HALF_ANGLE = 1.05    # 只看正前方 ±60°,再宽就把侧面的锥桶也算进来了
STALE = 1.0                # 这么久没有新扫描就当雷达掉线
WHEELBASE = 0.35           # [实测 2026-09-30] 轴距(前轴雷达前 0.205 + 后轴雷达后 0.145),算转弯半径用。09-22 记的 0.33,出厂源码的 0.305 是别的车
STRAIGHT_STEER = 0.02      # 转角小于这个就按直行算(R > 15m,和直线没区别)
MAX_LOOKAHEAD = 4.0        # 沿弧最多看这么远,免得绕一圈把身后的东西算进来
SWEEP_STEP = 0.02          # 转弯时车身每推进这么多 m 查一次
# 逃生转角(2026-10-23):指令转角的圆弧净空不够刹停时,在它左右 ESCAPE_WINDOW 内找净空够的转角,有就微调过去不截停。
# 恒定转角的圆弧外推两三米很不准(差 0.03 rad、2.8 m 外横向就差 0.34 m),高速下锥桶路几乎每圈都误截停;
# 10-23 纯雷达 8 圈回算:2.5 m/s 原规则 28% 帧截停,加逃生转角后 0.2%,微调中位 0.03 rad、最大 0.12 rad
ESCAPE_WINDOW = 0.15
ESCAPE_STEP = 0.01
GUARD_STOP = 0.25           # 保险杠前净空小于这个就截停(米),guard_base_speed 下实车验证过


def guard_stop(speed, p):
    """截停距离:GUARD_STOP 在 guard_base_speed 下实车验证过;更快时加上多出来的延迟距离和刹车距离。
    策略(road_car_node)和网页手动(server)共用,2026-10-24 从 road_car_node 挪过来。"""
    v0 = p.guard_base_speed
    extra = p.guard_latency * (speed - v0) + (speed * speed - v0 * v0) / (2 * p.guard_brake_decel)
    return GUARD_STOP + max(0.0, extra)


class ScanGuard:
    """喂 LaserScan,问 gap(steer):保险杠前方净空(米)。没有可信数据返回 None。"""

    def __init__(self):
        self.points: list[tuple[float, float]] = []   # 后轴系里的前方障碍点
        self.stamp = 0.0

    def update(self, msg):
        inc = msg.angle_increment
        pts = []
        for i, r in enumerate(msg.ranges):
            # 折回 [-π, π):Gazebo 的雷达按 0..2π 发角度,右前方那半边落在数组末尾,
            # 按 angle_min 直接切窗口会把它整段漏掉 —— 不报错,只是前方只看得见左半边。
            ang = (msg.angle_min + i * inc + math.pi) % (2 * math.pi) - math.pi
            if abs(ang) > FRONT_HALF_ANGLE:
                continue
            if r != r or r < msg.range_min or r > msg.range_max or math.isinf(r):
                continue
            x = r * math.cos(ang) + LIDAR_X         # 换到后轴系
            y = r * math.sin(ang)
            if x > 0:
                pts.append((x, y))
        self.points = pts
        self.stamp = time.time()

    def _clear(self, steer):
        """后轴沿当前路径还能走多远(米)车身矩形才碰到第一个障碍点;碰不到返回 inf。"""
        if not self.points:
            return float('inf')
        p = np.asarray(self.points)
        if abs(steer) < STRAIGHT_STEER:
            ahead = p[(np.abs(p[:, 1]) <= HALF_CORRIDOR) & (p[:, 0] >= -REAR_OVERHANG)]
            if len(ahead) == 0:
                return float('inf')
            return float(max(0.0, (ahead[:, 0] - NOSE_AHEAD).min()))
        # 后轴走半径 R 的圆,圆心在正侧方 (0, s*R)。车走过弧长 d,等价于点绕圆心反转 d/R。
        sgn = 1.0 if steer > 0 else -1.0
        radius = WHEELBASE / abs(math.tan(steer))
        cy = sgn * radius
        # 车身矩形离圆心的距离只在 [R - 半宽, hypot(R + 半宽, 车头)] 之间,环带外的点永远扫不到
        r = np.hypot(p[:, 0], p[:, 1] - cy)
        p = p[(r >= radius - HALF_CORRIDOR) & (r <= math.hypot(radius + HALF_CORRIDOR, NOSE_AHEAD))]
        if len(p) == 0:
            return float('inf')
        d = np.arange(0.0, MAX_LOOKAHEAD + 1e-9, SWEEP_STEP)
        phi = -sgn * d / radius                                  # (k,)
        c, sn = np.cos(phi)[:, None], np.sin(phi)[:, None]
        px, py = p[:, 0][None, :], (p[:, 1] - cy)[None, :]
        x = c * px - sn * py
        y = sn * px + c * py + cy
        hit = ((x >= -REAR_OVERHANG) & (x <= NOSE_AHEAD) & (np.abs(y) <= HALF_CORRIDOR)).any(1)
        k = np.flatnonzero(hit)
        return float(d[k[0]]) if len(k) else float('inf')

    def escape(self, steer, need, max_steer):
        """离 steer 最近、圆弧净空 >= need 的转角(在 ±ESCAPE_WINDOW、±max_steer 以内);没有返回 None。"""
        n = int(round(ESCAPE_WINDOW / ESCAPE_STEP))
        for k in range(1, n + 1):
            for s in (steer - k * ESCAPE_STEP, steer + k * ESCAPE_STEP):
                if abs(s) <= max_steer and self._clear(s) >= need:
                    return s
        return None

    def gap(self, steer=0.0, now=None):
        """沿行驶路径还能走多远(米)车身才碰到障碍;雷达没数据或数据过期返回 None。
        直行时就是车头到前方障碍的距离。

        steer 是前轮转角(rad,左正)。不给就按直行算 —— 老调用方(网页遥控、
        pwm_sweep)本来就是直着走的。
        """
        now = time.time() if now is None else now
        if not self.stamp or now - self.stamp > STALE:
            return None
        return self._clear(steer)
