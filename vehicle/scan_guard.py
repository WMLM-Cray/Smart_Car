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
"""
from __future__ import annotations

import math
import time

# [实测 2026-09-30 用户量] 雷达圆心到车身前端 0.24、后端 0.20 m;前轮比车身前端多伸出 0.03、后轮比后端 0.01;
# 轮径 ~0.13;雷达离地 0.28;车尾里程计展开后再伸出 0.17。推出:最前端是前轮(雷达前 0.27),
# 前轴在雷达前 0.205、后轴在雷达后 0.145(轴距 0.35,旧记录 0.33)。原来按厂家 TF:后轴->雷达 0.07、雷达->车头 0.36。
BUMPER_AHEAD = 0.27        # [实测] 雷达到车最前端(前轮胎面),直行净空只用它
LIDAR_X = 0.145            # [实测推算] 后轴到雷达
NOSE_AHEAD = LIDAR_X + BUMPER_AHEAD    # 0.415,后轴到车最前端
HALF_CORRIDOR = 0.28       # 车半宽 0.175 + 余量,只管这条走廊里的东西
FRONT_HALF_ANGLE = 1.05    # 只看正前方 ±60°,再宽就把侧面的锥桶也算进来了
STALE = 1.0                # 这么久没有新扫描就当雷达掉线
WHEELBASE = 0.35           # [实测 2026-09-30] 轴距(前轴雷达前 0.205 + 后轴雷达后 0.145),算转弯半径用。09-22 记的 0.33,出厂源码的 0.305 是别的车
STRAIGHT_STEER = 0.02      # 转角小于这个就按直行算(R > 15m,和直线没区别)
MAX_LOOKAHEAD = 4.0        # 沿弧最多看这么远,免得绕一圈把身后的东西算进来


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
        """后轴沿当前路径到最近障碍的距离(米),没有就 inf。"""
        if abs(steer) < STRAIGHT_STEER:
            return min((x for x, y in self.points if abs(y) <= HALF_CORRIDOR),
                       default=float('inf'))
        # 后轴走半径 R 的圆,圆心在正侧方 (0, s*R)。左打方向 s=+1,逆时针。
        s = 1.0 if steer > 0 else -1.0
        radius = WHEELBASE / abs(math.tan(steer))
        best = float('inf')
        car_ang = math.atan2(-s * radius, 0.0)      # 车相对圆心的方位角
        for x, y in self.points:
            d = math.hypot(x, y - s * radius)
            if abs(d - radius) > HALF_CORRIDOR:     # 不在扫掠环带里,绕得过去
                continue
            theta = (math.atan2(y - s * radius, x) - car_ang) * s
            theta %= 2 * math.pi                    # 只算前方那一侧,身后的绕一圈变成大角
            along = theta * radius
            if along < best:
                best = along
        return best if best <= MAX_LOOKAHEAD else float('inf')

    def gap(self, steer=0.0, now=None):
        """车头到前方最近障碍沿行驶路径的距离;雷达没数据或数据过期返回 None。

        steer 是前轮转角(rad,左正)。不给就按直行算 —— 老调用方(网页遥控、
        pwm_sweep)本来就是直着走的。
        """
        now = time.time() if now is None else now
        if not self.stamp or now - self.stamp > STALE:
            return None
        clear = self._clear(steer)
        if math.isinf(clear):
            return float('inf')
        return clear - NOSE_AHEAD
