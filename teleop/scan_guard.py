#!/usr/bin/env python3
"""用 /scan 做的基础防撞:只看正前方一条和车同宽的走廊,算出保险杠前面还有多少空间。

给遥控(server.py)和测速(pwm_sweep.py)共用。只管前进方向,倒车不拦 —— 雷达在车头,
背后本来就看不见,拦了反而没法把车倒出来。

雷达装在后轴前 7cm,车头在后轴前 43cm,所以雷达读到的距离要减掉 36cm 才是保险杠到
障碍物的距离(BUMPER_AHEAD)。这个偏移不减,车头会先撞上。
"""
from __future__ import annotations

import math
import time

BUMPER_AHEAD = 0.36        # [代码] 车头到雷达的距离 = (length - rear_overhang) - lidar_x
HALF_CORRIDOR = 0.28       # 车半宽 0.175 + 余量,只管这条走廊里的东西
FRONT_HALF_ANGLE = 1.05    # 只看正前方 ±60°,再宽就把侧面的锥桶也算进来了
STALE = 1.0                # 这么久没有新扫描就当雷达掉线


class ScanGuard:
    """喂 LaserScan,问 gap():保险杠前方净空(米)。没有可信数据返回 None。"""

    def __init__(self):
        self.clear = float('inf')
        self.stamp = 0.0

    def update(self, msg):
        n = len(msg.ranges)
        inc = msg.angle_increment
        lo = max(0, int((-FRONT_HALF_ANGLE - msg.angle_min) / inc))
        hi = min(n, int((FRONT_HALF_ANGLE - msg.angle_min) / inc) + 1)
        best = float('inf')
        for i in range(lo, hi):
            r = msg.ranges[i]
            if r != r or r < msg.range_min or r > msg.range_max or math.isinf(r):
                continue
            ang = msg.angle_min + i * inc
            x, y = r * math.cos(ang), r * math.sin(ang)
            if x > 0 and abs(y) <= HALF_CORRIDOR and x < best:
                best = x
        self.clear = best
        self.stamp = time.time()

    def gap(self, now=None):
        """保险杠到正前方最近障碍的距离;雷达没数据或数据过期返回 None。"""
        now = time.time() if now is None else now
        if not self.stamp or now - self.stamp > STALE:
            return None
        if math.isinf(self.clear):
            return float('inf')
        return self.clear - BUMPER_AHEAD
