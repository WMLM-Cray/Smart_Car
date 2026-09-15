#!/usr/bin/env python3
"""SLAM 短距离建图冒烟驱动+验图(2026-09-16,阶段3 前置)。

前置:sim_bringup(trackA,headless)与 slam_toolbox online_async 已在跑。
本节点:等车辆就绪 → 底道前进 ~1.75m → 缓弧右转(采集转弯畸变样本)→
停车 → 等 /map → 判据(free/occupied 格数)→ 存 PGM+yaml 到 docs/track/。
只发 /cmd_vel、只读话题,不改任何场景/几何/参数。
退出码:判据全过=0。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

OUT_PREFIX = '/home/lzx/hwcart_sim/docs/track/trackA_slam_smoke'


class SlamSmoke(Node):

    def __init__(self):
        super().__init__('slam_smoke_driver')
        self.pose = None
        self.map_msg = None
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(ModelStates, '/model_states',
                                 self._on_models, 10)
        self.create_subscription(OccupancyGrid, '/map',
                                 self._on_map, qos_profile_sensor_data)

    def _on_models(self, msg):
        try:
            i = msg.name.index('racecar')
        except ValueError:
            return
        p = msg.pose[i].position
        self.pose = (p.x, p.y)

    def _on_map(self, msg):
        self.map_msg = msg

    def spin_for(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def drive(self, v, w, wall_s, rate_hz=20):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        end = time.time() + wall_s
        last = 0.0
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
            if time.time() - last >= 1.0 / rate_hz:
                self.cmd.publish(msg)
                last = time.time()

    def run(self):
        print('[1/5] 等 racecar(spawn 完成)…')
        end = time.time() + 90
        while time.time() < end and self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pose is None:
            print('[FAIL] 90s 未等到 racecar')
            return False
        p0 = self.pose
        print(f"[OK] 起点 ({p0[0]:.2f},{p0[1]:.2f})")

        print('[2/5] 前进 ~1.75m(底道,锥桶走廊内)…')
        self.drive(0.35, 0.0, 5.0)
        print('[3/5] 缓弧右转 4s(转弯畸变样本)…')
        self.drive(0.25, -0.35, 4.0)
        msg = Twist()
        self.cmd.publish(msg)
        self.spin_for(1.0)
        p1 = self.pose
        dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        print(f"[OK] 终点 ({p1[0]:.2f},{p1[1]:.2f}) 行驶 {dist:.2f}m")

        print('[4/5] 等 /map(slam_toolbox 增量输出,15s)…')
        end = time.time() + 15
        while time.time() < end and self.map_msg is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.map_msg is None:
            print('[FAIL] /map 15s 无消息(slam_toolbox 是否已起?)')
            return False

        m = self.map_msg
        cells = list(m.data)
        n = m.info.width * m.info.height
        free = sum(1 for c in cells if c == 0)
        occ = sum(1 for c in cells if c > 65)
        unk = n - free - sum(1 for c in cells if 0 < c <= 65) - occ
        print(f"[INFO] /map {m.info.width}x{m.info.height} "
              f"res={m.info.resolution:.3f}m "
              f"free={free} occ={occ} unknown={unk}")
        ok1 = m.info.width > 50 and m.info.height > 50
        ok2 = free > 500
        ok3 = occ > 30        # 至少扫到部分锥桶/墙
        print(f"[{'PASS' if ok1 else 'FAIL'}] 地图尺寸(判据>50x50)")
        print(f"[{'PASS' if ok2 else 'FAIL'}] free 格 {free}(判据>500)")
        print(f"[{'PASS' if ok3 else 'FAIL'}] occupied 格 {occ}(判据>30)")

        print('[5/5] 存 PGM+yaml …')
        with open(OUT_PREFIX + '.pgm', 'wb') as f:
            f.write(f'P5\n{m.info.width} {m.info.height}\n255\n'.encode())
            row = bytearray(m.info.width * m.info.height)
            for i, c in enumerate(cells):
                row[i] = 205 if c < 0 else (0 if c > 65 else 254)
            f.write(bytes(row))
        with open(OUT_PREFIX + '.yaml', 'w') as f:
            f.write(
                f'image: {OUT_PREFIX}.pgm\nresolution: {m.info.resolution}\n'
                f'origin: [{m.info.origin.position.x}, '
                f'{m.info.origin.position.y}, '
                f'{m.info.origin.orientation.z}]\nnegate: 0\noccupied_thresh: '
                f'0.65\nfree_thresh: 0.196\n')
        print(f"[OK] 已存 {OUT_PREFIX}.pgm/.yaml")
        return ok1 and ok2 and ok3


def main():
    rclpy.init()
    node = SlamSmoke()
    try:
        ok = node.run()
        print('slam_smoke_rc=' + ('0 PASS' if ok else '1 FAIL'))
        sys.exit(0 if ok else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
