#!/usr/bin/env python3
"""trackA 运行时四项验证器(入库版,2026-09-16)。

背景:/tmp/validate_trackA.py 未入库已丢失(两处核清教训),本文件为重写
归档版并按任务书扩充。判据与工况:
  [启动]   joint_states 频率 + racecar 初始位姿与 spawn 对账(世界系真值)
  [扫描]   /scan 有效点数 + 最小距离(静态锥碰撞体,期望区间宽放)
  [成像]   /image_raw:按 encoding 字段显式解析通道(核清二定案要求);
           判据=计数+判据自洽(蓝组 B 主导/红组 R 主导);
           方位实测为记录项(TF 已声明 REP104 但 gazebo 像素 u 方向与
           frame 声明无关,2026-09-16 实测指向图像水平镜像,视觉阶段终验)
  [运动]   W1 前进→停止(看门狗收敛) W2 零速指令停车
           W3 倒车 W4 倒车左转(ω 跟踪)
           —— 任务书 2026-09-16:验证停止与倒车转向
发令方式:常驻 rclpy publisher,墙钟时长驱动,记录首末指令时刻
(阶段1回归工具方向,不用 ros2 topic pub——其 daemon 冷启动
~1.5s 吃指令窗口;实测 WSL headless 实时因子≈1)。
位姿来源:/model_states(gazebo_ros_state WorldPlugin 世界系真值,
不信任里程计账面)。
退出码:全部 PASS=0,任一 FAIL=1。

用法(WSL,已 source ROS2+工作区,trackA world 车辆已由 launch 拉起):
  python3 tools/trackA/validate_trackA_runtime.py [--csv docs/track/trackA_cones.csv]
只读验证,不修改任何场景/几何/参数。
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Image, LaserScan, JointState
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

SPAWN_X, SPAWN_Y, SPAWN_YAW = 11.0, 0.75, 0.0


def yaw_of(q):
    """四元数→yaw(ZYX 序,车模平面运动只取 yaw)。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Validator(Node):

    def __init__(self, csv_path):
        super().__init__('validate_trackA_runtime')
        self.csv_path = csv_path
        # 采集缓存
        self.js_stamps = []          # joint_states 到达时刻(墙钟,仅频率)
        self.scan_msg = None
        self.img_msg = None
        self.clock_now = None
        self.pose = None             # racecar 世界系位姿 (x,y,yaw)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(JointState, '/joint_states',
                                 self._on_js, 10)
        self.create_subscription(LaserScan, '/scan',
                                 self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Image, '/image_raw',
                                 self._on_img, qos_profile_sensor_data)
        self.create_subscription(ModelStates, '/model_states',
                                 self._on_models, 10)
        self.create_subscription(Clock, '/clock', self._on_clock, 10)

    # ---------- 回调 ----------
    def _on_js(self, msg):
        self.js_stamps.append(time.time())

    def _on_scan(self, msg):
        if self.scan_msg is None:
            self.scan_msg = msg

    def _on_img(self, msg):
        self.img_msg = msg  # 持续取最新帧

    def _on_models(self, msg):
        try:
            i = msg.name.index('racecar')
        except ValueError:
            return
        p = msg.pose[i].position
        self.pose = (p.x, p.y, yaw_of(msg.pose[i].orientation))

    def _on_clock(self, msg):
        self.clock_now = msg.clock.sec + msg.clock.nanosec * 1e-9

    # ---------- 基础 ----------
    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def sim_now(self):
        return self.clock_now if self.clock_now is not None else 0.0

    def wait_pose(self, timeout=90.0):
        print('[等待] /model_states 出现 racecar(spawn 完成)…')
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pose is not None:
                return True
        return False

    def send_cmd(self, v, w, dur_wall_s, rate_hz=20):
        """按墙钟持续发指令 dur_wall_s,返回(首指令,末指令)墙钟时刻。

        说明:实测 WSL headless 实时因子≈1(位移/角度与墙钟窗吻合),
        统一墙钟驱动;/clock 是否到达留诊断(见 check_startup 的 INFO)。"""
        t_end = time.time() + dur_wall_s
        last_pub = 0.0
        t_first = t_last = None
        msg = Twist()
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.01)
            now = time.time()
            if now - last_pub >= 1.0 / rate_hz:
                msg.linear.x = float(v)
                msg.angular.z = float(w)
                self.cmd_pub.publish(msg)
                if t_first is None:
                    t_first = now
                t_last = now
                last_pub = now
        return t_first, t_last

    def send_zero_once(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    # ---------- 四项 ----------
    def check_startup(self):
        print('\n===== [1/4] 启动 =====')
        # 等 joint_states 首条:spawn 完成≠控制器激活(jsb_spawner 仍在跑),
        # 过早开测会假 FAIL(首版教训:0 msg/s 但位姿/运动全正常)
        deadline = time.time() + 60
        n0 = len(self.js_stamps)
        while time.time() < deadline and n0 == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.js_stamps.clear()
        self.spin_for(3.0)
        hz = (len(self.js_stamps) - 1) / 3.0 if len(self.js_stamps) > 1 else 0
        ok = hz >= 30
        print(f"[{'PASS' if ok else 'FAIL'}] joint_states {hz:.1f} msg/s(判据≥30)")
        if self.clock_now is not None:
            print(f"[INFO] /clock 已收(sim={self.clock_now:.1f}s)")
        else:
            print('[INFO] /clock 未收到(节点未设 use_sim_time,不影响订阅话题;'
                  '发令窗按墙钟,实测实时因子≈1)')
        if self.pose is None:
            print('[FAIL] racecar 位姿未取到')
            return ok, False
        dx = math.hypot(self.pose[0] - SPAWN_X, self.pose[1] - SPAWN_Y)
        dyaw = abs((self.pose[2] - SPAWN_YAW + math.pi) % (2 * math.pi) - math.pi)
        ok2 = dx < 0.05 and dyaw < 0.02
        print(f"[{'PASS' if ok2 else 'FAIL'}] 初始位姿 "
              f"({self.pose[0]:.3f},{self.pose[1]:.3f},{self.pose[2]:.3f}) "
              f"vs spawn 偏差 {dx:.3f}m/{math.degrees(dyaw):.2f}°")
        return ok, ok2

    def check_scan(self):
        print('\n===== [2/4] 扫描 =====')
        deadline = time.time() + 10
        while self.scan_msg is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.scan_msg is None:
            print('[FAIL] /scan 10s 无消息')
            return False
        vals = [r for r in self.scan_msg.ranges
                if math.isfinite(r) and r > self.scan_msg.range_min]
        mn = min(vals) if vals else float('inf')
        ok1 = len(vals) > 100
        ok2 = 0.3 <= mn <= 1.3
        print(f"[{'PASS' if ok1 else 'FAIL'}] /scan 有效点 {len(vals)}(判据>100)")
        print(f"[{'PASS' if ok2 else 'FAIL'}] /scan min={mn:.3f}m(判据 0.3~1.3,"
              f"静态锥碰撞体;历史参考 0.639/0.601)")
        return ok1 and ok2

    def check_image(self):
        print('\n===== [3/4] 成像(encoding 显式解析+方位对照)=====')
        deadline = time.time() + 10
        while self.img_msg is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.img_msg is None:
            print('[FAIL] /image_raw 10s 无消息')
            return False
        m = self.img_msg
        enc = m.encoding.lower()
        print(f"[INFO] encoding={m.encoding} {m.width}x{m.height} step={m.step}")
        if enc not in ('rgb8', 'bgr8'):
            print(f'[FAIL] 未支持的 encoding:{m.encoding}(需人工扩展判据)')
            return False
        h, w = m.height, m.width
        px = bytes(m.data)
        idx_b, idx_r = [], []
        for i in range(0, h * w):
            o = i * 3
            if enc == 'rgb8':
                r, g, b = px[o], px[o + 1], px[o + 2]
            else:
                r, g, b = px[o + 2], px[o + 1], px[o]
            if b - r > 25 and b - g > 15:
                idx_b.append(i)
            elif r - b > 25 and r - g > 15:
                idx_r.append(i)
        ok1 = len(idx_b) > 50 and len(idx_r) > 50
        print(f"[{'PASS' if ok1 else 'FAIL'}] 蓝判据像素 {len(idx_b)} / "
              f"红判据像素 {len(idx_r)}(判据各>50)")
        if not (idx_b and idx_r):
            return ok1

        # 通道均值(核清二证据,按 encoding 显式解析)
        def mean_rgb(idx):
            n = len(idx)
            rs = gs = bs = 0
            for i in idx:
                o = i * 3
                if enc == 'rgb8':
                    rs += px[o]; gs += px[o + 1]; bs += px[o + 2]
                else:
                    rs += px[o + 2]; gs += px[o + 1]; bs += px[o]
            return (round(rs / n), round(gs / n), round(bs / n))
        mb, mr = mean_rgb(idx_b), mean_rgb(idx_r)
        # 硬判据:判据-像素自洽(蓝判据组须 B 通道主导,红判据组须 R 主导)
        ok2 = mb[2] > mb[0] and mr[0] > mr[2]
        print(f"[{'PASS' if ok2 else 'FAIL'}] 判据自洽:蓝组 RGB={mb}(B 主导) "
              f"红组 RGB={mr}(R 主导)")
        print(f"[INFO] encoding={m.encoding};核清二定案:历史日志'BGR=(7,14,64)'"
              f"与本解析同一组像素(3291≈3297)→历史代码变量名 b 实取第2通道,"
              f"通道解析一直正确,红蓝判据-像素对应成立")

        # 方位实测(记录项,不作 FAIL):TF 已声明 REP104(x=车右),但 gazebo
        # 像素 u 方向与 frame 声明无关;2026-09-16 实测蓝像素在画面右(车左),
        # 指向图像等效水平镜像,视觉阶段须做左右对照实验终验后再定判据方向
        bx = sum(i % w for i in idx_b) / len(idx_b)
        rx = sum(i % w for i in idx_r) / len(idx_r)
        print(f"[INFO] 方位实测:蓝像素 x̄={bx:.0f} 红像素 x̄={rx:.0f}"
              f"(差 {rx - bx:+.0f}px;车朝+x 时蓝锥物理在车左/红锥在车右;"
              f"实测方向与之相反=图像镜像疑点,记录待视觉阶段对照终验)")
        return ok1 and ok2

    def check_motion(self):
        print('\n===== [4/4] 运动:停止/倒车/倒车转向 =====')
        results = []

        # W1 前进 2.5s → 停发(看门狗)
        p0 = self.pose
        f, l = self.send_cmd(0.3, 0.0, 2.5)
        self.spin_for(1.2)                       # 停发后让看门狗+惯性收敛
        p1 = self.pose
        d_fwd = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        print(f"[INFO] W1 前进 v=0.3 2.5s:指令窗墙钟[{f:.1f},{l:.1f}] "
              f"位移 {d_fwd:.3f}m")
        results.append(('W1前进', d_fwd > 0.3))
        q0 = self.pose
        self.spin_for(1.5)                       # 再等 1.5s 看是否真停
        q1 = self.pose
        d_drift = math.hypot(q1[0] - q0[0], q1[1] - q0[1])
        print(f"[{'PASS' if d_drift < 0.05 else 'FAIL'}] W1a 停发后漂移 "
              f"{d_drift:.3f}m(判据<0.05,看门狗 0.5s 零速)")
        results.append(('W1a看门狗停车', d_drift < 0.05))

        # W2 显式零速指令
        self.send_zero_once()
        self.spin_for(1.0)
        r0 = self.pose
        self.spin_for(1.0)
        r1 = self.pose
        d_zero = math.hypot(r1[0] - r0[0], r1[1] - r0[1])
        print(f"[{'PASS' if d_zero < 0.02 else 'FAIL'}] W2 零速指令后位移 "
              f"{d_zero:.3f}m(判据<0.02)")
        results.append(('W2零速停车', d_zero < 0.02))

        # W3 倒车 3s
        s0 = self.pose
        f, l = self.send_cmd(-0.3, 0.0, 3.0)
        self.spin_for(0.8)
        s1 = self.pose
        ddx = s1[0] - s0[0]
        print(f"[INFO] W3 倒车 v=-0.3 3s:指令窗墙钟[{f:.1f},{l:.1f}] Δx={ddx:.3f}")
        ok = ddx < -0.5
        print(f"[{'PASS' if ok else 'FAIL'}] W3 倒车 Δx<-0.5")
        results.append(('W3倒车', ok))

        # W4 倒车左转(ω=+0.4)3s:ω 跟踪→Δyaw>0;仍在倒行→Δx<0
        t0 = self.pose
        f, l = self.send_cmd(-0.3, 0.4, 3.0)
        self.spin_for(0.8)
        t1 = self.pose
        dyaw = (t1[2] - t0[2] + math.pi) % (2 * math.pi) - math.pi
        ddx = t1[0] - t0[0]
        print(f"[INFO] W4 倒车左转 v=-0.3 ω=+0.4 3s:指令窗墙钟[{f:.1f},{l:.1f}] "
              f"Δyaw={math.degrees(dyaw):.1f}° Δx={ddx:.3f}")
        ok_yaw = dyaw > 0.3
        ok_x = ddx < -0.1
        print(f"[{'PASS' if ok_yaw else 'FAIL'}] W4a Δyaw>+0.3rad"
              f"(ω 指令跟踪;实际 {dyaw:.3f})")
        print(f"[{'PASS' if ok_x else 'FAIL'}] W4b 仍在倒行 Δx<−0.1(实际 {ddx:.3f})")
        results.append(('W4倒车转向', ok_yaw and ok_x))

        # 收尾:确保停车
        self.send_zero_once()
        self.spin_for(0.5)
        self.send_zero_once()
        return all(ok for _, ok in results)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default='docs/track/trackA_cones.csv',
                    help='冻结 CSV(仅留档,不影响判据)')
    args = ap.parse_args()

    rclpy.init()
    node = Validator(args.csv)
    try:
        if not node.wait_pose():
            print('[FAIL] 90s 未等到 racecar 出现(launch 是否已拉起?)')
            sys.exit(2)
        r1 = node.check_startup()
        r2 = node.check_scan()
        r3 = node.check_image()
        r4 = node.check_motion()
        all_ok = all(r1) and r2 and r3 and r4
        print('\n===== 总结 =====')
        print(f"validator_rc={'0 ALL PASS' if all_ok else '1 FAIL EXISTS'}")
        sys.exit(0 if all_ok else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
