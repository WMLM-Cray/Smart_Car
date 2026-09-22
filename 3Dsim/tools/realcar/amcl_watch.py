#!/usr/bin/env python3
# AMCL 位姿/粒子云采样记录器 (阶段2, 2026-09-17)
# 任务书用途: ①"静止观察>=30s 记录位姿/朝向/协方差变化" ②"粒子云集中到车附近(不能只凭话题存在)"
# 订阅: /amcl_pose (geometry_msgs/PoseWithCovarianceStamped, reliable+volatile)
#       /particle_cloud (nav2_msgs/ParticleCloud — Humble nav2 amcl 实际类型, 非 PoseArray)
# 输出: 每 5s 一行摘要; 结束打印统计块(位置/yaw 波动范围、首末差、协方差对比、粒子散布)
# 用法: python3 amcl_watch.py [--duration 35] [--out 文件路径]
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud

# nav2 amcl 实际发布 QoS(humble 源码口径, 2026-09-17 实车实测):
#   /amcl_pose      = transient_local + reliable (latch: 订阅建立即补发最后一帧)
#   /particle_cloud = SensorDataQoS (best_effort + volatile, 每次滤波更新才发一帧)
# 静止时未达 update_min_d/a 门限 -> 不发新帧; "latch 帧之后 0 新帧" = 静止无漂移的正常证据


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Watch(Node):
    def __init__(self, duration, out_path):
        super().__init__('amcl_watch')
        self.duration = duration
        self.out_path = out_path
        self.out = open(out_path, 'w', encoding='utf-8') if out_path else None
        self.poses = []        # (t, x, y, yaw, cov_x, cov_y, cov_yaw)
        self.particles = []    # (t, n, d_mean, d_max, w_sum)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.on_pose,
                                 QoSProfile(depth=1,
                                            reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(ParticleCloud, '/particle_cloud',
                                 self.on_cloud,
                                 QoSProfile(depth=5,
                                            reliability=ReliabilityPolicy.BEST_EFFORT,
                                            durability=DurabilityPolicy.VOLATILE))
        self.t0 = time.time()
        self.last_print = 0.0

    def say(self, line):
        print(line, flush=True)
        if self.out:
            self.out.write(line + '\n')
            self.out.flush()

    def on_pose(self, m):
        t = time.time() - self.t0
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        c = m.pose.covariance
        self.poses.append((t, p.x, p.y, yaw_of(q), c[0], c[7], c[35]))
        if t - self.last_print >= 5.0:
            self.last_print = t
            r = self.poses[-1]
            self.say(f"[{r[0]:6.1f}s] pose x={r[1]:+.3f} y={r[2]:+.3f} "
                     f"yaw={math.degrees(r[3]):+7.1f}°  cov(x={r[4]:.4f} y={r[5]:.4f} yaw={r[6]:.4f}) "
                     f"n_pose={len(self.poses)} n_cloud={len(self.particles)}")

    def on_cloud(self, m):
        t = time.time() - self.t0
        # nav2 humble ParticleCloud 结构: header + particles[](每个含 pose + weight)
        parts = list(m.particles)
        n = len(parts)
        if n == 0:
            self.particles.append((t, 0, float('nan'), float('nan'), 0.0))
            return
        w = [p.weight for p in parts]
        wsum = sum(w)
        if wsum <= 0:
            w = [1.0 / n] * n
            wsum = 1.0
        mx = sum(wi * p.pose.position.x for wi, p in zip(w, parts)) / wsum
        my = sum(wi * p.pose.position.y for wi, p in zip(w, parts)) / wsum
        ds = [math.hypot(p.pose.position.x - mx, p.pose.position.y - my) for p in parts]
        self.particles.append((t, n, sum(ds) / n, max(ds), wsum))

    def spin(self):
        end = time.time() + self.duration
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.3)
        self.report()

    def report(self):
        self.say("=" * 78)
        self.say(f"采样时长 {self.duration}s: 位姿 {len(self.poses)} 帧, 粒子云 {len(self.particles)} 帧")
        if not self.poses:
            self.say("!! 连 latch 帧都未收到 /amcl_pose (初始位姿未发? AMCL 未 active?) 判 FAIL")
        else:
            n_latch = 1 if self.poses[0][0] < 2.0 else 0   # 订阅建立瞬间补发的最后一帧
            n_new = len(self.poses) - n_latch
            self.say(f"位姿帧构成: latch 补发 {n_latch} 帧 + 采样期新帧 {n_new} 帧")
            if n_new == 0:
                self.say("静止期无新帧: 未达 update_min_d/a 更新门限, AMCL 未重估位姿")
                self.say("  -> 位姿无漂移/无跳变 (静止段以 latch 帧为准) PASS")
            xs = [r[1] for r in self.poses]
            ys = [r[2] for r in self.poses]
            yaws = [r[3] for r in self.poses]
            self.say(f"位姿 x ∈ [{min(xs):+.4f}, {max(xs):+.4f}] (跨度 {max(xs)-min(xs):.4f} m)")
            self.say(f"位姿 y ∈ [{min(ys):+.4f}, {max(ys):+.4f}] (跨度 {max(ys)-min(ys):.4f} m)")
            self.say(f"yaw  跨度 {math.degrees(max(yaws)-min(yaws)):.2f}°")
            f, l = self.poses[0], self.poses[-1]
            self.say(f"首帧 ({f[0]:.1f}s): x={f[1]:+.4f} y={f[2]:+.4f} yaw={math.degrees(f[3]):+.1f}° "
                     f"cov(x={f[4]:.4f} y={f[5]:.4f} yaw={f[6]:.4f})")
            self.say(f"末帧 ({l[0]:.1f}s): x={l[1]:+.4f} y={l[2]:+.4f} yaw={math.degrees(l[3]):+.1f}° "
                     f"cov(x={l[4]:.4f} y={l[5]:.4f} yaw={l[6]:.4f})")
            self.say(f"首->末 位移差 dx={l[1]-f[1]:+.4f} dy={l[2]-f[2]:+.4f} "
                     f"dyaw={math.degrees(l[3]-f[3]):+.2f}°")
            finite = all(math.isfinite(v) for r in self.poses for v in r[1:4])
            self.say(f"数值有限性: {'全有限 PASS' if finite else '存在 NaN/inf 判 FAIL'}")
        if not self.particles:
            self.say("!! 未收到 /particle_cloud (volatile, 仅滤波更新瞬间发布)")
            self.say("   静止无新帧属正常; 粒子云集中度证据需在重发 /initialpose 触发更新时采集")
        else:
            import statistics
            ns = [p[1] for p in self.particles]
            dms = [p[2] for p in self.particles if not math.isnan(p[2])]
            dxs = [p[3] for p in self.particles if not math.isnan(p[3])]
            self.say(f"粒子数 n ∈ [{min(ns)}, {max(ns)}]")
            if dms:
                self.say(f"粒子散布(距加权均值): mean 均值={statistics.mean(dms):.3f} m "
                         f"末值={dms[-1]:.3f} m; max 距离末值={dxs[-1]:.3f} m")
        self.say("=" * 78)
        if self.out:
            self.out.close()
            print(f"输出已存: {self.out_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=35.0)
    ap.add_argument('--out', default='')
    args = ap.parse_args()
    rclpy.init()
    node = Watch(args.duration, args.out)
    try:
        node.spin()
    except KeyboardInterrupt:
        node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
