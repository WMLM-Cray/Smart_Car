#!/usr/bin/env python3
# 车机本机扫图监控渲染: /map + /scan + 轨迹 -> PNG (SSH 拉取显示用)
# 2026-09-17 阶段1; 与 slam_scan.sh 同用; 只读订阅, 不动控制链
# 退出: SIGINT/SIGTERM (自接管, 避免 rclpy 默认 handler 先 shutdown 的坑 6bb97d8)
import os, signal, sys, time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros

OUT = os.path.expanduser("~/racecar_team/logs/slam/monitor.png")
TMP = OUT + ".tmp.png"


class Monitor(Node):
    def __init__(self):
        super().__init__("slam_map_monitor")
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                              durability=DurabilityPolicy.VOLATILE,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_subscription(LaserScan, "/scan", self.on_scan, scan_qos)
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)
        self.map = None
        self.scan_pts = None
        self.trail = []

    def on_map(self, msg):
        self.map = msg

    def on_scan(self, msg):
        r = np.array(msg.ranges)
        a = np.arange(len(r)) * msg.angle_increment + msg.angle_min
        m = np.isfinite(r) & (r > msg.range_min) & (r < 8.0)
        try:
            t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            tyaw = 2.0 * np.arctan2(t.transform.rotation.z, t.transform.rotation.w)
        except Exception:
            return  # TF 未就绪跳过本帧
        x = r[m] * np.cos(a[m])
        y = r[m] * np.sin(a[m])
        self.scan_pts = (tx + x * np.cos(tyaw) - y * np.sin(tyaw),
                         ty + x * np.sin(tyaw) + y * np.cos(tyaw))
        self.trail.append((tx, ty))


def render(mon):
    fig, ax = plt.subplots(figsize=(8, 8), dpi=100)
    if mon.map is not None:
        m = mon.map
        w, h = m.info.width, m.info.height
        grid = np.array(m.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w), 205, dtype=np.uint8)          # 未知 浅灰
        img[grid > 50] = 0                                  # 占据 黑
        img[(grid >= 0) & (grid <= 50)] = 254               # 自由 白
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        ax.imshow(img, cmap="gray", origin="lower",
                  extent=[ox, ox + w * res, oy, oy + h * res])
    if mon.trail:
        ax.plot([p[0] for p in mon.trail], [p[1] for p in mon.trail], "-b", lw=1, label="trail")
    if mon.scan_pts is not None:
        ax.plot(mon.scan_pts[0], mon.scan_pts[1], ".r", ms=2, label="scan")
    if mon.trail:
        ax.plot(mon.trail[-1][0], mon.trail[-1][1], "^g", ms=12, label="car")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("SLAM map monitor  (map + scan + trail)")
    ax.legend(loc="upper right")
    ax.set_aspect("equal")
    fig.savefig(TMP, bbox_inches="tight")
    plt.close(fig)
    os.replace(TMP, OUT)   # 原子替换, 拉图不会拉到半张


def main():
    rclpy.init()
    mon = Monitor()
    running = {"v": True}

    def on_sig(sig, frame):
        running["v"] = False

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)
    last = 0.0
    while running["v"]:
        rclpy.spin_once(mon, timeout_sec=0.2)
        now = time.time()
        if now - last > 2.0:
            try:
                render(mon)
            except Exception as e:
                print("render err:", e, file=sys.stderr)
            last = now
    mon.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
