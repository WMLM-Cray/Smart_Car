#!/usr/bin/env python3
# /map(OccupancyGrid, latched) -> map_server 标准格式 PGM+YAML
# 背景: Humble map_saver_cli 对 latched /map 收帧不稳(Failed to spin/超时);
#       本脚本 transient_local 订阅直接收帧转存, 阈值与 map_saver trinary 默认一致
# 用法: python3 save_map_pgm.py <输出目录> [地图名,默认map]
# PGM 行序: 图像顶行=地图 y 最大行(OccupancyGrid data 首行是 y 最小行, 写盘时翻转)
import sys, os, time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
name = sys.argv[2] if len(sys.argv) > 2 else "map"

rclpy.init()
node = rclpy.create_node("map_saver_py")
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
got = {}


def cb(m):
    got["m"] = m


node.create_subscription(OccupancyGrid, "/map", cb, qos)
t0 = time.time()
while "m" not in got and time.time() - t0 < 30:
    rclpy.spin_once(node, timeout_sec=0.5)
if "m" not in got:
    print("TIMEOUT: 30s 未收到 /map (latched)")
    sys.exit(1)

m = got["m"]
w, h = m.info.width, m.info.height
res = m.info.resolution
ox, oy = m.info.origin.position.x, m.info.origin.position.y

g = np.array(m.data, dtype=np.int16).reshape(h, w)
img = np.full((h, w), 205, dtype=np.uint8)   # 未知/中间 灰
img[g > 65] = 0                              # 占据 黑 (概率>0.65)
img[(g >= 0) & (g < 25)] = 254               # 自由 白 (概率<0.25)
img = img[::-1, :]                           # 翻转到 PGM 图像行序

with open(os.path.join(out_dir, name + ".pgm"), "wb") as f:
    f.write(f"P5\n{w} {h}\n255\n".encode())
    f.write(img.tobytes())
with open(os.path.join(out_dir, name + ".yaml"), "w") as f:
    f.write(f"image: {name}.pgm\n"
            f"resolution: {res}\n"
            f"origin: [{ox}, {oy}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.25\n")
print(f"SAVED {name}: {w}x{h}@{res:.4f} origin=({ox:.3f},{oy:.3f}) -> {out_dir}")
node.destroy_node()
rclpy.shutdown()
