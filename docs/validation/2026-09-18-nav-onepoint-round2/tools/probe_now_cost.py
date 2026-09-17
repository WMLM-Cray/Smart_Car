# 探针 v4: 查当前 TF 位姿处 costmap 代价(中心格+footprint外接圆+5x5邻域)
# 背景: probe_costmap.py 的 PX,PY=0.564,-0.370 是昨晚旧候选硬编码,不反映当前车位置;
#       Nav2 判 start invalid 按 footprint 覆盖区(0.64x0.36m)是否含 inscribed(99)/lethal(100)
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

rclpy.init()
n = Node('probe_now_cost')
got = {}
n.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                      lambda m: got.setdefault('g', m), 10)
tfb = Buffer()
TransformListener(tfb, n, spin_thread=False)

t0 = time.time()
while time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.2)  # 固定spin满:costmap到了也要继续喂TF(DDS发现~4s,提前退出=buffer空)
if 'g' not in got:
    print('NO COSTMAP')
    raise SystemExit(1)

m = got['g']
info = m.info
res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
W, H = info.width, info.height
d = np.array(m.data, dtype=np.int16).reshape(H, W)

tfb.can_transform('map', 'base_footprint', rclpy.time.Time(),
                  timeout=rclpy.duration.Duration(seconds=2))
t = tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time())
bx, by = t.transform.translation.x, t.transform.translation.y
q = t.transform.rotation
byaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.z * q.z + q.y * q.y))

cx, cy = int((bx - ox) // res), int((by - oy) // res)
print(f'TF ({bx:.3f},{by:.3f},{math.degrees(byaw):.1f}deg) -> 格({cx},{cy}) 中心cost={d[cy, cx]}')
print('5x5邻域(上北):')
print(d[max(0, cy - 2):cy + 3, max(0, cx - 2):cx + 3])
# footprint 0.64x0.36 外接圆半径 sqrt(0.32^2+0.18^2)≈0.367m (保守上界,含旋转任意角)
rr = int(0.367 / res)
sub = d[max(0, cy - rr):cy + rr + 1, max(0, cx - rr):cx + rr + 1]
print(f'外接圆r={rr}格: max={sub.max()} lethal100={(sub == 100).sum()}格 inscribed99={(sub == 99).sum()}格')

# 障碍格相对车的距离/方位(车系:前=x+, 左=y+), 判贴身障碍真实距离
ys, xs = np.where(sub >= 99)
if len(ys):
    print('障碍格(车系 dx=前 dy=左, 单位m):')
    for yy, xx in zip(ys, xs):
        gx, gy = max(0, cx - rr) + xx, max(0, cy - rr) + yy
        dx_w, dy_w = (gx + 0.5) * res + ox - bx, (gy + 0.5) * res + oy - by
        c_, s_ = math.cos(byaw), math.sin(byaw)
        dx, dy = c_ * dx_w + s_ * dy_w, -s_ * dx_w + c_ * dy_w
        print(f'  ({gx},{gy}) cost={d[gy, gx]:3d}  前{dx:+.2f} 左{dy:+.2f} 距{math.hypot(dx, dy):.2f}m')
