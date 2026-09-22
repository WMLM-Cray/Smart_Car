# 探针 v3: 渲染 global costmap + 当前激光投影 + TF 位姿, 存图排查起点 lethal 来源
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

PX, PY = 0.564, -0.370  # initpose 期望位姿(仅参考线)

rclpy.init()
n = Node('probe_costmap3')
got = {}


def cb_map(m):
    got.setdefault('g', m)


def cb_scan(m):
    got.setdefault('s', m)


n.create_subscription(OccupancyGrid, '/global_costmap/costmap', cb_map, 10)
n.create_subscription(LaserScan, '/scan_filtered', cb_scan, qos_profile_sensor_data)

# TF: map -> base_footprint (costmap 投影激光用的就是它)
from tf2_ros import Buffer, TransformListener
tfb = Buffer()
TransformListener(tfb, n, spin_thread=False)

t0 = time.time()
while time.time() - t0 < 12 and ('g' not in got or 's' not in got):
    rclpy.spin_once(n, timeout_sec=0.2)

if 'g' not in got:
    print('NO COSTMAP')
    raise SystemExit(1)

m = got['g']
info = m.info
res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
W, H = info.width, info.height
d = np.array(m.data, dtype=np.int16).reshape(H, W)
print(f'size={W}x{H} 起点(0.564,-0.370) cost={d[int((-0.370-oy)//res), int((0.564-ox)//res)]}')

# TF 位姿
bx = by = byaw = None
try:
    tfb.can_transform('map', 'base_footprint', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2))
    t = tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time())
    bx = t.transform.translation.x
    by = t.transform.translation.y
    q = t.transform.rotation
    byaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.z * q.z + q.y * q.y))
    print(f'TF map->base: ({bx:.3f},{by:.3f},{math.degrees(byaw):.1f}deg)')
except Exception as e:
    print('TF fail:', e)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

fig, ax = plt.subplots(figsize=(9, 7.5))
# OccupancyGrid: row0=y最低 -> imshow origin=lower
cmap = matplotlib.colors.ListedColormap(['#111', '#c8c8c8', '#ffd700', 'red'])
norm = matplotlib.colors.BoundaryNorm([-0.5, 1, 95, 99.5, 100.5], cmap.N)
show = np.ma.masked_inside(d, 1, 1)  # placeholder
disp = d.copy().astype(float)
disp[disp == 90] = 2
disp[(disp == 99) | (disp == 100)] = np.where(disp[(disp == 99) | (disp == 100)] == 99, 3, 4)[0] if False else disp[(disp == 99) | (disp == 100)]
disp[~np.isin(disp, [90, 99, 100])] = 1
disp[d == 0] = 0
disp[d == 90] = 2
disp[d == 99] = 3
disp[d == 100] = 4
ax.imshow(disp, cmap=cmap, norm=norm, extent=[ox, ox + W * res, oy, oy + H * res],
          origin='lower', interpolation='nearest')

# 激光投影 (按 TF 位姿; 无 TF 用期望位姿)
px, py, pyaw = (bx, by, byaw) if bx is not None else (PX, PY, 0.0)
if 's' in got:
    s = got['s']
    r = np.array([v if v == v and v != float('inf') and v > 0.15 else np.nan for v in s.ranges])
    a = s.angle_min + np.arange(len(r)) * s.angle_increment
    lx, ly = 0.07 + r * np.cos(a), r * np.sin(a)
    c_, s_ = math.cos(pyaw), math.sin(pyaw)
    wx, wy = px + c_ * lx - s_ * ly, py + s_ * lx + c_ * ly
    ax.plot(wx, wy, 'c.', markersize=1.2, alpha=0.8)
    print(f'激光有效点 {int(np.isfinite(r).sum())} 个已叠加(青色)')

ax.plot(PX, PY, 'g^', markersize=12, label='initpose expect')
if bx is not None:
    ax.plot(bx, by, 'w*', markersize=14, label='TF base')
ax.plot(-0.39, -0.07, 'b+', markersize=14, mew=2, label='goal')
ax.legend(loc='upper right')
ax.set_title('global costmap: black=free gray=90 yellow=99 red=100 cyan=live scan')
fig.savefig('/tmp/costmap_vis.png', dpi=110, bbox_inches='tight')
print('saved /tmp/costmap_vis.png')
