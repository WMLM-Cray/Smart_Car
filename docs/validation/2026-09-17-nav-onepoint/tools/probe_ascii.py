# ASCII 渲染 global costmap: # =100(lethal) +=99(inscribed) ·=90 .=0(自由)
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

rclpy.init()
n = Node('probe_ascii')
got = {}
n.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                      lambda m: got.setdefault('g', m), 10)
t0 = time.time()
while 'g' not in got and time.time() - t0 < 12:
    rclpy.spin_once(n, timeout_sec=0.2)
if 'g' not in got:
    print('NO COSTMAP')
    raise SystemExit(1)

m = got['g']
info = m.info
res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
W, H = info.width, info.height
d = np.array(m.data, dtype=np.int16).reshape(H, W)

# 2x2 降采样: 优先显示高代价
out = []
for ry in range(0, H, 2):
    line = []
    for rx in range(0, W, 2):
        blk = d[ry:ry+2, rx:rx+2]
        if (blk == 100).any():
            ch = '#'
        elif (blk == 99).any():
            ch = '+'
        elif (blk == 90).any():
            ch = ':'
        elif (blk == 0).any():
            ch = '.'
        else:
            ch = ' '
        line.append(ch)
    out.append(''.join(line))

# 起点 (0.564,-0.370) / 目标 (-0.39,-0.07) 转到降采样坐标 (col=x, row: row0=y低)
sx = int((0.564 - ox) / res) // 2
sy = H - 1 - int((-0.370 - oy) / res)  # y低->行底, 翻转到数组行序(row0=y高? out里row0=H-2..H,即y高)
# out 的 row0 对应 d 的 row0..1 = y 最低; 打印时直接 row0=y低
sx = int((0.564 - ox) / res) // 2
sy = int((-0.370 - oy) / res) // 2
gx = int((-0.39 - ox) / res) // 2
gy = int((-0.07 - oy) / res) // 2

print('图例: #=100lethal +=99inscribed :=90膨胀 .=0自由  S=起点 G=目标')
print(f'格宽0.1m  总尺寸 {W}x{H}@{res}')
for r_i, line in enumerate(out):
    if r_i == sy and sx < len(line):
        line = line[:sx] + 'S' + line[sx+1:]
    if r_i == gy and gx < len(line):
        line = line[:gx] + 'G' + line[gx+1:]
    print(f'{r_i:2d} {line}')
