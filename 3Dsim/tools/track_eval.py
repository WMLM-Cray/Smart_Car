#!/usr/bin/env python3
"""在 Gazebo 里给一次跑圈打分:圈数、撞锥、出界、圈速。

判据全部来自 /model_states(gazebo_ros_state 插件),不依赖 Gazebo 的接触传感器 ——
track_generator 出的锥桶是静态 box,没装接触传感器,碰撞只能靠几何判。

  撞锥  车辆外接圆与锥桶 box 的水平距离 < 阈值。锥桶 0.28m 见方,车按 params.py
        的 length/width 取半对角。同一个锥桶连续接触只算一次,离开 0.3m 后才重新计。
  出界  到最近锥桶的距离超过 OFF_TRACK —— 赛道宽 1.5m,正常行驶到边界不超过 0.75m,
        超过 1.6m 基本只能是冲出去了。
  圈数  过起点线(start_pose 处垂直于起跑方向的一条线)且方向正确。

用法(仿真跑起来之后,另开一个终端):
    python3 3Dsim/tools/track_eval.py trackA_layout.yaml --seconds 180
"""
import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
import yaml
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node

CONE_HALF = 0.14        # 锥桶代理 box 0.28m 见方
HIT_CLEAR = 0.30        # 离开这个距离才允许把同一个锥桶再记一次
OFF_TRACK = 1.60        # 到最近锥桶超过这个距离算出界(赛道半宽 0.75)
LAP_BAND = 1.0          # 过线判定:离起跑线中心这么近才算


class Eval(Node):
    def __init__(self, cones, start, car_radius, seconds):
        super().__init__('track_eval')
        self.cones = cones                  # [(id, x, y)]
        self.sx, self.sy, self.syaw = start
        self.hit_dist = car_radius + CONE_HALF
        self.deadline = time.time() + seconds

        self.lock = threading.Lock()
        self.hits, self.touching = {}, set()
        self.off_track = 0
        self.laps, self.lap_times = 0, []
        self.t0 = None
        self.last_side = None
        self.pos = None
        self.path = []
        self.max_off = 0.0
        self.create_subscription(ModelStates, '/model_states', self.on_states, 10)

    def on_states(self, m):
        try:
            i = m.name.index('racecar')
        except ValueError:
            return
        p = m.pose[i].position
        now = time.time()
        with self.lock:
            if self.t0 is None:
                self.t0 = now
            self.pos = (p.x, p.y)
            self.path.append((round(now - self.t0, 2), round(p.x, 3), round(p.y, 3)))
            self._score(p.x, p.y, now)

    def _score(self, x, y, now):
        nearest = min(math.hypot(x - cx, y - cy) for _, cx, cy in self.cones)
        self.max_off = max(self.max_off, nearest)
        if nearest > OFF_TRACK:
            self.off_track += 1

        for cid, cx, cy in self.cones:
            d = math.hypot(x - cx, y - cy)
            if d < self.hit_dist and cid not in self.touching:
                self.touching.add(cid)
                self.hits[cid] = self.hits.get(cid, 0) + 1
            elif d > self.hit_dist + HIT_CLEAR:
                self.touching.discard(cid)

        # 过线:把位置投到起跑方向上,符号翻转且横向距离够近 = 过了一次
        dx, dy = x - self.sx, y - self.sy
        ahead = dx * math.cos(self.syaw) + dy * math.sin(self.syaw)
        side = -dx * math.sin(self.syaw) + dy * math.cos(self.syaw)
        cur = ahead >= 0
        if (self.last_side is not None and cur and not self.last_side
                and abs(side) < LAP_BAND):
            self.laps += 1
            self.lap_times.append(round(now - self.t0, 2))
        self.last_side = cur

    def report(self):
        with self.lock:
            splits = [self.lap_times[0]] + [round(b - a, 2) for a, b in
                                            zip(self.lap_times, self.lap_times[1:])] \
                     if self.lap_times else []
            return {
                'laps': self.laps,
                'lap_splits_s': splits,
                'cone_hits': sum(self.hits.values()),
                'cones_hit_ids': sorted(self.hits),
                'off_track_samples': self.off_track,
                'max_dist_to_nearest_cone_m': round(self.max_off, 3),
                'elapsed_s': round((time.time() - self.t0), 1) if self.t0 else 0.0,
                'final_xy': [round(v, 3) for v in self.pos] if self.pos else None,
                'path': self.path[::4],      # 抽稀,画轨迹用
            }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('layout', type=Path)
    ap.add_argument('--cones', type=Path, help='锥桶 CSV(默认用 layout 里的)')
    ap.add_argument('--seconds', type=float, default=180.0)
    ap.add_argument('--out', type=Path)
    args = ap.parse_args()

    layout = yaml.safe_load(args.layout.read_text(encoding='utf-8'))
    if args.cones:
        import csv
        rows = list(csv.DictReader(args.cones.open(encoding='utf-8-sig')))
        cones = [(r['id'], float(r['x']), float(r['y'])) for r in rows]
    else:
        cones = [(c['id'], c['x'], c['y']) for c in layout['cones']]
    sp = layout['start_pose']
    start = (sp['xy'][0], sp['xy'][1], math.radians(sp['yaw_deg']))

    # 往上找 fastsim:仓库里这个脚本在 3Dsim/tools/,同步到车/WSL 后可能在别处。
    here = Path(__file__).resolve()
    for root in [here.parent, *here.parents, Path.cwd()]:
        if (root / 'fastsim').is_dir():
            sys.path.insert(0, str(root))
            break
    else:
        raise SystemExit('找不到 fastsim/,在仓库根目录或 ~/autocar 下运行')
    from fastsim.params import CarParams
    p = CarParams()
    # /model_states 给的是 base_footprint(后轴)的位姿,不是车的几何中心 ——
    # 用外接圆半径 hypot(length,width)/2 会短算 0.1m,车头顶着锥桶都判不出撞。
    # 取后轴到车头的距离,这是最长的那一边。
    car_radius = math.hypot(p.length - p.rear_overhang, p.width / 2)

    rclpy.init()
    node = Eval(cones, start, car_radius, args.seconds)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print(f'赛道 {layout["track"]},{len(cones)} 个锥桶,起点 {start[:2]},'
          f'撞锥阈值 {node.hit_dist:.2f}m,跑 {args.seconds:.0f}s', flush=True)
    try:
        while time.time() < node.deadline:
            time.sleep(2)
            r = node.report()
            print(f'  t={r["elapsed_s"]:6.1f}s  圈{r["laps"]}  撞{r["cone_hits"]}  '
                  f'出界{r["off_track_samples"]}  xy={r["final_xy"]}', flush=True)
    except KeyboardInterrupt:
        pass
    r = node.report()
    print(json.dumps(r, ensure_ascii=False, indent=2), flush=True)
    if args.out:
        args.out.write_text(json.dumps(r, ensure_ascii=False, indent=2), encoding='utf-8')
    import os
    sys.stdout.flush()   # os._exit 不 flush,不加这句最后的报告会丢
    os._exit(0)          # rclpy.shutdown 会和 spin 线程抢,C++ 层直接 abort


if __name__ == '__main__':
    main()
