#!/usr/bin/env python3
"""在 Gazebo 里给一次跑圈打分:圈数、撞锥、出界、圈速。

判据全部来自 /model_states(gazebo_ros_state 插件),不依赖 Gazebo 的接触传感器 ——
track_generator 出的锥桶是静态 box,没装接触传感器,碰撞只能靠几何判。

  撞锥  车辆外接圆与锥桶 box 的水平距离 < 阈值。锥桶 0.28m 见方,车按 params.py
        的 length/width 取半对角。同一个锥桶连续接触只算一次,离开 0.3m 后才重新计。
  出界  到最近锥桶的距离超过 OFF_TRACK —— 赛道宽 1.5m,正常行驶到边界不超过 0.75m,
        超过 1.6m 基本只能是冲出去了。
  圈数  过起点线(start_pose 处垂直于起跑方向的一条线)且方向正确。

计时口径(2026-09-25 改):主时间轴是 **仿真时间**(/clock),不是墙钟。
  * 为什么:并行验收时 N 个 worker 抢 CPU,墙钟会被拖长,同一策略在不同 worker
    上的圈速就不可比。仿真时间是物理时间,和 CPU 负载无关,这才可比。
  * 怎么拿:/clock(rosgraph_msgs/msg/Clock,由 libgazebo_ros_init.so 发布)。
    QoS 必须 BEST_EFFORT —— 发布端是 rclcpp::ClockQoS(BEST_EFFORT/VOLATILE/depth1),
    RELIABLE 订阅者和它不兼容,会静默收不到(不报错)。
  * 回退:万一 /clock 没起来(世界缺 gazebo_ros_init),自动退回墙钟并在
    clock_source 字段里写明,不会静默算错。
  * 同时保留墙钟:lap_splits_wall_s / elapsed_wall_s 是诊断值,不参与判据。

  运行预算 --seconds 是**仿真秒**(无 /clock 时退化为墙钟秒);--max-wall-factor
  给一个墙钟硬上限,防止 gzserver 卡死时 worker 永久挂着。

用法(仿真跑起来之后,另开一个终端):
    python3 simulation/gazebo/track_eval.py runs/gz_official/track_A/layout.yaml --seconds 180
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
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosgraph_msgs.msg import Clock

CONE_HALF = 0.14        # 锥桶代理 box 0.28m 见方
HIT_CLEAR = 0.30        # 离开这个距离才允许把同一个锥桶再记一次
OFF_TRACK = 1.60        # 到最近锥桶超过这个距离算出界(赛道半宽 0.75)
LAP_BAND = 1.0          # 过线判定:离起跑线中心这么近才算
CLOCK_STALE = 2.0       # /clock 超过这么多墙钟秒没更新,视为时钟已死
CLOCK_WAIT = 10.0       # 等 /clock 的墙钟秒数,超时退回墙钟


class Eval(Node):
    def __init__(self, cones, start, car_length, car_width, seconds,
                 max_wall=None):
        super().__init__('track_eval')
        self.cones = cones                  # [(id, x, y)]
        self.sx, self.sy, self.syaw = start
        self.car_half_length = car_length / 2.0
        self.car_half_width = car_width / 2.0
        # 时间预算:seconds 是主时钟(仿真秒)的预算;max_wall 是墙钟硬上限,
        # 防止 gzserver 卡死时 worker 永久挂着(仿真时间冻结时主时钟走不动)。
        self.seconds = float(seconds)
        self.max_wall = float(max_wall) if max_wall else None
        self.created_wall = time.time()

        self.lock = threading.Lock()
        self.hits, self.touching = {}, set()
        self.off_track = 0
        self.laps, self.lap_times = 0, []
        self.lap_times_wall = []
        # t0 是主时钟起点(仿真秒),t0_wall 是同一次采样的墙钟。
        self.t0 = None
        self.t0_wall = None
        self.clock_source = None            # 'sim' / 'wall' / None(未锚定)
        self.sim_t = None                   # 最近一条 /clock 的仿真时刻(s)
        self.sim_wall_recv = 0.0            # 收到它的墙钟时刻
        self.last_side = None
        self.pos = None
        self.path = []
        self.max_off = 0.0
        # /clock 的发布端是 rclcpp::ClockQoS(BEST_EFFORT/VOLATILE/depth1)。
        # 订阅端 QoS 必须兼容,否则静默收不到 —— 不会报错,只会永远走墙钟回退。
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Clock, '/clock', self.on_clock, clock_qos)
        self.create_subscription(ModelStates, '/model_states', self.on_states, 10)

    def on_clock(self, m):
        """仿真时钟。gazebo_ros_init 以最高频率刷,这里只记最新值。"""
        with self.lock:
            self.sim_t = m.clock.sec + m.clock.nanosec * 1e-9
            self.sim_wall_recv = time.time()

    def _clock_live(self, wall_now):
        """仿真时钟是否新鲜。超过 CLOCK_STALE 墙钟秒没更新视为已死。"""
        return (self.sim_t is not None
                and (wall_now - self.sim_wall_recv) <= CLOCK_STALE)

    def on_states(self, m):
        try:
            i = m.name.index('racecar')
        except ValueError:
            return
        p = m.pose[i].position
        q = m.pose[i].orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        wall_now = time.time()
        with self.lock:
            if self.t0 is None:
                # 锚定主时钟。优先仿真时间;等 CLOCK_WAIT 秒还没有 /clock 就
                # 退回墙钟(世界缺 gazebo_ros_init 时),并写明来源,不静默算错。
                if self._clock_live(wall_now):
                    self.clock_source = 'sim'
                    self.t0 = self.sim_t
                    self.t0_wall = wall_now
                elif wall_now - self.created_wall >= CLOCK_WAIT:
                    self.clock_source = 'wall'
                    self.t0 = wall_now
                    self.t0_wall = wall_now
                    self.get_logger().warn(
                        '/clock 超时未到,退回墙钟计时;并行时圈速不可比')
                else:
                    return          # 还没等到时钟,先不计时
            # sim 模式下即使 /clock 后来停了也用最后已知值(时间冻结),
            # 绝不混用墙钟 —— 混用会在圈速里制造假跳变。终止交给 max_wall。
            now = self.sim_t if self.clock_source == 'sim' else wall_now
            self.pos = (p.x, p.y)
            self._yaw = yaw
            self.path.append((round(now - self.t0, 2), round(p.x, 3),
                              round(p.y, 3)))
            self._score(p.x, p.y, now, wall_now)

    def _score(self, x, y, now, wall_now):
        nearest = min(math.hypot(x - cx, y - cy) for _, cx, cy in self.cones)
        self.max_off = max(self.max_off, nearest)
        if nearest > OFF_TRACK:
            self.off_track += 1

        yaw = getattr(self, '_yaw', 0.0)
        for cid, cx, cy in self.cones:
            hit = self._rect_hits_box(
                x, y, yaw, cx, cy, self.car_half_length,
                self.car_half_width, CONE_HALF
            )
            if hit and cid not in self.touching:
                self.touching.add(cid)
                self.hits[cid] = self.hits.get(cid, 0) + 1
            elif not hit and math.hypot(x - cx, y - cy) > HIT_CLEAR:
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
            self.lap_times_wall.append(round(wall_now - self.t0_wall, 2))
        self.last_side = cur

    def finished(self):
        """主循环退出条件。主时钟到点,或墙钟超过硬上限。"""
        wall_now = time.time()
        with self.lock:
            if self.t0 is None:
                # 一直没锚定(比如车压根没 spawn):给足等待时间后收工
                return wall_now - self.created_wall > self.seconds + CLOCK_WAIT
            now = self.sim_t if self.clock_source == 'sim' else wall_now
            if now - self.t0 >= self.seconds:
                return True
            if self.max_wall and wall_now - self.t0_wall >= self.max_wall:
                return True
            return False

    def report(self):
        wall_now = time.time()
        with self.lock:
            splits = [self.lap_times[0]] + [round(b - a, 2) for a, b in
                                            zip(self.lap_times, self.lap_times[1:])] \
                     if self.lap_times else []
            splits_wall = [self.lap_times_wall[0]] + \
                [round(b - a, 2) for a, b in
                 zip(self.lap_times_wall, self.lap_times_wall[1:])] \
                if self.lap_times_wall else []
            if self.t0 is None:
                elapsed = elapsed_wall = 0.0
            else:
                now = self.sim_t if self.clock_source == 'sim' else wall_now
                elapsed = round(now - self.t0, 1)
                elapsed_wall = round(wall_now - self.t0_wall, 1)
            return {
                'laps': self.laps,
                # 主判据:仿真时间。并行时与 CPU 争抢无关,可比。
                'lap_splits_s': splits,
                # 诊断:墙钟。只用来发现"仿真时间对但墙钟很长"这类异常,不参与判据。
                'lap_splits_wall_s': splits_wall,
                'cone_hits': sum(self.hits.values()),
                'cones_hit_ids': sorted(self.hits),
                'off_track_samples': self.off_track,
                'max_dist_to_nearest_cone_m': round(self.max_off, 3),
                'elapsed_s': elapsed,
                'elapsed_wall_s': elapsed_wall,
                'clock_source': self.clock_source or 'unknown',
                'final_xy': [round(v, 3) for v in self.pos] if self.pos else None,
                'path': self.path[::4],      # 抽稀,画轨迹用
            }

    @staticmethod
    def _rect_hits_box(x, y, yaw, cx, cy, hx, hy, ex):
        """SAT intersection of the yawed chassis box and a world-axis box."""
        ux, uy = math.cos(yaw), math.sin(yaw)
        vx, vy = -uy, ux
        dx, dy = cx - x, cy - y
        for ax, ay in ((ux, uy), (vx, vy), (1.0, 0.0), (0.0, 1.0)):
            centre = abs(dx * ax + dy * ay)
            car_radius = hx * abs(ux * ax + uy * ay) + \
                hy * abs(vx * ax + vy * ay)
            cone_radius = ex * (abs(ax) + abs(ay))
            if centre > car_radius + cone_radius:
                return False
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('layout', type=Path)
    ap.add_argument('--cones', type=Path, help='锥桶 CSV(默认用 layout 里的)')
    ap.add_argument('--seconds', type=float, default=180.0,
                    help='主时钟(仿真秒)预算;无 /clock 时退化为墙钟秒')
    ap.add_argument('--max-wall-factor', type=float, default=3.0,
                    help='墙钟硬上限 = seconds x 这个系数,防止 gzserver 卡死时永久挂着')
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

    # 往上找仓库根:本脚本在 simulation/gazebo/ 下,同步到车/WSL 后位置可能变。
    here = Path(__file__).resolve()
    for root in [here.parent, *here.parents, Path.cwd()]:
        if (root / 'common' / 'params.py').is_file():
            sys.path.insert(0, str(root))
            break
    else:
        raise SystemExit('找不到 common/params.py,在仓库根目录下运行')
    from common.params import CarParams
    p = CarParams()
    rclpy.init()
    node = Eval(cones, start, p.length, p.width, args.seconds,
                max_wall=args.seconds * args.max_wall_factor)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print(f'赛道 {layout["track"]},{len(cones)} 个锥桶,起点 {start[:2]},'
          f'车体矩形 {p.length:.2f}x{p.width:.2f}m,跑 {args.seconds:.0f}s',
          flush=True)
    try:
        while not node.finished():
            time.sleep(2)
            r = node.report()
            print(f'  t={r["elapsed_s"]:6.1f}s  圈{r["laps"]}  撞{r["cone_hits"]}  '
                  f'出界{r["off_track_samples"]}  xy={r["final_xy"]}', flush=True)
    except KeyboardInterrupt:
        pass
    r = node.report()
    if r['clock_source'] != 'sim':
        print(f'警告:计时源是 {r["clock_source"]},不是仿真时钟 —— '
              f'并行时圈速不可比', flush=True)
    print(json.dumps(r, ensure_ascii=False, indent=2), flush=True)
    if args.out:
        args.out.write_text(json.dumps(r, ensure_ascii=False, indent=2), encoding='utf-8')
    import os
    sys.stdout.flush()   # os._exit 不 flush,不加这句最后的报告会丢
    os._exit(0)          # rclpy.shutdown 会和 spin 线程抢,C++ 层直接 abort


if __name__ == '__main__':
    main()
