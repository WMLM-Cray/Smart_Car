#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[仿真测试驾驶器] tools/trackA/test_driver.py
=================================================
身份声明:本节点是【仿真测试驾驶器】——只在仿真中沿预先算好的车道内
参考路线低速行驶,用于驱动 SLAM/定位/导航验收。它不是自主探索,
不是 Nav2,不具备感知避障能力;轨迹跟踪反馈使用 Gazebo 真值
(/model_states),该真值仅用于驾驶器跟踪与验收记录,不进入 SLAM 链路
(SLAM 只用模拟传感器 /scan 与正常 TF)。

任务书硬性要求(逐条落实):
 1. 参考路线由冻结赛道几何生成,且必须校验【整车轮廓】通行余量——
    车辆按 0.56×0.35m 矩形(历史冻结基线的轮距0.30+轮宽
    0.05 外缘=0.35 不超底盘),锥桶按 0.28×0.28 方底座外接圆 r=0.198,
    路线上每 0.05m 采样点对全部 63 锥逐一计算"矩形-圆"距离;
 2. 速度/转向满足阿克曼约束:|ω| ≤ v·tan(0.55°)/0.33(冻结参数),
    低速 0.3 m/s;
 3. 分段扩展:--route short|u_right|half|full,先短后长;
 4. 失速/碰撞/偏出 → 立即停车并报告,不硬闯;
 5. 启动时校验 /cmd_vel 无其他发布者(键盘/手柄/Nav2 速度源必须离线),
    运行中周期复检,发现冲突即停;
 6. 路线常量逐点对照冻结 CSV(docs/track/trackA_cones.csv)推导,
    锥桶坐标零改动。

参考路线几何(推导记录见 docs/track/trackA_阶段3_2_测试驾驶器.md):
  底道直 y=0.75(x 11.0→14.03) → 右U弧 c=(14.03,2.75) R=2.0
  (270°→90° 逆时针) → 顶右直 y=4.75(x 14.03→8.41) → 锥桶V谷折线
  (8.41,4.75)→(5.578,3.578)→(2.75,4.75) → 左U复合弯(切角小弧
  c=(1.75,3.75)R1.0 → x=0.75 直线 → 左下大弧 c=(2.75,2.75)R2.0) →
  底道直 y=0.75(x 2.75→11.0) 回起点。
用法:
  python3 tools/trackA/test_driver.py --route short [--speed 0.3]
      [--cones docs/track/trackA_cones.csv]
      [--report docs/track/test_driver_<route>_report.json]
"""
import argparse
import csv
import json
import math
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

TAG = "[仿真测试驾驶器]"

# ---- 冻结几何常量(与 params.yaml / trackA_cones.csv 同源) ----
VEH_HALF_L = 0.28          # 底盘长 0.56 之半
VEH_HALF_W = 0.175         # 底盘宽 0.35 之半(轮外缘同宽)
CONE_R = 0.198             # 锥底座 0.28×0.28 方盒的外接圆半径(保守)
MARGIN = 0.10              # 静态余量要求(锥沿 + 0.10m)
REQ_CLEAR = CONE_R + MARGIN
ABORT_CLEAR = CONE_R + 0.03   # 运行中触碰锥沿即判碰撞停车
WHEELBASE = 0.33           # params.yaml wheel_base
MAX_STEER = 0.55           # params.yaml max_steer_angle

BOT_Y, TOP_Y = 0.75, 4.75
RX, RY, R = 14.03, 2.75, 2.0      # 右U弧(内环R1.25/外环R2.75 的同心中线)
# 左U弯不能单圆弧:内岛是直角块(左上角锥 I8=(1.5,4.0) 距 (2.75,2.75) 圆心
# R2 圆弧仅 0.23m,整车轮廓过不去)。按走廊中线改为三段复合:
#   ① 左上切角弧 c=(1.75,3.75) R=1.0(90°→180°,同时避开外角(0,5.5)与 I8)
#   ② x=0.75 直线下行(左墙 x=0 与内岛左缘 x=1.5 的中线,走廊恰 1.5m 宽)
#   ③ 左下大弧 c=(2.75,2.75) R=2.0(180°→270°,与外缘 E10 弧 R2.744 同心)
TL_C, TL_R = (1.75, 3.75), 1.0
BL_C, BL_R = (2.75, 2.75), 2.0
V_NODES = [                       # 锥桶V段中线(相邻锥对中点折线)
    (8.410, 4.750),
    (7.4385, 4.4625),
    (6.5465, 3.8665),
    (5.578, 3.578),
    (4.6105, 3.8665),
    (3.718, 4.4625),
    (2.750, 4.750),
]
START = (11.0, 0.75)              # 出生点(与验收基线一致)


def seg_line(p0, p1, step=0.05):
    n = max(2, int(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / step) + 1)
    return [(p0[0] + (p1[0] - p0[0]) * i / (n - 1),
             p0[1] + (p1[1] - p0[1]) * i / (n - 1)) for i in range(n)]


def seg_arc(cx, cy, r, a0, a1, step=0.05):
    n = max(2, int(abs(a1 - a0) * r / step) + 1)
    return [(cx + r * math.cos(a0 + (a1 - a0) * i / (n - 1)),
             cy + r * math.sin(a0 + (a1 - a0) * i / (n - 1))) for i in range(n)]


def build_route(name):
    """返回路线折线点列(起点→终点,逆时针)。"""
    if name == "short":       # 第1段:底道一小段 2m
        pts = seg_line(START, (13.0, BOT_Y))
    elif name == "u_right":   # 第2段:底道全 + 右U弯,上到顶道
        pts = seg_line(START, (RX, BOT_Y))[:-1] + \
              seg_arc(RX, RY, R, -math.pi / 2, math.pi / 2)[1:]
    elif name == "half":      # 第3段:+顶右直 + V谷 + 左U复合弯,下回底道西端
        pts = seg_line(START, (RX, BOT_Y))[:-1] + \
              seg_arc(RX, RY, R, -math.pi / 2, math.pi / 2)[1:-1] + \
              seg_line((RX, TOP_Y), V_NODES[0])[1:-1]
        for a, b in zip(V_NODES, V_NODES[1:]):
            pts += seg_line(a, b)[1:-1]
        pts += seg_line(V_NODES[-1], (TL_C[0], TOP_Y))[1:-1] + \
               seg_arc(TL_C[0], TL_C[1], TL_R,
                       math.pi / 2, math.pi)[1:-1] + \
               seg_line((0.75, TL_C[1]), (0.75, BL_C[1]))[1:-1] + \
               seg_arc(BL_C[0], BL_C[1], BL_R,
                       math.pi, 3 * math.pi / 2)[1:]
    elif name == "full":      # 第4段:整圈回到起点
        pts = build_route("half")[:-1] + \
              seg_line((BL_C[0], BOT_Y), START)[1:]
    else:
        sys.exit(f"{TAG} 未知路线 {name}")
    return pts


def load_cones(path):
    cones = []
    with open(path) as f:
        for r in csv.DictReader(f):
            cones.append((float(r["x"]), float(r["y"])))
    return cones


def heading_at(pts, i):
    if i == 0:
        dx, dy = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
    else:
        dx, dy = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
    return math.atan2(dy, dx)


def rect_clear(px, py, th, cx, cy):
    """锥心在车体矩形的间隙距离(0=在矩形上)。整车轮廓判据。"""
    dx, dy = cx - px, cy - py
    c, s = math.cos(-th), math.sin(-th)
    x = dx * c - dy * s
    y = dx * s + dy * c
    ex, ey = abs(x) - VEH_HALF_L, abs(y) - VEH_HALF_W
    if ex <= 0 and ey <= 0:
        return 0.0
    return math.hypot(max(ex, 0.0), max(ey, 0.0))


def static_clearance(pts, cones):
    """静态预检:路线上每点整车矩形 vs 全部锥的最小间隙。"""
    worst, where = 1e9, None
    for i, (px, py) in enumerate(pts):
        th = heading_at(pts, i)
        for cx, cy in cones:
            d = rect_clear(px, py, th, cx, cy)
            if d < worst:
                worst, where = d, i
    return worst, where


class TestDriver(Node):
    def __init__(self, args):
        super().__init__("sim_test_driver")
        self.route_name = args.route
        self.v_nom = args.speed
        self.ld = args.lookahead
        self.pts = build_route(args.route)
        n = len(self.pts)
        self.ss = [0.0] * n
        for i in range(1, n):
            self.ss[i] = self.ss[i - 1] + math.hypot(
                self.pts[i][0] - self.pts[i - 1][0],
                self.pts[i][1] - self.pts[i - 1][1])
        self.s_total = self.ss[-1]
        self.cones = load_cones(args.cones)

        # 硬性要求5:启动时 /cmd_vel 必须无其他发布者(键盘/手柄/Nav2 离线)
        others = self.count_publishers("/cmd_vel")
        if others > 0:
            sys.exit(f"{TAG} 拒绝启动:/cmd_vel 已有 {others} 个发布者"
                     "(疑似键盘/手柄/Nav2 在线),测试驾驶器要求独占")
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_path = self.create_publisher(Path, "/test_driver/route", 10)
        self.create_subscription(ModelStates, "/model_states",
                                 self.on_models, 10)
        self.pose = None          # (x, y, yaw) Gazebo 真值,仅用于跟踪/验收
        self.state = "WAIT"
        self.t0 = self.t_drive = None
        self.idx = 0              # 当前进度索引
        self.min_clear = 1e9
        self.max_dev = 0.0
        self.max_w = 0.0
        self.driven = 0.0
        self.last_pos = None
        self.hist = deque()       # (t, s) 失速检测窗
        self.result = None
        self.tmr = self.create_timer(0.05, self.tick)          # 20Hz 控制环
        self.tmr_ck = self.create_timer(2.0, self.check_excl)  # 独占复检
        self.path_seq = 0
        print(f"{TAG} 路线={args.route} 里程={self.s_total:.2f}m "
              f"速度={self.v_nom}m/s 前视={self.ld}m "
              f"阿克曼|ω|max={self.v_nom * math.tan(MAX_STEER) / WHEELBASE:.3f}rad/s")

    # ---------- 感知 ----------
    def on_models(self, msg):
        try:
            k = msg.name.index("racecar")
        except ValueError:
            return
        p = msg.pose[k]
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.position.x, p.position.y, yaw)

    def check_excl(self):
        if self.state in ("DONE", "ABORT"):
            return
        if self.count_publishers("/cmd_vel") > 1:
            self.abort("cmd_conflict",
                       "/cmd_vel 出现第二个发布者(键盘/手柄/Nav2?)即停")

    # ---------- 跟踪 ----------
    def project(self, x, y):
        """窗口内最近折线投影 → (s, 横向偏差, 索引)。"""
        lo = max(0, self.idx - 10)
        hi = min(len(self.pts) - 1, self.idx + 90)
        best = (1e9, 0.0, lo)
        for i in range(lo, hi):
            ax, ay = self.pts[i]
            bx, by = self.pts[min(i + 1, len(self.pts) - 1)]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy or 1e-12
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            px, py = ax + t * dx, ay + t * dy
            d = math.hypot(x - px, y - py)
            if d < best[0]:
                s = self.ss[i] + t * math.sqrt(L2)
                best = (d, s, i)
        return best

    def tick(self):
        if self.state in ("DONE", "ABORT"):
            return
        now = time.time()
        self.publish_path()
        if self.pose is None:
            return
        x, y, yaw = self.pose
        if self.state == "WAIT":
            # 出生点就位确认(整车质心在起点 0.5m 内)后起步
            if math.hypot(x - START[0], y - START[1]) > 0.5:
                print(f"{TAG} 等待就位,当前 ({x:.2f},{y:.2f})")
                return
            self.state = "DRIVE"
            self.t0 = self.t_drive = now
            self.hist.append((now, 0.0))
            print(f"{TAG} 起步 @({x:.2f},{y:.2f},yaw={yaw:.2f})")
        # DRIVING -----
        dev, s_v, self.idx = self.project(x, y)
        remain = self.s_total - s_v
        # 硬性要求4:三项监控即停
        if dev > 0.35:
            self.abort("deviation",
                       f"横向偏出参考路线 {dev:.2f}m (>0.35m) @({x:.2f},{y:.2f})")
            return
        cmin = min(rect_clear(x, y, yaw, cx, cy) for cx, cy in self.cones)
        self.min_clear = min(self.min_clear, cmin)
        if cmin < ABORT_CLEAR:
            self.abort("collision",
                       f"整车轮廓距锥沿仅 {cmin:.3f}m (<{ABORT_CLEAR}) @({x:.2f},{y:.2f})")
            return
        self.max_dev = max(self.max_dev, dev)
        self.hist.append((now, s_v))
        while self.hist and now - self.hist[0][0] > 5.0:
            self.hist.popleft()
        if now - self.hist[0][0] >= 5.0 and s_v - self.hist[0][1] < 0.05:
            self.abort("stall",
                       f"5s 内进度仅 {s_v - self.hist[0][1]:.2f}m @({x:.2f},{y:.2f})")
            return
        if self.last_pos:
            self.driven += math.hypot(x - self.last_pos[0], y - self.last_pos[1])
        self.last_pos = (x, y)
        # 终点判据
        if remain < 0.15:
            self.finish(now)
            return
        # 纯追踪目标点
        tgt = None
        for i in range(self.idx, len(self.ss)):
            if self.ss[i] - s_v >= self.ld:
                tgt = self.pts[i]
                break
        if tgt is None:
            tgt = self.pts[-1]
        dx, dy = tgt[0] - x, tgt[1] - y
        c, s = math.cos(-yaw), math.sin(-yaw)
        ty = dx * s + dy * c
        tx = dx * c - dy * s
        alpha = math.atan2(ty, tx)
        v = self.v_nom if remain > 1.2 else max(0.12, self.v_nom * remain / 1.2)
        w = 2.0 * v * math.sin(alpha) / self.ld
        w_max = v * math.tan(MAX_STEER) / WHEELBASE   # 阿克曼约束(硬性要求2)
        w = max(-w_max, min(w_max, w))
        self.max_w = max(self.max_w, abs(w))
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub_cmd.publish(msg)

    # ---------- 终态 ----------
    def stop_cmd(self):
        for _ in range(8):
            self.pub_cmd.publish(Twist())
            time.sleep(0.05)

    def report(self, result, reason="", extra=None):
        r = {
            "tag": "仿真测试驾驶器",
            "route": self.route_name,
            "result": result,
            "reason": reason,
            "s_total_m": round(self.s_total, 3),
            "driven_m": round(self.driven, 3),
            "duration_s": round(time.time() - self.t0, 1) if self.t0 else 0,
            "min_cone_clear_m": round(self.min_clear, 3) if self.min_clear < 1e9 else None,
            "max_dev_m": round(self.max_dev, 3),
            "max_omega": round(self.max_w, 3),
            "v_nom": self.v_nom,
        }
        if extra:
            r.update(extra)
        print(f"{TAG} ===== 报告 {json.dumps(r, ensure_ascii=False)}")
        try:
            with open(REPORT_PATH[0], "w") as f:
                json.dump(r, f, ensure_ascii=False, indent=1)
        except OSError:
            pass
        return r

    def finish(self, now):
        self.state = "DONE"
        self.stop_cmd()
        self.report("PASS", "到达路线终点并停车",
                    {"s_end_m": round(self.s_total, 3)})
        raise DriveExit()

    def abort(self, kind, detail):
        self.state = "ABORT"
        self.stop_cmd()
        self.report("FAIL", f"{kind}: {detail}")
        raise DriveExit()

    def publish_path(self):
        if self.path_seq % 40:      # 2Hz,低频足矣
            self.path_seq += 1
            return
        self.path_seq += 1
        p = Path()
        p.header.frame_id = "odom"
        p.header.stamp = self.get_clock().now().to_msg()
        for i in range(0, len(self.pts), 4):
            ps = PoseStamped()
            ps.header.frame_id = "odom"
            ps.pose.position.x, ps.pose.position.y = self.pts[i]
            p.poses.append(ps)
        self.pub_path.publish(p)


REPORT_PATH = [""]


class DriveExit(Exception):
    """到达/中止时从定时器回调抛出,解出 spin。"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", default="short",
                    choices=["short", "u_right", "half", "full"])
    ap.add_argument("--speed", type=float, default=0.3)
    ap.add_argument("--lookahead", type=float, default=0.55)
    ap.add_argument("--cones", default="/home/lzx/hwcart_sim/docs/track/trackA_cones.csv")
    ap.add_argument("--report", default="")
    args = ap.parse_args()
    REPORT_PATH[0] = args.report or \
        f"/home/lzx/hwcart_sim/docs/track/test_driver_{args.route}_report.json"

    pts = build_route(args.route)
    cones = load_cones(args.cones)
    worst, wi = static_clearance(pts, cones)
    print(f"{TAG} 静态余量预检(整车矩形 vs 63锥):"
          f"最小间隙 {worst:.3f}m @路线点{wi} "
          f"(要求 ≥{REQ_CLEAR:.3f}m = 锥外接圆{CONE_R}+余量{MARGIN})")
    if worst < REQ_CLEAR:
        print(f"{TAG} [拒绝行驶] 参考路线余量不足,不发动。")
        sys.exit(2)

    rclpy.init()
    node = TestDriver(args)
    try:
        rclpy.spin(node)
    except (DriveExit, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    print(f"{TAG} 退出")


if __name__ == "__main__":
    main()
