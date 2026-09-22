#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[Nav2导航验收器] tools/trackA/nav_check.py
=================================================
身份声明:本节点是阶段3 第四步的【Nav2 单点导航验收器】——向
bt_navigator(/navigate_to_pose)依次发送目标点(短直道→弯道→连续目标),
统计导航结果与运行质量;不发布任何速度指令(车速全部来自 Nav2 控制链)。
真值(/model_states)只用于误差/间隙统计,不进导航栈。

原理:
  - 坐标:map 系原点=出生点 Gazebo(11.0,0.75),真值换算 map=gz-(11.0,0.75);
  - 目标序列(地图坐标,顺完整一圈;按 63 锥真值重排:顶道中心=4.0
      (4.75 是外墙锥线),左段=S 弯+远左发卡;第三轮 S 弯拆段加 2 个
      中间目标抑震荡):
      G1 单目标·短直道 (2.5, 0.0, 0°)
      G2 单目标·右U中段 (5.0, 2.0, 90°)
      G3 单目标·右U上口 (2.9, 4.0, 200°) —— 出口弧锚点防切 r16
      G4 单目标·弯道右U出口 (2.0, 4.0, 200°) —— 与上口锚点带对中防绕圈
      G4 连续目标1·顶道左行 (-2.0, 4.0, 180°)
      G5 连续目标2·S弯东口 (-3.6, 3.7, 200°)
      G6 连续目标3·S弯中点 (-5.4, 2.85, 200°)
      G7 连续目标4·顶道西段 (-8.0, 4.0, 185°)
      G8 连续目标5·发卡拐角 (-10.1, 3.65, 240°) —— 拆急转防切内
      G9 连续目标6·远左发卡 (-10.25, 2.6, 270°)
      G10 连续目标7·回出生点 (0.0, 0.0, 0°)
  - 判据(全过才 PASS):
      ①每个目标 NavigateToPose 结果 = SUCCEEDED;
      ②到达时真值-目标距离 ≤0.30m(goal_checker amcl 判 0.15,加定位余量);
      ③AMCL估计-真值全程最大 ≤0.25m;
      ④整车矩形(0.56×0.35,冻结)对锥桶碰撞半径0.14 的最小间隙 ≥0.05m
        (无接触+半格量化余量;第十二轮 0.041 未达标 → 按原判据 FAIL 归档,
        见 TH_CLEAR 注);
      ⑤阿克曼上界按【车辆实际运动】(里程计 /ackermann_steering_controller
        /odometry)判定:运动 |ω|≤|v|·tan(0.55)/0.36+0.15;静止 |ω|≤0.15
        ——即"不原地转向"的运行时验证。指令层 /cmd_vel 的曲率超限
        (RPP 追几何胡萝卜不追运动学,峰值 ω/v 可达 4+,由 gz 舵角钳制
        0.55rad 消化)不判 FAIL,仅计数上报 cmd_vel_curvature_overshoots。

用法(需已起 navigation_bringup 并 source):
  python3 tools/trackA/nav_check.py \
      [--report /home/lzx/hwcart_sim/docs/track/nav_check_report.json] \
      [--cones docs/track/trackA_cones.csv]
"""
import argparse
import csv
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from nav2_msgs.action import NavigateToPose

TAG = "[Nav2导航验收器]"

GZ_SPAWN = (11.0, 0.75)          # 出生点(Gazebo 世界系)= map 系原点
CONE_R = 0.14                    # 锥桶碰撞半径(098c3f0 官方 0.28 底径/2)
HALF_L, HALF_W = 0.28, 0.175     # 冻结整车矩形 0.56×0.35 的半长/半宽
WHEELBASE, MAX_STEER = 0.36, 0.55
W_SLOPE = math.tan(MAX_STEER) / WHEELBASE   # ω-v 上界斜率 ≈1.703

TH_ARRIVAL_GT, TH_AMCL_GT, TH_CLEAR = 0.30, 0.25, 0.05
# 间隙判据 0.10→0.05(第六/七轮定标,2026-09-16 用户终审确认保留,不回改):
# 0.10 初值被瓶紧处逐轮数据证伪;0.05=无接触+半格量化余量。第十二轮终验
# 0.041<0.05 → 按原判据 FAIL 归档(不重标通过)。"5cm 像格量化是主因/唯一
# 根因""细地图(0.025m)能治本"均为【待验证假设】(依据=离线合成代价地图
# 推断,未做对照实验);同配置 4 采样 0.076/0.074/0.033/0.041 全无接触但
# 掷硬币,间隙鲁棒性改善列为独立后续任务(阶段3_4文档§9)。

# 目标位姿均经 tools/trackA/goal_footprint_audit.py 离线足印验证(max cost
# ≤62,无一压 253 带);几何依据 docs/track/trackA_cones.csv 63 锥真值
GOALS = [
    ("单目标_短直道", 2.5, 0.0, 0.0, 240.0),
    ("单目标_右U中段", 5.0, 2.0, math.pi / 2, 240.0),
    # 右U上口:出口弧中段斜车身切内弧顶锥 r16 (3.05,3.25)(第八轮 0.046,
    # 同配置第六轮 0.128,方差 ±0.04);锚点压外线+预转 200° 拉宽出口弧
    ("单目标_右U上口", 2.9, 4.0, math.radians(200), 240.0),
    # 右U出口 yaw 180°→200°(第十一轮修复):上口锚点终态朝向带 [171°,229°]
    # (yaw 容差 0.5rad),0.9m 短跳原要求终点再修 20°——第十一轮抽到恶性侧,
    # xy 差 0.15 不收 → 终点朝向绕圈 → AMCL 漂 0.231 → 重规划环线南掉进
    # r15/r16 凹兜撞锥((3.12,2.8,65°) 对 r16 −0.016)。改 200° 短跳朝向
    # 中性,终态误差≈锚点残差(≤28.6°)不再绕圈;20° 修正留给顶道 2.2m 长跳。
    ("单目标_弯道右U出口", 2.0, 4.0, math.radians(200), 240.0),
    ("连续目标1_顶道左行", -2.0, 4.0, math.pi, 240.0),
    ("连续目标2_S弯东口", -3.6, 3.7, math.radians(200), 240.0),
    ("连续目标3_S弯中点", -5.4, 2.85, math.radians(200), 240.0),
    ("连续目标4_顶道西段", -8.0, 4.0, math.radians(185), 240.0),
    # 发卡拐角:提前压外线下降,把 85° 急转拆成 55°+30° 两个缓转(第五轮
    # 拐角中途 (−9.83,3.43,−128°) 贴内墙角锥 r24 (−9.5,3.25) 仅 0.051m)
    ("连续目标5_发卡拐角", -10.1, 3.65, math.radians(240), 240.0),
    ("连续目标6_远左发卡", -10.25, 2.6, math.pi * 1.5, 240.0),
    # 注:发卡出口(b36-I10 斜缝 1.21m)不放中间目标——第七轮试过 (−9.35,
    # 0.3):北线弧中段位姿在 padding 0.08 下压 253 带(垫后足印对 b36 像素
    # 净距 0.02m),Smac 被逼南绕 b36 蹭锥中止;斜缝内也放不下任何垫后足印
    # 目标位姿。据离线合成代价地图推断(待验证):5cm 像格+格心判定或把
    # 该处规划器几何地板限制在 ~0.076m 附近。
    ("连续目标7_回出生点", 0.0, 0.0, 0.0, 300.0),
]

STATUS_NAME = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING",
               3: "CANCELING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED",
               7: "ABORTED"}

REPORT_PATH = [""]
RESULT = [""]


class CheckExit(Exception):
    pass


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavCheck(Node):
    def __init__(self, cones_map):
        super().__init__("nav_check")
        self.cones = cones_map           # [(x,y)] map 系
        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(ModelStates, "/model_states",
                                 self.on_models, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self.on_amcl, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 20)
        self.create_subscription(Odometry,
                                 "/ackermann_steering_controller/odometry",
                                 self.on_odom, 20)
        self.gt = None                   # (x, y, yaw) map 系真值
        self.phase = "WAIT"
        self.goal_xy = None
        self.amcl_errs = []              # (phase, err)
        self.w_bad = []                  # 车辆实际 ω 超界(odom 判定)
        self.cmd_over = []               # 指令层曲率超限(仅信息)
        self.min_clear = [1e9]
        self.worst = {}      # 最差间隙发生现场(相/位姿/锥)
        self.phase_min = {}  # 每阶段最小间隙(定位次差区域,不被全局最差掩盖)
        self.tmr = self.create_timer(0.10, self.tick)
        self.last_log = 0.0

    # ---------- 感知 ----------
    def on_models(self, msg):
        try:
            k = msg.name.index("racecar")
        except ValueError:
            return
        p = msg.pose[k]
        self.gt = (p.position.x - GZ_SPAWN[0], p.position.y - GZ_SPAWN[1],
                   yaw_of(p.orientation))

    def on_amcl(self, msg):
        if self.gt is None:
            return
        p = msg.pose.pose.position
        err = math.hypot(p.x - self.gt[0], p.y - self.gt[1])
        self.amcl_errs.append((self.phase, err))

    def on_cmd(self, msg):
        # 指令层:RPP 追逐几何胡萝卜的 κ=2y/L² 不受运动学上限约束,低速时
        # ω/v 可远超 tan(0.55)/0.36=1.703;由 gz 舵角钳制消化,仅计数上报
        v, w = msg.linear.x, msg.angular.z
        if abs(v) > 0.05 and abs(w) / max(abs(v), 1e-6) > W_SLOPE * 1.10:
            self.cmd_over.append((self.phase, round(v, 3), round(w, 3),
                                  round(abs(w) / max(abs(v), 1e-6), 2)))

    def on_odom(self, msg):
        # 车辆实际运动:这才是"不原地转向"的物理事实判据
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        bound = (abs(v) * W_SLOPE + 0.15) if abs(v) > 0.05 else 0.15
        if abs(w) > bound:
            self.w_bad.append((self.phase, round(v, 3), round(w, 3),
                               round(bound, 3)))

    # ---------- 间隙(整车矩形 vs 锥桶碰撞圆) ----------
    def clearance(self):
        if self.gt is None:
            return
        gx, gy, yaw = self.gt
        c, s = math.cos(yaw), math.sin(yaw)
        worst = 1e9
        for (cx, cy) in self.cones:
            dx, dy = cx - gx, cy - gy
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            qx = max(-HALF_L, min(HALF_L, lx))
            qy = max(-HALF_W, min(HALF_W, ly))
            d = math.hypot(lx - qx, ly - qy)
            if d - CONE_R < worst:
                worst = d - CONE_R
                worst_here = (cx, cy)
        if worst < self.min_clear[0]:
            self.min_clear[0] = worst
            self.worst = {"phase": self.phase,
                          "pose_map": (round(gx, 2), round(gy, 2),
                                       round(math.degrees(yaw))),
                          "cone_map": worst_here,
                          "clear_m": round(worst, 3)}
        if worst < self.phase_min.get(self.phase, 1e9):
            self.phase_min[self.phase] = worst

    def tick(self):
        self.clearance()
        now = time.time()
        if now - self.last_log > 5.0 and self.gt is not None:
            self.last_log = now
            d = (math.hypot(self.gt[0] - self.goal_xy[0],
                            self.gt[1] - self.goal_xy[1])
                 if self.goal_xy else -1.0)
            e = self.amcl_errs[-1][1] if self.amcl_errs else -1.0
            print(f"{TAG} 阶段[{self.phase}] 距目标{d:.2f}m "
                  f"AMCL误差{e:.3f}m 最小间隙{self.min_clear[0]:.3f}m "
                  f"车辆ω违规{len(self.w_bad)} 指令超限{len(self.cmd_over)}")

    # ---------- 单目标流程 ----------
    def run_goal(self, name, gx, gy, gyaw, tmo):
        print(f"{TAG} ---- 目标[{name}] ({gx:.2f},{gy:.2f},"
              f"{math.degrees(gyaw):.0f}°) 超时{tmo:.0f}s ----")
        self.phase = name
        self.goal_xy = (gx, gy)
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        ps.pose.orientation.z = math.sin(gyaw / 2.0)
        ps.pose.orientation.w = math.cos(gyaw / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = ps
        goal.behavior_tree = ""          # 用 bt_navigator 默认(极简树)
        t0 = time.time()
        fut = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return {"name": name, "result": "REJECTED", "duration_s":
                    round(time.time() - t0, 1)}
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=tmo)
        dur = round(time.time() - t0, 1)
        if rf.result() is None:          # 超时:取消并记
            cf = gh.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cf, timeout_sec=5.0)
            return {"name": name, "result": "TIMEOUT", "duration_s": dur}
        st = rf.result().status
        arrival = (math.hypot(self.gt[0] - gx, self.gt[1] - gy)
                   if self.gt else None)
        return {"name": name,
                "result": STATUS_NAME.get(st, str(st)),
                "duration_s": dur,
                "arrival_gt_m": round(arrival, 3) if arrival is not None
                else None}

    def evaluate(self, goal_rs):
        ok_goals = all(g["result"] == "SUCCEEDED" for g in goal_rs)
        ok_arr = all((g.get("arrival_gt_m") is not None and
                      g["arrival_gt_m"] <= TH_ARRIVAL_GT)
                     for g in goal_rs if g["result"] == "SUCCEEDED")
        amcl_max = max((e for (_, e) in self.amcl_errs), default=1e9)
        min_clear = self.min_clear[0]
        ok = (ok_goals and ok_arr and len(self.amcl_errs) >= 10
              and amcl_max <= TH_AMCL_GT and min_clear >= TH_CLEAR
              and not self.w_bad)
        detail = (f"目标{len(goal_rs)}/{sum(1 for g in goal_rs if g['result'] == 'SUCCEEDED')}成功 "
                  f"AMCL-真值最大{amcl_max:.3f}m 最小间隙{min_clear:.3f}m "
                  f"车辆ω违规{len(self.w_bad)} "
                  f"指令曲率超限{len(self.cmd_over)}(信息项) | 判据 全SUCCEEDED "
                  f"到达≤{TH_ARRIVAL_GT} AMCL≤{TH_AMCL_GT} 间隙≥{TH_CLEAR} 车辆ω违规=0")
        return {"tag": "Nav2导航验收器",
                "result": "PASS" if ok else "FAIL",
                "reason": detail,
                "goals": goal_rs,
                "path": "短直道→右U→顶道左行→S弯→远左发卡→回出生点(整圈)",
                "amcl_gt_max_m": round(amcl_max, 3),
                "min_clear_m": round(min_clear, 3),
                "worst_clear_event": self.worst,
                "phase_min_clear_m": {k: round(v, 3)
                                      for k, v in self.phase_min.items()},
                "omega_bound_violations": len(self.w_bad),
                "w_violation_samples": self.w_bad[:5],
                "cmd_vel_curvature_overshoots": len(self.cmd_over),
                "cmd_overshoot_samples": self.cmd_over[:5],
                "gt_usage": "真值仅误差/间隙统计,不进导航栈;车速全部来自Nav2",
                }


def main():
    global REPORT_PATH
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", default="")
    ap.add_argument("--cones", default="docs/track/trackA_cones.csv")
    args = ap.parse_args()
    REPORT_PATH = args.report or \
        "/home/lzx/hwcart_sim/docs/track/nav_check_report.json"

    cones_map = []
    try:
        with open(args.cones) as f:
            for row in csv.DictReader(f):
                cones_map.append((float(row["x"]) - GZ_SPAWN[0],
                                  float(row["y"]) - GZ_SPAWN[1]))
    except OSError as e:
        sys.exit(f"{TAG} 锥桶 CSV 读取失败({args.cones}):{e}")

    rclpy.init()
    node = NavCheck(cones_map)
    print(f"{TAG} 锥桶{len(cones_map)}个 | 等 /navigate_to_pose 服务"
          "(bt_navigator)与真值…")
    t0 = time.time()
    while not node.ac.wait_for_server(timeout_sec=2.0):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.gt is not None and time.time() - t0 > 10.0:
            print(f"{TAG} 等服务中…({time.time() - t0:.0f}s)")
        if time.time() - t0 > 180.0:
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(f"{TAG} FAIL: 180s 等 /navigate_to_pose 无服务")
    while node.gt is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    n_pub = node.count_publishers("/cmd_vel")
    print(f"{TAG} 服务在线,真值可用,/cmd_vel 发布者={n_pub}"
          "(应为1=velocity_smoother)")

    goal_rs = []
    try:
        for (name, gx, gy, gyaw, tmo) in GOALS:
            goal_rs.append(node.run_goal(name, gx, gy, gyaw, tmo))
            time.sleep(3.0)                       # 目标间稍歇
        r = node.evaluate(goal_rs)
        RESULT[0] = r["result"]
        print(f"{TAG} ===== 报告 {json.dumps(r, ensure_ascii=False)}")
        try:
            with open(REPORT_PATH, "w") as f:
                json.dump(r, f, ensure_ascii=False, indent=1)
        except OSError:
            pass
        raise CheckExit()
    except KeyboardInterrupt:
        RESULT[0] = "FAIL"
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"{TAG} 退出")
    sys.exit(0 if RESULT[0] == "PASS" else 1)


if __name__ == "__main__":
    main()
