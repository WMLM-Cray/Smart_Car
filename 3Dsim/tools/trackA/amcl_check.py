#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[AMCL定位验收器] tools/trackA/amcl_check.py
=================================================
身份声明:本节点是阶段3 第三步的【AMCL 定位验收器】——在
map_server+AMCL 场景(无 SLAM、无测试驾驶器)中统计 AMCL 定位质量,
并发布少量固定时长的开环速度指令(等效人工键盘轻推)让车产生运动,
检验 AMCL 在运动中的跟踪。它不是仿真测试驾驶器:不跟踪参考路线、
不做余量预检;运动只在底道走廊安全区内(出生点前 2.5m)往返。

原理:
  - /amcl_pose(AMCL 估计,map 系)与 /model_states(Gazebo 真值,仅验收
    用,不进 AMCL)换算到同一 map 系:map = gz - (11.0, 0.75)(见
    config/amcl.yaml 头注的坐标系推导);
  - 误差统计:初始静止段均值 / 运动中最大值 / RMS / 末段静止均值;
  - PASS 判据:样本≥5 且 初始≤0.25m 且 运动中最大≤0.25m 且 末段≤0.15m。

用法(需已起 localization_bringup 并 source):
  python3 tools/trackA/amcl_check.py \
      [--report /home/lzx/hwcart_sim/docs/track/amcl_check_report.json]
"""
import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped

TAG = "[AMCL定位验收器]"

GZ_SPAWN = (11.0, 0.75)      # 出生点(Gazebo 世界系)= map 系原点

# 判据阈值(米)
TH_INIT, TH_MAX, TH_FINAL = 0.25, 0.25, 0.15

# 运动阶段:(名称, 时长s, 线速度 m/s)。前进 0.25×10=2.5m(底道走廊内
# 出生点 x11.0→13.5,右端安全到 14.03),倒车 0.20×12.5=2.5m 返回。
# NUDGE 微调段:轻推 0.3m 往返,让"初始误差"统计带上真实运动样本;
# 注:给出初始位姿(set_initial_pose/initialpose)后,首帧激光即发布首个
# 位姿并广播 TF(humble amcl 源码 force_publication 路径,已实测);
# 之后的滤波更新才需运动超过 update_min_d/update_min_a。
PHASES = [
    ("NUDGE_F_触发滤波", 1.5, 0.20),
    ("NUDGE_R_回位", 1.5, -0.20),
    ("HOLD0_初始静止", 4.0, 0.0),
    ("FWD_前进2.5m", 10.0, 0.25),
    ("HOLD1_停", 4.0, 0.0),
    ("REV_倒回2.5m", 12.5, -0.20),
    ("HOLD2_末段静止", 4.0, 0.0),
]
# 各阶段的统计归类:init(出发微调段,即"初始定位")/motion(主运动)/final(末段)
PHASE_KINDS = ["init", "init", "init", "motion", "motion", "motion", "final"]

REPORT_PATH = [""]
RESULT = [""]


class CheckExit(Exception):
    """验收结束(正常/失败)从定时器回调抛出,解出 spin。"""


class AmclCheck(Node):
    def __init__(self):
        super().__init__("amcl_check")
        # 独占 /cmd_vel(本场景不应有其他速度源;有即环境异常,拒绝开动)
        others = self.count_publishers("/cmd_vel")
        if others > 0:
            sys.exit(f"{TAG} 拒绝启动:/cmd_vel 已有 {others} 个发布者"
                     "(定位场景不应有键盘/驾驶器/Nav2 速度源)")
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self.on_amcl, 10)
        self.create_subscription(ModelStates, "/model_states",
                                 self.on_models, 10)
        self.gt = None            # (x, y, yaw) map 系真值
        self.samples = []         # (kind, err, yaw_err)
        self.phase_idx = -1       # -1=WAIT(AMCL 发布者在线+真值可用)
        self.phase_t0 = None
        self.t_start = time.time()
        self.tmr = self.create_timer(0.05, self.tick)   # 20Hz
        self.last_log = 0.0
        print(f"{TAG} 就绪:等 AMCL 发布者在线(给初始位姿后首帧激光即出"
              f"首个位姿)→ 开环轻推 前进2.5m/倒回2.5m"
              f" → 误差判据 初始≤{TH_INIT} 最大≤{TH_MAX} "
              f"末段≤{TH_FINAL} (m)")

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
        # Gazebo 世界系 → map 系(出生点为原点,初始朝向一致)
        self.gt = (p.position.x - GZ_SPAWN[0], p.position.y - GZ_SPAWN[1],
                   yaw)

    def on_amcl(self, msg):
        if self.gt is None or self.phase_idx < 0:
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        ayaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        err = math.hypot(p.x - self.gt[0], p.y - self.gt[1])
        dyaw = math.atan2(math.sin(ayaw - self.gt[2]),
                          math.cos(ayaw - self.gt[2]))
        if self.phase_idx < len(PHASE_KINDS):
            kind = PHASE_KINDS[self.phase_idx]
        else:
            kind = "final"
        self.samples.append((kind, err, abs(dyaw)))

    # ---------- 主循环 ----------
    def tick(self):
        now = time.time()
        if self.phase_idx == -1:            # 等 AMCL 在线 + 真值可用
            if now - self.t_start > 90.0:
                self.end({"tag": "AMCL定位验收器", "result": "FAIL",
                          "reason": "no_amcl: 90s /amcl_pose 无发布者"
                                    "(lifecycle 未激活或地图未载入)"})
                return
            amcl_up = self.count_publishers("/amcl_pose") >= 1
            if amcl_up and self.gt is not None:
                print(f"{TAG} AMCL 在线+真值可用,开始验收序列"
                      "(set_initial_pose 已给出生点初值)")
                self.enter_phase(0, now)
            elif now - self.last_log > 5.0:
                self.last_log = now
                up = "是" if amcl_up else "否"
                print(f"{TAG} 等 AMCL 发布者(当前:{up})与真值…"
                      f"({now - self.t_start:.0f}s)")
            return
        if self.phase_idx >= len(PHASES):             # 已结束
            return
        name, dur, v = PHASES[self.phase_idx]
        if now - self.phase_t0 >= dur:
            self.enter_phase(self.phase_idx + 1, now)
            return
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = 0.0
        self.pub_cmd.publish(msg)
        if now - self.last_log > 5.0:
            self.last_log = now
            n = len(self.samples)
            e = self.samples[-1][1] if n else -1.0
            print(f"{TAG} 阶段[{name}] {now - self.phase_t0:4.1f}s "
                  f"v={v:+.2f} 样本{n} 当前误差{e:.3f}m")

    def enter_phase(self, i, now):
        self.phase_idx = i
        self.phase_t0 = now
        if i >= len(PHASES):
            self.end(self.evaluate())
            return
        name, dur, v = PHASES[i]
        print(f"{TAG} ---- 进入阶段[{name}] {dur}s v={v:+.2f}m/s ----")

    # ---------- 汇总 ----------
    def seg(self, kind):
        return [e for (k, e, _) in self.samples if k == kind]

    def evaluate(self):
        init = self.seg("init")     # 出发微调段(NUDGE×2+HOLD0)的均值
        fin = self.seg("final")
        maxerr = max((e for (_, e, _) in self.samples), default=1e9)
        rms = (sum(e * e for (_, e, _) in self.samples) /
               len(self.samples)) ** 0.5 if self.samples else 1e9
        e_init = sum(init) / len(init) if init else 1e9
        e_fin = sum(fin) / len(fin) if fin else 1e9
        ok = (len(self.samples) >= 5 and e_init <= TH_INIT
              and maxerr <= TH_MAX and e_fin <= TH_FINAL)
        detail = (f"样本{len(self.samples)} 初始{e_init:.3f} "
                  f"最大{maxerr:.3f} RMS{rms:.3f} 末段{e_fin:.3f} (m)"
                  f" | 判据 样本≥5 初始≤{TH_INIT} 最大≤{TH_MAX} "
                  f"末段≤{TH_FINAL}")
        return {
            "tag": "AMCL定位验收器",
            "result": "PASS" if ok else "FAIL",
            "reason": detail,
            "samples": len(self.samples),
            "err_init_m": round(e_init, 3) if init else None,
            "err_max_m": round(maxerr, 3),
            "err_rms_m": round(rms, 3),
            "err_final_m": round(e_fin, 3) if fin else None,
            "motion": "开环轻推:微调0.3m触发滤波→前进2.5m→停→倒回2.5m"
                      "(底道走廊安全区)",
            "gt_usage": "真值(/model_states)仅误差统计,不进 AMCL",
        }

    def end(self, r):
        RESULT[0] = r.get("result", "FAIL")
        for _ in range(8):
            self.pub_cmd.publish(Twist())
            time.sleep(0.05)
        print(f"{TAG} ===== 报告 {json.dumps(r, ensure_ascii=False)}")
        try:
            with open(REPORT_PATH[0], "w") as f:
                json.dump(r, f, ensure_ascii=False, indent=1)
        except OSError:
            pass
        raise CheckExit()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", default="")
    args = ap.parse_args()
    REPORT_PATH[0] = args.report or \
        "/home/lzx/hwcart_sim/docs/track/amcl_check_report.json"

    rclpy.init()
    node = AmclCheck()
    rc = 0 if RESULT[0] in ("PASS", "") else 1
    try:
        rclpy.spin(node)
    except (CheckExit, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    print(f"{TAG} 退出")
    sys.exit(rc)


if __name__ == "__main__":
    main()
