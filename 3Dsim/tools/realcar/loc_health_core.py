#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""loc_health_core.py —— /localization_ok 定位健康发布端的状态机核心（纯 Python，无 ROS 依赖）。

只做「证据 → 判定」的纯逻辑，不 import rclpy，可离线单测（pytest）。
ROS 订阅 / TF 查询 / 时钟 / 发布都在 localization_health.py，把「接收时刻(单调钟)」与
各字段值喂给本类。

判定语义对齐 tools/mission/LOCALIZATION_HEALTH_INTERFACE.md（2026-09-20 落地）：
- 输出 /localization_ok = std_msgs/Bool，True 只在全部判据满足时发。
- Bool 无法表达状态，诊断用 state 字符串 + reasons 区分七类非健康态：
    init / evidence_insufficient / input_abnormal / time_abnormal /
    tf_abnormal / threshold_exceeded / unverified_thresholds
- 阈值全部外置；verified=False 时一律不判健康（不能偷偷用默认值放行）。

关键口径（用户 2026-09-20 三点，必须遵守）：
1. AMCL 静止无新 scan / 未达 update_min_d/a 门限时**不持续发布新估计**，amcl_pose 未更新 ≠ 失效。
   判据须结合运动状态（odom twist）、scan 接收、TF 可用性综合判断；不确定时不得判健康。
2. AMCL 与 odom 共享里程计，一致性只作**辅助**检查，不作定位正确的独立证明（两者会一起漂）。
3. 协方差检查区分位置（米）与角度（弧度），不混用一个阈值。

时间基准（失效安全核心）：
- 所有「断流 / 新鲜度」用**单调时钟接收时刻**（recv_t，由节点 time.monotonic() 提供），
  不依赖消息 header.stamp（车机钟慢 5 个月、latch 帧 stamp 可落后 159s，不可作新鲜度依据）。
- header.stamp 只用于「回退检测」（同一话题单调性），回退超过容忍记 TIME_ABNORMAL 候选。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

# ---- 状态常量 ----
INIT = "init"                                # 启动，尚无任何证据
EVIDENCE_INSUFFICIENT = "evidence_insufficient"   # 关键证据缺失（amcl/scan/odom/tf 未到）
INPUT_ABNORMAL = "input_abnormal"            # NaN/Inf/无效四元数
TIME_ABNORMAL = "time_abnormal"              # 断流 / header.stamp 回退
TF_ABNORMAL = "tf_abnormal"                  # TF 缺失 / 过期 / 不可查
THRESHOLD_EXCEEDED = "threshold_exceeded"    # 协方差 / 一致性超阈值
UNVERIFIED_THRESHOLDS = "unverified_thresholds"  # 阈值未标定
HEALTHY = "healthy"

# 非健康态全集（发布 False 的状态）
UNHEALTHY_STATES = (
    INIT, EVIDENCE_INSUFFICIENT, INPUT_ABNORMAL, TIME_ABNORMAL,
    TF_ABNORMAL, THRESHOLD_EXCEEDED, UNVERIFIED_THRESHOLDS,
)


@dataclass
class Thresholds:
    """定位健康判据阈值，全部外置。verified=False 表示尚未现场标定，状态机一律不判 HEALTHY。"""
    verified: bool = False

    # 断流超时（秒，单调时钟接收时刻）
    scan_timeout: float = 1.0     # /scan 断流
    tf_timeout: float = 1.0       # TF map→base_footprint 过期
    odom_timeout: float = 1.0     # /odom_combined 断流（运动状态未知）

    # AMCL 更新判据：按「自上次 AMCL 更新以来的累计运动 + 持续时间」判定，
    # 不用瞬时速度 + 固定超时（静止/微动时 AMCL 不更新是设计行为，不是失效）。
    # motion_cum_linear/angular 应取现役 AMCL update_min_d/a 的 2~3 倍（现役值待现场取证：
    # 本地 amcl_realcar.yaml=0.25/0.2618、nav_safe.yaml=0.025/0.06 两套并存，须确认车端跑哪套）。
    amcl_stale_moving: float = 1.0      # 累计运动已超门限后，AMCL 仍不更新的容忍时间(秒)
    amcl_stale_static: float = 10.0     # 弃用：静止不更新≠失效，保留字段兼容，不参与判定
    motion_cum_linear: float = 0.5      # 自上次 AMCL 更新以来的累计位移门限(米)
    motion_cum_angular: float = 0.5     # 累计转角门限(弧度)

    # 运动判定阈值（弃用：瞬时速度判据已被累计运动取代，保留字段兼容）
    motion_linear_thresh: float = 0.05    # 线速度 (m/s)
    motion_angular_thresh: float = 0.05   # 角速度 (rad/s)

    # 协方差 σ 阈值（区分单位，不混比）
    cov_pos_sigma: float = 0.5        # 位置 σ，米
    cov_yaw_sigma: float = 0.5        # 角度 σ，弧度

    # 一致性（辅助检查，不作定位正确独立证明）
    consistency_dist: float = 0.5     # 米：AMCL pose vs (odom 经 map→odom TF 转 map 系)

    # header.stamp 回退容忍（秒），同一话题回退超过此值记异常
    stamp_regress_tol: float = 0.5


@dataclass
class Verdict:
    healthy: bool
    state: str
    reasons: list = field(default_factory=list)


def _yaw_of(qw, qx, qy, qz):
    """四元数 → yaw（绕 z 轴）。"""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _wrap_pi(a):
    """角度归一到 (-π, π]。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _q_norm_ok(qw, qx, qy, qz):
    """四元数范数校验：全有限且范数≈1。"""
    n2 = qw * qw + qx * qx + qy * qy + qz * qz
    return math.isfinite(n2) and abs(math.sqrt(n2) - 1.0) <= 1e-2


class LocalizationHealth:
    """证据 → 判定的纯逻辑。所有时间由外部传入（单调时钟秒），不自己取时钟。

    输入（由 localization_health.py 从 ROS 剥字段后调用）：
    - on_amcl_pose：/amcl_pose 位置 + 四元数 + 协方差（cov[0]=xx, [7]=yy, [35]=yaw）
    - on_odom：/odom_combined 位置(odom 系) + 四元数 + twist 速度
    - on_scan：/scan 到达
    - on_tf_base：TF map→base_footprint 查询结果
    - on_tf_odom：TF map→odom_combined 查询结果（一致性换算用）
    - evaluate(now)：给定当前单调时钟，输出 Verdict
    """

    def __init__(self, thresholds: Thresholds | None = None):
        self.t = thresholds or Thresholds()
        # 最近一帧证据（recv_t = 单调时钟接收时刻，秒）
        self.amcl = None     # (recv_t, x, y, cov_xx, cov_yy, cov_yaw)
        self.odom = None     # (recv_t, x, y, yaw, vx, vy, wz)
        self.scan_t = None   # recv_t
        self.tf_base = None  # (recv_t, ok, x, y)         ok=False 表示不可查
        self.tf_odom = None  # (recv_t, ok, dx, dy, dyaw)  map→odom_combined
        # 逐帧异常（随下一帧正常自动恢复，不锁死）
        self._input_bad = {}   # source -> [reasons]
        self._stamp_bad = {}   # topic -> reason
        self._last_stamp = {}  # topic -> 上次 header.stamp
        # 上次 AMCL 更新时刻的 odom 位姿快照（累计运动起点），(x, y, yaw) 或 None
        self._amcl_odom_ref = None

    # ------------------------------------------------------------------ 输入
    def on_amcl_pose(self, recv_t, x, y, qw, qx, qy, qz,
                     cov_xx, cov_yy, cov_yaw, stamp=None):
        self._check_stamp("amcl_pose", stamp)
        bad = []
        for name, v in (("x", x), ("y", y)):
            if not math.isfinite(v):
                bad.append("amcl_pose.%s=%r 非有限值" % (name, v))
        for name, v in (("cov_xx", cov_xx), ("cov_yy", cov_yy), ("cov_yaw", cov_yaw)):
            if not math.isfinite(v) or v < 0:
                bad.append("amcl_pose %s=%r 无效" % (name, v))
        if not _q_norm_ok(qw, qx, qy, qz):
            bad.append("amcl_pose 四元数范数异常 (%.4f,%.4f,%.4f,%.4f)" % (qw, qx, qy, qz))
        self._set_input_bad("amcl", bad)
        self.amcl = (recv_t, x, y, cov_xx, cov_yy, cov_yaw)
        # 记录本次 AMCL 更新时的 odom 位姿作为累计运动起点
        self._amcl_odom_ref = (self.odom[1], self.odom[2], self.odom[3]) if self.odom else None

    def on_odom(self, recv_t, x, y, qw, qx, qy, qz, vx, vy, wz, stamp=None):
        self._check_stamp("odom", stamp)
        bad = []
        for name, v in (("x", x), ("y", y), ("vx", vx), ("vy", vy), ("wz", wz)):
            if not math.isfinite(v):
                bad.append("odom.%s=%r 非有限值" % (name, v))
        if not _q_norm_ok(qw, qx, qy, qz):
            bad.append("odom 四元数范数异常 (%.4f,%.4f,%.4f,%.4f)" % (qw, qx, qy, qz))
        self._set_input_bad("odom", bad)
        self.odom = (recv_t, x, y, _yaw_of(qw, qx, qy, qz), vx, vy, wz)
        # amcl 先到、odom 后到时补记累计运动起点
        if self._amcl_odom_ref is None and self.amcl is not None:
            self._amcl_odom_ref = (x, y, self.odom[3])

    def on_scan(self, recv_t, stamp=None):
        self._check_stamp("scan", stamp)
        self.scan_t = recv_t

    def on_tf_base(self, recv_t, ok, x, y, stamp=None):
        self._check_stamp("tf_base", stamp)
        bad = []
        if ok:
            for name, v in (("x", x), ("y", y)):
                if not math.isfinite(v):
                    bad.append("tf_base.%s=%r 非有限值" % (name, v))
        self._set_input_bad("tf_base", bad)
        self.tf_base = (recv_t, ok, x, y)

    def on_tf_odom(self, recv_t, ok, dx, dy, dyaw, stamp=None):
        self._check_stamp("tf_odom", stamp)
        bad = []
        if ok:
            for name, v in (("dx", dx), ("dy", dy), ("dyaw", dyaw)):
                if not math.isfinite(v):
                    bad.append("tf_odom.%s=%r 非有限值" % (name, v))
        self._set_input_bad("tf_odom", bad)
        self.tf_odom = (recv_t, ok, dx, dy, dyaw)

    def _set_input_bad(self, source, reasons):
        if reasons:
            self._input_bad[source] = reasons
        else:
            self._input_bad.pop(source, None)

    def _check_stamp(self, topic, stamp):
        """header.stamp 回退检测（仅单调性，不作绝对时间基准）。"""
        if stamp is None:
            return
        prev = self._last_stamp.get(topic)
        if prev is not None and stamp < prev - self.t.stamp_regress_tol:
            self._stamp_bad[topic] = (
                "%s header.stamp 回退 %r -> %r (回退 %.3fs)"
                % (topic, prev, stamp, prev - stamp))
        else:
            self._stamp_bad.pop(topic, None)
        self._last_stamp[topic] = stamp

    # ------------------------------------------------------------------ 判定
    def evaluate(self, now):
        t = self.t
        # 0. 阈值未标定：绝不判健康（不能偷偷用默认值放行）
        if not t.verified:
            return Verdict(False, UNVERIFIED_THRESHOLDS,
                           ["阈值未标定(verified=false)，禁止判健康；现场标定后置 true"])
        # 1. 最近一帧输入异常
        if self._input_bad:
            reasons = [r for rs in self._input_bad.values() for r in rs]
            return Verdict(False, INPUT_ABNORMAL, reasons)
        if self._stamp_bad:
            return Verdict(False, TIME_ABNORMAL, list(self._stamp_bad.values()))
        # 2. 证据充足
        if self.amcl is None:
            return Verdict(False, EVIDENCE_INSUFFICIENT, ["未收到 /amcl_pose"])
        if self.scan_t is None:
            return Verdict(False, EVIDENCE_INSUFFICIENT, ["未收到 /scan"])
        if self.odom is None:
            return Verdict(False, EVIDENCE_INSUFFICIENT, ["未收到 /odom_combined"])
        if self.tf_base is None or not self.tf_base[1]:
            return Verdict(False, TF_ABNORMAL, ["TF map→base_footprint 不可查"])
        if self.tf_odom is None or not self.tf_odom[1]:
            return Verdict(False, TF_ABNORMAL, ["TF map→odom_combined 不可查(一致性无法核验)"])
        # 3. 断流（新鲜度，单调时钟）
        if now - self.scan_t > t.scan_timeout:
            return Verdict(False, TIME_ABNORMAL,
                           ["/scan 断流 %.2fs > %.2fs" % (now - self.scan_t, t.scan_timeout)])
        if now - self.tf_base[0] > t.tf_timeout:
            return Verdict(False, TF_ABNORMAL,
                           ["TF map→base_footprint 过期 %.2fs > %.2fs"
                            % (now - self.tf_base[0], t.tf_timeout)])
        if now - self.odom[0] > t.odom_timeout:
            return Verdict(False, TIME_ABNORMAL,
                           ["/odom_combined 断流 %.2fs，运动状态未知" % (now - self.odom[0])])
        # 4. AMCL 更新 vs 累计运动（自上次 AMCL 更新以来），不用瞬时速度+固定超时：
        #    累计运动超过门限 → AMCL 早该更新；若持续超 amcl_stale_moving 仍不更新 = 异常。
        #    累计运动未达门限（静止/微动）→ AMCL 不更新是设计行为，不判失效。
        amcl_age = now - self.amcl[0]
        if self._amcl_odom_ref is not None:
            rx, ry, ryaw = self._amcl_odom_ref
            ox, oy, oyaw = self.odom[1], self.odom[2], self.odom[3]
            cum_linear = math.hypot(ox - rx, oy - ry)
            cum_angular = abs(_wrap_pi(oyaw - ryaw))
            if (cum_linear > t.motion_cum_linear or cum_angular > t.motion_cum_angular) \
                    and amcl_age > t.amcl_stale_moving:
                return Verdict(False, TIME_ABNORMAL,
                               ["累计运动 %.2fm/%.2frad 已超门限(%.2fm/%.2frad)，"
                                "AMCL %.2fs 仍不更新 > %.2fs"
                                % (cum_linear, cum_angular, t.motion_cum_linear,
                                   t.motion_cum_angular, amcl_age, t.amcl_stale_moving)])
        # 5. 协方差（位置米 / 角度弧度分开，不混比）
        cov_xx, cov_yy, cov_yaw = self.amcl[3], self.amcl[4], self.amcl[5]
        pos_sigma = math.sqrt(max(cov_xx, cov_yy))
        if pos_sigma > t.cov_pos_sigma:
            return Verdict(False, THRESHOLD_EXCEEDED,
                           ["位置 σ=%.2fm > %.2fm" % (pos_sigma, t.cov_pos_sigma)])
        yaw_sigma = math.sqrt(cov_yaw)
        if yaw_sigma > t.cov_yaw_sigma:
            return Verdict(False, THRESHOLD_EXCEEDED,
                           ["角度 σ=%.2frad > %.2frad" % (yaw_sigma, t.cov_yaw_sigma)])
        # 6. 一致性（辅助）：odom 位置经 map→odom_combined TF 转到 map 系，与 AMCL pose 比
        ox, oy = self.odom[1], self.odom[2]
        dx, dy, dyaw = self.tf_odom[2], self.tf_odom[3], self.tf_odom[4]
        c, s = math.cos(dyaw), math.sin(dyaw)
        odom_in_map_x = ox * c - oy * s + dx
        odom_in_map_y = ox * s + oy * c + dy
        dist = math.hypot(self.amcl[1] - odom_in_map_x, self.amcl[2] - odom_in_map_y)
        if dist > t.consistency_dist:
            return Verdict(False, THRESHOLD_EXCEEDED,
                           ["AMCL↔odom 一致性 %.2fm > %.2fm(辅助判据,非定位正确独立证明)"
                            % (dist, t.consistency_dist)])
        # 7. 全部通过 → 健康
        return Verdict(True, HEALTHY, [])


def verdict_to_bool(v: Verdict) -> bool:
    """Verdict → /localization_ok 的 Bool 值。仅 HEALTHY 为 True。"""
    return v.state == HEALTHY
