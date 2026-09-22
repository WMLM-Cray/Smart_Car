#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""localization_health.py —— /localization_ok 定位健康发布端节点（主线，2026-09-20）。

只做「订阅 + TF 查询 + 单调时钟 + 状态机 + 发布」，判定逻辑全在 loc_health_core.py（纯 Python，
可离线单测）。本节点不发布 /cmd_vel、不控制底盘、不保证停车——停车由主线安全链负责。

输入：
- /amcl_pose   (geometry_msgs/PoseWithCovarianceStamped, transient_local+reliable)
- /odom_combined (nav_msgs/Odometry)
- /scan        (sensor_msgs/LaserScan)
- TF map→base_footprint（最终定位）、TF map→odom_combined（一致性换算）

输出：
- /localization_ok         (std_msgs/Bool)  持续发布，True 只在全部判据满足时发
- /localization_health_diag (std_msgs/String) JSON 诊断，含 state + reasons，供影子运行记录

失效安全语义：
- 启动、配置不完整、关键证据缺失时**不发 True**。
- 阈值外置；verified=false（未现场标定）时一律不发 True。
- 持续输出当前判定（2Hz），任务器 watchdog 监控本节点失联（两种超时分开）。
- 发布 False 只触发任务器既有处置，不宣称能保证停车。

时间基准：所有新鲜度用 time.monotonic()（单调时钟），不依赖消息 header.stamp（车机钟慢、
latch 帧 stamp 可落后）。header.stamp 只用于回退检测。

用法（车机/WSL）：
  python3 localization_health.py --ros-args -p thresholds_file:=/path/localization_health.yaml
  # 或逐项 ROS 参数覆盖（含 verified:=true 表示阈值已现场标定）
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from loc_health_core import (
    HEALTHY, LocalizationHealth, Thresholds, verdict_to_bool,
)

# Thresholds 的字段名（与 ROS 参数名、YAML key 对齐）
_THRESHOLD_KEYS = (
    "verified",
    "scan_timeout", "tf_timeout", "odom_timeout",
    "amcl_stale_moving", "amcl_stale_static",
    "motion_cum_linear", "motion_cum_angular",
    "motion_linear_thresh", "motion_angular_thresh",
    "cov_pos_sigma", "cov_yaw_sigma",
    "consistency_dist", "stamp_regress_tol",
)


def _stamp_to_sec(stamp) -> float | None:
    """builtin_interfaces/Time → 秒；stamp 为 None 返回 None（供回退检测，TF 传 None）。"""
    if stamp is None:
        return None
    return stamp.sec + stamp.nanosec * 1e-9


def _yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class LocalizationHealthNode(Node):
    def __init__(self, overrides=None):
        super().__init__("localization_health")
        # 阈值参数（外置；默认 verified=false 未标定）
        for k in _THRESHOLD_KEYS:
            # verified 用 bool 默认值(False)，否则声明成 double 会让 overrides 的 bool 失效
            default = False if k == "verified" else 1.0
            self.declare_parameter(k, default)
        # 话题 / frame / 节奏
        self.declare_parameter("thresholds_file", "")
        self.declare_parameter("amcl_topic", "/amcl_pose")
        self.declare_parameter("odom_topic", "/odom_combined")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("loc_topic", "/localization_ok")
        self.declare_parameter("diag_topic", "/localization_health_diag")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("odom_frame", "odom_combined")
        self.declare_parameter("publish_rate", 2.0)

        # 测试/脚本注入参数（declare 后、取值前应用，风格同 mock_nav2/executor_node）
        if overrides:
            from rclpy.parameter import Parameter
            self.set_parameters([Parameter(k, value=v) for k, v in overrides.items()])

        p = lambda n: self.get_parameter(n).value
        self.map_frame = p("map_frame")
        self.base_frame = p("base_frame")
        self.odom_frame = p("odom_frame")
        self.publish_rate = float(p("publish_rate"))
        self.diag_topic = p("diag_topic")

        # 状态机核心（纯 Python）
        self.core = LocalizationHealth(self._load_thresholds())

        # TF
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 订阅
        self.create_subscription(
            PoseWithCovarianceStamped, p("amcl_topic"), self._on_amcl,
            self._qos_amcl())
        self.create_subscription(Odometry, p("odom_topic"), self._on_odom, 10)
        self.create_subscription(LaserScan, p("scan_topic"), self._on_scan, 10)

        # 发布
        self._loc_pub = self.create_publisher(Bool, p("loc_topic"), 10)
        self._diag_pub = self.create_publisher(String, self.diag_topic, 10)

        # 持续输出（2Hz），任务器 watchdog 靠它监控本节点失联
        self.create_timer(1.0 / self.publish_rate, self._tick)

        self._last_state = None
        self.get_logger().info(
            "定位健康发布端就绪: amcl=%s odom=%s scan=%s -> %s (diag=%s), 发布率 %.1fHz, "
            "verified=%s"
            % (p("amcl_topic"), p("odom_topic"), p("scan_topic"), p("loc_topic"),
               self.diag_topic, self.publish_rate, self.core.t.verified))
        if not self.core.t.verified:
            self.get_logger().warn(
                "阈值未标定(verified=false)：本节点将**永不发 True**，仅供影子运行记录判定。"
                "现场标定后置 verified=true 才可能发 True。")
        self.get_logger().info(
            "任务器侧须显式 localization_watchdog > 0（建议 2.0s），与本节点发布节奏分开，"
            "发布端失联才由任务器 watchdog 判 STALE。")

    # ------------------------------------------------------------------ 阈值加载
    def _qos_amcl(self):
        """/amcl_pose 实际 QoS = transient_local + reliable（amcl_watch.py 实车实测）。"""
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        return QoSProfile(depth=1,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def _load_thresholds(self) -> Thresholds:
        params = {k: self.get_parameter(k).value for k in _THRESHOLD_KEYS}
        f = self.get_parameter("thresholds_file").value or ""
        if f and os.path.isfile(f):
            try:
                import yaml
                with open(f, "r", encoding="utf-8") as fh:
                    data = yaml.safe_load(fh) or {}
                for k in _THRESHOLD_KEYS:
                    if k in data:
                        params[k] = data[k]
                self.get_logger().info("阈值已从 %s 加载" % f)
            except Exception as e:
                self.get_logger().warn("加载阈值文件 %s 失败: %s（用 ROS 参数默认值）" % (f, e))
        return Thresholds(**params)

    # ------------------------------------------------------------------ 订阅回调
    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        t = time.monotonic()
        o = msg.pose.pose.orientation
        cov = msg.pose.covariance
        # cov 至少 36 元素：cov[0]=xx, [7]=yy, [35]=yaw（PoseWithCovariance 标准布局）
        self.core.on_amcl_pose(
            t, msg.pose.pose.position.x, msg.pose.pose.position.y,
            o.w, o.x, o.y, o.z,
            cov[0], cov[7], cov[35],
            stamp=_stamp_to_sec(msg.header.stamp))

    def _on_odom(self, msg: Odometry):
        t = time.monotonic()
        o = msg.pose.pose.orientation
        self.core.on_odom(
            t, msg.pose.pose.position.x, msg.pose.pose.position.y,
            o.w, o.x, o.y, o.z,
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z,
            stamp=_stamp_to_sec(msg.header.stamp))

    def _on_scan(self, msg: LaserScan):
        self.core.on_scan(time.monotonic(), stamp=_stamp_to_sec(msg.header.stamp))

    # ------------------------------------------------------------------ TF + 判定 + 发布
    def _lookup(self, target, source):
        """查最新 transform，返回 (ok, x, y, yaw, stamp_sec)。失败返回 (False, 0, 0, 0, None)。"""
        try:
            tr = self._tf_buffer.lookup_transform(target, source, Time())
            q = tr.transform.rotation
            return (True, tr.transform.translation.x, tr.transform.translation.y,
                    _yaw_from_quat(q.x, q.y, q.z, q.w), _stamp_to_sec(tr.header.stamp))
        except Exception:
            return (False, 0.0, 0.0, 0.0, None)

    def _tick(self):
        now = time.monotonic()
        # TF：map→base_footprint（最终定位）、map→odom_combined（一致性换算）
        ok, x, y, _, tf_base_stamp = self._lookup(self.map_frame, self.base_frame)
        self.core.on_tf_base(now, ok, x, y, stamp=tf_base_stamp)   # stamp 供回退检测
        ok, dx, dy, dyaw, tf_odom_stamp = self._lookup(self.map_frame, self.odom_frame)
        self.core.on_tf_odom(now, ok, dx, dy, dyaw, stamp=tf_odom_stamp)

        v = self.core.evaluate(now)
        self._loc_pub.publish(Bool(data=verdict_to_bool(v)))

        diag = json.dumps({"state": v.state, "healthy": v.healthy,
                           "reasons": v.reasons, "ts": now})
        self._diag_pub.publish(String(data=diag))

        # 状态变化时打日志（持续输出靠 publish，日志只打变化，避免刷屏）。
        # 统一用 warn：rclpy 同一 RcutilsLogger 在 info/warn 间切换会抛
        # 'Logger severity cannot be changed between calls'，故固定单一级别。
        if v.state != self._last_state:
            self.get_logger().warn("定位健康 -> %s%s" %
                                   (v.state, (" | " + "; ".join(v.reasons)) if v.reasons else ""))
            self._last_state = v.state


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationHealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
