#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""loc_health_integration_test.py —— 定位健康发布端 ROS 隔离集成测试（需 ROS2 环境执行）。

本机 Windows 无 rclpy，本脚本在 WSL(ubuntu2204-ros2) 或车机执行，验证发布端 ROS 胶水层：
  1. 启动无数据 → /localization_ok = False（证据不足，不发 True）
  2. 喂健康证据（静态 TF + amcl/odom/scan）→ True
  3. 停喂 /scan → False（断流）
  4. 停喂 /odom → False（运动状态未知）
  5. 恢复喂全部 → True（恢复不锁死）
  6. header.stamp 回退 → False（时间异常）
  7. verified=false → 证据齐全也不 True（未标定）
  8. 影子话题：loc_topic=/localization_ok_shadow → 主话题无消息、影子话题有消息
  9. 发布端退出 → 停止发布（任务器 watchdog 判 STALE 的前提）

用法（WSL/车机，需 source 后执行）：
  python3 loc_health_integration_test.py

注：状态机纯逻辑已由 test_loc_health_core.py 离线 20/20 覆盖；本脚本只验 ROS 订阅/TF/发布胶水层。
"""
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster

from localization_health import LocalizationHealthNode


def _tf(parent, child, x, y, stamp):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.rotation.w = 1.0
    return t


def _amcl_msg(stamp):
    m = PoseWithCovarianceStamped()
    m.header.stamp = stamp
    m.header.frame_id = "map"
    m.pose.pose.position.x = 1.0
    m.pose.pose.position.y = 2.0
    m.pose.pose.orientation.w = 1.0
    m.pose.covariance[0] = 0.04
    m.pose.covariance[7] = 0.09
    m.pose.covariance[35] = 0.01
    return m


def _odom_msg(stamp):
    m = Odometry()
    m.header.stamp = stamp
    m.header.frame_id = "odom_combined"
    m.pose.pose.orientation.w = 1.0
    m.twist.twist.linear.x = 0.0
    m.twist.twist.angular.z = 0.0
    return m


def _scan_msg(stamp):
    m = LaserScan()
    m.header.stamp = stamp
    return m


def main():
    rclpy.init()
    driver = rclpy.create_node("loc_health_itest")
    tfb = StaticTransformBroadcaster(driver)
    tfb.sendTransform([
        _tf("map", "odom_combined", 1.0, 2.0, driver.get_clock().now().to_msg()),
        _tf("odom_combined", "base_footprint", 0.0, 0.0, driver.get_clock().now().to_msg()),
    ])

    # /amcl_pose 发布端须与订阅端 QoS 对齐（transient_local + reliable），否则 DURABILITY 不兼容收不到
    _qos_amcl = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub_amcl = driver.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", _qos_amcl)
    pub_odom = driver.create_publisher(Odometry, "/odom_combined", 10)
    pub_scan = driver.create_publisher(LaserScan, "/scan", 10)

    S = {"ok": None, "main_count": 0, "shadow_count": 0}

    def on_main(m):
        S["ok"] = bool(m.data)
        S["main_count"] += 1

    def on_shadow(m):
        S["shadow_count"] += 1

    driver.create_subscription(Bool, "/localization_ok", on_main, 10)
    driver.create_subscription(Bool, "/localization_ok_shadow", on_shadow, 10)

    def spin_once(secs, lh):
        end = time.time() + secs
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(driver, timeout_sec=0.02)
            if lh is not None:
                rclpy.spin_once(lh, timeout_sec=0.02)

    def feed_all(lh, secs):
        end = time.time() + secs
        while time.time() < end and rclpy.ok():
            now = driver.get_clock().now().to_msg()
            pub_amcl.publish(_amcl_msg(now))
            pub_odom.publish(_odom_msg(now))
            pub_scan.publish(_scan_msg(now))
            rclpy.spin_once(driver, timeout_sec=0.02)
            rclpy.spin_once(lh, timeout_sec=0.02)

    results = []

    # 1. 启动无数据 → False
    S["ok"], S["main_count"] = None, 0
    lh = LocalizationHealthNode(overrides={"verified": True})
    spin_once(1.5, lh)
    results.append(("无数据应 False", S["ok"] is False))
    lh.destroy_node()

    # 2. 健康证据 → True
    S["ok"] = None
    lh = LocalizationHealthNode(overrides={"verified": True})
    feed_all(lh, 1.5)
    results.append(("健康证据应 True", S["ok"] is True))

    # 3. 停 scan → False（断流）
    S["ok"] = None
    stop = time.time() + 2.0
    while time.time() < stop and rclpy.ok():
        pub_odom.publish(_odom_msg(driver.get_clock().now().to_msg()))
        rclpy.spin_once(driver, timeout_sec=0.02)
        rclpy.spin_once(lh, timeout_sec=0.02)
    spin_once(0.6, lh)  # 排空最后一条判定消息再断言
    results.append(("停 scan 应 False", S["ok"] is False))

    # 4. 停 odom → False（运动状态未知）；恢复 scan/amcl
    S["ok"] = None
    stop = time.time() + 2.0
    while time.time() < stop and rclpy.ok():
        now = driver.get_clock().now().to_msg()
        pub_amcl.publish(_amcl_msg(now))
        pub_scan.publish(_scan_msg(now))
        rclpy.spin_once(driver, timeout_sec=0.02)
        rclpy.spin_once(lh, timeout_sec=0.02)
    spin_once(0.6, lh)  # 排空最后一条判定消息再断言
    results.append(("停 odom 应 False", S["ok"] is False))

    # 5. 恢复全部 → True（不锁死）
    S["ok"] = None
    feed_all(lh, 1.5)
    results.append(("恢复应 True", S["ok"] is True))

    # 6. header.stamp 回退 → False（时间异常）
    S["ok"] = None
    feed_all(lh, 0.8)  # 先把 amcl stamp 抬到当前
    back = driver.get_clock().now() - Duration(seconds=2.0)
    pub_amcl.publish(_amcl_msg(back.to_msg()))
    spin_once(1.0, lh)
    results.append(("stamp 回退应 False", S["ok"] is False))
    lh.destroy_node()

    # 7. verified=false → 证据齐全也不 True（未标定）
    S["ok"] = None
    lh = LocalizationHealthNode(overrides={"verified": False})
    feed_all(lh, 1.5)
    results.append(("未标定应 False", S["ok"] is False))
    lh.destroy_node()

    # 8. 影子话题 → 主话题无消息、影子话题有消息
    S["main_count"], S["shadow_count"] = 0, 0
    spin_once(0.5, None)  # 排空阶段 7 残留的 /localization_ok 再清零
    S["main_count"], S["shadow_count"] = 0, 0
    lh = LocalizationHealthNode(overrides={
        "verified": True,
        "loc_topic": "/localization_ok_shadow",
        "diag_topic": "/localization_health_diag_shadow",
    })
    feed_all(lh, 1.5)
    results.append(("影子话题不干扰主话题", S["main_count"] == 0,
                    "main_count=%d" % S["main_count"]))
    results.append(("影子话题有消息", S["shadow_count"] > 0,
                    "shadow_count=%d" % S["shadow_count"]))

    # 9. 发布端退出 → 停止发布（任务器 watchdog 判 STALE 的前提）
    S["shadow_count"] = 0
    spin_once(0.5, lh)  # 排空队列 + 确认在发
    lh.destroy_node()
    S["shadow_count"] = 0
    spin_once(1.0, None)  # 只转 driver，无发布端
    results.append(("发布端退出后停止发布", S["shadow_count"] == 0,
                    "退出后新增 %d 条" % S["shadow_count"]))

    driver.destroy_node()
    rclpy.shutdown()

    print("=" * 70)
    ok_all = True
    for item in results:
        name, passed = item[0], item[1]
        detail = item[2] if len(item) > 2 else ""
        print(("PASS " if passed else "FAIL ") + name +
              (("  [%s]" % detail) if detail else ""))
        ok_all = ok_all and passed
    print("=" * 70)
    print("集成测试: %s" % ("全部通过" if ok_all else "存在失败"))
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
