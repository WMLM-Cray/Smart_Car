#!/usr/bin/env python3
# path_display.py —— 规划路径渲染的纯逻辑(无 ROS 依赖, 可离线单测)
# 2026-09-19 v3: 从 map_monitor.py 抽出, 便于本机(无 rclpy/nav_msgs/tf2_ros)做行为验证。
# 职责: ①路径 frame 对齐(与地图 frame 一致直用 / 不同走 TF / 无 TF 拒绝绘制, 绝不套坐标)
#       ②路径状态行文案(只标"最近收到的路径+年龄", 不暗示它仍是当前有效路径)。
import math

# 路径默认对齐到的地图坐标系(与 scan/轨迹所用的 map frame 一致)
MAP_FRAME = "map"


def transform_path_poses(poses, path_frame, tf_lookup):
    """把 path 的 poses 从 path_frame 对齐到地图 frame。

    参数:
      poses: 可迭代, 每项有 .pose.position.x / .pose.position.y(duck typing, 不 import nav_msgs)
      path_frame: 路径 header.frame_id
      tf_lookup: callable(target_frame, source_frame) -> (tx, ty, yaw) 弧度; 查不到须抛异常
    返回 (xy, err):
      - 对齐成功: xy=[(x,y),...] (空路径则 []), err=None
      - frame 不同且 TF 不可得: xy=[], err=拒绝绘制提示(绝不直接套坐标)
    """
    if path_frame == MAP_FRAME:
        return [(p.pose.position.x, p.pose.position.y) for p in poses], None
    try:
        tx, ty, yaw = tf_lookup(MAP_FRAME, path_frame)
    except Exception:
        return [], f"plan frame '{path_frame}' 无 TF→{MAP_FRAME}, 拒绝绘制"
    c, s = math.cos(yaw), math.sin(yaw)
    xy = []
    for p in poses:
        x, y = p.pose.position.x, p.pose.position.y
        xy.append((tx + x * c - y * s, ty + x * s + y * c))
    return xy, None


def path_status_text(mon, now):
    """路径状态行: 返回 (文本, 颜色) 或 None(未订阅 path 话题时不显示该行)。

    只标"最近一次收到的路径 + 年龄"; 不判 STALE(路径按需更新, 不持续是正常的),
    也不暗示它是当前仍有效的路径——本工具无导航状态订阅, 目标结束/取消后旧线可能仍显示。
    """
    if not getattr(mon, "path_topic", None):
        return None
    if getattr(mon, "path", None) is None:
        return ("plan: 无数据", "gray")
    a = now - mon.last_path_recv
    return (f"plan: 最近收到 {a:.0f}s ago", "gray")
