#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# camera_monitor.py —— 车机本机相机帧只读渲染: /image_raw -> PNG (SSH 拉取显示用)
# 2026-09-21 收口轮新增, 与 map_monitor.py 同款设计:
#   - 只读订阅, 不动控制链、不改相机标定、不写内核驱动、不加遥控。
#   - 相机采集仍由厂家 usb_cam 节点完成(vendor camer.launch.py: /dev/video0, 320x240 yuyv,
#     frame_id=camera), 本脚本只把已发布的最新图像帧渲染成 PNG。
#   - 地图窗口 = map_monitor.py(已交付); 视频窗口 = 本脚本。各自独立 PNG, 分窗口显示。
#
# 图像源约定: 厂家 usb_cam 发布 /image_raw(sensor_msgs/Image)。本脚本默认订 /image_raw,
#   可用 --image-topic 覆盖。现场实际话题/编码以 `ros2 topic info <话题> -v` 实测为准,
#   不预设、不硬改编码(见 _to_rgb 的编码自适应)。
#
# 编码支持(读 msg.encoding 自适应, 不猜): mono8/rgb8/bgr8/16uc1 完整; yuyv/uyvy 仅取亮度
#   Y 通道灰度(4:2:2 无色彩重建, 只保证看得见)。未知编码 -> 占位提示, 不崩。
# 新鲜度: 用墙钟 time.time() 相对差值(车机钟慢 5 个月, 相对差值可用), 超 2s 标 [STALE]。
# 退出: SIGINT/SIGTERM 自接管(同 map_monitor.py 6bb97d8 的坑)。
import argparse, os, signal, sys, time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

DEFAULT_OUT = os.path.expanduser("~/racecar_team/logs/slam/camera.png")
OUT = DEFAULT_OUT
TMP = DEFAULT_OUT + ".tmp.png"


def _to_rgb(msg):
    """sensor_msgs/Image -> (h,w,3) uint8 RGB; 解码不了/不认识返回 None(不崩)。"""
    try:
        w, h = msg.width, msg.height
        enc = msg.encoding
        if enc in ("rgb8",):
            return np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
        if enc in ("bgr8",):
            return np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)[:, :, ::-1].copy()
        if enc in ("mono8",):
            g = np.frombuffer(msg.data, np.uint8).reshape(h, w)
            return np.repeat(g[:, :, None], 3, axis=2)
        if enc in ("yuyv", "uyvy"):   # 4:2:2, 只取亮度 Y 作灰度
            step = msg.step if msg.step else w * 2
            buf = np.frombuffer(msg.data, np.uint8).reshape(h, step)[:, :w * 2]
            idx = 0 if enc == "yuyv" else 1   # yuyv: Y 在偶数位; uyvy: Y 在奇数位
            g = buf[:, idx::2]
            return np.repeat(g[:, :, None], 3, axis=2)
        if enc in ("16uc1",):
            d = np.frombuffer(msg.data, np.uint16).reshape(h, w).astype(np.float32)
            d = np.clip(d / 255.0, 0, 255).astype(np.uint8)
            return np.repeat(d[:, :, None], 3, axis=2)
    except Exception:
        return None
    return None


class CameraMonitor(Node):
    def __init__(self, image_topic):
        super().__init__("camera_monitor")
        self.image_topic = image_topic
        self.create_subscription(Image, image_topic, self.on_image, 10)
        self.frame = None            # 最新 (h,w,3) uint8 RGB, 解不出为 None
        self.last_recv = None        # 最后收到帧的墙钟时刻
        self.last_encoding = None
        self.last_shape = None
        self.get_logger().info("订阅图像话题: %s (只读渲染, 不动控制链)" % image_topic)

    def on_image(self, msg):
        self.last_recv = time.time()
        self.last_encoding = msg.encoding
        self.last_shape = (msg.width, msg.height)
        self.frame = _to_rgb(msg)


def render(mon):
    fig, ax = plt.subplots(figsize=(6.4, 4.8), dpi=100)   # 4:3 (与 320x240 同比例)
    fig.patch.set_facecolor("black")
    ax.axis("off")
    if mon.frame is not None:
        ax.imshow(mon.frame)
        if mon.last_shape:
            ax.set_title("image: %s %sx%s" % (mon.last_encoding,
                                              mon.last_shape[0], mon.last_shape[1]),
                         color="white", fontsize=8)
    else:
        ax.text(0.5, 0.5, "无画面 (尚无图像帧)", ha="center", va="center",
                color="white", fontsize=12, transform=ax.transAxes)
    # 左下角新鲜度(同 map_monitor 的权威判据): image age > 2s 标红
    if mon.last_recv is not None:
        age = time.time() - mon.last_recv
        stale = age > 2.0
        txt = "image: %.1fs ago %s" % (age, "[STALE]" if stale else "[ok]")
        col = "red" if stale else "lime"
    else:
        txt, col = "image: 无数据 [STALE]", "red"
    ax.text(0.02, 0.02, txt, transform=ax.transAxes, color=col, fontsize=9,
            va="bottom", fontweight="bold",
            bbox=dict(facecolor="black", alpha=0.6, edgecolor="none"))
    fig.savefig(TMP, bbox_inches="tight", facecolor="black")
    plt.close(fig)
    os.replace(TMP, OUT)   # 原子替换, 拉图拉不到半张


def main():
    parser = argparse.ArgumentParser(description="车机相机帧只读渲染 (image->PNG)")
    parser.add_argument("--image-topic", default="/image_raw",
                        help="图像话题名(默认 /image_raw, 现场以 ros2 topic info 实测为准)")
    parser.add_argument("--out", default=DEFAULT_OUT, help="输出 PNG 路径")
    args, unknown = parser.parse_known_args()
    global OUT, TMP
    OUT = args.out
    TMP = OUT + ".tmp.png"
    rclpy.init(args=[sys.argv[0]] + unknown)
    mon = CameraMonitor(args.image_topic)
    running = {"v": True}

    def on_sig(sig, frame):
        running["v"] = False

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)
    last = 0.0
    while running["v"]:
        rclpy.spin_once(mon, timeout_sec=0.2)
        now = time.time()
        if now - last > 2.0:
            try:
                render(mon)
            except Exception as e:
                print("render err:", e, file=sys.stderr)
            last = now
    mon.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()