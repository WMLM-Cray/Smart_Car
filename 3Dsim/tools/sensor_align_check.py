#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[传感器对齐验收器] tools/sensor_align_check.py
=================================================
身份声明:实车传感器参数对齐(2026-09-16 任务书)的验收器——只订阅
/scan 与 /IMU_data 被动测量,不发布任何指令,不进控制链。

模式声明(文档统一口径):
  实车模式(用户 2026-09-16 确认):雷达发布 11.99Hz、每帧 1667 点;IMU 100Hz。
  仿真目标(取整,不为复现测量小数设 11.99):12Hz/1667 点/IMU 100Hz。

判据(全过才 PASS):
  ①/scan 按消息时间戳(header.stamp,仿真秒)频率 ∈ 12±1.2Hz——优先判据;
    接收端墙钟频率与实时因子一并记录,防把电脑运行慢误判成传感器频率错误;
  ②/scan 连续帧 ranges 长度全为 1667;
  ③/scan header.stamp 严格递增;
  ④角度字段自洽:|increment−(max−min)/(N−1)|≤1e-9(当前端点 ±π 含两端
    → 步进 2π/1666≈0.21608°,且首尾两条射线同方向、重复方向未消除——
    本判据只验证字段与数组长度自洽,不声称与实车角度约定一致);
  ⑤/scan frame_id 恒为 laser_sensor_link(话题名不变由本订阅本身保证);
  ⑥/IMU_data 时间戳频率 ∈ 100±10Hz;
  ⑦/IMU_data header.stamp 严格递增;
  ⑧/IMU_data frame_id 恒为 imu_sensor_link;
  ⑨采样窗口 ≥30s 仿真时间(按 scan 首末 stamp 计)。

尚未逐项对齐(记录不判 FAIL):实车 angle_min/angle_max/angle_increment/
scan_time/time_increment 数值未取得;当前采用的雷达插件/配置未模拟逐点
时间戳与运动畸变(扫描期间载具运动引起的点云畸变),仿真为瞬时整帧
扫描、时间字段输出 0——此为插件/配置现状,不扩大为平台整体无法实现。
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

TAG = "[传感器对齐验收器]"
SCAN_HZ_NOM, SCAN_HZ_TOL = 12.0, 1.2
IMU_HZ_NOM, IMU_HZ_TOL = 100.0, 10.0
EXPECT_SAMPLES = 1667
WINDOW_SIM_S = 30.0
WALL_TIMEOUT_S = 300.0          # 墙钟兜底:实时因子过低时防挂死
REPORT_PATH = [""]
RESULT = [""]


def stamp_sec(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class SensorAlignCheck(Node):
    def __init__(self):
        super().__init__("sensor_align_check")
        # best_effort 订阅兼容可靠/最佳努力两种发布端
        self.create_subscription(LaserScan, "/scan", self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, "/IMU_data", self.on_imu,
                                 qos_profile_sensor_data)
        self.scans = []          # (stamp_sim, wall, n_ranges, frame_id)
        self.imus = []           # (stamp_sim, wall, frame_id)
        self.scan_fields = None  # 首帧角度/时间字段(逐帧不变则代表全帧)
        self.done = False

    def on_scan(self, m):
        self.scans.append((stamp_sec(m.header), time.time(),
                           len(m.ranges), m.header.frame_id))
        if self.scan_fields is None:
            self.scan_fields = {
                "angle_min": m.angle_min, "angle_max": m.angle_max,
                "angle_increment": m.angle_increment,
                "scan_time": m.scan_time, "time_increment": m.time_increment,
                "range_min": m.range_min, "range_max": m.range_max,
            }
        if self.scans[-1][0] - self.scans[0][0] >= WINDOW_SIM_S:
            self.done = True

    def on_imu(self, m):
        self.imus.append((stamp_sec(m.header), time.time(),
                          m.header.frame_id))


def freqs(rows):
    """(按 stamp 频率, 按墙钟频率, 仿真跨度, 墙钟跨度)"""
    if len(rows) < 2:
        return None, None, 0.0, 0.0
    sim = rows[-1][0] - rows[0][0]
    wall = rows[-1][1] - rows[0][1]
    return (len(rows) - 1) / sim if sim > 0 else None, \
           (len(rows) - 1) / wall if wall > 0 else None, sim, wall


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", default="")
    args = ap.parse_args()
    REPORT_PATH[0] = args.report or ("/home/lzx/hwcart_sim/docs/validation"
                                      "/2026-09-16/sensor_align_report.json")
    rclpy.init()
    node = SensorAlignCheck()
    print(f"{TAG} 等 /scan 与 /IMU_data 数据(采样窗 {WINDOW_SIM_S:.0f}s 仿真时间)…")
    t0 = time.time()
    while not node.done and time.time() - t0 < WALL_TIMEOUT_S:
        rclpy.spin_once(node, timeout_sec=0.2)

    fails = []
    notes = []
    sc, im = node.scans, node.imus
    if not sc:
        fails.append("/scan 无数据")
    if not im:
        fails.append("/IMU_data 无数据")

    s_hz, s_wall, s_sim, s_wallspan = (freqs(sc) if sc
                                       else (None, None, 0.0, 0.0))
    i_hz, i_wall, i_sim, i_wallspan = (freqs(im) if im
                                       else (None, None, 0.0, 0.0))
    rtf = (s_sim / s_wallspan) if s_wallspan > 0 else None

    if s_hz is not None and not (SCAN_HZ_NOM - SCAN_HZ_TOL
                                 <= s_hz <= SCAN_HZ_NOM + SCAN_HZ_TOL):
        fails.append(f"/scan 时间戳频率 {s_hz:.3f}Hz 超出 {SCAN_HZ_NOM}±{SCAN_HZ_TOL}")
    if i_hz is not None and not (IMU_HZ_NOM - IMU_HZ_TOL
                                 <= i_hz <= IMU_HZ_NOM + IMU_HZ_TOL):
        fails.append(f"/IMU_data 时间戳频率 {i_hz:.3f}Hz 超出 {IMU_HZ_NOM}±{IMU_HZ_TOL}")

    n_set = sorted({r[2] for r in sc})
    if n_set != [EXPECT_SAMPLES]:
        fails.append(f"ranges 长度集合 {n_set} ≠ [{EXPECT_SAMPLES}]")
    frames_s = sorted({r[3] for r in sc})
    if frames_s != ["laser_sensor_link"]:
        fails.append(f"/scan frame_id 集合 {frames_s} ≠ [laser_sensor_link]")
    frames_i = sorted({r[2] for r in im})
    if frames_i != ["imu_sensor_link"]:
        fails.append(f"/IMU_data frame_id 集合 {frames_i} ≠ [imu_sensor_link]")

    if any(sc[k + 1][0] <= sc[k][0] for k in range(len(sc) - 1)):
        fails.append("/scan 时间戳非严格递增")
    if any(im[k + 1][0] <= im[k][0] for k in range(len(im) - 1)):
        fails.append("/IMU_data 时间戳非严格递增")

    angle_selfconsist = None
    if node.scan_fields and n_set == [EXPECT_SAMPLES]:
        f = node.scan_fields
        inc_exp = ((f["angle_max"] - f["angle_min"])
                   / (EXPECT_SAMPLES - 1))
        angle_selfconsist = abs(f["angle_increment"] - inc_exp) <= 1e-9
        if not angle_selfconsist:
            fails.append(f"角度不自洽:increment={f['angle_increment']:.9f} vs "
                         f"(max-min)/(N-1)={inc_exp:.9f}")
    if s_sim < WINDOW_SIM_S:
        fails.append(f"采样窗 {s_sim:.1f}s < {WINDOW_SIM_S:.0f}s(仿真时间)")

    notes.append("实车 angle_min/max/increment/scan_time/time_increment "
                 "数值未取得,本轮仅对齐频率与点数,角度/时间字段尚未逐项对齐;"
                 "仿真端点±π含两端=首尾同方向射线,重复方向未消除")
    notes.append("当前雷达插件/配置未模拟逐点时间与运动畸变(瞬时整帧扫描),"
                 "时间字段输出为0,不代表平台整体无法实现")
    if rtf is not None and rtf < 0.8:
        notes.append(f"实时因子 {rtf:.3f} 偏低:接收端频率将低于时间戳频率,"
                     "以时间戳频率为准")

    ok = not fails and len(sc) >= 2 and len(im) >= 2
    r = {
        "tag": TAG,
        "result": "PASS" if ok else "FAIL",
        "mode": {
            "实车模式": "雷达 11.99Hz/1667 点/帧;IMU 100Hz(用户 2026-09-16 确认)",
            "仿真目标": "12Hz/1667 点/帧;IMU 100Hz(取整,不复刻 11.99)",
        },
        "scan": {
            "n_frames": len(sc), "stamp_hz": round(s_hz, 3) if s_hz else None,
            "recv_wall_hz": round(s_wall, 3) if s_wall else None,
            "sim_span_s": round(s_sim, 1),
            "ranges_len_set": n_set,
            "frame_id_set": frames_s,
            "fields_first_frame": ({k: round(v, 9) for k, v in
                                    node.scan_fields.items()}
                                   if node.scan_fields else None),
            "angle_selfconsistent": angle_selfconsist,
            "topic": "/scan",
        },
        "imu": {
            "n_msgs": len(im), "stamp_hz": round(i_hz, 2) if i_hz else None,
            "recv_wall_hz": round(i_wall, 2) if i_wall else None,
            "sim_span_s": round(i_sim, 1),
            "frame_id_set": frames_i,
            "topic": "/IMU_data",
        },
        "realtime_factor": round(rtf, 3) if rtf else None,
        "criteria": ("scan stamp 12±1.2Hz;ranges=1667;stamp 严格递增;角度自洽;"
                     "frame_id 恒定;IMU stamp 100±10Hz 且递增;窗≥30s 仿真时间"),
        "fails": fails,
        "not_yet_aligned": notes,
    }
    print(f"{TAG} ===== 报告 {json.dumps(r, ensure_ascii=False)}")
    try:
        with open(REPORT_PATH[0], "w") as fh:
            json.dump(r, fh, ensure_ascii=False, indent=1)
    except OSError as e:
        print(f"{TAG} 报告写入失败({REPORT_PATH[0]}):{e}")
    node.destroy_node()
    rclpy.shutdown()
    RESULT[0] = r["result"]
    sys.exit(0 if RESULT[0] == "PASS" else 1)


if __name__ == "__main__":
    main()
