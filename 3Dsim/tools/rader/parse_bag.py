#!/usr/bin/env python3
"""从 rosbag2(.db3)里解出 LaserScan,导出成好用的格式。

本机没装 rosbags,所以直接实现 CDR 反序列化 —— 只需要 LaserScan 一种类型,
字段固定,几十行就够,且不依赖任何网络安装。

用法:
    python parse_bag.py carscan_bag/carscan_bag_0.db3            # 统计摘要
    python parse_bag.py carscan_bag/carscan_bag_0.db3 --npz      # 导出 scan.npz
    python parse_bag.py carscan_bag/carscan_bag_0.db3 --csv      # 导出 scan.csv
"""
from __future__ import annotations

import argparse
import sqlite3
import struct
import sys

import numpy as np


class Cdr:
    """极简 CDR 读取器(小端,无对齐异常处理)。

    CDR 的对齐是相对数据体开头的 —— 即跳过 4 字节 encapsulation header 之后。
    """

    def __init__(self, data: bytes):
        self.d = data
        self.i = 4                      # 跳过 encapsulation header

    def align(self, n: int) -> None:
        pad = (-self.i) % n
        self.i += pad

    def i32(self) -> int:
        self.align(4)
        v = struct.unpack_from("<i", self.d, self.i)[0]
        self.i += 4
        return v

    def u32(self) -> int:
        self.align(4)
        v = struct.unpack_from("<I", self.d, self.i)[0]
        self.i += 4
        return v

    def f32(self) -> float:
        self.align(4)
        v = struct.unpack_from("<f", self.d, self.i)[0]
        self.i += 4
        return v

    def string(self) -> str:
        n = self.u32()                  # 含结尾 \0
        raw = self.d[self.i:self.i + n]
        self.i += n
        return raw.split(b"\0", 1)[0].decode("utf-8", "replace")

    def f32_array(self) -> np.ndarray:
        n = self.u32()
        self.align(4)
        a = np.frombuffer(self.d, dtype="<f4", count=n, offset=self.i)
        self.i += 4 * n
        return a


def decode_laserscan(blob: bytes) -> dict:
    c = Cdr(blob)
    sec, nsec = c.i32(), c.u32()
    frame_id = c.string()
    return dict(
        stamp=sec + nsec * 1e-9,
        sec=sec,
        nsec=nsec,
        frame_id=frame_id,
        angle_min=c.f32(),
        angle_max=c.f32(),
        angle_increment=c.f32(),
        time_increment=c.f32(),
        scan_time=c.f32(),
        range_min=c.f32(),
        range_max=c.f32(),
        ranges=c.f32_array(),
        intensities=c.f32_array(),
    )


def load_scans(db3: str) -> list[dict]:
    con = sqlite3.connect(db3)
    tid = con.execute(
        "SELECT id FROM topics WHERE name='/scan'").fetchone()
    if tid is None:
        sys.exit(f"{db3} 里没有 /scan 话题")
    rows = con.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (tid[0],)).fetchall()
    out = []
    for ts, blob in rows:
        s = decode_laserscan(blob)
        s["bag_time"] = ts * 1e-9
        out.append(s)
    return out


def summarize(scans: list[dict]) -> None:
    if not scans:
        sys.exit("没有扫描数据")
    s0 = scans[0]
    n = len(s0["ranges"])
    print(f"帧数            : {len(scans)}")
    print(f"点数/帧         : {n}")
    print(f"frame_id        : {s0['frame_id']!r}")
    print(f"angle_min/max   : {s0['angle_min']:.6f} / {s0['angle_max']:.6f} rad")
    print(f"angle_increment : {s0['angle_increment']:.10f}  (2π/n = {2*np.pi/n:.10f})")
    print(f"range_min/max   : {s0['range_min']:.3f} / {s0['range_max']:.3f} m")
    print(f"scan_time       : {s0['scan_time']}  time_increment: {s0['time_increment']}")

    # 频率:用 bag 时间戳算,不用 header.stamp
    bt = np.array([s["bag_time"] for s in scans])
    dt = np.diff(bt)
    print()
    print(f"时长            : {bt[-1]-bt[0]:.2f} s")
    print(f"频率(平均)      : {1/dt.mean():.3f} Hz")
    print(f"周期抖动 std    : {dt.std()*1000:.2f} ms  (min {dt.min()*1000:.1f} / "
          f"max {dt.max()*1000:.1f})")

    # header 时间戳是否有效
    st = np.array([s["stamp"] for s in scans])
    print(f"header.stamp 范围: {st[0]:.3f} .. {st[-1]:.3f}  "
          f"({'有效' if st[0] > 0 else '全 0 —— 未填充'})")

    # 距离统计
    R = np.stack([s["ranges"] for s in scans])
    finite = np.isfinite(R)
    print()
    print(f"有效点占比      : {finite.mean()*100:.1f}%")
    if finite.any():
        v = R[finite]
        print(f"距离 min/mean/max: {v.min():.3f} / {v.mean():.3f} / {v.max():.3f} m")
    inf_frac = np.isinf(R).mean() * 100
    print(f"inf(超量程)占比 : {inf_frac:.1f}%")
    nan_frac = np.isnan(R).mean() * 100
    print(f"nan 占比        : {nan_frac:.2f}%")

    # 同一角度上的距离抖动 = 噪声水平(静止时)
    per_angle_std = np.nanstd(np.where(np.isfinite(R), R, np.nan), axis=0)
    ok = np.isfinite(per_angle_std)
    if ok.any():
        print(f"逐角度标准差中位: {np.median(per_angle_std[ok])*1000:.2f} mm "
              f"(静止时≈噪声;车在动则含运动)")


def export_npz(scans: list[dict], path: str) -> None:
    np.savez_compressed(
        path,
        ranges=np.stack([s["ranges"] for s in scans]),
        stamp=np.array([s["stamp"] for s in scans]),
        bag_time=np.array([s["bag_time"] for s in scans]),
        angle_min=scans[0]["angle_min"],
        angle_increment=scans[0]["angle_increment"],
        range_min=scans[0]["range_min"],
        range_max=scans[0]["range_max"],
        frame_id=scans[0]["frame_id"],
    )
    print(f"已写出 {path}")


def export_csv(scans: list[dict], path: str) -> None:
    """每行 = 一个点(t, 角度, 距离),便于 Excel / pandas 直接看。"""
    inc, amin = scans[0]["angle_increment"], scans[0]["angle_min"]
    angles = amin + np.arange(len(scans[0]["ranges"])) * inc
    with open(path, "w", encoding="utf-8") as f:
        f.write("frame,bag_time,angle_rad,angle_deg,range_m\n")
        for k, s in enumerate(scans):
            for a, r in zip(angles, s["ranges"]):
                f.write(f"{k},{s['bag_time']:.6f},{a:.6f},{np.degrees(a):.3f},{r}\n")
    print(f"已写出 {path}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("db3")
    ap.add_argument("--npz", action="store_true")
    ap.add_argument("--csv", action="store_true")
    a = ap.parse_args()

    scans = load_scans(a.db3)
    summarize(scans)
    if a.npz:
        export_npz(scans, "scan.npz")
    if a.csv:
        export_csv(scans, "scan.csv")


if __name__ == "__main__":
    main()
