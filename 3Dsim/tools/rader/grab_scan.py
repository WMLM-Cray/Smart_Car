#!/usr/bin/env python3
"""一键:在车上录 /scan → 传回本机 → 解析 → 出图 → 清理车上临时文件。

用法:
    python grab_scan.py                     # 录 20 秒,输出到 scan_<时间戳>/
    python grab_scan.py -d 30               # 录 30 秒
    python grab_scan.py -n cone_test        # 指定名字
    python grab_scan.py --no-plot           # 只取数据不画图
    python grab_scan.py --topics /scan /IMU_data

依赖同目录的 car_ssh.py(连接)和 .car_env(凭据)、parse_bag.py、view_scan.py。

开始录之前会先做雷达健康检查 —— 我们踩过的坑:USB 串口掉线重连后设备号变了
(ACM0→ACM1),驱动还攥着已删除的 fd,话题在、进程在、但一个字节都收不到。
这个脚本会提前发现并给出恢复命令,而不是白录一份空数据。
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SSH = HERE / "car_ssh.py"
DEFAULT_TOPICS = ["/scan", "/lslidar_point_cloud", "/tf", "/tf_static",
                  "/IMU_data", "/encoder", "/odom_combined"]


def sh(args: list[str], **kw) -> subprocess.CompletedProcess:
    """跑一条本机命令,文本模式。"""
    return subprocess.run(args, capture_output=True, text=True,
                          encoding="utf-8", errors="replace", **kw)


def car(cmd: str, check: bool = True) -> str:
    """在车上跑一条命令,返回 stdout。"""
    r = sh([sys.executable, str(SSH), cmd])
    if check and r.returncode != 0:
        sys.exit(f"车上命令失败(rc={r.returncode}):\n{r.stderr.strip()}")
    return r.stdout


def car_binary(remote: str, local: Path) -> None:
    """把车上文件按字节原样拉到本机。"""
    with open(local, "wb") as f:
        r = subprocess.run([sys.executable, str(SSH), f"cat {remote}"], stdout=f)
    if r.returncode != 0:
        sys.exit(f"拉取 {remote} 失败")


def health_check() -> None:
    """录之前确认雷达真的在出数据。不通过就退出并给出修复命令。"""
    print("[1/5] 雷达健康检查 ...")
    src = "source /opt/ros/humble/setup.bash 2>/dev/null; "

    # 驱动进程 + 它握着的串口 fd
    pid = car(src + "pgrep -f lslidar_driver_node | head -1", check=False).strip()
    if not pid:
        sys.exit("雷达驱动进程没在跑。先启动:\n"
                 "  sudo systemctl start racecar-chassis.service")
    fd = car(f"ls -l /proc/{pid}/fd/ 2>/dev/null | grep -oE '/dev/tty[A-Za-z0-9]+' | head -1",
             check=False).strip()
    print(f"      驱动 pid={pid}  串口={fd or '(无)'}")

    if not fd:
        sys.exit("驱动没有打开任何串口 —— 可能是串口掉了。恢复:\n"
                 "  sudo systemctl restart racecar-chassis.service")
    if "(deleted)" in fd:
        sys.exit(f"驱动握着已删除的串口 {fd} —— 就是掉线重连后 fd 失效那个问题。恢复:\n"
                 "  sudo systemctl restart racecar-chassis.service")

    # 真的收到数据了吗
    hz = car(src + "timeout 12 ros2 topic hz /scan 2>&1 | head -2", check=False)
    print(f"      /scan: {hz.strip().splitlines()[-1] if hz.strip() else '(无输出)'}")
    if "average rate" not in hz:
        sys.exit("/scan 没有数据流出来。恢复:\n"
                 "  sudo systemctl restart racecar-chassis.service")

    # 车在不在动(影响噪声/畸变分析的解读)
    v = car(src + "timeout 6 ros2 topic echo /encoder --field twist.twist.linear.x "
                  "--once 2>/dev/null | head -1", check=False).strip()
    print(f"      车速: {v or '(取不到)'}  "
          f"{'← 静止,底噪可直接用' if v in ('0.0', '0') else '← 在动'}")
    print()


def md5_of(path: Path) -> str:
    """算本地文件 MD5。用 hashlib 而不是 certutil —— 后者的输出随系统语言变,
    中文 Windows 上解析会挂。"""
    import hashlib
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--duration", type=float, default=20.0, help="录制秒数")
    ap.add_argument("-n", "--name", default=None, help="输出目录名(默认带时间戳)")
    ap.add_argument("--topics", nargs="+", default=DEFAULT_TOPICS)
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--no-health", action="store_true", help="跳过健康检查")
    a = ap.parse_args()

    if not (HERE / ".car_env").is_file():
        sys.exit(f"找不到 {HERE / '.car_env'},先按模板填好连接信息")

    if not a.no_health:
        health_check()

    name = a.name or f"scan_{time.strftime('%Y%m%d_%H%M%S')}"
    out = HERE / name
    out.mkdir(parents=True, exist_ok=True)
    remote = f"/tmp/{name}"
    topics = " ".join(a.topics)

    # ---- 录 ----
    print(f"[2/5] 车上录制 {a.duration:.0f} 秒 → {remote}")
    car(f"source /opt/ros/humble/setup.bash && rm -rf {remote} && "
        f"timeout {a.duration + 3:.0f} ros2 bag record -o {remote} {topics} "
        f">/dev/null 2>&1; true")
    listing = car(f"ls {remote}/ 2>&1", check=False)
    print(f"      {listing.strip().replace(chr(10), '  ')}")
    if ".db3" not in listing:
        sys.exit(f"录制没产生 .db3,检查车上 {remote}")

    # ---- 传 ----
    print("[3/5] 传输到本机 + 校验")
    db3 = next(p for p in listing.split() if p.endswith(".db3"))
    car_binary(f"{remote}/{db3}", out / db3)
    car_binary(f"{remote}/metadata.yaml", out / "metadata.yaml")

    remote_md5 = car(f"md5sum {remote}/{db3} | cut -d' ' -f1").strip()
    local_md5 = md5_of(out / db3)
    if remote_md5 != local_md5:
        sys.exit(f"MD5 不一致!车上 {remote_md5} / 本机 {local_md5}")
    print(f"      MD5 一致 {remote_md5[:16]}...  大小 "
          f"{(out / db3).stat().st_size / 1e6:.1f} MB")

    # ---- 清理车上 ----
    car(f"rm -rf {remote}")
    print("      车上临时文件已删除")

    # ---- 解析 ----
    print("[4/5] 解析")
    r = sh([sys.executable, str(HERE / "parse_bag.py"), str(out / db3),
            "--npz"])
    if r.returncode != 0:
        sys.exit(f"解析失败:\n{r.stderr}")
    # parse_bag 固定写 scan.npz,挪进输出目录
    npz = HERE / "scan.npz"
    target = out / "scan.npz"
    if npz.is_file():
        if target.exists():
            target.unlink()
        npz.replace(target)
    for line in r.stdout.splitlines():
        if "RuntimeWarning" not in line and "nanvar" not in line:
            print("      " + line)

    # ---- 出图 ----
    if a.no_plot:
        print("[5/5] 跳过出图(--no-plot)")
    else:
        print("[5/5] 出图")
        r = sh([sys.executable, str(HERE / "view_scan.py"), str(target),
                "--out", str(out / "overview.png")])
        print("      " + r.stdout.strip() if r.stdout.strip() else r.stderr.strip())

    print(f"\n完成 → {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
