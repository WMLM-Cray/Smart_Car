"""地面分割节点(bev_v2 的 FREE 层):摄像头 -> 昇腾 NPU 上的 SegFormer-B0(ADE20K,地面判断已并进模型)
-> 查表投到地面 -> 发车体系地面点。只读摄像头,不发控制。

    source /usr/local/Ascend/ascend-toolkit/set_env.sh
    PYTHONPATH=~/ground_bench/pylib:$PYTHONPATH python3 -m vehicle.ground_seg_node \\
        --model ~/ground_bench/segformer_b0_ade_ground_512x512.om

发 /ground_free(sensor_msgs/PointCloud2):字段 x、y(float32,车体系 = 后轴原点,x 前 y 左,地面 z=0),
header.stamp = 取到这帧的时刻(road_car_node 按这个时刻的位姿登记)。点已按 5 cm 栅格去重。
模型由 SegFormer 导出(地面类 = ADE20K floor/road/grass/sidewalk/earth/rug/... 共 12 类),输入 1×3×H×W
RGB 归一化,输出 1×H/4×W/4 uint8。NPU 实测 512×512 每帧 54 ms、256×320 22 ms(2026-10-10)。
投影只用 CarParams 的相机内参、畸变、外参;每帧用 /scan 在线修俯仰/横滚、按方向和雷达墙对齐、去孤岛、
雷达截断(common/ground_fix.py):姿态被雷达确认时取相机前 3 m 内,否则 2 m 内(和雷达墙对齐的方向到墙为止)。
"""
from __future__ import annotations

import argparse
import collections
import multiprocessing as mp
import queue
import threading
import time

import numpy as np

from common.ground_fix import GroundFixer
from common.params import CarParams

TOPIC = '/ground_free'
DEBUG_TOPIC = '/ground_debug'     # 每帧(~3 Hz,有人订才发):原图叠地面蒙版 + 姿态状态(JPEG),影子网页和开车回看用
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)
SCAN_DT = 0.05                    # 雷达和相机帧时间差超过这么多 s 就不拿来修俯仰 / 截断


def preprocess(cv2, frame, size):
    h, w = size
    x = cv2.cvtColor(cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
    x = (x.astype(np.float32) / 255 - MEAN) / STD
    return np.ascontiguousarray(x.transpose(2, 0, 1)[None])


class LatestFrame:
    """后台一直读摄像头,只留最新一帧和取到它的时刻 —— 不让 OpenCV 缓冲里的旧帧被拿去分割。"""

    def __init__(self, cv2, clock, dev=0, size=(640, 480)):
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.clock, self.frame, self.stamp, self.seq = clock, None, 0.0, 0
        self.cond = threading.Condition()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            ok, f = self.cap.read()
            t = self.clock()
            if not ok:
                time.sleep(0.01)
                continue
            with self.cond:
                self.frame, self.stamp, self.seq = f, t, self.seq + 1
                self.cond.notify_all()

    def get(self, after):
        with self.cond:
            self.cond.wait_for(lambda: self.seq > after, timeout=1.0)
            return self.seq, self.frame, self.stamp


def npu_worker(model, conn):
    """子进程里跑 NPU 推理:ais_bench 推理期间不放 Python 全局锁,放在主进程里会卡住后处理线程和 ROS 回调。"""
    from ais_bench.infer.interface import InferSession
    sess = InferSession(0, model)
    conn.send('ready')
    while (x := conn.recv()) is not None:
        conn.send(sess.infer([x])[0][0])


def to_cloud(PointCloud2, PointField, xy, stamp_ns, frame_id='base_footprint'):
    m = PointCloud2()
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(int(stamp_ns), 1_000_000_000)
    m.header.frame_id = frame_id
    m.height, m.width = 1, len(xy)
    m.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1)]
    m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 8, 8 * len(xy), True
    m.data = np.ascontiguousarray(xy, np.float32).tobytes()
    return m


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--model', required=True, help='segformer_b0_ade_ground_HxW.om')
    ap.add_argument('--size', type=int, nargs=2, default=None, metavar=('H', 'W'),
                    help='模型输入尺寸;默认从文件名 *_HxW.om 读')
    ap.add_argument('--rate', type=float, default=10.0, help='最高帧率 Hz(训练里地面分割 10 Hz)')
    ap.add_argument('--fill', action='store_true',
                    help='补远处像素行之间的缝(远处一行投到地面 ~0.5 m,不补是一道道条纹);v4 第 4 段起的模型按补过的训')
    a = ap.parse_args(argv)
    size = tuple(a.size) if a.size else tuple(int(v) for v in a.model.rsplit('_', 1)[1].split('.')[0].split('x'))

    import cv2
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage, LaserScan, PointCloud2, PointField

    npu, child = mp.get_context('spawn').Pipe()
    worker = mp.get_context('spawn').Process(target=npu_worker, args=(a.model, child), daemon=True)
    worker.start()
    assert npu.recv() == 'ready'
    rclpy.init(args=[])
    node = Node('ground_seg_node')
    pub = node.create_publisher(PointCloud2, TOPIC, 2)
    dbg = node.create_publisher(CompressedImage, DEBUG_TOPIC, 1)
    log = node.get_logger()
    p = CarParams()
    fixer = GroundFixer(size[0] // 4, size[1] // 4, p, fill=a.fill)
    scans = collections.deque(maxlen=12)             # 最近 ~1 s(12 Hz):后处理时这帧已拍下 ~0.35 s,(时刻 ns, (ranges, angle_min, angle_increment))

    def on_scan(m):
        scans.append((m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec,
                      (np.asarray(m.ranges, np.float32), m.angle_min, m.angle_increment)))
    node.create_subscription(LaserScan, '/scan', on_scan, 2)
    cam = LatestFrame(cv2, lambda: node.get_clock().now().nanoseconds, size=p.camera_size)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    log.info(f'{a.model} 输入 {size[0]}x{size[1]},发 {TOPIC} 最高 {a.rate:.0f} Hz,订 /scan 修俯仰')

    # 推理(子进程,NPU)和后处理(修俯仰、发布、调试图,CPU)流水线并行:NPU 算这一帧时 CPU 处理上一帧
    jobs = queue.Queue(maxsize=1)
    stats = {'n': 0, 'pre': 0.0, 'npu': 0.0, 'post': 0.0, 't0': time.monotonic()}

    def post_loop():
        while True:
            seq, frame, stamp, mask = jobs.get()
            t = time.monotonic()
            near = min(list(scans), key=lambda s: abs(s[0] - stamp), default=None)
            scan = near[1] if near is not None and abs(near[0] - stamp) <= SCAN_DT * 1e9 else None
            xy, info = fixer.process(mask > 0, stamp * 1e-9, scan)
            pub.publish(to_cloud(PointCloud2, PointField, xy, stamp))
            if dbg.get_subscription_count():                # 有人订才编码(影子网页、road_car_node 逐帧回看)
                g = cv2.resize(mask.astype(np.uint8), (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
                ov = frame.copy()
                ov[g] = (0.5 * ov[g] + 0.5 * np.array([40, 160, 60])).astype(np.uint8)
                cv2.putText(ov, f'pitch {np.degrees(info.pitch):+.2f} roll {np.degrees(info.roll):+.2f} {info.source} {info.why} '
                            f'walls {info.walls} sec {info.aligned} -{info.cut} +{info.filled}',
                            (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                im = CompressedImage(format='jpeg', data=cv2.imencode('.jpg', ov, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes())
                im.header.stamp.sec, im.header.stamp.nanosec = divmod(int(stamp), 1_000_000_000)
                dbg.publish(im)
            stats['n'] += 1
            stats['post'] += time.monotonic() - t
            dt = time.monotonic() - stats['t0']
            if dt > 10.0:
                n = stats['n']
                log.info(f'{n / dt:.1f} Hz;每帧 预处理 {1000 * stats["pre"] / n:.0f} + NPU {1000 * stats["npu"] / n:.0f} ms,'
                         f'后处理 {1000 * stats["post"] / n:.0f} ms(并行);'
                         f'俯仰 {np.degrees(info.pitch):+.2f}° 横滚 {np.degrees(info.roll):+.2f}° {info.source} {info.why} '
                         f'墙点 {info.walls} 对齐扇区 {info.aligned} 删 {info.cut} 补 {info.filled}')
                stats.update(n=0, pre=0.0, npu=0.0, post=0.0, t0=time.monotonic())
    threading.Thread(target=post_loop, daemon=True).start()

    seq, period = 0, 1.0 / a.rate
    try:
        while rclpy.ok():
            start = time.monotonic()
            seq, frame, stamp = cam.get(seq)
            if frame is None:
                log.warn('摄像头没有画面', throttle_duration_sec=2.0)
                continue
            x = preprocess(cv2, frame, size)
            t = time.monotonic()
            npu.send(x)
            mask = npu.recv()
            stats['pre'] += t - start
            stats['npu'] += time.monotonic() - t
            jobs.put((seq, frame, stamp, mask))          # 后处理还没做完上一帧就在这儿等
            time.sleep(max(0.0, period - (time.monotonic() - start)))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
