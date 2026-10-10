"""地面分割节点(bev_v2 的 FREE 层):摄像头 -> 昇腾 NPU 上的 SegFormer-B0(ADE20K,地面判断已并进模型)
-> 查表投到地面 -> 发车体系地面点。只读摄像头,不发控制。

    source /usr/local/Ascend/ascend-toolkit/set_env.sh
    PYTHONPATH=~/ground_bench/pylib:$PYTHONPATH python3 -m vehicle.ground_seg_node \\
        --model ~/ground_bench/segformer_b0_ade_ground_512x512.om

发 /ground_free(sensor_msgs/PointCloud2):字段 x、y(float32,车体系 = 后轴原点,x 前 y 左,地面 z=0),
header.stamp = 取到这帧的时刻(road_car_node 按这个时刻的位姿登记)。点已按 5 cm 栅格去重。
模型由 SegFormer 导出(地面类 = ADE20K floor/road/grass/sidewalk/earth/rug/... 共 12 类),输入 1×3×H×W
RGB 归一化,输出 1×H/4×W/4 uint8。NPU 实测 512×512 每帧 54 ms、256×320 22 ms(2026-10-10)。
投影只用 CarParams 的相机内参、畸变、外参(common/ground_proj.py),只取相机前 3 m 内。
"""
from __future__ import annotations

import argparse
import threading
import time

import numpy as np

from common.ground_proj import GroundLut
from common.params import CarParams

TOPIC = '/ground_free'
DEBUG_TOPIC = '/ground_debug'     # 2 Hz:原图叠地面蒙版(JPEG,sensor_msgs/CompressedImage),给影子模式网页看
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


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
    ap.add_argument('--max-range', type=float, default=3.0, help='只取相机前这么远 m')
    a = ap.parse_args(argv)
    size = tuple(a.size) if a.size else tuple(int(v) for v in a.model.rsplit('_', 1)[1].split('.')[0].split('x'))

    import cv2
    import rclpy
    from ais_bench.infer.interface import InferSession
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage, PointCloud2, PointField

    rclpy.init(args=[])
    node = Node('ground_seg_node')
    pub = node.create_publisher(PointCloud2, TOPIC, 2)
    dbg = node.create_publisher(CompressedImage, DEBUG_TOPIC, 1)
    log = node.get_logger()
    p = CarParams()
    lut = GroundLut(size[0] // 4, size[1] // 4, p, a.max_range)
    sess = InferSession(0, a.model)
    cam = LatestFrame(cv2, lambda: node.get_clock().now().nanoseconds, size=p.camera_size)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    log.info(f'{a.model} 输入 {size[0]}x{size[1]},地面格 {int(lut.valid.sum())},发 {TOPIC} 最高 {a.rate:.0f} Hz')

    seq, period, n, t_sum, t0 = 0, 1.0 / a.rate, 0, 0.0, time.monotonic()
    try:
        while rclpy.ok():
            start = time.monotonic()
            seq, frame, stamp = cam.get(seq)
            if frame is None:
                log.warn('摄像头没有画面', throttle_duration_sec=2.0)
                continue
            mask = sess.infer([preprocess(cv2, frame, size)])[0][0]
            pub.publish(to_cloud(PointCloud2, PointField, lut.points(mask), stamp))
            if seq % 5 == 0 and dbg.get_subscription_count():
                g = cv2.resize(mask.astype(np.uint8), (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
                ov = frame.copy()
                ov[g] = (0.5 * ov[g] + 0.5 * np.array([40, 160, 60])).astype(np.uint8)
                im = CompressedImage(format='jpeg', data=cv2.imencode('.jpg', ov, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes())
                im.header.stamp.sec, im.header.stamp.nanosec = divmod(int(stamp), 1_000_000_000)
                dbg.publish(im)
            n, t_sum = n + 1, t_sum + time.monotonic() - start
            if time.monotonic() - t0 > 10.0:
                log.info(f'{n / (time.monotonic() - t0):.1f} Hz,每帧 {1000 * t_sum / n:.0f} ms')
                n, t_sum, t0 = 0, 0.0, time.monotonic()
            time.sleep(max(0.0, period - (time.monotonic() - start)))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
