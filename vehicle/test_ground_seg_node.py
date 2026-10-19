"""无新相机帧时不能把缓存当成新分割输入;不依赖相机、ROS 或 NPU。"""
import threading

from vehicle.ground_seg_node import LatestFrame


def test_camera_timeout_does_not_return_cached_frame():
    cam = LatestFrame.__new__(LatestFrame)
    cam.cond = threading.Condition()
    cam.seq, cam.frame, cam.stamp = 1, object(), 100.0
    assert cam.get(after=1, timeout=0) == (1, None, 0.0)
    assert cam.get(after=0, timeout=0) == (1, cam.frame, 100.0)
    cam.seq, cam.stamp = 2, 101.0
    assert cam.get(after=1, timeout=0) == (2, cam.frame, 101.0)
