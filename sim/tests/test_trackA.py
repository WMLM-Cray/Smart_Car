"""集成新增测试:冻结 A 赛道适配器 + 传感器刷新周期接线(2026-09-16 任务书 2/3 项)。

参考线折线长度 36.073m(2026-09-16 逐段实测定标;连续几何 36.089,0.05m 离散+
去重后直连再 −0.015,推导详见 test_trackA_centerline_length_matches_geometry)。
注意:阶段3 测试驾驶器记录的"整圈 35.46m"是车辆**实际行驶轨迹**里程,与参考线
折线长是两个量,行驶轨迹不与参考线逐点重合,两者相差约 0.6m 属正常,不在此互相对账。
"""
import math

import numpy as np
import pytest

from sim.env import ConeTrackEnv
from sim.trackA import DEFAULT_CSV, load_trackA


def test_trackA_loads_frozen_baseline():
    t = load_trackA()
    assert len(t.left) == 26 and len(t.right) == 37      # 红=左(内),蓝=右(外)
    assert t.spawn == (11.0, 0.75, 0.0)
    assert t.name == "trackA"
    cones = np.vstack([t.left, t.right])
    assert np.isfinite(cones).all()
    # 参考线:起点=出生点、首段朝 +x、全程在赛道包络内
    assert np.allclose(t.centerline[0], (11.0, 0.75))
    d = t.centerline[5] - t.centerline[0]
    assert d[0] > 0 and abs(d[1]) < 0.05                 # 沿底道向 +x
    assert t.centerline[:, 0].max() < 16.8 and t.centerline[:, 1].max() < 5.5


def test_trackA_not_randomized_on_reload_and_reset():
    """两次加载与两次 reset(不同 seed)必须逐位一致——冻结赛道不做任何随机化。"""
    t1, t2 = load_trackA(), load_trackA()
    assert np.array_equal(t1.centerline, t2.centerline)
    assert np.array_equal(t1.left, t2.left) and np.array_equal(t1.right, t2.right)
    env = ConeTrackEnv(track=t1)
    poses = []
    for seed in (1, 2):
        env.reset(seed=seed)
        poses.append((env.car.x, env.car.y, env.car.yaw))
    assert poses[0] == poses[1] == (11.0, 0.75, 0.0)
    assert np.array_equal(env.track.left, t1.left)


def test_trackA_centerline_length_matches_geometry():
    """长度对账(两层推导,机器值 36.073 为准):
    连续几何长 36.089 = 底道3.03+右U弧6.283+顶右直5.62+V谷折线6.193+切角1.0+
    切角弧1.571+竖直1.0+左下弧3.142+底道8.25;0.05m 离散采样且各段 [1:-1]
    去重复合点后,节点处跨点直连再 −0.015 → 36.073(2026-09-16 逐段实测定标)。
    """
    t = load_trackA()
    length = float(np.hypot(*np.diff(np.vstack([t.centerline, t.centerline[:1]]), axis=0).T).sum())
    assert abs(length - 36.073) < 0.02


def test_trackA_centerline_clears_cones():
    """参考线(抽稀 0.2m)到全部 63 锥的距离下限——车道中线不应贴锥。"""
    t = load_trackA()
    cones = np.vstack([t.left, t.right])
    pts = t.centerline[::4]                              # 0.05m 采样 → ~0.2m
    d = np.linalg.norm(pts[:, None, :] - cones[None, :, :], axis=2)
    assert d.min() >= 0.40, f"参考线贴锥:最小距离 {d.min():.3f}m"


def test_trackA_schema_rejections(tmp_path):
    """坏格式必须明确报错:颜色词、id 前缀、非数字坐标、缺文件,都不许静默。"""
    good = "id,color,x,y,segment\nb01,blue,2.75,0.0,E1\nr01,red,2.75,1.5,I1\n"
    bad_colour = good.replace("blue", "cyan")
    p = tmp_path / "bad_colour.csv"; p.write_text(bad_colour)
    with pytest.raises(ValueError, match="blue/red"):
        load_trackA(p)
    bad_id = good.replace("b01", "c01")
    p = tmp_path / "bad_id.csv"; p.write_text(bad_id)
    with pytest.raises(ValueError, match=r"\^\[br\]"):
        load_trackA(p)
    bad_num = good.replace("2.75,0.0", "2.75,零")
    p = tmp_path / "bad_num.csv"; p.write_text(bad_num)
    with pytest.raises(ValueError, match="不是数字"):
        load_trackA(p)
    with pytest.raises(FileNotFoundError):
        load_trackA(tmp_path / "missing.csv")


def test_sensor_refresh_periods_wired():
    """三路传感器按 params 频率独立刷新(2026-09-16 接线,此前 IMU/encoder 跟雷达帧)。

    1.0s 仿真时间:雷达 12Hz / IMU 100Hz / encoder 50Hz,各自计数对账。
    """
    env = ConeTrackEnv()
    env.reset(seed=0)
    for _ in range(20):                                  # 20 × control_dt(0.05) = 1.0s
        env.step([0.0, 0.0])
    assert abs(env.n_scan - 12) <= 1, env.n_scan
    assert abs(env.n_imu - 100) <= 2, env.n_imu
    assert abs(env.n_encoder - 50) <= 1, env.n_encoder
    # 默认 CSV 在仓库内,适配器开箱即用
    assert DEFAULT_CSV.exists()


def test_trackA_env_matches_gazebo_baseline_params():
    """对齐口径落地检查:轴距/量程与主线 params.yaml 一致(0.36/25.0)。"""
    from sim.params import CarParams
    cp = CarParams()
    assert cp.wheelbase == 0.36
    assert cp.lidar_max == 25.0
    assert cp.lidar_hz == 12.0 and cp.lidar_points == 1667 and cp.imu_hz == 100.0
