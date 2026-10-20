"""road_lap2(方案C第二圈仿真)的单元测试:滑停物理、速度规划曲线、停车状态机边界。"""
import math

import numpy as np
import pytest

from dataclasses import replace
from evaluation.road_lap2 import HOLD_T, PARK_BUDGET, STOP_OFFSET, v_profile
from evaluation.road_track import official
from simulation.fastsim.road_env import PHYSICS_DT, Plant, SpeedPlant


def _drive_to(sp: SpeedPlant, v_cmd: float, t: float) -> None:
    n = int(t / PHYSICS_DT)
    for _ in range(n):
        sp.step(v_cmd)


class TestSpeedPlantCoast:
    """cmd<=0 = 松油滑停(实测 0.68~0.78 m/s²);cmd>0 行为不变。"""

    def test_coast_to_zero(self):
        pl = Plant(decel=0.73, launch_delay=0.0, dead_time=0.0, v_plateau=0.5, v_top=1.0)
        sp = SpeedPlant(pl)
        _drive_to(sp, 0.6, 3.0)
        v0 = sp.v
        assert 0.5 < v0 <= 0.6
        dist = 0.0
        for _ in range(int(5.0 / PHYSICS_DT)):
            v = sp.step(0.0)
            if v <= 0.0:
                break
            dist += v * PHYSICS_DT
        assert sp.v == 0.0                                   # 减到停,不弹回平台
        assert dist == pytest.approx(v0 ** 2 / (2 * 0.73), rel=0.15)  # 滑距 ≈ v²/2a

    def test_coast_stays_zero(self):
        pl = Plant(decel=0.73, launch_delay=0.0, dead_time=0.0)
        sp = SpeedPlant(pl)
        _drive_to(sp, 0.0, 1.0)
        assert sp.v == 0.0
        for _ in range(200):
            assert sp.step(0.0) == 0.0                       # 停住后保持静止

    def test_positive_cmd_unchanged(self):
        """正指令仍被钳到 [v_plateau, v_top],破静点行为不变(兼容既有评测/训练)。"""
        pl = Plant(launch_delay=0.0, dead_time=0.0, v_plateau=0.55, v_top=1.0)
        sp = SpeedPlant(pl)
        _drive_to(sp, 0.2, 2.0)
        assert sp.v == pytest.approx(0.55, abs=0.02)         # 低指令 -> 停在平台
        _drive_to(sp, 1.5, 5.0)
        assert sp.v == pytest.approx(1.0, abs=0.02)          # 高指令 -> 顶到 v_top


@pytest.fixture(scope='module')
def road_lap():
    road, lap_len = official('A', True, laps=3)
    return road, lap_len


class TestVProfile:
    """先验速度规划:曲率限速 + 前/后向传播 + 停车曲线。"""

    def test_profile_ends_at_zero(self, road_lap):
        road, lap_len = road_lap
        s_target = 2.0 * lap_len + STOP_OFFSET
        s, v = v_profile(road, lap_len, s_target, 1.0, 1.0, 1.0, 0.5, 0.73)
        assert s[-1] >= s_target - 0.1
        assert v[-1] < 0.05                                  # 停车曲线把末端压到 ~0
        assert np.all(np.diff(v) < 1.0)                      # 无跳变(传播约束住了)

    def test_profile_curvature_limit(self, road_lap):
        road, lap_len = road_lap
        s_target = 2.0 * lap_len + STOP_OFFSET
        s1, v1 = v_profile(road, lap_len, s_target, 0.5, 1.0, 1.0, 0.5, 0.73)
        s2, v2 = v_profile(road, lap_len, s_target, 2.0, 1.0, 1.0, 0.5, 0.73)
        assert np.all(v1 <= v2 + 1e-9)                       # 摩擦假设越保守,速度越低
        assert v1.max() <= 1.0 and v2.max() <= 1.0

    def test_profile_none_uses_cap(self, road_lap):
        road, lap_len = road_lap
        s, v = v_profile(road, lap_len, 2.0 * lap_len + STOP_OFFSET, None, 0.8, 1.0, 0.5, 0.73)
        assert v.max() <= 0.8 + 1e-9                         # 不超 cap
        assert v.max() == pytest.approx(0.8, abs=0.05)       # 直道段到达 cap
        mid = v[(s > lap_len + 2) & (s < 2 * lap_len - 1.5)]  # 中段不受停车曲线影响
        _, v05 = v_profile(road, lap_len, 2.0 * lap_len + STOP_OFFSET, 0.5, 0.8, 1.0, 0.5, 0.73)
        assert np.all(mid + 1e-9 >= v05[:len(mid)])          # None 档处处不低于保守摩擦档


class TestStopGeometry:
    """停车目标几何:START 框中心 = 第 2 圈过线点,STOP_OFFSET 之后的滑距可停进框内。"""

    def test_stop_offset_inside_box(self):
        # 框半长 0.57(START x_range ~1.14m)。松油点 ≈ s_target - v_lo²/(2·0.73) ≈ s_target-0.14,
        # 松油车速被压顶指令钳在 v_lo≈0.45(最坏 0.6,含 dead_time 迟到一拍),本局 decel 最低 0.68。
        coast = 0.6 ** 2 / (2 * 0.68) + 0.35 * 0.6
        stop_worst = STOP_OFFSET - 0.14 + coast               # 相对框中心的最坏停车点
        assert stop_worst < 0.50                              # 最坏也停在框内且留余量
        assert STOP_OFFSET + 0.14 > 0.0                       # 最好(滑距短)也过了框中心
