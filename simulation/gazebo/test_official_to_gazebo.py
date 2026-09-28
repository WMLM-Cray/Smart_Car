"""official_to_gazebo(maps/ 原坐标)和 sim_reset 参数检查的测试,不需要 ROS。"""
import math

import numpy as np
import pytest
import yaml

from evaluation.road_track import official
from simulation.gazebo import official_to_gazebo, sim_reset


@pytest.mark.parametrize('name', ['A', 'C'])
def test_keeps_maps_coordinates_and_real_start(name):
    road, _ = official(name, fill_v=True)
    c = official_to_gazebo.convert(name)
    np.testing.assert_allclose(c['cones'], road.cones)          # 不旋转不平移
    x, y, yaw = c['start']
    assert (x, y) == pytest.approx(tuple(road.centre[0]))
    assert yaw == pytest.approx(float(road.heading[0]))


def test_layout_start_pose_round_trips_to_spawn(tmp_path, monkeypatch):
    monkeypatch.setattr('sys.argv', ['x', '--out', str(tmp_path), '--tracks', 'A'])
    official_to_gazebo.main()
    lay = yaml.safe_load((tmp_path / 'track_A' / 'layout.yaml').read_text(encoding='utf-8'))
    road, _ = official('A', fill_v=True)
    assert lay['start_pose']['xy'] == pytest.approx(list(road.centre[0]), abs=1e-3)
    x, y, yaw = sim_reset.start_from_layout(tmp_path / 'track_A' / 'layout.yaml')
    assert (x, y) == pytest.approx(tuple(road.centre[0]), abs=1e-3)
    assert yaw == pytest.approx(float(road.heading[0]), abs=1e-4)


def test_sim_reset_needs_a_pose_and_a_sim_domain(monkeypatch):
    with pytest.raises(SystemExit):
        sim_reset.parse([])                                       # 没给 layout 也没给 x/y
    for dom in (None, '42'):
        if dom is None:
            monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
        else:
            monkeypatch.setenv('ROS_DOMAIN_ID', dom)
        with pytest.raises(SystemExit, match='仿真域'):
            sim_reset.main(['--x', '1', '--y', '2'])
