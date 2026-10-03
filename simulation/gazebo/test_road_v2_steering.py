"""Cross-platform contract checks for road_v2 -> Gazebo Ackermann steering.

These tests exercise the shared inference/runtime math and inspect the Gazebo
configuration; they do not substitute for a ROS/Gazebo dynamics run.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from common.control import to_cmd_vel
from common.params import CarParams
from common.pwm import action_to_steer
from common.road_runtime import RoadRuntime


ROOT = Path(__file__).resolve().parents[2]
GAZEBO_SRC = ROOT / 'simulation' / 'gazebo' / 'src'


class FixedPolicy:
    def __init__(self, steer_action):
        self.action = np.asarray([steer_action, 0.5], dtype=np.float32)

    def reset(self):
        pass

    def __call__(self, observation):
        return self.action


@pytest.mark.parametrize('steer_action', [-1.0, -0.5, 0.5, 1.0])
def test_road_runtime_twist_encodes_the_measured_curve(steer_action):
    """The model action must pass through RoadRuntime and the measured curve;
    angular.z must be body yaw rate, not a PWM or a linear action angle."""
    p = CarParams()
    now = 1.0
    runtime = RoadRuntime(FixedPolicy(steer_action), params=p)
    runtime.on_scan(np.full(1667, 2.0), -math.pi, 2 * math.pi / 1666, now)
    runtime.on_odometry(0.0, now)
    runtime.on_imu(0.0, now)
    runtime.set_enabled(True, now)

    linear_x, angular_z = runtime.step(now)
    expected_steer = action_to_steer(steer_action, p)
    assert linear_x > 0.0
    assert angular_z == pytest.approx(
        to_cmd_vel(linear_x, expected_steer, p)[1], rel=1e-6, abs=1e-8)
    assert math.atan(p.wheelbase * angular_z / linear_x) == pytest.approx(
        expected_steer, abs=1e-6)
    # Endpoints clamp to the measured maximum, so compare only interior actions
    # to prove this is not a linear action->max_steer mapping.
    if -1.0 < steer_action < 1.0:
        assert abs(expected_steer - steer_action * p.max_steer) > 0.01


def test_controller_geometry_and_urdf_limits_support_the_measured_curve():
    p = CarParams()
    config = yaml.safe_load((GAZEBO_SRC / 'racecar_sim/config/params.yaml').read_text(
        encoding='utf-8'))['racecar_sim']
    controller = (GAZEBO_SRC / 'racecar_sim/config/ros2_control.yaml.in').read_text(
        encoding='utf-8')
    assert config['wheel_base'] == pytest.approx(p.wheelbase)
    assert '__WHEELBASE__' in controller
    rendered_controller = (controller
                           .replace('__WHEELBASE__', str(config['wheel_base']))
                           .replace('__WHEEL_TRACK__', str(config['wheel_track']))
                           .replace('__WHEEL_RADIUS__', str(config['wheel_radius'])))
    controller_params = yaml.safe_load(rendered_controller)[
        'ackermann_steering_controller']['ros__parameters']
    assert controller_params['wheelbase'] == pytest.approx(p.wheelbase)
    assert controller_params['front_wheel_track'] == pytest.approx(config['wheel_track'])
    assert controller_params['use_stamped_vel'] is False

    # Ackermann's inner front wheel needs a larger angle than the equivalent
    # bicycle angle. Verify both curve endpoints fit the xacro joint limit.
    track = float(config['wheel_track'])
    joint_limit = float(config['max_steer_angle'])
    curve_angles = [abs(action_to_steer(a, p)) for a in (-1.0, 1.0)]
    required_wheel_angles = []
    for delta in curve_angles:
        radius = p.wheelbase / math.tan(delta)
        assert radius > track / 2
        required_wheel_angles.append(math.atan(p.wheelbase / (radius - track / 2)))
        required_wheel_angles.append(math.atan(p.wheelbase / (radius + track / 2)))
    assert max(required_wheel_angles) < joint_limit

    xacro_root = ET.parse(GAZEBO_SRC / 'racecar_sim/urdf/racecar.xacro').getroot()
    joints = {j.attrib['name']: j for j in xacro_root.findall('joint')}
    for name in ('left_steering_joint', 'right_steering_joint'):
        limit = joints[name].find('limit')
        assert limit is not None
        assert limit.attrib['lower'] == "${-cfg['max_steer_angle']}"
        assert limit.attrib['upper'] == "${cfg['max_steer_angle']}"


def test_gazebo_wiring_forwards_body_twist_without_a_second_conversion():
    wiring = (GAZEBO_SRC / 'racecar_sim/racecar_sim/sim_wiring.py').read_text(
        encoding='utf-8')
    assert "'/cmd_vel'" in wiring
    assert "'/ackermann_steering_controller/reference_unstamped'" in wiring
    assert 'self.vel_pub.publish(msg)' in wiring
    assert 'angular.z' not in wiring
    road_script = (ROOT / 'simulation/gazebo/run_road_policy_once.sh').read_text(
        encoding='utf-8')
    assert 'vehicle/road_policy_node.py' in road_script
    assert 'python3 "$REPO/vehicle/road_policy_node.py" "$MODEL"' in road_script


def test_generated_official_a_to_d_tracks_are_scoreable(tmp_path, monkeypatch):
    from simulation.gazebo import official_to_gazebo

    monkeypatch.setattr('sys.argv', [
        'official_to_gazebo.py', '--out', str(tmp_path), '--tracks', 'A', 'B', 'C', 'D'])
    official_to_gazebo.main()
    for name in 'ABCD':
        track_dir = tmp_path / f'track_{name}'
        layout = yaml.safe_load((track_dir / 'layout.yaml').read_text(encoding='utf-8'))
        world = ET.parse(track_dir / 'track_sim.world').getroot()
        assert layout['track'].startswith(f'official {name}')
        assert len(layout['cones']) >= 3
        expected = official_to_gazebo.convert(name)['cones']
        np.testing.assert_allclose(
            [(cone['x'], cone['y']) for cone in layout['cones']], expected, atol=5e-5)
        assert len(list(world.iter('model'))) == len(layout['cones']) + 1
        assert any(p.attrib.get('name') == 'gazebo_ros_state'
                   for p in world.iter('plugin'))
