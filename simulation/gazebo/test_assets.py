"""跨平台检查恢复工程的资源闭包；不冒充 Gazebo 运行测试。"""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from common.params import CarParams

ROOT = Path(__file__).parent / 'src'

def test_geometry_and_lidar_contract():
    p = yaml.safe_load((ROOT/'racecar_sim/config/params.yaml').read_text(encoding='utf-8'))['racecar_sim']
    car = CarParams()
    assert p['wheel_base'] == car.wheelbase
    assert p['lidar_samples'] == car.lidar_points
    assert p['lidar_hz'] == car.lidar_hz
    template = (ROOT/'racecar_sim/config/ros2_control.yaml.in').read_text(encoding='utf-8')
    for key, token in [('wheel_base','__WHEELBASE__'),('wheel_track','__WHEEL_TRACK__'),('wheel_radius','__WHEEL_RADIUS__')]:
        template = template.replace(token, str(p[key]))
    assert '__WHEEL' not in template
    assert isinstance(yaml.safe_load(template), dict)

def test_world_and_model_resources_resolve():
    models = ROOT/'racecar_test_assets/models'
    worlds = ROOT/'racecar_sim/worlds'
    for path in [*worlds.glob('*.world'), *models.rglob('*.sdf')]:
        tree = ET.parse(path)
        for item in tree.iter('uri'):
            uri = item.text.strip()
            if uri in ('model://sun', 'model://ground_plane'):
                continue  # supplied by Gazebo Classic
            assert uri.startswith('model://'), uri
            target = models/uri.removeprefix('model://')
            assert target.exists(), (path, uri)
            if target.is_dir():
                assert (target/'model.config').is_file()
                assert (target/'model.sdf').is_file()

def test_ros_package_files_are_complete():
    for name in ['racecar_sim','racecar_test_assets']:
        assert ET.parse(ROOT/name/'package.xml').getroot().findtext('name') == name
    assert (ROOT/'racecar_sim/resource/racecar_sim').is_file()
    assert (ROOT/'racecar_test_assets/env-hooks/racecar_test_assets.dsv').is_file()
    ET.parse(ROOT/'racecar_sim/urdf/racecar.xacro')
