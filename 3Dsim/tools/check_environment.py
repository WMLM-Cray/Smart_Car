"""Read-only readiness probe; run inside the intended Ubuntu shell."""
import json, os, platform, shutil, subprocess
from pathlib import Path

def main():
    release = Path('/etc/os-release')
    os_release = release.read_text() if release.exists() else ''
    result = {'system': platform.system(), 'release': os_release,
              'ROS_DISTRO': os.environ.get('ROS_DISTRO'), 'packages': {}}
    required = ['gazebo_ros', 'gazebo_plugins', 'nav2_bringup', 'slam_toolbox',
                'robot_localization', 'xacro', 'robot_state_publisher']
    for command in ['ros2', 'gazebo', 'colcon']:
        result[command] = shutil.which(command)
    if result['ros2']:
        for package in required:
            p = subprocess.run(['ros2', 'pkg', 'prefix', package], capture_output=True,
                               text=True, timeout=15)
            result['packages'][package] = p.returncode == 0
    if result['gazebo']:
        p = subprocess.run(['gazebo', '--version'], capture_output=True, text=True, timeout=15)
        result['gazebo_version'] = p.stdout.strip()
    result['ready'] = (platform.system() == 'Linux' and 'ID=ubuntu' in os_release
        and 'VERSION_ID="22.04"' in os_release and result['ROS_DISTRO'] == 'humble'
        and bool(result['colcon']) and all(result['packages'].get(k, False) for k in required)
        and 'version 11.' in result.get('gazebo_version', ''))
    print(json.dumps(result, ensure_ascii=True, indent=2))
    return 0 if result['ready'] else 1
if __name__ == '__main__':
    raise SystemExit(main())
