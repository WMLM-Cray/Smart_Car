import os
from glob import glob
from setuptools import setup

package_name = 'racecar_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    entry_points={
        'console_scripts': [
            'mission_executor = racecar_mission.executor_node:main',
            'mock_nav2 = racecar_mission.mock_nav2:main',
            'run_offline_regression = racecar_mission.offline_regression:main',
            'gen_goals = racecar_mission.gen_goals:main',
            'signal_exit_test = racecar_mission.signal_exit_test:main',
            'use_sim_time_check = racecar_mission.use_sim_time_check:main',
        ],
    },
    install_requires=['setuptools'],
    zip_safe=True,
)
