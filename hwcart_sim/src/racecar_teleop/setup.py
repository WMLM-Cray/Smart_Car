import os

from setuptools import find_packages, setup

package_name = 'racecar_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), ['config/generic.yaml']),
        (os.path.join('share', package_name, 'launch'), ['launch/teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Teammate A',
    maintainer_email='teammateA@example.com',
    description='Joystick teleop publishing standard geometry_msgs/Twist for Ackermann vehicles',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_node = racecar_teleop.teleop_node:main',
        ],
    },
)
