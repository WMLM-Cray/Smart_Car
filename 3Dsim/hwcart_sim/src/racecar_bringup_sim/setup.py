from setuptools import setup
import os
from glob import glob

setup(name='racecar_bringup_sim', version='0.1.0', packages=['racecar_bringup_sim'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/racecar_bringup_sim']),
                  ('share/racecar_bringup_sim', ['package.xml']),
                  (os.path.join('share/racecar_bringup_sim', 'launch'),
                   glob('launch/*.launch.py')),
                  (os.path.join('share/racecar_bringup_sim', 'rviz'),
                   glob('rviz/*.rviz')),
                  (os.path.join('share/racecar_bringup_sim', 'config'),
                   glob('config/*.yaml')),
                  (os.path.join('share/racecar_bringup_sim', 'behavior'),
                   glob('behavior/*.xml')),
                  (os.path.join('share/racecar_bringup_sim', 'maps'),
                   glob('maps/*'))],
      install_requires=['setuptools'], zip_safe=True)
