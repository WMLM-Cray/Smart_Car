import os
from glob import glob
from setuptools import setup

setup(name='racecar_sim', version='0.1.1', packages=['racecar_sim'],
      data_files=[
          ('share/ament_index/resource_index/packages', ['resource/racecar_sim']),
          ('share/racecar_sim', ['package.xml']),
          ('share/racecar_sim/config', glob('config/*.yaml')),
          ('share/racecar_sim/urdf', glob('urdf/*.xacro')),
          ('share/racecar_sim/launch', glob('launch/*.launch.py')),
      ],
      install_requires=['setuptools'], zip_safe=True)
