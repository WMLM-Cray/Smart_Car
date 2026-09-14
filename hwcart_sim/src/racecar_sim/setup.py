import os
from glob import glob
from setuptools import setup

setup(name='racecar_sim', version='0.1.1', packages=['racecar_sim'],
      data_files=[
          ('share/ament_index/resource_index/packages', ['resource/racecar_sim']),
          ('share/racecar_sim', ['package.xml']),
          ('share/racecar_sim/config', glob('config/*.yaml') + glob('config/*.yaml.in')),
          ('share/racecar_sim/worlds', glob('worlds/*.world')),
          ('share/racecar_sim/urdf', glob('urdf/*.xacro')),
          ('share/racecar_sim/launch', glob('launch/*.launch.py')),
      ],
      entry_points={
          'console_scripts': [
              'sim_wiring = racecar_sim.sim_wiring:main',
          ],
      },
      install_requires=['setuptools'], zip_safe=True)
