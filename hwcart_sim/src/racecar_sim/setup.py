import os
from glob import glob
from setuptools import setup

# 锥桶资产已切换到队友包 racecar_test_assets(cone_{blue,red},官方630/280包络,
# static 测试专用):models 由该包 CMake 安装并用 env-hook 导出
# GAZEBO_MODEL_PATH;本包不再自带 model 目录(历史 trackA_cone_* 见 git)。
setup(name='racecar_sim', version='0.1.2', packages=['racecar_sim'],
      data_files=[
          ('share/ament_index/resource_index/packages', ['resource/racecar_sim']),
          ('share/racecar_sim', ['package.xml']),
          ('share/racecar_sim/config', glob('config/*.yaml') + glob('config/*.yaml.in')),
          ('share/racecar_sim/worlds', glob('worlds/*.world')),
          ('share/racecar_sim/rviz', glob('rviz/*.rviz')),
          ('share/racecar_sim/urdf', glob('urdf/*.xacro')),
          ('share/racecar_sim/launch', glob('launch/*.launch.py')),
      ],
      entry_points={
          'console_scripts': [
              'sim_wiring = racecar_sim.sim_wiring:main',
          ],
      },
      install_requires=['setuptools'], zip_safe=True)
