import os
from glob import glob
from setuptools import setup

# Gazebo model 目录递归装载(trackA_cone_{blue,red}:model.config/model.sdf
# + meshes/*.dae)。data_files 是平铺的 (目标目录, 文件表),须逐子目录展开。
models_files = []
for _root, _dirs, _files in os.walk('models'):
    _rel = os.path.relpath(_root, 'models')
    _dst = ('share/racecar_sim/models' if _rel == '.'
            else 'share/racecar_sim/models/%s' % _rel.replace(os.sep, '/'))
    _fl = [os.path.join(_root, f) for f in _files]
    if _fl:
        models_files.append((_dst, _fl))

setup(name='racecar_sim', version='0.1.1', packages=['racecar_sim'],
      data_files=[
          ('share/ament_index/resource_index/packages', ['resource/racecar_sim']),
          ('share/racecar_sim', ['package.xml']),
          ('share/racecar_sim/config', glob('config/*.yaml') + glob('config/*.yaml.in')),
          ('share/racecar_sim/worlds', glob('worlds/*.world')),
          ('share/racecar_sim/rviz', glob('rviz/*.rviz')),
          ('share/racecar_sim/urdf', glob('urdf/*.xacro')),
          ('share/racecar_sim/launch', glob('launch/*.launch.py')),
      ] + models_files,
      entry_points={
          'console_scripts': [
              'sim_wiring = racecar_sim.sim_wiring:main',
          ],
      },
      install_requires=['setuptools'], zip_safe=True)
