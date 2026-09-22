from setuptools import setup
setup(name='racecar_strategy', version='0.1.0', packages=['racecar_strategy'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/racecar_strategy']),
                  ('share/racecar_strategy', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True)
