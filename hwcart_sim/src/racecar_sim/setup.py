from setuptools import setup
setup(name='racecar_sim', version='0.1.0', packages=['racecar_sim'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/racecar_sim']),
                  ('share/racecar_sim', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True)
