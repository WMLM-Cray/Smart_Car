from setuptools import setup
setup(name='track_generator', version='0.1.0', packages=['track_generator'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/track_generator']),
                  ('share/track_generator', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True)
