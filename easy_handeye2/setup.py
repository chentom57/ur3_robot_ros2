import os
from glob import glob

from setuptools import setup

package_name = 'easy_handeye2'

setup(
 name=package_name,
 version='0.5.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
     ('share/' + package_name + '/resource', glob(os.path.join('resource', '*.ui'))),
     ('share/' + package_name, ['package.xml', 'README.md', 'README.zh-TW.md', 'LICENSE.md']),
     ('share/' + package_name, ['plugin_evaluator.xml','plugin_calibrator.xml']),
     (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py')))
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Marco Esposito',
 maintainer_email='esposito@imfusion.com',
 description='Simple, hardware-independent ROS2 library for hand-eye calibration',
 license='LGPLv3',
 entry_points={
     'console_scripts': [
                'handeye_server = easy_handeye2.handeye_server:main',
                'rqt_calibrator.py = easy_handeye2.handeye_rqt_calibrator:main',
                'rqt_evaluator.py = easy_handeye2.handeye_rqt_evaluator:main',
     ],
   },
)
