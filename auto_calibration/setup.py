from glob import glob
from setuptools import setup

package_name = 'auto_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'README.zh-TW.md', 'LICENSE.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chentom',
    maintainer_email='chentom@example.com',
    description='UR3 automatic hand-eye sampling and camera-root TF publication.',
    license='LGPLv3',
    entry_points={'console_scripts': [
        'calibrate = auto_calibration.calibrate:main',
        'handeye_publisher = auto_calibration.publisher:main',
    ]},
)
