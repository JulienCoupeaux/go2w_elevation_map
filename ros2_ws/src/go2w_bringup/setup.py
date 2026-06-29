from setuptools import setup
import os
from glob import glob

package_name = 'go2w_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    entry_points={'console_scripts': []},
)
