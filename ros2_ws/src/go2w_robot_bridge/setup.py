import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'go2w_robot_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='julien',
    maintainer_email='juliencoupeaux@gmail.com',
    description='Ponts ROS<->Unitree pour le vrai go2w (odom + cmd_vel->sport Move).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_to_sport = go2w_robot_bridge.cmd_vel_to_sport:main',
            'sportmode_to_odom = go2w_robot_bridge.sportmode_to_odom:main',
        ],
    },
)
