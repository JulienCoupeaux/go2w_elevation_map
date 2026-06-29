from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'go2w_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'terrain_cost_publisher = go2w_perception.terrain_cost_publisher:main',
            'traversability_mapper = go2w_perception.traversability_mapper:main',
            'twist_to_stamped = go2w_perception.twist_to_stamped:main',
        ],
    },
)
