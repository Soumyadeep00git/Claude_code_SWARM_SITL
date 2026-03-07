from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'follower_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swarm_dev',
    maintainer_email='dev@swarm.local',
    description='ROS2 follower drone nodes for Orin AGX hardware',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follower_control_node = follower_pkg.follower_control_node:main',
            'swarm_bridge_node = follower_pkg.swarm_bridge_node:main',
        ],
    },
)
