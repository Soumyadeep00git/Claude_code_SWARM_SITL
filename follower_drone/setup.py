from setuptools import setup
import os
from glob import glob

package_name = 'follower_drone'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='operator',
    maintainer_email='operator@example.com',
    description='Follower drone agent for leader-follower formation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follow_controller = follower_drone.follow_controller:main',
            'peer_monitor = follower_drone.peer_monitor:main',
            'failsafe_node = follower_drone.failsafe_node:main',
        ],
    },
)
