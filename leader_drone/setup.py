from setuptools import setup
import os
from glob import glob

package_name = 'leader_drone'

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
    install_requires=['setuptools', 'pymavlink', 'follower_drone'],
    zip_safe=True,
    maintainer='operator',
    maintainer_email='operator@example.com',
    description='Leader drone agent for leader-follower formation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'leader_node = leader_drone.leader_node:main',
            'rc_relay_node = leader_drone.rc_relay_node:main',
            'peer_monitor = leader_drone.peer_monitor:main',
            'failsafe_node = leader_drone.failsafe_node:main',
        ],
    },
)
