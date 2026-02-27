from setuptools import setup
import os
from glob import glob

package_name = 'gcs'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='operator',
    maintainer_email='operator@example.com',
    description='Ground control station for leader-follower drone system',
    license='MIT',
    entry_points={
        'console_scripts': [
            'telemetry_bridge = gcs.telemetry_bridge:main',
            'command_relay = gcs.command_relay:main',
        ],
    },
)
