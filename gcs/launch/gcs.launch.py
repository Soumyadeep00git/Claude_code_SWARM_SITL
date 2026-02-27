"""Launch file for the GCS laptop — telemetry logging and command relay."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('gcs')
    params = os.path.join(pkg_dir, 'config', 'gcs_params.yaml')

    return LaunchDescription([
        # Telemetry bridge: reads both drones from RFD9000, logs to CSV
        Node(
            package='gcs',
            executable='telemetry_bridge',
            name='telemetry_bridge',
            output='screen',
            parameters=[params],
        ),

        # Command relay: sends follow offset changes to follower
        Node(
            package='gcs',
            executable='command_relay',
            name='command_relay',
            output='screen',
            parameters=[params],
        ),
    ])
