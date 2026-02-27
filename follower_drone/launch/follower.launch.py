"""Launch file for the follower drone — starts mavlink-router, MAVROS2, and all custom nodes."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('follower_drone')
    params = os.path.join(pkg_dir, 'config', 'follower_params.yaml')
    mavlink_conf = os.path.join(pkg_dir, 'config', 'mavlink_router.conf')

    return LaunchDescription([
        # mavlink-router: connects CO+ ↔ RFD9000 ↔ MAVROS2 ↔ custom nodes
        ExecuteProcess(
            cmd=['mavlink-routerd', '-c', mavlink_conf],
            name='mavlink_router',
            output='screen',
        ),

        # MAVROS2: ROS2 ↔ MAVLink bridge for local CubeOrange+ (SYSID=2)
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            namespace='follower',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://:14550@127.0.0.1:14550',
                'gcs_url': '',
                'target_system_id': 2,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
            }],
        ),

        # Peer monitor: reads leader MAVLink from radio mesh
        Node(
            package='follower_drone',
            executable='peer_monitor',
            name='peer_monitor',
            namespace='follower',
            output='screen',
            parameters=[params],
        ),

        # Follow controller: PD offset tracking + potential field
        Node(
            package='follower_drone',
            executable='follow_controller',
            name='follow_controller',
            namespace='follower',
            output='screen',
            parameters=[params],
        ),

        # Failsafe: follower state machine (leader-loss detection)
        Node(
            package='follower_drone',
            executable='failsafe_node',
            name='failsafe_node',
            namespace='follower',
            output='screen',
            parameters=[params],
        ),
    ])
