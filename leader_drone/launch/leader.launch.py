"""Launch file for the leader drone — starts mavlink-router, MAVROS2, and all custom nodes."""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('leader_drone')
    params = os.path.join(pkg_dir, 'config', 'leader_params.yaml')
    mavlink_conf = os.path.join(pkg_dir, 'config', 'mavlink_router.conf')

    return LaunchDescription([
        # mavlink-router: connects CO+ ↔ RFD9000 ↔ MAVROS2 ↔ custom nodes
        ExecuteProcess(
            cmd=['mavlink-routerd', '-c', mavlink_conf],
            name='mavlink_router',
            output='screen',
        ),

        # MAVROS2: ROS2 ↔ MAVLink bridge for local CubeOrange+ (SYSID=1)
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            namespace='leader',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://:14550@127.0.0.1:14550',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0',
            }],
        ),

        # Peer monitor: reads follower MAVLink from radio mesh
        Node(
            package='leader_drone',
            executable='peer_monitor',
            name='peer_monitor',
            namespace='leader',
            output='screen',
            parameters=[params],
        ),

        # Leader node: collision avoidance overlay
        Node(
            package='leader_drone',
            executable='leader_node',
            name='leader_node',
            namespace='leader',
            output='screen',
            parameters=[params],
        ),

        # RC relay: forwards RC sticks to follower when switch active
        Node(
            package='leader_drone',
            executable='rc_relay_node',
            name='rc_relay_node',
            namespace='leader',
            output='screen',
            parameters=[params],
        ),

        # Failsafe: leader state machine
        Node(
            package='leader_drone',
            executable='failsafe_node',
            name='failsafe_node',
            namespace='leader',
            output='screen',
            parameters=[params],
        ),
    ])
