"""Launch file for Follower drone on Orin AGX.

Launches:
  1. MAVROS2 — connects to CubeOrange+ via USB-TTL on TELEM2
  2. swarm_bridge_node — RFD900x peer telemetry + WiFi GCS bridge
  3. follower_control_node — mission dispatch with guidance + failsafe

Hardware:
  - CubeOrange+ TELEM2 → USB-TTL → /dev/ttyUSB0 (FCU)
  - RFD900x → USB → /dev/ttyUSB1 (peer radio)
  - WiFi → GCS laptop (UDP telemetry + commands)

Requires guidance_lib and failsafe_lib on PYTHONPATH.
Set in your .bashrc or workspace setup:
  export PYTHONPATH=/home/orin/swarm_ws/src:$PYTHONPATH

Usage:
  ros2 launch follower_pkg follower_bringup.launch.py
  ros2 launch follower_pkg follower_bringup.launch.py fcu_url:=/dev/ttyTHS1:115200
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('follower_pkg')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # ── Launch arguments ────────────────────────────────────────
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='/dev/ttyUSB0:115200',
        description='CubeOrange+ TELEM2 serial (device:baudrate)')

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='MAVLink GCS URL (optional)')

    # ── MAVROS2 node (CubeOrange+ via TELEM2) ──────────────────
    # target_system_id must match FC's SYSID_THISMAV (follower = 2)
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'target_system_id': 2,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
        }],
    )

    # ── Swarm bridge node (RFD900x serial + WiFi GCS) ──────────
    swarm_bridge = Node(
        package='follower_pkg',
        executable='swarm_bridge_node',
        name='swarm_bridge_node',
        output='screen',
        parameters=[params_file],
    )

    # ── Follower control node ───────────────────────────────────
    follower_control = Node(
        package='follower_pkg',
        executable='follower_control_node',
        name='follower_control_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        mavros_node,
        swarm_bridge,
        follower_control,
    ])
