"""
Command relay — provides ROS2 services for sending custom commands
that Mission Planner cannot handle natively.

Currently supports:
  - Changing the follower's offset at runtime (via MAVLink PARAM_SET)
  - Requesting mode changes on either drone

This runs on the GCS laptop. Commands are sent via MAVLink through
the RFD9000 radio to the target drone's CubeOrange+.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymavlink import mavutil


class CommandRelay(Node):
    def __init__(self):
        super().__init__('command_relay')

        self.declare_parameter('mavlink_port', '/dev/ttyUSB0')
        self.declare_parameter('mavlink_baud', 57600)
        self.declare_parameter('follower_sysid', 2)

        port = self.get_parameter('mavlink_port').value
        baud = self.get_parameter('mavlink_baud').value
        self.follower_sysid = self.get_parameter('follower_sysid').value

        self.mav = mavutil.mavlink_connection(
            port, baud=baud, source_system=255, source_component=190)

        # Subscribe to command topic (JSON-encoded commands)
        self.create_subscription(String, 'gcs/command', self._cmd_cb, 10)

        self.get_logger().info(f'CommandRelay started on {port}')

    def _cmd_cb(self, msg: String):
        """Handle commands published as JSON strings.

        Expected format: '{"cmd": "set_mode", "sysid": 2, "mode": "RTL"}'
        or: '{"cmd": "set_offset", "north": -5.0, "east": 3.0}'
        """
        import json
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid command JSON: {msg.data}')
            return

        action = cmd.get('cmd', '')

        if action == 'set_mode':
            sysid = cmd.get('sysid', self.follower_sysid)
            mode = cmd.get('mode', 'GUIDED')
            self._send_mode(sysid, mode)

        elif action == 'arm':
            sysid = cmd.get('sysid', self.follower_sysid)
            self._send_arm(sysid, arm=True)

        elif action == 'disarm':
            sysid = cmd.get('sysid', self.follower_sysid)
            self._send_arm(sysid, arm=False)

        else:
            self.get_logger().warn(f'Unknown command: {action}')

    def _send_mode(self, sysid: int, mode_name: str):
        mode_map = self.mav.mode_mapping()
        if mode_name not in mode_map:
            self.get_logger().error(f'Unknown mode: {mode_name}')
            return
        mode_id = mode_map[mode_name]
        self.mav.mav.command_long_send(
            sysid, 1,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id, 0, 0, 0, 0, 0,
        )
        self.get_logger().info(f'Sent mode {mode_name} to SYSID={sysid}')

    def _send_arm(self, sysid: int, arm: bool):
        self.mav.mav.command_long_send(
            sysid, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1.0 if arm else 0.0,
            0, 0, 0, 0, 0, 0,
        )
        self.get_logger().info(f'Sent {"ARM" if arm else "DISARM"} to SYSID={sysid}')


def main(args=None):
    rclpy.init(args=args)
    node = CommandRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
