"""Leader Control Node — emergency override only (RC-pilot mode).

The RC pilot flies the Leader drone directly via CubeOrange+ RC receiver.
The Leader Jetson reads position/velocity from MAVROS2 (handled by
swarm_bridge_node which relays it to follower and GCS).

This node only accepts GCS emergency commands:
  KILL  — force disarm (motors stop immediately)
  RTL   — set flight mode to RTL
  LAND  — set flight mode to LAND

All other GCS commands (TAKEOFF, WASD, HOVER, FOLLOW, etc.) are ignored
because the pilot has direct RC control.

Subscribes:
    /mavros/state          — FC health (for logging)
    /swarm/gcs_command     — GCS commands from bridge

Services:
    /mavros/set_mode       — Flight mode changes (RTL, LAND)
    /mavros/cmd/arming     — Arm/disarm (KILL)
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

from swarm_msgs.msg import SwarmCommand

from .config_loader import load_hardware_config


class LeaderControlNode(Node):
    def __init__(self):
        super().__init__('leader_control_node')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('hierarchy_path', '/home/orin/hierarchy.yaml')
        self.declare_parameter('gcs_host', '')

        drone_id = self.get_parameter('drone_id').value
        hierarchy_path = self.get_parameter('hierarchy_path').value
        gcs_host = self.get_parameter('gcs_host').value

        self._cfg = load_hardware_config(
            hierarchy_path=hierarchy_path,
            drone_id=drone_id,
            gcs_host=gcs_host,
        )

        # ── FC state (for logging) ─────────────────────────────
        self._mavros_connected = False
        self._armed = False
        self._mode = ""

        # ── MAVROS2 state subscriber ───────────────────────────
        self.create_subscription(
            State, '/mavros/state', self._on_state, 10)

        # ── GCS command subscriber ─────────────────────────────
        self.create_subscription(
            SwarmCommand, '/swarm/gcs_command',
            self._on_gcs_command, 10)

        # ── MAVROS2 service clients (emergency only) ───────────
        self._set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self._arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.get_logger().info(
            f"LeaderControl (RC-pilot mode): drone_id={self._cfg.drone_id} "
            f"— emergency override only (KILL/RTL/LAND)")

    # ── MAVROS2 callback ────────────────────────────────────────

    def _on_state(self, msg: State):
        prev_conn = self._mavros_connected
        self._mavros_connected = msg.connected
        self._armed = msg.armed
        self._mode = msg.mode
        if msg.connected and not prev_conn:
            self.get_logger().info("MAVROS2 connected to CubeOrange+")

    # ── GCS command handling (emergency only) ───────────────────

    def _on_gcs_command(self, msg: SwarmCommand):
        cmd = msg.cmd

        if cmd == SwarmCommand.CMD_KILL:
            self.get_logger().info("GCS EMERGENCY: KILL — force disarm")
            self._call_arm(False)
            return

        if cmd == SwarmCommand.CMD_RTL:
            self.get_logger().info("GCS EMERGENCY: RTL — setting mode")
            self._call_set_mode("RTL")
            return

        if cmd == SwarmCommand.CMD_LAND:
            self.get_logger().info("GCS EMERGENCY: LAND — setting mode")
            self._call_set_mode("LAND")
            return

        # All other commands ignored — pilot has RC control
        self.get_logger().debug(
            f"GCS cmd={cmd} ignored (RC-pilot mode)")

    # ── MAVROS2 service calls ───────────────────────────────────

    def _call_set_mode(self, mode: str):
        # service_is_ready() is non-blocking — won't freeze the ROS2 executor.
        if not self._set_mode_cli.service_is_ready():
            self.get_logger().warning("set_mode service not available")
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self._set_mode_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"set_mode({mode}): {f.result().mode_sent}"))

    def _call_arm(self, arm: bool):
        if not self._arm_cli.service_is_ready():
            self.get_logger().warning("arming service not available")
            return
        req = CommandBool.Request()
        req.value = arm
        future = self._arm_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"arm({arm}): {f.result().success}"))


def main(args=None):
    rclpy.init(args=args)
    node = LeaderControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
