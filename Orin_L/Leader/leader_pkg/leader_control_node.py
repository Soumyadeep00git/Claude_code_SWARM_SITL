"""Leader Control Node — RC-pilot mode with autonomous failsafe monitoring.

The RC pilot flies the Leader drone directly via CubeOrange+ RC receiver.
The Leader Jetson monitors safety conditions and triggers autonomous RTL/LAND
if the pilot cannot maintain safe flight.

Failsafe checks (only when armed, 2 Hz):
  1. RC input stale        → RTL  (no RC data for rc_loss_timeout seconds)
  2. Own GPS lost          → LAND (can't navigate without GPS)
  3. Geofence breach       → RTL  (outside geofence_radius_m from home)
  4. Altitude ceiling      → RTL  (above max_altitude_m)

GCS emergency commands (always active):
  KILL  — force disarm (motors stop immediately)
  RTL   — set flight mode to RTL
  LAND  — set flight mode to LAND

Subscribes:
    /mavros/state                       — FC health (armed, mode, connected)
    /mavros/global_position/global      — GPS position
    /mavros/global_position/rel_alt     — Relative altitude
    /mavros/rc/in                       — RC input (freshness tracking)
    /swarm/gcs_command                  — GCS commands from bridge

Services:
    /mavros/set_mode       — Flight mode changes (RTL, LAND)
    /mavros/cmd/arming     — Arm/disarm (KILL)
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from mavros_msgs.msg import State, RCIn
from mavros_msgs.srv import SetMode, CommandBool

from swarm_msgs.msg import SwarmCommand

from .config_loader import load_hardware_config

from failsafe_lib import (
    FailsafeState, compute_failsafe,
    load_config as load_failsafe_config,
)


_FAILSAFE_HZ = 2  # failsafe check frequency


class LeaderControlNode(Node):
    def __init__(self):
        super().__init__('leader_control_node')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('drone_id', 1)
        self.declare_parameter('hierarchy_path', '/home/orin/hierarchy.yaml')
        self.declare_parameter('gcs_host', '')
        self.declare_parameter('rc_loss_timeout', 3.0)

        drone_id = self.get_parameter('drone_id').value
        hierarchy_path = self.get_parameter('hierarchy_path').value
        gcs_host = self.get_parameter('gcs_host').value
        self._rc_loss_timeout = self.get_parameter('rc_loss_timeout').value

        self._cfg = load_hardware_config(
            hierarchy_path=hierarchy_path,
            drone_id=drone_id,
            gcs_host=gcs_host,
        )

        # ── FC state ────────────────────────────────────────────
        self._mavros_connected = False
        self._armed = False
        self._mode = ""

        # ── GPS state ───────────────────────────────────────────
        self._lat = 0.0
        self._lon = 0.0
        self._alt = 0.0
        self._gps_valid = False

        # ── RC state ────────────────────────────────────────────
        self._rc_last_time = 0.0  # time.time() of last /mavros/rc/in

        # ── Failsafe state ──────────────────────────────────────
        self._fs_cfg = load_failsafe_config(
            home_lat=self._cfg.home_lat,
            home_lon=self._cfg.home_lon,
        )
        self._fs_state = FailsafeState()
        self._failsafe_triggered = False  # one-shot: prevents repeated triggers

        # ── MAVROS2 subscribers ─────────────────────────────────
        self.create_subscription(
            State, '/mavros/state', self._on_state, 10)
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self._on_global_pos, qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/mavros/global_position/rel_alt',
            self._on_rel_alt, qos_profile_sensor_data)
        self.create_subscription(
            RCIn, '/mavros/rc/in',
            self._on_rc_in, qos_profile_sensor_data)

        # ── GCS command subscriber ──────────────────────────────
        self.create_subscription(
            SwarmCommand, '/swarm/gcs_command',
            self._on_gcs_command, 10)

        # ── MAVROS2 service clients ─────────────────────────────
        self._set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self._arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')

        # ── Failsafe timer ──────────────────────────────────────
        self._fs_timer = self.create_timer(
            1.0 / _FAILSAFE_HZ, self._failsafe_tick)

        self.get_logger().info(
            f"LeaderControl: drone_id={self._cfg.drone_id} "
            f"rc_loss={self._rc_loss_timeout}s "
            f"geofence={self._fs_cfg.geofence_radius_m}m "
            f"alt_ceil={self._fs_cfg.max_altitude_m}m")

    # ═══════════════════════════════════════════════════════════
    # MAVROS2 callbacks
    # ═══════════════════════════════════════════════════════════

    def _on_state(self, msg: State):
        prev_conn = self._mavros_connected
        self._mavros_connected = msg.connected
        self._armed = msg.armed
        self._mode = msg.mode

        # Reset failsafe latch when disarmed (pilot landed or killed)
        if not msg.armed and self._failsafe_triggered:
            self._failsafe_triggered = False
            self._fs_state = FailsafeState()
            self.get_logger().info("Disarmed — failsafe latch reset")

        if msg.connected and not prev_conn:
            self.get_logger().info("MAVROS2 connected to CubeOrange+")

    def _on_global_pos(self, msg: NavSatFix):
        self._lat = msg.latitude
        self._lon = msg.longitude
        self._gps_valid = (msg.latitude != 0.0 or msg.longitude != 0.0)

    def _on_rel_alt(self, msg: Float64):
        self._alt = msg.data

    def _on_rc_in(self, msg: RCIn):
        self._rc_last_time = time.time()

    # ═══════════════════════════════════════════════════════════
    # GCS command handling (emergency — always active)
    # ═══════════════════════════════════════════════════════════

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

    # ═══════════════════════════════════════════════════════════
    # Failsafe tick (2 Hz)
    # ═══════════════════════════════════════════════════════════

    def _failsafe_tick(self):
        """Check safety conditions. Only active when armed."""
        if not self._mavros_connected or not self._armed:
            return
        if self._failsafe_triggered:
            return

        # ── 1. RC input stale → RTL ──────────────────────────────
        # Skip until we've received at least one RC message (avoids false
        # trigger during boot before rc_io plugin starts publishing).
        if self._rc_last_time > 0:
            rc_age = time.time() - self._rc_last_time
            if rc_age > self._rc_loss_timeout:
                self.get_logger().error(
                    f"FAILSAFE: RC LOST ({rc_age:.1f}s) -> RTL")
                self._trigger_failsafe("RTL")
                return

        # ── 2-4. GPS / geofence / altitude (via failsafe_lib) ────
        # Pass dummy peer args — leader has no leader to track.
        fs = compute_failsafe(
            own_lat=self._lat,
            own_lon=self._lon,
            own_alt=self._alt,
            own_gps_valid=self._gps_valid,
            peer_lat=0.0,
            peer_lon=0.0,
            peer_gps_valid=False,
            leader_fresh=True,      # skip leader-stale check
            in_catchup=False,       # N/A for leader
            cfg=self._fs_cfg,
            state=self._fs_state,
        )

        if not fs['safe']:
            # Map actions to leader-appropriate FC modes:
            #   LAND → LAND  (GPS lost — can't navigate to home)
            #   anything else → RTL  (ArduPilot flies home)
            mode = 'LAND' if fs['action'] == 'LAND' else 'RTL'
            self.get_logger().error(
                f"FAILSAFE: {fs['flags']} -> {mode}")
            self._trigger_failsafe(mode)

    def _trigger_failsafe(self, mode: str):
        """One-shot: set FC mode and latch to prevent repeated triggers."""
        self._failsafe_triggered = True
        self._call_set_mode(mode)

    # ═══════════════════════════════════════════════════════════
    # MAVROS2 service calls
    # ═══════════════════════════════════════════════════════════

    def _call_set_mode(self, mode: str):
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
