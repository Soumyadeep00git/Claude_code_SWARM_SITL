"""Follower Control Node — mission-dispatch with guidance + failsafe via MAVROS2.

Mirrors docker_sim/follower_missions.py + follower_controller.py but uses
MAVROS2 for flight control and ROS2 topics for swarm communication.

GUIDED MODE GATE: The entire control loop only runs when the FC is in
GUIDED mode. When the safety pilot switches to any other mode (STABILIZE,
LOITER, etc.), the Jetson stops sending velocity commands immediately.
On re-entering GUIDED, guidance state is reset for a clean handoff.

Mission state machine (same as Docker sim):
    IDLE → TAKEOFF → HOVER/FOLLOW/GEOFENCE_RETURN/RTL/LAND/KILL

Reuses guidance_lib and failsafe_lib as pure Python libraries (unchanged).

Subscribes:
    /mavros/global_position/global      — GPS position
    /mavros/global_position/rel_alt     — Relative altitude
    /mavros/local_position/velocity_local — Velocity (ENU from MAVROS)
    /mavros/global_position/compass_hdg — Heading
    /mavros/state                       — Armed, mode, connected
    /swarm/peer_states                  — Peer states from bridge
    /swarm/gcs_command                  — GCS commands from bridge

Publishes:
    /mavros/setpoint_raw/local          — Velocity NED commands
    /swarm/guidance_mode                — Current guidance mode (for bridge)
"""

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL

from swarm_msgs.msg import PeerState, PeerStates, SwarmCommand

from .config_loader import load_hardware_config

# Import pure-Python guidance and failsafe libs (on PYTHONPATH)
from guidance_lib import (
    GuidanceConfig, GuidanceState, compute_guidance,
    CommandSmoother,
)
from guidance_lib.target import TargetComputer
from failsafe_lib import FailsafeConfig, FailsafeState, compute_failsafe, load_config as load_failsafe_config


# ═══════════════════════════════════════════════════════════════
# Mission enum (matches docker_sim/follower_missions.py)
# ═══════════════════════════════════════════════════════════════

class Mission(Enum):
    IDLE            = "IDLE"
    TAKEOFF         = "TAKEOFF"
    HOVER           = "HOVER"
    FOLLOW          = "FOLLOW"
    GEOFENCE_RETURN = "GEOFENCE_RETURN"
    RTL             = "RTL"
    LAND            = "LAND"
    KILL            = "KILL"


_METERS_PER_DEG_LAT = 111_320.0
_GEOFENCE_RETURN_SPEED = 3.0  # m/s

_RC_CMD_MAP = {
    SwarmCommand.CMD_TAKEOFF: Mission.TAKEOFF,
    SwarmCommand.CMD_FOLLOW:  Mission.FOLLOW,
    SwarmCommand.CMD_HOVER:   Mission.HOVER,
    SwarmCommand.CMD_RTL:     Mission.RTL,
    SwarmCommand.CMD_LAND:    Mission.LAND,
    SwarmCommand.CMD_KILL:    Mission.KILL,
}

_MODE_CODES = {'TRACKING': 1, 'CATCHUP': 2, 'EVASION': 3}


class FollowerControlNode(Node):
    def __init__(self):
        super().__init__('follower_control_node')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('drone_id', 2)
        self.declare_parameter('hierarchy_path', '/home/orin/hierarchy.yaml')
        self.declare_parameter('control_hz', 10)
        self.declare_parameter('takeoff_alt_m', 10.0)
        self.declare_parameter('gcs_host', '')

        drone_id = self.get_parameter('drone_id').value
        hierarchy_path = self.get_parameter('hierarchy_path').value
        control_hz = self.get_parameter('control_hz').value
        self._takeoff_alt = self.get_parameter('takeoff_alt_m').value
        gcs_host = self.get_parameter('gcs_host').value

        self._cfg = load_hardware_config(
            hierarchy_path=hierarchy_path,
            drone_id=drone_id,
            gcs_host=gcs_host,
            takeoff_alt_m=self._takeoff_alt,
            control_hz=control_hz,
        )

        self._control_hz = control_hz

        self.get_logger().info(
            f"FollowerControl: drone_id={self._cfg.drone_id} "
            f"leader_id={self._cfg.leader_id} "
            f"offset=({self._cfg.offset_n:.1f}, {self._cfg.offset_e:.1f}, {self._cfg.offset_d:.1f}) "
            f"takeoff_alt={self._takeoff_alt}m"
        )

        # ── MAVROS2 state ──────────────────────────────────────
        self._mavros_connected = False
        self._armed = False
        self._mode_str = ""
        self._in_guided = False  # tracks GUIDED mode transitions
        self._gps_valid = False

        self._lat = 0.0
        self._lon = 0.0
        self._alt = 0.0
        self._vn = 0.0
        self._ve = 0.0
        self._vd = 0.0
        self._heading = 0.0

        # ── Peer state (from swarm_bridge) ──────────────────────
        self._leader_state = None  # dict-like: {lat, lon, alt, vx, vy, vz, ...}
        self._leader_state_time = 0.0  # time.time() when leader last appeared in peer_states
        self._all_peer_states = []  # list of PeerState msgs

        # ── Mission state ───────────────────────────────────────
        self._mission = Mission.IDLE
        self._is_airborne = False
        self._wants_follow = False
        self._no_leader_count = 0
        self._last_guidance_mode = ""
        self._rtl_sent = False
        self._landed = False

        # ── Pending GCS commands (queue) ────────────────────────
        self._pending_commands = []

        # ── Takeoff state ───────────────────────────────────────
        self._takeoff_sent = False
        self._takeoff_cmd_time = 0.0

        # ── Guidance + failsafe ─────────────────────────────────
        self._fs_cfg = load_failsafe_config(
            home_lat=self._cfg.home_lat,
            home_lon=self._cfg.home_lon,
        )
        self._guidance_cfg = GuidanceConfig(max_altitude_m=self._fs_cfg.max_altitude_m)
        self._guidance_state = GuidanceState()
        self._fs_state = FailsafeState()
        # alpha=1.0 disables smoothing for initial hardware testing (raw guidance output).
        # Once guidance is validated on hardware, reduce to 0.3-0.5 for smoother
        # flight. Lower alpha = smoother but adds latency on top of GPS + radio lag.
        self._smoother = CommandSmoother(alpha=1.0, dt=1.0 / control_hz)
        self._target_computer = TargetComputer()

        # ── MAVROS2 Subscribers ─────────────────────────────────
        self.create_subscription(
            State, '/mavros/state', self._on_state, 10)
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self._on_global_pos, qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/mavros/global_position/rel_alt',
            self._on_rel_alt, qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self._on_velocity, qos_profile_sensor_data)
        self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg',
            self._on_heading, qos_profile_sensor_data)

        # ── Swarm subscribers ───────────────────────────────────
        self.create_subscription(
            PeerStates, '/swarm/peer_states',
            self._on_peer_states, 10)
        self.create_subscription(
            SwarmCommand, '/swarm/gcs_command',
            self._on_gcs_command, 10)

        # ── Publishers ──────────────────────────────────────────
        self._setpoint_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', 10)
        self._guidance_mode_pub = self.create_publisher(
            Float64, '/swarm/guidance_mode', 10)
        self._mission_state_pub = self.create_publisher(
            String, '/swarm/mission_state', 10)

        # ── MAVROS2 service clients ─────────────────────────────
        self._set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self._arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self._takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        # ── Control loop timer ──────────────────────────────────
        self._timer = self.create_timer(1.0 / control_hz, self._control_tick)

        self.get_logger().info("Follower control node started")

    # ═══════════════════════════════════════════════════════════
    # MAVROS2 callbacks
    # ═══════════════════════════════════════════════════════════

    def _on_state(self, msg: State):
        self._mavros_connected = msg.connected
        self._armed = msg.armed
        self._mode_str = msg.mode

    def _on_global_pos(self, msg: NavSatFix):
        self._lat = msg.latitude
        self._lon = msg.longitude
        self._gps_valid = (msg.latitude != 0.0 or msg.longitude != 0.0)

    def _on_rel_alt(self, msg: Float64):
        self._alt = msg.data

    def _on_velocity(self, msg: TwistStamped):
        # MAVROS2 velocity_local is in ENU frame: x=East, y=North, z=Up
        # Convert to NED: North=y, East=x, Down=-z
        # PRE-FLIGHT CHECK: Verify this mapping by moving the drone by hand
        # and comparing `ros2 topic echo /mavros/local_position/velocity_local`
        # with expected direction. If axes are swapped, adjust here.
        self._vn = msg.twist.linear.y
        self._ve = msg.twist.linear.x
        self._vd = -msg.twist.linear.z

    def _on_heading(self, msg: Float64):
        self._heading = msg.data

    # ═══════════════════════════════════════════════════════════
    # Swarm callbacks
    # ═══════════════════════════════════════════════════════════

    def _on_peer_states(self, msg: PeerStates):
        """Store all peer states, extract leader state."""
        self._all_peer_states = msg.peers
        self._leader_state = None
        for peer in msg.peers:
            if peer.drone_id == self._cfg.leader_id:
                self._leader_state = {
                    'drone_id': peer.drone_id,
                    'lat': peer.latitude,
                    'lon': peer.longitude,
                    'alt': peer.altitude,
                    'vx': peer.vn,
                    'vy': peer.ve,
                    'vz': peer.vd,
                    'heading': peer.heading,
                    'guidance_mode': peer.guidance_mode,
                    'stamp': peer.stamp,  # sender's Unix timestamp
                }
                self._leader_state_time = time.time()  # local receive time
                break

    def _on_gcs_command(self, msg: SwarmCommand):
        """Queue GCS commands for processing in control tick."""
        self._pending_commands.append(msg)

    # ═══════════════════════════════════════════════════════════
    # Main control loop (10 Hz)
    # ═══════════════════════════════════════════════════════════

    def _control_tick(self):
        """Mission-dispatch architecture: check → decide → execute.

        GUIDED mode gate: when safety pilot switches FC away from GUIDED,
        all velocity commands stop immediately. On re-entering GUIDED,
        guidance state is reset for a clean handoff.
        """
        # Pre-flight: wait for MAVROS2
        if not self._mavros_connected:
            return

        # ── GUIDED mode gate ──────────────────────────────────
        is_guided = ('GUIDED' in self._mode_str)

        if not is_guided:
            if self._in_guided:
                # Just left GUIDED — safety pilot took over
                self.get_logger().info(
                    f"FC mode: {self._mode_str} — "
                    f"suspending guidance (was {self._mission.value})")
                # Reset guidance state so we don't resume mid-maneuver
                self._smoother.reset()
                self._guidance_state = GuidanceState()
                self._fs_state = FailsafeState()
                self._last_guidance_mode = ""
                self._no_leader_count = 0
                self._in_guided = False
            # Drain pending GCS commands (discard — pilot has control)
            while self._pending_commands:
                cmd_msg = self._pending_commands.pop(0)
                self.get_logger().debug(
                    f"GCS cmd={cmd_msg.cmd} discarded (FC not in GUIDED)")
            # Still publish mission state so debug logger can see
            state_msg = String()
            state_msg.data = f"NOT_GUIDED ({self._mode_str})"
            self._mission_state_pub.publish(state_msg)
            return  # Do NOT execute any mission logic

        if not self._in_guided:
            # Just entered GUIDED — detect if already airborne
            self.get_logger().info("FC mode: GUIDED — resuming control")
            self._in_guided = True
            if self._armed and self._alt > 2.0:
                # Safety pilot took off manually, then switched to GUIDED
                self.get_logger().info(
                    f"Already airborne at {self._alt:.1f}m — setting HOVER")
                self._is_airborne = True
                if self._mission == Mission.IDLE:
                    self._mission = Mission.HOVER
            elif self._mission not in (Mission.IDLE, Mission.TAKEOFF):
                # Re-entering GUIDED on the ground after being in-flight
                self._mission = Mission.IDLE
                self._is_airborne = False
        # ── End GUIDED mode gate ──────────────────────────────

        # Leader is "fresh" only if we received data within the last 2 seconds.
        # This prevents stale cached data from being used if the radio link dies.
        leader_fresh = (self._leader_state is not None
                        and time.time() - self._leader_state_time < 2.0)

        # ── 1. Determine mission ────────────────────────────────
        mission = self._get_mission(leader_fresh)

        # ── 2. Handle transitions ───────────────────────────────
        if mission != self._mission:
            self._on_exit(self._mission)
            self._on_enter(mission)
            self.get_logger().info(f"Mission: {self._mission.value} -> {mission.value}")
            self._mission = mission

        # ── 3. Publish mission state for debug logger ───────────
        state_msg = String()
        state_msg.data = self._mission.value
        self._mission_state_pub.publish(state_msg)

        # ── 4. Execute mission ──────────────────────────────────
        self._execute_mission(self._mission, leader_fresh)

    # ═══════════════════════════════════════════════════════════
    # 1. get_mission — THE ONLY place transitions happen
    # ═══════════════════════════════════════════════════════════

    def _get_mission(self, leader_fresh: bool) -> Mission:
        mission = self._mission

        # ── RC commands (drain all, last wins) ──────────────────
        rc_mission = None
        while self._pending_commands:
            cmd_msg = self._pending_commands.pop(0)
            mapped = _RC_CMD_MAP.get(cmd_msg.cmd)
            if mapped is not None:
                rc_mission = mapped

        if rc_mission is not None:
            if rc_mission == Mission.FOLLOW and not self._gps_valid:
                self.get_logger().warning("RC: FOLLOW denied — no GPS")
            elif rc_mission == Mission.FOLLOW and not self._is_airborne:
                self.get_logger().warning("RC: FOLLOW denied — not airborne")
            elif rc_mission == Mission.TAKEOFF and mission != Mission.IDLE:
                self.get_logger().warning(
                    f"RC: TAKEOFF denied — not in IDLE (currently {mission.value})")
            else:
                mission = rc_mission
                if rc_mission == Mission.FOLLOW:
                    self._wants_follow = True
                elif rc_mission in (Mission.HOVER, Mission.RTL, Mission.LAND, Mission.KILL):
                    self._wants_follow = False

        # ── Global failsafe ─────────────────────────────────────
        failsafe_clear = True
        if mission in (Mission.HOVER, Mission.FOLLOW, Mission.GEOFENCE_RETURN):
            check_leader = (mission == Mission.FOLLOW
                            or (mission == Mission.HOVER and self._wants_follow))

            if check_leader:
                fs = compute_failsafe(
                    own_lat=self._lat,
                    own_lon=self._lon,
                    own_alt=self._alt,
                    own_gps_valid=self._gps_valid,
                    peer_lat=self._leader_state['lat'] if self._leader_state else 0.0,
                    peer_lon=self._leader_state['lon'] if self._leader_state else 0.0,
                    peer_gps_valid=bool(self._leader_state and self._leader_state['lat'] != 0),
                    leader_fresh=leader_fresh,
                    in_catchup=(self._last_guidance_mode == 'CATCHUP'),
                    cfg=self._fs_cfg,
                    state=self._fs_state,
                )
            else:
                fs = compute_failsafe(
                    own_lat=self._lat,
                    own_lon=self._lon,
                    own_alt=self._alt,
                    own_gps_valid=self._gps_valid,
                    peer_lat=0.0,
                    peer_lon=0.0,
                    peer_gps_valid=False,
                    leader_fresh=True,
                    in_catchup=False,
                    cfg=self._fs_cfg,
                    state=self._fs_state,
                )

            if not fs['safe']:
                failsafe_clear = False
                self.get_logger().warning(f"FAILSAFE: {fs['flags']} action={fs['action']}")
                if fs['action'] == 'RTL':
                    mission = Mission.RTL
                elif fs['action'] == 'LAND':
                    mission = Mission.LAND
                elif fs['action'] == 'GEOFENCE_RETURN':
                    if mission not in (Mission.RTL, Mission.LAND, Mission.KILL):
                        mission = Mission.GEOFENCE_RETURN
                else:
                    if mission not in (Mission.RTL, Mission.LAND, Mission.KILL):
                        mission = Mission.HOVER

        # ── Geofence recovery ───────────────────────────────────
        if mission == Mission.GEOFENCE_RETURN and failsafe_clear:
            self.get_logger().info("Back inside geofence — resuming HOVER")
            mission = Mission.HOVER

        # ── Auto-resume FOLLOW ──────────────────────────────────
        if (mission == Mission.HOVER and self._wants_follow
                and failsafe_clear
                and leader_fresh and self._gps_valid):
            self.get_logger().info("Failsafe cleared — resuming FOLLOW")
            mission = Mission.FOLLOW

        # ── Takeoff complete ────────────────────────────────────
        if mission == Mission.TAKEOFF and self._alt >= self._takeoff_alt * 0.90:
            self.get_logger().info(f"Airborne at {self._alt:.1f}m!")
            mission = Mission.HOVER
            self._is_airborne = True

        # ── Leader lost timeout ─────────────────────────────────
        if mission == Mission.FOLLOW and self._no_leader_count > self._control_hz * 30:
            self.get_logger().warning("Leader lost for 30s — RTL")
            mission = Mission.RTL

        # ── Landed detection ────────────────────────────────────
        if self._landed and mission in (Mission.RTL, Mission.LAND):
            self.get_logger().info("Grounded — returning to IDLE")
            mission = Mission.IDLE
            self._landed = False
            self._is_airborne = False
            self._rtl_sent = False
            self._wants_follow = False

        return mission

    # ═══════════════════════════════════════════════════════════
    # Transition handlers
    # ═══════════════════════════════════════════════════════════

    def _on_exit(self, old: Mission):
        if old == Mission.FOLLOW:
            self._fs_state.stale_counter = 0
            self._fs_state.catchup_ticks = 0
            self._last_guidance_mode = ""
            self._smoother.reset()
            self._guidance_state = GuidanceState()

    def _on_enter(self, new: Mission):
        if new == Mission.TAKEOFF:
            self._takeoff_sent = False
            self._takeoff_cmd_time = 0.0
        elif new == Mission.FOLLOW:
            self._no_leader_count = 0
        elif new == Mission.RTL:
            if not self._rtl_sent:
                self._call_set_mode("RTL")
                self._rtl_sent = True
        elif new == Mission.LAND:
            self._call_set_mode("LAND")
        elif new == Mission.KILL:
            self._call_arm(False)
            self.get_logger().info("FORCE DISARM (KILL)")

    # ═══════════════════════════════════════════════════════════
    # 3. execute_mission — dispatch to per-mission handler
    # ═══════════════════════════════════════════════════════════

    def _execute_mission(self, mission: Mission, leader_fresh: bool):
        if mission == Mission.IDLE:
            self._do_idle()
        elif mission == Mission.TAKEOFF:
            self._do_takeoff()
        elif mission == Mission.HOVER:
            self._do_hover()
        elif mission == Mission.FOLLOW:
            self._do_follow(leader_fresh)
        elif mission == Mission.GEOFENCE_RETURN:
            self._do_geofence_return()
        elif mission == Mission.RTL:
            self._do_rtl()
        elif mission == Mission.LAND:
            self._do_land()
        elif mission == Mission.KILL:
            pass  # Already disarmed in _on_enter

    # ═══════════════════════════════════════════════════════════
    # Mission handlers
    # ═══════════════════════════════════════════════════════════

    def _do_idle(self):
        """Waiting for TAKEOFF command."""
        pass

    def _do_takeoff(self):
        """Non-blocking takeoff: GUIDED → ARM → TAKEOFF → detect altitude.

        FIRST FLIGHT RECOMMENDATION: Have the safety pilot arm and take off
        manually in STABILIZE/LOITER, then switch to GUIDED once at altitude.
        The code handles this via the GUIDED mode gate ("already airborne"
        detection). This avoids relying on automatic arming/takeoff until
        the system is proven in the field.
        """
        now = self.get_clock().now().nanoseconds / 1e9

        if now - self._takeoff_cmd_time < 2.0:
            return

        if not self._gps_valid:
            self.get_logger().info("Takeoff: waiting for GPS...")
            self._takeoff_cmd_time = now
            return

        if 'GUIDED' not in self._mode_str:
            self.get_logger().info("Takeoff: setting GUIDED mode")
            self._call_set_mode("GUIDED")
            self._takeoff_cmd_time = now
            return

        if not self._armed:
            self.get_logger().info("Takeoff: arming")
            self._call_arm(True)
            self._takeoff_cmd_time = now
            return

        if not self._takeoff_sent:
            self.get_logger().info(f"Takeoff: commanding {self._takeoff_alt:.0f}m")
            self._call_takeoff(self._takeoff_alt)
            self._takeoff_sent = True
            self._takeoff_cmd_time = now
            return

        # Re-send takeoff if stalled
        if now - self._takeoff_cmd_time >= 3.0:
            self._call_takeoff(self._takeoff_alt)
            self._takeoff_cmd_time = now

    def _do_hover(self):
        """Hold position — zero velocity."""
        self._send_velocity_ned(0.0, 0.0, 0.0)

    def _do_follow(self, leader_fresh: bool):
        """Full guidance loop with leader tracking."""
        if leader_fresh:
            self._no_leader_count = 0
        else:
            self._no_leader_count += 1

        leader = self._leader_state
        if leader is None or (leader['lat'] == 0.0 and leader['lon'] == 0.0):
            self._send_velocity_ned(0.0, 0.0, 0.0)
            return

        if self._lat == 0.0 and self._lon == 0.0:
            self._send_velocity_ned(0.0, 0.0, 0.0)
            return

        # Compute target = leader + offset + feedforward
        target_lat, target_lon, target_alt = self._target_computer.compute(
            leader['lat'], leader['lon'], leader['alt'],
            leader['vx'], leader['vy'],
            self._cfg.offset_n, self._cfg.offset_e, self._cfg.offset_d,
            ff_gain=self._guidance_cfg.ff_gain, dt=1.0 / self._control_hz,
        )

        # Gather neighbor states for collision avoidance
        neighbors = None
        if self._all_peer_states:
            neighbors = []
            for peer in self._all_peer_states:
                neighbors.append({
                    'drone_id': peer.drone_id,
                    'lat': peer.latitude,
                    'lon': peer.longitude,
                    'alt': peer.altitude,
                    'vx': peer.vn,
                    'vy': peer.ve,
                })

        # 3-mode guidance + collision avoidance
        result = compute_guidance(
            my_lat=self._lat, my_lon=self._lon, my_alt=self._alt,
            my_vn=self._vn, my_ve=self._ve, my_vd=self._vd,
            peer_lat=leader['lat'], peer_lon=leader['lon'], peer_alt=leader['alt'],
            peer_vn=leader['vx'], peer_ve=leader['vy'],
            goal_lat=target_lat, goal_lon=target_lon, goal_alt=target_alt,
            cfg=self._guidance_cfg, state=self._guidance_state,
            neighbors=neighbors, my_id=self._cfg.drone_id,
        )

        # Smooth & send
        sm_vn, sm_ve, sm_vd = self._smoother.filter(
            result['vn'], result['ve'], result['vd'])
        self._send_velocity_ned(sm_vn, sm_ve, sm_vd)

        # Report guidance mode to bridge for telemetry
        mode_code = _MODE_CODES.get(result['mode'], 0)
        mode_msg = Float64()
        mode_msg.data = float(mode_code)
        self._guidance_mode_pub.publish(mode_msg)

        if result['mode'] != 'TRACKING':
            self.get_logger().info(
                f"Guidance: {result['mode']} "
                f"(w_e={result['w_evasion']:.2f} "
                f"w_t={result['w_tracking']:.2f} "
                f"w_c={result['w_catchup']:.2f})")

        self._last_guidance_mode = result['mode']

    def _do_geofence_return(self):
        """Fly radially toward home to get back inside geofence."""
        # Report FAILSAFE mode to bridge
        mode_msg = Float64()
        mode_msg.data = 4.0
        self._guidance_mode_pub.publish(mode_msg)

        if not self._gps_valid:
            self._send_velocity_ned(0.0, 0.0, 0.0)
            return

        home_lat = self._fs_cfg.home_lat
        home_lon = self._fs_cfg.home_lon

        dn = (home_lat - self._lat) * _METERS_PER_DEG_LAT
        de = ((home_lon - self._lon) * _METERS_PER_DEG_LAT
              * math.cos(math.radians(home_lat)))
        dist = math.sqrt(dn * dn + de * de + 1e-6)

        vn = _GEOFENCE_RETURN_SPEED * dn / dist
        ve = _GEOFENCE_RETURN_SPEED * de / dist

        self._send_velocity_ned(vn, ve, 0.0)

    def _do_rtl(self):
        """Wait for landing after RTL."""
        if self._alt < 1.0 and not self._armed:
            self.get_logger().info("Landed (RTL)")
            self._landed = True

    def _do_land(self):
        """Wait for landing after LAND."""
        if self._alt < 1.0 and not self._armed:
            self.get_logger().info("Landed (LAND)")
            self._landed = True

    # ═══════════════════════════════════════════════════════════
    # Velocity command
    # ═══════════════════════════════════════════════════════════

    def _send_velocity_ned(self, vn: float, ve: float, vd: float):
        """Send NED velocity command via MAVROS2 setpoint_raw/local."""
        if not (math.isfinite(vn) and math.isfinite(ve) and math.isfinite(vd)):
            self.get_logger().warning(
                f"NaN/Inf in velocity (vn={vn:.2f} ve={ve:.2f} vd={vd:.2f}) — zeroing")
            vn, ve, vd = 0.0, 0.0, 0.0

        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        )
        msg.velocity.x = float(vn)  # North
        msg.velocity.y = float(ve)  # East
        msg.velocity.z = float(vd)  # Down
        self._setpoint_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════
    # MAVROS2 service calls (async)
    # ═══════════════════════════════════════════════════════════

    def _call_set_mode(self, mode: str):
        # service_is_ready() is non-blocking — won't freeze the 10Hz control loop.
        # wait_for_service() would block the ROS2 executor for up to 1s, stalling
        # all subscriptions and timers.
        if not self._set_mode_cli.service_is_ready():
            self.get_logger().warning("set_mode service not available")
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self._set_mode_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f"set_mode({mode}): {f.result().mode_sent}"))

    def _call_arm(self, arm: bool):
        if not self._arm_cli.service_is_ready():
            self.get_logger().warning("arming service not available")
            return
        req = CommandBool.Request()
        req.value = arm
        future = self._arm_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f"arm({arm}): {f.result().success}"))

    def _call_takeoff(self, alt: float):
        if not self._takeoff_cli.service_is_ready():
            self.get_logger().warning("takeoff service not available")
            return
        req = CommandTOL.Request()
        req.altitude = alt
        future = self._takeoff_cli.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f"takeoff({alt}m): {f.result().success}"))


def main(args=None):
    rclpy.init(args=args)
    node = FollowerControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
