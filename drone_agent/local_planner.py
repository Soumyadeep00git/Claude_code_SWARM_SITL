"""
Local planner — executes immediate movement commands.
Translates high-level intents (waypoint, velocity, takeoff, land, formation)
into MAVLink calls via DroneConnection.

FORMATION state uses a deterministic PD + potential field controller
for stable, collision-aware velocity commands.
ALL velocity-producing states (WAYPOINT, VELOCITY, HOLD, IDLE) apply
exponential collision avoidance with velocity cancellation.

Cooperative: receives PROXIMITY_ALERT from peers and decelerates
when identified as threat or nearby bystander.
"""

import time
import math
import logging
import numpy as np
from math import cos, radians

from mavlink_layer.drone_connection import DroneConnection
from drone_agent.formation_controller import FormationController, MAX_SPEED
from drone_agent.geo import gps_to_ned
from config import ISOLATION_RADIUS_M, ISOLATION_SOFT_ZONE_M

# Repulsion constants for _collision_overlay (non-formation modes)
REPEL_GAIN = 2.0     # Repulsion strength multiplier

log = logging.getLogger(__name__)


# Task states
IDLE = "IDLE"
TAKEOFF = "TAKEOFF"
WAYPOINT = "WAYPOINT"
VELOCITY = "VELOCITY"
LAND = "LAND"
HOLD = "HOLD"
FORMATION = "FORMATION"
RL_MODE = "RL_MODE"

# Waypoint PD gains
KP_WP = 0.5
MAX_VERT_WP = 1.0

# Alert deceleration factors
HARD_DECEL_FACTOR = 0.3   # Threat drone: 30% speed
SOFT_DECEL_FACTOR = 0.6   # Nearby bystander: 60% speed
ALERT_HARD_TTL = 3.0      # Seconds before hard alert expires
ALERT_SOFT_TTL = 2.0      # Seconds before soft alert expires


class LocalPlanner:
    """Executes movement commands against one SITL instance."""

    def __init__(self, conn: DroneConnection):
        self.conn = conn
        self.task: str = IDLE

        # Waypoint target (global coords)
        self.wp_lat: float = 0.0
        self.wp_lon: float = 0.0
        self.wp_alt: float = 0.0

        # Velocity target (NED m/s)
        self.vel_n: float = 0.0
        self.vel_e: float = 0.0
        self.vel_d: float = 0.0

        # Takeoff state machine
        self._takeoff_alt: float = 0.0
        self._arm_sent_time: float = 0.0
        self._takeoff_sent: bool = False

        # Formation controller (PD + potential field)
        self._formation_ctrl = FormationController()

        # Formation tracking data (set by agent each tick)
        self._goal_ned = np.zeros(3, dtype=np.float64)
        self._path_waypoints_ned = np.empty((0, 3), dtype=np.float64)
        self._peer_positions_ned = np.empty((0, 3), dtype=np.float64)
        self._peer_velocities_ned = np.empty((0, 3), dtype=np.float64)

        # Own state in NED (updated by agent each tick)
        self._my_pos_ned = np.zeros(3, dtype=np.float64)
        self._my_vel_ned = np.zeros(3, dtype=np.float64)

        # GPS-based peer tracking for collision avoidance (ALL modes)
        self._own_lat: float = 0.0
        self._own_lon: float = 0.0
        self._own_alt: float = 0.0
        self._peer_gps: list[tuple[float, float]] = []

        # Configurable isolation radius (updated via SAFETY_CONFIG_CMD)
        self.isolation_radius: float = ISOLATION_RADIUS_M
        self.soft_zone: float = ISOLATION_SOFT_ZONE_M

        # PROXIMITY_ALERT cooperative deceleration
        # Maps alerting_drone_id -> {"expiry": float, "severity": "hard"|"soft",
        #                            "lat": float, "lon": float}
        self._active_alerts: dict[int, dict] = {}

        # RL controller (lazy-initialized on first enable)
        self._rl_controller = None  # RLController | None
        self._rl_goal_ned = np.zeros(3, dtype=np.float64)
        self._rl_enabled = False

    def set_isolation_radius(self, radius_m: float):
        """Update isolation radius (from SAFETY_CONFIG_CMD).
        Only affects _collision_overlay (non-formation modes).
        Formation controller uses fixed internal safety zones."""
        self.isolation_radius = max(1.0, radius_m)
        self.soft_zone = self.isolation_radius * 1.6
        log.info("Isolation radius set to %.1fm (soft zone %.1fm)",
                 self.isolation_radius, self.soft_zone)

    def receive_proximity_alert(self, alert_data: dict, my_drone_id: int):
        """Process incoming PROXIMITY_ALERT from a peer.
        Determines if we're the threat or just nearby, and sets deceleration."""
        alerting = alert_data.get("alerting_drone")
        close_peers = alert_data.get("close_peers", [])
        severity = alert_data.get("severity", "WARNING")
        now = time.time()

        if my_drone_id in close_peers:
            # I'm the threat — hard deceleration
            self._active_alerts[alerting] = {
                "expiry": now + ALERT_HARD_TTL,
                "severity": "hard",
                "lat": alert_data.get("lat", 0.0),
                "lon": alert_data.get("lon", 0.0),
            }
            log.warning("Drone %d: PROXIMITY ALERT from D%d — I am threat, decelerating",
                        my_drone_id, alerting)
        else:
            # I'm a bystander — soft deceleration
            self._active_alerts[alerting] = {
                "expiry": now + ALERT_SOFT_TTL,
                "severity": "soft",
                "lat": alert_data.get("lat", 0.0),
                "lon": alert_data.get("lon", 0.0),
            }

    def _get_decel_factor(self) -> float:
        """Return velocity scaling factor based on active alerts.
        1.0 = no deceleration, lower = more deceleration."""
        now = time.time()
        factor = 1.0
        expired = []

        for drone_id, alert in self._active_alerts.items():
            if now > alert["expiry"]:
                expired.append(drone_id)
                continue
            if alert["severity"] == "hard":
                factor = min(factor, HARD_DECEL_FACTOR)
            else:
                factor = min(factor, SOFT_DECEL_FACTOR)

        for d in expired:
            del self._active_alerts[d]

        return factor

    def _alert_repulsion(self) -> tuple[float, float]:
        """Compute repulsion velocity away from alerting drones (when I'm threat)."""
        if not self._active_alerts or (self._own_lat == 0.0 and self._own_lon == 0.0):
            return 0.0, 0.0

        now = time.time()
        repel_n, repel_e = 0.0, 0.0
        cos_lat = cos(radians(self._own_lat))

        for drone_id, alert in self._active_alerts.items():
            if now > alert["expiry"] or alert["severity"] != "hard":
                continue
            alat, alon = alert["lat"], alert["lon"]
            if alat == 0.0 and alon == 0.0:
                continue
            # Vector FROM alerting drone TO self (escape direction)
            dn = (self._own_lat - alat) * 111320.0
            de = (self._own_lon - alon) * 111320.0 * cos_lat
            dist = (dn * dn + de * de) ** 0.5
            if dist < 0.1:
                dn, de, dist = 1.0, 0.0, 1.0
            # Push away at 1.5 m/s
            repel_n += 1.5 * (dn / dist)
            repel_e += 1.5 * (de / dist)

        return repel_n, repel_e

    def enable_rl_mode(self):
        """Activate RL controller. Lazy-initializes on first call."""
        if self._rl_controller is None:
            from drone_agent.rl_controller import RLController
            self._rl_controller = RLController()
        self._rl_enabled = True
        if self.task not in (TAKEOFF, LAND):
            self.task = RL_MODE
        log.info("Drone %d: RL mode ENABLED (onnx=%s)",
                 self.conn.drone_id, self._rl_controller.is_onnx)

    def disable_rl_mode(self):
        """Deactivate RL controller, return to IDLE."""
        self._rl_enabled = False
        if self.task == RL_MODE:
            self.task = IDLE
        log.info("Drone %d: RL mode DISABLED", self.conn.drone_id)

    def set_rl_goal(self, goal_ned):
        """Set the goal position for RL controller (NED meters)."""
        self._rl_goal_ned = np.asarray(goal_ned, dtype=np.float64)

    def update_peer_gps(self, own_lat: float, own_lon: float, own_alt: float,
                        peer_gps: list[tuple[float, float]]):
        """Update own GPS + peer GPS list for collision avoidance in all modes.
        Called by agent every tick, regardless of formation mode."""
        self._own_lat = own_lat
        self._own_lon = own_lon
        self._own_alt = own_alt
        self._peer_gps = peer_gps

    def _collision_overlay(self, vn: float, ve: float, vd: float
                           ) -> tuple[float, float, float]:
        """Two-zone repulsion + velocity cancellation from nearby peers.

        Soft zone (iso_r to soft_z): Quadratic ramp — gentle push during approach.
        Hard zone (below iso_r): Exponential barrier — prevents boundary crossing.

        For each peer inside the soft zone:
          1. Compute escape direction (unit vector FROM peer TO self)
          2. Check if current velocity has a closing component (toward peer)
          3. Cancel the closing component
          4. Add repulsion (quadratic soft / exponential hard)
          5. Inside isolation radius: repulsion overrides task velocity

        Returns adjusted (vn, ve, vd).
        """
        if not self._peer_gps or (self._own_lat == 0.0 and self._own_lon == 0.0):
            return vn, ve, vd

        iso_r = self.isolation_radius
        soft_z = self.soft_zone
        cos_lat = cos(radians(self._own_lat))

        repel_n, repel_e = 0.0, 0.0
        cancel_n, cancel_e = 0.0, 0.0
        any_inside_isolation = False

        for plat, plon in self._peer_gps:
            # Vector FROM peer TO self (escape direction)
            dn = (self._own_lat - plat) * 111320.0
            de = (self._own_lon - plon) * 111320.0 * cos_lat
            dist = (dn * dn + de * de) ** 0.5

            if dist >= soft_z or dist < 0.01:
                continue

            # Unit escape vector
            un, ue = dn / dist, de / dist

            # 1. Check closing velocity: project velocity onto peer direction
            #    Negative dot = moving TOWARD peer
            closing_speed = -(vn * un + ve * ue)

            if closing_speed > 0:
                # We're moving toward this peer — cancel the closing component
                cancel_n += closing_speed * un
                cancel_e += closing_speed * ue

            # 2. Two-zone repulsion
            if dist < iso_r:
                # Hard zone: exponential barrier
                strength = REPEL_GAIN * math.exp(iso_r / max(dist, 0.3) - 1.0)
                any_inside_isolation = True
            else:
                # Soft zone: quadratic ramp (0 at soft_z, REPEL_GAIN at iso_r)
                t = (soft_z - dist) / (soft_z - iso_r)
                strength = REPEL_GAIN * t * t

            repel_n += strength * un
            repel_e += strength * ue

        # Apply velocity cancellation (remove closing components)
        vn += cancel_n
        ve += cancel_e

        if any_inside_isolation:
            # Inside hard boundary — repulsion overrides task velocity
            vn = repel_n
            ve = repel_e
        else:
            # Outside hard boundary — add repulsion to task velocity
            vn += repel_n
            ve += repel_e

        # Apply alert deceleration (cooperative protocol)
        decel = self._get_decel_factor()
        if decel < 1.0:
            vn *= decel
            ve *= decel
            # Add alert repulsion (push away from alerting drone)
            ar_n, ar_e = self._alert_repulsion()
            vn += ar_n
            ve += ar_e

        # Clamp horizontal speed
        h_speed = (vn * vn + ve * ve) ** 0.5
        if h_speed > MAX_SPEED:
            vn *= MAX_SPEED / h_speed
            ve *= MAX_SPEED / h_speed

        return vn, ve, vd

    def do_takeoff(self, alt: float):
        """Initiate takeoff sequence (non-blocking). Handled in tick()."""
        log.info("Drone %d: takeoff requested to %.1f m", self.conn.drone_id, alt)
        self._takeoff_alt = alt
        self._takeoff_sent = False
        self._arm_sent_time = 0.0
        self.task = TAKEOFF

    def do_land(self):
        """Switch to LAND mode."""
        log.info("Drone %d: landing", self.conn.drone_id)
        self.conn.land()
        self.task = LAND

    def set_waypoint(self, lat: float, lon: float, alt: float):
        """Set a GPS waypoint target. Drone flies there in GUIDED mode."""
        self.wp_lat = lat
        self.wp_lon = lon
        self.wp_alt = alt
        self.task = WAYPOINT

    def set_velocity(self, vn: float, ve: float, vd: float):
        """Set a NED velocity setpoint."""
        self.vel_n = vn
        self.vel_e = ve
        self.vel_d = vd
        self.task = VELOCITY

    def hold(self):
        """Stop all motion — send zero velocity."""
        self.vel_n = 0.0
        self.vel_e = 0.0
        self.vel_d = 0.0
        self.task = HOLD

    def update_own_state_ned(self, lat, lon, alt, vx, vy, vz,
                             ref_lat, ref_lon, ref_alt):
        """Update own NED state from GPS telemetry. Called by agent each tick."""
        self._my_pos_ned = gps_to_ned(lat, lon, alt, ref_lat, ref_lon, ref_alt)
        self._my_vel_ned = np.array([vx, vy, vz], dtype=np.float64)

    def set_formation_target(self, goal_ned, path_waypoints_ned,
                             peer_positions_ned, ref_lat, ref_lon, ref_alt,
                             peer_velocities_ned=None):
        """
        Set formation tracking parameters. Called by agent every tick
        when in formation mode.

        Args:
            goal_ned: (3,) formation slot target in NED
            path_waypoints_ned: list of (n, e) tuples from Hybrid A*
            peer_positions_ned: (P, 3) peer positions in NED
            ref_lat, ref_lon, ref_alt: formation reference for conversions
            peer_velocities_ned: optional (P, 3) peer velocities in NED for prediction
        """
        self._goal_ned = np.asarray(goal_ned, dtype=np.float64)

        # Convert 2D path waypoints to 3D NED
        if path_waypoints_ned and len(path_waypoints_ned) > 0:
            self._path_waypoints_ned = np.array(
                [[wp[0], wp[1], 0.0] for wp in path_waypoints_ned],
                dtype=np.float64
            )
        else:
            self._path_waypoints_ned = np.empty((0, 3), dtype=np.float64)

        if peer_positions_ned is not None and len(peer_positions_ned) > 0:
            self._peer_positions_ned = np.asarray(peer_positions_ned, dtype=np.float64)
        else:
            self._peer_positions_ned = np.empty((0, 3), dtype=np.float64)

        if peer_velocities_ned is not None and len(peer_velocities_ned) > 0:
            self._peer_velocities_ned = np.asarray(peer_velocities_ned, dtype=np.float64)
        else:
            self._peer_velocities_ned = np.empty((0, 3), dtype=np.float64)

        # Switch to FORMATION state if not in a priority state.
        # HOLD is NOT excluded: after proximity clears, formation must resume.
        # (During EMERGENCY_PROXIMITY, failsafe blocks the planner entirely.)
        if self.task not in (TAKEOFF, LAND):
            self.task = FORMATION

    def tick(self, armed: bool = False, mode: str = "", has_gps: bool = False,
             alt: float = 0.0):
        """
        Called every loop iteration. Handles state machine for takeoff
        and resends active commands. Collision avoidance is applied in
        WAYPOINT, VELOCITY, HOLD, and IDLE modes via _collision_overlay().
        Formation mode has its own exponential potential field.
        """
        if self.task == TAKEOFF:
            self._tick_takeoff(armed, mode, has_gps, alt)

        elif self.task == FORMATION:
            # Formation controller has its own exponential potential field
            self._tick_formation()

        elif self.task == WAYPOINT:
            self._tick_waypoint()

        elif self.task == VELOCITY:
            vn, ve, vd = self._collision_overlay(self.vel_n, self.vel_e, self.vel_d)
            self.conn.send_velocity_ned(vn, ve, vd)

        elif self.task == RL_MODE:
            self._tick_rl()

        elif self.task == HOLD:
            vn, ve, vd = self._collision_overlay(0.0, 0.0, 0.0)
            self.conn.send_velocity_ned(vn, ve, vd)

        elif self.task == IDLE:
            # Even in idle, actively avoid collisions
            vn, ve, vd = self._collision_overlay(0.0, 0.0, 0.0)
            if abs(vn) > 0.05 or abs(ve) > 0.05:
                self.conn.send_velocity_ned(vn, ve, vd)

        # LAND: no resend needed (ArduPilot handles descent)

    def _tick_waypoint(self):
        """Velocity-based waypoint tracking with collision avoidance.
        Converts GPS waypoint to velocity command so repulsion can be added."""
        if self._own_lat == 0.0 and self._own_lon == 0.0:
            # No GPS yet — fall back to position command
            self.conn.send_goto_global(self.wp_lat, self.wp_lon, self.wp_alt)
            return

        # PD toward waypoint in NED
        cos_lat = cos(radians(self._own_lat))
        dn = (self.wp_lat - self._own_lat) * 111320.0
        de = (self.wp_lon - self._own_lon) * 111320.0 * cos_lat
        dalt = self.wp_alt - self._own_alt
        dist = (dn * dn + de * de) ** 0.5

        if dist < 0.5:
            # Arrived — just hold position with collision avoidance
            vn, ve = 0.0, 0.0
        else:
            speed = min(KP_WP * dist, MAX_SPEED)
            vn = speed * dn / dist
            ve = speed * de / dist

        # NED: negative vd = go up. If dalt > 0, we want to go up.
        vd = -min(max(KP_WP * dalt, -MAX_VERT_WP), MAX_VERT_WP)

        vn, ve, vd = self._collision_overlay(vn, ve, vd)
        self.conn.send_velocity_ned(vn, ve, vd)

    def _tick_formation(self):
        """PD + exponential potential field formation tracking."""
        vel_cmd = self._formation_ctrl.compute(
            self._my_pos_ned,
            self._my_vel_ned,
            self._goal_ned,
            self._peer_positions_ned,
        )

        # Apply alert deceleration if cooperative alerts are active
        decel = self._get_decel_factor()
        if decel < 1.0:
            vel_cmd[:2] *= decel
            ar_n, ar_e = self._alert_repulsion()
            vel_cmd[0] += ar_n
            vel_cmd[1] += ar_e
            h_speed = np.linalg.norm(vel_cmd[:2])
            if h_speed > MAX_SPEED:
                vel_cmd[:2] *= MAX_SPEED / h_speed

        self.conn.send_velocity_ned(float(vel_cmd[0]),
                                    float(vel_cmd[1]),
                                    float(vel_cmd[2]))

    def _tick_rl(self):
        """RL mode: build observation, run inference, apply collision overlay."""
        if self._rl_controller is None:
            return

        obs = self._rl_controller.build_observation(
            self._my_pos_ned,
            self._my_vel_ned,
            self._rl_goal_ned,
            self._peer_positions_ned,
        )

        vel = self._rl_controller.infer(obs)
        vn, ve, vd = float(vel[0]), float(vel[1]), float(vel[2])
        vn, ve, vd = self._collision_overlay(vn, ve, vd)
        self.conn.send_velocity_ned(vn, ve, vd)

    def _tick_takeoff(self, armed: bool, mode: str, has_gps: bool,
                      alt: float = 0.0):
        """Non-blocking takeoff state machine."""
        now = time.time()

        # Step 0: Wait for GPS fix before doing anything
        if not has_gps:
            if now - self._arm_sent_time > 5.0:
                log.info("Drone %d: waiting for GPS fix before takeoff...",
                         self.conn.drone_id)
                self._arm_sent_time = now
            return

        # Step 1: Set GUIDED mode (resend every 2s until confirmed)
        if "GUIDED" not in mode:
            if now - self._arm_sent_time > 2.0:
                self.conn.set_mode("GUIDED")
                self._arm_sent_time = now
            return

        # Step 2: Arm (resend every 2s until confirmed)
        if not armed:
            if now - self._arm_sent_time > 2.0:
                self.conn.arm()
                self._arm_sent_time = now
            return

        # Step 3: Send takeoff command (resend every 3s until airborne)
        if not self._takeoff_sent or (now - self._arm_sent_time > 3.0):
            self.conn.takeoff(self._takeoff_alt)
            self._takeoff_sent = True
            self._arm_sent_time = now
            log.info("Drone %d: takeoff command sent (%.1fm)",
                     self.conn.drone_id, self._takeoff_alt)

        # Step 4: Detect takeoff complete — at 80% of target altitude
        if self._takeoff_sent and alt >= self._takeoff_alt * 0.8:
            log.info("Drone %d: takeoff complete (alt=%.1fm, target=%.1fm)",
                     self.conn.drone_id, alt, self._takeoff_alt)
            self.task = IDLE
