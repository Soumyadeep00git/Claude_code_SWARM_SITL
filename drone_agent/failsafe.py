"""
Failsafe manager — autonomous state machine for failure handling.

Each drone independently runs a state machine that detects peer dropout,
compacts formations, elects a new leader, and recovers from comms loss.

States:
  NOMINAL            — All peers present, no issues
  DEGRADED           — One or more peers lost, formation compacted
  COMMS_LOST         — No GCS contact → RTL
  COMMS_RECOVERY     — GCS restored, switching to GUIDED to rejoin
  EMERGENCY_PROXIMITY — Too close to peer → active repulsion + PROXIMITY_ALERT
  EMERGENCY_BATTERY  — Low battery → LAND
  LANDED             — Terminal state after LAND

Priority: EMERGENCY_PROXIMITY > COMMS_LOST > EMERGENCY_BATTERY > peer staleness
"""

import time
import math
import logging
from enum import Enum
from math import radians, sin, cos, sqrt, atan2

from config import (
    COMMS_TIMEOUT_S, LOW_BATTERY_PCT,
    PEER_STALE_TIMEOUT_S, COMMS_RECOVERY_TIMEOUT_S,
    ISOLATION_RADIUS_M,
)
from mavlink_layer.drone_connection import DroneConnection
from drone_agent.local_planner import LocalPlanner
from drone_agent.state import DroneState, PeerTable
from drone_agent.global_planner import GlobalPlanner

log = logging.getLogger(__name__)


class SwarmState(Enum):
    NOMINAL = "NOMINAL"
    DEGRADED = "DEGRADED"
    COMMS_LOST = "COMMS_LOST"
    COMMS_RECOVERY = "COMMS_RECOVERY"
    EMERGENCY_PROXIMITY = "EMERGENCY_PROXIMITY"
    EMERGENCY_BATTERY = "EMERGENCY_BATTERY"
    LANDED = "LANDED"


class FailsafeManager:
    """Autonomous state machine for drone failure handling."""

    def __init__(self, drone_id: int, local_planner: LocalPlanner,
                 connection: DroneConnection, global_planner: GlobalPlanner,
                 expected_peers: set[int]):
        self.drone_id = drone_id
        self.planner = local_planner
        self.conn = connection
        self.global_planner = global_planner
        self._expected_peers = expected_peers  # All drone IDs in the swarm

        self.override_active: bool = False
        self.last_gcs_time: float = 0.0  # 0 = no GCS contact yet
        self._gcs_ever_contacted: bool = False
        self._alerts: list[dict] = []

        # State machine
        self.state: SwarmState = SwarmState.NOMINAL
        self._previous_state: SwarmState = SwarmState.NOMINAL
        self._alive_peers: set[int] = set(expected_peers)  # Start assuming all alive
        self._recovery_start: float = 0.0

        # Per-peer heartbeat tracking (P2P mesh)
        self._peer_last_contact: dict[int, float] = {}

        # Per-peer proximity tracking (prevents single-boolean oscillation)
        self._proximity_peers: set[int] = set()
        self._prox_log_times: dict[int, float] = {}  # Rate-limit logs per peer

        # Configurable isolation radius (updated via SAFETY_CONFIG_CMD)
        self.isolation_radius: float = ISOLATION_RADIUS_M

    @property
    def clear_distance(self) -> float:
        """Hysteresis clear threshold: 1.5× isolation radius."""
        return self.isolation_radius * 1.5

    def set_isolation_radius(self, radius_m: float):
        """Update isolation radius from SAFETY_CONFIG_CMD."""
        self.isolation_radius = max(1.0, radius_m)
        log.info("Drone %d: failsafe isolation radius set to %.1fm",
                 self.drone_id, self.isolation_radius)

    def update_gcs_heartbeat(self):
        """Call whenever any UDP message arrives from GCS."""
        self.last_gcs_time = time.time()
        self._gcs_ever_contacted = True

    def update_peer_heartbeat(self, peer_id: int):
        """Call whenever a direct or relayed message arrives from a peer."""
        self._peer_last_contact[peer_id] = time.time()

    def elect_leader(self) -> int:
        """Leader = min(alive_ids). Deterministic, no election messages needed."""
        if self._alive_peers:
            return min(self._alive_peers)
        return self.drone_id  # Fallback: we are the only one

    # ── State transitions ─────────────────────────────────

    def _transition(self, new_state: SwarmState):
        """Log state change, emit alert, trigger reslot on DEGRADED entry."""
        if new_state == self.state:
            return
        old = self.state
        self._previous_state = old
        self.state = new_state
        log.info("Drone %d: %s -> %s", self.drone_id, old.value, new_state.value)
        self._alert("STATE_CHANGE",
                     f"{old.value} -> {new_state.value}",
                     new_state.value)

        # Trigger reslot when entering DEGRADED or alive set changed while DEGRADED
        if new_state == SwarmState.DEGRADED:
            self._do_reslot()

    def _do_reslot(self):
        """Compact formation slots based on alive peers."""
        leader_id = self.elect_leader()
        self.global_planner.reslot(self._alive_peers, leader_id,
                                   peers=None)  # peers passed in tick
        self._alert("RESLOT",
                     f"Reslotted: alive={sorted(self._alive_peers)}, leader={leader_id}",
                     "RESLOT")

    # ── Peer health tracking ──────────────────────────────

    def _update_peer_health(self, peers: PeerTable):
        """Scan PeerTable staleness and direct heartbeats, update alive set."""
        # Build alive set: self + peers alive via EITHER data source
        new_alive = {self.drone_id}
        now = time.time()
        for pid in self._expected_peers:
            if pid == self.drone_id:
                continue
            # PeerTable path (GCS relay or P2P updates PeerTable)
            peer_table_alive = (pid in peers.last_update and
                                not peers.is_stale(pid, PEER_STALE_TIMEOUT_S))
            # Direct heartbeat path (P2P mesh)
            direct_alive = (pid in self._peer_last_contact and
                            (now - self._peer_last_contact[pid]) <= PEER_STALE_TIMEOUT_S)
            if peer_table_alive or direct_alive:
                new_alive.add(pid)

        # Detect changes
        lost = self._alive_peers - new_alive
        returned = new_alive - self._alive_peers

        for pid in lost:
            log.warning("Drone %d: peer %d lost (stale > %.1fs)",
                        self.drone_id, pid, PEER_STALE_TIMEOUT_S)
            self._alert("PEER_LOST", f"Peer {pid} lost", "DEGRADED")

        for pid in returned:
            log.info("Drone %d: peer %d returned", self.drone_id, pid)
            self._alert("PEER_RETURNED", f"Peer {pid} returned", "NOMINAL")

        self._alive_peers = new_alive
        return lost, returned

    # ── Check helpers ─────────────────────────────────────

    def _check_proximity(self, peers: PeerTable, my_state: DroneState) -> bool:
        """Check 3D distance to all peers using configurable isolation radius.
        Returns True if ANY peer too close.
        Uses per-peer tracking to prevent oscillation from mixed far/close peers."""
        now = time.time()
        iso_r = self.isolation_radius
        clear_d = self.clear_distance

        for peer_id, peer in peers.peers.items():
            if peer_id == self.drone_id:
                continue
            if peer.lat == 0.0 and peer.lon == 0.0:
                continue
            dist_2d = _haversine_m(my_state.lat, my_state.lon, peer.lat, peer.lon)
            dalt = abs(my_state.alt - peer.alt) if peer.alt > 0 else 0.0
            dist_3d = (dist_2d ** 2 + dalt ** 2) ** 0.5

            if dist_3d < iso_r:
                if peer_id not in self._proximity_peers:
                    # Newly too close — trigger hold and alert
                    self._proximity_peers.add(peer_id)
                    self.planner.hold()
                    severity = "CRITICAL" if dist_3d < iso_r * 0.5 else "WARNING"
                    self._alert("PROXIMITY",
                                f"Too close to drone {peer_id} ({dist_3d:.1f}m 3D < {iso_r:.1f}m) [{severity}]",
                                "HOLD")
                    log.warning("Drone %d: PROXIMITY — %.1fm (3D) from drone %d [%s]",
                                self.drone_id, dist_3d, peer_id, severity)
                    self._prox_log_times[peer_id] = now
                elif now - self._prox_log_times.get(peer_id, 0) > 2.0:
                    # Rate-limited ongoing proximity log (every 2s per peer)
                    log.warning("Drone %d: still close to drone %d (%.1fm 3D)",
                                self.drone_id, peer_id, dist_3d)
                    self._prox_log_times[peer_id] = now

            elif dist_3d > clear_d and peer_id in self._proximity_peers:
                # Peer cleared — remove from tracked set
                self._proximity_peers.discard(peer_id)
                self._prox_log_times.pop(peer_id, None)
                log.info("Drone %d: proximity clear from drone %d (%.1fm 3D)",
                         self.drone_id, peer_id, dist_3d)

        return len(self._proximity_peers) > 0

    def _check_gcs_lost(self) -> bool:
        """Returns True if GCS contact is lost."""
        if not self._gcs_ever_contacted:
            return False
        return (time.time() - self.last_gcs_time) > COMMS_TIMEOUT_S

    def _check_battery(self, battery_pct: int) -> bool:
        """Returns True if battery is critically low."""
        return 0 <= battery_pct < LOW_BATTERY_PCT

    # ── Per-state tick handlers ───────────────────────────

    def _tick_nominal(self, peers: PeerTable, my_state: DroneState):
        """NOMINAL: check proximity → comms → battery → peer staleness."""
        if self._check_proximity(peers, my_state):
            self._transition(SwarmState.EMERGENCY_PROXIMITY)
            return

        if self._check_gcs_lost():
            self.conn.rtl()
            self._alert("COMMS_LOST",
                        f"No GCS contact for {time.time() - self.last_gcs_time:.1f}s",
                        "RTL")
            self._transition(SwarmState.COMMS_LOST)
            return

        if self._check_battery(my_state.battery_pct):
            self.planner.do_land()
            self._alert("LOW_BATTERY", f"Battery at {my_state.battery_pct}%", "LAND")
            self._transition(SwarmState.EMERGENCY_BATTERY)
            return

        # Peer staleness check
        lost, _ = self._update_peer_health(peers)
        if self._alive_peers != self._expected_peers:
            self._transition(SwarmState.DEGRADED)

    def _tick_degraded(self, peers: PeerTable, my_state: DroneState):
        """DEGRADED: same checks + watch for all peers returning → NOMINAL."""
        if self._check_proximity(peers, my_state):
            self._transition(SwarmState.EMERGENCY_PROXIMITY)
            return

        if self._check_gcs_lost():
            self.conn.rtl()
            self._alert("COMMS_LOST",
                        f"No GCS contact for {time.time() - self.last_gcs_time:.1f}s",
                        "RTL")
            self._transition(SwarmState.COMMS_LOST)
            return

        if self._check_battery(my_state.battery_pct):
            self.planner.do_land()
            self._alert("LOW_BATTERY", f"Battery at {my_state.battery_pct}%", "LAND")
            self._transition(SwarmState.EMERGENCY_BATTERY)
            return

        # Update peer health and check if all returned
        old_alive = set(self._alive_peers)
        self._update_peer_health(peers)

        if self._alive_peers == self._expected_peers:
            # All peers back — restore to NOMINAL
            self._transition(SwarmState.NOMINAL)
            # Reslot back to full formation
            leader_id = self.elect_leader()
            self.global_planner.reslot(self._alive_peers, leader_id)
        elif self._alive_peers != old_alive:
            # Alive set changed but not full — reslot
            self._do_reslot()

    def _tick_comms_lost(self):
        """COMMS_LOST: watch for GCS heartbeat restore → COMMS_RECOVERY."""
        if self._gcs_ever_contacted and (time.time() - self.last_gcs_time) <= COMMS_TIMEOUT_S:
            # GCS contact restored
            log.info("Drone %d: GCS restored, entering COMMS_RECOVERY", self.drone_id)
            self._recovery_start = time.time()
            self.conn.set_mode("GUIDED")
            self._transition(SwarmState.COMMS_RECOVERY)

    def _tick_comms_recovery(self, my_state: DroneState):
        """COMMS_RECOVERY: wait for GUIDED mode confirmed, then rejoin."""
        # Check if comms lost again
        if self._check_gcs_lost():
            self.conn.rtl()
            self._alert("COMMS_LOST", "Comms lost again during recovery", "RTL")
            self._transition(SwarmState.COMMS_LOST)
            return

        # Check timeout
        if time.time() - self._recovery_start > COMMS_RECOVERY_TIMEOUT_S:
            log.warning("Drone %d: COMMS_RECOVERY timed out", self.drone_id)
            self.conn.rtl()
            self._alert("COMMS_LOST", "Recovery timeout", "RTL")
            self._transition(SwarmState.COMMS_LOST)
            return

        # Wait for GUIDED mode confirmation
        if "GUIDED" in my_state.mode:
            # Rejoin — pick the right state based on current alive set
            if self._alive_peers == self._expected_peers:
                self._transition(SwarmState.NOMINAL)
            else:
                self._transition(SwarmState.DEGRADED)
        else:
            # Resend GUIDED mode request periodically
            self.conn.set_mode("GUIDED")

    def _tick_emergency_proximity(self, peers: PeerTable, my_state: DroneState):
        """EMERGENCY_PROXIMITY: actively push away from close peers.
        Clears when all peers > clear_distance."""
        # Re-check proximity — _check_proximity handles hysteresis
        still_close = self._check_proximity(peers, my_state)

        if still_close:
            # Actively push away from close peers
            self._repel_from_close_peers(peers, my_state)
            return

        if not still_close:
            # Restore to previous state
            if self._alive_peers == self._expected_peers:
                self._transition(SwarmState.NOMINAL)
            else:
                self._transition(SwarmState.DEGRADED)

    def _repel_from_close_peers(self, peers: PeerTable, my_state: DroneState):
        """Compute and send exponential repulsion velocity away from close peers."""
        repel_n, repel_e = 0.0, 0.0
        iso_r = self.isolation_radius

        for peer_id in self._proximity_peers:
            peer = peers.peers.get(peer_id)
            if peer is None or (peer.lat == 0.0 and peer.lon == 0.0):
                continue
            # Vector from peer to self (repulsion direction)
            dn = (my_state.lat - peer.lat) * 111320.0
            de = (my_state.lon - peer.lon) * (111320.0 * cos(radians(my_state.lat)))
            dist = (dn ** 2 + de ** 2) ** 0.5
            if dist < 0.1:
                # Nearly on top — push north by default
                dn, de, dist = 1.0, 0.0, 1.0

            # Exponential repulsion — strong push
            strength = min(3.0, 2.0 * math.exp(iso_r / max(dist, 0.3) - 1.0))
            repel_n += strength * (dn / dist)
            repel_e += strength * (de / dist)

        # Cap total repulsion speed at 3.0 m/s
        mag = (repel_n ** 2 + repel_e ** 2) ** 0.5
        if mag > 3.0:
            repel_n *= 3.0 / mag
            repel_e *= 3.0 / mag

        self.conn.send_velocity_ned(repel_n, repel_e, 0.0)

    def get_proximity_alert_data(self, my_state: DroneState) -> dict | None:
        """Build PROXIMITY_ALERT payload if any peers are inside isolation zone.
        Called by agent to broadcast alerts."""
        if not self._proximity_peers:
            return None
        return {
            "alerting_drone": self.drone_id,
            "close_peers": list(self._proximity_peers),
            "lat": my_state.lat,
            "lon": my_state.lon,
            "isolation_radius": self.isolation_radius,
            "severity": "CRITICAL" if any(
                _peer_dist_2d(my_state, peers_obj)
                < self.isolation_radius * 0.5
                for peers_obj in []  # Placeholder — checked below
            ) else "WARNING",
        }

    def build_proximity_alert(self, peers: PeerTable, my_state: DroneState) -> dict | None:
        """Build PROXIMITY_ALERT if peers are inside isolation zone."""
        if not self._proximity_peers:
            return None

        # Determine severity
        severity = "WARNING"
        for peer_id in self._proximity_peers:
            peer = peers.peers.get(peer_id)
            if peer is None:
                continue
            dist = _haversine_m(my_state.lat, my_state.lon, peer.lat, peer.lon)
            if dist < self.isolation_radius * 0.5:
                severity = "CRITICAL"
                break

        return {
            "alerting_drone": self.drone_id,
            "close_peers": list(self._proximity_peers),
            "lat": my_state.lat,
            "lon": my_state.lon,
            "isolation_radius": self.isolation_radius,
            "severity": severity,
        }

    # ── Main tick ──────────────────────────────────────────

    def tick(self, peers: PeerTable, my_state: DroneState) -> bool:
        """
        Run state machine. Returns True if planner should be blocked.
        Planner runs in NOMINAL and DEGRADED only.
        Failsafes disabled until valid GPS fix and armed.
        """
        # Don't run failsafes until we have valid GPS and are armed
        if my_state.lat == 0.0 and my_state.lon == 0.0:
            self.override_active = False
            return False
        if not my_state.armed:
            self.override_active = False
            return False

        # Dispatch to current state handler
        if self.state == SwarmState.NOMINAL:
            self._tick_nominal(peers, my_state)
        elif self.state == SwarmState.DEGRADED:
            self._tick_degraded(peers, my_state)
        elif self.state == SwarmState.COMMS_LOST:
            self._tick_comms_lost()
        elif self.state == SwarmState.COMMS_RECOVERY:
            self._tick_comms_recovery(my_state)
        elif self.state == SwarmState.EMERGENCY_PROXIMITY:
            self._tick_emergency_proximity(peers, my_state)
        elif self.state == SwarmState.EMERGENCY_BATTERY:
            pass  # Terminal-ish: LAND in progress
        elif self.state == SwarmState.LANDED:
            pass  # Terminal

        # Planner runs in NOMINAL and DEGRADED only
        self.override_active = self.state not in (SwarmState.NOMINAL, SwarmState.DEGRADED)
        return self.override_active

    # ── Alerts ─────────────────────────────────────────────

    def _alert(self, code: str, message: str, action: str):
        self._alerts.append({
            "level": "WARN",
            "code": code,
            "message": message,
            "action_taken": action,
        })

    def pop_alerts(self) -> list[dict]:
        """Drain and return pending alerts."""
        alerts = self._alerts
        self._alerts = []
        return alerts


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Distance in meters between two GPS coordinates."""
    R = 6371000.0  # Earth radius in meters
    rlat1, rlat2 = radians(lat1), radians(lat2)
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat / 2) ** 2 + cos(rlat1) * cos(rlat2) * sin(dlon / 2) ** 2
    return R * 2 * atan2(sqrt(a), sqrt(1 - a))
