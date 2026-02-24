"""
Failsafe manager — autonomous state machine for failure handling.

Each drone independently runs a state machine that detects peer dropout,
compacts formations, elects a new leader, and recovers from comms loss.

States:
  NOMINAL            — All peers present, no issues
  DEGRADED           — One or more peers lost, formation compacted
  COMMS_LOST         — No GCS contact → RTL
  COMMS_RECOVERY     — GCS restored, switching to GUIDED to rejoin
  EMERGENCY_PROXIMITY — Too close to peer → HOLD
  EMERGENCY_BATTERY  — Low battery → LAND
  LANDED             — Terminal state after LAND

Priority: EMERGENCY_PROXIMITY > COMMS_LOST > EMERGENCY_BATTERY > peer staleness
"""

import time
import logging
from enum import Enum
from math import radians, sin, cos, sqrt, atan2

from config import (
    SAFE_DISTANCE_M, SAFE_DISTANCE_CLEAR_M, COMMS_TIMEOUT_S,
    LOW_BATTERY_PCT, PEER_STALE_TIMEOUT_S, COMMS_RECOVERY_TIMEOUT_S,
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

        # Legacy flags for proximity hysteresis
        self._proximity_triggered: bool = False

    def update_gcs_heartbeat(self):
        """Call whenever any UDP message arrives from GCS."""
        self.last_gcs_time = time.time()
        self._gcs_ever_contacted = True

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
        """Scan PeerTable staleness, update alive set, emit alerts on changes."""
        # Build alive set: self + non-stale peers that are in expected set
        new_alive = {self.drone_id}
        for pid in self._expected_peers:
            if pid == self.drone_id:
                continue
            if pid in peers.last_update and not peers.is_stale(pid, PEER_STALE_TIMEOUT_S):
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
        """Check distance to all peers. Returns True if too close."""
        for peer_id, peer in peers.peers.items():
            if peer_id == self.drone_id:
                continue
            if peer.lat == 0.0 and peer.lon == 0.0:
                continue
            dist = _haversine_m(my_state.lat, my_state.lon, peer.lat, peer.lon)
            if dist < SAFE_DISTANCE_M and not self._proximity_triggered:
                self._proximity_triggered = True
                self.planner.hold()
                self._alert("PROXIMITY",
                            f"Too close to drone {peer_id} ({dist:.1f}m < {SAFE_DISTANCE_M}m)",
                            "HOLD")
                log.warning("Drone %d: PROXIMITY — %.1fm from drone %d",
                            self.drone_id, dist, peer_id)
                return True
            elif dist > SAFE_DISTANCE_CLEAR_M and self._proximity_triggered:
                self._proximity_triggered = False
                log.info("Drone %d: proximity clear", self.drone_id)
        return self._proximity_triggered

    def _check_comms_lost(self) -> bool:
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

        if self._check_comms_lost():
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

        if self._check_comms_lost():
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
        if self._check_comms_lost():
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
        """EMERGENCY_PROXIMITY: clears when distance > SAFE_DISTANCE_CLEAR_M."""
        # Re-check proximity — _check_proximity handles hysteresis
        still_close = self._check_proximity(peers, my_state)
        if not still_close:
            # Restore to previous state
            if self._alive_peers == self._expected_peers:
                self._transition(SwarmState.NOMINAL)
            else:
                self._transition(SwarmState.DEGRADED)

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
