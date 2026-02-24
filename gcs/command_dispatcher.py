"""
Command dispatcher — builds and sends commands to drone agents.
"""

import logging

from config import GCS_HOST, AGENT_BASE_PORT, AGENT_PORT_STEP
from comms.protocol import make_msg
from comms.udp_node import UDPNode

log = logging.getLogger(__name__)


class CommandDispatcher:
    """Sends commands to drone agents over UDP."""

    def __init__(self, udp: UDPNode, num_drones: int, state_collector=None):
        self.udp = udp
        self.num_drones = num_drones
        self.collector = state_collector

    def _drone_port(self, drone_id: int) -> int:
        return AGENT_BASE_PORT + drone_id * AGENT_PORT_STEP

    def _send_to(self, drone_id: int, msg_type: str, data: dict):
        raw = make_msg(msg_type, 0, data)
        self.udp.send(raw, GCS_HOST, self._drone_port(drone_id))

    def _send_to_all(self, msg_type: str, data: dict,
                     drone_ids: set[int] | None = None):
        targets = drone_ids or set(range(1, self.num_drones + 1))
        for i in targets:
            self._send_to(i, msg_type, data)

    # ── Commands ───────────────────────────────────────────

    def takeoff_all(self, alt: float):
        """Command all drones to takeoff to alt meters."""
        log.info("CMD: takeoff all to %.1f m", alt)
        self._send_to_all("TAKEOFF_CMD", {"target_id": 0, "alt": alt})

    def takeoff(self, drone_id: int, alt: float):
        """Command a single drone to takeoff."""
        log.info("CMD: takeoff drone %d to %.1f m", drone_id, alt)
        self._send_to(drone_id, "TAKEOFF_CMD", {"target_id": drone_id, "alt": alt})

    def land_all(self):
        """Command all drones to land."""
        log.info("CMD: land all")
        self._send_to_all("LAND_CMD", {"target_id": 0})

    def land(self, drone_id: int):
        """Command a single drone to land."""
        log.info("CMD: land drone %d", drone_id)
        self._send_to(drone_id, "LAND_CMD", {"target_id": drone_id})

    def send_waypoint(self, drone_id: int, lat: float, lon: float, alt: float):
        """Send a waypoint to a specific drone."""
        log.info("CMD: waypoint drone %d -> (%.6f, %.6f, %.1f)", drone_id, lat, lon, alt)
        self._send_to(drone_id, "WAYPOINT_CMD", {
            "target_id": drone_id,
            "lat": lat, "lon": lon, "alt": alt,
        })

    def send_velocity(self, drone_id: int, vn: float, ve: float, vd: float):
        """Send a velocity setpoint to a specific drone."""
        log.info("CMD: velocity drone %d -> (%.1f, %.1f, %.1f)", drone_id, vn, ve, vd)
        self._send_to(drone_id, "VELOCITY_CMD", {
            "target_id": drone_id,
            "vn": vn, "ve": ve, "vd": vd,
        })

    def set_formation(self, shape: str, heading_deg: float, spacing_m: float):
        """Command all drones to form a specific formation."""
        # Use leader's current position as reference
        ref_lat, ref_lon, ref_alt = 0.0, 0.0, 10.0
        if self.collector:
            leader_pos = self.collector.get_leader_position()
            if leader_pos:
                ref_lat, ref_lon, ref_alt = leader_pos

        data = {
            "formation": shape.upper(),
            "leader_id": 1,
            "ref_lat": ref_lat,
            "ref_lon": ref_lon,
            "ref_alt": ref_alt,
            "heading_deg": heading_deg,
            "spacing_m": spacing_m,
        }
        log.info("CMD: formation %s heading=%.0f spacing=%.1f", shape, heading_deg, spacing_m)
        self._send_to_all("FORMATION_CMD", data)
