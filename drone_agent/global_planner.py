"""
Global planner — formation slot management + Hybrid A* path planning.
Computes this drone's target position based on its assigned slot,
the current formation shape, and the formation reference point.
When the target is far or obstructed, Hybrid A* generates intermediate
waypoints for the local planner to follow.
"""

import time
import logging
import numpy as np
from math import cos, sin, radians, pi

from config import (DEFAULT_SPACING_M, DEFAULT_FORMATION, NUM_DRONES,
                    ASTAR_CONFIG, ASTAR_REPLAN_INTERVAL_S)
from drone_agent.hybrid_astar import HybridAStarPlanner
from drone_agent.geo import gps_to_ned_2d

log = logging.getLogger(__name__)


class GlobalPlanner:
    """Formation offset calculator for one drone."""

    def __init__(self, drone_id: int, num_drones: int = NUM_DRONES):
        self.drone_id = drone_id
        self.num_drones = num_drones
        self.slot: int = drone_id - 1  # 0-indexed slot in formation

        # Formation parameters (set by FORMATION_CMD)
        self.formation: str = "NONE"  # NONE means no formation active
        self.spacing: float = DEFAULT_SPACING_M
        self.ref_lat: float = 0.0
        self.ref_lon: float = 0.0
        self.ref_alt: float = 0.0
        self.ref_heading: float = 0.0  # degrees
        self.leader_id: int = 0  # 0 = unset; derived from GCS or failsafe election

        # Hybrid A* path planner
        self._astar = HybridAStarPlanner(ASTAR_CONFIG)
        self._last_replan: float = 0.0
        self._cached_path: list = []   # List of (north, east) NED waypoints
        self._wp_index: int = 0

    def set_formation(self, data: dict):
        """Update formation parameters from a FORMATION_CMD message."""
        self.formation = data.get("formation", self.formation)
        self.leader_id = data.get("leader_id", self.leader_id)
        self.ref_lat = data.get("ref_lat", self.ref_lat)
        self.ref_lon = data.get("ref_lon", self.ref_lon)
        self.ref_alt = data.get("ref_alt", self.ref_alt)
        self.ref_heading = data.get("heading_deg", self.ref_heading)
        self.spacing = data.get("spacing_m", self.spacing)
        log.info(
            "Drone %d: formation=%s slot=%d heading=%.0f spacing=%.1f",
            self.drone_id, self.formation, self.slot,
            self.ref_heading, self.spacing,
        )

    def get_target_position(self) -> tuple[float, float, float]:
        """
        Compute this drone's target lat/lon/alt based on its slot.
        Returns (lat, lon, alt).
        """
        offset_n, offset_e = self._compute_slot_offset()

        # Convert NED meters to lat/lon degrees
        target_lat = self.ref_lat + (offset_n / 111320.0)
        target_lon = self.ref_lon + (offset_e / (111320.0 * cos(radians(self.ref_lat))))
        target_alt = self.ref_alt

        return (target_lat, target_lon, target_alt)

    def _compute_slot_offset(self) -> tuple[float, float]:
        """
        Returns (north_m, east_m) offset for this drone's slot.
        Offset is in LOCAL frame, then rotated by formation heading.
        """
        s = self.slot
        sp = self.spacing
        n = self.num_drones
        hdg = radians(self.ref_heading)

        # Compute local (unrotated) offset
        if self.formation == "LINE":
            # Side-by-side perpendicular to heading
            local_n = 0.0
            local_e = (s - (n - 1) / 2.0) * sp

        elif self.formation == "V":
            # V shape: slot 0 at the tip
            if s == 0:
                local_n = 0.0
                local_e = 0.0
            else:
                side = 1 if s % 2 == 1 else -1
                rank = (s + 1) // 2
                local_n = -rank * sp * cos(pi / 6)  # 30-degree V angle
                local_e = side * rank * sp * sin(pi / 6)

        elif self.formation == "COLUMN":
            # Single file along heading
            local_n = -s * sp
            local_e = 0.0

        elif self.formation == "DIAMOND":
            # Diamond: slot 0 front, then left/right, then back
            if s == 0:
                local_n = sp
                local_e = 0.0
            elif s == 1:
                local_n = 0.0
                local_e = -sp
            elif s == 2:
                local_n = 0.0
                local_e = sp
            elif s == 3:
                local_n = -sp
                local_e = 0.0
            else:
                # Extra drones: extend backward
                local_n = -((s - 3) + 2) * sp
                local_e = 0.0

        else:
            local_n = 0.0
            local_e = 0.0

        # Rotate by formation heading
        north = local_n * cos(hdg) - local_e * sin(hdg)
        east = local_n * sin(hdg) + local_e * cos(hdg)

        return (north, east)

    def reslot(self, alive_ids: set[int], leader_id: int, peers=None):
        """
        Compact slot assignment based on alive drone IDs.
        sorted(alive_ids) determines slot order — gaps from dead drones close.
        Updates ref_lat/ref_lon from centroid of alive peers (not leader).
        """
        sorted_ids = sorted(alive_ids)
        if self.drone_id not in alive_ids:
            return  # We're not in the alive set; don't reslot
        old_slot = self.slot
        self.slot = sorted_ids.index(self.drone_id)
        self.num_drones = len(sorted_ids)
        self.leader_id = leader_id

        # Update reference position from centroid of alive peers
        if peers is not None:
            valid_lats, valid_lons = [], []
            for pid in alive_ids:
                if pid == self.drone_id:
                    continue
                peer = peers.get_peer(pid)
                if peer and (peer.lat != 0.0 or peer.lon != 0.0):
                    valid_lats.append(peer.lat)
                    valid_lons.append(peer.lon)
            if valid_lats:
                self.ref_lat = sum(valid_lats) / len(valid_lats)
                self.ref_lon = sum(valid_lons) / len(valid_lons)

        if old_slot != self.slot or self.num_drones != len(sorted_ids):
            log.info(
                "Drone %d: reslot %d->%d (alive=%s, leader=%d, n=%d)",
                self.drone_id, old_slot, self.slot,
                sorted_ids, leader_id, self.num_drones,
            )

    # ── Hybrid A* path planning ──────────────────────────────

    def get_target_ned(self):
        """Get formation slot target as NED offset [north, east, down]."""
        n, e = self._compute_slot_offset()
        return np.array([n, e, 0.0], dtype=np.float64)

    def plan_path(self, my_lat, my_lon, peer_gps_list):
        """
        Compute collision-free path to formation slot via Hybrid A*.
        Runs A* at 1 Hz, caches path between replans.

        Args:
            my_lat, my_lon: current GPS position
            peer_gps_list: list of (lat, lon) for known peers

        Returns:
            (path_ned, goal_ned): path is list of (north, east) waypoints in NED,
                                  goal is np.array [n, e, d] of slot target.
        """
        goal_ned = self.get_target_ned()
        now = time.time()

        # Only replan at configured interval
        if now - self._last_replan < ASTAR_REPLAN_INTERVAL_S:
            return self._cached_path, goal_ned

        self._last_replan = now

        # Convert own position to NED relative to formation reference
        my_n, my_e = gps_to_ned_2d(my_lat, my_lon, self.ref_lat, self.ref_lon)
        start = np.array([my_n, my_e])
        goal_2d = np.array([goal_ned[0], goal_ned[1]])

        # Convert peer positions to NED
        peer_ned = []
        for plat, plon in peer_gps_list:
            if plat == 0.0 and plon == 0.0:
                continue
            pn, pe = gps_to_ned_2d(plat, plon, self.ref_lat, self.ref_lon)
            peer_ned.append([pn, pe])

        peer_arr = np.array(peer_ned) if peer_ned else np.empty((0, 2))

        # Run Hybrid A* (short-circuits internally for close/clear paths)
        path = self._astar.plan(start, goal_2d, peer_arr)

        self._cached_path = path
        self._wp_index = 0

        return self._cached_path, goal_ned

    def get_current_waypoint_ned(self, my_n, my_e, arrival_radius=2.0):
        """
        Get the next waypoint to track from cached path.
        Advances index when drone arrives within arrival_radius.

        Returns: (north, east) tuple of current waypoint.
        """
        if not self._cached_path:
            n, e = self._compute_slot_offset()
            return (n, e)

        # Advance past arrived waypoints
        while self._wp_index < len(self._cached_path) - 1:
            wp = self._cached_path[self._wp_index]
            dist = np.hypot(wp[0] - my_n, wp[1] - my_e)
            if dist < arrival_radius:
                self._wp_index += 1
            else:
                break

        return self._cached_path[self._wp_index]
