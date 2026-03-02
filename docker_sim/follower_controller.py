"""FollowerController — 3-mode hard-switched guidance wrapper.

Wraps guidance_lib's compute_guidance() with:
  - Own-state tracking
  - TargetComputer (leader + offset + feedforward)
  - CommandSmoother (EMA output filter)
  - Velocity send to MAVLink

Key: leader state is passed DIRECTLY each tick (None = stale/missing).
"""

import logging

from docker_sim.mavlink_conn import MavlinkConn

from guidance_lib import (
    GuidanceConfig, GuidanceState, compute_guidance,
    CommandSmoother,
)
from guidance_lib.target import TargetComputer

log = logging.getLogger(__name__)


class FollowerController:
    """3-mode hard-switched guidance with modular safety."""

    def __init__(self, conn: MavlinkConn, cfg: GuidanceConfig,
                 offset_n: float, offset_e: float, offset_d: float,
                 control_hz: int = 10, bridge=None, my_id: int = 0):
        self.conn = conn
        self.cfg = cfg
        self.state = GuidanceState()
        self.smoother = CommandSmoother(dt=1.0 / control_hz)
        self.target_computer = TargetComputer()

        # Formation offset
        self.offset_n = offset_n
        self.offset_e = offset_e
        self.offset_d = offset_d
        self.control_hz = control_hz

        # Collision avoidance: bridge for neighbor awareness
        self.bridge = bridge
        self.my_id = my_id

        # Own state
        self.my_lat = 0.0
        self.my_lon = 0.0
        self.my_alt = 0.0
        self.my_vn = 0.0
        self.my_ve = 0.0
        self.my_vd = 0.0

    def update_own_state(self, pos: dict):
        self.my_lat = pos['lat']
        self.my_lon = pos['lon']
        self.my_alt = pos['alt']
        self.my_vn = pos['vx']
        self.my_ve = pos['vy']
        self.my_vd = pos['vz']

    def tick(self, leader: dict) -> dict:
        """Run one guidance tick.

        Args:
            leader: Leader state dict {lat, lon, alt, vx, vy} or None if stale.

        Returns:
            Guidance result dict, or None if own state not ready.
        """
        if self.my_lat == 0.0 and self.my_lon == 0.0:
            return None

        if leader is None or (leader['lat'] == 0.0 and leader['lon'] == 0.0):
            return None

        leader_lat = leader['lat']
        leader_lon = leader['lon']
        leader_alt = leader['alt']
        leader_vn = leader['vx']
        leader_ve = leader['vy']

        # Compute target = leader + offset + feedforward
        target_lat, target_lon, target_alt = self.target_computer.compute(
            leader_lat, leader_lon, leader_alt,
            leader_vn, leader_ve,
            self.offset_n, self.offset_e, self.offset_d,
            ff_gain=self.cfg.ff_gain, dt=1.0 / self.control_hz)

        # Get neighbor states for collision avoidance
        neighbors = None
        if self.bridge is not None:
            peers = self.bridge.get_all_peer_states()
            if peers:
                neighbors = list(peers.values())

        # 3-mode guidance + collision avoidance filter
        result = compute_guidance(
            my_lat=self.my_lat, my_lon=self.my_lon, my_alt=self.my_alt,
            my_vn=self.my_vn, my_ve=self.my_ve, my_vd=self.my_vd,
            peer_lat=leader_lat, peer_lon=leader_lon, peer_alt=leader_alt,
            peer_vn=leader_vn, peer_ve=leader_ve,
            goal_lat=target_lat, goal_lon=target_lon, goal_alt=target_alt,
            cfg=self.cfg, state=self.state,
            neighbors=neighbors, my_id=self.my_id,
        )

        # Smooth & send
        sm_vn, sm_ve, sm_vd = self.smoother.filter(
            result['vn'], result['ve'], result['vd'])
        self.conn.send_velocity_ned(sm_vn, sm_ve, sm_vd)

        if result['mode'] != 'TRACKING':
            log.info("Mode: %s (w_e=%.2f w_t=%.2f w_c=%.2f)",
                     result['mode'], result['w_evasion'],
                     result['w_tracking'], result['w_catchup'])

        return result

    def reset(self):
        """Reset all internal state. Call on failsafe transitions."""
        self.smoother.reset()
        self.state = GuidanceState()
