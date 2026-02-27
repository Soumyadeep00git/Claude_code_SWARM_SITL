"""10Hz follower control loop — 3-mode hard-switched guidance.

Modes: Evasion > Catch-up > Tracking — hard if/else priority switches.
No adaptive controller, no APF. Simple and stable.
"""

import logging
import math
import time
from dataclasses import asdict

from sim.config import (
    FOLLOW_OFFSET_N, FOLLOW_OFFSET_E, FOLLOW_OFFSET_D,
    METERS_PER_DEG_LAT, CONTROL_HZ,
)
from sim.mavlink_conn import MavlinkConn

from follower_drone.guidance import GuidanceConfig, GuidanceState, compute_guidance
from follower_drone.geo_utils import ned_to_gps, gps_to_ned
from follower_drone.command_smoother import CommandSmoother

log = logging.getLogger(__name__)

# Default config values for web_gcs override validation
DEFAULT_GUIDANCE = asdict(GuidanceConfig())


class FollowerController:
    """Runs the 3-mode hard-switched guidance loop."""

    def __init__(self, conn: MavlinkConn):
        self.conn = conn
        self.cfg = GuidanceConfig()
        self.state = GuidanceState()
        self.cmd_smoother = CommandSmoother()

        # Own state
        self.my_lat = 0.0
        self.my_lon = 0.0
        self.my_alt = 0.0
        self.my_vn = 0.0
        self.my_ve = 0.0
        self.my_vd = 0.0

        # Leader state
        self.leader_lat = 0.0
        self.leader_lon = 0.0
        self.leader_alt = 0.0
        self.leader_vn = 0.0
        self.leader_ve = 0.0

        # Diagnostics
        self.last_result = None
        self.target_lat = 0.0
        self.target_lon = 0.0

    def update_own_state(self, pos_data: dict):
        self.my_lat = pos_data['lat']
        self.my_lon = pos_data['lon']
        self.my_alt = pos_data['alt']
        self.my_vn = pos_data['vx']
        self.my_ve = pos_data['vy']
        self.my_vd = pos_data['vz']

    def update_leader_state(self, pos_data: dict):
        self.leader_lat = pos_data['lat']
        self.leader_lon = pos_data['lon']
        self.leader_alt = pos_data['alt']
        self.leader_vn = pos_data['vx']
        self.leader_ve = pos_data['vy']

    def tick(self) -> dict | None:
        """Run one 10Hz guidance tick. Returns result dict or None."""
        if self.my_lat == 0.0 and self.my_lon == 0.0:
            return None
        if self.leader_lat == 0.0 and self.leader_lon == 0.0:
            return None

        # ── Compute target = leader + NED offset ─────────────
        target_lat, target_lon = ned_to_gps(
            FOLLOW_OFFSET_N, FOLLOW_OFFSET_E,
            self.leader_lat, self.leader_lon)
        target_alt = self.leader_alt - FOLLOW_OFFSET_D

        # ── Feedforward: shift target ahead of leader ────────
        ff = self.cfg.ff_gain
        if ff > 0.01 and abs(self.leader_lat) > 1e-6:
            cos_lat = math.cos(math.radians(self.leader_lat))
            target_lat += ff * self.leader_vn * 0.1 / METERS_PER_DEG_LAT
            target_lon += (ff * self.leader_ve * 0.1
                           / (METERS_PER_DEG_LAT * max(cos_lat, 1e-6)))

        self.target_lat = target_lat
        self.target_lon = target_lon

        # ── 3-mode guidance ──────────────────────────────────
        result = compute_guidance(
            my_lat=self.my_lat, my_lon=self.my_lon, my_alt=self.my_alt,
            my_vn=self.my_vn, my_ve=self.my_ve, my_vd=self.my_vd,
            peer_lat=self.leader_lat, peer_lon=self.leader_lon,
            peer_alt=self.leader_alt,
            peer_vn=self.leader_vn, peer_ve=self.leader_ve,
            goal_lat=target_lat, goal_lon=target_lon, goal_alt=target_alt,
            cfg=self.cfg,
            state=self.state,
        )

        # ── Smooth & send velocity command ───────────────────
        sm_vn, sm_ve, sm_vd = self.cmd_smoother.filter(
            result['vn'], result['ve'], result['vd'])
        self.conn.send_velocity_ned(sm_vn, sm_ve, sm_vd)

        result['raw_vn'] = result['vn']
        result['raw_ve'] = result['ve']
        result['raw_vd'] = result['vd']
        result['vn'] = sm_vn
        result['ve'] = sm_ve
        result['vd'] = sm_vd
        result['speed'] = math.sqrt(sm_vn * sm_vn + sm_ve * sm_ve)

        # Inject offset error for logging
        off_n, off_e = gps_to_ned(
            self.my_lat, self.my_lon, target_lat, target_lon)
        result['offset_err_n'] = off_n
        result['offset_err_e'] = off_e
        result['ff_gain'] = self.cfg.ff_gain

        self.last_result = result

        if result.get('emergency'):
            log.warning("GUIDANCE EMERGENCY: flags=%s mode=%s",
                        result['flags'], result['mode'])

        if result['mode'] != 'TRACKING':
            log.info("Mode: %s (w_e=%.2f w_t=%.2f w_c=%.2f)",
                     result['mode'], result['w_evasion'],
                     result['w_tracking'], result['w_catchup'])

        return result
