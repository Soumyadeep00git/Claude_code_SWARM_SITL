"""
Drone agent — main process for a single drone.
Each drone runs one instance of this script.

Usage:
    python -m drone_agent.agent <drone_id>

Main loop order:
  1. Read MAVLink telemetry (update own state)
  2. Read UDP messages (commands from GCS, peer states)
  3. Failsafe checks (can override planner)
  4. Execute planner (if no failsafe override)
  5. Report state to GCS
  6. Send alerts
"""

import sys
import time
import signal
import logging

import numpy as np

from config import (
    NUM_DRONES, GCS_HOST, GCS_PORT,
    AGENT_BASE_PORT, AGENT_PORT_STEP,
    AGENT_LOOP_HZ, STATE_REPORT_HZ,
    P2P_ENABLED, MESH_SIM_ENABLED, MESH_SIM_RANGE_M,
)
from comms.protocol import make_msg, parse_msg
from comms.udp_node import UDPNode
from mavlink_layer.drone_connection import DroneConnection
from drone_agent.state import DroneState, PeerTable
from drone_agent.local_planner import LocalPlanner, RL_MODE
from drone_agent.global_planner import GlobalPlanner
from drone_agent.failsafe import FailsafeManager, SwarmState
from drone_agent.geo import gps_to_ned_2d

if MESH_SIM_ENABLED:
    from comms.mesh_sim import SimulatedUDPNode
    from drone_agent.mesh_router import MeshRouter

log = logging.getLogger(__name__)


class DroneAgent:
    """Encapsulates the full drone agent lifecycle."""

    def __init__(self, drone_id: int):
        self.drone_id = drone_id
        self.running = True

        # Components
        self.conn = DroneConnection(drone_id)
        self.state = DroneState(drone_id=drone_id)
        self.peers = PeerTable()

        # UDP node: simulated mesh or plain
        if MESH_SIM_ENABLED:
            self.udp = SimulatedUDPNode(
                AGENT_BASE_PORT + drone_id * AGENT_PORT_STEP,
                drone_id=drone_id,
                position_getter=self._get_all_positions,
                max_range_m=MESH_SIM_RANGE_M,
            )
            self.mesh_router = MeshRouter(drone_id, NUM_DRONES)
        else:
            self.udp = UDPNode(AGENT_BASE_PORT + drone_id * AGENT_PORT_STEP)
            self.mesh_router = None

        self.local_planner = LocalPlanner(self.conn)
        self.global_planner = GlobalPlanner(drone_id, NUM_DRONES)
        self.failsafe = FailsafeManager(
            drone_id, self.local_planner, self.conn,
            self.global_planner,
            expected_peers=set(range(1, NUM_DRONES + 1)),
        )

        # P2P peer targets: (host, port) for all other drones
        if P2P_ENABLED:
            self._peer_targets = [
                (GCS_HOST, AGENT_BASE_PORT + i * AGENT_PORT_STEP)
                for i in range(1, NUM_DRONES + 1)
                if i != drone_id
            ]
        else:
            self._peer_targets = []

        # Timing
        self._loop_period = 1.0 / AGENT_LOOP_HZ
        self._report_interval = 1.0 / STATE_REPORT_HZ
        self._last_report = 0.0

        # PROXIMITY_ALERT rate limiting (max 1 per second)
        self._last_prox_alert_time = 0.0

        log.info("Drone %d agent initialized", drone_id)

    def run(self):
        """Main loop — runs until self.running is False or interrupted."""
        log.info("Drone %d: entering main loop at %d Hz", self.drone_id, AGENT_LOOP_HZ)

        while self.running:
            t0 = time.time()

            # 1. READ MAVLINK — update own state from SITL
            self._read_mavlink()

            # 2. READ UDP — process commands from GCS and peer states
            self._read_udp()

            # 3. FAILSAFE CHECK — state machine
            fs_active = self.failsafe.tick(self.peers, self.state)
            self.state.failsafe_active = fs_active
            self.state.swarm_state = self.failsafe.state.value
            self.state.leader_id = self.failsafe.elect_leader()
            self.state.alive_count = len(self.failsafe._alive_peers)

            # 3b. BROADCAST PROXIMITY_ALERT — cooperative collision avoidance
            if self.failsafe._proximity_peers and t0 - self._last_prox_alert_time >= 1.0:
                alert_data = self.failsafe.build_proximity_alert(self.peers, self.state)
                if alert_data:
                    raw = make_msg("PROXIMITY_ALERT", self.drone_id, alert_data)
                    self.udp.send(raw, GCS_HOST, GCS_PORT)
                    if self._peer_targets:
                        self.udp.send_to_multiple(raw, self._peer_targets)
                    self._last_prox_alert_time = t0

            # 4. PLAN & EXECUTE — runs in NOMINAL and DEGRADED
            if self.failsafe.state in (SwarmState.NOMINAL, SwarmState.DEGRADED):
                has_gps = (self.state.lat != 0.0 or self.state.lon != 0.0)

                # Always collect peer GPS for collision avoidance (all modes)
                peer_gps = []
                if has_gps:
                    for pid, plat, plon, palt, pvx, pvy, pvz in self.peers.get_all_states():
                        if pid == self.drone_id or (plat == 0.0 and plon == 0.0):
                            continue
                        peer_gps.append((plat, plon))
                self.local_planner.update_peer_gps(
                    self.state.lat, self.state.lon, self.state.alt, peer_gps)

                if self.global_planner.formation != "NONE" and has_gps:
                    # Gather peer NED positions for formation controller
                    peer_ned = []
                    peer_vel_ned = []
                    ref_lat = self.global_planner.ref_lat
                    ref_lon = self.global_planner.ref_lon
                    ref_alt = self.global_planner.ref_alt

                    for pid, plat, plon, palt, pvx, pvy, pvz in self.peers.get_all_states():
                        if pid == self.drone_id or (plat == 0.0 and plon == 0.0):
                            continue
                        pn, pe = gps_to_ned_2d(plat, plon, ref_lat, ref_lon)
                        peer_ned.append([pn, pe, -(palt - ref_alt)])
                        peer_vel_ned.append([pvx, pvy, pvz])

                    peer_ned_arr = np.array(peer_ned) if peer_ned else np.empty((0, 3))
                    peer_vel_arr = np.array(peer_vel_ned) if peer_vel_ned else np.empty((0, 3))

                    # Global planner: Hybrid A* path at 1 Hz
                    path_ned, goal_ned = self.global_planner.plan_path(
                        self.state.lat, self.state.lon, peer_gps)

                    # Feed NED state and formation data to local planner
                    self.local_planner.update_own_state_ned(
                        self.state.lat, self.state.lon, self.state.alt,
                        self.state.vx, self.state.vy, self.state.vz,
                        ref_lat, ref_lon, ref_alt)
                    self.local_planner.set_formation_target(
                        goal_ned, path_ned, peer_ned_arr,
                        ref_lat, ref_lon, ref_alt,
                        peer_velocities_ned=peer_vel_arr)

                elif self.local_planner._rl_enabled and has_gps:
                    # Feed NED state for RL mode (no formation active)
                    ref_lat = self.global_planner.ref_lat if self.global_planner.ref_lat != 0.0 else self.state.lat
                    ref_lon = self.global_planner.ref_lon if self.global_planner.ref_lon != 0.0 else self.state.lon
                    ref_alt = self.global_planner.ref_alt if self.global_planner.ref_alt != 0.0 else self.state.alt

                    self.local_planner.update_own_state_ned(
                        self.state.lat, self.state.lon, self.state.alt,
                        self.state.vx, self.state.vy, self.state.vz,
                        ref_lat, ref_lon, ref_alt)

                    peer_ned = []
                    for pid, plat, plon, palt, pvx, pvy, pvz in self.peers.get_all_states():
                        if pid == self.drone_id or (plat == 0.0 and plon == 0.0):
                            continue
                        pn, pe = gps_to_ned_2d(plat, plon, ref_lat, ref_lon)
                        peer_ned.append([pn, pe, -(palt - ref_alt)])
                    self.local_planner._peer_positions_ned = (
                        np.array(peer_ned) if peer_ned else np.empty((0, 3)))

                self.local_planner.tick(
                    armed=self.state.armed,
                    mode=self.state.mode,
                    has_gps=has_gps,
                    alt=self.state.alt,
                )

            # 5. MESH ROUTER — update neighbors and advertise
            if self.mesh_router is not None and MESH_SIM_ENABLED:
                link_map = self.udp.get_link_quality_map()
                self.mesh_router.update_neighbors(link_map)
                if self.mesh_router.should_advertise():
                    ad_data = self.mesh_router.get_advertisement_data()
                    raw = make_msg("NEIGHBOR_AD", self.drone_id, ad_data)
                    self.udp.send(raw, GCS_HOST, GCS_PORT)
                    if self._peer_targets:
                        self.udp.send_to_multiple(raw, self._peer_targets)
                # Update mesh stats on state
                self.state.mesh_stats = {
                    "link_qualities": {str(k): v for k, v in link_map.items()},
                    "link_stats": {str(k): v for k, v in self.udp.get_link_stats().items()},
                    "routing_table": {str(k): v for k, v in self.mesh_router.get_routing_table().items()},
                    "forwarded": self.mesh_router.forwarded_count,
                }

            # 6. REPORT — send state to GCS
            now = time.time()
            if now - self._last_report >= self._report_interval:
                self._send_state_report()
                self._last_report = now

            # 7. SEND ALERTS
            for alert in self.failsafe.pop_alerts():
                raw = make_msg("ALERT", self.drone_id, alert)
                self.udp.send(raw, GCS_HOST, GCS_PORT)

            # 8. SLEEP to maintain loop rate
            elapsed = time.time() - t0
            sleep_time = self._loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        """Signal the main loop to exit."""
        self.running = False
        self.udp.close()
        log.info("Drone %d agent stopped", self.drone_id)

    # ── Internal methods ───────────────────────────────────

    def _read_mavlink(self):
        """Poll MAVLink for latest telemetry. Drain buffer to get freshest data."""
        # Drain all messages, keeping only the latest of each type
        last_pos = None
        last_batt = None
        last_hb = None

        while True:
            msg = self.conn.conn.recv_match(blocking=False)
            if msg is None:
                break
            msg_type = msg.get_type()
            if msg_type == "GLOBAL_POSITION_INT":
                last_pos = msg
            elif msg_type == "SYS_STATUS":
                last_batt = msg
            elif msg_type == "HEARTBEAT":
                last_hb = msg

        if last_pos:
            self.state.update_from_position({
                "lat": last_pos.lat / 1e7,
                "lon": last_pos.lon / 1e7,
                "alt": last_pos.relative_alt / 1000.0,
                "vx": last_pos.vx / 100.0,
                "vy": last_pos.vy / 100.0,
                "vz": last_pos.vz / 100.0,
                "heading": last_pos.hdg / 100.0,
            })

        if last_batt:
            self.state.battery_pct = last_batt.battery_remaining

        if last_hb and last_hb.get_srcSystem() > 0:
            from pymavlink import mavutil as _mu
            mode = _mu.mode_string_v10(last_hb)
            armed = bool(last_hb.base_mode & _mu.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self.state.update_from_heartbeat({"mode": mode, "armed": armed})

    def _read_udp(self):
        """Process all pending UDP messages."""
        for raw, addr in self.udp.recv_all():
            msg = parse_msg(raw)
            if msg is None:
                continue

            # Any message from GCS counts as a heartbeat
            if msg["src"] == 0:
                self.failsafe.update_gcs_heartbeat()

            self._handle_message(msg)

    def _handle_message(self, msg: dict):
        """Dispatch a single message to the appropriate handler."""
        t = msg["type"]
        d = msg["data"]
        my_id = self.drone_id

        if t == "TAKEOFF_CMD":
            if d["target_id"] in (0, my_id):
                self.local_planner.do_takeoff(d["alt"])

        elif t == "LAND_CMD":
            if d["target_id"] in (0, my_id):
                self.local_planner.do_land()

        elif t == "WAYPOINT_CMD":
            if d["target_id"] == my_id:
                self.global_planner.formation = "NONE"
                self.local_planner.set_waypoint(d["lat"], d["lon"], d["alt"])

        elif t == "VELOCITY_CMD":
            if d["target_id"] == my_id:
                self.global_planner.formation = "NONE"
                self.local_planner.set_velocity(d["vn"], d["ve"], d["vd"])

        elif t == "FORMATION_CMD":
            self.global_planner.set_formation(d)
            self.state.formation_slot = self.global_planner.slot

        elif t == "SWARM_WAYPOINT_CMD":
            if self.global_planner.formation != "NONE":
                self.global_planner.ref_lat = d["ref_lat"]
                self.global_planner.ref_lon = d["ref_lon"]
                self.global_planner.ref_alt = d["ref_alt"]
                self.global_planner._last_replan = 0.0
                log.info("Drone %d: swarm waypoint -> (%.6f, %.6f)",
                         self.drone_id, d["ref_lat"], d["ref_lon"])
            else:
                self.local_planner.set_waypoint(d["ref_lat"], d["ref_lon"], d["ref_alt"])

        elif t == "STATE_REPORT":
            # Peer state relayed from GCS
            peer_id = d.get("drone_id")
            if peer_id and peer_id != my_id:
                self.peers.update_peer(peer_id, d, msg.get("ts", time.time()))
                self.failsafe.update_peer_heartbeat(peer_id)

        elif t == "PEER_HEARTBEAT":
            # Direct peer-to-peer state update (P2P mesh)
            peer_id = d.get("drone_id")
            if peer_id and peer_id != my_id:
                self.peers.update_peer(peer_id, d, msg.get("ts", time.time()))
                self.failsafe.update_peer_heartbeat(peer_id)

        elif t == "NEIGHBOR_AD":
            if self.mesh_router is not None:
                peer_id = msg["src"]
                if peer_id != my_id:
                    self.mesh_router.handle_neighbor_ad(peer_id, d)

        elif t == "MESH_FORWARD":
            if self.mesh_router is not None:
                if d.get("dest") == my_id:
                    # Payload is for us — process it
                    payload = d.get("payload")
                    if payload:
                        self._handle_message(payload)
                else:
                    # Forward to next hop
                    fwd = self.mesh_router.handle_mesh_forward(d)
                    if fwd:
                        next_hop = fwd.get("next_hop")
                        if next_hop:
                            port = AGENT_BASE_PORT + next_hop * AGENT_PORT_STEP
                            self.udp.send(
                                make_msg("MESH_FORWARD", self.drone_id, fwd),
                                GCS_HOST, port)

        elif t == "MESH_CONFIG_CMD":
            if MESH_SIM_ENABLED and hasattr(self.udp, 'set_range'):
                self.udp.set_range(float(d.get("range_m", MESH_SIM_RANGE_M)))

        elif t == "PROXIMITY_ALERT":
            # Cooperative collision avoidance — peer is warning us
            self.local_planner.receive_proximity_alert(d, my_id)

        elif t == "SAFETY_CONFIG_CMD":
            # Update isolation radius on all components
            radius = float(d.get("isolation_radius_m", 5.0))
            self.local_planner.set_isolation_radius(radius)
            self.failsafe.set_isolation_radius(radius)
            log.info("Drone %d: isolation radius updated to %.1fm", my_id, radius)

        elif t == "RL_MODE_CMD":
            if d.get("target_id", 0) in (0, my_id):
                enable = d.get("enable", False)
                if enable:
                    self.local_planner.enable_rl_mode()
                    self.state.rl_mode = True
                    if "goal_ned" in d:
                        goal = np.array(d["goal_ned"], dtype=np.float64)
                        self.local_planner.set_rl_goal(goal)
                else:
                    self.local_planner.disable_rl_mode()
                    self.state.rl_mode = False

    def _get_all_positions(self) -> dict[int, tuple[float, float]]:
        """Return {drone_id: (lat, lon)} for self and all known peers.
        Used as position_getter callback by SimulatedUDPNode."""
        positions = {}
        if self.state.lat != 0.0 or self.state.lon != 0.0:
            positions[self.drone_id] = (self.state.lat, self.state.lon)
        for pid, plat, plon, palt in self.peers.get_all_positions():
            if plat != 0.0 or plon != 0.0:
                positions[pid] = (plat, plon)
        return positions

    def _send_state_report(self):
        """Send current state to GCS, and directly to peers if P2P enabled."""
        state_data = self.state.to_dict()
        # Always send STATE_REPORT to GCS (unchanged)
        self.udp.send(make_msg("STATE_REPORT", self.drone_id, state_data),
                      GCS_HOST, GCS_PORT)
        # P2P: send PEER_HEARTBEAT directly to all peers
        if self._peer_targets:
            raw = make_msg("PEER_HEARTBEAT", self.drone_id, state_data)
            self.udp.send_to_multiple(raw, self._peer_targets)


# ── Entry point ────────────────────────────────────────────

def main():
    if len(sys.argv) != 2:
        print(f"Usage: python -m drone_agent.agent <drone_id>")
        sys.exit(1)

    drone_id = int(sys.argv[1])

    logging.basicConfig(
        level=logging.INFO,
        format=f"[D{drone_id}] %(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    agent = DroneAgent(drone_id)

    def shutdown(sig, frame):
        log.info("Drone %d: caught signal %d, shutting down", drone_id, sig)
        agent.stop()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    try:
        agent.run()
    except KeyboardInterrupt:
        pass
    finally:
        agent.stop()


if __name__ == "__main__":
    main()
