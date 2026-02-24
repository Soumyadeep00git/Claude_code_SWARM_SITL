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

from config import (
    NUM_DRONES, GCS_HOST, GCS_PORT,
    AGENT_BASE_PORT, AGENT_PORT_STEP,
    AGENT_LOOP_HZ, STATE_REPORT_HZ,
)
from comms.protocol import make_msg, parse_msg
from comms.udp_node import UDPNode
from mavlink_layer.drone_connection import DroneConnection
from drone_agent.state import DroneState, PeerTable
from drone_agent.local_planner import LocalPlanner
from drone_agent.global_planner import GlobalPlanner
from drone_agent.failsafe import FailsafeManager, SwarmState

log = logging.getLogger(__name__)


class DroneAgent:
    """Encapsulates the full drone agent lifecycle."""

    def __init__(self, drone_id: int):
        self.drone_id = drone_id
        self.running = True

        # Components
        self.conn = DroneConnection(drone_id)
        self.udp = UDPNode(AGENT_BASE_PORT + drone_id * AGENT_PORT_STEP)
        self.state = DroneState(drone_id=drone_id)
        self.peers = PeerTable()
        self.local_planner = LocalPlanner(self.conn)
        self.global_planner = GlobalPlanner(drone_id, NUM_DRONES)
        self.failsafe = FailsafeManager(
            drone_id, self.local_planner, self.conn,
            self.global_planner,
            expected_peers=set(range(1, NUM_DRONES + 1)),
        )

        # Timing
        self._loop_period = 1.0 / AGENT_LOOP_HZ
        self._report_interval = 1.0 / STATE_REPORT_HZ
        self._last_report = 0.0

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

            # 4. PLAN & EXECUTE — runs in NOMINAL and DEGRADED
            if self.failsafe.state in (SwarmState.NOMINAL, SwarmState.DEGRADED):
                if self.global_planner.formation != "NONE":
                    target = self.global_planner.get_target_position()
                    self.local_planner.set_waypoint(*target)
                self.local_planner.tick(
                    armed=self.state.armed,
                    mode=self.state.mode,
                    has_gps=(self.state.lat != 0.0 or self.state.lon != 0.0),
                )

            # 5. REPORT — send state to GCS
            now = time.time()
            if now - self._last_report >= self._report_interval:
                self._send_state_report()
                self._last_report = now

            # 6. SEND ALERTS
            for alert in self.failsafe.pop_alerts():
                raw = make_msg("ALERT", self.drone_id, alert)
                self.udp.send(raw, GCS_HOST, GCS_PORT)

            # 7. SLEEP to maintain loop rate
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

        elif t == "STATE_REPORT":
            # Peer state relayed from GCS
            peer_id = d.get("drone_id")
            if peer_id and peer_id != my_id:
                self.peers.update_peer(peer_id, d, msg.get("ts", time.time()))

    def _send_state_report(self):
        """Send current state to GCS."""
        raw = make_msg("STATE_REPORT", self.drone_id, self.state.to_dict())
        self.udp.send(raw, GCS_HOST, GCS_PORT)


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
