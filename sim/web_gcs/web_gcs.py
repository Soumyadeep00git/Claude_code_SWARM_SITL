"""Web-based GCS for interactive Leader-Follower collision avoidance testing.

Flask + SocketIO server that controls 2 ArduCopter SITL instances.
Opens a browser UI with Leaflet map, telemetry panels, and controls.

Usage:
    cd /mnt/d/LAT_D/Leader_Follower_ORIN_CO+
    python3 -m sim.web_gcs.web_gcs [--port 5000]
"""

import logging
import math
import os
import signal
import socket
import sys
import threading
import time
from enum import Enum

from flask import Flask, render_template
from flask_socketio import SocketIO

# Add follower_drone package to Python path
_PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_PROJECT_DIR, "follower_drone"))

from sim.config import (
    SITL_BASE_PORT, SITL_PORT_STEP,
    LEADER_ID, FOLLOWER_ID,
    TAKEOFF_ALT_M, CONTROL_HZ,
    LOG_DIR, OUTPUT_DIR,
    HOME_LAT, HOME_LON,
    FOLLOW_OFFSET_N, FOLLOW_OFFSET_E, FOLLOW_OFFSET_D,
    ERRATIC_CHARGE_SPEED, ERRATIC_ZIGZAG_SPEED, ERRATIC_ZIGZAG_PERIOD,
    ERRATIC_SPRINT_SPEED, ERRATIC_PHASE_TIME,
    LEADER_SPEED_MS, METERS_PER_DEG_LAT,
)
from sim.sitl_launcher import SITLLauncher
from sim.mavlink_conn import MavlinkConn
from sim.takeoff import TakeoffManager
from sim.leader_mission import LeaderMission, Phase
from sim.follower_loop import FollowerController, DEFAULT_GUIDANCE

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("web_gcs")


class State(Enum):
    IDLE = "IDLE"
    LAUNCHING = "LAUNCHING"
    CONNECTING = "CONNECTING"
    TAKEOFF = "TAKEOFF"
    FLYING = "FLYING"
    LANDING = "LANDING"


class WebGCS:
    """Web-based GCS with SITL control, APF follower loop, and SocketIO broadcast."""

    def __init__(self, host: str = "0.0.0.0", port: int = 5000):
        self.host = host
        self.port = port
        self.state = State.IDLE
        self._running = True

        # SITL resources (created on start)
        self.launcher = SITLLauncher()
        self.leader_conn: MavlinkConn | None = None
        self.follower_conn: MavlinkConn | None = None
        self.leader_mission: LeaderMission | None = None
        self.follower_ctrl: FollowerController | None = None

        # Telemetry state
        self.leader_pos: dict | None = None
        self.leader_hb: dict | None = None
        self.follower_pos: dict | None = None
        self.follower_hb: dict | None = None
        self.last_apf_result: dict | None = None

        # Manual leader override
        self._manual_vn = 0.0
        self._manual_ve = 0.0
        self._manual_vd = 0.0
        self._manual_active = False
        self._manual_timeout = 0.0  # Auto-zero after 0.5s of no input

        # Click-to-waypoint
        self._click_target_lat = 0.0
        self._click_target_lon = 0.0
        self._click_active = False

        # Erratic scenario override
        self._erratic_override: str | None = None
        self._erratic_start = 0.0

        # Guidance config overrides
        self._guidance_overrides: dict = {}

        # Follower offset (can be changed live)
        self._offset_n = FOLLOW_OFFSET_N
        self._offset_e = FOLLOW_OFFSET_E
        self._offset_d = FOLLOW_OFFSET_D

        # Auto-mission flag
        self._auto_mission = False

        # Lock for thread-safe state updates
        self._lock = threading.Lock()

        # Flask + SocketIO
        template_dir = os.path.join(os.path.dirname(__file__), "templates")
        self.app = Flask(__name__, template_folder=template_dir)
        self.app.config["SECRET_KEY"] = "leader-follower-gcs"
        self.sio = SocketIO(self.app, async_mode="threading", cors_allowed_origins="*")

        self._register_routes()
        self._register_events()

    # ── Routes ─────────────────────────────────────────────────

    def _register_routes(self):
        @self.app.route("/")
        def index():
            return render_template("index.html",
                                   home_lat=HOME_LAT, home_lon=HOME_LON)

    # ── SocketIO Events ────────────────────────────────────────

    def _register_events(self):
        sio = self.sio

        @sio.on("connect")
        def on_connect():
            log.info("Browser connected")
            sio.emit("state_change", {"state": self.state.value})

        @sio.on("cmd_start")
        def on_start():
            if self.state != State.IDLE:
                sio.emit("log", {"msg": f"Cannot start: state={self.state.value}"})
                return
            threading.Thread(target=self._do_start, daemon=True).start()

        @sio.on("cmd_land")
        def on_land():
            if self.state not in (State.FLYING, State.TAKEOFF):
                return
            threading.Thread(target=self._do_land, daemon=True).start()

        @sio.on("cmd_auto_mission")
        def on_auto_mission():
            if self.state != State.FLYING:
                sio.emit("log", {"msg": "Must be FLYING to start auto mission"})
                return
            with self._lock:
                self._auto_mission = True
                self._manual_active = False
                self._click_active = False
                self._erratic_override = None
            sio.emit("log", {"msg": "Auto mission started"})

        @sio.on("cmd_leader_velocity")
        def on_leader_vel(data):
            with self._lock:
                self._manual_vn = float(data.get("vn", 0))
                self._manual_ve = float(data.get("ve", 0))
                self._manual_vd = float(data.get("vd", 0))
                self._manual_active = True
                self._manual_timeout = time.time() + 0.5
                self._auto_mission = False
                self._click_active = False

        @sio.on("cmd_click_waypoint")
        def on_click_wp(data):
            with self._lock:
                self._click_target_lat = float(data["lat"])
                self._click_target_lon = float(data["lon"])
                self._click_active = True
                self._manual_active = False
                self._auto_mission = False
            sio.emit("log", {"msg": f"Waypoint: ({data['lat']:.6f}, {data['lon']:.6f})"})

        @sio.on("cmd_erratic_scenario")
        def on_erratic(data):
            scenario = data.get("scenario", "").upper()
            valid = {"CHARGE", "ZIGZAG", "SUDDEN_STOP", "SPRINT"}
            if scenario not in valid:
                sio.emit("log", {"msg": f"Unknown scenario: {scenario}"})
                return
            with self._lock:
                self._erratic_override = scenario
                self._erratic_start = time.time()
                self._auto_mission = False
                self._manual_active = False
                self._click_active = False
            sio.emit("log", {"msg": f"Erratic scenario: {scenario}"})

        @sio.on("cmd_apf_params")
        def on_apf_params(data):
            with self._lock:
                for k, v in data.items():
                    if k in DEFAULT_GUIDANCE:
                        self._guidance_overrides[k] = float(v)
            sio.emit("log", {"msg": f"Guidance params updated: {list(data.keys())}"})

        @sio.on("cmd_set_offset")
        def on_set_offset(data):
            with self._lock:
                self._offset_n = float(data.get("n", self._offset_n))
                self._offset_e = float(data.get("e", self._offset_e))
                self._offset_d = float(data.get("d", self._offset_d))
            sio.emit("log", {"msg": f"Offset: N={self._offset_n} E={self._offset_e} D={self._offset_d}"})

        @sio.on("cmd_toggle_adaptive")
        def on_toggle_adaptive(data):
            pass  # No adaptive controller in 3-mode guidance

        @sio.on("cmd_stop_manual")
        def on_stop_manual():
            with self._lock:
                self._manual_active = False
                self._click_active = False
                self._erratic_override = None
                self._auto_mission = False
            if self.leader_conn:
                self.leader_conn.send_velocity_ned(0, 0, 0)

    # ── Start / Land sequences ─────────────────────────────────

    def _set_state(self, s: State):
        self.state = s
        self.sio.emit("state_change", {"state": s.value})
        log.info("State -> %s", s.value)

    def _wait_port(self, port: int, timeout: float = 90.0) -> bool:
        deadline = time.time() + timeout
        while self._running and time.time() < deadline:
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=2):
                    return True
            except (ConnectionRefusedError, OSError):
                time.sleep(1)
        return False

    def _do_start(self):
        """Launch SITL, connect, arm, takeoff — then transition to FLYING."""
        try:
            self._set_state(State.LAUNCHING)
            os.makedirs(LOG_DIR, exist_ok=True)
            os.makedirs(OUTPUT_DIR, exist_ok=True)

            self.launcher.launch_all()

            leader_port = SITL_BASE_PORT + LEADER_ID * SITL_PORT_STEP
            follower_port = SITL_BASE_PORT + FOLLOWER_ID * SITL_PORT_STEP

            self.sio.emit("log", {"msg": "Waiting for SITL ports..."})
            if not self._wait_port(leader_port):
                raise TimeoutError("Leader SITL port not reachable")
            if not self._wait_port(follower_port):
                raise TimeoutError("Follower SITL port not reachable")

            # ── Connect ──
            self._set_state(State.CONNECTING)
            self.leader_conn = MavlinkConn(LEADER_ID, timeout=60)
            self.follower_conn = MavlinkConn(FOLLOWER_ID, timeout=60)
            self.sio.emit("log", {"msg": "Both drones connected"})

            # ── GPS fix ──
            self.sio.emit("log", {"msg": "Waiting for GPS fix..."})
            gps_deadline = time.time() + 90.0
            l_gps = f_gps = False
            while self._running and time.time() < gps_deadline:
                ld = self.leader_conn.drain_latest()
                fd = self.follower_conn.drain_latest()
                lp = ld.get("position")
                fp = fd.get("position")
                if lp and lp["lat"] != 0:
                    l_gps = True
                if fp and fp["lat"] != 0:
                    f_gps = True
                if l_gps and f_gps:
                    break
                time.sleep(0.5)
            if not (l_gps and f_gps):
                raise TimeoutError("GPS fix timeout")

            # ── Takeoff ──
            self._set_state(State.TAKEOFF)
            leader_takeoff = TakeoffManager(self.leader_conn, TAKEOFF_ALT_M)
            follower_takeoff = TakeoffManager(self.follower_conn, TAKEOFF_ALT_M)

            while self._running:
                ld = self.leader_conn.drain_latest()
                fd = self.follower_conn.drain_latest()
                lp, lh = ld.get("position"), ld.get("heartbeat")
                fp, fh = fd.get("position"), fd.get("heartbeat")

                l_done = leader_takeoff.tick(
                    has_gps=bool(lp and lp["lat"] != 0),
                    mode=lh["mode"] if lh else "",
                    armed=lh["armed"] if lh else False,
                    alt=lp["alt"] if lp else 0.0,
                )
                f_done = follower_takeoff.tick(
                    has_gps=bool(fp and fp["lat"] != 0),
                    mode=fh["mode"] if fh else "",
                    armed=fh["armed"] if fh else False,
                    alt=fp["alt"] if fp else 0.0,
                )
                if lp:
                    self.leader_pos = lp
                if fp:
                    self.follower_pos = fp

                if l_done and f_done:
                    break
                if not self.launcher.check_alive():
                    raise RuntimeError("SITL process died during takeoff")
                time.sleep(1.0 / CONTROL_HZ)

            # ── Ready to fly ──
            ld = self.leader_conn.drain_latest()
            lp = ld.get("position", {})
            home_lat = lp.get("lat", HOME_LAT)
            home_lon = lp.get("lon", HOME_LON)

            self.leader_mission = LeaderMission(self.leader_conn, home_lat, home_lon)
            self.follower_ctrl = FollowerController(self.follower_conn)

            self._set_state(State.FLYING)
            self.sio.emit("log", {"msg": "Both drones airborne — ready for commands"})

            # Start control + broadcast loops
            threading.Thread(target=self._control_loop, daemon=True).start()
            threading.Thread(target=self._broadcast_loop, daemon=True).start()

        except Exception as e:
            log.error("Start failed: %s", e, exc_info=True)
            self.sio.emit("log", {"msg": f"Start failed: {e}"})
            self._cleanup()
            self._set_state(State.IDLE)

    def _do_land(self):
        """RTL both drones, wait for landing, cleanup."""
        try:
            self._set_state(State.LANDING)
            self.sio.emit("log", {"msg": "Landing..."})

            if self.leader_conn:
                self.leader_conn.set_mode("RTL")
            if self.follower_conn:
                self.follower_conn.set_mode("RTL")

            # Wait for both to descend
            deadline = time.time() + 45.0
            while self._running and time.time() < deadline:
                ld = self.leader_conn.drain_latest() if self.leader_conn else {}
                fd = self.follower_conn.drain_latest() if self.follower_conn else {}
                lp = ld.get("position")
                fp = fd.get("position")
                l_down = lp and lp["alt"] < 1.0
                f_down = fp and fp["alt"] < 1.0
                if l_down and f_down:
                    break
                time.sleep(1.0)

            self.sio.emit("log", {"msg": "Landed — shutting down SITL"})
        except Exception as e:
            log.error("Land error: %s", e)
        finally:
            self._cleanup()
            self._set_state(State.IDLE)

    def _cleanup(self):
        """Kill SITL, clear connections."""
        for conn in [self.leader_conn, self.follower_conn]:
            if conn:
                try:
                    conn.set_mode("LAND")
                except Exception:
                    pass
        time.sleep(1)
        self.launcher.kill_all()
        self.leader_conn = None
        self.follower_conn = None
        self.leader_mission = None
        self.follower_ctrl = None
        self.leader_pos = None
        self.follower_pos = None
        self.last_apf_result = None
        self._auto_mission = False
        self._manual_active = False
        self._click_active = False
        self._erratic_override = None

    # ── Control Loop (10 Hz) ───────────────────────────────────

    def _control_loop(self):
        """Main 10 Hz loop: drive leader, run follower APF, collect telemetry."""
        log.info("Control loop started")
        while self._running and self.state == State.FLYING:
            t0 = time.time()
            try:
                self._control_tick()
            except Exception as e:
                log.error("Control tick error: %s", e, exc_info=True)
            elapsed = time.time() - t0
            sleep_t = (1.0 / CONTROL_HZ) - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)
        log.info("Control loop stopped")

    def _control_tick(self):
        """One iteration of the control loop."""
        if not self.leader_conn or not self.follower_conn:
            return

        # Drain telemetry
        ld = self.leader_conn.drain_latest()
        fd = self.follower_conn.drain_latest()

        lp = ld.get("position")
        lh = ld.get("heartbeat")
        fp = fd.get("position")
        fh = fd.get("heartbeat")

        if lp:
            self.leader_pos = lp
        if lh:
            self.leader_hb = lh
        if fp:
            self.follower_pos = fp
        if fh:
            self.follower_hb = fh

        # Update follower controller with latest states
        if self.follower_ctrl:
            if self.leader_pos:
                self.follower_ctrl.update_leader_state(self.leader_pos)
            if self.follower_pos:
                self.follower_ctrl.update_own_state(self.follower_pos)

        # ── Drive leader ────────────────────────────────────
        self._drive_leader()

        # ── Apply guidance overrides ──────────────────────────
        if self.follower_ctrl and self._guidance_overrides:
            with self._lock:
                for k, v in self._guidance_overrides.items():
                    if hasattr(self.follower_ctrl.cfg, k):
                        setattr(self.follower_ctrl.cfg, k, v)

        # ── Follower guidance tick ────────────────────────────
        if self.follower_ctrl:
            from sim import follower_loop
            follower_loop.FOLLOW_OFFSET_N = self._offset_n
            follower_loop.FOLLOW_OFFSET_E = self._offset_e
            follower_loop.FOLLOW_OFFSET_D = self._offset_d

            result = self.follower_ctrl.tick()
            if result:
                self.last_apf_result = result

    def _drive_leader(self):
        """Send velocity to leader based on current control mode."""
        if not self.leader_conn:
            return

        now = time.time()

        with self._lock:
            # Priority: erratic > manual > click-waypoint > auto-mission > hover

            # Erratic scenario override
            if self._erratic_override:
                elapsed = now - self._erratic_start
                if elapsed < ERRATIC_PHASE_TIME:
                    self._send_erratic(self._erratic_override, elapsed)
                    return
                else:
                    self._erratic_override = None
                    self.sio.emit("log", {"msg": "Erratic scenario complete"})

            # Manual WASD
            if self._manual_active:
                if now < self._manual_timeout:
                    self.leader_conn.send_velocity_ned(
                        self._manual_vn, self._manual_ve, self._manual_vd)
                    return
                else:
                    self._manual_active = False
                    self.leader_conn.send_velocity_ned(0, 0, 0)

            # Click-to-waypoint
            if self._click_active and self.leader_pos:
                dn = (self._click_target_lat - self.leader_pos["lat"]) * METERS_PER_DEG_LAT
                de = ((self._click_target_lon - self.leader_pos["lon"])
                      * METERS_PER_DEG_LAT * math.cos(math.radians(self.leader_pos["lat"])))
                dist = math.sqrt(dn * dn + de * de)
                if dist < 2.0:
                    self._click_active = False
                    self.leader_conn.send_velocity_ned(0, 0, 0)
                    self.sio.emit("log", {"msg": "Waypoint reached"})
                else:
                    speed = min(LEADER_SPEED_MS, dist)
                    vn = speed * dn / dist
                    ve = speed * de / dist
                    self.leader_conn.send_velocity_ned(vn, ve, 0)
                return

            # Auto-mission
            if self._auto_mission and self.leader_mission and self.leader_pos:
                phase = self.leader_mission.tick(
                    self.leader_pos["lat"],
                    self.leader_pos["lon"],
                    self.leader_pos["alt"],
                )
                if phase == Phase.DONE:
                    self._auto_mission = False
                    self.sio.emit("log", {"msg": "Auto mission complete"})
                return

            # Default: hover
            self.leader_conn.send_velocity_ned(0, 0, 0)

    def _send_erratic(self, scenario: str, elapsed: float):
        """Send erratic velocity based on scenario type."""
        if scenario == "CHARGE":
            # Fly toward follower's expected position
            if self.leader_pos and self.follower_pos:
                dn = (self.follower_pos["lat"] - self.leader_pos["lat"]) * METERS_PER_DEG_LAT
                de = ((self.follower_pos["lon"] - self.leader_pos["lon"])
                      * METERS_PER_DEG_LAT * math.cos(math.radians(self.leader_pos["lat"])))
                dist = math.sqrt(dn * dn + de * de + 0.01)
                vn = ERRATIC_CHARGE_SPEED * dn / dist
                ve = ERRATIC_CHARGE_SPEED * de / dist
                self.leader_conn.send_velocity_ned(vn, ve, 0)
            else:
                self.leader_conn.send_velocity_ned(-ERRATIC_CHARGE_SPEED, 0, 0)

        elif scenario == "ZIGZAG":
            half = ERRATIC_ZIGZAG_PERIOD / 2.0
            cycle = elapsed % ERRATIC_ZIGZAG_PERIOD
            ve = ERRATIC_ZIGZAG_SPEED if cycle < half else -ERRATIC_ZIGZAG_SPEED
            self.leader_conn.send_velocity_ned(LEADER_SPEED_MS * 0.5, ve, 0)

        elif scenario == "SUDDEN_STOP":
            if elapsed < 5.0:
                self.leader_conn.send_velocity_ned(ERRATIC_SPRINT_SPEED, 0, 0)
            else:
                self.leader_conn.send_velocity_ned(0, 0, 0)

        elif scenario == "SPRINT":
            self.leader_conn.send_velocity_ned(0, -ERRATIC_SPRINT_SPEED, 0)

    # ── Broadcast Loop (4 Hz) ──────────────────────────────────

    def _broadcast_loop(self):
        """Emit telemetry to all connected browsers at 4 Hz."""
        log.info("Broadcast loop started")
        while self._running and self.state == State.FLYING:
            try:
                self._broadcast_tick()
            except Exception as e:
                log.error("Broadcast error: %s", e)
            time.sleep(0.25)
        log.info("Broadcast loop stopped")

    def _broadcast_tick(self):
        """Build and emit one state_update."""
        lp = self.leader_pos or {}
        fp = self.follower_pos or {}
        apf = self.last_apf_result or {}

        # Compute peer distance
        peer_dist = 0.0
        if lp.get("lat") and fp.get("lat"):
            dn = (lp["lat"] - fp["lat"]) * METERS_PER_DEG_LAT
            de = ((lp["lon"] - fp["lon"]) * METERS_PER_DEG_LAT
                  * math.cos(math.radians(fp["lat"])))
            peer_dist = math.sqrt(dn * dn + de * de)

        # Leader speed
        l_speed = 0.0
        if lp.get("vx") is not None:
            l_speed = math.sqrt(lp["vx"] ** 2 + lp["vy"] ** 2)

        # Follower speed
        f_speed = 0.0
        if fp.get("vx") is not None:
            f_speed = math.sqrt(fp["vx"] ** 2 + fp["vy"] ** 2)

        # Current control mode
        mode = "HOVER"
        with self._lock:
            if self._erratic_override:
                mode = f"ERRATIC:{self._erratic_override}"
            elif self._manual_active:
                mode = "MANUAL"
            elif self._click_active:
                mode = "WAYPOINT"
            elif self._auto_mission:
                phase = self.leader_mission.phase.value if self.leader_mission else "?"
                mode = f"AUTO:{phase}"

        payload = {
            "leader": {
                "lat": lp.get("lat", 0),
                "lon": lp.get("lon", 0),
                "alt": lp.get("alt", 0),
                "vn": lp.get("vx", 0),
                "ve": lp.get("vy", 0),
                "speed": round(l_speed, 2),
            },
            "follower": {
                "lat": fp.get("lat", 0),
                "lon": fp.get("lon", 0),
                "alt": fp.get("alt", 0),
                "vn": fp.get("vx", 0),
                "ve": fp.get("vy", 0),
                "speed": round(f_speed, 2),
            },
            "peer_dist": round(peer_dist, 2),
            "offset_err_n": round(apf.get("offset_err_n", 0), 2),
            "offset_err_e": round(apf.get("offset_err_e", 0), 2),
            "guidance": {
                "mode": apf.get("mode", "TRACKING"),
                "w_evasion": round(apf.get("w_evasion", 0), 3),
                "w_tracking": round(apf.get("w_tracking", 1), 3),
                "w_catchup": round(apf.get("w_catchup", 0), 3),
                "vn": round(apf.get("vn", 0), 2),
                "ve": round(apf.get("ve", 0), 2),
                "vd": round(apf.get("vd", 0), 2),
                "emergency": apf.get("emergency", False),
                "flags": apf.get("flags", ""),
            },
            "mode": mode,
            "state": self.state.value,
            "d_safe": self.follower_ctrl.cfg.safety_dist_m if self.follower_ctrl else 1.0,
        }

        self.sio.emit("state_update", payload)

    # ── Run ────────────────────────────────────────────────────

    def run(self):
        """Start the Flask-SocketIO server (blocking)."""
        log.info("=" * 60)
        log.info("Leader-Follower Web GCS")
        log.info("Open http://localhost:%d in your browser", self.port)
        log.info("=" * 60)
        try:
            self.sio.run(self.app, host=self.host, port=self.port,
                         allow_unsafe_werkzeug=True)
        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            self._cleanup()


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Leader-Follower Web GCS")
    parser.add_argument("--port", type=int, default=5000, help="HTTP port")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address")
    args = parser.parse_args()

    gcs = WebGCS(host=args.host, port=args.port)
    gcs.run()


if __name__ == "__main__":
    main()
