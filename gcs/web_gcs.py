import eventlet
eventlet.monkey_patch()

"""
Web-based Ground Control Station — Flask + SocketIO.
Interactive browser UI for launching, commanding, and monitoring drones.

Usage:
    python -m gcs.web_gcs [--max-drones N] [--port 5000] [--output-dir DIR]
"""

import os
import sys
import json
import time
import signal
import threading
import logging
import argparse

from flask import Flask, render_template
from flask_socketio import SocketIO

from config import (
    NUM_DRONES, GCS_PORT, GCS_HOST,
    AGENT_BASE_PORT, AGENT_PORT_STEP,
    COMMS_TIMEOUT_S, WEB_GCS_PORT,
    DOCKER_MODE, MESH_SIM_ENABLED,
    GHOST_PRUNE_TIMEOUT_S,
)
from comms.protocol import make_msg, parse_msg, encode_msg
from comms.udp_node import UDPNode
from gcs.state_collector import StateCollector
from gcs.command_dispatcher import CommandDispatcher
from gcs.flight_logger import FlightLogger
from gcs.drone_manager import DroneManager
from gcs.geo_utils import ned_to_gps
from gcs.diagnostics import run_all_tests, run_single_test
from gcs.network_viz import NetworkAggregator

log = logging.getLogger(__name__)


class WebGCS:
    """Web-based Ground Control Station with drone management."""

    def __init__(self, max_drones: int = NUM_DRONES,
                 output_dir: str = "output", web_port: int = WEB_GCS_PORT):
        self.max_drones = max_drones
        self.output_dir = output_dir
        self.web_port = web_port
        self.running = True
        self._start_time = time.time()

        # Dynamic drone tracking
        self.known_drones: set[int] = set()
        self._known_lock = threading.Lock()

        # Core components
        self.udp = UDPNode(GCS_PORT)
        self.collector = StateCollector(max_drones)
        self.dispatcher = CommandDispatcher(self.udp, max_drones, self.collector,
                                               known_drones=self.known_drones)
        self.logger = FlightLogger(num_drones=max_drones, output_dir=output_dir)
        self.drone_mgr = DroneManager(max_drones)

        # Failure injection state
        self.blocked_drones: set[int] = set()
        self._block_lock = threading.Lock()

        # Network mesh aggregator
        self.network_agg = NetworkAggregator()

        # Alert buffer (UDP thread -> broadcast thread)
        self._alert_buffer: list[dict] = []
        self._alert_lock = threading.Lock()

        # Logger lock (multi-thread writes)
        self._logger_lock = threading.Lock()

        # Flask app
        self.app = Flask(
            __name__,
            template_folder=os.path.join(os.path.dirname(__file__), "templates"),
            static_folder=os.path.join(os.path.dirname(__file__), "static"),
        )
        self.app.config["SECRET_KEY"] = "swarm-gcs"
        self.socketio = SocketIO(self.app, async_mode="eventlet",
                                 cors_allowed_origins="*")
        self._register_routes()
        self._register_socketio_handlers()

    # ── Routes ────────────────────────────────────────────

    def _register_routes(self):
        @self.app.route("/")
        def index():
            return render_template("index.html", max_drones=self.max_drones)

    # ── SocketIO handlers ─────────────────────────────────

    def _register_socketio_handlers(self):
        sio = self.socketio

        @sio.on("connect")
        def on_connect():
            log.info("Web client connected")

        @sio.on("disconnect")
        def on_disconnect():
            log.info("Web client disconnected")

        # ── Global commands ──

        @sio.on("cmd_takeoff")
        def on_takeoff(data):
            alt = float(data.get("alt", 10.0))
            with self._known_lock:
                targets = set(self.known_drones)
            self.dispatcher.takeoff_all(alt)
            with self._logger_lock:
                self.logger.log_command("TAKEOFF_ALL", {"alt": alt})
            self._emit_log("INFO", f"Takeoff all to {alt:.1f}m")

        @sio.on("cmd_land")
        def on_land(data):
            self.dispatcher.land_all()
            with self._logger_lock:
                self.logger.log_command("LAND_ALL", {})
            self._emit_log("INFO", "Land all commanded")

        @sio.on("cmd_formation")
        def on_formation(data):
            shape = data["shape"]
            heading = float(data["heading_deg"])
            spacing = float(data["spacing_m"])
            self.dispatcher.set_formation(shape, heading, spacing)
            with self._logger_lock:
                self.logger.log_command("FORMATION", data)
            self._emit_log("INFO",
                           f"Formation {shape} hdg={heading:.0f} spc={spacing:.1f}")

        @sio.on("cmd_waypoint")
        def on_waypoint(data):
            did = int(data["drone_id"])
            lat = float(data["lat"])
            lon = float(data["lon"])
            alt = float(data["alt"])
            self.dispatcher.send_waypoint(did, lat, lon, alt)
            with self._logger_lock:
                self.logger.log_command("WAYPOINT", data)
            self._emit_log("INFO",
                           f"Waypoint D{did} -> ({lat:.6f}, {lon:.6f}, {alt:.1f})")

        @sio.on("cmd_swarm_waypoint")
        def on_swarm_waypoint(data):
            lat = float(data["lat"])
            lon = float(data["lon"])
            alt = float(data.get("alt", 10.0))
            self.dispatcher.send_swarm_waypoint(lat, lon, alt)
            with self._logger_lock:
                self.logger.log_command("SWARM_WAYPOINT", data)
            self._emit_log("INFO",
                           f"Swarm waypoint -> ({lat:.6f}, {lon:.6f}, {alt:.1f})")

        # ── Per-drone commands ──

        @sio.on("cmd_takeoff_drone")
        def on_takeoff_drone(data):
            did = int(data["drone_id"])
            alt = float(data.get("alt", 10.0))
            self.dispatcher.takeoff(did, alt)
            with self._logger_lock:
                self.logger.log_command("TAKEOFF", {"drone_id": did, "alt": alt})
            self._emit_log("INFO", f"Takeoff D{did} to {alt:.1f}m")

        @sio.on("cmd_land_drone")
        def on_land_drone(data):
            did = int(data["drone_id"])
            self.dispatcher.land(did)
            with self._logger_lock:
                self.logger.log_command("LAND", {"drone_id": did})
            self._emit_log("INFO", f"Land D{did}")

        # ── NED waypoint ──

        @sio.on("cmd_waypoint_ned")
        def on_waypoint_ned(data):
            did = int(data["drone_id"])
            north = float(data["north"])
            east = float(data["east"])
            alt = float(data["alt"])
            ref = self._get_reference_position()
            if ref[0] == 0 and ref[1] == 0:
                self._emit_log("ERROR", "No reference position available")
                return
            lat, lon = ned_to_gps(ref[0], ref[1], north, east)
            self.dispatcher.send_waypoint(did, lat, lon, alt)
            with self._logger_lock:
                self.logger.log_command("WAYPOINT_NED", data)
            self._emit_log("INFO",
                f"Waypoint D{did} NED({north:.1f},{east:.1f}) "
                f"-> ({lat:.7f},{lon:.7f},{alt:.1f})")

        # ── Drone process management ──

        @sio.on("cmd_launch_drone")
        def on_launch_drone(data):
            did = int(data["drone_id"])
            result = self.drone_mgr.launch(did)
            if result["ok"]:
                with self._known_lock:
                    self.known_drones.add(did)
                sio.emit("drone_launched", {
                    "drone_id": did, "pid": result["pid"], "ts": time.time(),
                })
                self._emit_log("INFO", f"Drone {did} launched (PID {result['pid']})")
            else:
                self._emit_log("ERROR", result["error"])

        @sio.on("cmd_kill_drone")
        def on_kill(data):
            did = int(data["drone_id"])
            result = self.drone_mgr.kill(did)
            if result["ok"]:
                sio.emit("drone_killed", {
                    "drone_id": did, "ts": time.time(),
                })
                self._emit_log("WARN", f"Drone {did} killed")
            else:
                self._emit_log("WARN", result["error"])

        @sio.on("cmd_launch_all")
        def on_launch_all(data):
            self._emit_log("INFO", f"Launching all drones (1-{self.max_drones})...")
            sio.start_background_task(self._staggered_launch_all)

        @sio.on("cmd_kill_all")
        def on_kill_all(data):
            self.drone_mgr.kill_all()
            self._emit_log("WARN", "All drones killed")

        # ── Failure injection ──

        @sio.on("cmd_block_comms")
        def on_block(data):
            did = int(data["drone_id"])
            with self._block_lock:
                self.blocked_drones.add(did)
            self._emit_log("WARN", f"Comms BLOCKED for drone {did}")

        @sio.on("cmd_restore_comms")
        def on_restore(data):
            did = int(data["drone_id"])
            with self._block_lock:
                self.blocked_drones.discard(did)
            self._emit_log("INFO", f"Comms RESTORED for drone {did}")

        # ── Diagnostics ──

        @sio.on("run_all_tests")
        def on_run_all_tests(data):
            self._emit_log("INFO", "Running all diagnostics...")
            sio.start_background_task(self._run_diagnostics_all)

        @sio.on("run_test")
        def on_run_test(data):
            test_id = data.get("test_id", "")
            self._emit_log("INFO", f"Running test: {test_id}")
            sio.start_background_task(self._run_diagnostics_single, test_id)

        # ── Mesh network commands ──

        @sio.on("cmd_set_isolation_radius")
        def on_set_isolation_radius(data):
            radius = float(data.get("radius_m", 5.0))
            config_data = {"isolation_radius_m": radius}
            with self._known_lock:
                peers = set(self.known_drones)
            for i in peers:
                port = AGENT_BASE_PORT + i * AGENT_PORT_STEP
                self.udp.send(
                    make_msg("SAFETY_CONFIG_CMD", 0, config_data),
                    GCS_HOST, port)
            self._emit_log("INFO", f"Isolation radius set to {radius:.1f}m")

        @sio.on("cmd_set_mesh_range")
        def on_set_mesh_range(data):
            range_m = float(data.get("range_m", 100))
            config_data = {"range_m": range_m}
            with self._known_lock:
                peers = set(self.known_drones)
            for i in peers:
                port = AGENT_BASE_PORT + i * AGENT_PORT_STEP
                self.udp.send(
                    make_msg("MESH_CONFIG_CMD", 0, config_data),
                    GCS_HOST, port)
            self._emit_log("INFO", f"Mesh range set to {range_m:.0f}m")

        # ── RL Mode ──

        @sio.on("cmd_rl_mode")
        def on_rl_mode(data):
            enable = bool(data.get("enable", False))
            target = data.get("target_id", 0)  # 0 = all drones

            cmd_data = {"enable": enable, "target_id": target}
            if "goal_ned" in data:
                cmd_data["goal_ned"] = data["goal_ned"]

            if target == 0:
                with self._known_lock:
                    peers = set(self.known_drones)
                for i in peers:
                    cd = dict(cmd_data)
                    cd["target_id"] = i
                    port = AGENT_BASE_PORT + i * AGENT_PORT_STEP
                    self.udp.send(
                        make_msg("RL_MODE_CMD", 0, cd),
                        GCS_HOST, port)
            else:
                port = AGENT_BASE_PORT + target * AGENT_PORT_STEP
                self.udp.send(
                    make_msg("RL_MODE_CMD", 0, cmd_data),
                    GCS_HOST, port)

            action = "ENABLED" if enable else "DISABLED"
            target_str = f"D{target}" if target > 0 else "ALL"
            with self._logger_lock:
                self.logger.log_command("RL_MODE", data)
            self._emit_log("INFO", f"RL mode {action} for {target_str}")

    # ── Background threads ────────────────────────────────

    def _staggered_launch_all(self):
        """Launch all drones with 2s stagger.
        In Docker mode, drones already run as containers — just register them."""
        if DOCKER_MODE:
            self._emit_log("INFO",
                           "Docker mode: drones managed by docker-compose, "
                           "registering as known...")
            for i in range(1, self.max_drones + 1):
                with self._known_lock:
                    self.known_drones.add(i)
                self.socketio.emit("drone_launched", {
                    "drone_id": i, "pid": "docker", "ts": time.time(),
                })
            self._emit_log("INFO",
                           f"Registered {self.max_drones} drones "
                           "(auto-discovered via STATE_REPORT)")
            return

        for i in range(1, self.max_drones + 1):
            if not self.running:
                break
            result = self.drone_mgr.launch(i)
            if result["ok"]:
                with self._known_lock:
                    self.known_drones.add(i)
                self.socketio.emit("drone_launched", {
                    "drone_id": i, "pid": result["pid"], "ts": time.time(),
                })
                self._emit_log("INFO", f"Drone {i} launched (PID {result['pid']})")
            else:
                self._emit_log("WARN", f"Drone {i}: {result['error']}")
            if i < self.max_drones:
                time.sleep(2)

    def _run_diagnostics_all(self):
        """Run all diagnostic tests in background and emit results."""
        try:
            results = run_all_tests(collector=self.collector)
            self.socketio.emit("test_results", {"results": results})
            passed = sum(1 for r in results if r["status"] == "pass")
            self._emit_log("INFO",
                           f"Diagnostics complete: {passed}/{len(results)} passed")
        except Exception as e:
            self.socketio.emit("test_results", {"results": [], "error": str(e)})
            self._emit_log("ERROR", f"Diagnostics error: {e}")

    def _run_diagnostics_single(self, test_id: str):
        """Run one diagnostic test in background and emit result."""
        try:
            result = run_single_test(test_id, collector=self.collector)
            self.socketio.emit("test_result", {"result": result})
            self._emit_log("INFO",
                f"Test '{result.get('test_name', test_id)}': "
                f"{result['status'].upper()} ({result.get('duration_ms', 0):.0f}ms)")
        except Exception as e:
            self.socketio.emit("test_result", {
                "result": {"test_id": test_id, "status": "error",
                           "details": {"error": str(e)}}
            })
            self._emit_log("ERROR", f"Test error: {e}")

    def _udp_loop(self):
        """Receive drone states, relay to peers, update collector.
        Exception-safe: any per-message error is logged and skipped,
        keeping the relay alive for all other drones."""
        while self.running:
            try:
                for raw, addr in self.udp.recv_all():
                    try:
                        msg = parse_msg(raw)
                        if msg is None:
                            continue

                        if msg["type"] == "STATE_REPORT":
                            src = msg["data"].get("drone_id", msg["src"])
                            # Auto-discover drones
                            with self._known_lock:
                                self.known_drones.add(src)
                            # Extract mesh stats before storing
                            mesh_stats = msg["data"].pop("mesh_stats", None)
                            self.network_agg.update(src, mesh_stats)
                            self.collector.update(src, msg["data"],
                                                  msg.get("ts", time.time()))
                            self._relay_to_peers(msg, src)
                            with self._logger_lock:
                                self.logger.log_state(src, msg["data"])

                        elif msg["type"] in ("SLOT_BID", "SLOT_TIEBREAK",
                                               "SLOT_CONFIRM"):
                            # Relay negotiation messages to all peers
                            self._relay_to_peers(msg, msg["src"])

                        elif msg["type"] == "PROXIMITY_ALERT":
                            self.socketio.emit("proximity_alert", {
                                "drone_id": msg["src"],
                                **msg["data"],
                                "ts": time.time(),
                            })

                        elif msg["type"] == "ALERT":
                            self._handle_alert(msg)
                            with self._logger_lock:
                                self.logger.log_alert(msg["src"], msg["data"])

                    except Exception as e:
                        log.warning("Error processing UDP message from %s: %s", addr, e)

            except Exception as e:
                log.error("UDP loop outer error: %s", e, exc_info=True)

            time.sleep(0.01)

    def _broadcast_loop(self):
        """Push state snapshots to web clients at 4Hz."""
        while self.running:
            time.sleep(0.25)

            # Prune ghost drones that have been silent too long
            pruned = self.collector.prune_stale(GHOST_PRUNE_TIMEOUT_S)
            if pruned:
                with self._known_lock:
                    for did in pruned:
                        self.known_drones.discard(did)
                active_ids = set(self.collector.get_all_states().keys())
                self.network_agg.prune(active_ids)
                log.info("Pruned ghost drones: %s", pruned)

            states = self.collector.get_all_states()
            stale = self.collector.get_stale_drones(COMMS_TIMEOUT_S)
            process_status = self.drone_mgr.get_all_status()

            with self._block_lock:
                blocked = list(self.blocked_drones)

            with self._alert_lock:
                alerts = self._alert_buffer
                self._alert_buffer = []

            with self._known_lock:
                known = sorted(self.known_drones)

            payload = {
                "drones": {str(k): v for k, v in states.items()},
                "stale": stale,
                "blocked": blocked,
                "process_status": process_status,
                "known_drones": known,
                "elapsed_s": round(time.time() - self._start_time, 1),
                "ts": time.time(),
            }
            self.socketio.emit("state_update", payload)

            # Network topology update (mesh sim)
            net_payload = self.network_agg.get_topology_payload(stale_ids=stale)
            if net_payload["links"] or net_payload["nodes"]:
                self.socketio.emit("network_update", net_payload)

            for alert in alerts:
                self.socketio.emit("alert", alert)

    def _relay_to_peers(self, msg: dict, source_id: int):
        """Relay STATE_REPORT to peer drones, respecting comms blocks.
        With P2P mesh enabled, this relay is redundant — peers get direct
        PEER_HEARTBEATs. Kept active for resilience."""
        relay_msg = dict(msg)
        relay_msg["src"] = 0  # Mark as GCS so agents count as heartbeat
        raw = encode_msg(relay_msg)

        with self._block_lock:
            blocked = set(self.blocked_drones)

        with self._known_lock:
            peers = set(self.known_drones)

        targets = []
        for i in peers:
            if i == source_id:
                continue
            if i in blocked or source_id in blocked:
                continue
            targets.append((GCS_HOST, AGENT_BASE_PORT + i * AGENT_PORT_STEP))

        if targets:
            self.udp.send_to_multiple(raw, targets)

    def _handle_alert(self, msg: dict):
        """Process an ALERT from a drone."""
        d = msg["data"]
        alert = {
            "drone_id": msg["src"],
            "level": d.get("level", "WARN"),
            "code": d.get("code", "UNKNOWN"),
            "message": d.get("message", ""),
            "action_taken": d.get("action_taken", ""),
            "ts": time.time(),
        }
        with self._alert_lock:
            self._alert_buffer.append(alert)
        log.warning("ALERT D%d: [%s] %s", msg["src"],
                    d.get("code"), d.get("message"))

    def _emit_log(self, level: str, message: str):
        """Send a log event to all web clients."""
        self.socketio.emit("log_event", {
            "level": level,
            "message": message,
            "ts": time.time(),
        })

    def _get_reference_position(self) -> tuple[float, float, float]:
        """Get centroid of all drones for NED-to-GPS conversion."""
        centroid_pos = self.collector.get_centroid()
        if centroid_pos:
            return centroid_pos
        return (0.0, 0.0, 10.0)

    # ── Run / Stop ────────────────────────────────────────

    def run(self):
        """Start background threads and run Flask-SocketIO server."""
        if DOCKER_MODE:
            log.info("Docker mode detected — drones managed by docker-compose")
        self.socketio.start_background_task(self._udp_loop)
        self.socketio.start_background_task(self._broadcast_loop)

        log.info("Web GCS starting on http://0.0.0.0:%d", self.web_port)
        self.socketio.run(self.app, host="0.0.0.0", port=self.web_port,
                          debug=False, use_reloader=False)

    def stop(self):
        if not self.running:
            return
        self.running = False
        try:
            self.drone_mgr.kill_all()
        except Exception as e:
            log.error("Error killing drones: %s", e)
        try:
            with self._logger_lock:
                self.logger.save_metadata()
                self.logger.close()
        except Exception as e:
            log.error("Error closing logger: %s", e)
        try:
            self.udp.close()
        except Exception as e:
            log.error("Error closing UDP: %s", e)
        log.info("Web GCS stopped")


# ── Entry point ───────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Web-based Swarm GCS")
    parser.add_argument("--max-drones", type=int, default=NUM_DRONES)
    parser.add_argument("--port", type=int, default=WEB_GCS_PORT)
    parser.add_argument("--output-dir", type=str, default="output")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="[WebGCS] %(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    os.makedirs(args.output_dir, exist_ok=True)

    gcs = WebGCS(
        max_drones=args.max_drones,
        output_dir=args.output_dir,
        web_port=args.port,
    )

    def shutdown(sig, frame):
        log.info("Caught signal %d, shutting down", sig)
        gcs.stop()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    try:
        gcs.run()
    except KeyboardInterrupt:
        pass
    finally:
        gcs.stop()


if __name__ == "__main__":
    main()
