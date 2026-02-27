"""
Ground Control Station — main process.
Collects drone states, dispatches commands, relays peer info, visualizes.
Records flight logs and session metadata. Can save GIF output.

Usage:
    python -m gcs.gcs [--no-viz] [--headless] [--gif]

CLI commands:
    takeoff <alt>                   — All drones takeoff to <alt> m
    takeoff <id> <alt>              — Drone <id> takeoff to <alt> m
    land                            — All drones land
    land <id>                       — Drone <id> land
    formation <shape> <hdg> <spc>   — Set formation (LINE/V/COLUMN/DIAMOND)
    waypoint <id> <lat> <lon> <alt> — Send waypoint to drone <id>
    velocity <id> <vn> <ve> <vd>    — Send velocity to drone <id>
    status                          — Print all drone states
    quit                            — Shutdown GCS
"""

import time
import signal
import logging
import traceback
import threading

from config import (
    NUM_DRONES, GCS_PORT, GCS_HOST,
    AGENT_BASE_PORT, AGENT_PORT_STEP,
    COMMS_TIMEOUT_S, GCS_LOOP_HZ,
    GHOST_PRUNE_TIMEOUT_S,
)
from comms.protocol import parse_msg, encode_msg
from comms.udp_node import UDPNode
from gcs.state_collector import StateCollector
from gcs.command_dispatcher import CommandDispatcher
from gcs.visualizer import SwarmVisualizer
from gcs.flight_logger import FlightLogger

log = logging.getLogger(__name__)


class GCS:
    """Ground Control Station main class."""

    def __init__(self, num_drones: int = NUM_DRONES, enable_viz: bool = True,
                 headless: bool = False, save_gif: bool = False,
                 orchestrator_port: int = 0, output_dir: str = ""):
        self.num_drones = num_drones
        self.running = True
        self.save_gif = save_gif
        self.orchestrator_port = orchestrator_port

        self.udp = UDPNode(GCS_PORT)
        self.collector = StateCollector(num_drones)
        self.dispatcher = CommandDispatcher(self.udp, num_drones, self.collector)
        kwargs = {"num_drones": num_drones, "enabled": enable_viz,
                  "headless": headless}
        if output_dir:
            kwargs["output_dir"] = output_dir
        self.viz = SwarmVisualizer(**kwargs)
        kwargs_log = {"num_drones": num_drones}
        if output_dir:
            kwargs_log["output_dir"] = output_dir
        self.logger = FlightLogger(**kwargs_log)

        self._loop_period = 1.0 / GCS_LOOP_HZ
        self._last_stale_warning: dict[int, float] = {}
        if orchestrator_port:
            log.info("GCS started on port %d for %d drones (relay → orchestrator:%d)",
                     GCS_PORT, num_drones, orchestrator_port)
        else:
            log.info("GCS started on port %d for %d drones", GCS_PORT, num_drones)

    def run(self):
        """Main GCS loop."""
        # Start CLI only if stdin is a real terminal (not piped/subprocess)
        import sys
        if sys.stdin.isatty():
            cli = threading.Thread(target=self._cli_loop, daemon=True)
            cli.start()

        while self.running:
            try:
                t0 = time.time()

                # 1. RECEIVE — collect state reports and alerts
                for raw, addr in self.udp.recv_all():
                    msg = parse_msg(raw)
                    if msg is None:
                        continue

                    if msg["type"] == "STATE_REPORT":
                        self.collector.update(msg["src"], msg["data"], msg.get("ts", time.time()))
                        self._relay_to_peers(msg, msg["src"])
                        self.logger.log_state(msg["src"], msg["data"])

                    elif msg["type"] == "ALERT":
                        self._handle_alert(msg)
                        self.logger.log_alert(msg["src"], msg["data"])

                # 2. CHECK — detect drones that stopped reporting (throttled)
                now = time.time()
                stale = self.collector.get_stale_drones(COMMS_TIMEOUT_S)
                for did in stale:
                    last = self._last_stale_warning.get(did, 0)
                    if now - last >= 10.0:
                        log.warning("Drone %d: no contact (stale)", did)
                        self._last_stale_warning[did] = now

                # 2b. PRUNE — remove ghost drones
                pruned = self.collector.prune_stale(GHOST_PRUNE_TIMEOUT_S)
                for did in pruned:
                    self._last_stale_warning.pop(did, None)
                    log.info("Pruned ghost drone %d", did)

                # 3. VISUALIZE
                self.viz.update(self.collector.get_all_states())

                # 4. SLEEP
                elapsed = time.time() - t0
                sleep_time = self._loop_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
            except Exception:
                log.error("GCS loop error:\n%s", traceback.format_exc())

    def stop(self):
        if not self.running:
            return  # Already stopped
        self.running = False
        # Save outputs
        if self.save_gif:
            self.viz.save_gif()
        self.viz.save_final_frame()
        self.logger.save_metadata()
        self.logger.close()
        self.udp.close()
        log.info("GCS stopped")

    # ── Internal ───────────────────────────────────────────

    def _relay_to_peers(self, msg: dict, source_id: int):
        """Relay a STATE_REPORT to all other drone agents for peer awareness.
        Rewrites src to 0 (GCS) so it counts as a GCS heartbeat.
        Only relays to drones that have reported state (avoids blind sends).
        Also relays to orchestrator if configured."""
        relay_msg = dict(msg)
        relay_msg["src"] = 0  # Mark as from GCS so agents update their heartbeat
        raw = encode_msg(relay_msg)
        for i in list(self.collector.get_all_states().keys()):
            if i != source_id:
                port = AGENT_BASE_PORT + i * AGENT_PORT_STEP
                self.udp.send(raw, GCS_HOST, port)
        # Relay to orchestrator (isolated demo mode)
        if self.orchestrator_port:
            self.udp.send(raw, GCS_HOST, self.orchestrator_port)

    def _handle_alert(self, msg: dict):
        """Log alerts from drones."""
        d = msg["data"]
        log.warning(
            "ALERT from drone %d: [%s] %s — action: %s",
            msg["src"], d.get("code"), d.get("message"), d.get("action_taken"),
        )

    def _cli_loop(self):
        """Interactive command line for the operator."""
        print("\n=== Swarm GCS ===")
        print("Type 'help' for available commands.\n")

        while self.running:
            try:
                line = input("gcs> ").strip()
                if not line:
                    continue
                self._dispatch_cli(line)
            except (EOFError, KeyboardInterrupt):
                self.running = False
                break
            except Exception as e:
                print(f"Error: {e}")

    def _dispatch_cli(self, line: str):
        """Parse and dispatch a CLI command."""
        parts = line.split()
        cmd = parts[0].lower()

        if cmd == "help":
            print("Commands:")
            print("  takeoff <alt>                   — All drones takeoff")
            print("  takeoff <id> <alt>              — Single drone takeoff")
            print("  land                            — All drones land")
            print("  land <id>                       — Single drone land")
            print("  formation <shape> <hdg> <spc>   — Set formation")
            print("    shapes: LINE, V, COLUMN, DIAMOND")
            print("  waypoint <id> <lat> <lon> <alt> — Send waypoint")
            print("  velocity <id> <vn> <ve> <vd>    — Send velocity")
            print("  status                          — Print drone states")
            print("  quit                            — Shutdown")

        elif cmd == "takeoff":
            if len(parts) == 2:
                self.dispatcher.takeoff_all(float(parts[1]))
                self.logger.log_command("TAKEOFF_ALL", {"alt": float(parts[1])})
            elif len(parts) == 3:
                self.dispatcher.takeoff(int(parts[1]), float(parts[2]))
                self.logger.log_command("TAKEOFF", {"id": int(parts[1]), "alt": float(parts[2])})
            else:
                print("Usage: takeoff <alt> OR takeoff <id> <alt>")

        elif cmd == "land":
            if len(parts) == 1:
                self.dispatcher.land_all()
                self.logger.log_command("LAND_ALL", {})
            elif len(parts) == 2:
                self.dispatcher.land(int(parts[1]))
                self.logger.log_command("LAND", {"id": int(parts[1])})

        elif cmd == "formation":
            if len(parts) == 4:
                self.dispatcher.set_formation(parts[1], float(parts[2]), float(parts[3]))
                self.logger.log_command("FORMATION", {
                    "shape": parts[1], "heading": float(parts[2]),
                    "spacing": float(parts[3]),
                })
            else:
                print("Usage: formation <shape> <heading_deg> <spacing_m>")
                print("  shapes: LINE, V, COLUMN, DIAMOND")

        elif cmd == "waypoint":
            if len(parts) == 5:
                self.dispatcher.send_waypoint(
                    int(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]))
                self.logger.log_command("WAYPOINT", {
                    "id": int(parts[1]), "lat": float(parts[2]),
                    "lon": float(parts[3]), "alt": float(parts[4]),
                })
            else:
                print("Usage: waypoint <id> <lat> <lon> <alt>")

        elif cmd == "velocity":
            if len(parts) == 5:
                self.dispatcher.send_velocity(
                    int(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]))
                self.logger.log_command("VELOCITY", {
                    "id": int(parts[1]), "vn": float(parts[2]),
                    "ve": float(parts[3]), "vd": float(parts[4]),
                })
            else:
                print("Usage: velocity <id> <vn> <ve> <vd>")

        elif cmd == "status":
            print("=== Drone Status ===")
            self.collector.print_summary()

        elif cmd == "quit":
            self.running = False

        else:
            print(f"Unknown command: {cmd}. Type 'help' for commands.")


# ── Entry point ────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Swarm GCS")
    parser.add_argument("--no-viz", action="store_true", help="Disable visualization")
    parser.add_argument("--headless", action="store_true", help="Headless mode (Agg backend)")
    parser.add_argument("--gif", action="store_true", help="Save GIF output")
    parser.add_argument("--num-drones", type=int, default=NUM_DRONES)
    parser.add_argument("--orchestrator-port", type=int, default=0,
                        help="UDP port to relay states to orchestrator (0=disabled)")
    parser.add_argument("--output-dir", type=str, default="",
                        help="Directory for output files (GIF, logs, metadata)")
    args = parser.parse_args()

    enable_viz = not args.no_viz
    headless = args.headless
    save_gif = args.gif or headless

    logging.basicConfig(
        level=logging.INFO,
        format="[GCS] %(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    gcs = GCS(args.num_drones, enable_viz=enable_viz,
              headless=headless, save_gif=save_gif,
              orchestrator_port=args.orchestrator_port,
              output_dir=args.output_dir)

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
