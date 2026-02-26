"""
Automated demo — launches the full swarm, runs a scripted mission,
captures GIF, flight logs, and metadata, then shuts down.

Usage:
    python -m scripts.run_demo [--num-drones N] [--skip-sitl]

Each drone is launched as a self-contained subprocess (drones/drone_N/run.py)
that manages its own SITL instance and agent. The GCS runs inline.

Mission sequence:
  1. Launch drone subprocesses (each starts own SITL + agent)
  2. Start GCS (headless, GIF capture)
  3. Wait for all drones to report
  4. Takeoff all to 10m
  5. Wait for stable altitude
  6. Form V-shape → hold 20s
  7. Form LINE → hold 15s
  8. Form DIAMOND → hold 15s
  9. Land all → wait 30s
  10. Save GIF + logs + metadata
  11. Shutdown everything
"""

import os
import sys
import time
import json
import signal
import subprocess
import logging
import argparse

# Add project root to path
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_DIR)

from config import (
    NUM_DRONES, SITL_BASE_PORT, SITL_PORT_STEP,
    GCS_HOST, GCS_PORT, AGENT_BASE_PORT, AGENT_PORT_STEP,
    LOG_DIR,
)
from comms.protocol import make_msg, parse_msg
from comms.udp_node import UDPNode
from gcs.state_collector import StateCollector
from gcs.visualizer import SwarmVisualizer
from gcs.flight_logger import FlightLogger

log = logging.getLogger("demo")


def wait_for_sitl_port(drone_id: int, timeout: float = 90.0) -> bool:
    """Wait for a SITL TCP port to become reachable."""
    import socket
    port = SITL_BASE_PORT + drone_id * SITL_PORT_STEP
    start = time.time()
    while time.time() - start < timeout:
        try:
            s = socket.create_connection(("127.0.0.1", port), timeout=2)
            s.close()
            return True
        except (ConnectionRefusedError, OSError):
            time.sleep(1)
    return False


def send_to_drone(udp: UDPNode, drone_id: int, msg_type: str, data: dict):
    """Send a command to a specific drone agent."""
    raw = make_msg(msg_type, 0, data)
    port = AGENT_BASE_PORT + drone_id * AGENT_PORT_STEP
    udp.send(raw, GCS_HOST, port)


def send_to_all(udp: UDPNode, num_drones: int, msg_type: str, data: dict):
    """Send a command to all drone agents."""
    for i in range(1, num_drones + 1):
        send_to_drone(udp, i, msg_type, data)


def relay_to_peers(udp: UDPNode, msg: dict, source_id: int, num_drones: int):
    """Relay state report to all other drones. Src=0 so it counts as GCS heartbeat."""
    relay_msg = dict(msg)
    relay_msg["src"] = 0
    raw = json.dumps(relay_msg).encode("utf-8")
    for i in range(1, num_drones + 1):
        if i != source_id:
            port = AGENT_BASE_PORT + i * AGENT_PORT_STEP
            udp.send(raw, GCS_HOST, port)


def collect_states(udp: UDPNode, collector: StateCollector,
                   viz: SwarmVisualizer, flight_logger: FlightLogger,
                   num_drones: int, duration_s: float):
    """Run the GCS receive loop for duration_s seconds."""
    end_time = time.time() + duration_s
    last_keepalive = 0.0
    while time.time() < end_time:
        for raw, addr in udp.recv_all():
            msg = parse_msg(raw)
            if msg is None:
                continue
            if msg["type"] == "STATE_REPORT":
                collector.update(msg["src"], msg["data"], msg.get("ts", time.time()))
                relay_to_peers(udp, msg, msg["src"], num_drones)
                flight_logger.log_state(msg["src"], msg["data"])
            elif msg["type"] == "ALERT":
                d = msg["data"]
                log.warning("ALERT D%d: [%s] %s", msg["src"], d.get("code"), d.get("message"))
                flight_logger.log_alert(msg["src"], msg["data"])

        # Send periodic keepalive to all agents (counts as GCS heartbeat)
        now = time.time()
        if now - last_keepalive > 1.0:
            keepalive = make_msg("STATE_REPORT", 0, {"keepalive": True})
            for i in range(1, num_drones + 1):
                port = AGENT_BASE_PORT + i * AGENT_PORT_STEP
                udp.send(keepalive, GCS_HOST, port)
            last_keepalive = now

        viz.update(collector.get_all_states())
        time.sleep(0.1)


def wait_for_reports(udp, collector, viz, logger, num_drones, timeout=120):
    """Wait until all drones have reported at least once."""
    log.info("Waiting for all %d drones to report...", num_drones)
    start = time.time()
    while len(collector.states) < num_drones:
        collect_states(udp, collector, viz, logger, num_drones, 1.0)
        elapsed = time.time() - start
        if int(elapsed) % 5 == 0:
            log.info("  %d/%d drones reporting (%.0fs)", len(collector.states), num_drones, elapsed)
        if elapsed > timeout:
            log.warning("Timeout waiting for all drones — continuing with %d", len(collector.states))
            break


def wait_for_alt(udp, collector, viz, logger, num_drones, target_alt, tolerance=2.0, timeout=60):
    """Wait until all drones are near target altitude."""
    log.info("Waiting for drones to reach %.1f m...", target_alt)
    start = time.time()
    while True:
        collect_states(udp, collector, viz, logger, num_drones, 1.0)
        states = collector.get_all_states()
        if states:
            alts = [s.get("alt", 0) for s in states.values()]
            min_alt = min(alts)
            log.info("  Altitudes: %s (min=%.1f, target=%.1f)",
                     [f"{a:.1f}" for a in alts], min_alt, target_alt)
            if min_alt >= target_alt - tolerance:
                log.info("  All drones at altitude!")
                return
        if time.time() - start > timeout:
            log.warning("  Timeout — continuing")
            return


def main():
    parser = argparse.ArgumentParser(description="Run automated swarm demo")
    parser.add_argument("--num-drones", type=int, default=NUM_DRONES)
    parser.add_argument("--skip-sitl", action="store_true",
                        help="Skip SITL launch (agents only, SITL already running)")
    args = parser.parse_args()

    num = args.num_drones

    logging.basicConfig(
        level=logging.INFO,
        format="[DEMO] %(asctime)s %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    os.makedirs(LOG_DIR, exist_ok=True)
    output_dir = os.path.join(PROJECT_DIR, "output")
    os.makedirs(output_dir, exist_ok=True)

    drone_procs = []
    agent_procs = []  # Only used in --skip-sitl mode
    udp = None

    try:
        if not args.skip_sitl:
            # ═══════════════════════════════════════════════════
            # PHASE 1: Launch drone subprocesses (each runs own SITL + agent)
            # ═══════════════════════════════════════════════════
            log.info("═══ Phase 1: Launching %d drone systems ═══", num)
            for i in range(1, num + 1):
                run_script = os.path.join(PROJECT_DIR, "drones", f"drone_{i}", "run.py")
                log_file = open(os.path.join(LOG_DIR, f"drone_{i}.log"), "w")
                p = subprocess.Popen(
                    [sys.executable, run_script],
                    cwd=PROJECT_DIR,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                )
                drone_procs.append(p)
                log.info("  Drone %d launched (pid %d) — %s", i, p.pid, run_script)
                time.sleep(5)  # Stagger to avoid SITL build races

            # Wait for all SITL ports
            log.info("Waiting for all SITL instances...")
            for i in range(1, num + 1):
                if wait_for_sitl_port(i):
                    log.info("  SITL %d ready (port %d)", i, SITL_BASE_PORT + i * SITL_PORT_STEP)
                else:
                    log.error("  SITL %d FAILED", i)
                    raise RuntimeError(f"SITL {i} did not start")
        else:
            # ═══════════════════════════════════════════════════
            # PHASE 1 (skip-sitl): Launch agent-only subprocesses
            # ═══════════════════════════════════════════════════
            log.info("═══ Phase 1: Launching %d agents (--skip-sitl) ═══", num)
            for i in range(1, num + 1):
                log_file = open(os.path.join(LOG_DIR, f"agent_{i}.log"), "w")
                p = subprocess.Popen(
                    [sys.executable, "-m", "drone_agent.agent", str(i)],
                    cwd=PROJECT_DIR,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                )
                agent_procs.append(p)
                log.info("  Agent %d started (pid %d)", i, p.pid)
                time.sleep(0.5)

        # ═══════════════════════════════════════════════════
        # PHASE 2: Start GCS components (inline, not subprocess)
        # ═══════════════════════════════════════════════════
        log.info("═══ Phase 2: Starting GCS (headless + GIF) ═══")
        udp = UDPNode(GCS_PORT)
        collector = StateCollector(num)
        viz = SwarmVisualizer(num, enabled=True, headless=True, output_dir=output_dir)
        flight_logger = FlightLogger(num, output_dir=output_dir)

        # Wait for all drones to start reporting
        wait_for_reports(udp, collector, viz, flight_logger, num, timeout=120)

        # ═══════════════════════════════════════════════════
        # PHASE 3: Mission — Takeoff
        # ═══════════════════════════════════════════════════
        target_alt = 10.0
        log.info("═══ Phase 3: Takeoff to %.1f m ═══", target_alt)
        send_to_all(udp, num, "TAKEOFF_CMD", {"target_id": 0, "alt": target_alt})
        flight_logger.log_command("TAKEOFF_ALL", {"alt": target_alt})

        wait_for_alt(udp, collector, viz, flight_logger, num, target_alt, timeout=90)
        # Extra settle time
        collect_states(udp, collector, viz, flight_logger, num, 5.0)

        # ═══════════════════════════════════════════════════
        # PHASE 4: Formation — V shape
        # ═══════════════════════════════════════════════════
        log.info("═══ Phase 4: Formation V ═══")
        centroid = collector.get_centroid()
        if centroid:
            ref_lat, ref_lon, ref_alt = centroid
        else:
            ref_lat, ref_lon, ref_alt = -35.3632620, 149.1652370, target_alt
        valid_ids = collector.get_valid_drone_ids()
        leader_id = min(valid_ids) if valid_ids else 1

        formation_data = {
            "formation": "V",
            "leader_id": leader_id,
            "ref_lat": ref_lat,
            "ref_lon": ref_lon,
            "ref_alt": ref_alt,
            "heading_deg": 0.0,
            "spacing_m": 8.0,
        }
        send_to_all(udp, num, "FORMATION_CMD", formation_data)
        flight_logger.log_command("FORMATION", formation_data)

        log.info("  Holding V formation for 20s...")
        collect_states(udp, collector, viz, flight_logger, num, 20.0)

        # ═══════════════════════════════════════════════════
        # PHASE 5: Formation — LINE
        # ═══════════════════════════════════════════════════
        log.info("═══ Phase 5: Formation LINE ═══")
        centroid = collector.get_centroid()
        if centroid:
            ref_lat, ref_lon, ref_alt = centroid
        valid_ids = collector.get_valid_drone_ids()
        leader_id = min(valid_ids) if valid_ids else leader_id

        formation_data = {
            "formation": "LINE",
            "leader_id": leader_id,
            "ref_lat": ref_lat,
            "ref_lon": ref_lon,
            "ref_alt": ref_alt,
            "heading_deg": 90.0,
            "spacing_m": 6.0,
        }
        send_to_all(udp, num, "FORMATION_CMD", formation_data)
        flight_logger.log_command("FORMATION", formation_data)

        log.info("  Holding LINE formation for 15s...")
        collect_states(udp, collector, viz, flight_logger, num, 15.0)

        # ═══════════════════════════════════════════════════
        # PHASE 6: Formation — DIAMOND
        # ═══════════════════════════════════════════════════
        log.info("═══ Phase 6: Formation DIAMOND ═══")
        centroid = collector.get_centroid()
        if centroid:
            ref_lat, ref_lon, ref_alt = centroid
        valid_ids = collector.get_valid_drone_ids()
        leader_id = min(valid_ids) if valid_ids else leader_id

        formation_data = {
            "formation": "DIAMOND",
            "leader_id": leader_id,
            "ref_lat": ref_lat,
            "ref_lon": ref_lon,
            "ref_alt": ref_alt,
            "heading_deg": 45.0,
            "spacing_m": 7.0,
        }
        send_to_all(udp, num, "FORMATION_CMD", formation_data)
        flight_logger.log_command("FORMATION", formation_data)

        log.info("  Holding DIAMOND formation for 15s...")
        collect_states(udp, collector, viz, flight_logger, num, 15.0)

        # ═══════════════════════════════════════════════════
        # PHASE 7: Land
        # ═══════════════════════════════════════════════════
        log.info("═══ Phase 7: Landing ═══")
        send_to_all(udp, num, "LAND_CMD", {"target_id": 0})
        flight_logger.log_command("LAND_ALL", {})

        log.info("  Waiting for landing (30s)...")
        collect_states(udp, collector, viz, flight_logger, num, 30.0)

        # ═══════════════════════════════════════════════════
        # PHASE 8: Save outputs
        # ═══════════════════════════════════════════════════
        log.info("═══ Phase 8: Saving outputs ═══")

        gif_path = viz.save_gif("swarm_demo.gif", fps=5.0)
        png_path = viz.save_final_frame("swarm_final.png")
        meta_path = flight_logger.save_metadata({
            "mission": "automated_demo",
            "formations": ["V", "LINE", "DIAMOND"],
            "target_altitude_m": target_alt,
        })
        flight_logger.close()

        log.info("═══ Demo Complete ═══")
        log.info("Outputs in: %s", output_dir)
        if gif_path:
            log.info("  GIF:      %s", gif_path)
        if png_path:
            log.info("  Final:    %s", png_path)
        log.info("  Metadata: %s", meta_path)
        log.info("  Logs:     %s/flight_log.csv", output_dir)
        for i in range(1, num + 1):
            log.info("            %s/drone_%d_log.csv", output_dir, i)

    except KeyboardInterrupt:
        log.info("Demo interrupted by user")
    except Exception as e:
        log.error("Demo failed: %s", e, exc_info=True)
    finally:
        # ── Cleanup ────────────────────────────────────────
        log.info("Cleaning up...")
        if udp:
            udp.close()

        # Kill drone subprocesses (each manages its own SITL + agent)
        for p in drone_procs:
            if p.poll() is None:
                p.terminate()
        for p in drone_procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()

        # Kill agent-only procs (--skip-sitl mode)
        for p in agent_procs:
            if p.poll() is None:
                p.terminate()
        for p in agent_procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()

        log.info("All processes stopped.")


if __name__ == "__main__":
    main()
