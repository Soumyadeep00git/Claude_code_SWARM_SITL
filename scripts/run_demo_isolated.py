"""
Per-core isolated demo — each drone on its own CPU core,
GCS on another core, orchestrator on its own core.
Simulates real-world deployment where each drone is a separate computer.

Usage:
    python -m scripts.run_demo_isolated [--num-drones N] [--skip-sitl]

Process topology:
  Core 0: Orchestrator (this process — sends commands, monitors states)
  Core 1: GCS subprocess (receive, relay, visualize, log)
  Core 2: Drone 1 (SITL + Agent — SITL child inherits core affinity)
  Core 3: Drone 2
  ...

Mission sequence is identical to run_demo.py.
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
    ORCHESTRATOR_PORT, LOG_DIR, MONITOR_SAMPLE_HZ,
)
from comms.protocol import make_msg, parse_msg
from comms.udp_node import UDPNode
from gcs.state_collector import StateCollector
from scripts.cpu_affinity import (
    assign_cores, get_available_cores, pin_process, pin_process_tree,
    log_affinity_map,
)

log = logging.getLogger("demo_isolated")


# ── Utility functions (shared with run_demo.py) ──────────────

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


# ── Orchestrator-specific collect (no relay, no keepalive) ───

def collect_states(udp: UDPNode, collector: StateCollector,
                   num_drones: int, duration_s: float):
    """Receive GCS-relayed state reports for duration_s seconds.

    Unlike run_demo.py, the orchestrator does NOT relay to peers
    or send keepalives — that's the GCS's job now.
    """
    end_time = time.time() + duration_s
    while time.time() < end_time:
        for raw, addr in udp.recv_all():
            msg = parse_msg(raw)
            if msg is None:
                continue
            if msg["type"] == "STATE_REPORT":
                src = msg.get("data", {}).get("drone_id", msg.get("src", 0))
                collector.update(src, msg["data"], msg.get("ts", time.time()))
            elif msg["type"] == "ALERT":
                d = msg["data"]
                log.warning("ALERT D%d: [%s] %s",
                            msg["src"], d.get("code"), d.get("message"))
        time.sleep(0.1)


def wait_for_reports(udp, collector, num_drones, timeout=120):
    """Wait until all drones have reported at least once."""
    log.info("Waiting for all %d drones to report...", num_drones)
    start = time.time()
    while len(collector.states) < num_drones:
        collect_states(udp, collector, num_drones, 1.0)
        elapsed = time.time() - start
        if int(elapsed) % 5 == 0:
            log.info("  %d/%d drones reporting (%.0fs)",
                     len(collector.states), num_drones, elapsed)
        if elapsed > timeout:
            log.warning("Timeout — continuing with %d", len(collector.states))
            break


def wait_for_alt(udp, collector, num_drones, target_alt, tolerance=2.0, timeout=60):
    """Wait until all drones are near target altitude."""
    log.info("Waiting for drones to reach %.1f m...", target_alt)
    start = time.time()
    while True:
        collect_states(udp, collector, num_drones, 1.0)
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


# ── Main ─────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Per-core isolated swarm demo")
    parser.add_argument("--num-drones", type=int, default=NUM_DRONES)
    parser.add_argument("--skip-sitl", action="store_true",
                        help="Skip SITL launch (agents only, SITL already running)")
    parser.add_argument("--no-monitor", action="store_true",
                        help="Disable CPU/process monitor")
    parser.add_argument("--web", action="store_true",
                        help="Launch web-based GCS (open browser to http://localhost:5000)")
    args = parser.parse_args()

    num = args.num_drones

    logging.basicConfig(
        level=logging.INFO,
        format="[ISOLATED] %(asctime)s %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    os.makedirs(LOG_DIR, exist_ok=True)
    output_dir = os.path.join(PROJECT_DIR, "output")
    os.makedirs(output_dir, exist_ok=True)

    # ═══════════════════════════════════════════════════
    # CORE ASSIGNMENT
    # ═══════════════════════════════════════════════════
    assignments = assign_cores(num)
    if assignments:
        log_affinity_map(assignments)
        # Pin orchestrator (this process)
        pin_process(os.getpid(), assignments["orchestrator"])
        log.info("Orchestrator pinned to core %d (PID %d)",
                 assignments["orchestrator"], os.getpid())

    drone_procs = []
    gcs_proc = None
    monitor_proc = None
    udp = None

    try:
        # ═══════════════════════════════════════════════════
        # PHASE 1: Launch GCS as separate subprocess
        # ═══════════════════════════════════════════════════
        log.info("=== Phase 1: Launching GCS subprocess ===")
        if args.web:
            gcs_cmd = [
                sys.executable, "-m", "gcs.web_gcs",
                "--orchestrator-port", str(ORCHESTRATOR_PORT),
                "--num-drones", str(num),
                "--output-dir", output_dir,
            ]
            log.info("  Using web GCS — open http://localhost:5000")
        else:
            gcs_cmd = [
                sys.executable, "-m", "gcs.gcs",
                "--headless", "--gif",
                "--orchestrator-port", str(ORCHESTRATOR_PORT),
                "--num-drones", str(num),
                "--output-dir", output_dir,
            ]
        gcs_log_file = open(os.path.join(LOG_DIR, "gcs.log"), "w")
        gcs_proc = subprocess.Popen(
            gcs_cmd,
            cwd=PROJECT_DIR,
            stdout=gcs_log_file,
            stderr=subprocess.STDOUT,
        )
        if assignments:
            pin_process(gcs_proc.pid, assignments["gcs"])
            log.info("  GCS pinned to core %d (PID %d)",
                     assignments["gcs"], gcs_proc.pid)
        else:
            log.info("  GCS launched (PID %d) — no pinning", gcs_proc.pid)

        time.sleep(1)  # Let GCS bind its port

        # ═══════════════════════════════════════════════════
        # PHASE 2: Launch drone subprocesses
        # ═══════════════════════════════════════════════════
        if not args.skip_sitl:
            log.info("=== Phase 2: Launching %d drone systems ===", num)
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

                if assignments:
                    core = assignments[f"drone_{i}"]
                    pin_process(p.pid, core)
                    log.info("  Drone %d pinned to core %d (PID %d)", i, core, p.pid)
                else:
                    log.info("  Drone %d launched (PID %d) — no pinning", i, p.pid)

                time.sleep(5)  # Stagger to avoid SITL build races

            # Wait for all SITL ports
            log.info("Waiting for all SITL instances...")
            for i in range(1, num + 1):
                if wait_for_sitl_port(i):
                    log.info("  SITL %d ready (port %d)",
                             i, SITL_BASE_PORT + i * SITL_PORT_STEP)
                else:
                    log.error("  SITL %d FAILED", i)
                    raise RuntimeError(f"SITL {i} did not start")

            # Re-pin process trees (belt and suspenders for SITL grandchildren)
            if assignments:
                log.info("Re-pinning process trees...")
                time.sleep(2)  # Let SITL children fully spawn
                for i, p in enumerate(drone_procs, 1):
                    pin_process_tree(p.pid, assignments[f"drone_{i}"])
        else:
            log.info("=== Phase 2: Launching %d agents (--skip-sitl) ===", num)
            for i in range(1, num + 1):
                log_file = open(os.path.join(LOG_DIR, f"agent_{i}.log"), "w")
                p = subprocess.Popen(
                    [sys.executable, "-m", "drone_agent.agent", str(i)],
                    cwd=PROJECT_DIR,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                )
                drone_procs.append(p)
                if assignments:
                    core = assignments[f"drone_{i}"]
                    pin_process(p.pid, core)
                    log.info("  Agent %d pinned to core %d (PID %d)", i, core, p.pid)
                time.sleep(0.5)

        # Write drone PIDs for web GCS kill functionality
        pid_file = os.path.join(output_dir, "drone_pids.json")
        drone_pid_map = {}
        for i, p in enumerate(drone_procs, 1):
            drone_pid_map[str(i)] = p.pid
        with open(pid_file, "w") as f:
            json.dump(drone_pid_map, f)
        log.info("Drone PIDs written to %s", pid_file)

        # ═══════════════════════════════════════════════════
        # Launch process monitor (between Phase 2 and Phase 3)
        # ═══════════════════════════════════════════════════
        if not args.no_monitor:
            pid_map = {"orchestrator": os.getpid(), "gcs": gcs_proc.pid}
            for i, p in enumerate(drone_procs, 1):
                pid_map[f"drone_{i}"] = p.pid

            monitor_interval = 1.0 / MONITOR_SAMPLE_HZ
            monitor_cmd = [
                sys.executable, "-m", "scripts.process_monitor",
                "--pids", json.dumps(pid_map),
                "--output-dir", output_dir,
                "--interval", str(monitor_interval),
                "--duration", "600",
            ]
            monitor_log_file = open(os.path.join(LOG_DIR, "monitor.log"), "w")
            monitor_proc = subprocess.Popen(
                monitor_cmd,
                cwd=PROJECT_DIR,
                stdout=monitor_log_file,
                stderr=subprocess.STDOUT,
            )
            # Pin monitor to next free core after all drone cores
            available = get_available_cores()
            used_cores = set(assignments.values()) if assignments else set()
            free_cores = [c for c in available if c not in used_cores]
            if free_cores and assignments:
                pin_process(monitor_proc.pid, free_cores[0])
                log.info("  Monitor pinned to core %d (PID %d)",
                         free_cores[0], monitor_proc.pid)
            else:
                log.info("  Monitor launched (PID %d) — no free core for pinning",
                         monitor_proc.pid)

        # ═══════════════════════════════════════════════════
        # PHASE 3: Orchestrator — bind UDP and wait for reports
        # ═══════════════════════════════════════════════════
        log.info("=== Phase 3: Orchestrator listening on port %d ===", ORCHESTRATOR_PORT)
        udp = UDPNode(ORCHESTRATOR_PORT)
        collector = StateCollector(num)

        wait_for_reports(udp, collector, num, timeout=120)

        # Print isolation summary with PIDs
        if assignments:
            log.info("=== Process Isolation Summary ===")
            log.info("  %-14s  %-6s  %-8s", "Role", "Core", "PID")
            log.info("  %-14s  %-6s  %-8s", "-" * 14, "----", "---")
            log.info("  %-14s  %-6d  %-8d", "Orchestrator",
                     assignments["orchestrator"], os.getpid())
            log.info("  %-14s  %-6d  %-8d", "GCS",
                     assignments["gcs"], gcs_proc.pid)
            for i, p in enumerate(drone_procs, 1):
                log.info("  %-14s  %-6d  %-8d", f"Drone {i}",
                         assignments[f"drone_{i}"], p.pid)

        # ═══════════════════════════════════════════════════
        # PHASE 4: Takeoff
        # ═══════════════════════════════════════════════════
        target_alt = 10.0
        log.info("=== Phase 4: Takeoff to %.1f m ===", target_alt)
        send_to_all(udp, num, "TAKEOFF_CMD", {"target_id": 0, "alt": target_alt})

        wait_for_alt(udp, collector, num, target_alt, timeout=90)
        collect_states(udp, collector, num, 5.0)  # Settle time

        # ═══════════════════════════════════════════════════
        # PHASE 5: Formation — V
        # ═══════════════════════════════════════════════════
        log.info("=== Phase 5: Formation V ===")
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

        log.info("  Holding V formation for 20s...")
        collect_states(udp, collector, num, 20.0)

        # ═══════════════════════════════════════════════════
        # PHASE 6: Formation — LINE
        # ═══════════════════════════════════════════════════
        log.info("=== Phase 6: Formation LINE ===")
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

        log.info("  Holding LINE formation for 15s...")
        collect_states(udp, collector, num, 15.0)

        # ═══════════════════════════════════════════════════
        # PHASE 7: Formation — DIAMOND
        # ═══════════════════════════════════════════════════
        log.info("=== Phase 7: Formation DIAMOND ===")
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

        log.info("  Holding DIAMOND formation for 15s...")
        collect_states(udp, collector, num, 15.0)

        # ═══════════════════════════════════════════════════
        # PHASE 8: Land
        # ═══════════════════════════════════════════════════
        log.info("=== Phase 8: Landing ===")
        send_to_all(udp, num, "LAND_CMD", {"target_id": 0})

        log.info("  Waiting for landing (30s)...")
        collect_states(udp, collector, num, 30.0)

        # ═══════════════════════════════════════════════════
        # PHASE 9: Save metadata
        # ═══════════════════════════════════════════════════
        log.info("=== Phase 9: Saving session metadata ===")
        meta = {
            "mission": "automated_demo_isolated",
            "formations": ["V", "LINE", "DIAMOND"],
            "target_altitude_m": target_alt,
            "num_drones": num,
            "mode": "per_core_isolated",
        }
        if assignments:
            meta["core_assignments"] = assignments
            meta["pids"] = {
                "orchestrator": os.getpid(),
                "gcs": gcs_proc.pid,
            }
            for i, p in enumerate(drone_procs, 1):
                meta["pids"][f"drone_{i}"] = p.pid

        meta_path = os.path.join(output_dir, "isolation_metadata.json")
        with open(meta_path, "w") as f:
            json.dump(meta, f, indent=2)
        log.info("  Metadata: %s", meta_path)

        log.info("=== Demo Complete ===")
        log.info("Outputs in: %s (GIF/logs saved by GCS subprocess)", output_dir)

    except KeyboardInterrupt:
        log.info("Demo interrupted by user")
    except Exception as e:
        log.error("Demo failed: %s", e, exc_info=True)
    finally:
        # ── Cleanup ────────────────────────────────────────
        log.info("Cleaning up...")
        if udp:
            udp.close()

        # Kill drone subprocesses
        for p in drone_procs:
            if p.poll() is None:
                p.terminate()
        for p in drone_procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()

        # Stop monitor (sends SIGTERM so it generates the chart on exit)
        if monitor_proc and monitor_proc.poll() is None:
            monitor_proc.terminate()
            try:
                monitor_proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                monitor_proc.kill()

        # Kill GCS subprocess
        if gcs_proc and gcs_proc.poll() is None:
            gcs_proc.terminate()
            try:
                gcs_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                gcs_proc.kill()

        log.info("All processes stopped.")


if __name__ == "__main__":
    main()
