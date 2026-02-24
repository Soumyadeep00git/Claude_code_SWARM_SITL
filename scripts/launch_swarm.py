"""
Master launcher — starts N drone systems and one GCS.

Each drone is launched as a self-contained subprocess (drones/drone_N/run.py)
that manages its own SITL instance and agent.

Usage:
    python -m scripts.launch_swarm [--num-drones N] [--no-viz] [--skip-sitl]
"""

import os
import sys
import time
import signal
import subprocess
import logging
import argparse

# Add project root to path
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_DIR)

from config import (
    NUM_DRONES, SITL_BASE_PORT, SITL_PORT_STEP,
    LOG_DIR, PROJECT_DIR as CFG_PROJECT_DIR,
)

log = logging.getLogger(__name__)


def wait_for_sitl(drone_id: int, timeout: float = 90.0) -> bool:
    """Wait for a SITL instance to become reachable on its TCP port."""
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


def main():
    parser = argparse.ArgumentParser(description="Launch Swarm SITL Stack")
    parser.add_argument("--num-drones", type=int, default=NUM_DRONES,
                        help=f"Number of drones (default: {NUM_DRONES})")
    parser.add_argument("--no-viz", action="store_true",
                        help="Disable GCS visualization")
    parser.add_argument("--skip-sitl", action="store_true",
                        help="Skip SITL launch (agents only, SITL already running)")
    args = parser.parse_args()

    num = args.num_drones

    logging.basicConfig(
        level=logging.INFO,
        format="[LAUNCH] %(asctime)s %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    os.makedirs(LOG_DIR, exist_ok=True)

    drone_procs = []
    agent_procs = []  # Only used in --skip-sitl mode
    gcs_proc = None

    try:
        if not args.skip_sitl:
            # ── Phase 1: Launch drone subprocesses ─────────────
            log.info("Launching %d drone systems...", num)
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
                log.info("  Drone %d launched (pid %d)", i, p.pid)
                time.sleep(5)  # Stagger to avoid SITL build races

            log.info("Waiting for %d SITL instances to initialize...", num)
            for i in range(1, num + 1):
                if wait_for_sitl(i):
                    log.info("  SITL %d ready on port %d", i, SITL_BASE_PORT + i * SITL_PORT_STEP)
                else:
                    log.error("  SITL %d failed to start!", i)
                    raise RuntimeError(f"SITL instance {i} did not start")
        else:
            # ── Phase 1 (skip-sitl): Launch agent-only subprocesses
            log.info("Launching %d agents (--skip-sitl)...", num)
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

        # ── Phase 2: Launch GCS ────────────────────────────
        log.info("Launching GCS...")
        gcs_script = os.path.join(PROJECT_DIR, "gcs", "run_gcs.py")
        gcs_cmd = [sys.executable, gcs_script]
        if args.no_viz:
            gcs_cmd.append("--no-viz")
        gcs_cmd.extend(["--num-drones", str(num)])
        gcs_proc = subprocess.Popen(
            gcs_cmd,
            cwd=PROJECT_DIR,
        )
        log.info("GCS started (pid %d)", gcs_proc.pid)

        # ── Phase 3: Wait for GCS to exit ─────────────────
        log.info("=== Swarm is running! ===")
        log.info("GCS CLI is active. Type 'help' for commands.")
        gcs_proc.wait()

    except KeyboardInterrupt:
        log.info("Keyboard interrupt — shutting down...")
    except Exception as e:
        log.error("Error: %s", e)
    finally:
        # ── Cleanup ────────────────────────────────────────
        log.info("Shutting down all processes...")

        if gcs_proc and gcs_proc.poll() is None:
            gcs_proc.terminate()
            try:
                gcs_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                gcs_proc.kill()

        for p in drone_procs:
            if p.poll() is None:
                p.terminate()
        for p in drone_procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()

        for p in agent_procs:
            if p.poll() is None:
                p.terminate()
        for p in agent_procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()

        log.info("All processes stopped. Goodbye.")


if __name__ == "__main__":
    main()
