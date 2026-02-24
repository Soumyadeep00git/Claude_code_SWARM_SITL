#!/usr/bin/env python3
"""
Self-contained drone launcher — starts its own SITL instance and agent.

Run from any working directory:
    python drones/drone_1/run.py

The drone ID is auto-detected from the directory name (drones/drone_3/ → ID=3).
This file is identical across all 5 drone directories.
"""

import os
import re
import sys
import time
import signal
import socket
import logging

# ── Resolve project root and add to path ─────────────────
DRONE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(DRONE_DIR, "..", ".."))
sys.path.insert(0, PROJECT_DIR)

from config import SITL_BASE_PORT, SITL_PORT_STEP
from sitl.launcher import launch_sitl_instance, kill_sitl_process
from drone_agent.agent import DroneAgent

# ── Auto-detect drone ID from directory name ──────────────
_match = re.search(r"drone_(\d+)", os.path.basename(DRONE_DIR))
if not _match:
    print(f"ERROR: Cannot detect drone ID from directory: {DRONE_DIR}")
    print("Expected directory name like 'drone_1', 'drone_2', etc.")
    sys.exit(1)

DRONE_ID = int(_match.group(1))


def wait_for_sitl_port(drone_id: int, timeout: float = 90.0) -> bool:
    """Wait for SITL TCP port to accept connections."""
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
    logging.basicConfig(
        level=logging.INFO,
        format=f"[D{DRONE_ID}] %(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )
    log = logging.getLogger(f"drone_{DRONE_ID}")

    sitl_proc = None
    agent = None

    def shutdown(sig=None, frame=None):
        nonlocal agent, sitl_proc
        log.info("Shutting down drone %d...", DRONE_ID)
        if agent:
            agent.stop()
            agent = None
        if sitl_proc:
            kill_sitl_process(sitl_proc)
            sitl_proc = None

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    try:
        # 1. Launch own SITL
        log.info("Starting SITL instance %d...", DRONE_ID)
        sitl_proc = launch_sitl_instance(DRONE_ID)

        # 2. Wait for SITL to be ready
        port = SITL_BASE_PORT + DRONE_ID * SITL_PORT_STEP
        log.info("Waiting for SITL on port %d...", port)
        if not wait_for_sitl_port(DRONE_ID):
            log.error("SITL %d failed to start (port %d not reachable)", DRONE_ID, port)
            sys.exit(1)
        log.info("SITL %d ready on port %d", DRONE_ID, port)

        # 3. Create and run agent
        log.info("Starting agent for drone %d...", DRONE_ID)
        agent = DroneAgent(DRONE_ID)
        agent.run()

    except KeyboardInterrupt:
        log.info("Interrupted")
    except Exception as e:
        log.error("Fatal: %s", e, exc_info=True)
    finally:
        shutdown()
        log.info("Drone %d stopped.", DRONE_ID)


if __name__ == "__main__":
    main()
