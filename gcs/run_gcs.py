#!/usr/bin/env python3
"""
Standalone GCS entry point — run in its own terminal.

Usage:
    python gcs/run_gcs.py [--no-viz] [--headless] [--gif]
    python gcs/run_gcs.py --num-drones 3

Starts the Ground Control Station with CLI, visualization,
state collection, and command dispatch.
"""

import os
import sys
import signal
import logging
import argparse

# ── Resolve project root and add to path ─────────────────
GCS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(GCS_DIR, ".."))
sys.path.insert(0, PROJECT_DIR)

from config import NUM_DRONES
from gcs.gcs import GCS


def main():
    parser = argparse.ArgumentParser(description="Swarm GCS — Ground Control Station")
    parser.add_argument("--num-drones", type=int, default=NUM_DRONES,
                        help=f"Number of drones to manage (default: {NUM_DRONES})")
    parser.add_argument("--no-viz", action="store_true",
                        help="Disable visualization")
    parser.add_argument("--headless", action="store_true",
                        help="Headless mode (no display, save GIF)")
    parser.add_argument("--gif", action="store_true",
                        help="Save GIF of the session")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="[GCS] %(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    gcs = GCS(
        num_drones=args.num_drones,
        enable_viz=not args.no_viz,
        headless=args.headless,
        save_gif=args.gif or args.headless,
    )

    def shutdown(sig, frame):
        logging.getLogger(__name__).info("Caught signal %d, shutting down", sig)
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
