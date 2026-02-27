"""Main orchestrator for the Leader-Follower SITL simulation test.

Usage:
    cd /mnt/d/LAT_D/Leader_Follower_ORIN_CO+
    python3 -m sim              [auto-detects headless, saves GIF+PNG]
    python3 -m sim --headless   [force headless mode, saves GIF+PNG]
    python3 -m sim --gif        [save GIF even with live display]
    python3 -m sim --no-viz     [disable visualization entirely]

Phases:
    1. Launch 2 ArduCopter SITL instances
    2. Connect via pymavlink, wait for heartbeats
    3. Wait for GPS fix on both drones
    4. Takeoff both to 10m
    5. Run mission: leader flies waypoints, follower tracks with 5-layer APF
    6. Shutdown: RTL/LAND both, kill SITL processes
"""

import argparse
import logging
import os
import signal
import socket
import sys
import time

# Add follower_drone package to Python path (pure Python, no ROS2)
_PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_PROJECT_DIR, "follower_drone"))

from sim.config import (
    SITL_BASE_PORT, SITL_PORT_STEP,
    LEADER_ID, FOLLOWER_ID,
    TAKEOFF_ALT_M, CONTROL_HZ,
    LOG_DIR, OUTPUT_DIR,
)
from sim.sitl_launcher import SITLLauncher
from sim.mavlink_conn import MavlinkConn
from sim.takeoff import TakeoffManager
from sim.leader_mission import LeaderMission, Phase
from sim.follower_loop import FollowerController
from sim.viz import LiveVisualizer
from sim.csv_logger import CSVLogger

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("sim")

# Graceful shutdown flag
_running = True


def _signal_handler(sig, frame):
    global _running
    log.info("Interrupted — shutting down...")
    _running = False


def _wait_port(host: str, port: int, timeout: float = 90.0) -> bool:
    """Wait for a TCP port to become reachable."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection((host, port), timeout=2):
                return True
        except (ConnectionRefusedError, OSError):
            time.sleep(1)
    return False


def _extract_state(data: dict) -> tuple[dict | None, dict | None]:
    """Extract position and heartbeat from drain_latest() result."""
    return data.get('position'), data.get('heartbeat')


def main():
    global _running

    parser = argparse.ArgumentParser(description="Leader-Follower SITL Test")
    parser.add_argument("--no-viz", action="store_true",
                        help="Disable matplotlib visualization entirely")
    parser.add_argument("--headless", action="store_true",
                        help="Headless mode (Agg backend, auto-saves GIF)")
    parser.add_argument("--gif", action="store_true",
                        help="Save GIF animation of the flight")
    args = parser.parse_args()

    # Auto-detect headless (WSL2, SSH, CI — no DISPLAY env var)
    import os as _os
    if not args.no_viz and not _os.environ.get("DISPLAY"):
        log.info("No DISPLAY detected — enabling headless mode")
        args.headless = True

    signal.signal(signal.SIGINT, _signal_handler)

    launcher = SITLLauncher()
    leader_conn = None
    follower_conn = None
    csv_logger = None
    viz = None

    try:
        # ── Phase 1: Launch SITL ──────────────────────────
        log.info("=" * 60)
        log.info("Phase 1: Launching SITL instances")
        log.info("=" * 60)

        os.makedirs(LOG_DIR, exist_ok=True)
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        launcher.launch_all()

        # Wait for TCP ports to become reachable
        leader_port = SITL_BASE_PORT + LEADER_ID * SITL_PORT_STEP
        follower_port = SITL_BASE_PORT + FOLLOWER_ID * SITL_PORT_STEP

        log.info("Waiting for SITL ports %d and %d...", leader_port, follower_port)
        if not _wait_port("127.0.0.1", leader_port):
            raise TimeoutError(f"Leader SITL port {leader_port} not reachable")
        if not _wait_port("127.0.0.1", follower_port):
            raise TimeoutError(f"Follower SITL port {follower_port} not reachable")
        log.info("SITL ports reachable")

        if not _running:
            return

        # ── Phase 2: Connect pymavlink ────────────────────
        log.info("=" * 60)
        log.info("Phase 2: Connecting pymavlink")
        log.info("=" * 60)

        leader_conn = MavlinkConn(LEADER_ID, timeout=60)
        follower_conn = MavlinkConn(FOLLOWER_ID, timeout=60)
        log.info("Both drones connected")

        if not _running:
            return

        # ── Phase 3: Wait for GPS fix ─────────────────────
        log.info("=" * 60)
        log.info("Phase 3: Waiting for GPS fix")
        log.info("=" * 60)

        gps_deadline = time.time() + 90.0
        leader_has_gps = False
        follower_has_gps = False

        while _running and time.time() < gps_deadline:
            ld = leader_conn.drain_latest()
            fd = follower_conn.drain_latest()

            lp = ld.get('position')
            fp = fd.get('position')

            if lp and lp['lat'] != 0:
                if not leader_has_gps:
                    log.info("Leader GPS fix: (%.7f, %.7f)", lp['lat'], lp['lon'])
                leader_has_gps = True
            if fp and fp['lat'] != 0:
                if not follower_has_gps:
                    log.info("Follower GPS fix: (%.7f, %.7f)", fp['lat'], fp['lon'])
                follower_has_gps = True

            if leader_has_gps and follower_has_gps:
                break
            time.sleep(0.5)

        if not (leader_has_gps and follower_has_gps):
            raise TimeoutError("GPS fix timeout (90s)")

        log.info("Both drones have GPS fix")

        if not _running:
            return

        # ── Phase 4: Takeoff ──────────────────────────────
        log.info("=" * 60)
        log.info("Phase 4: Takeoff to %.0fm", TAKEOFF_ALT_M)
        log.info("=" * 60)

        leader_takeoff = TakeoffManager(leader_conn, TAKEOFF_ALT_M)
        follower_takeoff = TakeoffManager(follower_conn, TAKEOFF_ALT_M)

        while _running:
            ld = leader_conn.drain_latest()
            fd = follower_conn.drain_latest()
            lp, lh = _extract_state(ld)
            fp, fh = _extract_state(fd)

            l_done = leader_takeoff.tick(
                has_gps=bool(lp and lp['lat'] != 0),
                mode=lh['mode'] if lh else '',
                armed=lh['armed'] if lh else False,
                alt=lp['alt'] if lp else 0.0,
            )
            f_done = follower_takeoff.tick(
                has_gps=bool(fp and fp['lat'] != 0),
                mode=fh['mode'] if fh else '',
                armed=fh['armed'] if fh else False,
                alt=fp['alt'] if fp else 0.0,
            )

            if l_done and f_done:
                break

            if not launcher.check_alive():
                raise RuntimeError("SITL process died during takeoff")

            time.sleep(1.0 / CONTROL_HZ)

        log.info("Both drones airborne")

        if not _running:
            return

        # ── Phase 5: Mission loop ─────────────────────────
        log.info("=" * 60)
        log.info("Phase 5: Running mission")
        log.info("=" * 60)

        # Get leader home position from latest data
        ld = leader_conn.drain_latest()
        lp = ld.get('position', {})
        leader_home_lat = lp.get('lat', 0)
        leader_home_lon = lp.get('lon', 0)

        leader_mission = LeaderMission(leader_conn, leader_home_lat, leader_home_lon)
        follower_ctrl = FollowerController(follower_conn)
        csv_logger = CSVLogger()
        viz = LiveVisualizer(
            enabled=not args.no_viz,
            headless=args.headless,
            save_gif=args.gif or args.headless,
        )

        tick_count = 0
        # Cache latest states for viz/logging
        leader_pos = None
        leader_hb = None
        follower_pos = None
        follower_hb = None

        while _running:
            tick_start = time.time()
            tick_count += 1

            # 1. Drain MAVLink
            ld = leader_conn.drain_latest()
            fd = follower_conn.drain_latest()

            lp_new, lh_new = _extract_state(ld)
            fp_new, fh_new = _extract_state(fd)
            if lp_new:
                leader_pos = lp_new
            if lh_new:
                leader_hb = lh_new
            if fp_new:
                follower_pos = fp_new
            if fh_new:
                follower_hb = fh_new

            # 2. Update follower with latest states
            if leader_pos:
                follower_ctrl.update_leader_state(leader_pos)
            if follower_pos:
                follower_ctrl.update_own_state(follower_pos)

            # 3. Leader mission tick
            phase = leader_mission.tick(
                leader_pos['lat'] if leader_pos else 0,
                leader_pos['lon'] if leader_pos else 0,
                leader_pos['alt'] if leader_pos else 0,
            )

            # 4. Follower APF tick
            apf_result = follower_ctrl.tick()

            # 5. Log
            csv_logger.log_leader(leader_pos, leader_hb, phase.value)
            csv_logger.log_follower(
                follower_pos, follower_hb, phase.value,
                apf_result, follower_ctrl.target_lat, follower_ctrl.target_lon)

            # 6. Visualize (every other tick = 5Hz)
            if tick_count % 2 == 0:
                viz.update(
                    leader_pos, follower_pos, apf_result,
                    follower_ctrl.target_lat, follower_ctrl.target_lon,
                    phase.value)

            # 7. Check mission complete
            if phase == Phase.DONE:
                log.info("Leader mission complete")
                break

            # 8. Check SITL alive
            if tick_count % 50 == 0 and not launcher.check_alive():
                raise RuntimeError("SITL process died during mission")

            # 9. Rate control
            elapsed = time.time() - tick_start
            sleep_time = (1.0 / CONTROL_HZ) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        # ── Phase 6: Shutdown ─────────────────────────────
        log.info("=" * 60)
        log.info("Phase 6: Shutdown")
        log.info("=" * 60)

        # Set follower to RTL
        if follower_conn:
            follower_conn.set_mode("RTL")
        time.sleep(2)

        # Wait for both to descend
        land_deadline = time.time() + 30.0
        while time.time() < land_deadline:
            ld = leader_conn.drain_latest()
            fd = follower_conn.drain_latest()
            lp = ld.get('position')
            fp = fd.get('position')

            l_landed = lp and lp['alt'] < 1.0
            f_landed = fp and fp['alt'] < 1.0

            if l_landed and f_landed:
                log.info("Both drones landed")
                break
            time.sleep(1.0)

        log.info("Mission complete. Check output in sim/output/")

    except KeyboardInterrupt:
        log.info("Interrupted by user")
    except Exception as e:
        log.error("Fatal error: %s", e, exc_info=True)
    finally:
        # Always clean up
        for conn in [leader_conn, follower_conn]:
            if conn:
                try:
                    conn.set_mode("LAND")
                except Exception:
                    pass
        time.sleep(2)
        launcher.kill_all()
        if csv_logger:
            csv_logger.close()
        if viz:
            gif_path = viz.save_gif()
            png_path = viz.save_final_frame()
            viz.close()
            if gif_path:
                log.info("Visual output: %s", gif_path)
            if png_path:
                log.info("Visual output: %s", png_path)
        log.info("Cleanup complete")


if __name__ == '__main__':
    main()
