"""Leader drone container entry point.

Launches ArduCopter SITL, connects, starts MAVLink bridge,
takes off, then waits for GCS commands to fly.

After takeoff the leader enters HOVER (hold position).
The GCS web UI can then:
  - WASD: manual velocity control (NED)
  - Waypoint: click-on-map, leader flies to that point at operating speed
  - Speed: set operating speed slider
  - HOVER: stop and hold position
  - RTL / LAND / KILL: safety commands
"""

import logging
import os
import signal
import sys
import time

from docker_sim.config import (
    DRONE_ID, TAKEOFF_ALT_M, CONTROL_HZ,
    PEER_HOST, BROADCAST_PORT, LISTEN_PORT,
    GCS_HOST, GCS_TELEM_PORT, GCS_CMD_PORT,
    OUTPUT_DIR, PEER_STALE_TIMEOUT,
)
from docker_sim.sitl_launcher import SITLLauncher
from docker_sim.mavlink_conn import MavlinkConn
from docker_sim.takeoff import TakeoffManager
from docker_sim.mavlink_bridge import (
    MavlinkBridge, CMD_NAMES,
    CMD_RTL, CMD_LAND, CMD_KILL, CMD_HOVER,
    CMD_WASD, CMD_TAKEOFF,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [LEADER] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("leader")

_running = True

def _signal_handler(sig, frame):
    global _running
    log.info("Interrupted -- shutting down...")
    _running = False


# ── Main ─────────────────────────────────────────────────────

def main():
    global _running
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    launcher = SITLLauncher()
    conn = None
    bridge = MavlinkBridge(
        PEER_HOST, BROADCAST_PORT, LISTEN_PORT,
        gcs_host=GCS_HOST, gcs_telem_port=GCS_TELEM_PORT,
        gcs_cmd_port=GCS_CMD_PORT,
        stale_timeout=PEER_STALE_TIMEOUT,
    )

    try:
        # Phase 1: Launch SITL
        log.info("=" * 50)
        log.info("Phase 1: Launching SITL")
        log.info("=" * 50)
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        launcher.launch()
        if not launcher.wait_ready():
            raise TimeoutError("SITL port not reachable")

        # Phase 2: Connect
        log.info("=" * 50)
        log.info("Phase 2: Connecting pymavlink")
        log.info("=" * 50)
        conn = MavlinkConn(timeout=60)

        # Phase 3: Start bridge + wait GPS
        log.info("=" * 50)
        log.info("Phase 3: Waiting for GPS fix")
        log.info("=" * 50)
        bridge.start()

        gps_deadline = time.time() + 90.0
        has_gps = False
        while _running and time.time() < gps_deadline:
            data = conn.drain_latest()
            pos = data.get('position')
            if pos and pos['lat'] != 0:
                log.info("GPS fix: (%.7f, %.7f)", pos['lat'], pos['lon'])
                has_gps = True
                break
            time.sleep(0.5)
        if not has_gps:
            raise TimeoutError("GPS fix timeout")

        # Phase 4: Wait for GCS TAKEOFF command
        log.info("=" * 50)
        log.info("Phase 4: Ready — waiting for GCS TAKEOFF command")
        log.info("=" * 50)

        takeoff_requested = False
        while _running and not takeoff_requested:
            data = conn.drain_latest()
            pos = data.get('position')
            if pos:
                bridge.update_own_state(pos)

            while True:
                gcs_cmd = bridge.get_pending_command()
                if gcs_cmd is None:
                    break
                c = gcs_cmd['cmd']
                if c == CMD_TAKEOFF:
                    log.info("GCS: TAKEOFF received")
                    takeoff_requested = True
                    break
                elif c == CMD_KILL:
                    log.info("GCS: KILL (pre-takeoff)")
                    conn.force_disarm()
                    _running = False
                    break
                else:
                    log.info("GCS: Ignoring %s (waiting for TAKEOFF)",
                             CMD_NAMES.get(c, f"?{c}"))

            time.sleep(1.0 / CONTROL_HZ)

        # Phase 4b: Execute takeoff
        if _running and takeoff_requested:
            log.info("Phase 4b: Takeoff to %.0fm", TAKEOFF_ALT_M)
            takeoff = TakeoffManager(conn, TAKEOFF_ALT_M)

            while _running:
                data = conn.drain_latest()
                pos = data.get('position')
                hb = data.get('heartbeat')

                if pos:
                    bridge.update_own_state(pos)

                done = takeoff.tick(
                    has_gps=bool(pos and pos['lat'] != 0),
                    mode=hb['mode'] if hb else '',
                    armed=hb['armed'] if hb else False,
                    alt=pos['alt'] if pos else 0.0,
                )
                if done:
                    break
                time.sleep(1.0 / CONTROL_HZ)

            log.info("Airborne!")

        # Phase 5: GCS-controlled flight
        log.info("=" * 50)
        log.info("Phase 5: GCS-controlled (HOVER)")
        log.info("=" * 50)

        mode = "HOVER"
        wasd_vn = 0.0
        wasd_ve = 0.0

        while _running:
            tick_start = time.time()

            data = conn.drain_latest()
            pos = data.get('position')
            if pos:
                bridge.update_own_state(pos)

            # ── Process GCS commands (drain all pending) ──
            while True:
                gcs_cmd = bridge.get_pending_command()
                if gcs_cmd is None:
                    break
                c = gcs_cmd['cmd']

                if c == CMD_RTL:
                    log.info("GCS: RTL")
                    conn.set_mode("RTL")
                    _running = False
                    break
                elif c == CMD_LAND:
                    log.info("GCS: LAND")
                    conn.set_mode("LAND")
                    _running = False
                    break
                elif c == CMD_KILL:
                    log.info("GCS: KILL")
                    conn.force_disarm()
                    _running = False
                    break
                elif c == CMD_HOVER:
                    if mode != "HOVER":
                        log.info("GCS: HOVER")
                    mode = "HOVER"
                    wasd_vn = 0.0
                    wasd_ve = 0.0
                elif c == CMD_WASD:
                    wasd_vn = gcs_cmd.get('vn', 0.0)
                    wasd_ve = gcs_cmd.get('ve', 0.0)
                    mode = "WASD" if (wasd_vn != 0 or wasd_ve != 0) else "HOVER"
                elif c == CMD_TAKEOFF:
                    log.info("GCS: TAKEOFF ignored (already airborne)")
                else:
                    log.warning("Ignoring unexpected command: %s",
                                CMD_NAMES.get(c, f"?{c}"))

            if not _running:
                break

            # ── Execute current mode ──
            if mode == "HOVER":
                conn.send_velocity_ned(0, 0, 0)
            elif mode == "WASD":
                conn.send_velocity_ned(wasd_vn, wasd_ve, 0)

            elapsed = time.time() - tick_start
            sleep_time = (1.0 / CONTROL_HZ) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        # Phase 6: Shutdown
        log.info("=" * 50)
        log.info("Phase 6: Shutdown")
        log.info("=" * 50)

        land_deadline = time.time() + 60.0
        while time.time() < land_deadline:
            data = conn.drain_latest()
            pos = data.get('position')
            if pos:
                bridge.update_own_state(pos)
            if pos and pos['alt'] < 1.0:
                log.info("Landed")
                break
            time.sleep(1.0)

    except Exception as e:
        log.error("Fatal: %s", e, exc_info=True)
    finally:
        bridge.stop()
        if conn:
            try:
                conn.set_mode("LAND")
            except Exception:
                pass
        time.sleep(2)
        launcher.kill()
        log.info("Cleanup complete")


if __name__ == "__main__":
    main()
