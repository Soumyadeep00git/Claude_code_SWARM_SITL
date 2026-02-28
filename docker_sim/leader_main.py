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
import math
import os
import signal
import sys
import time

from docker_sim.config import (
    DRONE_ID, TAKEOFF_ALT_M, CONTROL_HZ,
    PEER_HOST, BROADCAST_PORT, LISTEN_PORT,
    GCS_HOST, GCS_TELEM_PORT, GCS_CMD_PORT,
    MY_HOME_LAT, MY_HOME_LON,
    METERS_PER_DEG_LAT, OUTPUT_DIR,
    PEER_STALE_TIMEOUT,
)
from docker_sim.sitl_launcher import SITLLauncher
from docker_sim.mavlink_conn import MavlinkConn
from docker_sim.takeoff import TakeoffManager
from docker_sim.mavlink_bridge import (
    MavlinkBridge,
    CMD_RTL, CMD_LAND, CMD_KILL, CMD_HOVER,
    CMD_WASD, CMD_WAYPOINT, CMD_SPEED,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [LEADER] %(levelname)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("leader")

_running = True

# Default operating speed (m/s), changeable via GCS
DEFAULT_SPEED = 1.5
WAYPOINT_ARRIVAL_M = 2.0


def _signal_handler(sig, frame):
    global _running
    log.info("Interrupted -- shutting down...")
    _running = False


def _gps_dist(lat1, lon1, lat2, lon2):
    dn = (lat2 - lat1) * METERS_PER_DEG_LAT
    de = (lon2 - lon1) * METERS_PER_DEG_LAT * math.cos(math.radians(lat1))
    return math.sqrt(dn * dn + de * de)


def _vel_toward(own_lat, own_lon, tgt_lat, tgt_lon, speed):
    """Compute NED velocity vector toward a waypoint at given speed."""
    dn = (tgt_lat - own_lat) * METERS_PER_DEG_LAT
    de = (tgt_lon - own_lon) * METERS_PER_DEG_LAT * math.cos(math.radians(own_lat))
    dist = math.sqrt(dn * dn + de * de)
    if dist < 0.1:
        return 0.0, 0.0
    scale = speed / dist
    return dn * scale, de * scale


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

        # Phase 4: Takeoff
        log.info("=" * 50)
        log.info("Phase 4: Takeoff to %.0fm", TAKEOFF_ALT_M)
        log.info("=" * 50)
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

        # State: "HOVER", "WASD", "GOTO"
        mode = "HOVER"
        operating_speed = DEFAULT_SPEED
        wasd_vn = 0.0
        wasd_ve = 0.0
        wp_lat = 0.0
        wp_lon = 0.0

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
                    mode = "WASD"
                    wasd_vn = gcs_cmd.get('vn', 0.0)
                    wasd_ve = gcs_cmd.get('ve', 0.0)
                elif c == CMD_WAYPOINT:
                    wp_lat = gcs_cmd['lat']
                    wp_lon = gcs_cmd['lon']
                    mode = "GOTO"
                    log.info("GCS: GOTO (%.7f, %.7f)", wp_lat, wp_lon)
                elif c == CMD_SPEED:
                    operating_speed = max(0.5, min(gcs_cmd['speed'], 10.0))
                    log.info("GCS: SPEED %.1f m/s", operating_speed)
                else:
                    log.warning("Ignoring unexpected command: %s", c)

            if not _running:
                break

            # ── Execute current mode ──
            if mode == "HOVER":
                conn.send_velocity_ned(0, 0, 0)

            elif mode == "WASD":
                conn.send_velocity_ned(wasd_vn, wasd_ve, 0)

            elif mode == "GOTO":
                if pos and pos['lat'] != 0:
                    dist = _gps_dist(pos['lat'], pos['lon'], wp_lat, wp_lon)
                    if dist < WAYPOINT_ARRIVAL_M:
                        log.info("Waypoint reached (dist=%.1fm)", dist)
                        mode = "HOVER"
                        conn.send_velocity_ned(0, 0, 0)
                    else:
                        vn, ve = _vel_toward(
                            pos['lat'], pos['lon'], wp_lat, wp_lon,
                            operating_speed)
                        conn.send_velocity_ned(vn, ve, 0)

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
