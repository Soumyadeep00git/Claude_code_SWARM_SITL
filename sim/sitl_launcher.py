"""Launch and manage 2 ArduCopter SITL instances."""

import logging
import math
import os
import signal
import subprocess
import time

from sim.config import (
    ARDUCOPTER_BIN, COPTER_DEFAULTS,
    HOME_LAT, HOME_LON, HOME_ALT, HOME_HEADING,
    SITL_BASE_PORT, SITL_PORT_STEP,
    HOME_SPACING_M, LOG_DIR, METERS_PER_DEG_LAT,
)

log = logging.getLogger(__name__)


class SITLLauncher:
    """Manages lifecycle of ArduCopter SITL processes."""

    def __init__(self):
        self.processes: dict[int, subprocess.Popen] = {}

    def _compute_home(self, drone_id: int) -> str:
        """Compute home position string. Follower is offset east."""
        offset_east = (drone_id - 1) * HOME_SPACING_M
        cos_lat = math.cos(math.radians(HOME_LAT))
        lon = HOME_LON + offset_east / (METERS_PER_DEG_LAT * cos_lat)
        return f"{HOME_LAT},{lon},{HOME_ALT},{HOME_HEADING}"

    def _launch_one(self, drone_id: int) -> subprocess.Popen:
        """Launch a single ArduCopter SITL instance."""
        if not os.path.isfile(ARDUCOPTER_BIN):
            raise FileNotFoundError(
                f"ArduCopter binary not found at {ARDUCOPTER_BIN}\n"
                f"Build it: cd ~/ardupilot && ./waf configure --board sitl && ./waf copter")

        # Each instance needs its own working directory (EEPROM isolation)
        work_dir = os.path.join(LOG_DIR, f"sitl_instance_{drone_id}")
        os.makedirs(work_dir, exist_ok=True)

        sysid = drone_id + 1
        home = self._compute_home(drone_id)
        port = SITL_BASE_PORT + drone_id * SITL_PORT_STEP

        cmd = [
            ARDUCOPTER_BIN,
            "--model", "+",
            "--speedup", "1",
            "--slave", "0",
            "--defaults", COPTER_DEFAULTS,
            f"-I{drone_id}",
            "--home", home,
            "--sysid", str(sysid),
        ]

        log.info("Launching SITL drone_id=%d sysid=%d port=%d home=%s",
                 drone_id, sysid, port, home)

        proc = subprocess.Popen(
            cmd,
            cwd=work_dir,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        self.processes[drone_id] = proc
        return proc

    def launch_all(self):
        """Launch leader then follower with staggered delay."""
        from sim.config import LEADER_ID, FOLLOWER_ID

        self._launch_one(LEADER_ID)
        time.sleep(2.0)
        self._launch_one(FOLLOWER_ID)

        log.info("Both SITL instances launched")

    def kill_all(self):
        """Terminate all SITL processes (SIGTERM then SIGKILL)."""
        for drone_id, proc in self.processes.items():
            if proc.poll() is not None:
                continue
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGTERM)
                log.info("Sent SIGTERM to drone_id=%d (pgid=%d)", drone_id, pgid)
            except ProcessLookupError:
                pass

        # Wait up to 5s for graceful exit
        deadline = time.time() + 5.0
        for proc in self.processes.values():
            remaining = max(0.1, deadline - time.time())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    pgid = os.getpgid(proc.pid)
                    os.killpg(pgid, signal.SIGKILL)
                    log.warning("Sent SIGKILL to pgid=%d", pgid)
                except ProcessLookupError:
                    pass

        self.processes.clear()
        log.info("All SITL processes terminated")

    def check_alive(self) -> bool:
        """Return True if all SITL processes are still running."""
        return all(p.poll() is None for p in self.processes.values())
