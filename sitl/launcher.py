"""
SITL instance launcher and manager.
Launches ArduCopter binary directly (bypasses sim_vehicle.py rebuild).
Each instance gets its own working directory to avoid EEPROM conflicts.
"""

import os
import subprocess
import signal
import time
import logging
from math import cos, radians

from config import (
    HOME_LAT, HOME_LON, HOME_ALT, HOME_HEADING,
    SITL_BASE_PORT, SITL_PORT_STEP, LOG_DIR,
    ARDUPILOT_DIR,
)

log = logging.getLogger(__name__)

# ArduCopter binary and default params
ARDUCOPTER_BIN = os.path.join(ARDUPILOT_DIR, "build", "sitl", "bin", "arducopter")
COPTER_DEFAULTS = os.path.join(ARDUPILOT_DIR, "Tools", "autotest",
                                "default_params", "copter.parm")


def compute_home(drone_id: int, spacing_m: float = 10.0) -> str:
    """Compute home lat,lon,alt,heading string. Each drone offset east."""
    offset_east = (drone_id - 1) * spacing_m
    lat = HOME_LAT
    lon = HOME_LON + (offset_east / (111320.0 * cos(radians(HOME_LAT))))
    return f"{lat},{lon},{HOME_ALT},{HOME_HEADING}"


def launch_sitl_instance(drone_id: int) -> subprocess.Popen:
    """Launch a single SITL instance directly via the arducopter binary.

    Bypasses sim_vehicle.py to avoid the 60s+ waf rebuild on every launch.
    The process is started in its own process group for clean shutdown.
    """
    if not os.path.isfile(ARDUCOPTER_BIN):
        raise FileNotFoundError(
            f"ArduCopter binary not found: {ARDUCOPTER_BIN}\n"
            "Build it first: cd ~/ardupilot && ./waf configure --board sitl && ./waf copter"
        )

    home = compute_home(drone_id)
    sysid = drone_id + 1  # Match --auto-sysid convention

    work_dir = os.path.join(LOG_DIR, f"sitl_instance_{drone_id}")
    os.makedirs(work_dir, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)

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

    log_file = open(os.path.join(LOG_DIR, f"sitl_{drone_id}.log"), "w")

    log.info("Launching SITL %d (port %d): home=%s sysid=%d",
             drone_id, SITL_BASE_PORT + drone_id * SITL_PORT_STEP, home, sysid)

    proc = subprocess.Popen(
        cmd,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        cwd=work_dir,
        preexec_fn=os.setsid,
    )
    return proc


def kill_sitl_process(proc: subprocess.Popen):
    """Kill a SITL process and its entire process group."""
    if proc and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=5)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


class SITLLauncher:
    """Manages lifecycle of multiple ArduCopter SITL processes."""

    def __init__(self, num_drones: int):
        self.num_drones = num_drones
        self.processes: dict[int, subprocess.Popen] = {}

    def launch_instance(self, drone_id: int) -> subprocess.Popen:
        """Launch a single SITL instance."""
        proc = launch_sitl_instance(drone_id)
        self.processes[drone_id] = proc
        return proc

    def launch_all(self):
        """Launch all SITL instances with staggered timing."""
        log.info("Launching %d SITL instances...", self.num_drones)
        for i in range(1, self.num_drones + 1):
            self.launch_instance(i)
            time.sleep(2)

    def kill_instance(self, drone_id: int):
        """Kill a single SITL instance and its process group."""
        proc = self.processes.pop(drone_id, None)
        if proc:
            kill_sitl_process(proc)
            log.info("SITL instance %d killed", drone_id)

    def kill_all(self):
        """Kill all SITL instances."""
        log.info("Killing all SITL instances...")
        for drone_id in list(self.processes.keys()):
            self.kill_instance(drone_id)

    def is_alive(self, drone_id: int) -> bool:
        """Check if a SITL instance is still running."""
        proc = self.processes.get(drone_id)
        return proc is not None and proc.poll() is None
