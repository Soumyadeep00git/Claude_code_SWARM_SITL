"""Launch and manage a single ArduCopter SITL instance."""

import logging
import os
import signal
import socket
import subprocess
import time

from docker_sim.config import (
    ARDUCOPTER_BIN, COPTER_DEFAULTS,
    DRONE_ID, SYSID, SITL_PORT, MP_PORT, HOME_STR, WORK_DIR,
)

log = logging.getLogger(__name__)


class SITLLauncher:
    """Manages lifecycle of one ArduCopter SITL process."""

    def __init__(self):
        self.proc: subprocess.Popen | None = None

    def launch(self):
        """Launch the ArduCopter SITL instance."""
        if not os.path.isfile(ARDUCOPTER_BIN):
            raise FileNotFoundError(
                f"ArduCopter binary not found at {ARDUCOPTER_BIN}\n"
                f"Build: cd ~/ardupilot && ./waf configure --board sitl && ./waf copter")

        os.makedirs(WORK_DIR, exist_ok=True)

        # Build defaults list: stock copter params + Mission Planner serial
        mp_params = os.environ.get("MP_PARAMS", "")
        defaults = COPTER_DEFAULTS
        if mp_params and os.path.isfile(mp_params):
            defaults = f"{COPTER_DEFAULTS},{mp_params}"

        cmd = [
            ARDUCOPTER_BIN,
            "--model", "+",
            "--speedup", "1",
            "--slave", "0",
            "--defaults", defaults,
            f"--serial2=tcp:{MP_PORT}",
            f"-I{DRONE_ID}",
            "--home", HOME_STR,
            "--sysid", str(SYSID),
        ]

        log.info("Launching SITL drone_id=%d sysid=%d port=%d mp_port=%d home=%s",
                 DRONE_ID, SYSID, SITL_PORT, MP_PORT, HOME_STR)

        self.proc = subprocess.Popen(
            cmd,
            cwd=WORK_DIR,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )

    def wait_ready(self, timeout: float = 90.0) -> bool:
        """Wait for SITL TCP port to become reachable."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                with socket.create_connection(("127.0.0.1", SITL_PORT), timeout=2):
                    log.info("SITL port %d reachable", SITL_PORT)
                    return True
            except (ConnectionRefusedError, OSError):
                time.sleep(1)
        return False

    def is_alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def kill(self):
        """Terminate the SITL process."""
        if self.proc is None or self.proc.poll() is not None:
            return
        try:
            pgid = os.getpgid(self.proc.pid)
            os.killpg(pgid, signal.SIGTERM)
            log.info("Sent SIGTERM to SITL (pgid=%d)", pgid)
        except ProcessLookupError:
            pass

        try:
            self.proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                pgid = os.getpgid(self.proc.pid)
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass

        log.info("SITL process terminated")
