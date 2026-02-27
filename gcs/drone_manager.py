"""
Drone process lifecycle manager.
Handles launching, killing, and monitoring drone subprocesses.
"""

import os
import sys
import signal
import subprocess
import threading
import logging

from config import PROJECT_DIR, LOG_DIR, DOCKER_MODE

log = logging.getLogger(__name__)


class DroneManager:
    """Manages drone subprocess lifecycles."""

    def __init__(self, max_drones: int = 10):
        self.max_drones = max_drones
        self._processes: dict[int, subprocess.Popen] = {}
        self._log_files: dict[int, object] = {}
        self._lock = threading.Lock()

    def launch(self, drone_id: int) -> dict:
        """Launch SITL + agent for drone_id. Returns status dict.
        In Docker mode, drones run as separate containers — skip subprocess launch."""
        if DOCKER_MODE:
            log.info("Docker mode: drone %d managed by docker-compose (skipping launch)",
                     drone_id)
            return {"ok": True, "pid": 0, "docker": True}

        with self._lock:
            if drone_id in self._processes:
                proc = self._processes[drone_id]
                if proc.poll() is None:
                    return {"ok": False, "error": f"Drone {drone_id} already running"}

            run_script = os.path.join(
                PROJECT_DIR, "drones", f"drone_{drone_id}", "run.py"
            )
            if not os.path.exists(run_script):
                return {"ok": False, "error": f"No run.py for drone {drone_id}"}

            os.makedirs(LOG_DIR, exist_ok=True)
            log_path = os.path.join(LOG_DIR, f"drone_{drone_id}.log")
            log_file = open(log_path, "w")

            env = os.environ.copy()
            # Ensure PYTHONPATH includes project root
            pp = env.get("PYTHONPATH", "")
            if PROJECT_DIR not in pp:
                env["PYTHONPATH"] = PROJECT_DIR + (":" + pp if pp else "")

            try:
                proc = subprocess.Popen(
                    [sys.executable, run_script],
                    cwd=PROJECT_DIR,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid,
                    env=env,
                )
            except Exception as e:
                log_file.close()
                return {"ok": False, "error": f"Failed to launch drone {drone_id}: {e}"}
            self._processes[drone_id] = proc
            self._log_files[drone_id] = log_file

            log.info("Launched drone %d (PID %d) -> %s", drone_id, proc.pid, log_path)
            return {"ok": True, "pid": proc.pid}

    def kill(self, drone_id: int) -> dict:
        """Kill drone process and its SITL children."""
        with self._lock:
            proc = self._processes.get(drone_id)
            if proc is None:
                return {"ok": False, "error": f"No process for drone {drone_id}"}
            if proc.poll() is not None:
                self._cleanup(drone_id)
                return {"ok": False, "error": f"Drone {drone_id} already dead"}

            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGTERM)
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            except ProcessLookupError:
                pass

            self._cleanup(drone_id)
            log.info("Killed drone %d", drone_id)
            return {"ok": True}

    def _cleanup(self, drone_id: int):
        """Remove process and close log file."""
        self._processes.pop(drone_id, None)
        lf = self._log_files.pop(drone_id, None)
        if lf:
            try:
                lf.close()
            except Exception:
                pass

    def is_running(self, drone_id: int) -> bool:
        with self._lock:
            proc = self._processes.get(drone_id)
            return proc is not None and proc.poll() is None

    def get_status(self, drone_id: int) -> str:
        """Return 'running', 'stopped', or 'unmanaged'."""
        if DOCKER_MODE:
            return "docker"
        with self._lock:
            proc = self._processes.get(drone_id)
            if proc is None:
                return "unmanaged"
            return "running" if proc.poll() is None else "stopped"

    def get_all_status(self) -> dict[int, str]:
        result = {}
        for did in range(1, self.max_drones + 1):
            result[did] = self.get_status(did)
        return result

    def get_pids(self) -> dict[int, int]:
        with self._lock:
            return {did: p.pid for did, p in self._processes.items()
                    if p.poll() is None}

    def kill_all(self):
        if DOCKER_MODE:
            log.info("Docker mode: drones managed by docker-compose, kill_all is a no-op")
            return
        for did in list(self._processes.keys()):
            self.kill(did)
