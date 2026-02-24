"""
Locate sim_vehicle.py on this system.

Search order:
  1. $SIM_VEHICLE_PATH  environment variable (exact path to the script)
  2. config.ARDUPILOT_DIR / Tools / autotest / sim_vehicle.py
  3. 'sim_vehicle.py' on PATH  (shutil.which)
  4. Common install locations (~/ and /opt)

Raises FileNotFoundError with install instructions if not found.
"""

import os
import shutil

from config import ARDUPILOT_DIR


def find_sim_vehicle() -> str:
    """Return the absolute path to sim_vehicle.py, or raise."""

    # 1. Explicit env var
    env_path = os.environ.get("SIM_VEHICLE_PATH")
    if env_path and os.path.isfile(env_path):
        return os.path.abspath(env_path)

    # 2. config.ARDUPILOT_DIR
    cfg_path = os.path.join(ARDUPILOT_DIR, "Tools", "autotest", "sim_vehicle.py")
    if os.path.isfile(cfg_path):
        return os.path.abspath(cfg_path)

    # 3. On PATH
    which = shutil.which("sim_vehicle.py")
    if which:
        return os.path.abspath(which)

    # 4. Common locations
    for base in [
        os.path.expanduser("~/ardupilot"),
        "/opt/ardupilot",
        os.path.expanduser("~/src/ardupilot"),
    ]:
        candidate = os.path.join(base, "Tools", "autotest", "sim_vehicle.py")
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)

    raise FileNotFoundError(
        "sim_vehicle.py not found.\n"
        "Tried:\n"
        f"  $SIM_VEHICLE_PATH  = {env_path!r}\n"
        f"  config.ARDUPILOT_DIR = {cfg_path}\n"
        "  PATH lookup\n"
        "  ~/ardupilot, /opt/ardupilot, ~/src/ardupilot\n\n"
        "Fix options:\n"
        "  export SIM_VEHICLE_PATH=/path/to/sim_vehicle.py\n"
        "  -or- set ARDUPILOT_DIR in config.py\n"
        "  -or- add ArduPilot's Tools/autotest to your PATH"
    )
