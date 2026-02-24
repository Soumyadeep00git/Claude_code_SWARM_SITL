#!/usr/bin/env bash
#
# Launch a single ArduCopter SITL instance.
#
# Usage:
#   ./sitl/start_sitl.sh <instance_id> <home_string>
#
# Example:
#   ./sitl/start_sitl.sh 1 "-35.3632620,149.1652370,584,270"
#
# The script locates sim_vehicle.py using the same search order as
# sitl/find_sim_vehicle.py:
#   1. $SIM_VEHICLE_PATH
#   2. ~/ardupilot/Tools/autotest/sim_vehicle.py
#   3. sim_vehicle.py on PATH
#

set -euo pipefail

if [ $# -lt 2 ]; then
    echo "Usage: $0 <instance_id> <home_string>"
    echo "  instance_id : integer (1-based drone ID)"
    echo "  home_string : lat,lon,alt,heading"
    exit 1
fi

INSTANCE_ID="$1"
HOME_STR="$2"

# ── Locate sim_vehicle.py ──────────────────────────────────
if [ -n "${SIM_VEHICLE_PATH:-}" ] && [ -f "$SIM_VEHICLE_PATH" ]; then
    SIM_VEHICLE="$SIM_VEHICLE_PATH"
elif [ -f "$HOME/ardupilot/Tools/autotest/sim_vehicle.py" ]; then
    SIM_VEHICLE="$HOME/ardupilot/Tools/autotest/sim_vehicle.py"
elif command -v sim_vehicle.py &>/dev/null; then
    SIM_VEHICLE="$(command -v sim_vehicle.py)"
else
    echo "ERROR: sim_vehicle.py not found."
    echo "  Set \$SIM_VEHICLE_PATH or install ArduPilot to ~/ardupilot"
    exit 1
fi

echo "SITL instance $INSTANCE_ID: using $SIM_VEHICLE"
echo "  Home: $HOME_STR"

# ── Create working directory ───────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WORK_DIR="$SCRIPT_DIR/logs/sitl_instance_$INSTANCE_ID"
mkdir -p "$WORK_DIR"

# ── Launch ─────────────────────────────────────────────────
exec python3 "$SIM_VEHICLE" \
    -v ArduCopter \
    -I "$INSTANCE_ID" \
    --no-mavproxy \
    --auto-sysid \
    -l "$HOME_STR" \
    --speedup 1 \
    --wipe-eeprom
