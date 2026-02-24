#!/usr/bin/env bash
set -euo pipefail

PROJ_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== Swarm SITL Stack Setup ==="

# ── Step 1: System packages ──
echo "[1/5] Installing system dependencies..."
sudo apt-get update -qq
sudo apt-get install -y -qq git python3 python3-pip python3-venv \
    build-essential cmake

# ── Step 2: Python venv ──
echo "[2/5] Creating Python virtual environment..."
if [ ! -d "$PROJ_DIR/venv" ]; then
    python3 -m venv "$PROJ_DIR/venv"
fi
source "$PROJ_DIR/venv/bin/activate"

# ── Step 3: Python packages ──
echo "[3/5] Installing Python packages..."
pip install --upgrade pip -q
pip install -r "$PROJ_DIR/requirements.txt" -q

# ── Step 4: Clone ArduPilot ──
echo "[4/5] Setting up ArduPilot..."
if [ ! -d "$HOME/ardupilot" ]; then
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git \
        "$HOME/ardupilot"
    cd "$HOME/ardupilot"
    Tools/environment_install/install-prereqs-ubuntu.sh -y
    . ~/.profile
else
    echo "  ArduPilot already present at ~/ardupilot"
fi

# ── Step 5: Build ArduCopter SITL ──
echo "[5/5] Building ArduCopter SITL..."
cd "$HOME/ardupilot"
./waf configure --board sitl
./waf copter

echo ""
echo "=== Setup Complete ==="
echo "Activate:  source $PROJ_DIR/venv/bin/activate"
echo "Launch:    python -m scripts.launch_swarm"
