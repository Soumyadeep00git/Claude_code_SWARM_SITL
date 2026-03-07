#!/usr/bin/env bash
# ================================================================
#  Orin Swarm — one-shot setup script
# ================================================================
#  Run on each Jetson Orin AGX:
#    chmod +x install.sh && ./install.sh leader   # Leader Orin
#    chmod +x install.sh && ./install.sh follower  # Follower Orin
#
#  What it does:
#    1. Checks ROS2 Humble is installed
#    2. Installs MAVROS2 + pip dependencies
#    3. Downloads GeographicLib datasets (MAVROS2 requirement)
#    4. Creates ~/swarm_ws/src workspace
#    5. Symlinks the correct packages into the workspace
#    6. Copies hierarchy.yaml to /home/orin/
#    7. Adds PYTHONPATH for guidance_lib/failsafe_lib to .bashrc
#    8. Builds with colcon
# ================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROLE="${1:-}"

if [[ "$ROLE" != "leader" && "$ROLE" != "follower" ]]; then
    echo "Usage: $0 <leader|follower>"
    echo ""
    echo "  $0 leader    — set up Leader Orin"
    echo "  $0 follower  — set up Follower Orin"
    exit 1
fi

echo "================================================"
echo "  Orin Swarm Setup — role: $ROLE"
echo "================================================"

# ── 1. Check ROS2 Humble ─────────────────────────────────────
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "[ERROR] ROS2 Humble not found at /opt/ros/humble/setup.bash"
    echo "Install ROS2 Humble first: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi
source /opt/ros/humble/setup.bash
echo "[OK] ROS2 Humble found"

# ── 2. Install MAVROS2 + system dependencies ─────────────────
echo "[STEP] Installing MAVROS2 and dependencies..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    python3-pip \
    python3-yaml \
    2>/dev/null
echo "[OK] MAVROS2 installed"

# ── 3. Install pip dependencies ───────────────────────────────
echo "[STEP] Installing Python dependencies..."
pip3 install --user pymavlink pyyaml 2>/dev/null
echo "[OK] pymavlink + pyyaml installed"

# ── 4. GeographicLib datasets (MAVROS2 needs these) ──────────
GEOGRAPHICLIB_SCRIPT="/opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh"
if [ -f "$GEOGRAPHICLIB_SCRIPT" ]; then
    if [ ! -d "/usr/share/GeographicLib/geoids" ]; then
        echo "[STEP] Installing GeographicLib datasets (this may take a minute)..."
        sudo "$GEOGRAPHICLIB_SCRIPT"
        echo "[OK] GeographicLib datasets installed"
    else
        echo "[OK] GeographicLib datasets already present"
    fi
else
    echo "[WARN] GeographicLib install script not found — MAVROS2 may fail"
    echo "  Try: sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh"
fi

# ── 5. Create workspace ──────────────────────────────────────
WS=~/swarm_ws
echo "[STEP] Setting up workspace at $WS..."
mkdir -p "$WS/src"

# Symlink swarm_msgs (always needed)
ln -sfn "$SCRIPT_DIR/swarm_msgs" "$WS/src/swarm_msgs"

# Symlink guidance_lib and failsafe_lib
ln -sfn "$SCRIPT_DIR/guidance_lib" "$WS/src/guidance_lib"
ln -sfn "$SCRIPT_DIR/failsafe_lib" "$WS/src/failsafe_lib"

# Symlink role-specific package
if [ "$ROLE" = "leader" ]; then
    ln -sfn "$SCRIPT_DIR/Leader" "$WS/src/Leader"
    echo "[OK] Leader package symlinked"
elif [ "$ROLE" = "follower" ]; then
    ln -sfn "$SCRIPT_DIR/Follower" "$WS/src/Follower"
    echo "[OK] Follower package symlinked"
fi

# ── 6. Copy hierarchy.yaml ───────────────────────────────────
if [ -f "$SCRIPT_DIR/hierarchy.yaml" ]; then
    cp "$SCRIPT_DIR/hierarchy.yaml" ~/hierarchy.yaml
    echo "[OK] hierarchy.yaml copied to ~/hierarchy.yaml"
    echo "  >>> EDIT ~/hierarchy.yaml with your GPS home position before flight <<<"
    echo "  >>> hierarchy_path in params.yaml must point to /home/$(whoami)/hierarchy.yaml <<<"
fi

# ── 7. Add PYTHONPATH to .bashrc ─────────────────────────────
PYPATH_LINE="export PYTHONPATH=$WS/src:\$PYTHONPATH"
ROS_SOURCE_LINE="source /opt/ros/humble/setup.bash"
WS_SOURCE_LINE="source $WS/install/setup.bash 2>/dev/null || true"

# Add ROS2 source
if ! grep -qF "$ROS_SOURCE_LINE" ~/.bashrc 2>/dev/null; then
    echo "$ROS_SOURCE_LINE" >> ~/.bashrc
    echo "[OK] Added ROS2 source to .bashrc"
fi

# Add workspace source
if ! grep -qF "$WS_SOURCE_LINE" ~/.bashrc 2>/dev/null; then
    echo "$WS_SOURCE_LINE" >> ~/.bashrc
    echo "[OK] Added workspace source to .bashrc"
fi

# Add PYTHONPATH for guidance/failsafe libs
if ! grep -qF "PYTHONPATH=$WS/src" ~/.bashrc 2>/dev/null; then
    echo "$PYPATH_LINE" >> ~/.bashrc
    echo "[OK] Added PYTHONPATH to .bashrc (guidance_lib + failsafe_lib)"
fi

# ── 8. Copy preflight check script ────────────────────────────
if [ -f "$SCRIPT_DIR/preflight_check.py" ]; then
    cp "$SCRIPT_DIR/preflight_check.py" ~/preflight_check.py
    chmod +x ~/preflight_check.py
    echo "[OK] preflight_check.py copied to ~/preflight_check.py"
fi

# ── 9. Build ─────────────────────────────────────────────────
echo "[STEP] Building ROS2 packages..."
cd "$WS"
source /opt/ros/humble/setup.bash

if [ "$ROLE" = "leader" ]; then
    colcon build --packages-select swarm_msgs leader_pkg --symlink-install
elif [ "$ROLE" = "follower" ]; then
    colcon build --packages-select swarm_msgs follower_pkg --symlink-install
fi

source "$WS/install/setup.bash"
echo "[OK] Build complete"

# ── Done ──────────────────────────────────────────────────────
echo ""
echo "================================================"
echo "  Setup complete! ($ROLE)"
echo "================================================"
echo ""
echo "Before first flight:"
echo "  1. Edit ~/hierarchy.yaml — set your GPS home position"
if [ "$ROLE" = "leader" ]; then
    echo "  2. Edit $WS/src/Leader/config/params.yaml — set gcs_host, serial devices"
    echo "  3. Verify serial ports: ls /dev/ttyUSB*"
    echo "  4. Launch: ros2 launch leader_pkg leader_bringup.launch.py"
elif [ "$ROLE" = "follower" ]; then
    echo "  2. Edit $WS/src/Follower/config/params.yaml — set gcs_host, serial devices"
    echo "  3. Verify serial ports: ls /dev/ttyUSB*"
    echo "  4. Launch: ros2 launch follower_pkg follower_bringup.launch.py"
fi
echo ""
echo "To rebuild after code changes:"
echo "  cd $WS && colcon build --symlink-install && source install/setup.bash"
echo ""
echo "Pre-flight check (run after launching MAVROS2):"
echo "  python3 ~/preflight_check.py"
echo ""
