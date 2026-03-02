#!/usr/bin/env bash
set -e

echo "=== Drone container starting ==="
echo "  DRONE_ID       : ${DRONE_ID:-1}"
echo "  HIERARCHY_PATH : ${HIERARCHY_PATH:-/app/hierarchy.yaml}"

# Unified entry point — role comes from hierarchy, not env
exec python3 -m docker_sim.drone_main
