#!/usr/bin/env bash
set -e

echo "=== Drone container starting ==="
echo "  DRONE_ROLE : ${DRONE_ROLE:-leader}"
echo "  DRONE_ID   : ${DRONE_ID:-1}"
echo "  PEER_HOST  : ${PEER_HOST:-localhost}"

if [ "${DRONE_ROLE}" = "follower" ]; then
    exec python3 -m docker_sim.follower_main
else
    exec python3 -m docker_sim.leader_main
fi
