#!/usr/bin/env python3
"""Generate docker-compose.yml from hierarchy.yaml.

Usage:
    python docker/generate_compose.py [hierarchy.yaml]
    # Writes to docker/docker-compose.yml

Or to stdout:
    python docker/generate_compose.py hierarchy.yaml --stdout
"""

import os
import sys

# Add project root to path
_PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _PROJECT_DIR)

import yaml
from swarm_lib.hierarchy import SwarmTopology
from swarm_lib.swarm_config import (
    container_name, mp_port, gcs_telem_port, gcs_cmd_port,
)


def generate(hierarchy_path: str) -> dict:
    topo = SwarmTopology.from_yaml(hierarchy_path)
    errors = topo.validate()
    if errors:
        print(f"ERROR: Hierarchy validation failed: {errors}", file=sys.stderr)
        sys.exit(1)

    services = {}

    # ── Drone services ──
    for did in topo.all_drone_ids():
        node = topo.get_node(did)
        cname = container_name(did)
        m_port = mp_port(did)

        svc = {
            "build": {"context": "..", "dockerfile": "docker/Dockerfile"},
            "container_name": cname,
            "ports": [f"{m_port}:{m_port}"],
            "environment": {
                "DRONE_ID": str(did),
                "HIERARCHY_PATH": "/app/hierarchy.yaml",
                "GCS_HOST": "gcs",
            },
            "networks": ["drone_net"],
            "volumes": [
                "../output:/app/output",
            ],
            "stdin_open": True,
            "tty": True,
        }

        # Followers depend on their leader starting first
        if node.leader_id is not None:
            leader_cname = container_name(node.leader_id)
            svc["depends_on"] = [leader_cname]

        services[cname] = svc

    # ── GCS service ──
    services["gcs"] = {
        "build": {"context": "..", "dockerfile": "docker/Dockerfile.gcs"},
        "container_name": "gcs",
        "environment": {
            "HIERARCHY_PATH": "/app/hierarchy.yaml",
            "WEB_PORT": "5000",
        },
        "networks": ["drone_net"],
        "ports": ["5000:5000"],
        "stdin_open": True,
        "tty": True,
    }

    compose = {
        "version": "3.8",
        "services": services,
        "networks": {"drone_net": {"driver": "bridge"}},
    }
    return compose


def main():
    hierarchy_path = sys.argv[1] if len(sys.argv) > 1 else "hierarchy.yaml"

    if not os.path.exists(hierarchy_path):
        print(f"ERROR: {hierarchy_path} not found", file=sys.stderr)
        sys.exit(1)

    compose = generate(hierarchy_path)
    output = yaml.dump(compose, default_flow_style=False, sort_keys=False)

    if "--stdout" in sys.argv:
        print(output)
    else:
        # Write to docker/docker-compose.yml
        out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "docker-compose.yml")
        with open(out_path, "w") as f:
            f.write(output)
        print(f"Generated: {out_path}")

        # Summary
        topo = SwarmTopology.from_yaml(hierarchy_path)
        print(f"  Drones: {len(topo.all_drone_ids())} "
              f"({len(topo.leader_ids())} leaders, {len(topo.follower_ids())} followers)")
        for did in topo.all_drone_ids():
            node = topo.get_node(did)
            cname = container_name(did)
            m = mp_port(did)
            print(f"  {cname} ({node.role.value}) — MP port: {m}")


if __name__ == "__main__":
    main()
