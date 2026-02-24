"""
Stop all swarm processes — drone systems, SITL instances, agents, and GCS.

Usage:
    python -m scripts.stop_swarm
"""

import subprocess
import sys


def kill_by_pattern(pattern: str):
    """Kill all processes matching a pattern."""
    result = subprocess.run(
        ["pgrep", "-f", pattern],
        capture_output=True, text=True,
    )
    pids = result.stdout.strip().split("\n")
    pids = [p for p in pids if p]

    if not pids:
        print(f"  No processes matching '{pattern}'")
        return

    for pid in pids:
        try:
            subprocess.run(["kill", pid], check=True)
            print(f"  Killed PID {pid} ({pattern})")
        except subprocess.CalledProcessError:
            # Try harder
            subprocess.run(["kill", "-9", pid], check=False)


def main():
    print("=== Stopping Swarm SITL Stack ===")

    print("\nStopping orchestrator...")
    kill_by_pattern("scripts.run_demo_isolated")

    print("\nStopping GCS...")
    kill_by_pattern("gcs.gcs")
    kill_by_pattern("gcs/run_gcs.py")

    print("\nStopping drone systems...")
    kill_by_pattern("drones/drone_.*/run.py")

    print("\nStopping drone agents...")
    kill_by_pattern("drone_agent.agent")

    print("\nStopping SITL instances...")
    kill_by_pattern("arducopter")
    kill_by_pattern("sim_vehicle.py")

    print("\nDone.")


if __name__ == "__main__":
    main()
