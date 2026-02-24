"""
CPU affinity utilities for per-core process isolation.
Pins each swarm component to a dedicated CPU core, simulating
real-world deployment where each drone is a separate computer.
"""

import os
import logging

log = logging.getLogger(__name__)


def get_available_cores() -> list[int]:
    """Return sorted list of CPU cores available to this process."""
    return sorted(os.sched_getaffinity(0))


def assign_cores(num_drones: int) -> dict[str, int] | None:
    """Assign CPU cores to swarm roles.

    Returns mapping like {'orchestrator': 0, 'gcs': 1, 'drone_1': 2, ...}
    or None if not enough cores for full isolation.
    """
    available = get_available_cores()
    needed = num_drones + 2  # drones + GCS + orchestrator

    if len(available) < needed:
        log.warning(
            "Only %d cores available, need %d for full isolation. "
            "Running without CPU pinning.", len(available), needed,
        )
        return None

    assignments = {
        "orchestrator": available[0],
        "gcs": available[1],
    }
    for i in range(1, num_drones + 1):
        assignments[f"drone_{i}"] = available[1 + i]

    return assignments


def pin_process(pid: int, core_id: int) -> bool:
    """Pin a process to a specific CPU core. Returns True on success."""
    try:
        os.sched_setaffinity(pid, {core_id})
        return True
    except (ProcessLookupError, PermissionError, OSError) as e:
        log.warning("Failed to pin PID %d to core %d: %s", pid, core_id, e)
        return False


def _get_children(pid: int) -> list[int]:
    """Find child PIDs by reading /proc/{pid}/task/*/children."""
    children = []
    task_dir = f"/proc/{pid}/task"
    try:
        for tid in os.listdir(task_dir):
            children_file = f"{task_dir}/{tid}/children"
            try:
                with open(children_file) as f:
                    for child_pid in f.read().split():
                        children.append(int(child_pid))
            except (FileNotFoundError, PermissionError, ValueError):
                pass
    except FileNotFoundError:
        pass
    return children


ZSdef pin_process_tree(pid: int, core_id: int):
    """Recursively pin a process and all its descendants to a core."""
    pin_process(pid, core_id)
    for child in _get_children(pid):
        pin_process_tree(child, core_id)


def log_affinity_map(assignments: dict[str, int]):
    """Log the core assignment table."""
    log.info("=== Per-Core Isolation ===")
    log.info("  %-14s  %s", "Role", "Core")
    log.info("  %-14s  %s", "-" * 14, "----")
    for role, core in sorted(assignments.items(), key=lambda x: x[1]):
        log.info("  %-14s  %d", role, core)
