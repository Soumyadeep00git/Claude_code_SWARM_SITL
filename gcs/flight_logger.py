"""
Flight logger — records drone states to CSV and session metadata to JSON.
One CSV per drone + one aggregated CSV + one metadata JSON.
"""

import os
import csv
import json
import time
import logging
from datetime import datetime, timezone

from config import LOG_DIR, NUM_DRONES

log = logging.getLogger(__name__)

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output")

CSV_HEADER = [
    "timestamp", "elapsed_s", "drone_id",
    "lat", "lon", "alt",
    "vx", "vy", "vz",
    "heading", "battery_pct", "mode",
    "armed", "formation_slot", "failsafe_active",
    "swarm_state", "leader_id", "alive_count",
]


class FlightLogger:
    """Records flight data to CSV files and session metadata to JSON."""

    def __init__(self, num_drones: int = NUM_DRONES, output_dir: str = OUTPUT_DIR):
        self.num_drones = num_drones
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        self._start_time = time.time()
        self._start_dt = datetime.now(timezone.utc).isoformat()
        self._sample_count = 0
        self._alert_log: list[dict] = []
        self._commands_log: list[dict] = []

        # Aggregated CSV (all drones interleaved)
        agg_path = os.path.join(output_dir, "flight_log.csv")
        self._agg_file = open(agg_path, "w", newline="")
        self._agg_writer = csv.writer(self._agg_file)
        self._agg_writer.writerow(CSV_HEADER)

        # Per-drone CSVs
        self._drone_files = {}
        self._drone_writers = {}
        for i in range(1, num_drones + 1):
            path = os.path.join(output_dir, f"drone_{i}_log.csv")
            f = open(path, "w", newline="")
            w = csv.writer(f)
            w.writerow(CSV_HEADER)
            self._drone_files[i] = f
            self._drone_writers[i] = w

        log.info("Flight logger started — output: %s", output_dir)

    def log_state(self, drone_id: int, state_data: dict):
        """Record one state sample for a drone."""
        now = time.time()
        elapsed = now - self._start_time

        row = [
            f"{now:.3f}",
            f"{elapsed:.3f}",
            drone_id,
            f"{state_data.get('lat', 0):.7f}",
            f"{state_data.get('lon', 0):.7f}",
            f"{state_data.get('alt', 0):.2f}",
            f"{state_data.get('vx', 0):.3f}",
            f"{state_data.get('vy', 0):.3f}",
            f"{state_data.get('vz', 0):.3f}",
            f"{state_data.get('heading', 0):.1f}",
            state_data.get("battery_pct", -1),
            state_data.get("mode", "?"),
            state_data.get("armed", False),
            state_data.get("formation_slot", -1),
            state_data.get("failsafe_active", False),
            state_data.get("swarm_state", "NOMINAL"),
            state_data.get("leader_id", 1),
            state_data.get("alive_count", 0),
        ]

        # Write to aggregated log
        self._agg_writer.writerow(row)

        # Write to per-drone log
        w = self._drone_writers.get(drone_id)
        if w:
            w.writerow(row)

        self._sample_count += 1

    def log_alert(self, drone_id: int, alert_data: dict):
        """Record a failsafe alert."""
        self._alert_log.append({
            "timestamp": time.time(),
            "elapsed_s": time.time() - self._start_time,
            "drone_id": drone_id,
            **alert_data,
        })

    def log_command(self, command_type: str, data: dict):
        """Record a command sent by the GCS."""
        self._commands_log.append({
            "timestamp": time.time(),
            "elapsed_s": time.time() - self._start_time,
            "type": command_type,
            "data": data,
        })

    def save_metadata(self, extra: dict = None):
        """Write session metadata JSON."""
        end_time = time.time()
        metadata = {
            "session": {
                "start_time_utc": self._start_dt,
                "end_time_utc": datetime.now(timezone.utc).isoformat(),
                "duration_s": round(end_time - self._start_time, 2),
                "num_drones": self.num_drones,
                "total_samples": self._sample_count,
            },
            "alerts": self._alert_log,
            "commands": self._commands_log,
        }
        if extra:
            metadata["session"].update(extra)

        path = os.path.join(self.output_dir, "session_metadata.json")
        with open(path, "w") as f:
            json.dump(metadata, f, indent=2, default=str)
        log.info("Metadata saved: %s", path)
        return path

    def close(self):
        """Flush and close all files."""
        self._agg_file.close()
        for f in self._drone_files.values():
            f.close()
        log.info("Flight logger closed — %d samples recorded", self._sample_count)
