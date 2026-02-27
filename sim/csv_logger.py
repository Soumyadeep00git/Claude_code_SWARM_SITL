"""CSV flight data logger for SITL test."""

import csv
import logging
import math
import os
import time

from sim.config import OUTPUT_DIR, METERS_PER_DEG_LAT

log = logging.getLogger(__name__)

CSV_HEADER = [
    'timestamp', 'elapsed_s', 'role',
    'lat', 'lon', 'alt', 'vn', 've', 'vd',
    'mode', 'armed', 'phase',
    'target_lat', 'target_lon', 'err_n', 'err_e', 'err_total',
    'cmd_vn', 'cmd_ve', 'cmd_vd', 'cmd_speed',
    'peer_dist', 'emergency', 'flags',
    'guidance_mode', 'w_evasion', 'w_tracking', 'w_catchup', 'ff_gain',
]


class CSVLogger:
    """Logs flight data to CSV for post-analysis."""

    def __init__(self):
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        path = os.path.join(OUTPUT_DIR, f"sitl_test_{ts}.csv")

        self._file = open(path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(CSV_HEADER)
        self._start = time.time()
        self._buffer = []
        log.info("Logging to %s", path)

    def log_leader(self, pos: dict | None, hb: dict | None, phase: str):
        """Log one leader row."""
        now = time.time()
        row = [
            f'{now:.3f}', f'{now - self._start:.2f}', 'leader',
        ]

        if pos:
            row.extend([
                f'{pos["lat"]:.7f}', f'{pos["lon"]:.7f}', f'{pos["alt"]:.2f}',
                f'{pos["vx"]:.2f}', f'{pos["vy"]:.2f}', f'{pos["vz"]:.2f}',
            ])
        else:
            row.extend(['', '', '', '', '', ''])

        mode = hb['mode'] if hb else ''
        armed = hb['armed'] if hb else ''
        row.extend([mode, armed, phase])

        # Leader has no target/guidance columns
        # 5 (target/err) + 4 (cmd) + 3 (peer/emerg/flags) + 5 (guidance) = 17
        row.extend([''] * 17)

        self._buffer.append(row)
        self._maybe_flush()

    def log_follower(self, pos: dict | None, hb: dict | None,
                     phase: str, result: dict | None,
                     target_lat: float, target_lon: float):
        """Log one follower row."""
        now = time.time()
        row = [
            f'{now:.3f}', f'{now - self._start:.2f}', 'follower',
        ]

        if pos:
            row.extend([
                f'{pos["lat"]:.7f}', f'{pos["lon"]:.7f}', f'{pos["alt"]:.2f}',
                f'{pos["vx"]:.2f}', f'{pos["vy"]:.2f}', f'{pos["vz"]:.2f}',
            ])
        else:
            row.extend(['', '', '', '', '', ''])

        mode = hb['mode'] if hb else ''
        armed = hb['armed'] if hb else ''
        row.extend([mode, armed, phase])

        # Offset error
        if pos and target_lat != 0:
            en = (target_lat - pos['lat']) * METERS_PER_DEG_LAT
            ee = ((target_lon - pos['lon']) * METERS_PER_DEG_LAT
                  * math.cos(math.radians(pos['lat'])))
            et = math.sqrt(en * en + ee * ee)
            row.extend([f'{target_lat:.7f}', f'{target_lon:.7f}',
                         f'{en:.2f}', f'{ee:.2f}', f'{et:.2f}'])
        else:
            row.extend(['', '', '', '', ''])

        # Guidance output
        if result:
            row.extend([
                f'{result["vn"]:.3f}',
                f'{result["ve"]:.3f}',
                f'{result["vd"]:.3f}',
                f'{result["speed"]:.3f}',
                f'{result["peer_dist"]:.2f}',
                str(result.get('emergency', False)),
                str(result.get('flags', {})),
                result.get('mode', 'TRACKING'),
                f'{result.get("w_evasion", 0.0):.3f}',
                f'{result.get("w_tracking", 1.0):.3f}',
                f'{result.get("w_catchup", 0.0):.3f}',
                f'{result.get("ff_gain", 0.0):.3f}',
            ])
        else:
            row.extend([''] * 12)

        self._buffer.append(row)
        self._maybe_flush()

    def _maybe_flush(self):
        if len(self._buffer) >= 50:
            self._writer.writerows(self._buffer)
            self._buffer.clear()
            self._file.flush()

    def close(self):
        if self._buffer:
            self._writer.writerows(self._buffer)
            self._buffer.clear()
        self._file.flush()
        self._file.close()
        log.info("CSV logger closed")
