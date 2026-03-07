#!/usr/bin/env python3
"""GCS Time-Sync Server — master clock for Jetson fleet.

Runs on the GCS laptop. Sends periodic time-sync beacons to all
Jetsons via UDP, and responds to sync requests with precise
round-trip offset calculations.

Protocol (all messages are struct-packed, no framing needed on UDP):

  BEACON   (GCS → Jetson, every 1s):
      !Bd     tag=0x01, gcs_unix_time

  SYNC_REQ (Jetson → GCS):
      !Bd     tag=0x02, jetson_t1

  SYNC_RESP (GCS → Jetson):
      !Bddd   tag=0x03, jetson_t1, gcs_t2, gcs_t3

  The Jetson computes offset = ((gcs_t2 - jetson_t1) + (gcs_t3 - jetson_t4)) / 2
  where jetson_t4 is the local receive time of SYNC_RESP.

Usage:
  python3 time_sync_server.py

  Env vars (optional):
    SYNC_PORT     — UDP port (default 14590)
    BEACON_HZ     — beacon rate (default 1.0)
"""

import os
import socket
import struct
import time
import threading

SYNC_PORT = int(os.environ.get('SYNC_PORT', '14590'))
BEACON_HZ = float(os.environ.get('BEACON_HZ', '1.0'))

# Tags
TAG_BEACON = 0x01
TAG_SYNC_REQ = 0x02
TAG_SYNC_RESP = 0x03

# Target Jetsons — add more as needed
TARGETS = [
    ('172.16.0.159', SYNC_PORT),   # Leader  (iris1)
    ('172.16.0.199', SYNC_PORT),   # Follower (gaurav)
]

_BEACON_FMT = '!Bd'       # tag(1) + time(8) = 9 bytes
_REQ_FMT = '!Bd'          # tag(1) + t1(8) = 9 bytes
_RESP_FMT = '!Bddd'       # tag(1) + t1(8) + t2(8) + t3(8) = 25 bytes


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('0.0.0.0', SYNC_PORT))
    sock.settimeout(0.5)

    print(f"[TimeSync] Listening on :{SYNC_PORT}, beacon rate={BEACON_HZ} Hz")
    print(f"[TimeSync] Targets: {TARGETS}")

    # ── Beacon thread ─────────────────────────────────────────
    def _beacon_loop():
        interval = 1.0 / BEACON_HZ
        while True:
            now = time.time()
            pkt = struct.pack(_BEACON_FMT, TAG_BEACON, now)
            for addr in TARGETS:
                try:
                    sock.sendto(pkt, addr)
                except OSError:
                    pass
            time.sleep(interval)

    threading.Thread(target=_beacon_loop, daemon=True).start()

    # ── Response loop (handles SYNC_REQ from Jetsons) ─────────
    while True:
        try:
            data, addr = sock.recvfrom(64)
        except socket.timeout:
            continue
        except KeyboardInterrupt:
            break

        if len(data) < 1:
            continue

        tag = data[0]
        if tag == TAG_SYNC_REQ and len(data) >= struct.calcsize(_REQ_FMT):
            gcs_t2 = time.time()
            _, jetson_t1 = struct.unpack(_REQ_FMT, data[:struct.calcsize(_REQ_FMT)])
            gcs_t3 = time.time()
            resp = struct.pack(_RESP_FMT, TAG_SYNC_RESP, jetson_t1, gcs_t2, gcs_t3)
            sock.sendto(resp, addr)

    print("[TimeSync] Shutting down")


if __name__ == '__main__':
    main()
