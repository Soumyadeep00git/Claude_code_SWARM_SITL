"""
Generic non-blocking UDP send/receive node.
Used by both GCS and drone agents.
"""

import socket
import select
import logging

log = logging.getLogger(__name__)


class UDPNode:
    """Non-blocking UDP socket for sending and receiving datagrams."""

    def __init__(self, listen_port: int, buf_size: int = 4096):
        self.port = listen_port
        self.buf_size = buf_size
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", listen_port))
        self.sock.setblocking(False)
        log.info("UDP node listening on port %d", listen_port)

    def send(self, data: bytes, host: str, port: int):
        """Send a datagram to host:port."""
        self.sock.sendto(data, (host, port))

    def recv_all(self) -> list[tuple[bytes, tuple[str, int]]]:
        """Drain all pending datagrams. Returns list of (data, addr)."""
        messages = []
        while True:
            ready, _, _ = select.select([self.sock], [], [], 0)
            if not ready:
                break
            try:
                data, addr = self.sock.recvfrom(self.buf_size)
                messages.append((data, addr))
            except BlockingIOError:
                break
        return messages

    def close(self):
        """Close the socket."""
        self.sock.close()
        log.info("UDP node on port %d closed", self.port)
