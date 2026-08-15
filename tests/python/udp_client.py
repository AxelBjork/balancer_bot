from __future__ import annotations

import socket
from typing import Any


class UdpClient:
    BRIDGE_HOST = "127.0.0.1"
    BRIDGE_PORT = 9000

    def __init__(self, bridge_host: str = BRIDGE_HOST, bridge_port: int = BRIDGE_PORT):
        self._bridge = (bridge_host, bridge_port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Full-rate simulator telemetry is intentionally emitted at 400 Hz.  A
        # large receive queue prevents a fast deterministic run from dropping
        # its terminal SimRunDone datagram while the client drains telemetry.
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 32 * 1024 * 1024)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.settimeout(1.0)

    def register(self) -> None:
        self._sock.sendto(b"\x00\x00", self._bridge)

    def send(self, msg_id: int, payload: bytes = b"") -> None:
        self._sock.sendto(int(msg_id).to_bytes(2, "little") + payload, self._bridge)

    def recv(self, timeout: float | None = None) -> tuple[int, bytes]:
        old_timeout = self._sock.gettimeout()
        if timeout is not None:
            self._sock.settimeout(timeout)
        try:
            while True:
                data, _addr = self._sock.recvfrom(4096)
                if len(data) < 2:
                    continue
                return int.from_bytes(data[:2], "little"), data[2:]
        finally:
            if timeout is not None:
                self._sock.settimeout(old_timeout)

    def drain(self) -> None:
        old_timeout = self._sock.gettimeout()
        self._sock.settimeout(0.0)
        try:
            while True:
                self._sock.recvfrom(4096)
        except (BlockingIOError, TimeoutError, socket.timeout):
            pass
        finally:
            self._sock.settimeout(old_timeout)

    def close(self) -> None:
        self._sock.close()

    def __enter__(self) -> "UdpClient":
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()
