"""NTRIP v1/v2 TCP streaming without third-party runtime dependencies."""

# HH_260811 - Stream authenticated NTRIP data with sanitized failures and reconnect support.

import base64
import socket
import threading
from typing import Callable, Dict, Optional, Tuple

from .config import NtripConfig


class NtripError(RuntimeError):
    """A deliberately non-sensitive NTRIP failure."""


def build_request(config: NtripConfig) -> bytes:
    mountpoint = config.mountpoint.lstrip("/")
    token = base64.b64encode(
        (config.username + ":" + config.password).encode("utf-8")
    ).decode("ascii")
    http_version = "HTTP/1.0" if config.protocol_version == "1.0" else "HTTP/1.1"
    lines = [
        f"GET /{mountpoint} {http_version}",
        f"Host: {config.host}:{config.port}",
        "User-Agent: NTRIP pc3_ntrip_client/0.1",
        "Accept: */*",
        f"Authorization: Basic {token}",
        "Connection: close",
    ]
    if config.protocol_version == "2.0":
        lines.append("Ntrip-Version: Ntrip/2.0")
    return ("\r\n".join(lines) + "\r\n\r\n").encode("ascii")


def _parse_response(buffer: bytes) -> Tuple[Dict[str, str], bytes]:
    marker = buffer.find(b"\r\n\r\n")
    if marker < 0:
        raise NtripError("NTRIP caster returned an incomplete response header")
    header_blob = buffer[:marker]
    body = buffer[marker + 4:]
    lines = header_blob.split(b"\r\n")
    try:
        status_line = lines[0].decode("ascii", errors="strict")
    except (IndexError, UnicodeDecodeError) as exc:
        raise NtripError("NTRIP caster returned an invalid status line") from exc

    parts = status_line.split()
    is_http = len(parts) >= 2 and parts[0].startswith("HTTP/") and parts[1] == "200"
    is_icy = len(parts) >= 2 and parts[0] == "ICY" and parts[1] == "200"
    if not (is_http or is_icy):
        # Only the status code is safe and useful. Never include response headers.
        code = parts[1] if len(parts) >= 2 and parts[1].isdigit() else "unknown"
        raise NtripError(f"NTRIP caster rejected the request (status {code})")

    headers: Dict[str, str] = {}
    for line in lines[1:]:
        if b":" not in line:
            continue
        key, value = line.split(b":", 1)
        try:
            headers[key.decode("ascii").strip().lower()] = value.decode(
                "ascii", errors="strict"
            ).strip().lower()
        except UnicodeDecodeError as exc:
            raise NtripError("NTRIP caster returned an invalid response header") from exc
    return headers, body


class HttpChunkDecoder:
    """Incrementally decode HTTP chunked transfer coding."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._remaining: Optional[int] = None
        self.complete = False

    def feed(self, data: bytes) -> bytes:
        if self.complete:
            return b""
        self._buffer.extend(data)
        output = bytearray()
        while True:
            if self._remaining is None:
                marker = self._buffer.find(b"\r\n")
                if marker < 0:
                    if len(self._buffer) > 128:
                        raise NtripError("invalid HTTP chunk header")
                    break
                line = bytes(self._buffer[:marker]).split(b";", 1)[0]
                del self._buffer[: marker + 2]
                try:
                    self._remaining = int(line, 16)
                except ValueError as exc:
                    raise NtripError("invalid HTTP chunk size") from exc
                if self._remaining == 0:
                    self.complete = True
                    break
            assert self._remaining is not None
            if len(self._buffer) < self._remaining + 2:
                break
            output.extend(self._buffer[: self._remaining])
            if self._buffer[self._remaining:self._remaining + 2] != b"\r\n":
                raise NtripError("invalid HTTP chunk terminator")
            del self._buffer[: self._remaining + 2]
            self._remaining = None
        return bytes(output)


class NtripStream:
    MAX_HEADER_BYTES = 16384

    def __init__(self, config: NtripConfig) -> None:
        self.config = config
        self._socket: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def close(self) -> None:
        with self._lock:
            connection = self._socket
            self._socket = None
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                connection.close()
            except OSError:
                pass

    def stream(self, stop_event: threading.Event, on_data: Callable[[bytes], None]) -> None:
        try:
            connection = socket.create_connection(
                (self.config.host, self.config.port),
                timeout=self.config.connect_timeout_sec,
            )
            connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            connection.settimeout(self.config.read_timeout_sec)
        except OSError as exc:
            raise NtripError("NTRIP caster connection failed") from exc

        with self._lock:
            if stop_event.is_set():
                connection.close()
                return
            self._socket = connection

        try:
            connection.sendall(build_request(self.config))
            response = bytearray()
            legacy_icy_header_end: Optional[int] = None
            while b"\r\n\r\n" not in response and legacy_icy_header_end is None:
                if stop_event.is_set():
                    return
                try:
                    chunk = connection.recv(4096)
                except socket.timeout as exc:
                    raise NtripError("NTRIP caster response timed out") from exc
                if not chunk:
                    raise NtripError("NTRIP caster closed before streaming")
                response.extend(chunk)
                # NTRIP revision 1 casters are also permitted to return only
                # "ICY 200 OK\r\n" before the binary stream (no HTTP headers).
                first_line_end = response.find(b"\r\n")
                is_legacy_icy = response.startswith(b"ICY 200 OK\r\n")
                body_byte_available = len(response) > first_line_end + 2
                body_starts_rtcm = (
                    body_byte_available and response[first_line_end + 2] == 0xD3
                )
                if is_legacy_icy and first_line_end >= 0 and body_starts_rtcm:
                    legacy_icy_header_end = first_line_end
                if len(response) > self.MAX_HEADER_BYTES:
                    raise NtripError("NTRIP caster response header is too large")

            if legacy_icy_header_end is not None:
                body_start = legacy_icy_header_end + 2
                response = bytearray(
                    response[:legacy_icy_header_end] + b"\r\n\r\n" + response[body_start:]
                )
            headers, initial = _parse_response(bytes(response))
            chunked = "chunked" in headers.get("transfer-encoding", "")
            decoder = HttpChunkDecoder() if chunked else None
            first_payload = True

            def deliver(raw: bytes) -> None:
                nonlocal first_payload
                payload = decoder.feed(raw) if decoder is not None else raw
                if not payload:
                    return
                if first_payload:
                    first_payload = False
                    if payload.lstrip().startswith(b"SOURCETABLE"):
                        raise NtripError("caster returned a sourcetable instead of RTCM data")
                on_data(payload)

            deliver(initial)
            while not stop_event.is_set() and not (decoder and decoder.complete):
                try:
                    chunk = connection.recv(8192)
                except socket.timeout as exc:
                    raise NtripError("NTRIP correction stream timed out") from exc
                if not chunk:
                    raise NtripError("NTRIP correction stream disconnected")
                deliver(chunk)
        except OSError as exc:
            raise NtripError("NTRIP correction stream failed") from exc
        finally:
            self.close()
