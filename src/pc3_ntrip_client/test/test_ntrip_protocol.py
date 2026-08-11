import socket
import threading

# HH_260811 - Verify NTRIP v1/v2 response and HTTP chunk decoding behavior.

from pc3_ntrip_client.config import NtripConfig
from pc3_ntrip_client.ntrip import _parse_response, HttpChunkDecoder, NtripError, NtripStream
from pc3_ntrip_client.rtcm import crc24q


def test_accept_http_and_icy_success_responses():
    headers, body = _parse_response(
        b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\nabc"
    )
    assert headers["transfer-encoding"] == "chunked"
    assert body == b"abc"
    headers, body = _parse_response(b"ICY 200 OK\r\nContent-Type: gnss/data\r\n\r\nxyz")
    assert headers["content-type"] == "gnss/data"
    assert body == b"xyz"


def test_incremental_chunked_decoder():
    decoder = HttpChunkDecoder()
    assert decoder.feed(b"3\r\na") == b""
    assert decoder.feed(b"bc\r\n2\r\nde\r\n") == b"abcde"
    assert decoder.feed(b"0\r\n\r\n") == b""
    assert decoder.complete


def test_legacy_icy_single_line_response():
    payload = bytes((0x43, 0x20, 1))
    packet = bytes((0xD3, 0, len(payload))) + payload
    frame = packet + crc24q(packet).to_bytes(3, "big")
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)

    def caster():
        connection, _ = listener.accept()
        with connection:
            request = bytearray()
            while b"\r\n\r\n" not in request:
                request.extend(connection.recv(1024))
            connection.sendall(b"ICY 200 OK\r\n" + frame)
        listener.close()

    thread = threading.Thread(target=caster)
    thread.start()
    config = NtripConfig(
        host="127.0.0.1",
        port=listener.getsockname()[1],
        mountpoint="TEST",
        username="user",
        password="password",
        protocol_version="1.0",
        connect_timeout_sec=1.0,
        read_timeout_sec=1.0,
        reconnect_initial_sec=0.1,
        reconnect_max_sec=1.0,
    )
    chunks = []
    try:
        NtripStream(config).stream(threading.Event(), chunks.append)
    except NtripError as exc:
        assert "disconnected" in str(exc)
    thread.join(timeout=2.0)
    assert b"".join(chunks) == frame
