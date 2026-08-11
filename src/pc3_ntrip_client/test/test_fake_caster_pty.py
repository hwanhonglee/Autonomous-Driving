import base64
import os
import pty
import select
import socket
import threading

# HH_260811 - Exercise the full local caster-to-pseudo-TTY correction path.

from pc3_ntrip_client.config import NtripConfig
from pc3_ntrip_client.ntrip import NtripStream
from pc3_ntrip_client.rtcm import crc24q, Rtcm3Parser
from pc3_ntrip_client.serial_sink import NOVATEL_RTCM_INPUT_COMMAND, SerialSink


def make_frame(payload: bytes) -> bytes:
    header = bytes((0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF))
    packet = header + payload
    return packet + crc24q(packet).to_bytes(3, "big")


def test_local_fake_caster_to_pseudo_tty():
    first = make_frame(bytes((0x43, 0x20, 1, 2, 3, 4)))
    bad = bytearray(make_frame(bytes((0x44, 0xA0, 5, 6))))
    bad[-2] ^= 0x80
    second = make_frame(bytes((0x46, 0x50, 7, 8, 9)))
    wire_data = first + bytes(bad) + second

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    caster_port = listener.getsockname()[1]
    request_holder = []
    caster_errors = []

    def fake_caster():
        try:
            connection, _ = listener.accept()
            with connection:
                request = bytearray()
                while b"\r\n\r\n" not in request:
                    request.extend(connection.recv(1024))
                request_holder.append(bytes(request))
                header = (
                    b"HTTP/1.1 200 OK\r\n"
                    b"Content-Type: gnss/data\r\n"
                    b"Transfer-Encoding: chunked\r\n\r\n"
                )
                # Split RTCM frames at arbitrary boundaries and use HTTP chunking.
                pieces = (wire_data[:2], wire_data[2:11], wire_data[11:])
                encoded = b"".join(
                    f"{len(piece):X}\r\n".encode() + piece + b"\r\n" for piece in pieces
                ) + b"0\r\n\r\n"
                connection.sendall(header + encoded[:7])
                connection.sendall(encoded[7:])
        except Exception as exc:  # surfaced in the test thread below
            caster_errors.append(exc)
        finally:
            listener.close()

    caster_thread = threading.Thread(target=fake_caster)
    caster_thread.start()

    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)
    sink = SerialSink(slave_path, 115200, 0.1, configure_novatel_rtcm_input=True)
    parser = Rtcm3Parser()
    received_frames = []

    config = NtripConfig(
        host="127.0.0.1",
        port=caster_port,
        mountpoint="TEST-MOUNT",
        username="fake-user",
        password="fake-password",
        protocol_version="2.0",
        connect_timeout_sec=1.0,
        read_timeout_sec=1.0,
        reconnect_initial_sec=0.1,
        reconnect_max_sec=1.0,
    )

    def consume(chunk):
        for frame in parser.feed(chunk):
            received_frames.append(frame)
            assert sink.write(frame)

    try:
        NtripStream(config).stream(threading.Event(), consume)
        caster_thread.join(timeout=2.0)
        assert not caster_errors
        assert received_frames == [first, second]
        assert parser.crc_errors == 1

        expected = NOVATEL_RTCM_INPUT_COMMAND + first + second
        output = bytearray()
        while len(output) < len(expected):
            readable, _, _ = select.select([master_fd], [], [], 1.0)
            assert readable
            output.extend(os.read(master_fd, 4096))
        assert bytes(output) == expected
        assert sink.setup_attempts == 1
        assert sink.setup_successes == 1
        assert sink.setup_succeeded
        # Receiver setup bytes are deliberately excluded from RTCM accounting.
        assert sink.bytes_written == len(first) + len(second)

        request = request_holder[0]
        token = base64.b64encode(b"fake-user:fake-password")
        assert b"GET /TEST-MOUNT HTTP/1.1\r\n" in request
        assert b"Authorization: Basic " + token + b"\r\n" in request
        assert b"Ntrip-Version: Ntrip/2.0\r\n" in request
    finally:
        sink.close()
        os.close(master_fd)
        os.close(slave_fd)
