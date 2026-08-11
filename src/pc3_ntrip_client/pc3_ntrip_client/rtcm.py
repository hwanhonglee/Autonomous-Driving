"""Incremental RTCM3 framing with CRC-24Q validation."""

# HH_260811 - Reject corrupt RTCM3 frames before topic fanout or receiver injection.

from typing import List


CRC24Q_POLYNOMIAL = 0x1864CFB


def crc24q(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= CRC24Q_POLYNOMIAL
    return crc & 0xFFFFFF


def message_type(frame: bytes) -> int:
    if len(frame) < 8:
        return 0
    return (frame[3] << 4) | (frame[4] >> 4)


class Rtcm3Parser:
    """Accept arbitrary byte chunks and emit only complete, CRC-valid RTCM3 frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.bytes_received = 0
        self.frames_valid = 0
        self.crc_errors = 0
        self.bytes_discarded = 0

    def feed(self, data: bytes) -> List[bytes]:
        self.bytes_received += len(data)
        self._buffer.extend(data)
        frames = []

        while self._buffer:
            preamble = self._buffer.find(0xD3)
            if preamble < 0:
                self.bytes_discarded += len(self._buffer)
                self._buffer.clear()
                break
            if preamble:
                self.bytes_discarded += preamble
                del self._buffer[:preamble]
            if len(self._buffer) < 3:
                break
            # The upper six bits after the preamble are reserved and must be zero.
            if self._buffer[1] & 0xFC:
                self.bytes_discarded += 1
                del self._buffer[0]
                continue

            payload_length = ((self._buffer[1] & 0x03) << 8) | self._buffer[2]
            frame_length = payload_length + 6
            if len(self._buffer) < frame_length:
                break

            frame = bytes(self._buffer[:frame_length])
            expected = int.from_bytes(frame[-3:], byteorder="big")
            if crc24q(frame[:-3]) == expected:
                del self._buffer[:frame_length]
                self.frames_valid += 1
                frames.append(frame)
            else:
                # Discard just this candidate preamble, then search again. This also
                # recovers if a valid packet follows a truncated/corrupt packet.
                self.crc_errors += 1
                self.bytes_discarded += 1
                del self._buffer[0]

        return frames
