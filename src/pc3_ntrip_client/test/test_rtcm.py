from pc3_ntrip_client.rtcm import crc24q, message_type, Rtcm3Parser

# HH_260811 - Verify CRC validation and partial-stream resynchronization.


def make_frame(payload: bytes) -> bytes:
    header = bytes((0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF))
    packet = header + payload
    return packet + crc24q(packet).to_bytes(3, "big")


def test_crc24q_reference_vector():
    assert crc24q(b"123456789") == 0xCDE703


def test_partial_frames_crc_rejection_and_resynchronization():
    first = make_frame(bytes((0x43, 0x20, 1, 2, 3)))  # RTCM type 1074
    corrupt = bytearray(make_frame(bytes((0x44, 0xA0, 4, 5))))
    corrupt[-1] ^= 0x01
    second = make_frame(bytes((0x46, 0x50, 6, 7)))
    parser = Rtcm3Parser()

    assert parser.feed(b"junk" + first[:4]) == []
    output = parser.feed(first[4:] + bytes(corrupt) + second[:2])
    output += parser.feed(second[2:])

    assert output == [first, second]
    assert parser.frames_valid == 2
    assert parser.crc_errors == 1
    assert parser.bytes_discarded >= 5
    assert message_type(first) == 1074
