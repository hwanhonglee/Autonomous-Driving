"""HH_261001 - Check serialized Unreal names and reject guessed package formats."""

import struct

import pytest

from scripts.e2e.audit_ctrack_imports_20261001 import Reader, decode_imports


def test_fname_internal_suffix_is_external_number_plus_one():
    assert Reader(struct.pack("<ii", 0, 3)).name(["T_Clouds"]) == "T_Clouds_2"
    assert Reader(struct.pack("<ii", 0, 0)).name(["Road"]) == "Road"
    with pytest.raises(ValueError, match="FName"):
        Reader(struct.pack("<ii", 1, 0)).name(["Road"])


def test_fstring_supports_utf16_and_checks_bounds():
    raw = "Road\0".encode("utf-16le")
    assert Reader(struct.pack("<i", -5) + raw).string() == "Road"
    with pytest.raises(ValueError, match="length exceeds"):
        Reader(struct.pack("<i", 100)).string()


def test_import_parser_refuses_cooked_or_unknown_version():
    payload = struct.pack("<Iiiii", 0x9E2A83C1, -7, 864, 0, 0)
    with pytest.raises(ValueError, match="unsupported package version"):
        decode_imports(payload)
