"""HH_261008 - Prepared bounded synthetic ZIP/NPY tests; no real data, training or model jobs.

Fixtures stream <=64KiB blocks into small compressed archives; they never create
729/480 full arrays or extracted NPY files. All fixture writes together are
capped at32MiB. The production-count fixture is logically large but compressed
zero runs are small. Tests inspect early slots; no production-count tail replay.
"""

import ast
import hashlib
import os
from pathlib import Path
import struct
import zipfile

import pytest

from portable_e2e import map_native_pair_reader_20261008_v1 as reader


WRITTEN_FIXTURE_BYTES = 0


def _header(shape, descr="|u1", fortran=False, raw=None):
    text = repr(dict(descr=descr, fortran_order=fortran, shape=tuple(shape))).encode("latin1") if raw is None else raw
    size = 128 - 10
    assert len(text) + 1 <= size
    return b"\x93NUMPY\x01\x00" + struct.pack("<H", size) + text + b" " * (size - len(text) - 1) + b"\n"


def _account(path):
    global WRITTEN_FIXTURE_BYTES
    WRITTEN_FIXTURE_BYTES += path.stat().st_size
    assert WRITTEN_FIXTURE_BYTES <= 32 * 1024 ** 2, "aggregate fixture-write limit exceeded"
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(reader.BLOCK), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_member(archive, name, count, channels, *, shape=None, descr="|u1",
                  fortran=False, first=(11, 29), raw_header=None, body_delta=0):
    # HH_261008 - No large array or large byte string is constructed for logical native fixtures.
    native = reader.HEIGHT * reader.WIDTH * channels
    shape = shape if shape is not None else ((count, reader.HEIGHT, reader.WIDTH, 3)
        if channels == 3 else (count, reader.HEIGHT, reader.WIDTH))
    with archive.open(name, "w", force_zip64=True) as stream:
        stream.write(_header(shape, descr, fortran, raw_header))
        for slot in range(count):
            remaining = native + (body_delta if slot == count - 1 else 0)
            value = first[slot] if slot < len(first) else 0
            while remaining:
                size = min(reader.BLOCK, remaining)
                stream.write(bytes((value,)) * size)
                remaining -= size


def _archive(path, count=2, *, paired=False, compression=zipfile.ZIP_DEFLATED,
             rgb_options=None, gt_options=None, extra=False):
    kwargs = dict(compression=compression)
    if compression == zipfile.ZIP_DEFLATED:
        kwargs["compresslevel"] = 1
    with zipfile.ZipFile(path, "w", **kwargs) as archive:
        _write_member(archive, "rgb_false.npy" if paired else "rgb.npy", count, 3, **(rgb_options or {}))
        if paired:
            # This intentionally invalid TRUE NPY is metadata-only; body decode must never occur.
            _write_member(archive, "rgb_true.npy", count, 3, raw_header=b"not an NPY dictionary")
        _write_member(archive, "target.npy", count, 1, first=(5, 6), **(gt_options or {}))
        if extra:
            archive.writestr("DEV.npy", b"denied")
    return _account(path)


def _small(tmp_path, *, paired=False, **kwargs):
    path = tmp_path / ("paired.npz" if paired else "original.npz")
    pin = _archive(path, paired=paired, **kwargs)
    count, rgb = 2, "rgb_false.npy" if paired else "rgb.npy"
    names = ("rgb_false.npy", "rgb_true.npy", "target.npy") if paired else ("rgb.npy", "target.npy")
    return path, pin, count, rgb, names


@pytest.fixture(scope="module")
def production_pair(tmp_path_factory):
    folder = tmp_path_factory.mktemp("bounded_native_pair")
    original, paired = folder / "original.npz", folder / "paired.npz"
    original_sha = _archive(original, reader.ORIGINAL_COUNT)
    paired_sha = _archive(paired, reader.PAIRED_COUNT, paired=True)
    assert original.stat().st_size + paired.stat().st_size < 16 * 1024 ** 2
    return original, original_sha, paired, paired_sha


def test_source_has_no_unbounded_loader_model_or_extraction_calls():
    tree = ast.parse(Path(reader.__file__).read_bytes())
    imports = {n.name for node in ast.walk(tree) if isinstance(node, ast.Import) for n in node.names}
    assert not imports & {"torch", "carla", "rclpy", "PIL", "zipfile"}
    calls = {ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node, ast.Call)}
    assert not calls & {"np.load", "numpy.load", "np.memmap", "tempfile.TemporaryDirectory",
        "zipfile.ZipFile", "os.system", "subprocess.run", "eval", "exec"}
    assert reader.BLOCK == 64 * 1024 and reader.SCRATCH_LIMIT_BYTES <= 16 * 1024 ** 2
    assert reader.ORIGINAL_COUNT == 729 and reader.PAIRED_COUNT == 480 and reader.TOTAL == 1209


def test_fixed_public_slot_routes_owned_uint8_and_never_decodes_TRUE(production_pair, monkeypatch):
    seen = []
    original_init = reader._MemberStream.__init__
    def observe(self, owner, entry):
        seen.append(entry["name"])
        assert entry["name"] != "rgb_true.npy"
        original_init(self, owner, entry)
    monkeypatch.setattr(reader._MemberStream, "__init__", observe)
    with reader.BoundedNativePairReader(*production_pair) as source:
        for slot, value, label in ((0, 11, 5), (1, 29, 6), (729, 11, 5), (730, 29, 6)):
            rgb, gt = source.read(slot)
            assert rgb.shape == (360, 640, 3) and gt.shape == (360, 640)
            assert str(rgb.dtype) == str(gt.dtype) == "uint8"
            assert rgb.flags.owndata and gt.flags.owndata and rgb.flags.c_contiguous and gt.flags.c_contiguous
            assert rgb.nbytes == 691200 and gt.nbytes == 230400
            assert (rgb == value).all() and (gt == label).all()
            receipt = source.last_read_receipt
            assert receipt["slot"] == slot and receipt["rgb_true_decoded"] is False
            assert receipt["caller_admission_authenticated_by_reader"] is False
            assert receipt["closing_whole_file_SHA_checked"] is False
            assert all(not item["whole_member_crc_verified"] for item in receipt["members"].values())
            if slot in (1, 730):
                assert receipt["members"]["target.npy"]["prefix_body_bytes_inflated"] == 230400
            rgb.fill(99)
            gt.fill(99)
        rgb, gt = source.read(0)
        assert (rgb == 11).all() and (gt == 5).all()
    assert set(seen) == {"rgb.npy", "rgb_false.npy", "target.npy"}
    with pytest.raises(ValueError, match="closed"):
        source.read(0)
    source.close()


@pytest.mark.parametrize("slot", [True, False, -1, 1209, 0.0, "0", None])
def test_public_invalid_slots_denied_before_io(production_pair, slot):
    with reader.BoundedNativePairReader(*production_pair) as source:
        with pytest.raises(ValueError, match="integer slot"):
            source.read(slot)


@pytest.mark.parametrize("compression", [zipfile.ZIP_DEFLATED, zipfile.ZIP_STORED])
def test_small_native_member_partial_and_tail_crc_scope(tmp_path, compression):
    with_archive = reader._NativeArchive(*_small(tmp_path, compression=compression))
    try:
        rgb, gt, first = with_archive.read(0)
        assert (rgb == 11).all() and (gt == 5).all()
        assert all(not r["whole_member_crc_verified"] for r in first.values())
        rgb, gt, tail = with_archive.read(1)
        assert (rgb == 29).all() and (gt == 6).all()
        assert all(r["member_EOF_and_trailer_checked"] for r in tail.values())
        assert all(r["whole_member_crc_verified"] is (compression == zipfile.ZIP_DEFLATED)
            for r in tail.values())
    finally:
        with_archive.close()


@pytest.mark.parametrize("rgb_options", [dict(descr="|i1"), dict(descr="|O"), dict(fortran=True),
    dict(shape=(2, 360, 640)), dict(shape=(2, 360, 641, 3)), dict(shape=(True, 360, 640, 3)),
    dict(raw_header=b"{'descr':'|u1','descr':'|u1','shape':(2,360,640,3)}"),
    dict(raw_header=b"{'descr':'|u1','fortran_order':False,'shape':(999999999,360,640,3)}")])
def test_malformed_native_header_denied_before_array_allocation(tmp_path, rgb_options, monkeypatch):
    import numpy as np
    def forbidden(*args, **kwargs):
        raise AssertionError("allocation must follow strict header admission")
    monkeypatch.setattr(np, "empty", forbidden)
    with pytest.raises(ValueError):
        reader._NativeArchive(*_small(tmp_path, rgb_options=rgb_options))


@pytest.mark.parametrize("delta", [-1, 1])
def test_native_payload_truncation_or_padding_denied(tmp_path, delta):
    with pytest.raises(ValueError, match="payload|truncation/padding"):
        reader._NativeArchive(*_small(tmp_path, rgb_options=dict(body_delta=delta)))


def test_gt_label7_is_failure_no_clipping(tmp_path):
    path = tmp_path / "labels.npz"
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=1) as archive:
        _write_member(archive, "rgb.npy", 2, 3)
        _write_member(archive, "target.npy", 2, 1, first=(7, 6))
    with_archive = reader._NativeArchive(path, _account(path), 2, "rgb.npy", ("rgb.npy", "target.npy"))
    try:
        with pytest.raises(ValueError, match="0..6"):
            with_archive.read(0)
    finally:
        with_archive.close()


def test_extra_DEV_member_denied_without_body_decode(tmp_path, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("extra ZIP members must fail before stream decoding")
    monkeypatch.setattr(reader._MemberStream, "__init__", forbidden)
    with pytest.raises(ValueError, match="footer|member"):
        reader._NativeArchive(*_small(tmp_path, extra=True))


def test_wrong_SHA_denied_before_zip_metadata_or_member_decode(tmp_path, monkeypatch):
    args = _small(tmp_path)
    def forbidden(*args, **kwargs):
        raise AssertionError("SHA mismatch must fail before ZIP/member decode")
    monkeypatch.setattr(reader._NativeArchive, "_census", forbidden)
    with pytest.raises(ValueError, match="SHA256 differs"):
        reader._NativeArchive(args[0], "f" * 64, *args[2:])


@pytest.mark.parametrize("pin", [None, "a", "A" * 64, "g" * 64, 1])
def test_missing_or_invalid_expected_pins_are_not_authority(pin):
    with pytest.raises(ValueError, match="explicit expected SHA256"):
        reader.BoundedNativePairReader("/does/not/exist", pin, "/does/not/exist", "b" * 64)


def test_canonical_symlink_and_hardlink_inputs_denied(tmp_path):
    args = _small(tmp_path)
    alias = tmp_path / "alias.npz"
    alias.symlink_to(args[0])
    with pytest.raises(ValueError, match="nonsymlink"):
        reader._NativeArchive(alias, *args[1:])
    hard = tmp_path / "hard.npz"
    os.link(args[0], hard)
    with pytest.raises(ValueError, match="single-link"):
        reader._NativeArchive(*args)


def test_path_replacement_detected_and_close_still_releases_owned_fd(tmp_path):
    args = _small(tmp_path)
    source = reader._NativeArchive(*args)
    fd = source.fd
    replacement = tmp_path / "replacement.npz"
    replacement.write_bytes(args[0].read_bytes())
    _account(replacement)
    replacement.replace(args[0])
    with pytest.raises(ValueError, match="generation"):
        source.read(0)
    with pytest.raises(ValueError, match="generation"):
        source.close()
    assert source.fd is None
    with pytest.raises(OSError):
        os.fstat(fd)


def test_same_inode_mutation_detected_by_generation(tmp_path):
    args = _small(tmp_path)
    source = reader._NativeArchive(*args)
    with args[0].open("ab") as stream:
        stream.write(b"changed")
    with pytest.raises(ValueError, match="generation"):
        source.read(0)
    with pytest.raises(ValueError, match="generation"):
        source.close()


def test_inherited_process_and_concurrent_rpc_denied(production_pair, monkeypatch):
    source = reader.BoundedNativePairReader(*production_pair)
    assert source._lock.acquire(blocking=False)
    try:
        with pytest.raises(ValueError, match="concurrent"):
            source.read(0)
        with pytest.raises(ValueError, match="concurrent"):
            source.close()
    finally:
        source._lock.release()
    real_pid = os.getpid()
    monkeypatch.setattr(reader.os, "getpid", lambda: real_pid + 1)
    with pytest.raises(ValueError, match="inherited"):
        source.read(0)
    monkeypatch.setattr(reader.os, "getpid", lambda: real_pid)
    source.close()


@pytest.mark.parametrize("fault", ["huge_central", "huge_members", "huge_name", "huge_decoded", "trailer"])
def test_huge_or_malformed_ZIP_metadata_denied_before_inflation(tmp_path, fault, monkeypatch):
    args = _small(tmp_path)
    raw = bytearray(args[0].read_bytes())
    cd = struct.unpack_from("<L", raw, len(raw) - 6)[0]
    if fault == "huge_central":
        struct.pack_into("<L", raw, len(raw) - 10, 0xffffffff)
    elif fault == "huge_members":
        struct.pack_into("<HH", raw, len(raw) - 14, 65535, 65535)
    elif fault == "huge_name":
        struct.pack_into("<H", raw, cd + 28, 65535)
    elif fault == "huge_decoded":
        struct.pack_into("<L", raw, cd + 24, 0xffffffff)
    else:
        raw.extend(b"trailer")
    args[0].write_bytes(raw)
    pin = _account(args[0])
    def forbidden(*args, **kwargs):
        raise AssertionError("ZIP metadata denial must precede stream inflation")
    monkeypatch.setattr(reader._MemberStream, "__init__", forbidden)
    with pytest.raises(ValueError):
        reader._NativeArchive(args[0], pin, *args[2:])


def test_whole_tail_CRC_corruption_fails_and_partial_scope_is_honest(tmp_path):
    args = _small(tmp_path)
    raw = bytearray(args[0].read_bytes())
    cd = struct.unpack_from("<L", raw, len(raw) - 6)[0]
    local = struct.unpack_from("<L", raw, cd + 42)[0]
    crc = struct.unpack_from("<L", raw, cd + 16)[0] ^ 1
    struct.pack_into("<L", raw, cd + 16, crc)
    struct.pack_into("<L", raw, local + 14, crc)
    args[0].write_bytes(raw)
    source = reader._NativeArchive(args[0], _account(args[0]), *args[2:])
    try:
        _, _, partial = source.read(0)
        assert partial["rgb.npy"]["whole_member_crc_verified"] is False
        with pytest.raises(ValueError, match="CRC"):
            source.read(1)
    finally:
        source.close()


def test_NPY_header_length_bound_is_checked_before_reading_header_body():
    class PrefixOnly:
        calls = []
        def read_exact(self, size):
            self.calls.append(size)
            assert size == 10, "huge header body must never be requested"
            return b"\x93NUMPY\x01\x00\xff\xff"
    stream = PrefixOnly()
    with pytest.raises(ValueError, match="bounded NPY"):
        reader._npy_header(stream, (2, 360, 640, 3))
    assert stream.calls == [10]


def test_extra_compressed_tail_byte_fails_only_when_whole_member_reached(tmp_path):
    args = _small(tmp_path)
    raw = bytearray(args[0].read_bytes())
    old_cd = struct.unpack_from("<L", raw, len(raw) - 6)[0]
    first_local = struct.unpack_from("<L", raw, old_cd + 42)[0]
    compressed = struct.unpack_from("<L", raw, old_cd + 20)[0]
    nlen, elen = struct.unpack_from("<HH", raw, first_local + 26)
    assert elen == 20
    insertion = first_local + 30 + nlen + elen + compressed
    raw[insertion:insertion] = b"x"
    cd = old_cd + 1
    struct.pack_into("<L", raw, cd + 20, compressed + 1)
    struct.pack_into("<Q", raw, first_local + 30 + nlen + 12, compressed + 1)
    second = cd + 46 + struct.unpack_from("<H", raw, cd + 28)[0]
    later_local = struct.unpack_from("<L", raw, second + 42)[0]
    struct.pack_into("<L", raw, second + 42, later_local + 1)
    struct.pack_into("<L", raw, len(raw) - 6, cd)
    args[0].write_bytes(raw)
    source = reader._NativeArchive(args[0], _account(args[0]), *args[2:])
    try:
        _, _, receipt = source.read(0)
        assert receipt["rgb.npy"]["member_EOF_and_trailer_checked"] is False
        with pytest.raises(ValueError, match="trailer"):
            source.read(1)
    finally:
        source.close()
