"""HH_261008 - V3 prepared bounded synthetic ZIP/NPY tests; no real data, training or model jobs.

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

from portable_e2e import map_native_pair_reader_20261008_v3 as reader


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


def _read_then_close(args):
    source = reader._NativeArchive(*args)
    try:
        return source.read(0)
    finally:
        source.close()


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
        _read_then_close(_small(tmp_path, rgb_options=rgb_options))


@pytest.mark.parametrize("delta", [-1, 1])
def test_native_payload_truncation_or_padding_denied(tmp_path, delta):
    with pytest.raises(ValueError, match="payload|truncation/padding"):
        _read_then_close(_small(tmp_path, rgb_options=dict(body_delta=delta)))


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
    if struct.unpack_from("<L", raw, first_local + 18)[0] != 0xffffffff:
        # HH_261008 - Keep a Python3.10 inline size and its ZIP64 extension consistent in this trailer fixture.
        struct.pack_into("<L", raw, first_local + 18, compressed + 1)
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


def _set_zip64_form(raw, form):
    """HH_261008 - Author explicit valid inline/sentinel metadata independently of host writer version."""
    assert form in ("inline", "sentinel")
    cd = struct.unpack_from("<L", raw, len(raw) - 6)[0]
    cursor = cd
    for _ in range(2):
        local = struct.unpack_from("<L", raw, cursor + 42)[0]
        compressed, decoded = struct.unpack_from("<LL", raw, cursor + 20)
        nlen, elen = struct.unpack_from("<HH", raw, local + 26)
        assert elen == 20
        needed = 20 if form == "inline" else 45
        struct.pack_into("<H", raw, cursor + 6, needed)
        struct.pack_into("<H", raw, local + 4, needed)
        struct.pack_into("<LL", raw, local + 18,
            compressed if form == "inline" else 0xffffffff,
            decoded if form == "inline" else 0xffffffff)
        struct.pack_into("<QQ", raw, local + 30 + nlen + 4, decoded, compressed)
        cursor += 46 + struct.unpack_from("<H", raw, cursor + 28)[0]
    return raw


@pytest.mark.parametrize("form", ["inline", "sentinel"])
def test_valid_force_ZIP64_local_size_forms_remain_strict_and_decode_native_tail(tmp_path, form):
    args = _small(tmp_path)
    raw = _set_zip64_form(bytearray(args[0].read_bytes()), form)
    args[0].write_bytes(raw)
    source = reader._NativeArchive(args[0], _account(args[0]), *args[2:])
    try:
        rgb, gt, receipts = source.read(1)
        assert (rgb == 29).all() and (gt == 6).all()
        assert all(value["whole_member_crc_verified"] for value in receipts.values())
        assert source.entries["rgb.npy"]["needed"] == (20 if form == "inline" else 45)
    finally:
        source.close()


@pytest.mark.parametrize("form,fault", [("inline", "tag"), ("inline", "length"),
    ("inline", "truncated_extra"), ("inline", "extra_decoded"), ("inline", "extra_compressed"),
    ("inline", "inline_decoded"), ("inline", "inline_compressed"), ("inline", "versions_differ"),
    ("sentinel", "tag"), ("sentinel", "length"), ("sentinel", "truncated_extra"),
    ("sentinel", "extra_decoded"), ("sentinel", "extra_compressed"),
    ("sentinel", "mixed_sentinels"), ("sentinel", "sentinels_need20")])
def test_wrong_ZIP64_tag_length_size_or_truncation_denied_before_decode(tmp_path, form, fault, monkeypatch):
    args = _small(tmp_path)
    raw = _set_zip64_form(bytearray(args[0].read_bytes()), form)
    cd = struct.unpack_from("<L", raw, len(raw) - 6)[0]
    local = struct.unpack_from("<L", raw, cd + 42)[0]
    nlen = struct.unpack_from("<H", raw, local + 26)[0]
    extra = local + 30 + nlen
    if fault == "tag":
        struct.pack_into("<H", raw, extra, 2)
    elif fault == "length":
        struct.pack_into("<H", raw, extra + 2, 15)
    elif fault == "truncated_extra":
        struct.pack_into("<H", raw, local + 28, 19)
    elif fault == "extra_decoded":
        value = struct.unpack_from("<Q", raw, extra + 4)[0]
        struct.pack_into("<Q", raw, extra + 4, value + 1)
    elif fault == "extra_compressed":
        value = struct.unpack_from("<Q", raw, extra + 12)[0]
        struct.pack_into("<Q", raw, extra + 12, value + 1)
    elif fault in ("inline_decoded", "inline_compressed"):
        field = local + (22 if fault == "inline_decoded" else 18)
        value = struct.unpack_from("<L", raw, field)[0]
        struct.pack_into("<L", raw, field, value + 1)
    elif fault == "versions_differ":
        struct.pack_into("<H", raw, local + 4, 45)
    elif fault == "mixed_sentinels":
        value = struct.unpack_from("<L", raw, cd + 20)[0]
        struct.pack_into("<L", raw, local + 18, value)
    else:
        struct.pack_into("<H", raw, cd + 6, 20)
        struct.pack_into("<H", raw, local + 4, 20)
    args[0].write_bytes(raw)
    pin = _account(args[0])
    def forbidden(*args, **kwargs):
        raise AssertionError("malformed local ZIP64 metadata must fail before member decode")
    monkeypatch.setattr(reader._MemberStream, "__init__", forbidden)
    with pytest.raises(ValueError, match="ZIP|sizes differ"):
        reader._NativeArchive(args[0], pin, *args[2:])


def _checkpoint_args(tmp_path, count=12, paired=False):
    path = tmp_path / ("checkpoint_paired.npz" if paired else "checkpoint_original.npz")
    pin = _archive(path, count=count, paired=paired)
    return (path, pin, count, "rgb_false.npy" if paired else "rgb.npy",
        ("rgb_false.npy", "rgb_true.npy", "target.npy") if paired else ("rgb.npy", "target.npy"))


def _array_hashes(rgb, gt):
    return tuple(hashlib.sha256(memoryview(value).cast("B")).hexdigest() for value in (rgb, gt))


def test_constructor_does_not_create_or_decode_any_member_stream(production_pair, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("constructor must not decode even an NPY header")
    monkeypatch.setattr(reader._MemberStream, "__init__", forbidden)
    with reader.BoundedNativePairReader(*production_pair) as source:
        assert all(archive.headers == {} for archive in source._archives)
        assert source.stats_snapshot()["current_checkpoint_states"] == 0
        with pytest.raises(AssertionError, match="constructor must not decode"):
            source.read(0)


@pytest.mark.parametrize("enabled", [False, True])
def test_cached_reverse_repeat_and_tail_hashes_match_frozen_V2(tmp_path, monkeypatch, enabled):
    from portable_e2e import map_native_pair_reader_20261008_v2 as frozen
    # Smaller spacing affects this synthetic test only; production spacing remains fixed32MiB.
    monkeypatch.setattr(reader, "CHECKPOINT_STRIDE_BYTES", 4 * reader.BLOCK)
    args = _checkpoint_args(tmp_path)
    control = frozen._NativeArchive(*args)
    candidate = reader._NativeArchive(*args, cache=reader._CheckpointStore(enabled=enabled))
    try:
        hit_seen = False
        for index in (11, 8, 0, 10, 1, 11):
            expected_rgb, expected_gt, expected = control.read(index)
            rgb, gt, receipts = candidate.read(index)
            assert _array_hashes(rgb, gt) == _array_hashes(expected_rgb, expected_gt)
            assert rgb.flags.owndata and gt.flags.owndata
            for name in ("rgb.npy", "target.npy"):
                actual = receipts[name]
                assert actual["whole_member_crc_verified"] == expected[name]["whole_member_crc_verified"]
                assert actual["member_EOF_and_trailer_checked"] == expected[name]["member_EOF_and_trailer_checked"]
                assert actual["prefix_body_bytes_inflated"] + actual["prefix_body_bytes_reused"] == actual["logical_prefix_body_bytes"]
                hit_seen |= actual["checkpoint_hit"]
                if actual["checkpoint_hit"]:
                    assert actual["prefix_body_bytes_inflated"] < actual["logical_prefix_body_bytes"]
            rgb.fill(254)
            gt.fill(254)
        stats = candidate.cache.snapshot()
        assert hit_seen is enabled
        assert stats["checkpoint_cache_enabled"] is enabled
        assert stats["native_row_cache_payload_bytes"] == 0
        assert stats["maximum_cache_accounted_bytes"] <= reader.CHECKPOINT_CACHE_ACCOUNT_LIMIT_BYTES
        if not enabled:
            assert stats["checkpoint_creations"] == stats["checkpoint_hits"] == stats["cache_accounted_bytes"] == 0
            assert stats["cache_account_limit_bytes"] == stats["max_checkpoint_states"] == 0
    finally:
        control.close()
        candidate.close()


def test_four_allowed_members_share_one_global_cache_budget_and_evict(tmp_path, monkeypatch):
    monkeypatch.setattr(reader, "CHECKPOINT_STRIDE_BYTES", reader.BLOCK)
    cache = reader._CheckpointStore()
    original = reader._NativeArchive(*_checkpoint_args(tmp_path), cache=cache)
    paired = reader._NativeArchive(*_checkpoint_args(tmp_path, paired=True), cache=cache)
    try:
        original.read(11)
        paired.read(11)
        stats = cache.snapshot()
        assert stats["checkpoint_evictions"] > 0
        assert stats["maximum_checkpoint_states_observed"] == reader.CHECKPOINT_MAX_STATES
        assert stats["maximum_cache_accounted_bytes"] == reader.CHECKPOINT_CACHE_ACCOUNT_LIMIT_BYTES
        assert len(cache.states) <= 48
        assert {state["member_generation"][4] for state in cache.states.values()} <= {
            "rgb.npy", "rgb_false.npy", "target.npy"}
        assert all(len(state["pending"]) <= reader.BLOCK and not hasattr(state["pending"], "shape")
            for state in cache.states.values())
        original.close()
        assert all(state["member_generation"][0] == str(paired.path) for state in cache.states.values())
    finally:
        original.close()
        paired.close()
    assert cache.snapshot()["current_checkpoint_states"] == cache.snapshot()["cache_accounted_bytes"] == 0
    assert cache.snapshot()["maximum_cache_accounted_bytes"] == reader.CHECKPOINT_CACHE_ACCOUNT_LIMIT_BYTES


def test_generation_change_rejects_cached_reuse_and_clears_states(tmp_path, monkeypatch):
    monkeypatch.setattr(reader, "CHECKPOINT_STRIDE_BYTES", 4 * reader.BLOCK)
    args = _checkpoint_args(tmp_path)
    source = reader._NativeArchive(*args)
    source.read(10)
    assert source.cache.snapshot()["current_checkpoint_states"] > 0
    with args[0].open("ab") as stream:
        stream.write(b"changed")
    with pytest.raises(ValueError, match="generation"):
        source.read(10)
    assert source.cache.snapshot()["current_checkpoint_states"] == 0
    with pytest.raises(ValueError, match="generation"):
        source.close()
    assert source.fd is None


def test_corrupt_checkpoint_FD_offset_is_rejected_before_returning_pixels(tmp_path, monkeypatch):
    monkeypatch.setattr(reader, "CHECKPOINT_STRIDE_BYTES", 4 * reader.BLOCK)
    source = reader._NativeArchive(*_checkpoint_args(tmp_path))
    try:
        source.read(10)
        target = len(source.headers["rgb.npy"]) + 10 * reader.NATIVE_RGB_BYTES
        matching = [(key, state) for key, state in source.cache.states.items()
            if state["member_generation"][4] == "rgb.npy" and state["position"] <= target]
        _, state = max(matching, key=lambda item: item[1]["position"])
        state["compressed_FD_offset"] += 1
        with pytest.raises(ValueError, match="checkpoint position"):
            source.read(10)
    finally:
        source.close()


def test_cached_tail_retains_whole_CRC_corruption_failure(tmp_path, monkeypatch):
    monkeypatch.setattr(reader, "CHECKPOINT_STRIDE_BYTES", 4 * reader.BLOCK)
    args = _checkpoint_args(tmp_path)
    raw = bytearray(args[0].read_bytes())
    cd = struct.unpack_from("<L", raw, len(raw) - 6)[0]
    local = struct.unpack_from("<L", raw, cd + 42)[0]
    crc = struct.unpack_from("<L", raw, cd + 16)[0] ^ 1
    struct.pack_into("<L", raw, cd + 16, crc)
    struct.pack_into("<L", raw, local + 14, crc)
    args[0].write_bytes(raw)
    source = reader._NativeArchive(args[0], _account(args[0]), *args[2:])
    try:
        _, _, partial = source.read(9)
        assert not partial["rgb.npy"]["whole_member_crc_verified"]
        with pytest.raises(ValueError, match="CRC"):
            source.read(11)
        assert source.cache.snapshot()["checkpoint_hits"] > 0
    finally:
        source.close()


def test_production_checkpoint_stride_boundary_matches_V2_without_large_arrays(tmp_path):
    from portable_e2e import map_native_pair_reader_20261008_v2 as frozen
    # HH_261008 - Stream52 logical frames to exercise the real32MiB boundary; only individual owned frames are allocated.
    args = _checkpoint_args(tmp_path, count=52)
    control, candidate = frozen._NativeArchive(*args), reader._NativeArchive(*args)
    try:
        for index in (48, 49, 47, 51):
            expected_rgb, expected_gt, expected = control.read(index)
            rgb, gt, receipts = candidate.read(index)
            assert _array_hashes(rgb, gt) == _array_hashes(expected_rgb, expected_gt)
            assert receipts["rgb.npy"]["whole_member_crc_verified"] == expected["rgb.npy"]["whole_member_crc_verified"]
            if index == 49:
                assert receipts["rgb.npy"]["checkpoint_hit"] is True
            if index == 47:
                assert receipts["rgb.npy"]["checkpoint_hit"] is False
        assert candidate.cache.snapshot()["checkpoint_stride_bytes"] == 32 * 1024 ** 2
    finally:
        control.close()
        candidate.close()


def test_cached_tail_retains_extra_compressed_trailer_denial(tmp_path, monkeypatch):
    monkeypatch.setattr(reader, "CHECKPOINT_STRIDE_BYTES", 4 * reader.BLOCK)
    args = _checkpoint_args(tmp_path)
    raw = bytearray(args[0].read_bytes())
    old_cd = struct.unpack_from("<L", raw, len(raw) - 6)[0]
    local = struct.unpack_from("<L", raw, old_cd + 42)[0]
    compressed = struct.unpack_from("<L", raw, old_cd + 20)[0]
    nlen, elen = struct.unpack_from("<HH", raw, local + 26)
    assert elen == 20
    insertion = local + 30 + nlen + elen + compressed
    raw[insertion:insertion] = b"x"
    cd = old_cd + 1
    struct.pack_into("<L", raw, cd + 20, compressed + 1)
    struct.pack_into("<Q", raw, local + 30 + nlen + 12, compressed + 1)
    if struct.unpack_from("<L", raw, local + 18)[0] != 0xffffffff:
        struct.pack_into("<L", raw, local + 18, compressed + 1)
    second = cd + 46 + struct.unpack_from("<H", raw, cd + 28)[0]
    later_local = struct.unpack_from("<L", raw, second + 42)[0]
    struct.pack_into("<L", raw, second + 42, later_local + 1)
    struct.pack_into("<L", raw, len(raw) - 6, cd)
    args[0].write_bytes(raw)
    source = reader._NativeArchive(args[0], _account(args[0]), *args[2:])
    try:
        _, _, partial = source.read(9)
        assert not partial["rgb.npy"]["member_EOF_and_trailer_checked"]
        with pytest.raises(ValueError, match="trailer"):
            source.read(11)
        assert source.cache.snapshot()["checkpoint_hits"] > 0
    finally:
        source.close()


def test_owner_checkpoint_cadence_and_mandatory_cleanup_counts(tmp_path):
    events = []
    source = reader._NativeArchive(*_small(tmp_path), checkpoint=lambda: events.append(True))
    assert len(events) >= 3  # constructor entry, hash completion, constructor exit
    before = len(events)
    for _ in range(reader.OWNER_CHECKPOINT_GUARD_STRIDE):
        source.guard()
    assert len(events) == before + 1
    before = len(events)
    source.guard(owner_checkpoint=True)
    source.guard(owner_checkpoint=True)
    assert len(events) == before + 2
    before = len(events)
    source.close()
    assert len(events) == before + 2
    assert source.cache.snapshot()["owner_checkpoint_calls"] == len(events)
    assert source.cache.snapshot()["owner_checkpoint_guard_stride"] == 16


def test_owner_guard_failure_releases_FD_and_clears_cache(tmp_path, monkeypatch):
    monkeypatch.setattr(reader, "CHECKPOINT_STRIDE_BYTES", 4 * reader.BLOCK)
    denied = [False]
    def owner_guard():
        if denied[0]:
            raise RuntimeError("owner budget denied")
    source = reader._NativeArchive(*_checkpoint_args(tmp_path), checkpoint=owner_guard)
    source.read(9)
    assert source.cache.snapshot()["current_checkpoint_states"] > 0
    fd = source.fd
    denied[0] = True
    with pytest.raises(RuntimeError, match="owner budget denied"):
        source.read(9)
    with pytest.raises(RuntimeError, match="owner budget denied"):
        source.close()
    assert source.fd is None and source.cache.snapshot()["current_checkpoint_states"] == 0
    with pytest.raises(OSError):
        os.fstat(fd)


@pytest.mark.parametrize("enabled", [0, 1, None, "true"])
def test_cache_enable_requires_exact_bool_before_path_IO(enabled):
    with pytest.raises(ValueError, match="exact bool"):
        reader.BoundedNativePairReader("/missing", "a" * 64, "/missing2", "b" * 64, cache_enabled=enabled)


def test_public_stats_are_scalar_owned_snapshots_and_survive_close(production_pair):
    with reader.BoundedNativePairReader(*production_pair, cache_enabled=False) as source:
        source.read(1)
        stats = source.stats_snapshot()
        assert all(type(value) in (bool, int) for value in stats.values())
        assert source.last_read_receipt["checkpoint_cache"] == stats
        stats["checkpoint_hits"] = 999
        assert source.stats_snapshot()["checkpoint_hits"] == 0
        assert source.stats_snapshot()["total_selected_payload_bytes"] == 921600
    assert source.stats_snapshot()["cache_accounted_bytes"] == 0


@pytest.mark.parametrize("kind", [RuntimeError, KeyboardInterrupt])
@pytest.mark.parametrize("fd_error", [False, True])
def test_close_preserves_first_error_and_all_secondary_errors_and_releases_both_FDs(
        production_pair, monkeypatch, kind, fd_error):
    state = dict(denied=False, calls=0)
    def owner_guard():
        if state["denied"]:
            state["calls"] += 1
            raise kind("close owner error " + str(state["calls"]))
    source = reader.BoundedNativePairReader(*production_pair, checkpoint=owner_guard)
    descriptors = [archive.fd for archive in source._archives]
    if fd_error:
        real_close = os.close
        owned = set(descriptors)
        def close_then_fail(fd):
            real_close(fd)
            if fd in owned:
                owned.remove(fd)
                raise OSError("secondary fd cleanup error")
        monkeypatch.setattr(reader.os, "close", close_then_fail)
    state["denied"] = True
    with pytest.raises(kind, match="close owner error 1"):
        source.close()
    assert all(archive.fd is None for archive in source._archives)
    for fd in descriptors:
        with pytest.raises(OSError):
            os.fstat(fd)
    expected_scopes = ["before_close_guard", "fd_close", "after_close_owner"] if fd_error else [
        "before_close_guard", "after_close_owner"]
    assert [row["scope"] for row in source.closing_errors] == [
        prefix + scope for prefix in ("original:", "paired:") for scope in expected_scopes]
    messages = [row["message"] for row in source.closing_errors if row["type"] == kind.__name__]
    assert messages == ["close owner error " + str(index) for index in range(1, 5)]
    assert len(source.closing_errors) == (6 if fd_error else 4)
    assert all(set(row) == {"scope", "type", "message"} and len(row["type"]) <= 64
        and len(row["message"]) <= 512 for row in source.closing_errors)
    assert source.stats_snapshot()["cache_accounted_bytes"] == 0
    source.close()


def test_closing_error_metadata_never_calls_foreign_str_or_repr():
    class Foreign(RuntimeError):
        def __str__(self):
            raise AssertionError("foreign string hook denied")
        def __repr__(self):
            raise AssertionError("foreign repr hook denied")
    row = reader._closing_error("after_close_owner", Foreign("x" * 4096))
    assert row == dict(scope="after_close_owner", type="Foreign", message="x" * 512)
