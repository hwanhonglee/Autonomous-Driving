"""HH_261008 - Preserve tested V1; describe oversized generic print loss without pretending lossless capture.

The final metadata-only fixture reads preserved V11 JSONL bytes. It performs no
pixel/array/model decode and is run only after ROOT approves this source suite.
"""
from __future__ import annotations

import gzip
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import zlib

import pytest

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tools/e2e/owned_lossless_jsonl_20261008_v2.py"
spec = importlib.util.spec_from_file_location("_hh_lossless_jsonl_261008_v2_tests", SOURCE)
codec = importlib.util.module_from_spec(spec)
spec.loader.exec_module(codec)


def row(ordinal=0):
    return (json.dumps(dict(ordinal=ordinal, native_frame=3999 + ordinal,
        timestamp_ns=18227820199123456789 + ordinal, status="FAILED_RETAINED" if ordinal == 2 else "PREFILL",
        text="실제 JSON\\n바이트", value=5.0), sort_keys=True, separators=(",", ":")) + "\n").encode()


def limits(**updates):
    result = dict(compressed_row_limit=4096, logical_total_limit=65536, max_rows=10)
    result.update(updates)
    return result


def writer(tmp_path, **updates):
    return codec.LosslessJSONLLedger(tmp_path / "ledger", guard=lambda added: None, **limits(**updates))


def read(ledger, **updates):
    return list(codec.iter_rows(ledger.root, **limits(**updates)))


def rewrite_index(ledger, change):
    path = ledger.root / "index.jsonl"
    values = [json.loads(raw) for raw in path.read_bytes().splitlines()]
    change(values)
    path.write_bytes(b"".join(codec.index_bytes(value) for value in values))


def test_exact_row_bytes_identity_crc_and_stream_hash(tmp_path):
    ledger = writer(tmp_path)
    expected = [row(i) for i in range(3)]
    for raw in expected:
        entry = ledger.append(raw)
        assert entry["raw_sha256"] == hashlib.sha256(raw).hexdigest()
        assert entry["identity"]["timestamp_ns"] >= 18227820199123456789
    ledger.close()
    assert read(ledger) == expected
    verified = codec.verify_ledger(ledger.root, **limits())
    assert verified["rows"] == 3
    assert verified["original_stream_sha256"] == hashlib.sha256(b"".join(expected)).hexdigest()
    assert verified["NN_inferences"] == verified["training_steps"] == 0


def test_original_print_bytes_exact_position_and_flush(tmp_path):
    calls = []
    ledger = codec.LosslessJSONLLedger(tmp_path / "ledger", guard=lambda added: calls.append(added), **limits())
    text = json.dumps(dict(ordinal=0, spaces="  preserved  ", number=5.0, 한국어=True))
    ledger.print(text, flush=True)
    assert read(ledger) == [(text + "\n").encode()]
    assert len(calls) == 2 and calls[0] == 0 and calls[1] > 0
    assert ledger.count == 1 and ledger.pending_raw is None
    ledger.close()


def test_builtin_print_partial_fragment_is_retained_without_newline(tmp_path):
    ledger = writer(tmp_path)
    class FailingValue:
        def __str__(self): raise RuntimeError("actual value conversion failed")
    with pytest.raises(RuntimeError): ledger.print('{"ordinal":0,"pending":', FailingValue(), flush=True)
    assert ledger.pending_raw == b'{"ordinal":0,"pending": '
    assert not ledger.pending_raw.endswith(b"\n")
    assert ledger.count == 0 and ledger.failure["stage"] == "original_print_fragment"
    ledger.close()


def test_actual_non_json_print_row_retained_and_not_reencoded(tmp_path):
    ledger = writer(tmp_path)
    with pytest.raises(ValueError): ledger.print("actual failure fragment not JSON", flush=True)
    assert ledger.pending_raw == b"actual failure fragment not JSON\n"
    assert ledger.count == 0
    ledger.close()


def test_original_write_fragment_sequence_and_incomplete_flush(tmp_path):
    ledger = writer(tmp_path)
    stream = codec.OriginalPrintStream(ledger)
    assert stream.write('{"ordinal":0}') == 13
    with pytest.raises(ValueError): stream.flush()
    assert stream.pending == b'{"ordinal":0}' and ledger.count == 0
    stream.write("\n"); stream.flush()
    assert read(ledger) == [b'{"ordinal":0}\n']
    ledger.close()


def test_no_flush_partial_print_close_denies_without_fabricating_newline(tmp_path):
    ledger = writer(tmp_path)
    ledger.print('{"ordinal":0,"unfinished":', end="", flush=False)
    assert ledger.pending_raw == b'{"ordinal":0,"unfinished":' and ledger.failure is None
    with pytest.raises(ValueError, match="codec close failed"): ledger.close()
    assert ledger.closed and ledger._index.closed and ledger.count == 0
    assert ledger.pending_raw == b'{"ordinal":0,"unfinished":'
    assert ledger.failure["stage"] == "closing_incomplete_original_print"
    assert ledger.failure["complete_prefix_is_PASS"] is False
    ledger.close()


def test_prior_print_failure_close_does_not_mask_original_error(tmp_path):
    ledger = writer(tmp_path)
    original = RuntimeError("genuine original conversion failure")
    class FailingValue:
        def __str__(self): raise original
    with pytest.raises(RuntimeError) as caught: ledger.print('{"partial":', FailingValue(), flush=True)
    assert caught.value is original
    prior = dict(ledger.failure)
    ledger.close()
    assert ledger.closed and ledger._index.closed and ledger.failure == prior
    assert ledger.pending_raw == b'{"partial": '


def test_shards_rotate_without_cross_anchor_loss(tmp_path):
    ledger = codec.LosslessJSONLLedger(tmp_path / "ledger", guard=lambda added: None,
        **limits(compressed_row_limit=512), shard_limit=512)
    expected = [row(i) for i in range(8)]
    for raw in expected: ledger.append(raw)
    ledger.close()
    shards = sorted(ledger.root.glob("rows_*.jsonl.gz"))
    assert len(shards) > 1 and all(path.stat().st_size <= 512 for path in shards)
    assert read(ledger, compressed_row_limit=512) == expected


@pytest.mark.parametrize("raw", [b"{}", b"{}\n{}\n", b'{"x":NaN}\n', b'{"x":1e999}\n',
    b'{"x":1,"x":2}\n', b"[]\n", b'{"x":"\xff"}\n', b'{"ordinal":true}\n'])
def test_invalid_rows_deny_and_pending_failure_is_not_erased(tmp_path, raw):
    ledger = writer(tmp_path)
    with pytest.raises((ValueError, UnicodeDecodeError)): ledger.append(raw)
    assert ledger.count == 0 and ledger.pending_raw == raw
    assert ledger.failure["pending_raw_retained"] is True
    assert ledger.failure["complete_prefix_is_PASS"] is False
    with pytest.raises(ValueError): ledger.append(row())
    assert ledger.pending_raw == raw
    ledger.close()


def test_oversized_input_denies_without_unbounded_pending_copy(tmp_path):
    ledger = writer(tmp_path)
    raw = b" " * (codec.RAW_ROW_LIMIT + 1)
    with pytest.raises(ValueError): ledger.append(raw)
    assert ledger.pending_raw is None
    assert ledger.failure["oversized_input_not_retained"] is True
    ledger.close()


@pytest.mark.parametrize("updates", [dict(compressed_row_limit=16), dict(logical_total_limit=1), dict(max_rows=1)])
def test_prospective_caps_retain_failed_row(tmp_path, updates):
    ledger = writer(tmp_path, **updates)
    raw = row()
    if updates.get("max_rows") == 1: ledger.append(raw)
    with pytest.raises(ValueError): ledger.append(raw)
    assert ledger.pending_raw == raw and ledger.failure is not None
    ledger.close()


def test_external_whole_quota_guard_precedes_actual_write(tmp_path):
    calls = []
    def guard(added):
        calls.append(added)
        if added: raise ValueError("whole128MiB guard denied")
    ledger = codec.LosslessJSONLLedger(tmp_path / "ledger", guard=guard, **limits())
    with pytest.raises(ValueError, match="whole128MiB"): ledger.append(row())
    assert ledger.pending_raw == row()
    assert [p.name for p in ledger.root.iterdir()] == ["index.jsonl"]
    assert ledger.failure["stage"] == "whole_output_guard"
    ledger.close()


def test_partial_index_write_leaves_real_unaccepted_tail(tmp_path):
    ledger = writer(tmp_path)
    original = ledger._index
    class FaultIndex:
        def write(self, raw):
            original.write(raw[:12]); original.flush()
            raise OSError("actual partial index fixture")
        def close(self): original.close()
    ledger._index = FaultIndex()
    with pytest.raises(OSError): ledger.append(row())
    assert ledger.pending_raw == row() and ledger.count == 0
    ledger.close()
    with pytest.raises(ValueError): read(ledger)
    assert next(ledger.root.glob("rows_*.jsonl.gz")).stat().st_size > 0


@pytest.mark.parametrize("raises", [False, True])
def test_partial_gzip_write_preserves_actual_bytes_and_raw_pending(tmp_path, raises):
    ledger = writer(tmp_path)
    # HH_261008 - Admit one real row, then simulate a short write in the same actual shard.
    ledger.append(row())
    original = ledger._shard
    before = ledger.shard_bytes
    class FaultShard:
        def write(self, raw):
            original.write(raw[:11]); original.flush()
            if raises: raise OSError("actual partial gzip fixture")
            return 11
        def close(self): original.close()
    ledger._shard = FaultShard()
    with pytest.raises((ValueError, OSError)): ledger.append(row(1))
    assert ledger.pending_raw == row(1) and ledger.count == 1
    assert ledger.failure["stage"] == "write_gzip_member"
    ledger.close()
    assert next(ledger.root.glob("rows_*.jsonl.gz")).stat().st_size == before + 11
    with pytest.raises(ValueError): read(ledger)


@pytest.mark.parametrize("fault", ["crc", "truncate", "extra_bytes", "two_members", "oversized_raw"])
def test_direct_member_decoder_checks_crc_eof_framing_and_raw_bound(fault):
    member = gzip.compress(row(), compresslevel=6, mtime=0)
    if fault == "crc": member = member[:-8] + bytes([member[-8] ^ 1]) + member[-7:]
    elif fault == "truncate": member = member[:-3]
    elif fault == "extra_bytes": member += b"not-another-member"
    elif fault == "two_members": member += gzip.compress(row(1), mtime=0)
    elif fault == "oversized_raw": member = gzip.compress(b" " * (codec.RAW_ROW_LIMIT + 1), mtime=0)
    with pytest.raises((ValueError, zlib.error)): codec.decompress_member(member)


@pytest.mark.parametrize("change", [
    lambda values: values[0].update(row=1),
    lambda values: values[0].update(row=True),
    lambda values: values[0].update(offset=1),
    lambda values: values[0].update(raw_bytes=True),
    lambda values: values[0].update(raw_sha256="0" * 64),
    lambda values: values[0].update(compressed_sha256="0" * 64),
    lambda values: values[0].update(compressed_sha256=True),
    lambda values: values[0].update(compressed_sha256=[]),
    lambda values: values[0].update(compressed_bytes=True),
    lambda values: values[0].update(compressed_bytes="10"),
    lambda values: values[0].update(shard="../foreign.gz"),
    lambda values: values[0].update(identity=dict(ordinal=True)),
    lambda values: values[0].update(identity=dict(ordinal=77)),
    lambda values: values[0].update(extra="denied"),
])
def test_index_tampering_cannot_relabel_or_skip_records(tmp_path, change):
    ledger = writer(tmp_path); ledger.append(row()); ledger.close()
    rewrite_index(ledger, change)
    with pytest.raises(ValueError): read(ledger)


@pytest.mark.parametrize("fault", ["truncate", "crc", "concatenate", "extra_tail", "extra_file", "missing_index"])
def test_framing_crc_and_whole_census_denials(tmp_path, fault):
    ledger = writer(tmp_path); ledger.append(row()); ledger.close()
    shard = next(ledger.root.glob("rows_*.jsonl.gz"))
    original = shard.read_bytes()
    if fault == "truncate": shard.write_bytes(original[:-3])
    elif fault == "crc": shard.write_bytes(original[:-8] + bytes([original[-8] ^ 1]) + original[-7:])
    elif fault == "concatenate":
        changed = original + gzip.compress(row(1), mtime=0)
        shard.write_bytes(changed)
        rewrite_index(ledger, lambda values: values[0].update(compressed_bytes=len(changed), compressed_sha256=hashlib.sha256(changed).hexdigest()))
    elif fault == "extra_tail": shard.write_bytes(original + b"actual-unindexed-tail")
    elif fault == "extra_file": (ledger.root / "foreign.raw").write_bytes(b"preserved")
    elif fault == "missing_index": (ledger.root / "index.jsonl").unlink()
    with pytest.raises((ValueError, zlib.error)): read(ledger)


def test_empty_prefix_is_a_valid_empty_ledger_not_performance_pass(tmp_path):
    ledger = writer(tmp_path); ledger.close()
    result = codec.verify_ledger(ledger.root, **limits())
    assert result["rows"] == 0 and result["independent_runtime_performance_claimed"] is False


def test_symlink_hardlink_and_unexpected_files_denied(tmp_path):
    outside = tmp_path / "outside"; outside.mkdir()
    alias = tmp_path / "alias"; alias.symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError): codec.LosslessJSONLLedger(alias / "new", guard=lambda added: None, **limits())
    ledger = writer(tmp_path); ledger.append(row()); ledger.close()
    original = ledger.root / "index.jsonl"
    (tmp_path / "extra_hardlink").hardlink_to(original)
    with pytest.raises(ValueError): read(ledger)


def test_actual_file_generation_change_during_append_denies(tmp_path):
    ledger = writer(tmp_path); ledger.append(row())
    index = ledger.root / "index.jsonl"
    index.rename(ledger.root / "retained_original_index")
    index.write_bytes(b"")
    with pytest.raises(ValueError): ledger.append(row(1))
    assert ledger.pending_raw == row(1)
    ledger.close()


def test_same_size_mutation_before_next_append_denies_and_retains_pending(tmp_path):
    ledger = writer(tmp_path); ledger.append(row())
    index = ledger.root / "index.jsonl"
    before = index.stat(); raw = index.read_bytes()
    changed = bytes([raw[0] ^ 1]) + raw[1:]
    index.write_bytes(changed)
    os.utime(index, ns=(before.st_atime_ns, before.st_mtime_ns + 1))
    assert index.stat().st_size == before.st_size
    with pytest.raises(ValueError, match="mtime/ctime"): ledger.append(row(1))
    assert ledger.count == 1 and ledger.pending_raw == row(1)
    assert ledger.failure["stage"] == "validate_raw"
    ledger.close()
    assert index.read_bytes() == changed


@pytest.mark.parametrize("invalid", [True, 0, -1, 1.5])
def test_limits_are_integer_not_bool(tmp_path, invalid):
    with pytest.raises(ValueError): writer(tmp_path, compressed_row_limit=invalid)


def test_prospective_full918_storage_capacity_is_explicit_not_ratio_promise():
    value = codec.prospective_capacity()
    assert sum(part["max_rows"] for part in value["components"].values()) == 9182
    assert value["total_reserved_bytes"] == 106_724_864
    assert value["remaining_working_bytes"] == 10_715_648
    assert value["total_reserved_bytes"] <= 112 * 1024**2
    assert value["closing_reserve_bytes"] == 16 * 1024**2
    assert value["guaranteed_generic_compression_ratio"] is False
    with pytest.raises(ValueError): codec.prospective_capacity(metadata_limit=32 * 1024**2)


def test_preserved_v11_whole_metadata_rows_exact_roundtrip_and_caps():
    # HH_261008 - This is a stored-byte codec check, NOT another DDS/NN/pixel timing experiment.
    folder = ROOT / "storage/experiments/diagnostics/2026-10-07/temporal_recorded_ros_shadow_v11"
    names = {"direct_anchors": "direct_anchors.jsonl", "subscriber_anchors": "subscriber_anchors.jsonl",
        "publisher_anchors": "publisher_anchors.jsonl", "subscriber_stdout": "subscriber.stdout",
        "publisher_stdout": "publisher.stdout", "subscriber_output_calls": "subscriber_output_calls.jsonl",
        "publisher_received_outputs": "publisher_received_outputs.jsonl", "comparison": "comparison.jsonl"}
    counts = {}
    for name, filename in names.items():
        cap, maximum_rows = codec.COMPONENT_LIMITS[name]
        original = (folder / filename).read_bytes()
        reconstructed_hash = hashlib.sha256()
        rows = 0
        for raw in original.splitlines(keepends=True):
            codec.decode_row(raw)
            member = gzip.compress(raw, compresslevel=6, mtime=0)
            assert len(member) <= cap
            assert codec.decompress_member(member) == raw
            reconstructed_hash.update(codec.decompress_member(member))
            rows += 1
        assert reconstructed_hash.hexdigest() == hashlib.sha256(original).hexdigest()
        assert rows <= maximum_rows
        counts[name] = rows
    assert counts["direct_anchors"] == counts["comparison"] == 918
    assert counts["subscriber_anchors"] == counts["publisher_anchors"] == 43
    assert counts["subscriber_stdout"] == counts["publisher_stdout"] == 43


def test_v2_preserves_exact_frozen_v1_and_historical_test_scope():
    assert codec.prior_sources_unchanged() == codec.PRIOR_SOURCE_PINS
    assert codec.HISTORICAL_V1_TEST_SCOPE["ROOT_reported_pure_cases_passed"] == 59
    assert codec.HISTORICAL_V1_TEST_SCOPE["ROOT_reported_wall_seconds"] == 1.29
    assert codec.HISTORICAL_V1_TEST_SCOPE["oversized_generic_print_fragment_full_retention_verified"] is False
    assert codec.HISTORICAL_V1_TEST_SCOPE["runtime_or_feature_approval"] is False


@pytest.mark.parametrize("kind", ["ascii", "multibyte"])
def test_v2_oversized_first_print_has_full_fingerprint_not_full_byte_retention(tmp_path, kind):
    ledger = writer(tmp_path)
    text = "X" * (codec.RAW_ROW_LIMIT + 17) if kind == "ascii" else "도로" * (codec.RAW_ROW_LIMIT // 6 + 17)
    expected = text.encode("utf-8")
    with pytest.raises(ValueError, match="full bytes NOT retained"): ledger.print(text, flush=True)
    receipt = ledger.oversized_fragment_receipt
    assert receipt["incoming_fragment_byte_count"] == len(expected)
    assert receipt["incoming_fragment_character_count"] == len(text)
    assert receipt["incoming_fragment_sha256"] == hashlib.sha256(expected).hexdigest()
    assert receipt["incoming_fragment_prefix64_hex"] == expected[:64].hex()
    assert receipt["incoming_fragment_suffix64_hex"] == expected[-64:].hex()
    assert receipt["whole_attempt_stream_sha256"] == hashlib.sha256(expected).hexdigest()
    assert receipt["existing_prefix_byte_count"] == 0
    assert receipt["oversized_fragment_full_bytes_NOT_RETAINED"] is True
    assert receipt["lossless_reconstruction_of_failed_stream_claimed"] is False
    assert ledger.failure["stage"] == "oversized_incoming_original_print_fragment"
    assert ledger.failure["complete_prefix_is_PASS"] is False
    assert ledger.pending_raw is None and ledger.count == 0
    ledger.close()
    assert ledger.closed and ledger._index.closed
    assert list(ledger.root.iterdir()) == [ledger.root / "index.jsonl"]


def test_v2_oversized_after_real_builtin_prefix_preserves_exact_prefix_no_fake_newline(tmp_path):
    ledger = writer(tmp_path)
    first = '{"ordinal":0,"unfinished":'
    text = "Z" * (codec.RAW_ROW_LIMIT + 3)
    expected_prefix = (first + " ").encode()
    with pytest.raises(ValueError, match="full bytes NOT retained"): ledger.print(first, text, flush=True)
    receipt = ledger.oversized_fragment_receipt
    assert ledger.pending_raw == expected_prefix and not ledger.pending_raw.endswith(b"\n")
    assert receipt["existing_prefix_sha256"] == hashlib.sha256(expected_prefix).hexdigest()
    assert receipt["whole_attempt_stream_sha256"] == hashlib.sha256(expected_prefix + text.encode()).hexdigest()
    assert receipt["whole_attempt_stream_byte_count"] == len(expected_prefix) + len(text)
    assert receipt["incoming_fragment_accepted_or_written"] is False
    assert receipt["whole_attempt_scope"].endswith("no uncalled separator/newline invented")
    saved_failure = dict(ledger.failure)
    ledger.close()
    assert ledger._index.closed and ledger.failure == saved_failure and ledger.pending_raw == expected_prefix


def test_v2_prior_success_does_not_make_oversized_failed_stream_lossless(tmp_path):
    ledger = writer(tmp_path)
    ledger.print(row().decode().rstrip("\n"), flush=True)
    with pytest.raises(ValueError): ledger.print("A" * (codec.RAW_ROW_LIMIT + 1), flush=True)
    ledger.close()
    assert read(ledger) == [row()]
    assert ledger.failure["accepted_rows"] == 1 and ledger.failure["complete_prefix_is_PASS"] is False
    assert ledger.failure["lossless_reconstruction_of_failed_stream_claimed"] is False


def test_v2_streaming_fragment_descriptor_includes_actual_unicode_byte_ends():
    prefix = b'{"actual-prefix":'
    text = "교차로🌍" * 17000
    raw = text.encode()
    receipt = codec.fragment_fingerprint(text, prefix)
    assert receipt["incoming_fragment_character_count"] == len(text)
    assert receipt["incoming_fragment_byte_count"] == len(raw)
    assert receipt["incoming_fragment_prefix64_hex"] == raw[:64].hex()
    assert receipt["incoming_fragment_suffix64_hex"] == raw[-64:].hex()
    assert receipt["whole_attempt_stream_sha256"] == hashlib.sha256(prefix + raw).hexdigest()


def test_v2_normal_bounded_nonascii_print_remains_exact_bytes(tmp_path):
    ledger = writer(tmp_path)
    text = json.dumps(dict(ordinal=0, road="교차로🌍", number=5.0), ensure_ascii=False)
    ledger.print(text, flush=True)
    ledger.close()
    assert read(ledger) == [(text + "\n").encode()]
    assert ledger.oversized_fragment_receipt is None and ledger.failure is None
