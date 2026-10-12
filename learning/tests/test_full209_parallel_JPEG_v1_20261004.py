"""HH_261004 - Review the whole-denominator JPEG-only probe without reading actual drive/image payloads.

Synthetic scheduling/recorders here are not an actual901-bundle parity replay,
a timing benchmark, original sensor BGRA, learned driving or an approval.
"""

import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import benchmark_full209_parallel_JPEG_v1_20261004 as b
from scripts.e2e import parallel_exact_jpeg_after_ack_v1_20261004 as h


@pytest.mark.parametrize("count", [30, 31, 32, 37, 450, 900, 901])
def test_fixed_indices_first_to_last_even_spacing_no_outcome_selection(count):
    expected = [(count-1) * index // 29 for index in range(30)]
    indices = b.selected_indices(count)
    assert indices == expected and len(set(indices)) == 30
    assert indices[0] == 0 and indices[-1] == count - 1
    assert all(left < right for left, right in zip(indices, indices[1:]))


@pytest.mark.parametrize("count", [True, False, -1, 0, 1, 29, 30., None])
def test_indices_reject_invalid_or_incomplete_denominator(count):
    with pytest.raises(RuntimeError):
        b.selected_indices(count)


def test_actual_fixed_contract_is_complete_and_loses_no_first_or_failed_last():
    contract = b.contract()
    assert (contract["all_camera_bundles"], contract["all_original_JPEGs"]) == (901, 5406)
    assert contract["workers"] == 3 and contract["jpeg_quality"] == 95
    assert contract["pairs"] == 30 and contract["timed_calls"] == 60
    assert contract["representative_indices"][-1] == 900
    assert contract["all_pairs_and_failed_tails_retained"] is True
    assert contract["byte_exact_outputs_required"] is True
    assert contract["first_candidate_flush_retains_cold_worker_startup"] is True
    assert contract["source_integrity_and_whole_parity_warm_cache_disclosed"] is True
    assert contract["lossy_JPEG_reconstruction"] is True
    for name in ("original_lossless_BGRA_parity_proven", "actual_runtime_activated",
                 "actual_sensor_latency_gain_proven", "model_promoted", "formal_function_approval"):
        assert contract[name] is False


@pytest.mark.parametrize("values,expected", [
    ([1.], dict(median_ms=1., p95_ms=1., maximum_ms=1., mean_ms=1.)),
    ([4., 1., 3., 2.], dict(median_ms=2.5, p95_ms=3.85, maximum_ms=4., mean_ms=2.5)),
    ([1., 9., 3.], dict(median_ms=3., p95_ms=8.4, maximum_ms=9., mean_ms=13./3)),
])
def test_stats_uses_every_cold_and_slow_value(values, expected):
    result = b.stats(values)
    assert result["count"] == len(values)
    for name, value in expected.items():
        assert result[name] == pytest.approx(value)


@pytest.mark.parametrize("values", [[], [float("nan")], [float("inf")], [True], [-1.]])
def test_stats_denies_nonfinite_or_invalid_duration(values):
    with pytest.raises(RuntimeError):
        b.stats(values)


def synthetic_images(frame=2):
    return tuple(SimpleNamespace(frame=frame, timestamp=frame*.05, width=640, height=360,
                                 raw_data=bytes([index]) * (640*360*4)) for index in range(6))


def synthetic_encode(source, target, quality):
    return b"synthetic-replay-encoded-bytes:" + bytes([source.raw_data[0], quality])


class ObservedFuture:
    def __init__(self, index, calls, *, error=None, value=None):
        self.index, self.calls, self.error, self.value = index, calls, error, value

    def result(self):
        self.calls.append(self.index)
        if self.error is not None:
            raise self.error
        return self.value


class ObservedPool:
    """HH_261004 - Observe every join/error in pure tests; this is not a threaded speed surrogate."""
    def __init__(self, result_calls, *, errors=None, submit_error=None):
        self.result_calls = result_calls
        self.errors = errors or {}
        self.submit_error = submit_error
        self.submitted = []

    def submit(self, function, *args):
        index = len(self.submitted)
        if self.submit_error is not None and index == 3:
            raise self.submit_error
        future = ObservedFuture(index, self.result_calls, error=self.errors.get(index),
                                value=None if index in self.errors else function(*args))
        self.submitted.append(future)
        return future


def fake_modules(monkeypatch):
    # HH_261004 - Only inject numeric work for synthetic task lifecycle; no original input integrity check is faked as actual evidence.
    monkeypatch.setattr(h, "_encode_image", synthetic_encode)
    monkeypatch.setattr(b, "modules", lambda: (None, None, h))


def test_parallel_parity_joins_all_six_and_has_exact_original_role_order(monkeypatch):
    fake_modules(monkeypatch)
    images = synthetic_images()
    joined = []
    pool = ObservedPool(joined)
    a, c = b.encode_bundle(images), b.encode_bundle(images, pool)
    assert a == c and joined == list(range(6)) and len(pool.submitted) == 6
    assert b.compare(a, c) == [hashlib.sha256(value).hexdigest() for value in a]


def test_two_worker_errors_still_join_all_and_raise_original_earliest_role(monkeypatch):
    fake_modules(monkeypatch)
    errors = {1: ValueError("role1 original error"), 4: OSError("role4 original error")}
    joined = []
    pool = ObservedPool(joined, errors=errors)
    with pytest.raises(ValueError) as caught:
        b.encode_bundle(synthetic_images(), pool)
    assert caught.value is errors[1] and joined == list(range(6))


def test_partial_submit_error_still_joins_every_previously_submitted_source(monkeypatch):
    fake_modules(monkeypatch)
    error = OSError("third submit infrastructure failure")
    joined = []
    pool = ObservedPool(joined, submit_error=error)
    with pytest.raises(OSError) as caught:
        b.encode_bundle(synthetic_images(), pool)
    assert caught.value is error and joined == [0, 1, 2]


@pytest.mark.parametrize("bad", [(), (b"x",)*5, (b"x",)*7])
def test_compare_never_approves_an_incomplete_or_extended_camera_set(bad):
    with pytest.raises(RuntimeError):
        b.compare(bad, (b"x",)*6)


def test_compare_rejects_any_byte_difference_or_nonbytes():
    with pytest.raises(RuntimeError):
        b.compare((b"x",)*6, (b"x",)*5+(b"changed",))
    with pytest.raises(RuntimeError):
        b.compare((bytearray(b"x"),)*6, (b"x",)*6)


def synthetic_recorder_roots(tmp_path):
    result = {arm: tmp_path / arm for arm in ("A", "B")}
    for output in result.values():
        for role in b.ROLES:
            (output / "images" / role).mkdir(parents=True)
    return result


def test_actual_recorder_flush_seam_timed_guards_hashes_writes_and_first_cold_are_retained(tmp_path, monkeypatch):
    # HH_261004 - Exercise the real old/new recorders on synthetic pixels; record elapsed values without speed assertions.
    monkeypatch.setattr(h, "exact_encoder", lambda _: synthetic_encode)
    outputs = synthetic_recorder_roots(tmp_path)
    def original_save(image, target, quality):
        target.write_bytes(synthetic_encode(image, target, quality))
    a = h.original.ExactJPEGAfterACK(original_save, outputs["A"])
    c = h.ParallelExactJPEGAfterACK(original_save, outputs["B"])
    try:
        results = []
        for pair, frame in enumerate((2, 4)):
            images = synthetic_images(frame)
            values = {}
            for arm in (("A", "B") if pair == 0 else ("B", "A")):
                payloads, record = b.timed_recorder(images, a if arm == "A" else c,
                                                   parallel=arm == "B", pair=pair)
                values[arm] = payloads
                results.append(record)
                assert record["elapsed_ns"] > 0 and math.isfinite(record["elapsed_ms"])
                assert record["offline_ACK_claimed"] is False
                assert record["original_recording_receipt"]["acknowledged_before_recording"] is False
            b.compare(values["A"], values["B"])
        assert [(row["pair"], row["arm"]) for row in results] == [(0, "A"), (0, "B"), (1, "B"), (1, "A")]
        assert [row["first_candidate_flush_cold_workers"] for row in results] == [False, True, False, False]
        assert len(a.events) == len(c.events) == 2 and not a.pending and not c.pending
        detail = c.events[0]["parallel_encoding_research"]
        assert len(detail["encode_rows"]) == 6 and detail["all_submitted_encodes_joined"] is True
        assert all(row["encoding_finished_monotonic_ns"] <= detail["all_encodes_joined_monotonic_ns"]
                   <= row["write_started_monotonic_ns"] for row in detail["encode_rows"])
    finally:
        c.close()


def test_timed_flush_write_failure_is_not_relabelled_a_pair_success(tmp_path, monkeypatch):
    monkeypatch.setattr(h, "exact_encoder", lambda _: synthetic_encode)
    output = synthetic_recorder_roots(tmp_path)["B"]
    c = h.ParallelExactJPEGAfterACK(lambda *args: None, output)
    real_write = h._write_create_only
    failed = OSError("synthetic disk write failure")
    def write(path, payload):
        if path.parent.name == b.ROLES[4]:
            raise failed
        real_write(path, payload)
    monkeypatch.setattr(h, "_write_create_only", write)
    try:
        with pytest.raises(OSError) as caught:
            b.timed_recorder(synthetic_images(), c, parallel=True, pair=0)
        assert caught.value is failed
        assert c.events[-1]["written_camera_count"] == 4 and len(c.pending) == 2
        assert c.events[-1]["all_queued_images_written"] is False
        assert len(c.events[-1]["parallel_encoding_research"]["encode_rows"]) == 6
    finally:
        c.close()


def test_timed_flush_duplicate_frame_no_overwrite_or_hidden_retry(tmp_path, monkeypatch):
    monkeypatch.setattr(h, "exact_encoder", lambda _: synthetic_encode)
    output = synthetic_recorder_roots(tmp_path)["B"]
    c = h.ParallelExactJPEGAfterACK(lambda *args: None, output)
    try:
        first, _ = b.timed_recorder(synthetic_images(), c, parallel=True, pair=0)
        with pytest.raises(RuntimeError, match="already exists"):
            b.timed_recorder(synthetic_images(), c, parallel=True, pair=1)
        assert len(c.events) == 1
        assert tuple((output/"images"/role/"00000002.jpg").read_bytes() for role in b.ROLES) == first
    finally:
        c.close()


@pytest.mark.parametrize("kind", ["existing", "relative", "dangling_symlink", "parent_symlink", "dotdot"])
def test_fresh_output_rejects_existing_or_alias_target_and_preserves_it(tmp_path, kind):
    path = tmp_path / "new_probe"
    if kind == "existing":
        path.mkdir()
    elif kind == "relative":
        path = Path("new_probe")
    elif kind == "dangling_symlink":
        path.symlink_to(tmp_path / "missing_target")
    elif kind == "parent_symlink":
        real = tmp_path / "real"
        real.mkdir()
        alias = tmp_path / "alias"
        alias.symlink_to(real, target_is_directory=True)
        path = alias / "new_probe"
    else:
        path = tmp_path / "nested" / ".." / "new_probe"
    with pytest.raises(RuntimeError):
        b.fresh_output(path)
    if kind == "existing":
        assert path.is_dir()
    elif kind == "dangling_symlink":
        assert path.is_symlink()


def test_fresh_canonical_missing_output_and_plan_are_checked_without_creating_them(tmp_path):
    for path in (tmp_path / "new_probe", tmp_path / "new_plan" / "declaration.json"):
        assert b.fresh_output(path) == path
        assert not path.exists()


@pytest.mark.parametrize("difference,passed", [(0, True), (1, True), (-1, False), (-12*1024**2, False)])
def test_exact_original_disk_floor_plus_whole_next_bundle_reserve(monkeypatch, difference, passed):
    assert b.DISK_FLOOR == 5*1024**3 + 16*1024**2
    assert b.NEXT_BUNDLE_RESERVE == 12*1024**2 + 128*1024
    calls = []
    def usage(path):
        calls.append(path)
        return SimpleNamespace(free=b.DISK_FLOOR + b.NEXT_BUNDLE_RESERVE + difference)
    monkeypatch.setattr(b.shutil, "disk_usage", usage)
    if passed:
        b.disk_guard()
    else:
        with pytest.raises(RuntimeError, match="closing floor"):
            b.disk_guard()
    assert calls == [b.ROOT]


def test_disk_denial_precedes_timed_enqueue_and_flush(tmp_path, monkeypatch):
    output = synthetic_recorder_roots(tmp_path)["A"]
    recorder = h.original.ExactJPEGAfterACK(lambda *args: pytest.fail("no encode allowed"), output)
    monkeypatch.setattr(b.shutil, "disk_usage", lambda _: SimpleNamespace(free=b.DISK_FLOOR))
    with pytest.raises(RuntimeError, match="reserve"):
        b.timed_recorder(synthetic_images(), recorder, parallel=False, pair=0)
    assert recorder.pending == recorder.events == []


def test_each_prefix_journal_row_is_closed_readable_before_next_row(tmp_path, monkeypatch):
    monkeypatch.setattr(b, "OUTPUT", tmp_path)
    values = [dict(camera_ordinal=ordinal, frame=2+ordinal*2,
                   all_six_JPEG_bytes_equal=True, encoded_SHA256=["synthetic"]*6) for ordinal in range(3)]
    for ordinal, value in enumerate(values):
        b.append_journal("all_parity_rows.jsonl", value)
        actual = [json.loads(line) for line in (tmp_path / "all_parity_rows.jsonl").read_text().splitlines()]
        assert actual == values[:ordinal+1]
    timing = dict(pair=0, arm="A", elapsed_ns=100, first_candidate_flush_cold_workers=False)
    b.append_journal("all_timing_rows.jsonl", timing)
    assert json.loads((tmp_path / "all_timing_rows.jsonl").read_text()) == timing


def test_journal_alias_race_is_denied_without_mutating_foreign_content(tmp_path, monkeypatch):
    monkeypatch.setattr(b, "OUTPUT", tmp_path)
    other = tmp_path / "foreign.jsonl"
    other.write_text("foreign-original\n")
    (tmp_path / "all_parity_rows.jsonl").symlink_to(other)
    with pytest.raises(RuntimeError, match="symlink"):
        b.append_journal("all_parity_rows.jsonl", dict(camera_ordinal=0))
    assert other.read_text() == "foreign-original\n"


@pytest.mark.parametrize("complete_parity", [False, True])
def test_failed_attempt_preserves_complete_prefix_original_exception_and_failed_denominator(tmp_path, monkeypatch, complete_parity):
    monkeypatch.setattr(b, "OUTPUT", tmp_path)
    count = b.CAMERAS if complete_parity else 2
    parity = [dict(camera_ordinal=index, all_six_JPEG_bytes_equal=True,
                   encoded_SHA256=["synthetic"]*6) for index in range(count)]
    timings = [dict(pair=0, arm="A", elapsed_ms=999., first_candidate_flush_cold_workers=False)]
    error = OSError("synthetic failing next-call, not hidden or retried")
    with pytest.raises(OSError) as caught:
        with b.retain_attempt(parity, timings):
            raise error
    assert caught.value is error
    record = json.loads((tmp_path / "FAILED_attempt.json").read_text())
    assert record["status"] == "FAILED" and record["planned_camera_bundles"] == 901
    assert record["completed_parity_count"] == count and record["completed_timed_call_count"] == 1
    assert record["all_completed_parity_rows"] == parity and record["all_completed_timing_rows"] == timings
    assert record["next_unfinished_parity_ordinal"] == (None if complete_parity else count)
    assert record["exception_type"] == "OSError" and record["exception"] == str(error)
    assert record["failed_partial_recorder_files_retained"] is True
    assert record["original_drive_FAIL_not_changed"] is True and record["formal_function_approved"] is False


def test_success_attempt_does_not_fabricate_failure_artifact(tmp_path, monkeypatch):
    monkeypatch.setattr(b, "OUTPUT", tmp_path)
    with b.retain_attempt([], []):
        pass
    assert not (tmp_path / "FAILED_attempt.json").exists()


def test_failure_artifact_is_create_only_never_overwrites_previous_failure(tmp_path, monkeypatch):
    monkeypatch.setattr(b, "OUTPUT", tmp_path)
    output = tmp_path / "FAILED_attempt.json"
    output.write_bytes(b"original-failed-attempt")
    with pytest.raises(FileExistsError):
        with b.retain_attempt([], []):
            raise ValueError("new denied retry")
    assert output.read_bytes() == b"original-failed-attempt"
