"""HH_261008 - Synthetic serial ABI/ACK regressions; no TRAIN package, model, job or file writer.

These tests are authored for the external ROOT test owner. NumPy is imported
only inside synthetic fixtures/tests, and the fixture crop is not an assertion
that the frozen original crop source or any actual exposure rows are admitted.
"""
import ast
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import pytest

from portable_e2e import map_serial_training_input_adapter_20261008_v1 as adapter


def sha(value):
    view = memoryview(value).cast("B")
    try:
        return hashlib.sha256(view).hexdigest()
    finally:
        view.release()


def crop(rgb, target, box):
    """HH_261008 - Deterministic toy nearest crop, never the actual frozen crop provider."""
    import numpy as np
    y, x, height, width = box
    yy = y + np.arange(360, dtype=np.int64) * height // 360
    xx = x + np.arange(640, dtype=np.int64) * width // 640
    return (np.array(rgb[yy[:, None], xx[None, :]], dtype=np.uint8, order="C", copy=True),
        np.array(target[yy[:, None], xx[None, :]], dtype=np.uint8, order="C", copy=True))


class Reader:
    def __init__(self, positive=True):
        import numpy as np
        self.rgb = np.empty((360, 640, 3), np.uint8)
        self.rgb[:, :, 0] = np.arange(640, dtype=np.int64) % 256
        self.rgb[:, :, 1] = (np.arange(360, dtype=np.int64) % 256)[:, None]
        self.rgb[:, :, 2] = 127
        self.target = np.zeros((360, 640), np.uint8)
        self.target[0, 0] = 2
        if positive:
            self.target[300:320, 550:570] = 5
        self.calls, self.closes, self.closing_errors = [], 0, []
        self.fail_on, self.close_error = None, None

    def read(self, slot):
        self.calls.append(slot)
        if self.fail_on is not None and len(self.calls) == self.fail_on:
            raise OSError("synthetic native read failed")
        return self.rgb.copy(), self.target.copy()

    def close(self):
        self.closes += 1
        if self.close_error is not None:
            raise self.close_error


def rows(reader, number=2):
    import numpy as np
    boxes = [[0, 0, 120, 300] for _ in range(8)]
    rgb, target = crop(reader.rgb, reader.target, boxes[0])
    rgb_sha, gt_sha = sha(np.stack([rgb] * 8)), sha(np.stack([target] * 8))
    return [dict(step=step, physical_slot_indices=[0, 1, 728, 729, 730, 1208, 42, 42],
        shared_crop_boxes=deepcopy(boxes), actual_paired_TRUE_bits=[0] * 8,
        RGB_sha256=rgb_sha, target_sha256=gt_sha) for step in range(1, number+1)]


def fixture(mode=adapter.ORIGINAL_CROP_CONTROL, *, number=2, checkpoint=None, positive=True):
    reader = Reader(positive)
    declared = rows(reader, number)
    return adapter.SerialTrainingInputAdapter(declared, reader, crop, mode=mode, checkpoint=checkpoint), reader, declared


def consumer(batch):
    return dict(consumer="synthetic-CPU-value-checker", accepted=True,
        host_RGB_sha256=sha(batch.host_RGB), host_target_sha256=sha(batch.host_target))


def counts(snapshot):
    return snapshot["PASS_batches"], snapshot["FAIL_batches"], snapshot["UNEXECUTED_batches"]


@pytest.mark.parametrize("mode", adapter.MODES)
def test_actual_owned_arrays_cpu_normalization_exact_receipt_and_order(mode):
    import numpy as np
    serial, reader, declared = fixture(mode)
    source = (sha(reader.rgb), sha(reader.target))
    for step in (1, 2):
        batch = serial.next_batch()
        assert batch.step == step and set(batch.receipt) == adapter.RECEIPT_FIELDS
        assert batch.receipt["provided_baseline"] == declared[step-1]
        assert batch.receipt["provided_baseline_sha256"] == adapter._metadata_sha(declared[step-1])
        assert len(json.dumps(batch.receipt, sort_keys=True, separators=(",", ":")).encode()) <= 16 * 1024
        assert len(batch.receipt["native_slots"]) == 8
        for position, native in enumerate(batch.receipt["native_slots"]):
            assert native == dict(position=position, slot=declared[step-1]["physical_slot_indices"][position],
                native_RGB_sha256=source[0], native_target_sha256=source[1],
                reader_provenance_authenticated_by_worker=False)
        expected = np.empty((8, 3, 360, 640), np.float32)
        for channel in range(3):
            expected[:, channel] = batch.native_RGB[:, :, :, channel]
        np.divide(expected, np.float32(255), out=expected)
        assert np.array_equal(batch.host_RGB, expected)
        assert np.array_equal(batch.host_target, batch.native_target.astype(np.int64))
        values = (batch.native_RGB, batch.native_target, batch.host_RGB, batch.host_target)
        assert all(value.flags.owndata and value.flags.c_contiguous for value in values)
        assert not any(np.shares_memory(left, right) for i, left in enumerate(values) for right in values[i+1:])
        if mode == adapter.ORIGINAL_CROP_CONTROL:
            assert batch.receipt["control_exposure"] == declared[step-1]
            assert len(batch.receipt["control_exposure"]) == 6 and batch.receipt["candidate_worker_receipt"] is None
            assert sha(batch.native_RGB) == declared[step-1]["RGB_sha256"]
            assert sha(batch.native_target) == declared[step-1]["target_sha256"]
        else:
            assert batch.receipt["control_exposure"] is None
            receipt = batch.receipt["candidate_worker_receipt"]
            assert receipt["journal"]["rescued"] == [True, False] * 4
            assert receipt["journal"]["full_GT_sign_pixels"] == [400] * 8
            assert receipt["journal"]["original_NEAREST_sign_pixels"] == [0] * 8
            assert sha(batch.native_RGB) == receipt["journal"]["candidate_RGB_sha256"]
            assert sha(batch.native_target) == receipt["journal"]["candidate_target_sha256"]
        assert serial.stats_snapshot()["produced_batches"] == step
        assert serial.stats_snapshot()["PASS_batches"] == step-1
        ack = serial.acknowledge(step, consumer_receipt=consumer(batch))
        assert ack["native_and_host_buffers_rehashed"] is True and ack["consumer_execution_authenticated"] is False
    assert (sha(reader.rgb), sha(reader.target)) == source
    assert reader.calls == declared[0]["physical_slot_indices"] * 2
    with pytest.raises(StopIteration, match="no epoch"):
        serial.next_batch()
    snapshot = serial.close()
    assert counts(snapshot) == (2, 0, 0) and snapshot["production_attempts"] == 2
    assert snapshot["consumption_attempted_batches"] == snapshot["consumption_PASS_batches"] == 2
    assert snapshot["consumption_FAIL_batches"] == snapshot["production_FAIL_batches"] == 0
    assert snapshot["scope"]["NN"] == snapshot["scope"]["optimizer_steps"] == 0
    assert snapshot["scope"]["ROOT_or_data_authenticated_by_adapter"] is False
    assert snapshot["scope"]["Torch_or_GPU_numerical_equivalence_claimed"] is False
    assert reader.closes == 1 and serial.close() == snapshot
    with pytest.raises(ValueError, match="rerun"):
        serial.next_batch()


@pytest.mark.parametrize("mode", adapter.MODES)
@pytest.mark.parametrize("field", ["RGB_sha256", "target_sha256"])
def test_both_baseline_hashes_must_be_reproduced_before_production(mode, field):
    reader = Reader()
    declared = rows(reader)
    declared[0][field] = "f" * 64
    serial = adapter.SerialTrainingInputAdapter(declared, reader, crop, mode=mode)
    with pytest.raises(ValueError, match="original|CONTROL"):
        serial.next_batch()
    snapshot = serial.close()
    assert counts(snapshot) == (0, 1, 1) and snapshot["produced_batches"] == 0
    assert snapshot["production_FAIL_batches"] == 1
    assert snapshot["consumption_attempted_batches"] == snapshot["consumption_FAIL_batches"] == 0
    assert snapshot["consumption_UNEXECUTED_batches"] == 2


@pytest.mark.parametrize("field", ["native_RGB", "native_target", "host_RGB", "host_target"])
def test_each_current_buffer_is_rehashed_and_mutation_never_consumed(field):
    serial, _, _ = fixture()
    batch = serial.next_batch()
    original_receipt = deepcopy(batch.receipt)
    value = getattr(batch, field)
    value.flat[0] = value.flat[0] + 1
    with pytest.raises(ValueError, match="bytes changed|range"):
        serial.acknowledge(1, consumer_receipt=consumer(batch))
    snapshot = serial.close()
    assert counts(snapshot) == (0, 1, 1) and snapshot["produced_batches"] == 1
    assert snapshot["consumption_FAIL_batches"] == 1 and snapshot["production_FAIL_batches"] == 0
    assert snapshot["failed_tail"]["produced_receipt"] == original_receipt
    assert snapshot["pending_step"] == 1 and serial._pending is serial._pending_arrays is serial._pending_expected is None


def test_simultaneous_array_and_exported_receipt_forgery_cannot_replace_private_truth():
    serial, _, _ = fixture()
    batch = serial.next_batch()
    original = deepcopy(batch.receipt)
    batch.host_RGB.flat[0] = 0.25
    batch.receipt["host_arrays"]["RGB"]["sha256"] = sha(batch.host_RGB)
    with pytest.raises(ValueError, match="production receipt"):
        serial.acknowledge(1, consumer_receipt=consumer(batch))
    snapshot = serial.close()
    tail = snapshot["failed_tail"]
    assert tail["produced_receipt"] == original
    assert tail["exported_production_receipt"] == batch.receipt != original
    assert tail["current_buffers"]["host_RGB"]["sha256"] == sha(batch.host_RGB)
    assert snapshot["PASS_batches"] == 0


@pytest.mark.parametrize("fault", ["step_bool", "step_float", "buffer_rebound", "receipt_only"])
def test_exported_binding_and_metadata_modifications_denied(fault):
    serial, _, _ = fixture()
    batch = serial.next_batch()
    if fault == "step_bool":
        object.__setattr__(batch, "step", True)
    elif fault == "step_float":
        object.__setattr__(batch, "step", 1.0)
    elif fault == "buffer_rebound":
        object.__setattr__(batch, "host_RGB", batch.host_RGB.copy())
    else:
        batch.receipt["scope"]["ROOT_or_data_authenticated_by_adapter"] = True
    with pytest.raises(ValueError, match="rebound|production receipt"):
        serial.acknowledge(1, consumer_receipt=consumer(batch))
    assert counts(serial.close()) == (0, 1, 1)


@pytest.mark.parametrize("fault", ["extra", "missing", "accepted_false", "accepted_int", "consumer_empty", "consumer_long",
    "consumer_NUL", "sha_upper", "sha_wrong", "sha_type", "step_bool", "step_float", "step_order"])
def test_ack_exact_fields_types_and_chronology(fault):
    serial, _, _ = fixture()
    batch = serial.next_batch()
    proof, step = consumer(batch), 1
    if fault == "extra": proof["unapproved"] = 0
    elif fault == "missing": del proof["consumer"]
    elif fault == "accepted_false": proof["accepted"] = False
    elif fault == "accepted_int": proof["accepted"] = 1
    elif fault == "consumer_empty": proof["consumer"] = ""
    elif fault == "consumer_long": proof["consumer"] = "x" * 129
    elif fault == "consumer_NUL": proof["consumer"] = "x\0y"
    elif fault == "sha_upper": proof["host_RGB_sha256"] = "A" * 64
    elif fault == "sha_wrong": proof["host_target_sha256"] = "f" * 64
    elif fault == "sha_type": proof["host_RGB_sha256"] = 0
    elif fault == "step_bool": step = True
    elif fault == "step_float": step = 1.0
    else: step = 2
    with pytest.raises(ValueError):
        serial.acknowledge(step, consumer_receipt=proof)
    before = serial.stats_snapshot()
    with pytest.raises(ValueError, match="again"):
        serial.acknowledge(1, consumer_receipt=consumer(batch))
    assert serial.stats_snapshot() == before
    assert counts(serial.close()) == (0, 1, 1)


def test_pending_production_request_is_not_another_production_or_consumer_attempt():
    serial, reader, _ = fixture()
    serial.next_batch()
    with pytest.raises(ValueError, match="pending"):
        serial.next_batch()
    snapshot = serial.close()
    assert counts(snapshot) == (0, 1, 1)
    assert snapshot["produced_batches"] == snapshot["production_attempts"] == 1
    assert snapshot["production_FAIL_batches"] == snapshot["consumption_attempted_batches"] == 0
    assert len(reader.calls) == 8


def test_duplicate_ack_after_success_never_rewinds_or_invents_failed_batch():
    serial, reader, _ = fixture()
    batch = serial.next_batch()
    serial.acknowledge(1, consumer_receipt=consumer(batch))
    with pytest.raises(ValueError, match="pending step"):
        serial.acknowledge(1, consumer_receipt=consumer(batch))
    snapshot = serial.close()
    assert counts(snapshot) == (1, 0, 1) and snapshot["consumption_attempted_batches"] == 1
    assert snapshot["production_attempts"] == 1 and len(reader.calls) == 8


def test_reader_failure_retains_partial_slots_and_separates_unattempted_consumption():
    serial, reader, _ = fixture()
    reader.fail_on = 4
    with pytest.raises(OSError, match="native read"):
        serial.next_batch()
    snapshot = serial.close()
    assert counts(snapshot) == (0, 1, 1) and snapshot["actual_native_read_attempts"] == 4
    assert len(snapshot["failed_tail"]["actual_partial_native_slots"]) == 3
    assert snapshot["production_FAIL_batches"] == 1 and snapshot["produced_batches"] == 0
    assert snapshot["consumption_attempted_batches"] == snapshot["consumption_FAIL_batches"] == 0
    assert snapshot["consumption_UNEXECUTED_batches"] == 2
    with pytest.raises(ValueError, match="rerun"):
        serial.next_batch()


def test_consumer_abort_keeps_primary_before_failed_checkpoint_and_close_errors():
    failure = {"enabled": False}
    def checkpoint():
        if failure["enabled"]:
            raise RuntimeError("secondary owner checkpoint")
    serial, reader, _ = fixture(checkpoint=checkpoint)
    batch = serial.next_batch()
    failure["enabled"] = True
    reader.close_error = OSError("secondary owned close")
    reader.closing_errors = [dict(scope="original:fd_close", type="OSError", message="secondary owned close")]
    serial.abort(ValueError("first consumer failure"), step=1)
    snapshot = serial.close()
    assert snapshot["failure"]["error"]["message"] == "first consumer failure"
    assert [row["scope"] for row in snapshot["cleanup_errors"]] == ["abort_checkpoint", "before_reader_close", "reader_close", "after_reader_close"]
    assert snapshot["reader_closing_observations"] == reader.closing_errors and reader.closes == 1
    assert counts(snapshot) == (0, 1, 1) and snapshot["consumption_FAIL_batches"] == 1
    assert snapshot["failed_tail"]["current_buffers"]["host_RGB"]["sha256"] == sha(batch.host_RGB)
    assert serial._pending is serial._pending_arrays is serial._pending_expected is None


@pytest.mark.parametrize("interrupt_at", ["guard", "reader"])
def test_close_interrupts_are_retained_but_owned_reader_close_is_attempted(interrupt_at):
    enabled = {"value": False}
    def checkpoint():
        if enabled["value"] and interrupt_at == "guard":
            raise KeyboardInterrupt("synthetic close guard interrupt")
    serial, reader, _ = fixture(checkpoint=checkpoint)
    enabled["value"] = True
    if interrupt_at == "reader":
        reader.close_error = KeyboardInterrupt("synthetic reader close interrupt")
    snapshot = serial.close()
    assert reader.closes == 1 and counts(snapshot) == (0, 0, 2)
    assert any(row["error"]["type"] == "KeyboardInterrupt" for row in snapshot["cleanup_errors"])
    assert snapshot["status"] == "FAILED_RETAINED"


def test_close_before_attempt_and_unacknowledged_close_have_distinct_denominators():
    serial, reader, _ = fixture()
    untouched = serial.close()
    assert counts(untouched) == (0, 0, 2) and untouched["production_attempts"] == 0 and reader.calls == []
    serial, _, _ = fixture()
    serial.next_batch()
    pending = serial.close()
    assert counts(pending) == (0, 1, 1) and pending["produced_batches"] == 1
    assert pending["production_FAIL_batches"] == pending["consumption_attempted_batches"] == pending["consumption_FAIL_batches"] == 0
    assert pending["pending_step"] == 1 and pending["failed_tail"]["produced_receipt"] is not None


def test_cleanup_failure_after_all_acks_preserves_completed_denominator():
    enabled = {"value": False}
    def checkpoint():
        if enabled["value"]:
            raise SystemExit("post-ACK closing denied")
    serial, reader, _ = fixture(number=2, checkpoint=checkpoint)
    for step in (1, 2):
        batch = serial.next_batch()
        serial.acknowledge(step, consumer_receipt=consumer(batch))
    enabled["value"] = True
    snapshot = serial.close()
    assert counts(snapshot) == (2, 0, 0) and snapshot["failure"] is None
    assert snapshot["consumption_PASS_batches"] == 2 and snapshot["pending_step"] is None
    assert snapshot["status"] == "FAILED_RETAINED" and reader.closes == 1


@pytest.mark.parametrize("mode", adapter.MODES)
def test_proposal_mutation_and_source_mutation_fail_closed(mode):
    for fault in ("proposal", "source"):
        reader = Reader()
        declared = rows(reader)
        calls = {"count": 0}
        def mutating(rgb, target, box):
            result = crop(rgb, target, box)
            if calls["count"] == 0:
                if fault == "proposal": box[-1] = 301
                else: rgb[0, 0, 0] ^= 1
            calls["count"] += 1
            return result
        serial = adapter.SerialTrainingInputAdapter(declared, reader, mutating, mode=mode)
        with pytest.raises(ValueError, match="changed"):
            serial.next_batch()
        snapshot = serial.close()
        assert counts(snapshot) == (0, 1, 1) and snapshot["produced_batches"] == 0
        assert snapshot["failed_tail"]["original_provided_baseline"] == declared[0]


@pytest.mark.parametrize("owner", ["pid", "thread"])
def test_foreign_owner_denied_before_read_or_close(monkeypatch, owner):
    serial, reader, _ = fixture()
    with monkeypatch.context() as patch:
        if owner == "pid": patch.setattr(adapter.os, "getpid", lambda: serial.owner_pid+1)
        else: patch.setattr(adapter.threading, "get_ident", lambda: serial.owner_thread_ident+1)
        with pytest.raises(ValueError, match="owner|thread"):
            serial.next_batch()
        with pytest.raises(ValueError, match="owner|thread"):
            serial.close()
    assert reader.calls == [] and reader.closes == 0 and serial.stats_snapshot()["production_attempts"] == 0
    serial.close()
    with pytest.raises(TypeError, match="single-process"):
        serial.__getstate__()


def test_reentrant_callback_denied_before_native_read():
    holder = {}
    def checkpoint():
        holder["serial"].next_batch()
    serial, reader, _ = fixture(checkpoint=checkpoint)
    holder["serial"] = serial
    with pytest.raises(ValueError, match="reentrant"):
        serial.next_batch()
    assert reader.calls == [] and serial.stats_snapshot()["production_attempts"] == 0
    snapshot = serial.close()
    assert counts(snapshot) == (0, 0, 2) and reader.closes == 1


def test_exception_args_descriptor_does_not_invoke_foreign_getattribute():
    class ForeignError(RuntimeError):
        def __getattribute__(self, name):
            if name == "args":
                raise AssertionError("foreign exception args hook must not run")
            return super().__getattribute__(name)
    serial, _, _ = fixture()
    snapshot = serial.abort(ForeignError("owned scalar first error"))
    assert snapshot["failure"]["error"]["message"] == "owned scalar first error"
    assert counts(serial.close()) == (0, 0, 2)


@pytest.mark.parametrize("fault", ["TRUE", "bool_step", "noncontiguous", "duplicate", "extra"])
def test_invalid_fixed_rows_denied_before_read(fault):
    reader = Reader()
    declared = rows(reader)
    if fault == "TRUE": declared[0]["actual_paired_TRUE_bits"][0] = 1
    elif fault == "bool_step": declared[0]["step"] = True
    elif fault == "noncontiguous": declared[1]["step"] = 3
    elif fault == "duplicate": declared[1]["step"] = 1
    else: declared[0]["unsupported"] = 0
    with pytest.raises(ValueError):
        adapter.SerialTrainingInputAdapter(declared, reader, crop, mode=adapter.ORIGINAL_CROP_CONTROL)
    assert reader.calls == [] and reader.closes == 0


def test_contiguous_shards_full_metadata_only_and_no_extra_epoch():
    assert adapter.plan_contiguous_shards(list(range(1, 17))) == [
        dict(ordinal=0, declared_steps=list(range(1, 9)), first_step=1, last_step=8, expected_batches=8),
        dict(ordinal=1, declared_steps=list(range(9, 17)), first_step=9, last_step=16, expected_batches=8)]
    shards = adapter.plan_contiguous_shards(list(range(1, 20001)))
    assert len(shards) == 2500 and sum(item["expected_batches"] for item in shards) == 20000
    assert [step for shard in shards for step in shard["declared_steps"]] == list(range(1, 20001))
    assert set(shards[0]) == {"ordinal", "declared_steps", "first_step", "last_step", "expected_batches"}


@pytest.mark.parametrize("steps,size", [([], 8), ([True], 8), ([0], 8), ([20001], 8), ([1, 3], 8),
    ([2, 1], 8), ([1, 1], 8), ([1], True), ([1], 0), ([1], 65)])
def test_shard_invalid_metadata_denied(steps, size):
    with pytest.raises(ValueError):
        adapter.plan_contiguous_shards(steps, batches_per_shard=size)


def test_source_contains_only_lazy_numpy_and_no_train_loader_or_file_writer():
    source = Path(adapter.__file__).read_text()
    tree = ast.parse(source)
    top_imports = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))]
    names = [alias.name for node in top_imports for alias in node.names]
    assert not any(name.split(".")[0] in {"numpy", "torch", "carla"} for name in names)
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            assert node.func.attr not in {"load", "save", "savez", "open", "write", "fit", "backward", "step"}
        if isinstance(node, ast.Import):
            assert all(alias.name.split(".")[0] not in {"torch", "carla", "multiprocessing"} for alias in node.names)
