"""HH_261008 - Prepared scalar/mock chronology fixtures only; no arrays, data, jobs or training."""

import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import hashlib
import os

import pytest

from scripts.e2e import verify_map_serial_training_input_20261008_v1 as verify


def baseline(step=1):
    return dict(step=step, physical_slot_indices=[0, 1, 728, 729, 730, 1208, 42, 42],
        shared_crop_boxes=[[0, 0, 300, 400] for _ in range(8)], actual_paired_TRUE_bits=[0]*8,
        RGB_sha256="a"*64, target_sha256="b"*64)


def consumer_receipt():
    return dict(consumer="nontraining_numpy_host_value_consumer_v1", accepted=True,
        host_RGB_sha256="c"*64, host_target_sha256="d"*64)


class Sink:
    def __init__(self, events, scope, fail=False):
        self.events, self.scope, self.fail, self.rows = events, scope, fail, []
    def append(self, *values):
        self.events.append(self.scope)
        if self.fail:
            raise RuntimeError(self.scope+" failure")
        self.rows.append(deepcopy(values[-1]))


class Adapter:
    def __init__(self, events, fail=False):
        self.events, self.fail, self.acked = events, fail, []
    def acknowledge(self, step, *, consumer_receipt):
        self.events.append("ACK")
        if self.fail:
            raise RuntimeError("ACK failure")
        self.acked.append(step)
        return dict(step=step, consumer_receipt=deepcopy(consumer_receipt),
            native_and_host_buffers_rehashed=True, consumer_execution_authenticated=False)


def fake_batch(step=1):
    return SimpleNamespace(step=step, native_RGB=None, native_target=None,
        receipt=dict(control_exposure=baseline(step), candidate_worker_receipt=dict(step=step, synthetic=True)))


def process_fixture(monkeypatch, *, arm=0, native_fail=False, host_fail=False, ack_fail=False, guard_fail=None):
    events, calls, tail = [], [0], dict(phase="produce")
    native, host, adapter = Sink(events, "native", native_fail), Sink(events, "host", host_fail), Adapter(events, ack_fail)
    ledger = verify.Ledger()
    monkeypatch.setattr(verify, "owned_bytes", lambda: 0)
    def guard():
        calls[0] += 1
        if calls[0] == guard_fail:
            raise RuntimeError("guard failure")
    def consume(batch, checkpoint):
        events.append("consume")
        return consumer_receipt(), dict(synthetic_scalar_fixture_not_actual_array_validation=True)
    args = (fake_batch(), verify.ARMS[arm], baseline(), adapter, native, host, ledger, consume, guard, tail)
    return args, events, native, host, adapter, ledger, tail


def test_fixed_FIRST16_two_contiguous_shards_and_metadata_only_full20k_projection():
    fixed = verify.contract()
    assert fixed["declared_steps"] == list(range(1, 17))
    assert fixed["arms"] == ["ORIGINAL_CROP_CONTROL", "SIGN_PRESERVING_CANDIDATE"]
    assert fixed["same_reader_source_and_cache_enabled"] is True
    assert fixed["checkpoint_cache_account_limit_bytes"] == 12*1024**2
    assert fixed["original20k_batches_outside_declaration_per_arm"] == 19984
    assert [shard["declared_steps"] for shard in fixed["shards_per_arm"]] == [list(range(1, 9)), list(range(9, 17))]
    projection = fixed["prospective_full20k_sharding"]
    assert projection["shard_count"] == 2500 and projection["numeric_batches_executed"] == 0
    assert projection["full20k_execution_authorized"] is False
    assert fixed["Torch_numeric_equivalence_proven"] is fixed["trainer_connected"] is False
    assert fixed["main_approved"] == fixed["sub_approved"] == fixed["NN"] == 0


def test_new_authority_does_not_extend_old_cache_crop_or_training_grants():
    grant = verify.authority("f"*64)
    assert grant["schema"] == verify.SCHEMA+".ROOT_authority"
    assert grant["old_target_only_or_cache_AB_authority_reused"] is False
    assert grant["training_authorized"] is grant["NN_authorized"] is grant["Torch_authorized"] is False
    assert grant["nontraining_NumPy_consumer_and_sharded_metadata_authorized"] is True


@pytest.mark.parametrize("steps,size", [([], 8), ([1, 3], 8), ([True], 8), ([1, 2], True), ([1, 2], 0)])
def test_shard_plan_denies_coercion_holes_or_nonpositive_sizes(steps, size):
    with pytest.raises(ValueError):
        verify.shard_plan(steps, size)


@pytest.mark.parametrize("field,value", [("accepted", 1), ("accepted", False), ("consumer", "trainer"),
    ("host_RGB_sha256", "A"*64), ("host_target_sha256", False)])
def test_ACK_receipt_is_exact_positive_bool_and_actual_host_SHA_contract(field, value):
    receipt = consumer_receipt()
    receipt[field] = value
    with pytest.raises(ValueError):
        verify.validate_ack(receipt)


@pytest.mark.parametrize("arm", [0, 1])
def test_mock_pipeline_ACK_follows_consume_then_both_appends(monkeypatch, arm):
    args, events, native, host, adapter, ledger, tail = process_fixture(monkeypatch, arm=arm)
    row = verify.process_produced(*args)
    assert events == ["consume", "native", "host", "ACK"]
    assert row["adapter_ACK_received"] is True and adapter.acked == [1]
    assert ledger.snapshot()["PASS_batches"] == 1 and ledger.snapshot()["produced_batches"] == 1
    assert host.rows[0]["chronology"] == "CONSUMER_CHECKED_METADATA_WRITTEN_BEFORE_ADAPTER_ACK"
    if arm == 0:
        assert native.rows == [baseline()]
        assert set(native.rows[0]) == {"step", "physical_slot_indices", "shared_crop_boxes",
            "actual_paired_TRUE_bits", "RGB_sha256", "target_sha256"}
    else:
        assert native.rows == [dict(step=1, synthetic=True)]


@pytest.mark.parametrize("phase", ["native", "host", "ACK", "post_native_guard", "post_host_guard"])
def test_written_and_produced_facts_are_retained_without_unearned_ACK(monkeypatch, phase):
    args, events, native, host, adapter, ledger, tail = process_fixture(monkeypatch,
        native_fail=phase == "native", host_fail=phase == "host", ack_fail=phase == "ACK",
        guard_fail=1 if phase == "post_native_guard" else 2 if phase == "post_host_guard" else None)
    with pytest.raises(RuntimeError):
        verify.process_produced(*args)
    ledger.fail(1)
    snapshot = ledger.snapshot()
    assert snapshot["produced_batches"] == snapshot["consumer_checked_batches"] == 1
    assert snapshot["native_rows_written"] == len(native.rows)
    assert snapshot["host_rows_written"] == len(host.rows)
    assert snapshot["PASS_batches"] == 0 and snapshot["FAIL_batches"] == 1 and snapshot["UNEXECUTED_batches"] == 15
    assert adapter.acked == []


def test_last_step_failure_and_later_close_error_do_not_double_count_PASS():
    ledger = verify.Ledger()
    for step in range(1, 16):
        for event in ("produced", "consumer_checked", "native_written", "host_written", "acknowledged"):
            ledger.event(event, step)
    ledger.event("produced", 16)
    ledger.event("consumer_checked", 16)
    ledger.event("native_written", 16)
    ledger.event("host_written", 16)
    ledger.fail(16)
    assert ledger.snapshot()["PASS_batches"] == 15
    assert ledger.snapshot()["FAIL_batches"] == 1 and ledger.snapshot()["UNEXECUTED_batches"] == 0
    complete = verify.Ledger()
    for step in range(1, 17):
        for event in ("produced", "consumer_checked", "native_written", "host_written", "acknowledged"):
            complete.event(event, step)
    complete.fail(16)
    assert complete.snapshot()["PASS_batches"] == 16 and complete.snapshot()["FAIL_batches"] == 0


@pytest.mark.parametrize("event", ["consumer_checked", "native_written", "host_written", "acknowledged"])
def test_no_stage_can_skip_production_or_earlier_completion(event):
    ledger = verify.Ledger()
    with pytest.raises(ValueError):
        ledger.event(event, 1)


def test_summary_only_adapter_closing_errors_cannot_be_called_success():
    summary = dict(closed=True, status="COMPLETE_ACKNOWLEDGED_INPUTS_NOT_TRAINING_OR_MODEL_APPROVAL",
        PASS_batches=16, FAIL_batches=0, UNEXECUTED_batches=0, failure=None, cleanup_errors=[], reader_closing_observations=[])
    verify.adapter_closed(summary)
    summary["cleanup_errors"] = [dict(scope="after_reader_close", error="synthetic")]
    with pytest.raises(ValueError):
        verify.adapter_closed(summary)


def test_close_all_resources_preserves_secondary_BaseExceptions():
    errors = []
    class Failing:
        def close(self):
            raise KeyboardInterrupt("synthetic stop")
    verify.close_one(Failing(), "first", errors)
    verify.close_one(Failing(), "second", errors)
    assert [row["scope"] for row in errors] == ["first", "second"]
    assert all(row["type"] == "KeyboardInterrupt" for row in errors)


def test_safe_failure_scalar_never_calls_foreign_string_hooks():
    class Foreign(RuntimeError):
        def __str__(self):
            raise AssertionError("foreign error string hook must not run")
    row = verify.safe_error("consumer", Foreign("x"*4096))
    assert row == dict(scope="consumer", type="Foreign", message="x"*1024)


def test_source_uses_own_scope_no_old_budget_authority_or_numeric_import_on_bootstrap():
    tree = ast.parse(Path(verify.__file__).read_bytes())
    calls = {ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node, ast.Call)}
    assert not calls & {"old.Budget", "old.owned_bytes", "old.write_new", "old.prepare", "old.execute", "old.authority", "old.fixed"}
    top_imports = {alias.name for node in tree.body if isinstance(node, ast.Import) for alias in node.names}
    assert not top_imports & {"numpy", "PIL", "torch", "carla", "rclpy"}
    assert len(verify.SOURCES) == 14 and len(set(verify.SOURCES)) == 14
    assert any(path.endswith("map_serial_training_input_adapter_20261008_v1.py") for path in verify.SOURCES)
    assert verify.LIMIT == 2*1024**2 and verify.CLOSING == 512*1024 and verify.RSS == 512*1024**2
    assert verify.DEADLINE == 900 and verify.FLOOR == 2*1024**3 and verify.RESERVE == 64*1024**2


@pytest.fixture
def owned_tiny_output(tmp_path, monkeypatch):
    # HH_261008 - ROOT-requested isolated tiny fictional JSON files only; no source data or numeric fixtures.
    output, plan = tmp_path/"output", tmp_path/"plan"
    output.mkdir()
    plan.mkdir()
    monkeypatch.setattr(verify, "OUT", output)
    monkeypatch.setattr(verify, "PLAN_DIR", plan)
    return output


def test_tiny_create_only_eight_row_shard_closing_RAW_SHA_and_generation(owned_tiny_output):
    path = owned_tiny_output/"fictional.jsonl"
    writer = verify.ScalarShardWriter(path, list(range(1, 9)), 4096, lambda: None)
    expected = b""
    for step in range(1, 9):
        row = dict(step=step, fictional=True)
        expected += verify.encoded(row)
        writer.append(row)
    writer.close()
    snapshot = writer.snapshot()
    assert path.read_bytes() == expected
    assert snapshot["rows_written"] == 8 and snapshot["bytes_written"] == len(expected)
    checked = snapshot["closing_check"]
    assert checked["status"] == "MATCHED" and checked["bytes_read"] == len(expected)
    assert checked["sha256"] == checked["written_sha256"] == hashlib.sha256(expected).hexdigest()
    assert checked["generation"] == verify.old.generation(path.stat())
    assert writer.stream.closed and snapshot["closing_errors"] == []
    with pytest.raises(ValueError, match="fresh owned"):
        verify.ScalarShardWriter(path, list(range(1, 9)), 4096, lambda: None)
    assert path.read_bytes() == expected


def test_tiny_same_inode_same_length_tamper_is_DENIED_and_FD_closed(owned_tiny_output):
    path = owned_tiny_output/"tamper.jsonl"
    writer = verify.ScalarShardWriter(path, [1], 4096, lambda: None)
    writer.append(dict(step=1, fictional="a"))
    fd, before = writer.stream.fileno(), path.stat()
    raw = path.read_bytes().replace(b'"a"', b'"b"')
    assert len(raw) == before.st_size
    with path.open("r+b") as stream:
        stream.write(raw)
        stream.flush()
    assert path.stat().st_ino == before.st_ino and path.stat().st_size == before.st_size
    with pytest.raises(ValueError, match="closing raw scalar"):
        writer.close()
    assert writer.closed and writer.stream.closed and writer.snapshot()["closing_errors"]
    assert writer.snapshot()["closing_check"]["status"] == "FAILED_CHECKING"
    with pytest.raises(OSError):
        os.fstat(fd)


def test_tiny_prewrite_checkpoint_failure_leaves_zero_saved_rows(owned_tiny_output):
    denied = [True]
    def guard():
        if denied[0]:
            raise RuntimeError("synthetic checkpoint denied")
    writer = verify.ScalarShardWriter(owned_tiny_output/"denied.jsonl", [1], 4096, guard)
    with pytest.raises(RuntimeError, match="checkpoint denied"):
        writer.append(dict(step=1, fictional=True))
    assert writer.count == writer.bytes_written == 0
    assert writer.path.stat().st_size == 0
    denied[0] = False
    writer.close()
    assert writer.snapshot()["closing_check"]["status"] == "MATCHED"


@pytest.mark.parametrize("failure", ["line", "shard", "quota"])
def test_tiny_line_shard_and_combined_quota_DENY_before_write(owned_tiny_output, monkeypatch, failure):
    maximum = 8 if failure == "shard" else 128*1024
    writer = verify.ScalarShardWriter(owned_tiny_output/(failure+".jsonl"), [1], maximum, lambda: None)
    row = dict(step=1, fictional="x"*verify.HOST_LINE) if failure == "line" else dict(step=1)
    if failure == "quota":
        monkeypatch.setattr(verify, "owned_bytes", lambda: verify.LIMIT-verify.CLOSING-1)
    with pytest.raises(ValueError, match="bounded scalar|combined write quota"):
        writer.append(row)
    assert writer.count == writer.bytes_written == 0 and writer.path.stat().st_size == 0
    writer.close()
    assert writer.stream.closed


def test_tiny_raw_closing_checkpoint_DENY_retains_NOTCHECKED_and_releases_FD(owned_tiny_output):
    denied = [False]
    def guard():
        if denied[0]:
            raise RuntimeError("closing denied")
    writer = verify.ScalarShardWriter(owned_tiny_output/"closing.jsonl", [1], 4096, guard)
    writer.append(dict(step=1, fictional=True))
    denied[0] = True
    with pytest.raises(RuntimeError, match="closing denied"):
        writer.close()
    snapshot = writer.snapshot()
    assert snapshot["rows_written"] == 1
    assert snapshot["closing_check"] == dict(status="FAILED_NOTCHECKED", bytes_read=0)
    assert snapshot["closing_errors"][0]["scope"] == "shard_raw_closing_check"
    assert writer.stream.closed
