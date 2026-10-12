"""HH_261008 - Scalar-only fictional saved serial-input contracts; no arrays or actual files.

ROOT alone executes these tests after reviewing both sources. Fictional SHA and
metadata dictionaries prove validation behavior only, not actual input, NumPy
conversion, consumer execution, training, ownership or driving functionality.
"""
import ast
from copy import deepcopy
from pathlib import Path

import pytest

from scripts.e2e import check_map_serial_training_input_saved_20261008_v1 as check


def original(step=1):
    return dict(step=step, physical_slot_indices=[0, 1, 728, 729, 730, 1208, 42, 42],
        shared_crop_boxes=[[0, 0, 300, 400] for _ in range(8)], actual_paired_TRUE_bits=[0]*8,
        RGB_sha256="a"*64, target_sha256="b"*64)


def slots(row):
    return [dict(position=position, slot=slot, native_RGB_sha256="c"*64, native_target_sha256="d"*64,
        reader_provenance_authenticated_by_worker=False) for position, slot in enumerate(row["physical_slot_indices"])]


def descriptor(shape, dtype, pin, upper):
    amount = 1
    for value in shape:
        amount *= value
    return dict(shape=shape, dtype=dtype, nbytes=amount*({"uint8":1, "float32":4, "int64":8}[dtype]),
        owned=True, C_contiguous=True, finite=True, minimum=0.0 if dtype == "float32" else 0,
        maximum=float(upper) if dtype == "float32" else upper, sha256=pin)


def worker(row, rescue=False):
    chosen = deepcopy(row["shared_crop_boxes"])
    if rescue:
        chosen[0] = [0, 0, 360, 640]
    journal = dict(schema="portable_e2e.map_crop_preservation_journal.20261008.v1", policy_source_sha256=check.POLICY_PIN,
        baseline_exposure=deepcopy(row), selected_crop_boxes=chosen,
        full_GT_sign_pixels=[3]+[0]*7, original_NEAREST_sign_pixels=[0 if rescue else 3]+[0]*7,
        protected=[True, False]*4, rescued=[rescue]+[False]*7,
        candidate_RGB_sha256="e"*64 if rescue else row["RGB_sha256"],
        candidate_target_sha256="f"*64 if rescue else row["target_sha256"])
    return dict(schema="portable_e2e.map_crop_batch_worker.20261008.v3.batch", step=row["step"], journal=journal,
        native_slots=slots(row), actual_baseline_RGB_and_GT_hashes_reproduced=True,
        candidate_hashes_computed_from_actual_owned_arrays=True, same_box_for_actual_RGB_and_GT=True,
        source_and_baseline_RAW_authenticated_by_worker=False, training_steps=0, NN=0)


def saved(mode=check.ARMS[0], rescue=False):
    row = original()
    candidate = worker(row, rescue) if mode == check.ARMS[1] else None
    native_row = candidate if candidate else deepcopy(row)
    rgb_pin = candidate["journal"]["candidate_RGB_sha256"] if candidate else row["RGB_sha256"]
    gt_pin = candidate["journal"]["candidate_target_sha256"] if candidate else row["target_sha256"]
    native = dict(RGB=descriptor([8,360,640,3], "uint8", rgb_pin, 255), target=descriptor([8,360,640], "uint8", gt_pin, 6))
    host = dict(RGB=descriptor([8,3,360,640], "float32", "1"*64, 1), target=descriptor([8,360,640], "int64", "2"*64, 6))
    receipt = dict(schema=check.ADAPTER_SCHEMA+".batch", mode=mode, step=1,
        provided_baseline=deepcopy(row), provided_baseline_sha256=check.metadata_sha(row),
        control_exposure=deepcopy(row) if candidate is None else None, candidate_worker_receipt=candidate,
        native_slots=slots(row), native_arrays=native, host_arrays=host, scope=deepcopy(check.ADAPTER_SCOPE))
    ack = dict(consumer=check.CONSUMER, accepted=True, host_RGB_sha256="1"*64, host_target_sha256="2"*64)
    proof = dict(all_actual_host_values_compared=True, finite_RGB_in_unit_interval=True, GT_labels_0_to_6=True,
        normalized_RGB_sha256="1"*64, int64_target_sha256="2"*64, Torch_imported=False,
        Torch_numeric_equivalence_proven=False, NN=0, optimizer_updates=0)
    record = dict(schema=check.PRODUCER_SCHEMA+".host_receipt", arm=mode, step=1,
        adapter_receipt=receipt, actual_consumer_receipt=ack, host_value_proof=proof,
        chronology="CONSUMER_CHECKED_METADATA_WRITTEN_BEFORE_ADAPTER_ACK")
    return record, row, native_row


@pytest.mark.parametrize("raw", [b'{"a":1,"a":2}', b'{"a":NaN}', b'{"a":Infinity}', b'{"a":1e309}'])
def test_duplicate_nonfinite_metadata_denied(raw):
    with pytest.raises(ValueError):
        check.decode(raw)


def test_int64_ns_and_exact_bool_types_do_not_coerce():
    value = 1791453600123456789
    assert check.decode(check.encoded({"ns":value}))["ns"] == value
    assert not check.same(0, False) and not check.same([1], [1.0])
    assert not check.integer(True) and not check.sha("A"*64)


def test_contract_and_authority_keep_FIRST16_NUMPY_only_worker0_and_original_denials():
    contract = check.contract()
    assert contract["original20k_batches_outside_declaration_per_arm"] == 19984
    assert [row["declared_steps"] for row in contract["shards_per_arm"]] == [list(range(1,9)), list(range(9,17))]
    assert contract["prospective_full20k_sharding"]["numeric_batches_executed"] == 0
    assert contract["Torch_numeric_equivalence_proven"] is contract["trainer_connected"] is False
    grant = check.authority("a"*64)
    assert grant["training_authorized"] is grant["NN_authorized"] is grant["Torch_authorized"] is False
    assert grant["old_target_only_or_cache_AB_authority_reused"] is False
    assert len(check.SOURCE_NAMES) == 14 and len(check.INPUTS) == 7


@pytest.mark.parametrize("mode,rescue", [(check.ARMS[0],False), (check.ARMS[1],False), (check.ARMS[1],True)])
def test_fictional_control_candidate_and_host_ACK_metadata_join(mode, rescue):
    record, row, native = saved(mode, rescue)
    observed = check.host_record(record, row, mode, native)
    assert observed["native_receipts"] == 8 and observed["rescued_slots"] == int(rescue)
    ack = record["actual_consumer_receipt"]
    inner = dict(step=1, consumer_receipt=ack, native_and_host_buffers_rehashed=True, consumer_execution_authenticated=False)
    witness = dict(step=1, adapter_ACK_received=True, consumer_receipt=ack, adapter_acknowledgement=inner)
    assert check.acknowledgement(witness, record, 1) == inner
    witness["adapter_ACK_received"] = 1
    with pytest.raises(ValueError):
        check.acknowledgement(witness, record, 1)


@pytest.mark.parametrize("fault", ["control_candidate", "mode", "original_step_bool", "metadataSHA", "nativeSHA",
    "host_dtype", "NCHW", "finite", "owned", "scope", "ACKbool", "ACKSHA", "Torch", "chronology"])
def test_forged_or_coerced_mode_native_host_scope_and_consumer_metadata_denied(fault):
    record, row, native = saved()
    adapter = record["adapter_receipt"]
    if fault == "control_candidate": adapter["candidate_worker_receipt"] = worker(row)
    elif fault == "mode": adapter["mode"] = check.ARMS[1]
    elif fault == "original_step_bool": adapter["provided_baseline"]["step"] = True
    elif fault == "metadataSHA": adapter["provided_baseline_sha256"] = "0"*64
    elif fault == "nativeSHA": adapter["native_arrays"]["RGB"]["sha256"] = "c"*64
    elif fault == "host_dtype": adapter["host_arrays"]["RGB"]["dtype"] = "float64"
    elif fault == "NCHW": adapter["host_arrays"]["RGB"]["shape"] = [8,360,640,3]
    elif fault == "finite": adapter["host_arrays"]["RGB"]["finite"] = 1
    elif fault == "owned": adapter["host_arrays"]["target"]["owned"] = False
    elif fault == "scope": adapter["scope"]["TRAIN_admitted_by_adapter"] = True
    elif fault == "ACKbool": record["actual_consumer_receipt"]["accepted"] = 1
    elif fault == "ACKSHA": record["actual_consumer_receipt"]["host_target_sha256"] = "3"*64
    elif fault == "Torch": record["host_value_proof"]["Torch_numeric_equivalence_proven"] = True
    else: record["chronology"] = "PRODUCED_EQUALS_CONSUMED"
    with pytest.raises(ValueError):
        check.host_record(record, row, check.ARMS[0], native)


@pytest.mark.parametrize("fault", ["falsebits", "rescued_bool", "full_support", "selected_box", "native_slot", "worker_NN"])
def test_candidate_policy_and_native_witness_are_not_just_supplied_SHA_strings(fault):
    row, saved_worker = original(), worker(original(), rescue=True)
    if fault == "falsebits": saved_worker["journal"]["baseline_exposure"]["actual_paired_TRUE_bits"][0] = False
    elif fault == "rescued_bool": saved_worker["journal"]["rescued"][0] = 1
    elif fault == "full_support": saved_worker["journal"]["full_GT_sign_pixels"][0] = 0
    elif fault == "selected_box": saved_worker["journal"]["selected_crop_boxes"][0] = [0,0,300,400]
    elif fault == "native_slot": saved_worker["native_slots"][0]["slot"] = "0"
    else: saved_worker["NN"] = False
    with pytest.raises(ValueError):
        check.candidate_worker(saved_worker, row)


def test_complete_pipeline_has_all_distinct_stage_facts_and16_denominator():
    value = dict(expected_batches=16, produced_batches=16, consumer_checked_batches=16,
        native_rows_written=16, host_rows_written=16, acknowledged_batches=16,
        PASS_batches=16, FAIL_batches=0, UNEXECUTED_batches=0, failed_step=None,
        original20k_batches_outside_this_declaration=19984)
    check.pipeline(value)
    for key in ("produced_batches", "consumer_checked_batches", "host_rows_written", "acknowledged_batches"):
        changed = deepcopy(value)
        changed[key] = 15
        with pytest.raises(ValueError):
            check.pipeline(changed)


def summary_fixture():
    selected = [dict(step=step, baseline=original(step)) for step in range(1,17)]
    acks = [dict(step=step, consumer_receipt=dict(consumer=check.CONSUMER, accepted=True,
        host_RGB_sha256="a"*64, host_target_sha256="b"*64), native_and_host_buffers_rehashed=True,
        consumer_execution_authenticated=False) for step in range(1,17)]
    value = dict(schema=check.ADAPTER_SCHEMA+".summary", mode=check.ARMS[0], declared_steps=list(range(1,17)),
        closed=True, status="COMPLETE_ACKNOWLEDGED_INPUTS_NOT_TRAINING_OR_MODEL_APPROVAL", scope=deepcopy(check.ADAPTER_SCOPE),
        expected_batches=16, PASS_batches=16, FAIL_batches=0, UNEXECUTED_batches=0,
        produced_batches=16, production_attempts=16, production_FAIL_batches=0, production_UNEXECUTED_batches=0,
        consumption_attempts=16, consumption_attempted_batches=16, consumption_PASS_batches=16,
        consumption_FAIL_batches=0, consumption_UNEXECUTED_batches=0, actual_native_read_attempts=128,
        original20000_batches_outside_declaration=19984, pending_step=None, failure=None, failed_tail=None,
        cleanup_errors=[], reader_closing_observations=[], acknowledgements=deepcopy(acks), owner_PID=123,
        owner_thread_ident=456, provided_rows_sha256=check.metadata_sha([item["baseline"] for item in selected]))
    return value, selected, acks


def test_full16_summary_produced_and_consumed_ACK_join_remains_metadata_only():
    value, selected, acks = summary_fixture()
    check.adapter_summary(value, check.ARMS[0], selected, acks)


@pytest.mark.parametrize("fault", ["ACKlist", "owner_bool", "unconsumed", "pending", "native_reads"])
def test_summary_cannot_hide_ACK_ownership_type_pending_or_unconsumed_tail(fault):
    value, selected, acks = summary_fixture()
    if fault == "ACKlist": value["acknowledgements"] = value["acknowledgements"][:-1]
    elif fault == "owner_bool": value["owner_PID"] = True
    elif fault == "unconsumed": value["consumption_PASS_batches"] = 15
    elif fault == "pending": value["pending_step"] = 16
    else: value["actual_native_read_attempts"] = 127
    with pytest.raises(ValueError):
        check.adapter_summary(value, check.ARMS[0], selected, acks)


@pytest.mark.parametrize("fault", ["bool_nbytes", "float_target", "nan", "max", "extra"])
def test_descriptor_does_not_accept_type_range_or_added_claims(fault):
    value = descriptor([8,360,640], "int64", "a"*64, 6)
    if fault == "bool_nbytes": value["nbytes"] = True
    elif fault == "float_target": value["minimum"] = 0.0
    elif fault == "nan": value["maximum"] = float("nan")
    elif fault == "max": value["maximum"] = 7
    else: value["OS_memory_containment"] = True
    with pytest.raises(ValueError):
        check.descriptor(value, [8,360,640], "int64", 6)


def test_source_is_stdlib_only_create_only_and_no_producer_or_array_replay():
    tree = ast.parse(Path(check.__file__).read_bytes())
    imports = {alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
    imports |= {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
    assert not imports & {"numpy", "PIL", "torch", "carla", "rclpy", "zipfile", "subprocess", "importlib"}
    calls = {ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node, ast.Call)}
    assert not calls & {"np.load", "np.array", "torch.load", "Image.open", "exec", "eval"}
    source = ast.unparse(tree)
    assert "'xb'" in source and "close_seals" in source
    assert "native_or_host_array_SHA_independently_recomputed=False" in source
    assert "host_normalization_second_replays=0" in source
