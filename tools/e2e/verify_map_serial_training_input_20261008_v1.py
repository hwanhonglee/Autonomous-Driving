"""HH_261008 - Fresh bounded FIRST16 serial NumPy host-input verification, never training.

SOURCE_ONLY until ROOT reads the complete source/tests and explicitly executes
fresh preparation and RAW-pinned ROOT plan/authority. Original crop and sign
preservation use the same V3 reader with its fixed cache enabled. Consumer
acceptance, written metadata and adapter ACK are separate facts. No Torch,
optimizer, parallel loader, simulator, raster output or full20k numeric replay.
Old helper imports below are pure reads only; their old scoped Budget/writers,
fixed contracts, grants and execution functions are never used or mutated.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import fcntl
import hashlib
import math
import os
from pathlib import Path
import stat
import sys
import time

from scripts.e2e import verify_map_crop_rescue_reader_ab_20261008_v1 as old


ROOT = Path(__file__).resolve().parents[2]
NAME = "map_serial_training_input_v1"
SCHEMA = "portable_e2e.map_serial_training_input_verification.20261008.v1"
PLAN_DIR = ROOT / "storage/experiments/plans/2026-10-08" / NAME
OUT = ROOT / "storage/experiments/diagnostics/2026-10-08" / NAME
LOCK = ROOT / "storage/assets/locks/autoware_e2e_runtime.lock"
DATA, RUN = old.DATA, old.RUN
STEPS = list(range(1, 17))
ARMS = ("ORIGINAL_CROP_CONTROL", "SIGN_PRESERVING_CANDIDATE")
FLOOR, RESERVE = 2 * 1024**3, 64 * 1024**2
LIMIT, CLOSING, RSS, DEADLINE = 2 * 1024**2, 512 * 1024, 512 * 1024**2, 900
NATIVE_LINE, HOST_LINE = 8 * 1024, 16 * 1024
NATIVE_SHARD_LIMIT, HOST_SHARD_LIMIT = 64 * 1024, 128 * 1024
SOURCES = [
    "tools/e2e/verify_map_crop_rescue_reader_ab_20261008_v1.py",
    "tools/workspace_python.py", "tools/canonical_workspace.py",
    "learning/portable_e2e/__init__.py", "learning/portable_e2e/contract.py",
    "learning/portable_e2e/map_native_pair_reader_20261008_v3.py",
    "learning/portable_e2e/map_crop_batch_worker_20261008_v3.py",
    "learning/portable_e2e/map_crop_preservation_journal_20261008_v1.py",
    "tools/e2e/map_train_sign_preserving_crop_policy_20261008_v1.py",
    "tools/e2e/map_appearance_matched_contract_20261001.py",
    "learning/portable_e2e/map_serial_training_input_adapter_20261008_v1.py",
    "learning/tests/test_map_serial_training_input_adapter_20261008_v1.py",
    "tools/e2e/verify_map_serial_training_input_20261008_v1.py",
    "learning/tests/test_verify_map_serial_training_input_20261008_v1.py",
]
INPUTS = {path: pin for path, pin in old.INPUTS.items() if path != str(old.SELECTION)}
require, canonical, digest = old.require, old.canonical, old.digest
encoded, decode, equal = old.encoded, old.decode, old.typed_equal


def integer(value, minimum=0, maximum=2**63-1):
    return type(value) is int and minimum <= value <= maximum


def sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def safe_error(scope, error):
    args = BaseException.args.__get__(error, type(error))
    message = args[0][:1024] if args and type(args[0]) is str else "<non-string exception argument>"
    return dict(scope=scope, type=type(error).__name__[:64], message=message)


def shard_plan(steps=STEPS, size=8):
    require(type(steps) is list and steps and all(integer(step, 1, 20000) for step in steps)
        and steps == list(range(steps[0], steps[-1]+1)) and integer(size, 1, 20000),
        "fixed contiguous chronological shard steps required")
    return [dict(ordinal=index//size, declared_steps=steps[index:index+size],
        first_step=steps[index], last_step=steps[min(index+size, len(steps))-1],
        expected_batches=len(steps[index:index+size])) for index in range(0, len(steps), size)]


def contract():
    return dict(declared_steps=STEPS, expected_batches_per_arm=16, expected_slot_exposures_per_arm=128,
        arms=list(ARMS), fixed_sequential_arm_order=list(ARMS), shards_per_arm=shard_plan(),
        reader_revision="V3", same_reader_source_and_cache_enabled=True,
        checkpoint_max_states=48, checkpoint_state_account_bytes=256*1024,
        checkpoint_cache_account_limit_bytes=12*1024**2, checkpoint_stride_bytes=32*1024**2,
        owner_checkpoint_guard_stride=16, cache_accounting_is_reservation_not_exact_native_allocation=True,
        host_RGB_shape=[8, 3, 360, 640], host_RGB_dtype="float32", RGB_divisor=255,
        host_target_shape=[8, 360, 640], host_target_dtype="int64", labels=list(range(7)),
        normalization_proven="NumPy owned C-order float32 NCHW /np.float32(255); int64 GT",
        consumer="nontraining_numpy_host_value_consumer_v1", consumer_checks_all_actual_host_values=True,
        ACK_after_consumer_and_both_metadata_appends=True, written_not_automatically_consumed=True,
        original_control_exposure_has_exact_six_fields=True, candidate_worker_policy_journal_distinct=True,
        allowed_original_members=["rgb.npy", "target.npy"], allowed_paired_members=["rgb_false.npy", "target.npy"],
        TRUE_DEV_HOLDOUT_TEST_weights_deniedRAW_decode_authorized=False,
        original20k_batches_outside_declaration_per_arm=19984,
        prospective_full20k_sharding=dict(metadata_only=True, batches_per_shard=8, shard_count=2500,
            numeric_batches_executed=0, full20k_execution_authorized=False),
        CPU_affinity=[3], threads=1, sampled_VmHWM_cap_bytes=RSS, deadline_seconds=DEADLINE,
        floor_bytes=FLOOR, reserve_bytes=RESERVE, maximum_combined_PLAN_OUTPUT_bytes=LIMIT,
        closing_bytes=CLOSING, native_journal_maximum_bytes_per_shard=NATIVE_SHARD_LIMIT,
        host_receipt_maximum_bytes_per_shard=HOST_SHARD_LIMIT, PNG_created=False,
        same_source_pins_before_after=True, selected_scientific_providers=6,
        sampled_guards_not_OS_containment=True, old_authority_reused=False,
        Torch_imported=False, Torch_numeric_equivalence_proven=False, trainer_connected=False,
        multiworker_implemented=False, optimizer_updates=0, NN=0, GPU=0,
        main_approved=0, sub_approved=0, model_performance_gain_proven=False)


def authority(plan_pin):
    return dict(schema=SCHEMA+".ROOT_authority", plan_path=str(PLAN_DIR/"ROOT_plan.json"), plan_sha256=plan_pin,
        serial_FIRST16_original_and_FALSE_RGB_GT_host_verification_authorized=True,
        original_control_and_sign_preserving_candidate_authorized=True,
        nontraining_NumPy_consumer_and_sharded_metadata_authorized=True,
        training_authorized=False, NN_authorized=False, Torch_authorized=False,
        TRUE_DEV_HOLDOUT_TEST_weights_deniedRAW_decode_authorized=False,
        old_target_only_or_cache_AB_authority_reused=False, output=str(OUT), contract=contract())


def owned_bytes():
    total = 0
    for folder in (PLAN_DIR, OUT):
        canonical(folder, False)
        if not folder.exists():
            continue
        for path in folder.rglob("*"):
            canonical(path)
            info = path.lstat()
            require(stat.S_ISREG(info.st_mode) or stat.S_ISDIR(info.st_mode), "owned special path denied")
            if stat.S_ISREG(info.st_mode):
                require(info.st_nlink == 1, "owned output hardlink denied")
                total += info.st_size
    return total


class Budget:
    """HH_261008 - Own only this fresh plan/output and observed process/held lease generation."""

    def __init__(self, lease):
        self.lease, self.started, self.identity = lease, time.monotonic(), old.process_identity()
        self.maximum_HWM, self.guard_calls, self.last = 0, 0, None
        self.lease_generation = old.generation(os.fstat(lease.fileno()))

    def guard(self):
        self.guard_calls += 1
        require(not self.lease.closed and old.process_identity() == self.identity, "owned process/lease generation differs")
        path, fd = canonical(LOCK).stat(), os.fstat(self.lease.fileno())
        require(stat.S_ISREG(path.st_mode) and path.st_nlink == fd.st_nlink == 1
            and old.generation(path) == old.generation(fd) == self.lease_generation, "held lease FD/path replaced")
        raw = Path("/proc/self/status").read_text()
        self.last = {line.split(":")[0]: int(line.split()[1])*1024 for line in raw.splitlines()
            if line.startswith(("VmRSS:", "VmHWM:"))}
        self.maximum_HWM = max(self.maximum_HWM, self.last["VmHWM"])
        require(self.maximum_HWM <= RSS and time.monotonic()-self.started < DEADLINE,
            "unchanged sampled memory/deadline limit exceeded")
        fs = os.statvfs(ROOT)
        require(fs.f_bavail*fs.f_frsize >= FLOOR+RESERVE, "unchanged floor plus reserve unavailable")
        require(owned_bytes() <= LIMIT-CLOSING, "unchanged closing reserve unavailable")


def write_new(path, raw, *, closing=False):
    path = canonical(path, False)
    require(type(raw) is bytes and len(raw) <= CLOSING and
        (path.is_relative_to(PLAN_DIR) or path.is_relative_to(OUT)), "bounded fresh owned metadata required")
    require(owned_bytes()+len(raw) <= LIMIT-(0 if closing else CLOSING), "unchanged combined2MiB quota exceeded")
    with path.open("xb") as stream:
        require(stream.write(raw) == len(raw), "partial fresh metadata write")
        stream.flush()


def sources(budget):
    require(len(SOURCES) == len(set(SOURCES)) == 14, "exact imported/own source closure14 required")
    return {str(ROOT/path): digest(ROOT/path, budget.guard) for path in SOURCES}


def archives(plan):
    return {str(PLAN_DIR/"sources"/(pin+".py")): pin for pin in set(plan["source_sha256"].values())}


def seals(plan):
    groups = (plan["source_sha256"], plan["input_sha256"], plan["environment"]["selected_provider_sha256"], archives(plan))
    value = {}
    for group in groups:
        for path, pin in group.items():
            require(sha(pin) and (path not in value or value[path] == pin), "overlapping/source seal conflict")
            value[path] = pin
    return value


def quota(source_bytes, plan_bytes):
    require(integer(source_bytes) and integer(plan_bytes, 1, CLOSING), "bounded prospective source/plan bytes required")
    total = source_bytes + 4*plan_bytes + 4*(NATIVE_SHARD_LIMIT+HOST_SHARD_LIMIT) + CLOSING + 64*1024
    return dict(source_archive_bytes=source_bytes, four_plan_copy_allowance_bytes=4*plan_bytes,
        maximum_four_native_and_four_host_shard_bytes=4*(NATIVE_SHARD_LIMIT+HOST_SHARD_LIMIT),
        closing_bytes=CLOSING, extra_ROOT_metadata_allowance_bytes=64*1024,
        prospective_combined_PLAN_OUTPUT_bytes=total, unchanged_limit_bytes=LIMIT,
        fits_unchanged_limits=total < LIMIT, metadata_envelope_not_actual_array_execution=True)


def prepare(budget):
    require(not PLAN_DIR.exists() and not OUT.exists(), "fresh preparation only; no overwrite/resume")
    budget.guard()
    env = old.environment()
    table = sources(budget)
    require(len(env["selected_provider_sha256"]) == 6, "unchanged selected scientific provider6 required")
    for path, pin in INPUTS.items():
        require(digest(path, budget.guard) == pin, "original admitted input seal differs")
    registration = old.registration()
    rows = old.baseline_rows(16)
    plan = dict(schema=SCHEMA, provenance="HH_261008 - Fresh FIRST16 serial NumPy host input, no training.",
        contract=contract(), source_sha256=table, input_sha256=INPUTS, environment=env,
        selected_baseline_rows=rows, registration_scope=registration,
        output=str(OUT), lock_generation=budget.lease_generation, source_archive_count=len(set(table.values())),
        prepare_process_generation=budget.identity)
    raw = encoded(plan)
    proof = quota(sum(Path(path).stat().st_size for path in table), len(raw)+4096)
    require(proof["fits_unchanged_limits"], "fresh FIRST16 quota does not fit unchanged2MiB cap")
    plan["prospective_storage"] = proof
    raw = encoded(plan)
    require(quota(sum(Path(path).stat().st_size for path in table), len(raw))["fits_unchanged_limits"], "final exact plan quota denied")
    PLAN_DIR.mkdir(parents=True, exist_ok=False)
    (PLAN_DIR/"sources").mkdir(exist_ok=False)
    for path, pin in table.items():
        raw_source = canonical(path).read_bytes()
        require(len(raw_source) <= CLOSING and hashlib.sha256(raw_source).hexdigest() == pin, "source changed before archive copy")
        destination = PLAN_DIR/"sources"/(pin+".py")
        if not destination.exists():
            write_new(destination, raw_source)
    write_new(PLAN_DIR/"candidate_plan.json", raw)
    return dict(status="PREPARED_NOT_AUTHORIZED_OR_RUNNING", candidate_sha256=hashlib.sha256(raw).hexdigest(),
        source_archive_count=len(set(table.values())), RGB_GT_or_host_arrays_decoded=0, contract=contract())


def admission(plan_pin, grant_pin, budget):
    require(sha(plan_pin) and sha(grant_pin), "external RAW ROOT plan/authority SHA required")
    for path in (PLAN_DIR/"ROOT_plan.json", PLAN_DIR/"ROOT_authority.json"):
        require(canonical(path).stat().st_size <= CLOSING, "bounded RAW ROOT metadata required")
    raw_plan = (PLAN_DIR/"ROOT_plan.json").read_bytes()
    raw_grant = (PLAN_DIR/"ROOT_authority.json").read_bytes()
    require(hashlib.sha256(raw_plan).hexdigest() == plan_pin and hashlib.sha256(raw_grant).hexdigest() == grant_pin,
        "external RAW ROOT seals differ")
    plan, grant = decode(raw_plan), decode(raw_grant)
    require(plan["schema"] == SCHEMA and equal(plan["contract"], contract()) and equal(grant, authority(plan_pin))
        and plan["output"] == str(OUT) and equal(plan["input_sha256"], INPUTS), "fresh exact serial ROOT contract/grant required")
    require(equal(plan["source_sha256"], sources(budget)) and equal(plan["environment"], old.environment())
        and equal(plan["lock_generation"], budget.lease_generation), "source/provider/runtime lease generation differs")
    require(digest(PLAN_DIR/"candidate_plan.json", budget.guard) == plan_pin, "ROOT plan differs from prepared candidate")
    for path, pin in seals(plan).items():
        require(digest(path, budget.guard) == pin, "whole upfront source/input/provider/archive seal differs")
    require(equal(plan["selected_baseline_rows"], old.baseline_rows(16))
        and equal(plan["registration_scope"], old.registration()), "original FIRST16 or denied-source roles differ")
    require(not OUT.exists(), "fresh actual output required; no overwrite/retry")
    return plan, raw_plan, raw_grant


class Ledger:
    """HH_261008 - Acknowledge only after consume and both appends; retain produced/written facts separately."""

    def __init__(self, expected=16):
        require(integer(expected, 1, 16), "bounded dry-consumer denominator required")
        self.expected = expected
        self.produced = self.consumer_checked = self.native_written = self.host_written = self.acknowledged = 0
        self.failed_step = None

    def event(self, kind, step):
        require(integer(step, 1, self.expected) and self.failed_step is None, "strict in-scope chronology required")
        previous = self.acknowledged
        if kind == "produced":
            require(step == previous+1 and self.produced == previous, "one pending produced batch required")
            self.produced += 1
        elif kind == "consumer_checked":
            require(step == self.produced and self.consumer_checked == previous, "consume follows actual production")
            self.consumer_checked += 1
        elif kind == "native_written":
            require(step == self.consumer_checked and self.native_written == previous, "native append follows consume")
            self.native_written += 1
        elif kind == "host_written":
            require(step == self.native_written and self.host_written == previous, "host append follows native append")
            self.host_written += 1
        elif kind == "acknowledged":
            require(step == self.host_written and self.acknowledged == self.produced-1, "ACK follows both successful appends")
            self.acknowledged += 1
        else:
            raise ValueError("unknown pipeline event")

    def fail(self, step):
        require(integer(step, 1, self.expected), "in-scope attempted failure step required")
        if step > self.acknowledged:
            require(step == self.acknowledged+1, "failure cannot skip chronology")
            self.failed_step = step

    def snapshot(self):
        fail = int(self.failed_step is not None)
        return dict(expected_batches=self.expected, produced_batches=self.produced,
            consumer_checked_batches=self.consumer_checked, native_rows_written=self.native_written,
            host_rows_written=self.host_written, acknowledged_batches=self.acknowledged,
            PASS_batches=self.acknowledged, FAIL_batches=fail,
            UNEXECUTED_batches=self.expected-self.acknowledged-fail, failed_step=self.failed_step,
            original20k_batches_outside_this_declaration=20000-self.expected)


class ScalarShardWriter:
    """HH_261008 - Create-only bounded host/control JSONL, preserving raw closing verification states."""

    def __init__(self, path, steps, maximum, checkpoint):
        self.path, self.steps = canonical(path, False), list(steps)
        require(not self.path.exists() and self.path.is_relative_to(OUT), "fresh owned shard path required")
        require(integer(maximum, 1, HOST_SHARD_LIMIT) and callable(checkpoint), "bounded shard allowance/guard required")
        self.maximum, self.checkpoint = maximum, checkpoint
        self.stream = self.path.open("x+b")
        self.generation = old.generation(os.fstat(self.stream.fileno()))
        self.count = self.bytes_written = 0
        self.hash = hashlib.sha256()
        self.closing_check = dict(status="NOTCHECKED", bytes_read=0)
        self.closing_errors = []
        self.closed = False

    def append(self, row):
        require(not self.closed and self.count < len(self.steps) and type(row) is dict
            and type(row.get("step")) is int and row["step"] == self.steps[self.count], "exact chronological scalar append required")
        raw = encoded(row)
        require(len(raw) <= HOST_LINE and self.bytes_written+len(raw) <= self.maximum, "bounded scalar row/shard required")
        self.checkpoint()
        require(owned_bytes()+len(raw) <= LIMIT-CLOSING, "combined write quota with closing reserve denied")
        require(self.stream.write(raw) == len(raw), "partial scalar metadata append")
        self.stream.flush()
        self.hash.update(raw)
        self.bytes_written += len(raw)
        self.count += 1

    def close(self):
        if self.closed:
            return
        failure = None
        self.closing_check = dict(status="NOTCHECKED", bytes_read=0)
        try:
            self.checkpoint()
            self.stream.flush()
            before, fd = canonical(self.path).stat(), os.fstat(self.stream.fileno())
            require(before.st_nlink == fd.st_nlink == 1 and
                (before.st_dev, before.st_ino) == (fd.st_dev, fd.st_ino) == (self.generation[4], self.generation[5])
                and before.st_size == fd.st_size == self.bytes_written, "scalar shard FD/path/size changed")
            observed = hashlib.sha256()
            self.stream.seek(0)
            self.closing_check["status"] = "CHECKING"
            for block in iter(lambda: self.stream.read(64*1024), b""):
                self.checkpoint()
                observed.update(block)
                self.closing_check["bytes_read"] += len(block)
            after = self.path.stat()
            require(old.generation(before) == old.generation(after) == old.generation(os.fstat(self.stream.fileno()))
                and observed.hexdigest() == self.hash.hexdigest(), "closing raw scalar shard bytes/generation differ")
            self.closing_check.update(status="MATCHED", sha256=observed.hexdigest(), written_sha256=self.hash.hexdigest(),
                bytes=self.bytes_written, generation=old.generation(after))
        except BaseException as error:
            self.closing_errors.append(safe_error("shard_raw_closing_check", error))
            self.closing_check["status"] = "FAILED_"+self.closing_check["status"]
            failure = error
        finally:
            try:
                self.stream.close()
            except BaseException as error:
                self.closing_errors.append(safe_error("shard_FD_close", error))
                if failure is None:
                    failure = error
            self.closed = True
        if failure is not None:
            raise failure

    def snapshot(self):
        return dict(path=str(self.path), declared_steps=self.steps, rows_written=self.count,
            bytes_written=self.bytes_written, maximum_bytes=self.maximum,
            closing_check=deepcopy(self.closing_check), closing_errors=deepcopy(self.closing_errors))


def host_consumer(batch, checkpoint):
    """HH_261008 - Compare every actual NumPy host value independently of supplied scalar SHA claims."""
    import numpy as np
    checkpoint()
    rgb, target, host, labels = batch.native_RGB, batch.native_target, batch.host_RGB, batch.host_target
    require(all(type(value) is np.ndarray and value.flags.owndata and value.flags.c_contiguous
        for value in (rgb, target, host, labels)), "owned exact contiguous host/native arrays required")
    require(rgb.shape == (8, 360, 640, 3) and target.shape == (8, 360, 640)
        and rgb.dtype == target.dtype == np.uint8 and host.shape == (8, 3, 360, 640)
        and host.dtype == np.float32 and labels.shape == (8, 360, 640) and labels.dtype == np.int64,
        "fixed actual native/float32NCHW/int64GT ABI required")
    require(not np.shares_memory(host, rgb) and not np.shares_memory(labels, target)
        and not np.shares_memory(host, labels), "host/native storage alias denied")
    expected_rgb = np.ascontiguousarray(rgb.transpose(0, 3, 1, 2), dtype=np.float32)
    expected_rgb /= np.float32(255)
    expected_target = np.ascontiguousarray(target, dtype=np.int64)
    checkpoint()
    require(bool(np.isfinite(host).all()) and float(host.min()) >= 0 and float(host.max()) <= 1
        and int(labels.min()) >= 0 and int(labels.max()) <= 6
        and np.array_equal(host, expected_rgb) and np.array_equal(labels, expected_target),
        "actual host values differ from independent uint8 /255 or int64 conversion")
    hashes = [hashlib.sha256(memoryview(value).cast("B")).hexdigest() for value in (host, labels)]
    proof = dict(all_actual_host_values_compared=True, finite_RGB_in_unit_interval=True, GT_labels_0_to_6=True,
        normalized_RGB_sha256=hashlib.sha256(memoryview(expected_rgb).cast("B")).hexdigest(),
        int64_target_sha256=hashlib.sha256(memoryview(expected_target).cast("B")).hexdigest(),
        Torch_imported=False, Torch_numeric_equivalence_proven=False, NN=0, optimizer_updates=0)
    del expected_rgb, expected_target
    checkpoint()
    return dict(consumer="nontraining_numpy_host_value_consumer_v1", accepted=True,
        host_RGB_sha256=hashes[0], host_target_sha256=hashes[1]), proof


def validate_ack(value):
    require(type(value) is dict and set(value) == {"consumer", "accepted", "host_RGB_sha256", "host_target_sha256"}
        and value["consumer"] == "nontraining_numpy_host_value_consumer_v1" and value["accepted"] is True
        and sha(value["host_RGB_sha256"]) and sha(value["host_target_sha256"]), "exact accepted actual-consumer ACK receipt required")


def process_produced(batch, mode, baseline, adapter, native_writer, host_writer, ledger, consumer, checkpoint, tail):
    """HH_261008 - Explicit stage order; scalar fixtures may inject a consumer but do not certify host values."""
    step = batch.step
    require(type(step) is int and mode in ARMS, "fixed produced step/mode required")
    ledger.event("produced", step)
    tail.update(adapter_receipt=batch.receipt, phase="consume")
    ack, proof = consumer(batch, checkpoint)
    validate_ack(ack)
    ledger.event("consumer_checked", step)
    tail.update(consumer_receipt=ack, phase="native_append")
    if mode == ARMS[1]:
        value = batch.receipt["candidate_worker_receipt"]
        raw = encoded(value)
        require(len(raw) <= NATIVE_LINE and owned_bytes()+len(raw) <= LIMIT-CLOSING,
            "candidate original writer cannot exceed new owner quota")
        checkpoint()
        native_writer.append(batch.native_RGB, batch.native_target, value)
    else:
        value = batch.receipt["control_exposure"]
        require(equal(value, baseline) and len(encoded(value)) <= NATIVE_LINE,
            "exact bounded original six-field control exposure required")
        native_writer.append(value)
    ledger.event("native_written", step)
    tail["phase"] = "post_native_append_guard"
    checkpoint()
    tail["phase"] = "host_append"
    host_writer.append(dict(schema=SCHEMA+".host_receipt", arm=mode, step=step,
        adapter_receipt=batch.receipt, actual_consumer_receipt=ack, host_value_proof=proof,
        chronology="CONSUMER_CHECKED_METADATA_WRITTEN_BEFORE_ADAPTER_ACK"))
    ledger.event("host_written", step)
    tail["phase"] = "post_host_append_guard"
    checkpoint()
    tail["phase"] = "adapter_ACK"
    observed = adapter.acknowledge(step, consumer_receipt=ack)
    require(type(observed) is dict and set(observed) == {"step", "consumer_receipt",
        "native_and_host_buffers_rehashed", "consumer_execution_authenticated"}
        and equal(observed["step"], step) and equal(observed["consumer_receipt"], ack)
        and observed["native_and_host_buffers_rehashed"] is True
        and observed["consumer_execution_authenticated"] is False, "exact observed library acknowledgement required")
    ledger.event("acknowledged", step)
    return dict(step=step, adapter_ACK_received=True, consumer_receipt=ack, adapter_acknowledgement=observed)


def adapter_closed(summary):
    require(type(summary) is dict and summary["closed"] is True
        and summary["status"] == "COMPLETE_ACKNOWLEDGED_INPUTS_NOT_TRAINING_OR_MODEL_APPROVAL"
        and equal(summary["PASS_batches"], 16) and equal(summary["FAIL_batches"], 0)
        and equal(summary["UNEXECUTED_batches"], 0) and summary["failure"] is None
        and equal(summary["cleanup_errors"], []) and equal(summary["reader_closing_observations"], []),
        "adapter close summary retained errors or incomplete acknowledgements")


def close_one(value, scope, errors):
    if value is not None:
        try:
            value.close()
        except BaseException as error:
            errors.append(safe_error(scope, error))


def execute(plan_pin, grant_pin, budget):
    plan, raw_plan, raw_grant = admission(plan_pin, grant_pin, budget)
    arms = {mode: dict(pipeline=Ledger().snapshot(), adapter=None, shards=[], acknowledgements=[]) for mode in ARMS}
    failure, failure_tail, closing_errors, closing = None, None, [], []
    adapter = reader = native_writer = host_writer = None
    mode = step = None
    active = False
    try:
        OUT.mkdir(parents=True, exist_ok=False)
        write_new(OUT/"ROOT_plan.json", raw_plan)
        write_new(OUT/"ROOT_authority.json", raw_grant)
        # HH_261008 - Numeric setup remains inside the failure/closing ledger after fresh whole upfront seals.
        from portable_e2e import map_native_pair_reader_20261008_v3 as native
        from portable_e2e import map_crop_batch_worker_20261008_v3 as worker
        from portable_e2e import map_serial_training_input_adapter_20261008_v1 as serial
        require(serial.ORIGINAL_CROP_CONTROL == ARMS[0] and serial.SIGN_PRESERVING_CANDIDATE == ARMS[1], "fixed adapter modes differ")
        crop = worker.compile_original_crop(ROOT/"tools/e2e/map_appearance_matched_contract_20261001.py")
        require(equal(serial.plan_contiguous_shards(STEPS, batches_per_shard=8), shard_plan()), "adapter shard hierarchy differs")
        for mode in ARMS:
            ledger = Ledger()
            folder = OUT/mode
            folder.mkdir(exist_ok=False)
            reader = native.BoundedNativePairReader(DATA/"original_train729.npz", INPUTS[str(DATA/"original_train729.npz")],
                DATA/"paired_train480.npz", INPUTS[str(DATA/"paired_train480.npz")], checkpoint=budget.guard, cache_enabled=True)
            adapter = serial.SerialTrainingInputAdapter([row["baseline"] for row in plan["selected_baseline_rows"]],
                reader, crop, mode=mode, checkpoint=budget.guard)
            for shard in shard_plan():
                number, declared = shard["ordinal"], shard["declared_steps"]
                native_path = folder/("shard_%04d_exposures.jsonl" % number)
                host_path = folder/("shard_%04d_host_receipts.jsonl" % number)
                native_writer = (worker.BatchJournalWriter(native_path, declared, maximum_bytes=NATIVE_SHARD_LIMIT)
                    if mode == ARMS[1] else ScalarShardWriter(native_path, declared, NATIVE_SHARD_LIMIT, budget.guard))
                host_writer = ScalarShardWriter(host_path, declared, HOST_SHARD_LIMIT, budget.guard)
                record = dict(**shard, native=None, host=None)
                arms[mode]["shards"].append(record)
                for step in declared:
                    active = True
                    failure_tail = dict(arm=mode, step=step, phase="produce", adapter_receipt=None, consumer_receipt=None)
                    budget.guard()
                    batch = adapter.next_batch()
                    require(type(batch.step) is int and batch.step == step, "serial produced chronology differs")
                    acknowledgement = process_produced(batch, mode, plan["selected_baseline_rows"][step-1]["baseline"],
                        adapter, native_writer, host_writer, ledger, host_consumer, budget.guard, failure_tail)
                    arms[mode]["acknowledgements"].append(acknowledgement)
                    arms[mode]["pipeline"] = ledger.snapshot()
                    active, failure_tail = False, None
                    del batch
                    budget.guard()
                close_one(native_writer, "native_shard_close", closing_errors)
                if mode == ARMS[1]:
                    record["native"] = dict(path=str(native_path), rows_written=native_writer.count,
                        bytes_written=native_writer.bytes_written, closing_RAW_sha256=digest(native_path, budget.guard),
                        closing_check="MATCHED" if not closing_errors else "FAILED_OR_NOTCHECKED")
                else:
                    record["native"] = native_writer.snapshot()
                close_one(host_writer, "host_shard_close", closing_errors)
                record["host"] = host_writer.snapshot()
                native_writer = host_writer = None
                require(not closing_errors, "shard closing failed; preserve all preceding ACKs")
            close_one(adapter, "adapter_close", closing_errors)
            arms[mode]["adapter"] = adapter.stats_snapshot()
            adapter_closed(arms[mode]["adapter"])
            arms[mode]["reader_closing_errors"] = deepcopy(reader.closing_errors)
            arms[mode]["reader_stats_after_close"] = reader.stats_snapshot()
            adapter = reader = None
            require(not closing_errors, "adapter closing failed; no additional batch FAIL")
    except BaseException as error:
        failure = safe_error("pipeline", error)
        if mode is not None:
            if active and step is not None:
                ledger.fail(step)
                if adapter is not None:
                    try:
                        adapter.abort(error, step=step)
                    except BaseException as secondary:
                        closing_errors.append(safe_error("adapter_abort_secondary", secondary))
            arms[mode]["pipeline"] = ledger.snapshot()
    finally:
        close_one(native_writer, "native_shard_close_secondary", closing_errors)
        close_one(host_writer, "host_shard_close_secondary", closing_errors)
        if mode is not None and arms[mode]["shards"]:
            last = arms[mode]["shards"][-1]
            if host_writer is not None:
                last["host"] = host_writer.snapshot()
            if native_writer is not None:
                last["native"] = (native_writer.snapshot() if mode == ARMS[0] else dict(path=str(native_writer.path),
                    rows_written=native_writer.count, bytes_written=native_writer.bytes_written,
                    closing_check="FAILED_OR_NOTCHECKED", bounded_writer_summary=native_writer.summary()))
        close_one(adapter, "adapter_close_secondary", closing_errors)
        if adapter is not None and mode is not None:
            arms[mode]["adapter"] = adapter.stats_snapshot()
            if arms[mode]["adapter"].get("cleanup_errors"):
                closing_errors.append(dict(scope="adapter_summary_cleanup", type="AdapterCleanupErrors",
                    message="Adapter retained secondary closing errors; see complete adapter summary"))
        if reader is not None:
            close_one(reader, "reader_close_secondary", closing_errors)
            arms[mode]["reader_closing_errors"] = deepcopy(reader.closing_errors)
            arms[mode]["reader_stats_after_close"] = reader.stats_snapshot()
        for path, pin in seals(plan).items():
            observation = dict(path=path, expected_sha256=pin, observed_sha256=None, status="NOTCHECKED")
            try:
                actual = digest(path, budget.guard)
                observation.update(observed_sha256=actual, status="MATCHED" if actual == pin else "DIFFERENT")
                require(actual == pin, "closing source/input/provider/archive seal differs")
            except BaseException as error:
                observation["error"] = safe_error("closing_seal", error)
                closing_errors.append(observation["error"])
                if failure is None:
                    failure = observation["error"]
            closing.append(observation)
    completed = failure is None and not closing_errors and all(arms[name]["pipeline"]["PASS_batches"] == 16 for name in ARMS)
    result = dict(schema=SCHEMA+".result", status="COMPLETE_SERIAL_HOST_INPUT_NOT_TRAINING" if completed else "FAILED_WITH_COMPLETE_DECLARED_DENOMINATOR",
        ROOT_plan_sha256=plan_pin, ROOT_authority_sha256=grant_pin, contract=contract(), arms=arms,
        failure=failure, full_failed_tail=failure_tail, secondary_closing_errors=closing_errors,
        closing_seal_observations=closing, elapsed_seconds=time.monotonic()-budget.started,
        sampled_maximum_VmHWM_bytes=budget.maximum_HWM, sampled_closing_memory=budget.last,
        NN=0, optimizer_updates=0, Torch_imported=False, Torch_numeric_equivalence_proven=False,
        trainer_connected=False, multiworker_implemented=False, model_performance_gain_proven=False,
        main_approved=0, sub_approved=0)
    write_new(OUT/"result.json", encoded(result), closing=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("prepare", "execute"), required=True)
    parser.add_argument("--plan-sha256")
    parser.add_argument("--authority-sha256")
    args = parser.parse_args()
    require(sys.flags.dont_write_bytecode and sys.flags.optimize == 0, "clean -B assertion-enabled owner required")
    with canonical(LOCK).open("rb") as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            budget = Budget(lease)
            result = prepare(budget) if args.mode == "prepare" else execute(args.plan_sha256, args.authority_sha256, budget)
            print(encoded(dict(status=result["status"], NN=0, optimizer_updates=0)).decode(), end="")
            return 0 if result["status"].startswith(("PREPARED", "COMPLETE")) else 2
        finally:
            fcntl.flock(lease, fcntl.LOCK_UN)


if __name__ == "__main__":
    raise SystemExit(main())
