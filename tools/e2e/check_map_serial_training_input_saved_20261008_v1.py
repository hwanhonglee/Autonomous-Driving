"""HH_261008 - SOURCE-only independent saved serial host-input metadata/byte checker.

ROOT must read the final source/tests and separately authorize --execute with
external RAW plan/grant/source/test pins. Standard-library only; never import
producers, arrays, Torch, image decoders, models, simulators or remote clients.
Whole compressed NPZ seals do not inflate members. Native/host SHA, normalized
values and ACKs are producer measurements joined as saved metadata, not a second
independent array replay. Preserve originals and create one fresh failed receipt
on a check failure. This checker cannot authenticate a training consumer or
prove Torch/GPU equivalence, model gain or autonomous-driving functionality.
"""
from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
NAME = "map_serial_training_input_v1"
PRODUCER_SCHEMA = "portable_e2e.map_serial_training_input_verification.20261008.v1"
ADAPTER_SCHEMA = "portable_e2e.map_serial_training_input_adapter.20261008.v1"
SCHEMA = "portable_e2e.map_serial_training_input_saved_check.20261008.v1"
PLAN_DIR = ROOT/"storage/experiments/plans/2026-10-08"/NAME
OUT = ROOT/"storage/experiments/diagnostics/2026-10-08"/NAME
RECEIPT = OUT.parent/(NAME+"_independent_v1.json")
TEST_SOURCE = ROOT/"learning/tests/test_check_map_serial_training_input_saved_20261008_v1.py"
LOCK = ROOT/"storage/assets/locks/autoware_e2e_runtime.lock"
DATA = ROOT/"storage/experiments/training/2026-10-02/map_paired_balanced_20261002_v1"
RUN = ROOT/"storage/experiments/training/2026-10-02/map_native_fusion_pair_v3"
STEPS, ARMS = list(range(1, 17)), ("ORIGINAL_CROP_CONTROL", "SIGN_PRESERVING_CANDIDATE")
BLOCK, MAX_METADATA, NATIVE_LIMIT, HOST_LIMIT = 64*1024, 512*1024, 64*1024, 128*1024
FLOOR, RESERVE, RSS, DEADLINE = 2*1024**3, 64*1024**2, 512*1024**2, 900
CONSUMER = "nontraining_numpy_host_value_consumer_v1"
POLICY_PIN = "566b3e2afb7a0c879071c806c0cbfef6d31b8e974df99423339b7bf48cd6209f"
SOURCE_NAMES = {
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
}
INPUTS = {
    str(DATA/"original_train729.npz"): "d2d7b8b306fcba9672a229b4b1222b208e8d17fe500caf43755687612025e8f3",
    str(DATA/"paired_train480.npz"): "47c23a3f6fe67aa40ea7f5bb0c770fd6037463ed090f2b9390a89317c4dedee3",
    str(RUN/"control/exposures.jsonl"): "1fa5bfba0e979f049b04a9001e3fd6f52100bd77a45a7ce1ef63d1881d7b6225",
    str(RUN/"candidate/exposures.jsonl"): "1fa5bfba0e979f049b04a9001e3fd6f52100bd77a45a7ce1ef63d1881d7b6225",
    str(RUN/"inputs/manifest.json"): "614840471e938a0e9ecd169368d6240e96c01fdf76c74b56f82863c9d49d2b1b",
    str(RUN/"inputs/study_train_registry.json"): "ca892a6b6e34602b2c983fdec83972a3693bfd763baafb4abfac2f1a488bad8d",
    str(RUN/"inputs/paired_samples.json"): "899d82128ae9597beca6e22fbad4abaaff40c7e1fbc502418f946f71c72b9d6b",
}
ADAPTER_SCOPE = dict(worker_count=0, epochs_supported=False, multiprocess_supported=False,
    ROOT_or_data_authenticated_by_adapter=False, injected_crop_source_authenticated_by_adapter=False,
    TRAIN_admitted_by_adapter=False, consumer_execution_authenticated_by_adapter=False,
    Torch_or_GPU_numerical_equivalence_claimed=False, filesystem_written_by_adapter=False,
    public_arrays_immutable=False, owner_thread_bound=True, reentrant_calls_supported=False,
    sampled_guards_are_OS_containment=False, optimizer_steps=0, NN=0)


def require(ok, message):
    if not ok:
        raise ValueError(message)


def sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def integer(value, minimum=0, maximum=2**63-1):
    return type(value) is int and minimum <= value <= maximum


def same(left, right):
    if type(left) is not type(right):
        return False
    if type(left) is dict:
        return set(left) == set(right) and all(same(left[k], right[k]) for k in left)
    if type(left) is list:
        return len(left) == len(right) and all(same(a, b) for a, b in zip(left, right))
    return left == right


def decode(raw):
    def pairs(rows):
        result = {}
        for key, value in rows:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    def number(value):
        parsed = float(value)
        require(math.isfinite(parsed), "overflowing JSON float")
        return parsed
    return json.loads(raw, object_pairs_hook=pairs, parse_float=number,
        parse_constant=lambda _: require(False, "nonfinite JSON token"))


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)+"\n").encode()


def metadata_sha(value):
    # HH_261008 - Match the adapter's scalar metadata encoding, which deliberately has no newline.
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def seconds(value):
    return type(value) in (int, float) and math.isfinite(value) and 0 <= value < DEADLINE


def canonical(path, exists=True):
    path = Path(path)
    require(path.is_absolute() and ".." not in path.parts and path == path.resolve(strict=exists)
        and not any(p.is_symlink() for p in (path, *path.parents)), "canonical nonsymlink path required")
    return path


def generation(info):
    return [getattr(info, key) for key in ("st_mode", "st_uid", "st_gid", "st_nlink", "st_dev",
        "st_ino", "st_size", "st_mtime_ns", "st_ctime_ns")]


class Audit:
    """HH_261008 - Own one shared lease and independently seal bounded files before/after joins."""

    def __init__(self, lease):
        self.lease, self.started, self.files, self.maximum_HWM = lease, time.monotonic(), {}, 0
        self.pid = os.getpid()

    def guard(self):
        require(os.getpid() == self.pid and not self.lease.closed, "inherited/closed checker denied")
        path, fd = canonical(LOCK).stat(), os.fstat(self.lease.fileno())
        require(stat.S_ISREG(path.st_mode) and path.st_nlink == 1 and generation(path) == generation(fd),
            "held runtime lease FD/path differs")
        memory = {line.split(":")[0]: int(line.split()[1])*1024
            for line in Path("/proc/self/status").read_text().splitlines() if line.startswith(("VmRSS:", "VmHWM:"))}
        self.maximum_HWM = max(self.maximum_HWM, memory["VmHWM"])
        require(self.maximum_HWM <= RSS and time.monotonic()-self.started < DEADLINE, "sampled checker memory/deadline exceeded")
        fs = os.statvfs(ROOT)
        require(fs.f_bavail*fs.f_frsize >= FLOOR+RESERVE, "unchanged floor plus reserve unavailable")

    def read(self, path, expected=None, metadata=False, maximum=1024**3):
        path, digest, chunks = canonical(path), hashlib.sha256(), []
        before = path.stat()
        require(stat.S_ISREG(before.st_mode) and before.st_nlink == 1
            and before.st_size <= (min(MAX_METADATA, maximum) if metadata else maximum), "bounded regular single-link evidence required")
        require(expected is None or sha(expected), "exact lowercase RAW seal required")
        if str(path) in self.files:
            require(generation(before) == self.files[str(path)]["generation"], "admitted evidence changed before reread")
        with path.open("rb") as stream:
            opened = os.fstat(stream.fileno())
            for block in iter(lambda: stream.read(BLOCK), b""):
                self.guard()
                digest.update(block)
                if metadata:
                    chunks.append(block)
            closed = os.fstat(stream.fileno())
        observed = generation(before)
        require(observed == generation(opened) == generation(closed) == generation(path.stat()), "evidence generation changed during read")
        pin = digest.hexdigest()
        require(expected is None or pin == expected, "evidence RAW seal differs: "+str(path))
        witness = dict(sha256=pin, bytes=before.st_size, generation=observed)
        if str(path) in self.files:
            require(same(witness, self.files[str(path)]), "evidence changed between independent reads")
        self.files[str(path)] = witness
        return b"".join(chunks) if metadata else pin

    def close_seals(self):
        for path, witness in list(self.files.items()):
            self.read(path, witness["sha256"])
        self.guard()


def shard_plan():
    return [dict(ordinal=index, declared_steps=STEPS[index*8:(index+1)*8], first_step=index*8+1,
        last_step=(index+1)*8, expected_batches=8) for index in range(2)]


BASELINE_FIELDS = {"step", "physical_slot_indices", "shared_crop_boxes", "actual_paired_TRUE_bits", "RGB_sha256", "target_sha256"}


def box(value):
    require(type(value) is list and len(value) == 4 and all(integer(v, 0, 640) for v in value), "strict crop box required")
    top, left, height, width = value
    require(height > 0 and width > 0 and top+height <= 360 and left+width <= 640, "crop exceeds native bounds")


def baseline(row, step):
    require(type(row) is dict and set(row) == BASELINE_FIELDS and same(row["step"], step)
        and integer(step, 1, 20000), "exact original six-field step required")
    require(type(row["physical_slot_indices"]) is list and len(row["physical_slot_indices"]) == 8
        and all(integer(v, 0, 1208) for v in row["physical_slot_indices"]), "eight exact TRAIN slots required")
    require(type(row["shared_crop_boxes"]) is list and len(row["shared_crop_boxes"]) == 8, "eight original crop boxes required")
    for value in row["shared_crop_boxes"]:
        box(value)
    require(same(row["actual_paired_TRUE_bits"], [0]*8) and sha(row["RGB_sha256"]) and sha(row["target_sha256"]),
        "FALSE-only bits and original RGB/GT SHA required")


def original_selected(audit):
    selected, count = [], 0
    path = canonical(RUN/"control/exposures.jsonl")
    with path.open("rb") as stream:
        while True:
            raw = stream.readline(4097)
            if not raw:
                break
            audit.guard()
            require(len(raw) <= 4096 and raw.endswith(b"\n"), "bounded full original metadata line required")
            count += 1
            require(count <= 20000, "extra original metadata row")
            row = decode(raw)
            baseline(row, count)
            if count <= 16:
                selected.append(dict(step=count, RAW_line_sha256=hashlib.sha256(raw).hexdigest(), baseline=row))
    require(count == 20000 and same([row["step"] for row in selected], STEPS), "full original20k metadata tail required")
    audit.read(path, INPUTS[str(path)])
    return selected


def native_slots(value, row):
    require(type(value) is list and len(value) == 8, "eight native hash witnesses required")
    for position, item in enumerate(value):
        require(type(item) is dict and set(item) == {"position", "slot", "native_RGB_sha256", "native_target_sha256",
            "reader_provenance_authenticated_by_worker"} and same(item["position"], position)
            and same(item["slot"], row["physical_slot_indices"][position])
            and sha(item["native_RGB_sha256"]) and sha(item["native_target_sha256"])
            and item["reader_provenance_authenticated_by_worker"] is False, "native position/slot/SHA binding differs")


def candidate_worker(value, original):
    fields = {"schema", "step", "journal", "native_slots", "actual_baseline_RGB_and_GT_hashes_reproduced",
        "candidate_hashes_computed_from_actual_owned_arrays", "same_box_for_actual_RGB_and_GT",
        "source_and_baseline_RAW_authenticated_by_worker", "training_steps", "NN"}
    require(type(value) is dict and set(value) == fields and value["schema"] == "portable_e2e.map_crop_batch_worker.20261008.v3.batch"
        and same(value["step"], original["step"]), "exact worker V3 candidate chronology required")
    row = value["journal"]
    jfields = {"schema", "policy_source_sha256", "baseline_exposure", "selected_crop_boxes", "full_GT_sign_pixels",
        "original_NEAREST_sign_pixels", "protected", "rescued", "candidate_RGB_sha256", "candidate_target_sha256"}
    require(type(row) is dict and set(row) == jfields and row["schema"] == "portable_e2e.map_crop_preservation_journal.20261008.v1"
        and row["policy_source_sha256"] == POLICY_PIN and same(row["baseline_exposure"], original), "candidate original/policy join differs")
    require(all(type(row[k]) is list and len(row[k]) == 8 for k in
        ("selected_crop_boxes", "full_GT_sign_pixels", "original_NEAREST_sign_pixels", "protected", "rescued")), "eight policy witnesses required")
    for position, proposal in enumerate(original["shared_crop_boxes"]):
        full, cropped = row["full_GT_sign_pixels"][position], row["original_NEAREST_sign_pixels"][position]
        require(integer(full, 0, 230400) and integer(cropped, 0, 230400), "strict GT support counts required")
        protected = ((original["step"]-1)*8+position) % 2 == 0
        rescued = protected and full > 0 and cropped == 0
        chosen = [0, 0, 360, 640] if rescued else proposal
        require(row["protected"][position] is protected and row["rescued"][position] is rescued
            and same(row["selected_crop_boxes"][position], chosen), "crop support/parity/selected box differs")
        box(row["selected_crop_boxes"][position])
    require(sha(row["candidate_RGB_sha256"]) and sha(row["candidate_target_sha256"]), "candidate byte SHA invalid")
    if not any(row["rescued"]):
        require(row["candidate_RGB_sha256"] == original["RGB_sha256"] and row["candidate_target_sha256"] == original["target_sha256"],
            "no-rescue candidate must retain historical batch SHA")
    native_slots(value["native_slots"], original)
    require(all(value[k] is True for k in ("actual_baseline_RGB_and_GT_hashes_reproduced",
        "candidate_hashes_computed_from_actual_owned_arrays", "same_box_for_actual_RGB_and_GT"))
        and value["source_and_baseline_RAW_authenticated_by_worker"] is False
        and same(value["training_steps"], 0) and same(value["NN"], 0), "candidate worker scope differs")
    return row


def descriptor(value, shape, dtype, maximum):
    fields = {"shape", "dtype", "nbytes", "owned", "C_contiguous", "finite", "minimum", "maximum", "sha256"}
    amount = math.prod(shape)*({"uint8": 1, "float32": 4, "int64": 8}[dtype])
    require(type(value) is dict and set(value) == fields and same(value["shape"], shape)
        and value["dtype"] == dtype and same(value["nbytes"], amount)
        and all(value[k] is True for k in ("owned", "C_contiguous", "finite")) and sha(value["sha256"]),
        "exact owned C-order native/host descriptor required")
    require(all(type(value[k]) in ((float,) if dtype == "float32" else (int,)) and math.isfinite(value[k])
        for k in ("minimum", "maximum")) and 0 <= value["minimum"] <= value["maximum"] <= maximum,
        "finite typed native/host descriptor range required")


def consumer_receipt(value, host):
    require(type(value) is dict and set(value) == {"consumer", "accepted", "host_RGB_sha256", "host_target_sha256"}
        and value["consumer"] == CONSUMER and value["accepted"] is True
        and value["host_RGB_sha256"] == host["RGB"]["sha256"] and value["host_target_sha256"] == host["target"]["sha256"],
        "actual dry consumer/host SHA receipt differs")


def host_record(value, original, mode, native_row):
    require(type(value) is dict and set(value) == {"schema", "arm", "step", "adapter_receipt",
        "actual_consumer_receipt", "host_value_proof", "chronology"} and value["schema"] == PRODUCER_SCHEMA+".host_receipt"
        and value["arm"] == mode and same(value["step"], original["step"])
        and value["chronology"] == "CONSUMER_CHECKED_METADATA_WRITTEN_BEFORE_ADAPTER_ACK", "pre-ACK host metadata chronology differs")
    receipt = value["adapter_receipt"]
    fields = {"schema", "mode", "step", "provided_baseline", "provided_baseline_sha256", "control_exposure",
        "candidate_worker_receipt", "native_slots", "native_arrays", "host_arrays", "scope"}
    require(type(receipt) is dict and set(receipt) == fields and receipt["schema"] == ADAPTER_SCHEMA+".batch"
        and receipt["mode"] == mode and same(receipt["step"], original["step"])
        and same(receipt["provided_baseline"], original) and receipt["provided_baseline_sha256"] == metadata_sha(original)
        and same(receipt["scope"], ADAPTER_SCOPE), "exact adapter original/mode/scope receipt differs")
    native_slots(receipt["native_slots"], original)
    if mode == ARMS[0]:
        require(same(receipt["control_exposure"], original) and same(native_row, original)
            and receipt["candidate_worker_receipt"] is None, "control must retain exact original six fields, not a candidate journal")
        rgb_pin, gt_pin, rescued = original["RGB_sha256"], original["target_sha256"], 0
    else:
        require(receipt["control_exposure"] is None and same(receipt["candidate_worker_receipt"], native_row),
            "candidate journal cannot be relabeled control")
        candidate = candidate_worker(native_row, original)
        require(same(receipt["native_slots"], native_row["native_slots"]), "adapter/worker actual native witnesses differ")
        rgb_pin, gt_pin, rescued = candidate["candidate_RGB_sha256"], candidate["candidate_target_sha256"], sum(candidate["rescued"])
    native, host = receipt["native_arrays"], receipt["host_arrays"]
    require(type(native) is type(host) is dict and set(native) == set(host) == {"RGB", "target"}, "RGB/GT descriptor pairs required")
    descriptor(native["RGB"], [8, 360, 640, 3], "uint8", 255)
    descriptor(native["target"], [8, 360, 640], "uint8", 6)
    descriptor(host["RGB"], [8, 3, 360, 640], "float32", 1)
    descriptor(host["target"], [8, 360, 640], "int64", 6)
    require(native["RGB"]["sha256"] == rgb_pin and native["target"]["sha256"] == gt_pin
        and same(native["target"]["minimum"], host["target"]["minimum"])
        and same(native["target"]["maximum"], host["target"]["maximum"]), "native byte/host GT descriptor join differs")
    consumer_receipt(value["actual_consumer_receipt"], host)
    proof = value["host_value_proof"]
    expected = dict(all_actual_host_values_compared=True, finite_RGB_in_unit_interval=True, GT_labels_0_to_6=True,
        normalized_RGB_sha256=host["RGB"]["sha256"], int64_target_sha256=host["target"]["sha256"],
        Torch_imported=False, Torch_numeric_equivalence_proven=False, NN=0, optimizer_updates=0)
    require(same(proof, expected), "NumPy host reference/consumer scope proof differs")
    return dict(step=original["step"], mode=mode, native_receipts=8, rescued_slots=rescued,
        host_RGB_sha256=host["RGB"]["sha256"], host_target_sha256=host["target"]["sha256"])


def contract():
    return dict(declared_steps=STEPS, expected_batches_per_arm=16, expected_slot_exposures_per_arm=128,
        arms=list(ARMS), fixed_sequential_arm_order=list(ARMS), shards_per_arm=shard_plan(), reader_revision="V3",
        same_reader_source_and_cache_enabled=True, checkpoint_max_states=48, checkpoint_state_account_bytes=256*1024,
        checkpoint_cache_account_limit_bytes=12*1024**2, checkpoint_stride_bytes=32*1024**2,
        owner_checkpoint_guard_stride=16, cache_accounting_is_reservation_not_exact_native_allocation=True,
        host_RGB_shape=[8, 3, 360, 640], host_RGB_dtype="float32", RGB_divisor=255,
        host_target_shape=[8, 360, 640], host_target_dtype="int64", labels=list(range(7)),
        normalization_proven="NumPy owned C-order float32 NCHW /np.float32(255); int64 GT",
        consumer=CONSUMER, consumer_checks_all_actual_host_values=True,
        ACK_after_consumer_and_both_metadata_appends=True, written_not_automatically_consumed=True,
        original_control_exposure_has_exact_six_fields=True, candidate_worker_policy_journal_distinct=True,
        allowed_original_members=["rgb.npy", "target.npy"], allowed_paired_members=["rgb_false.npy", "target.npy"],
        TRUE_DEV_HOLDOUT_TEST_weights_deniedRAW_decode_authorized=False,
        original20k_batches_outside_declaration_per_arm=19984,
        prospective_full20k_sharding=dict(metadata_only=True, batches_per_shard=8, shard_count=2500,
            numeric_batches_executed=0, full20k_execution_authorized=False),
        CPU_affinity=[3], threads=1, sampled_VmHWM_cap_bytes=RSS, deadline_seconds=DEADLINE,
        floor_bytes=FLOOR, reserve_bytes=RESERVE, maximum_combined_PLAN_OUTPUT_bytes=2*1024**2,
        closing_bytes=512*1024, native_journal_maximum_bytes_per_shard=NATIVE_LIMIT,
        host_receipt_maximum_bytes_per_shard=HOST_LIMIT, PNG_created=False, same_source_pins_before_after=True,
        selected_scientific_providers=6, sampled_guards_not_OS_containment=True, old_authority_reused=False,
        Torch_imported=False, Torch_numeric_equivalence_proven=False, trainer_connected=False,
        multiworker_implemented=False, optimizer_updates=0, NN=0, GPU=0,
        main_approved=0, sub_approved=0, model_performance_gain_proven=False)


def authority(plan_pin):
    return dict(schema=PRODUCER_SCHEMA+".ROOT_authority", plan_path=str(PLAN_DIR/"ROOT_plan.json"), plan_sha256=plan_pin,
        serial_FIRST16_original_and_FALSE_RGB_GT_host_verification_authorized=True,
        original_control_and_sign_preserving_candidate_authorized=True,
        nontraining_NumPy_consumer_and_sharded_metadata_authorized=True,
        training_authorized=False, NN_authorized=False, Torch_authorized=False,
        TRUE_DEV_HOLDOUT_TEST_weights_deniedRAW_decode_authorized=False,
        old_target_only_or_cache_AB_authority_reused=False, output=str(OUT), contract=contract())


def pipeline(value):
    expected = dict(expected_batches=16, produced_batches=16, consumer_checked_batches=16,
        native_rows_written=16, host_rows_written=16, acknowledged_batches=16,
        PASS_batches=16, FAIL_batches=0, UNEXECUTED_batches=0, failed_step=None,
        original20k_batches_outside_this_declaration=19984)
    require(same(value, expected), "produced/consumer/written/ACK full16 pipeline facts differ")


def acknowledgement(value, host_row, step):
    ack = host_row["actual_consumer_receipt"]
    inner = dict(step=step, consumer_receipt=ack, native_and_host_buffers_rehashed=True, consumer_execution_authenticated=False)
    expected = dict(step=step, adapter_ACK_received=True, consumer_receipt=ack, adapter_acknowledgement=inner)
    require(same(value, expected), "positive library ACK is not joined to the pre-ACK actual consumer record")
    return inner


def adapter_summary(value, mode, selected, acknowledgements):
    require(type(value) is dict and value["schema"] == ADAPTER_SCHEMA+".summary" and value["mode"] == mode
        and same(value["declared_steps"], STEPS) and value["closed"] is True
        and value["status"] == "COMPLETE_ACKNOWLEDGED_INPUTS_NOT_TRAINING_OR_MODEL_APPROVAL"
        and same(value["scope"], ADAPTER_SCOPE), "closed acknowledged NumPy-only adapter scope differs")
    counts = dict(expected_batches=16, PASS_batches=16, FAIL_batches=0, UNEXECUTED_batches=0,
        produced_batches=16, production_attempts=16, production_FAIL_batches=0, production_UNEXECUTED_batches=0,
        consumption_attempts=16, consumption_attempted_batches=16, consumption_PASS_batches=16,
        consumption_FAIL_batches=0, consumption_UNEXECUTED_batches=0, actual_native_read_attempts=128,
        original20000_batches_outside_declaration=19984)
    require(all(same(value[k], expected) for k, expected in counts.items())
        and value["pending_step"] is None and value["failure"] is None and value["failed_tail"] is None
        and same(value["cleanup_errors"], []) and same(value["reader_closing_observations"], [])
        and same(value["acknowledgements"], acknowledgements)
        and value["provided_rows_sha256"] == metadata_sha([item["baseline"] for item in selected])
        and integer(value["owner_PID"], 1) and integer(value["owner_thread_ident"], 1),
        "adapter produced/consumed/actual128-read denominator or ACK join differs")


def reader_stats(value):
    integer_fields = {"current_checkpoint_states", "maximum_checkpoint_states_observed", "max_checkpoint_states",
        "checkpoint_state_account_bytes", "cache_accounted_bytes", "maximum_cache_accounted_bytes", "cache_account_limit_bytes",
        "owner_checkpoint_calls", "archive_generation_guard_calls", "owner_checkpoint_guard_stride", "checkpoint_hits",
        "checkpoint_misses", "checkpoint_creations", "checkpoint_evictions", "total_prefix_body_bytes_inflated",
        "total_selected_payload_bytes", "native_row_cache_payload_bytes", "checkpoint_stride_bytes"}
    require(type(value) is dict and set(value) == integer_fields | {"checkpoint_cache_enabled",
        "state_accounting_is_reservation_not_exact_native_allocation"} and all(integer(value[k]) for k in integer_fields),
        "exact reader V3 typed snapshot required")
    require(value["checkpoint_cache_enabled"] is True and value["state_accounting_is_reservation_not_exact_native_allocation"] is True
        and value["current_checkpoint_states"] == value["cache_accounted_bytes"] == value["native_row_cache_payload_bytes"] == 0
        and value["max_checkpoint_states"] == 48 and value["maximum_checkpoint_states_observed"] <= 48
        and value["checkpoint_state_account_bytes"] == 256*1024 and value["cache_account_limit_bytes"] == 12*1024**2
        and value["maximum_cache_accounted_bytes"] == value["maximum_checkpoint_states_observed"]*256*1024 <= 12*1024**2
        and value["checkpoint_stride_bytes"] == 32*1024**2 and value["owner_checkpoint_guard_stride"] == 16
        and value["total_selected_payload_bytes"] == 128*921600, "closed reader cache/accounted128 payload bounds differ")


def read_jsonl(audit, path, maximum, expected_steps, line_limit):
    raw = audit.read(path, metadata=True, maximum=maximum)
    lines = raw.splitlines(keepends=True)
    require(len(lines) == len(expected_steps) and all(line.endswith(b"\n") and len(line) <= line_limit for line in lines),
        "exact complete bounded chronological shard rows required")
    rows = [decode(line) for line in lines]
    require(all(type(row) is dict and same(row.get("step"), step) for row, step in zip(rows, expected_steps)),
        "shard step chronology differs")
    return rows


def shard_snapshot(value, path, expected, audit, *, host=False, candidate=False):
    witness = audit.files[str(path)]
    require(type(value) is dict and value["path"] == str(path) and same(value["rows_written"], 8)
        and same(value["bytes_written"], witness["bytes"]), "saved shard path/rows/byte count differs")
    if candidate:
        require(value["closing_check"] == "MATCHED" and value["closing_RAW_sha256"] == witness["sha256"],
            "candidate original writer closing RAW witness differs")
    else:
        require(same(value["declared_steps"], expected["declared_steps"])
            and same(value["maximum_bytes"], HOST_LIMIT if host else NATIVE_LIMIT) and same(value["closing_errors"], []),
            "scalar shard steps/cap/closing errors differ")
        close = value["closing_check"]
        require(type(close) is dict and close["status"] == "MATCHED" and same(close["bytes_read"], witness["bytes"])
            and close["sha256"] == close["written_sha256"] == witness["sha256"]
            and same(close["bytes"], witness["bytes"]) and same(close["generation"], witness["generation"]),
            "scalar shard closing bytes/generation/SHA differ")


def inspect(audit, plan_pin, grant_pin, source_pin, test_pin):
    audit.read(Path(__file__).resolve(), source_pin)
    audit.read(TEST_SOURCE, test_pin)
    raw_plan = audit.read(PLAN_DIR/"ROOT_plan.json", plan_pin, True)
    raw_grant = audit.read(PLAN_DIR/"ROOT_authority.json", grant_pin, True)
    require(audit.read(PLAN_DIR/"candidate_plan.json", plan_pin, True) == raw_plan
        and audit.read(OUT/"ROOT_plan.json", plan_pin, True) == raw_plan
        and audit.read(OUT/"ROOT_authority.json", grant_pin, True) == raw_grant, "ROOT original RAW copies differ")
    plan, grant = decode(raw_plan), decode(raw_grant)
    require(plan["schema"] == PRODUCER_SCHEMA and same(plan["contract"], contract())
        and same(plan["input_sha256"], INPUTS) and plan["output"] == str(OUT)
        and same(grant, authority(plan_pin)), "exact fresh serial-input ROOT contract/grant differs")
    sources, providers = plan["source_sha256"], plan["environment"]["selected_provider_sha256"]
    require(type(sources) is dict and set(sources) == {str(ROOT/name) for name in SOURCE_NAMES}
        and all(sha(pin) for pin in sources.values()) and type(providers) is dict and len(providers) == 6
        and all(type(path) is str and sha(pin) for path, pin in providers.items())
        and same(plan["source_archive_count"], len(set(sources.values()))), "exact imported source14/provider6/archive membership differs")
    require(sources[str(ROOT/"learning/portable_e2e/map_crop_batch_worker_20261008_v3.py")]
        == "882f3a5fdd2eadfb1f48cabd8b87a3b3b9d9c03d0572dcd810ae326e152be456"
        and sources[str(ROOT/"tools/e2e/map_train_sign_preserving_crop_policy_20261008_v1.py")] == POLICY_PIN,
        "unchanged worker V3/policy source pins differ")
    archives = {str(PLAN_DIR/"sources"/(pin+".py")): pin for pin in set(sources.values())}
    seals = {}
    for group in (sources, INPUTS, providers, archives):
        for path, pin in group.items():
            require(path not in seals or seals[path] == pin, "overlapping seal pin conflict")
            seals[path] = pin
    for path, pin in seals.items():
        audit.read(path, pin)
    selected = original_selected(audit)
    require(same(selected, plan["selected_baseline_rows"]), "original20k/FIRST16 RAW metadata join differs")
    result = decode(audit.read(OUT/"result.json", metadata=True))
    require(result["schema"] == PRODUCER_SCHEMA+".result" and result["ROOT_plan_sha256"] == plan_pin
        and result["ROOT_authority_sha256"] == grant_pin and same(result["contract"], contract()), "result ROOT contract binding differs")
    # HH_261008 - Failed/incomplete originals remain intact and cannot be relabeled as successful host input.
    require(result["status"] == "COMPLETE_SERIAL_HOST_INPUT_NOT_TRAINING" and result["failure"] is None
        and result["full_failed_tail"] is None and same(result["secondary_closing_errors"], []),
        "original producer failed/incomplete; retain its whole declared denominator and evidence")
    require(all(same(result[k], 0) for k in ("NN", "optimizer_updates", "main_approved", "sub_approved"))
        and all(result[k] is False for k in ("Torch_imported", "Torch_numeric_equivalence_proven", "trainer_connected",
            "multiworker_implemented", "model_performance_gain_proven")), "host boundary cannot claim training/Torch/model approval")
    memory = result["sampled_closing_memory"]
    require(type(memory) is dict and all(integer(memory[k], 1, RSS) for k in ("VmRSS", "VmHWM"))
        and integer(result["sampled_maximum_VmHWM_bytes"], memory["VmHWM"], RSS) and seconds(result["elapsed_seconds"]),
        "sampled producer memory/deadline witness differs")
    closing = result["closing_seal_observations"]
    require(type(closing) is list and len(closing) == len(seals)
        and all(type(row) is dict and row.get("path") in seals for row in closing)
        and len({row["path"] for row in closing}) == len(closing)
        and all(same(row, dict(path=row["path"], expected_sha256=seals[row["path"]], observed_sha256=seals[row["path"]], status="MATCHED"))
            for row in closing), "producer named closing byte seals differ or were not checked")
    arms, checked, native_by_slot = result["arms"], {}, {}
    require(type(arms) is dict and set(arms) == set(ARMS), "exact two crop-mode arms required")
    for mode in ARMS:
        arm = arms[mode]
        pipeline(arm["pipeline"])
        require(type(arm["shards"]) is list and len(arm["shards"]) == 2
            and type(arm["acknowledgements"]) is list and len(arm["acknowledgements"]) == 16,
            "two fixed shards and sixteen actual ACK witnesses required")
        mode_checked, mode_ack = [], []
        for record, expected in zip(arm["shards"], shard_plan()):
            require(type(record) is dict and set(record) == set(expected) | {"native", "host"}
                and same({k: record[k] for k in expected}, expected), "fixed 1–8/9–16 shard hierarchy differs")
            number = expected["ordinal"]
            native_path = OUT/mode/("shard_%04d_exposures.jsonl" % number)
            host_path = OUT/mode/("shard_%04d_host_receipts.jsonl" % number)
            native_rows = read_jsonl(audit, native_path, NATIVE_LIMIT, expected["declared_steps"], 8*1024)
            host_rows = read_jsonl(audit, host_path, HOST_LIMIT, expected["declared_steps"], 16*1024)
            shard_snapshot(record["native"], native_path, expected, audit, candidate=mode == ARMS[1])
            shard_snapshot(record["host"], host_path, expected, audit, host=True)
            for native_row, host_row, step in zip(native_rows, host_rows, expected["declared_steps"]):
                original = selected[step-1]["baseline"]
                if mode == ARMS[0]:
                    baseline(native_row, step)
                checked_row = host_record(host_row, original, mode, native_row)
                inner = acknowledgement(arm["acknowledgements"][step-1], host_row, step)
                mode_checked.append(checked_row)
                mode_ack.append(inner)
                for native in host_row["adapter_receipt"]["native_slots"]:
                    pair = [native["native_RGB_sha256"], native["native_target_sha256"]]
                    require(native["slot"] not in native_by_slot or same(native_by_slot[native["slot"]], pair),
                        "same native slot reports inconsistent actual source SHA across modes/exposures")
                    native_by_slot[native["slot"]] = pair
        adapter_summary(arm["adapter"], mode, selected, mode_ack)
        require(same(arm["reader_closing_errors"], []), "reader retained closing errors")
        reader_stats(arm["reader_stats_after_close"])
        checked[mode] = mode_checked
    audit.close_seals()
    return dict(status="COMPLETE_INDEPENDENT_SAVED_METADATA_AND_BYTE_SEALS_ONLY", declared_batches_per_arm=16,
        PASS_batches_per_arm=16, FAIL_batches_per_arm=0, UNEXECUTED_batches_per_arm=0,
        produced_and_consumed_batches_per_arm=16, native_receipts_per_arm=128, native_receipts_both_arms=256,
        fixed_native_shards=4, fixed_host_receipt_shards=4, original20k_metadata_rows=20000,
        original20k_numeric_batches_independently_replayed=0, original20k_batches_outside_declaration_per_arm=19984,
        checked_batches=checked, candidate_rescued_slots=sum(row["rescued_slots"] for row in checked[ARMS[1]]),
        producer_native_and_host_array_claims_checked_as_metadata=True,
        native_or_host_array_SHA_independently_recomputed=False, NumPy_conversion_independently_replayed=False,
        actual_consumer_execution_independently_authenticated=False, Torch_numeric_equivalence_proven=False,
        native_decoder_exact_allocation_independently_measured=False, source_files=14,
        source_archives=len(archives), original_inputs=7, selected_scientific_providers=6,
        original_result_status=result["status"], original_result_failure=result["failure"], original_closing_errors=result["secondary_closing_errors"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    for name in ("plan-sha256", "authority-sha256", "script-sha256", "test-source-sha256"):
        parser.add_argument("--"+name, required=True)
    args = parser.parse_args()
    pins = [args.plan_sha256, args.authority_sha256, args.script_sha256, args.test_source_sha256]
    require(args.execute and all(sha(pin) for pin in pins), "explicit ROOT execution with external RAW pins required")
    require(sys.flags.dont_write_bytecode and sys.flags.optimize == 0, "clean non-bytecode checker required")
    canonical(RECEIPT, False)
    require(not RECEIPT.exists(), "fresh independent receipt required; no overwrite/retry")
    with canonical(LOCK).open("rb") as lease:
        fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        audit = Audit(lease)
        try:
            try:
                audit.guard()
                outcome = inspect(audit, *pins)
            except Exception as error:
                outcome = dict(status="FAILED_INDEPENDENT_SAVED_CHECK_RETAINED", error_type=type(error).__name__,
                    error=str(error)[:4096], original_evidence_changed_or_deleted_by_checker=False)
            receipt = dict(schema=SCHEMA, provenance="HH_261008 - Independently seal saved serial metadata; no second array replay or function approval.",
                ROOT_plan_RAW_sha256=pins[0], ROOT_authority_RAW_sha256=pins[1], checker_source_sha256=pins[2],
                scalar_test_source_sha256=pins[3], result=outcome, independently_read_file_seals=audit.files,
                sampled_maximum_VmHWM_bytes=audit.maximum_HWM, elapsed_seconds=time.monotonic()-audit.started,
                sampled_guards_not_OS_containment=True, array_decodes=0, host_normalization_second_replays=0,
                NN=0, optimizer_updates=0, training_steps=0, GPU=0, SSH=0, CARLA=0,
                Torch_imported=False, trainer_connected=False, formal_main_approved=0, formal_sub_approved=0)
            raw = encoded(receipt)
            require(len(raw) <= MAX_METADATA, "bounded fresh independent metadata receipt required")
            with RECEIPT.open("xb") as output:
                require(output.write(raw) == len(raw), "partial independent receipt write")
                output.flush()
            print(encoded(dict(status=outcome["status"], receipt=str(RECEIPT), bytes=len(raw))).decode(), end="")
            return 0 if outcome["status"].startswith("COMPLETE_") else 2
        finally:
            fcntl.flock(lease, fcntl.LOCK_UN)


if __name__ == "__main__":
    raise SystemExit(main())
