"""HH_261012 - Independently join stored CPU tensor-boundary metadata and RAW seals only.

ROOT must read source/tests and authorize --execute using four external RAW pins.
Standard library only: no producer imports, array/member decoding, Torch import,
trainer-reference replay, model/loss/backward or external consumer authentication.
Whole compressed-file hashes do not decode TRUE or any other array. Producer
numeric measurements remain explicitly producer measurements. Preserve every
original and create one fresh success/failure receipt outside the producer OUT.
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
NAME = "map_torch_input_boundary_v1"
PRODUCER = "portable_e2e.map_torch_input_boundary.20261012.v1"
SERIAL = "portable_e2e.map_serial_training_input_adapter.20261008.v1"
BRIDGE = "portable_e2e.map_torch_cpu_input_bridge.20261012.v1"
SCHEMA = "portable_e2e.map_torch_input_boundary_saved_check.20261012.v1"
PLAN = ROOT/"storage/experiments/plans/2026-10-12"/NAME
OUT = ROOT/"storage/experiments/diagnostics/2026-10-12"/NAME
RECEIPT = OUT.parent/(NAME+"_independent_v1.json")
TEST = ROOT/"learning/tests/test_check_map_torch_input_boundary_saved_20261012_v1.py"
LOCK = ROOT/"storage/assets/locks/autoware_e2e_runtime.lock"
DATA = ROOT/"storage/experiments/training/2026-10-02/map_paired_balanced_20261002_v1"
RUN = ROOT/"storage/experiments/training/2026-10-02/map_native_fusion_pair_v3"
STEPS = list(range(1, 17))
ARMS = ("ORIGINAL_CROP_CONTROL", "SIGN_PRESERVING_CANDIDATE")
CONSUMER = "map_torch_cpu_value_consumer_v1"
REFERENCE = "tools/e2e/train_map_native_fusion_pair_20261002.py"
REFERENCE_PIN = "3918b5a529f08e17cfd294d24a21fbd545ee0adccc0e1ac3adaf79c60eb3f841"
POLICY_PIN = "566b3e2afb7a0c879071c806c0cbfef6d31b8e974df99423339b7bf48cd6209f"
BLOCK, METADATA, NATIVE_LIMIT, TENSOR_LIMIT, LINE_LIMIT = 64*1024, 512*1024, 64*1024, 128*1024, 8*1024
RSS, DEADLINE, FLOOR, RESERVE = 512*1024**2, 900, 2*1024**3, 64*1024**2
SOURCE_NAMES = {
    "tools/e2e/verify_map_crop_rescue_reader_ab_20261008_v1.py", "tools/workspace_python.py",
    "tools/canonical_workspace.py", "learning/portable_e2e/__init__.py", "learning/portable_e2e/contract.py",
    "learning/portable_e2e/map_native_pair_reader_20261008_v3.py",
    "learning/portable_e2e/map_crop_batch_worker_20261008_v3.py",
    "learning/portable_e2e/map_crop_preservation_journal_20261008_v1.py",
    "tools/e2e/map_train_sign_preserving_crop_policy_20261008_v1.py",
    "tools/e2e/map_appearance_matched_contract_20261001.py",
    "learning/portable_e2e/map_serial_training_input_adapter_20261008_v1.py",
    "learning/tests/test_map_serial_training_input_adapter_20261008_v1.py",
    "tools/e2e/verify_map_serial_training_input_20261008_v1.py",
    "learning/tests/test_verify_map_serial_training_input_20261008_v1.py",
    "learning/portable_e2e/map_torch_cpu_input_bridge_20261012_v1.py",
    "learning/tests/test_map_torch_cpu_input_bridge_20261012_v1.py", REFERENCE,
    "tools/e2e/verify_map_torch_input_boundary_20261012_v1.py",
    "learning/tests/test_verify_map_torch_input_boundary_20261012_v1.py",
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
SERIAL_SCOPE = dict(worker_count=0, epochs_supported=False, multiprocess_supported=False,
    ROOT_or_data_authenticated_by_adapter=False, injected_crop_source_authenticated_by_adapter=False,
    TRAIN_admitted_by_adapter=False, consumer_execution_authenticated_by_adapter=False,
    Torch_or_GPU_numerical_equivalence_claimed=False, filesystem_written_by_adapter=False,
    public_arrays_immutable=False, owner_thread_bound=True, reentrant_calls_supported=False,
    sampled_guards_are_OS_containment=False, optimizer_steps=0, NN=0)
BRIDGE_SCOPE = dict(ROOT_authenticated_by_bridge=False, data_authenticated_by_bridge=False,
    provider_authenticated_by_bridge=False, TRAIN_admitted_by_bridge=False,
    consumer_execution_authenticated_by_bridge=False, frozen_trainer_reference_authenticated_by_bridge=False,
    frozen_trainer_CPU_recipe_parity_claimed=False, GPU_numeric_equivalence_claimed=False,
    public_buffers_immutable=False, unique_storage_against_arbitrary_external_aliases_claimed=False,
    sampled_guards_are_OS_containment=False, source_buffer_copies_retained=0,
    source_normalization_performed=False, lifecycle_or_ACK_implemented=False,
    CUDA_API_invoked_by_bridge=False, filesystem_written_by_bridge=False,
    worker_count=0, model_forwards=0, backward=0, optimizer_updates=0, epochs=0, NN=0, GPU=0)


def require(value, message):
    if not value:
        raise ValueError(message)


def integer(value, low=0, high=2**63-1):
    return type(value) is int and low <= value <= high


def sha(value):
    return type(value) is str and len(value) == 64 and all(c in "0123456789abcdef" for c in value)


def same(left, right):
    if type(left) is not type(right):
        return False
    if type(left) is dict:
        return set(left) == set(right) and all(same(left[key], right[key]) for key in left)
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
    def floating(value):
        result = float(value)
        require(math.isfinite(result), "overflowing JSON float")
        return result
    return json.loads(raw, object_pairs_hook=pairs, parse_float=floating,
        parse_constant=lambda _: require(False, "nonfinite JSON token"))


def encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)+"\n").encode()


def metadata_sha(value):
    return hashlib.sha256(encoded(value)[:-1]).hexdigest()


def canonical(path, exists=True):
    path = Path(path)
    require(path.is_absolute() and ".." not in path.parts and path == path.resolve(strict=exists)
        and not any(item.is_symlink() for item in (path, *path.parents)), "canonical nonsymlink evidence required")
    return path


def generation(info):
    return [getattr(info, key) for key in ("st_mode", "st_uid", "st_gid", "st_nlink", "st_dev", "st_ino",
        "st_size", "st_mtime_ns", "st_ctime_ns")]


class Audit:
    """HH_261012 - Hash bounded RAW files under an own retained runtime lease; never inflate archives."""
    def __init__(self, lease):
        self.lease, self.lease_generation = lease, generation(os.fstat(lease.fileno()))
        self.pid, self.started, self.files, self.maximum_HWM, self.aliases = os.getpid(), time.monotonic(), {}, 0, {}

    def guard(self):
        require(os.getpid() == self.pid and not self.lease.closed, "foreign or closed checker denied")
        info, fd = canonical(LOCK).stat(), os.fstat(self.lease.fileno())
        require(stat.S_ISREG(info.st_mode) and info.st_nlink == fd.st_nlink == 1
            and generation(info) == generation(fd) == self.lease_generation, "retained lease generation changed")
        check_aliases(self.aliases)
        hwm = next(int(line.split()[1])*1024 for line in Path("/proc/self/status").read_text().splitlines()
            if line.startswith("VmHWM:"))
        self.maximum_HWM = max(self.maximum_HWM, hwm)
        fs = os.statvfs(ROOT)
        require(self.maximum_HWM <= RSS and time.monotonic()-self.started < DEADLINE
            and fs.f_bavail*fs.f_frsize >= FLOOR+RESERVE, "unchanged sampled checker budget exceeded")

    def read(self, path, expected=None, metadata=False, maximum=1024**3):
        path, digest, chunks = canonical(path), hashlib.sha256(), []
        before = path.stat()
        require(stat.S_ISREG(before.st_mode) and before.st_nlink == 1
            and before.st_size <= (min(METADATA, maximum) if metadata else maximum), "bounded single-link file required")
        require(expected is None or sha(expected), "exact lowercase external RAW seal required")
        if str(path) in self.files:
            require(same(generation(before), self.files[str(path)]["generation"]), "admitted evidence generation changed")
        with path.open("rb") as stream:
            opened = os.fstat(stream.fileno())
            for part in iter(lambda: stream.read(BLOCK), b""):
                self.guard()
                digest.update(part)
                if metadata:
                    chunks.append(part)
            ended = os.fstat(stream.fileno())
        require(generation(before) == generation(opened) == generation(ended) == generation(path.stat()), "RAW FD/path changed")
        pin = digest.hexdigest()
        require(expected is None or expected == pin, "RAW byte seal differs: "+str(path))
        witness = dict(sha256=pin, bytes=before.st_size, generation=generation(before))
        require(str(path) not in self.files or same(witness, self.files[str(path)]), "RAW evidence changed between reads")
        self.files[str(path)] = witness
        return b"".join(chunks) if metadata else pin

    def close_seals(self):
        for path, witness in list(self.files.items()):
            self.read(path, witness["sha256"])
        self.guard()


def check_aliases(aliases):
    require(type(aliases) is dict, "explicit alias metadata required")
    for alias, target in aliases.items():
        require(type(alias) is type(target) is str and Path(alias).is_absolute()
            and ".." not in Path(alias).parts and str(Path(alias).resolve(strict=True)) == target,
            "selected imported Torch alias changed")
        canonical(target)


def contract():
    return dict(declared_steps=STEPS, arms=list(ARMS), expected_batches_per_arm=16,
        expected_slot_exposures_per_arm=128, same_128_source_exposures_repeated_across_arms=True,
        original20k_batches_outside_declaration_per_arm=19984, reader_revision="V3", cache_enabled=True,
        checkpoint_max_states=48, checkpoint_state_account_bytes=256*1024,
        checkpoint_cache_account_limit_bytes=12*1024**2, checkpoint_stride_bytes=32*1024**2,
        owner_checkpoint_guard_stride=16, cache_accounting_is_reservation_not_exact_native_allocation=True,
        native_RGB_shape=[8,360,640,3], native_target_shape=[8,360,640], native_dtype="uint8",
        tensor_RGB_shape=[8,3,360,640], tensor_RGB_dtype="float32", tensor_target_shape=[8,360,640],
        tensor_target_dtype="int64", tensor_device="cpu", labels=list(range(7)),
        original_trainer_source=str(ROOT/REFERENCE), original_trainer_source_sha256=REFERENCE_PIN,
        original_trainer_imported=False, reference_function="tensors", reference_device="cpu",
        all_reference_RGB_float32_bits_and_GT_values_equal_required=True, NumPy_host_normalization_reapplied=False,
        consumer=CONSUMER, positive_serial_ACK_after_tensor_reverification_and_both_appends=True,
        produced_written_and_ACKed_are_distinct=True, native_files=2, tensor_receipt_files=2,
        declared_steps_in_each_file=STEPS, native_maximum_bytes_per_file=NATIVE_LIMIT,
        tensor_receipt_maximum_bytes_per_file=TENSOR_LIMIT, maximum_line_bytes=LINE_LIMIT,
        allowed_original_members=["rgb.npy","target.npy"], allowed_paired_members=["rgb_false.npy","target.npy"],
        TRUE_DEV_HOLDOUT_TEST_weights_deniedRAW_decode_authorized=False,
        source_count=19, selected_provider_count=12, complete_transitive_provider_closure_claimed=False,
        Torch_library_aliases_explicitly_resolved_and_rechecked=True,
        CPU_affinity=[3], threads=1, sampled_VmHWM_cap_bytes=RSS, deadline_seconds=DEADLINE,
        maximum_combined_PLAN_OUTPUT_bytes=2*1024**2, closing_bytes=METADATA, floor_bytes=FLOOR, reserve_bytes=RESERVE,
        sampled_guards_not_OS_containment=True, same_source_input_provider_seals_before_after=True,
        old_authority_reused=False, Torch_imported=True, Torch_CPU_input_numeric_equivalence_scope="declared FIRST16 only",
        GPU_numeric_equivalence_proven=False, actual_trainer_connected=False,
        same_model_crop_AB_training_implemented=False, full20k_numeric_execution_authorized=False,
        worker_count=0, prefetch_implemented=False, PNG_created=False,
        NN_forward=0, backward=0, optimizer_updates=0, training_steps=0, GPU=0, SSH=0, CARLA=0,
        main_approved=0, sub_approved=0, model_performance_gain_proven=False)


def authority(plan_pin):
    return dict(schema=PRODUCER+".ROOT_authority", plan_path=str(PLAN/"ROOT_plan.json"), plan_sha256=plan_pin,
        FIRST16_original_and_FALSE_RGB_GT_Torch_CPU_input_boundary_authorized=True,
        original_control_and_sign_preserving_candidate_authorized=True,
        Torch_import_and_CPU_tensor_operations_authorized=True, original_trainer_AST_tensors_reference_authorized=True,
        training_authorized=False, NN_forward_authorized=False, backward_authorized=False, GPU_authorized=False,
        TRUE_DEV_HOLDOUT_TEST_weights_deniedRAW_decode_authorized=False,
        old_authority_reused=False, output=str(OUT), contract=contract())


def baseline(row, step):
    require(type(row) is dict and set(row) == {"step","physical_slot_indices","shared_crop_boxes",
        "actual_paired_TRUE_bits","RGB_sha256","target_sha256"} and same(row["step"], step)
        and integer(step,1,20000), "exact original six-field chronology required")
    require(type(row["physical_slot_indices"]) is list and len(row["physical_slot_indices"]) == 8
        and all(integer(slot,0,1208) for slot in row["physical_slot_indices"])
        and type(row["shared_crop_boxes"]) is list and len(row["shared_crop_boxes"]) == 8
        and same(row["actual_paired_TRUE_bits"],[0]*8) and sha(row["RGB_sha256"]) and sha(row["target_sha256"]),
        "eight admitted FALSE-only slots and actual historical SHA required")
    for value in row["shared_crop_boxes"]:
        box(value)


def box(value):
    require(type(value) is list and len(value) == 4 and all(integer(v,0,640) for v in value), "strict crop box required")
    top,left,height,width = value
    require(height > 0 and width > 0 and top+height <= 360 and left+width <= 640, "native crop bounds differ")


def original_selected(audit):
    selected, count = [], 0
    path = canonical(RUN/"control/exposures.jsonl")
    with path.open("rb") as stream:
        while True:
            raw = stream.readline(4097)
            if not raw:
                break
            audit.guard()
            count += 1
            require(count <= 20000 and len(raw) <= 4096 and raw.endswith(b"\n"), "bounded complete original metadata row required")
            value = decode(raw)
            baseline(value,count)
            if count <= 16:
                selected.append(dict(step=count, RAW_line_sha256=hashlib.sha256(raw).hexdigest(), baseline=value))
    require(count == 20000, "full20k metadata denominator required")
    audit.read(path,INPUTS[str(path)])
    return selected


def native_slots(rows, original):
    require(type(rows) is list and len(rows) == 8, "eight actual native witnesses required")
    for position,row in enumerate(rows):
        require(type(row) is dict and set(row) == {"position","slot","native_RGB_sha256","native_target_sha256",
            "reader_provenance_authenticated_by_worker"} and same(row["position"],position)
            and same(row["slot"],original["physical_slot_indices"][position])
            and sha(row["native_RGB_sha256"]) and sha(row["native_target_sha256"])
            and row["reader_provenance_authenticated_by_worker"] is False, "native position/slot/hash join differs")


def candidate(value, original):
    fields = {"schema","step","journal","native_slots","actual_baseline_RGB_and_GT_hashes_reproduced",
        "candidate_hashes_computed_from_actual_owned_arrays","same_box_for_actual_RGB_and_GT",
        "source_and_baseline_RAW_authenticated_by_worker","training_steps","NN"}
    require(type(value) is dict and set(value) == fields
        and value["schema"] == "portable_e2e.map_crop_batch_worker.20261008.v3.batch"
        and same(value["step"],original["step"]) and all(value[key] is True for key in
        ("actual_baseline_RGB_and_GT_hashes_reproduced","candidate_hashes_computed_from_actual_owned_arrays","same_box_for_actual_RGB_and_GT"))
        and value["source_and_baseline_RAW_authenticated_by_worker"] is False
        and same(value["training_steps"],0) and same(value["NN"],0), "unchanged candidate V3 worker scope differs")
    native_slots(value["native_slots"],original)
    row = value["journal"]
    require(type(row) is dict and set(row) == {"schema","policy_source_sha256","baseline_exposure","selected_crop_boxes",
        "full_GT_sign_pixels","original_NEAREST_sign_pixels","protected","rescued","candidate_RGB_sha256","candidate_target_sha256"}
        and row["schema"] == "portable_e2e.map_crop_preservation_journal.20261008.v1"
        and row["policy_source_sha256"] == POLICY_PIN and same(row["baseline_exposure"],original), "candidate policy/original binding differs")
    require(all(type(row[key]) is list and len(row[key]) == 8 for key in
        ("selected_crop_boxes","full_GT_sign_pixels","original_NEAREST_sign_pixels","protected","rescued")), "eight policy observations required")
    rescued = []
    for position,proposal in enumerate(original["shared_crop_boxes"]):
        full,cropped = row["full_GT_sign_pixels"][position],row["original_NEAREST_sign_pixels"][position]
        require(integer(full,0,230400) and integer(cropped,0,230400), "strict actual support integers required")
        protected = ((original["step"]-1)*8+position)%2 == 0
        rescue = protected and full > 0 and cropped == 0
        require(row["protected"][position] is protected and row["rescued"][position] is rescue
            and same(row["selected_crop_boxes"][position],[0,0,360,640] if rescue else proposal), "support/parity/chosen box differs")
        box(row["selected_crop_boxes"][position])
        if rescue:
            rescued.append([original["step"],position,original["physical_slot_indices"][position],full])
    require(sha(row["candidate_RGB_sha256"]) and sha(row["candidate_target_sha256"]), "candidate actual SHA invalid")
    if not rescued:
        require(row["candidate_RGB_sha256"] == original["RGB_sha256"] and row["candidate_target_sha256"] == original["target_sha256"],
            "nonrescue batch must reproduce original byte SHA")
    return row,rescued


def array_descriptor(value, shape, dtype, maximum):
    require(type(value) is dict and set(value) == {"shape","dtype","nbytes","owned","C_contiguous","finite","minimum","maximum","sha256"}
        and same(value["shape"],shape) and value["dtype"] == dtype
        and same(value["nbytes"],math.prod(shape)*{"uint8":1,"float32":4,"int64":8}[dtype])
        and all(value[key] is True for key in ("owned","C_contiguous","finite")) and sha(value["sha256"]), "exact native/host descriptor required")
    types = (float,) if dtype == "float32" else (int,)
    require(all(type(value[key]) in types and math.isfinite(value[key]) for key in ("minimum","maximum"))
        and 0 <= value["minimum"] <= value["maximum"] <= maximum, "typed finite array range differs")


def tensor_descriptor(value, host):
    require(type(value) is dict and set(value) == {"shape","dtype","device","layout","C_contiguous","strides","storage_offset",
        "requires_grad","grad_fn_is_None","is_pinned","has_no_base","nbytes","storage_nbytes","minimum","maximum","finite","sha256"},
        "exact17-field CPU tensor descriptor required")
    shape = host["shape"]
    expected = dict(shape=shape,dtype=host["dtype"],device="cpu",layout="strided",C_contiguous=True,
        strides=[691200,230400,640,1] if len(shape) == 4 else [230400,640,1],storage_offset=0,
        requires_grad=False,grad_fn_is_None=True,is_pinned=False,has_no_base=True,nbytes=host["nbytes"],
        storage_nbytes=host["nbytes"],minimum=host["minimum"],maximum=host["maximum"],finite=True,sha256=host["sha256"])
    require(same(value,expected), "owned CPU clone descriptor must join exact host bits/ABI")


def tensor_record(value, original, mode, native_row):
    fields = {"schema","arm","step","source_native_slots","source_serial_scope","native_record_metadata_sha256",
        "bridge_receipt","bridge_copy_proof","trainer_CPU_reference_proof","chronology"}
    require(type(value) is dict and set(value) == fields and value["schema"] == PRODUCER+".tensor_receipt"
        and value["arm"] == mode and same(value["step"],original["step"])
        and value["chronology"] == "TENSOR_AND_REFERENCE_CHECKED_BOTH_METADATA_APPENDS_BEFORE_REVERIFY_AND_SERIAL_ACK"
        and value["native_record_metadata_sha256"] == metadata_sha(native_row)
        and same(value["source_serial_scope"],SERIAL_SCOPE), "saved tensor/native/serial scope chronology differs")
    native_slots(value["source_native_slots"],original)
    receipt = value["bridge_receipt"]
    require(type(receipt) is dict and set(receipt) == {"schema","step","mode","source_receipt_sha256","provided_baseline_sha256",
        "native_arrays","host_arrays","tensor_arrays","copy_from_host_bytes_verified","scope"}
        and receipt["schema"] == BRIDGE+".batch" and same(receipt["step"],original["step"])
        and receipt["mode"] == mode and receipt["provided_baseline_sha256"] == metadata_sha(original)
        and receipt["copy_from_host_bytes_verified"] is True and same(receipt["scope"],BRIDGE_SCOPE), "exact bridge10 receipt differs")
    if mode == ARMS[0]:
        require(same(native_row,original), "control must retain exact six original fields")
        rgb_pin,gt_pin,rescues = original["RGB_sha256"],original["target_sha256"],[]
    else:
        row,rescues = candidate(native_row,original)
        require(same(native_row["native_slots"],value["source_native_slots"]), "serial/worker native witness differs")
        rgb_pin,gt_pin = row["candidate_RGB_sha256"],row["candidate_target_sha256"]
    native,host,tensors = receipt["native_arrays"],receipt["host_arrays"],receipt["tensor_arrays"]
    require(all(type(pair) is dict and set(pair) == {"RGB","target"} for pair in (native,host,tensors)), "three RGB/GT descriptor pairs required")
    array_descriptor(native["RGB"],[8,360,640,3],"uint8",255)
    array_descriptor(native["target"],[8,360,640],"uint8",6)
    array_descriptor(host["RGB"],[8,3,360,640],"float32",1)
    array_descriptor(host["target"],[8,360,640],"int64",6)
    require(native["RGB"]["sha256"] == rgb_pin and native["target"]["sha256"] == gt_pin
        and all(same(native["target"][key],host["target"][key]) for key in ("minimum","maximum")), "native crop/host GT descriptor differs")
    for key in ("RGB","target"):
        tensor_descriptor(tensors[key],host[key])
    # HH_261012 - Reconstruct, do not import, the frozen source11 receipt and complete bridge12 proof.
    serial = dict(schema=SERIAL+".batch",mode=mode,step=original["step"],provided_baseline=original,
        provided_baseline_sha256=metadata_sha(original),control_exposure=native_row if mode == ARMS[0] else None,
        candidate_worker_receipt=native_row if mode == ARMS[1] else None,native_slots=value["source_native_slots"],
        native_arrays=native,host_arrays=host,scope=SERIAL_SCOPE)
    require(receipt["source_receipt_sha256"] == metadata_sha(serial), "full source11 metadata seal differs")
    consumer = dict(consumer=CONSUMER,accepted=True,host_RGB_sha256=host["RGB"]["sha256"],host_target_sha256=host["target"]["sha256"])
    proof = dict(schema=BRIDGE+".proof",status="CPU_CLONE_COPY_VERIFIED_NOT_TRAINER_REFERENCE",step=original["step"],mode=mode,
        source_receipt_sha256=receipt["source_receipt_sha256"],provided_baseline_sha256=metadata_sha(original),
        native_and_host_buffers_rechecked=True,CPU_tensor_buffers_rechecked=True,copy_from_host_bytes_verified=True,
        tensor_arrays=tensors,consumer_receipt=consumer,scope=BRIDGE_SCOPE)
    compact = {key:item for key,item in proof.items() if key not in ("tensor_arrays","scope")}
    compact["full_proof_metadata_sha256"] = metadata_sha(proof)
    require(same(value["bridge_copy_proof"],compact), "reconstructed full bridge12 proof seal/consumer differs")
    reference = dict(schema=PRODUCER+".trainer_CPU_reference",function="tensors",source_sha256=REFERENCE_PIN,device="cpu",
        all_RGB_float32_bits_equal=True,all_GT_int64_values_equal=True,RGB_sha256=host["RGB"]["sha256"],
        target_sha256=host["target"]["sha256"],original_trainer_module_imported=False,
        NN_forward=0,backward=0,optimizer_updates=0,GPU_numeric_equivalence_proven=False)
    require(same(value["trainer_CPU_reference_proof"],reference), "actual frozen CPU recipe proof scope/hash differs")
    inner = dict(step=original["step"],consumer_receipt=consumer,native_and_host_buffers_rehashed=True,consumer_execution_authenticated=False)
    ack = dict(step=original["step"],actual_serial_ACK_received=True,serial_acknowledgement=inner,
        full_bridge_reverification_metadata_sha256=metadata_sha(proof),trainer_CPU_reference_proof_sha256=metadata_sha(reference))
    return dict(step=original["step"],mode=mode,rescued_slots=rescues,host_RGB_sha256=host["RGB"]["sha256"],
        host_target_sha256=host["target"]["sha256"]),ack,inner


def pipeline(value):
    require(same(value,dict(expected_batches=16,produced_batches=16,tensor_checked_batches=16,native_rows_written=16,
        tensor_rows_written=16,acknowledged_batches=16,failed_step=None,PASS_batches=16,FAIL_batches=0,
        UNEXECUTED_batches=0,original20k_batches_outside_declaration=19984)), "whole declared production/check/write/ACK denominator differs")


def acknowledgement(value, expected):
    require(same(value,expected), "saved pre-ACK proof is not joined to actual positive serial ACK")


def adapter_summary(value,mode,selected,acks):
    require(type(value) is dict and value["schema"] == SERIAL+".summary" and value["mode"] == mode
        and same(value["declared_steps"],STEPS) and value["closed"] is True
        and value["status"] == "COMPLETE_ACKNOWLEDGED_INPUTS_NOT_TRAINING_OR_MODEL_APPROVAL"
        and same(value["scope"],SERIAL_SCOPE), "closed serial input summary scope differs")
    counts = dict(expected_batches=16,PASS_batches=16,FAIL_batches=0,UNEXECUTED_batches=0,produced_batches=16,
        production_attempts=16,production_FAIL_batches=0,production_UNEXECUTED_batches=0,consumption_attempts=16,
        consumption_attempted_batches=16,consumption_PASS_batches=16,consumption_FAIL_batches=0,
        consumption_UNEXECUTED_batches=0,actual_native_read_attempts=128,original20000_batches_outside_declaration=19984)
    require(all(same(value[key],expected) for key,expected in counts.items()) and value["pending_step"] is None
        and value["failure"] is None and value["failed_tail"] is None and same(value["cleanup_errors"],[])
        and same(value["reader_closing_observations"],[]) and same(value["acknowledgements"],acks)
        and value["provided_rows_sha256"] == metadata_sha([row["baseline"] for row in selected])
        and integer(value["owner_PID"],1) and integer(value["owner_thread_ident"],1), "actual ACK/128 native-read/owner summary differs")


def reader_stats(value):
    fields = {"current_checkpoint_states","maximum_checkpoint_states_observed","max_checkpoint_states","checkpoint_state_account_bytes",
        "cache_accounted_bytes","maximum_cache_accounted_bytes","cache_account_limit_bytes","owner_checkpoint_calls",
        "archive_generation_guard_calls","owner_checkpoint_guard_stride","checkpoint_hits","checkpoint_misses","checkpoint_creations",
        "checkpoint_evictions","total_prefix_body_bytes_inflated","total_selected_payload_bytes","native_row_cache_payload_bytes","checkpoint_stride_bytes"}
    require(type(value) is dict and set(value) == fields | {"checkpoint_cache_enabled","state_accounting_is_reservation_not_exact_native_allocation"}
        and all(integer(value[key]) for key in fields) and value["checkpoint_cache_enabled"] is True
        and value["state_accounting_is_reservation_not_exact_native_allocation"] is True, "typed V3 reader snapshot required")
    require(value["current_checkpoint_states"] == value["cache_accounted_bytes"] == value["native_row_cache_payload_bytes"] == 0
        and value["max_checkpoint_states"] == 48 and value["maximum_checkpoint_states_observed"] <= 48
        and value["checkpoint_state_account_bytes"] == 256*1024 and value["cache_account_limit_bytes"] == 12*1024**2
        and value["maximum_cache_accounted_bytes"] == value["maximum_checkpoint_states_observed"]*256*1024 <= 12*1024**2
        and value["checkpoint_stride_bytes"] == 32*1024**2 and value["owner_checkpoint_guard_stride"] == 16
        and value["total_selected_payload_bytes"] == 128*921600, "closed cache and native128 payload bounds differ")


def writer_summary(value,path,maximum,audit):
    witness = audit.files[str(path)]
    expected = dict(path=str(path),declared_steps=STEPS,rows_written=16,flushed_rows=16,bytes_written=witness["bytes"],
        maximum_bytes=maximum,closed=True,closing_check=dict(status="MATCHED",bytes_read=witness["bytes"],
        sha256=witness["sha256"],written_sha256=witness["sha256"],generation=witness["generation"],bytes=witness["bytes"]),closing_errors=[])
    require(same(value,expected), "whole16 flushed journal closing RAW SHA/generation differs")


def read_rows(audit,path,maximum):
    lines = audit.read(path,metadata=True,maximum=maximum).splitlines(keepends=True)
    require(len(lines) == 16 and all(line.endswith(b"\n") and len(line) <= LINE_LIMIT for line in lines), "complete bounded FIRST16 journal required")
    rows = [decode(line) for line in lines]
    require(all(type(row) is dict and same(row.get("step"),step) for row,step in zip(rows,STEPS)), "saved journal order differs")
    return rows


def providers(environment):
    base,selected,pins,aliases = (environment[key] for key in ("base","selected_Torch_provider_paths","selected_provider_sha256","imported_alias_to_resolved_target"))
    require(type(base) is dict and type(base["selected_provider_sha256"]) is dict and len(base["selected_provider_sha256"]) == 6
        and type(selected) is dict and set(selected) == {"torch.init","torch.version","torch._C","torch.lib/libtorch_cpu.so",
        "torch.lib/libtorch_python.so","torch.lib/libc10.so"} and len(set(selected.values())) == 6
        and type(pins) is dict and len(pins) == 12 and all(type(path) is str and sha(pin) for path,pin in pins.items()), "selected scientific6/Torch6 membership differs")
    require(set(pins) == set(base["selected_provider_sha256"]) | set(selected.values())
        and all(pins[path] == pin for path,pin in base["selected_provider_sha256"].items())
        and environment["Torch_origin"] == selected["torch.init"] and type(environment["Torch_version"]) is str
        and environment["complete_transitive_provider_closure_claimed"] is False
        and environment["historical_environment_equivalence_claimed"] is False
        and environment["Torch_CUDA_initialized"] is False and same(environment["Torch_threads"],1)
        and same(environment["Torch_interop_threads"],1), "selected provider identity/scope differs")
    origin = canonical(selected["torch.init"])
    expected = {str(origin.parent/"lib"/name):selected["torch.lib/"+name] for name in ("libtorch_cpu.so","libtorch_python.so","libc10.so")}
    require(same(aliases,expected) and selected["torch.version"] == str(origin.parent/"version.py"), "named Torch aliases/version path differ")
    check_aliases(aliases)
    return pins,aliases


def inspect(audit,plan_pin,grant_pin,source_pin,test_pin):
    audit.read(Path(__file__).resolve(),source_pin)
    audit.read(TEST,test_pin)
    raw_plan,raw_grant = audit.read(PLAN/"ROOT_plan.json",plan_pin,True),audit.read(PLAN/"ROOT_authority.json",grant_pin,True)
    require(audit.read(PLAN/"candidate_plan.json",plan_pin,True) == raw_plan
        and audit.read(OUT/"ROOT_plan.json",plan_pin,True) == raw_plan
        and audit.read(OUT/"ROOT_authority.json",grant_pin,True) == raw_grant, "RAW ROOT copies differ")
    plan,grant = decode(raw_plan),decode(raw_grant)
    require(plan["schema"] == PRODUCER and same(plan["contract"],contract()) and plan["output"] == str(OUT)
        and same(plan["input_sha256"],INPUTS) and same(grant,authority(plan_pin)), "fresh exact ROOT plan/grant differs")
    sources = plan["source_sha256"]
    require(type(sources) is dict and set(sources) == {str(ROOT/name) for name in SOURCE_NAMES}
        and all(sha(pin) for pin in sources.values()) and same(plan["source_archive_count"],len(set(sources.values())))
        and sources[str(ROOT/REFERENCE)] == REFERENCE_PIN
        and sources[str(ROOT/"tools/e2e/map_train_sign_preserving_crop_policy_20261008_v1.py")] == POLICY_PIN
        and sources[str(ROOT/"learning/portable_e2e/map_crop_batch_worker_20261008_v3.py")]
        == "882f3a5fdd2eadfb1f48cabd8b87a3b3b9d9c03d0572dcd810ae326e152be456", "exact frozen/new SOURCE19 membership differs")
    provider_pins,audit.aliases = providers(plan["environment"])
    archives = {str(PLAN/"sources"/(pin+".py")):pin for pin in set(sources.values())}
    seals = {}
    for group in (sources,INPUTS,provider_pins,archives):
        for path,pin in group.items():
            require(path not in seals or seals[path] == pin, "overlapping RAW seal conflict")
            seals[path] = pin
            audit.read(path,pin)
    selected = original_selected(audit)
    require(same(selected,plan["selected_baseline_rows"]), "original20k/FIRST16 RAW rows differ")
    result = decode(audit.read(OUT/"result.json",metadata=True))
    require(result["schema"] == PRODUCER+".result" and result["ROOT_plan_sha256"] == plan_pin
        and result["ROOT_authority_sha256"] == grant_pin and same(result["contract"],contract())
        and result["status"] == "COMPLETE_CPU_TORCH_INPUT_BOUNDARY_NOT_TRAINING" and result["failure"] is None
        and result["full_failed_tail"] is None and same(result["secondary_closing_errors"],[]), "producer failed/incomplete; preserve its original declared denominator")
    require(result["Torch_imported"] is True and result["Torch_CUDA_initialized"] is False
        and result["actual_trainer_connected"] is False and result["model_performance_gain_proven"] is False
        and all(same(result[key],0) for key in ("NN_forward","backward","optimizer_updates","training_steps","GPU","SSH","CARLA","main_approved","sub_approved"))
        and same(result["imported_Torch_library_aliases"],audit.aliases), "Torch boundary cannot claim GPU/model/training approval")
    memory = result["sampled_closing_memory"]
    require(type(memory) is dict and set(memory) == {"VmRSS","VmHWM"}
        and all(integer(memory[key],1,RSS) for key in memory) and integer(result["sampled_maximum_VmHWM_bytes"],memory["VmHWM"],RSS)
        and type(result["elapsed_seconds"]) in (int,float) and math.isfinite(result["elapsed_seconds"])
        and 0 <= result["elapsed_seconds"] < DEADLINE and integer(result["owner_guard_calls"],1), "producer sampled resource witness differs")
    closing_expected = {**seals,str(PLAN/"ROOT_plan.json"):plan_pin,str(PLAN/"candidate_plan.json"):plan_pin,
        str(PLAN/"ROOT_authority.json"):grant_pin,str(OUT/"ROOT_plan.json"):plan_pin,str(OUT/"ROOT_authority.json"):grant_pin}
    closing = result["closing_seal_observations"]
    require(type(closing) is list and len(closing) == len(closing_expected), "complete producer closing seal table required")
    seen = set()
    for value in closing:
        require(type(value) is dict and set(value) == {"path","expected_sha256","status","before","after"}
            and value["path"] in closing_expected and value["path"] not in seen, "exact once-only closing path required")
        path,pin = value["path"],closing_expected[value["path"]]
        witness = audit.files[path]
        expected = dict(path=path,sha256=pin,generation=witness["generation"],bytes=witness["bytes"])
        require(value["expected_sha256"] == pin and value["status"] == "MATCHED"
            and same(value["before"],expected) and same(value["after"],expected), "producer closing RAW byte/generation witness differs")
        seen.add(path)
    arms,checked,slots = result["arms"],{},{}
    require(type(arms) is dict and set(arms) == set(ARMS), "two exact crop modes required")
    rescues = []
    for mode in ARMS:
        arm = arms[mode]
        pipeline(arm["pipeline"])
        require(type(arm["acknowledgements"]) is list and len(arm["acknowledgements"]) == 16, "sixteen actual serial ACK records required")
        native_path,tensor_path = OUT/mode/"exposures.jsonl",OUT/mode/"tensor_receipts.jsonl"
        native_rows,tensor_rows = read_rows(audit,native_path,NATIVE_LIMIT),read_rows(audit,tensor_path,TENSOR_LIMIT)
        writer_summary(arm["native"],native_path,NATIVE_LIMIT,audit)
        writer_summary(arm["tensor"],tensor_path,TENSOR_LIMIT,audit)
        mode_checked,inner_acks,mode_rescues = [],[],[]
        for step,native_row,tensor_row in zip(STEPS,native_rows,tensor_rows):
            row,ack,inner = tensor_record(tensor_row,selected[step-1]["baseline"],mode,native_row)
            acknowledgement(arm["acknowledgements"][step-1],ack)
            mode_checked.append(row)
            inner_acks.append(inner)
            mode_rescues.extend(row["rescued_slots"])
            for value in tensor_row["source_native_slots"]:
                pair = [value["native_RGB_sha256"],value["native_target_sha256"]]
                require(value["slot"] not in slots or same(slots[value["slot"]],pair), "same source slot has inconsistent actual native hash across modes")
                slots[value["slot"]] = pair
        adapter_summary(arm["adapter"],mode,selected,inner_acks)
        reader_stats(arm["reader_stats_after_close"])
        require(same(arm["reader_closing_errors"],[]) and same(arm["observed_rescued_slots"],len(mode_rescues)), "actual rescue count/reader closing differs")
        checked[mode] = mode_checked
        rescues.extend(mode_rescues)
    require(same(rescues,[[6,2,1036,56],[10,4,344,37]]), "known FIRST16 actual rescue witnesses differ; do not hide changes")
    audit.close_seals()
    return dict(status="COMPLETE_INDEPENDENT_SAVED_METADATA_AND_RAW_BYTES_ONLY",checked_batches=checked,
        declared_batches_per_arm=16,PASS_batches_per_arm=16,FAIL_batches_per_arm=0,UNEXECUTED_batches_per_arm=0,
        produced_tensor_checked_saved_and_ACKed_per_arm=16,native_receipts_per_arm=128,
        same_128_source_exposures_repeated_across_arms=True,original20k_metadata_rows=20000,
        original20k_batches_outside_declaration_per_arm=19984,candidate_rescue_witnesses=rescues,
        source_files=19,source_archives=len(archives),original_inputs=7,selected_providers=12,
        native_files=2,tensor_receipt_files=2,source11_and_full_bridge12_metadata_reconstructed=True,
        producer_numeric_comparisons_checked_as_saved_metadata=True,array_SHA_independently_recomputed=False,
        original_trainer_CPU_reference_independently_replayed=False,actual_consumer_execution_independently_authenticated=False,
        complete_transitive_provider_closure_proven=False,GPU_numeric_equivalence_proven=False,model_gain_proven=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--execute",action="store_true")
    for name in ("plan-sha256","authority-sha256","script-sha256","test-source-sha256"):
        parser.add_argument("--"+name,required=True)
    args = parser.parse_args()
    pins = [args.plan_sha256,args.authority_sha256,args.script_sha256,args.test_source_sha256]
    require(args.execute and all(sha(pin) for pin in pins) and sys.flags.dont_write_bytecode and sys.flags.optimize == 0,
        "explicit ROOT clean execution with external RAW pins required")
    require(not any(name == "numpy" or name == "PIL" or name.startswith("torch") for name in sys.modules), "stdlib-only fresh checker required")
    canonical(RECEIPT,False)
    require(not RECEIPT.exists(), "fresh independent receipt only; no overwrite/retry")
    with canonical(LOCK).open("rb") as lease:
        fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
        audit = Audit(lease)
        try:
            try:
                audit.guard()
                outcome = inspect(audit,*pins)
            except Exception as error:
                arguments = BaseException.args.__get__(error,type(error))
                message = arguments[0][:4096] if arguments and type(arguments[0]) is str else "No exact-string failure message"
                outcome = dict(status="FAILED_INDEPENDENT_SAVED_CHECK_RETAINED",error_type=type(error).__name__[:64],error=message,
                    original_evidence_modified_or_deleted=False)
            receipt = dict(schema=SCHEMA,provenance="HH_261012 - Saved RAW bytes/metadata only; no second tensor/reference replay.",
                ROOT_plan_RAW_sha256=pins[0],ROOT_authority_RAW_sha256=pins[1],checker_source_sha256=pins[2],scalar_test_source_sha256=pins[3],
                result=outcome,independently_read_file_seals=audit.files,elapsed_seconds=time.monotonic()-audit.started,
                sampled_maximum_VmHWM_bytes=audit.maximum_HWM,sampled_guards_not_OS_containment=True,
                array_decodes=0,array_SHA_second_recomputations=0,Torch_imported=False,trainer_reference_second_replays=0,
                NN_forward=0,backward=0,optimizer_updates=0,training_steps=0,GPU=0,SSH=0,CARLA=0,
                formal_main_approved=0,formal_sub_approved=0)
            raw = encoded(receipt)
            require(len(raw) <= METADATA, "bounded independent receipt required")
            with RECEIPT.open("xb") as stream:
                require(stream.write(raw) == len(raw), "partial independent receipt write")
                stream.flush()
            print(encoded(dict(status=outcome["status"],receipt=str(RECEIPT),bytes=len(raw))).decode(),end="")
            return 0 if outcome["status"].startswith("COMPLETE_") else 2
        finally:
            fcntl.flock(lease,fcntl.LOCK_UN)


if __name__ == "__main__":
    raise SystemExit(main())
