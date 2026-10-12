"""HH_261012 - Fresh FIRST16 CPU Torch input boundary, not a trainer or model run.

SOURCE_ONLY until ROOT reviews this source/tests and executes fresh preparation
and a RAW-pinned grant. No model construction, NN forward/backward, optimizer,
CUDA initialization, multiprocessing, installation, simulator or remote action.
Old modules supply pure reads only; their budgets, writers, grants and globals
are never used or changed. The original trainer is read as pinned source and
only its tensors function is AST-selected into a private CPU reference namespace.
"""
from __future__ import annotations

import argparse
import ast
from copy import deepcopy
import fcntl
import hashlib
import importlib.util
import os
from pathlib import Path
import stat
import sys
import threading
import time

from scripts.e2e import verify_map_serial_training_input_20261008_v1 as pure

ROOT = Path(__file__).resolve().parents[2]
NAME = "map_torch_input_boundary_v1"
SCHEMA = "portable_e2e.map_torch_input_boundary.20261012.v1"
PLAN_DIR = ROOT/"storage/experiments/plans/2026-10-12"/NAME
OUT = ROOT/"storage/experiments/diagnostics/2026-10-12"/NAME
LOCK, DATA, RUN = pure.LOCK, pure.DATA, pure.RUN
STEPS, ARMS = list(range(1, 17)), tuple(pure.ARMS)
FLOOR, RESERVE, LIMIT, CLOSING = 2*1024**3, 64*1024**2, 2*1024**2, 512*1024
RSS, DEADLINE = 512*1024**2, 900
NATIVE_LIMIT, TENSOR_LIMIT, LINE_LIMIT = 64*1024, 128*1024, 8*1024
REFERENCE = "tools/e2e/train_map_native_fusion_pair_20261002.py"
REFERENCE_PIN = "3918b5a529f08e17cfd294d24a21fbd545ee0adccc0e1ac3adaf79c60eb3f841"
BRIDGE = "learning/portable_e2e/map_torch_cpu_input_bridge_20261012_v1.py"
SOURCES = [*pure.SOURCES, BRIDGE,
    "learning/tests/test_map_torch_cpu_input_bridge_20261012_v1.py", REFERENCE,
    "tools/e2e/verify_map_torch_input_boundary_20261012_v1.py",
    "learning/tests/test_verify_map_torch_input_boundary_20261012_v1.py"]
INPUTS = dict(pure.INPUTS)
require, canonical, encoded, decode = pure.require, pure.canonical, pure.encoded, pure.decode
equal, integer, sha, safe_error = pure.equal, pure.integer, pure.sha, pure.safe_error
FACTS = ("produced_batches", "tensor_checked_batches", "native_rows_written",
    "tensor_rows_written", "acknowledged_batches")


def contract():
    return dict(declared_steps=STEPS, arms=list(ARMS), expected_batches_per_arm=16,
        expected_slot_exposures_per_arm=128, same_128_source_exposures_repeated_across_arms=True,
        original20k_batches_outside_declaration_per_arm=19984, reader_revision="V3", cache_enabled=True,
        checkpoint_max_states=48, checkpoint_state_account_bytes=256*1024,
        checkpoint_cache_account_limit_bytes=12*1024**2, checkpoint_stride_bytes=32*1024**2,
        owner_checkpoint_guard_stride=16, cache_accounting_is_reservation_not_exact_native_allocation=True,
        native_RGB_shape=[8,360,640,3], native_target_shape=[8,360,640], native_dtype="uint8",
        tensor_RGB_shape=[8,3,360,640], tensor_RGB_dtype="float32",
        tensor_target_shape=[8,360,640], tensor_target_dtype="int64", tensor_device="cpu", labels=list(range(7)),
        original_trainer_source=str(ROOT/REFERENCE), original_trainer_source_sha256=REFERENCE_PIN,
        original_trainer_imported=False, reference_function="tensors", reference_device="cpu",
        all_reference_RGB_float32_bits_and_GT_values_equal_required=True,
        NumPy_host_normalization_reapplied=False, consumer="map_torch_cpu_value_consumer_v1",
        positive_serial_ACK_after_tensor_reverification_and_both_appends=True,
        produced_written_and_ACKed_are_distinct=True, native_files=2, tensor_receipt_files=2,
        declared_steps_in_each_file=STEPS, native_maximum_bytes_per_file=NATIVE_LIMIT,
        tensor_receipt_maximum_bytes_per_file=TENSOR_LIMIT, maximum_line_bytes=LINE_LIMIT,
        allowed_original_members=["rgb.npy","target.npy"], allowed_paired_members=["rgb_false.npy","target.npy"],
        TRUE_DEV_HOLDOUT_TEST_weights_deniedRAW_decode_authorized=False,
        source_count=19, selected_provider_count=12, complete_transitive_provider_closure_claimed=False,
        Torch_library_aliases_explicitly_resolved_and_rechecked=True,
        CPU_affinity=[3], threads=1, sampled_VmHWM_cap_bytes=RSS, deadline_seconds=DEADLINE,
        maximum_combined_PLAN_OUTPUT_bytes=LIMIT, closing_bytes=CLOSING, floor_bytes=FLOOR, reserve_bytes=RESERVE,
        sampled_guards_not_OS_containment=True, same_source_input_provider_seals_before_after=True,
        old_authority_reused=False, Torch_imported=True, Torch_CPU_input_numeric_equivalence_scope="declared FIRST16 only",
        GPU_numeric_equivalence_proven=False, actual_trainer_connected=False,
        same_model_crop_AB_training_implemented=False, full20k_numeric_execution_authorized=False,
        worker_count=0, prefetch_implemented=False, PNG_created=False,
        NN_forward=0, backward=0, optimizer_updates=0, training_steps=0, GPU=0, SSH=0, CARLA=0,
        main_approved=0, sub_approved=0, model_performance_gain_proven=False)


def authority(plan_pin):
    return dict(schema=SCHEMA+".ROOT_authority", plan_path=str(PLAN_DIR/"ROOT_plan.json"), plan_sha256=plan_pin,
        FIRST16_original_and_FALSE_RGB_GT_Torch_CPU_input_boundary_authorized=True,
        original_control_and_sign_preserving_candidate_authorized=True,
        Torch_import_and_CPU_tensor_operations_authorized=True, original_trainer_AST_tensors_reference_authorized=True,
        training_authorized=False, NN_forward_authorized=False, backward_authorized=False, GPU_authorized=False,
        TRUE_DEV_HOLDOUT_TEST_weights_deniedRAW_decode_authorized=False,
        old_authority_reused=False, output=str(OUT), contract=contract())


def owned_bytes():
    total = 0
    for directory in (PLAN_DIR, OUT):
        canonical(directory, False)
        if not directory.exists():
            continue
        for path in directory.rglob("*"):
            canonical(path)
            info = path.lstat()
            require(stat.S_ISREG(info.st_mode) or stat.S_ISDIR(info.st_mode), "owned special path denied")
            if stat.S_ISREG(info.st_mode):
                require(info.st_nlink == 1, "owned hardlink denied")
                total += info.st_size
    return total


class Budget:
    """HH_261012 - Scope sampled limits to this new plan/output and retained lease only."""
    def __init__(self, lease):
        self.lease, self.started = lease, time.monotonic()
        self.identity, self.lease_generation = pure.old.process_identity(), pure.old.generation(os.fstat(lease.fileno()))
        self.maximum_HWM, self.last, self.guard_calls = 0, None, 0
        self.aliases = None

    def guard(self):
        self.guard_calls += 1
        if self.aliases is not None:
            check_aliases(self.aliases)
        if "torch" in sys.modules:
            require(sys.modules["torch"].cuda.is_initialized() is False, "CUDA initialized in CPU-only owner")
        require(not self.lease.closed and pure.old.process_identity() == self.identity, "owned process/lease changed")
        path, fd = canonical(LOCK).stat(), os.fstat(self.lease.fileno())
        require(stat.S_ISREG(path.st_mode) and path.st_nlink == fd.st_nlink == 1
            and pure.old.generation(path) == pure.old.generation(fd) == self.lease_generation, "lease FD/path generation changed")
        self.last = {line.split(":")[0]: int(line.split()[1])*1024
            for line in Path("/proc/self/status").read_text().splitlines() if line.startswith(("VmRSS:","VmHWM:"))}
        self.maximum_HWM = max(self.maximum_HWM, self.last["VmHWM"])
        require(self.maximum_HWM <= RSS and time.monotonic()-self.started < DEADLINE, "sampled memory/deadline exceeded")
        fs = os.statvfs(ROOT)
        require(fs.f_bavail*fs.f_frsize >= FLOOR+RESERVE, "unchanged floor plus reserve unavailable")
        require(owned_bytes() <= LIMIT-CLOSING, "unchanged closing reserve unavailable")


def write_new(path, raw, *, closing=False):
    path = canonical(path, False)
    require(type(raw) is bytes and len(raw) <= CLOSING and
        (path.is_relative_to(PLAN_DIR) or path.is_relative_to(OUT)), "bounded owned metadata required")
    require(owned_bytes()+len(raw) <= LIMIT-(0 if closing else CLOSING), "combined2MiB write allowance denied")
    with path.open("xb") as stream:
        require(stream.write(raw) == len(raw), "partial metadata write")
        stream.flush()


def file_seal(path, expected, checkpoint):
    path = canonical(path)
    before = pure.old.generation(path.stat())
    actual = pure.digest(path, checkpoint)
    require(actual == expected and before == pure.old.generation(path.stat()), "RAW source/input/provider seal differs")
    return dict(path=str(path), sha256=actual, generation=before, bytes=before[6])


def read_bounded(path,maximum=CLOSING):
    path = canonical(path)
    before = path.stat()
    require(stat.S_ISREG(before.st_mode) and before.st_nlink == 1 and before.st_size <= maximum, "bounded regular metadata required")
    with path.open("rb") as stream:
        opened = os.fstat(stream.fileno())
        raw = stream.read(maximum+1)
        ended = os.fstat(stream.fileno())
    require(len(raw) <= maximum and pure.old.generation(before) == pure.old.generation(opened)
        == pure.old.generation(ended) == pure.old.generation(path.stat()), "metadata FD/path generation or bound differs")
    return raw


def torch_aliases(directory):
    aliases = {str(Path(directory)/"lib"/name): str((Path(directory)/"lib"/name).resolve(strict=True))
        for name in ("libtorch_cpu.so","libtorch_python.so","libc10.so")}
    for target in aliases.values():
        canonical(target)
    return aliases


def check_aliases(aliases):
    require(type(aliases) is dict and len(aliases) == 3 and all(type(a) is type(t) is str for a,t in aliases.items()),
        "three explicit imported Torch library aliases required")
    for alias, target in aliases.items():
        require(str(Path(alias).resolve(strict=True)) == target, "imported Torch library alias changed")
        canonical(target)


def environment(checkpoint):
    # HH_261012 - The old provider-origin helper runs only before Torch imports; never relax its denied imports.
    checkpoint()
    base = pure.old.environment()
    spec = importlib.util.find_spec("torch")
    require(spec is not None and spec.origin, "existing Torch provider required; no installation")
    origin = canonical(Path(spec.origin).resolve())
    aliases = torch_aliases(origin.parent)
    import torch
    require(Path(torch.__file__).resolve() == origin and not torch.cuda.is_initialized(), "Torch origin/CUDA initialization differs")
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    require(torch.get_num_threads() == torch.get_num_interop_threads() == 1, "Torch thread1 required")
    selected = {"torch.init":str(origin), "torch.version":str(canonical(Path(torch.version.__file__).resolve())),
        "torch._C":str(canonical(Path(torch._C.__file__).resolve())),
        **{"torch.lib/"+Path(alias).name: target for alias,target in aliases.items()}}
    require(len(selected) == len(set(selected.values())) == 6, "six distinct selected Torch providers required")
    providers = dict(base["selected_provider_sha256"])
    for path in selected.values():
        providers[path] = pure.digest(path, checkpoint)
    require(len(providers) == 12, "exact selected NumPy/PIL6 plus Torch6 required")
    check_aliases(aliases)
    checkpoint()
    return dict(base=base, Torch_version=str(torch.__version__), Torch_compiled_CUDA=torch.version.cuda,
        Torch_origin=str(origin), selected_Torch_provider_paths=selected, imported_alias_to_resolved_target=aliases,
        selected_provider_sha256=providers, complete_transitive_provider_closure_claimed=False,
        historical_environment_equivalence_claimed=False, Torch_CUDA_initialized=False,
        Torch_threads=1, Torch_interop_threads=1)


def source_pins(checkpoint):
    require(len(SOURCES) == len(set(SOURCES)) == 19, "exact old14 plus new4 and source-only reference required")
    result = {str(ROOT/name):pure.digest(ROOT/name, checkpoint) for name in SOURCES}
    require(result[str(ROOT/REFERENCE)] == REFERENCE_PIN, "frozen original trainer reference source differs")
    return result


def metadata_sha(value):
    return hashlib.sha256(encoded(value)[:-1]).hexdigest()


def archives(plan):
    return {str(PLAN_DIR/"sources"/(pin+".py")):pin for pin in set(plan["source_sha256"].values())}


def seal_table(plan):
    result = {}
    for group in (plan["source_sha256"], INPUTS, plan["environment"]["selected_provider_sha256"], archives(plan)):
        for path,pin in group.items():
            require(sha(pin) and (path not in result or result[path] == pin), "conflicting RAW pin")
            result[path] = pin
    return result


def quota(source_bytes, plan_bytes):
    require(integer(source_bytes) and integer(plan_bytes, 1, CLOSING), "bounded source/plan allowance required")
    amount = source_bytes+4*plan_bytes+2*(NATIVE_LIMIT+TENSOR_LIMIT)+CLOSING+64*1024
    return dict(source_archive_bytes=source_bytes, four_plan_copy_allowance_bytes=4*plan_bytes,
        maximum_two_native_and_two_tensor_files_bytes=2*(NATIVE_LIMIT+TENSOR_LIMIT),
        closing_bytes=CLOSING, extra_ROOT_metadata_allowance_bytes=64*1024,
        prospective_combined_PLAN_OUTPUT_bytes=amount, unchanged_limit_bytes=LIMIT,
        fits_unchanged_limits=amount < LIMIT, arrays_executed_by_this_estimate=0)


def prepare(budget):
    require(not PLAN_DIR.exists() and not OUT.exists(), "fresh prepare only; no overwrite/resume")
    env, table = environment(budget.guard), source_pins(budget.guard)
    before = [file_seal(path,pin,budget.guard) for group in (table, INPUTS, env["selected_provider_sha256"]) for path,pin in group.items()]
    plan = dict(schema=SCHEMA, provenance="HH_261012 - Fresh CPU tensor boundary, no trainer/model run.",
        contract=contract(), source_sha256=table, input_sha256=INPUTS, environment=env,
        selected_baseline_rows=pure.old.baseline_rows(16), registration_scope=pure.old.registration(),
        prepare_file_seals=before, output=str(OUT), lock_generation=budget.lease_generation,
        source_archive_count=len(set(table.values())), prepare_process_generation=budget.identity)
    source_bytes = sum(Path(path).stat().st_size for path in table)
    plan["prospective_storage"] = quota(source_bytes,len(encoded(plan))+4096)
    raw = encoded(plan)
    require(plan["prospective_storage"]["fits_unchanged_limits"] and quota(source_bytes,len(raw))["fits_unchanged_limits"],
        "unchanged2MiB metadata envelope denied")
    PLAN_DIR.mkdir(parents=True,exist_ok=False)
    (PLAN_DIR/"sources").mkdir(exist_ok=False)
    for path,pin in table.items():
        value = read_bounded(path)
        require(len(value) <= CLOSING and hashlib.sha256(value).hexdigest() == pin, "source changed before archive")
        destination = PLAN_DIR/"sources"/(pin+".py")
        if not destination.exists():
            write_new(destination,value)
    write_new(PLAN_DIR/"candidate_plan.json",raw)
    return dict(status="PREPARED_CPU_TORCH_METADATA_NOT_AUTHORIZED_OR_RUNNING", candidate_sha256=hashlib.sha256(raw).hexdigest(),
        source_archive_count=len(set(table.values())), arrays_decoded=0, NN_forward=0, backward=0, contract=contract())


def admission(plan_pin,grant_pin,budget):
    require(sha(plan_pin) and sha(grant_pin), "external RAW ROOT plan/grant pins required")
    values = []
    for name,pin in (("ROOT_plan.json",plan_pin),("ROOT_authority.json",grant_pin)):
        path = canonical(PLAN_DIR/name)
        require(path.stat().st_size <= CLOSING, "bounded ROOT metadata required")
        file_seal(path,pin,budget.guard)
        raw = read_bounded(path)
        require(hashlib.sha256(raw).hexdigest() == pin, "ROOT metadata changed before parse")
        values.append(raw)
    plan,grant = map(decode,values)
    require(plan["schema"] == SCHEMA and equal(plan["contract"],contract()) and equal(grant,authority(plan_pin))
        and plan["output"] == str(OUT) and equal(plan["input_sha256"],INPUTS), "fresh exact Torch CPU contract/grant required")
    require(equal(plan["source_sha256"],source_pins(budget.guard)) and len(plan["environment"]["selected_provider_sha256"]) == 12
        and equal(plan["lock_generation"],budget.lease_generation), "source/provider/lease declaration differs")
    check_aliases(plan["environment"]["imported_alias_to_resolved_target"])
    table = {**seal_table(plan),str(PLAN_DIR/"ROOT_plan.json"):plan_pin,
        str(PLAN_DIR/"candidate_plan.json"):plan_pin,str(PLAN_DIR/"ROOT_authority.json"):grant_pin}
    before = {path:file_seal(path,pin,budget.guard) for path,pin in table.items()}
    require(equal(plan["selected_baseline_rows"],pure.old.baseline_rows(16))
        and equal(plan["registration_scope"],pure.old.registration()), "historical FIRST16 or denied source roles differ")
    require(not OUT.exists(), "fresh output required; no overwrite/retry")
    return plan,values[0],values[1],table,before


def reference_function(raw,np_provider):
    """HH_261012 - Compile only the authenticated tensors definition, never its trainer module body."""
    require(type(raw) is bytes and hashlib.sha256(raw).hexdigest() == REFERENCE_PIN, "RAW trainer reference pin differs")
    definitions = [node for node in ast.parse(raw).body if isinstance(node,ast.FunctionDef) and node.name == "tensors"]
    require(len(definitions) == 1 and not definitions[0].decorator_list
        and [arg.arg for arg in definitions[0].args.args] == ["rgb","target","device"], "one exact tensors definition required")
    namespace = dict(BATCH=8,require=require,np=np_provider)
    exec(compile(ast.fix_missing_locations(ast.Module(body=definitions,type_ignores=[])),str(ROOT/REFERENCE),"exec"),namespace)
    return namespace["tensors"]


def ledger():
    return {**{key:0 for key in FACTS},"failed_step":None}


def event(value,kind,step):
    require(kind in FACTS and integer(step,1,16) and value["failed_step"] is None, "strict pipeline chronology required")
    index = FACTS.index(kind)
    require(value[kind] == step-1 and all(value[key] == step for key in FACTS[:index])
        and all(value[key] == step-1 for key in FACTS[index+1:]), "one pending stage required")
    value[kind] += 1


def failed(value,step):
    require(integer(step,1,16), "declared failure step required")
    if step > value["acknowledged_batches"]:
        require(step == value["acknowledged_batches"]+1, "cannot skip failed chronology")
        value["failed_step"] = step


def snapshot(value):
    count,fail = value["acknowledged_batches"],int(value["failed_step"] is not None)
    return dict(expected_batches=16,**deepcopy(value),PASS_batches=count,FAIL_batches=fail,
        UNEXECUTED_batches=16-count-fail,original20k_batches_outside_declaration=19984)


class Writer:
    """HH_261012 - New-scope create-only JSONL with explicit written/flushed and closing RAW witnesses."""
    def __init__(self,path,maximum,checkpoint):
        self.path = canonical(path,False)
        require(self.path.is_relative_to(OUT) and not self.path.exists() and maximum in (NATIVE_LIMIT,TENSOR_LIMIT),
            "fresh bounded new-scope journal required")
        self.stream = self.path.open("x+b")
        self.maximum,self.checkpoint = maximum,checkpoint
        self.open_generation = pure.old.generation(os.fstat(self.stream.fileno()))
        self.count = self.bytes_written = self.flushed_rows = 0
        self.hash,self.errors,self.closed = hashlib.sha256(),[],False
        self.closing_check = dict(status="NOTCHECKED",bytes_read=0)

    def append(self,row):
        require(not self.closed and type(row) is dict and equal(row.get("step"),self.count+1) and self.count < 16,
            "strict FIRST16 journal row required")
        raw = encoded(row)
        require(len(raw) <= LINE_LIMIT and self.bytes_written+len(raw) <= self.maximum, "line/journal cap denied before write")
        self.checkpoint()
        require(owned_bytes()+len(raw) <= LIMIT-CLOSING, "combined quota denied before write")
        require(self.stream.write(raw) == len(raw), "partial journal write")
        self.hash.update(raw)
        self.bytes_written += len(raw)
        self.count += 1
        self.stream.flush()
        self.flushed_rows = self.count

    def close(self):
        if self.closed:
            return
        primary = None
        try:
            self.checkpoint()
            self.stream.flush()
            self.flushed_rows = self.count
            path,fd = canonical(self.path).stat(),os.fstat(self.stream.fileno())
            require(path.st_nlink == fd.st_nlink == 1 and path.st_size == fd.st_size == self.bytes_written
                and (path.st_dev,path.st_ino) == (fd.st_dev,fd.st_ino) == tuple(self.open_generation[4:6]), "journal FD/path/size differs")
            before,observed = pure.old.generation(path),hashlib.sha256()
            self.closing_check["status"] = "CHECKING"
            self.stream.seek(0)
            for block in iter(lambda:self.stream.read(64*1024),b""):
                self.checkpoint()
                observed.update(block)
                self.closing_check["bytes_read"] += len(block)
            require(before == pure.old.generation(self.path.stat()) == pure.old.generation(os.fstat(self.stream.fileno()))
                and observed.hexdigest() == self.hash.hexdigest(), "closing RAW journal SHA/generation differs")
            self.closing_check.update(status="MATCHED",sha256=observed.hexdigest(),written_sha256=self.hash.hexdigest(),
                generation=before,bytes=self.bytes_written)
        except BaseException as error:
            primary = error
            self.errors.append(safe_error("journal_RAW_close",error))
            self.closing_check["status"] = "FAILED_"+self.closing_check["status"]
        finally:
            try:
                self.stream.close()
            except BaseException as error:
                self.errors.append(safe_error("journal_FD_close",error))
                if primary is None:
                    primary = error
            self.closed = True
        if primary is not None:
            raise primary

    def snapshot(self):
        return dict(path=str(self.path),declared_steps=STEPS,rows_written=self.count,flushed_rows=self.flushed_rows,
            bytes_written=self.bytes_written,maximum_bytes=self.maximum,closed=self.closed,
            closing_check=deepcopy(self.closing_check),closing_errors=deepcopy(self.errors))


def tensor_sha(value):
    view = memoryview(value.numpy()).cast("B")
    try:
        return hashlib.sha256(view).hexdigest()
    finally:
        view.release()


def reference_check(source,bundle,reference,checkpoint):
    """HH_261012 - Compare actual CPU bits with the exact source-only trainer recipe, not a model."""
    import torch
    checkpoint()
    rgb,target = reference(source.native_RGB,source.native_target,"cpu")
    require(type(rgb) is type(target) is torch.Tensor and rgb.device.type == target.device.type == "cpu"
        and rgb.dtype == torch.float32 and target.dtype == torch.int64
        and tuple(rgb.shape) == (8,3,360,640) and tuple(target.shape) == (8,360,640)
        and rgb.is_contiguous() and target.is_contiguous() and not rgb.requires_grad and not target.requires_grad,
        "source-only trainer CPU reference ABI differs")
    actual = tensor_sha(bundle.RGB),tensor_sha(bundle.target)
    expected = tensor_sha(rgb),tensor_sha(target)
    require(torch.equal(bundle.RGB,rgb) and torch.equal(bundle.target,target) and actual == expected,
        "actual bridge CPU float32 bits/int64 GT differ from frozen trainer recipe")
    require(not torch.cuda.is_initialized(), "CUDA initialized during CPU input reference")
    checkpoint()
    return dict(schema=SCHEMA+".trainer_CPU_reference", function="tensors", source_sha256=REFERENCE_PIN, device="cpu",
        all_RGB_float32_bits_equal=True, all_GT_int64_values_equal=True,
        RGB_sha256=expected[0],target_sha256=expected[1], original_trainer_module_imported=False,
        NN_forward=0,backward=0,optimizer_updates=0,GPU_numeric_equivalence_proven=False)


def compact_proof(proof,receipt):
    # HH_261012 - Store scope/descriptors once; reconstruct and seal the complete unchanged bridge proof.
    fields = {"schema","status","step","mode","source_receipt_sha256","provided_baseline_sha256",
        "native_and_host_buffers_rechecked","CPU_tensor_buffers_rechecked","copy_from_host_bytes_verified",
        "tensor_arrays","consumer_receipt","scope"}
    require(type(proof) is dict and set(proof) == fields and equal(proof["tensor_arrays"],receipt["tensor_arrays"])
        and equal(proof["scope"],receipt["scope"]), "exact bridge proof/receipt join required")
    for key in ("step","mode","source_receipt_sha256","provided_baseline_sha256"):
        require(equal(proof[key],receipt[key]), "bridge proof chronology/source binding differs")
    require(proof["status"] == "CPU_CLONE_COPY_VERIFIED_NOT_TRAINER_REFERENCE"
        and all(proof[key] is True for key in ("native_and_host_buffers_rechecked","CPU_tensor_buffers_rechecked","copy_from_host_bytes_verified")),
        "actual complete bridge buffer recheck required")
    return {**{key:deepcopy(value) for key,value in proof.items() if key not in ("tensor_arrays","scope")},
        "full_proof_metadata_sha256":metadata_sha(proof)}


def validate_consumer(value,receipt):
    require(type(value) is dict and set(value) == {"consumer","accepted","host_RGB_sha256","host_target_sha256"}
        and value["consumer"] == "map_torch_cpu_value_consumer_v1" and value["accepted"] is True
        and value["host_RGB_sha256"] == receipt["host_arrays"]["RGB"]["sha256"]
        and value["host_target_sha256"] == receipt["host_arrays"]["target"]["sha256"], "strict actual CPU consumer host ACK required")


def process_produced(source,mode,baseline,adapter,native_writer,tensor_writer,value,build,verify,reference,checkpoint,tail):
    """HH_261012 - Injectable scalar seams test chronology; only the actual owner supplies numeric operations."""
    step = source.step
    event(value,"produced_batches",step)
    tail.update(phase="build_CPU_tensors",serial_receipt=source.receipt)
    bundle = build(source)
    tail.update(phase="verify_CPU_tensors",bridge_receipt=bundle.receipt)
    proof = verify(bundle)
    compact = compact_proof(proof,bundle.receipt)
    consumer = proof["consumer_receipt"]
    validate_consumer(consumer,bundle.receipt)
    tail["phase"] = "frozen_trainer_CPU_reference"
    ref = reference(source,bundle)
    event(value,"tensor_checked_batches",step)
    native_row = source.receipt["control_exposure"] if mode == ARMS[0] else source.receipt["candidate_worker_receipt"]
    require(type(native_row) is dict and equal(native_row.get("step"),step)
        and (mode != ARMS[0] or equal(native_row,baseline)), "native control/candidate chronology differs")
    tail["phase"] = "native_append"
    try:
        native_writer.append(native_row)
    finally:
        value["native_rows_written"] = native_writer.count
    checkpoint()
    tail["phase"] = "tensor_append"
    row = dict(schema=SCHEMA+".tensor_receipt",arm=mode,step=step,
        source_native_slots=deepcopy(source.receipt["native_slots"]),source_serial_scope=deepcopy(source.receipt["scope"]),
        native_record_metadata_sha256=metadata_sha(native_row),bridge_receipt=deepcopy(bundle.receipt),
        bridge_copy_proof=compact,trainer_CPU_reference_proof=ref,
        chronology="TENSOR_AND_REFERENCE_CHECKED_BOTH_METADATA_APPENDS_BEFORE_REVERIFY_AND_SERIAL_ACK")
    try:
        tensor_writer.append(row)
    finally:
        value["tensor_rows_written"] = tensor_writer.count
    checkpoint()
    tail["phase"] = "reverify_CPU_tensors_before_serial_ACK"
    after = verify(bundle)
    require(equal(after,proof), "bridge proof changed after metadata writes")
    tail["phase"] = "actual_serial_ACK"
    ack = adapter.acknowledge(step,consumer_receipt=consumer)
    expected = dict(step=step,consumer_receipt=consumer,native_and_host_buffers_rehashed=True,consumer_execution_authenticated=False)
    require(equal(ack,expected), "exact positive serial library ACK required")
    event(value,"acknowledged_batches",step)
    return dict(step=step,actual_serial_ACK_received=True,serial_acknowledgement=ack,
        full_bridge_reverification_metadata_sha256=metadata_sha(after),trainer_CPU_reference_proof_sha256=metadata_sha(ref))


def close_one(resource,scope,errors):
    if resource is not None:
        try:
            resource.close()
        except BaseException as error:
            errors.append(safe_error(scope,error))


def closing_snapshot(resource,method,scope,errors):
    try:
        return getattr(resource,method)()
    except BaseException as error:
        errors.append(safe_error(scope,error))
        return dict(status="NOTCHECKED",error=safe_error(scope,error))


def execute(plan_pin,grant_pin,budget):
    plan,raw_plan,raw_grant,table,before = admission(plan_pin,grant_pin,budget)
    arms = {mode:dict(pipeline=snapshot(ledger()),adapter=None,acknowledgements=[],native=None,tensor=None) for mode in ARMS}
    failure,tail,errors,closing = None,None,[],[]
    reader = adapter = native_writer = tensor_writer = None
    mode = step = None
    active,value = False,None
    try:
        OUT.mkdir(parents=True,exist_ok=False)
        for name,raw,pin in (("ROOT_plan.json",raw_plan,plan_pin),("ROOT_authority.json",raw_grant,grant_pin)):
            path = OUT/name
            write_new(path,raw)
            table[str(path)] = pin
            before[str(path)] = file_seal(path,pin,budget.guard)
        budget.aliases = plan["environment"]["imported_alias_to_resolved_target"]
        env = environment(budget.guard)
        require(equal(env,plan["environment"]), "actual imported CPU Torch/NumPy/PIL providers differ from prepared grant")
        import numpy as np
        import PIL
        require(str(Path(np.__file__).resolve()) == env["base"]["origins"]["numpy"]
            and str(Path(PIL.__file__).resolve()) == env["base"]["origins"]["PIL"], "actual numeric/imaging origins differ")
        from portable_e2e import map_native_pair_reader_20261008_v3 as native
        from portable_e2e import map_crop_batch_worker_20261008_v3 as worker
        from portable_e2e import map_serial_training_input_adapter_20261008_v1 as serial
        from portable_e2e import map_torch_cpu_input_bridge_20261012_v1 as bridge
        reference = reference_function(read_bounded(ROOT/REFERENCE),np)
        crop = worker.compile_original_crop(ROOT/"tools/e2e/map_appearance_matched_contract_20261001.py")
        for mode in ARMS:
            value = ledger()
            directory = OUT/mode
            directory.mkdir(exist_ok=False)
            reader = native.BoundedNativePairReader(DATA/"original_train729.npz",INPUTS[str(DATA/"original_train729.npz")],
                DATA/"paired_train480.npz",INPUTS[str(DATA/"paired_train480.npz")],checkpoint=budget.guard,cache_enabled=True)
            adapter = serial.SerialTrainingInputAdapter([item["baseline"] for item in plan["selected_baseline_rows"]],
                reader,crop,mode=mode,checkpoint=budget.guard)
            owner = adapter.stats_snapshot()
            native_writer = Writer(directory/"exposures.jsonl",NATIVE_LIMIT,budget.guard)
            tensor_writer = Writer(directory/"tensor_receipts.jsonl",TENSOR_LIMIT,budget.guard)
            rescued = 0
            for step in STEPS:
                active = True
                tail = dict(arm=mode,step=step,phase="produce",serial_receipt=None,bridge_receipt=None)
                budget.guard()
                source = adapter.next_batch()
                require(type(source.step) is int and source.step == step, "actual produced chronology differs")
                build = lambda item:bridge.build_cpu_tensor_batch(item,expected_step=step,expected_mode=mode,
                    expected_baseline=plan["selected_baseline_rows"][step-1]["baseline"],owner_pid=owner["owner_PID"],
                    owner_thread_ident=owner["owner_thread_ident"],checkpoint=budget.guard)
                verify = lambda item:bridge.verify_cpu_tensor_batch(item,checkpoint=budget.guard)
                reference_call = lambda item,bundle:reference_check(item,bundle,reference,budget.guard)
                ack = process_produced(source,mode,plan["selected_baseline_rows"][step-1]["baseline"],adapter,
                    native_writer,tensor_writer,value,build,verify,reference_call,budget.guard,tail)
                arms[mode]["acknowledgements"].append(ack)
                if mode == ARMS[1]:
                    rescued += sum(source.receipt["candidate_worker_receipt"]["journal"]["rescued"])
                arms[mode]["pipeline"] = snapshot(value)
                arms[mode]["observed_rescued_slots"] = rescued
                active,tail = False,None
                del source
                budget.guard()
            close_one(native_writer,"native_journal_close",errors)
            close_one(tensor_writer,"tensor_journal_close",errors)
            arms[mode]["native"],arms[mode]["tensor"] = native_writer.snapshot(),tensor_writer.snapshot()
            native_writer = tensor_writer = None
            require(not errors, "journal closing failed; preceding ACKs remain ACKed")
            close_one(adapter,"serial_adapter_close",errors)
            arms[mode]["adapter"] = adapter.stats_snapshot()
            pure.adapter_closed(arms[mode]["adapter"])
            arms[mode]["reader_closing_errors"] = deepcopy(reader.closing_errors)
            arms[mode]["reader_stats_after_close"] = reader.stats_snapshot()
            adapter = reader = None
            require(not errors, "adapter closing failed; do not add a duplicate batch failure")
    except BaseException as error:
        failure = safe_error("pipeline",error)
        if mode is not None and value is not None:
            if active and step is not None:
                failed(value,step)
                if adapter is not None and step > value["acknowledged_batches"]:
                    try:
                        adapter.abort(error,step=step)
                    except BaseException as secondary:
                        errors.append(safe_error("serial_abort_secondary",secondary))
            arms[mode]["pipeline"] = snapshot(value)
    finally:
        for resource,scope in ((native_writer,"native_close_secondary"),(tensor_writer,"tensor_close_secondary"),
            (adapter,"serial_close_secondary"),(reader,"reader_close_secondary")):
            close_one(resource,scope,errors)
        if mode is not None:
            if native_writer is not None:
                arms[mode]["native"] = closing_snapshot(native_writer,"snapshot","native_closing_snapshot",errors)
            if tensor_writer is not None:
                arms[mode]["tensor"] = closing_snapshot(tensor_writer,"snapshot","tensor_closing_snapshot",errors)
            if adapter is not None:
                arms[mode]["adapter"] = closing_snapshot(adapter,"stats_snapshot","adapter_closing_snapshot",errors)
                if arms[mode]["adapter"].get("cleanup_errors"):
                    errors.append(dict(scope="adapter_retained_cleanup",type="AdapterCleanupErrors",message="See complete adapter summary"))
            if reader is not None:
                try:
                    arms[mode]["reader_closing_errors"] = deepcopy(reader.closing_errors)
                except BaseException as error:
                    errors.append(safe_error("reader_closing_error_snapshot",error))
                arms[mode]["reader_stats_after_close"] = closing_snapshot(reader,"stats_snapshot","reader_closing_snapshot",errors)
        for path,pin in table.items():
            observation = dict(path=path,expected_sha256=pin,status="NOTCHECKED",before=before[path],after=None)
            try:
                observed = file_seal(path,pin,budget.guard)
                require(equal(observed,before[path]), "source/input/provider/archive generation changed during execution")
                observation.update(status="MATCHED",after=observed)
            except BaseException as error:
                observation["error"] = safe_error("closing_seal",error)
                errors.append(observation["error"])
                if failure is None:
                    failure = observation["error"]
            closing.append(observation)
    complete = failure is None and not errors and all(arms[mode]["pipeline"]["PASS_batches"] == 16 for mode in ARMS)
    result = dict(schema=SCHEMA+".result",status="COMPLETE_CPU_TORCH_INPUT_BOUNDARY_NOT_TRAINING" if complete else "FAILED_WITH_DECLARED_DENOMINATOR",
        ROOT_plan_sha256=plan_pin,ROOT_authority_sha256=grant_pin,contract=contract(),arms=arms,
        failure=failure,full_failed_tail=tail,secondary_closing_errors=errors,closing_seal_observations=closing,
        imported_Torch_library_aliases=plan["environment"]["imported_alias_to_resolved_target"],
        sampled_closing_memory=budget.last,sampled_maximum_VmHWM_bytes=budget.maximum_HWM,
        elapsed_seconds=time.monotonic()-budget.started,owner_guard_calls=budget.guard_calls,
        Torch_imported="torch" in sys.modules,
        Torch_CUDA_initialized=sys.modules["torch"].cuda.is_initialized() if "torch" in sys.modules else None,
        actual_trainer_connected=False,
        NN_forward=0,backward=0,optimizer_updates=0,training_steps=0,GPU=0,SSH=0,CARLA=0,
        main_approved=0,sub_approved=0,model_performance_gain_proven=False)
    write_new(OUT/"result.json",encoded(result),closing=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument("--mode",choices=("prepare","execute"),required=True)
    parser.add_argument("--plan-sha256")
    parser.add_argument("--authority-sha256")
    args = parser.parse_args()
    require(sys.flags.dont_write_bytecode and sys.flags.optimize == 0, "clean -B assertion-enabled owner required")
    with canonical(LOCK).open("rb") as lease:
        fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:
            budget = Budget(lease)
            result = prepare(budget) if args.mode == "prepare" else execute(args.plan_sha256,args.authority_sha256,budget)
            print(encoded(dict(status=result["status"],NN_forward=0,backward=0,optimizer_updates=0)).decode(),end="")
            return 0 if result["status"].startswith(("PREPARED","COMPLETE")) else 2
        finally:
            fcntl.flock(lease,fcntl.LOCK_UN)


if __name__ == "__main__":
    raise SystemExit(main())
