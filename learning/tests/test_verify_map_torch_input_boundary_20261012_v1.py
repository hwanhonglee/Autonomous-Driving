"""HH_261012 - Scalar/mock and tiny fictional-file seams for the CPU boundary owner.

ROOT alone runs these after reviewing both sources. No real packages, Torch
imports, tensors, models or images are created here. Real-shaped scalar receipt
envelopes exercise unchanged byte caps; fictional hashes are not numeric proof.
"""
import ast
from copy import deepcopy
import hashlib
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import verify_map_torch_input_boundary_20261012_v1 as run
from portable_e2e import map_torch_cpu_input_bridge_20261012_v1 as bridge


def baseline(step=1):
    return dict(step=step,physical_slot_indices=[1208,1207,1036,729,344,728,497,232],
        shared_crop_boxes=[[100,200,260,440] for _ in range(8)],actual_paired_TRUE_bits=[0]*8,
        RGB_sha256="a"*64,target_sha256="b"*64)


def slots(row):
    return [dict(position=position,slot=slot,native_RGB_sha256="c"*64,native_target_sha256="d"*64,
        reader_provenance_authenticated_by_worker=False) for position,slot in enumerate(row["physical_slot_indices"])]


def array_descriptor(shape,dtype,pin,maximum):
    amount = 1
    for item in shape:
        amount *= item
    return dict(shape=shape,dtype=dtype,nbytes=amount*({"uint8":1,"float32":4,"int64":8}[dtype]),
        owned=True,C_contiguous=True,finite=True,minimum=0.003921568859368563 if dtype == "float32" else 0,
        maximum=0.9960784316062927 if dtype == "float32" else maximum,sha256=pin)


def tensor_descriptor(shape,dtype,pin,maximum):
    source = array_descriptor(shape,dtype,pin,maximum)
    return dict(shape=shape,dtype=dtype,device="cpu",layout="strided",C_contiguous=True,
        strides=[691200,230400,640,1] if len(shape) == 4 else [230400,640,1],storage_offset=0,
        requires_grad=False,grad_fn_is_None=True,is_pinned=False,has_no_base=True,
        nbytes=source["nbytes"],storage_nbytes=source["nbytes"],minimum=source["minimum"],maximum=source["maximum"],
        finite=True,sha256=pin)


def shaped_metadata(step=1,mode=run.ARMS[0]):
    """Full published shape/key/scope widths, no allocated array or actual source claim."""
    row = baseline(step)
    witness = slots(row)
    rescued = [False]*8
    if step == 6: rescued[2] = True
    if step == 10: rescued[4] = True
    candidate = dict(schema="portable_e2e.map_crop_batch_worker.20261008.v3.batch",step=step,
        native_slots=deepcopy(witness),actual_baseline_RGB_and_GT_hashes_reproduced=True,
        candidate_hashes_computed_from_actual_owned_arrays=True,same_box_for_actual_RGB_and_GT=True,
        source_and_baseline_RAW_authenticated_by_worker=False,training_steps=0,NN=0,
        journal=dict(schema="portable_e2e.map_crop_preservation_journal.20261008.v1",
            policy_source_sha256="566b3e2afb7a0c879071c806c0cbfef6d31b8e974df99423339b7bf48cd6209f",
            baseline_exposure=deepcopy(row),selected_crop_boxes=[[0,0,360,640] if bit else box
                for bit,box in zip(rescued,row["shared_crop_boxes"])],
            full_GT_sign_pixels=[230400]*8,original_NEAREST_sign_pixels=[0 if bit else 230400 for bit in rescued],
            protected=[True,False]*4,rescued=rescued,
            candidate_RGB_sha256="e"*64 if any(rescued) else row["RGB_sha256"],
            candidate_target_sha256="f"*64 if any(rescued) else row["target_sha256"]))
    native_row = row if mode == run.ARMS[0] else candidate
    native_rgb = row["RGB_sha256"] if mode == run.ARMS[0] else candidate["journal"]["candidate_RGB_sha256"]
    native_gt = row["target_sha256"] if mode == run.ARMS[0] else candidate["journal"]["candidate_target_sha256"]
    native = dict(RGB=array_descriptor([8,360,640,3],"uint8",native_rgb,255),
        target=array_descriptor([8,360,640],"uint8",native_gt,6))
    host = dict(RGB=array_descriptor([8,3,360,640],"float32","1"*64,1),
        target=array_descriptor([8,360,640],"int64","2"*64,6))
    serial = dict(schema=bridge.serial.SCHEMA+".batch",mode=mode,step=step,provided_baseline=row,
        provided_baseline_sha256=run.metadata_sha(row),control_exposure=row if mode == run.ARMS[0] else None,
        candidate_worker_receipt=candidate if mode == run.ARMS[1] else None,native_slots=witness,
        native_arrays=native,host_arrays=host,scope=deepcopy(bridge.serial.SCOPE))
    tensors = dict(RGB=tensor_descriptor([8,3,360,640],"float32","1"*64,1),
        target=tensor_descriptor([8,360,640],"int64","2"*64,6))
    receipt = dict(schema=bridge.SCHEMA+".batch",step=step,mode=mode,source_receipt_sha256=run.metadata_sha(serial),
        provided_baseline_sha256=run.metadata_sha(row),native_arrays=native,host_arrays=host,tensor_arrays=tensors,
        copy_from_host_bytes_verified=True,scope=deepcopy(bridge.SCOPE))
    consumer = dict(consumer=bridge.CONSUMER,accepted=True,host_RGB_sha256="1"*64,host_target_sha256="2"*64)
    proof = dict(schema=bridge.SCHEMA+".proof",status="CPU_CLONE_COPY_VERIFIED_NOT_TRAINER_REFERENCE",step=step,mode=mode,
        source_receipt_sha256=receipt["source_receipt_sha256"],provided_baseline_sha256=receipt["provided_baseline_sha256"],
        native_and_host_buffers_rechecked=True,CPU_tensor_buffers_rechecked=True,copy_from_host_bytes_verified=True,
        tensor_arrays=deepcopy(tensors),consumer_receipt=consumer,scope=deepcopy(bridge.SCOPE))
    reference = dict(schema=run.SCHEMA+".trainer_CPU_reference",function="tensors",source_sha256=run.REFERENCE_PIN,device="cpu",
        all_RGB_float32_bits_equal=True,all_GT_int64_values_equal=True,RGB_sha256="1"*64,target_sha256="2"*64,
        original_trainer_module_imported=False,NN_forward=0,backward=0,optimizer_updates=0,GPU_numeric_equivalence_proven=False)
    return row,native_row,serial,receipt,proof,reference


class MockWriter:
    def __init__(self,fault=None):
        self.fault,self.count,self.rows = fault,0,[]
    def append(self,row):
        if self.fault == "before": raise RuntimeError("before append")
        self.rows.append(deepcopy(row))
        self.count += 1
        if self.fault == "flush": raise RuntimeError("after full write before flush")


class MockAdapter:
    def __init__(self,fail=False):
        self.fail,self.calls = fail,[]
    def acknowledge(self,step,*,consumer_receipt):
        self.calls.append(step)
        if self.fail: raise RuntimeError("consumer ACK rejected")
        return dict(step=step,consumer_receipt=consumer_receipt,native_and_host_buffers_rehashed=True,
            consumer_execution_authenticated=False)


def test_contract_is_new_FIRST16_CPU_TORCH_scope_not_old_training_grant():
    value = run.contract()
    assert value["source_count"] == len(run.SOURCES) == len(set(run.SOURCES)) == 19
    assert value["selected_provider_count"] == 12 and len(run.INPUTS) == 7
    assert value["native_files"] == value["tensor_receipt_files"] == 2
    assert value["original20k_batches_outside_declaration_per_arm"] == 19984
    assert value["Torch_imported"] is True and value["original_trainer_imported"] is False
    assert value["actual_trainer_connected"] is value["GPU_numeric_equivalence_proven"] is False
    assert value["NN_forward"] == value["backward"] == value["optimizer_updates"] == 0
    grant = run.authority("a"*64)
    assert grant["Torch_import_and_CPU_tensor_operations_authorized"] is True
    assert grant["training_authorized"] is grant["NN_forward_authorized"] is grant["backward_authorized"] is False
    assert grant["old_authority_reused"] is False
    assert "2026-10-12" in str(run.PLAN_DIR) and run.NAME == "map_torch_input_boundary_v1"


@pytest.mark.parametrize("mode",run.ARMS)
def test_real_shaped_scalar_receipts_fit_fixed_line_and_file_envelopes_without_duplicate_scopes(mode):
    native_total = tensor_total = rescued = 0
    for step in run.STEPS:
        original,native_row,serial,receipt,proof,reference = shaped_metadata(step,mode)
        assert set(receipt) == bridge.RECEIPT_FIELDS and set(proof) == bridge.PROOF_FIELDS
        assert all(set(item) == bridge.TENSOR_FIELDS for item in receipt["tensor_arrays"].values())
        compact = run.compact_proof(proof,receipt)
        reconstructed = {**{key:value for key,value in compact.items() if key != "full_proof_metadata_sha256"},
            "tensor_arrays":receipt["tensor_arrays"],"scope":receipt["scope"]}
        assert run.equal(reconstructed,proof) and run.metadata_sha(reconstructed) == compact["full_proof_metadata_sha256"]
        reconstructed_serial = dict(schema=bridge.serial.SCHEMA+".batch",step=step,mode=mode,provided_baseline=original,
            provided_baseline_sha256=receipt["provided_baseline_sha256"],control_exposure=native_row if mode == run.ARMS[0] else None,
            candidate_worker_receipt=native_row if mode == run.ARMS[1] else None,native_slots=serial["native_slots"],
            native_arrays=receipt["native_arrays"],host_arrays=receipt["host_arrays"],scope=serial["scope"])
        assert run.metadata_sha(reconstructed_serial) == receipt["source_receipt_sha256"]
        row = dict(schema=run.SCHEMA+".tensor_receipt",arm=mode,step=step,source_native_slots=serial["native_slots"],
            source_serial_scope=serial["scope"],native_record_metadata_sha256=run.metadata_sha(native_row),
            bridge_receipt=receipt,bridge_copy_proof=compact,trainer_CPU_reference_proof=reference,
            chronology="TENSOR_AND_REFERENCE_CHECKED_BOTH_METADATA_APPENDS_BEFORE_REVERIFY_AND_SERIAL_ACK")
        native_raw,tensor_raw = run.encoded(native_row),run.encoded(row)
        assert len(native_raw) <= run.LINE_LIMIT and len(tensor_raw) <= run.LINE_LIMIT
        native_total += len(native_raw)
        tensor_total += len(tensor_raw)
        if mode == run.ARMS[1]: rescued += sum(native_row["journal"]["rescued"])
    assert native_total <= run.NATIVE_LIMIT and tensor_total <= run.TENSOR_LIMIT
    assert rescued == (2 if mode == run.ARMS[1] else 0)  # Fictional metadata count, not actual RGB proof.


@pytest.mark.parametrize("fault",[None,"build","verify1","reference","native_before","native_flush",
    "postnative","tensor_before","tensor_flush","posttensor","verify2","ACK"])
def test_produced_checked_written_flushed_and_ACK_facts_are_distinct_on_every_failure_seam(fault):
    row,native_row,serial,receipt,proof,ref = shaped_metadata()
    source,bundle = SimpleNamespace(step=1,receipt=serial),SimpleNamespace(receipt=receipt)
    native = MockWriter("before" if fault == "native_before" else "flush" if fault == "native_flush" else None)
    tensor = MockWriter("before" if fault == "tensor_before" else "flush" if fault == "tensor_flush" else None)
    adapter,value,tail = MockAdapter(fault == "ACK"),run.ledger(),{}
    calls = dict(verify=0,guard=0)
    def build(_):
        if fault == "build": raise RuntimeError("build failure")
        return bundle
    def verify(_):
        calls["verify"] += 1
        if fault == "verify"+str(calls["verify"]): raise RuntimeError("verify failure")
        return deepcopy(proof)
    def reference(*_):
        if fault == "reference": raise RuntimeError("reference failure")
        return ref
    def guard():
        calls["guard"] += 1
        if fault == ("postnative" if calls["guard"] == 1 else "posttensor"): raise RuntimeError("guard after written row")
    if fault is None:
        observed = run.process_produced(source,run.ARMS[0],row,adapter,native,tensor,value,build,verify,reference,guard,tail)
        assert observed["actual_serial_ACK_received"] is True
        assert run.snapshot(value)["PASS_batches"] == 1 and calls["verify"] == 2
    else:
        with pytest.raises(RuntimeError):
            run.process_produced(source,run.ARMS[0],row,adapter,native,tensor,value,build,verify,reference,guard,tail)
        run.failed(value,1)
        assert run.snapshot(value)["PASS_batches"] == 0 and run.snapshot(value)["FAIL_batches"] == 1
        assert run.snapshot(value)["UNEXECUTED_batches"] == 15
    assert value["native_rows_written"] == native.count and value["tensor_rows_written"] == tensor.count
    assert value["acknowledged_batches"] == int(fault is None)


def test_final_ACK_then_guard_or_close_failure_does_not_create_seventeenth_failure():
    value = run.ledger()
    for step in run.STEPS:
        for key in run.FACTS: run.event(value,key,step)
    run.failed(value,16)
    observed = run.snapshot(value)
    assert (observed["PASS_batches"],observed["FAIL_batches"],observed["UNEXECUTED_batches"]) == (16,0,0)
    with pytest.raises(ValueError): run.failed(value,17)


def test_last_pending_ACK_denial_retains15_PASS1_FAIL_not16_PASS_plusFAIL():
    value = run.ledger()
    for step in range(1,16):
        for key in run.FACTS: run.event(value,key,step)
    for key in run.FACTS[:-1]: run.event(value,key,16)
    run.failed(value,16)
    observed = run.snapshot(value)
    assert (observed["PASS_batches"],observed["FAIL_batches"],observed["UNEXECUTED_batches"]) == (15,1,0)
    assert observed["produced_batches"] == observed["tensor_rows_written"] == 16


@pytest.mark.parametrize("fault",["extra","scope","descriptor","step","bool","consumerSHA"])
def test_compact_proof_and_consumer_do_not_accept_coercion_or_changed_joins(fault):
    _,_,_,receipt,proof,_ = shaped_metadata()
    if fault == "extra": proof["trainer_connected"] = True
    elif fault == "scope": proof["scope"]["TRAIN_admitted_by_bridge"] = True
    elif fault == "descriptor": proof["tensor_arrays"]["RGB"]["sha256"] = "f"*64
    elif fault == "step": proof["step"] = True
    elif fault == "bool": proof["CPU_tensor_buffers_rechecked"] = 1
    else: proof["consumer_receipt"]["host_RGB_sha256"] = "f"*64
    with pytest.raises(ValueError):
        if fault == "consumerSHA": run.validate_consumer(proof["consumer_receipt"],receipt)
        else: run.compact_proof(proof,receipt)


def test_exact_reference_AST_never_runs_trainer_module_body(monkeypatch):
    raw = b'raise RuntimeError("whole trainer body ran")\ndef tensors(rgb,target,device):\n    return rgb,target,BATCH\n'
    monkeypatch.setattr(run,"REFERENCE_PIN",hashlib.sha256(raw).hexdigest())
    fn = run.reference_function(raw,None)
    assert fn("r","t","cpu") == ("r","t",8)
    with pytest.raises(ValueError): run.reference_function(raw+b"#changed",None)


def test_reference_duplicate_function_denied_before_compilation(monkeypatch):
    raw = b'def tensors(rgb,target,device):\n return 1\ndef tensors(rgb,target,device):\n return 2\n'
    monkeypatch.setattr(run,"REFERENCE_PIN",hashlib.sha256(raw).hexdigest())
    with pytest.raises(ValueError): run.reference_function(raw,None)


def test_closing_helpers_attempt_BaseException_cleanup_and_retain_secondary_snapshot_errors():
    calls,errors = [],[]
    class Bad:
        def close(self):
            calls.append("close")
            raise KeyboardInterrupt("first close failure")
        def snapshot(self): raise SystemExit("snapshot secondary")
    run.close_one(Bad(),"first",errors)
    run.close_one(Bad(),"second",errors)
    observed = run.closing_snapshot(Bad(),"snapshot","metadata",errors)
    assert calls == ["close","close"] and [item["scope"] for item in errors] == ["first","second","metadata"]
    assert observed["status"] == "NOTCHECKED" and errors[0]["type"] == "KeyboardInterrupt"


@pytest.fixture
def owner_files(tmp_path,monkeypatch):
    output,plan = tmp_path/"output",tmp_path/"plan"
    output.mkdir()
    plan.mkdir()
    monkeypatch.setattr(run,"OUT",output)
    monkeypatch.setattr(run,"PLAN_DIR",plan)
    return output


def test_tiny_fictional16_row_writer_createonly_closing_rawSHA_generation(owner_files):
    path = owner_files/"fictional.jsonl"
    writer = run.Writer(path,run.NATIVE_LIMIT,lambda:None)
    expected = b"".join(run.encoded(dict(step=step,fictional=True)) for step in run.STEPS)
    for step in run.STEPS: writer.append(dict(step=step,fictional=True))
    writer.close()
    observed = writer.snapshot()
    assert observed["rows_written"] == observed["flushed_rows"] == 16 and observed["closed"] is True
    assert observed["closing_check"]["sha256"] == hashlib.sha256(expected).hexdigest()
    assert observed["closing_check"]["generation"] == run.pure.old.generation(path.stat())
    assert writer.stream.closed and run.read_bounded(path) == expected
    with pytest.raises(ValueError): run.Writer(path,run.NATIVE_LIMIT,lambda:None)


def test_same_inode_same_length_tamper_denied_and_ownedFD_closed(owner_files):
    path = owner_files/"tamper.jsonl"
    writer = run.Writer(path,run.NATIVE_LIMIT,lambda:None)
    writer.append(dict(step=1))
    inode,amount = path.stat().st_ino,path.stat().st_size
    with path.open("r+b") as stream: stream.write(run.encoded(dict(step=9)))
    assert path.stat().st_ino == inode and path.stat().st_size == amount
    with pytest.raises(ValueError): writer.close()
    assert writer.stream.closed and writer.snapshot()["closing_check"]["status"] == "FAILED_CHECKING"


@pytest.mark.parametrize("fault",["guard","line","quota"])
def test_writer_limits_and_prewrite_guard_leave_zero_written_rows(owner_files,monkeypatch,fault):
    def guard():
        if fault == "guard": raise RuntimeError("prewrite denial")
    writer = run.Writer(owner_files/(fault+".jsonl"),run.NATIVE_LIMIT,guard)
    if fault == "quota": monkeypatch.setattr(run,"owned_bytes",lambda:run.LIMIT-run.CLOSING)
    row = dict(step=1,padding="q"*run.LINE_LIMIT) if fault == "line" else dict(step=1)
    with pytest.raises((ValueError,RuntimeError)): writer.append(row)
    assert writer.count == writer.bytes_written == writer.flushed_rows == 0
    if fault == "guard":
        with pytest.raises(RuntimeError): writer.close()
        assert writer.snapshot()["closing_check"]["status"] == "FAILED_NOTCHECKED"
    else: writer.close()
    assert writer.stream.closed


def test_writer_full_write_then_flush_failure_preserves_distinct_counts_and_closes(owner_files):
    writer = run.Writer(owner_files/"flush.jsonl",run.NATIVE_LIMIT,lambda:None)
    actual = writer.stream
    class OnceBadFlush:
        def __init__(self): self.failed = False
        def __getattr__(self,name): return getattr(actual,name)
        def flush(self):
            if not self.failed:
                self.failed = True
                raise RuntimeError("flush failed after write")
            return actual.flush()
    writer.stream = OnceBadFlush()
    with pytest.raises(RuntimeError): writer.append(dict(step=1))
    assert writer.snapshot()["rows_written"] == 1 and writer.snapshot()["flushed_rows"] == 0
    assert writer.snapshot()["bytes_written"] == len(run.encoded(dict(step=1)))
    writer.close()
    assert writer.snapshot()["flushed_rows"] == 1 and actual.closed


def test_bounded_metadata_reader_refuses_excess_without_unbounded_read(owner_files):
    path = owner_files/"metadata.json"
    path.write_bytes(b'{"fictional":true}\n')
    assert run.read_bounded(path,64) == b'{"fictional":true}\n'
    with pytest.raises(ValueError): run.read_bounded(path,3)


def test_explicit_system_library_alias_seam_accepts_named_targets_then_rejects_retarget(tmp_path):
    directory = tmp_path/"torch"
    (directory/"lib").mkdir(parents=True)
    for name in ("libtorch_cpu.so","libtorch_python.so","libc10.so"):
        target = tmp_path/(name+".fictional")
        target.write_bytes(b"fictional provider metadata\n")
        (directory/"lib"/name).symlink_to(target)
    aliases = run.torch_aliases(directory)
    run.check_aliases(aliases)
    alias = directory/"lib/libc10.so"
    alias.unlink()
    replacement = tmp_path/"replacement.fictional"
    replacement.write_bytes(b"different fictional provider\n")
    alias.symlink_to(replacement)
    with pytest.raises(ValueError): run.check_aliases(aliases)


def test_source_does_not_import_or_run_trainer_NN_GPU_or_old_scoped_helpers():
    tree = ast.parse(Path(run.__file__).read_bytes())
    imports = {item.module for item in ast.walk(tree) if isinstance(item,ast.ImportFrom)}
    assert not any(value and ("train_map_native_fusion" in value or value in ("torch.nn","carla","rclpy")) for value in imports)
    calls = {ast.unparse(item.func) for item in ast.walk(tree) if isinstance(item,ast.Call)}
    assert not calls & {"pure.Budget","pure.Writer","pure.execute","pure.authority","pure.old.execute",
        "pure.old.Budget","torch.cuda.is_available","torch.cuda.init","torch.cuda.empty_cache","torch.load","np.load",
        "optimizer.step","loss.backward","model"}
    assert "reference_function" in calls and "process_produced" in calls
    assert not any(isinstance(item,ast.Assign) and any(isinstance(target,ast.Attribute)
        and ast.unparse(target).startswith("pure.") for target in item.targets) for item in ast.walk(tree))
