"""HH_261012 - Scalar-only saved CPU-boundary joins for ROOT; no arrays, Torch or real archives.

Fictional hashes and scalar descriptors exercise the checker contract, not
numeric evidence. ROOT alone runs this suite after reading both complete files.
"""
import ast
from copy import deepcopy
from pathlib import Path

import pytest

from scripts.e2e import check_map_torch_input_boundary_saved_20261012_v1 as check


def original(step=1):
    return dict(step=step,physical_slot_indices=[1208,1207,1036,729,344,728,497,232],
        shared_crop_boxes=[[100,200,260,440] for _ in range(8)],actual_paired_TRUE_bits=[0]*8,
        RGB_sha256="a"*64,target_sha256="b"*64)


def descriptor(shape,dtype,pin,maximum):
    amount = 1
    for value in shape:
        amount *= value
    return dict(shape=shape,dtype=dtype,nbytes=amount*{"uint8":1,"float32":4,"int64":8}[dtype],
        owned=True,C_contiguous=True,finite=True,minimum=0.0 if dtype == "float32" else 0,
        maximum=float(maximum) if dtype == "float32" else maximum,sha256=pin)


def tensor(host):
    return dict(shape=host["shape"],dtype=host["dtype"],device="cpu",layout="strided",C_contiguous=True,
        strides=[691200,230400,640,1] if len(host["shape"]) == 4 else [230400,640,1],storage_offset=0,
        requires_grad=False,grad_fn_is_None=True,is_pinned=False,has_no_base=True,nbytes=host["nbytes"],
        storage_nbytes=host["nbytes"],minimum=host["minimum"],maximum=host["maximum"],finite=True,sha256=host["sha256"])


def record(mode=check.ARMS[0],step=1):
    row = original(step)
    slots = [dict(position=position,slot=slot,native_RGB_sha256="c"*64,native_target_sha256="d"*64,
        reader_provenance_authenticated_by_worker=False) for position,slot in enumerate(row["physical_slot_indices"])]
    rescued,full,cropped = [False]*8,[0]*8,[0]*8
    if step in (6,10):
        position,amount = (2,56) if step == 6 else (4,37)
        rescued[position],full[position] = True,amount
    journal = dict(schema="portable_e2e.map_crop_preservation_journal.20261008.v1",policy_source_sha256=check.POLICY_PIN,
        baseline_exposure=row,selected_crop_boxes=[[0,0,360,640] if rescue else proposal
            for rescue,proposal in zip(rescued,row["shared_crop_boxes"])],full_GT_sign_pixels=full,
        original_NEAREST_sign_pixels=cropped,protected=[True,False]*4,rescued=rescued,
        candidate_RGB_sha256="e"*64 if any(rescued) else row["RGB_sha256"],
        candidate_target_sha256="f"*64 if any(rescued) else row["target_sha256"])
    worker = dict(schema="portable_e2e.map_crop_batch_worker.20261008.v3.batch",step=step,journal=journal,native_slots=slots,
        actual_baseline_RGB_and_GT_hashes_reproduced=True,candidate_hashes_computed_from_actual_owned_arrays=True,
        same_box_for_actual_RGB_and_GT=True,source_and_baseline_RAW_authenticated_by_worker=False,training_steps=0,NN=0)
    native_row = row if mode == check.ARMS[0] else worker
    native = dict(RGB=descriptor([8,360,640,3],"uint8",row["RGB_sha256"] if mode == check.ARMS[0] else journal["candidate_RGB_sha256"],255),
        target=descriptor([8,360,640],"uint8",row["target_sha256"] if mode == check.ARMS[0] else journal["candidate_target_sha256"],6))
    host = dict(RGB=descriptor([8,3,360,640],"float32","1"*64,1),target=descriptor([8,360,640],"int64","2"*64,6))
    tensors = {key:tensor(value) for key,value in host.items()}
    serial = dict(schema=check.SERIAL+".batch",mode=mode,step=step,provided_baseline=row,
        provided_baseline_sha256=check.metadata_sha(row),control_exposure=row if mode == check.ARMS[0] else None,
        candidate_worker_receipt=worker if mode == check.ARMS[1] else None,native_slots=slots,
        native_arrays=native,host_arrays=host,scope=deepcopy(check.SERIAL_SCOPE))
    receipt = dict(schema=check.BRIDGE+".batch",step=step,mode=mode,source_receipt_sha256=check.metadata_sha(serial),
        provided_baseline_sha256=check.metadata_sha(row),native_arrays=native,host_arrays=host,tensor_arrays=tensors,
        copy_from_host_bytes_verified=True,scope=deepcopy(check.BRIDGE_SCOPE))
    consumer = dict(consumer=check.CONSUMER,accepted=True,host_RGB_sha256="1"*64,host_target_sha256="2"*64)
    proof = dict(schema=check.BRIDGE+".proof",status="CPU_CLONE_COPY_VERIFIED_NOT_TRAINER_REFERENCE",step=step,mode=mode,
        source_receipt_sha256=receipt["source_receipt_sha256"],provided_baseline_sha256=receipt["provided_baseline_sha256"],
        native_and_host_buffers_rechecked=True,CPU_tensor_buffers_rechecked=True,copy_from_host_bytes_verified=True,
        tensor_arrays=tensors,consumer_receipt=consumer,scope=deepcopy(check.BRIDGE_SCOPE))
    compact = {key:value for key,value in proof.items() if key not in ("tensor_arrays","scope")}
    compact["full_proof_metadata_sha256"] = check.metadata_sha(proof)
    ref = dict(schema=check.PRODUCER+".trainer_CPU_reference",function="tensors",source_sha256=check.REFERENCE_PIN,device="cpu",
        all_RGB_float32_bits_equal=True,all_GT_int64_values_equal=True,RGB_sha256="1"*64,target_sha256="2"*64,
        original_trainer_module_imported=False,NN_forward=0,backward=0,optimizer_updates=0,GPU_numeric_equivalence_proven=False)
    value = dict(schema=check.PRODUCER+".tensor_receipt",arm=mode,step=step,source_native_slots=slots,
        source_serial_scope=deepcopy(check.SERIAL_SCOPE),native_record_metadata_sha256=check.metadata_sha(native_row),
        bridge_receipt=receipt,bridge_copy_proof=compact,trainer_CPU_reference_proof=ref,
        chronology="TENSOR_AND_REFERENCE_CHECKED_BOTH_METADATA_APPENDS_BEFORE_REVERIFY_AND_SERIAL_ACK")
    inner = dict(step=step,consumer_receipt=consumer,native_and_host_buffers_rehashed=True,consumer_execution_authenticated=False)
    ack = dict(step=step,actual_serial_ACK_received=True,serial_acknowledgement=inner,
        full_bridge_reverification_metadata_sha256=check.metadata_sha(proof),trainer_CPU_reference_proof_sha256=check.metadata_sha(ref))
    return row,native_row,value,ack,inner


@pytest.mark.parametrize("mode,step",[(check.ARMS[0],1),(check.ARMS[1],1),(check.ARMS[1],6),(check.ARMS[1],10)])
def test_complete_scalar_serial11_bridge12_reference_and_actual_ACK_join(mode,step):
    row,native,value,ack,inner = record(mode,step)
    check.baseline(row,step)
    checked,expected,expected_inner = check.tensor_record(value,row,mode,native)
    check.acknowledgement(ack,expected)
    assert check.same(inner,expected_inner) and checked["step"] == step
    assert checked["rescued_slots"] == ([] if mode == check.ARMS[0] or step == 1 else [[6,2,1036,56]] if step == 6 else [[10,4,344,37]])


@pytest.mark.parametrize("fault",["step_bool","TRUE","slot_float","box_bool","SHA_bytes","source11_SHA","scope_training",
    "tensor_device","tensor_gradient","tensor_stride","tensor_storage_bool","tensor_SHA","float_min_int",
    "copy_bool_int","proof_SHA","consumer_not_accepted","reference_GPU","reference_source","chronology","extra_receipt"])
def test_scalar_metadata_coercion_forgery_and_changed_CPU_joins_rejected(fault):
    row,native,value,_,_ = record()
    receipt = value["bridge_receipt"]
    if fault == "step_bool": value["step"] = True
    elif fault == "TRUE": row["actual_paired_TRUE_bits"][0] = 1
    elif fault == "slot_float": value["source_native_slots"][0]["slot"] = 1208.0
    elif fault == "box_bool": row["shared_crop_boxes"][0][0] = True
    elif fault == "SHA_bytes": row["RGB_sha256"] = b"a"*64
    elif fault == "source11_SHA": receipt["source_receipt_sha256"] = "f"*64
    elif fault == "scope_training": receipt["scope"]["TRAIN_admitted_by_bridge"] = True
    elif fault == "tensor_device": receipt["tensor_arrays"]["RGB"]["device"] = "cuda:0"
    elif fault == "tensor_gradient": receipt["tensor_arrays"]["RGB"]["requires_grad"] = True
    elif fault == "tensor_stride": receipt["tensor_arrays"]["RGB"]["strides"][0] += 1
    elif fault == "tensor_storage_bool": receipt["tensor_arrays"]["RGB"]["storage_offset"] = False
    elif fault == "tensor_SHA": receipt["tensor_arrays"]["RGB"]["sha256"] = "f"*64
    elif fault == "float_min_int": receipt["host_arrays"]["RGB"]["minimum"] = 0
    elif fault == "copy_bool_int": receipt["copy_from_host_bytes_verified"] = 1
    elif fault == "proof_SHA": value["bridge_copy_proof"]["full_proof_metadata_sha256"] = "f"*64
    elif fault == "consumer_not_accepted": value["bridge_copy_proof"]["consumer_receipt"]["accepted"] = False
    elif fault == "reference_GPU": value["trainer_CPU_reference_proof"]["GPU_numeric_equivalence_proven"] = True
    elif fault == "reference_source": value["trainer_CPU_reference_proof"]["source_sha256"] = "f"*64
    elif fault == "chronology": value["chronology"] = "WRITTEN_COUNTS_AS_CONSUMED"
    else: receipt["trainer_connected"] = True
    with pytest.raises(ValueError):
        check.baseline(row,1)
        check.tensor_record(value,row,check.ARMS[0],native)


@pytest.mark.parametrize("fault",["parity","selected_box","GT_bool","worker_training","nonrescue_changed_SHA"])
def test_candidate_policy_and_actual_rescue_metadata_not_relabelled(fault):
    row,native,value,_,_ = record(check.ARMS[1],6 if fault != "nonrescue_changed_SHA" else 1)
    if fault == "parity": native["journal"]["protected"][2] = False
    elif fault == "selected_box": native["journal"]["selected_crop_boxes"][2] = row["shared_crop_boxes"][2]
    elif fault == "GT_bool": native["journal"]["full_GT_sign_pixels"][2] = True
    elif fault == "worker_training": native["training_steps"] = 1
    else: native["journal"]["candidate_RGB_sha256"] = "e"*64
    with pytest.raises(ValueError): check.candidate(native,row)


@pytest.mark.parametrize("fault",["ACK_false","ACK_step_bool","bridge_recheck_SHA","reference_SHA","consumer_host_SHA"])
def test_actual_ACK_cannot_be_replaced_by_saved_production_or_reference_dict(fault):
    _,_,_,ack,_ = record()
    changed = deepcopy(ack)
    if fault == "ACK_false": changed["actual_serial_ACK_received"] = False
    elif fault == "ACK_step_bool": changed["step"] = True
    elif fault == "bridge_recheck_SHA": changed["full_bridge_reverification_metadata_sha256"] = "f"*64
    elif fault == "reference_SHA": changed["trainer_CPU_reference_proof_sha256"] = "f"*64
    else: changed["serial_acknowledgement"]["consumer_receipt"]["host_RGB_sha256"] = "f"*64
    with pytest.raises(ValueError): check.acknowledgement(changed,ack)


def summary_fixture():
    selected,acks = [],[]
    for step in check.STEPS:
        row,_,_,_,inner = record(step=step)
        selected.append(dict(step=step,RAW_line_sha256="9"*64,baseline=row))
        acks.append(inner)
    value = dict(schema=check.SERIAL+".summary",mode=check.ARMS[0],declared_steps=list(check.STEPS),closed=True,
        status="COMPLETE_ACKNOWLEDGED_INPUTS_NOT_TRAINING_OR_MODEL_APPROVAL",scope=deepcopy(check.SERIAL_SCOPE),
        expected_batches=16,PASS_batches=16,FAIL_batches=0,UNEXECUTED_batches=0,produced_batches=16,
        production_attempts=16,production_FAIL_batches=0,production_UNEXECUTED_batches=0,consumption_attempts=16,
        consumption_attempted_batches=16,consumption_PASS_batches=16,consumption_FAIL_batches=0,
        consumption_UNEXECUTED_batches=0,actual_native_read_attempts=128,original20000_batches_outside_declaration=19984,
        pending_step=None,failure=None,failed_tail=None,cleanup_errors=[],reader_closing_observations=[],acknowledgements=acks,
        provided_rows_sha256=check.metadata_sha([row["baseline"] for row in selected]),owner_PID=7,owner_thread_ident=11)
    return value,selected,acks


def test_full16_actual_ACK_summary_and_pipeline_not_produced_only():
    value,selected,acks = summary_fixture()
    check.adapter_summary(value,check.ARMS[0],selected,acks)
    check.pipeline(dict(expected_batches=16,produced_batches=16,tensor_checked_batches=16,native_rows_written=16,
        tensor_rows_written=16,acknowledged_batches=16,failed_step=None,PASS_batches=16,FAIL_batches=0,
        UNEXECUTED_batches=0,original20k_batches_outside_declaration=19984))


@pytest.mark.parametrize("fault",["owner_bool","missing_ACK","unconsumed","pending","native_reads","cleanup"])
def test_scalar_summary_failure_tail_and_consumption_denominator_are_not_hidden(fault):
    value,selected,acks = summary_fixture()
    if fault == "owner_bool": value["owner_PID"] = True
    elif fault == "missing_ACK": value["acknowledgements"] = value["acknowledgements"][:-1]
    elif fault == "unconsumed": value["consumption_PASS_batches"] = 15
    elif fault == "pending": value["pending_step"] = 16
    elif fault == "native_reads": value["actual_native_read_attempts"] = 127
    else: value["cleanup_errors"] = [{"error":"fictional retained error"}]
    with pytest.raises(ValueError): check.adapter_summary(value,check.ARMS[0],selected,acks)


@pytest.mark.parametrize("raw",[b'{"k":1,"k":2}',b'{"k":NaN}',b'{"k":Infinity}',b'{"k":1e999}'])
def test_duplicate_or_nonfinite_JSON_is_denied(raw):
    with pytest.raises(ValueError): check.decode(raw)


def test_strict_int64_ns_and_new_declaration_scope_preserve_RAW_types():
    exact = 1780000000123456789
    assert check.decode(check.encoded({"mtime_ns":exact}))["mtime_ns"] == exact
    assert check.integer(exact) and not check.integer(True) and not check.same(1,True) and not check.same(1,1.0)
    assert len(check.SOURCE_NAMES) == 19 and len(check.INPUTS) == 7
    value = check.contract()
    assert value["selected_provider_count"] == 12 and value["original20k_batches_outside_declaration_per_arm"] == 19984
    assert value["Torch_imported"] is True and value["actual_trainer_connected"] is False
    assert value["NN_forward"] == value["backward"] == value["training_steps"] == 0
    grant = check.authority("a"*64)
    assert grant["Torch_import_and_CPU_tensor_operations_authorized"] is True and grant["NN_forward_authorized"] is False


def test_checker_SOURCE_only_imports_stdlib_and_never_replays_numeric_reference():
    tree = ast.parse(Path(check.__file__).read_bytes())
    allowed = {"__future__","argparse","fcntl","hashlib","json","math","os","pathlib","stat","sys","time"}
    for node in ast.walk(tree):
        if isinstance(node,ast.Import): assert all(alias.name.split(".")[0] in allowed for alias in node.names)
        elif isinstance(node,ast.ImportFrom): assert node.module in allowed
    names = {ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node,ast.Call)}
    assert not names & {"exec","eval","compile","np.load","torch.load","reference_function","model","loss.backward"}
    assert "xb" in Path(check.__file__).read_text() and "original_trainer_CPU_reference_independently_replayed=False" in Path(check.__file__).read_text()
