"""HH_261005 - Scalar bytes/source fixtures only; never import Torch/NumPy, read weights/engines/cameras, compile, invoke CUDA, or start a native child."""

import ast
import hashlib
from pathlib import Path
import struct
import sys

import pytest

from portable_e2e import temporal_cached_tensorrt_backend_v1_20261005 as backend


@pytest.mark.parametrize("count",(6,12,18,24,30))
def test_all_warm_missing_slots_preserve_group_order_and_explicit_native_calls(count):
    assert backend.native_batches(count,cold=False)==(6,)*(count//6)
    assert sum(backend.native_batches(count,cold=False))==count


def test_cold_all30_not_five_warm_calls_or_cropped():
    assert backend.native_batches(30,cold=True)==(30,)
    with pytest.raises(RuntimeError):backend.native_batches(6,cold=True)


@pytest.mark.parametrize("count",(True,False,0,1,5,7,31,6.0,"6"))
def test_nonoriginal_CNN_batch_denied(count):
    with pytest.raises(RuntimeError):backend.native_batches(count,cold=False)


@pytest.mark.parametrize("batch",(6,30))
def test_exact_le_byte_request_size_source_digest_and_large_sequence(batch):
    sequence=2**63+123;source=hashlib.sha256(b"synthetic source only").digest()
    request=backend.request_header(batch,sequence,source)
    assert len(request)==56 and backend.REQUEST.unpack(request)==(backend.MAGIC,1,sequence,batch,batch*3*180*320*4,source)


@pytest.mark.parametrize("sequence",(True,False,0,-1,2**64,1.0))
def test_bad_request_ordinal_denied_without_transport(sequence):
    with pytest.raises(RuntimeError):backend.request_header(6,sequence,b"x"*32)


def response_fixture(batch=6,sequence=1,status=1):
    source=hashlib.sha256(b"same complete synthetic normalized input").digest()
    raw=b"\0"*(batch*60*128*4) if status==1 else b"retained synthetic failure"
    header=backend.RESPONSE.pack(backend.MAGIC,1,sequence,batch,len(raw),status,source,hashlib.sha256(raw).digest())
    return header,raw,source


def test_source_bound_finite_features_and_full_fixed_size():
    header,raw,source=response_fixture()
    assert len(header)==92 and backend.decode_response(header,raw,sequence=1,batch=6,input_sha256=source)==raw


@pytest.mark.parametrize("fault",("sequence","batch","source","length","checksum","NaN","status","magic","version"))
def test_packet_stale_mismatch_mutation_nonfinite_denied(fault):
    header,raw,source=response_fixture();values=list(backend.RESPONSE.unpack(header))
    if fault=="sequence":values[2]+=1
    if fault=="batch":values[3]=30
    if fault=="source":values[6]=b"different".ljust(32,b"x")
    if fault=="length":values[4]-=4
    if fault=="checksum":values[7]=b"x"*32
    if fault=="NaN":raw=struct.pack("<f",float("nan"))+raw[4:];values[7]=hashlib.sha256(raw).digest()
    if fault=="status":values[5]=3
    if fault=="magic":values[0]+=1
    if fault=="version":values[1]+=1
    with pytest.raises(RuntimeError):backend.decode_response(backend.RESPONSE.pack(*values),raw,sequence=1,batch=6,input_sha256=source)


@pytest.mark.parametrize("value",(float("nan"),float("inf"),float("-inf")))
def test_scalar_all_FP32_values_finite_not_just_checksum(value):
    with pytest.raises(RuntimeError):backend.finite_float32(struct.pack("<ff",1.0,value))


def test_failed_native_request_preserved_not_relabelled_features():
    header,raw,source=response_fixture(status=2)
    with pytest.raises(RuntimeError,match="retained synthetic failure"):
        backend.decode_response(header,raw,sequence=1,batch=6,input_sha256=source)


@pytest.mark.parametrize("field",("pid","ppid","pgid","sid","start_ticks","argv","state"))
def test_owned_native_generation_argv_or_dead_identity_fault_is_denied(field):
    core=dict(pid=123,ppid=111,pgid=123,sid=123,start_ticks=456)
    observed=dict(pid=123,ppid=111,pgid=123,sid=123,start_ticks=456,argv=["exact-native"],state="S")
    backend.verify_native_identity(observed,core,["exact-native"])
    observed[field]="Z" if field=="state" else (["foreign"] if field=="argv" else observed[field]+1)
    with pytest.raises(RuntimeError):backend.verify_native_identity(observed,core,["exact-native"])


def test_actual_entry_unconditionally_denied_before_model_attributes_Torch_OR_native_IO(monkeypatch):
    before=set(sys.modules)
    monkeypatch.setattr(backend.os,"write",lambda *args:pytest.fail("source-only entry cannot write a native pipe"))
    monkeypatch.setattr(backend.os,"read",lambda *args:pytest.fail("source-only entry cannot read a native pipe"))
    monkeypatch.setattr(backend,"require_parent_sources",lambda:pytest.fail("actual denial precedes model/source admission"))
    with pytest.raises(RuntimeError,match="unconditionally denied"):backend.PreparedTensorRTClientV1(True,ROOT_approved=True)
    with pytest.raises(RuntimeError,match="unconditionally denied"):backend.create_backend_model(object(),object())
    with pytest.raises(RuntimeError,match="unconditionally denied"):backend._bounded_pipe(9,b"x",writing=True,deadline=9999999999)
    client=object.__new__(backend.PreparedTensorRTClientV1)
    with pytest.raises(RuntimeError,match="unconditionally denied"):client.encode(object(),cold=True)
    with pytest.raises(RuntimeError,match="unconditionally denied"):client._request(object(),6)
    assert not any(name in set(sys.modules)-before for name in ("torch","numpy","onnx","tensorrt"))


def test_complete_original_cache_forward_inverse_two_sites_and_no_trust_forgery():
    proof=backend.cache_inverse_ast_proof()
    assert proof["two_CNN_sites_only"] and proof["original_cache_inverse_equal"]
    assert proof["ordinary_request_None_uses_original_CPU"] and not proof["masked_copy_trust_forged"]
    assert not proof["whole888_strict5head_numerical_parity_verified"]


def test_original_cached_route_fusion_hook_math_not_overridden_and101seal_uses_original_hash_once():
    tree=ast.parse(Path(backend.__file__).read_text())
    model=next(node for node in ast.walk(tree) if isinstance(node,ast.ClassDef) and node.name=="TensorRTCachedRouteRecoveryV1")
    methods=[node for node in model.body if isinstance(node,ast.FunctionDef)]
    assert [node.name for node in methods]==["_state_identity"]
    assert ast.unparse(methods[0]).count("super()._state_identity()") == 1
    source=Path(backend.__file__).read_text()
    assert "len(state)==101" in source and "pointers==" in source and "callbacks_and_cache(candidate)" in source
    assert "__class__ =" not in source and "_trusted_readonly_encoder =" not in source
    assert "result = result.transpose(1,2).contiguous().transpose(1,2)" in source


def test_native_source_guards_onlyCNN_FIFO_FP32_engine_size_source_sha_and_deadlines():
    source=(backend.ROOT/"tools/e2e/good6000_cnn_tensorrt_runtime_v1_20261005.cpp").read_text()
    for name in ("PR_SET_PDEATHSIG","O_NOFOLLOW","LOCK_EX|LOCK_NB","cudaStreamQuery","std::isfinite","SHA256(","32*1024*1024","milliseconds(100)"):
        assert name in source
    assert 'std::string(argv[9])!="HH_CNN_FP32_BYTE_V1"' in source
    assert "batch==6 || batch==30" in source and "sequence==previous+1" in source
    assert "buildSerializedNetwork" not in source and "spawn_actor" not in source and "apply_control" not in source
    assert "warm(6" in source and "cold(30" in source and "createExecutionContext" in source and "enqueueV3" in source
    assert "getTensorDataType(name)==nvinfer1::DataType::kFLOAT" in source
    assert "getTensorFormat(name)==nvinfer1::TensorFormat::kLINEAR" in source
