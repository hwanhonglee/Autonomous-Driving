"""HH_261005 - Scalar/AST and fully fake lifecycle tests only; no Torch/NumPy/weights/engines/compiler/native/GPU/subprocess execution."""

import ast
import hashlib
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

import pytest

from portable_e2e import temporal_cached_tensorrt_backend_v2_20261005 as backend
from scripts.e2e import prepare_good6000_cnn_tensorrt_runtime_v2_20261005 as owner


@pytest.mark.parametrize("count",(6,12,18,24,30))
def test_all_ordered_warm_groups(count):
    assert backend.native_batches(count,cold=False)==(6,)*(count//6)


def test_cold30_and_source_cache_inverse():
    assert backend.native_batches(30,cold=True)==(30,)
    with pytest.raises(RuntimeError):backend.native_batches(6,cold=True)
    assert backend.cache_inverse_ast_proof()["original_cache_inverse_equal"]


@pytest.mark.parametrize("count",(True,False,0,1,5,7,31,6.0,"6"))
def test_nonoriginal_CNN_count_rejected(count):
    with pytest.raises(RuntimeError):backend.native_batches(count,cold=False)


@pytest.mark.parametrize("batch",(6,30))
def test_complete_packet_fixed_size_and_large_exact_integer(batch):
    source=hashlib.sha256(b"fake fixture only").digest();seq=2**63+123
    assert backend.REQUEST.unpack(backend.request_header(batch,seq,source))==(backend.MAGIC,2,seq,batch,batch*3*180*320*4,source)


@pytest.mark.parametrize("sequence",(True,False,0,-1,2**64,1.0))
def test_bad_request_sequence_denied(sequence):
    with pytest.raises(RuntimeError):backend.request_header(6,sequence,b"x"*32)
    with pytest.raises(RuntimeError):backend.shutdown_header(sequence)


def header(batch=6,sequence=1,status=1,payload=None):
    raw=b"\0"*(batch*60*128*4) if payload is None else payload;source=hashlib.sha256(b"fixture input").digest()
    return backend.RESPONSE.pack(backend.MAGIC,2,sequence,batch,len(raw),status,source,hashlib.sha256(raw).digest()),raw,source


@pytest.mark.parametrize("fault",("sequence","batch","source","length","checksum","NaN","status","magic","version"))
def test_source_packet_mutations_are_denied(fault):
    packet,raw,source=header();values=list(backend.RESPONSE.unpack(packet))
    if fault=="sequence":values[2]+=1
    if fault=="batch":values[3]=30
    if fault=="source":values[6]=b"x"*32
    if fault=="length":values[4]-=4
    if fault=="checksum":values[7]=b"x"*32
    if fault=="NaN":raw=struct.pack("<f",float("nan"))+raw[4:];values[7]=hashlib.sha256(raw).digest()
    if fault=="status":values[5]=4
    if fault=="magic":values[0]+=1
    if fault=="version":values[1]=1
    with pytest.raises(RuntimeError):backend.decode_response(backend.RESPONSE.pack(*values),raw,sequence=1,batch=6,input_sha256=source)


@pytest.mark.parametrize("status,sequence",((3,0),(4,2)))
def test_distinct_READY_and_shutdown_ACK(status,sequence):
    source=b"\0"*32 if status==3 else hashlib.sha256(b"").digest()
    packet=backend.RESPONSE.pack(backend.MAGIC,2,sequence,0,0,status,source,hashlib.sha256(b"").digest())
    backend.verify_control_response(packet,sequence=sequence,status=status)
    values=list(backend.RESPONSE.unpack(packet));values[2]+=1
    with pytest.raises(RuntimeError):backend.verify_control_response(backend.RESPONSE.pack(*values),sequence=sequence,status=status)
    if status==4:assert backend.REQUEST.unpack(backend.shutdown_header(sequence))==(backend.MAGIC,2,sequence,0,0,hashlib.sha256(b"").digest())


@pytest.mark.parametrize("field",("pid","ppid","pgid","sid","start_ticks","argv","state"))
def test_current_owned_identity_all_fields(field):
    core=dict(pid=123,ppid=111,pgid=123,sid=123,start_ticks=456)
    observed=dict(core,argv=["exact-native"],state="S");backend.verify_native_identity(observed,core,["exact-native"])
    observed[field]="Z" if field=="state" else (["foreign"] if field=="argv" else observed[field]+1)
    with pytest.raises(RuntimeError):backend.verify_native_identity(observed,core,["exact-native"])


def test_missing_fixed_ROOT_authority_denies_before_Torch_native_IO_or_bool_override(monkeypatch):
    modules=set(sys.modules)
    monkeypatch.setattr(backend.subprocess,"Popen",lambda *a,**k:pytest.fail("no subprocess"))
    with pytest.raises(RuntimeError,match="external exact ROOT SHA"):
        backend.PreparedTensorRTClientV2("not-SHA","not-SHA",9)
    with pytest.raises(TypeError):backend.PreparedTensorRTClientV2(True,ROOT_approved=True)
    forged=object.__new__(backend.PreparedTensorRTClientV2)
    forged._guard_owner_lease_sources_engines=lambda:None
    with pytest.raises((AttributeError,RuntimeError)):backend._require_actual_admission(forged)
    with pytest.raises(RuntimeError):backend.create_backend_model(object(),object())
    assert not any(name in set(sys.modules)-modules for name in ("torch","numpy","onnx","tensorrt"))


def fake_client(monkeypatch,tmp_path,*,ready_fault=False):
    folder=tmp_path.resolve();(folder/"GPU0").mkdir();monkeypatch.setattr(owner,"OUTPUT",folder)
    worker=dict(pid=owner.os.getpid(),ppid=111,pgid=owner.os.getpid(),sid=owner.os.getpid(),start_ticks=10)
    native=dict(pid=12345,ppid=worker["pid"],pgid=12345,sid=12345,start_ticks=11)
    plan={"GPU_uuid":"GPU-fixture-only","successful_engine_producer":{"engine":{"6":{"sha256":"a"*64},"30":{"sha256":"b"*64}}}}
    events=[];argv=owner.native_argv(plan,worker,9)
    manifest={"full101_state":{str(i):dict(shape=[1],dtype="float32",raw_sha256="0"*64) for i in range(101)},
        "exact14_CNN_state":{str(i):{} for i in range(14)}}
    owner.new_json(folder/"original101_exact14_manifest.json",manifest)
    owner.complete_json(folder/"GPU0/native_supervision_admitted.json",dict(core=native,argv=argv,status="COHERENT_STRICT_VERIFIED"))
    owner.complete_json(folder/"GPU0/native_close_owner_ack.json",dict(core=native,sequence=0,status="NO_GPU_OBSERVATION_IN_FLIGHT"))
    monkeypatch.setattr(owner,"client_context",lambda *args:(plan,dict(pid=111),{}))
    monkeypatch.setattr(owner,"client_guard",lambda client:events.append("guard"))
    monkeypatch.setattr(owner.build.seals,"process_core",lambda pid:worker if pid==worker["pid"] else native)
    monkeypatch.setattr(owner,"observe_process",lambda pid:dict(native,argv=argv,state="S"))
    class Pipe:
        def __init__(self,fd):self.fd=fd
        def fileno(self):return self.fd
        def close(self):events.append("pipe_closed")
    proc=SimpleNamespace(pid=native["pid"],stdin=Pipe(6),stdout=Pipe(7),returncode=None)
    proc.poll=lambda:proc.returncode
    def wait(timeout):proc.returncode=0;events.append("normal_wait");return 0
    proc.wait=wait
    monkeypatch.setattr(backend.subprocess,"Popen",lambda *a,**kwargs:(events.append(dict(Popen_kwargs=kwargs)) or proc))
    monkeypatch.setattr(backend.os,"set_blocking",lambda *a:None)
    cleanup=dict(status="OWNED_GENERATIONS_GONE",signalled=[],errors=[])
    monkeypatch.setattr(owner,"OwnedTree",lambda *a:SimpleNamespace(stop=lambda:(events.append("cleanup") or cleanup)))
    monkeypatch.setattr(owner,"verify",lambda *a:plan);monkeypatch.setattr(owner,"archive_verify",lambda *a:None)
    def transfer(fd,data,*,writing,deadline,client,normal_exit_ACK=False):
        if writing:events.append(dict(write_header=backend.REQUEST.unpack(data)));return None
        status=3 if client.sequence==0 else 4;source=b"\0"*32 if status==3 else hashlib.sha256(b"").digest()
        if ready_fault and status==3:status=1
        events.append("ready_read" if status==3 else "ACK_read")
        return backend.RESPONSE.pack(backend.MAGIC,2,client.sequence,0,0,status,source,hashlib.sha256(b"").digest())
    monkeypatch.setattr(backend,"_bounded_pipe",transfer)
    return events,proc


def test_fake_owned_start_bothREADY_no_warmup_and_ordered_shutdown_normal_exit(monkeypatch,tmp_path):
    events,proc=fake_client(monkeypatch,tmp_path)
    client=backend.PreparedTensorRTClientV2("a"*64,"b"*64,9)
    assert client.lifecycle["status"]=="BOTH_READY_AND_OWNED_SUPERVISION_ADMITTED_NO_INFERENCE"
    launch=next(row for row in events if isinstance(row,dict) and "Popen_kwargs" in row)["Popen_kwargs"]
    assert launch["start_new_session"] and launch["pass_fds"]==(9,) and "preexec_fn" not in launch
    assert not any(isinstance(row,dict) and "write_header" in row for row in events)
    close=client.close();assert close["status"]=="NORMAL_ACK_EXIT_AND_OWNED_GENERATIONS_GONE"
    assert close["ACK_sequence"]==1 and proc.returncode==0 and events.index("ACK_read")<events.index("normal_wait")<events.index("cleanup")
    assert client.close()["status"]=="FAIL_RETAINED"


def test_fake_BAD_READY_retains_failure_and_known_cleanup_no_inference(monkeypatch,tmp_path):
    events,_=fake_client(monkeypatch,tmp_path,ready_fault=True)
    with pytest.raises(RuntimeError,match="READY/shutdown ACK"):
        backend.PreparedTensorRTClientV2("a"*64,"b"*64,9)
    assert "cleanup" in events and not any(isinstance(row,dict) and "write_header" in row for row in events)
    assert owner.read_json(owner.OUTPUT/"GPU0/native_setup_result.json")["status"]=="FAIL_RETAINED"


def test_native_inverse_only_protocol_shutdown_not_CNN_math():
    old=(backend.ROOT/"tools/e2e/good6000_cnn_tensorrt_runtime_v1_20261005.cpp").read_text()
    new=(backend.ROOT/"tools/e2e/good6000_cnn_tensorrt_runtime_v2_20261005.cpp").read_text()
    assert old[old.index("struct Runner {"):old.index("} // namespace")]==new[new.index("struct Runner {"):new.index("} // namespace")]
    assert old[old.index("void owner_guard"):old.index("struct Runner {")]==new[new.index("void owner_guard"):new.index("struct Runner {")]
    assert 'if(batch==0)' in new and 'sequence==previous+1' in new and 'response(sequence,0,4,nullptr,0,input_sha,deadline);return 0;' in new
    assert 'version = 2' in new and 'HH_CNN_FP32_BYTE_V2' in new and 'buildSerializedNetwork' not in new


def test_original_model_guard_stride_and_no_trusted_type_forgery():
    source=Path(backend.__file__).read_text();tree=ast.parse(source)
    model=next(node for node in ast.walk(tree) if isinstance(node,ast.ClassDef) and node.name=="TensorRTCachedRouteRecoveryV2")
    methods=[node for node in model.body if isinstance(node,ast.FunctionDef)]
    assert [node.name for node in methods]==["_state_identity"] and ast.unparse(methods[0]).count("super()._state_identity()")==1
    assert "len(state)==101" in source and "pointers==" in source and "result = result.transpose(1,2).contiguous().transpose(1,2)" in source
    assert "__class__ =" not in source and "_trusted_readonly_encoder =" not in source and "GLOBAL" not in backend.PARENT_SOURCE_PINS
