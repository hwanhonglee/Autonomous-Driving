"""HH_261004 - Exercise owned JPEG normalization parity and prospective benchmark guards using synthetic pixels only."""

import ast
from dataclasses import asdict
import hashlib
from io import BytesIO
import json
from pathlib import Path

import pytest

from scripts.e2e import benchmark_full209_JPEG_normalization_v1_20261004 as bench


@pytest.fixture(scope="module",autouse=True)
def cpu_only():
    # HH_261004 - Test the original existing CPU installation read-only; never install or expose a GPU.
    bench.environment(configure=True)


def toy_frames(kind="ramp",quality=95):
    proof,_,np,runtime=bench.modules()
    from PIL import Image
    y,x=np.indices((360,640),dtype=np.uint32)
    if kind=="ramp":pixels=np.stack(((x+y)%256,(3*x+y)%256,(x+5*y)%256),axis=-1).astype(np.uint8)
    elif kind=="black":pixels=np.zeros((360,640,3),dtype=np.uint8)
    else:pixels=np.random.default_rng(20261004).integers(0,256,size=(360,640,3),dtype=np.uint8)
    encoded=BytesIO();Image.fromarray(pixels,"RGB").save(encoded,format="JPEG",quality=quality)
    payload=encoded.getvalue()
    return tuple(runtime.RuntimeCameraFrame(100_000_000,payload,"jpeg") for _ in bench.ROLES),proof.ModelConfig()


@pytest.mark.parametrize("kind",["ramp","black","noise"])
@pytest.mark.parametrize("quality",[70,95])
def test_every_six_synthetic_JPEG_normalized_byte_and_all_allocations_match(kind,quality):
    frames,config=toy_frames(kind,quality)
    before=[hashlib.sha256(frame.payload).hexdigest() for frame in frames]
    funcs={name:bench.compile_pipeline(candidate=name=="B",witness=True) for name in ("A","B")}
    receipt=bench.bundle_parity(frames,config,funcs)
    assert receipt["passed"] and receipt["normalized_shape"]==[1,6,3,180,320]
    assert len(receipt["all_six_actual_JPEGs"])==6
    assert all(row["exact_output_bytes_equal"] and row["original_payload_RGB_resize_mean_std_unchanged"] for row in receipt["all_six_actual_JPEGs"])
    assert before==[hashlib.sha256(frame.payload).hexdigest() for frame in frames]


def test_inverse_proven_timed_harness_contains_no_parity_hash_work():
    frames,config=toy_frames()
    old=bench.compile_pipeline();new=bench.compile_pipeline(candidate=True)
    a,wa=old(frames,config,bench.ROLES);b,wb=new(frames,config,bench.ROLES)
    assert wa==wb==[] and bench.modules()[0].tensor_bytes(a)==bench.modules()[0].tensor_bytes(b)
    assert "capture_prefix" not in old.__code__.co_names and "capture_prefix" not in new.__code__.co_names
    assert "private_work_guard" not in old.__code__.co_names and "private_work_guard" in new.__code__.co_names


@pytest.mark.parametrize("fault",["view","resize_alias","stats_alias","dtype"])
def test_candidate_denies_nonowned_or_shared_work(fault):
    _,torch,np,_=bench.modules()
    resized=np.zeros((180,320,3),dtype=np.float32);array=np.zeros((360,640,3),dtype=np.uint8)
    tensor=torch.from_numpy(resized).permute(2,0,1).contiguous();mean=tensor.new_tensor([.485,.456,.406]).view(3,1,1);std=tensor.new_tensor([.229,.224,.225]).view(3,1,1)
    if fault=="view":tensor=tensor.view_as(tensor)
    elif fault=="resize_alias":resized=tensor.numpy()
    elif fault=="stats_alias":std=mean
    else:tensor=tensor.double()
    with pytest.raises(RuntimeError):bench.private_work_guard(tensor,array,resized,mean,std)


@pytest.mark.parametrize("fault",["malformed","wrong_size","PNG"])
def test_real_JPEG_decoder_guard_denies_synthetic_invalid_payload(fault):
    _,_,_,runtime=bench.modules()
    from PIL import Image
    encoded=BytesIO()
    if fault=="malformed":payload=b"not an image"
    else:
        Image.new("RGB",(640,360) if fault=="PNG" else (320,180)).save(encoded,format="PNG" if fault=="PNG" else "JPEG")
        payload=encoded.getvalue()
    frames=tuple(runtime.RuntimeCameraFrame(100_000_000,payload,"jpeg") for _ in bench.ROLES)
    with pytest.raises(Exception):bench.compile_pipeline(candidate=True)(frames,bench.modules()[0].ModelConfig(),bench.ROLES)


def test_all_parity_retains_every_synthetic_failed_tail(monkeypatch):
    monkeypatch.setattr(bench,"read_bundle",lambda camera,manifest:camera)
    monkeypatch.setattr(bench,"compile_pipeline",lambda **kwargs:None)
    def check(camera,config,functions):
        if camera["frame"] in (1,3):raise RuntimeError("synthetic parity failure")
        return dict(passed=True)
    monkeypatch.setattr(bench,"bundle_parity",check)
    rows=bench.all_parity([dict(frame=i,timestamp=i*.1) for i in range(4)],{},None)
    assert len(rows)==4 and [row["ordinal"] for row in rows]==[0,1,2,3]
    assert [row["passed"] for row in rows]==[True,False,True,False]
    assert rows[-1]["error"]["message"]=="synthetic parity failure"


def test_thirty_representatives_are_outcome_independent_and_include_last():
    indices=bench.selected_indices()
    assert len(indices)==len(set(indices))==30 and indices==[(917*i)//29 for i in range(30)]
    assert indices[0]==0 and indices[-1]==917
    with pytest.raises(RuntimeError):bench.selected_indices(29)


def test_all_ABBA_timing_rows_and_failure_denominators_retained(monkeypatch):
    _,torch,_,_=bench.modules()
    catalog=[dict(frame=i,timestamp=i*.1) for i in range(918)]
    monkeypatch.setattr(bench,"read_bundle",lambda camera,manifest:camera)
    calls=[]
    def build(*,candidate=False):
        def run(frame,config,roles):
            calls.append("B" if candidate else "A")
            if candidate and frame["frame"]==917:raise RuntimeError("synthetic final candidate failure")
            return torch.zeros(1,1,dtype=torch.float32),[]
        return run
    monkeypatch.setattr(bench,"compile_pipeline",build)
    ticks=iter(range(1,121))
    rows=bench.measured_pairs(catalog,{},None,clock=lambda:next(ticks))
    assert len(rows)==60 and calls==list("ABBA"*15)
    assert all(row["elapsed_ns"]==1 for row in rows)
    assert sum(row["passed"] for row in rows)==59 and sum(row["paired_output_SHA_equal"] for row in rows)==58
    assert rows[-2]["implementation"]=="B" and rows[-2]["passed"] is False
    result=bench.statistics_for(rows)
    assert result["A"]["actual_timed_calls"]==result["B"]["actual_timed_calls"]==30
    assert result["B"]["failures"]==1 and result["B"]["all_attempted_timings_included"]


def test_output_SHA_work_is_after_every_timed_finish(monkeypatch):
    proof,torch,_,_=bench.modules();events=[]
    monkeypatch.setattr(bench,"compile_pipeline",lambda **kwargs:lambda *args:(torch.zeros(1,1),[]))
    monkeypatch.setattr(bench,"read_bundle",lambda *args:None)
    old=proof.tensor_bytes
    monkeypatch.setattr(proof,"tensor_bytes",lambda tensor:(events.append("hash"),old(tensor))[1])
    n=iter(range(1,121))
    def clock():events.append("clock");return next(n)
    bench.measured_pairs([dict(frame=i) for i in range(918)],{},None,clock=clock)
    assert events==["clock","clock","hash"]*60


def test_unreadable_representative_retains_two_explicit_unmeasured_rows(monkeypatch):
    monkeypatch.setattr(bench,"compile_pipeline",lambda **kwargs:lambda *args:pytest.fail("unexpected call"))
    monkeypatch.setattr(bench,"read_bundle",lambda *args:(_ for _ in ()).throw(RuntimeError("source read denied")))
    rows=bench.measured_pairs([dict(frame=i) for i in range(918)],{},None,clock=lambda:pytest.fail("unexpected timer"))
    assert len(rows)==60 and all(row["timed_call_attempted"] is False and row["elapsed_ns"] is None for row in rows)
    assert bench.statistics_for(rows)["A"]["milliseconds"] is None


def test_read_bundle_caps_total_and_exact_source_SHA_using_toy_JPEGs(tmp_path,monkeypatch):
    frames,_=toy_frames();root=tmp_path/"run";episode=root/"episode";episode.mkdir(parents=True)
    monkeypatch.setattr(bench,"RUN",root);monkeypatch.setattr(bench,"EPISODE",episode)
    camera=dict(timestamp=.1,images={});manifest={}
    for role,frame in zip(bench.ROLES,frames):
        path=episode/"images"/role/"00000001.jpg";path.parent.mkdir(parents=True);path.write_bytes(frame.payload)
        camera["images"][role]=str(path.relative_to(episode));manifest[str(path.relative_to(root))]=hashlib.sha256(frame.payload).hexdigest()
    actual=bench.read_bundle(camera,manifest)
    assert len(actual)==6 and all(type(frame.payload) is bytes and frame.encoding=="jpeg" for frame in actual)
    manifest[next(iter(manifest))]="0"*64
    with pytest.raises(RuntimeError,match="SHA"):bench.read_bundle(camera,manifest)


def test_declaration_is_pre_measurement_create_only_and_full_config_sealed(tmp_path,monkeypatch):
    monkeypatch.setattr(bench,"PLAN",tmp_path/"plan/declaration.json");monkeypatch.setattr(bench,"OUTPUT",tmp_path/"output")
    monkeypatch.setattr(bench,"input_snapshot",lambda:dict(JPEG_count=5508))
    monkeypatch.setattr(bench,"camera_catalog",lambda:[])
    monkeypatch.setattr(bench,"all_parity",lambda *args:pytest.fail("parity before approval"))
    monkeypatch.setattr(bench,"measured_pairs",lambda *args:pytest.fail("timing before approval"))
    result=bench.declare();document=bench.read_json(bench.PLAN)
    assert result["status"]=="DECLARED_NOT_MEASURED" and not bench.OUTPUT.exists()
    assert document["config"]==asdict(bench.modules()[0].ModelConfig())
    assert document["contract"]["all_JPEGs"]==5508 and document["contract"]["timed_calls"]==60
    with pytest.raises(RuntimeError,match="fresh"):bench.declare()


def test_source_pins_preserve_frozen_helper_and_existing_runtime():
    sources=bench.source_snapshot()
    assert len(sources)==10 and all(sources[name]==sha for name,sha in bench.FIXED_SOURCE_SHA.items())
    assert sources[str(bench.HERE.relative_to(bench.ROOT))]==bench.digest(bench.HERE)
    assert sources[str(bench.TEST.relative_to(bench.ROOT))]==bench.digest(bench.TEST)


@pytest.mark.parametrize("name",["CUDA_VISIBLE_DEVICES","OMP_NUM_THREADS","MKL_NUM_THREADS","OPENBLAS_NUM_THREADS"])
def test_environment_mismatch_denied_before_actual_work(monkeypatch,name):
    monkeypatch.setenv(name,"wrong")
    with pytest.raises(RuntimeError,match="environment"):bench.environment()


def test_no_live_BGRA_or_model_or_cold_cache_equivalence_claimed():
    contract=bench.contract()
    assert contract["lossy_JPEG_replay"] and contract["warm_cache"] and not contract["actual_original_live_BGRA_tensor_parity_proven"]
    assert not contract["actual_sensor_latency_gain_proven"] and not contract["actual_runtime_activated"]
    assert not contract["model_NN_forward_or_control_test"] and not contract["formal_function_approval"]
    assert contract["no_normalization_cache_or_source_SHA_omission"]
    tree=ast.parse(bench.HERE.read_text())
    assert not any(isinstance(node,ast.Call) and ast.unparse(node.func).endswith(("cuda","install","set_affinity")) for node in ast.walk(tree))
