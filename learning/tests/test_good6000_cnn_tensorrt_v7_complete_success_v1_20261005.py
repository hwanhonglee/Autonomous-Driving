"""HH_261005 - Pure captured-byte/identity and fixed-interface fixtures only; no actual source/asset tree audit, GPU query, weight/native/engine execution."""

import ast
import base64
import copy
import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import audit_good6000_cnn_tensorrt_v7_complete_success_v1_20261005 as audit
from scripts.e2e import prepare_good6000_cnn_tensorrt_runtime_v2_20261005 as runtime


def fixture(*,compute=False,disagreement=False):
    graphics=[dict(pid=index,ppid=1,pgid=index,start_ticks=index+5,argv=["sealed-fixture-G-%d"%index]) for index in (10,11,12,13)]
    core=dict(pid=20,ppid=2,pgid=20,sid=20,start_ticks=25);argv=["sealed-owned-builder"]
    gpu="GPU-fixture";full="Processes\n"+"".join("|  0  N/A N/A %d G name |\n"%row["pid"] for row in graphics)
    if compute:full+="|  0  N/A N/A 20 C exact-builder |\n"
    apps=[dict(pid=20,uuid=gpu)] if compute and not disagreement else []
    outputs=["0, GPU-fixture, 6000, 300, 20\n","20, GPU-fixture\n" if apps else "",full]
    raw=dict(started_monotonic_ns=2**63+1,finished_monotonic_ns=2**63+5,commands=[])
    options=(["--query-gpu=index,uuid,memory.free,memory.used,utilization.gpu","--format=csv,noheader,nounits"],
        ["--query-compute-apps=pid,gpu_uuid","--format=csv,noheader,nounits"],[])
    for text,args in zip(outputs,options):
        row=dict(argv=[str(audit.producer.NVIDIA_SMI),"-i","0",*args],returncode=0,
            started_monotonic_ns=2**63+1,finished_monotonic_ns=2**63+4,timeout_seconds=3)
        for name,data in (("stdout",text.encode()),("stderr",b"")):
            row.update({name+"_base64":base64.b64encode(data).decode(),name+"_bytes":len(data),
                name+"_sha256":hashlib.sha256(data).hexdigest(),name+"_truncated":False})
        raw["commands"].append(row)
    identities=[dict(kind="G",identity=row,state="S") for row in graphics]
    if compute:identities.append(dict(kind="C",identity={key:value for key,value in core.items() if key!="sid"}|{"argv":argv},state="S"))
    value=dict(index=0,uuid=gpu,free_MiB=6000,used_MiB=300,utilization_percent=20,apps=apps,
        full_NVIDIA_GPU0_inventory=full,full_process_identities=identities,
        app_process_identities=[dict(state="S",identity={key:value for key,value in core.items() if key!="sid"}|{"argv":argv})] if apps else [])
    plan=dict(GPU_uuid=gpu,existing_desktop_graphics=graphics)
    return raw,value,plan,core,argv


def test_initial_exact_known_desktop_and_recorded_int64_times():
    raw,value,plan,core,argv=fixture()
    assert audit.verify_observation(raw,value,plan,native_core=core,native_argv=argv)
    assert raw["commands"][0]["started_monotonic_ns"]==2**63+1


@pytest.mark.parametrize("disagreement",(False,True))
def test_only_owned_historical_compute_coherence_without_current_process_fabrication(disagreement):
    raw,value,plan,core,argv=fixture(compute=True,disagreement=disagreement)
    assert audit.verify_observation(raw,value,plan,native_core=core,native_argv=argv,baseline=dict(used_MiB=300)) is not disagreement


@pytest.mark.parametrize("fault",("raw_checksum","truncated","wrong_command","foreign_PID","generation","argv","dead","unknown_kind","desktop","UUID","memory","duplicate","raw_inventory"))
def test_any_unknown_foreign_or_unsealed_observation_is_denied(fault):
    raw,value,plan,core,argv=fixture(compute=True)
    if fault=="raw_checksum":raw["commands"][0]["stdout_sha256"]="0"*64
    if fault=="truncated":raw["commands"][0]["stdout_truncated"]=True
    if fault=="wrong_command":raw["commands"][0]["argv"]=["foreign-query"]
    if fault=="foreign_PID":value["app_process_identities"][0]["identity"]["pid"]=21
    if fault=="generation":value["full_process_identities"][-1]["identity"]["start_ticks"]+=1
    if fault=="argv":value["full_process_identities"][-1]["identity"]["argv"]=["foreign-builder"]
    if fault=="dead":value["full_process_identities"][-1]["state"]="Z"
    if fault=="unknown_kind":value["full_process_identities"][-1]["kind"]="C+G"
    if fault=="desktop":value["full_process_identities"][0]["identity"]=dict(value["full_process_identities"][0]["identity"],start_ticks=999)
    if fault=="UUID":value["uuid"]="GPU-foreign"
    if fault=="memory":value["used_MiB"]=9999
    if fault=="duplicate":value["full_process_identities"].append(copy.deepcopy(value["full_process_identities"][-1]))
    if fault=="raw_inventory":value["full_NVIDIA_GPU0_inventory"]+="forged text"
    with pytest.raises(RuntimeError):audit.verify_observation(raw,value,plan,native_core=core,native_argv=argv,baseline=dict(used_MiB=300))


def test_exact_frozen_runtime_interface_and_no_ROOT_write(tmp_path):
    plan=dict(source_sha256={"fixture_source":"1"*64});result=dict(engine_sha256={"6":"2"*64,"30":"3"*64})
    value=audit.successful_interface(plan,result,"4"*64)
    assert value==dict(schema=runtime.build.SCHEMA+".ROOT_complete_independent_postcheck",status="PASS_OF_COMPLETE_SUCCESS",
        result_sha256="4"*64,source_sha256=plan["source_sha256"],engine_sha256=result["engine_sha256"],
        all_original101_exact14_sources_assets_archives_headers_libraries_and_owned_cleanup_verified=True)
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize("fault",(None,"four_samples","missing_closing","decision_changed"))
def test_complete_journal_keeps_owned_disagreements_and_requires_coherent_closing(monkeypatch,tmp_path,fault):
    # HH_261005 - Fake recorded dictionaries only; exercise every sample and closing proof without querying a GPU or reading actual artifacts.
    records={};journal=[];first=None;last=None;plan=None;core=None;argv=None
    cases=[(False,False,1),(True,True,1),(True,True,2),(True,False,3),(False,False,1)]
    if fault=="four_samples":cases[3]=(True,False,4)
    if fault=="missing_closing":cases=cases[:-1]
    for index,(compute,disagreement,sample) in enumerate(cases,1):
        raw,value,plan,core,argv=fixture(compute=compute,disagreement=disagreement)
        prefix="gpu_observations/%04d"%index
        row=dict(raw_path=prefix+".raw.json",identity_path=prefix+".identity.json",sample=sample,
            status="OWNED_ONLY_LIST_DISAGREEMENT_NOT_ACCEPTED" if disagreement else "COHERENT_STRICT_VERIFIED")
        for name,obj in ((row["raw_path"],raw),(row["identity_path"],value)):
            records[name]=obj;row["raw_sha256" if name.endswith(".raw.json") else "identity_sha256"]=hashlib.sha256(json.dumps(obj,sort_keys=True).encode()).hexdigest()
        records[prefix+".decision.json"]=dict(row);journal.append(row)
        if first is None:first=value
        last=value
    if fault=="decision_changed":records["gpu_observations/0002.decision.json"]["sample"]=2
    monkeypatch.setattr(audit,"OUTPUT",tmp_path)
    monkeypatch.setattr(audit,"read_parent_json",lambda path:records[str(path.relative_to(tmp_path))])
    monkeypatch.setattr(audit,"sha",lambda path:hashlib.sha256(json.dumps(records[str(path.relative_to(tmp_path))],sort_keys=True).encode()).hexdigest())
    result=dict(GPU_observation_journal=journal,native_child_creation_core=core,actual_native_argv=argv,
        actual_before_native_known_desktop_inventory=first,actual_after_native_known_desktop_inventory=last)
    if fault is None:assert len(audit.audit_journal(plan,result))==15
    else:
        with pytest.raises(RuntimeError):audit.audit_journal(plan,result)


def test_original_evidence_retains_static_full_closure_but_not_dynamic_census():
    recorded=dict(current_bavail_bytes=audit.producer.FLOOR+audit.producer.SHARED+audit.producer.CLOSING,
        process_metadata_records_checked=100,original101_and_archives="sealed-fixture")
    audit.original_evidence_unchanged(recorded,dict(recorded,current_bavail_bytes=recorded["current_bavail_bytes"]+99,process_metadata_records_checked=101))
    with pytest.raises(RuntimeError):audit.original_evidence_unchanged(recorded,dict(recorded,original101_and_archives="changed"))
    with pytest.raises(RuntimeError):audit.original_evidence_unchanged(dict(recorded,process_metadata_records_checked=0),recorded)


def test_parent_metadata_oversize_denied_before_decode(monkeypatch,tmp_path):
    path=tmp_path/"fixture.json";path.write_bytes(b" "+b"x"*16)
    monkeypatch.setattr(audit.producer,"PARENT_JSON_LIMIT",16)
    monkeypatch.setattr(audit,"read_json",lambda path:pytest.fail("oversize must not decode"))
    with pytest.raises(RuntimeError):audit.read_parent_json(path)


def test_missing_successful_result_denies_before_original_asset_audit(monkeypatch,tmp_path):
    monkeypatch.setattr(audit,"OUTPUT",tmp_path.resolve())
    monkeypatch.setattr(audit.producer,"verify",lambda *args:dict(source_sha256={str(i):"0"*64 for i in range(73)},fixed_batches=[6,30]))
    monkeypatch.setattr(audit.producer.producer,"audit",lambda:pytest.fail("cannot read actual model assets"))
    with pytest.raises((RuntimeError,FileNotFoundError)):audit.audit("1"*64,"2"*64,"3"*64,audit.sha(Path(audit.__file__)))


def test_auditor_source_readonly_no_new_native_inference_authority_or_binary_ABI_claim():
    source=Path(audit.__file__).read_text();tree=ast.parse(source)
    # HH_261005 - Check imported module roots, not substrings inside the approved source-only producer filename.
    roots={alias.name.split(".")[0] for node in tree.body if isinstance(node,ast.Import) for alias in node.names}
    roots|={node.module.split(".")[0] for node in tree.body if isinstance(node,ast.ImportFrom) and node.module}
    assert roots.isdisjoint({"torch","numpy","onnx","tensorrt","subprocess"})
    assert "producer.gpu_snapshot(" not in source and "producer.execute(" not in source and "producer.compile_argv(" not in source
    assert not any(isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr in ("write_bytes","write_text","mkdir","unlink","remove") for node in ast.walk(tree))
    assert "runtime_inference_numerical_parity_latency_or_feature_approved=False" in source
