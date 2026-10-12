"""HH_261005 - Pure fake ownership/admission/budget/failed-ledger fixtures. No real compiler, process, GPU query, model, engine or camera payload is used."""

import ast
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import prepare_good6000_cnn_tensorrt_runtime_v2_20261005 as owner


def test_all_failed_denominators_are_fixed_complete_cold_and_tail():
    value=owner.failure_denominator()
    assert value["status"]=="FAIL_RETAINED" and value["planned_arms"]==["CPU6","GPU0"]
    for arm in value["all_arms"]:
        assert len(arm["complete_context_status"])==888 and len(arm["complete_timing_status"])==30
        assert arm["timed_contexts"][0]==0 and arm["timed_contexts"][-1]==887 and len(set(arm["timed_contexts"]))==30
    assert owner.screen.RAW_ATOL==owner.screen.RAW_RTOL==1e-6


def test_metadata_NaN_or_oversize_denied_before_file_creation(tmp_path):
    path=tmp_path.resolve()/"not_created.json"
    with pytest.raises(RuntimeError):owner.new_json(path,{"oversize":"x"*owner.JSON_LIMIT})
    assert not path.exists()
    with pytest.raises(ValueError):owner.new_json(path,{"not_finite":float("nan")})
    assert not path.exists()


def test_commit_marker_occurs_only_after_closed_json(monkeypatch,tmp_path):
    path=tmp_path.resolve()/"control.json";events=[];old=owner.new_json
    def record(target,value):
        assert not Path(str(path)+".complete").exists();old(target,value)
        assert owner.read_json(path)==value;events.append("data_complete")
    monkeypatch.setattr(owner,"new_json",record)
    owner.complete_json(path,{"sequence":2**63+5})
    assert events==["data_complete"] and Path(str(path)+".complete").stat().st_size==0
    # HH_261005 - Restore the recorder fixture before asserting the production create-only duplicate denial.
    monkeypatch.setattr(owner,"new_json",old)
    with pytest.raises(FileExistsError):owner.complete_json(path,{"sequence":3})


def minimal_plan():
    proof=dict(build_plan_sha256="1"*64,build_authority_sha256="2"*64,result_sha256="3"*64,postcheck_sha256="4"*64)
    return dict(schema=owner.SCHEMA+".ROOT_plan",successful_engine_producer=proof,source_sha256={},model_asset_sha256={},runtime_source_sha256={},
        sdk_header_alias_to_canonical={},sdk_header_sha256={},sdk_library_sha256={},GPU_uuid="GPU-FAKE",all_original_files={})


def authority(plan):
    return dict(schema=owner.SCHEMA+".ROOT_execution_authority",explicit_private_CPU_compile_GPU0_CNN_and_full888_counterfactual_approved=True,
        plan_path=str(owner.PLAN),plan_sha256=owner.sha(owner.PLAN),source_sha256=plan["source_sha256"],model_asset_sha256={},runtime_source_sha256={},
        successful_engine_producer=plan["successful_engine_producer"],sdk_header_alias_to_canonical={},sdk_header_sha256={},sdk_library_sha256={},
        GPU_uuid=plan["GPU_uuid"],all_original_files={},actual_simulator_or_control_or_TRAIN_or_TEST_or_model_promotion_authorized=False)


@pytest.mark.parametrize("fault",("plan_path","plan_sha256","approved","schema","source","engine_proof","actual_control"))
def test_fixed_ROOT_admission_denies_wrong_binding_no_callbacks(monkeypatch,tmp_path,fault):
    monkeypatch.setattr(owner,"PLAN",tmp_path.resolve()/"ROOT_plan.json")
    monkeypatch.setattr(owner,"AUTHORITY",tmp_path.resolve()/"ROOT_execution_authority.json")
    plan=minimal_plan();owner.new_json(owner.PLAN,plan);auth=authority(plan)
    if fault=="plan_path":auth["plan_path"]=str(owner.PLAN.with_name("foreign.json"))
    if fault=="plan_sha256":auth["plan_sha256"]="0"*64
    if fault=="approved":auth["explicit_private_CPU_compile_GPU0_CNN_and_full888_counterfactual_approved"]=False
    if fault=="schema":auth["schema"]="self-approved"
    if fault=="source":auth["source_sha256"]={"unsealed":"0"*64}
    if fault=="engine_proof":auth["successful_engine_producer"]=dict(plan["successful_engine_producer"],result_sha256="0"*64)
    if fault=="actual_control":auth["actual_simulator_or_control_or_TRAIN_or_TEST_or_model_promotion_authorized"]=True
    owner.new_json(owner.AUTHORITY,auth);monkeypatch.setattr(owner,"contract",lambda *args:plan)
    monkeypatch.setattr(owner.subprocess,"Popen",lambda *args,**kwargs:pytest.fail("must not spawn"))
    with pytest.raises(RuntimeError,match="exact ROOT authority"):
        owner.verify(owner.sha(owner.PLAN),owner.sha(owner.AUTHORITY))


def test_actual_missing_plan_and_missing_successful_V7_proof_fail_before_jobs(monkeypatch,tmp_path):
    monkeypatch.setattr(owner,"PLAN",tmp_path.resolve()/"absent.json")
    monkeypatch.setattr(owner.subprocess,"Popen",lambda *args,**kwargs:pytest.fail("no jobs"))
    with pytest.raises(FileNotFoundError):owner.verify("1"*64,"2"*64)
    monkeypatch.setattr(owner.build,"verify",lambda *args:{})
    monkeypatch.setattr(owner.build,"OUTPUT",tmp_path.resolve()/"missing_success")
    with pytest.raises((FileNotFoundError,RuntimeError)):owner.engine_proof("1"*64,"2"*64,"3"*64,"4"*64)


def fake_owned(monkeypatch,*,foreign=False,reused=False):
    root=dict(pid=100,ppid=owner.os.getpid(),pgid=100,sid=100,start_ticks=10)
    child=dict(pid=101,ppid=100,pgid=100,sid=100,start_ticks=11)
    live={100:root,101:child};signals=[]
    if foreign:live[999]=dict(pid=999,ppid=777,pgid=100,sid=100,start_ticks=12)
    monkeypatch.setattr(owner.build.seals,"process_core",lambda pid:live.get(pid))
    monkeypatch.setattr(owner.processes,"all_processes",lambda:[dict(row,state="S") for row in live.values()])
    proc=SimpleNamespace(pid=100,returncode=None)
    def poll():
        if 100 not in live:proc.returncode=0
        return proc.returncode
    proc.poll=poll;proc.wait=lambda timeout:poll()
    monkeypatch.setattr(owner.os,"pidfd_open",lambda pid,flags:pid)
    monkeypatch.setattr(owner.os,"close",lambda fd:None)
    def send(fd,sig):signals.append((fd,int(sig)));live.pop(fd,None)
    monkeypatch.setattr(owner.signal,"pidfd_send_signal",send)
    tree=owner.OwnedTree(proc,root);tree.track()
    if reused:live[101]=dict(child,start_ticks=99)
    return tree,signals,live


def test_individual_owned_children_stop_before_parent_no_killpg(monkeypatch):
    tree,signals,live=fake_owned(monkeypatch)
    result=tree.stop();assert result["status"]=="OWNED_GENERATIONS_GONE" and not live
    assert [pid for pid,signum in signals]==[101,100]


@pytest.mark.parametrize("fault",("foreign","reused"))
def test_unknown_or_reused_generation_never_signalled_and_failure_retained(monkeypatch,fault):
    tree,signals,live=fake_owned(monkeypatch,**{fault:True})
    result=tree.stop();assert result["status"]=="FAIL_NOT_VERIFIED" and result["errors"]
    assert 100 not in live and not any(pid==(999 if fault=="foreign" else 101) for pid,signum in signals)


@pytest.mark.parametrize("fault",("failure","budget","generation","deadline","cleanup"))
def test_fake_compiler_failure_timeout_or_quota_preserves_cleanup_and_full2arm_denominator(monkeypatch,tmp_path,fault):
    folder=tmp_path.resolve();monkeypatch.setattr(owner,"OUTPUT",folder)
    core=dict(pid=100,ppid=owner.os.getpid(),pgid=100,sid=100,start_ticks=10);calls=[]
    proc=SimpleNamespace(pid=100,returncode=None)
    proc.poll=lambda:proc.returncode
    def wait(timeout):proc.returncode=1 if fault=="failure" else 0;return proc.returncode
    proc.wait=wait
    monkeypatch.setattr(owner.subprocess,"Popen",lambda *a,**k:proc)
    monkeypatch.setattr(owner.build.seals,"process_core",lambda pid:dict(core,start_ticks=11) if fault=="generation" and calls else core)
    def track():calls.append("track")
    def stop():calls.append("cleanup");proc.returncode=-9 if proc.returncode is None else proc.returncode;return dict(status="FAIL_NOT_VERIFIED" if fault=="cleanup" else "OWNED_GENERATIONS_GONE")
    monkeypatch.setattr(owner,"OwnedTree",lambda *a:SimpleNamespace(track=track,stop=stop))
    monkeypatch.setattr(owner,"header_seals",lambda:({},{}));monkeypatch.setattr(owner,"sha",lambda path:"f"*64)
    def budget():
        if fault=="budget":raise RuntimeError("fake budget breach")
    monkeypatch.setattr(owner,"budget",budget)
    moments=iter((0.,31.,32.)) if fault=="deadline" else iter((0.,1.,2.,3.,4.))
    monkeypatch.setattr(owner.time,"monotonic",lambda:next(moments))
    plan=dict(compiler_sha256="f"*64,sdk_header_alias_to_canonical={},sdk_header_sha256={},compile_argv=["fake-compiler-only"])
    with pytest.raises(RuntimeError,match="owned compiler failure"):
        owner.compile_owned(plan,{})
    row=json.loads((folder/"compile_result.json").read_text())
    assert row["status"]=="FAIL_RETAINED" and "cleanup" in calls
    assert (folder/"compile.stdout.log").exists() and (folder/"compile.stderr.log").exists()
    assert len(owner.failure_denominator()["all_arms"])==2


def child_ledger(arm="GPU0"):
    return dict(schema=owner.SCHEMA+".arm",arm=arm,status="FAIL_RETAINED",planned_contexts=888,planned_timers=30,
        complete_context_status=["COMPLETED"]*2+["FAIL_NOT_VERIFIED"]+["UNEXECUTED_NOT_VERIFIED"]*885,
        complete_timing_status=["COMPLETED"]+["UNEXECUTED_NOT_VERIFIED"]*29,completed_contexts=2,
        attempted_timers=1,reconstructed_camera_bundles=15,error="original fake failure retained")


def test_worker_FAIL_full_ledger_propagates_without_changing_parent_failure(monkeypatch,tmp_path):
    monkeypatch.setattr(owner,"OUTPUT",tmp_path.resolve());(owner.OUTPUT/"GPU0").mkdir()
    owner.new_json(owner.OUTPUT/"GPU0/result.json",child_ledger());row=owner.failure_denominator()["all_arms"][1]
    row["status"]="STARTED_FAIL_NOT_VERIFIED";owner.retain_child_result(row,"GPU0")
    assert row["status"]=="STARTED_FAIL_NOT_VERIFIED" and row["child_result_status"]=="FAIL_RETAINED"
    assert row["completed_contexts"]==2 and row["complete_context_status"][2]=="FAIL_NOT_VERIFIED"
    assert len(row["complete_context_status"])==888 and len(row["complete_timing_status"])==30
    assert row["result_sha256"]==owner.sha(owner.OUTPUT/"GPU0/result.json")


@pytest.mark.parametrize("fault",("absent","short","wrong_arm","counts"))
def test_child_unknown_or_malformed_ledger_stays_FAIL_UNKNOWN(monkeypatch,tmp_path,fault):
    monkeypatch.setattr(owner,"OUTPUT",tmp_path.resolve());(owner.OUTPUT/"GPU0").mkdir()
    ledger=child_ledger();row=owner.failure_denominator()["all_arms"][1]
    if fault=="short":ledger["complete_context_status"].pop()
    if fault=="wrong_arm":ledger["arm"]="CPU6"
    if fault=="counts":ledger["completed_contexts"]=3
    if fault!="absent":
        owner.new_json(owner.OUTPUT/"GPU0/result.json",ledger)
        with pytest.raises(RuntimeError):owner.retain_child_result(row,"GPU0")
    else:owner.retain_child_result(row,"GPU0")
    assert row["status"]=="UNEXECUTED_NOT_VERIFIED" and set(row["complete_context_status"])=={"UNKNOWN_NOT_RECORDED"}


def test_runtime_source_has_proven_reconstructor_rawguards_no_controller_or_math_reimplementation():
    source=Path(owner.__file__).read_text();tree=ast.parse(source)
    for name in ("recorded.reconstructed_frames","recorded.models()","screen.raw_comparison","screen.ordinary_decoder_screen","screen.initialize_worker_runtime","screen.set_threads_after_load"):
        assert name in source
    assert "killpg" not in source.replace("no killpg","") and "pidfd_send_signal" in source
    assert "import torch" not in "\n".join(ast.unparse(node) for node in tree.body if isinstance(node,(ast.Import,ast.ImportFrom)))
    assert "buildSerializedNetwork" not in source and "load_route_joint_research_npz(" not in source and "apply_control" not in source
