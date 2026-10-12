"""HH_261002 - Synthetic protect scopes and original owned lifecycle only; no real simulator/model/data jobs."""
from contextlib import contextmanager, ExitStack
import ast
import copy
import importlib.util
import inspect
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest import mock

import pytest
from scripts.e2e import collect_temporal_intrinsic_town07_signed45_20261002 as live
from scripts.e2e import research_temporal_intrinsic_town07_signed45_20261002 as owner
from scripts.e2e.defer_exact_jpeg_until_ACK_research_20261001 import deferred_recording


@contextmanager
def synthetic_scope(tmp_path, monkeypatch, arm="control"):
    pins = {name+"_sha256":str(i)*64 for i,name in enumerate(("weights","metadata","checkpoint","declaration"),1)}
    source = tmp_path/"authority.json"; run=tmp_path/"run"
    value=dict(endpoints={name:dict(pins) for name in owner.ARMS},source_sha256={},arm_order=list(owner.ARMS))
    monkeypatch.setattr(owner.paired,"authority",lambda *a:(source,value,run))
    monkeypatch.setattr(owner.paired,"verify_exports",lambda *a:{})
    monkeypatch.setattr(owner.paired,"evidence_assets",lambda *a:{})
    monkeypatch.setattr(owner,"runtime_sources",lambda:{"synthetic":"a"*64})
    with owner.activated(source,"a"*64,arm):yield


def test_original_capture_and_numeric_binding_provider_identity():
    live.verify_frozen()
    assert live._joint is live.proof._joint
    assert live._original_capture.__globals__ is vars(live.collector)
    assert [p["seed"] for p in live.PROFILES] == [2026102100,2026102101,2026102102]
    assert live.PREVIEW_SCHEMA == live.previous.PREVIEW_SCHEMA


@pytest.mark.parametrize("trial",range(3))
def test_REAL_owned_fixed_plan_argv_public_validator_preflight(tmp_path,monkeypatch,trial):
    before = owner.numeric.MODULE,owner.numeric.WEIGHTS,owner.numeric._exports,owner.base.live,owner.protocol.live,owner.signed.live,owner.paired._scope
    monkeypatch.setattr(owner,"ROOT",tmp_path)
    with synthetic_scope(tmp_path,monkeypatch), owner.protocol.scoped_trial(trial):
        config=owner.fixed_contract("v1",True)
        assert config["schema"]==owner.SCHEMA and config["arm"]=="control"
        assert config["known_total_optimizer_updates"]==18200 and config["parent_known_total_optimizer_updates"]==12200
        assert config["training_source_count"]==29 and config["prospective_physical_case_count"]==6
        assert config["total_capture_seconds"]==45 and config["controller_target_cap_mps"]==2.4
        assert config["causal_selected_speed_preview_calibration"]["configured_speed_preview_s"]==.5
        assert config["cpu_thread_numerical_preset"]["intra_op_threads"]==4
        assert config["sensor_and_ACK_screen"]["native_state_count"]==900
        assert config["body_clearance_screen"]["all_snapshot_linear_sweeps"]==899
        assert config["worker_command"][config["worker_command"].index("-m")+1]==owner.MODULE
        assert "--arm" in config["worker_command"] and "--completed-pair-authority" in config["worker_command"]
        args=live.frozen.parse_args(owner.base.capture_argv("v1",True,"a"*64))
        live.frozen.validate_capture(args,live.collector.load_route(args.route_file))
        assert args.seed==live.PROFILES[trial]["seed"] and args.agent_initialization=="after_bootstrap"
        assert str(args.research_weights)==str(owner._scope.weights)
        with ExitStack() as stack:
            stack.enter_context(mock.patch.object(owner.base,"digest",return_value="a"*64))
            stack.enter_context(mock.patch.object(owner.base,"runtime_sources",return_value={"synthetic":"a"*64}))
            stack.enter_context(mock.patch.object(owner.base,"external_sources",return_value={"api":"a"*64}))
            stack.enter_context(mock.patch.object(owner.base,"ASSETS",{p:"a"*64 for p in owner.base.ASSETS}))
            plan=owner.base.build_plan("v1",True)
            stack.enter_context(mock.patch.object(owner.base,"verify_plan",return_value=plan))
            stack.enter_context(mock.patch.object(owner.base.lifecycle,"_assert_no_other_carla"))
            stack.enter_context(mock.patch.object(owner.base,"ports_free"))
            stack.enter_context(mock.patch.object(owner.base,"disk_check",return_value={}))
            imported=SimpleNamespace(stdout=str(owner.base.EGG)+"/carla.so\n"+str(owner.base.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n1.8.0\n")
            stack.enter_context(mock.patch.object(owner.base.subprocess,"run",return_value=imported))
            stack.enter_context(mock.patch.object(owner.base,"load_npz",return_value=SimpleNamespace(_research_temporal_checkpoint_sha256="3"*64)))
            assert owner.base.preflight("v1","a"*64)[1]["status"]=="PREFLIGHT_PASS_NO_SIMULATOR"
    assert before==(owner.numeric.MODULE,owner.numeric.WEIGHTS,owner.numeric._exports,owner.base.live,owner.protocol.live,owner.signed.live,owner.paired._scope)


@pytest.mark.parametrize("fail",[False,True])
def test_REAL_private_worker_model_authority_and_all_capture_scopes_restore(tmp_path,monkeypatch,fail):
    signed=live.previous.previous.previous.previous.previous.previous
    recovery=live.proof.previous.previous.signed1.previous; fast45=recovery.previous
    monkeypatch.setattr(live.proof,"verify_scope",lambda description:None)
    basic=SimpleNamespace(BasicAgent=type("FakeBasicAgent",(),{}))
    before=live.collector.collect_episode,live.collector._save_jpeg,live._joint.validate_model_binding,owner.numeric._owner_plan_sha
    with synthetic_scope(tmp_path,monkeypatch):
        with owner.protocol.scoped_trial(0):args=live.frozen.parse_args(owner.base.capture_argv("v1",True,"a"*64))
        model=SimpleNamespace(model_id=recovery.MODEL_ID,modules=lambda:[],state_dict=lambda:{},
            _research_speed_preview_calibration=live.PREVIEW_SCHEMA,_research_cpu_thread_numerical_revision=live.THREAD_SCHEMA,
            _research_recovery_simulator_control_authorized=True,_research_recovery_numeric_loader_schema=live.proof.LOADER_SCHEMA,
            _research_temporal_export_metadata=dict(arm="control"),_research_intrinsic_completed_pair_authority_sha256="b"*64,
            _research_recovery_owner_binding=dict(schema=live.proof.NUMERIC_OWNER_SCHEMA,arm="control",fixed_endpoint_updates=6000,
                purpose="owned_carla_research_only",simulation_only=True,simulator_control_authorized=True,
                owner_plan_sha256="a"*64,completed_pair_authority_sha256="b"*64,vehicle_control_approved=False,model_promoted=False))
        def actual_worker(*args_):
            assert owner.numeric._owner_plan_sha=="a"*64 and owner.paired._scope.numeric is owner.numeric
            assert owner.numeric._exports==owner._scope.pins
            with live.activated(model,args):
                actual=live.proof._recorder()
                assert actual.enqueue.__self__ is actual
                assert inspect.getclosurevars(live.collector.collect_episode).nonlocals["delegated"] is not live._original_capture
                assert live.collector.checked_agent_initialization_observation is live.manual.checked_first_gear_initialization_observation
                assert live._original_capture.__globals__["_save_jpeg"].__self__ is actual
                if fail:raise RuntimeError("synthetic worker failure")
                return 0
        with ExitStack() as stack:
            stack.enter_context(mock.patch.object(owner.numeric,"_original_worker",side_effect=actual_worker))
            stack.enter_context(mock.patch.object(owner.base,"verify_plan",return_value={}))
            guard=stack.enter_context(mock.patch.object(owner.base,"assert_owned_worker"))
            stack.enter_context(mock.patch.object(recovery,"RouteRecoveryTemporalModelV3",SimpleNamespace))
            stack.enter_context(mock.patch.object(fast45,"CPUFiniteCachedTemporalModelV2",SimpleNamespace))
            stack.enter_context(mock.patch.object(live.previous.previous.torch,"get_num_threads",return_value=4))
            stack.enter_context(mock.patch.object(live.previous.previous.torch,"get_num_interop_threads",return_value=1))
            stack.enter_context(mock.patch.dict(sys.modules,{"agents":SimpleNamespace(),"agents.navigation":SimpleNamespace(basic_agent=basic),"agents.navigation.basic_agent":basic}))
            if fail:
                with pytest.raises(RuntimeError,match="synthetic worker"):owner.base.worker("v1","a"*64)
            else:assert owner.base.worker("v1","a"*64)==0
            guard.assert_called_once_with({},"a"*64)
        assert owner.numeric._owner_plan_sha is None
    assert before==(live.collector.collect_episode,live.collector._save_jpeg,live._joint.validate_model_binding,owner.numeric._owner_plan_sha)


def test_SIX_plan_declarations_before_any_execution(tmp_path,monkeypatch):
    monkeypatch.setattr(owner,"ROOT",tmp_path);seen=[]
    with synthetic_scope(tmp_path,monkeypatch):pass
    @contextmanager
    def scope(path,sha,arm):
        assert not owner.pair_path().exists()
        owner._scope=SimpleNamespace(arm=arm)
        try:yield
        finally:owner._scope=None
    def declare(run):
        arm=owner._scope.arm
        for trial in range(3):
            seen.append((arm,trial));assert not owner.pair_path().exists()
        output,path=owner.batch_paths(run);output.mkdir(parents=True)
        owner.write_new(path,dict(trials=[dict(trial_index=i) for i in range(3)]))
        return dict(batch_declaration_sha256=owner.digest(path))
    monkeypatch.setattr(owner,"activated",scope);monkeypatch.setattr(owner.protocol,"declare_batch",declare)
    assert owner.main(["--mode","declare","--completed-pair-authority",str(tmp_path/"auth.json"),"--completed-pair-authority-sha256","a"*64])==0
    receipt=json.loads(owner.pair_path().read_text())
    assert seen==[(arm,trial) for arm in owner.ARMS for trial in range(3)]
    assert receipt["physical_case_count"]==6 and receipt["simulator_started"] is False


def test_SIX_archive_reserve_not_three(tmp_path,monkeypatch):
    with synthetic_scope(tmp_path,monkeypatch):
        expected=owner.base.FLOOR+6*owner.base.ARM_RESERVE+owner.base.SHARED_RESERVE
        assert owner.disk_check(expected)["prospective_trials"]==6
        with pytest.raises(Exception):owner.disk_check(expected-1)


def test_every_owner_alias_restores_on_failure(tmp_path,monkeypatch):
    before=owner.base.live,owner.numeric.WEIGHTS,owner.numeric.MODULE,owner.numeric._exports,owner.signed.live,owner.protocol.fixed_contract,owner.paired.MODULE,owner.paired._scope
    with pytest.raises(RuntimeError,match="sentinel"):
        with synthetic_scope(tmp_path,monkeypatch,"intrinsic_curve"):raise RuntimeError("sentinel")
    assert before==(owner.base.live,owner.numeric.WEIGHTS,owner.numeric.MODULE,owner.numeric._exports,owner.signed.live,owner.protocol.fixed_contract,owner.paired.MODULE,owner.paired._scope)
    assert owner._scope is None and not owner._active


def test_REAL_full_original_capture_ALL_source_JPEGs_after_same_ACK(tmp_path,monkeypatch):
    # HH_261002 - Adapt ONLY the test harness from compiled Town03 to the ORIGINAL live-module capture; physics code is unchanged.
    root=Path(__file__).resolve().parents[2]
    path=root/"learning/tests/test_temporal_route_joint_town03_right_v3_20261002.py"
    spec=importlib.util.spec_from_file_location("hh_signed45_original_capture_fixture",path)
    fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
    function=next(n for n in ast.parse(path.read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name=="run_compiled_capture")
    changed=copy.deepcopy(function); anchors=[]
    for node in ast.walk(changed):
        if isinstance(node,ast.Assign) and ast.unparse(node.targets[0])=="capture" and ast.unparse(node.value)=="live.runtime_factory()":
            node.value=ast.parse("live._original_capture",mode="eval").body;anchors.append("original_capture")
        if isinstance(node,ast.Assign) and ast.unparse(node.targets[0])=="function" and "getclosurevars(capture)" in ast.unparse(node.value):
            node.value=ast.parse("capture",mode="eval").body;anchors.append("live_globals")
        if isinstance(node,ast.keyword) and node.arg=="_save_jpeg":
            node.value=ast.parse("source_save",mode="eval").body;anchors.append("synthetic_writer")
        if isinstance(node,ast.Constant) and node.value=="actual_runtime_binding.json":
            node.value="actual_recording_binding.json";anchors.append("proof_filename")
    assert sorted(anchors)==["live_globals","original_capture","proof_filename","proof_filename","synthetic_writer"]
    view=SimpleNamespace(**dict(vars(live.proof),_original_capture=live._original_capture,
        capture_factory=lambda:live.capture_factory(live._original_capture),runtime_factory=lambda:live._original_capture))
    saves=[]
    def source_save(image,output,quality):
        assert type(image.frame) is int
        output.parent.mkdir(parents=True,exist_ok=True)
        with output.open("xb") as stream:stream.write(b"synthetic source JPEG bytes, not real evidence")
        saves.append((image.frame,str(output)))
    def namespace(*args,**kwargs):
        if type(kwargs.get("timestamp")) in (int,float) and "frame" not in kwargs:kwargs["frame"]=int(round(kwargs["timestamp"]/.05))
        return SimpleNamespace(*args,**kwargs)
    ns=dict(vars(fixture),live=view,source_save=source_save,SimpleNamespace=namespace)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[changed],type_ignores=[])),__file__+":original_capture_only_test","exec"),ns)
    original=live.frozen.activated
    @contextmanager
    def recorded(model,args):
        with original(model,args),deferred_recording(live.collector,args.output):yield
    before=live.collector._save_jpeg,live.collector.AcknowledgedControlTransport,live.collector.exact_camera_bundle
    monkeypatch.setattr(live.frozen,"activated",recorded)
    calls,states,manifest,error,proof=ns["run_compiled_capture"](tmp_path)
    assert error.rstrip().endswith("RuntimeError: intentional next-tick failure after real first-fresh proposal")
    records=[json.loads(line) for line in (tmp_path/"episode.partial/recording_after_ack.jsonl").read_text().splitlines()]
    assert len(states)==25 and len(records)==proof["actual_recording_bundle_count"]==13
    assert len(saves)==78 and proof["actual_after_ACK_receipt_count"]==13 and proof["pending_source_camera_count"]==0
    assert proof["original_capture_live_module_globals"] and proof["enqueue_and_ACK_share_exact_recorder"]
    assert not proof["exposure_to_ACK_latency_measured"]
    assert all(r["written_camera_count"]==r["enqueued_camera_count"]==6 and r["all_queued_images_written"]
        and r["acknowledged_before_recording"] and r["failure"] is None for r in records)
    assert [r["frame"] for r in records]==list(range(102,127,2))
    assert sum(c[0]=="observe" for c in calls)==13 and ("propose","next_driving_control") in calls
    assert ("ACK","exception_cleanup_abort") in calls and manifest["cleanup"]["completed"]
    assert before==(live.collector._save_jpeg,live.collector.AcknowledgedControlTransport,live.collector.exact_camera_bundle)


@pytest.mark.parametrize("fault",[None,"other_arm_case","other_arm_batch","path_escape","case_order","retry","authority"])
def test_ALL_SIX_cases_authenticated_even_before_control_first_trial(tmp_path,monkeypatch,fault):
    monkeypatch.setattr(owner,"ROOT",tmp_path)
    owner.pair_root().mkdir(parents=True)
    pins={}
    for arm in owner.ARMS:
        cases=[]
        for index in range(3):
            path=owner.pair_root()/(arm+"_trial_%02d_plan.json"%index)
            owner.write_new(path,dict(arm=arm,trial_index=index))
            cases.append(dict(trial_index=index,declaration_path=str(path),declaration_sha256=owner.digest(path)))
        if fault=="path_escape" and arm=="intrinsic_curve":cases[2]["declaration_path"]=str(tmp_path/"other.json")
        if fault=="case_order" and arm=="intrinsic_curve":cases.reverse()
        batch=owner.pair_root()/(arm+"_batch_plan.json")
        owner.write_new(batch,dict(schema=owner.SCHEMA+".batch",prospective_trial_count=3,trials=cases))
        pins[arm]=owner.digest(batch)
    value=dict(schema=owner.SCHEMA+".paired_batch",arm_order=list(owner.ARMS),physical_case_count=6,trial_count_per_arm=3,
        completed_pair_authority_sha256="b"*64,arm_batch_sha256=pins,automatic_retry=fault=="retry",
        outcome_selected_retry=False,model_promoted=False,vehicle_control_approved=False)
    owner.write_new(owner.pair_path(),value);expected=owner.digest(owner.pair_path())
    if fault=="other_arm_case":(owner.pair_root()/"intrinsic_curve_trial_02_plan.json").write_bytes(b"unselected future case mutated")
    if fault=="other_arm_batch":(owner.pair_root()/"intrinsic_curve_batch_plan.json").write_bytes(b"unselected future batch mutated")
    with synthetic_scope(tmp_path,monkeypatch):
        owner._scope.authority_sha="a"*64 if fault=="authority" else "b"*64
        if fault is None:assert owner.verify_pair(expected)==value
        else:
            with pytest.raises(Exception):owner.verify_pair(expected)


@pytest.mark.parametrize("wall",[False,True])
def test_final_PASS_requires_unchanged_wall_conjunction(tmp_path,monkeypatch,wall):
    monkeypatch.setattr(owner,"_diagnostic",lambda *a:dict(learned_driving_screen_passed=True,
        predeclared_observed_bundle_to_ACK_wall_latency_screen_passed=wall))
    monkeypatch.setattr(owner.readback,"configuration_report",lambda *a:dict(passed=True))
    monkeypatch.setattr(owner.paired.old.threads,"readback_report",lambda *a:dict(passed=True))
    monkeypatch.setattr(owner.readback.proto,"acceleration_report",lambda *a:dict(passed=True))
    monkeypatch.setattr(owner.protocol,"verify_derived_source_closure",lambda *a:None)
    monkeypatch.setattr(owner,"read_json",lambda *a:dict(schema=live.SCHEMA+".actual_recording_binding",
        original_capture_live_module_globals=True,all_actual_providers_current=True,copied_mutable_provider_names=list(live.NAMES),
        enqueue_and_ACK_share_exact_recorder=True,actual_recording_bundle_count=450,actual_after_ACK_receipt_count=450,pending_source_camera_count=0))
    report=owner.diagnostic_report(tmp_path,dict(arm="control"),{})
    assert report["learned_driving_screen_passed"] is True
    assert report["joint_motion_body_policy_wall100ms_passed"] is wall
    assert report["status"].startswith("PASS")==wall
