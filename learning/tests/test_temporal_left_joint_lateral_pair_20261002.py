"""HH_261002 - Synthetic-only fixed lateral A/B, real inherited capture and namespace regression tests."""
import ast
import copy
from contextlib import ExitStack
from dataclasses import asdict, replace
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import collect_temporal_left_joint_lateral_pair_20261002 as live
from scripts.e2e import research_temporal_left_joint_lateral_pair_20261002 as owner
from portable_e2e.simulation_research import ControllerConfig
from scripts.e2e.defer_exact_jpeg_until_ACK_research_20261001 import deferred_recording


def test_fixed_source_contract_preserves_every_old_gate_and_new_endpoints():
    old=live.previous.source_contract();new=live.source_contract()
    assert new["arms"]==["baseline","short_lookahead"] and new["fixed_endpoint_updates"]==20000
    assert live.PROFILES==dict(baseline=(1.5,.8),short_lookahead=(1.,.5))
    for key,value in old.items():
        if key not in ("schema","arms"):assert new[key]==value
    assert new["matched_baseline_repeat_required"] and not new["outcome_selected_retry"]
    for arm in live.ARMS:
        assert live.calibration_contract(arm)["original_STOP_and_non_GO_tracker_preserved"]
    with pytest.raises(Exception):live.calibration_contract("LEFT20000")


@pytest.mark.parametrize("arm",live.ARMS)
def test_actual_actor_factory_changes_only_two_declared_lateral_fields(monkeypatch,arm):
    old=ControllerConfig(wheelbase_m=2.819186218917395,maximum_steer_angle_rad=.7,speed_preview_s=.5)
    monkeypatch.setattr(live,"_owner",SimpleNamespace(_scope=SimpleNamespace(arm=arm)))
    monkeypatch.setattr(live.preview,"actor_controller_config",lambda *a:old)
    new=live.actor_controller_config(None,2.5)
    assert new.wheelbase_m==old.wheelbase_m
    assert (new.minimum_lookahead_m,new.lookahead_time_s)==live.PROFILES[arm]
    assert {k:v for k,v in asdict(new).items() if k not in ("minimum_lookahead_m","lookahead_time_s")}=={k:v for k,v in asdict(old).items() if k not in ("minimum_lookahead_m","lookahead_time_s")}


@pytest.mark.parametrize("arm",live.ARMS)
@pytest.mark.parametrize("speed",[0.,.5,2.31])
@pytest.mark.parametrize("fallback",[False,True])
def test_exact_tracker_math_singlecall_selected_unchanged_STOP_full_oldtracker(monkeypatch,arm,speed,fallback):
    config=ControllerConfig(wheelbase_m=2.819,maximum_steer_angle_rad=.7,speed_preview_s=.3 if fallback else .5,
        minimum_lookahead_m=live.PROFILES[arm][0],lookahead_time_s=live.PROFILES[arm][1])
    selected=SimpleNamespace(xy_base_m=[(.1*i,.005*i*i) for i in range(1,65)],speed_mps=[.0 if fallback else 2.3]*64,timestep_s=.1)
    before=copy.deepcopy(vars(selected));calls=[]
    original=live._base_tracker
    def counted(*args):calls.append(args);return original(*args)
    monkeypatch.setattr(live,"_active",True);monkeypatch.setattr(live,"_owner",SimpleNamespace(_scope=SimpleNamespace(arm=arm)))
    monkeypatch.setattr(live,"_base_tracker",counted)
    result=live.trajectory_control(selected,speed,config)
    used=replace(config,minimum_lookahead_m=1.5,lookahead_time_s=.8) if fallback else config
    assert result==original(selected,speed,used) and len(calls)==1 and vars(selected)==before
    assert result["desired_speed_mps"]==(.0 if fallback else 2.3)


@pytest.mark.parametrize("arm",live.ARMS)
def test_REAL_task_buildplan_strictparse_public_preflight_not_old_parent_arm(tmp_path,monkeypatch,arm):
    assets={name:tmp_path/(name+".json") for name in ("route","mapping","calibration","xodr")}
    for path in assets.values():path.write_text("{}")
    pins={name:"b"*64 for name in owner.loader.PARENT}
    scope=SimpleNamespace(arm=arm,authority_path=tmp_path/"authority.json",authority_sha="a"*64,
        pins=pins,weights=tmp_path/"weights.npz",metadata=tmp_path/"metadata.json")
    for name,value in dict(_scope=scope,MAPPING=assets["mapping"],CALIBRATION=assets["calibration"],XODR=assets["xodr"],
        ASSETS={p:owner.digest(p) for p in assets.values()}).items():monkeypatch.setattr(owner,name,value)
    monkeypatch.setattr(owner,"root_path",lambda:tmp_path)
    monkeypatch.setattr(live.task,"ROUTE",assets["route"]);monkeypatch.setattr(live.task,"ROUTE_SHA",owner.digest(assets["route"]))
    task=live.task_namespace(owner);monkeypatch.setattr(owner,"task",task)
    monkeypatch.setattr(owner,"runtime_sources",lambda:{})
    monkeypatch.setattr(owner,"external_sources",lambda:{})
    monkeypatch.setattr(owner,"disk_check",lambda:{})
    ns=owner.adapted_originals();monkeypatch.setattr(owner,"_originals",ns)
    plan=ns["build_plan"]("v1","pilot_00");path=tmp_path/(arm+"_plan.json");owner.write_new(path,plan);pin=owner.digest(path)
    assert plan["arm"]==arm and plan["fixed_endpoint_updates"]==20000 and plan["known_total_optimizer_updates"]==32200
    assert plan["actual_training_source_count"]==30 and plan["lateral_lookahead_calibration"]==live.calibration_contract(arm)
    assert plan["arm_order"]==list(live.ARMS) and plan["speed_preview_s"]==.5
    assert ns["verify_case"]("v1","pilot_00",pin)==plan
    parsed=live.parse_args(owner.capture_argv("v1","pilot_00",pin),owner)
    assert parsed.research_source_sha256==parsed.moving_task_plan_sha256==pin
    assert parsed.stationary_tail_sec==6.5 and parsed.agent_initialization=="after_bootstrap"
    with mock.patch.object(ns["lifecycle"].lifecycle,"_assert_no_other_carla"),mock.patch.object(ns["lifecycle"],"ports_free"), \
        mock.patch.object(owner.subprocess,"run",return_value=SimpleNamespace(stdout=str(owner.EGG)+"/carla.so\n"+str(owner.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n")),mock.patch.object(owner,"load_model",return_value="synthetic_numeric") as loaded:
        assert owner.preflight("v1",pin)[1]["status"]=="PREFLIGHT_PASS_NO_SIMULATOR"
    loaded.assert_called_once_with()
    assert owner.previous._scope is None and live.previous._owner is None


@pytest.mark.parametrize("arm",live.ARMS)
def test_both_load_model_branches_use_exact_NEW20k_loader_and_binding(monkeypatch,arm):
    pins={name:"b"*64 for name in owner.loader.PARENT};model=SimpleNamespace()
    scope=SimpleNamespace(arm=arm,authority_sha="c"*64,pins=pins,weights=Path("weights.npz"),metadata=Path("meta.json"),authority_path=Path("authority.json"))
    monkeypatch.setattr(owner,"_scope",scope);monkeypatch.setattr(owner.threads,"configure_threads",lambda:None)
    monkeypatch.setattr(owner.threads.live,"thread_readback",lambda:None)
    with mock.patch.object(owner.loader,"load_left_joint_research_npz",return_value=model) as new, \
        mock.patch.object(owner.numeric,"load_route_joint_research_npz",side_effect=AssertionError("old6000 loader forbidden")):
        assert owner.load_model("a"*64) is model
    assert new.call_count==1 and new.call_args.kwargs["research_owner_binding"]["fixed_endpoint_updates"]==20000
    assert new.call_args.kwargs["research_owner_binding"]["owner_plan_sha256"]=="a"*64
    assert model._research_lateral_lookahead_calibration==live.calibration_contract(arm)


def test_configuration_readback_all_original_ACK_math_and_exactprofile(tmp_path,monkeypatch):
    proto=owner.previous.preview_owner.proto
    tree=ast.parse(Path(proto.__file__).read_bytes());node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="configuration_report")
    assert "complete normal preview readback denominator differs" in ast.dump(node)
    monkeypatch.setattr(owner,"_scope",SimpleNamespace(arm="short_lookahead"))
    # HH_261002 - Empty witness must remain fail-closed; changing only expected literals cannot make an empty denominator pass.
    monkeypatch.setattr(proto.parent,"jsonl",lambda path:[])
    with pytest.raises(Exception,match="complete normal preview"):owner.configuration_report(tmp_path)


@pytest.mark.parametrize("arm",live.ARMS)
@pytest.mark.parametrize("fault",[None,"lateral","steer","time","missing"])
def test_allfresh_exact_configuration_geometry_and_ACK_readback_positive_and_negatives(tmp_path,monkeypatch,arm,fault):
    import json
    c=ControllerConfig(wheelbase_m=2.819,maximum_steer_angle_rad=.7,speed_preview_s=.5,
        minimum_lookahead_m=live.PROFILES[arm][0],lookahead_time_s=live.PROFILES[arm][1])
    selected=SimpleNamespace(xy_base_m=[[.1*i,.005*i*i] for i in range(1,65)],speed_mps=[2.3]*64,timestep_s=.1)
    preview=live.preview.preview_selection(selected,c)
    control=live._base_tracker(selected,.5,c)
    tracker=dict(preview,original_tracker_control=control)
    raw=dict(ack_receipt_sequence=2,command_reason="next_driving_control",control_source="learned_trajectory_tracker",
        learned_control_proposed=True,held_between_camera_frames=False,inference_frame=100,selected=vars(selected),
        controller=asdict(c),control=control,measured_current_speed_mps=.5)
    record=dict(schema=live.PREVIEW_SCHEMA,command_frame=100,command_reason="next_driving_control",actual_policy_controller=asdict(c),
        proposal_started_monotonic_ns=20,proposal_finished_monotonic_ns=30,control_source=raw["control_source"],
        learned_control_proposed=True,held_between_camera_frames=False,inference_frame=100,actual_returned_control=control,tracker_receipt=tracker)
    timing=dict(ACK_sequence=2,command_frame=100,command_reason="next_driving_control",proposal_started_monotonic_ns=10,
        proposal_finished_monotonic_ns=40,ACK_started_monotonic_ns=50,ACK_finished_monotonic_ns=60)
    if fault=="lateral":record["actual_policy_controller"]["lookahead_time_s"]+=.1
    elif fault=="steer":record["tracker_receipt"]["original_tracker_control"]["steer"]+=.1
    elif fault=="time":record["proposal_finished_monotonic_ns"]=61
    payloads={"speed_preview_configuration.jsonl":[record],"wall_timing.jsonl":[timing],"learned_control.jsonl":[raw]}
    if fault=="missing":payloads["wall_timing.jsonl"]=[]
    for name,rows in payloads.items():(tmp_path/name).write_text("".join(json.dumps(r)+"\n" for r in rows))
    monkeypatch.setattr(owner,"_scope",SimpleNamespace(arm=arm))
    if fault is None:
        r=owner.configuration_report(tmp_path)
        assert r["passed"] and r["fresh_tracker_count"]==r["normal_proposal_denominator"]==1
    else:
        with pytest.raises(Exception):owner.configuration_report(tmp_path)


@pytest.mark.parametrize("arm",live.ARMS)
@pytest.mark.parametrize("fault",[None,"oldloader","old6000","wrongplan","preview","thread"])
def test_REAL_model_identity_requires_LEFT20k_for_both_profiles(monkeypatch,arm,fault):
    pins={name:"b"*64 for name in live.loader.PARENT}
    binding=dict(schema=live.loader.OWNER_SCHEMA,purpose="owned_carla_research_only",mode="RESEARCH_ONLY",simulation_only=True,
        owner_plan_sha256="a"*64,simulator_control_authorized=True,model_promoted=False,vehicle_control_approved=False,
        fixed_endpoint_updates=20000,completed_run_authority_sha256="c"*64)
    model=SimpleNamespace(_research_recovery_owner_binding=binding,_research_recovery_simulator_control_authorized=True,
        _research_recovery_numeric_loader_schema=live.loader.SCHEMA,_research_temporal_export_metadata=dict(optimizer_updates=20000),
        _research_temporal_checkpoint_sha256="b"*64,_research_temporal_export_npz_sha256="b"*64,_research_temporal_metadata_sha256="b"*64,
        _research_cpu_thread_numerical_revision=live.THREAD_SCHEMA,_research_speed_preview_calibration=live.PREVIEW_SCHEMA)
    monkeypatch.setattr(live,"_owner",SimpleNamespace(_scope=SimpleNamespace(arm=arm,pins=pins,authority_sha="c"*64)))
    monkeypatch.setattr(live.loader,"verify_scope",lambda d:None)
    if fault=="oldloader":model._research_recovery_numeric_loader_schema="old_joint6000"
    elif fault=="old6000":binding["fixed_endpoint_updates"]=6000
    elif fault=="wrongplan":binding["owner_plan_sha256"]="d"*64
    elif fault=="preview":model._research_speed_preview_calibration="wrong"
    elif fault=="thread":model._research_cpu_thread_numerical_revision="wrong"
    if fault is None:live.verify_model(model,SimpleNamespace(research_source_sha256="a"*64))
    else:
        with pytest.raises(Exception):live.verify_model(model,SimpleNamespace(research_source_sha256="a"*64))


def old_fixture(arm):
    # HH_261002 - Recompile the exact full frozen fake-CARLA capture fixture; only its prospective arm/model declaration changes.
    import math,json,sys
    from collections import deque
    from contextlib import contextmanager
    from dataclasses import dataclass
    from types import ModuleType
    path=owner.ROOT/"learning/tests/test_temporal_left_joint_left45_20261002.py"
    tree=ast.parse(path.read_bytes());node=copy.deepcopy(next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="compiled_capture"))
    for n in ast.walk(node):
        if isinstance(n,ast.Constant) and n.value=="parent6000":n.value=arm
    calls=[n for n in ast.walk(node) if isinstance(n,ast.Call) and ast.dump(n.func)==ast.dump(ast.parse("live.activated",mode="eval").body)]
    assert len(calls)==1
    calls[0].args[0]=ast.parse("SimpleNamespace(_research_lateral_lookahead_calibration=live.calibration_contract(arm))",mode="eval").body
    namespace=dict(globals(),deque=deque,contextmanager=contextmanager,dataclass=dataclass,ModuleType=ModuleType,
        math=math,json=json,sys=sys,arm=arm,Control=type("Control",(SimpleNamespace,),{"__init__":lambda self,**kw:SimpleNamespace.__init__(self,**dict(dict(throttle=0.,brake=0.,steer=0.,reverse=False,hand_brake=False,gear=1,manual_gear_shift=True),**kw))}))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node],type_ignores=[])),__file__,"exec"),namespace)
    return namespace["compiled_capture"]


@pytest.mark.parametrize("arm",live.ARMS)
@pytest.mark.parametrize("failure",[None,"tick","bundle","proposal"])
def test_REAL_compiled_sixproviders_ACK899_boundary900_coast_and_exception_restore(tmp_path,monkeypatch,arm,failure):
    states,cameras,calls,manifest,error,proof=old_fixture(arm)(tmp_path,monkeypatch,
        fail_at=127 if failure=="tick" else None,drop_bundle=failure=="bundle",drop_proposal=failure=="proposal")
    assert live.preview._trajectory_control is live._base_tracker
    if failure is None:
        assert error is None,str(error)
        assert (len(states),len(cameras),proof["actual_after_ACK_receipt_count"])==(1055,528,528)
        drive=[s for s in states if s["capture_phase"]=="driving"]
        assert drive[-2]["moving_task"]["driving_relative_native_frames"]==899 and drive[-2]["research_control_source"]=="learned_trajectory_tracker"
        assert drive[-1]["moving_task"]["driving_relative_native_frames"]==900 and drive[-1]["research_control_source"]=="none"
        assert all(s["current_control"]["brake"]==s["current_control"]["throttle"]==0. for s in states[-130:])
        assert any(c[:2]==("ACK","left45_post_capture_cleanup_full_brake") for c in calls)
        import json
        commands=[json.loads(s) for s in (tmp_path/"episode.partial/learned_control.jsonl").read_text().splitlines()]
        assert commands and all(row["decoder_gate_tracker_math_unchanged_except_disclosed_headroom_and_speed_preview"] is False
            and row["decoder_gate_tracker_math_unchanged_except_disclosed_headroom_speed_preview_and_lateral"] is True
            and row["causal_lateral_lookahead_calibration"]==live.calibration_contract(arm) for row in commands)
    else:
        assert error is not None and len(states)<=25
        assert any(c[:2]==("ACK","exception_cleanup_abort") for c in calls)
    assert proof["pending_source_camera_count"]==0 and sum(c[0]=="fixture_restored" for c in calls)==1


def test_fixed_arc_bins_include_all_rows_overlap_and_undefined_stationary_not_fakezero():
    states=[dict(frame=i,x=float(i),y=0.,vx=1.,vy=0.) for i in range(9)]
    rows=[dict(frame=i,prior_ACK_sequence=i,actual_prior_learned_ACK_effect=True,unclamped_rear_XY_progress_m=float(i)) for i in range(9)]
    decisions=[dict(ack_receipt_sequence=i,control=dict(pursuit_xy_base_m=[1.,.1])) for i in range(9)]
    motion=dict(original_XY_turn_entry_m=2.,original_XY_turn_exit_m=6.,all_driving_rows=rows)
    r=owner.segment_diagnostics(states,motion,decisions,[])
    assert r["fixed_arc_centers_m"]==dict(entry=2.,middle=4.,exit=6.)
    assert {k:len(v) for k,v in r["all_in_band_rows"].items()}==dict(entry=5,middle=5,exit=5)
    assert r["all_in_band_rows"]["entry"][0]["actual_rear_three_pose_signed_curvature_1pm"] is None
    assert r["all_in_band_rows"]["entry"][1]["actual_rear_three_pose_signed_curvature_1pm"]==0.
    assert r["all_in_band_rows"]["exit"][-1]["prior_learned_pursuit_curvature_1pm"]==2*.1/(1+.1**2)
    assert r["not_an_additional_PASS_threshold"]


def test_original_full_capture_and_empty_failure_gates_are_not_changed(tmp_path):
    (tmp_path/"episode.partial").mkdir();(tmp_path/"episode.partial/manifest.json").write_text("{}")
    result=owner.diagnostic_report(tmp_path,dict(arm="baseline"),dict(cleanup_errors=[],source_hashes_match=True,external_source_hashes_match=True))
    assert not result["joint_motion_body_policy_wall100ms_passed"] and result["diagnostic_error"]
    assert result["raw_source_sha256"] and result["lateral_lookahead_calibration"]==live.calibration_contract("baseline")
