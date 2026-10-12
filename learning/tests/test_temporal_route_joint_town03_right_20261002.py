"""HH_261002 - Synthetic/source-only known-DEV turn proof; never launch a simulator or load model weights."""

import ast
from contextlib import contextmanager,ExitStack
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

from scripts.e2e import collect_temporal_route_joint_town03_right_20261002 as live
from scripts.e2e import research_temporal_route_joint_town03_right_20261002 as owner


def pins():return {name+"_sha256":str(i)*64 for i,name in enumerate(("weights","metadata","checkpoint","declaration"),1)}


def args():
    with mock.patch.object(owner,"_pins",pins()):
        return live.frozen.parse_args(owner.capture_argv("v999",True,"a"*64))


def route():return live.collector.load_route(owner.ROUTE)


def test_exact_original_route_and_uint32_actual_numpy_seed():
    assert owner.digest(owner.ROUTE)==live.ROUTE_SHA and len(route()["route"])==213
    state=np.random.get_state()
    try:np.random.seed(live.SEED)
    finally:np.random.set_state(state)
    assert live.profile(live.SEED)["signed_lateral_offset_ros_m"]==0


@pytest.mark.parametrize("seed",[True,False,-1,2**32,20261002100,2026102100,1.])
def test_exact_seed_and_boolean_negative(seed):
    with pytest.raises(Exception):live.profile(seed)


def test_real_parser_and_public_validator_no_Town07_scope_needed():
    actual=args();live.validate_capture(actual,route())
    assert actual.max_duration_sec==110 and actual.stationary_tail_sec==0
    phases=live.collector.capture_phase_schedule(actual.physics_hz,actual.max_duration_sec,actual.stationary_warmup_sec,actual.stationary_tail_sec)
    assert phases["stationary_warmup"]["scheduled_ticks"]==24
    assert phases["driving"]["maximum_ticks"]==2176
    assert phases["stationary_tail"]["scheduled_ticks"]==0


@pytest.mark.parametrize("name,value",[("max_duration_sec",111.2),("stationary_tail_sec",6.5),("agent_initialization","before_bootstrap"),
    ("allow_map_load",True),("port",2000),("research_longitudinal_pi",False),("research_speed_cap_mps",2.6)])
def test_original_safety_and_exact_total_budget_not_loosened(name,value):
    actual=args();setattr(actual,name,value)
    with pytest.raises(Exception):live.validate_capture(actual,route())


def test_complete_collect_AST_inverse_proof_and_post_recorded_event_only():
    with mock.patch.object(live,"_observer",SimpleNamespace(completed=False)):
        function=live.cloned_collect_episode()
    assert callable(function)
    # HH_261002 - The generated function's source closure retains original native/control/map cleanup dependencies.
    assert function.__globals__["collector"] if "collector" in function.__globals__ else function.__globals__["CollectionError"]
    assert function.__globals__["observed_after_tick"] is live.observed_after_tick


@pytest.mark.parametrize("failure",[False,True])
def test_event_summary_written_before_outer_partial_to_final_rename(tmp_path,failure):
    partial=tmp_path/"episode.partial";partial.mkdir();manifest={}
    o=observer()
    def original(*a):
        assert a[3]==partial and manifest["capture_contract"]["known_DEV_right_turn_event"]["dataset_split"]=="val"
        if failure:raise RuntimeError("original capture error")
        return "original result"
    with mock.patch.object(live,"_observer",o):
        if failure:
            with pytest.raises(RuntimeError,match="original capture error"):
                live.run_event_capture(original,None,None,None,partial,None,None,manifest)
        else:assert live.run_event_capture(original,None,None,None,partial,None,None,manifest)=="original result"
    assert json.loads((partial/"known_dev_right_turn_summary.json").read_text())["completed"] is False
    final=tmp_path/"episode";partial.rename(final)
    assert (final/"known_dev_right_turn_summary.json").is_file()


def synthetic_route():
    return dict(route=[dict(x=0.,y=0.,distance_m=0.,vad_command=3),
        dict(x=0.,y=1.,distance_m=1.,vad_command=1),dict(x=25.,y=1.,distance_m=26.,vad_command=3)])


def row(i,x,y,yaw,**extra):
    return dict(frame=100+i,timestamp=i*.05,x=x,y=y,yaw=yaw,capture_phase="driving",collision=False,lane_invasion=False,**extra)


def observer():
    return live.RightTurnObserver(synthetic_route())


def test_raw_projection_retains_regression_frozen_navigation_still_clamps():
    projector=live.collector.RouteProjector(synthetic_route())
    assert projector.project(4.,1.,5.).progress_m==5.
    assert live.raw_project_function()(projector,4.,1.,5.).progress_m==5. # route arc1+XY4
    assert live.raw_project_function()(projector,3.,1.,5.).progress_m==4.


def drive_observer(*,bad_frame=None,outgoing=True):
    o=observer()
    for i in range(1,101):
        if i<=10:state=row(i,0.,i*.1,math.pi/2)
        else:state=row(i,(i-10)*.12,1.,0.)
        if o.observe(state,learned_effect=i!=bad_frame,outgoing_safe=outgoing):break
    return o


def test_actual_right_entry_then_exit_actual_arc_and_continuous_three_seconds():
    with mock.patch.object(live,"ENTRY",.8),mock.patch.object(live,"EXIT",1.2):
        o=drive_observer()
    assert o.completed and o.records[-1]["actual_unwrapped_turn_deg"]==pytest.approx(-90)
    assert o.records[-1]["post_exit_actual_rear_XY_arc_m"]>=10
    assert o.records[-1]["stable_postturn_seconds"]>=3
    assert o.summary()["event_is_not_catalog_goal_or_learned_STOP"]


@pytest.mark.parametrize("bad_frame",[8,20,50])
def test_any_missing_teacher_or_emergency_tick_after_entry_cannot_pass(bad_frame):
    with mock.patch.object(live,"ENTRY",.8),mock.patch.object(live,"EXIT",1.2):o=drive_observer(bad_frame=bad_frame)
    assert not o.completed and o.invalid_after_entry


def test_no_learned_entry_or_outgoing_safety_failure_cannot_close():
    with mock.patch.object(live,"ENTRY",.8),mock.patch.object(live,"EXIT",1.2):o=drive_observer(outgoing=False)
    assert not o.completed


def test_backtracking_or_zigzag_distance_cannot_fake_net_ten_meter_exit():
    with mock.patch.object(live,"ENTRY",.8),mock.patch.object(live,"EXIT",1.2):
        o=observer()
        o.observe(row(1,0.,.7,math.pi/2),learned_effect=True,outgoing_safe=True)
        o.observe(row(2,0.,.8,math.pi/2),learned_effect=True,outgoing_safe=True)
        for i in range(3,103):
            x=.3+(i%2)*.3
            o.observe(row(i,x,1.,0.),learned_effect=True,outgoing_safe=True)
    assert o.post_distance>10 and not o.completed and o.progress<live.POST_DISTANCE


def test_no_fake_turn_from_wrapped_heading():
    with mock.patch.object(live,"ENTRY",.8),mock.patch.object(live,"EXIT",1.2):
        o=observer();o.observe(row(1,0.,.7,math.radians(179)),learned_effect=True,outgoing_safe=True)
        o.observe(row(2,0.,.8,math.radians(179)),learned_effect=True,outgoing_safe=True)
        o.observe(row(3,.12,1.,math.radians(-179)),learned_effect=True,outgoing_safe=True)
    assert o.records[-1]["actual_unwrapped_turn_deg"]==pytest.approx(2.)


@pytest.mark.parametrize("change",["frame_gap","time_gap","NaN","bool"])
def test_actual_chain_finite_boolean_guards(change):
    o=observer();o.observe(row(1,0.,.1,math.pi/2),learned_effect=True,outgoing_safe=True)
    candidate=row(2,0.,.2,math.pi/2)
    if change=="frame_gap":candidate["frame"]+=1
    elif change=="time_gap":candidate["timestamp"]+=.01
    elif change=="NaN":candidate["yaw"]=float("nan")
    else:candidate["x"]=False
    with pytest.raises(Exception):o.observe(candidate,learned_effect=True,outgoing_safe=True)


def test_ACK_index_uses_current_prior_effect_not_outgoing_annotation(tmp_path):
    o=observer();state=row(1,0.,.1,math.pi/2,control_transport=dict(expected_receipt_sequence=7,status="PASS"))
    session=SimpleNamespace(wall_timing_records=[dict(ACK_sequence=7,ACK_server_accepted=True,learned_control_proposed=True,
        control_source="learned_trajectory_tracker")],latched_emergency=None,output=tmp_path)
    o.session=session
    with mock.patch.object(live,"_observer",o):
        assert not live.observed_after_tick([state])
        session.wall_timing_records.append(dict(ACK_sequence=8,ACK_server_accepted=True,learned_control_proposed=False,control_source="expert_startup"))
        state=row(2,0.,.2,math.pi/2,control_transport=dict(expected_receipt_sequence=8,status="PASS"),research_control_source="learned_trajectory_tracker")
        assert not live.observed_after_tick([state])
    assert o.ACK_cursor==2 and not o.records[-1]["current_prior_ACK_is_learned_effect"]


def test_known_DEV_MRO_clone_keeps_dynamic_policy_and_preview_globals():
    modules={live.generic:("TemporalLiveResearchSessionV2",),live.owned:("OwnedCameraTemporalLiveResearchSessionV2",),
        live.previous.previous:("Thread4ObservedSession","_original_session"),live.previous:("Preview05ObservedSession","_original_session")}
    saved={m:{n:getattr(m,n) for n in names} for m,names in modules.items()}
    try:
        cls,thread=live.known_dev_session();causal=cls.__mro__[4]
        assert causal.propose.__globals__ is vars(live.generic)
        assert "val" in causal.propose.__code__.co_consts and "train" not in causal.propose.__code__.co_consts
        assert "val" in causal.observe_bundle.__code__.co_consts and "train" not in causal.observe_bundle.__code__.co_consts
        assert cls.__mro__[1].propose.__globals__ is vars(live.previous)
        assert thread.propose.__globals__["_original_session"] is cls.__mro__[3]
    finally:
        for m,values in saved.items():
            for n,v in values.items():setattr(m,n,v)


@pytest.mark.parametrize("fail",[False,True])
def test_actual_collector_scope_restores_every_new_class_alias_on_error(fail):
    saved=(live.generic.activated,live.generic.TemporalLiveResearchSessionV2,live.owned.OwnedCameraTemporalLiveResearchSessionV2,
        live.previous.Preview05ObservedSession,live.previous.previous.Thread4ObservedSession,live.collector.collect_episode,
        live.signed2.profile,live.signed3.validate_capture)
    @contextmanager
    def delegate(*a):
        assert live.previous.Preview05ObservedSession.__name__=="KnownDevRightObservedSession"
        assert live.signed3.profile(live.SEED)["signed_yaw_offset_ros_deg"]==0
        if fail:raise RuntimeError("original simulated failure")
        yield
    with mock.patch.object(live.previous.previous.previous,"validate_model_binding"),mock.patch.object(live.previous,"activated",side_effect=delegate):
        if fail:
            with pytest.raises(RuntimeError,match="original simulated"): 
                with live.activated(SimpleNamespace(),args()):pass
        else:
            with live.activated(SimpleNamespace(),args()):
                with pytest.raises(Exception):
                    with live.activated(SimpleNamespace(),args()):pass
    assert saved==(live.generic.activated,live.generic.TemporalLiveResearchSessionV2,live.owned.OwnedCameraTemporalLiveResearchSessionV2,
        live.previous.Preview05ObservedSession,live.previous.previous.Thread4ObservedSession,live.collector.collect_episode,
        live.signed2.profile,live.signed3.validate_capture)
    assert not live._active and live._observer is None


@contextmanager
def owner_scope():
    with mock.patch.object(owner.numeric,"verify_exports"),mock.patch.object(owner.threads,"evidence_assets",return_value={}):
        with owner.activated(pins()):yield


def test_new_fixed_contract_single_knownDEV_and_exact_original_safety():
    with owner_scope():
        contract=owner.fixed_contract("v999",True)
        assert contract["town"]=="Town03" and contract["dataset_split"]=="val"
        assert contract["total_capture_seconds"]==110 and contract["maximum_driving_native_ticks"]==2176
        assert contract["event"]["event_completion_is_not_goal_reached"]
        assert contract["precursor_motion_screen"]["net_learned_progress_minimum_m"]==90
        assert contract["model_speed_cap_mps"]==2.5 and contract["policy_deadline_seconds"]==.1
        assert contract["speed_preview_s"]==.5 and contract["controller_target_cap_mps"]==2.4
        assert contract["server_command"][2]=="Town03" and "--" in contract["worker_command"]
        assert not contract["full_RTE03_approved"] and not contract["training_data_approved"]


def test_real_base_build_plan_parse_capture_and_public_preflight_path(tmp_path):
    # HH_261002 - Exercise the original outside-activation preflight validator, not only a convenient new helper.
    imported=SimpleNamespace(stdout=str(owner.base.EGG)+"/carla.so\n"+str(owner.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n1.8.0\n")
    with owner_scope(),ExitStack() as stack:
        stack.enter_context(mock.patch.object(owner,"digest",return_value="a"*64))
        stack.enter_context(mock.patch.object(owner.base,"digest",return_value="a"*64))
        stack.enter_context(mock.patch.object(owner.base,"runtime_sources",return_value={"code":"a"*64}))
        stack.enter_context(mock.patch.object(owner.base,"external_sources",return_value={"API":"a"*64}))
        original_assets=owner.base.ASSETS
        owner.base.ASSETS={path:"a"*64 for path in original_assets}
        try:
            plan=owner.base.build_plan("v999",True)
        finally:owner.base.ASSETS=original_assets
        actual=live.frozen.parse_args(owner.capture_argv("v999",True,"a"*64));live.frozen.validate_capture(actual,route())
        stack.enter_context(mock.patch.object(owner.base,"verify_plan",return_value=plan))
        stack.enter_context(mock.patch.object(owner.base.lifecycle,"_assert_no_other_carla"))
        stack.enter_context(mock.patch.object(owner.base,"ports_free"))
        stack.enter_context(mock.patch.object(owner.base,"disk_check",return_value={}))
        stack.enter_context(mock.patch.object(owner.base.subprocess,"run",return_value=imported))
        stack.enter_context(mock.patch.object(owner.base,"load_npz",return_value=SimpleNamespace(_research_temporal_checkpoint_sha256=pins()["checkpoint_sha256"])))
        _,result=owner.base.preflight("v999","a"*64)
    assert result["status"]=="PREFLIGHT_PASS_NO_SIMULATOR"


def test_owner_restore_source_configuration_and_worker_auth():
    before=(owner.base.ASSETS,owner.base.ROUTE,owner.base.XODR,owner.base.live,owner.base.worker,live.frozen.validate_capture,owner.numeric._exports)
    with pytest.raises(RuntimeError):
        with owner_scope():raise RuntimeError("original")
    assert before==(owner.base.ASSETS,owner.base.ROUTE,owner.base.XODR,owner.base.live,owner.base.worker,live.frozen.validate_capture,owner.numeric._exports)
    with pytest.raises(Exception):owner.private_worker("v999","a"*64)


def test_REAL_entire_inherited_activation_chain_reaches_val_singlecopy_policy_without_simulator():
    # HH_261002 - Mock only actual model/API objects, not any validator/profile/activation scope in the real chain.
    recovery=live.signed1.previous;fast45=recovery.previous
    basic=SimpleNamespace(BasicAgent=type("FakeBasicAgent",(),{}))
    fake=SimpleNamespace(model_id=recovery.MODEL_ID,modules=lambda:[],state_dict=lambda:{},
        _research_speed_preview_calibration=live.PREVIEW_SCHEMA,
        _research_cpu_thread_numerical_revision=live.THREAD_SCHEMA,
        _research_recovery_simulator_control_authorized=True,
        _research_recovery_owner_binding=dict(purpose="owned_carla_research_only",simulation_only=True,owner_plan_sha256="a"*64))
    before=(live.collector.collect_episode,live.generic.TemporalLiveResearchSessionV2,
        live.generic.TemporalSimulationResearchPolicyV2,live.signed2.validate_capture)
    with mock.patch.object(live.previous.previous.previous,"validate_model_binding"), \
        mock.patch.object(recovery,"RouteRecoveryTemporalModelV3",SimpleNamespace), \
        mock.patch.object(fast45,"CPUFiniteCachedTemporalModelV2",SimpleNamespace), \
        mock.patch.object(live.previous.previous.torch,"get_num_threads",return_value=4), \
        mock.patch.object(live.previous.previous.torch,"get_num_interop_threads",return_value=1), \
        mock.patch.dict("sys.modules",{"agents":SimpleNamespace(),"agents.navigation":SimpleNamespace(basic_agent=basic),
            "agents.navigation.basic_agent":basic}):
        with live.activated(fake,args()):
            assert live.generic.TemporalLiveResearchSessionV2.__name__=="KnownDevRightObservedSession"
            assert live.generic.TemporalSimulationResearchPolicyV2 is recovery.RecoverySingleCopyPolicyV3
            live.generic.validate_capture(args(),route())
            assert live.signed2.profile(live.SEED)["seed"]==live.SEED
            assert live.generic.TemporalLiveResearchSessionV2.__mro__[4].propose.__globals__ is vars(live.generic)
    assert before==(live.collector.collect_episode,live.generic.TemporalLiveResearchSessionV2,
        live.generic.TemporalSimulationResearchPolicyV2,live.signed2.validate_capture)


def test_external_source_only_Town03_map_not_false_Town07():
    with mock.patch.object(owner,"digest",return_value=owner.UMAP_SHA):sources=owner.external_sources()
    assert str(owner.UMAP) in sources and not any("Town07.umap" in n for n in sources)


def interval_fixture(count=28,completed=True):
    states=[dict(frame=100+i,capture_phase="stationary_warmup" if i<24 else "driving") for i in range(count)]
    cameras=[dict(frame=s["frame"]) for s in states[::2]]
    manifest=dict(status="complete" if completed else "failed",result=dict(goal_reached=False,
        known_dev_right_turn_event=dict(completed=completed)),capture_contract=dict(
        capture_phase_schedule=dict(stationary_tail=dict(scheduled_ticks=0))))
    if not completed:manifest["error"]="BasicAgent did not finish within the driving-phase tick budget before the maximum capture duration"
    return manifest,states,cameras,dict(completed=completed)


def test_variable_complete_event_and_exact_full110_failed_budget():
    assert owner.complete_interval(*interval_fixture())
    assert owner.complete_interval(*interval_fixture(2200,False))
    assert not owner.complete_interval(*interval_fixture(28,False))


@pytest.mark.parametrize("mutation",["goal_true","gap","missing_camera","tail","too_long","phase"])
def test_no_incomplete_suffix_or_fake_goal_complete_interval(mutation):
    m,s,c,e=interval_fixture()
    if mutation=="goal_true":m["result"]["goal_reached"]=True
    elif mutation=="gap":s[-1]["frame"]+=1
    elif mutation=="missing_camera":c.pop()
    elif mutation=="tail":m["capture_contract"]["capture_phase_schedule"]["stationary_tail"]["scheduled_ticks"]=1
    elif mutation=="too_long":m,s,c,e=interval_fixture(2201)
    else:s[-1]["capture_phase"]="stationary_tail"
    assert not owner.complete_interval(m,s,c,e)


def test_original_hard_abort_still_assesses_recorded_body_and_keeps_all_raw_pins(tmp_path):
    episode=tmp_path/"episode.partial";episode.mkdir()
    for name in ("manifest.json","states.jsonl","camera_frames.jsonl","learned_control.jsonl","control_receipts.jsonl","wall_timing.jsonl"):
        (episode/name).write_text("{}\n")
    states=[dict(frame=i,capture_phase="driving",control_transport={}) for i in (1,2)]
    manifest=dict(status="failed",error="original hard native guard abort")
    original_owner=dict(cleanup_errors=[],source_hashes_match=True,external_source_hashes_match=True)
    body=dict(body_and_swept_clearance_passed=True,all_native_boxes=2,all_snapshot_linear_sweeps=1)
    with mock.patch.object(owner,"read_json",return_value=manifest), \
        mock.patch.object(owner,"jsonl",side_effect=lambda p: states if p.name=="states.jsonl" else []), \
        mock.patch.object(owner,"body_report",return_value=body) as audit, \
        mock.patch.object(owner,"independent_event",side_effect=RuntimeError("missing post-ACK event input")):
        result=owner.diagnostic_report(tmp_path,{},original_owner)
    assert audit.call_count==1 and result["independent_body"]==body
    assert len(result["raw_source_sha256"])==6 and result["raw_before_after_unchanged"]
    assert result["actual_native_state_count"]==2 and not result["all_failed_tail_retained"]
    assert not result["bounded_capture_complete"] and not result["joint_turn_motion_body_policy_wall100ms_passed"]


def test_body_failure_in_abort_is_not_hidden_by_later_reader_error(tmp_path):
    episode=tmp_path/"episode.partial";episode.mkdir();(episode/"manifest.json").write_text("{}")
    states=[dict(frame=i,control_transport={}) for i in (1,2)]
    original_owner=dict(cleanup_errors=[],source_hashes_match=True,external_source_hashes_match=True)
    with mock.patch.object(owner,"read_json",return_value={}), \
        mock.patch.object(owner,"jsonl",side_effect=lambda p:states if p.name=="states.jsonl" else []), \
        mock.patch.object(owner,"body_report",side_effect=RuntimeError("all body inputs denied")), \
        mock.patch.object(owner,"independent_event",side_effect=RuntimeError("later error")):
        result=owner.diagnostic_report(tmp_path,{},original_owner)
    assert result["independent_body"]["status"]=="FAIL" and "body inputs denied" in result["independent_body"]["error"]
    assert result["raw_before_after_unchanged"] and not result["learned_driving_screen_passed"]


@pytest.mark.parametrize("rate,passed",[(3.,True),(3.01,False),(-6.,True),(-6.01,False)])
def test_native_rate_exact_original_additive_limits_and_first_boundary(rate,passed):
    initial=1.;following=initial+.05*rate
    rows=[dict(frame=1,timestamp=0.,capture_phase="stationary_warmup",world_velocity_carla=[initial,0.,0.]),
        dict(frame=2,timestamp=.05,capture_phase="driving",world_velocity_carla=[following,0.,0.]),
        dict(frame=3,timestamp=.10,capture_phase="driving",world_velocity_carla=[following,0.,0.])]
    # HH_261002 - Use exactly representable boundary speeds to avoid asserting false floating-point equality at−6.
    if rate==-6.:rows[0]["world_velocity_carla"][0]=.5;rows[1]["world_velocity_carla"][0]=.2;rows[2]["world_velocity_carla"][0]=.2
    result=owner.native_rate_report(rows)
    assert result["passed"]==passed and result["interval_count"]==2 and result["first_warmup_to_driving_boundary_included"]


@pytest.mark.parametrize("field,value",[("post_handoff_actor_CTE_p95_m",.351),("post_handoff_actor_CTE_maximum_m",.751),
    ("post_handoff_actor_progress_m",89.9),("post_handoff_actor_geometry_denominator",5)])
def test_actor_CTE_progress_thresholds_cannot_be_replaced_with_good_rear_event(field,value):
    body=dict(post_handoff_actor_CTE_p95_m=.35,post_handoff_actor_CTE_maximum_m=.75,
        post_handoff_actor_progress_m=90.,post_handoff_actor_geometry_denominator=6)
    assert owner.actor_motion_report(body,6)
    body[field]=value
    assert not owner.actor_motion_report(body,6)
