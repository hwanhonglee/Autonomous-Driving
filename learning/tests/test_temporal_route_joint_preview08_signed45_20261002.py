"""HH_261002 - Prove causal preview-only calibration, original STOP behavior and fully restored ownership hooks."""

from contextlib import contextmanager, ExitStack
from dataclasses import asdict, replace
import json
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from portable_e2e.runtime_contract import SelectedTrajectory
from portable_e2e.simulation_research import ControllerConfig
from scripts.e2e import collect_temporal_route_joint_preview08_signed45_20261002 as live
from scripts.e2e import research_temporal_route_joint_preview08_signed45_town07_20261002 as owner
from test_temporal_route_joint_thread4_signed45_20261002 import args, pins


def selected(speeds=None):
    return SelectedTrajectory(0, tuple((float(i+1)*.2, .03) for i in range(64)),
        tuple(speeds if speeds is not None else (.1+i*.025 for i in range(64))), .1, 0., 12.8)


def config():
    return ControllerConfig(2.819186218917395, .65, speed_preview_s=.8)


def test_only_preview_field_changes_with_measured_actor_geometry():
    original=replace(config(),speed_preview_s=.3)
    with mock.patch.object(live,"_actor_config",return_value=original) as delegate:
        actual=live.actor_controller_config("physics",2.5)
    delegate.assert_called_once_with("physics",2.5)
    assert asdict(actual)==dict(asdict(original),speed_preview_s=.8)
    assert actual.wheelbase_m!=2.85


def test_unknown_original_preview_factory_rejects():
    with mock.patch.object(live,"_actor_config",return_value=config()),pytest.raises(Exception):
        live.actor_controller_config("p",2.5)


def test_exact_original_round_indices_and_no_speed_floor():
    used=selected(); actual=live.preview_selection(used,config())
    assert actual["original_preview_index"]==2 and actual["proposed_preview_index"]==7
    assert actual["original_selected_target_mps"]==used.speed_mps[2]
    assert actual["effective_selected_target_mps"]==used.speed_mps[7]
    assert actual["effective_speed_preview_s"]==.8
    assert not actual["selected_trajectory_changed"] and not actual["actual_measured_speed_clipped"]


@pytest.mark.parametrize("position,value",[(2,0.),(2,.05),(7,0.),(7,.05)])
def test_either_stop_target_preserves_entire_original_tracker(position,value):
    speeds=[.6]*64;speeds[position]=value
    chosen=selected(speeds); scope=SimpleNamespace(_preview_tracker_receipt=None)
    with mock.patch.object(live,"_active",True),mock.patch.object(live,"_proposal_session",scope):
        result=live.trajectory_control(chosen,.4,config())
    assert result==live._trajectory_control(chosen,.4,replace(config(),speed_preview_s=.3))
    assert scope._preview_tracker_receipt["effective_speed_preview_s"]==.3
    if position==7 and value<=.05:
        assert result["brake"]<.3 and result["desired_speed_mps"]==.6
    else:
        assert result["brake"]>=.3 and result["throttle"]==0.


@pytest.mark.parametrize("intent",[1,2,"GO",None,True])
def test_any_available_noninteger_GO_intent_preserves_old_tracker(intent):
    chosen=SimpleNamespace(**asdict(selected()),intent=intent)
    assert live.preview_selection(chosen,config())["effective_speed_preview_s"]==.3


def test_available_exact_GO_intent_uses_new_preview():
    chosen=SimpleNamespace(**asdict(selected()),intent=0)
    assert live.preview_selection(chosen,config())["effective_speed_preview_s"]==.8


@pytest.mark.parametrize("change",["nan","inf","negative","bool","short","xy_short","dt","config"])
def test_all64_knots_and_fixed_abi_fail_closed(change):
    speeds=list(selected().speed_mps);xy=selected().xy_base_m;dt=.1;controller=config()
    if change in ("nan","inf","negative","bool"):
        speeds[63]={"nan":float("nan"),"inf":float("inf"),"negative":-.1,"bool":True}[change]
    elif change=="short":speeds.pop()
    elif change=="xy_short":xy=xy[:-1]
    elif change=="dt":dt=.2
    else:controller=replace(controller,speed_preview_s=.3)
    with pytest.raises(Exception):live.preview_selection(replace(selected(),speed_mps=tuple(speeds),xy_base_m=xy,timestep_s=dt),controller)


def test_tracker_once_without_mutating_selected_or_original_control():
    chosen=selected();before=asdict(chosen);scope=SimpleNamespace(_preview_tracker_receipt=None)
    with mock.patch.object(live,"_active",True),mock.patch.object(live,"_proposal_session",scope), \
         mock.patch.object(live,"_trajectory_control",wraps=live._trajectory_control) as delegate:
        result=live.trajectory_control(chosen,.5,config())
        assert delegate.call_count==1
        with pytest.raises(Exception):live.trajectory_control(chosen,.5,config())
        assert delegate.call_count==1
    assert asdict(chosen)==before and result["desired_speed_mps"]==chosen.speed_mps[7]
    # HH_261002 - The wrapper does not add its receipt into the unchanged control/PI contract.
    assert set(result)==set(live._trajectory_control(chosen,.5,config()))


def test_no_tracker_call_outside_owned_proposal():
    with pytest.raises(Exception):live.trajectory_control(selected(),.5,config())


@pytest.mark.parametrize("trial",[0,1,2])
def test_same_uint32_manual_start_profiles_and_capture_validator(trial):
    assert live.PROFILES==live.previous.PROFILES
    live.validate_capture(args(live.PROFILES[trial]["seed"]),dict(town="Town07",scenario="straight"))


def test_session_records_complete_config_and_delegates_once_without_second_PI(tmp_path):
    session=object.__new__(live.Preview08ObservedSession)
    session.states=[dict(frame=42)];session.output=tmp_path;session.policy=None
    session.handed_off=False
    returned=object(); decision=dict(control_source="expert_startup",learned_control_proposed=False)
    def proposal(*a):session.policy=SimpleNamespace(controller=config());return returned,decision
    with mock.patch.object(live,"_active",True),mock.patch.object(live._original_session,"propose",side_effect=proposal) as delegate:
        assert session.propose("t","teacher","next_driving_control")== (returned,decision)
    delegate.assert_called_once_with("t","teacher","next_driving_control")
    record=json.loads((tmp_path/"speed_preview_configuration.jsonl").read_text())
    assert record["actual_policy_controller"]==asdict(config()) and record["tracker_receipt"] is None
    assert record["journal_outside_policy100ms_inside_original_proposal_to_ACK"]
    assert live._proposal_session is None


def test_session_original_exception_kept_and_timing_scope_restored(tmp_path):
    session=object.__new__(live.Preview08ObservedSession)
    session.states=[dict(frame=42)];session.output=tmp_path;session.policy=SimpleNamespace(controller=config())
    with mock.patch.object(live,"_active",True),mock.patch.object(live._original_session,"propose",side_effect=OSError("original")), \
         pytest.raises(OSError,match="original"):
        session.propose("t","teacher","next_driving_control")
    assert live._proposal_session is None and not (tmp_path/"speed_preview_configuration.jsonl").exists()


@pytest.mark.parametrize("failure",[False,True])
def test_collector_restores_all_live_bindings_on_success_or_error(failure):
    original=(live.previous.SCHEMA,live.previous.Thread4ObservedSession,live.frozen.original.actor_controller_config,
              live.runtime.trajectory_control,live.collector.collect_episode)
    @contextmanager
    def inherited(m,a):
        assert live.previous.Thread4ObservedSession is live.Preview08ObservedSession
        assert live.runtime.trajectory_control is live.trajectory_control
        assert live.frozen.original.actor_controller_config is live.actor_controller_config
        if failure:raise OSError("original")
        yield
    model=SimpleNamespace(_research_speed_preview_calibration=live.PREVIEW_SCHEMA)
    with mock.patch.object(live,"verify_frozen"),mock.patch.object(live.previous.previous,"validate_model_binding"), \
         mock.patch.object(live.previous,"activated",side_effect=inherited):
        try:
            with live.activated(model,args()):
                with pytest.raises(Exception):
                    with live.activated(model,args()):pass
        except OSError:assert failure
    assert original==(live.previous.SCHEMA,live.previous.Thread4ObservedSession,live.frozen.original.actor_controller_config,
                      live.runtime.trajectory_control,live.collector.collect_episode)
    assert not live._active


def test_only_owned_new_preview_marker_authorizes_activation():
    with mock.patch.object(live,"verify_frozen"),mock.patch.object(live.previous.previous,"validate_model_binding"),pytest.raises(Exception):
        with live.activated(SimpleNamespace(),args()):pass


def journal_fixture():
    controller=asdict(config());used=selected()
    receipt=dict(live.preview_selection(used,config()),original_tracker_control=live._trajectory_control(used,.5,config()))
    wall=[dict(command_frame=10+i,command_reason="next_driving_control",ACK_sequence=i+1,
        proposal_started_monotonic_ns=10,proposal_finished_monotonic_ns=40,ACK_started_monotonic_ns=50,ACK_finished_monotonic_ns=60) for i in range(3)]
    records=[dict(schema=live.PREVIEW_SCHEMA,command_frame=r["command_frame"],command_reason=r["command_reason"],
        actual_policy_controller=controller,proposal_started_monotonic_ns=20,proposal_finished_monotonic_ns=30,
        tracker_receipt=receipt if i==1 else None) for i,r in enumerate(wall)]
    raw=[dict(command_reason=r["command_reason"],ack_receipt_sequence=i+1,learned_control_proposed=i>0,
        held_between_camera_frames=None if i==0 else i==2,
        selected=asdict(used) if i else None,controller=controller if i else None,
        control=dict(desired_speed_mps=used.speed_mps[7]) if i else None) for i,r in enumerate(wall)]
    for row,command in zip(records,raw):
        row.update(control_source=command.get("control_source"),learned_control_proposed=bool(command.get("learned_control_proposed")),
            held_between_camera_frames=bool(command.get("held_between_camera_frames")),inference_frame=command.get("inference_frame"),
            actual_returned_control=command.get("control"))
    return records,wall,raw


@pytest.mark.parametrize("change",[None,"missing","duplicate","preview","geometry","target","timing","held_tracker","missingfresh","rawACK"])
def test_complete_actual_config_raw_selected_ACK_denominator(change,tmp_path):
    records,wall,raw=journal_fixture()
    if change=="missing":records.pop()
    elif change=="duplicate":records[2]=records[1]
    elif change=="preview":records[1]["actual_policy_controller"]=dict(records[1]["actual_policy_controller"],speed_preview_s=.3)
    elif change=="geometry":records[1]["actual_policy_controller"]=dict(records[1]["actual_policy_controller"],wheelbase_m=2.85)
    elif change=="target":records[1]["tracker_receipt"]["proposed_selected_target_mps"]+=.01
    elif change=="timing":records[1]["proposal_finished_monotonic_ns"]=51
    elif change=="held_tracker":records[2]["tracker_receipt"]=records[1]["tracker_receipt"]
    elif change=="missingfresh":records[1]["tracker_receipt"]=None
    elif change=="rawACK":raw[2]["ack_receipt_sequence"]=2
    data={"speed_preview_configuration.jsonl":records,"wall_timing.jsonl":wall,"learned_control.jsonl":raw}
    with mock.patch.object(owner.parent,"jsonl",side_effect=lambda p:data[p.name]),mock.patch.object(owner,"digest",return_value="a"*64):
        if change is None:
            result=owner.configuration_report(tmp_path)
            assert result["passed"] and result["normal_proposal_denominator"]==3 and result["fresh_tracker_count"]==1
            assert result["actual_controller"]["wheelbase_m"]!=2.85
        else:
            with pytest.raises(Exception):owner.configuration_report(tmp_path)


def test_empty_tracker_denominator_never_vacuously_passes(tmp_path):
    records,wall,raw=journal_fixture();records[1]["tracker_receipt"]=None;raw[1]["learned_control_proposed"]=False
    data={"speed_preview_configuration.jsonl":records,"wall_timing.jsonl":wall,"learned_control.jsonl":raw}
    with mock.patch.object(owner.parent,"jsonl",side_effect=lambda p:data[p.name]),pytest.raises(Exception):owner.configuration_report(tmp_path)


@pytest.mark.parametrize("trial",[0,1,2])
def test_real_owner_scoped_trial_validator_worker_and_all_fixed_gates(tmp_path,trial):
    proof=tmp_path/"proof";proof.write_bytes(b"x")
    baseline=dict(source_sha256={"fixture":"a"*64},final_endpoint_sha256=pins(),
        prospective_preview_acceleration_guard_applied_to_all_original_baseline_arms=[])
    with mock.patch.object(owner,"BASELINE",proof),mock.patch.object(owner,"baseline_proof",return_value=baseline), \
         mock.patch.object(owner,"digest",return_value=owner.FROZEN_OWNER_SHA), \
         mock.patch.object(owner.previous,"evidence_assets",return_value={proof:"f"*64}), \
         mock.patch.object(owner.previous,"configure_threads"),mock.patch.object(owner.previous.torch,"get_num_threads",return_value=1), \
         mock.patch.object(owner.previous.torch,"set_num_threads"),mock.patch.object(owner.previous.previous,"verify_exports",return_value={}), \
         mock.patch.object(owner,"runtime_sources",return_value={}):
        with owner.activated(pins()),owner.parent.scoped_trial(trial):
            result=owner.base.fixed_contract("v1",True)
            capture=live.frozen.parse_args(result["capture_argv"]);capture.research_source_sha256="e"*64
            live.frozen.validate_capture(capture,dict(town="Town07",scenario="straight"))
            assert owner.base.load_npz is owner.load_model and owner.parent.live is live
    assert result["schema"]==owner.SCHEMA and result["seed"]==live.PROFILES[trial]["seed"]
    assert result["worker_command"][result["worker_command"].index("-m")+1]==owner.MODULE
    assert result["optimizer_updates"]==6000 and result["controller_target_cap_mps"]==2.4
    assert result["total_capture_seconds"]==45 and result["precursor_motion_screen"]["net_learned_progress_minimum_m"]==90.
    assert result["precursor_motion_screen"]["post_handoff_base_planar_speed_maximum_mps"]==2.5
    assert result["sensor_and_ACK_screen"]["native_state_count"]==900 and result["body_clearance_screen"]["all_snapshot_linear_sweeps"]==899
    assert result["causal_selected_speed_preview_calibration"]["configured_speed_preview_s"]==.8
    assert not result["learned_selected_decoded_path_actual_v0_PI_and_handoff_math_unchanged"]
    assert result["additional_three_trial_baseline_report_archive_reserve_bytes"]==3


@pytest.mark.parametrize("failure",[False,True])
def test_owner_bindings_and_baseline_assets_restore_on_success_or_error(failure):
    names=("batch_paths","MODULE","SCHEMA","live","fixed_contract","runtime_sources","load_model","diagnostic_report","batch_summary","disk_check")
    before={name:getattr(owner.previous,name) for name in names};assets=owner.base.ASSETS
    @contextmanager
    def inherited(p):
        assert owner.previous.live is live and owner.previous.disk_check is owner.disk_check
        if failure:raise OSError("original")
        yield
    with mock.patch.object(owner,"baseline_proof",return_value={}),mock.patch.object(owner,"digest",return_value=owner.FROZEN_OWNER_SHA), \
         mock.patch.object(owner.previous,"activated",side_effect=inherited):
        try:
            with owner.activated(pins()):
                with pytest.raises(Exception):
                    with owner.activated(pins()):pass
        except OSError:assert failure
    assert all(getattr(owner.previous,k) is v for k,v in before.items()) and owner.base.ASSETS is assets
    assert not owner._active and owner._baseline is None


def test_loader_changes_only_explicit_preview_marker_after_frozen_load():
    model=SimpleNamespace(_research_cpu_thread_numerical_revision=live.previous.THREAD_SCHEMA)
    with mock.patch.object(owner,"_load",return_value=model) as delegate:
        assert owner.load_model("w","m",weights_sha256="a"*64,metadata_sha256="b"*64) is model
    delegate.assert_called_once_with("w","m",weights_sha256="a"*64,metadata_sha256="b"*64)
    assert model._research_speed_preview_calibration==live.PREVIEW_SCHEMA


@pytest.mark.parametrize("free,passed",[(100,True),(99,False)])
def test_archive_reserve_includes_three_complete_baseline_reports(tmp_path,free,passed):
    report=tmp_path/"report";report.write_bytes(b"1234567890")
    with mock.patch.object(owner,"BASELINE",report),mock.patch.object(owner,"_disk",return_value=dict(free_bytes=free,required_bytes=70)):
        if passed:assert owner.disk_check()["required_bytes"]==100
        else:
            with pytest.raises(Exception):owner.disk_check()


def test_native_acceleration_kept_as_truthful_separate_diagnostic(tmp_path):
    rows=[dict(frame=i,capture_phase="driving",timestamp=i*.05,world_acceleration_carla=[0,4,0],world_velocity_carla=[i*.2,0,0]) for i in range(900)]
    with mock.patch.object(owner.parent,"jsonl",return_value=rows):result=owner.acceleration_report(tmp_path)
    assert result["actual_acceleration_above3_observation_count"]==900
    assert result["native_speed_rate_outside_minus6_plus3_count"]==899 and not result["passed"]
    assert result["no_original_safety_threshold_changed"]


@pytest.mark.parametrize("change",[None,"frame_gap","timestamp_gap","late_phase","boundary_acceleration","boundary_brake"])
def test_additive_rate_screen_includes_first_warmup_driving_boundary_and_all_tail(change,tmp_path):
    rows=[dict(frame=i,capture_phase="stationary_warmup" if i<24 else "driving",timestamp=i*.05,
        world_acceleration_carla=[0,0,0],world_velocity_carla=[.5,0,0]) for i in range(900)]
    if change=="frame_gap":rows[500]["frame"]+=1
    elif change=="timestamp_gap":rows[500]["timestamp"]+=.01
    elif change=="late_phase":rows[899]["capture_phase"]="unknown"
    elif change=="boundary_acceleration":rows[23]["world_velocity_carla"]=[0.,0,0]
    elif change=="boundary_brake":rows[23]["world_velocity_carla"]=[1.,0,0]
    with mock.patch.object(owner.parent,"jsonl",return_value=rows):
        if change in ("frame_gap","timestamp_gap","late_phase"):
            with pytest.raises(Exception):owner.acceleration_report(tmp_path)
        else:
            result=owner.acceleration_report(tmp_path)
            assert result["driving_speed_rate_interval_denominator"]==876
            assert result["warmup_to_first_driving_interval_included"]
            assert result["passed"] is (change is None)
            if change is not None:assert result["native_speed_rate_violations"][0]["native_frame"]==24


@pytest.mark.parametrize("trial",[0,1,2])
def test_actual_base_preflight_with_all_preview_thread_joint_aliases_before_collector_activation(tmp_path,trial):
    report=tmp_path/"proof";report.write_bytes(b"x")
    baseline=dict(source_sha256={},final_endpoint_sha256=pins(),
        prospective_preview_acceleration_guard_applied_to_all_original_baseline_arms=[])
    numeric=SimpleNamespace(_research_temporal_checkpoint_sha256="c"*64)
    api=SimpleNamespace(stdout=str(owner.base.EGG)+"/__init__.py\n"+
        str(owner.base.CARLA_ROOT/"PythonAPI/carla/agents/__init__.py")+"\n1.8.0\n")
    old_validator=live.frozen.validate_capture
    patches=(mock.patch.object(owner,"BASELINE",report),mock.patch.object(owner,"baseline_proof",return_value=baseline),
        mock.patch.object(owner,"digest",return_value=owner.FROZEN_OWNER_SHA),
        mock.patch.object(owner.previous,"evidence_assets",return_value={report:"f"*64}),
        mock.patch.object(owner.previous,"configure_threads"),mock.patch.object(owner.previous.torch,"get_num_threads",return_value=1),
        mock.patch.object(owner.previous.torch,"set_num_threads"),mock.patch.object(owner.previous.previous,"verify_exports",return_value={}),
        mock.patch.object(owner,"runtime_sources",return_value={}),mock.patch.object(owner.base,"verify_plan",return_value=dict(longitudinal_pi=True)),
        mock.patch.object(owner.base.lifecycle,"_assert_no_other_carla"),mock.patch.object(owner.base,"ports_free"),
        mock.patch.object(owner.base.subprocess,"run",return_value=api),mock.patch.object(owner.base,"environment",return_value={}),
        mock.patch.object(live.collector,"load_route",return_value=dict(town="Town07",scenario="straight")))
    with ExitStack() as stack:
        for patch in patches:stack.enter_context(patch)
        with owner.activated(pins()),owner.parent.scoped_trial(trial):
            assert owner.base.live is live and not live._active
            with mock.patch.object(owner.base,"paths",return_value=(tmp_path/"absent",None)), \
                 mock.patch.object(owner.base,"disk_check",return_value={}),mock.patch.object(owner.base,"load_npz",return_value=numeric):
                _,result=owner.base.preflight("v1","e"*64)
            assert result["status"]=="PREFLIGHT_PASS_NO_SIMULATOR" and not result["learned_control_executed"]
    assert live.frozen.validate_capture is old_validator and not owner._active


def baseline_fixture(tmp_path):
    sources={};batch=tmp_path/"old_batch"
    for index in range(3):
        episode=batch/("trial_%02d"%index)/"episode.partial";episode.mkdir(parents=True)
        states=episode/"states.jsonl"
        states.write_text("".join(json.dumps(dict(frame=i,timestamp=i*.05,capture_phase="driving",
            world_acceleration_carla=[0,0,0],world_velocity_carla=[.5,0,0]))+"\n" for i in range(900)))
        sources[str(states)]=owner.digest(states)
    report=dict(final_endpoint_sha256=pins(),prospective_trials=3,joint_motion_body_policy_wall_trial_pass_count=0,
        all_failed_tail_retained=True,all_three_complete_thread_readbacks_passed=True,
        all_actual_native_states=2700,all_actual_camera_bundles=1350,all_actual_ACKs=2712,actual_JPEG_assets=8100,
        trials=[dict(trial_index=i) for i in range(3)],cpu_thread_numerical_preset=owner.previous.numerical_preset(),
        source_sha256=sources,batch=str(batch))
    path=tmp_path/"baseline.json";path.write_text(json.dumps(report))
    return path,report


@pytest.mark.parametrize("change",[None,"endpoint","omit_case","changed_raw","path_escape","missing_sources"])
def test_prior0p3_all_three_failures_and_complete_source_output_pins_are_not_selected(change,tmp_path):
    path,report=baseline_fixture(tmp_path)
    if change=="endpoint":report["final_endpoint_sha256"]["weights_sha256"]="f"*64
    elif change=="omit_case":report["trials"].pop()
    elif change=="changed_raw":Path(next(iter(report["source_sha256"]))).write_text("changed\n")
    elif change=="path_escape":report["source_sha256"][str(tmp_path/"../outside")]= "a"*64
    elif change=="missing_sources":report["source_sha256"]={}
    path.write_text(json.dumps(report));sha=owner.digest(path)
    with mock.patch.object(owner,"ROOT",tmp_path),mock.patch.object(owner,"BASELINE",path),mock.patch.object(owner,"BASELINE_SHA",sha):
        if change is None:
            result=owner.baseline_proof(pins())
            assert len(result["prospective_preview_acceleration_guard_applied_to_all_original_baseline_arms"])==3
            assert all(r["passed"] for r in result["prospective_preview_acceleration_guard_applied_to_all_original_baseline_arms"])
        else:
            with pytest.raises(Exception):owner.baseline_proof(pins())


def test_additive_acceleration_failure_cannot_keep_old_basic_PASS(tmp_path):
    with mock.patch.object(owner,"_diagnostic",return_value=dict(learned_driving_screen_passed=True)), \
         mock.patch.object(owner,"configuration_report",return_value=dict(passed=True)), \
         mock.patch.object(owner,"acceleration_report",return_value=dict(passed=False)), \
         mock.patch.object(owner,"_baseline",dict(final_endpoint_sha256=pins())),mock.patch.object(owner,"baseline_proof"), \
         mock.patch.object(owner.parent,"verify_derived_source_closure"):
        result=owner.diagnostic_report(tmp_path,{}, {})
    assert not result["learned_driving_screen_passed"] and not result["native_acceleration_diagnostic"]["passed"]
