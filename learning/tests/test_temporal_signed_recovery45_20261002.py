"""HH_261002 - Keep signed recovery, causal ACK effects and every frozen 45-second guard explicit."""

from contextlib import contextmanager
from copy import deepcopy
import math
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from scripts.e2e import collect_temporal_signed_recovery45_20261002 as live
from scripts.e2e import research_temporal_signed_recovery45_town07_20261002 as owner


def valid_args(seed=20261002100):
    return SimpleNamespace(host="127.0.0.1",port=2440,allow_map_load=False,control_transport="acknowledged_batch",
        goal_stop_profile="disabled",physics_hz=20,capture_hz=10,stationary_warmup_sec=1.2,max_duration_sec=45,
        target_speed_kmh=9,weather="ClearNoon",seed=seed,spawn_z_offset_m=.5,wheelbase_m=2.85,
        research_speed_cap_mps=2.5,research_max_cte_m=1.,research_source_sha256="a"*64,
        research_longitudinal_pi=True,agent_initialization="after_bootstrap")


@pytest.mark.parametrize("seed",[20261002100,20261002101,20261002102])
def test_exact_three_profiles_and_same_frozen_runtime_guards(seed):
    live.validate_capture(valid_args(seed),dict(town="Town07",scenario="straight"))
    assert live.profile(seed)["seed"]==seed


@pytest.mark.parametrize("field,value",[("seed",0),("seed",True),("port",2000),("allow_map_load",True),
    ("physics_hz",10),("capture_hz",5),("max_duration_sec",44),("weather","WetNoon"),
    ("research_speed_cap_mps",2.6),("research_max_cte_m",1.1),("research_longitudinal_pi",False),
    ("control_transport","direct"),("goal_stop_profile","comfortable_v1"),("agent_initialization","before_bootstrap")])
def test_contract_variants_fail_before_sensor_or_actor_changes(field,value):
    args=valid_args();setattr(args,field,value)
    with pytest.raises(Exception):live.validate_capture(args,dict(town="Town07",scenario="straight"))


def route():
    return dict(route=[dict(x=0.,y=0.,distance_m=0.,vad_command=3),dict(x=300.,y=0.,distance_m=300.,vad_command=3)])


def effects(*,sign=1.,first_cte=.20,first_yaw=1.5,converged_at=1.,duration=15.,emergency=None):
    states=[];commands=[]
    for i in range(round(duration*20)+1):
        elapsed=i*.05
        cte=sign*first_cte if elapsed<converged_at else sign*.02
        yaw=sign*first_yaw if elapsed<converged_at else .1
        states.append(dict(frame=100+i,timestamp=10.+elapsed,x=2.*elapsed,y=cte,yaw=math.radians(yaw),
            route_progress_m=2.*elapsed,capture_phase="driving",
            control_transport=dict(status="PASS",expected_receipt_sequence=i+1)))
        commands.append(dict(ack_receipt_sequence=i+1,application=dict(learned_control_applied=emergency!=i)))
    return states,commands


@pytest.mark.parametrize("profile_index,sign",[(0,-1.),(1,1.)])
def test_actual_signed_recovery_requires_real_first_ACK_effect_then_two_seconds_native_convergence(profile_index,sign):
    states,commands=effects(sign=sign)
    result=owner.recovery_metrics(states,commands,route(),live.PROFILES[profile_index])
    assert result["recovery_screen_passed"] and result["noncentered_recovery_case"]
    assert result["first_learned_effect"]["native_frame"]==100
    assert result["convergence"]["first_converged_simulation_seconds"]==1.
    assert result["convergence"]["continuous_converged_seconds"]>=2.-1.e-6
    assert len(result["full_post_effect_metric_rows"])==len(states)


@pytest.mark.parametrize("kwargs",[dict(first_cte=.09),dict(first_yaw=.49),dict(sign=-1.),dict(converged_at=10.1),dict(duration=2.)])
def test_teacher_removed_error_wrong_sign_late_or_short_convergence_is_not_recovery(kwargs):
    states,commands=effects(**kwargs)
    assert owner.recovery_metrics(states,commands,route(),live.PROFILES[1])["recovery_screen_passed"] is False


def test_later_nonlearned_or_emergency_native_ticks_not_excised_from_interval():
    states,commands=effects(emergency=250)
    result=owner.recovery_metrics(states,commands,route(),live.PROFILES[1])
    assert not result["recovery_screen_passed"] and not result["complete_post_effect_learned_interval"]
    assert result["post_effect_native_count"]==len(states)


def test_no_handoff_is_not_vacuous_recovery_success():
    states,commands=effects()
    for row in commands:row["application"]["learned_control_applied"]=False
    result=owner.recovery_metrics(states,commands,route(),live.PROFILES[1])
    assert not result["recovery_screen_passed"] and result["first_learned_effect"] is None


def test_center_comparator_does_not_claim_noncentered_recovery():
    states,commands=effects(first_cte=.02,first_yaw=.1,converged_at=0.)
    result=owner.recovery_metrics(states,commands,route(),live.PROFILES[2])
    assert result["case_screen_passed"] and result["centered_setup_screen_passed"]
    assert result["recovery_screen_passed"] is False and result["noncentered_recovery_case"] is False
    assert result["status"]=="PASS_CENTER_SETUP_COMPARATOR"


def test_convergence_clock_and_yaw_wrap_are_measured_not_counter_or_absolute_angle():
    projector=live.collector.RouteProjector(dict(route=[dict(x=0.,y=0.,distance_m=0.,vad_command=3),
        dict(x=-300.,y=.001,distance_m=300.,vad_command=3)]))
    cte,yaw=owner.rear_route_error(dict(x=-10.,y=-.2,yaw=-math.pi+.001,route_progress_m=10.,timestamp=1.),projector)
    assert cte>.19 and abs(yaw)<.1


class FakeCarla:
    @staticmethod
    def VehicleControl(**kwargs):
        return SimpleNamespace(**(dict(throttle=0.,brake=0.,steer=0.,hand_brake=False,reverse=False,gear=0,manual_gear_shift=False)|kwargs))


@pytest.mark.parametrize("raise_after",[False,True])
def test_actual_spawn_once_and_all_setup_hooks_restore_even_when_capture_raises(tmp_path,raise_after):
    args=valid_args()
    agent=SimpleNamespace(BasicAgent=type("Agent",(),{}))
    manifest=dict(capture_contract={})
    center=dict(x=0.,y=0.,z=0.,yaw=0.,pitch=0.,roll=0.)
    def original_collect(*positional,**keywords):
        first=live.collector.apply_spawn_z_offset(center,.5)
        assert first["y"]>.19 and first["yaw"]==1.5
        with pytest.raises(Exception):live.collector.apply_spawn_z_offset(center,.5)
        if raise_after:raise OSError("retained actual capture error")
        return "complete"
    with mock.patch.object(live.collector,"collect_episode",side_effect=original_collect), \
         mock.patch.object(live.collector,"apply_spawn_z_offset",side_effect=lambda pose,z:dict(pose,z=pose["z"]+z)):
        names=("collect_episode","apply_spawn_z_offset","_blank_brake_control","checked_agent_initialization_observation")
        before={name:getattr(live.collector,name) for name in names};original_agent=agent.BasicAgent
        try:
            with live.startup_spawn_scope(args,agent_module=agent):
                assert live.collector.checked_agent_initialization_observation is live.manual.checked_first_gear_initialization_observation
                live.collector.collect_episode(None,None,None,tmp_path,None,None,manifest)
        except OSError:assert raise_after
        assert all(getattr(live.collector,name) is value for name,value in before.items())
        assert agent.BasicAgent is original_agent
    assert manifest["result"]["actual_spawn_perturbation_call_count"]==1
    assert manifest["capture_contract"]["actual_signed_spawn"]["pixels_or_observed_poses_post_transformed"] is False


def test_missing_actual_spawn_cannot_leave_a_successful_capture(tmp_path):
    agent=SimpleNamespace(BasicAgent=type("Agent",(),{}));manifest=dict(capture_contract={})
    with mock.patch.object(live.collector,"collect_episode",return_value="would look complete"), \
         live.startup_spawn_scope(valid_args(),agent_module=agent),pytest.raises(Exception,match="exactly once"):
        live.collector.collect_episode(None,None,None,tmp_path,None,None,manifest)


def test_new_paths_cannot_write_to_old_frozen_outputs():
    output,path=owner.batch_paths("v1")
    assert "2026-10-02/temporal_signed_recovery45_town07_v1" in str(output)
    assert output!=owner.previous.batch_paths("v1")[0]
    for value in ("../v1","v0","v1/x"):
        with pytest.raises(RuntimeError):owner.batch_paths(value)


def test_new_declared_seed_and_capture_parameters_bind_exact_profile():
    with mock.patch.object(owner.parent,"_trial",0),mock.patch.object(owner,"_capture",return_value=["--seed","0","--max-duration-sec","45"]):
        assert owner.capture_argv("v1",True)==["--seed","20261002100","--max-duration-sec","45","--agent-initialization","after_bootstrap"]


def test_fixed_declaration_seed_matches_actualCLI_and_recovery_metrics_before_any_launch():
    with mock.patch.object(owner.parent,"_trial",1), \
         mock.patch.object(owner,"_fixed",return_value={}), \
         mock.patch.object(owner,"_capture",return_value=["--seed","0"]):
        result=owner.fixed_contract("v1",True)
    assert result["seed"]==result["signed_recovery_profile"]["seed"]==20261002101
    assert result["capture_argv"]==["--seed","20261002101","--agent-initialization","after_bootstrap"]
    assert result["agent_initialization"]=="after_bootstrap"
    assert result["recovery_screen"]["required_noncentered_case"] is True
    assert result["full_MAIN9_completion_claimed"] is False


@pytest.mark.parametrize("failure",[False,True])
def test_private_owner_scope_preserves_restores_all_frozen_globals(failure):
    names=("batch_paths","fixed_contract","runtime_sources","live","MODULE","SCHEMA","diagnostic_report")
    before={name:getattr(owner.previous,name) for name in names}
    capture=owner.parent.capture_argv;summary=owner.parent.batch_summary;assets=owner.base.ASSETS
    @contextmanager
    def fake_binding():
        assert owner.previous.MODULE==owner.MODULE
        assert owner.previous.live is live
        yield
    with mock.patch.object(owner,"runtime_sources",return_value={}), \
         mock.patch.object(owner.previous,"bound_fixed_recovery_exports",side_effect=fake_binding):
        try:
            with owner.activated():
                assert owner.parent.capture_argv is owner.capture_argv
                assert owner.base.ASSETS[owner.GEOMETRY]==owner.GEOMETRY_SHA
                with pytest.raises(RuntimeError,match="nest"):
                    with owner.activated():pass
                if failure:raise OSError("preflight error")
        except OSError:assert failure
    assert all(getattr(owner.previous,name) is value for name,value in before.items())
    assert owner.parent.capture_argv is capture and owner.parent.batch_summary is summary and owner.base.ASSETS==assets and not owner._active


def test_offline_initial_geometry_binds_only_exact_existing_footprint_and_SHA():
    result=owner.initial_geometry_proof(live.PROFILES[0])
    assert result["exact_geometry_case"]=="train_00" and not result["actual_new_capture_pass_claimed"]
    with mock.patch.object(owner,"digest",return_value="0"*64),pytest.raises(RuntimeError,match="changed"):
        owner.initial_geometry_proof(live.PROFILES[0])


def test_actual_negative_and_positive_spawn_are_not_captured_pose_or_image_transforms():
    center=dict(x=10.,y=20.,z=1.,yaw=0.,pitch=0.,roll=0.)
    first=dict(center)
    for spec in live.PROFILES:
        changed,rear=live.task.perturb_actor_center(center,wheelbase_m=2.85,
            signed_lateral_offset_ros_m=spec["signed_lateral_offset_ros_m"],
            signed_yaw_offset_ros_deg=spec["signed_yaw_offset_ros_deg"])
        assert rear["y"]==20.-spec["signed_lateral_offset_ros_m"]
        assert changed["yaw"]==-spec["signed_yaw_offset_ros_deg"]
        assert center==first


@pytest.mark.parametrize("failure",[False,True])
def test_learned_activation_delegate_and_seed_validator_restore_without_fallback(failure):
    args=valid_args();args.route_file=Path("unused")
    saved_validate=live.previous.previous.validate_capture
    calls=[]
    @contextmanager
    def startup(used):
        assert used is args;calls.append("startup")
        yield
    @contextmanager
    def inherited(model,used):
        assert model=="unchanged actual V3" and used is args
        assert live.previous.previous.validate_capture is live.validate_capture
        calls.append("inherited_unchanged_V3")
        if failure:raise ValueError("invalid original model binding")
        yield
    with mock.patch.object(live,"verify_frozen"), \
         mock.patch.object(live.collector,"load_route",return_value=dict(town="Town07",scenario="straight")), \
         mock.patch.object(live,"startup_spawn_scope",side_effect=startup), \
         mock.patch.object(live.previous,"activated",side_effect=inherited):
        try:
            with live.activated("unchanged actual V3",args):
                assert live._active
                with pytest.raises(Exception,match="nest"):
                    with live.activated("unchanged actual V3",args):pass
        except ValueError:assert failure
    assert calls==["startup","inherited_unchanged_V3"]
    assert live.previous.previous.validate_capture is saved_validate and not live._active


def test_center_is_not_counted_in_two_signed_recovery_denominator(tmp_path):
    for index in range(3):(tmp_path/("trial_%02d"%index)).mkdir()
    reports={0:dict(bounded_signed_recovery=dict(recovery_screen_passed=True,case_screen_passed=True)),
        1:dict(bounded_signed_recovery=dict(recovery_screen_passed=False,case_screen_passed=False)),
        2:dict(bounded_signed_recovery=dict(recovery_screen_passed=False,centered_setup_screen_passed=True,case_screen_passed=True))}
    paths={str(tmp_path/("trial_%02d"%i)/"diagnostic_report.json"):report for i,report in reports.items()}
    with mock.patch.object(owner,"_summary",return_value={}),mock.patch.object(owner,"batch_paths",return_value=(tmp_path,None)), \
         mock.patch.object(Path,"is_file",return_value=True),mock.patch.object(owner,"read_json",side_effect=lambda path:paths[str(path)]), \
         mock.patch.object(owner,"digest",return_value="a"*64):
        result=owner.batch_summary("v1","pinned")
    assert result["signed_recovery_case_count"]==2 and result["signed_recovery_pass_count"]==1
    assert not result["all_two_signed_recovery_cases_passed"] and result["centered_setup_comparator_passed"]


def test_manual_normal_launch_does_not_rewrite_original_BasicAgent_emergency_return(tmp_path):
    clock=SimpleNamespace(frame=100,timestamp=SimpleNamespace(elapsed_seconds=5.))
    vehicle=SimpleNamespace(get_velocity=lambda:SimpleNamespace(x=0.,y=0.),
        get_world=lambda:SimpleNamespace(get_snapshot=lambda:clock))
    raw=FakeCarla.VehicleControl(throttle=.9,brake=.3,steer=.2)
    planner=SimpleNamespace(_vehicle=vehicle,_vehicle_controller=SimpleNamespace(max_brake=1.,max_throt=.75),
        run_step=lambda *args,**kwargs:deepcopy(raw))
    hazard=FakeCarla.VehicleControl(throttle=0.,brake=1.,steer=.37,manual_gear_shift=False,gear=2)
    class OriginalAgent:
        def __init__(self):self._vehicle=vehicle;self.target=None;self.normal=None
        def get_local_planner(self):return planner
        def set_target_speed(self,target):self.target=target
        def run_step(self,*args,**kwargs):self.normal=planner.run_step();return hazard
    session=dict(manifest=dict(capture_contract={}),output=tmp_path)
    teacher=live.teacher_class(OriginalAgent,session)()
    returned=teacher.run_step()
    assert returned is hazard and returned.brake==1. and returned.steer==.37 and returned.gear==2
    assert returned.manual_gear_shift is False
    assert teacher.normal.throttle==.11 and teacher.normal.brake==0. and teacher.normal.steer==0.
    assert teacher.normal.manual_gear_shift is True and teacher.normal.gear==1
    assert raw.throttle==.9 and raw.brake==.3 and raw.steer==.2
    rows=owner.parent.jsonl(tmp_path/"signed_startup_normal_teacher.jsonl")
    assert len(rows)==1 and rows[0]["normal_policy"]["raw_normal_planner_proposal"]["brake"]==.3
    assert rows[0]["applied_learned_control_claimed"] is False
