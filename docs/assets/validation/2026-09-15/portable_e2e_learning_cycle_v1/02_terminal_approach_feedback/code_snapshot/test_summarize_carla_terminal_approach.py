"""HH_260906 - CPU-only terminal-feedback audit tests; no ports, images, models or simulation are executed."""
import copy
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path

import pytest

from scripts.e2e import summarize_carla_terminal_approach as audit
from test_summarize_carla_hold_brake_ramp_abba import readiness_fixture, StoredControl





def control_fixture(arm, *, emergency_index=None, event_index=None, jitter=False):
    config = replace(audit.profiles.TerminalApproachDevelopmentGoalStopConfig(), terminal_approach_mode=arm)
    gov = audit.profiles.DevelopmentGoalStopGovernor(config, 8., 20.)
    states, native, receipts = [], [], []
    measurements = [("stationary_warmup", 0., 100.), ("stationary_warmup", 0., 100.),
        ("driving", .5, 99.), ("driving", 3., 20.), ("driving", .75, 5.),
        ("driving", math.nextafter(.75, math.inf), 4.), ("driving", .2, 4.),
        ("driving", .3, 3.), ("driving", 0., 2.), ("driving", .36, 1.5), ("driving", .3, .8)]
    measurements += [("driving", .1 if i == 0 else .101 if jitter and i == 2 else .02, .8) for i in range(44 if jitter else 41)]
    measurements += [("stationary_tail", 0., .8)]
    if emergency_index is not None:
        measurements = measurements[:emergency_index+1]
    if event_index is not None:
        measurements = measurements[:event_index+1]
    previous = audit.common.blank_control()
    for index, (phase, speed, remaining) in enumerate(measurements):
        stamp, frame, initial = 100.+index*.05, 1000+index, index == 1
        record = gov.update(remaining, remaining, speed, -remaining, timestamp=stamp,
            longitudinal_speed_mps=speed, driving=phase == "driving", count_hold=phase == "driving")
        if initial:
            record = gov.update(remaining, remaining, speed, -remaining, timestamp=stamp,
                longitudinal_speed_mps=speed, driving=True, count_hold=False)
            record["next_control_starts_driving"] = True
        if index==event_index:
            gov.fail(audit.PROFILE+"_observed_event_abort")
        stop = gov.failure_reason or ("comfortable_goal_measured_stop_and_dwell" if phase == "driving" and record["complete"] else None)
        driving_command = initial or (phase == "driving" and stop is None)
        proposed = gov.normal_control(StoredControl()) if driving_command else StoredControl(**audit.common.blank_control())
        if index == emergency_index:
            proposed.throttle, proposed.brake, proposed.hand_brake = 0., .5, False
            gov.emergency_override_count += 1
            gov.fail(audit.PROFILE+"_emergency_override")
        record["termination_reason"] = stop
        audit.profiles.annotate_development_control(record, gov, stop)
        record["low_speed_steering_event_monitor"] = dict(observed_event_counts=dict(collision=int(index==event_index), lane_invasion=0),
            source_timestamp=stamp, asynchronous_event_delivery=True, zero_latency_abort_proven=False)
        audit.profiles.apply_low_speed_steering(proposed, gov, record, driving_command=driving_command)
        final = dict(vars(proposed))
        if not gov.emergency_override_count and speed < .3 and (final["brake"] > .05 or final["hand_brake"]):
            final["steer"] = 0.
        reason = "driving_end_brake" if stop else "initial_drive_control" if initial else "next_"+phase+"_control"
        if stop:
            final = audit.common.blank_control()
        states.append(dict(frame=frame, timestamp=stamp, capture_phase=phase, vx=speed, vy=0.,
            world_velocity_carla=[speed, 0., 0.], goal_stop=record, current_control=previous,
            next_control=final, collision=[{}] if index==event_index else [], lane_invasion=[],
            control_transport=dict(status="PASS", next_command_receipt_sequence=index+1)))
        native.append(dict(frame=frame, timestamp=stamp, stopped_in_goal=speed <= .1 and remaining <= 1.,
            remaining_route_m=remaining,goal_error_m=remaining,terminal_overshoot_m=-remaining))
        receipts.append(dict(sequence=index+1, reason=reason, requested_control=copy.deepcopy(final)))
        previous = copy.deepcopy(final)
    manifest = dict(capture_contract=dict(goal_stop_profile=dict(effective_control=dict(unchanged_basic_agent_emergency_brake=.5))))
    return states, native, manifest, receipts, arm



def plan_fixture():
    old=json.loads((audit.ROOT/"artifacts/training/2026-09-14/hold_brake_ramp_abba_v7/plan.json").read_bytes())
    plan={key:copy.deepcopy(value) for key,value in old.items() if key not in ("cases","protocol_configuration_by_arm")}
    plan.update(schema="carla.terminal_approach_plan.v1", physics_profile_by_arm=audit.PHYSICS.copy(),
        protocol_configuration_by_arm={arm:asdict(replace(audit.profiles.TerminalApproachDevelopmentGoalStopConfig(), terminal_approach_mode=arm)) for _,arm in audit.CASES},
        required_original_substeps=dict(substepping=True,max_substep_delta_time=.01,max_substeps=10),
        requested_max_substep_delta_time_by_arm=dict(coast=.01,feedback=.01),maximum_attempts_per_revision=2,fixed_max_substeps=10,
        explicit_settings_readback_required=True,analysis_sha256=dict(audit.ANALYSIS_PINS,**{audit.SELF_PATH:audit.scalar.sha(audit.ROOT/audit.SELF_PATH)}),
        orchestration_sha256={name:"a"*64 for name in audit.ORCHESTRATION_PATHS})
    plan["source_sha256"]["scripts/e2e/carla_physics_substeps.py"]="b"*64
    plan["cases"]=[]
    for case_id,arm in audit.CASES:
        command=copy.deepcopy(old["cases"][0]["command"])
        command[2]=audit.CAMPAIGN_PATH+"/"+case_id
        command[command.index("--physics-substep-profile")+1]=audit.PHYSICS[arm]
        command[command.index("--goal-stop-profile")+1]=audit.PROFILE
        command.extend(["--terminal-approach-mode",arm])
        plan["cases"].append(dict(id=case_id,arm=arm,command=command,maximum_attempts=1))
    return plan


def settings_fixture(arm):
    before=dict(synchronous_mode=False,no_rendering_mode=False,substepping=True,deterministic_ragdolls=True,
        spectator_as_ego=True,fixed_delta_seconds=None,max_substep_delta_time=.01,max_culling_distance=0.,
        tile_stream_distance=3000.,actor_active_distance=2000.,max_substeps=10)
    after=dict(before,synchronous_mode=True,fixed_delta_seconds=.05,max_substep_delta_time=.01)
    record=audit.substeps.new_record(audit.PHYSICS[arm]);record.update(status="PASS",before=before,requested=copy.deepcopy(after),after=after)
    fields=("synchronous_mode","fixed_delta_seconds","no_rendering_mode","substepping","max_substep_delta_time","max_substeps")
    return dict(capture_contract=dict(physics_substeps=record),runtime=dict(
        original_world_settings={key:before[key] for key in fields},capture_world_settings={key:after[key] for key in fields}))


@pytest.mark.parametrize("arm",["coast","feedback"])
def test_eleven_settings_and_original_six_bound_without_mutation(arm):
    value=settings_fixture(arm);saved=copy.deepcopy(value)
    got=audit.validate_substeps(value,arm)
    assert got["status"]=="PASS" and got["measured_internal_substeps"] is False
    assert got["independent_restoration_proven"] is False and value==saved


@pytest.mark.parametrize("mutate",[
    lambda m:m["capture_contract"]["physics_substeps"].update(profile="inherited"),
    lambda m:m["capture_contract"]["physics_substeps"].update(status="FAIL"),
    lambda m:m["capture_contract"]["physics_substeps"].update(error="failed"),
    lambda m:m["capture_contract"]["physics_substeps"].update(training_data_approved=True),
    lambda m:m["capture_contract"]["physics_substeps"]["before"].update(max_substeps=20),
    lambda m:m["capture_contract"]["physics_substeps"]["before"].update(substepping=1),
    lambda m:m["capture_contract"]["physics_substeps"]["before"].update(tile_stream_distance=math.inf),
    lambda m:m["capture_contract"]["physics_substeps"]["requested"].update(max_substep_delta_time=.005),
    lambda m:m["capture_contract"]["physics_substeps"]["after"].update(actor_active_distance=1000.),
    lambda m:m["capture_contract"]["physics_substeps"]["after"].pop("deterministic_ragdolls"),
    lambda m:m["runtime"]["capture_world_settings"].update(max_substep_delta_time=.005),
    lambda m:m["runtime"]["original_world_settings"].update(substepping=False),
])
def test_settings_substitution_or_partial_success_is_rejected(mutate):
    value=settings_fixture("feedback");mutate(value)
    with pytest.raises((ValueError,KeyError,TypeError)):audit.validate_substeps(value,"feedback")


def test_fixed_plan_and_current_analysis_bytes():
    plan=plan_fixture();saved=copy.deepcopy(plan)
    audit.validate_plan(plan);ledger={};audit.check_analysis_pins(plan,ledger)
    assert len(ledger)==4 and plan==saved


@pytest.mark.parametrize("mutate",[
    lambda p:p.update(schema="carla.hold_brake_ramp_abba_plan.v1"),
    lambda p:p.update(automatic_retry=True),
    lambda p:p["cases"].reverse(),
    lambda p:p["cases"][0].update(maximum_attempts=2),
    lambda p:p["physics_profile_by_arm"].update(feedback="inherited"),
    lambda p:p["protocol_configuration_by_arm"]["feedback"].update(hold_brake_mode="ramp"),
    lambda p:p["protocol_configuration_by_arm"]["feedback"].update(low_speed_steering_mode="native"),
    lambda p:p["source_sha256"].pop("scripts/e2e/carla_physics_substeps.py"),
    lambda p:p["analysis_sha256"].pop(audit.SELF_PATH),
    lambda p:p["analysis_sha256"].update({next(iter(audit.ANALYSIS_PINS)):"c"*64}),
    lambda p:p["orchestration_sha256"].update({"../escape":"c"*64}),
    lambda p:p.update(fixed_max_substeps=20),
    lambda p:p.update(physics_hz=40),
    lambda p:p["cases"][1]["command"].extend(["--seed","1"]),
    lambda p:p["cases"][1]["command"].extend(["--new-flag","1"]),
    lambda p:p["cases"][1]["command"].__setitem__(p["cases"][1]["command"].index("--weather")+1,"WetNoon"),
])
def test_plan_cannot_change_a_second_factor_or_weaken_pins(mutate):
    plan=plan_fixture();mutate(plan)
    with pytest.raises((ValueError,KeyError)):audit.validate_plan(plan)


def test_changed_current_analysis_bytes_rejected():
    plan=plan_fixture();plan["analysis_sha256"][audit.SELF_PATH]="0"*64
    with pytest.raises(ValueError,match="SHA"):audit.check_analysis_pins(plan,{})


def residual_fixture():
    rows=[]
    for i in range(3):
        rows.append(dict(frame=10+i,timestamp=1.+i*.05,x=i*.05,y=0.,yaw=0.,vx=1.,vy=0.,yaw_rate=0.,
            actor_snapshot_transform_carla=dict(x=1.425+i*.05,y=0.),world_velocity_carla=[1.,0.,0.],
            capture_phase="driving" if i<2 else "stationary_tail",goal_stop=dict(pilot_state="service_brake_hold"),
            next_control=audit.common.blank_control(),current_control=audit.common.blank_control()))
    return rows


def test_signed_residuals_include_midpoint_and_driving_tail_boundary():
    rows=residual_fixture();saved=copy.deepcopy(rows);got=audit.native_residuals(rows)
    assert len(got)==2 and rows==saved
    assert max(r["actor_residual_norm_m"] for r in got)<1e-14
    assert got[-1]["source_phase"]=="driving" and got[-1]["destination_phase"]=="stationary_tail"
    rows[1]["world_velocity_carla"][0]=-1.
    got=audit.native_residuals(rows)
    assert sum(r["actor_signed_velocity_integral_ros_xy_m"][0] for r in got)==0.
    assert sum(r["actor_delta_ros_xy_m"][0] for r in got)==pytest.approx(.1)
    assert all(r["physical_substep_truth_proven"] is False for r in got)


def test_y_handedness_yaw_wrap_and_reference_shift():
    rows=residual_fixture()[:2]
    for i,r in enumerate(rows):
        r.update(x=0.,y=i*.05,yaw=math.pi/2,vx=1.,vy=0.)
        r["actor_snapshot_transform_carla"].update(x=0.,y=-i*.05)
        r["world_velocity_carla"]=[0.,-1.,0.]
    got=audit.native_residuals(rows)[0]
    assert got["actor_delta_ros_xy_m"][1]==.05
    assert got["actor_signed_velocity_integral_ros_xy_m"][1]==pytest.approx(.05)
    assert got["virtual_base_residual_norm_m"]<1e-14
    rows[0]["yaw"],rows[1]["yaw"]=math.pi-.001,-math.pi+.001
    assert audit.native_residuals(rows)[0]["measured_yaw_change_rad"]==pytest.approx(.002)


@pytest.mark.parametrize("field,value",[("frame",15),("timestamp",1.1),("vx",math.nan),("yaw_rate",math.inf)])
def test_residual_gap_nonfinite_never_bridged(field,value):
    rows=residual_fixture();rows[1][field]=value
    with pytest.raises(ValueError):audit.native_residuals(rows)


def trial(case_id,arm,count=10,denominator=100,maximum=20.):
    return dict(case_id=case_id,arm=arm,status="COMPLETE_DIAGNOSTIC_NOT_ADMITTED",native_state_count=3,camera_anchor_count=2,
        measured_future=dict(full_64_point_anchors=dict(sample_count=denominator,metrics=dict(
            xy_curvature=dict(violating_anchor_count=count,violating_step_count=count,
                unique_episode_relative_100ms_violation_tick_count=2,maximum=dict(value=maximum)),
            xy_lateral_acceleration=dict(violating_anchor_count=0,violating_step_count=0,maximum=dict(value=0.))))))


def test_two_case_completion_not_smaller_curvature_is_primary():
    trials=[trial(*audit.CASES[0]),trial(*audit.CASES[1],count=5,denominator=120,maximum=30.)]
    got=audit.comparison_verdict(trials,{})
    assert got["status"]=="BOTH_COMPLETED_CONDITION_SPECIFIC_NOT_ADMITTED"
    assert got["repeatability_assessed"] is False and got["training_data_approved"] is False
    assert got["observed_feedback_minus_coast_metric_deltas"]["xy_curvature"]["maximum"]==10.
    trials[1]["status"]="FAILED_CAPTURE_OR_QA_NOT_ADMITTED"
    assert audit.comparison_verdict(trials,{})["status"]=="INCOMPLETE_OR_FAILED_NO_COMPARATIVE_VERDICT"
    with pytest.raises(ValueError):audit.comparison_verdict(trials[:1],{})


def test_feedback_exact_continuous_reference_slew_and_immediate_reduction():
    first=audit.feedback_law(4.,.1,False,0.,.05)
    assert first==dict(target_speed_mps=.35,positive_speed_error_mps=.24999999999999997,
        raw_throttle=.15,mathematical_throttle=.005000000000000001)
    assert audit.feedback_law(4.,.35,False,.15,.05)["mathematical_throttle"]==0.
    assert audit.feedback_law(4.,.8,False,.15,.05)["mathematical_throttle"]==0.
    assert audit.feedback_law(.8,.2,True,.15,.05)["target_speed_mps"]==0.
    assert audit.feedback_law(.8,.2,True,.15,.05)["mathematical_throttle"]==0.
    jitter=audit.feedback_law(4.,0.,False,.01,.0500000007)
    assert jitter["mathematical_throttle"]==.01+.1*.0500000007
    assert audit.feedback_law(1.01,0.,False,0.,.05)["target_speed_mps"]==.12


@pytest.mark.parametrize("args",[(4.,math.nan,False,0.,.05),(4.,0.,1,0.,.05),
    (4.,-.1,False,0.,.05),(4.,0.,False,.2,.05),(4.,0.,False,0.,0.),(True,0.,False,0.,.05)])
def test_feedback_nonfinite_bool_negative_inputs_rejected(args):
    with pytest.raises(ValueError):audit.feedback_law(*args)


@pytest.mark.parametrize("arm",["coast","feedback"])
def test_complete_control_chain_replays_latch_slew_goal_zero_hold_and_tail(arm):
    args=control_fixture(arm);saved=copy.deepcopy(args)
    got=audit.terminal_control_protocol(*args)
    assert got["status"]=="PASS",got["failures"]
    assert args==saved and got["state_receipt_count"]==len(args[0])
    assert got["normal_hold_commands"][0]["mathematical_requested_brake"]==1.
    if arm=="feedback":
        rows=args[0]
        assert rows[4]["goal_stop"]["terminal_approach"]["feedback_latched"] is True
        assert rows[4]["next_control"]["throttle"]==0.
        assert rows[6]["next_control"]["throttle"]>0.
        assert rows[9]["next_control"]["throttle"]==0.
        assert rows[10]["next_control"]["throttle"]==0. and rows[10]["next_control"]["brake"]==0.
        assert rows[11]["next_control"]["brake"]==1.


@pytest.mark.parametrize("arm",["coast","feedback"])
@pytest.mark.parametrize("emergency_index",[4,6,11,13])
def test_emergency_keeps_predecision_and_normal_receipt_but_invalidates_live_feedback(arm,emergency_index):
    args=control_fixture(arm,emergency_index=emergency_index)
    got=audit.terminal_control_protocol(*args)
    assert got["status"]=="PASS",got["failures"]
    last=args[0][-1]["goal_stop"]
    assert last["terminal_approach"]["eligible"] is False
    assert last["terminal_approach_control"]["normal_control_executed"] is True
    assert last["terminal_approach_control"]["superseded_by_emergency"] is True
    assert args[0][-1]["next_control"]["brake"]==.5


@pytest.mark.parametrize("change,check",[
    (lambda a:a[0][6]["goal_stop"]["terminal_approach"].update(source_timestamp=0.),"independent_terminal_decision_latch_law_and_request_memory"),
    (lambda a:a[0][6]["goal_stop"]["terminal_approach"].update(mathematical_throttle=.15),"independent_terminal_decision_latch_law_and_request_memory"),
    (lambda a:a[0][6]["goal_stop"]["terminal_approach"].update(previous_requested_throttle=.1),"independent_terminal_decision_latch_law_and_request_memory"),
    (lambda a:a[0][6]["goal_stop"]["terminal_approach"].update(rate_interval_seconds=.1),"independent_terminal_decision_latch_law_and_request_memory"),
    (lambda a:a[0][6]["goal_stop"]["terminal_approach"].update(feedback_source_timestamp=1.),"independent_terminal_decision_latch_law_and_request_memory"),
    (lambda a:a[0][6]["goal_stop"]["terminal_approach_control"]["decision_at_normal_control"].update(raw_throttle=.12),"terminal_pre_emergency_decision_snapshot"),
    (lambda a:a[0][6]["goal_stop"]["terminal_approach_control"]["returned_control"].update(throttle=.15),"terminal_normal_controls_match_original_hold_receipt"),
    (lambda a:a[0][6]["goal_stop"]["terminal_approach_control"].update(physical_application_proven=True),"terminal_normal_source_scope"),
    (lambda a:a[0][6]["goal_stop"]["hold_brake_control"]["returned_control"].update(steer=0.),"original_normal_pedals_and_only_pedals"),
    (lambda a:a[3][6]["requested_control"].update(throttle=.15),"guarded_final_command_and_ACK"),
    (lambda a:a[0][11]["goal_stop"].update(hold_ticks=1),"unchanged_measured_dwell"),
    (lambda a:a[0][-1]["goal_stop"]["terminal_approach_control"].update(decision_at_normal_control={}),"no_stale_terminal_normal_receipt"),
])
def test_feedback_control_corruption_retains_every_witness_and_fails(change,check):
    args=control_fixture("feedback");change(args)
    got=audit.terminal_control_protocol(*args)
    assert got["status"]=="FAIL" and any(f["check"]==check for f in got["failures"])
    assert got["state_receipt_count"]==len(args[0])


@pytest.mark.parametrize("event_index",[4,6,11])
def test_event_before_normal_call_retains_latch_but_has_no_stale_command(event_index):
    args=control_fixture("feedback",event_index=event_index)
    got=audit.terminal_control_protocol(*args)
    assert got["status"]=="PASS",got["failures"]
    last=args[0][-1]["goal_stop"]
    assert last["terminal_approach"]["eligible"] is False
    assert last["terminal_approach_control"]["normal_control_executed"] is False
    assert last["terminal_approach_control"]["decision_at_normal_control"] is None


def test_prefix_keeps_zero_throttle_latched_period_until_first_retained_positive_request():
    first,second=control_fixture("coast")[0],control_fixture("feedback")[0]
    for rows in (first,second):
        for row in rows:
            for key in audit.common.PHYSICAL_FIELDS:
                row.setdefault(key,0.)
    got=audit.before_feedback_comparison(first,second)
    assert got["status"]=="EXACT_MATCH" and got["compared_native_count"]==7
    assert second[4]["goal_stop"]["terminal_approach"]["feedback_latched"] is True
    second[5]["next_control"]["throttle"] = .01
    assert audit.before_feedback_comparison(first,second)["status"]=="DIFFERENT"
    aborted=control_fixture("feedback",emergency_index=6)[0]
    assert audit.before_feedback_comparison(first,aborted)["status"]=="UNAVAILABLE"


def test_consistent_recorded_distance_and_nested_receipts_cannot_replace_independent_projection():
    args=control_fixture("feedback")
    gp=args[0][6]["goal_stop"]
    gp["remaining_route_arc_m"]+=.1
    gp["terminal_approach"]["source_remaining_arc_m"]+=.1
    gp["terminal_approach_control"]["decision_at_normal_control"]["source_remaining_arc_m"]+=.1
    got=audit.terminal_control_protocol(*args)
    assert got["status"]=="FAIL"
    assert got["failures"]==[{"frame":1006,"check":"independent_pose_route_geometry_binding"}]


def test_failed_or_absent_case_stops_remaining_execution():
    for failure in ("NOT_RUN","READINESS_BLOCKED_NOT_ATTEMPTED","WRAPPER_FAILED_WITH_OUTPUT"):
        records=[dict(status="WRAPPER_COMPLETED"),dict(status=failure),dict(status="NOT_RUN"),dict(status="NOT_RUN")]
        for i,record in enumerate(records):
            record.update(declared_at_utc=f"2026-09-15T00:00:0{i*2}+00:00",completed_at_utc=f"2026-09-15T00:00:0{i*2+1}+00:00")
        audit.enforce_attempt_order(records)
        records[-1]["status"]="WRAPPER_COMPLETED"
        with pytest.raises(ValueError):audit.enforce_attempt_order(records)


def test_next_declaration_and_actual_continuation_flags_are_bound():
    rows=[dict(status="WRAPPER_COMPLETED",declared_at_utc="2026-09-15T00:00:00+00:00",completed_at_utc="2026-09-15T00:01:00+00:00"),
          dict(status="WRAPPER_COMPLETED",declared_at_utc="2026-09-15T00:00:59+00:00",completed_at_utc="2026-09-15T00:02:00+00:00")]
    with pytest.raises(ValueError,match="before previous"):audit.enforce_attempt_order(rows)
    expected=dict(case_id="A1",status="COMPLETE_DIAGNOSTIC_NOT_ADMITTED",execution_and_scalar_pixel_protocol_clear=True,all_original_future_metrics_clear=False)
    records=[dict(status="WRAPPER_COMPLETED",prior_case_continuation=[]),dict(status="WRAPPER_COMPLETED",prior_case_continuation=[copy.deepcopy(expected)])]
    audit.verify_continuation_results(records,[expected])
    records[1]["prior_case_continuation"][0]["all_original_future_metrics_clear"]=True
    with pytest.raises(ValueError,match="actual independent"):audit.verify_continuation_results(records,[expected])


def write_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(audit.encoded(value)+b"\n")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def execution_fixture(tmp_path,monkeypatch):
    monkeypatch.setattr(audit,"ROOT",tmp_path)
    campaign=tmp_path/"campaign";case=dict(id=audit.CASES[0][0],arm="coast",command=["bash","mock-wrapper"])
    directory=campaign/"execution"/case["id"];directory.mkdir(parents=True)
    stamp="2026-09-14T07:59:00+00:00";plan=dict(declared_at_utc=stamp,cases=[case],
        orchestration_sha256={"scripts/e2e/probe_carla_port_readiness.py":"a"*64})
    write_json(directory/"declaration.json",dict(schema="carla.v9_case_execution_declaration.v1",case_id=case["id"],arm="coast",
        command=case["command"],plan_sha256="b"*64,maximum_wrapper_attempts=1,automatic_retry=False,training_data_approved=False,
        declared_at_utc=stamp,prior_case_continuation=[]))
    result=dict(schema="carla.v9_case_execution_result.v1",case_id=case["id"],arm="coast",completed_at_utc="2026-09-14T08:01:00+00:00",
        wrapper_attempt_count=0,status="PIN_MISMATCH_NOT_ATTEMPTED",error="changed source",automatic_retry=False,training_data_approved=False)
    write_json(directory/"result.json",result)
    return campaign,case,plan,directory,result


def test_exceptional_pin_failure_preserves_zero_attempt_no_cleanup_claim(tmp_path,monkeypatch):
    campaign,case,plan,directory,result=execution_fixture(tmp_path,monkeypatch)
    got=audit.execution_record(campaign,case,plan,0,{},"b"*64)
    assert got["wrapper_attempt_count"]==0 and got["status"]=="PIN_MISMATCH_NOT_ATTEMPTED"
    assert got["readiness"]["ready_observed"] is None
    with pytest.raises(ValueError):audit.execution_record(campaign,case,plan,0,{},"c"*64)
    (campaign/case["id"]).mkdir()
    with pytest.raises(ValueError):audit.execution_record(campaign,case,plan,0,{},"b"*64)


def test_successful_durable_wrapper_binds_ready_journal_log_and_owner(tmp_path,monkeypatch):
    campaign,case,plan,directory,result=execution_fixture(tmp_path,monkeypatch)
    ready,journal=readiness_fixture()
    path=directory/"readiness.json.samples.jsonl"
    path.write_bytes(b"".join(audit.encoded(row)+b"\n" for row in journal))
    ready["sample_journal_sha256"]=hashlib.sha256(path.read_bytes()).hexdigest()
    readysha=write_json(directory/"readiness.json",ready)
    write_json(directory/"wrapper_started.json",dict(schema="carla.v9_wrapper_invocation.v1",case_id=case["id"],
        command=case["command"],started_at_utc="2026-09-14T08:00:05+00:00",readiness_sha256=readysha,
        wrapper_attempt_count=1,automatic_retry=False))
    log=directory/"wrapper.log";log.write_bytes(b"owned mock finished")
    ownersha=write_json(campaign/case["id"]/"owner_result.json",dict(exit_code=0,completed_at_utc="2026-09-14T08:00:59+00:00"))
    result.update(status="WRAPPER_COMPLETED",wrapper_attempt_count=1,exit_code=0,readiness_sha256=readysha,
        wrapper_log_sha256=hashlib.sha256(log.read_bytes()).hexdigest(),owner_result_sha256=ownersha)
    result.pop("error")
    write_json(directory/"result.json",result)
    got=audit.execution_record(campaign,case,plan,0,{},"b"*64)
    assert got["status"]=="WRAPPER_COMPLETED" and got["readiness"]["sample_count"]==3
    assert got["prior_case_continuation"]==[] and got["declared_at_utc"]==plan["declared_at_utc"]
    log.write_bytes(b"swapped")
    with pytest.raises(ValueError,match="SHA"):audit.execution_record(campaign,case,plan,0,{},"b"*64)


def test_late_wrapper_intent_cannot_authenticate_earlier_owner_launch(tmp_path,monkeypatch):
    monkeypatch.setattr(audit,"ROOT",tmp_path)
    root=tmp_path/"campaign"/"01_coast_A";case=dict(id=root.name,arm="coast",command=["bash","mock"])
    plan=dict(source_sha256={},declared_at_utc="2026-09-14T07:59:00+00:00")
    write_json(root/"owner_plan.json",dict(source_sha256={},source_bytes_archived=True,bounds_source_bytes_archived=True,
        planned_at_utc="2026-09-14T08:00:04+00:00"))
    write_json(root/"owner_started.json",dict(started_at_utc="2026-09-14T08:00:06+00:00"))
    write_json(root/"owner_result.json",dict(source_bytes_unchanged_and_archived=True,source_checks={},completed_at_utc="2026-09-14T08:00:59+00:00"))
    write_json(root.parent/"execution"/case["id"]/"wrapper_started.json",dict(schema="carla.v9_wrapper_invocation.v1",
        case_id=case["id"],command=case["command"],started_at_utc="2026-09-14T08:00:05+00:00"))
    with pytest.raises(ValueError,match="follows owner launch"):audit.verify_owner(root,case,plan,{})


def test_prelaunch_not_run_checks_analysis_and_explicit_case_target(tmp_path,monkeypatch):
    plan=plan_fixture()
    for name in plan["analysis_sha256"]:
        destination=tmp_path/name;destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_bytes((audit.ROOT/name).read_bytes())
    monkeypatch.setattr(audit,"ROOT",tmp_path)
    case=plan["cases"][0]
    got,images,futures,states=audit.summarize_one(tmp_path/case["command"][2],case,plan)
    assert got["status"]=="NOT_RUN" and not images and not futures and not states
    with pytest.raises(ValueError,match="planned target"):audit.summarize_one(tmp_path/"different",case,plan)
    (tmp_path/audit.SELF_PATH).write_bytes(b"changed")
    with pytest.raises(ValueError,match="SHA"):audit.summarize_one(tmp_path/case["command"][2],case,plan)


def test_source_paths_and_nonfinite_json_are_fail_closed(tmp_path):
    p=tmp_path/"bad.json";p.write_text('{"x":1e999}')
    with pytest.raises(ValueError):audit.checked(p,{})
    p.write_text('{"x":0,"x":1}')
    with pytest.raises(ValueError):audit.checked(p,{})
    with pytest.raises(ValueError):audit.safe(tmp_path/"x"/".."/"bad.json")
    link=tmp_path/"link";link.symlink_to(p)
    with pytest.raises(ValueError):audit.checked(link,{})
