"""HH_260906 - CPU-only numerical-setting audit tests; no ports, images, models or simulation are executed."""
import copy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

import pytest

from scripts.e2e import summarize_carla_hold_substep_abba as audit
from test_summarize_carla_hold_brake_ramp_abba import readiness_fixture, fixture as control_fixture

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


def plan_fixture():
    old=json.loads((_hh_source_path(audit.ROOT, "artifacts/training/2026-09-14/hold_brake_ramp_abba_v7/plan.json")).read_bytes())
    plan={key:copy.deepcopy(value) for key,value in old.items() if key not in ("cases","protocol_configuration_by_arm")}
    plan.update(schema="carla.hold_substep_abba_plan.v1", physics_profile_by_arm=audit.PHYSICS.copy(),
        protocol_configuration=asdict(audit.profiles.HoldBrakeRampDevelopmentGoalStopConfig()),
        required_original_substeps=dict(substepping=True,max_substep_delta_time=.01,max_substeps=10),
        requested_max_substep_delta_time_by_arm=dict(reference=.01,fine=.005),fixed_max_substeps=10,
        explicit_settings_readback_required=True,analysis_sha256=dict(audit.ANALYSIS_PINS,**{audit.SELF_PATH:audit.scalar.sha(_hh_source_path(audit.ROOT, audit.SELF_PATH))}),
        orchestration_sha256={name:"a"*64 for name in audit.ORCHESTRATION_PATHS})
    plan["source_sha256"]["scripts/e2e/carla_physics_substeps.py"]="b"*64
    plan["cases"]=[]
    for case_id,arm in audit.CASES:
        command=copy.deepcopy(old["cases"][0]["command"])
        command[2]=audit.CAMPAIGN_PATH+"/"+case_id
        command[command.index("--physics-substep-profile")+1]=audit.PHYSICS[arm]
        plan["cases"].append(dict(id=case_id,arm=arm,command=command,maximum_attempts=1))
    return plan


def settings_fixture(arm):
    before=dict(synchronous_mode=False,no_rendering_mode=False,substepping=True,deterministic_ragdolls=True,
        spectator_as_ego=True,fixed_delta_seconds=None,max_substep_delta_time=.01,max_culling_distance=0.,
        tile_stream_distance=3000.,actor_active_distance=2000.,max_substeps=10)
    after=dict(before,synchronous_mode=True,fixed_delta_seconds=.05,max_substep_delta_time=.01 if arm=="reference" else .005)
    record=audit.substeps.new_record(audit.PHYSICS[arm]);record.update(status="PASS",before=before,requested=copy.deepcopy(after),after=after)
    fields=("synchronous_mode","fixed_delta_seconds","no_rendering_mode","substepping","max_substep_delta_time","max_substeps")
    return dict(capture_contract=dict(physics_substeps=record),runtime=dict(
        original_world_settings={key:before[key] for key in fields},capture_world_settings={key:after[key] for key in fields}))


@pytest.mark.parametrize("arm",["reference","fine"])
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
    lambda m:m["capture_contract"]["physics_substeps"]["requested"].update(max_substep_delta_time=.01),
    lambda m:m["capture_contract"]["physics_substeps"]["after"].update(actor_active_distance=1000.),
    lambda m:m["capture_contract"]["physics_substeps"]["after"].pop("deterministic_ragdolls"),
    lambda m:m["runtime"]["capture_world_settings"].update(max_substep_delta_time=.01),
    lambda m:m["runtime"]["original_world_settings"].update(substepping=False),
])
def test_settings_substitution_or_partial_success_is_rejected(mutate):
    value=settings_fixture("fine");mutate(value)
    with pytest.raises((ValueError,KeyError,TypeError)):audit.validate_substeps(value,"fine")


def test_fixed_plan_and_current_analysis_bytes():
    plan=plan_fixture();saved=copy.deepcopy(plan)
    audit.validate_plan(plan);ledger={};audit.check_analysis_pins(plan,ledger)
    assert len(ledger)==3 and plan==saved


@pytest.mark.parametrize("mutate",[
    lambda p:p.update(schema="carla.hold_brake_ramp_abba_plan.v1"),
    lambda p:p.update(automatic_retry=True),
    lambda p:p["cases"].reverse(),
    lambda p:p["cases"][0].update(maximum_attempts=2),
    lambda p:p["physics_profile_by_arm"].update(fine="inherited"),
    lambda p:p["protocol_configuration"].update(hold_brake_mode="ramp"),
    lambda p:p["protocol_configuration"].update(low_speed_steering_mode="native"),
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


def test_reused_immediate_protocol_has_original_dwell_emergency_and_full_controls():
    args=control_fixture("immediate",emergency_index=6)
    got=audit.prior.hold_and_steering_protocol(*args)
    assert got["status"]=="PASS" and got["normal_hold_commands"][0]["mathematical_requested_brake"]==1.
    assert got["requests"][-1]["final_next_control"]["brake"]==.5


def trial(case_id,arm,count=10,denominator=100,maximum=20.):
    return dict(case_id=case_id,arm=arm,status="COMPLETE_DIAGNOSTIC_NOT_ADMITTED",native_state_count=3,camera_anchor_count=2,
        measured_future=dict(full_64_point_anchors=dict(sample_count=denominator,metrics=dict(
            xy_curvature=dict(violating_anchor_count=count,violating_step_count=count,
                unique_episode_relative_100ms_violation_tick_count=2,maximum=dict(value=maximum)),
            xy_lateral_acceleration=dict(violating_anchor_count=0)))))


def test_four_way_counts_rates_worst_and_incomplete_are_separate():
    trials=[trial(*case,count=5 if i in (1,2) else 10,maximum=30. if i in (1,2) else 20.) for i,case in enumerate(audit.CASES)]
    got=audit.comparison_verdict(trials,{"state":dict(status="DIFFERENT")})
    assert got["status"]=="FAVORABLE_CONDITION_SPECIFIC_SIGNAL_NOT_ADMITTED"
    assert all(row["worst_observed_curvature_increased"] for row in got["observed_cross_arm_comparisons"])
    assert got["exact_repeats_and_cross_arm_observations"] is False
    trials[-1]["status"]="NOT_RUN"
    assert audit.comparison_verdict(trials,{})["status"]=="INCOMPLETE_OR_FAILED_NO_COMPARATIVE_VERDICT"


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
    campaign=tmp_path/"campaign";case=dict(id=audit.CASES[0][0],arm="reference",command=["bash","mock-wrapper"])
    directory=campaign/"execution"/case["id"];directory.mkdir(parents=True)
    stamp="2026-09-14T07:59:00+00:00";plan=dict(declared_at_utc=stamp,cases=[case],
        orchestration_sha256={"scripts/e2e/probe_carla_port_readiness.py":"a"*64})
    write_json(directory/"declaration.json",dict(schema="carla.v8_case_execution_declaration.v1",case_id=case["id"],arm="reference",
        command=case["command"],plan_sha256="b"*64,maximum_wrapper_attempts=1,automatic_retry=False,training_data_approved=False,
        declared_at_utc=stamp,prior_case_continuation=[]))
    result=dict(schema="carla.v8_case_execution_result.v1",case_id=case["id"],arm="reference",completed_at_utc="2026-09-14T08:01:00+00:00",
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
    write_json(directory/"wrapper_started.json",dict(schema="carla.v8_wrapper_invocation.v1",case_id=case["id"],
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
    root=tmp_path/"campaign"/"01_reference_A1";case=dict(id=root.name,arm="reference",command=["bash","mock"])
    plan=dict(source_sha256={},declared_at_utc="2026-09-14T07:59:00+00:00")
    write_json(root/"owner_plan.json",dict(source_sha256={},source_bytes_archived=True,bounds_source_bytes_archived=True,
        planned_at_utc="2026-09-14T08:00:04+00:00"))
    write_json(root/"owner_started.json",dict(started_at_utc="2026-09-14T08:00:06+00:00"))
    write_json(root/"owner_result.json",dict(source_bytes_unchanged_and_archived=True,source_checks={},completed_at_utc="2026-09-14T08:00:59+00:00"))
    write_json(root.parent/"execution"/case["id"]/"wrapper_started.json",dict(schema="carla.v8_wrapper_invocation.v1",
        case_id=case["id"],command=case["command"],started_at_utc="2026-09-14T08:00:05+00:00"))
    with pytest.raises(ValueError,match="follows owner launch"):audit.verify_owner(root,case,plan,{})


def test_prelaunch_not_run_checks_analysis_and_explicit_case_target(tmp_path,monkeypatch):
    plan=plan_fixture()
    for name in plan["analysis_sha256"]:
        destination=tmp_path/name;destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_bytes((_hh_source_path(audit.ROOT, name)).read_bytes())
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
