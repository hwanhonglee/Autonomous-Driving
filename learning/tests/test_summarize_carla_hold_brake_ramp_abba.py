"""HH_260906 - Test the independent v7 source-clock/control witness without CARLA, models, or dataset access."""
import copy
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path

import pytest

from scripts.e2e import summarize_carla_hold_brake_ramp_abba as audit


class StoredControl:
    def __init__(self, **values):
        for key, value in (values or dict(throttle=.4, steer=.2, brake=.1, hand_brake=False,
                reverse=False, gear=0, manual_gear_shift=False)).items():
            setattr(self, key, value)

    def __setattr__(self, key, value):
        object.__setattr__(self, key, audit.float32(value) if key in ("throttle", "brake", "steer") else value)


def fixture(arm, *, emergency_index=None, jitter=False):
    config = replace(audit.profiles.HoldBrakeRampDevelopmentGoalStopConfig(), hold_brake_mode=arm)
    gov = audit.profiles.DevelopmentGoalStopGovernor(config, 8., 20.)
    states, native, receipts = [], [], []
    measurements = [("stationary_warmup", 0., 100.), ("stationary_warmup", 0., 100.),
        ("driving", .5, 99.), ("driving", 3., 20.), ("driving", .75, 5.),
        ("driving", math.nextafter(.75, math.inf), 4.)]
    measurements += [("driving", .1 if i == 0 else .101 if jitter and i == 2 else .02, .8) for i in range(44 if jitter else 41)]
    measurements += [("stationary_tail", 0., .8)]
    if emergency_index is not None:
        measurements = measurements[:emergency_index+1]
    previous = audit.common.blank_control()
    for index, (phase, speed, remaining) in enumerate(measurements):
        stamp, frame, initial = 100.+index*.05, 1000+index, index == 1
        record = gov.update(remaining, remaining, speed, -remaining, timestamp=stamp,
            longitudinal_speed_mps=speed, driving=phase == "driving", count_hold=phase == "driving")
        if initial:
            record = gov.update(remaining, remaining, speed, -remaining, timestamp=stamp,
                longitudinal_speed_mps=speed, driving=True, count_hold=False)
            record["next_control_starts_driving"] = True
        stop = "comfortable_goal_measured_stop_and_dwell" if phase == "driving" and record["complete"] else None
        driving_command = initial or (phase == "driving" and stop is None)
        proposed = gov.normal_control(StoredControl()) if driving_command else StoredControl(**audit.common.blank_control())
        if index == emergency_index:
            proposed.throttle, proposed.brake, proposed.hand_brake = 0., .5, False
            gov.emergency_override_count += 1
            gov.fail(audit.PROFILE+"_emergency_override")
        record["termination_reason"] = stop
        audit.profiles.annotate_development_control(record, gov, stop)
        record["low_speed_steering_event_monitor"] = dict(observed_event_counts=dict(collision=0, lane_invasion=0),
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
            next_control=final, collision=[], lane_invasion=[],
            control_transport=dict(status="PASS", next_command_receipt_sequence=index+1)))
        native.append(dict(frame=frame, timestamp=stamp, stopped_in_goal=speed <= .1 and remaining <= 1.))
        receipts.append(dict(sequence=index+1, reason=reason, requested_control=copy.deepcopy(final)))
        previous = copy.deepcopy(final)
    manifest = dict(capture_contract=dict(goal_stop_profile=dict(effective_control=dict(unchanged_basic_agent_emergency_brake=.5))))
    return states, native, manifest, receipts, arm


@pytest.mark.parametrize("arm", ["immediate", "ramp"])
def test_all_control_stages_hold_dwell_and_no_mutation(arm):
    args = fixture(arm)
    before = copy.deepcopy(args)
    got = audit.hold_and_steering_protocol(*args)
    assert got["status"] == "PASS", got["failures"]
    assert args == before
    assert len(got["requests"]) == got["state_receipt_count"] == len(args[0])
    assert got["hold_source_timestamp"] == args[0][6]["timestamp"]
    assert got["normal_hold_commands"][0]["mathematical_requested_brake"] == (.1 if arm == "ramp" else 1.)
    assert got["normal_hold_commands"][0]["stored_normal_brake"] == audit.float32(.1 if arm == "ramp" else 1.)
    assert got["requests"][-1]["normal_request"]["normal_control_executed"] is False
    assert got["requests"][-2]["final_replacement_reason"] == "driving_end_brake"


def test_actual_source_clock_plus_dt_without_tick_rounding():
    assert audit.requested_hold_brake("ramp", 10., 10.) == .1
    assert audit.requested_hold_brake("ramp", 10.4499999, 10.) < 1.
    assert audit.requested_hold_brake("ramp", 10.4500001, 10.) == 1.
    assert audit.requested_hold_brake("immediate", 10., 10.) == 1.
    assert audit.float32(.1) != .1
    for source, hold in [(9., 10.), (True, 0.), (math.inf, 0.)]:
        with pytest.raises(ValueError): audit.requested_hold_brake("ramp", source, hold)
    with pytest.raises(ValueError): audit.float32(1e100)


def test_speed_jitter_resets_measured_dwell_not_latched_ramp_clock():
    args = fixture("ramp", jitter=True)
    got = audit.hold_and_steering_protocol(*args)
    assert got["status"] == "PASS", got["failures"]
    assert got["hold_source_timestamp"] == args[0][6]["timestamp"]
    assert args[0][8]["goal_stop"]["hold_ticks"] == 0
    assert got["normal_hold_commands"][2]["mathematical_requested_brake"] > .29


@pytest.mark.parametrize("arm", ["immediate", "ramp"])
@pytest.mark.parametrize("emergency_index", [6, 8])
def test_original_emergency_supersedes_ramp_without_changing_steer(arm, emergency_index):
    args = fixture(arm, emergency_index=emergency_index)
    got = audit.hold_and_steering_protocol(*args)
    assert got["status"] == "PASS", got["failures"]
    last = got["requests"][-1]
    assert last["normal_request"]["normal_control_executed"] is True
    assert last["normal_request"]["superseded_by_emergency"] is True
    assert last["final_next_control"]["brake"] == .5
    assert last["final_next_control"]["steer"] == audit.float32(.2)


@pytest.mark.parametrize("change,check", [
    (lambda a: a[0][6]["goal_stop"]["hold_brake_control"].update(source_timestamp=1.), "normal_source_execution_scope"),
    (lambda a: a[0][6]["goal_stop"]["hold_brake_control"].update(mode="immediate"), "normal_source_execution_scope"),
    (lambda a: a[0][6]["goal_stop"]["hold_brake_control"].update(mathematical_hold_brake=0.), "independent_measured_clock_plus_dt_formula"),
    (lambda a: a[0][6]["goal_stop"]["hold_brake_control"].update(next_interval_end_elapsed_seconds=0.), "independent_measured_clock_plus_dt_formula"),
    (lambda a: a[0][6]["goal_stop"]["hold_brake_control"]["returned_control"].update(brake=.1), "original_normal_pedals_and_only_pedals"),
    (lambda a: a[0][6]["goal_stop"]["hold_brake_control"]["returned_control"].update(steer=0.), "original_normal_pedals_and_only_pedals"),
    (lambda a: a[0][6]["goal_stop"]["hold_brake_control"].update(physical_application_proven=True), "normal_source_execution_scope"),
    (lambda a: a[0][6]["goal_stop"]["low_speed_steering"].update(mode="native"), "steering_source_decision_scope"),
    (lambda a: a[0][6]["goal_stop"]["low_speed_steering"]["input_control"].update(brake=1.), "original_emergency_supersession_or_normal_output"),
    (lambda a: a[0][6]["next_control"].update(brake=.1), "guarded_final_command_and_ACK"),
    (lambda a: a[3][6]["requested_control"].update(steer=1e-7), "guarded_final_command_and_ACK"),
    (lambda a: a[0][6]["goal_stop"].update(hold_ticks=1), "unchanged_measured_dwell"),
    (lambda a: a[0][7]["goal_stop"].update(hold_brake_source_timestamp=a[0][7]["timestamp"]), "unchanged_hold_latch"),
    (lambda a: a[0][-1]["goal_stop"]["hold_brake_control"].update(input_control=a[0][6]["goal_stop"]["hold_brake_control"]["input_control"]), "no_stale_normal_receipt_for_non_driving_command"),
])
def test_corruption_preserved_as_failure_with_all_witnesses(change, check):
    args = fixture("ramp")
    change(args)
    got = audit.hold_and_steering_protocol(*args)
    assert got["status"] == "FAIL"
    assert any(row["check"] == check for row in got["failures"])
    assert got["state_receipt_count"] == len(args[0]) and got["training_data_approved"] is False


def test_frame_timestamp_and_sequence_binding_fail_closed():
    args = fixture("ramp")
    args[1][2]["frame"] += 1
    with pytest.raises(ValueError, match="count/frame/time"): audit.hold_and_steering_protocol(*args)
    args = fixture("ramp")
    args[3].append(args[3][0])
    with pytest.raises(ValueError, match="duplicate"): audit.hold_and_steering_protocol(*args)


def test_emergency_brake_not_ramped_or_steer_zeroed():
    args = fixture("ramp", emergency_index=8)
    args[0][-1]["goal_stop"]["low_speed_steering"]["input_control"]["brake"] = .3
    got = audit.hold_and_steering_protocol(*args)
    assert any(row["check"] == "original_emergency_supersession_or_normal_output" for row in got["failures"])


def test_nonfinite_bool_and_path_alias_rejected(tmp_path):
    for value in (math.nan, math.inf, True):
        args = fixture("ramp")
        args[0][6]["goal_stop"]["hold_brake_control"]["input_control"]["brake"] = value
        with pytest.raises(ValueError): audit.hold_and_steering_protocol(*args)
    with pytest.raises(ValueError): audit.safe(tmp_path/"other"/".."/"raw")
    path = tmp_path/"bad.json"
    path.write_text('{"x":1e999}')
    with pytest.raises(ValueError): audit.checked(path, {})


def test_before_hold_excludes_only_treated_next_command():
    def physical(index):
        row = {name: 0. for name in audit.common.PHYSICAL_FIELDS}
        row.update(frame=100+index, timestamp=index*.05, current_control=audit.common.blank_control(),
            next_control=audit.common.blank_control(), goal_stop=dict(pilot_transition="zero_pedal_coast_to_service_brake_hold" if index == 2 else None))
        return row
    first = [physical(i) for i in range(4)]
    second = copy.deepcopy(first)
    second[2]["next_control"]["brake"] = .1
    assert audit.before_hold_comparison(first, second)["status"] == "EXACT_MATCH"
    second[1]["next_control"]["brake"] = .1
    assert audit.before_hold_comparison(first, second)["status"] == "DIFFERENT"
    second = copy.deepcopy(first)
    second[2]["current_control"]["brake"] = .1
    assert audit.before_hold_comparison(first, second)["status"] == "DIFFERENT"
    assert audit.before_hold_comparison([], first)["status"] == "UNAVAILABLE"


def readiness_fixture():
    samples, journal = [], [dict(kind="started", source_sha256="a"*64, started_at_utc="2026-09-14T08:00:00+00:00")]
    for i in range(3):
        sample = dict(index=i, elapsed_seconds=i*2., observed_at_utc=f"2026-09-14T08:00:0{i*2}+00:00",
            bindings=[dict(host="127.0.0.1", port=p, protocol=proto, status="CLEAR")
                for p in (2100,2101,2102) for proto in ("TCP","UDP")],
            all_six_clear=True, consecutive_clear_checks=i+1, ss=[])
        samples.append(sample)
        journal.append(dict(kind="bind_sample", **copy.deepcopy(sample)))
    report = dict(schema="carla.local_port_readiness.v1", status="READY_OBSERVED_NOT_RESERVED", ready_observed=True,
        bind_host="127.0.0.1", ports=[2100,2101,2102], protocols=["TCP","UDP"],
        observation_budget_seconds=60., minimum_sample_spacing_seconds=2., required_consecutive_clear_checks=3,
        source_sha256_before="a"*64, source_sha256_after="a"*64, samples=samples, elapsed_seconds=4.,
        final_consecutive_clear_checks=3, error=None, started_at_utc="2026-09-14T08:00:00+00:00",
        completed_at_utc="2026-09-14T08:00:04+00:00", sample_journal="readiness.json.samples.jsonl")
    report.update({key:False for key in ("simulator_contacted", "simulator_launched", "wrapper_invoked",
        "port_reserved_after_probe", "reuse_socket_options_used", "automatic_trial_retry", "foreign_processes_signaled", "installations_performed")})
    journal.append(dict(kind="finished", status=report["status"]))
    return report, journal


def test_readiness_exact_six_bind_streak_journal_and_no_contact():
    report, journal = readiness_fixture()
    got = audit.validate_readiness(report, journal, "a"*64)
    assert got["ready_observed"] is True and got["sample_count"] == 3 and got["port_reserved"] is False


@pytest.mark.parametrize("mutate", [
    lambda r,j:r.update(port_reserved_after_probe=True),
    lambda r,j:r.update(source_sha256_after="b"*64),
    lambda r,j:r["samples"][1].update(elapsed_seconds=1.999),
    lambda r,j:r["samples"][2]["bindings"].pop(),
    lambda r,j:r["samples"][2]["bindings"][0].update(status="BLOCKED"),
    lambda r,j:j[2].update(all_six_clear=False),
    lambda r,j:r.update(final_consecutive_clear_checks=2),
    lambda r,j:r.update(elapsed_seconds=60.1),
])
def test_readiness_false_success_rejected(mutate):
    report, journal = readiness_fixture()
    mutate(report,journal)
    with pytest.raises(ValueError): audit.validate_readiness(report,journal,"a"*64)


def write_json(path, value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(audit.encoded(value)+b"\n")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def execution_fixture(tmp_path, monkeypatch, *, blocked=False):
    monkeypatch.setattr(audit,"ROOT",tmp_path)
    campaign=tmp_path/"campaign"
    case=dict(id=audit.CASES[0][0],arm="immediate",command=["bash","wrapper"])
    directory=campaign/"execution"/case["id"]
    directory.mkdir(parents=True)
    report,journal=readiness_fixture()
    journal_path=directory/"readiness.json.samples.jsonl"
    journal_path.write_bytes(b"".join(audit.encoded(row)+b"\n" for row in journal))
    report["sample_journal_sha256"]=hashlib.sha256(journal_path.read_bytes()).hexdigest()
    readysha=write_json(directory/"readiness.json",report)
    plan=dict(declared_at_utc="2026-09-14T07:59:00+00:00", cases=[case],
        orchestration_sha256={"scripts/e2e/probe_carla_port_readiness.py":"a"*64})
    write_json(directory/"declaration.json",dict(schema="carla.v7_case_execution_declaration.v1",case_id=case["id"],arm="immediate",
        command=case["command"],plan_sha256=audit.PLAN_SHA256,maximum_wrapper_attempts=1,automatic_retry=False,
        training_data_approved=False,declared_at_utc="2026-09-14T07:59:59+00:00",prior_case_continuation=[]))
    write_json(directory/"wrapper_started.json",dict(schema="carla.v7_wrapper_invocation.v1",case_id=case["id"],command=case["command"],
        started_at_utc="2026-09-14T08:00:05+00:00",readiness_sha256=readysha,wrapper_attempt_count=1,automatic_retry=False))
    (directory/"wrapper.log").write_bytes(b"occupied\n" if blocked else b"done\n")
    ownersha=None if blocked else write_json(campaign/case["id"]/"owner_result.json",dict(exit_code=0,completed_at_utc="2026-09-14T08:01:00+00:00"))
    result=dict(schema="carla.v7_case_execution_result.v1",case_id=case["id"],arm="immediate",completed_at_utc="2026-09-14T08:01:01+00:00",
        wrapper_attempt_count=1,exit_code=1 if blocked else 0,status="PRELAUNCH_BLOCKED_NOT_CAPTURED" if blocked else "WRAPPER_COMPLETED",
        readiness_sha256=readysha,wrapper_log_sha256=hashlib.sha256((directory/"wrapper.log").read_bytes()).hexdigest(),
        owner_result_sha256=ownersha,automatic_retry=False,training_data_approved=False)
    write_json(directory/"result.json",result)
    return campaign,case,plan,directory


@pytest.mark.parametrize("blocked",[False,True])
def test_durable_execution_binds_journal_log_owner_and_absence(tmp_path,monkeypatch,blocked):
    campaign,case,plan,directory=execution_fixture(tmp_path,monkeypatch,blocked=blocked)
    got=audit.execution_record(campaign,case,plan,0,{})
    assert got["status"] == ("PRELAUNCH_BLOCKED_NOT_CAPTURED" if blocked else "WRAPPER_COMPLETED")
    assert got["wrapper_attempt_count"] == 1 and got["readiness"]["sample_count"] == 3
    (directory/"wrapper.log").write_bytes(b"changed")
    with pytest.raises(ValueError,match="SHA"): audit.execution_record(campaign,case,plan,0,{})


def test_durable_prelaunch_block_cannot_hide_raw_capture(tmp_path,monkeypatch):
    campaign,case,plan,_=execution_fixture(tmp_path,monkeypatch,blocked=True)
    (campaign/case["id"]).mkdir()
    with pytest.raises(ValueError,match="contradicts raw"): audit.execution_record(campaign,case,plan,0,{})


def trial(case_id,arm,count=10,denominator=100,maximum=20.):
    return dict(case_id=case_id,arm=arm,status="COMPLETE_DIAGNOSTIC_NOT_ADMITTED",native_state_count=10,camera_anchor_count=5,
        measured_future=dict(full_64_point_anchors=dict(sample_count=denominator,metrics=dict(
            xy_curvature=dict(violating_anchor_count=count,violating_step_count=count,
                unique_episode_relative_100ms_violation_tick_count=2,maximum=dict(value=maximum)),
            lateral_acceleration=dict(violating_anchor_count=0)))))


def test_full_four_denominators_no_cherry_pick_or_worse_max_suppression():
    trials=[trial(*case,count=5 if i in (1,2) else 10,maximum=30. if i in (1,2) else 20.) for i,case in enumerate(audit.CASES)]
    got=audit.comparison_verdict(trials,{"secondary":dict(status="DIFFERENT")})
    assert got["status"] == "FAVORABLE_CONDITION_SPECIFIC_SIGNAL_NOT_ADMITTED"
    assert got["exact_repeats_and_pretreatment_prefixes"] is False
    assert len(got["observed_cross_arm_comparisons"]) == 4
    assert all(row["worst_observed_curvature_increased"] for row in got["observed_cross_arm_comparisons"])
    trials[-1]["status"]="PRELAUNCH_BLOCKED_NOT_CAPTURED"
    got=audit.comparison_verdict(trials,{})
    assert got["status"] == "INCOMPLETE_OR_FAILED_NO_COMPARATIVE_VERDICT"
    with pytest.raises(ValueError): audit.comparison_verdict(trials[:3],{})


def test_count_improvement_with_worse_rate_is_not_favorable():
    trials=[trial(*case,count=5 if i in (1,2) else 10,denominator=20 if i in (1,2) else 100) for i,case in enumerate(audit.CASES)]
    got=audit.comparison_verdict(trials,{})
    assert got["status"] == "MIXED_OR_NO_CONSISTENT_IMPROVEMENT_NOT_ADMITTED"


def test_attempt_companion_complete_alias_still_binds_actual_result(tmp_path):
    records=[dict(case_id=case_id,arm=arm,status="WRAPPER_COMPLETED",wrapper_attempt_count=1,
        execution_result_path=f"execution/{case_id}/result.json",execution_result_sha256="a"*64) for case_id,arm in audit.CASES]
    plan=dict(cases=[dict(id=case_id,arm=arm,command=["fixed",case_id]) for case_id,arm in audit.CASES])
    doc=dict(schema="carla.hold_brake_ramp_execution_attempts.v1",plan_sha256=audit.PLAN_SHA256,training_data_approved=False,
        learned_model_control=False,recorded_at_utc="2026-09-14T08:10:00+00:00",records=[dict(case_id=case["id"],arm=case["arm"],command=case["command"],
        status="CAPTURE_COMPLETED",attempt_count=1,execution_result_path=record["execution_result_path"],execution_result_sha256=record["execution_result_sha256"])
        for case,record in zip(plan["cases"],records)])
    path=tmp_path/"attempts.json"
    pin=write_json(path,doc)
    assert audit.attempts_companion(path,pin,plan,records,{})==doc
    doc["records"][-1]["execution_result_sha256"]="b"*64
    pin=write_json(path,doc)
    with pytest.raises(ValueError,match="durable result"):audit.attempts_companion(path,pin,plan,records,{})


@pytest.mark.parametrize("field",["readiness_sha256","wrapper_log_sha256","owner_result_sha256"])
def test_mandatory_durable_hash_may_not_be_null(tmp_path,monkeypatch,field):
    campaign,case,plan,directory=execution_fixture(tmp_path,monkeypatch)
    result=json.loads((directory/"result.json").read_text())
    result[field]=None
    write_json(directory/"result.json",result)
    with pytest.raises(ValueError,match="mandatory SHA"):audit.execution_record(campaign,case,plan,0,{})


def test_readiness_journal_hash_may_not_be_null(tmp_path,monkeypatch):
    campaign,case,plan,directory=execution_fixture(tmp_path,monkeypatch)
    report=json.loads((directory/"readiness.json").read_text())
    report["sample_journal_sha256"]=None
    pin=write_json(directory/"readiness.json",report)
    result=json.loads((directory/"result.json").read_text())
    result["readiness_sha256"]=pin
    write_json(directory/"result.json",result)
    with pytest.raises(ValueError,match="mandatory SHA"):audit.execution_record(campaign,case,plan,0,{})


@pytest.mark.parametrize("status,count",[("READINESS_INVOCATION_FAILED_NOT_ATTEMPTED",0),
    ("PIN_MISMATCH_NOT_ATTEMPTED",0),("WRAPPER_INVOCATION_OR_WAIT_FAILED_STATUS_UNCONFIRMED",1)])
def test_orchestration_exception_without_exit_code_retained_not_cleanup_success(tmp_path,monkeypatch,status,count):
    campaign,case,plan,directory=execution_fixture(tmp_path,monkeypatch,blocked=True)
    if count==0:
        (directory/"wrapper_started.json").unlink()
    write_json(directory/"result.json",dict(schema="carla.v7_case_execution_result.v1",case_id=case["id"],
        completed_at_utc="2026-09-14T08:01:01+00:00",status=status,wrapper_attempt_count=count,error="observed exception",
        automatic_retry=False,training_data_approved=False))
    got=audit.execution_record(campaign,case,plan,0,{})
    assert got["status"]==status and got["wrapper_attempt_count"]==count and "exit_code" not in got
    assert got["readiness"]["ready_observed"] is None


def make_plan():
    return dict(schema="carla.hold_brake_ramp_abba_plan.v1", cases=[dict(id=case_id,arm=arm,command=["fixed",case_id],maximum_attempts=1)
        for case_id,arm in audit.CASES],maximum_attempts_per_revision=4,automatic_retry=False,
        source_sha256={name:"a"*64 for name in audit.common.EXECUTION_SOURCES},input_sha256={name:"a"*64 for name in audit.common.INPUT_PATHS},
        audit_math_sha256={name:"a"*64 for name in audit.common.AUDIT_MATH_PATHS},
        orchestration_sha256={name:"a"*64 for name in audit.ORCHESTRATION_PATHS},
        protocol_configuration_by_arm={arm:asdict(replace(audit.profiles.HoldBrakeRampDevelopmentGoalStopConfig(),hold_brake_mode=arm)) for arm in ("immediate","ramp")},
        training_data_approved=False,learned_model_control=False,vehicle_control_approved=False,dataset_export=False,target_retiming=False,gate_changes=False,
        declared_at_utc="2026-09-14T08:00:00+00:00")


def test_plan_v7_family_configuration_and_split_modes_explicit():
    plan=make_plan()
    audit.validate_plan(plan)
    plan["protocol_configuration_by_arm"]["ramp"]["hold_brake_mode"]="immediate"
    with pytest.raises(ValueError,match="configuration"):audit.validate_plan(plan)
    plan=make_plan()
    plan["schema"]="carla.low_speed_steering_abba_plan.v1"
    with pytest.raises(ValueError):audit.validate_plan(plan)


def test_outer_fresh_outputs_witness_denominators_and_sha_seal(tmp_path,monkeypatch):
    plan=make_plan()
    plan_path=tmp_path/"plan.json"
    pin=write_json(plan_path,plan)
    monkeypatch.setattr(audit,"PLAN_SHA256",pin)
    realchecked=audit.checked
    def checked(path,ledger,expected=None,**options):
        if options.get("binary"):
            return b"source-input-check-mocked"
        return realchecked(path,ledger,expected,**options)
    monkeypatch.setattr(audit,"checked",checked)
    monkeypatch.setattr(audit.raw,"source_identity",lambda:dict(files={}))
    monkeypatch.setattr(audit,"execution_record",lambda campaign,case,plan,index,ledger:dict(case_id=case["id"],arm=case["arm"],
        status="WRAPPER_COMPLETED",wrapper_attempt_count=1))
    def summarized(path,case,plan):
        report=trial(case["id"],case["arm"])
        states=[]
        requests=[]
        for i in range(10):
            row={key:0. for key in audit.common.PHYSICAL_FIELDS}
            row.update(frame=100+i,timestamp=i*.05,capture_phase="driving",goal_stop=dict(pilot_transition=None),next_control=audit.common.blank_control())
            states.append(row)
            requests.append(dict(frame=100+i,notice="all synthetic requests retained"))
        report["hold_and_steering_policy"]=dict(status="PASS",state_receipt_count=10,requests=requests)
        return report,[dict(path="one.jpg",sha256="a"*64)],[dict(frame=100)],states
    monkeypatch.setattr(audit,"summarize_one",summarized)
    output=tmp_path/"analysis"
    got,pairs=audit.run(tmp_path/"campaign",plan_path,pin,output)
    assert got["planned_case_count"]==4 and got["wrapper_attempted_case_count"]==4
    assert got["control_policy_witness_count"]==40 and got["audited_native_state_count"]==40
    witnesses=[json.loads(line) for line in (output/"control_policy_witnesses.jsonl").read_text().splitlines()]
    assert len(witnesses)==40 and all("measured_physical_state" in row for row in witnesses)
    for report in got["cases"]:
        policy=report["hold_and_steering_policy"]
        assert "requests" not in policy and policy["witness_count"]==10
        assert sum(row["case_id"]==report["case_id"] for row in witnesses)==policy["witness_count"]
    for line in (output/"SHA256SUMS").read_text().splitlines():
        expected,name=line.split("  ")
        assert hashlib.sha256((output/name).read_bytes()).hexdigest()==expected
    with pytest.raises(ValueError,match="fresh output"):audit.run(tmp_path/"campaign",plan_path,pin,output)
    assert pairs["prospective_primary_comparison"]["training_data_approved"] is False
