"""HH_260906 - Retain failed/blocked cases and independently test steering receipts without simulator execution."""
import copy
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import summarize_carla_low_speed_steering_abba as audit

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(json.dumps(value, allow_nan=False).encode()+b"\n")


def control(steer=.2):
    return SimpleNamespace(throttle=.4, steer=steer, brake=.1, hand_brake=False, reverse=False, gear=0, manual_gear_shift=False)


def policy_fixture(arm):
    config = replace(audit.profiles.LowSpeedSteeringDevelopmentGoalStopConfig(), low_speed_steering_mode=arm)
    gov = audit.profiles.DevelopmentGoalStopGovernor(config, 8., 20.)
    states, native, receipts = [], [], []
    measurements = [("stationary_warmup", 0., 100.), ("stationary_warmup", 0., 100.),
        ("driving", .5, 99.), ("driving", 3., 20.), ("driving", .75, 5.),
        ("driving", math.nextafter(.75, math.inf), 4.)]
    measurements += [("driving", .1 if i == 0 else .02, .8) for i in range(41)]
    measurements += [("stationary_tail", 0., .8)]
    previous = audit.blank_control()
    for index, (phase, speed, remaining) in enumerate(measurements):
        stamp, frame = index*.05, 1000+index
        initial = index == 1
        record = gov.update(remaining, remaining, speed, -remaining, timestamp=stamp,
            longitudinal_speed_mps=speed, driving=phase == "driving", count_hold=phase == "driving")
        if initial:
            record = gov.update(remaining, remaining, speed, -remaining, timestamp=stamp,
                longitudinal_speed_mps=speed, driving=True, count_hold=False)
            record["next_control_starts_driving"] = True
        stop = "comfortable_goal_measured_stop_and_dwell" if phase == "driving" and record["complete"] else None
        driving_command = initial or (phase == "driving" and stop is None)
        proposed = gov.normal_control(control()) if driving_command else SimpleNamespace(**audit.blank_control())
        record["termination_reason"] = stop
        audit.profiles.annotate_development_control(record, gov, stop)
        record["low_speed_steering_event_monitor"] = dict(observed_event_counts=dict(collision=0, lane_invasion=0),
            source_timestamp=stamp, asynchronous_event_delivery=True, zero_latency_abort_proven=False)
        audit.profiles.apply_low_speed_steering(proposed, gov, record, driving_command=driving_command)
        final = dict(vars(proposed))
        if speed < .3 and (final["brake"] > .05 or final["hand_brake"]): final["steer"] = 0.
        reason = "driving_end_brake" if stop else "initial_drive_control" if initial else "next_"+phase+"_control"
        if stop: final = audit.blank_control()
        row = dict(frame=frame, timestamp=stamp, capture_phase=phase, vx=speed, vy=0.,
            world_velocity_carla=[speed, 0., 0.], goal_stop=record,
            current_control=previous, next_control=final, collision=[], lane_invasion=[],
            control_transport=dict(status="PASS", next_command_receipt_sequence=index+1))
        states.append(row)
        native.append(dict(frame=frame, timestamp=stamp, stopped_in_goal=speed <= .1 and remaining <= 1.))
        receipts.append(dict(sequence=index+1, reason=reason, requested_control=copy.deepcopy(final)))
        previous = copy.deepcopy(final)
    return states, native, {}, receipts, arm


@pytest.mark.parametrize("arm", ["native", "zero"])
def test_original_policy_only_steer_guard_initial_hold_and_all_witnesses(arm):
    args = policy_fixture(arm)
    before = copy.deepcopy(args)
    result = audit.steering_protocol(*args)
    assert result["status"] == "PASS" and result["failures"] == []
    assert args == before
    assert result["state_receipt_count"] == len(args[0]) == len(result["requests"])
    assert result["initial_engagement_source_count"] == 1
    assert result["hold_source_timestamp"] == args[0][6]["timestamp"]
    assert result["requests"][1]["initial_engagement"] is True
    assert result["requests"][-2]["final_replacement_reason"] == "driving_end_brake"
    assert args[0][4]["goal_stop"]["low_speed_steering"]["eligible"] is True
    assert args[0][5]["goal_stop"]["low_speed_steering"]["eligible"] is False


@pytest.mark.parametrize("change,check", [
    (lambda a: a[0][2]["goal_stop"]["low_speed_steering"].update(source_timestamp=999.), "policy_source_decision_and_scope"),
    (lambda a: a[0][2]["goal_stop"]["low_speed_steering"].update(source_measured_speed_mps=0.), "policy_source_decision_and_scope"),
    (lambda a: a[0][2]["goal_stop"]["low_speed_steering"].update(threshold_mps=.8), "policy_source_decision_and_scope"),
    (lambda a: a[0][2]["goal_stop"]["low_speed_steering"].update(mode="native"), "policy_source_decision_and_scope"),
    (lambda a: a[0][2]["goal_stop"]["low_speed_steering"]["returned_control"].update(throttle=.6), "only_requested_steer_changed"),
    (lambda a: a[0][2]["goal_stop"]["low_speed_steering"].update(physical_application_proven=True), "policy_source_decision_and_scope"),
    (lambda a: a[0][2]["next_control"].update(steer=.4), "guarded_final_command_and_ACK"),
    (lambda a: a[0][2]["next_control"].update(steer=1e-7), "guarded_final_command_and_ACK"),
    (lambda a: a[3][2]["requested_control"].update(brake=.9), "guarded_final_command_and_ACK"),
    (lambda a: a[0][7]["goal_stop"].update(normal_hold_brake=.5), "unchanged_hold_brake_latch"),
    (lambda a: a[0][7]["goal_stop"].update(hold_ticks=100), "unchanged_measured_dwell"),
    (lambda a: a[0][2]["goal_stop"]["low_speed_steering_event_monitor"].update(zero_latency_abort_proven=True), "bounded_asynchronous_event_monitor"),
])
def test_policy_corruption_is_retained_as_failure_not_admitted(change, check):
    args = policy_fixture("zero")
    change(args)
    result = audit.steering_protocol(*args)
    assert result["status"] == "FAIL"
    assert any(row["check"] == check for row in result["failures"])
    assert len(result["requests"]) == len(args[0])
    assert result["training_data_approved"] is False


def test_truncated_native_rows_and_missing_initial_source_are_rejected():
    args = policy_fixture("zero")
    args[1].pop()
    with pytest.raises(ValueError, match="count/frame/time"): audit.steering_protocol(*args)
    args = policy_fixture("zero")
    args[0][1]["goal_stop"].pop("next_control_starts_driving")
    with pytest.raises(ValueError, match="initial command source"): audit.steering_protocol(*args)


def test_nonfinite_and_bool_control_cannot_become_a_valid_receipt():
    args = policy_fixture("zero")
    args[0][2]["goal_stop"]["low_speed_steering"]["input_control"]["steer"] = math.nan
    with pytest.raises(ValueError): audit.steering_protocol(*args)
    args = policy_fixture("zero")
    args[0][2]["goal_stop"]["low_speed_steering"]["input_control"]["brake"] = True
    with pytest.raises(ValueError): audit.steering_protocol(*args)


def physical_row(index=0):
    row = {name: 0. for name in audit.PHYSICAL_FIELDS}
    row.update(frame=1000+index, timestamp=index*.05, capture_phase="stationary_warmup" if index < 2 else "driving",
        current_control=audit.blank_control(), next_control=audit.blank_control())
    return row


def test_exact_repeats_report_mismatches_without_clock_normalization():
    first = [physical_row(i) for i in range(4)]
    second = copy.deepcopy(first)
    for row in second: row["frame"] += 77; row["timestamp"] += 10.
    assert audit.compare_physical(first, second)["status"] == "EXACT_MATCH"
    second[1]["next_control"]["steer"] = .1
    assert audit.compare_physical(first, second, warmup_only=True)["status"] == "EXACT_MATCH"
    assert audit.compare_physical(first, second)["mismatched_native_count"] == 1
    second[1]["x"] = 1e-16
    got = audit.compare_physical(first, second, warmup_only=True)
    assert got["status"] == "DIFFERENT" and got["maximum_absolute_scalar_difference"]["x"] == 1e-16
    assert audit.compare_physical([], first)["status"] == "UNAVAILABLE"


def report(case, *, count=134, denominator=666, maximum=16.79, status="COMPLETE_DIAGNOSTIC_NOT_ADMITTED", other=0):
    return dict(case_id=case["id"], arm=case["arm"], status=status, native_state_count=10, camera_anchor_count=5,
        training_data_approved=False, measured_future=dict(full_64_point_anchors=dict(sample_count=denominator,
            metrics=dict(xy_curvature=dict(violating_anchor_count=count, violating_step_count=count, maximum=dict(value=maximum),
                unique_episode_relative_100ms_violation_tick_count=3), speed_limit=dict(violating_anchor_count=other)))))


def test_primary_requires_both_zero_repeats_against_both_native_and_preserves_maximum_regression():
    cases = [dict(id=name, arm=arm) for name, arm in audit.CASES]
    rows = [report(case, count=115 if case["arm"] == "zero" else 134, denominator=671 if case["arm"] == "zero" else 666,
        maximum=18.44 if case["arm"] == "zero" else 16.79) for case in cases]
    result = audit.comparison_verdict(rows)
    assert result["status"] == "FAVORABLE_CONDITION_SPECIFIC_SIGNAL_NOT_ADMITTED"
    assert len(result["observed_cross_arm_comparisons"]) == 4
    assert all(r["worst_observed_curvature_increased"] for r in result["observed_cross_arm_comparisons"])
    assert result["training_data_approved"] is False
    rows[2] = report(cases[2], count=140)
    assert audit.comparison_verdict(rows)["status"] == "MIXED_OR_NO_CONSISTENT_IMPROVEMENT_NOT_ADMITTED"
    rows[2] = report(cases[2], count=115, other=1)
    assert audit.comparison_verdict(rows)["status"] == "MIXED_OR_NO_CONSISTENT_IMPROVEMENT_NOT_ADMITTED"


def test_incomplete_abba_reports_available_pairs_but_never_a_favorable_verdict():
    cases = [dict(id=name, arm=arm) for name, arm in audit.CASES]
    rows = [report(case, count=115 if case["arm"] == "zero" else 134) for case in cases]
    rows[-1] = dict(case_id=cases[-1]["id"], arm="native", status="PRELAUNCH_BLOCKED_NOT_CAPTURED")
    result = audit.comparison_verdict(rows)
    assert result["status"] == "INCOMPLETE_OR_FAILED_NO_COMPARATIVE_VERDICT"
    assert result["available_cross_arm_comparison_count"] == 2
    assert result["per_case_counts_and_rates"][cases[-1]["id"]]["curvature_violating_anchor_fraction"] is None


@pytest.fixture
def campaign(tmp_path, monkeypatch):
    root = tmp_path/"campaign"
    root.mkdir()
    plan = dict(schema="carla.low_speed_steering_abba_plan.v1", declared_at_utc="2026-09-14T07:00:00+00:00",
        maximum_attempts_per_revision=4, automatic_retry=False, training_data_approved=False,
        learned_model_control=False, vehicle_control_approved=False,
        source_sha256={name: digest(_hh_source_path(audit.ROOT, name)) for name in audit.EXECUTION_SOURCES},
        input_sha256={name: digest(_hh_source_path(audit.ROOT, name)) for name in audit.INPUT_PATHS},
        audit_math_sha256={name: digest(_hh_source_path(audit.ROOT, name)) for name in audit.AUDIT_MATH_PATHS},
        protocol_configuration_by_arm={arm: asdict(replace(audit.profiles.LowSpeedSteeringDevelopmentGoalStopConfig(), low_speed_steering_mode=arm)) for arm in ("native", "zero")},
        cases=[dict(id=name, arm=arm, maximum_attempts=1, command=["bash", "scripts/e2e/run_owned_carla_expert_trial.sh", str(root/name), next(iter(audit.INPUT_PATHS)), "--", "--low-speed-steering-mode", arm]) for name, arm in audit.CASES])
    write(root/"plan.json", plan)
    pin = digest(root/"plan.json")
    monkeypatch.setattr(audit, "PLAN_SHA256", pin)
    return root, root/"plan.json", pin, plan


def test_plan_normalized_whole_quantity_is_allowed_but_bool_or_bound_changes_rejected(campaign):
    plan = campaign[-1]
    audit.validate_plan(plan)
    plan["protocol_configuration_by_arm"]["native"]["hold_seconds"] = 2
    audit.validate_plan(plan)
    plan["protocol_configuration_by_arm"]["native"]["hold_seconds"] = True
    with pytest.raises(ValueError, match="configuration differs"): audit.validate_plan(plan)


def mock_cases(monkeypatch):
    def fake(root, case, plan):
        row = report(case)
        row["steering_policy"] = dict(status="PASS", state_receipt_count=2, requests=[dict(frame=10), dict(frame=11)])
        return row, [dict(path="images/actual.jpg")], [], []
    monkeypatch.setattr(audit, "summarize_one", fake)


def test_create_only_output_links_every_policy_witness_and_preserves_denials(campaign, tmp_path, monkeypatch):
    mock_cases(monkeypatch)
    root, plan, pin, _ = campaign
    output = tmp_path/"report"
    summary, pairs = audit.run(root, plan, pin, output)
    rows = [json.loads(line) for line in (output/"steering_policy_witnesses.jsonl").read_text().splitlines()]
    assert len(rows) == summary["steering_policy_witness_count"] == 8
    for case in summary["cases"]:
        policy = case["steering_policy"]
        assert "requests" not in policy
        assert sum(r["case_id"] == case["case_id"] for r in rows) == policy["witness_count"] == policy["state_receipt_count"]
        assert (output/policy["witness_file"]).is_file()
    assert set(p.name for p in output.iterdir()) == {"summary.json", "paired_comparisons.json", "image_inventory.jsonl", "future_windows.jsonl", "steering_policy_witnesses.jsonl", "SHA256SUMS"}
    for line in (output/"SHA256SUMS").read_text().splitlines():
        expected, name = line.split("  ")
        assert digest(output/name) == expected
    assert summary["training_data_approved"] is False and summary["dataset_export"] is False
    with pytest.raises(ValueError, match="fresh output"): audit.run(root, plan, pin, output)


def companion(campaign):
    root, _, pin, plan = campaign
    records = []
    for case in plan["cases"]:
        base = dict(case_id=case["id"], arm=case["arm"], command=case["command"], attempt_count=1)
        if case["id"] == audit.CASES[-1][0]:
            base.update(status="PRELAUNCH_BLOCKED_NOT_CAPTURED", exit_code=1, observed_output="CARLA port 2101 is already occupied",
                execution_tool_session_id=7813, output_directory_created=False, owner_started=False, collector_started=False, automatic_retry=False)
        else:
            path = root/case["id"]/"owner_result.json"
            write(path, dict(exit_code=0))
            base.update(status="CAPTURE_COMPLETED", owner_result_path=str(path))
        records.append(base)
    document = dict(schema="carla.low_speed_steering_execution_attempts.v1", recorded_at_utc="2026-09-14T08:01:28+00:00",
        plan_sha256=pin, records=records, training_data_approved=False, learned_model_control=False)
    path = root/"execution_attempts.json"
    write(path, document)
    return path, digest(path)


def test_prelaunch_attempt_is_counted_without_inventing_raw_rows_or_complete_abba(campaign, monkeypatch):
    path, attempt_pin = companion(campaign)
    root, plan, pin, _ = campaign
    def fake(case_root, case, plan):
        return (report(case) if case_root.exists() else dict(case_id=case["id"], arm=case["arm"], status="NOT_RUN", native_state_count=None, camera_anchor_count=None)), [], [], []
    monkeypatch.setattr(audit, "summarize_one", fake)
    summary, pairs = audit.run(root, plan, pin, execution_attempts_path=path, execution_attempts_sha256=attempt_pin)
    assert summary["case_status_counts"] == {"COMPLETE_DIAGNOSTIC_NOT_ADMITTED": 3, "PRELAUNCH_BLOCKED_NOT_CAPTURED": 1}
    assert summary["planned_case_count"] == summary["attempted_case_count"] == 4
    assert summary["cases"][-1]["native_state_count"] is None
    assert summary["all_four_completed_diagnostics"] is False
    assert pairs["prospective_primary_comparison"]["status"] == "INCOMPLETE_OR_FAILED_NO_COMPARATIVE_VERDICT"
    assert not (root/audit.CASES[-1][0]).exists()
    with pytest.raises(ValueError, match="must be hash-bound"): audit.run(root, plan, pin)


@pytest.mark.parametrize("mutation", [lambda d: d.update(plan_sha256="0"*64),
    lambda d: d["records"][-1].update(attempt_count=2),
    lambda d: d["records"][-1].update(collector_started=True),
    lambda d: d["records"][-1].update(exit_code=0)])
def test_attempt_claims_cannot_hide_retries_or_invent_a_block(campaign, mutation):
    path, _ = companion(campaign)
    document = json.loads(path.read_text()); mutation(document); write(path, document)
    with pytest.raises(ValueError): audit.execution_attempts(path, digest(path), campaign[-1], campaign[0], {})


def test_absent_and_unfinalized_cases_do_not_open_raw_payload(campaign, monkeypatch):
    root, _, _, plan = campaign
    case = plan["cases"][0]
    result = audit.summarize_one(root/case["id"], case, plan)
    assert result[0]["status"] == "NOT_RUN" and result[0]["native_state_count"] is None
    (root/case["id"]).mkdir()
    monkeypatch.setattr(audit, "checked", lambda *args, **kwargs: pytest.fail("no finalized receipt: raw read prohibited"))
    assert audit.summarize_one(root/case["id"], case, plan)[0]["status"] == "NOT_FINALIZED"


def test_validation_failure_stays_in_four_case_denominator(campaign, monkeypatch):
    monkeypatch.setattr(audit, "summarize_one", lambda *args: (_ for _ in ()).throw(ValueError("bad ACK")))
    summary, _ = audit.run(*campaign[:3])
    assert len(summary["cases"]) == 4 and summary["case_status_counts"] == {"FAILED_VALIDATION": 4}
    assert summary["all_four_completed_diagnostics"] is False


def test_source_tampering_and_parent_traversal_block_publication(campaign, tmp_path, monkeypatch):
    mock_cases(monkeypatch)
    root, path, pin, plan = campaign
    with pytest.raises(ValueError, match="frozen prospective"): audit.run(root, path, "0"*64)
    (tmp_path/"other").mkdir()
    with pytest.raises(ValueError, match="parent traversal"):
        audit.run(root, path, pin, tmp_path/"other/../campaign"/audit.CASES[0][0]/"new")
    plan["audit_math_sha256"]["portable_e2e/model.py"] = "0"*64
    write(path, plan); new_pin = digest(path); monkeypatch.setattr(audit, "PLAN_SHA256", new_pin)
    with pytest.raises(ValueError, match="SHA-256 mismatch"): audit.run(root, path, new_pin, tmp_path/"new")
    assert not (tmp_path/"new").exists()


def test_json_overflow_duplicates_and_cli_abbreviations_rejected(tmp_path):
    path = tmp_path/"input.json"
    for payload in ('{"a":1e999}', '{"a":NaN}', '{"a":1,"a":2}'):
        path.write_text(payload)
        with pytest.raises(ValueError): audit.checked(path, {})
    with pytest.raises(SystemExit): audit.main(["--campaign", "x"])


def test_repeated_unpinned_read_cannot_replace_initial_digest(tmp_path):
    path = tmp_path/"metadata.json"
    write(path, dict(value=1))
    ledger = {}
    audit.checked(path, ledger)
    write(path, dict(value=2))
    with pytest.raises(ValueError, match="repeated reads"): audit.checked(path, ledger)


def test_payload_growth_after_stat_is_still_bounded(tmp_path, monkeypatch):
    path = tmp_path/"metadata.json"
    path.write_text("{}")
    monkeypatch.setattr(audit, "MAX_EVIDENCE_BYTES", 16)
    monkeypatch.setattr(Path, "read_bytes", lambda self: b"x"*17)
    with pytest.raises(ValueError, match="exceeds byte bound"): audit.checked(path, {})
