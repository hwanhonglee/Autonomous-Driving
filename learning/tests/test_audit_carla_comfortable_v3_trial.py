"""HH_260906 - Keep measured scalar QA separate from strict control-record continuity and source provenance."""

import copy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import audit_carla_comfortable_v3_trial as audit
from scripts.e2e.carla_goal_stop_profile import (
    DevelopmentGoalStopConfig, DevelopmentGoalStopGovernor, annotate_development_control,
    goal_stop_termination_reason,
)


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value) + "\n")


def protocol_fixture(emergency_tick=None):
    # HH_260906 - Synthetic kinematics exercise record contracts only, never physical data qualification.
    governor = DevelopmentGoalStopGovernor(DevelopmentGoalStopConfig(), 8.0, 20.0)
    states, rows = [], []
    for i in range(285 if emergency_tick is None else emergency_tick + 2):
        phase = "stationary_warmup" if i == 0 else "driving" if i <= 154 else "stationary_tail"
        speed, remaining = (0.0, 200.0) if i == 0 else (.5, 180.0) if i == 1 else (8.0, 100.0) if i <= 110 else (3.0, 30.0) if i == 111 else (3.0, 20.0) if i == 112 else (.05, .8)
        active = phase != "stationary_tail"
        record = governor.update(remaining, remaining, speed, -remaining, timestamp=i * .05,
            longitudinal_speed_mps=speed, driving=active, count_hold=phase == "driving")
        reason = goal_stop_termination_reason(record, False, 0.0, governor.config) if phase == "driving" else None
        command = SimpleNamespace(throttle=0.4, brake=0.0, steer=(i % 4) * .01)
        if not active or reason:
            command = SimpleNamespace(throttle=0.0, brake=1.0, steer=0.0)
        else:
            governor.normal_control(command)
        if emergency_tick == i:
            governor.emergency_override_count += 1
            governor.fail("comfortable_v3_emergency_override")
            command.throttle, command.brake = 0.0, 0.5
        record.update(basic_agent_done=False, termination_reason=reason, control_source="synthetic_normal_request")
        if i == 0:
            record["next_control_starts_driving"] = True
        annotate_development_control(record, governor, reason)
        current = copy.deepcopy(rows[-1]["next_control"]) if rows else dict(throttle=0., brake=1., steer=0.)
        row = dict(timestamp=i * .05, frame=1000 + i, phase=phase, speed_mps=speed, remaining_route_m=remaining,
            goal_error_m=remaining, terminal_overshoot_m=-remaining, recomputed_route_cte_m=0.,
            stopped_in_goal=remaining <= 1.0 and speed <= .1, current_control=current, next_control=vars(command))
        states.append(dict(vx=speed, goal_stop=record))
        rows.append(row)
    return states, rows


def test_independent_reconstruction_matches_frozen_governor_without_mutating_inputs():
    states, rows = protocol_fixture()
    before = copy.deepcopy((states, rows))
    report = audit.analyze_protocol(states, rows)
    assert report["all_checks_pass"]
    assert report["longest_continuous_cruise_seconds"] == pytest.approx(5.4)
    assert [x["transition"] for x in report["transitions"]] == ["launch_to_normal_pid", "normal_pid_to_zero_pedal_coast"]
    assert report["control_observation_alignment"]["one_row_mismatch_count"] == 0
    assert (states, rows) == before


@pytest.mark.parametrize("key,value,flag", [
    ("driving_elapsed_sec", 99., "governor_state_and_timers"),
    ("handoff_elapsed_sec", 99., "governor_state_and_timers"),
    ("pilot_state", "coast_low", "governor_state_and_timers"),
    ("normal_throttle_cap", .4, "governor_state_and_timers"),
    ("target_speed_mps", 8., "target_envelope_and_slew"),
    ("distance_speed_envelope_mps", 7., "target_envelope_and_slew"),
    ("training_data_approved", 0, "governor_state_and_timers"),
    ("next_control_starts_driving", True, "engagement_boundary"),
])
def test_forged_record_fields_do_not_change_recomputed_contract(key, value, flag):
    states, rows = protocol_fixture()
    states[20]["goal_stop"][key] = value
    assert flag in audit.analyze_protocol(states, rows)["failed_flags"]


@pytest.mark.parametrize("index,key,value", [(0, "throttle", .2), (20, "throttle", .4), (20, "brake", .11),
                                            (112, "throttle", .01), (120, "brake", .001), (155, "steer", .1)])
def test_phase_pedal_and_abort_controls_are_checked(index, key, value):
    states, rows = protocol_fixture()
    rows[index]["next_control"][key] = value
    assert "normal_pedal_protocol" in audit.analyze_protocol(states, rows)["failed_flags"]


def test_two_row_match_is_diagnostic_and_never_silently_realigned_to_pass():
    states, rows = protocol_fixture()
    rows[20]["current_control"] = copy.deepcopy(rows[18]["next_control"])
    report = audit.analyze_protocol(states, rows)
    assert report["failed_flags"] == ["requested_to_applied_control_link"]
    control = report["control_observation_alignment"]
    assert control["one_row_mismatch_count"] == 1
    assert control["nearest_matching_lookback_histogram"]["two_rows"] == 1
    assert control["mismatches"][0]["nearest_matching_lookback_rows"] == 2


def test_unmatched_control_does_not_get_a_fabricated_delay():
    states, rows = protocol_fixture()
    rows[20]["current_control"]["steer"] = .77
    control = audit.analyze_protocol(states, rows)["control_observation_alignment"]
    assert control["nearest_matching_lookback_histogram"]["neither_of_previous_two"] == 1
    assert control["mismatches"][0]["nearest_matching_lookback_rows"] is None


@pytest.mark.parametrize("field", ["planar", "vx"])
def test_actual_speed_hard_limit_has_no_comparison_epsilon(field):
    states, rows = protocol_fixture()
    value = math.nextafter(30 / 3.6, math.inf)
    if field == "planar": rows[20]["speed_mps"] = value
    else: states[20]["vx"] = value
    report = audit.analyze_protocol(states, rows)
    assert "actual_planar_and_vx_maximum" in report["failed_flags"]
    assert len(report["speed_violations"]) == 1


def test_cruise_requires_continuity_not_accumulated_hits():
    states, rows = protocol_fixture()
    rows[60]["speed_mps"] = 7.79
    assert not audit.analyze_protocol(states, rows)["flags"]["continuous_cruise"]
    states, rows = protocol_fixture()
    rows[60]["timestamp"] += .01
    assert not audit.analyze_protocol(states, rows)["flags"]["continuous_cruise"]


def test_emergency_keeps_original_half_brake_and_requires_following_observation():
    states, rows = protocol_fixture(emergency_tick=20)
    report = audit.analyze_protocol(states, rows)
    assert report["flags"]["emergency_observation_before_abort"]
    assert report["flags"]["normal_pedal_protocol"]
    assert report["emergency_override_count"] == 1
    assert not audit.analyze_protocol(states[:-1], rows[:-1])["flags"]["emergency_observation_before_abort"]
    rows[20]["next_control"]["brake"] = .1
    assert not audit.analyze_protocol(states, rows)["flags"]["emergency_observation_before_abort"]


@pytest.fixture
def source_trial(tmp_path, monkeypatch):
    trial = tmp_path / "run_001"
    sources = {name: name.encode() for name in audit.SOURCE_NAMES}
    pins = {name: hashlib.sha256(data).hexdigest() for name, data in sources.items()}
    monkeypatch.setattr(audit, "REVIEWED_SOURCES", {name: pins[name] for name in audit.REVIEWED_SOURCES})
    monkeypatch.setattr(audit, "recorded_commit_bytes", lambda commit, name: sources[name])
    for name, data in sources.items():
        path = trial / "provenance" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    plan = dict(source_bytes_archived=True, bounds_source_bytes_archived=True, source_sha256=pins,
        source_head_commit="a" * 40, source_worktree_status="", route_sha256=audit.ROUTE_SHA,
        map="Town07", capture_mode="expert", port=2100, host="127.0.0.1", planned_at_utc="2026-09-07T18:09:00+00:00")
    write(trial / "owner_plan.json", plan)
    write(trial / "owner_result.json", dict(source_bytes_unchanged_and_archived=True, source_checks={name: True for name in sources}))
    write(trial / "owner_started.json", dict(server_pid=123, server_pgid=123))
    for i, stage in enumerate(("ready", "after_capture", "stopped")):
        write(trial / f"lifecycle/{stage}.json", dict(status="PASS", read_only=True, stage=stage,
            owner_pid=123, owner_pgid=123, generation_id="expert_123", port=2100, host="127.0.0.1",
            expected_map="Town07", active_map_basename="Town07", mode="stopped" if i == 2 else "running",
            checked_at=f"2026-09-07T18:1{i}:00+00:00", port_released=True, owner_process_state=None if i == 2 else "S"))
    monkeypatch.setattr(audit.base, "summarize_trial", lambda root, bounds: (dict(source_manifest=[], raw_quality_candidate=False), None))
    return trial


def test_unrelated_dirty_analysis_is_reported_without_claiming_entire_worktree_clean(source_trial):
    path = source_trial / "owner_plan.json"
    plan = json.loads(path.read_text())
    plan["source_worktree_status"] = " M scripts/e2e/summarize_carla_goal_stop_trials.py"
    write(path, plan)
    result = audit.audit_trial(source_trial)
    assert not result["whole_worktree_was_clean"]
    assert result["recorded_source_worktree_status"] == plan["source_worktree_status"]
    assert result["archived_execution_sources_match_recorded_commit"]
    assert not result["development_screen_clear"]


def test_commit_bytes_must_equal_archive_even_if_archive_pin_matches(source_trial, monkeypatch):
    monkeypatch.setattr(audit, "recorded_commit_bytes", lambda *args: b"different committed execution code")
    with pytest.raises(audit.base.EvidenceError, match="differs from recorded Git commit"):
        audit.audit_trial(source_trial)


@pytest.mark.parametrize("mutation,match", [
    (lambda p: p.update(bounds_source_bytes_archived=False), "complete source and bounds"),
    (lambda p: p["source_sha256"].pop("portable_e2e/model.py"), "ten expected"),
    (lambda p: p.update(source_head_commit="HEAD"), "source identity"),
])
def test_missing_or_unsafe_source_provenance_fails_closed(source_trial, mutation, match):
    path = source_trial / "owner_plan.json"
    plan = json.loads(path.read_text()); mutation(plan); write(path, plan)
    with pytest.raises(audit.base.EvidenceError, match=match): audit.audit_trial(source_trial)


def test_modified_archived_source_fails_before_any_result(source_trial):
    (source_trial / "provenance/portable_e2e/model.py").write_bytes(b"modified")
    with pytest.raises(audit.base.EvidenceError, match="source SHA mismatch"): audit.audit_trial(source_trial)


@pytest.mark.parametrize("change", [dict(owner_pid=124), dict(generation_id="expert_999"), dict(mode="stopped"),
                                   dict(active_map_basename="Town03"), dict(checked_at="2026-09-07T17:00:00+00:00")])
def test_lifecycle_generations_map_and_time_cannot_be_substituted(source_trial, change):
    path = source_trial / "lifecycle/after_capture.json"
    record = json.loads(path.read_text()); record.update(change); write(path, record)
    with pytest.raises(audit.base.EvidenceError, match="lifecycle"): audit.audit_trial(source_trial)


def test_safe_reader_rejects_duplicate_keys_paths_and_symlinks(tmp_path):
    (tmp_path / "duplicate.json").write_text('{"a": 1, "a": 2}')
    with pytest.raises(audit.base.EvidenceError): audit.read_json(tmp_path, "duplicate.json", {})
    with pytest.raises(audit.base.EvidenceError): audit.checked_bytes(tmp_path, "../duplicate.json", {})
    (tmp_path / "alias").symlink_to(tmp_path / "duplicate.json")
    with pytest.raises(audit.base.EvidenceError): audit.checked_bytes(tmp_path, "alias", {})


@pytest.fixture
def campaign(source_trial):
    root = source_trial.parent / "campaign"
    target = root / "town07_straight_calibration/run_001"
    target.parent.mkdir(parents=True)
    source_trial.rename(target)
    write(root / "pilot_plan.json", dict(profile="comfortable_v3", maximum_attempts_per_revision=2,
        automatic_retry=False, training_data_approved=False, route_sha256=audit.ROUTE_SHA,
        source_commit_short="aaaaaaa", declared_at_utc="2026-09-07T18:08:00Z"))
    return root


def test_finalized_capture_without_episode_remains_in_denominator_and_new_output_only(campaign, tmp_path):
    output = tmp_path / "audit_output"
    result = audit.audit_campaign(campaign, output)
    assert result["trial_count"] == result["development_screen_fail_count"] == 1
    assert result["base_raw_quality_candidate_count"] == 0
    expected, name = (output / "SHA256SUMS").read_text().strip().split("  ")
    assert audit.base.sha(output / name) == expected
    with pytest.raises(audit.base.EvidenceError): audit.audit_campaign(campaign, output)
    with pytest.raises(audit.base.EvidenceError): audit.audit_campaign(campaign, campaign / "nested")


@pytest.mark.parametrize("name", ["run_002", "run_003", "run_bad"])
def test_missing_or_extra_attempt_is_never_silently_omitted(campaign, tmp_path, name):
    (campaign / "town07_straight_calibration" / name).mkdir()
    output = tmp_path / "output"
    with pytest.raises(audit.base.EvidenceError): audit.audit_campaign(campaign, output)
    assert not output.exists()


def test_source_changed_during_audit_is_detected_before_writing(campaign, tmp_path, monkeypatch):
    original = audit.audit_trial
    def mutate(root):
        result = original(root)
        (root / "provenance/portable_e2e/model.py").write_bytes(b"race")
        return result
    monkeypatch.setattr(audit, "audit_trial", mutate)
    with pytest.raises(audit.base.EvidenceError, match="changed during campaign audit"):
        audit.audit_campaign(campaign, tmp_path / "output")
    assert not (tmp_path / "output").exists()


def test_cli_rejects_abbreviations_and_commit_reader_has_no_network(monkeypatch):
    with pytest.raises(SystemExit): audit.main(["pilot", "--output-d", "output"])
    calls = []
    def run(argv, **kwargs):
        calls.append((argv, kwargs))
        return SimpleNamespace(returncode=0, stdout=b"frozen")
    monkeypatch.setattr(audit.subprocess, "run", run)
    assert audit.recorded_commit_bytes("a" * 40, "portable_e2e/model.py") == b"frozen"
    assert calls[0][0] == ["git", "-c", "protocol.allow=never", "show", "a" * 40 + ":portable_e2e/model.py"]
    assert calls[0][1]["timeout"] == 10
    assert calls[0][1]["env"]["GIT_NO_LAZY_FETCH"] == "1"
    assert calls[0][1]["env"]["GIT_ALLOW_PROTOCOL"] == ""
    assert calls[0][1]["env"]["GIT_TERMINAL_PROMPT"] == "0"
    with pytest.raises(audit.base.EvidenceError): audit.recorded_commit_bytes("HEAD", "portable_e2e/model.py")
