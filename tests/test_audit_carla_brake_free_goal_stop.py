"""HH_260906 - Exercise separate frozen V4 evidence without CARLA, GPU or public-file changes."""

import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import audit_carla_brake_free_goal_stop as audit
from scripts.e2e.carla_goal_stop_profile import (
    BrakeFreeDevelopmentGoalStopConfig, DevelopmentGoalStopGovernor,
    annotate_development_control, goal_stop_termination_reason,
)

PLAN = json.loads(r'''{
  "comment": "HH_260906 - Preregister zero normal brake as a single-parameter control comparison against both acknowledged v3 trials; retain every outcome.",
  "declared_at_utc": "2026-09-07T19:13:59Z",
  "source_commit": "9d3dcab9d425df1ba673f1e932686049bd17a011",
  "collector_sha256": "53d213bdef56895a27969cc6dd4eac638ba4fa6697badeaaa3367bd5741f01d0",
  "goal_helper_sha256": "6e70b4362b805b73be66bc727afc79a8684dfc12f2f5d69998fbfffa9eef3dab",
  "profile": "comfortable_v4",
  "control_transport": "acknowledged_batch",
  "changed_numerical_configuration": {
    "normal_brake_cap": {
      "from": 0.1,
      "to": 0
    }
  },
  "route_sha256": "8285e70a790d5e8ae75803db1aa538122b56e6fcfc3586e60eaa947d330417c9",
  "route_length_m": 210.5975914062836,
  "map": "Town07",
  "scenario": "straight",
  "vehicle": "vehicle.toyota.prius",
  "weather": "ClearNoon",
  "seed": 0,
  "quality": "Low",
  "nominal_target_speed_kmh": 28.8,
  "maximum_actual_speed_kmh": 30,
  "physics_hz": 20,
  "camera_hz": 10,
  "maximum_total_sim_seconds": 180,
  "wall_timeout_seconds": 900,
  "finish_before_utc": "2026-09-08T01:00:00Z",
  "maximum_attempts_per_revision": 2,
  "automatic_retry": false,
  "first_output": "town07_straight_calibration/run_001",
  "baseline_outputs": [
    "../acknowledged_control_v1/town07_straight_calibration/run_001",
    "../acknowledged_control_v1/town07_straight_calibration/run_002"
  ],
  "normal_lateral_controller_unchanged": true,
  "emergency_control_unchanged": true,
  "quality_limits_unchanged": true,
  "training_data_approved": false,
  "learned_model_control": false,
  "remote_training_started": false,
  "expected_risk": "With zero normal brake the desired deceleration envelope may be unachievable; existing coast-entry band and endpoint failures remain active. No late-brake rescue or parameter adjustment within a trial.",
  "notice": "Independent review required after run001 before optional final run002. All prior and new failures stay in their denominators. Server receipts and reported controls do not prove physical torque timing."
}''')

def protocol_fixture(emergency_tick=None):
    # HH_260906 - Synthetic kinematics exercise record contracts only, never physical data qualification.
    governor = DevelopmentGoalStopGovernor(BrakeFreeDevelopmentGoalStopConfig(), 8.0, 20.0)
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
            governor.fail("comfortable_v4_emergency_override")
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


def test_v4_explicit_profile_is_separate_from_frozen_v3():
    old_profile = copy.deepcopy(audit.pilot.FROZEN)
    old_sources = copy.deepcopy(audit.ack.REVIEWED_ACK_SOURCES)
    states, rows = protocol_fixture()
    previous = copy.deepcopy((states, rows))
    report = audit.analyze_protocol(states, rows)
    assert report["all_checks_pass"]
    assert (states, rows) == previous
    assert audit.pilot.FROZEN == old_profile and audit.ack.REVIEWED_ACK_SOURCES == old_sources
    assert {k for k in audit.FROZEN if audit.FROZEN[k] != old_profile[k]} == {"profile_id", "normal_brake_cap"}


@pytest.mark.parametrize("index,key,value", [(20, "brake", .000000001), (20, "brake", .1), (0, "throttle", .2),
    (20, "throttle", .41), (112, "throttle", .01), (120, "brake", .001), (155, "steer", .1)])
def test_zero_normal_brake_and_unchanged_phase_controls_fail_closed(index, key, value):
    states, rows = protocol_fixture()
    rows[index]["next_control"][key] = value
    assert "normal_pedal_protocol" in audit.analyze_protocol(states, rows)["failed_flags"]


@pytest.mark.parametrize("key,value", [("normal_brake_cap", .1), ("hold_ticks", 999), ("target_speed_mps", 8),
    ("pilot_state", "complete"), ("training_data_approved", 0), ("next_control_starts_driving", True)])
def test_forged_v4_row_metadata_cannot_create_pass(key, value):
    states, rows = protocol_fixture()
    states[20]["goal_stop"][key] = value
    assert not audit.analyze_protocol(states, rows)["all_checks_pass"]


def test_emergency_brake_and_latched_failed_partial_without_tail_are_retained():
    states, rows = protocol_fixture(emergency_tick=120)
    report = audit.analyze_protocol(states, rows)
    assert report["emergency_override_count"] == 1
    assert report["latched_failure"] == "comfortable_v4_emergency_override"
    assert report["flags"]["emergency_observation_before_abort"] is True
    assert all(r["phase"] != "stationary_tail" for r in rows)
    rows[-1]["next_control"]["brake"] = 0
    assert "normal_pedal_protocol" in audit.analyze_protocol(states, rows)["failed_flags"]


def test_failed_empty_protocol_is_not_admitted():
    result = audit.analyze_protocol([], [])
    assert not result["all_checks_pass"]
    transport = audit.ack.analyze_transport([], [], None)
    assert transport["status"] == "FAIL" and transport["receipt_count"] == 0


def test_frozen_prospective_document():
    audit.validate_plan_document(PLAN)


@pytest.mark.parametrize("key,value", [("profile", "comfortable_v3"), ("physics_hz", True), ("normal_lateral_controller_unchanged", 1),
    ("maximum_attempts_per_revision", 3), ("source_commit", "f" * 40), ("quality", "Epic"),
    ("changed_numerical_configuration", {"normal_brake_cap": {"from": .1, "to": .01}}),
    ("declared_at_utc", "2026-09-08T02:00:00Z")])
def test_changed_prospective_contract_rejected(key, value):
    plan = copy.deepcopy(PLAN)
    plan[key] = value
    with pytest.raises(audit.base.EvidenceError):
        audit.validate_plan_document(plan)


@pytest.mark.parametrize("change", ["extra", "missing"])
def test_extra_or_missing_plan_fields_rejected(change):
    plan = copy.deepcopy(PLAN)
    if change == "extra":
        plan["allow_relaxed_quality"] = True
    else:
        plan.pop("expected_risk")
    with pytest.raises(audit.base.EvidenceError):
        audit.validate_plan_document(plan)


@pytest.mark.parametrize("timestamp", ["2026-09-07T19:00:00", "2026-09-08T04:00:00+09:00", 123])
def test_non_utc_prospective_timestamp_refused(timestamp):
    with pytest.raises(audit.base.EvidenceError):
        audit.utc(timestamp)


def test_strict_nested_boolean_and_numerical_comparisons():
    assert audit.strict_equal(20, 20.)
    assert not audit.strict_equal(True, 1)
    assert not audit.strict_equal({"cap": False}, {"cap": 0})
    assert not audit.strict_equal(float("nan"), 0)


def test_public_sanitizer_preserves_source_hashes_without_private_paths():
    raw = {"path": "/home/anotheruser/autoware_e2e/artifacts/run", "hash": "d" * 64,
           "args": ["/home/person/private", "127.0.0.1"]}
    result = audit.sanitize(raw)
    assert result == {"path": "${REPO_ROOT}/artifacts/run", "hash": "d" * 64,
                      "args": ["${USER_HOME}/private", "${HOST}"]}
    assert raw["path"].startswith("/home/")


def test_existing_or_aliased_category_and_raw_child_are_refused_before_audit(tmp_path, monkeypatch):
    raw, output = tmp_path / "raw", tmp_path / "category"
    raw.mkdir()
    output.mkdir()
    (output / "keep").write_bytes(b"unchanged")
    monkeypatch.setattr(audit, "audit_campaign", lambda *a: pytest.fail("must fail before reading raw data"))
    with pytest.raises(audit.base.EvidenceError):
        audit.publish(raw, raw, output)
    assert (output / "keep").read_bytes() == b"unchanged"
    with pytest.raises(audit.base.EvidenceError):
        audit.new_output(raw / "new", [raw])
    alias = tmp_path / "alias"
    alias.symlink_to(tmp_path / "missing")
    with pytest.raises(audit.base.EvidenceError):
        audit.new_output(alias, [raw])


def test_unknown_cli_abbreviation_is_not_accepted(tmp_path):
    with pytest.raises(SystemExit):
        audit.main(["--camp", str(tmp_path), "--baseline-root", str(tmp_path), "--output", str(tmp_path / "new")])

@pytest.fixture
def owned_trial(tmp_path, monkeypatch):
    # HH_260906 - Only this test fixture replaces source pins; no production CLI accepts a source hash override.
    root = tmp_path / "run_001"
    def write(relative, value):
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(value if isinstance(value, bytes) else json.dumps(value).encode())
    sources = {name: (name + " synthetic").encode() for name in audit.pilot.SOURCE_NAMES}
    hashes = {name: hashlib.sha256(raw).hexdigest() for name, raw in sources.items()}
    for name, raw in sources.items():
        write("provenance/" + name, raw)
    monkeypatch.setattr(audit, "REVIEWED_SOURCES", {name: hashes[name] for name in audit.REVIEWED_SOURCES})
    monkeypatch.setattr(audit.pilot, "recorded_commit_bytes", lambda _commit, name: sources[name])
    plan = dict(source_bytes_archived=True, bounds_source_bytes_archived=True, source_sha256=hashes,
        source_head_commit="a" * 40, host="127.0.0.1", map="Town07", port=2100, capture_mode="expert", quality="Low",
        route_sha256=audit.pilot.ROUTE_SHA, collector_argv=["collector.py", "--control-transport", "acknowledged_batch"])
    write("owner_plan.json", plan)
    write("owner_result.json", dict(source_bytes_unchanged_and_archived=True, source_checks={name: True for name in sources},
                                   learned_model_control=False, vehicle_control_approved=False))
    write("owner_started.json", dict(server_pid=123, server_pgid=123, map="Town07", port=2100, quality="Low"))
    log = b"synthetic lifecycle log\n"
    write("server.log", log)
    for i, stage in enumerate(("ready", "after_capture", "stopped")):
        write("lifecycle/" + stage + ".json", dict(status="PASS", stage=stage, read_only=True, owner_pid=123, owner_pgid=123,
            generation_id="expert_123", host="127.0.0.1", port=2100, expected_map="Town07", active_map_basename="Town07",
            mode="stopped" if i == 2 else "running", port_released=True, owner_process_state=None,
            checked_at=f"2026-09-08T00:0{i}:00+00:00", server_log=dict(size_bytes=len(log), sha256=hashlib.sha256(log).hexdigest())))
    return root


def test_new_owned_protocol_sources_are_verified_without_mutating_old_reviewed_pins(owned_trial):
    old = copy.deepcopy(audit.pilot.REVIEWED_SOURCES)
    plan, _ = audit.verify_owner_sources(owned_trial, {})
    assert plan["capture_mode"] == "expert" and audit.pilot.REVIEWED_SOURCES == old


@pytest.mark.parametrize("relative,mutation", [
    ("owner_plan.json", lambda value: value.update(quality="Epic")),
    ("owner_plan.json", lambda value: value["collector_argv"].__setitem__(-1, "legacy_async")),
    ("owner_plan.json", lambda value: value["source_sha256"].pop("portable_e2e/model.py")),
    ("owner_result.json", lambda value: value["source_checks"].update({"portable_e2e/model.py": False})),
    ("lifecycle/stopped.json", lambda value: value.update(port_released=False)),
    ("lifecycle/ready.json", lambda value: value["server_log"].update(sha256="0" * 64)),
])
def test_owner_source_transport_and_lifecycle_corruption_fails_closed(owned_trial, relative, mutation):
    path = owned_trial / relative
    value = json.loads(path.read_text())
    mutation(value)
    path.write_text(json.dumps(value))
    with pytest.raises(audit.base.EvidenceError):
        audit.verify_owner_sources(owned_trial, {})


def test_unreviewed_new_collector_is_rejected_even_when_git_archive_pin_agrees(owned_trial, monkeypatch):
    monkeypatch.setitem(audit.REVIEWED_SOURCES, "scripts/e2e/collect_carla_vad_expert.py", "0" * 64)
    with pytest.raises(audit.base.EvidenceError, match="unreviewed"):
        audit.verify_owner_sources(owned_trial, {})


def test_active_trial_does_not_get_an_empty_pass(tmp_path):
    root = tmp_path / "run_001"
    root.mkdir()
    with pytest.raises(audit.base.EvidenceError, match="INCOMPLETE"):
        audit.audit_trial(root)


def test_archive_must_match_reviewed_commit_even_if_owner_commit_matches(owned_trial, monkeypatch):
    original = audit.pilot.recorded_commit_bytes
    monkeypatch.setattr(audit.pilot, "recorded_commit_bytes", lambda commit, name:
                        b"different reviewed code" if commit == audit.COMMIT else original(commit, name))
    with pytest.raises(audit.base.EvidenceError, match="reviewed V4 commit"):
        audit.verify_owner_sources(owned_trial, {})


@pytest.fixture
def prospective(tmp_path, monkeypatch):
    # HH_260906 - Synthetic files verify temporal semantics without modifying any executed campaign.
    campaign = tmp_path / "brake_free_goal_stop_v4"
    root = campaign / "town07_straight_calibration/run_001"
    baseline = tmp_path / "acknowledged_control_v1/town07_straight_calibration"
    root.mkdir(parents=True)
    baseline.mkdir(parents=True)
    (campaign / "pilot_plan.json").write_text(json.dumps(PLAN))
    sources = {name: name.encode() for name in audit.pilot.SOURCE_NAMES}
    hashes = {name: hashlib.sha256(value).hexdigest() for name, value in sources.items()}
    monkeypatch.setattr(audit.pilot, "recorded_commit_bytes", lambda commit, name: sources[name])
    for name, raw in sources.items():
        p = root / "provenance" / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(raw)
    def write(relative, value):
        p = root / relative
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(value))
    write("owner_plan.json", dict(collector_wall_timeout_sec=900, finish_before_utc=PLAN["finish_before_utc"],
        source_head_commit="b" * 40, source_sha256=hashes, planned_at_utc="2026-09-07T19:14:00Z"))
    write("owner_started.json", dict(started_at_utc="2026-09-07T19:14:02Z"))
    write("owner_result.json", dict(completed_at_utc="2026-09-07T19:14:05Z", exit_code=1))
    for stage, value in (("ready", "01"), ("after_capture", "03"), ("stopped", "04")):
        write("lifecycle/" + stage + ".json", dict(checked_at="2026-09-07T19:14:" + value + "Z"))
    result = dict(trial_id="run_001", source_head_commit="b" * 40, archived_source_sha256=hashes,
        all_ten_archives_match_owner_and_reviewed_commits=True, source_manifest=[])
    return campaign, baseline, [result]


def test_publication_only_owner_head_difference_with_exact_ten_bytes_is_explicit(prospective):
    report = audit.validate_campaign_plan(*prospective)
    assert report["attempt_count"] == 1 and report["maximum_attempt_count"] == 2
    assert report["historical_file_creation_time_proven"] is False
    assert report["discovered_attempts"][0]["owner_head"] != report["discovered_attempts"][0]["reviewed_execution_commit"]


@pytest.mark.parametrize("relative,key,value", [
    ("owner_plan.json", "planned_at_utc", "2026-09-07T19:13:58Z"),
    ("owner_started.json", "started_at_utc", "2026-09-07T19:14:00Z"),
    ("lifecycle/after_capture.json", "checked_at", "2026-09-07T19:14:01Z"),
    ("owner_result.json", "completed_at_utc", "2026-09-08T01:00:01Z"),
    ("owner_plan.json", "collector_wall_timeout_sec", 1000)])
def test_plan_order_owner_record_semantics_and_deadline_fail_closed(prospective, relative, key, value):
    root = prospective[0] / "town07_straight_calibration/run_001"
    p = root / relative
    record = json.loads(p.read_text())
    record[key] = value
    p.write_text(json.dumps(record))
    with pytest.raises(audit.base.EvidenceError):
        audit.validate_campaign_plan(*prospective)


def test_extra_attempt_or_successful_subset_never_passes(prospective):
    campaign, baseline, results = prospective
    (campaign / "town07_straight_calibration/run_003").mkdir()
    with pytest.raises(audit.base.EvidenceError, match="denominator"):
        audit.validate_campaign_plan(campaign, baseline, results)


def test_exact_first_review_recomputation_and_timestamp(tmp_path):
    qa = dict(raw_scalar_quality_clear=True, event_counts=dict(collision=0, lane_invasion=0),
        final_driving=dict(goal_error_m=.8), speed_rate_qa={name: {"by_phase": {"all": {"minimum_mps2": -1., "maximum_mps2": 2.}}}
            for name in ("native_20hz", "camera_10hz")})
    first = dict(independent_qa=qa, transport_protocol=dict(status="PASS", acknowledged_count=100, control_mismatch_count=0),
        owner_exit_code=0, normal_driving_nonzero_brake_count=0)
    first_plan, second_plan = dict(source_head_commit=audit.COMMIT), dict(source_head_commit="a" * 40)
    ledger = {"town07_straight_calibration/run_001/episode/" + name: {"sha256": "d" * 64}
              for name in ("states.jsonl", "control_receipts.jsonl")}
    review = dict(previous_output="town07_straight_calibration/run_001", next_output="town07_straight_calibration/run_002",
        reviewed_execution_source_commit=audit.COMMIT, previous_owner_head=audit.COMMIT, next_expected_owner_head="a" * 40,
        owner_exit_code=0, source_checks_passed=10, lifecycle_ready_after_stopped="PASS", independent_transport_status="PASS",
        acknowledged_receipts=100, control_mismatch_count=0, native_scalar_quality="PASS", collision_count=0, lane_invasion_count=0,
        normal_driving_nonzero_brake_count=0, source_and_parameters_changed=False, maximum_attempts_per_revision=2,
        automatic_retry=False, training_data_approved=False, goal_distance_m=.8, previous_states_sha256="d" * 64,
        previous_receipts_sha256="d" * 64, comment="HH_260906 - Test review.", reviewed_at_utc="2026-09-07T19:20:00Z",
        head_change_notice="Same execution source, changed publication HEAD.", reason="Repeat same condition.")
    for cadence in ("native", "camera"):
        review[f"minimum_{cadence}_speed_rate_mps2"], review[f"maximum_{cadence}_speed_rate_mps2"] = -1., 2.
    args = (first, first_plan, second_plan, audit.utc("2026-09-07T19:19:00Z"), audit.utc("2026-09-07T19:21:00Z"), ledger)
    audit.verify_second_review(review, *args)
    for field, changed in (("previous_states_sha256", "e" * 64), ("goal_distance_m", .9),
                           ("reviewed_at_utc", "2026-09-07T19:22:00Z"), ("training_data_approved", True)):
        altered = dict(review, **{field: changed})
        with pytest.raises(audit.base.EvidenceError):
            audit.verify_second_review(altered, *args)


