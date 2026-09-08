"""HH_260906 - Test strict matrix/resume/source/governor evidence with synthetic fixtures and no simulator or remote changes."""

import copy
from dataclasses import asdict
import hashlib
import io
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import audit_carla_turn_launch_matrix as audit
from scripts.e2e.carla_goal_stop_profile import (turn_launch_configuration, DevelopmentGoalStopGovernor,
    annotate_development_control, goal_stop_termination_reason)
from tests.test_audit_carla_turn_low_goal_stop import capture_flags as old_capture_flags


def capture_flags(profile):
    values = old_capture_flags()
    values[values.index("--goal-stop-profile") + 1] = profile
    return values + ["--agent-initialization", "after_bootstrap"]


def protocol_fixture(profile, emergency_tick=None):
    # HH_260906 - Synthetic kinematics exercise record contracts only, never physical data qualification.
    governor = DevelopmentGoalStopGovernor(turn_launch_configuration(profile), 4.0, 20.0)
    states, rows = [], []
    for i in range(285 if emergency_tick is None else emergency_tick + 2):
        phase = "stationary_warmup" if i == 0 else "driving" if i <= 154 else "stationary_tail"
        speed, remaining = (0.0, 200.0) if i == 0 else (.5, 180.0) if i == 1 else (4.0, 100.0) if i <= 110 else (3.0, 30.0) if i == 111 else (3.0, 20.0) if i == 112 else (.05, .8)
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
            governor.fail(f"{profile}_emergency_override")
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


@pytest.mark.parametrize("profile", audit.PEDALS)
def test_all_literal_launch_pedals_share_exact_later_origin_without_old_pin_changes(profile):
    originals = copy.deepcopy((audit.turn.FROZEN, audit.turn.REVIEWED_SOURCES, audit.initialization_audit.REVIEWED_SOURCES))
    assert audit.frozen_configuration(profile) == asdict(turn_launch_configuration(profile))
    states, rows = protocol_fixture(profile)
    report = audit.analyze_protocol(states, rows, profile)
    assert report["all_checks_pass"]
    assert rows[0]["next_control"]["throttle"] == audit.PEDALS[profile]
    assert rows[1]["next_control"]["throttle"] == .1525
    assert originals == (audit.turn.FROZEN, audit.turn.REVIEWED_SOURCES, audit.initialization_audit.REVIEWED_SOURCES)


@pytest.mark.parametrize("profile", audit.PEDALS)
@pytest.mark.parametrize("mutation", ["launch", "wrong_later_origin", "normal_brake", "profile", "target", "clock", "speed", "tail"])
def test_named_profile_and_all_phase_control_constraints_fail_closed(profile, mutation):
    states, rows = protocol_fixture(profile)
    if mutation == "launch": rows[0]["next_control"]["throttle"] += .01
    elif mutation == "wrong_later_origin":
        rows[1]["next_control"]["throttle"] = .4
        states[1]["goal_stop"]["normal_throttle_cap"] = .4
    elif mutation == "normal_brake": rows[50]["next_control"]["brake"] = 1e-12
    elif mutation == "profile": states[100]["goal_stop"]["profile_id"] = "turn_low_v1"
    elif mutation == "target": states[60]["goal_stop"]["target_speed_mps"] = 8.
    elif mutation == "clock": states[60]["goal_stop"]["handoff_elapsed_sec"] += .05
    elif mutation == "speed": states[50]["vx"], rows[50]["speed_mps"] = 4.3000001, 4.3000001
    elif mutation == "tail": rows[200]["next_control"]["throttle"] = .01
    assert not audit.analyze_protocol(states, rows, profile)["all_checks_pass"]


@pytest.mark.parametrize("profile", audit.PEDALS)
def test_emergency_failure_and_no_tail_are_retained(profile):
    states, rows = protocol_fixture(profile, emergency_tick=120)
    report = audit.analyze_protocol(states, rows, profile)
    assert report["latched_failure"] == profile + "_emergency_override"
    assert report["flags"]["emergency_observation_before_abort"]
    # HH_260906 - A correctly recorded emergency abort can satisfy the command protocol without completing raw goal/quality.
    assert report["all_checks_pass"] and all(row["phase"] != "stationary_tail" for row in rows)


def test_unknown_profiles_and_old_profile_not_accepted():
    for profile in ("turn_low_v1", "turn_launch_011_v1", True, None):
        with pytest.raises(audit.base.EvidenceError): audit.frozen_configuration(profile)


@pytest.mark.parametrize("mutation", ["duplicate_profile", "abbreviated", "old_profile", "extra_option", "old_initialization", "late_wall_flag"])
def test_exact_case_cli_no_unknown_arguments(tmp_path, mutation):
    argv = [str(tmp_path / "episode"), "route.json", "--host", "127.0.0.1", "--port", "2100", *capture_flags("turn_launch_013_v1")]
    if mutation == "duplicate_profile": argv += ["--goal-stop-profile", "turn_launch_013_v1"]
    elif mutation == "abbreviated": argv[argv.index("--goal-stop-profile")] = "--goal-stop-prof"
    elif mutation == "old_profile": argv[argv.index("--goal-stop-profile") + 1] = "turn_low_v1"
    elif mutation == "extra_option": argv += ["--allow-map-load"]
    elif mutation == "old_initialization": argv[-1] = "before_bootstrap"
    elif mutation == "late_wall_flag": argv += ["--wall-timing"]
    with pytest.raises(audit.base.EvidenceError): audit.validate_cli({"port": 2100, "route_path": "route.json", "collector_argv": argv}, tmp_path)


def test_case_order_two_each_and_no_automatic_extra_retry():
    cases = audit.expected_cases()
    assert [c["profile"] for c in cases] == list(audit.ORDER)
    assert len({c["output"] for c in cases}) == 8
    assert all(sum(c["profile"] == p for c in cases) == 2 for p in audit.PEDALS)


@pytest.fixture
def resume_fixture(tmp_path):
    # HH_260906 - The old plan remains immutable; only the six previously unstarted cases get a separate bound.
    campaign = tmp_path / "campaign"
    source = {"synthetic": "b" * 64}
    completed = {}
    for case in audit.expected_cases()[:2]:
        path = campaign / case["output"] / "owner_result.json"
        path.parent.mkdir(parents=True)
        path.write_text(json.dumps({"completed_at_utc": "2026-09-07T20:56:00Z", "exit_code": case["sequence"] - 1}))
        completed[case["case_id"]] = {"output": case["output"], "owner_result_sha256": audit.base.sha(path)}
    document = {"schema": "portable_e2e.turn_launch_resume_authorization.v1", "original_plan_sha256": audit.ORIGINAL_PLAN_SHA,
        "original_finish_before_utc": audit.ORIGINAL_DEADLINE, "resumed_finish_before_utc": audit.RESUMED_DEADLINE,
        "allowed_case_sequences": [3, 4, 5, 6, 7, 8], "frozen_source_commit": audit.COMMIT,
        "only_changed_execution_bound": "finish_before_utc for the six not-yet-started cases",
        "original_order_and_two_replicates_per_profile_unchanged": True, "new_attempts_added": 0,
        "automatic_retry": False, "precase_review_required": True, "no_training_admission": True, "old_cases_reclassified": False,
        "comment": "HH_260906 - Synthetic resume authorization.", "declared_at_utc": "2026-09-08T04:54:00Z",
        "user_request": "Continue.", "completed_cases": completed, "source_hashes": source,
        "interruption_notice": "Afternoon resumption, not uninterrupted overnight execution."}
    return document, {"source_hashes": source}, campaign


def test_explicit_resume_only_changes_unstarted_case_deadline(resume_fixture):
    document, plan, campaign = resume_fixture
    before = copy.deepcopy(document)
    ledger = {}
    assert audit.validate_resume_document(document, plan, campaign, ledger) == document
    assert len(ledger) == 2 and document == before


@pytest.mark.parametrize("key,value", [("original_plan_sha256", "a" * 64), ("frozen_source_commit", "a" * 40),
    ("resumed_finish_before_utc", "2026-09-08T09:00:00Z"), ("allowed_case_sequences", [2, 3, 4, 5, 6, 7, 8]),
    ("new_attempts_added", 1), ("new_attempts_added", False), ("automatic_retry", True), ("old_cases_reclassified", True),
    ("no_training_admission", False), ("declared_at_utc", "2026-09-07T20:54:00Z"),
    ("declared_at_utc", "2026-09-08T08:01:00Z"), ("source_hashes", {"changed": "f" * 64})])
def test_resume_cannot_retroactively_relax_sources_order_quota_or_scope(resume_fixture, key, value):
    document, plan, campaign = resume_fixture
    document[key] = value
    with pytest.raises(audit.base.EvidenceError): audit.validate_resume_document(document, plan, campaign, {})


@pytest.mark.parametrize("mutation", ["previous_sha", "previous_output", "missing_previous", "late_previous", "foreign_field"])
def test_resume_binds_exact_two_prior_outcomes(resume_fixture, mutation):
    document, plan, campaign = resume_fixture
    first = audit.expected_cases()[0]
    if mutation == "previous_sha": document["completed_cases"][first["case_id"]]["owner_result_sha256"] = "b" * 64
    elif mutation == "previous_output": document["completed_cases"][first["case_id"]]["output"] = "different"
    elif mutation == "missing_previous": document["completed_cases"].pop(first["case_id"])
    elif mutation == "late_previous":
        path = campaign / first["output"] / "owner_result.json"
        path.write_text(json.dumps({"completed_at_utc": "2026-09-08T04:56:00Z", "exit_code": 0}))
        document["completed_cases"][first["case_id"]]["owner_result_sha256"] = audit.base.sha(path)
    elif mutation == "foreign_field": document["training_approved"] = True
    with pytest.raises(audit.base.EvidenceError): audit.validate_resume_document(document, plan, campaign, {})


def test_tiny_negative_vx_has_no_invented_preregistered_reverse_threshold():
    trial = {"independent_qa": {"event_counts": {"collision": 0, "lane_invasion": 0}},
        "transport_protocol": {"status": "PASS"}, "initialization_protocol": {"status": "PASS"},
        "pilot_protocol": {"flags": {"actual_planar_and_vx_maximum": True}}}
    control = dict(throttle=0., brake=0., steer=0., reverse=False, hand_brake=False, manual_gear_shift=False, gear=1)
    state = dict(vx=-.00159, current_control=control, next_control=copy.deepcopy(control))
    result = audit.continuation_observations(trial, [state])
    assert result["minimum_recorded_vx_mps"] == -.00159 and not result["numerical_reverse_threshold_preregistered"]
    assert not result["established_continuation_stop_condition"]
    state["next_control"]["reverse"] = True
    assert audit.continuation_observations(trial, [state])["established_continuation_stop_condition"]


def test_output_must_be_new_and_outside_raw(tmp_path, monkeypatch):
    raw = tmp_path / "raw"
    raw.mkdir()
    monkeypatch.setattr(audit, "audit_campaign", lambda *_: pytest.fail("existing output must fail first"))
    for output in (raw, raw / "new"):
        with pytest.raises(audit.base.EvidenceError): audit.main([str(raw), "--output-dir", str(output)])


@pytest.fixture
def owned_trial(tmp_path, monkeypatch):
    # HH_260906 - Synthetic owner archives and lifecycle only, with the explicit new initialization CLI.
    root = tmp_path / "run_001"
    root.mkdir()
    sources = {name: ("synthetic source " + name).encode() for name in audit.SOURCES}
    hashes = {name: hashlib.sha256(raw).hexdigest() for name, raw in sources.items()}
    monkeypatch.setattr(audit, "recorded_bytes", lambda commit, name: sources[name])
    monkeypatch.setattr(audit, "REVIEWED_SOURCES", hashes)
    for name, raw in sources.items():
        path = root / "provenance" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(raw)
    plan = {"source_bytes_archived": True, "bounds_source_bytes_archived": True, "wall_timing_enabled": True,
        "wall_timing_source_bytes_archived": True, "wall_timing_schema": "carla.expert_wall_timing.v1", "source_sha256": hashes,
        "source_head_commit": audit.COMMIT, "host": "127.0.0.1", "map": audit.MAP, "quality": "Epic", "capture_mode": "expert",
        "worker_path": "scripts/e2e/collect_carla_vad_expert.py", "route_sha256": audit.ROUTE_SHA, "route_path": "route.json",
        "learned_model_control": False, "vehicle_control_approved": False, "server_extra_options": ["-RenderOffScreen", "-nosound"],
        "collector_wall_timeout_sec": 900, "finish_before_utc": "2026-09-08T01:00:00Z", "port": 2100,
        "planned_at_utc": "2026-09-07T20:00:00Z", "collector_argv": [str(root / "episode"), "route.json", "--host", "127.0.0.1", "--port", "2100", *capture_flags("turn_launch_013_v1")]}
    outcome = {"source_checks": {name: True for name in sources}, "source_bytes_unchanged_and_archived": True,
        "capture_mode": "expert", "learned_model_control": False, "vehicle_control_approved": False,
        "completed_at_utc": "2026-09-07T20:04:00Z", "exit_code": 1}
    started = {"map": audit.MAP, "quality": "Epic", "port": 2100, "server_pid": 123, "server_pgid": 123,
               "started_at_utc": "2026-09-07T20:01:01Z"}
    log = b"synthetic server evidence, no server was launched\n"
    (root / "server.log").write_bytes(log)
    def write(name, data):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data))
    for name, value in (("owner_plan.json", plan), ("owner_result.json", outcome), ("owner_started.json", started)):
        write(name, value)
    for minute, stage in ((1, "ready"), (2, "after_capture"), (3, "stopped")):
        write("lifecycle/" + stage + ".json", {"status": "PASS", "stage": stage, "read_only": True, "owner_pid": 123,
            "owner_pgid": 123, "generation_id": "expert_123", "host": "127.0.0.1", "port": 2100, "expected_map": audit.MAP,
            "mode": "stopped" if stage == "stopped" else "running", "port_released": stage == "stopped",
            "owner_process_state": None if stage == "stopped" else "running", "active_map_basename": audit.MAP,
            "server_log": {"size_bytes": len(log), "sha256": hashlib.sha256(log).hexdigest()},
            "checked_at": f"2026-09-07T20:0{minute}:00Z"})
    return root


def test_explicit_eleven_source_owner_and_stopped_proof(owned_trial):
    ledger = {}
    plan, owner, _, _ = audit.verify_owner_sources(owned_trial, ledger)
    assert set(plan["source_sha256"]) == audit.SOURCES and owner["exit_code"] == 1
    assert sum(name.startswith("provenance/") for name in ledger) == 11


@pytest.mark.parametrize("name,key,value", [("owner_plan.json", "quality", "Low"), ("owner_plan.json", "map", "Town07"),
    ("owner_plan.json", "wall_timing_enabled", False), ("owner_result.json", "source_bytes_unchanged_and_archived", False),
    ("owner_started.json", "server_pgid", 999), ("lifecycle/stopped.json", "port_released", False),
    ("lifecycle/after_capture.json", "checked_at", "2026-09-07T20:05:00Z")])
def test_owner_source_and_lifecycle_mutations_fail_closed(owned_trial, name, key, value):
    path = owned_trial / name
    data = json.loads(path.read_text())
    data[key] = value
    path.write_text(json.dumps(data))
    with pytest.raises(audit.base.EvidenceError): audit.verify_owner_sources(owned_trial, {})


def test_resumed_owner_must_use_only_explicit_new_deadline(owned_trial):
    path = owned_trial / "owner_plan.json"
    plan = json.loads(path.read_text())
    plan["finish_before_utc"] = audit.RESUMED_DEADLINE
    path.write_text(json.dumps(plan))
    with pytest.raises(audit.base.EvidenceError): audit.verify_owner_sources(owned_trial, {})
    assert audit.verify_owner_sources(owned_trial, {}, finish_before_utc=audit.RESUMED_DEADLINE)[0]["finish_before_utc"] == audit.RESUMED_DEADLINE


def test_publication_only_owner_head_may_differ_only_if_all_source_blobs_match(owned_trial, monkeypatch):
    path = owned_trial / "owner_plan.json"
    plan = json.loads(path.read_text())
    plan["source_head_commit"] = "f" * 40
    path.write_text(json.dumps(plan))
    seen = []
    def recorded(commit, name):
        seen.append((commit, name))
        return (owned_trial / "provenance" / name).read_bytes()
    monkeypatch.setattr(audit, "recorded_bytes", recorded)
    audit.verify_owner_sources(owned_trial, {})
    assert {(commit, name) for commit, name in seen} == {(commit, name) for name in audit.SOURCES for commit in (audit.COMMIT, "f" * 40)}
    monkeypatch.setattr(audit, "recorded_bytes", lambda commit, name: b"changed" if commit == "f" * 40 else (owned_trial / "provenance" / name).read_bytes())
    with pytest.raises(audit.base.EvidenceError): audit.verify_owner_sources(owned_trial, {})


@pytest.fixture
def review_fixture(tmp_path, monkeypatch):
    # HH_260906 - Synthetic independent measurements exercise exact historical review bindings, never actual pass criteria.
    root = tmp_path / "previous"
    root.mkdir()
    prefix = root / "episode.partial"
    prefix.mkdir()
    names = ["owner_plan.json", "owner_result.json", "lifecycle/stopped.json"] + ["episode.partial/" + n for n in
        ("manifest.json", "states.jsonl", "control_receipts.jsonl", "camera_frames.jsonl")]
    for name in names:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({"completed_at_utc": "2026-09-07T20:50:00Z", "synthetic": name}))
    native = {"maximum_mps2": 3.5, "minimum_mps2": -1.6}
    qa = {"speed_rate_qa": {"native_20hz": {"by_phase": {"all": native}},
                            "camera_10hz": {"by_phase": {"all": {"maximum_mps2": 2.4}}}},
        "phase_counts": {"stationary_warmup": {"camera_anchors": 35}, "driving": {"camera_anchors": 870}, "stationary_tail": {"camera_anchors": 65}},
        "failed_flags": ["native_decoder_speed_rate_clear"], "maximum_measured_speed_kmh": 14.4, "final_driving": {"goal_error_m": .95}}
    previous = {"independent_qa": qa, "transport_protocol": {"status": "PASS", "receipt_count": 1944, "camera_alignment": {"status": "PASS"}},
        "initialization_protocol": {"status": "PASS", "raw_state_count": 1939, "measurements": {"construction_frame": 100}},
        "continuation_observations": {"established_continuation_stop_condition": False, "minimum_recorded_vx_mps": -.001,
            "reverse_control_observation_count": 0, "manual_gear_control_observation_count": 0}}
    first, second = audit.expected_cases()[:2]
    review = {"schema": "portable_e2e.turn_launch_case_continuation_review.v1", "completed_case_id": first["case_id"],
        "completed_case_sequence": 1, "next_case_id": second["case_id"], "campaign_plan_sha256": audit.ORIGINAL_PLAN_SHA,
        "exact_eleven_source_archive_postcheck": True, "owned_lifecycle": True, "ack_status": "PASS", "initialization_status": "PASS",
        "camera_alignment_status": "PASS", "native_states": 1939, "camera_anchors": 970, "ack_receipts": 1944,
        "scalar_failed_flags": ["native_decoder_speed_rate_clear"], "maximum_native_acceleration_mps2": 3.5,
        "minimum_native_acceleration_mps2": -1.6, "maximum_camera_acceleration_mps2": 2.4, "maximum_speed_kmh": 14.4,
        "goal_error_m": .95, "initialization_measurements": {"construction_frame": 100},
        "full_matrix_governor_audit_complete": False, "jpeg_payload_sha_audit_complete": False,
        "training_data_approved": False, "automatic_retry": False, "comment": "HH_260906 - Synthetic review.",
        "reviewed_at_utc": "2026-09-07T20:51:00Z", "review_source_sha256": copy.deepcopy(audit.REVIEW_SOURCES),
        "raw_pins": {n: audit.base.sha(root / n) for n in names}, "next_predeclared_case_may_start": True,
        "reason": "Retain the scalar failure while inspecting the next fixed case, not a favourable retry."}
    def source(pins):
        audit.require(pins == audit.REVIEW_SOURCES, "unreviewed source pins")
        return {"synthetic": True}
    monkeypatch.setattr(audit, "resolve_review_sources", source)
    return review, previous, first, second, root, {"planned_at_utc": "2026-09-07T20:52:00Z"}, audit.ORIGINAL_PLAN_SHA


def test_intervening_review_preserves_scalar_failure_and_preliminary_scope(review_fixture):
    result = audit.validate_review(*review_fixture)
    assert result["independent_measurement_binding"] == "PASS" and not result["historical_review_was_full_matrix_audit"]


@pytest.mark.parametrize("key,value", [("next_case_id", "08_turn_launch_015_v1_r2"), ("completed_case_sequence", 2),
    ("campaign_plan_sha256", "b" * 64), ("native_states", 1938), ("camera_anchors", 971), ("ack_receipts", 1943),
    ("scalar_failed_flags", []), ("maximum_native_acceleration_mps2", 2.9), ("goal_error_m", 0),
    ("initialization_status", "FAIL"), ("training_data_approved", True), ("automatic_retry", True),
    ("full_matrix_governor_audit_complete", True), ("jpeg_payload_sha_audit_complete", True),
    ("next_predeclared_case_may_start", False), ("reviewed_at_utc", "2026-09-07T20:49:00Z"),
    ("reviewed_at_utc", "2026-09-07T20:53:00Z")])
def test_review_cannot_hide_failure_omission_or_retroactive_time(review_fixture, key, value):
    review_fixture[0][key] = value
    with pytest.raises(audit.base.EvidenceError): audit.validate_review(*review_fixture)


@pytest.mark.parametrize("mutation", ["raw_sha", "source_sha", "stop_condition", "resume_sha", "extra", "reverse_value"])
def test_review_raw_and_source_stop_resume_bindings(review_fixture, mutation):
    review, previous, *_ = review_fixture
    if mutation == "raw_sha": review["raw_pins"]["episode.partial/states.jsonl"] = "d" * 64
    elif mutation == "source_sha": review["review_source_sha256"]["scripts/e2e/audit_carla_agent_initialization.py"] = "d" * 64
    elif mutation == "stop_condition": previous["continuation_observations"]["established_continuation_stop_condition"] = True
    elif mutation == "resume_sha": review["resume_authorization_sha256"] = "d" * 64
    elif mutation == "extra": review["new_limit"] = 9
    elif mutation == "reverse_value": review["reverse_diagnostic"] = {"minimum_native_vx_mps": 0., "reverse_control_count": 0,
        "manual_gear_control_count": 0, "notice": "Incorrectly hidden negative velocity."}
    with pytest.raises(audit.base.EvidenceError): audit.validate_review(*review_fixture)


def test_partial_campaign_keeps_all_eight_and_never_claims_completion(tmp_path, monkeypatch):
    campaign = tmp_path / "campaign"
    campaign.mkdir()
    plan = {"cases": audit.expected_cases(), "synthetic": True}
    path = campaign / "pilot_plan.json"
    path.write_text(json.dumps(plan))
    monkeypatch.setattr(audit, "ORIGINAL_PLAN_SHA", audit.base.sha(path))
    monkeypatch.setattr(audit, "validate_plan_document", lambda _: {})
    report, audits = audit.audit_campaign(campaign)
    assert report["status"] == "INCOMPLETE" and report["not_run_cases"] == 8 and len(report["cases"]) == 8 and not audits
    first = campaign / audit.expected_cases()[0]["output"]
    first.mkdir(parents=True)
    report, audits = audit.audit_campaign(campaign)
    assert report["cases"][0]["status"] == "INCOMPLETE" and report["not_run_cases"] == 7 and not report["dataset_admission"]


@pytest.mark.parametrize("mutation", ["gap", "extra_attempt", "unknown_profile", "symlink"])
def test_campaign_never_drops_unknown_or_out_of_order_attempts(tmp_path, monkeypatch, mutation):
    campaign = tmp_path / "campaign"
    campaign.mkdir()
    path = campaign / "pilot_plan.json"
    path.write_text('{}')
    monkeypatch.setattr(audit, "ORIGINAL_PLAN_SHA", audit.base.sha(path))
    monkeypatch.setattr(audit, "validate_plan_document", lambda _: {})
    destination = campaign / (audit.expected_cases()[1]["output"] if mutation == "gap" else
        "c_track_left/turn_launch_015_v1/run_003" if mutation == "extra_attempt" else
        "c_track_left/turn_launch_011_v1/run_001" if mutation == "unknown_profile" else audit.expected_cases()[0]["output"])
    destination.parent.mkdir(parents=True)
    if mutation == "symlink": destination.symlink_to(tmp_path, target_is_directory=True)
    else: destination.mkdir()
    with pytest.raises(audit.base.EvidenceError): audit.audit_campaign(campaign)


@pytest.mark.parametrize("format,mode,size", [("PNG", "RGB", (640, 360)), ("JPEG", "L", (640, 360)),
    ("JPEG", "RGB", (320, 180)), ("JPEG", "RGB", (640, 359))])
def test_jpeg_rig_decode_rejects_substitution(format, mode, size):
    from PIL import Image
    raw = io.BytesIO()
    Image.new(mode, size).save(raw, format=format)
    with pytest.raises(audit.base.EvidenceError): audit.validate_jpeg_payload(raw.getvalue())


def test_jpeg_integrity_preserves_bytes_and_rejects_truncation():
    from PIL import Image
    raw = io.BytesIO()
    Image.new("RGB", (640, 360), (5, 9, 12)).save(raw, format="JPEG")
    value = raw.getvalue()
    before = hashlib.sha256(value).hexdigest()
    audit.validate_jpeg_payload(value)
    assert hashlib.sha256(value).hexdigest() == before
    with pytest.raises((OSError, audit.base.EvidenceError)): audit.validate_jpeg_payload(value[:len(value) // 2])


@pytest.fixture
def prospective_plan(monkeypatch):
    # HH_260906 - Synthetic sources exercise the fixed prospective grammar before any private-history dependency is accessed.
    raw = b"synthetic frozen source"
    plan = copy.deepcopy(audit.PLAN_FIXED)
    flags = capture_flags("turn_launch_015_v1")
    index = flags.index("--goal-stop-profile")
    common = flags[:index] + flags[index + 2:]
    plan.update(comment="HH_260906 - Synthetic matrix plan.", declared_at_utc="2026-09-07T20:49:00Z",
        source_hashes={name: hashlib.sha256(raw).hexdigest() for name in audit.SOURCES}, common_capture_flags=common,
        historical_lateral_configuration={}, comparison_notice="All cases retained.", expected_risks=["No admission."] * 3,
        historical_before_bootstrap_reference={}, initialization_contract=copy.deepcopy(audit.initialization_audit.INIT_CONTRACT),
        review_measurements=["Inspect all rows."] * 5, continuation_policy=copy.deepcopy(audit.CONTINUATION_POLICY),
        initialization_reference={}, cases=[{**c, "capture_flags": common + ["--goal-stop-profile", c["profile"]],
            "frozen_configuration": audit.frozen_configuration(c["profile"])} for c in audit.expected_cases()])
    monkeypatch.setattr(audit, "recorded_bytes", lambda *_: raw)
    return plan


@pytest.mark.parametrize("mutation", ["reorder", "missing_case", "extra_case", "extra_field", "source", "lateral", "pedal",
    "ramp_origin", "ramp_slew", "deadline", "automatic_retry", "normal_brake", "physics", "training", "route",
    "new_init", "max_replicates", "full_route", "timing", "unknown_cli"])
def test_matrix_plan_cannot_change_order_source_control_or_scope(prospective_plan, mutation):
    plan = prospective_plan
    if mutation == "reorder": plan["cases"][0], plan["cases"][1] = plan["cases"][1], plan["cases"][0]
    elif mutation == "missing_case": plan["cases"].pop()
    elif mutation == "extra_case": plan["cases"].append(copy.deepcopy(plan["cases"][0]))
    elif mutation == "extra_field": plan["new_limit"] = 9
    elif mutation == "source": plan["source_hashes"]["scripts/e2e/collect_carla_vad_expert.py"] = "c" * 64
    elif mutation == "lateral": plan["cases"][0]["capture_flags"][plan["cases"][0]["capture_flags"].index("--basic-agent-distance-ratio") + 1] = ".5"
    elif mutation == "pedal": plan["cases"][1]["frozen_configuration"]["launch_throttle"] = .15
    elif mutation == "ramp_origin": plan["cases"][1]["frozen_configuration"]["post_handoff_initial_throttle"] = .13
    elif mutation == "ramp_slew": plan["post_handoff_ramp_per_second"] = .1
    elif mutation == "deadline": plan["finish_before_utc"] = audit.RESUMED_DEADLINE
    elif mutation == "automatic_retry": plan["automatic_retry"] = True
    elif mutation == "normal_brake": plan["normal_brake_cap"] = .01
    elif mutation == "physics": plan["physics_hz"] = 10
    elif mutation == "training": plan["training_data_approved"] = True
    elif mutation == "route": plan["route_sha256"] = "d" * 64
    elif mutation == "new_init": plan["initialization_contract"]["pid_history_overwritten"] = True
    elif mutation == "max_replicates": plan["maximum_attempts_per_revision"] = 3
    elif mutation == "full_route": plan["launch_only_eight_second_probe"] = True
    elif mutation == "timing": plan["wall_timing"] = False
    elif mutation == "unknown_cli": plan["common_capture_flags"] += ["--allow-map-load"]
    with pytest.raises(audit.base.EvidenceError): audit.validate_plan_document(plan)


def test_auditor_sources_are_rechecked_during_campaign(tmp_path, monkeypatch):
    campaign = tmp_path / "campaign"
    campaign.mkdir()
    path = campaign / "pilot_plan.json"
    path.write_text('{}')
    monkeypatch.setattr(audit, "ORIGINAL_PLAN_SHA", audit.base.sha(path))
    original = audit.base.sha
    marker = {"changed": False}
    def history(_):
        marker["changed"] = True
        return {}
    monkeypatch.setattr(audit, "validate_plan_document", history)
    monkeypatch.setattr(audit.base, "sha", lambda p: "f" * 64 if marker["changed"] and Path(p) == Path(audit.__file__) else original(p))
    with pytest.raises(audit.base.EvidenceError, match="auditor implementation changed"): audit.audit_campaign(campaign)


@pytest.mark.parametrize("mutation", ["archive_bytes", "extra_source", "false_postcheck", "log_prefix"])
def test_archive_and_log_hash_evidence_cannot_be_substituted(owned_trial, mutation):
    if mutation == "archive_bytes":
        path = owned_trial / "provenance/scripts/e2e/carla_goal_stop_profile.py"
        path.write_bytes(path.read_bytes() + b"modified")
    elif mutation == "extra_source":
        (owned_trial / "provenance/foreign.py").write_bytes(b"foreign")
    elif mutation == "false_postcheck":
        path = owned_trial / "owner_result.json"
        data = json.loads(path.read_text())
        data["source_checks"]["scripts/e2e/carla_wall_timing.py"] = False
        path.write_text(json.dumps(data))
    else:
        (owned_trial / "server.log").write_bytes(b"changed")
    with pytest.raises(audit.base.EvidenceError): audit.verify_owner_sources(owned_trial, {})
