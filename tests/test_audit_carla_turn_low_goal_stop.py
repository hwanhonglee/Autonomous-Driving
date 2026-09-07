"""HH_260906 - Test the explicitly frozen C-track low-speed audit without simulator, GPU or raw-data mutation."""

import copy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts.e2e import audit_carla_turn_low_goal_stop as audit
from scripts.e2e.carla_goal_stop_profile import (
    TurnLowDevelopmentGoalStopConfig, DevelopmentGoalStopGovernor,
    annotate_development_control, goal_stop_termination_reason,
)

def protocol_fixture(emergency_tick=None):
    # HH_260906 - Synthetic kinematics exercise record contracts only, never physical data qualification.
    governor = DevelopmentGoalStopGovernor(TurnLowDevelopmentGoalStopConfig(), 4.0, 20.0)
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
            governor.fail("turn_low_v1_emergency_override")
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


def test_literal_turn_contract_does_not_override_any_historical_v4_pin():
    old_sources, old_config = copy.deepcopy(audit.v4.REVIEWED_SOURCES), copy.deepcopy(audit.v4.FROZEN)
    assert audit.FROZEN == asdict(TurnLowDevelopmentGoalStopConfig())
    states, rows = protocol_fixture()
    original = copy.deepcopy((states, rows))
    report = audit.analyze_protocol(states, rows)
    assert report["all_checks_pass"] and report["longest_continuous_cruise_seconds"] >= 5
    assert (states, rows) == original and audit.v4.REVIEWED_SOURCES == old_sources and audit.v4.FROZEN == old_config


@pytest.mark.parametrize("index,key,value", [(20, "brake", 1e-12), (20, "brake", .1), (0, "throttle", .2),
    (20, "throttle", .41), (112, "throttle", .01), (120, "brake", .001), (155, "steer", .1)])
def test_normal_brake_zero_and_setup_tail_bounds_unchanged(index, key, value):
    states, rows = protocol_fixture()
    rows[index]["next_control"][key] = value
    assert "normal_pedal_protocol" in audit.analyze_protocol(states, rows)["failed_flags"]


@pytest.mark.parametrize("key,value", [("normal_brake_cap", .1), ("hold_ticks", 999), ("target_speed_mps", 8),
    ("pilot_state", "complete"), ("training_data_approved", 0), ("next_control_starts_driving", True)])
def test_forged_low_speed_row_metadata_never_passes(key, value):
    states, rows = protocol_fixture()
    states[20]["goal_stop"][key] = value
    assert not audit.analyze_protocol(states, rows)["all_checks_pass"]


def test_strict_actual_cap_is_not_the_old_30_kph_cap():
    states, rows = protocol_fixture()
    states[20]["vx"], rows[20]["speed_mps"] = 4.30000000001, 4.30000000001
    report = audit.analyze_protocol(states, rows)
    assert not report["flags"]["actual_planar_and_vx_maximum"]
    assert report["speed_violations"][0]["frame"] == rows[20]["frame"]


def test_zero_or_fragmented_cruise_cannot_be_forged_by_governor_timer():
    states, rows = protocol_fixture()
    rows[60]["speed_mps"], states[60]["vx"] = 3.79, 3.79
    states[60]["goal_stop"]["longest_continuous_cruise_seconds"] = 1000
    assert not audit.analyze_protocol(states, rows)["flags"]["continuous_cruise"]


def test_emergency_failed_partial_retains_original_emergency_control_and_no_tail():
    states, rows = protocol_fixture(emergency_tick=120)
    result = audit.analyze_protocol(states, rows)
    assert result["emergency_override_count"] == 1 and result["latched_failure"] == "turn_low_v1_emergency_override"
    assert result["flags"]["emergency_observation_before_abort"]
    assert all(row["phase"] != "stationary_tail" for row in rows)
    rows[-1]["next_control"]["brake"] = 0
    assert not audit.analyze_protocol(states, rows)["flags"]["normal_pedal_protocol"]


def test_empty_trial_is_failed_not_stability_or_admission():
    result = audit.analyze_protocol([], [])
    assert not result["all_checks_pass"] and not result["flags"]["continuous_cruise"]
    assert audit.ack.analyze_transport([], [], None)["status"] == "FAIL"


def test_only_reviewed_source_paths_can_be_requested_without_fetch(monkeypatch):
    with pytest.raises(audit.base.EvidenceError): audit.recorded_bytes("main", "scripts/e2e/carla_wall_timing.py")
    with pytest.raises(audit.base.EvidenceError): audit.recorded_bytes(audit.COMMIT, "../private")
    def fake_run(argv, **kwargs):
        assert kwargs["env"]["GIT_NO_LAZY_FETCH"] == "1" and kwargs["env"]["GIT_ALLOW_PROTOCOL"] == ""
        assert argv[1:3] == ["-c", "protocol.allow=never"]
        return SimpleNamespace(returncode=1)
    monkeypatch.setattr(audit.subprocess, "run", fake_run)
    with pytest.raises(audit.base.EvidenceError, match="no fetch"):
        audit.recorded_bytes(audit.COMMIT, "scripts/e2e/carla_wall_timing.py")


def capture_flags():
    # HH_260906 - Exercise exact option names, duplication and new route-specific values without loading a client.
    options = {"--physics-hz": "20", "--capture-hz": "10", "--target-speed-kmh": "14.4", "--max-duration-sec": "180",
        "--stationary-warmup-sec": "3.5", "--stationary-tail-sec": "6.5", "--spawn-z-offset-m": "0.5", "--weather": "ClearNoon",
        "--seed": "0", "--goal-stop-profile": "turn_low_v1", "--goal-tolerance-m": "1.0",
        "--mapping": "autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml",
        "--calibration": "src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml",
        "--basic-agent-base-min-distance-m": "2.0", "--basic-agent-distance-ratio": "0.2", "--basic-agent-lateral-kp": "1.95",
        "--basic-agent-lateral-ki": "0.05", "--basic-agent-lateral-kd": "0.2", "--basic-agent-max-steering": "0.8",
        "--basic-agent-lane-offset-m": "0.0", "--control-transport": "acknowledged_batch"}
    return [value for pair in options.items() for value in pair] + ["--wall-timing"]


@pytest.fixture
def prospective(monkeypatch):
    raw = b"synthetic reviewed source, not executed"
    plan = copy.deepcopy(audit.PLAN_FIXED)
    control = {"effective_opt_dict": {"base_min_distance": 2., "distance_ratio": .2},
               "waypoint_purge_lookahead": {"base_min_distance_m": 2., "distance_ratio_s": .2}}
    plan.update(comment="HH_260906 - Synthetic prospective fixture.", declared_at_utc="2026-09-07T20:00:00Z",
        source_hashes={name: hashlib.sha256(raw).hexdigest() for name in audit.SOURCES}, capture_flags=capture_flags(),
        historical_lateral_configuration=copy.deepcopy(control), comparison_notice="Separate low-speed development, not identical between-map controls.",
        expected_risks=["Terrain may change coast distance.", "No actual lateral approval from geometry.", "Do not translate native coordinates."])
    monkeypatch.setattr(audit, "recorded_bytes", lambda *_: raw)
    def historical(_root, relative, ledger):
        ledger[relative] = {"sha256": plan["historical_lateral_baseline_sha256"], "size_bytes": 99}
        return {"capture_contract": {"basic_agent_control": copy.deepcopy(control)}}
    monkeypatch.setattr(audit.pilot, "read_json", historical)
    return plan


def test_exact_prospective_low_turn_contract(prospective):
    assert audit.validate_plan_document(prospective)


@pytest.mark.parametrize("key,value", [("source_commit", "f" * 40), ("profile", "comfortable_v4"), ("physics_hz", True),
    ("nominal_target_speed_kmh", 30), ("maximum_actual_speed_mps", 8.333333), ("quality", "Low"),
    ("normal_brake_cap", .01), ("maximum_attempts_per_revision", 3), ("review_required_before_second_attempt", False),
    ("native_carla_z_translation_applied_m", -15), ("qualification_30_kph", "PASS"), ("training_data_approved", True),
    ("declared_at_utc", "2026-09-08T02:00:00Z")])
def test_prospective_scope_and_bounds_cannot_be_relaxed(prospective, key, value):
    prospective[key] = value
    with pytest.raises(audit.base.EvidenceError): audit.validate_plan_document(prospective)


@pytest.mark.parametrize("mutation", ["missing", "extra", "sources", "lateral", "old_cli", "duplicate", "abbreviation", "timing", "route"])
def test_prospective_exact_fields_sources_and_cli_fail_closed(prospective, mutation):
    if mutation == "missing": prospective.pop("expected_risks")
    elif mutation == "extra": prospective["relax_limits"] = True
    elif mutation == "sources": prospective["source_hashes"]["scripts/e2e/carla_goal_stop_profile.py"] = "0" * 64
    elif mutation == "lateral": prospective["historical_lateral_configuration"]["effective_opt_dict"]["base_min_distance"] = 3.
    elif mutation == "old_cli": prospective["capture_flags"][prospective["capture_flags"].index("--basic-agent-distance-ratio") + 1] = ".5"
    elif mutation == "duplicate": prospective["capture_flags"] += ["--target-speed-kmh", "14.4"]
    elif mutation == "abbreviation": prospective["capture_flags"][0] = "--physics-h"
    elif mutation == "timing": prospective["capture_flags"].remove("--wall-timing")
    else: prospective["route_sha256"] = audit.v4.ROUTE_SHA
    with pytest.raises(audit.base.EvidenceError): audit.validate_plan_document(prospective)


def test_existing_and_nested_raw_output_are_refused(tmp_path):
    raw = tmp_path / "raw"
    raw.mkdir()
    for output in (raw, raw / "new"):
        with pytest.raises(audit.base.EvidenceError): audit.v4.new_output(output, [raw])
    link = tmp_path / "linked"
    link.symlink_to(raw, target_is_directory=True)
    with pytest.raises(audit.base.EvidenceError): audit.v4.new_output(link / "new", [raw])


def second_review_fixture():
    first = {"source_manifest": [{"path": "episode.partial/states.jsonl", "sha256": "a" * 64}],
        "transport_protocol": {"status": "PASS"}, "independent_qa": {"raw_scalar_quality_clear": False},
        "pilot_protocol": {"all_checks_pass": True}}
    owner = {"exit_code": 1, "completed_at_utc": "2026-09-07T20:10:00Z"}
    second = {"planned_at_utc": "2026-09-07T20:12:00Z"}
    review = {"schema": "portable_e2e.turn_low_second_review.v1", "source_commit": audit.COMMIT,
        "previous_output": "c_track_left/run_001", "next_output": "c_track_left/run_002",
        "previous_states_sha256": "a" * 64, "previous_owner_exit_code": 1, "previous_transport_status": "PASS",
        "previous_native_scalar_quality_clear": False, "previous_pilot_protocol_all_checks_pass": True,
        "source_and_parameters_changed": False, "automatic_retry": False, "training_data_approved": False,
        "qualification_30_kph": "NOT_CLAIMED", "reviewed_at_utc": "2026-09-07T20:11:00Z",
        "comment": "HH_260906 - Synthetic review.", "reason": "Review records failure, never a best-run filter."}
    return review, first, Path("first"), Path("second"), owner, second


def test_optional_second_review_preserves_first_failure():
    audit.validate_second_review(*second_review_fixture())


@pytest.mark.parametrize("key,value", [("previous_native_scalar_quality_clear", True), ("previous_owner_exit_code", 0),
    ("previous_states_sha256", "b" * 64), ("automatic_retry", True), ("source_and_parameters_changed", True),
    ("reviewed_at_utc", "2026-09-07T20:13:00Z"), ("reviewed_at_utc", "2026-09-07T20:09:00Z")])
def test_second_review_cannot_relabel_or_bypass_intervening_review(key, value):
    args = second_review_fixture()
    args[0][key] = value
    with pytest.raises(audit.base.EvidenceError): audit.validate_second_review(*args)


@pytest.fixture
def owned_trial(tmp_path, monkeypatch):
    # HH_260906 - Create fake source/lifecycle evidence only; no worker, server or actual published file is touched.
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
        "planned_at_utc": "2026-09-07T20:00:00Z", "collector_argv": [str(root / "episode"), "route.json", "--host", "127.0.0.1", "--port", "2100", *capture_flags()]}
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


def test_existing_publication_refused_before_reading_or_changing_input(tmp_path, monkeypatch):
    output = tmp_path / "published"
    output.mkdir()
    original = output / "keep.txt"
    original.write_bytes(b"existing user bytes")
    monkeypatch.setattr(audit, "audit_campaign", lambda *_: pytest.fail("existing category must fail before raw audit"))
    with pytest.raises(audit.base.EvidenceError): audit.publish(tmp_path / "raw", output, tmp_path / "visuals")
    assert original.read_bytes() == b"existing user bytes"


def test_auditor_source_has_no_live_worker_or_model_import():
    source = Path(audit.__file__).read_text()
    assert "import carla\n" not in source and "import torch" not in source
    assert "from .carla_goal_stop_profile import" not in source and "import collect_carla_vad_expert" not in source


def test_exact_byte_publication_source_pin_is_not_a_forged_governor_pass():
    assert "62b2d3d0a967a38810424071492d8643a5b66f3b38578531bdb446572ad3502f" in Path(audit.__file__).read_text()
    assert "4a7c7c89f79bd436e0d593a24a453669a1b37be5f83d9e950c5ed883f3a52e00" in Path(audit.__file__).read_text()
