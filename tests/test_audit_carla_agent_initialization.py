"""HH_260906 - Independently test initialization witnesses without CARLA, remote jobs, or published-file mutations."""

import copy
import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import audit_carla_agent_initialization as audit
from tests.test_audit_carla_turn_low_goal_stop import capture_flags, second_review_fixture


def control(**kwargs):
    return {"throttle": 0., "brake": 1., "steer": 0., "hand_brake": False, "reverse": False,
            "gear": 0, "manual_gear_shift": False, **kwargs}


def init_fixture():
    # HH_260906 - Synthetic frame records exercise provenance, not physical feasibility or measured performance.
    pose = dict(x=1., y=2., z=3., roll=0., pitch=0., yaw=0.)
    br = {"actor_id": 10, "response_actor_id": 10, "sequence": 1, "status": "ACKNOWLEDGED", "server_accepted": True,
          "do_tick": False, "before_frame": 99, "after_ack_frame": 99, "reason": "pre_capture_bootstrap"}
    bootstrap = {"frame": 100, "timestamp": 5., "actor_snapshot_transform_carla": pose,
                 "current_control": control(), "alignment": {"status": "PASS", "expected_receipt_sequence": 1}}
    proposed = control(throttle=.15, brake=0., steer=-.001)
    states = [{"frame": 101 + i, "timestamp": 5.05 + i * .05, "capture_phase": "stationary_warmup",
               "actor_snapshot_transform_carla": copy.deepcopy(pose), "current_control": control(),
               "world_velocity_carla": [0., 0., 0.], "next_control": control(), "goal_stop": {}, "control_transport": {}}
              for i in range(70)]
    states[-1].update(next_control=copy.deepcopy(proposed), goal_stop={"next_control_starts_driving": True},
                      control_transport={"next_command_receipt_sequence": 73})
    states.append({"frame": 171, "timestamp": 8.55, "capture_phase": "driving", "current_control": copy.deepcopy(proposed),
                   "control_transport": {"expected_receipt_sequence": 73, "status": "PASS"}})
    def witness(reference, pid=False):
        result = {"status": "PASS", "expected_frame": reference["frame"], "frame_before": reference["frame"],
            "frame_after": reference["frame"], "timestamp": reference["timestamp"], "actor_id": 10,
            "actor_snapshot_transform_carla": copy.deepcopy(reference["actor_snapshot_transform_carla"]),
            "current_control": copy.deepcopy(reference["current_control"]),
            "raw_state_reference": "CARLA actor snapshot API reference point; physical COM identity is unverified."}
        if pid: result["pid_past_steering"] = 0.
        return result
    metadata = {**audit.INIT_CONTRACT, "mode": "after_bootstrap", "required_control_transport": "acknowledged_batch",
        "construction": {"status": "PASS", "before": witness(bootstrap), "after": witness(bootstrap, True)},
        "engagement": {"status": "PASS", "first_command_acknowledged": True, "before": witness(states[69], True),
            "first_proposed_control": copy.deepcopy(proposed), "frame_after_agent_step": 170,
            "first_proposed_steering_delta": -.001, "first_sent_control": copy.deepcopy(proposed),
            "first_command_receipt_sequence": 73}}
    receipt = {"sequence": 73, "status": "ACKNOWLEDGED", "server_accepted": True, "actor_id": 10,
        "response_actor_id": 10, "expected_before_frame": 170, "before_frame": 170, "after_ack_frame": 170,
        "do_tick": False, "physical_actuation_proven": False, "requested_control": copy.deepcopy(proposed),
        "reason": "initial_drive_control"}
    return metadata, states, [br, receipt], bootstrap


def test_exact_frame_bound_chain_and_no_historical_mutation():
    args = init_fixture()
    original = copy.deepcopy(args)
    old = copy.deepcopy((audit.turn.REVIEWED_SOURCES, audit.turn.PLAN_FIXED))
    result = audit.analyze_initialization(*args)
    assert result["status"] == "PASS" and result["measurements"]["first_observed_frame"] == 171
    assert result["training_data_approved"] is result["physical_cause_proven"] is False
    assert args == original and (audit.turn.REVIEWED_SOURCES, audit.turn.PLAN_FIXED) == old


@pytest.mark.parametrize("stage,key,value", [("before", "frame_before", 99), ("after", "frame_after", 101),
    ("after", "timestamp", 5.01), ("before", "actor_id", 11), ("after", "pid_past_steering", -.8),
    ("after", "pid_past_steering", False), ("after", "pid_past_steering", float("nan")),
    ("before", "raw_state_reference", "Physical COM is proven."), ("after", "expected_frame", True)])
def test_constructor_frame_pose_pid_and_reference_cannot_be_forged(stage, key, value):
    args = init_fixture()
    args[0]["construction"][stage][key] = value
    assert audit.analyze_initialization(*args)["status"] == "FAIL"


@pytest.mark.parametrize("mutation", ["pose", "brake", "nonzero_steer", "bootstrap_failed", "bootstrap_receipt_actor",
    "bootstrap_receipt_tick", "bootstrap_duplicate", "extra_tick", "missing_warmup", "noncontiguous_warmup",
    "engagement_pid", "engagement_pose", "agent_extra_tick", "delta", "nonfinite_delta", "false_ack",
    "receipt_sequence", "receipt_actor", "receipt_frame", "receipt_control", "receipt_duplicate", "receipt_failure",
    "receipt_physical_claim", "warmup_next", "warmup_receipt", "first_observed_frame", "first_observed_control",
    "first_observed_receipt", "missing_first_observation", "unexplained_sent_change", "proposed_step_over_limit"])
def test_no_initialization_false_pass_on_broken_chain(mutation):
    m, states, receipts, bootstrap = args = init_fixture()
    e = m["engagement"]
    if mutation == "pose": m["construction"]["after"]["actor_snapshot_transform_carla"]["z"] += .01
    elif mutation == "brake": m["construction"]["before"]["current_control"]["brake"] = .9
    elif mutation == "nonzero_steer": m["construction"]["after"]["current_control"]["steer"] = .00001
    elif mutation == "bootstrap_failed": bootstrap["alignment"]["status"] = "FAIL"
    elif mutation == "bootstrap_receipt_actor": receipts[0]["response_actor_id"] = 11
    elif mutation == "bootstrap_receipt_tick": receipts[0]["do_tick"] = True
    elif mutation == "bootstrap_duplicate": receipts.append(copy.deepcopy(receipts[0]))
    elif mutation == "extra_tick": states[0]["frame"] += 1
    elif mutation == "missing_warmup": states.pop(0)
    elif mutation == "noncontiguous_warmup": states[0], states[1] = states[1], states[0]
    elif mutation == "engagement_pid": e["before"]["pid_past_steering"] = .2
    elif mutation == "engagement_pose": e["before"]["actor_snapshot_transform_carla"]["yaw"] = 1.
    elif mutation == "agent_extra_tick": e["frame_after_agent_step"] += 1
    elif mutation == "delta": e["first_proposed_steering_delta"] = 0.
    elif mutation == "nonfinite_delta": e["first_proposed_steering_delta"] = float("nan")
    elif mutation == "false_ack": e["first_command_acknowledged"] = 1
    elif mutation == "receipt_sequence": e["first_command_receipt_sequence"] = 72
    elif mutation == "receipt_actor": receipts[1]["actor_id"] = 11
    elif mutation == "receipt_frame": receipts[1]["after_ack_frame"] += 1
    elif mutation == "receipt_control": receipts[1]["requested_control"]["throttle"] = .3
    elif mutation == "receipt_duplicate": receipts.append(copy.deepcopy(receipts[1]))
    elif mutation == "receipt_failure": receipts[1]["status"] = "FAILED"
    elif mutation == "receipt_physical_claim": receipts[1]["physical_actuation_proven"] = True
    elif mutation == "warmup_next": states[69]["next_control"]["steer"] = .1
    elif mutation == "warmup_receipt": states[69]["control_transport"]["next_command_receipt_sequence"] = 74
    elif mutation == "first_observed_frame": states[70]["frame"] += 1
    elif mutation == "first_observed_control": states[70]["current_control"]["brake"] = 1.
    elif mutation == "first_observed_receipt": states[70]["control_transport"]["expected_receipt_sequence"] = 72
    elif mutation == "missing_first_observation": states.pop()
    elif mutation == "unexplained_sent_change": e["first_sent_control"]["steer"] = 0.
    elif mutation == "proposed_step_over_limit":
        e["first_proposed_control"]["steer"] = e["first_proposed_steering_delta"] = .100002
    assert audit.analyze_initialization(*args)["status"] == "FAIL"


def test_unchanged_stopped_brake_guard_is_explained_not_arbitrary_rewrite():
    m, states, receipts, bootstrap = args = init_fixture()
    e = m["engagement"]
    e["first_proposed_control"].update(throttle=0., brake=1.)
    sent = {**e["first_proposed_control"], "steer": 0.}
    e["first_sent_control"] = copy.deepcopy(sent)
    states[69]["next_control"] = copy.deepcopy(sent)
    states[70]["current_control"] = copy.deepcopy(sent)
    receipts[1]["requested_control"] = copy.deepcopy(sent)
    assert audit.analyze_initialization(*args)["status"] == "PASS"
    states[69]["goal_stop"]["next_control_is_unchanged_emergency_override"] = True
    assert audit.analyze_initialization(*args)["status"] == "FAIL"


@pytest.mark.parametrize("stage,status", [("construction", "FAIL"), ("construction", "NOT_REACHED"),
    ("engagement", "FAIL"), ("engagement", "VALIDATED_NOT_SENT"), ("engagement", "NOT_REACHED")])
def test_failed_and_no_observation_initialization_is_retained(stage, status):
    m, states, receipts, bootstrap = init_fixture()
    m[stage]["status"] = status
    result = audit.analyze_initialization(m, [], receipts[:1], bootstrap)
    assert result["status"] == "FAIL" and result["all_observations_retained"]
    assert result["original_" + stage + "_witness"]["status"] == status
    assert not result["training_data_approved"]


@pytest.mark.parametrize("key,value", [("additional_world_ticks", 1), ("additional_world_ticks", False),
    ("pid_history_overwritten", True), ("mode", "before_bootstrap"), ("required_control_transport", "async"),
    ("first_proposed_steering_delta_limit", .2), ("physical_cause_proven", True), ("training_data_approved", True)])
def test_initialization_abi_cannot_be_relaxed(key, value):
    args = init_fixture()
    args[0][key] = value
    with pytest.raises(audit.base.EvidenceError): audit.analyze_initialization(*args)


@pytest.mark.parametrize("extra", [[], ["--agent-initialization", "before_bootstrap"], ["--agent-init", "after_bootstrap"],
    ["--agent-initialization", "after_bootstrap", "--agent-initialization", "after_bootstrap"],
    ["--agent-initialization", "after_bootstrap", "--allow-map-load"]])
def test_explicit_new_cli_and_no_abbreviation_or_unknown_flags(tmp_path, extra):
    owner = {"port": 2100, "route_path": "route.json", "collector_argv": [str(tmp_path / "episode"), "route.json",
        "--host", "127.0.0.1", "--port", "2100", *capture_flags(), *extra]}
    with pytest.raises(audit.base.EvidenceError): audit.validate_cli(owner, tmp_path)


def test_exact_new_cli_leaves_old_reference_and_argv_unchanged(tmp_path):
    owner = {"port": 2100, "route_path": "route.json", "collector_argv": [str(tmp_path / "episode"), "route.json",
        "--host", "127.0.0.1", "--port", "2100", *capture_flags(), "--agent-initialization", "after_bootstrap"]}
    original = copy.deepcopy(owner)
    audit.validate_cli(owner, tmp_path)
    assert owner == original and audit.COMMIT != audit.turn.COMMIT
    assert audit.REVIEWED_SOURCES["scripts/e2e/collect_carla_vad_expert.py"] != audit.turn.REVIEWED_SOURCES["scripts/e2e/collect_carla_vad_expert.py"]


def test_optional_review_must_bind_initialization_failure_and_new_revision():
    args = second_review_fixture()
    args[1]["initialization_protocol"] = {"status": "FAIL"}
    args[0].update(schema="portable_e2e.agent_initialization_second_review.v1", source_commit=audit.COMMIT,
                   previous_initialization_status="FAIL")
    audit.validate_second_review(*args)
    args[0]["previous_initialization_status"] = "PASS"
    with pytest.raises(audit.base.EvidenceError): audit.validate_second_review(*args)


def test_create_only_output_preflight_precedes_any_raw_audit(tmp_path, monkeypatch):
    output = tmp_path / "existing"
    output.mkdir()
    (output / "preserve").write_bytes(b"existing bytes")
    monkeypatch.setattr(audit, "audit_campaign", lambda *_: pytest.fail("existing output must be refused first"))
    with pytest.raises(audit.base.EvidenceError): audit.main([str(tmp_path / "raw"), "--output-dir", str(output)])
    assert (output / "preserve").read_bytes() == b"existing bytes"


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
        "planned_at_utc": "2026-09-07T20:00:00Z", "collector_argv": [str(root / "episode"), "route.json", "--host", "127.0.0.1", "--port", "2100", *capture_flags(), "--agent-initialization", "after_bootstrap"]}
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


@pytest.fixture
def prospective(monkeypatch):
    # HH_260906 - Synthetic plan dependencies make the adversarial tests independent of private raw-data availability.
    raw = b"synthetic archived bytes, not actual execution evidence"
    digest = hashlib.sha256(raw).hexdigest()
    control = {"effective_opt_dict": {"base_min_distance": 2., "distance_ratio": .2},
               "waypoint_purge_lookahead": {"base_min_distance_m": 2., "distance_ratio_s": .2}}
    history = {name: digest for name in audit.HISTORICAL_SHA}
    monkeypatch.setattr(audit, "HISTORICAL_SHA", history)
    monkeypatch.setattr(audit, "recorded_bytes", lambda *_: raw)
    plan = copy.deepcopy(audit.PLAN_FIXED)
    plan.update(comment="HH_260906 - Synthetic prospective fixture.", declared_at_utc="2026-09-07T20:00:00Z",
        source_hashes={name: digest for name in audit.SOURCES},
        capture_flags=[*capture_flags(), "--agent-initialization", "after_bootstrap"],
        historical_lateral_configuration=copy.deepcopy(control), comparison_notice="Whole constructor context, not physical causality.",
        expected_risks=["Terrain.", "No physical guarantee.", "No pose rewriting."],
        initialization_contract=copy.deepcopy(audit.INIT_CONTRACT), review_measurements=["Review all observations."] * 5,
        after_first_trial="No automatic retry.", historical_before_bootstrap_reference={
            "campaign": audit.HISTORICAL_ROOT, "source_commit": audit.turn.COMMIT, "source_sha256": history,
            "native_launch_maximum_acceleration_mps2": 3.654932706817288, "initial_next_steering": -.8,
            "notice": "Historical nonrandomized reference only."})
    def read_json(_root, path, ledger):
        ledger[path] = {"sha256": plan["historical_lateral_baseline_sha256"], "size_bytes": 99}
        return {"capture_contract": {"basic_agent_control": copy.deepcopy(control)}}
    def checked(_root, path, ledger):
        ledger[path] = {"sha256": digest, "size_bytes": len(raw)}
        return raw
    monkeypatch.setattr(audit.pilot, "read_json", read_json)
    monkeypatch.setattr(audit.pilot, "checked_bytes", checked)
    return plan


def test_prospective_new_source_and_historical_reference_bound(prospective):
    ledger = audit.validate_plan_document(prospective)
    assert len(ledger) == 5


@pytest.mark.parametrize("key,value", [("source_commit", "f" * 40), ("schema", "portable_e2e.turn_low_plan.v1"),
    ("agent_initialization", "before_bootstrap"), ("physics_hz", True), ("nominal_target_speed_kmh", 30),
    ("maximum_actual_speed_mps", 8.3333), ("quality", "Low"), ("normal_brake_cap", .01),
    ("maximum_attempts_per_revision", 3), ("review_required_before_second_attempt", False),
    ("qualification_30_kph", "PASS"), ("training_data_approved", True), ("declared_at_utc", "2026-09-08T02:00:00Z")])
def test_prospective_settings_fail_closed(prospective, key, value):
    prospective[key] = value
    with pytest.raises(audit.base.EvidenceError): audit.validate_plan_document(prospective)


@pytest.mark.parametrize("mutation", ["source", "history_source", "history_control", "new_pid", "extra", "missing",
    "old_cli", "init_abbreviation", "hidden_retry", "history_sha"])
def test_prospective_source_and_obligations_cannot_be_substituted(prospective, mutation):
    if mutation == "source": prospective["source_hashes"]["scripts/e2e/collect_carla_vad_expert.py"] = "a" * 64
    elif mutation == "history_source": prospective["historical_before_bootstrap_reference"]["source_commit"] = audit.COMMIT
    elif mutation == "history_control": prospective["historical_before_bootstrap_reference"]["initial_next_steering"] = 0.
    elif mutation == "new_pid": prospective["initialization_contract"]["pid_history_overwritten"] = True
    elif mutation == "extra": prospective["relax_native_bound"] = True
    elif mutation == "missing": prospective.pop("review_measurements")
    elif mutation == "old_cli": prospective["capture_flags"][-1] = "before_bootstrap"
    elif mutation == "init_abbreviation": prospective["capture_flags"][-2] = "--agent-init"
    elif mutation == "hidden_retry": prospective["automatic_retry"] = True
    elif mutation == "history_sha": prospective["historical_before_bootstrap_reference"]["source_sha256"] = {"foreign": "b" * 64}
    with pytest.raises(audit.base.EvidenceError): audit.validate_plan_document(prospective)
