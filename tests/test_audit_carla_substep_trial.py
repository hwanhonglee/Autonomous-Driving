"""HH_260906 - Test the new substep auditor with synthetic evidence only; never launch a simulator or touch original captures."""

import copy
import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import audit_carla_substep_trial as audit
from tests.test_audit_carla_turn_launch_matrix import capture_flags, protocol_fixture
from tests.test_audit_carla_turn_launch_raw_geometry import geometry, state


def settings_record(profile="fine_5ms"):
    before = dict(synchronous_mode=False, no_rendering_mode=False, substepping=True, deterministic_ragdolls=False,
        spectator_as_ego=True, fixed_delta_seconds=None, max_substep_delta_time=.01, max_substeps=10,
        max_culling_distance=0., tile_stream_distance=3000., actor_active_distance=2000.)
    after = dict(before, synchronous_mode=True, fixed_delta_seconds=.05, max_substep_delta_time=audit.SUBSTEPS[profile])
    record = {"schema": "carla.physics_substep_experiment.v1", "profile": profile, "status": "PASS",
        "explicit_opt_in": True, "numerical_only": True, "training_data_approved": False, "development_only": True,
        "comparison_absolute_tolerance": 1e-12,
        "required_original": {"substepping": True, "max_substep_delta_time": .01, "max_substeps": 10},
        "requested_max_substep_delta_time": audit.SUBSTEPS[profile], "before": before, "requested": dict(after), "after": dict(after),
        "notice": "Maximum numerical substep setting, not measured internal substeps or improved physics accuracy; no dataset admission."}
    keys = ("synchronous_mode", "fixed_delta_seconds", "no_rendering_mode", "substepping", "max_substep_delta_time", "max_substeps")
    runtime = {"town": audit.matrix.MAP, "weather": "ClearNoon", "vehicle_type": "vehicle.toyota.prius",
        "original_world_settings": {k: before[k] for k in keys}, "capture_world_settings": {k: after[k] for k in keys}}
    return record, runtime


@pytest.mark.parametrize("profile", audit.SUBSTEPS)
def test_independent_eleven_settings_only_declared_change(profile):
    record, runtime = settings_record(profile)
    original = copy.deepcopy(record)
    result = audit.analyze_substeps(record, runtime, profile)
    assert result["all_checks_pass"] and all(result["flags"].values())
    assert not result["internal_substep_count_measured"]
    assert not result["vehicle_physics_parameters_independently_measured"]
    assert not result["training_data_approved"] and record == original


@pytest.mark.parametrize("field,value", [("max_substeps", 50), ("max_substeps", True), ("substepping", 1),
    ("max_substep_delta_time", .001), ("max_substep_delta_time", float("nan")),
    ("max_culling_distance", float("inf")), ("actor_active_distance", -1),
    ("spectator_as_ego", False), ("tile_stream_distance", 1000.), ("fixed_delta_seconds", .1)])
def test_readback_drift_or_nonfinite_cannot_claim_pass(field, value):
    record, runtime = settings_record()
    record["after"][field] = value
    with pytest.raises(audit.base.EvidenceError): audit.analyze_substeps(record, runtime, "fine_5ms")


@pytest.mark.parametrize("field,value", [("training_data_approved", True), ("development_only", False),
    ("explicit_opt_in", 1), ("comparison_absolute_tolerance", .01), ("profile", "reference_10ms"),
    ("requested_max_substep_delta_time", .001), ("required_original", {"substepping": True, "max_substep_delta_time": .01, "max_substeps": 50})])
def test_contract_fields_cannot_relax_bounds_or_change_arm(field, value):
    record, runtime = settings_record()
    record[field] = value
    with pytest.raises(audit.base.EvidenceError): audit.analyze_substeps(record, runtime, "fine_5ms")


def test_runtime_readback_mismatch_is_independent_failure():
    record, runtime = settings_record()
    runtime["capture_world_settings"]["max_substep_delta_time"] = .01
    result = audit.analyze_substeps(record, runtime, "fine_5ms")
    assert not result["all_checks_pass"] and result["failed_flags"] == ["runtime_summary_matches"]


@pytest.mark.parametrize("status", ["NOT_REACHED", "FAIL"])
def test_failed_or_unreached_application_is_retained_without_fabricated_pass(status):
    record, _ = settings_record()
    record.update(status=status, before=None, requested=None, after=None)
    if status == "FAIL": record["error"] = "ValueError: readback unavailable"
    result = audit.analyze_substeps(record, None, "fine_5ms")
    assert result["status"] == status and not result["all_checks_pass"]


def test_failed_readback_retains_actual_wrong_settings():
    record, _ = settings_record()
    record.update(status="FAIL", error="ValueError: CARLA physics substep readback mismatch")
    record["after"]["max_substep_delta_time"] = .01
    result = audit.analyze_substeps(record, None, "fine_5ms")
    assert not result["all_checks_pass"]
    assert result["original_metadata"]["after"]["max_substep_delta_time"] == .01


def argv(root, profile="fine_5ms"):
    return [str(root / "episode"), "route.json", "--host", "127.0.0.1", "--port", "2100",
            *capture_flags(audit.PROFILE), "--physics-substep-profile", profile]


@pytest.mark.parametrize("profile", audit.SUBSTEPS)
def test_same_legacy_cli_plus_only_explicit_substep_option(tmp_path, profile):
    original = copy.deepcopy((audit.matrix.REVIEWED_SOURCES, audit.matrix.PLAN_FIXED, audit.matrix.turn.FROZEN))
    audit.validate_cli({"port": 2100, "route_path": "route.json", "collector_argv": argv(tmp_path, profile)}, tmp_path, profile)
    assert original == (audit.matrix.REVIEWED_SOURCES, audit.matrix.PLAN_FIXED, audit.matrix.turn.FROZEN)


@pytest.mark.parametrize("mutation", ["duplicate", "abbreviation", "wrong_arm", "wrong_launch", "normal_brake", "foreign_host", "unknown"])
def test_cli_cannot_hide_other_profile_or_override(tmp_path, mutation):
    values = argv(tmp_path)
    if mutation == "duplicate": values += ["--physics-substep-profile", "fine_5ms"]
    elif mutation == "abbreviation": values[-2] = "--physics-substep-prof"
    elif mutation == "wrong_arm": values[-1] = "reference_10ms"
    elif mutation == "wrong_launch": values[values.index("--goal-stop-profile") + 1] = "turn_launch_015_v1"
    elif mutation == "normal_brake": values += ["--normal-brake", "0.1"]
    elif mutation == "foreign_host": values[3] = "example.invalid"
    else: values += ["--allow-map-load"]
    with pytest.raises(audit.base.EvidenceError):
        audit.validate_cli({"port": 2100, "route_path": "route.json", "collector_argv": values}, tmp_path, "fine_5ms")


def test_literal_governor_and_all_future_points_reuse_without_source_override():
    states, rows = protocol_fixture(audit.PROFILE)
    assert audit.matrix.analyze_protocol(states, rows, audit.PROFILE)["all_checks_pass"]
    rows[50]["next_control"]["brake"] = .000000001
    assert not audit.matrix.analyze_protocol(states, rows, audit.PROFILE)["all_checks_pass"]
    native = [state(i, phase="stationary_warmup" if i < 70 else "driving" if i < 80 else "stationary_tail") for i in range(210)]
    report, anchors = geometry(native, range(0, 210, 2))
    assert report["available_64_point_anchor_count"] == 40
    assert len(anchors) == 105 and report["anchor_count_by_phase"]["stationary_warmup"] == 35
    assert all(not item["training_data_approved"] for item in anchors)


def write(root, name, value):
    # HH_260906 - Only pytest temporary directories receive synthetic evidence.
    path = root / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value))
    return path


@pytest.fixture
def plan_fixture(monkeypatch):
    payload = {name: ("test source " + name).encode() for name in audit.SOURCES}
    monkeypatch.setattr(audit, "git_bytes", lambda _commit, name: payload[name])
    return {**audit.PLAN_FIXED, "source_commit": "c" * 40, "declared_at_utc": "2026-09-08T17:00:00Z",
        "source_sha256": {name: audit.digest(value) for name, value in payload.items()},
        "cases": [{"id": path, "substep_profile": profile} for path, profile in audit.ORDER],
        "collector_common_argv": capture_flags(audit.PROFILE)}


def test_plan_exact_four_cases_and_offline_source_binding(plan_fixture):
    audit.validate_plan(plan_fixture, "c" * 40)
    assert len(plan_fixture["source_sha256"]) == 12


@pytest.mark.parametrize("mutation", ["order", "extra", "missing", "source", "model", "quota", "deadline", "admission", "common"])
def test_plan_mutations_fail(plan_fixture, monkeypatch, mutation):
    plan = plan_fixture
    if mutation == "order": plan["cases"].reverse()
    elif mutation == "extra": plan["cases"].append(plan["cases"][0])
    elif mutation == "missing": plan["cases"].pop()
    elif mutation == "source": plan["source_sha256"][audit.HELPER] = "a" * 64
    elif mutation == "model":
        original = audit.git_bytes
        monkeypatch.setattr(audit, "git_bytes", lambda commit, name: b"changed" if commit == audit.matrix.COMMIT and name == "portable_e2e/model.py" else original(commit, name))
    elif mutation == "quota": plan["maximum_attempts"] = 6
    elif mutation == "deadline": plan["finish_before_utc"] = "2026-09-09T02:00:00Z"
    elif mutation == "admission": plan["training_data_approved"] = True
    else: plan["collector_common_argv"] += ["--physics-substep-profile", "reference_10ms"]
    with pytest.raises(audit.base.EvidenceError): audit.validate_plan(plan, "c" * 40)


@pytest.fixture
def campaign_fixture(tmp_path, monkeypatch, plan_fixture):
    root = tmp_path / "campaign"
    root.mkdir()
    write(root, "plan.json", plan_fixture)
    monkeypatch.setattr(audit, "source_identity", lambda: {"files": {name: plan_fixture["source_sha256"][name]
        for name in ("portable_e2e/model.py", "portable_e2e/runtime_contract.py")}})
    monkeypatch.setattr(audit, "recheck_trial", lambda *_: None)
    def finish(index, fail=False):
        case = plan_fixture["cases"][index]
        write(root, case["id"] + "/owner_plan.json", {"planned_at_utc": f"2026-09-08T17:{index * 3 + 1:02d}:00Z"})
        write(root, case["id"] + "/owner_result.json", {"completed_at_utc": f"2026-09-08T17:{index * 3 + 2:02d}:00Z"})
    def audited(_root, _plan, case):
        return {"case_id": case["id"], "independent_protocol_clear": True, "source_manifest": [],
                "continuation_observations": {"established_continuation_stop_condition": False}}, [], {"streams": {}}
    monkeypatch.setattr(audit, "audit_trial", audited)
    return root, plan_fixture, finish


def campaign_report(root):
    return audit.audit_campaign(root, expected_plan_sha256=audit.base.sha(root / "plan.json"), expected_source_commit="c" * 40)


def test_unrun_and_first_complete_pair_keep_all_four_denominators(campaign_fixture):
    root, _, finish = campaign_fixture
    report, audits = campaign_report(root)
    assert report["finalized_case_count"] == 0 and len(report["cases"]) == 4 and not audits
    finish(0); finish(1)
    report, audits = campaign_report(root)
    assert report["finalized_case_count"] == 2 and len(audits) == 2
    assert [case["status"] for case in report["cases"]] == ["FINALIZED", "FINALIZED", "NOT_RUN", "NOT_RUN"]
    assert report["status"] == "INCOMPLETE" and not report["training_data_approved"]


def test_failed_validator_is_not_omitted_or_passed(campaign_fixture, monkeypatch):
    root, _, finish = campaign_fixture
    finish(0)
    def fail(*_): raise audit.base.EvidenceError("intentional corrupt archive")
    monkeypatch.setattr(audit, "audit_trial", fail)
    report, audits = campaign_report(root)
    assert len(report["cases"]) == 4 and not audits
    assert report["cases"][0]["status"] == "AUDIT_FAILED"
    assert "corrupt archive" in report["cases"][0]["error"]
    assert not report["training_data_approved"]


def test_started_out_of_order_retains_existing_case_but_never_complete(campaign_fixture):
    root, _, finish = campaign_fixture
    finish(1)
    report, audits = campaign_report(root)
    assert len(report["cases"]) == 4 and len(audits) == 1
    assert report["ordering_errors"] and report["status"] == "INCOMPLETE"


def test_undeclared_extra_attempt_is_rejected(campaign_fixture):
    root, _, _ = campaign_fixture
    (root / "A_reference_10ms/run_003").mkdir(parents=True)
    with pytest.raises(audit.base.EvidenceError, match="undeclared"): campaign_report(root)


def test_output_existing_input_descendant_and_dataset_alias_rejected(tmp_path, monkeypatch):
    raw_root, repository = tmp_path / "input", tmp_path / "repo"
    raw_root.mkdir(); repository.mkdir()
    dataset = tmp_path / "data"
    dataset.mkdir(); (repository / "datasets").symlink_to(dataset, target_is_directory=True)
    monkeypatch.setattr(audit, "ROOT", repository)
    monkeypatch.setattr(audit, "audit_campaign", lambda *_a, **_k: pytest.fail("must reject before analysis"))
    for output in (raw_root, raw_root / "new", dataset / "new"):
        with pytest.raises(audit.base.EvidenceError):
            audit.run(raw_root, output, expected_plan_sha256="a" * 64, expected_source_commit="c" * 40)


def test_no_arbitrary_source_paths_or_network_git(monkeypatch):
    monkeypatch.setattr(audit.subprocess, "run", lambda *_a, **_k: pytest.fail("unsafe source reached subprocess"))
    for commit, name in (("HEAD", audit.HELPER), ("c" * 40, "../bad"), ("c" * 40, "private.key")):
        with pytest.raises(audit.base.EvidenceError): audit.git_bytes(commit, name)


def test_repeated_metadata_reads_cannot_overwrite_earlier_hash(tmp_path):
    write(tmp_path, "record.json", {"count": 1})
    ledger = {}
    assert audit.read_json(tmp_path, "record.json", ledger) == {"count": 1}
    original = copy.deepcopy(ledger)
    write(tmp_path, "record.json", {"count": 2})
    with pytest.raises(audit.base.EvidenceError, match="between analytical"):
        audit.read_json(tmp_path, "record.json", ledger)
    assert ledger == original


@pytest.mark.parametrize("payload", ['{"x":1,"x":2}', '{"x":NaN}', '{"x":Infinity}'])
def test_strict_duplicate_or_nonfinite_metadata_rejected(tmp_path, payload):
    (tmp_path / "bad.json").write_text(payload)
    with pytest.raises(audit.base.EvidenceError): audit.read_json(tmp_path, "bad.json", {})


def test_loaded_geometry_limits_may_not_be_relaxed(monkeypatch):
    monkeypatch.setattr(audit.raw.targets, "HEADING_MINIMUM_STEP_M", .01)
    with pytest.raises(audit.base.EvidenceError, match="bounds changed"): audit.source_identity()


def test_plan_changed_during_output_persistence_cannot_finalize(campaign_fixture, monkeypatch, tmp_path):
    root, _, _ = campaign_fixture
    report, audits = campaign_report(root)
    monkeypatch.setattr(audit, "audit_campaign", lambda *_a, **_k: (report, audits))
    original = audit.json.dumps
    def mutate(value, *args, **kwargs):
        if isinstance(value, dict) and value.get("schema") == audit.SCHEMA:
            (root / "plan.json").write_text('{"changed_during_persistence":true}')
        return original(value, *args, **kwargs)
    monkeypatch.setattr(audit.json, "dumps", mutate)
    output = tmp_path / "new_output"
    with pytest.raises(audit.base.EvidenceError, match="campaign/review bytes changed"):
        audit.run(root, output, expected_plan_sha256="x" * 64, expected_source_commit="c" * 40)
    assert (output / "audit.json").is_file() and not (output / "SHA256SUMS").exists()


def test_final_checksum_manifest_excludes_itself_and_verifies_every_payload(campaign_fixture, tmp_path):
    root, _, _ = campaign_fixture
    output = tmp_path / "complete_output"
    audit.run(root, output, expected_plan_sha256=audit.base.sha(root / "plan.json"), expected_source_commit="c" * 40)
    entries = [line.split("  ") for line in (output / "SHA256SUMS").read_text().splitlines()]
    assert len(entries) == 4 and {name for _, name in entries} == {
        "audit.json", "future_geometry_audit.jsonl", "native_snapshot_audit.jsonl", "jpeg_audit.jsonl"}
    assert all(audit.base.sha(output / name) == digest for digest, name in entries)


@pytest.mark.parametrize("kind", ["review", "interim", "new_trial"])
def test_final_campaign_recheck_covers_reviews_interim_and_not_run_inventory(tmp_path, monkeypatch, kind):
    campaign, repository = tmp_path / "campaign", tmp_path / "repository"
    campaign.mkdir(); repository.mkdir()
    monkeypatch.setattr(audit, "ROOT", repository)
    review = write(campaign, "reviews/first_pair.json", {"synthetic": 1})
    interim = write(repository, "artifacts/interim.json", {"synthetic": 2})
    report = {"source_manifest": [{"path": "reviews/first_pair.json", "sha256": audit.base.sha(review)}],
        "first_pair_review": {"interim_audit": {"path": "artifacts/interim.json", "sha256": audit.base.sha(interim)}},
        "cases": [{"id": name, "status": "NOT_RUN"} for name, _ in audit.ORDER]}
    audit.recheck_campaign(campaign, report)
    if kind == "review": review.write_text("changed")
    elif kind == "interim": interim.write_text("changed")
    else: (campaign / audit.ORDER[0][0]).mkdir(parents=True)
    with pytest.raises(audit.base.EvidenceError): audit.recheck_campaign(campaign, report)


@pytest.fixture
def owned_fixture(tmp_path, monkeypatch, plan_fixture):
    root = tmp_path / "run_001"
    root.mkdir()
    plan = plan_fixture
    case = plan["cases"][0]
    pins = plan["source_sha256"]
    for name in audit.SOURCES:
        path = root / "provenance" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(audit.git_bytes(plan["source_commit"], name))
    owner_plan = {"source_bytes_archived": True, "bounds_source_bytes_archived": True,
        "wall_timing_enabled": True, "wall_timing_source_bytes_archived": True,
        "physics_substeps_source_bytes_archived": True, "source_sha256": pins, "source_head_commit": "d" * 40,
        "physics_substep_profile": case["substep_profile"], "physics_substeps_schema": "carla.physics_substep_experiment.v1",
        "wall_timing_schema": "carla.expert_wall_timing.v1", "host": "127.0.0.1", "port": 2100,
        "map": audit.matrix.MAP, "quality": "Epic", "capture_mode": "expert",
        "worker_path": "scripts/e2e/collect_carla_vad_expert.py", "route_sha256": audit.matrix.ROUTE_SHA,
        "route_path": str(audit.ROOT / plan["route_path"]), "learned_model_control": False, "vehicle_control_approved": False,
        "server_extra_options": ["-RenderOffScreen", "-nosound"], "collector_wall_timeout_sec": 900,
        "finish_before_utc": plan["finish_before_utc"], "planned_at_utc": "2026-09-08T17:01:00Z",
        "source_worktree_status": " M unrelated_report.py"}
    owner_plan["collector_argv"] = [str(root / "episode"), owner_plan["route_path"], "--host", "127.0.0.1", "--port", "2100",
        *plan["collector_common_argv"], "--physics-substep-profile", case["substep_profile"]]
    owner = {"source_checks": {name: True for name in audit.SOURCES}, "source_bytes_unchanged_and_archived": True,
        "capture_mode": "expert", "learned_model_control": False, "vehicle_control_approved": False,
        "exit_code": 1, "completed_at_utc": "2026-09-08T17:04:00Z"}
    started = {"server_pid": 123, "server_pgid": 123, "map": audit.matrix.MAP, "quality": "Epic", "port": 2100,
        "started_at_utc": "2026-09-08T17:02:01Z"}
    log = b"synthetic log only\n"
    (root / "server.log").write_bytes(log)
    for name, value in (("owner_plan.json", owner_plan), ("owner_result.json", owner), ("owner_started.json", started)):
        write(root, name, value)
    for stage, moment in (("ready", "17:02:00"), ("after_capture", "17:03:00"), ("stopped", "17:03:30")):
        document = {"status": "PASS", "stage": stage, "read_only": True, "owner_pid": 123, "owner_pgid": 123,
            "generation_id": "expert_123", "host": "127.0.0.1", "port": 2100, "expected_map": audit.matrix.MAP,
            "mode": "stopped" if stage == "stopped" else "running", "active_map_basename": audit.matrix.MAP,
            "port_released": stage == "stopped", "owner_process_state": None if stage == "stopped" else "S",
            "server_log": {"size_bytes": len(log), "sha256": audit.digest(log)}, "checked_at": f"2026-09-08T{moment}Z"}
        write(root, "lifecycle/" + stage + ".json", document)
    return root, plan, case


def test_exact_twelve_owner_sources_distinct_publication_head_and_failed_capture_retained(owned_fixture):
    root, plan, case = owned_fixture
    ledger = {}
    owner_plan, owner, _, _ = audit.verify_owner(root, plan, case, ledger)
    assert owner_plan["source_head_commit"] != plan["source_commit"]
    assert owner["exit_code"] == 1 and len([p for p in ledger if p.startswith("provenance/")]) == 12
    assert owner_plan["source_worktree_status"] == " M unrelated_report.py"


@pytest.mark.parametrize("mutation", ["missing", "extra", "changed_bytes", "postcheck", "wrong_profile", "late_owner",
    "wrong_pid", "not_stopped", "changed_log", "owner_git_changed"])
def test_owner_fake_integration_fails_closed(owned_fixture, monkeypatch, mutation):
    root, plan, case = owned_fixture
    if mutation == "missing": (root / "provenance" / audit.HELPER).unlink()
    elif mutation == "extra": (root / "provenance" / "extra.py").write_text("extra")
    elif mutation == "changed_bytes": (root / "provenance" / audit.HELPER).write_text("changed")
    elif mutation == "changed_log": (root / "server.log").write_text("changed")
    elif mutation == "owner_git_changed":
        old = audit.git_bytes
        monkeypatch.setattr(audit, "git_bytes", lambda commit, name: b"other" if commit == "d" * 40 and name == audit.HELPER else old(commit, name))
    else:
        path = "owner_result.json" if mutation in ("postcheck", "late_owner") else "owner_plan.json" if mutation == "wrong_profile" else "lifecycle/stopped.json"
        value = json.loads((root / path).read_text())
        if mutation == "postcheck": value["source_checks"][audit.HELPER] = False
        elif mutation == "late_owner": value["completed_at_utc"] = "2026-09-09T01:00:01Z"
        elif mutation == "wrong_profile": value["physics_substep_profile"] = "fine_5ms"
        elif mutation == "wrong_pid": value["owner_pid"] = 999
        else: value["port_released"] = False
        write(root, path, value)
    with pytest.raises(audit.base.EvidenceError): audit.verify_owner(root, plan, case, {})


@pytest.fixture
def pair_review_fixture(tmp_path, monkeypatch):
    repository, campaign = tmp_path / "repository", tmp_path / "campaign"
    repository.mkdir(); campaign.mkdir()
    monkeypatch.setattr(audit, "ROOT", repository)
    pair, owner_pins = [], {}
    for index, (case_id, _) in enumerate(audit.ORDER[:2]):
        path = write(campaign, case_id + "/owner_result.json", {"completed_at_utc": f"2026-09-08T17:0{index + 2}:00Z"})
        owner_pins[case_id] = audit.base.sha(path)
        pair.append((case_id, {"independent_protocol_clear": True, "archived_source_sha256": {"one": "a" * 64},
            "source_manifest": [{"path": "states.jsonl", "sha256": "b" * 64}],
            "continuation_observations": {"established_continuation_stop_condition": False}}))
    interim = {"schema": audit.SCHEMA, "prospective_plan_sha256": "p" * 64, "reviewed_execution_commit": "c" * 40,
        "created_at_utc": "2026-09-08T17:04:00Z", "finalized_case_count": 2,
        "cases": [{"id": name, "status": "FINALIZED" if i < 2 else "NOT_RUN", "audit": pair[i][1] if i < 2 else None}
                  for i, (name, _) in enumerate(audit.ORDER)]}
    path = write(repository, "artifacts/interim/audit.json", interim)
    review = {"schema": "carla_expert.substep_first_pair_review.v1", "reviewed_at_utc": "2026-09-08T17:05:00Z",
        "plan_sha256": "p" * 64, "first_pair_owner_sha256": owner_pins,
        "interim_audit": {"path": "artifacts/interim/audit.json", "sha256": audit.base.sha(path)},
        "proceed": True, "rationale": "Both complete, repeat fixed pair without selecting outcomes.", "training_data_approved": False}
    write(campaign, "reviews/first_pair.json", review)
    return campaign, pair, {"source_commit": "c" * 40}, {"planned_at_utc": "2026-09-08T17:06:00Z"}


def test_repeat_pair_review_binds_both_owner_hashes_and_interim(pair_review_fixture):
    campaign, pair, plan, next_owner = pair_review_fixture
    result = audit.validate_first_pair_review(campaign, plan, "p" * 64, pair, next_owner, {})
    assert result["proceed"] is True and not result["training_data_approved"]


@pytest.mark.parametrize("mutation", ["owner_sha", "interim_sha", "early_review", "late_review", "stop", "admission", "protocol", "safety"])
def test_repeat_pair_review_cannot_waive_failure_or_time(pair_review_fixture, mutation):
    campaign, pair, plan, next_owner = pair_review_fixture
    review = json.loads((campaign / "reviews/first_pair.json").read_text())
    if mutation == "owner_sha": review["first_pair_owner_sha256"][pair[0][0]] = "c" * 64
    elif mutation == "interim_sha": review["interim_audit"]["sha256"] = "c" * 64
    elif mutation == "early_review": review["reviewed_at_utc"] = "2026-09-08T17:02:30Z"
    elif mutation == "late_review": review["reviewed_at_utc"] = "2026-09-08T17:06:01Z"
    elif mutation == "stop": review["proceed"] = False
    elif mutation == "admission": review["training_data_approved"] = True
    elif mutation == "protocol": pair[1][1]["independent_protocol_clear"] = False
    else: pair[1][1]["continuation_observations"]["established_continuation_stop_condition"] = True
    write(campaign, "reviews/first_pair.json", review)
    with pytest.raises(audit.base.EvidenceError):
        audit.validate_first_pair_review(campaign, plan, "p" * 64, pair, next_owner, {})
