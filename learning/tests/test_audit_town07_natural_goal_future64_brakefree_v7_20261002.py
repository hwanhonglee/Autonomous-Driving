"""HH_261002 - Synthetic V7 identity, caller-pinning and exact frozen true64 checks only; no actual raw scan, capture or TRAIN."""

from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import FunctionType

import pytest

from scripts.e2e import audit_town07_natural_goal_future64_brakefree_v7_20261002 as audit
from learning.tests import test_audit_town07_natural_goal_future64_brakefree_v6_20261002 as v6fixtures
from learning.tests import test_audit_town07_natural_goal_future64_brakefree_v5_20261002 as geometry_fixtures


def fixture():
    original = v6fixtures.fixture
    namespace = dict(original.__globals__, audit=audit)
    return FunctionType(original.__code__, namespace, original.__name__, original.__defaults__)()


def core(inputs):
    with audit.activated(): return audit.audit_core(*inputs)


def revision_fixture():
    plan = audit.task.fixed_contract("v7", "pilot_00")
    config = deepcopy(plan["natural_goal_configuration"])
    config["effective_control"] = dict(normal_brake_cap=0., normal_throttle_cap=.20,
        basic_agent_emergency_brake_modified=False, phase_hook_rebound_after_original_outer_cap=True,
        original_two_normal_cap_calls_and_requests_unchanged=True, downstream_BasicAgent_hazard_untouched=True)
    manifest = dict(control_source="Disclosed_BasicAgent_manual1_1p50_latched_brakefree_coast_low_speed_hold_v7",
        capture_contract=dict(goal_stop_profile=config, teacher_controller=dict(normal_brake_cap=0., normal_throttle_cap=.20,
            normal_launch_controller_revision=audit.TASK_SCHEMA + ".normal_control", manual_gear_shift=True, gear=1,
            inherited_BasicAgent_emergency_unchanged=True, causal_brakefree_terminal_mission=audit.task.mission_contract(),
            phase_hook_after_original_outer_cap=True)))
    return plan, manifest


def test_exact_same_frozen_v6_numeric_function_objects_and_v7_source_pins():
    assert audit.defined_geometry is audit.frozen_v6_audit.defined_geometry
    assert audit.audit_core is audit.frozen_v6_audit.audit_core
    assert audit.intent_witness is audit.frozen_v6_audit.intent_witness
    assert audit.governor_receipt_binding is audit.frozen_v6_audit.governor_receipt_binding
    assert audit.SOURCE_PINS[audit.FROZEN_V6_AUDIT_PATH] == audit.FROZEN_V6_AUDIT_SHA
    assert audit.V7_SOURCE_PINS == audit.V7_EXPECTED_SOURCES
    assert all(audit.SOURCE_PINS[name] == value for name, value in audit.task.FROZEN.items())
    assert audit.SCHEMA.endswith(".v7") and audit.task.SCHEMA == audit.TASK_SCHEMA
    audit.verify_frozen()


def test_same_full360_anchors23040_true_points_zero_future_and_NULL_masks():
    inputs = fixture(); unchanged = deepcopy(inputs)
    summary, windows = core(inputs)
    with audit.activated(): prior, original = audit.definition("audit_core")(*inputs)
    assert inputs == unchanged and summary["selected_anchor_count"] == 360 and summary["true_future_point_count"] == 23040
    assert summary["failed_fixed_window_count"] == 0 and summary["strict_defined_geometry_failed_window_count"] == 0
    for current, old in zip(windows, original):
        assert [{key: value for key, value in row.items() if key != "strict_defined_geometry_witness"}
            for row in current["true_future_rows"]] == old["true_future_rows"]
    assert summary["future_discrete_metric_counts"] == prior["future_discrete_metric_counts"]
    assert windows[-1]["true_future_rows"][-1]["speed_mps"] == 0.
    assert all(row["strict_defined_geometry_witness"]["metrics"]["xy_curvature"] == [None, False]
        for row in windows[-1]["true_future_rows"])
    assert summary["training_data_approved"] is False and summary["research_training_authorized"] is False


def test_low_speed_defined_curvature_still_fails_and_missing_future_not_fabricated():
    rows = geometry_fixtures.future_rows(speed=.001); original = deepcopy(rows)
    results = audit.defined_geometry(rows, initial_speed_mps=.001)
    assert rows == original and all(row["metrics"]["xy_curvature"][1] for row in results[1:])
    rows[4:] = [dict(index=i, valid=False, xy=None, yaw_rad=None, speed_mps=None) for i in range(4, 64)]
    results = audit.defined_geometry(rows, initial_speed_mps=.001)
    assert len(results) == 64 and results[-1]["metrics"]["xy_curvature"] == [None, False]
    rows[6] = geometry_fixtures.future_rows()[6]
    with pytest.raises(ValueError): audit.defined_geometry(rows, initial_speed_mps=.001)


def test_actual_v7_configuration_straight_terminal_and_original_cap_order_witness():
    plan, manifest = revision_fixture(); result = audit.teacher_revision_witness(plan, manifest)
    assert result["actual_native_config_and_installed_controls_bind_declared_v7"] is True
    assert "actual_native_config_and_installed_controls_bind_declared_v6" not in result
    assert result["actual_postconstructor_phase_order_witness_verified"] is True
    assert result["terminal_only_zero_normal_steering_declaration_verified"] is True
    assert result["steering_and_actual_ACK_match_is_prior_strict_owner_evidence_not_relaxed_here"] is True
    assert result["unchanged_true64_geometry_can_still_fail_and_never_grants_TRAIN"] is True


@pytest.mark.parametrize("mutation", ["old_v6_source", "old_v6_profile", "zero_steering_false", "changed_GO", "changed_hazard",
    "whole_route_claim", "changed_span", "missing_order", "integer_order", "nonzero_brake", "gear_bool"])
def test_v7_source_or_declared_intervention_cannot_be_forged_or_old_failure_relabelled(mutation):
    plan, manifest = revision_fixture(); capture = manifest["capture_contract"]
    mission = capture["teacher_controller"]["causal_brakefree_terminal_mission"]
    if mutation == "old_v6_source": manifest["control_source"] = manifest["control_source"].replace("v7", "v6")
    elif mutation == "old_v6_profile": capture["goal_stop_profile"]["profile_id"] = audit.frozen_v6_audit.PROFILE
    elif mutation == "zero_steering_false": mission["normal_COAST_and_HOLD_steering_zero"] = False
    elif mutation == "changed_GO": mission["original_GO_lateral_and_all_downstream_BasicAgent_hazards_unchanged"] = False
    elif mutation == "changed_hazard": mission["normal_COAST_HOLD_override_not_applied_to_hazard_outputs"] = False
    elif mutation == "whole_route_claim": mission["terminal_only_straightness_precondition"]["whole_route_collinearity_claim"] = True
    elif mutation == "changed_span": mission["terminal_only_straightness_precondition"]["assessed_terminal_span_m"] = 11.5
    elif mutation == "missing_order": capture["teacher_controller"].pop("phase_hook_after_original_outer_cap")
    elif mutation == "integer_order": capture["goal_stop_profile"]["effective_control"]["phase_hook_rebound_after_original_outer_cap"] = 1
    elif mutation == "nonzero_brake": capture["goal_stop_profile"]["effective_control"]["normal_brake_cap"] = .01
    else: capture["teacher_controller"]["gear"] = True
    with pytest.raises(ValueError): audit.teacher_revision_witness(plan, manifest)


def test_failed_owner_native_or_true64_never_gets_TRAIN_permission():
    inputs = fixture(); inputs[2][767]["vx"], inputs[2][768]["vx"], inputs[2][769]["vx"] = 1.5371, .721294, .000851
    inputs[1]["status"] = "failed"; inputs[1]["result"]["goal_stop_quality"]["status"] = "FAIL"
    inputs[1]["result"]["moving_task"]["task_complete"] = False
    inputs[1]["result"]["training_data_approved"] = False
    saved = deepcopy(inputs); summary, windows = core(inputs)
    assert inputs == saved and summary["source_capture_failed"] is True
    assert summary["native_explicit_denial_present"] is True and summary["raw_native_denials_overridden"] is False
    assert summary["selected_anchor_count"] == 360 and summary["true_future_point_count"] == 23040
    assert any("protected_native20Hz_dynamics_or_events" in row["failure_reasons"] for row in windows)
    assert summary["research_training_authorized"] is False


def test_both_identity_bindings_restore_after_error_and_no_nesting():
    names = ("task", "SCHEMA", "TASK_SCHEMA", "PROFILE", "SOURCE_PINS", "V5_SOURCE_PINS", "definition", "teacher_revision_witness")
    saved = {name: getattr(audit.previous, name) for name in names}
    inner_names = ("SCHEMA", "TASK_SCHEMA", "PROFILE", "SOURCE_PINS", "natural_task", "audit_core", "intent_witness", "governor_receipt_binding")
    inner = {name: getattr(audit.previous.previous, name) for name in inner_names}
    with pytest.raises(RuntimeError):
        with audit.activated():
            assert audit.previous.task is audit.task and audit.previous.previous.SCHEMA == audit.SCHEMA
            for name in ("inspect", "audit_core", "governor_receipt_binding"): assert callable(audit.definition(name))
            with pytest.raises(ValueError):
                with audit.activated(): pass
            raise RuntimeError("synthetic")
    assert all(getattr(audit.previous, name) is value for name, value in saved.items())
    assert all(getattr(audit.previous.previous, name) is value for name, value in inner.items())
    assert not audit._active and not audit.previous._active and not audit.frozen_v6_audit._active


def caller_fixture(monkeypatch, tmp_path):
    root = tmp_path / "workspace"; output = root / "pilot_00"; episode = output / "episode"; episode.mkdir(parents=True)
    plan = root / "pilot_00_plan.json"; plan.write_text("{}")
    expected_plan = hashlib.sha256(plan.read_bytes()).hexdigest()
    monkeypatch.setattr(audit, "CASE_PLAN_SHA256", expected_plan)
    monkeypatch.setattr(audit.task, "paths", lambda run, case: (output, plan))
    artifacts = {}
    for path in [episode / name for name in audit.EPISODE_FILES] + [output / name for name in audit.OWNER_FILES]:
        path.parent.mkdir(parents=True, exist_ok=True); path.write_text("{}")
        artifacts[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    data = dict(schema=audit.INPUT_SCHEMA, case_plan_sha256=expected_plan, artifact_sha256=artifacts)
    caller = root / "external_actual_pins.json"
    def write():
        caller.write_text(json.dumps(data)); return hashlib.sha256(caller.read_bytes()).hexdigest()
    return episode, plan, data, caller, write


def test_all13_external_artifacts_exactpins_and_firstread_recheck(monkeypatch, tmp_path):
    episode, plan, data, caller, write = caller_fixture(monkeypatch, tmp_path); receipt_sha = write()
    pins = audit.caller_inputs(episode, plan, audit.CASE_PLAN_SHA256, caller, receipt_sha)
    assert len(pins) == 15 and len(data["artifact_sha256"]) == 13 and pins[str(caller)] == receipt_sha
    (episode / "states.jsonl").write_text("{\"changed\":true}")
    with pytest.raises(ValueError): audit.previous.previous.recheck(pins)
    with pytest.raises(ValueError): audit.caller_inputs(episode, plan, audit.CASE_PLAN_SHA256, caller, receipt_sha)


@pytest.mark.parametrize("mutation", ["missing", "extra", "old_schema", "case_sha", "caller_sha", "null_sha", "bool_sha", "uppercase_sha", "path_escape", "old_episode"])
def test_external_caller_receipt_is_mandatory_and_cannot_omit_or_unpin_failed_inputs(monkeypatch, tmp_path, mutation):
    episode, plan, data, caller, write = caller_fixture(monkeypatch, tmp_path)
    first = next(iter(data["artifact_sha256"]))
    supplied_case = audit.CASE_PLAN_SHA256
    if mutation == "missing": data["artifact_sha256"].pop(first)
    elif mutation == "extra": data["artifact_sha256"][str(tmp_path / "other")] = "a" * 64
    elif mutation == "old_schema": data["schema"] = data["schema"].replace("v7", "v6")
    elif mutation == "case_sha": supplied_case = "0" * 64
    elif mutation == "null_sha": data["artifact_sha256"][first] = None
    elif mutation == "bool_sha": data["artifact_sha256"][first] = True
    elif mutation == "uppercase_sha": data["artifact_sha256"][first] = "A" * 64
    elif mutation == "path_escape": episode = episode.parent / "bad" / ".." / "episode"
    elif mutation == "old_episode": episode = episode.with_name("v6")
    receipt_sha = write()
    if mutation == "caller_sha": receipt_sha = None
    with pytest.raises(ValueError): audit.caller_inputs(episode, plan, supplied_case, caller, receipt_sha)


def test_caller_inputs_and_raw_rechecked_after_frozen_algorithm_before_publication(monkeypatch, tmp_path):
    episode, plan, data, caller, write = caller_fixture(monkeypatch, tmp_path); receipt_sha = write()
    def altered(*args):
        (episode / "states.jsonl").write_text("{\"changed\":true}")
        return dict(audit_identity_revision={}), [], {}
    monkeypatch.setattr(audit, "_identity_inspect", altered)
    with pytest.raises(ValueError): audit.inspect(episode, plan, audit.CASE_PLAN_SHA256,
        actual_input_pins=caller, actual_input_pins_sha256=receipt_sha)


def test_original_main_FAIL_exit_createonly_and_unmodified_raw_are_preserved(monkeypatch, tmp_path, capsys):
    root = tmp_path / "workspace"; root.mkdir(); raw = root / "raw/pilot_00/episode"; raw.mkdir(parents=True)
    output = root / "derived/v7"
    summary = dict(failed_fixed_window_count=1, full_raw_event_count=0, source_capture_failed=True,
        source_artifact_sha256={}, training_data_approved=False, research_training_authorized=False, vehicle_control_approved=False)
    calls = []
    def inspect(*args, **kwargs):
        calls.append(kwargs); return summary, [dict(source_frame=1, true_future_rows=[])], {}
    monkeypatch.setattr(audit, "ROOT", root); monkeypatch.setattr(audit, "verify_frozen", lambda: {})
    monkeypatch.setattr(audit, "inspect", inspect)
    monkeypatch.setattr(audit, "FROZEN_V5_AUDIT_PATH", str(Path(audit.previous.__file__)))
    argv = ["--actual-input-pins", str(root / "caller.json"), "--actual-input-pins-sha256", "b" * 64,
        "--episode", str(raw), "--case-plan", str(root / "plan.json"), "--case-plan-sha256", "a" * 64, "--output", str(output)]
    assert audit.main(argv) == 2 and not any(raw.iterdir())
    assert calls == [dict(actual_input_pins=str(root / "caller.json"), actual_input_pins_sha256="b" * 64)]
    assert set(path.name for path in output.iterdir()) == {"summary.json", "all_fixed_anchor_true_future64.jsonl"}
    assert json.loads((output / "summary.json").read_text())["research_training_authorized"] is False
    with pytest.raises(ValueError): audit.main(argv)
    argv[-1] = str(raw.parent / "illegal")
    with pytest.raises(ValueError): audit.main(argv)

