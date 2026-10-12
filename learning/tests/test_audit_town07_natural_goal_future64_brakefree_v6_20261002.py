"""HH_261002 - Synthetic v6 identity/order tests retain all frozen true64 formulas and failed-source semantics. No actual capture or TRAIN."""

from copy import deepcopy
from pathlib import Path

import pytest

from scripts.e2e import audit_town07_natural_goal_future64_brakefree_v6_20261002 as audit
from learning.tests import test_audit_town07_natural_goal_future64_brakefree_v5_20261002 as frozen_fixtures


def fixture():
    inputs = list(frozen_fixtures.fixture())
    inputs[0].update(schema=audit.TASK_SCHEMA, seed=audit.task.SEED)
    for row in inputs[2]: row["goal_stop"]["profile_id"] = audit.PROFILE
    return inputs


def core(inputs):
    with audit.activated(): return audit.audit_core(*inputs)


def revision_fixture():
    plan = audit.task.fixed_contract("v6", "pilot_00")
    config = deepcopy(plan["natural_goal_configuration"])
    config["effective_control"] = dict(normal_brake_cap=0., normal_throttle_cap=.20,
        basic_agent_emergency_brake_modified=False, phase_hook_rebound_after_original_outer_cap=True,
        original_two_normal_cap_calls_and_requests_unchanged=True, downstream_BasicAgent_hazard_untouched=True)
    manifest = dict(control_source="Disclosed_BasicAgent_manual1_1p50_latched_brakefree_coast_low_speed_hold_v6",
        capture_contract=dict(goal_stop_profile=config, teacher_controller=dict(normal_brake_cap=0., normal_throttle_cap=.20,
            normal_launch_controller_revision=audit.TASK_SCHEMA+".normal_control", manual_gear_shift=True, gear=1,
            inherited_BasicAgent_emergency_unchanged=True, causal_brakefree_terminal_mission=audit.task.mission_contract(),
            phase_hook_after_original_outer_cap=True)))
    return plan, manifest


def test_frozen_numerical_function_objects_and_all360_true64_labels_remain_exact():
    assert audit.defined_geometry is audit.previous.defined_geometry and audit.audit_core is audit.previous.audit_core
    assert audit.intent_witness is audit.previous.intent_witness
    assert audit.governor_receipt_binding is audit.previous.governor_receipt_binding
    inputs = fixture(); before = deepcopy(inputs)
    summary, windows = core(inputs)
    with audit.activated(): original_summary, original_windows = audit.definition("audit_core")(*inputs)
    assert inputs == before and summary["selected_anchor_count"] == 360 and summary["true_future_point_count"] == 23040
    assert summary["failed_fixed_window_count"] == 0 and summary["strict_defined_geometry_failed_window_count"] == 0
    for fresh, original in zip(windows, original_windows):
        assert [{k:v for k,v in r.items() if k != "strict_defined_geometry_witness"} for r in fresh["true_future_rows"]] == original["true_future_rows"]
    assert original_summary["future_discrete_metric_counts"] == summary["future_discrete_metric_counts"]
    assert all(r["strict_defined_geometry_witness"]["metrics"]["xy_curvature"] == [None,False] for r in windows[-1]["true_future_rows"])
    assert summary["research_training_authorized"] is False and summary["raw_native_denials_overridden"] is False


def test_existing_v5_defined_geometry_still_rejects_low_speed_above_bound():
    rows = frozen_fixtures.future_rows(speed=.001); before = deepcopy(rows)
    checked = audit.defined_geometry(rows, initial_speed_mps=.001)
    assert rows == before and all(r["metrics"]["xy_curvature"][1] for r in checked[1:])
    assert all(r["positive_or_low_speed_never_used_as_curvature_mask"] for r in checked)
    tiny = frozen_fixtures.future_rows(speed=0., delta=.0001)
    assert all(r["metrics"]["xy_curvature"] == [None,False] for r in audit.defined_geometry(tiny, initial_speed_mps=0.))


def test_v6_installed_configuration_and_two_cap_order_witness():
    plan, manifest = revision_fixture()
    result = audit.teacher_revision_witness(plan, manifest)
    assert result["actual_native_config_and_installed_controls_bind_declared_v6"] is True
    assert "actual_native_config_and_installed_controls_bind_declared_v5" not in result
    assert result["actual_postconstructor_phase_order_witness_verified"] is True
    assert result["order_witness_does_not_promote_FAILED_capture_or_defined_geometry"] is True


@pytest.mark.parametrize("key", ["phase_hook_after_original_outer_cap", "phase_hook_rebound_after_original_outer_cap",
    "original_two_normal_cap_calls_and_requests_unchanged", "downstream_BasicAgent_hazard_untouched"])
@pytest.mark.parametrize("value", [False, 1, None])
def test_v6_actual_order_witness_cannot_be_missing_false_or_integer_true(key, value):
    plan, manifest = revision_fixture(); capture = manifest["capture_contract"]
    target = capture["teacher_controller"] if key == "phase_hook_after_original_outer_cap" else capture["goal_stop_profile"]["effective_control"]
    if value is None: target.pop(key)
    else: target[key] = value
    with pytest.raises(ValueError): audit.teacher_revision_witness(plan, manifest)


@pytest.mark.parametrize("kind", ["v5_profile", "v5_source", "quota1gib", "brake_nonzero", "hazard_changed", "gear_bool"])
def test_no_renaming_old_failed_v5_or_loosening_original_configuration(kind):
    plan, manifest = revision_fixture(); capture = manifest["capture_contract"]
    if kind == "v5_profile": capture["goal_stop_profile"]["profile_id"] = audit.previous.task.PROFILE
    elif kind == "v5_source": manifest["control_source"] = manifest["control_source"].replace("v6", "v5")
    elif kind == "quota1gib": plan["episode_quota_bytes"] = 1024**3
    elif kind == "brake_nonzero": capture["goal_stop_profile"]["effective_control"]["normal_brake_cap"] = .01
    elif kind == "hazard_changed": capture["goal_stop_profile"]["effective_control"]["basic_agent_emergency_brake_modified"] = True
    else: capture["teacher_controller"]["gear"] = True
    with pytest.raises(ValueError): audit.teacher_revision_witness(plan, manifest)


def test_full_failed_capture_and_native2p95_interval_are_not_promoted():
    inputs = fixture(); inputs[2][767]["vx"], inputs[2][768]["vx"], inputs[2][769]["vx"] = 1.5371,.721294,.000851
    inputs[1]["status"] = "failed"; inputs[1]["result"]["goal_stop_quality"]["status"] = "FAIL"
    inputs[1]["result"]["moving_task"]["task_complete"] = False
    inputs[1]["result"]["training_data_approved"] = False
    before = deepcopy(inputs)
    summary, windows = core(inputs)
    assert inputs == before and summary["source_capture_failed"] and summary["selected_anchor_count"] == 360
    assert summary["native_explicit_denial_present"] and summary["training_data_approved"] is False
    assert summary["raw_native_denials_overridden"] is False
    assert any("protected_native20Hz_dynamics_or_events" in w["failure_reasons"] for w in windows)


def test_all_outer_and_inner_bindings_restore_after_exception_and_reject_nesting():
    names = ("task", "SCHEMA", "TASK_SCHEMA", "PROFILE", "SOURCE_PINS", "V5_SOURCE_PINS", "definition", "teacher_revision_witness")
    saved = {k:getattr(audit.previous,k) for k in names}
    base_names = ("SCHEMA", "TASK_SCHEMA", "PROFILE", "SOURCE_PINS", "natural_task", "audit_core", "intent_witness", "governor_receipt_binding")
    inner = {k:getattr(audit.previous.previous,k) for k in base_names}
    with pytest.raises(RuntimeError):
        with audit.activated():
            assert audit.previous.task is audit.task and audit.previous.previous.natural_task is audit.task
            assert audit.previous.previous.SCHEMA == audit.SCHEMA
            for name in ("inspect", "audit_core", "governor_receipt_binding"): assert callable(audit.definition(name))
            with pytest.raises(ValueError):
                with audit.activated(): pass
            raise RuntimeError("synthetic")
    assert all(getattr(audit.previous,k) is v for k,v in saved.items())
    assert all(getattr(audit.previous.previous,k) is v for k,v in inner.items())
    assert not audit._active and not audit.previous._active
    assert audit.SOURCE_PINS[audit.FROZEN_V5_AUDIT_PATH] == audit.FROZEN_V5_AUDIT_SHA
    assert all(audit.SOURCE_PINS[k] == v for k,v in audit.task.FROZEN.items())
    assert set(audit.V6_SOURCE_PINS).issubset(audit.SOURCE_PINS)
    with pytest.raises(ValueError): audit.revised_function("audit_core")


def test_actual_governor_pending_and_sameframe_semantics_use_frozen_binding():
    row, receipt = frozen_fixtures.governor_pair()
    row["goal_stop"]["profile_id"] = audit.PROFILE
    receipt.update(schema=audit.TASK_SCHEMA+".actual_governor_update", original_governor_update=deepcopy(row["goal_stop"]))
    with audit.activated(): report = audit.governor_receipt_binding([row],[receipt])
    assert report["actual_new_mission_fields_bind_sameframe_receipts"] is True
    assert report["original_governor_math_unchanged"] is False
    row["goal_stop"]["training_admission_state"] = "TRAIN_APPROVED"
    receipt["original_governor_update"] = deepcopy(row["goal_stop"])
    with audit.activated():
        with pytest.raises(ValueError): audit.governor_receipt_binding([row],[receipt])


def test_frozen_create_only_main_retains_fail_exit_and_no_admission(monkeypatch, tmp_path, capsys):
    import json
    root = tmp_path/"workspace"; root.mkdir(); raw = root/"raw/pilot_00/episode"; raw.mkdir(parents=True)
    output = root/"derived/v6"
    summary = dict(failed_fixed_window_count=1, full_raw_event_count=0, source_capture_failed=True,
        source_artifact_sha256={}, training_data_approved=False, research_training_authorized=False, vehicle_control_approved=False)
    monkeypatch.setattr(audit,"ROOT",root); monkeypatch.setattr(audit,"verify_frozen",lambda:{})
    monkeypatch.setattr(audit,"inspect",lambda *a:(summary,[dict(source_frame=1,true_future_rows=[])],{}))
    # HH_261002 - Only locate frozen code under its real repo root; synthetic output stays in a fresh independent test root.
    monkeypatch.setattr(audit,"FROZEN_V5_AUDIT_PATH",str(Path(audit.previous.__file__)))
    argv = ["--episode",str(raw),"--case-plan",str(root/"plan.json"),"--case-plan-sha256","a"*64,"--output",str(output)]
    assert audit.main(argv) == 2 and not any(raw.iterdir())
    assert json.loads((output/"summary.json").read_text())["research_training_authorized"] is False
    with pytest.raises(ValueError): audit.main(argv)
    argv[-1] = str(raw.parent/"illegal")
    with pytest.raises(ValueError): audit.main(argv)
