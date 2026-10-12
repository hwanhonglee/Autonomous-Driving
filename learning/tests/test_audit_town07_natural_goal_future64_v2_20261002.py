"""HH_261002 - Synthetic proof that v2 rebinding preserves all frozen true64/physical/denial checks."""

import copy
import hashlib
import json
from pathlib import Path

import pytest

from learning.tests import test_audit_town07_natural_goal_future64_20261002 as fixtures
from scripts.e2e import audit_town07_natural_goal_future64_v2_20261002 as audit


def v2_fixture():
    plan, manifest, states, cameras = fixtures.fixture()
    plan["schema"] = audit.TASK_SCHEMA
    for state in states:state["goal_stop"]["profile_id"] = audit.PROFILE
    return plan, manifest, states, cameras


def test_source_freeze_and_AST_revision_changes_only_literal_provider_run_id():
    pins = audit.verify_frozen()
    assert pins[str(audit.ROOT/audit.FROZEN_AUDIT_PATH)] == audit.FROZEN_AUDIT_SHA
    assert all(pins[str(audit.ROOT/name)] == value for name, value in audit.V2_SOURCE_PINS.items())
    with audit.activated():
        reader = audit.inspector_revision()
        assert reader.__name__ == "inspect"
        assert reader.__globals__["natural_task"] is audit.task
        assert "v2" in reader.__code__.co_consts and "v1" not in reader.__code__.co_consts
        assert reader.__globals__["audit_core"] is audit.previous.audit_core
        assert "torch" not in reader.__globals__ and "carla" not in reader.__globals__


@pytest.mark.parametrize("error", [False, True])
def test_all_aliases_and_functions_restore_after_normal_or_exceptional_scope(error):
    names = ("SCHEMA", "TASK_SCHEMA", "PROFILE", "SOURCE_PINS", "natural_task")
    saved = {name:getattr(audit.previous, name) for name in names}
    functions = {name:getattr(audit.previous, name) for name in (
        "future_window", "native_intervals", "audit_core", "intent_witness", "denial_witness", "governor_receipt_binding")}
    try:
        with audit.activated():
            assert audit.previous.natural_task is audit.task and audit.previous.PROFILE == audit.PROFILE
            assert all(getattr(audit.previous, name) is value for name, value in functions.items())
            with pytest.raises(ValueError):
                with audit.activated():pass
            if error:raise OSError("retained synthetic failure")
    except OSError:assert error
    assert all(getattr(audit.previous, name) is value for name, value in saved.items())
    assert all(getattr(audit.previous, name) is value for name, value in functions.items())
    assert not audit._active and not audit.task._active


def test_active_teacher_provider_cannot_be_attached_to_or_reset():
    with audit.task.activated():
        with pytest.raises(ValueError):
            with audit.activated():pass
        assert audit.task._active
    assert not audit.task._active


def test_identical_physical_truth_and_all_fixed_windows_under_v2_names():
    original = fixtures.fixture()
    old_summary, old_windows = audit.previous.audit_core(*original)
    inputs = v2_fixture(); before = copy.deepcopy(inputs)
    with audit.activated():summary, windows = audit.previous.audit_core(*inputs)
    assert inputs == before and windows == old_windows
    assert summary == dict(old_summary, schema=audit.SCHEMA)
    assert summary["selected_anchor_count"] == 360 and summary["true_future_point_count"] == 23040
    assert summary["research_training_authorized"] is False and summary["training_data_approved"] is False
    assert summary["research_holdout_evaluation_authorized"] is False and summary["vehicle_control_approved"] is False
    assert all(row["curvature_label_valid"] is False and row["xy"] == [0.,0.]
        and row["stationary_domain_metrics"]["xy_curvature"] == [None, False] for row in windows[-1]["true_future_rows"])


@pytest.mark.parametrize("change", ["v1_plan", "v1_state", "camera_gap", "camera_order", "timestamp", "crop", "tail_anchor", "hold"])
def test_unchanged_exact_version_camera_and_membership_guards_reject(change):
    plan, manifest, states, cameras = v2_fixture()
    if change == "v1_plan":plan["schema"] = "carla.actual_natural_goal_stop_teacher.20261002.v1"
    elif change == "v1_state":states[400]["goal_stop"]["profile_id"] = "natural_goal_stop_manual1_2p40_hold0p05.20261002.v1"
    elif change == "camera_gap":cameras.pop(120)
    elif change == "camera_order":cameras[120]["camera_order"].reverse()
    elif change == "timestamp":cameras[120]["source_timestamps"][audit.previous.contract.CAMERA_ORDER[0]] += .001
    elif change == "crop":cameras[120]["moving_task_selected"] = False
    elif change == "tail_anchor":cameras[-1]["moving_task_selected"] = True
    else:states[823]["goal_stop"]["hold_duration_sec"] = 1.95
    with audit.activated(), pytest.raises(ValueError):audit.previous.audit_core(plan, manifest, states, cameras)


@pytest.mark.parametrize("marker", ["training_data_approved", "development_only"])
def test_native_boolean_denials_are_never_lifted_for_v2(marker):
    inputs = v2_fixture();inputs[1]["result"][marker] = marker == "development_only"
    before = copy.deepcopy(inputs)
    with audit.activated():summary, _ = audit.previous.audit_core(*inputs)
    assert inputs == before and summary["native_explicit_denial_present"] is True
    assert summary["raw_native_denials_overridden"] is False and summary["training_data_approved"] is False


def test_same_native2p9_and3_bounds_hard_deceleration_denominators_and_stationary_masks():
    plan, manifest, states, cameras = v2_fixture()
    states[767]["vx"], states[768]["vx"], states[769]["vx"] = 1.5371, .721294, .000851
    manifest["status"] = "failed";manifest["result"]["goal_stop_quality"]["status"] = "FAIL"
    manifest["result"]["moving_task"]["task_complete"] = False
    with audit.activated():summary, windows = audit.previous.audit_core(plan, manifest, states, cameras)
    assert summary["source_capture_failed"] is True and summary["failed_fixed_window_count"] > 0
    assert summary["protected_unique_native_interval_failure_count"] >= 2
    assert any(r["native_scalar_dvdt_exceeds3"] and r["native_scalar_dvdt_exceeds_original_physical2p9"]
        for r in summary["all_raw_native_interval_failures"])
    assert summary["selected_anchor_count"] == 360 and len(windows) == 360
    assert summary["true_future_point_count"] == 360*64
    rows = [fixtures.state(i, speed=1. if i == 0 else 1.1475) for i in range(140)]
    for row in rows:row["goal_stop"]["profile_id"] = audit.PROFILE
    timeline = [(audit.previous.stamp(row), row) for row in rows]
    with audit.activated():
        intervals = audit.previous.native_intervals(timeline, 0)
        future = audit.previous.future_window(timeline, rows[0])
    assert intervals[0]["actual_scalar_dvdt_mps2"] == pytest.approx(2.95)
    assert intervals[0]["native_scalar_dvdt_exceeds3"] is False
    assert intervals[0]["native_scalar_dvdt_exceeds_original_physical2p9"] is True
    assert future[0]["stationary_domain_metrics"]["speed_acceleration"] == [pytest.approx(1.475), False]


def test_short_GO_stays_GO_and_true_missing_futures_stay_in_denominator():
    inputs = v2_fixture();inputs[2][400]["goal_stop"]["remaining_route_arc_m"] = 10.
    with audit.activated():
        summary, windows = audit.previous.audit_core(*inputs)
        short = next(row for row in windows if row["source_frame"] == 1400)
        tail = audit.previous.future_window([(audit.previous.stamp(row), row) for row in inputs[2][-10:]], inputs[2][-10])
    assert summary["GO_anchors_incompatible_with_existing_18m_support"] == 1
    assert short["current_semantic_witness"]["intent"] == "GO"
    assert len(tail) == 64 and tail[-1]["xy"] is None and tail[-1]["valid"] is False


def revision_fixture():
    plan = audit.task.fixed_contract("v2", "pilot_00")
    config = copy.deepcopy(plan["natural_goal_configuration"])
    config["effective_control"] = dict(normal_brake_cap=.01, normal_throttle_cap=.2,
        basic_agent_emergency_brake_modified=False)
    manifest = dict(control_source="Disclosed_BasicAgent_natural_goal_stop_gentle_normal_brakes_manual1_v2",
        capture_contract=dict(goal_stop_profile=config, teacher_controller=dict(normal_brake_cap=.01,
            normal_throttle_cap=.2, prospective_desired_deceleration_mps2=.2,
            normal_launch_controller_revision=audit.TASK_SCHEMA+".normal_control", manual_gear_shift=True, gear=1,
            native_float32_globals_modified=False, governor_math_changed=False,
            individual_factor_causal_effect_claim=False, inherited_BasicAgent_emergency_unchanged=True)))
    return plan, manifest


@pytest.mark.parametrize("change", [None, "old_config", "old_cap", "Boolean_gear", "hazard", "unknown_control", "global_float", "factor_claim"])
def test_actual_controller_and_governor_revision_requires_true_settings_not_renaming(change):
    plan, manifest = revision_fixture();capture = manifest["capture_contract"]
    if change == "old_config":capture["goal_stop_profile"]["desired_deceleration_mps2"] = 1.5
    elif change == "old_cap":capture["goal_stop_profile"]["effective_control"]["normal_brake_cap"] = .1
    elif change == "Boolean_gear":capture["teacher_controller"]["gear"] = True
    elif change == "hazard":capture["teacher_controller"]["inherited_BasicAgent_emergency_unchanged"] = False
    elif change == "unknown_control":manifest["control_source"] = "renamed_v1"
    elif change == "global_float":capture["teacher_controller"]["native_float32_globals_modified"] = True
    elif change == "factor_claim":capture["teacher_controller"]["individual_factor_causal_effect_claim"] = True
    if change is None:
        result = audit.teacher_revision_witness(plan, manifest)
        assert result["actual_native_config_and_installed_normal_caps_bind_declared_v2"]
        assert result["individual_factor_causal_effect_claim"] is False
    else:
        with pytest.raises((ValueError, KeyError)):audit.teacher_revision_witness(plan, manifest)


def closure_fixture(tmp_path, monkeypatch):
    workspace, carla = tmp_path/"workspace", tmp_path/"carla"
    workspace.mkdir();carla.mkdir();episode = workspace/"pilot_00/episode";episode.mkdir(parents=True)
    source, external, asset = workspace/"tools/owner.py", carla/"agents/basic.py", workspace/"route.json"
    paths = ((source, episode.parent/"source/tools/owner.py"),
        (external, episode.parent/"source_external/agents/basic.py"), (asset, episode.parent/"source/route.json"))
    hashes = {}
    for index, (live, archive) in enumerate(paths):
        payload = ("fixed-%d" % index).encode();live.parent.mkdir(parents=True, exist_ok=True)
        archive.parent.mkdir(parents=True, exist_ok=True);live.write_bytes(payload);archive.write_bytes(payload)
        hashes[str(live)] = hashlib.sha256(payload).hexdigest()
    monkeypatch.setattr(audit, "ROOT", workspace);monkeypatch.setattr(audit, "CARLA_ROOT", carla)
    monkeypatch.setattr(audit.task, "FROZEN_V1", {});monkeypatch.setattr(audit, "V2_SOURCE_PINS", {"tools/owner.py":hashes[str(source)]})
    plan = dict(source_sha256={"tools/owner.py":hashes[str(source)]},
        external_source_sha256={str(external):hashes[str(external)]}, asset_sha256={str(asset):hashes[str(asset)]})
    return plan, episode, paths


@pytest.mark.parametrize("change", [None, "source_live", "source_archive", "external_live", "external_archive", "asset_live", "asset_archive", "missing_new", "escaping_source", "missing_closure"])
def test_every_declared_live_archive_source_API_asset_is_first_read_pinned(tmp_path, monkeypatch, change):
    plan, episode, paths = closure_fixture(tmp_path, monkeypatch)
    mutation = {"source_live":paths[0][0], "source_archive":paths[0][1], "external_live":paths[1][0],
        "external_archive":paths[1][1], "asset_live":paths[2][0], "asset_archive":paths[2][1]}
    if change in mutation:mutation[change].write_bytes(b"corrupted")
    elif change == "missing_new":plan["source_sha256"] = {"tools/other.py":"a"*64}
    elif change == "escaping_source":plan["source_sha256"]["../escape.py"] = "a"*64
    elif change == "missing_closure":plan["external_source_sha256"] = {}
    pins = {}
    if change is None:
        result = audit.declared_closure(plan, episode, pins)
        assert result["all_declared_live_and_archive_bytes_rechecked"] and len(pins) == 6
        assert result["import_graph_recomputed_by_this_adapter"] is False
        paths[0][1].write_bytes(b"later corruption")
        with pytest.raises(ValueError):audit.previous.recheck(pins)
    else:
        with pytest.raises(ValueError):audit.declared_closure(plan, episode, pins)


def test_new_CLI_create_only_raw_exclusion_and_failure_exit_no_permission(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(audit, "ROOT", tmp_path)
    episode = tmp_path/"pilot_00/episode";episode.mkdir(parents=True)
    summary = dict(schema=audit.SCHEMA, failed_fixed_window_count=1, full_raw_event_count=0,
        source_capture_failed=True, source_artifact_sha256={}, research_training_authorized=False,
        training_data_approved=False, vehicle_control_approved=False)
    monkeypatch.setattr(audit, "inspect", lambda *args:(summary, [dict(source_frame=1, true_future_rows=[])], {}))
    base = ["--episode", str(episode), "--case-plan", str(tmp_path/"plan.json"), "--case-plan-sha256", "a"*64]
    with pytest.raises(ValueError):audit.main(base+["--output", str(episode.parent/"new")])
    output = tmp_path/"new_diagnostic"
    assert audit.main(base+["--output", str(output)]) == 2
    written = json.loads((output/"summary.json").read_text())
    assert written == summary and written["research_training_authorized"] is False
    assert len((output/"all_fixed_anchor_true_future64.jsonl").read_text().splitlines()) == 1
    with pytest.raises(ValueError):audit.main(base+["--output", str(output)])
    assert episode.is_dir();capsys.readouterr()
