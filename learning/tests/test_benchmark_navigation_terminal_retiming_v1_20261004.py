"""HH_261004 - Check benchmark provenance/census/create-only rules using explicit synthetic fixtures."""

import hashlib
import importlib.util
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from portable_e2e.contract import ContractError
from portable_e2e.navigation_terminal_retiming_v1_20261004 import NavigationTerminalRetimingV1
from portable_e2e.simulation_research import ControllerConfig, trajectory_control


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("nav_terminal_benchmark", ROOT / "tools/e2e/benchmark_navigation_terminal_retiming_v1_20261004.py")
benchmark = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(benchmark)


def synthetic_case(speed=2.4, curve=0.):
    # HH_261004 - Synthetic test values are not loaded from reserved data or claimed as real NN inference.
    controller = dict(wheelbase_m=2.819188361579792, maximum_steer_angle_rad=1.2217303432379762,
        speed_cap_mps=2.5, minimum_lookahead_m=1.5, lookahead_time_s=.8, speed_preview_s=.5,
        throttle_gain=.7, brake_gain=.7, maximum_throttle=.5)
    record = dict(inference_frame=1, raw_nn_seconds=.012, total_seconds=.05, controller=controller,
        measured_current_speed_mps=speed, raw_output=dict(curvature_knots=[curve] * 33, preferred_speed_knots=[2.4] * 33),
        owned_camera_history=dict(normalized_input_tensor_sha256=dict(ego_history="b" * 64)),
        longitudinal_pi_research=dict(integral_before=.3))
    return dict(source_index=0, source_line_sha256="a" * 64, actual_state_sha256="c" * 64,
        actual_state=dict(z=.1, x=25., y=0.), stamp_ns=1_000_000_000,
        original_camera_sha256=[str(i) * 64 for i in range(1, 7)], record=record)


def synthetic_geometry():
    return benchmark.route_geometry(dict(town="synthetic-only-map", route=[dict(x=0., y=0.), dict(x=0., y=0.), dict(x=200., y=0.)]))


@pytest.mark.parametrize("support", benchmark.SUPPORTS_M)
def test_complete_pipeline_keeps_counterfactual_and_original_timing_separate(support):
    case = synthetic_case()
    result = benchmark.run_cell(case, support, synthetic_geometry(), NavigationTerminalRetimingV1(), phase="measured")
    assert result["accepted"], result
    assert result["counterfactual_not_actual_endpoint_drive"] is True
    assert result["old_raw_NN_seconds_separate"] == .012
    assert result["old_policy_total_seconds_separate"] == .05
    assert result["measured_overhead_seconds"] >= sum(result["stage_seconds"].values())
    assert result["end_to_end100ms_proven"] is False
    assert result["vehicle_control_approved"] is False
    assert result["training_data_approved"] is False
    assert result["counterfactual_pose_provenance"]["counterfactual_not_observed"] is True
    assert result["counterfactual_pose_provenance"]["source_actual_state_sha256"] == case["actual_state_sha256"]
    assert result["counterfactual_pose_provenance"]["position_map_m"][0] == pytest.approx(200. - support)
    assert result["original_PI_before_restored"] is True
    assert result["PI_updates_in_this_independent_fixture"] == 1
    assert result["counterfactual_PI_audit"]["integral_before"] == .3
    assert result["controller"]["speed_preview_s"] == .5
    assert result["preview05_tracker_receipt"]["effective_speed_preview_s"] == .5
    assert result["counterfactual_PI_audit"]["controller_target_cap_mps"] == 2.4


def test_all_numerical_rejections_are_retained_not_turned_into_controls():
    result = benchmark.run_cell(synthetic_case(speed=2.6), 2.5, synthetic_geometry(), NavigationTerminalRetimingV1(), phase="measured")
    assert result["accepted"] is False
    assert result["rejection_stage"] == "physical_tensorization_and_navigation_decoder"
    assert result["rejection_code"] == "OBSERVED_SPEED_LIMIT"
    assert "counterfactual_control_request" not in result
    assert result["measured_overhead_seconds"] >= 0


def test_independent_gate_rejection_retains_decoded_plan_and_reason():
    result = benchmark.run_cell(synthetic_case(speed=1.7, curve=.5), 2.5, synthetic_geometry(), NavigationTerminalRetimingV1(), phase="measured")
    assert result["accepted"] is False
    assert result["rejection_stage"] == "original_runtime_trajectory_gate"
    assert "curvature" in result["rejection_reason"]
    assert "decoder_output" in result
    assert "counterfactual_control_request" not in result


def test_raw_witness_source_timestamp_and_counterfactual_pose_hash_are_separate():
    case = synthetic_case()
    mp, ms, op, os_, provenance = benchmark.counterfactual_payloads(case, 10., synthetic_geometry())
    mission, obs = json.loads(mp), json.loads(op)
    assert hashlib.sha256(mp).hexdigest() == ms
    assert hashlib.sha256(op).hexdigest() == os_
    assert mission["mission_id"].startswith("COUNTERFACTUAL")
    assert obs["pose_source_sha256"] != case["actual_state_sha256"]
    assert obs["camera_source_sha256"] == case["original_camera_sha256"]
    assert obs["anchor_timestamp_ns"] == case["stamp_ns"]
    assert obs["sequence_index"] == 0
    assert provenance["counterfactual_not_observed"] is True


def test_fresh_census_does_not_filter_out_failed_or_malformed_raw_outputs():
    rows = [(dict(held_between_camera_frames=False, emergency_reason="actual_failure", raw_output=None), "a" * 64, 0),
        (dict(held_between_camera_frames=True), "b" * 64, 1), (dict(control_source="expert_startup"), "c" * 64, 2)]
    fresh, census = benchmark.census_rows(rows)
    assert len(fresh) == 1 and fresh[0] == rows[0]
    assert census == dict(fresh_NN=1, held_NN=1, other_startup=1)


def test_source_closure_binds_actual415_and_two_prepared_navigation_files():
    closure = benchmark.source_closure()
    assert closure["learning/portable_e2e/navigation_terminal_retiming_v1_20261004.py"] == "99c90510a3bd53440d676cf0dedd8b1f3e7ea31bea1551784ef957cc77986690"
    assert closure["learning/tests/test_navigation_terminal_retiming_v1_20261004.py"] == "735b5c484dd3a15702259c15ca5aa541ae0cfbb82ee131102aa8d6e6fb3a4682"
    cases, census, geometry = benchmark.load_cases()
    assert len(cases) == benchmark.EXPECTED_FRESH_COUNT == 415
    assert census == dict(fresh_NN=415, held_NN=415, other_startup=47)
    assert geometry["total"] == pytest.approx(209.3699998276922, abs=1.e-9)
    assert cases[0]["record"]["inference_frame"] == 3655
    assert cases[-1]["record"]["inference_frame"] == 4483


def test_create_only_declaration_cannot_overwrite_a_previous_receipt(tmp_path):
    path = tmp_path / "synthetic_prospective_declaration.json"
    first = dict(status="PROSPECTIVE_BEFORE_ANY_TIMED_CELL", synthetic=True)
    receipt_sha = benchmark.write_json_create(path, first)
    assert hashlib.sha256(path.read_bytes()).hexdigest() == receipt_sha
    with pytest.raises(FileExistsError):
        benchmark.write_json_create(path, dict(status="overwrite-denied"))
    assert json.loads(path.read_bytes()) == first


def test_geometry_keeps_true_polyline_arc_and_only_removes_duplicate_points():
    geometry = synthetic_geometry()
    assert geometry["points"] == ((0., 0.), (200., 0.))
    assert geometry["total"] == 200.
    with pytest.raises(ContractError, match="nonzero"):
        benchmark.route_geometry(dict(town="synthetic", route=[dict(x=0., y=0.), dict(x=0., y=0.)]))


@pytest.mark.parametrize("intent,old_target,new_target,effective,reason", [
    (None, 1.2, 1.4, .5, None),
    (None, .04, 1.4, .3, "either_selected_preview_target_at_or_below_original_STOP_threshold"),
    (None, 1.2, 0., .3, "either_selected_preview_target_at_or_below_original_STOP_threshold"),
    (1, 1.2, 1.4, .3, "non_GO_selected_intent"),
    (True, 1.2, 1.4, .3, "non_GO_selected_intent"),
])
def test_exact_frozen_preview_whole_tracker_STOP_and_nonGO_fallback(intent, old_target, new_target, effective, reason):
    # HH_261004 - Exercise the original whole03 fallback, not merely a substituted speed target or STOP label.
    case = synthetic_case()
    cfg = ControllerConfig(**case["record"]["controller"])
    speed = [1.2] * 64
    speed[2], speed[4] = old_target, new_target
    selected = SimpleNamespace(speed_mps=tuple(speed), xy_base_m=tuple((.1 * (i + 1), .01) for i in range(64)), timestep_s=.1)
    if intent is not None:
        selected.intent = intent
    tracker = benchmark.PreparedPreview05Tracker()
    actual, receipt = tracker.control(selected, 1., cfg)
    expected = trajectory_control(selected, 1., replace(cfg, speed_preview_s=effective))
    assert actual == expected
    assert receipt["original_tracker_control"] == expected
    assert receipt["effective_speed_preview_s"] == effective
    assert receipt["fallback_reason"] == reason
    assert receipt["original_STOP_tracker_preserved"] is (reason is not None)
    assert receipt["selected_trajectory_changed"] is False
    assert tracker.ast_proof == dict(preview_selection=dict(preview_only_scalar_substitutions=4, inverse_original_AST_equal=True),
                                     trajectory_control=dict(preview_only_scalar_substitutions=0, inverse_original_AST_equal=True))
    assert tracker.namespace["_active"] is False and tracker.namespace["_proposal_session"] is None


def test_exact_frozen_private_tracker_requires_exclusive_and_single_receipt_scope():
    tracker = benchmark.PreparedPreview05Tracker()
    selected = SimpleNamespace(speed_mps=(1.,) * 64, xy_base_m=tuple((.1 * (i + 1), 0.) for i in range(64)), timestep_s=.1)
    cfg = ControllerConfig(**synthetic_case()["record"]["controller"])
    with pytest.raises(ContractError, match="exclusive observed preview"):
        tracker.namespace["trajectory_control"](selected, 1., cfg)
    tracker.namespace.update(_active=True, _proposal_session=SimpleNamespace(_preview_tracker_receipt={"already_called": True}))
    with pytest.raises(ContractError, match="must not be called twice"):
        tracker.namespace["trajectory_control"](selected, 1., cfg)


def test_source_reads_deny_file_and_parent_symlinks_and_nonregular_inputs(tmp_path):
    source_dir = tmp_path / "canonical"
    source_dir.mkdir()
    source = source_dir / "synthetic_source.txt"
    source.write_bytes(b"explicit synthetic fixture")
    assert benchmark.read_canonical_regular(source) == b"explicit synthetic fixture"
    link = source_dir / "synthetic_symlink.txt"
    link.symlink_to(source)
    with pytest.raises(ContractError, match="canonical with no symlink"):
        benchmark.read_canonical_regular(link)
    alias = tmp_path / "parent_alias"
    alias.symlink_to(source_dir, target_is_directory=True)
    with pytest.raises(ContractError, match="canonical with no symlink"):
        benchmark.read_canonical_regular(alias / source.name)
    with pytest.raises(ContractError, match="bounded regular"):
        benchmark.read_canonical_regular(source_dir)


def test_source_read_bound_cannot_be_bypassed_by_a_regular_file(tmp_path, monkeypatch):
    source = tmp_path / "synthetic_too_large.txt"
    source.write_bytes(b"not an actual source")
    monkeypatch.setattr(benchmark, "MAX_SOURCE_BYTES", 4)
    with pytest.raises(ContractError, match="bounded regular"):
        benchmark.read_canonical_regular(source)


def test_output_original_symlink_alias_and_existing_directory_are_denied(tmp_path, monkeypatch):
    synthetic_root = tmp_path / "synthetic_repository"
    diagnostics = synthetic_root / "storage/experiments/diagnostics"
    diagnostics.mkdir(parents=True)
    target = diagnostics / "existing_target"
    target.mkdir()
    alias = diagnostics / "alias"
    alias.symlink_to(target, target_is_directory=True)
    monkeypatch.setattr(benchmark, "ROOT", synthetic_root)
    with pytest.raises(ContractError, match="no symlink or alias"):
        benchmark.validate_output_path(alias / "new_output")
    with pytest.raises(ContractError, match="create-only"):
        benchmark.validate_output_path(target)
    fresh = diagnostics / "fresh_output"
    assert benchmark.validate_output_path(fresh) == fresh
