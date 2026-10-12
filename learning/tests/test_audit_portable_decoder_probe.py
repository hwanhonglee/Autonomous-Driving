"""HH_260906 - Check independent numerical/gate evidence without running an optimizer or opening real image payloads."""

import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

from PIL import Image
import pytest

from scripts.e2e import audit_portable_decoder_probe as audit


def candidate_fixture():
    item = {"target_xy": [[0., 0.]] * 64, "target_speed": [0.] * 64, "current_vx_mps": 0.}
    candidate = {"decoder_xy": [[.3, .4]] * 64, "decoder_speed": [2.] * 64,
        "xy_error_by_step_m": [.5] * 64, "speed_absolute_error_by_step_mps": [2.] * 64,
        "ade_m": .5, "fde_m": .5, "maximum_xy_error_m": .5, "speed_rmse_mps": 2.,
        "maximum_speed_error_mps": 2., "final_objective_m2": 4.25}
    return item, candidate


def test_independent_position_speed_objective_and_all_64_points():
    item, candidate = candidate_fixture()
    assert audit.candidate_metrics(item, candidate)["final_objective_m2"] == pytest.approx(4.25)


@pytest.mark.parametrize("key", ["ade_m", "fde_m", "maximum_xy_error_m", "speed_rmse_mps", "maximum_speed_error_mps", "final_objective_m2"])
def test_modified_stored_summary_metric_is_rejected(key):
    item, candidate = candidate_fixture(); candidate[key] += .001
    with pytest.raises(ValueError, match="differs"):
        audit.candidate_metrics(item, candidate)


def test_modified_individual_residual_cannot_hide_behind_unchanged_mean():
    item, candidate = candidate_fixture(); candidate["xy_error_by_step_m"][15] += .001
    with pytest.raises(ValueError): audit.candidate_metrics(item, candidate)


def test_missing_point_is_not_filtered():
    item, candidate = candidate_fixture(); candidate["decoder_speed"].pop()
    with pytest.raises(ValueError): audit.candidate_metrics(item, candidate)


def test_all_optimized_latents_must_be_retained_finite_and_shaped():
    row = {"optimized_raw_latents": [[0., 0.] for _ in range(64)]}
    audit.validate_latents(row)
    row["optimized_raw_latents"][17][1] = float("inf")
    with pytest.raises(ValueError): audit.validate_latents(row)
    row["optimized_raw_latents"] = [[0., 0.]] * 63
    with pytest.raises(ValueError): audit.validate_latents(row)


def stop_candidate(speeds):
    x = 0.; points = []
    for speed in speeds:
        x += speed * .1; points.append([x, 0.])
    return {"decoder_xy": points, "decoder_speed": speeds}


def test_stop_extent_failure_requires_all_three_unchanged_conditions():
    speeds = [.12] + [0.] * 62 + [.001]
    witness = audit.stop_extent_witness({"current_vx_mps": .15}, stop_candidate(speeds))
    assert witness["predicted_path_extent_m"] < .05 and witness["maximum_predicted_speed_mps"] > .1
    assert not witness["dynamically_consistent_stop"] and witness["insufficient_extent_condition"]
    assert witness["speed_increases"] == [{"point_index": 63, "entry_speed_mps": 0., "predicted_speed_mps": .001}]


def test_monotonic_true_stop_keeps_existing_exception_without_new_rule():
    witness = audit.stop_extent_witness({"current_vx_mps": .15}, stop_candidate([.12] + [0.] * 63))
    assert witness["dynamically_consistent_stop"] and not witness["insufficient_extent_condition"]


def test_raw_future_speed_is_not_confused_with_optimized_speed_overshoot():
    item = {"current_vx_mps": .08, "target_speed": [.075] + [0.] * 63, "target_xy": [[.01, 0.]] * 64}
    witness = audit.stop_extent_witness(item, stop_candidate([.11] + [0.] * 63))
    assert witness["raw_future_maximum_planar_speed_mps"] == .075
    assert witness["maximum_predicted_speed_mps"] == .11
    assert witness["raw_future_path_extent_m"] == pytest.approx(.01)
    assert witness["insufficient_extent_condition"]


def test_subtolerance_speed_increase_uses_original_one_nanometre_per_second_slack():
    witness = audit.stop_extent_witness({"current_vx_mps": .15}, stop_candidate([.12] + [0.] * 62 + [1e-10]))
    assert witness["speed_increase_count"] == 0 and not witness["insufficient_extent_condition"]


def history_fixture():
    history = [{"batch_start_anchor_index": start, "iteration": i, "optimizer_steps_completed": i,
        "mean_objective_by_initialization": [float(i)] * 6, "maximum_objective": float(i)}
        for start in range(0, 1337, 256) for i in range(513)]
    batches = [{"batch_start_anchor_index": start, "batch_size": min(256, 1337 - start), "iterations_completed": 512,
        "status": "COMPLETE"} for start in range(0, 1337, 256)]
    return history, batches


def test_exact_3078_history_and_partial_size_final_batch():
    audit.check_history(*history_fixture())


@pytest.mark.parametrize("change", ["omit", "duplicate", "wrong_batch", "wrong_iteration", "bad_last_batch"])
def test_history_or_batch_denominator_failure(change):
    history, batches = history_fixture()
    if change == "omit": history.pop(123)
    elif change == "duplicate": history[123] = copy.deepcopy(history[122])
    elif change == "wrong_batch": history[513]["batch_start_anchor_index"] = 0
    elif change == "wrong_iteration": history[123]["optimizer_steps_completed"] = 122
    else: batches[-1]["batch_size"] = 256
    with pytest.raises(ValueError): audit.check_history(history, batches)


def test_initialization_mismatch_is_unverified_not_pass(monkeypatch):
    row = {"anchor_index": 0}
    proof, predictions = audit.reconstruct_initial([row], "0" * 64, None)
    assert proof["status"] == "UNVERIFIED_INITIALIZATION_VERSION_DIFFERENCE" and predictions is None


def test_cross_device_initial_forward_gap_stays_unverified_without_tolerance_change():
    result = audit.initial_objective_check(1.7253797054290771, 1.725406946480036)
    assert result["status"] == "INITIAL_FORWARD_UNVERIFIED"
    assert result["absolute_difference_m2"] > 2.7e-5
    assert result["comparison_tolerance"] == {"absolute": 1e-5, "relative": 1e-5}
    assert audit.initial_objective_check(1., 1.000001)["status"] == "CPU_FORWARD_MATCH_WITH_PREDECLARED_TOLERANCE"
    with pytest.raises(ValueError): audit.initial_objective_check(float("nan"), 1.)


def test_historical_git_read_is_explicitly_offline_and_sha_bound(monkeypatch):
    expected_paths = ["scripts/e2e/probe_portable_decoder_representability.py", "portable_e2e/model.py", "portable_e2e/runtime_contract.py",
        "portable_e2e/contract.py", "portable_e2e/dataset.py", "portable_e2e/torch_dataset.py", "scripts/e2e/audit_carla_raw_pre_admission.py",
        "scripts/e2e/prepare_carla_common10_dataset.py", "scripts/e2e/summarize_carla_goal_stop_trials.py", "portable_e2e/config/common_10hz_v1.contract.json"]
    identity = {"files": {path: hashlib.sha256(path.encode()).hexdigest() for path in expected_paths}}
    def check(command, cwd, env, timeout):
        assert command[:4] == ["git", "-c", "protocol.allow=never", "show"] and timeout == 10
        assert command[4].startswith(audit.COMMIT + ":")
        assert env["GIT_NO_LAZY_FETCH"] == "1" and env["GIT_ALLOW_PROTOCOL"] == "" and env["GIT_TERMINAL_PROMPT"] == "0"
        return command[4].split(":", 1)[1].encode()
    monkeypatch.setattr(audit.subprocess, "check_output", check)
    monkeypatch.setattr(audit, "OWNER_SHA", hashlib.sha256(b"scripts/e2e/run_owned_portable_decoder_probe.py").hexdigest())
    monkeypatch.setattr(audit, "SHARED_OWNER_SHA", hashlib.sha256(b"scripts/e2e/run_portable_training_campaign.py").hexdigest())
    assert audit.historical_source_proof(identity)["verified_file_count"] == 10
    identity["files"][expected_paths[0]] = "0" * 64
    with pytest.raises(ValueError): audit.historical_source_proof(identity)


def test_metadata_path_escape_and_symlinks_fail_closed(tmp_path):
    (tmp_path / "record.json").write_text("{}")
    (tmp_path / "alias.json").symlink_to(tmp_path / "record.json")
    for name in ("../record.json", "alias.json"):
        with pytest.raises(ValueError): audit.read(tmp_path, name, {})


def test_cohort_summaries_are_overlapping_descriptive_not_filtered():
    c = {"ade_m": .001, "fde_m": .002, "maximum_xy_error_m": .003, "speed_rmse_mps": .01,
        "maximum_speed_error_mps": .02, "final_objective_m2": .001, "runtime_gate_status": "FAIL", "convergence_warning": True}
    rows = [{"sample_id": "a", "best_objective_index": 0, "candidates": [c] * 6}]
    result = audit.summarize(rows, {"a": ["all", "raw_curvature_failed", "governor_coast_low"]})
    assert set(result) == {"all", "raw_curvature_failed", "governor_coast_low"}
    assert result["all"]["initialization_0"]["runtime_gate_counts"] == {"FAIL": 1}
    assert result["all"]["initialization_0"]["descriptive_max_xy_error_coverage"]["0.001"] == {"within_count": 0, "denominator": 1}


def test_actual_history_plot_resolution_and_create_only_output(tmp_path):
    history, _ = history_fixture()
    rows = [{"best_objective_index": 0, "candidates": [{"maximum_xy_error_m": .001 * (k + 1)} for k in range(6)]}]
    audit.render(tmp_path, {}, rows, history)
    assert all(Image.open(path).size == (1920, 1080) for path in tmp_path.glob("*.png"))
    with pytest.raises(ValueError): audit.write_result(tmp_path, {}, [], [])
