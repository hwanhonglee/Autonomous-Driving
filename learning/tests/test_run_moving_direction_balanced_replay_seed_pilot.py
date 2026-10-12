# HH_260906 - Locate the repository independently of the relocated test directory.
"""HH_260906 - CPU-only tests for the strict F Pareto recovery gate."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

# HH_260906 - Keep logical source IDs, but open canonical nonsymlink files in the moved workspace.
from learning.portable_e2e.workspace_paths import workspace_path as _hh_source_path


ROOT = next(p for p in Path(__file__).resolve().parents if (p / ".git").exists())
PATH = _hh_source_path(ROOT, "scripts/e2e/run_moving_direction_balanced_replay_seed_pilot.py")
SPEC = importlib.util.spec_from_file_location("moving_direction_balanced_replay_runner_test", PATH)
m = importlib.util.module_from_spec(SPEC); sys.modules[SPEC.name] = m; SPEC.loader.exec_module(m)


def _subgroup():
    return {"episodes": [
        {"episode_id": m.VALIDATION_EPISODES[0]["episode_id"], "count": 337,
         "metrics": dict(m.MAXIMUM_F["town03_right_turn_dev"]),
         "metric_counts": {key: 337 for key in m.MAXIMUM_F["town03_right_turn_dev"]}},
        {"episode_id": m.VALIDATION_EPISODES[1]["episode_id"], "count": 328,
         "metrics": dict(m.MAXIMUM_F["town03_straight_dev"]),
         "metric_counts": {key: 328 for key in m.MAXIMUM_F["town03_straight_dev"]}},
    ]}


def _evaluation():
    return {"sample_count": 665,
        "metrics": dict(m.MAXIMUM_F["combined_val"]),
        "metric_counts": {key: 665 for key in m.MAXIMUM_F["combined_val"]}}


def _audit():
    return {"geometry": {"sample_count": 665,
        "selected_result": {"sample_count": 665, "geometry_pass_count": 664},
        "any_candidate_geometry_pass_count": 664,
        "all_candidates_geometry_pass_count": 664}}


def test_f_configuration_freezes_core_and_exact_exposure() -> None:
    assert m.TRAIN_CONFIG == {"seed": 20260906, "batch_size": 4,
        "learning_rate": 1e-4, "weight_decay": 1e-4, "max_steps": 8280,
        "checkpoint_interval": 414, "num_workers": 0,
        "maximum_gradient_norm": 5.0, "verify_image_sha256": True,
        "sampling_policy": "direction_balanced_cohort_replay", "domain_ratios": []}
    assert m.EXPECTED_TRAIN_STATE["samples_seen"] == 33_120
    assert m.EXPECTED_STREAM_SAMPLES == {"base": 29_500, "replay": 3_620}
    assert m.DENIALS["model_export"] is m.DENIALS["automatic_promotion"] is False


def test_exact_c_right_d_straight_combined_envelope_passes() -> None:
    gate = m.diagnostic_gate(_subgroup(), _evaluation(), _audit())
    assert gate["status"] == "PASS_SINGLE_SEED_DIAGNOSTIC"
    assert gate["pass"] is True
    assert gate["adoption_allowed"] is gate["automatic_promotion"] is False


def test_any_pareto_regression_fails_without_seed_confirmation() -> None:
    subgroup = _subgroup()
    subgroup["episodes"][0]["metrics"]["ade_6p4s_m"] += 1e-12
    gate = m.diagnostic_gate(subgroup, _evaluation(), _audit())
    assert gate["status"] == "FAIL_SINGLE_SEED_DIAGNOSTIC"
    assert gate["next_action"] == "do_not_confirm_or_promote_revise_training_data_or_sampling"


def test_combined_and_geometry_are_independent_hard_gates() -> None:
    evaluation = _evaluation(); evaluation["metrics"]["fde_6p4s_m"] += 1e-12
    assert m.diagnostic_gate(_subgroup(), evaluation, _audit())["pass"] is False
    audit = _audit(); audit["geometry"]["selected_result"]["geometry_pass_count"] = 663
    assert m.diagnostic_gate(_subgroup(), _evaluation(), audit)["pass"] is False
