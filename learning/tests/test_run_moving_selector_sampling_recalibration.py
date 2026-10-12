"""HH_260906 - Test the H runner's strict gate and create-only evidence helpers."""

import json

import pytest

from scripts.e2e import run_moving_selector_sampling_recalibration as module


def metric_row(values, count):
    return {
        "sample_count": count,
        "metrics": dict(values),
        "metric_counts": {name: count for name in values},
        "selected_histogram": [0] * 12,
        "composite_oracle_histogram": [0] * 12,
        "selected_composite_oracle_agreement_count": 0,
    }


def passing_metrics():
    return {
        "town03_right_turn_dev": metric_row(module.RIGHT_MAXIMUM, 337),
        "town03_straight_dev": metric_row(module.STRAIGHT_MAXIMUM, 328),
        "combined_val": metric_row(module.COMBINED_MAXIMUM, 665),
    }


def geometry():
    return {
        "selected_geometry_pass_count": 664,
        "any_candidate_geometry_pass_count": 664,
        "all_candidates_geometry_pass_count": 664,
        "sample_count": 665,
    }


def test_strict_gate_requires_all_nine_metrics_and_three_geometry_checks():
    result = module.strict_gate(passing_metrics(), geometry())
    assert result["pass"] is True
    assert result["metric_pass_count"] == result["metric_check_count"] == 9
    assert result["geometry_pass_count"] == result["geometry_check_count"] == 3
    assert result["promotion_eligible"] is False
    assert result["stop_guard"] == {
        "status": "NOT_EVALUABLE_NO_SEALED_STATIONARY_DEV_EPISODE",
        "sample_count": 0,
        "required_for_promotion": True,
        "pass": None,
        "reason": "The fixed DEV contract contains only Town03 right-turn and straight episodes; no stationary episode may be inferred, substituted, or selected post hoc.",
    }


@pytest.mark.parametrize(
    "group,metric",
    [
        ("town03_right_turn_dev", "ade_6p4s_m"),
        ("town03_straight_dev", "fde_6p4s_m"),
        ("combined_val", "speed_mae_6p4s_mps"),
    ],
)
def test_any_metric_regression_fails_gate(group, metric):
    values = passing_metrics()
    values[group]["metrics"][metric] += 1.0e-8
    result = module.strict_gate(values, geometry())
    assert result["pass"] is False
    assert result["status"] == "FAIL_STRICT_PERFORMANCE_GATE_NOT_PROMOTED"
    assert result["metric_pass_count"] == 8


def test_any_geometry_regression_fails_gate():
    values = geometry()
    values["selected_geometry_pass_count"] = 663
    result = module.strict_gate(passing_metrics(), values)
    assert result["pass"] is False
    assert result["geometry_pass_count"] == 2


def test_paired_gate_requires_h1_nonregression_on_all_six_values():
    h0 = passing_metrics()
    h1 = passing_metrics()
    result = module.paired_h1_vs_h0_gate(h0, h1)
    assert result["pass"] is True
    assert result["pass_count"] == result["check_count"] == 6
    h1["combined_val"]["metrics"]["fde_6p4s_m"] += 1.0e-8
    result = module.paired_h1_vs_h0_gate(h0, h1)
    assert result["pass"] is False
    assert result["pass_count"] == 5
    assert result["status"] == "FAIL_H1_PAIRED_NONREGRESSION_GATE"


def test_create_only_json_and_manifest(tmp_path):
    root = tmp_path / "output"
    module.write_new(root / "one.json", {"value": 1})
    assert json.loads((root / "one.json").read_text()) == {"value": 1}
    with pytest.raises(FileExistsError):
        module.write_new(root / "one.json", {"value": 2})
    proof = module.manifest(root)
    assert proof["file_count_excluding_manifest"] == 1
    line = (root / "SHA256SUMS").read_text().strip()
    assert line.endswith("  one.json")
    with pytest.raises(FileExistsError):
        module.manifest(root)


def test_fixed_output_and_safety_denials():
    assert module.OUTPUT_RELATIVE.endswith(
        "moving_selector_sampling_recalibration_seed20260906_20260921_v1"
    )
    assert module.F_CHECKPOINT_SHA256 == (
        "a551570aa6fc1697e9dcfd9fa714b2c1066f95e2194a5fefa9b0d9f3c20baf8c"
    )
    assert module.GPU_UUID == "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
    assert module.REPO_RELATIVE == (
        "source_worktrees/"
        "moving_selector_recalibration_h_seed20260906_20260921_v1"
    )
    assert all(value is False for value in module.DENIALS.values())
    assert set(module.RIGHT_MAXIMUM) == set(module.STRAIGHT_MAXIMUM) == set(
        module.COMBINED_MAXIMUM
    ) == {"ade_6p4s_m", "fde_6p4s_m", "speed_mae_6p4s_mps"}
