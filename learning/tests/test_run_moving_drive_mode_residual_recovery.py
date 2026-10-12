"""HH_260906 - Test the I runner's exact-parent, gate and create-only helpers."""

import hashlib
import json

import pytest

from scripts.e2e import run_moving_drive_mode_residual_recovery as module


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


def modes():
    row = {
        "mode_change_count": 0,
        "original_stop_selected_index_mismatch_count": 0,
        "original_stop_row_logit_mismatch_count": 0,
        "original_drive_group_max_mismatch_count": 0,
        "pass": True,
    }
    return {"train": dict(row), "val": dict(row)}


def test_strict_gate_requires_nine_metrics_three_geometry_and_two_mode_proofs():
    result = module.strict_gate(passing_metrics(), geometry(), modes())
    assert result["pass"] is True
    assert result["metric_pass_count"] == result["metric_check_count"] == 9
    assert result["geometry_pass_count"] == result["geometry_check_count"] == 3
    assert result["mode_pass_count"] == result["mode_check_count"] == 2
    assert result["promotion_eligible"] is False
    assert result["stop_guard"]["pass"] is None
    assert result["stop_guard"]["f_stop_mode_preservation_pass"] is True
    assert result["stop_guard"]["independent_stationary_performance_pass"] is None


@pytest.mark.parametrize("failure", ["metric", "geometry", "train_mode", "dev_mode"])
def test_any_absolute_gate_failure_rejects_arm(failure):
    metrics, geo, mode = passing_metrics(), geometry(), modes()
    if failure == "metric":
        metrics["combined_val"]["metrics"]["fde_6p4s_m"] += 1.0e-8
    elif failure == "geometry":
        geo["selected_geometry_pass_count"] = 663
    else:
        split = failure.removesuffix("_mode")
        if split == "dev":
            split = "val"
        mode[split]["mode_change_count"] = 1
        mode[split]["pass"] = False
    result = module.strict_gate(metrics, geo, mode)
    assert result["pass"] is False
    assert result["status"] == "FAIL_STRICT_PERFORMANCE_OR_MODE_GATE_NOT_PROMOTED"


def test_paired_gate_requires_i1_nonregression_on_all_six_values():
    i0, i1 = passing_metrics(), passing_metrics()
    result = module.paired_i1_vs_i0_gate(i0, i1)
    assert result["pass"] is True
    assert result["pass_count"] == result["check_count"] == 6
    i1["town03_straight_dev"]["metrics"]["ade_6p4s_m"] += 1.0e-8
    result = module.paired_i1_vs_i0_gate(i0, i1)
    assert result["pass"] is False
    assert result["pass_count"] == 5
    assert result["status"] == "FAIL_I1_PAIRED_NONREGRESSION_DIAGNOSTIC"


def test_exact_manifest_verifier_rejects_extra_or_changed_file(tmp_path, monkeypatch):
    root = tmp_path / "h"
    root.mkdir()
    (root / "one.json").write_text("{}\n")
    value = hashlib.sha256((root / "one.json").read_bytes()).hexdigest()
    (root / "SHA256SUMS").write_text(f"{value}  one.json\n")
    monkeypatch.setattr(
        module,
        "H_MANIFEST_SHA256",
        hashlib.sha256((root / "SHA256SUMS").read_bytes()).hexdigest(),
    )
    assert module.verify_exact_h_manifest(root)["verified_file_count"] == 1
    (root / "extra.json").write_text("{}\n")
    with pytest.raises(ValueError, match="inventory"):
        module.verify_exact_h_manifest(root)
    (root / "extra.json").unlink()
    (root / "one.json").write_text('{"changed":true}\n')
    with pytest.raises(ValueError, match="differs"):
        module.verify_exact_h_manifest(root)


def test_create_only_json_and_manifest(tmp_path):
    output = tmp_path / "output"
    module.write_new(output / "one.json", {"value": 1})
    assert json.loads((output / "one.json").read_text()) == {"value": 1}
    with pytest.raises(FileExistsError):
        module.write_new(output / "one.json", {"value": 2})
    proof = module.manifest(output)
    assert proof["file_count_excluding_manifest"] == 1
    assert (output / "SHA256SUMS").read_text().strip().endswith("  one.json")
    with pytest.raises(FileExistsError):
        module.manifest(output)


def test_fixed_parent_output_gpu_and_denials():
    assert module.OUTPUT_RELATIVE.endswith(
        "moving_drive_mode_residual_recovery_seed20260906_20260921_v1"
    )
    assert module.H_ROOT_RELATIVE.endswith(
        "moving_selector_sampling_recalibration_seed20260906_20260921_v1"
    )
    assert module.H_TRAIN_CACHE_SHA256 == (
        "a586abd602ca410e26b37785d30b371b3310e4ee31644870838c3569cc077ca0"
    )
    assert module.H_DEV_CACHE_SHA256 == (
        "cbc9040ea13f32826a701a3ccc112ffc79cdb9e6e47f3db6600f96fd83728e50"
    )
    assert module.H1_HISTORY_SHA256 == (
        "0bdbfeff48187e804c2639f9838674a398461c965bcbfb1ddac635f576f5cf92"
    )
    assert module.GPU_UUID == "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
    assert all(value is False for value in module.DENIALS.values())
    assert set(module.RIGHT_MAXIMUM) == set(module.STRAIGHT_MAXIMUM) == set(
        module.COMBINED_MAXIMUM
    ) == {"ade_6p4s_m", "fde_6p4s_m", "speed_mae_6p4s_mps"}
