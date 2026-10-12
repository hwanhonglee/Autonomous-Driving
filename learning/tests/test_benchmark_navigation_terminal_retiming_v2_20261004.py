"""HH_261004 - Validate the prepared V2 same-denominator CPU benchmark without actuating or promoting."""

import importlib.util
from pathlib import Path

import pytest

from portable_e2e.contract import ContractError
from portable_e2e.navigation_terminal_retiming_v2_20261004 import NavigationTerminalRetimingV2
import test_benchmark_navigation_terminal_retiming_v1_20261004 as original_tests


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("prepared_navigation_terminal_benchmark_v2", ROOT / "tools/e2e/benchmark_navigation_terminal_retiming_v2_20261004.py")
benchmark = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(benchmark)


@pytest.mark.parametrize("support", benchmark.SUPPORTS_M)
def test_full_cpu_pipeline_uses_new_mesh_but_unchanged_witness_gate_preview_PI(support):
    case = original_tests.synthetic_case()
    result = benchmark.run_cell(case, support, original_tests.synthetic_geometry(), NavigationTerminalRetimingV2(), phase="synthetic_test")
    assert result["accepted"], result
    assert result["schema"] == benchmark.SCHEMA
    assert result["authentic_route_geometry_changed"] is False
    assert result["original_raw_fields_changed"] is False
    assert result["old_raw_NN_seconds_separate"] == .012
    assert result["old_policy_total_seconds_separate"] == .05
    assert result["end_to_end100ms_proven"] is result["vehicle_control_approved"] is False
    assert result["preview05_tracker_receipt"]["effective_speed_preview_s"] == .5
    assert result["counterfactual_PI_audit"]["integral_before"] == .3


def test_frozen_prior_all421_failures_and1660_denominator_remain_bound():
    closure = benchmark.source_closure()
    prior = benchmark.load_frozen_v1_outcomes()
    assert len(prior) == 1660
    assert sum(not row["accepted"] for row, _ in prior.values()) == 421
    assert all(not prior[(i, 10.)][0]["accepted"] for i in range(415))
    assert closure[benchmark.V1_OUTPUT + "/all_warmup_and_measured_cells.jsonl"] == benchmark.REVISION_PINS[benchmark.V1_OUTPUT + "/all_warmup_and_measured_cells.jsonl"]
    assert closure["learning/portable_e2e/terminaldecoder_v2_20261004.py"] == "596e247bc6c5f5574d5bbba0fc337d82111080a62c3e937d0f147c031d79c23e"


def test_prior_rejected_slot_is_paired_not_used_to_filter_new_slot():
    case = original_tests.synthetic_case()
    old = dict(source_index=0, declared_counterfactual_support_m=10., source_line_sha256=case["source_line_sha256"],
               accepted=False, rejection_stage="frozen_synthetic_rejection", rejection_code="SPATIAL_ACCELERATION_LIMIT", rejection_reason="retained")
    result = benchmark.run_cell(case, 10., original_tests.synthetic_geometry(), NavigationTerminalRetimingV2(),
        phase="synthetic_test", prior_outcome=(old, "d" * 64))
    assert result["accepted"] is True
    assert result["prior_frozen_v1_slot"] == dict(accepted=False, rejection_stage="frozen_synthetic_rejection",
        rejection_code="SPATIAL_ACCELERATION_LIMIT", rejection_reason="retained", result_line_sha256="d" * 64)
    old["source_line_sha256"] = "f" * 64
    with pytest.raises(ContractError, match="same-slot"):
        benchmark.run_cell(case, 10., original_tests.synthetic_geometry(), NavigationTerminalRetimingV2(),
            phase="synthetic_test", prior_outcome=(old, "d" * 64))


def test_real_gate_or_measured_speed_failure_is_retained_without_fallback():
    speed = benchmark.run_cell(original_tests.synthetic_case(speed=2.6), 2.5, original_tests.synthetic_geometry(),
                              NavigationTerminalRetimingV2(), phase="synthetic_test")
    assert speed["accepted"] is False and speed["rejection_code"] == "OBSERVED_SPEED_LIMIT"
    curve = benchmark.run_cell(original_tests.synthetic_case(speed=1.7, curve=.5), 2.5, original_tests.synthetic_geometry(),
                              NavigationTerminalRetimingV2(), phase="synthetic_test")
    assert curve["accepted"] is False and curve["rejection_stage"] == "original_runtime_trajectory_gate"
    assert "curvature" in curve["rejection_reason"]
    assert "counterfactual_control_request" not in curve


def test_v2_read_createonly_preview_guards_are_frozen_helpers_not_reimplementations():
    assert benchmark.frozen.read_canonical_regular.__code__.co_filename.endswith("benchmark_navigation_terminal_retiming_v1_20261004.py")
    assert benchmark.frozen.validate_output_path.__code__.co_filename.endswith("benchmark_navigation_terminal_retiming_v1_20261004.py")
    assert benchmark.frozen.PreparedPreview05Tracker.control.__code__.co_filename.endswith("benchmark_navigation_terminal_retiming_v1_20261004.py")
