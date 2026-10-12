"""HH_261004 - Keep the complete frozen request-law tests while declaring a new numerical mesh."""

import math
from types import SimpleNamespace

import pytest
import torch

from portable_e2e import navigation_terminal_retiming_v2_20261004 as navigation
from portable_e2e.navigation_terminal_retiming_v1_20261004 import NavigationTerminalRetimingV1
from portable_e2e.path_time_decoder_terminal_v1_20261002 import TerminalDecoderError
import test_navigation_terminal_retiming_v1_20261004 as original_tests


@pytest.fixture
def frozen_law_tests_for_new_mesh(monkeypatch):
    # HH_261004 - Run unchanged assertions against V2 using a private test-only name binding; never edit frozen tests/source.
    monkeypatch.setattr(original_tests, "navigation", SimpleNamespace(NavigationTerminalRetimingV1=navigation.NavigationTerminalRetimingV2))


@pytest.mark.parametrize("name", [name for name in vars(original_tests) if name.startswith("test_")
    and not hasattr(vars(original_tests)[name], "pytestmark")
    and name != "test_frozen_three_source_closure_and_prepared_no_actuation_contract"])
def test_all_unparametrized_frozen_navigation_law_assertions_are_retained(name, frozen_law_tests_for_new_mesh):
    getattr(original_tests, name)()


@pytest.mark.parametrize("support", [17.9, 10., 5., 2.5])
@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
def test_original_four_support_and_runtime_gate_assertions(support, dtype, frozen_law_tests_for_new_mesh):
    original_tests.test_real_prepared_witness_short_support_decoder_and_original_gates(support, dtype)


@pytest.mark.parametrize("dtype", [torch.float32, torch.float64])
@pytest.mark.parametrize("mode", [0, 1, 2])
def test_original_ordinary_exact_delegate_assertions(dtype, mode, frozen_law_tests_for_new_mesh):
    original_tests.test_ordinary_branch_exact_outputs_and_original_errors(dtype, mode)


@pytest.mark.parametrize("mode", [0, 1, 2])
def test_original_residual_early_stop_assertions(mode, frozen_law_tests_for_new_mesh):
    original_tests.test_true_positive_residual_early_stop_is_not_fabricated_zero(mode)


@pytest.mark.parametrize("speed", [-.01, 2.500001, math.nan, math.inf])
def test_original_invalid_actual_speed_assertions(speed, frozen_law_tests_for_new_mesh):
    original_tests.test_invalid_observed_speed_rejects_without_clipping(speed)


@pytest.mark.parametrize("field,value", [("terminal_now_ns", 1_150_000_001), ("terminal_anchor_timestamp_ns", 999_999_999),
    ("terminal_observation_sha256", "d" * 64), ("terminal_mission_sha256", "d" * 64),
    ("terminal_route_artifact_sha256", "d" * 64), ("terminal_witness", {"status": "VERIFIED"})])
def test_original_untrusted_witness_assertions(field, value, frozen_law_tests_for_new_mesh):
    original_tests.test_unknown_stale_or_wrong_source_witness_rejects(field, value)


@pytest.mark.parametrize("support,speed", [(9.999999999818003, .5144831538200378),
    (4.9999999999313065, .953044056892395), (2.4999999999953406, .953044056892395)])
def test_diagnostic_grid_regression_synthetic_physical_fields_keep_gate(support, speed):
    # HH_261004 - These are synthetic constant fields at observed problematic D/v0 values, not a fake real NN endpoint drive.
    args, kwargs = original_tests._inputs(support=support, speed=speed, preferred=2.4, curvature=0.)
    result = navigation.NavigationTerminalRetimingV2()(*args, **kwargs)
    original_tests._assert_gates(result, args[2].item())
    witness = kwargs["terminal_witness"].witness
    assert result["navigation_stop_distance_request"].item() == min(witness.remaining_mission_arc_m, witness.provided_route_arc_m, 64.)
    spatial = result["spatial_speed"][0]
    ds = result["spatial_arc_m"][0, 1:] - result["spatial_arc_m"][0, :-1]
    rate = (spatial[1:].square() - spatial[:-1].square()) / (2 * ds)
    assert bool(((rate >= -2.000001) & (rate <= 1.500001)).all())


@pytest.mark.parametrize("support", [math.nextafter(2., 0.), math.nextafter(2., math.inf),
    math.nextafter(4., 0.), math.nextafter(4., math.inf), math.nextafter(10., 0.), math.nextafter(10., math.inf)])
def test_true_nextafter_raw_breakpoint_endpoint_does_not_invent_support(support):
    args, kwargs = original_tests._inputs(support=support, speed=1., preferred=2.4)
    # HH_261004 - A witness recomputes its real support; this is not rounded up to the requested nearby nominal endpoint.
    result = navigation.NavigationTerminalRetimingV2()(*args, **kwargs)
    original_tests._assert_gates(result, args[2].item())
    expected = min(kwargs["terminal_witness"].witness.remaining_mission_arc_m,
                   kwargs["terminal_witness"].witness.provided_route_arc_m, 64.)
    assert result["navigation_stop_distance_request"].item() == expected


def test_request_law_forward_identity_and_strict_new_source_pins(monkeypatch):
    assert navigation.NavigationTerminalRetimingV2.forward is NavigationTerminalRetimingV1.forward
    contract = navigation.prepared_contract()
    assert contract["request_law_forward_inherited_unchanged"] is True
    assert contract["model_or_weights_changed"] is False
    assert contract["vehicle_control_approved"] is False
    monkeypatch.setattr(navigation, "TERMINAL_V2_SHA256", "0" * 64)
    with pytest.raises(TerminalDecoderError, match="semantic mesh differs"):
        navigation.NavigationTerminalRetimingV2()
