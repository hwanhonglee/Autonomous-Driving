import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


SCRIPT = Path(__file__).parents[1] / "scripts/analysis/sweep_vad_trajectory_smoothing.py"
SPEC = importlib.util.spec_from_file_location("sweep_vad_trajectory_smoothing", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_endpoint_fixed_whittaker_preserves_endpoints_and_reduces_roughness():
    xy = np.asarray(
        [[0.0, 0.0], [1.0, 0.3], [2.0, -0.2], [3.0, 0.4], [4.0, 0.0]]
    )
    smoothed = MODULE.endpoint_fixed_whittaker(xy, 10.0)

    np.testing.assert_allclose(smoothed[[0, -1]], xy[[0, -1]], atol=1.0e-12)
    roughness = np.linalg.norm(np.diff(xy, n=2, axis=0))
    smoothed_roughness = np.linalg.norm(np.diff(smoothed, n=2, axis=0))
    assert smoothed_roughness < roughness


def test_endpoint_fixed_whittaker_zero_strength_is_exact_copy():
    xy = np.asarray([[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]])
    result = MODULE.endpoint_fixed_whittaker(xy, 0.0)
    np.testing.assert_array_equal(result, xy)
    assert result is not xy


@pytest.mark.parametrize("strength", [-1.0, float("nan"), float("inf")])
def test_endpoint_fixed_whittaker_rejects_invalid_strength(strength):
    with pytest.raises(ValueError):
        MODULE.endpoint_fixed_whittaker(np.zeros((3, 2)), strength)
