"""HH_260929 - Check exact RGB/luma accounting for the map appearance audit."""

import numpy as np
import pytest

from scripts.e2e import audit_map_appearance_shift as audit


def test_luma_bins_and_class_mask() -> None:
    rgb = np.zeros((1, 360, 640, 3), dtype=np.uint8)
    target = np.zeros((1, 360, 640), dtype=np.uint8)
    rgb[0, 0, 0] = (255, 255, 255)
    rgb[0, 0, 1] = (0, 0, 0)
    rgb[0, 0, 2] = (127, 127, 127)
    target[0, 0, :3] = 3
    result = audit.accumulate(rgb, target, 3, 0, 1)
    assert result["pixels"] == 3
    assert result["rgb_mean"] == [127.333, 127.333, 127.333]
    assert result["luma_bins_0_31_32_63_64_95_96_127_128_191_192_255"] == [1, 0, 0, 1, 0, 1]
    assert result["luma_0_63_fraction"] == pytest.approx(1 / 3, abs=1e-6)


def test_invalid_slice_is_rejected() -> None:
    rgb = np.zeros((1, 360, 640, 3), dtype=np.uint8)
    target = np.zeros((1, 360, 640), dtype=np.uint8)
    with pytest.raises(ValueError, match="ABI or slice"):
        audit.accumulate(rgb, target, 1, 0, 2)
