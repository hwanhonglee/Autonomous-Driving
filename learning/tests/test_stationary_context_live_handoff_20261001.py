"""HH_261001 - Audit associations must not manufacture identical research starts."""

import pytest
from portable_e2e.dataset import FEATURE_NAMES

from scripts.e2e.audit_stationary_context_live_handoff_20261001 import (
    COMMANDS, FEATURES, feature_comparison, handoff_history)


def test_command_order_matches_trained_abi():
    assert COMMANDS == FEATURE_NAMES[7:]


def test_history_uses_camera_receipts_not_frame_parity():
    states = [{"frame": frame} for frame in range(31)]
    cameras = [{"frame": frame} for frame in (1, 3, 6, 8, 11, 13, 16, 18, 21, 23, 26)]
    assert [row["frame"] for row in handoff_history(states, cameras, 26)] == [3, 6, 8, 11, 13, 16, 18, 21, 23, 26]
    with pytest.raises(ValueError):
        handoff_history(states, cameras, 27)


def test_exact_feature_match_does_not_round_away_small_differences():
    states = [dict(command=3, **{key: 0. for key in FEATURES.values()}) for _ in range(10)]
    refs = [dict(current_features=dict(bias=1., **{key: 0. for key in FEATURES},
                                     **{name: float(i == 3) for i, name in enumerate(COMMANDS)})) for _ in range(10)]
    assert feature_comparison(states, refs)["all_thirteen_features_exact_equal"]
    states[-1]["vx"] = 1e-12
    result = feature_comparison(states, refs)
    assert not result["all_thirteen_features_exact_equal"]
    assert result["max_abs_by_feature"]["velocity_x_mps"] == 1e-12
