"""HH_260929 - Check score counts distinguish confident errors from true positives."""

import numpy as np
import pytest

from scripts.e2e.audit_map_town05_confident_errors import audit_frame


def test_confident_errors_count_score_only_without_selecting_threshold():
    target = np.array([[1, 1, 3], [3, 1, 3]], dtype=np.uint8)
    predicted = np.array([[2, 1, 2], [3, 6, 0]], dtype=np.uint8)
    scores = np.array([[.95, .8, .91], [.4, .6, .49]], dtype=np.float32)
    result = audit_frame(target, predicted, scores)
    assert result["road_as_sidewalk"] == {
        "pixels": 1, "score_at_least_0_5": 1, "score_at_least_0_9": 1,
        "median_score": pytest.approx(.95), "p90_score": pytest.approx(.95)}
    assert result["lane_missed"]["pixels"] == 2
    assert result["lane_missed"]["score_at_least_0_5"] == 1
    assert result["lane_missed"]["score_at_least_0_9"] == 1
    assert result["lane_correct"]["score_at_least_0_5"] == 0
    assert result["all_wrong"]["pixels"] == 4
    with pytest.raises(ValueError, match="finite probability"):
        audit_frame(target, predicted, np.full_like(scores, np.nan))
