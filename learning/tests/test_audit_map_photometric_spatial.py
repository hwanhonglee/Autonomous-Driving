"""HH_260929 - Verify pooled spatial IoU math and frozen-prediction replay checks."""

import numpy as np
import pytest

from scripts.e2e import audit_map_photometric_spatial as spatial


def test_class_region_keeps_false_negative_and_false_positive_separate():
    target = np.array([[[3, 3], [1, 1]]], dtype=np.uint8)
    predicted = np.array([[[3, 1], [3, 1]]], dtype=np.uint8)
    row = spatial.class_region(target, predicted, 3)
    assert (row["true_positive"], row["false_positive"], row["false_negative"]) == (1, 1, 1)
    assert row["iou"] == pytest.approx(1 / 3)
    assert row["recall"] == pytest.approx(0.5)


def test_confusion_rejects_out_of_range_and_replays_exact_matrix():
    target = np.array([[[0, 3], [1, 3]]], dtype=np.uint8)
    predicted = np.array([[[0, 3], [3, 1]]], dtype=np.uint8)
    matrix = spatial.confusion(target, predicted)
    assert matrix[0][0] == 1 and matrix[3][3] == 1
    assert matrix[1][3] == 1 and matrix[3][1] == 1
    predicted[0, 0, 0] = 7
    with pytest.raises(ValueError, match="seven-class"):
        spatial.confusion(target, predicted)


def test_region_geometry_is_explicit_and_nested():
    frame = np.zeros((1, 360, 640), dtype=np.uint8)
    assert frame[spatial.REGIONS["full"]].shape == (1, 360, 640)
    assert frame[spatial.REGIONS["far"]].shape == (1, 180, 640)
    assert frame[spatial.REGIONS["near_ego"]].shape == (1, 180, 320)
