"""HH_260929 - Check the independent pixel replay counter and invalid labels."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


SOURCE = Path(__file__).resolve().parents[2] / "tools/e2e/audit_map_road_lane_ab_result.py"
SPEC = importlib.util.spec_from_file_location("audit_map_road_lane_ab_result", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_replay_confusion_counts_all_pixels():
    target = np.array([[0, 1, 1], [3, 3, 6]], dtype=np.uint8)
    prediction = np.array([[0, 1, 0], [3, 1, 6]], dtype=np.uint8)
    confusion = MODULE.replay_confusion(target, prediction)
    assert confusion.shape == (7, 7)
    assert confusion.sum() == target.size
    assert confusion[1, 0] == 1
    assert confusion[3, 1] == 1


def test_replay_confusion_rejects_bad_class_or_shape():
    target = np.array([[0]], dtype=np.uint8)
    with pytest.raises(ValueError, match="semantic class"):
        MODULE.replay_confusion(target, np.array([[7]], dtype=np.uint8))
    with pytest.raises(ValueError, match="contract"):
        MODULE.replay_confusion(target, np.zeros((1, 2), dtype=np.uint8))
