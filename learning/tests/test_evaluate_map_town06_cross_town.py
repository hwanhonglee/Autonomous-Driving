"""HH_260906 - Pure CPU checks for the fixed Town06 evaluation's mapping rules."""
import numpy as np
import pytest

from scripts.e2e.evaluate_map_town06_cross_town import class_metrics, retained_indices, semantic_target
from scripts.e2e.check_map_parent_dev_cpu_parity import confusion_matrix


def test_spacing_uses_last_retained_ego_position():
    rows = [{'ego_x': x, 'ego_y': 0.0} for x in (0, .1, .2, .26, .4, .52)]
    assert retained_indices(rows).tolist() == [0, 3, 5]


def test_semantic_table_preserves_other_for_unmapped_tags():
    semantic = np.zeros((360, 640, 4), dtype=np.uint8)
    semantic[..., 2] = 3
    semantic[0, 0, 2] = 8
    semantic[0, 1, 2] = 24
    target = semantic_target(semantic)
    assert (target[0, 0], target[0, 1], target[0, 2]) == (5, 3, 0)
    with pytest.raises(ValueError):
        semantic_target(semantic[..., :3])


def test_iou_precision_recall_from_fixed_confusion():
    confusion = np.zeros((7, 7), dtype=np.int64)
    confusion[0, 0] = 10
    confusion[0, 5] = 2
    confusion[5, 5] = 3
    confusion[5, 0] = 1
    result = class_metrics(confusion)
    sign = result['classes']['traffic_sign']
    assert sign['iou'] == pytest.approx(3 / 6)
    assert sign['precision'] == pytest.approx(3 / 5)
    assert sign['recall'] == pytest.approx(3 / 4)
    assert result['classes']['lane_marking']['iou'] is None


def test_parity_confusion_is_exact_pixel_count():
    target = np.array([[[0, 5], [5, 0]]], dtype=np.uint8)
    prediction = np.array([[[0, 0], [5, 5]]], dtype=np.uint8)
    confusion = confusion_matrix(target, prediction)
    assert confusion[0, 0] == confusion[0, 5] == confusion[5, 0] == confusion[5, 5] == 1
    assert confusion.sum() == target.size
