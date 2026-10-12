"""HH_260929 - Protect the CPU checkpoint-consistency arithmetic and fail-closed cases."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[2] / "tools/e2e/verify_map_town02_p00_checkpoint_cpu.py"
SPEC = importlib.util.spec_from_file_location("map_town02_cpu_verify", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_confusion_and_iou_do_not_swap_truth_and_prediction():
    target = np.array([[[1, 1], [3, 3]]], dtype=np.uint8)
    prediction = np.array([[[1, 3], [3, 1]]], dtype=np.uint8)
    matrix = MODULE.confusion_matrix(target, prediction, 4, np)
    assert matrix[1, 1] == matrix[1, 3] == matrix[3, 1] == matrix[3, 3] == 1
    iou = MODULE.iou_from_matrix(matrix, ("a", "road", "b", "lane"), np)
    assert iou["a"] is None and iou["b"] is None
    assert iou["road"] == iou["lane"] == 1 / 3


def test_comparison_counts_pixel_changes_and_keeps_undefined_class():
    target = np.array([[[0, 0], [1, 1]]], dtype=np.uint8)
    archived = np.array([[[0, 1], [1, 1]]], dtype=np.uint8)
    inferred = np.array([[[0, 0], [1, 1]]], dtype=np.uint8)
    result = MODULE.compare_predictions(inferred, archived, target,
                                        ("other", "road", "unused"), np)
    assert result["changed_pixels"] == 1
    assert result["pixel_agreement"] == 0.75
    assert result["worst_frame_index"] == 0
    assert result["cpu_minus_archived_iou"]["unused"] is None


def test_confusion_rejects_out_of_range_labels():
    target = np.array([[[7]]], dtype=np.uint8)
    prediction = np.array([[[0]]], dtype=np.uint8)
    with pytest.raises(ValueError, match="outside frozen vocabulary"):
        MODULE.confusion_matrix(target, prediction, 7, np)
