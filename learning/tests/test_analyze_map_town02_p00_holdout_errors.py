"""HH_260929 - Verify fixed image regions and paired map error accounting."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


SUBJECT_PATH = Path(__file__).resolve().parents[2] / "tools/e2e/analyze_map_town02_p00_holdout_errors.py"
SPEC = importlib.util.spec_from_file_location("analyze_map_town02_p00_holdout_errors", SUBJECT_PATH)
assert SPEC and SPEC.loader
subject = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(subject)


def test_regions_match_preexisting_full_upper_lower_and_center_masks():
    rois = subject.roi_slices(360, 640)
    full = np.zeros((360, 640), dtype=np.uint8)
    assert full[rois["full"]].shape == (360, 640)
    assert full[rois["far"]].shape == (180, 640)
    assert full[rois["near_full"]].shape == (180, 640)
    assert full[rois["near_ego"]].shape == (180, 320)


def test_counts_separate_misses_and_extra_predictions():
    target = np.array([[3, 3, 0, 0]], dtype=np.uint8)
    prediction = np.array([[3, 0, 3, 0]], dtype=np.uint8)
    result = subject.counts(target, prediction, 3)
    assert result == {"tp": 1, "fp": 1, "fn": 1,
                      "gt_pixels": 2, "predicted_pixels": 2}
    assert subject.iou(result) == 1 / 3


def test_paired_distribution_counts_predefined_temporal_thirds():
    a = [{"tp": 1, "fp": 1, "fn": 0},
         {"tp": 1, "fp": 1, "fn": 0},
         {"tp": 1, "fp": 0, "fn": 0}]
    b = [{"tp": 1, "fp": 0, "fn": 0},
         {"tp": 1, "fp": 1, "fn": 0},
         {"tp": 0, "fp": 1, "fn": 1}]
    result = subject.paired_distribution(a, b)
    assert (result["improved"], result["regressed"], result["tied"]) == (1, 1, 1)
    assert [part["frames"] for part in result["temporal_thirds"]] == [1, 1, 1]


def test_undefined_roi_iou_is_excluded_not_changed_to_zero():
    empty = {"tp": 0, "fp": 0, "fn": 0}
    false_positive = {"tp": 0, "fp": 1, "fn": 0}
    assert subject.iou(empty) is None
    result = subject.paired_distribution([empty, false_positive], [empty, false_positive])
    assert result["frames"] == 2
    assert result["defined_pairs"] == 1
    assert result["tied"] == 1
