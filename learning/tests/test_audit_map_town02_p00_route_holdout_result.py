"""HH_260929 - Check independent map A/B audit arithmetic and tamper detection."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest


SUBJECT_PATH = Path(__file__).resolve().parents[2] / "tools/e2e/audit_map_town02_p00_route_holdout_result.py"
SPEC = importlib.util.spec_from_file_location("audit_map_town02_p00_route_holdout_result", SUBJECT_PATH)
assert SPEC and SPEC.loader
subject = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(subject)


def test_confusion_counts_pairs_and_keeps_undefined_class_explicit():
    truth = np.zeros((360, 640), dtype=np.uint8)
    pred = truth.copy()
    truth[0, :3] = [1, 1, 3]
    pred[0, :3] = [1, 3, 1]
    matrix = subject.confusion_of(truth, pred)
    assert matrix[1, 1] == 1
    assert matrix[1, 3] == 1
    assert matrix[3, 1] == 1
    assert matrix.sum() == 360 * 640
    metrics = subject.metrics_of(matrix)
    assert metrics["iou"]["road"] == 1 / 3
    assert metrics["iou"]["lane_marking"] == 0
    assert metrics["iou"]["traffic_sign"] is None


def test_confusion_rejects_wrong_shape_and_invalid_class():
    target = np.zeros((360, 640), dtype=np.uint8)
    pred = target.copy()
    with pytest.raises(ValueError, match="ABI"):
        subject.confusion_of(target, pred[:100])
    pred[0, 0] = 7
    with pytest.raises(ValueError, match="class range"):
        subject.confusion_of(target, pred)


def test_saved_metric_comparison_rejects_one_changed_pixel_or_undefined_class():
    metrics = {"iou": {"road": 0.5, "traffic_sign": None},
               "confusion": [[10, 0], [0, 10]]}
    subject.compare_numbers(metrics, metrics.copy(), "fixture")
    changed = {"iou": dict(metrics["iou"]), "confusion": [[9, 1], [0, 10]]}
    with pytest.raises(ValueError, match="confusion"):
        subject.compare_numbers(metrics, changed, "fixture")
    changed = {"iou": {"road": 0.5, "traffic_sign": 0.0},
               "confusion": metrics["confusion"]}
    with pytest.raises(ValueError, match="traffic_sign"):
        subject.compare_numbers(metrics, changed, "fixture")


def test_gate_rejects_town01_lane_regression_despite_p02_gain():
    a = {split: {"iou": dict.fromkeys(subject.LABELS, 0.5)}
         for split in ("p02", "town01")}
    b = {split: {"iou": dict.fromkeys(subject.LABELS, 0.5)}
         for split in ("p02", "town01")}
    b["p02"]["iou"]["lane_marking"] = 0.53
    b["town01"]["iou"]["lane_marking"] = 0.48
    result = subject.recalculate_gates(a, b)
    assert result["gates"]["p02_lane_gain_at_least_2pp"] is True
    assert result["gates"]["town01_lane_drop_at_most_0p5pp"] is False
    assert result["all_gates_passed"] is False
    assert result["automatic_model_promotion"] is False
