"""HH_260929 - Check DEV error accounting with tiny synthetic masks only."""

import csv
import json

import numpy as np
import pytest

from scripts.e2e import analyze_map_road_lane_ab_town05_errors as errors


def _synthetic_masks():
    target = np.zeros((2, 4, 4), dtype=np.uint8)
    target[0, 0, 0] = errors.verified.LANE
    target[0, 3, 1] = errors.verified.LANE
    target[0, 2, 2] = errors.verified.ROAD
    target[0, 3, 2] = errors.verified.ROAD
    target[0, 0, 3] = errors.verified.LABELS.index("sidewalk")
    d0 = np.zeros_like(target)
    d0[0, 0, 0] = errors.verified.LANE
    d0[0, 3, 2] = errors.verified.ROAD
    d0[0, 0, 3] = errors.verified.ROAD
    d1 = np.zeros_like(target)
    d1[0, 3, 1] = errors.verified.LANE
    d1[0, 0, 1] = errors.verified.LANE
    d1[0, 3, 2] = errors.verified.ROAD
    d1[0, 0, 2] = errors.verified.ROAD
    d1[0, 0, 0] = errors.verified.ROAD
    return target, {"D0": d0, "D1": d1}


def test_full_far_and_near_ego_accounting_is_exact():
    target, predictions = _synthetic_masks()
    summary, rows = errors.analyze(target, predictions)
    assert len(rows) == 2 * 2 * 4
    assert summary["region_xyxy"] == {
        "full": [0, 0, 4, 4], "far": [0, 0, 4, 2],
        "near_full": [0, 2, 4, 4], "near_ego": [1, 2, 3, 4],
    }
    d0 = summary["arms"]["D0"]
    d1 = summary["arms"]["D1"]
    assert d0["full"]["lane"]["micro_iou"] == 0.5
    assert d1["full"]["lane"]["micro_iou"] == pytest.approx(1 / 3)
    assert d0["far"]["lane"]["micro_recall"] == 1
    assert d1["far"]["lane"]["micro_recall"] == 0
    assert d0["near_ego"]["lane"]["micro_recall"] == 0
    assert d1["near_ego"]["lane"]["micro_recall"] == 1
    assert d0["full"]["lane"]["frame_iou"]["defined_frames"] == 1
    assert d1["full"]["lane"]["zero_predicted_frames"] == 1
    assert summary["paired_frame_deltas"]["full"]["lane_iou"]["d1_lt_d0"] == 1


def test_road_false_positive_denominator_and_oracle_class_are_explicit():
    target, predictions = _synthetic_masks()
    summary, _ = errors.analyze(target, predictions)
    d0 = summary["arms"]["D0"]["full"]["road"]
    d1 = summary["arms"]["D1"]["full"]["road"]
    assert d0["nonroad_px"] == d1["nonroad_px"] == 30
    assert d0["fp"] == 1 and d0["fp_nonroad_rate"] == pytest.approx(1 / 30)
    assert d1["fp"] == 2 and d1["fp_nonroad_rate"] == pytest.approx(2 / 30)
    assert d0["fp_by_oracle_class"]["sidewalk"] == 1
    assert d1["fp_by_oracle_class"]["other"] == 1
    assert d1["fp_by_oracle_class"]["lane_marking"] == 1


def test_fresh_outputs_keep_all_per_frame_rows(tmp_path):
    target, predictions = _synthetic_masks()
    summary, rows = errors.analyze(target, predictions)
    output = tmp_path / "analysis"
    hashes = errors.write_outputs(output, summary, rows)
    assert len(hashes["summary_sha256"]) == 64
    assert json.loads((output / "summary.json").read_text())["frame_count"] == 2
    with (output / "per_frame.csv").open(newline="") as stream:
        saved = list(csv.DictReader(stream))
    assert len(saved) == 16
    assert saved[0]["frame_index"] == "0"
    assert saved[0]["lane_iou"] == "0.5"
    with pytest.raises(ValueError, match="fresh error-analysis output required"):
        errors.write_outputs(output, summary, rows)


def test_rejects_unaligned_or_nonsemantic_prediction():
    target, predictions = _synthetic_masks()
    predictions["D1"][0, 0, 0] = 7
    with pytest.raises(ValueError, match="aligned D0/D1 semantic predictions"):
        errors.analyze(target, predictions)
