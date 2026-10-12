"""HH_260929 - Guard fold isolation, one-factor weights, and sequential stop gates."""

from __future__ import annotations

import numpy as np
import pytest

from scripts.e2e import train_map_lane_weight_2x as trial


def test_nine_pair_order_and_whole_route_isolation() -> None:
    expected = [(fold, seed) for fold in ("historical", "p03", "p01")
                for seed in trial.SEEDS]
    assert [(trial.pair_contract(i)["fold"], trial.pair_contract(i)["seed"])
            for i in range(9)] == expected
    for pair_index, (name, begin, end) in enumerate(trial.FOLDS):
        train, holdout, weight_ids = trial.fold_indices(pair_index * 3)
        assert trial.pair_contract(pair_index * 3)["fold"] == name
        assert np.array_equal(holdout, np.arange(begin, end))
        assert np.array_equal(train[-232:], np.arange(497, 729))
        assert np.array_equal(weight_ids, train[:-232])
        assert set(train).isdisjoint(holdout)
        assert len(train) + len(holdout) == 729
    with pytest.raises(ValueError, match="0..8"):
        trial.pair_contract(9)


def test_lane_coefficient_is_the_only_changed_weight() -> None:
    base = np.array([0.1, 0.2, 0.3, 1.4, 1.9, 2.1, 0.9])
    changed = trial.lane_weights(base)
    assert changed[3] == 2.8
    assert np.array_equal(changed[[0, 1, 2, 4, 5, 6]],
                          base[[0, 1, 2, 4, 5, 6]])
    assert base[3] == 1.4


def test_gate_uses_preregistered_lane_and_road_thresholds() -> None:
    baseline = {"iou": {"road": 0.75, "lane_marking": 0.40}}
    passing = {"iou": {"road": 0.745, "lane_marking": 0.46}}
    failing_lane = {"iou": {"road": 0.75, "lane_marking": 0.449}}
    failing_road = {"iou": {"road": 0.739, "lane_marking": 0.46}}
    assert trial.gate(baseline, passing)["passed"] is True
    assert trial.gate(baseline, failing_lane)["passed"] is False
    assert trial.gate(baseline, failing_road)["passed"] is False


def test_failed_prior_pair_prevents_next_pair(tmp_path, monkeypatch) -> None:
    output = tmp_path / "new_run"
    monkeypatch.setattr(trial, "OUTPUT", output)
    trial.validate_previous(0, "a" * 64)
    pair = trial.pair_contract(0)
    directory = output / f"pair_00_{pair['fold']}_{pair['seed']}"
    directory.mkdir(parents=True)
    trial.write_new(directory / "summary.json", {
        "pair": pair, "status": "PAIR_FAIL_STOP", "gate": {"passed": False},
        "runner_sha256": "a" * 64, "data_manifest_sha256": trial.DATA_SHA256,
        "model_promotion": False, "file_sha256": {},
    })
    with pytest.raises(ValueError, match="prior pair failed"):
        trial.validate_previous(1, "a" * 64)


def test_confusion_iou_reports_absent_class() -> None:
    confusion = np.zeros((7, 7), dtype=np.int64)
    confusion[1, 1] = 9
    confusion[1, 3] = 1
    confusion[3, 3] = 4
    report = trial.metrics(confusion)
    assert report["iou"]["road"] == 0.9
    assert report["iou"]["lane_marking"] == 0.8
    assert report["iou"]["traffic_sign"] is None
