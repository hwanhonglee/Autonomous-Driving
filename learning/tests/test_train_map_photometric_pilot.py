"""HH_260929 - Verify the one-factor image policy and feasible exploratory gate."""

from __future__ import annotations

import inspect

import numpy as np
import pytest

from scripts.e2e import train_map_photometric_pilot as pilot
from scripts.e2e.dual_recovery_objectives import paired_crop


class FixedRng:
    """HH_260929 - Count draws and supply a known policy example."""

    def __init__(self, probability: float, blend: float = 0.0, scale: float = 1.0):
        self.probability = probability
        self.values = iter((blend, scale))
        self.draws = []

    def random(self) -> float:
        self.draws.append("probability")
        return self.probability

    def uniform(self, low: float, high: float) -> float:
        self.draws.append((low, high))
        return next(self.values)


def test_joint_policy_known_pixels_and_single_probability_draw() -> None:
    image = np.array([[[200, 100, 0], [0, 0, 0]]], dtype=np.uint8)
    enabled_rng = FixedRng(0.49, blend=1.0, scale=0.5)
    changed, enabled = pilot.photometric(image, enabled_rng)
    assert enabled is True
    assert changed.tolist() == [[[59, 59, 59], [0, 0, 0]]]
    assert enabled_rng.draws == ["probability", (0.0, 1.0), (0.35, 1.0)]
    assert image.tolist() == [[[200, 100, 0], [0, 0, 0]]]

    disabled_rng = FixedRng(0.5)
    unchanged, enabled = pilot.photometric(image, disabled_rng)
    assert enabled is False and np.array_equal(unchanged, image)
    assert unchanged is not image
    assert disabled_rng.draws == ["probability"]
    with pytest.raises(ValueError, match="uint8 RGB"):
        pilot.photometric(image.astype(np.float32), disabled_rng)


def test_separate_rng_preserves_sampler_crop_and_label_stream() -> None:
    rgb = np.arange(36 * 64 * 3, dtype=np.uint32).reshape(36, 64, 3).astype(np.uint8)
    target = (np.arange(36 * 64).reshape(36, 64) % 7).astype(np.uint8)
    baseline_sampler = np.random.default_rng(pilot.SEED)
    candidate_sampler = np.random.default_rng(pilot.SEED)
    baseline_crop = np.random.default_rng(pilot.CROP_SEED)
    candidate_crop = np.random.default_rng(pilot.CROP_SEED)
    separate_photo = np.random.default_rng(pilot.PHOTO_SEED)
    for _ in range(25):
        baseline_ids = baseline_sampler.integers(0, 729, size=pilot.BATCH_SIZE)
        candidate_ids = candidate_sampler.integers(0, 729, size=pilot.BATCH_SIZE)
        assert np.array_equal(baseline_ids, candidate_ids)
        for _ in baseline_ids:
            base_image, base_target = paired_crop(rgb, target, baseline_crop)
            candidate_image, candidate_target = paired_crop(rgb, target, candidate_crop)
            _, _ = pilot.photometric(candidate_image, separate_photo)
            assert np.array_equal(candidate_image, base_image)
            assert np.array_equal(candidate_target, base_target)


def fixture_metrics(town01_road=0.98383, town01_lane=0.89364,
                    town05_road=0.52302, town05_lane=0.07325):
    """HH_260929 - Keep all seven named classes present in gate tests."""
    def town(road, lane):
        result = {label: 0.1 for label in pilot.LABELS}
        result.update(road=road, lane_marking=lane)
        return {"iou": result}
    return {"town01": town(town01_road, town01_lane),
            "town05": town(town05_road, town05_lane)}


def test_feasible_exploratory_gate_and_all_class_deltas() -> None:
    baseline = fixture_metrics()
    candidate = fixture_metrics(town01_road=0.97883, town01_lane=0.88864,
                                town05_road=0.51802, town05_lane=0.12326)
    report = pilot.exploratory_gate(baseline, candidate)
    assert report["passed_exploratory_only"] is True
    assert len(report["checks"]) == 4
    assert set(report["iou_delta_vs_frozen_d1"]["town01"]) == set(pilot.LABELS)
    assert report["independent_validation"] is False
    assert report["model_promotion"] is False
    assert all(baseline[town]["iou"]["lane_marking"] + 0.05 <= 1.0
               for town in ("town01", "town05"))

    candidate["town05"]["iou"]["lane_marking"] = 0.12324
    assert pilot.exploratory_gate(baseline, candidate)["checks"][
        "town05_lane_plus_5pp_vs_d1"] is False
    candidate["town05"]["iou"]["lane_marking"] = 0.12326
    candidate["town05"]["iou"]["road"] = 0.51301
    assert pilot.exploratory_gate(baseline, candidate)["checks"][
        "town05_road_drop_at_most_1pp_vs_d1"] is False


def test_preflight_never_loads_dev_before_checkpoint() -> None:
    source = inspect.getsource(pilot.preflight)
    assert "load_train(DATA / \"train_d0.npz\"" in source
    assert "load_train(DATA / \"train_d1.npz\"" in source
    assert "evaluate(" not in source
    assert "np.load(" not in source
    run_source = inspect.getsource(pilot.run)
    assert run_source.index('digest(OUTPUT / "checkpoint.pt")') < run_source.index(
        'evaluate(model, split, manifest)')
    assert pilot.OUTPUT != pilot.BASELINE
