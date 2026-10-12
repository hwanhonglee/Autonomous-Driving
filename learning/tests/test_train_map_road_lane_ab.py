"""HH_260929 - Fail closed on map A/B data, replay, and non-promotion boundaries."""

import hashlib

import numpy as np
import pytest

from scripts.e2e import train_map_road_lane_ab as trainer


def _digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _tiny_package(tmp_path, monkeypatch):
    monkeypatch.setattr(trainer, "SPLITS", {
        "train_d0": 2, "train_d1": 3, "dev_town01": 1, "dev_town05": 1,
    })
    base_rgb = np.zeros((2, 360, 640, 3), dtype=np.uint8)
    base_target = np.zeros((2, 360, 640), dtype=np.uint8)
    arrays = {
        "train_d0": (base_rgb, base_target),
        "train_d1": (np.concatenate((base_rgb, base_rgb[:1])),
                     np.concatenate((base_target, base_target[:1]))),
        "dev_town01": (base_rgb[:1], base_target[:1]),
        "dev_town05": (base_rgb[:1], base_target[:1]),
    }
    files = {}
    for name, (rgb, target) in arrays.items():
        path = tmp_path / (name + ".npz")
        np.savez_compressed(path, rgb=rgb, target=target)
        files[name] = {"file": path.name, "count": len(rgb), "sha256": _digest(path)}
    manifest = {
        "schema": "portable_e2e.map_road_lane_ab_data.v1", "files": files,
        "parent_manifest_sha256": trainer.EXPECTED_PARENT_MANIFEST_SHA256,
        "town05_split": "DEV_ONLY", "town02_split": "TRAIN_EXPLORATORY_ROAD_LANE_ONLY",
    }
    return manifest, arrays


def test_package_accepts_only_exact_d0_prefix_and_dev_scope(tmp_path, monkeypatch):
    manifest, arrays = _tiny_package(tmp_path, monkeypatch)
    loaded = trainer.validate_package(tmp_path, manifest, _digest, np,
                                      ("other", "road", "sidewalk", "lane", "light", "sign", "pole"))
    assert np.array_equal(loaded["train_d1"][0][:2], arrays["train_d0"][0])
    manifest["town05_split"] = "TRAIN"
    with pytest.raises(ValueError, match="Town05 independent development split"):
        trainer.validate_package(tmp_path, manifest, _digest, np, range(7))
    manifest["town05_split"] = "DEV_ONLY"
    manifest["files"]["train_d1"]["sha256"] = "0" * 64
    with pytest.raises(ValueError, match="train_d1 bytes changed"):
        trainer.validate_package(tmp_path, manifest, _digest, np, range(7))


def test_d0_replay_requires_all_seven_classes_within_half_point():
    names = ("other", "road", "sidewalk", "lane_marking", "traffic_light_fixture",
             "traffic_sign", "pole")
    reference = {"iou": {name: 0.5 for name in names}}
    close = {"iou": {name: 0.504 for name in names}}
    differences, ok = trainer.reproduction_gate(reference, close)
    assert ok and all(value < trainer.IOU_REPRODUCTION_TOLERANCE
                      for value in differences.values())
    close["iou"]["traffic_sign"] = 0.51
    differences, ok = trainer.reproduction_gate(reference, close)
    assert not ok and differences["traffic_sign"] > trainer.IOU_REPRODUCTION_TOLERANCE


def test_road_lane_gain_does_not_promote_regressed_seven_class_model():
    reference = {"iou": {"road": 0.98, "lane_marking": 0.90, "traffic_sign": 0.10}}
    d0 = {"town05": {"iou": {"road": 0.60, "lane_marking": 0.30}}}
    d1 = {
        "town05": {"iou": {"road": 0.90, "lane_marking": 0.60}},
        "town01": {"iou": {"road": 0.98, "lane_marking": 0.90, "traffic_sign": 0.09}},
    }
    gates = trainer.development_gates(d0, d1, reference)
    assert gates["road_lane_development_target_met"] is True
    assert gates["all_town01_classes_non_regressing"] is False
    assert gates["automatic_model_promotion"] is False
    assert gates["independent_test_used"] is False
