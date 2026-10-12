"""HH_260929 - Exercise Town05 A/B visuals using synthetic predictions only."""

import hashlib
import json

import numpy as np
from PIL import Image
import pytest

from scripts.e2e import render_map_road_lane_ab_town05 as renderer


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path, value):
    path.write_text(json.dumps(value, sort_keys=True), encoding="utf-8")


def _confusion(target, prediction):
    return np.bincount((target.astype(np.int64) * len(renderer.LABELS)
                        + prediction).ravel(),
                       minlength=len(renderer.LABELS) ** 2).reshape(
                           len(renderer.LABELS), len(renderer.LABELS)).tolist()


@pytest.fixture
def synthetic_run(tmp_path, monkeypatch):
    """HH_260929 - Make a tiny-in-count, native-resolution run without a model."""
    count = 3
    data = tmp_path / "data"
    run = tmp_path / "run"
    data.mkdir()
    run.mkdir()
    rgb = np.full((count, renderer.HEIGHT, renderer.WIDTH, 3), (40, 65, 85),
                  dtype=np.uint8)
    for index in range(count):
        rgb[index, ..., 0] += index * 20
    target = np.zeros((count, renderer.HEIGHT, renderer.WIDTH), dtype=np.uint8)
    target[:, 180:, 150:490] = renderer.ROAD
    target[:, 180:, 319:322] = renderer.LANE
    d0 = target.copy()
    d0[d0 == renderer.LANE] = renderer.ROAD
    d1 = target.copy()
    dev_path = data / "dev_town05.npz"
    np.savez_compressed(dev_path, rgb=rgb, target=target)
    manifest = {
        "schema": "portable_e2e.map_road_lane_ab_data.v1",
        "town05_split": "DEV_ONLY", "image_size_wh": [renderer.WIDTH, renderer.HEIGHT],
        "labels": list(renderer.LABELS),
        "files": {"dev_town05": {
            "file": "dev_town05.npz", "count": count, "sha256": _sha(dev_path),
            "towns": ["Town05_Opt"],
            "class_pixels": np.bincount(target.ravel(),
                                         minlength=len(renderer.LABELS)).tolist(),
        }},
    }
    manifest_path = data / "manifest.json"
    _write_json(manifest_path, manifest)
    monkeypatch.setattr(renderer, "EXPECTED_COUNT", count)
    monkeypatch.setattr(renderer, "EXPECTED_DEV_SHA256", _sha(dev_path))
    monkeypatch.setattr(renderer, "EXPECTED_MANIFEST_SHA256", _sha(manifest_path))
    _write_json(run / "started.json", {
        "schema": "portable_e2e.map_road_lane_ab_run.v1",
        "data_manifest_sha256": _sha(manifest_path),
        "automatic_promotion": False,
    })
    _write_json(run / "d0_reproduction.json", {"pass": True})
    metrics = {}
    for arm, prediction in (("D0", d0), ("D1", d1)):
        arm_dir = run / arm
        arm_dir.mkdir()
        np.savez_compressed(arm_dir / "town05_predictions.npz", prediction=prediction)
        metrics[arm] = {"deployment_approved": False,
                        "town05": {"confusion": _confusion(target, prediction)}}
        _write_json(arm_dir / "metrics.json", metrics[arm])
    _write_json(run / "summary.json", {
        "status": "A_B_COMPLETE_EXPLORATORY_NOT_PROMOTED",
        "model_promotion": False, "gpu_training_executed": True,
        "gates": {"automatic_model_promotion": False,
                  "road_lane_development_target_met": False},
        "sign_dev_qualified": False, "hd_map_complete": False,
        "learned_closed_loop_driving": False,
        "D0": metrics["D0"], "D1": metrics["D1"],
    })
    return data, run, rgb


def _render(data, run, output):
    renderer.main(["--data", str(data), "--run-root", str(run),
                   "--output", str(output)])


def test_full_fov_overlay_near_ego_and_gif_are_synthetic_only(synthetic_run, tmp_path):
    data, run, rgb = synthetic_run
    output = tmp_path / "visuals"
    _render(data, run, output)
    manifest = json.loads((output / "manifest.json").read_text())
    assert manifest["indices"] == [0, 1, 2]
    assert manifest["split"] == "Town05_Opt DEV_ONLY"
    assert manifest["independent_test_used"] is False
    assert manifest["automatic_model_promotion"] is False
    assert manifest["provenance"]["run"]["road_lane_development_target_met"] is False
    assert manifest["provenance"]["data"]["dev"] == _sha(data / "dev_town05.npz")
    assert len(manifest["files_sha256"]) == 4
    assert all(_sha(output / name) == sha for name, sha in
               manifest["files_sha256"].items())

    with Image.open(output / "town05_dev_000_road_lane_ab.png") as image:
        assert image.size == renderer.CANVAS_SIZE
        assert image.getpixel((320, 102 + 250)) == tuple(rgb[0, 250, 320])
        assert image.getpixel((960, 102 + 250)) != tuple(rgb[0, 250, 320])
        assert image.getpixel((320, 490 + 250)) != image.getpixel((960, 490 + 250))
        assert image.getpixel((960, 490 + 270)) == image.getpixel((960, 878 + 180))
    with Image.open(output / "town05_dev_road_lane_ab.gif") as gif:
        assert gif.n_frames == 3
        assert gif.size == renderer.CANVAS_SIZE


def test_changed_prediction_fails_before_output(synthetic_run, tmp_path):
    data, run, _ = synthetic_run
    path = run / "D1/town05_predictions.npz"
    with np.load(path, allow_pickle=False) as archive:
        prediction = archive["prediction"].copy()
    prediction[0, 0, 0] = renderer.ROAD
    np.savez_compressed(path, prediction=prediction)
    output = tmp_path / "visuals"
    with pytest.raises(ValueError, match="D1 prediction differs from Town05 metrics"):
        _render(data, run, output)
    assert not output.exists()


def test_changed_dev_bytes_fail_before_output(synthetic_run, tmp_path):
    data, run, _ = synthetic_run
    path = data / "dev_town05.npz"
    with path.open("ab") as stream:
        stream.write(b"changed")
    output = tmp_path / "visuals"
    with pytest.raises(ValueError, match="Town05 DEV bytes changed"):
        _render(data, run, output)
    assert not output.exists()


def test_existing_output_is_not_overwritten(synthetic_run, tmp_path):
    data, run, _ = synthetic_run
    output = tmp_path / "visuals"
    output.mkdir()
    sentinel = output / "keep.txt"
    sentinel.write_text("unchanged")
    with pytest.raises(ValueError, match="fresh visualization output required"):
        _render(data, run, output)
    assert sentinel.read_text() == "unchanged"


def test_incomplete_d0_reproduction_never_renders_d1(synthetic_run, tmp_path):
    data, run, _ = synthetic_run
    summary_path = run / "summary.json"
    summary = json.loads(summary_path.read_text())
    summary["status"] = "D0_REPRODUCTION_FAILED_D1_NOT_RUN"
    _write_json(summary_path, summary)
    output = tmp_path / "visuals"
    with pytest.raises(ValueError, match="complete exploratory D0/D1 run contract"):
        _render(data, run, output)
    assert not output.exists()


def test_only_road_and_lane_are_tinted():
    rgb = np.full((renderer.HEIGHT, renderer.WIDTH, 3), 80, dtype=np.uint8)
    labels = np.zeros((renderer.HEIGHT, renderer.WIDTH), dtype=np.uint8)
    labels[0, 0] = renderer.ROAD
    labels[0, 1] = renderer.LANE
    labels[0, 2] = renderer.LABELS.index("traffic_sign")
    shown = renderer.overlay(rgb, labels)
    assert not np.array_equal(shown[0, 0], rgb[0, 0])
    assert not np.array_equal(shown[0, 1], rgb[0, 1])
    assert np.array_equal(shown[0, 2], rgb[0, 2])
