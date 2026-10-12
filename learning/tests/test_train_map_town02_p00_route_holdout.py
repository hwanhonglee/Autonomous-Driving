"""HH_260929 - Protect the exploratory p00 A/B data and immutable metric gates."""

from __future__ import annotations

import hashlib

import numpy as np
import pytest

from scripts.e2e import train_map_town02_p00_route_holdout as subject


LABELS = ("other", "road", "sidewalk", "lane_marking",
          "traffic_light_fixture", "traffic_sign", "pole")


def _fixture(tmp_path, monkeypatch):
    monkeypatch.setattr(subject, "COUNTS", {
        "train_a": 1, "train_b": 2, "holdout_town02_p02": 1, "dev_town01": 1})
    base = np.zeros((1, 360, 640, 3), dtype=np.uint8)
    base[:, 10:30, 10:30] = 120
    target = np.zeros((1, 360, 640), dtype=np.uint8)
    target[:, 10:30, 10:30] = 1
    rows = {
        "train_a": (base, target),
        "train_b": (np.concatenate((base, base + 1)),
                    np.concatenate((target, target))),
        "holdout_town02_p02": (base + 2, target),
        "dev_town01": (base + 3, target),
    }
    files = {}
    for name, (rgb, truth) in rows.items():
        path = tmp_path / f"{name}.npz"
        np.savez_compressed(path, rgb=rgb, target=truth)
        files[name] = {"file": path.name,
                       "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                       "count": len(rgb),
                       "class_pixels": np.bincount(truth.ravel(), minlength=7).tolist()}
    manifest = {"schema": subject.SCHEMA,
                "parent_manifest_sha256": subject.EXPECTED_PARENT_MANIFEST_SHA256,
                "dataset_admitted": False, "independent_dev_approved": False,
                "exploratory_ab_train_authorized": True,
                "provenance": {"research_scope_sha256": subject.EXPECTED_RESEARCH_SCOPE_SHA256},
                "files": files}
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    return rows, manifest, digest


def test_package_requires_exact_a_prefix_and_unapproved_split(tmp_path, monkeypatch):
    _, manifest, digest = _fixture(tmp_path, monkeypatch)
    arrays = subject.validate_package(tmp_path, manifest, digest, np, LABELS)
    assert arrays["train_b"][0].shape[0] == 2
    manifest["independent_dev_approved"] = True
    with pytest.raises(ValueError, match="claimed approval"):
        subject.validate_package(tmp_path, manifest, digest, np, LABELS)


def test_personal_output_rejects_traversal_and_symlink(tmp_path):
    home = tmp_path / "home" / "pro6000"
    workspace = home / "personal/hwanhong/portable_e2e"
    diagnostics = workspace / "runs/diagnostics"
    dataset = workspace.parent / "dataset"
    repo = workspace / "source_worktrees/moving_recovery_k_20260922_v1"
    for path in (diagnostics, dataset, repo):
        path.mkdir(parents=True)
    subject.validate_personal_paths(workspace, repo, dataset / "data_v1",
                                    diagnostics / "published_parent",
                                    diagnostics / "fresh_output")
    with pytest.raises(ValueError, match="escaped its exact personal parent"):
        subject.validate_personal_paths(workspace, repo, dataset / "data_v1",
                                        diagnostics / "published_parent",
                                        diagnostics / "../../shared/probe")
    (diagnostics / "link").symlink_to(tmp_path, target_is_directory=True)
    with pytest.raises(ValueError, match="resolves outside"):
        subject.validate_personal_paths(workspace, repo, dataset / "data_v1",
                                        diagnostics / "published_parent",
                                        diagnostics / "link")


def test_package_detects_b_prefix_change_and_hash_tamper(tmp_path, monkeypatch):
    rows, manifest, digest = _fixture(tmp_path, monkeypatch)
    path = tmp_path / "train_b.npz"
    changed = rows["train_b"][0].copy()
    changed[0, 0, 0, 0] = 99
    np.savez_compressed(path, rgb=changed, target=rows["train_b"][1])
    with pytest.raises(ValueError, match="bytes differ"):
        subject.validate_package(tmp_path, manifest, digest, np, LABELS)
    manifest["files"]["train_b"]["sha256"] = digest(path)
    with pytest.raises(ValueError, match="frozen Town01 TRAIN prefix"):
        subject.validate_package(tmp_path, manifest, digest, np, LABELS)


def test_reproduction_and_route_probe_gates_are_fixed():
    baseline = {"iou": {name: 0.50 for name in LABELS}}
    replay = {"iou": {name: 0.50 for name in LABELS}}
    differences, good = subject.reproduction_gate(baseline, replay)
    assert good and max(differences.values()) == 0
    replay["iou"]["road"] = 0.4949
    assert subject.reproduction_gate(baseline, replay)[1] is False
    a = {"p02": {"iou": dict.fromkeys(LABELS, 0.5)},
         "town01": {"iou": dict.fromkeys(LABELS, 0.5)}}
    b = {"p02": {"iou": dict.fromkeys(LABELS, 0.5)},
         "town01": {"iou": dict.fromkeys(LABELS, 0.5)}}
    b["p02"]["iou"]["lane_marking"] = 0.52
    b["p02"]["iou"]["road"] = 0.49
    gates = subject.probe_gates(a, b)
    assert gates["all_gates_passed"] and not gates["automatic_model_promotion"]
    b["town01"]["iou"]["traffic_sign"] = None
    gates = subject.probe_gates(a, b)
    assert not gates["all_gates_passed"]
    assert not gates["gates"]["town01_all_seven_drop_at_most_0p5pp"]


def test_confusion_undefined_class_is_not_silent_pass():
    confusion = np.zeros((7, 7), dtype=np.int64)
    confusion[0, 0] = 10
    metrics = subject.metrics_from_confusion(confusion, LABELS, np)
    assert metrics["iou"]["other"] == 1.0
    assert metrics["iou"]["road"] is None
    assert metrics["true_class_pixels"] == [10, 0, 0, 0, 0, 0, 0]
