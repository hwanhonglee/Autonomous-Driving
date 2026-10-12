"""HH_260906 - Guard pixel accounting and non-admission in Town02 p02 object QA."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "tools/e2e/audit_map_town02_p02_objects.py"
SPEC = importlib.util.spec_from_file_location("audit_map_town02_p02_objects", SOURCE)
assert SPEC is not None and SPEC.loader is not None
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def test_connected_component_pixels_and_diagonal_connectivity() -> None:
    mask = np.zeros((audit.HEIGHT, audit.WIDTH), dtype=np.bool_)
    mask[8, 9] = True
    mask[9, 10] = True
    mask[50, 70] = True
    rows = audit.components(mask, "frames/0008")
    assert sorted(row["pixels"] for row in rows) == [1, 2]
    assert sum(row["pixels"] for row in rows) == 3
    assert all(row["id"].startswith("0008_c") for row in rows)


def test_review_prefix_uses_all_pixels_and_inclusive_95pct() -> None:
    rows = [
        {"id": "a", "stem": "frames/0001", "pixels": 90, "bbox_xywh": [0, 0, 1, 1]},
        {"id": "b", "stem": "frames/0002", "pixels": 5, "bbox_xywh": [0, 0, 1, 1]},
        {"id": "c", "stem": "frames/0003", "pixels": 5, "bbox_xywh": [0, 0, 1, 1]},
    ]
    assert [row["id"] for row in audit.review_prefix(rows, 100)] == ["a", "b"]
    with pytest.raises(ValueError, match="denominator differs"):
        audit.review_prefix(rows, 99)


@pytest.fixture(scope="module")
def actual_inventory() -> dict:
    return audit.build_inventory()


def test_pinned_source_inventory_and_uncertified_gate(actual_inventory: dict) -> None:
    report = actual_inventory
    assert report["retained_tag8_pixels"] == 58074
    assert report["retained_frames"] == 232
    assert report["component_count"] == 731
    assert report["component_95pct_review_queue_count"] == 241
    assert report["component_95pct_review_queue_pixels"] == 55171
    assert report["component_95pct_review_queue_frames"] == 113
    assert report["component_95pct_review_queue_fraction"] >= 0.95
    assert report["small_sign_gate_passed"] is False
    assert report["physical_sign_objects_automatically_certified"] == 0
    assert report["object_quality_gate"] == "INCONCLUSIVE_PENDING_MANUAL_REVIEW"
    assert report["dataset_admitted"] is False
    assert report["model_trained"] is False


def test_contact_sheet_evidence_reproduces(actual_inventory: dict) -> None:
    audit.write_or_check(actual_inventory, write=False)


def _load(relative: str, name: str):
    source = ROOT / relative
    spec = importlib.util.spec_from_file_location(name, source)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_world_projection_rejects_nonrigid_camera_transform() -> None:
    geometry = _load("tools/e2e/audit_map_town02_p02_world_geometry.py", "p02_geometry")
    depth = np.zeros((audit.HEIGHT, audit.WIDTH, 4), dtype=np.uint8)
    depth[10, 12, 2] = 100
    matrix = np.eye(4)
    matrix[0, 0] = 2
    with pytest.raises(ValueError, match="not rigid"):
        geometry.world_centroid(depth, matrix, np.array([10]), np.array([12]))


def test_world_geometry_and_manual_review_fail_closed() -> None:
    geometry = _load("tools/e2e/audit_map_town02_p02_world_geometry.py", "p02_geometry_actual")
    scorer = _load("tools/e2e/score_map_town02_p02_manual_review.py", "p02_manual_score")
    world = geometry.build_geometry()
    saved_world = json.loads(geometry.OUTPUT.read_text())
    assert world == saved_world
    assert world["invalid_depth_pixels"] == 0
    assert world["zero_valid_depth_components"] == 0
    assert world["anchor_distances_and_static_drift"]["early_left_to_later_left_m"] > 29
    assert world["anchor_distances_and_static_drift"]["front_30_two_frame_drift_m"] < 0.01
    report = scorer.score()
    assert report == json.loads(scorer.OUTPUT.read_text())
    assert report["reviewed_component_count"] == 241
    assert report["reviewed_tag8_pixels"] == 55171
    assert report["retained_tag8_pixels"] == 58074
    assert report["rgb_confirmed_sign_pixels"] == 54805
    assert report["uncertain_pixels"] == 366
    assert report["invalid_non_sign_pixels"] == 0
    assert report["rgb_confirmed_distinct_physical_signs"] == 4
    assert report["rgb_confirmed_object_positive_retained_frames"] == 111
    assert report["reviewed_native_fov_frames"] == 113
    assert report["reviewed_contact_sheets"] == list(range(1, 32))
    assert report["gate_components"] == {
        "coverage_at_least_95pct": True,
        "invalid_or_uncertain_at_most_5pct": True,
        "distinct_rgb_signs_at_least_3": True,
        "confirmed_5px_frames_at_least_24": True,
    }
    assert report["object_quality_gate"] == "PASS_NOT_ADMITTED"
    assert report["small_sign_gate_passed"] is False
    assert report["dataset_admitted"] is False
    assert report["model_trained"] is False


def test_native_evidence_preserves_each_source_pixel_and_hash() -> None:
    """HH_260906 - Native audit panels cannot upscale or relabel the original sensors."""
    renderer = _load("tools/e2e/render_map_town02_p02_native_qa.py", "p02_native_evidence")
    manifest, pictures = renderer.build()
    renderer.write_or_check(manifest, pictures, write=False)
    assert len(manifest["frames"]) == 113
    assert len(manifest["pages"]) == 38
    assert manifest["native_tile_wh"] == [640, 360]
    source_by_stem = {row["stem"]: row for row in manifest["frames"]}
    for page in manifest["pages"]:
        with Image.open(renderer.OUTPUT / page["file"]) as saved:
            panel = np.asarray(saved.convert("RGB"))
        for index, stem in enumerate(page["frames"]):
            rgb_path = renderer.scout.OWNER / "mapping" / f"{stem}.png"
            semantic_path = renderer.scout.OWNER / "mapping" / f"{stem}.npz"
            row = source_by_stem[stem]
            assert renderer.sha256(rgb_path) == row["rgb_source_sha256"]
            assert renderer.sha256(semantic_path) == row["semantic_npz_source_sha256"]
            with Image.open(rgb_path) as original:
                rgb = np.asarray(original.convert("RGB"))
            with np.load(semantic_path, allow_pickle=False) as sensor:
                tags = sensor["semantic"][..., 2]
            y = 98 + index * 398
            assert np.array_equal(panel[y:y + 360, :640], rgb)
            oracle = panel[y:y + 360, 640:1280]
            assert np.array_equal(oracle[tags == 8],
                                  np.broadcast_to(renderer.scout.base.common.COLORS[
                                      renderer.scout.base.common.LABELS.index("traffic_sign")],
                                      oracle[tags == 8].shape))
            expected_overlay = rgb.copy()
            expected_overlay[tags == 8] = (255, 0, 200)
            assert np.array_equal(panel[y:y + 360, 1280:1920], expected_overlay)
