"""HH_260929 - Check TRAIN source binding and exact diagnostic bin semantics."""

from __future__ import annotations

import numpy as np
from PIL import Image
import pytest

from scripts.e2e import audit_map_lane_train as audit


def test_unknown_semantic_tags_remain_other_and_depth_is_bgra() -> None:
    tags = np.array([[0, 1, 24, 255]], dtype=np.uint8)
    assert audit.target_from_tags(tags).tolist() == [[0, 1, 3, 0]]
    depth = np.zeros((1, 4, 4), dtype=np.uint8)
    depth[0, :, 0] = [0, 0, 0, 255]
    meters = audit.optical_depth_m(depth, np.ones((1, 4), dtype=bool))
    assert meters[0] == 0
    assert meters[-1] == pytest.approx(255 * 65536 * 1000 / 16777215)


def test_totals_preserve_pixel_and_region_denominators() -> None:
    rgb = np.zeros((360, 640, 3), dtype=np.uint8)
    target = np.zeros((360, 640), dtype=np.uint8)
    target[180:, :] = 1
    target[200, 200] = target[300, 550] = 3
    rgb[target == 3] = [240, 220, 20]
    depth = np.zeros((360, 640, 4), dtype=np.uint8)
    depth[..., 1] = 65
    depth[..., 2] = 137
    tags = np.zeros((360, 640), dtype=np.uint8)
    tags[target == 1] = 1
    tags[target == 3] = 24
    totals = audit.Totals()
    totals.add(rgb, target, depth, tags, 2.2)
    report = totals.report()
    assert sum(report["class_pixels"]) == 360 * 640
    assert report["class_pixels"][1] == 180 * 640 - 2
    assert report["road_lane"]["lane_marking"]["near_ego_pixels"] == 1
    assert sum(map(sum, report["road_lane"]["lane_marking"]["grid_yx_pixels"])) == 2
    assert report["road_lane"]["lane_marking"]["rgb_mean"] == [240, 220, 20]
    assert report["road_lane"]["lane_marking"]["depth_invalid_0p1_10_20_40_80_160_1000m"][1] == 2


def test_raw_pair_rejects_semantic_misalignment(tmp_path) -> None:
    rgb = np.zeros((360, 640, 3), dtype=np.uint8)
    target = np.zeros((360, 640), dtype=np.uint8)
    target[200, 200] = 3
    semantic = np.zeros((360, 640, 4), dtype=np.uint8)
    semantic[200, 200, 2] = 24
    depth = np.zeros_like(semantic)
    camera = np.eye(4)
    camera[2, 3] = 2.2
    rgb_path = tmp_path / "frame.png"
    sensor_path = tmp_path / "frame.npz"
    Image.fromarray(rgb).save(rgb_path)
    np.savez_compressed(sensor_path, depth=depth, semantic=semantic,
                        world_from_camera=camera)
    rgb_hash, sensor_hash = audit.digest(rgb_path), audit.digest(sensor_path)
    _, tags, camera_z = audit.validate_sensor_pair(
        rgb, target, rgb_path, sensor_path, rgb_hash, sensor_hash)
    assert tags[200, 200] == 24 and camera_z == 2.2
    changed = target.copy()
    changed[200, 200] = 1
    with pytest.raises(ValueError, match="packaged target differs"):
        audit.validate_sensor_pair(rgb, changed, rgb_path, sensor_path,
                                   rgb_hash, sensor_hash)
    with pytest.raises(ValueError, match="raw sensor hash changed"):
        audit.validate_sensor_pair(rgb, target, rgb_path, sensor_path,
                                   "0" * 64, sensor_hash)
