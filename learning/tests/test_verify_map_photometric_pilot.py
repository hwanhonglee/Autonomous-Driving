"""HH_260929 - Exercise independent pixel replay of the archived one-policy result."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
from PIL import Image, ImageChops
import pytest


ROOT = Path(__file__).resolve().parents[2]
PILOT = ROOT / "docs/assets/validation/2026-09-29/map_photometric_pilot"
SPEC = importlib.util.spec_from_file_location("hh_map_photo_verify", PILOT / "verify_result.py")
assert SPEC is not None and SPEC.loader is not None
verify = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(verify)


def test_independent_confusion_and_corruption_detection() -> None:
    target = np.zeros((1, 360, 640), dtype=np.uint8)
    prediction = target.copy()
    target[0, 0, :3] = [1, 1, 3]
    prediction[0, 0, :3] = [1, 3, 3]
    result = verify.independent_metrics(target, prediction)
    assert result["confusion"][1][1] == 1
    assert result["confusion"][1][3] == 1
    assert result["confusion"][3][3] == 1
    assert result["iou"]["road"] == 0.5
    assert result["iou"]["lane_marking"] == 0.5
    verify.same_metrics(result, result)
    broken = dict(result)
    broken["confusion"] = [[0] * 7 for _ in range(7)]
    with pytest.raises(ValueError, match="confusion differs"):
        verify.same_metrics(broken, result)


def test_all_four_frozen_dev_confusions_and_gate_replay() -> None:
    report = verify.replay(
        ROOT / "storage/experiments/training/2026-09-29/map_road_lane_ab_data_v2",
        ROOT / "storage/experiments/training/2026-09-29/map_road_lane_ab_run_v1",
        PILOT / "remote_run_v1", PILOT / "SHA256SUMS")
    assert report["status"] == "INDEPENDENT_D1_AND_PHOTO_DEV_PIXEL_REPLAY_PASS"
    assert report["run_files_sha_verified"] == 15
    assert report["dev_confusions_replayed"] == 4
    assert report["gate"]["town05_lane_plus_5pp_vs_d1"] is True
    assert report["gate"]["town01_lane_drop_at_most_1pp_vs_d1"] is False
    assert report["model_promotion"] is False


def test_fixed_town05_gif_has_three_distinct_full_fovs_and_aligned_near_ego() -> None:
    """HH_260929 - The lower inset must magnify the same upper-panel pixels."""
    gif = PILOT / "visuals/town05_dev_000_115_231_four_panel_near_ego.gif"
    assert verify.sha256(gif) == "cc7dae21d79241444e84a1bf6221ab94b6c1672f60b3a3d3515558c7ed642fe5"
    assert f"](visuals/{gif.name})" in (PILOT / "README.md").read_text(encoding="utf-8")
    with Image.open(gif) as stream:
        assert stream.n_frames == 3
        assert stream.size == (2560, 800)
        assert stream.info["loop"] == 0
        frames = []
        for index in range(3):
            stream.seek(index)
            assert stream.info["duration"] == 1800
            frame = stream.convert("RGB").copy()
            for column in range(4):
                x = column * 640 + 320
                assert frame.getpixel((x, 80 + 270)) == frame.getpixel((x, 480 + 135))
            frames.append(frame)
    assert ImageChops.difference(frames[0], frames[1]).getbbox() is not None
    assert ImageChops.difference(frames[1], frames[2]).getbbox() is not None
