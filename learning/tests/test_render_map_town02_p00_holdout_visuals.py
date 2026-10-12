"""HH_260929 - Check native-pixel fidelity and source linkage of sampled map A/B views."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools/e2e/render_map_town02_p00_holdout_visuals.py"
SPEC = importlib.util.spec_from_file_location("render_map_town02_p00_holdout_visuals", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
renderer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(renderer)


def test_full_fov_panels_keep_native_pixels() -> None:
    """HH_260929 - Verify all four 640x360 panels without relying on renderer labels."""
    rgb = np.zeros((360, 640, 3), dtype=np.uint8)
    rgb[:, :, 0] = np.arange(640, dtype=np.uint8)
    rgb[:, :, 1] = np.arange(360, dtype=np.uint8)[:, None]
    rgb[:, :, 2] = 173
    target = np.full((360, 640), 1, dtype=np.uint8)
    a = np.full((360, 640), 3, dtype=np.uint8)
    b = np.full((360, 640), 6, dtype=np.uint8)
    board = np.asarray(renderer.render_frame("p02", 115, rgb, target, a, b))
    assert board.shape == (936, 1280, 3)
    assert np.array_equal(board[71:431, 0:640], rgb)
    assert np.array_equal(board[71:431, 640:1280], renderer.COLORS[target])
    assert np.array_equal(board[533:893, 0:640], renderer.COLORS[a])
    assert np.array_equal(board[533:893, 640:1280], renderer.COLORS[b])


def test_gallery_hashes_and_fixed_indices_if_available() -> None:
    """HH_260929 - Verify produced files against the exact archived A/B dataset and run."""
    if not renderer.OUTPUT.is_dir() or not renderer.DATA.is_dir() or not renderer.RUN.is_dir():
        pytest.skip("large local gallery and archived run are optional in a fresh clone")
    visual_manifest = json.loads((renderer.OUTPUT / "manifest.json").read_text())
    assert visual_manifest["frame_selection"]["p02"]["indices"] == [0, 115, 231]
    assert visual_manifest["frame_selection"]["town01"]["indices"] == [0, 58, 117]
    assert visual_manifest["p02_independent_test"] is False
    assert visual_manifest["model_promotion"] is False
    assert visual_manifest["source_sha256"]["data_manifest_sha256"] == renderer.digest(
        renderer.DATA / "manifest.json")
    assert visual_manifest["source_sha256"]["run_summary_sha256"] == renderer.digest(
        renderer.RUN / "summary.json")
    for name, expected in visual_manifest["files_sha256"].items():
        assert renderer.digest(renderer.OUTPUT / name) == expected
    for split, index in (("p02", 115), ("town01", 58)):
        dataset_name = renderer.SPLITS[split]["dataset"]
        with np.load(renderer.DATA / f"{dataset_name}.npz", allow_pickle=False) as archive:
            rgb, target = archive["rgb"][index].copy(), archive["target"][index].copy()
        with np.load(renderer.RUN / "A" / f"{split}_predictions.npz",
                     allow_pickle=False) as archive:
            a = archive["prediction"][index].copy()
        with np.load(renderer.RUN / "B" / f"{split}_predictions.npz",
                     allow_pickle=False) as archive:
            b = archive["prediction"][index].copy()
        image_path = renderer.OUTPUT / f"{split}_{index:03d}_full_fov_rgb_gt_a_b.png"
        with Image.open(image_path) as image:
            board = np.asarray(image.convert("RGB"))
        assert np.array_equal(board[71:431, 0:640], rgb)
        assert np.array_equal(board[71:431, 640:1280], renderer.COLORS[target])
        assert np.array_equal(board[533:893, 0:640], renderer.COLORS[a])
        assert np.array_equal(board[533:893, 640:1280], renderer.COLORS[b])
        with Image.open(renderer.OUTPUT / f"{split}_three_fixed_frames_sampled.gif") as gif:
            assert gif.n_frames == 3
            assert gif.size == (1280, 936)
