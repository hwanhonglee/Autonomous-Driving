"""HH_261001 - Preserve known positive, fixed-screen glyphs, and foliage negatives."""
from pathlib import Path

import numpy as np
from PIL import Image

from scripts.e2e import ctrack_warning_glyphs_20261001 as warning

ROOT = Path(__file__).resolve().parents[2]
RUNS = ROOT / "storage/experiments/validation/2026-10-01"


def test_known_shader_warning_positive_and_corrected_stationary_negative():
    for suffix, expected, sha in (("", True,
            "02b2adaee62cb532fa50628f8cb7906b49a0b45a69b9fd81e85c3e624298264e"),
            ("_v2", False,"72d21e9a62a3544e9915f4de900c1abcb44b4a05a5f0e1d0a426becfee7f75f9")):
        path = RUNS / ("ctrack_isolated_stationary_20261001"+suffix) / "capture/rgb.png"
        assert warning.digest(path) == sha
        assert warning.detect(np.asarray(Image.open(path)))["warning_detected"] is expected


def test_solid_yellow_and_mispositioned_font_are_not_warning():
    image = np.zeros((360,640,3),dtype=np.uint8)
    image[:32] = (255,220,50)
    assert not warning.detect(image)["warning_detected"]
    image[:] = 0
    for x,y,_,mask,_,_ in warning.templates():
        image[y:y+8,x:x+8][mask] = (255,190,63)
    assert warning.detect(image)["warning_detected"]
    assert not warning.detect(np.roll(image,40,axis=0))["warning_detected"]


def test_all_240_moving_frames_have_no_screen_locked_warning():
    source = RUNS / "ctrack_left_external_diagnostic_20261001/capture"
    import json
    records = json.loads((source/"records.json").read_text())
    assert len(records) == 240
    for row in records:
        path = source/(row["stem"]+".png")
        assert warning.digest(path) == row["artifact_sha256"][".png"]
        assert not warning.detect(np.asarray(Image.open(path)))["warning_detected"]
