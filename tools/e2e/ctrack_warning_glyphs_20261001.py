#!/usr/bin/env python3
"""HH_261001 - Detect the specific UE sky warning, not yellow moving foliage.

This source-specific QA detector is not a general OCR or render-quality model.
It preserves full RGB bytes. Thresholds are calibrated against one historical
positive and 241 exposed negatives, not against map-model predictions.
"""

from __future__ import annotations

from functools import lru_cache
import hashlib
from io import BytesIO
from pathlib import Path

import numpy as np
from PIL import Image

VERSION = "ue_sky_screen_locked_minifont_v2_20261001"
FONT = Path("/home/a/UnrealEngine_4.26/Engine/Content/EngineMaterials/MiniFont.uasset")
FONT_SHA256 = "bce9838357fc90338e84ecff4045b26d2b70c65a8af8d435c2a80bb5e8f4d2be"
SHADER = Path("/home/a/UnrealEngine_4.26/Engine/Shaders/Private/SkyAtmosphere.usf")
SHADER_SHA256 = "b322adde69133d82d713b860ecc25ccbc74b0073d42382515303a5cf44abaab1"
MINIFONT = SHADER.with_name("MiniFontCommon.ush")
MINIFONT_SHA256 = "43cf6fe2106e1a597441f69af142f476f66373204264a7149934c815c299f61d"
LINES = ("YOUR SCENE CONTAINS A SKYDOME MESH WITH A SKY MATERIAL,",
         "BUT IT DOES NOT COVER THAT PART OF THE SCREEN... ")
MIN_CORRELATION = .85
MIN_YELLOW_SALIENCE = 40.0
MIN_FOREGROUND_BACKGROUND_CONTRAST = 35.0
MIN_MATCHED_GLYPHS_PER_LINE = 3


def digest(path):
    if not path.is_file() or path.is_symlink():
        raise ValueError(f"regular source required: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


@lru_cache(maxsize=1)
def templates():
    """HH_261001 - Decode the real 512x8 source font, excluding its thumbnail."""
    for path, expected in ((FONT,FONT_SHA256),(SHADER,SHADER_SHA256),(MINIFONT,MINIFONT_SHA256)):
        if digest(path) != expected:
            raise ValueError(f"warning shader/font changed: {path}")
    payload, position, fonts = FONT.read_bytes(), 0, []
    while (offset := payload.find(b"\x89PNG\r\n\x1a\n", position)) >= 0:
        with Image.open(BytesIO(payload[offset:])) as image:
            if image.size == (512,8):
                fonts.append(np.asarray(image.convert("RGB"))[...,0].copy())
        position = offset + 8
    if len(fonts) != 1 or set(np.unique(fonts[0])) != {0,255}:
        raise ValueError("one exact binary MiniFont source bitmap required")
    result = []
    # HH_261001 - Shader PixelPos modulo (512,256), Start=(10,10), line step=12.
    for tile_y in (0,256):
        for tile_x in (0,512):
            for line_index,line in enumerate(LINES):
                for index,character in enumerate(line):
                    if character == " ":
                        continue
                    x, y = tile_x+10+8*index, tile_y+10+12*line_index
                    if x+8 > min(tile_x+440,640) or y+8 > 360:
                        continue
                    code = ord(character)-ord("A")+10 if character.isalpha() else {",":60,".":61}[character]
                    mask = fonts[0][:,code*8:code*8+8] > 128
                    centered = mask.astype(np.float64)-mask.mean()
                    result.append((x,y,character,mask,centered,float(np.linalg.norm(centered))))
    return tuple(result)


def detect(rgb):
    """HH_261001 - Correlate yellow glyph foreground AND its negative background."""
    if rgb.shape != (360,640,3) or rgb.dtype != np.uint8:
        raise ValueError("native 640x360 uint8 RGB required")
    pixels = rgb.astype(np.float64)
    salience = np.minimum(pixels[...,0],pixels[...,1])-pixels[...,2]
    matched, per_line, maximum = [], {}, 0.0
    for x,y,character,mask,centered,norm in templates():
        patch = salience[y:y+8,x:x+8]
        on, off = float(patch[mask].mean()), float(patch[~mask].mean())
        residual = patch-patch.mean()
        denominator = norm*float(np.linalg.norm(residual))
        correlation = float(np.sum(centered*residual)/denominator) if denominator else 0.0
        if on < MIN_YELLOW_SALIENCE or on-off < MIN_FOREGROUND_BACKGROUND_CONTRAST:
            continue
        maximum = max(maximum,correlation)
        if correlation >= MIN_CORRELATION:
            matched.append({"x":x,"y":y,"glyph":character,"correlation":correlation})
            key = f"{x//512}:{y}"
            per_line[key] = per_line.get(key,0)+1
    return {"detector_version":VERSION,"warning_detected":any(
        count >= MIN_MATCHED_GLYPHS_PER_LINE for count in per_line.values()),
        "matched_glyph_count":len(matched),"matched_glyphs":matched,
        "maximum_color_qualified_correlation":maximum,
        "matched_glyphs_per_shader_line":per_line,"visual_review_required":True}


def rgb_quality(raw_rgb):
    """HH_261001 - Compatible collector interface; retain old count as descriptive only."""
    rgb = np.frombuffer(raw_rgb,dtype=np.uint8).reshape(360,640,3)
    top, sky = rgb[:32], rgb[35:80,200:440]
    result = detect(rgb)
    result.update(yellow_top32_pixels=int(np.count_nonzero(
        (top[...,0]>=130)&(top[...,1]>=120)&(top[...,2]<100))),
        white_center_sky_fraction=float(np.all(sky>=250,axis=-1).mean()),
        dark_center_sky_fraction=float(np.all(sky<20,axis=-1).mean()))
    return result


def main():
    """HH_261001 - Preserve the exposed calibration corpus and every frame score."""
    import argparse
    from datetime import datetime, timezone
    import json
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.is_symlink():
        raise ValueError("fresh calibration receipt required")
    root = Path(__file__).resolve().parents[2]
    runs = root / "storage/experiments/validation/2026-10-01"
    positive = runs / "ctrack_isolated_stationary_20261001/capture/rgb.png"
    negative = runs / "ctrack_isolated_stationary_20261001_v2/capture/rgb.png"
    pinned = ((positive,True,"02b2adaee62cb532fa50628f8cb7906b49a0b45a69b9fd81e85c3e624298264e"),
              (negative,False,"72d21e9a62a3544e9915f4de900c1abcb44b4a05a5f0e1d0a426becfee7f75f9"))
    source = runs / "ctrack_left_external_diagnostic_20261001/capture"
    records = json.loads((source/"records.json").read_text())
    if len(records) != 240:
        raise ValueError("entire moving calibration sequence required")
    corpus = list(pinned)+[(source/(row["stem"]+".png"),False,row["artifact_sha256"][".png"])
                          for row in records]
    scores = []
    for path,expected,sha in corpus:
        if digest(path) != sha:
            raise ValueError("calibration bytes differ")
        result = detect(np.asarray(Image.open(path)))
        scores.append({"path":str(path),"sha256":sha,"known_warning":expected,
                       "matches_known_label":result["warning_detected"] is expected,**result})
    report = {"schema":"portable_e2e.ctrack_warning_calibration.v2",
        "status":"PASS" if all(row["matches_known_label"] for row in scores) else "FAIL",
        "created_at_utc":datetime.now(timezone.utc).isoformat(),"version":VERSION,
        "source_sha256":{str(path):digest(path) for path in
            (Path(__file__).resolve(),FONT,SHADER,MINIFONT,source/"records.json")},
        "minimum_correlation":MIN_CORRELATION,"minimum_yellow_salience":MIN_YELLOW_SALIENCE,
        "minimum_foreground_background_contrast":MIN_FOREGROUND_BACKGROUND_CONTRAST,
        "minimum_glyphs_per_line":MIN_MATCHED_GLYPHS_PER_LINE,
        "positive_count":1,"negative_count":241,"scores":scores,
        "exposed_qa_calibration_not_independent_validation":True,
        "map_model_predictions_used":False,"simulator_launched":False,
        "source_images_modified":False,"dataset_admitted":False,"train_eligible":False}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report,stream,indent=2,sort_keys=True,allow_nan=False)
        stream.write("\n")
    print(json.dumps({"status":report["status"],"positive_count":1,"negative_count":241}))


if __name__ == "__main__":
    main()
