#!/usr/bin/env python3
"""HH_260929 - Replay three frozen RGB-model frames into separate evidence maps."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import torch

from portable_e2e.map_perception import LABELS, MapPerception
from portable_e2e.mapping import SemanticEvidenceMap
from portable_e2e.mapping_projection import project_frame, project_predicted_frame


ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / "docs/assets/validation/2026-09-29/map_learned_projection_bridge/PLAN.md"
OUTPUT = ROOT / "docs/assets/validation/2026-09-29/map_learned_projection_bridge/run_v2"
SOURCE = ROOT / "storage/experiments/validation/2026-09-22/mapping_sensor_probe/moving_straight_v1/mapping"
CHECKPOINT = ROOT / "storage/experiments/training/2026-09-26/dual_data_ai_v1/mapping_ai_v2_checkpoint/checkpoint.pt"
PLAN_SHA256 = "3a91fe16a7e3981ee1e6ad7b4b3dc21ee95d89eeced78ba7ebb72c04efada831"
INPUT_SHA256 = {
    "checkpoint": "ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380",
    "summary.json": "95e30aa5b121d668c03bae2863f7208b961db13caea44724c66ba76855d3591e",
    "records.json": "51ffdcd08f8aa89d95c64bef664b352419db1e35437ed8a8079eb11336811a65",
    "frames/0000.npz": "dd1a1004b22e8ee03c2e5a2672bbd99846f6bf1a2afe4098d36af3e9371873b8",
    "frames/0090.npz": "21a897942101c4fbe6fd0224ae0dd3b7497deeaa36b3532ee366bc693f36602e",
    "frames/0179.npz": "19a556be9a39bb1e275c96a4bf623a9a43a0c79241624f4aef3fcbe93575dd69",
    "frames/0000.png": "059daf073396ec7c1a2f6035e0d3ecb65091ba1dfd9475683d139ae24969b9ea",
    "frames/0090.png": "94bcc362a3736b60e8511118eee26df6b1fabae2d449be56b85f90893606f00b",
    "frames/0179.png": "fbcc52650c26144d5881226bf024664010c15938f3eab591f7d840ba2fed67ec",
}
SOURCE_SHA256 = {
    "learning/portable_e2e/mapping_projection.py": "c42ab7d20454475f544b8073d6a23a104a0a2bb1cc2cbb7be092fa3e871cbbee",
    "learning/portable_e2e/mapping.py": "264f9dda759f6986bde9c614e5cf3ae21680a22ae58d855db11564ceb3b3b867",
    "learning/portable_e2e/map_perception.py": "1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b",
}
INDICES = (0, 90, 179)
FOV_DEG = 90.0
STRIDE = 3
MAX_DEPTH_M = 40.0
MIN_CONFIDENCE = 0.5
LABEL_TABLE = {1: "road", 2: "sidewalk", 3: "lane_marking",
               4: "traffic_light_fixture", 5: "traffic_sign", 6: "pole"}
COLORS = {"road": (67, 92, 123), "sidewalk": (141, 88, 147),
          "lane_marking": (246, 222, 90), "traffic_light_fixture": (246, 100, 70),
          "traffic_sign": (60, 220, 180), "pole": (179, 190, 199)}
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise ValueError(reason)


def digest(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def decode_depth_m(raw: np.ndarray) -> np.ndarray:
    """HH_260929 - Decode simulator optical-axis depth, never inferred RGB depth."""
    require(raw.shape == (360, 640, 4) and raw.dtype == np.uint8,
            "frozen CARLA BGRA depth required")
    value = raw.astype(np.float64)
    return ((value[..., 2] + value[..., 1] * 256 + value[..., 0] * 65536)
            * (1000.0 / 16777215.0)).astype(np.float32)


def semantic_target(raw: np.ndarray) -> np.ndarray:
    """HH_260929 - Use CARLA tags for diagnostics and oracle map only, never inference."""
    require(raw.shape == (360, 640, 4) and raw.dtype == np.uint8,
            "frozen CARLA BGRA semantics required")
    output = np.zeros((360, 640), dtype=np.uint8)
    for tag, class_id in ((1, 1), (2, 2), (24, 3), (7, 4), (8, 5), (6, 6)):
        output[raw[..., 2] == tag] = class_id
    return output


def pixel_metrics(target: np.ndarray, prediction: np.ndarray) -> dict:
    """HH_260929 - Expose full-frame class errors; three frames are not a DEV set."""
    require(target.shape == prediction.shape == (360, 640)
            and target.dtype == prediction.dtype == np.uint8
            and int(target.max()) < len(LABELS) and int(prediction.max()) < len(LABELS),
            "seven-class 640x360 images required")
    matrix = np.bincount((target.astype(np.int64) * len(LABELS) + prediction).ravel(),
                         minlength=len(LABELS) ** 2).reshape(len(LABELS), len(LABELS))
    inter = np.diag(matrix)
    union = matrix.sum(0) + matrix.sum(1) - inter
    return {"confusion": matrix.tolist(),
            "iou": {name: float(inter[i] / union[i]) if union[i] else None
                    for i, name in enumerate(LABELS)}}


def cell_overlap(oracle: SemanticEvidenceMap, learned: SemanticEvidenceMap) -> dict:
    """HH_260929 - Compare observed occupancy and dominant tags without claiming map accuracy."""
    common = set(oracle.cells) & set(learned.cells)
    union = set(oracle.cells) | set(learned.cells)
    same_label = sum(
        max(oracle.cells[cell], key=lambda label: oracle.cells[cell][label][1]) ==
        max(learned.cells[cell], key=lambda label: learned.cells[cell][label][1])
        for cell in common
    )
    return {"common_cells": len(common), "union_cells": len(union),
            "occupancy_jaccard": len(common) / len(union) if union else None,
            "dominant_label_agreement_in_common_cells":
                same_label / len(common) if common else None}


def _font(size: int) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(FONT, size)


def render_preview(rgb: Image.Image, prediction: np.ndarray, record: dict,
                   oracle: SemanticEvidenceMap,
                   learned: SemanticEvidenceMap, index: int) -> Image.Image:
    """HH_260929 - Keep the ego centered in matching oracle/learned BEV panels."""
    canvas = Image.new("RGB", (1280, 860), (18, 25, 35))
    draw = ImageDraw.Draw(canvas)
    draw.text((15, 8), f"TOWN01 frame {index:04d} | RGB model + CARLA oracle depth/pose | NOT HD MAP",
              font=_font(21), fill="white")
    canvas.paste(rgb, (0, 52))
    # HH_260929 - Show the actual learned pixel labels, not an empty comparison panel.
    palette = np.array([(28, 35, 46)] +
                       [COLORS[LABEL_TABLE[class_id]] for class_id in range(1, len(LABELS))],
                       dtype=np.uint8)
    segmentation = Image.fromarray(palette[prediction], mode="RGB")
    canvas.paste(Image.blend(rgb, segmentation, 0.65), (640, 52))
    draw.text((10, 412), "RGB input (model only sees this image)", font=_font(17), fill="white")
    draw.text((650, 412), "Learned semantic prediction over RGB", font=_font(17), fill="white")
    yaw = math.radians(record["ego_yaw_deg"])
    cosine, sine = math.cos(yaw), math.sin(yaw)

    def position(x: float, y: float, offset: int) -> tuple[float, float]:
        dx, dy = x - record["ego_x"], y - record["ego_y"]
        forward, right = cosine * dx + sine * dy, -sine * dx + cosine * dy
        return offset + 320 + right * 6, 640 - forward * 6

    # HH_260929 - Only observed static cells are drawn; dark background stays unknown.
    for offset, evidence, title in (
        (0, oracle, "CARLA semantic/depth/pose reference"),
        (640, learned, "RGB learned semantics + CARLA depth/pose"),
    ):
        draw.rectangle((offset, 445, offset + 639, 835), fill=(22, 31, 43))
        for (ix, iy), labels in evidence.cells.items():
            label = max(labels, key=lambda key: labels[key][1])
            x, y = (ix + .5) * evidence.resolution_m, (iy + .5) * evidence.resolution_m
            u, v = position(x, y, offset)
            if offset + 2 <= u < offset + 638 and 465 <= v < 832:
                draw.rectangle((u - 1, v - 1, u + 1, v + 1), fill=COLORS[label])
        draw.rectangle((offset + 314, 628, offset + 326, 652),
                       fill=(255, 110, 65), outline="white")
        draw.polygon([(offset + 320, 616), (offset + 314, 626), (offset + 326, 626)],
                     fill="white")
        draw.text((offset + 12, 813), title, font=_font(15), fill="white")
    draw.text((15, 840), "Blue road | yellow markings | purple sidewalk | dark unknown | sampled 0000/0090/0179",
              font=_font(14), fill="white")
    return canvas


def preflight() -> tuple[list[dict], dict]:
    """HH_260929 - Fail before CPU inference if source identity or role changed."""
    require(digest(PLAN) == PLAN_SHA256, "preregistered bridge plan changed")
    require(digest(CHECKPOINT) == INPUT_SHA256["checkpoint"], "AI v2 checkpoint changed")
    for relative, expected in SOURCE_SHA256.items():
        require(digest(ROOT / relative) == expected, f"projection/model source changed: {relative}")
    for relative, expected in INPUT_SHA256.items():
        if relative != "checkpoint":
            require(digest(SOURCE / relative) == expected, f"mapping frame changed: {relative}")
    summary = json.loads((SOURCE / "summary.json").read_text(encoding="utf-8"))
    records = json.loads((SOURCE / "records.json").read_text(encoding="utf-8"))
    require(summary.get("status") == "COMPLETE_ORACLE_PLUMBING_ONLY"
            and summary.get("town") == "Town01" and summary.get("frame_count") == 180
            and summary.get("oracle_pose") is True and summary.get("oracle_depth") is True
            and summary.get("oracle_semantics") is True
            and len(records) == 180,
            "historical mapping capture contract changed")
    require(all(records[i]["stem"] == f"frames/{i:04d}" for i in INDICES)
            and all(records[a]["timestamp_s"] < records[b]["timestamp_s"]
                    for a, b in zip(INDICES, INDICES[1:])),
            "frozen frame order changed")
    require(not OUTPUT.exists() and not OUTPUT.is_symlink()
            and OUTPUT.parent.is_dir() and not OUTPUT.parent.is_symlink(),
            "fresh output directory required")
    return records, summary


def run() -> dict:
    records, source_summary = preflight()
    require(not torch.cuda.is_initialized(), "local GPU must not be initialized")
    torch.set_num_threads(2)
    checkpoint = torch.load(CHECKPOINT, map_location="cpu")
    require(tuple(checkpoint["labels"]) == tuple(LABELS)
            and checkpoint.get("deployable") is False and checkpoint.get("steps") == 2000,
            "research-only AI v2 checkpoint contract changed")
    model = MapPerception().cpu().eval().requires_grad_(False)
    model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    oracle = SemanticEvidenceMap("carla_ue:Town01", "oracle_sensor", max_cells=200000)
    learned = SemanticEvidenceMap("carla_ue:Town01", "learned_prediction", max_cells=200000)
    OUTPUT.mkdir(parents=True, exist_ok=False)
    rows = []
    for index in INDICES:
        record = records[index]
        with np.load(SOURCE / (record["stem"] + ".npz"), allow_pickle=False) as archive:
            require(set(archive.files) == {"depth", "semantic", "world_from_camera"},
                    "raw sensor archive keys changed")
            depth_raw, semantic_raw, matrix = (archive[name] for name in
                                                ("depth", "semantic", "world_from_camera"))
        rgb = Image.open(SOURCE / (record["stem"] + ".png")).convert("RGB")
        require(rgb.size == (640, 360), "frozen RGB frame size changed")
        batch = torch.from_numpy(np.asarray(rgb).transpose(2, 0, 1).copy()).unsqueeze(0)
        started = time.perf_counter()
        with torch.no_grad():
            probability = model(batch.float() / 255).softmax(1)
            confidence, predicted = probability.max(1)
        inference_seconds = time.perf_counter() - started
        predicted = predicted[0].cpu().numpy().astype(np.uint8)
        confidence = confidence[0].cpu().numpy().astype(np.float32)
        target = semantic_target(semantic_raw)
        oracle_observation = project_frame(
            depth_raw, semantic_raw, matrix, frame_id=oracle.frame_id,
            timestamp_s=record["timestamp_s"], fov_deg=FOV_DEG,
            label_table={1: "road", 2: "sidewalk", 24: "lane_marking",
                         7: "traffic_light_fixture", 8: "traffic_sign", 6: "pole"},
            stride=STRIDE, max_depth_m=MAX_DEPTH_M)
        learned_observation = project_predicted_frame(
            decode_depth_m(depth_raw), predicted, confidence, matrix,
            frame_id=learned.frame_id, timestamp_s=record["timestamp_s"],
            fov_deg=FOV_DEG, label_table=LABEL_TABLE, stride=STRIDE,
            max_depth_m=MAX_DEPTH_M, min_confidence=MIN_CONFIDENCE)
        require(oracle_observation.source == "oracle_sensor" and
                learned_observation.source == "learned_prediction",
                "oracle and learned provenance were mixed")
        oracle.update(oracle_observation)
        learned.update(learned_observation)
        # HH_260929 - Retain exact inferred labels and probabilities for later replay.
        np.savez_compressed(OUTPUT / f"frame_{index:04d}_prediction.npz",
                            predicted_class=predicted, confidence=confidence)
        preview = render_preview(rgb, predicted, record, oracle, learned, index)
        preview.save(OUTPUT / f"frame_{index:04d}_ego_centered.png")
        rows.append({"index": index, "timestamp_s": record["timestamp_s"],
                     "inference_seconds_cpu_2threads": inference_seconds,
                     "pixel_metrics": pixel_metrics(target, predicted),
                     "oracle_projected_points": len(oracle_observation.points),
                     "learned_projected_points": len(learned_observation.points),
                     "oracle_accumulated_cells": len(oracle.cells),
                     "learned_accumulated_cells": len(learned.cells)})
    # HH_260929 - Keep learned and oracle fused-cell payloads independently reviewable.
    (OUTPUT / "oracle_evidence.json").write_text(
        json.dumps(oracle.snapshot(), indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (OUTPUT / "learned_evidence.json").write_text(
        json.dumps(learned.snapshot(), indent=2, allow_nan=False) + "\n", encoding="utf-8")
    artifacts = sorted(path for path in OUTPUT.iterdir() if path.is_file())
    result = {
        "schema": "portable_e2e.learned_mapping_projection_bridge.v1",
        "status": "CPU_ONLY_THREE_FRAME_PLUMBING_NOT_HD_MAP",
        "preregistration_sha256": PLAN_SHA256,
        "script_sha256": digest(Path(__file__)),
        "input_sha256": INPUT_SHA256, "support_sha256": SOURCE_SHA256,
        "indices": list(INDICES), "source_record_count": source_summary["frame_count"],
        "frames_actually_projected": len(INDICES),
        "model_input": "RGB only",
        "projection_depth": "CARLA optical-axis oracle depth",
        "projection_pose": "CARLA world_from_camera oracle pose",
        "oracle_semantics_used_by_model": False,
        "oracle_semantics_used_by_reference_only": True,
        "stride": STRIDE, "max_depth_m": MAX_DEPTH_M,
        "min_model_confidence": MIN_CONFIDENCE, "grid_resolution_m": .2,
        "oracle_source": oracle.source, "learned_source": learned.source,
        "cell_overlap_diagnostic": cell_overlap(oracle, learned),
        "artifact_sha256": {path.name: digest(path) for path in artifacts},
        "rows": rows, "independent_dev_count": 0, "test_used": False,
        "hd_map_approved": False, "learned_closed_loop_tested": False,
        "gpu_used": False,
    }
    (OUTPUT / "summary.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"status": result["status"], "frames": len(rows),
                      "oracle_cells": len(oracle.cells),
                      "learned_cells": len(learned.cells)}, sort_keys=True))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validate-only", action="store_true",
                        help="Check preregistered source hashes and run eligibility without writing")
    args = parser.parse_args()
    if args.validate_only:
        preflight()
        print("PRECHECK_OK: sealed inputs, source hashes and fresh output path")
    else:
        run()
