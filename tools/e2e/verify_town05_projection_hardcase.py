#!/usr/bin/env python3
"""HH_260929 - Independently replay Town05 repeated-DEV map evidence from saved predictions."""

from __future__ import annotations

from collections import Counter
import json
from pathlib import Path

import numpy as np
from PIL import Image

from scripts.e2e import audit_learned_mapping_bridge as base
from scripts.e2e import audit_town05_projection_hardcase as hard
from scripts.e2e import site_split_guard
from portable_e2e.mapping import SemanticEvidenceMap
from portable_e2e.mapping_projection import project_frame, project_predicted_frame


def check(condition: bool, reason: str) -> None:
    if not condition:
        raise ValueError(reason)


def verify() -> dict:
    """HH_260929 - Check byte provenance and reconstructed pixels/cells without inference."""
    split = json.loads(site_split_guard.REGISTRY.read_text(encoding="utf-8"))
    check(site_split_guard.check("Town05_Opt", "map", "reuse_exploratory_dev", split)
          and not site_split_guard.check("Town05_Opt", "map", "offline_train_scout", split)
          and split["independent_dev_sites"] == [],
          "Town05 exploratory-only site role changed")
    result = json.loads((hard.OUTPUT / "summary.json").read_text(encoding="utf-8"))
    check(result["schema"] == "portable_e2e.town05_mapping_projection_hardcase.v1"
          and result["status"] == "REPEATED_DEV_THREE_FRAME_DIAGNOSTIC_NOT_HD_MAP"
          and result["indices"] == list(hard.INDICES)
          and result["frames_actually_projected"] == 3
          and result["independent_dev_count"] == 0
          and result["repeated_dev_only"] is True
          and result["test_used"] is False
          and result["hd_map_approved"] is False
          and result["learned_closed_loop_tested"] is False
          and result["gpu_used"] is False,
          "exploratory repeated-DEV result contract changed")
    check(result["preregistration_sha256"] == base.digest(hard.PLAN)
          and result["script_sha256"] == base.digest(Path(hard.__file__))
          and result["capture_audit_sha256"] == base.digest(hard.AUDIT)
          and result["input_sha256"]["checkpoint"] == base.digest(base.CHECKPOINT),
          "plan, script, audit or checkpoint hash changed")
    for relative, expected in result["support_sha256"].items():
        check(base.digest(hard.ROOT / relative) == expected,
              f"support source hash changed: {relative}")
    for relative, expected in result["input_sha256"].items():
        if relative != "checkpoint":
            check(base.digest(hard.SOURCE / relative) == expected,
                  f"frozen input hash changed: {relative}")
    check(set(result["artifact_sha256"]) ==
          {f"frame_{index:04d}_{suffix}" for index in hard.INDICES
           for suffix in ("ego_centered.png", "prediction.npz")} |
          {"oracle_evidence.json", "learned_evidence.json"},
          "eight persisted artifacts required")
    for name, expected in result["artifact_sha256"].items():
        check(base.digest(hard.OUTPUT / name) == expected, f"artifact hash changed: {name}")
    records = json.loads((hard.SOURCE / "records.json").read_text(encoding="utf-8"))
    oracle = SemanticEvidenceMap("carla_ue:Town05_Opt", "oracle_sensor", max_cells=200000)
    learned = SemanticEvidenceMap("carla_ue:Town05_Opt", "learned_prediction", max_cells=200000)
    for index, row in zip(hard.INDICES, result["rows"], strict=True):
        check(row["index"] == index, "fixed source order changed")
        with np.load(hard.SOURCE / f"frames/{index:04d}.npz", allow_pickle=False) as raw:
            depth, semantic, pose = (raw[name] for name in
                                     ("depth", "semantic", "world_from_camera"))
        with np.load(hard.OUTPUT / f"frame_{index:04d}_prediction.npz",
                     allow_pickle=False) as inferred:
            check(set(inferred.files) == {"predicted_class", "confidence"},
                  "saved prediction keys changed")
            prediction, confidence = inferred["predicted_class"], inferred["confidence"]
        check(prediction.shape == confidence.shape == (360, 640)
              and prediction.dtype == np.uint8 and confidence.dtype == np.float32,
              "saved prediction type or shape changed")
        check(base.pixel_metrics(base.semantic_target(semantic), prediction) ==
              row["pixel_metrics"], "published seven-class confusion changed")
        timestamp = records[index]["timestamp_s"]
        o = project_frame(depth, semantic, pose, frame_id=oracle.frame_id,
                          timestamp_s=timestamp, fov_deg=base.FOV_DEG,
                          label_table={1: "road", 2: "sidewalk", 24: "lane_marking",
                                       7: "traffic_light_fixture", 8: "traffic_sign", 6: "pole"},
                          stride=base.STRIDE, max_depth_m=base.MAX_DEPTH_M)
        l = project_predicted_frame(
            base.decode_depth_m(depth), prediction, confidence, pose,
            frame_id=learned.frame_id, timestamp_s=timestamp, fov_deg=base.FOV_DEG,
            label_table=base.LABEL_TABLE, stride=base.STRIDE,
            max_depth_m=base.MAX_DEPTH_M, min_confidence=base.MIN_CONFIDENCE)
        check(o.source == "oracle_sensor" and l.source == "learned_prediction",
              "provenance mixed")
        oracle.update(o)
        learned.update(l)
        check((len(o.points), len(l.points), len(oracle.cells), len(learned.cells)) ==
              (row["oracle_projected_points"], row["learned_projected_points"],
               row["oracle_accumulated_cells"], row["learned_accumulated_cells"]),
              "published projected point/cell counts changed")
        with Image.open(hard.OUTPUT / f"frame_{index:04d}_ego_centered.png") as preview:
            check(preview.size == (1280, 860)
                  and preview.getpixel((320, 640)) == (255, 110, 65)
                  and preview.getpixel((960, 640)) == (255, 110, 65),
                  "vehicle not centered in both BEV panels")
    oracle_payload = json.loads(
        (hard.OUTPUT / "oracle_evidence.json").read_text(encoding="utf-8"))
    learned_payload = json.loads(
        (hard.OUTPUT / "learned_evidence.json").read_text(encoding="utf-8"))
    check(oracle.snapshot() == oracle_payload,
        "oracle fused evidence changed")
    check(learned.snapshot() == learned_payload,
        "learned fused evidence changed")
    check(base.cell_overlap(oracle, learned) == result["cell_overlap_diagnostic"],
          "published overlap diagnostic changed")
    # HH_260929 - Count the actual fused-cell failure modes behind the rendered gap.
    dominant = lambda cell: max(cell["labels"], key=lambda label:
                                cell["labels"][label]["confidence_sum"])
    oracle_cells = {(cell["ix"], cell["iy"]): dominant(cell)
                    for cell in oracle_payload["cells"]}
    learned_cells = {(cell["ix"], cell["iy"]): dominant(cell)
                     for cell in learned_payload["cells"]}
    confusion = Counter((oracle_cells[cell], learned_cells[cell])
                        for cell in oracle_cells.keys() & learned_cells.keys())
    road_as_sidewalk = confusion["road", "sidewalk"]
    road_as_pole = confusion["road", "pole"]
    lane_matched = confusion["lane_marking", "lane_marking"]
    lane_missing = sum(1 for cell, label in oracle_cells.items()
                       if label == "lane_marking" and cell not in learned_cells)
    check((road_as_sidewalk, road_as_pole, lane_matched, lane_missing) ==
          (2864, 594, 92, 272), "documented fused-cell diagnostic changed")
    return {"status": "VERIFIED_REPEATED_DEV_THREE_FRAME_DIAGNOSTIC",
            "frames": 3, "oracle_cells": len(oracle.cells),
            "learned_cells": len(learned.cells),
            "artifacts_hashed": len(result["artifact_sha256"]),
            "road_as_sidewalk_cells": road_as_sidewalk,
            "road_as_pole_cells": road_as_pole,
            "lane_matched_cells": lane_matched,
            "lane_missing_cells": lane_missing}


if __name__ == "__main__":
    print(json.dumps(verify(), sort_keys=True))
