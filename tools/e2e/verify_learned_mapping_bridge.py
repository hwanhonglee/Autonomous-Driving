#!/usr/bin/env python3
"""HH_260929 - Independently replay frozen learned/oracle map evidence without inference."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from PIL import Image

from scripts.e2e import audit_learned_mapping_bridge as bridge
from scripts.e2e import site_split_guard
from portable_e2e.mapping import SemanticEvidenceMap
from portable_e2e.mapping_projection import project_frame, project_predicted_frame


def check(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def verify() -> dict:
    """HH_260929 - Verify hashes, pixel metrics, both projected maps and centered ego."""
    split = json.loads(site_split_guard.REGISTRY.read_text(encoding="utf-8"))
    check(site_split_guard.check("Town01", "map", "reuse_existing_train", split)
          and split["independent_dev_sites"] == [],
          "Town01 TRAIN-exposed site role changed")
    output = bridge.OUTPUT
    result = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    check(result["schema"] == "portable_e2e.learned_mapping_projection_bridge.v1"
          and result["status"] == "CPU_ONLY_THREE_FRAME_PLUMBING_NOT_HD_MAP"
          and result["frames_actually_projected"] == 3
          and result["independent_dev_count"] == 0
          and result["test_used"] is False
          and result["hd_map_approved"] is False
          and result["learned_closed_loop_tested"] is False
          and result["gpu_used"] is False, "research-only result contract changed")
    check(result["preregistration_sha256"] == bridge.digest(bridge.PLAN)
          and result["script_sha256"] == bridge.digest(Path(bridge.__file__))
          and result["input_sha256"]["checkpoint"] == bridge.digest(bridge.CHECKPOINT),
          "plan, script or checkpoint hash changed")
    for relative, expected in result["support_sha256"].items():
        check(expected == bridge.digest(bridge.ROOT / relative),
              f"support source hash changed: {relative}")
    for relative, expected in result["input_sha256"].items():
        if relative != "checkpoint":
            check(expected == bridge.digest(bridge.SOURCE / relative),
                  f"frozen input hash changed: {relative}")
    check(set(result["artifact_sha256"]) ==
          {f"frame_{index:04d}_{suffix}" for index in bridge.INDICES
           for suffix in ("ego_centered.png", "prediction.npz")} |
          {"oracle_evidence.json", "learned_evidence.json"},
          "eight independent evidence files required")
    for name, expected in result["artifact_sha256"].items():
        check(bridge.digest(output / name) == expected, f"artifact hash changed: {name}")
    records = json.loads((bridge.SOURCE / "records.json").read_text(encoding="utf-8"))
    oracle = SemanticEvidenceMap("carla_ue:Town01", "oracle_sensor", max_cells=200000)
    learned = SemanticEvidenceMap("carla_ue:Town01", "learned_prediction", max_cells=200000)
    for index, row in zip(bridge.INDICES, result["rows"], strict=True):
        check(index == row["index"], "frame order changed")
        with np.load(bridge.SOURCE / f"frames/{index:04d}.npz", allow_pickle=False) as raw:
            depth, semantic, pose = (raw[name] for name in
                                     ("depth", "semantic", "world_from_camera"))
        with np.load(output / f"frame_{index:04d}_prediction.npz",
                     allow_pickle=False) as inferred:
            check(set(inferred.files) == {"predicted_class", "confidence"},
                  "prediction payload keys changed")
            prediction = inferred["predicted_class"]
            confidence = inferred["confidence"]
        check(prediction.shape == confidence.shape == (360, 640)
              and prediction.dtype == np.uint8 and confidence.dtype == np.float32,
              "prediction shape or dtype changed")
        check(bridge.pixel_metrics(bridge.semantic_target(semantic), prediction) ==
              row["pixel_metrics"], "published pixel confusion changed")
        timestamp = records[index]["timestamp_s"]
        o = project_frame(depth, semantic, pose, frame_id=oracle.frame_id,
                          timestamp_s=timestamp, fov_deg=bridge.FOV_DEG,
                          label_table={1: "road", 2: "sidewalk", 24: "lane_marking",
                                       7: "traffic_light_fixture", 8: "traffic_sign", 6: "pole"},
                          stride=bridge.STRIDE, max_depth_m=bridge.MAX_DEPTH_M)
        l = project_predicted_frame(
            bridge.decode_depth_m(depth), prediction, confidence, pose,
            frame_id=learned.frame_id, timestamp_s=timestamp, fov_deg=bridge.FOV_DEG,
            label_table=bridge.LABEL_TABLE, stride=bridge.STRIDE,
            max_depth_m=bridge.MAX_DEPTH_M,
            min_confidence=bridge.MIN_CONFIDENCE)
        check(o.source == "oracle_sensor" and l.source == "learned_prediction",
              "map provenance mixed")
        oracle.update(o)
        learned.update(l)
        check((len(o.points), len(l.points), len(oracle.cells), len(learned.cells)) ==
              (row["oracle_projected_points"], row["learned_projected_points"],
               row["oracle_accumulated_cells"], row["learned_accumulated_cells"]),
              "published projection counts changed")
        with Image.open(output / f"frame_{index:04d}_ego_centered.png") as preview:
            check(preview.size == (1280, 860)
                  and preview.getpixel((320, 640)) == (255, 110, 65)
                  and preview.getpixel((960, 640)) == (255, 110, 65),
                  "vehicle is not centered in both BEV panels")
    check(oracle.snapshot() ==
          json.loads((output / "oracle_evidence.json").read_text(encoding="utf-8")),
          "oracle fused-cell payload changed")
    check(learned.snapshot() ==
          json.loads((output / "learned_evidence.json").read_text(encoding="utf-8")),
          "learned fused-cell payload changed")
    check(bridge.cell_overlap(oracle, learned) == result["cell_overlap_diagnostic"],
          "published cell diagnostic changed")
    return {"status": "VERIFIED_CPU_ONLY_THREE_FRAME_PLUMBING",
            "frames": len(result["rows"]), "oracle_cells": len(oracle.cells),
            "learned_cells": len(learned.cells),
            "artifacts_hashed": len(result["artifact_sha256"])}


if __name__ == "__main__":
    print(json.dumps(verify(), sort_keys=True))
