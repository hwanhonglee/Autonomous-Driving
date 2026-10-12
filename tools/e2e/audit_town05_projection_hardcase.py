#!/usr/bin/env python3
"""HH_260929 - Replay fixed Town05 repeated-DEV RGB predictions into oracle-aided maps."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image, ImageDraw
import torch

from scripts.e2e import audit_learned_mapping_bridge as base
from portable_e2e.map_perception import LABELS, MapPerception
from portable_e2e.mapping import SemanticEvidenceMap
from portable_e2e.mapping_projection import project_frame, project_predicted_frame


ROOT = Path(__file__).resolve().parents[2]
PLAN = ROOT / "docs/assets/validation/2026-09-29/map_town05_projection_hardcase/PLAN.md"
OUTPUT = ROOT / "docs/assets/validation/2026-09-29/map_town05_projection_hardcase/run_v1"
SOURCE = ROOT / "storage/experiments/training/2026-09-27/map_town05_independent_dev_capture_epic_v2/mapping"
AUDIT = ROOT / "docs/assets/validation/2026-09-27/map_town05_dev_capture/epic_v2_audit.json"
PLAN_SHA256 = "412a37f913199fa2c763ad9aa9beb834b5a7836df9cc8e7ec24d39fab19e5655"
INPUT_SHA256 = {
    "checkpoint": "ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380",
    "summary.json": "99dcd8a68b36f0f24611c512f2471139f9f155ea222b697ce899090239376c89",
    "records.json": "966058d688e901469e1199070392a5c1c97d7e93041c992125a9c18d5c0a7f6a",
    "frames/0000.png": "6fb392d75fb0b8f330b1c41fbac780c1cb539089ad8b28ebdc09e5f88dc39258",
    "frames/0115.png": "a6322a4f11e8f5bc1fceabe4c89bb7f81a61574790b1d5ac6ed2ed1c7f286447",
    "frames/0231.png": "3a66683bae948bea92cbcaff6f60f7ae90a733f0372f970f5bea965f953eac85",
    "frames/0000.npz": "0eb681ae5deef4c189c369fe2362b7a4b3e8124dc6420b33df052504b17bc8d9",
    "frames/0115.npz": "b5bc2dfbcb63ad9a154e0877545ca647fdcf9164849ef43136fefb5ef7a75bcf",
    "frames/0231.npz": "915d2741bdcdd19240ec2016ca1576d969ab64f16cb007184ae49b15bef338f0",
}
SUPPORT_SHA256 = {**base.SOURCE_SHA256,
                  "tools/e2e/audit_learned_mapping_bridge.py":
                  "e900ac60fa5e66ec621cfd6a108d3960013d19e696ea7bc74e1e619669b8105d"}
AUDIT_SHA256 = "d64d77bf4f1d02892f59ac9debd18489c5acbf38842ec81ac4398f7052b950d1"
ROUTE_SHA256 = "efb8ccca533f25cae54b2eb7e03274c15795211488128185334ff2bc55f4744c"
INDICES = (0, 115, 231)


def check(condition: bool, reason: str) -> None:
    if not condition:
        raise ValueError(reason)


def preflight() -> tuple[list[dict], dict]:
    """HH_260929 - Guard repeated-DEV role and exact byte identities before any inference."""
    check(base.digest(PLAN) == PLAN_SHA256, "frozen PLAN changed")
    check(base.digest(base.CHECKPOINT) == INPUT_SHA256["checkpoint"], "checkpoint changed")
    check(base.digest(AUDIT) == AUDIT_SHA256, "source capture audit changed")
    for relative, expected in SUPPORT_SHA256.items():
        check(base.digest(ROOT / relative) == expected, f"support changed: {relative}")
    for relative, expected in INPUT_SHA256.items():
        if relative != "checkpoint":
            check(base.digest(SOURCE / relative) == expected, f"capture changed: {relative}")
    summary = json.loads((SOURCE / "summary.json").read_text(encoding="utf-8"))
    records = json.loads((SOURCE / "records.json").read_text(encoding="utf-8"))
    check(summary.get("status") == "COMPLETE_ORACLE_PLUMBING_ONLY"
          and summary.get("town") == "Town05_Opt"
          and summary.get("route_sha256") == ROUTE_SHA256
          and summary.get("frame_count") == len(records) == 240
          and summary.get("oracle_pose") is True
          and summary.get("oracle_depth") is True
          and summary.get("oracle_semantics") is True
          and summary.get("live_replay_exact_match") is True
          and summary.get("collision_events") == 0,
          "Town05 source contract changed")
    check(all(records[index]["stem"] == f"frames/{index:04d}" for index in INDICES)
          and all(records[a]["timestamp_s"] < records[b]["timestamp_s"]
                  for a, b in zip(INDICES, INDICES[1:])),
          "fixed source indices or time order changed")
    check(not OUTPUT.exists() and not OUTPUT.is_symlink()
          and OUTPUT.parent.is_dir() and not OUTPUT.parent.is_symlink(),
          "fresh result directory required")
    return records, summary


def preview_town05(rgb: Image.Image, predicted: np.ndarray, record: dict,
                   oracle: SemanticEvidenceMap, learned: SemanticEvidenceMap,
                   index: int) -> Image.Image:
    """HH_260929 - Reuse centered panels, replacing fixed Town01/sample headings."""
    preview = base.render_preview(rgb, predicted, record, oracle, learned, index)
    draw = ImageDraw.Draw(preview)
    draw.rectangle((0, 0, 1279, 51), fill=(18, 25, 35))
    draw.text((15, 8),
              f"TOWN05_OPT frame {index:04d} | REPEATED DEV | CARLA oracle depth/pose | NOT HD MAP",
              font=base._font(20), fill="white")
    draw.rectangle((0, 836, 1279, 859), fill=(18, 25, 35))
    draw.text((15, 840), "Blue road | yellow markings | purple sidewalk | dark unknown | sampled 0000/0115/0231",
              font=base._font(14), fill="white")
    return preview


def run() -> dict:
    records, source_summary = preflight()
    check(not torch.cuda.is_initialized(), "local GPU must not be initialized")
    torch.set_num_threads(2)
    checkpoint = torch.load(base.CHECKPOINT, map_location="cpu")
    check(tuple(checkpoint["labels"]) == tuple(LABELS)
          and checkpoint.get("deployable") is False and checkpoint.get("steps") == 2000,
          "research checkpoint contract changed")
    model = MapPerception().cpu().eval().requires_grad_(False)
    model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    oracle = SemanticEvidenceMap("carla_ue:Town05_Opt", "oracle_sensor", max_cells=200000)
    learned = SemanticEvidenceMap("carla_ue:Town05_Opt", "learned_prediction", max_cells=200000)
    OUTPUT.mkdir(parents=True, exist_ok=False)
    rows = []
    for index in INDICES:
        record = records[index]
        with np.load(SOURCE / f"frames/{index:04d}.npz", allow_pickle=False) as raw:
            check(set(raw.files) == {"depth", "semantic", "world_from_camera"},
                  "raw sensor archive keys changed")
            depth, semantic, pose = (raw[name] for name in
                                     ("depth", "semantic", "world_from_camera"))
        rgb = Image.open(SOURCE / f"frames/{index:04d}.png").convert("RGB")
        check(rgb.size == (640, 360), "frozen camera shape changed")
        batch = torch.from_numpy(np.asarray(rgb).transpose(2, 0, 1).copy()).unsqueeze(0)
        started = time.perf_counter()
        with torch.no_grad():
            probability = model(batch.float() / 255).softmax(1)
            confidence, prediction = probability.max(1)
        elapsed = time.perf_counter() - started
        predicted = prediction[0].cpu().numpy().astype(np.uint8)
        confidence = confidence[0].cpu().numpy().astype(np.float32)
        timestamp = record["timestamp_s"]
        o = project_frame(depth, semantic, pose, frame_id=oracle.frame_id,
                          timestamp_s=timestamp, fov_deg=base.FOV_DEG,
                          label_table={1: "road", 2: "sidewalk", 24: "lane_marking",
                                       7: "traffic_light_fixture", 8: "traffic_sign", 6: "pole"},
                          stride=base.STRIDE, max_depth_m=base.MAX_DEPTH_M)
        l = project_predicted_frame(
            base.decode_depth_m(depth), predicted, confidence, pose,
            frame_id=learned.frame_id, timestamp_s=timestamp, fov_deg=base.FOV_DEG,
            label_table=base.LABEL_TABLE, stride=base.STRIDE,
            max_depth_m=base.MAX_DEPTH_M, min_confidence=base.MIN_CONFIDENCE)
        check(o.source == "oracle_sensor" and l.source == "learned_prediction",
              "oracle/model provenance mixed")
        oracle.update(o)
        learned.update(l)
        np.savez_compressed(OUTPUT / f"frame_{index:04d}_prediction.npz",
                            predicted_class=predicted, confidence=confidence)
        preview_town05(rgb, predicted, record, oracle, learned, index).save(
            OUTPUT / f"frame_{index:04d}_ego_centered.png")
        rows.append({"index": index, "timestamp_s": timestamp,
                     "inference_seconds_cpu_2threads": elapsed,
                     "pixel_metrics": base.pixel_metrics(base.semantic_target(semantic), predicted),
                     "oracle_projected_points": len(o.points),
                     "learned_projected_points": len(l.points),
                     "oracle_accumulated_cells": len(oracle.cells),
                     "learned_accumulated_cells": len(learned.cells)})
    (OUTPUT / "oracle_evidence.json").write_text(
        json.dumps(oracle.snapshot(), indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (OUTPUT / "learned_evidence.json").write_text(
        json.dumps(learned.snapshot(), indent=2, allow_nan=False) + "\n", encoding="utf-8")
    artifacts = sorted(path for path in OUTPUT.iterdir() if path.is_file())
    result = {
        "schema": "portable_e2e.town05_mapping_projection_hardcase.v1",
        "status": "REPEATED_DEV_THREE_FRAME_DIAGNOSTIC_NOT_HD_MAP",
        "preregistration_sha256": PLAN_SHA256, "script_sha256": base.digest(Path(__file__)),
        "input_sha256": INPUT_SHA256, "support_sha256": SUPPORT_SHA256,
        "capture_audit_sha256": AUDIT_SHA256,
        "artifact_sha256": {path.name: base.digest(path) for path in artifacts},
        "indices": list(INDICES), "source_record_count": source_summary["frame_count"],
        "frames_actually_projected": len(rows), "model_input": "RGB only",
        "projection_depth": "CARLA optical-axis oracle depth",
        "projection_pose": "CARLA world_from_camera oracle pose",
        "oracle_semantics_used_by_model": False,
        "oracle_semantics_used_by_reference_only": True,
        "stride": base.STRIDE, "max_depth_m": base.MAX_DEPTH_M,
        "min_model_confidence": base.MIN_CONFIDENCE, "grid_resolution_m": .2,
        "oracle_source": oracle.source, "learned_source": learned.source,
        "cell_overlap_diagnostic": base.cell_overlap(oracle, learned),
        "rows": rows, "independent_dev_count": 0, "repeated_dev_only": True,
        "test_used": False, "hd_map_approved": False,
        "learned_closed_loop_tested": False, "gpu_used": False,
    }
    (OUTPUT / "summary.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"status": result["status"], "frames": len(rows),
                      "oracle_cells": len(oracle.cells), "learned_cells": len(learned.cells)}))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validate-only", action="store_true",
                        help="Validate sealed inputs and fresh output without inference")
    args = parser.parse_args()
    if args.validate_only:
        preflight()
        print("PRECHECK_OK: repeated DEV role, source identities and fresh output path")
    else:
        run()
