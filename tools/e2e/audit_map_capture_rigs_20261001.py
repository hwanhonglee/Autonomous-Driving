#!/usr/bin/env python3
"""HH_261001 - Verify archived TRAIN camera rigs; supersede an unsupported rig claim."""

import ast
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OWNERS = (
    "storage/experiments/validation/2026-09-22/mapping_sensor_probe/moving_straight_v1",
    "storage/experiments/training/2026-09-26/dual_data_ai_v1/mapping_town01_p01",
    "storage/experiments/training/2026-09-26/dual_data_ai_v1/mapping_town01_p03",
    "storage/experiments/training/2026-09-27/map_town02_sign_train_scout_epic_v1")
ATTRIBUTES = {"image_size_x":"640","image_size_y":"360","fov":"90", 
              "sensor_tick":"0","lens_k":"0","lens_kcube":"0"}


def digest(path):
    if not path.is_file() or path.is_symlink():
        raise ValueError("regular source required")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def archived_contract(path):
    tree = ast.parse(path.read_text())
    attrs,locations,pitches,postprocess = [],[],[],[]
    for node in ast.walk(tree):
        if isinstance(node,ast.Dict):
            try:
                value = ast.literal_eval(node)
                if isinstance(value,dict) and "image_size_x" in value:
                    attrs.append(value)
            except (ValueError,TypeError):
                pass
        if not isinstance(node,ast.Call) or not isinstance(node.func,ast.Attribute):
            continue
        if node.func.attr in ("Location","Rotation"):
            try:
                value = {row.arg:ast.literal_eval(row.value) for row in node.keywords}
            except (ValueError,TypeError):
                continue
            if node.func.attr == "Location" and "x" in value and "z" in value:
                locations.append(value)
            if node.func.attr == "Rotation" and "pitch" in value:
                pitches.append(value)
        if node.func.attr == "set_attribute" and len(node.args) == 2:
            try:
                args = tuple(ast.literal_eval(value) for value in node.args)
            except (ValueError,TypeError):
                continue
            if args[0] == "enable_postprocess_effects":
                postprocess.append(args[1])
    if attrs != [ATTRIBUTES] or {"x":1.5,"z":2.0} not in locations or {"pitch":-12} not in pitches:
        raise ValueError("archived camera geometry differs")
    if postprocess != ["false"]:
        raise ValueError("archived RGB postprocess does not match historical contract")
    return {"blueprint_attributes":ATTRIBUTES,"camera_mount":{"x":1.5,"y":0.0,"z":2.0,"pitch":-12.0},
            "rgb_enable_postprocess_effects":False,"evidence_kind":"owner-hash-verified archived collector AST"}


def main():
    results = []
    for relative in OWNERS:
        owner = ROOT/relative
        plan_path = owner/"owner_plan.json"
        plan = json.loads(plan_path.read_text())
        archive = owner/"provenance/scripts/e2e/collect_carla_mapping_probe.py"
        sha = digest(archive)
        if sha != plan["source_sha256"]["scripts/e2e/collect_carla_mapping_probe.py"]:
            raise ValueError("owner archive SHA differs")
        results.append({"owner":str(owner),"plan_sha256":digest(plan_path),"collector":str(archive),
            "collector_sha256":sha,"quality":plan["quality"],"route_sha256":plan["route_sha256"],
            **archived_contract(archive)})
    ctrack = ROOT/"storage/experiments/validation/2026-10-01/ctrack_straight_external_diagnostic_20261001_v2/capture/summary.json"
    summary = json.loads(ctrack.read_text())
    intrinsics = summary["sensor_intrinsics"]
    if summary["camera_mount"] != results[0]["camera_mount"] or not summary["rgb_enable_postprocess_effects"]:
        raise ValueError("CTrack rig or render contract differs unexpectedly")
    if (intrinsics["width"],intrinsics["height"],intrinsics["horizontal_fov_deg"],
        intrinsics["lens_k"],intrinsics["lens_kcube"]) != (640,360,90,0,0):
        raise ValueError("CTrack intrinsics differ")
    report = {"schema":"portable_e2e.map_camera_rig_correction.v1","status":"PASS",
        "train_capture_contracts":results,"ctrack_summary":str(ctrack),"ctrack_summary_sha256":digest(ctrack),
        "configured_camera_geometry_equal":True,"confirmed_rgb_postprocess_difference":True,
        "train_rgb_postprocess":False,"ctrack_rgb_postprocess":True,
        "correction":"Prior preregistration/results said camera rig differs; archived source proves configured geometry matches.",
        "old_receipts_modified":False,"retrospective_runtime_rig_identity_proven":False,
        "runtime_caveat":"Historical captures lack the new per-sensor runtime calibration metadata; equality here is configured geometry.",
        "remaining_shift_axes":["RGB lighting/postprocessing","map/mesh/material style","packaged versus source-editor runtime"],
        "causal_attribution_proven":False,"model_predictions_used_to_choose_rig":False,
        "ctrack_train_eligible":False,"model_promotion":False}
    destination = ROOT/"docs/assets/validation/2026-10-01/ctrack_external_maps/camera_rig_correction.json"
    with destination.open("x") as stream:
        json.dump(report,stream,indent=2,sort_keys=True)
        stream.write("\n")
    print(json.dumps({key:report[key] for key in ("status","configured_camera_geometry_equal","confirmed_rgb_postprocess_difference")}))


if __name__ == "__main__":
    main()
