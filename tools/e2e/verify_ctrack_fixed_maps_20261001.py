#!/usr/bin/env python3
"""HH_261001 - Recount every saved prediction and verify native comparison panels."""

import argparse
from datetime import datetime, timezone
from pathlib import Path
import json
import sys

sys.dont_write_bytecode = True
sys.path.insert(0,str(Path(__file__).resolve().parent))
import evaluate_ctrack_fixed_maps_20261001 as evaluate
import numpy as np
from PIL import Image


def verify(scenario):
    run = evaluate.OUTPUT_ROOT/f"ctrack_{scenario}_fixed_maps_20261001_v1"
    summary = evaluate.read_json(run/"summary.json")
    started = evaluate.read_json(run/"started.json")
    evaluate.validate_preregistration()
    source,records,qa = evaluate.validate_capture(scenario,evaluate.QA_PATHS[scenario])
    require = evaluate.require
    require(summary["status"] == "DIAGNOSTIC_COMPLETE_NOT_PROMOTED"
        and summary["frames_per_model"] == 240 and summary["fit_steps"] == 0
        and summary["intended_role"] == evaluate.ROLE
        and all(summary[key] is False for key in
            ("train_eligible","dataset_admitted","model_promotion","independent_dev_approved")),
        "diagnostic role/completion changed")
    require(summary["source_qa_status"] == qa["status"]
        and summary["original_owner_status"] == qa["owner_provenance"]["original_owner_status"]
        and summary["original_live_replay_exact_match"] == qa["original_live_replay_exact_match"],
        "forensic distinction changed")
    require(evaluate.digest(run/"per_frame.json") == summary["per_frame_sha256"],"per-frame bytes changed")
    frames = evaluate.read_json(run/"per_frame.json")
    require(len(frames) == 240,"per-frame count changed")
    predictions, matrices = {},{}
    for arm in ("A","B"):
        path = run/f"{arm}_all240_predictions.npz"
        require(evaluate.digest(path) == summary["prediction_artifacts"][path.name],"prediction bytes changed")
        with np.load(path,allow_pickle=False) as archive:
            require(archive.files == ["prediction"],"prediction archive keys differ")
            value = archive["prediction"].copy()
        require(value.shape == (240,360,640) and value.dtype == np.uint8 and int(value.max()) < 7,
                "complete native prediction ABI required")
        predictions[arm],matrices[arm] = value,np.zeros((7,7),dtype=np.int64)
    visual_matches = {}
    for index,row in enumerate(records):
        raw = source/(row["stem"]+".npz")
        require(evaluate.digest(raw) == row["artifact_sha256"][".npz"],"raw target bytes changed")
        with np.load(raw,allow_pickle=False) as archive:
            target = evaluate.targets(archive["semantic"])
            rgb = archive["rgb"][...,[2,1,0]]
        for arm in predictions:
            matrix = evaluate.confusion(target,predictions[arm][index])
            require(matrix.tolist() == frames[index][arm]["confusion"],"per-frame confusion differs")
            matrices[arm] += matrix
        if index in evaluate.VISUAL_INDICES:
            path = run/f"{scenario}_{index:03d}_full_fov.png"
            require(evaluate.digest(path) == summary["full_fov_visuals"][path.name],"comparison image changed")
            canvas = np.asarray(Image.open(path))
            require(canvas.shape == (860,1280,3),"comparison canvas ABI differs")
            for x,y,pixels in ((0,57,rgb),(640,57,evaluate.COLORS[target]),
                    (0,442,evaluate.COLORS[predictions["A"][index]]),
                    (640,442,evaluate.COLORS[predictions["B"][index]])):
                require(np.array_equal(canvas[y:y+360,x:x+640],pixels),"native full-FOV panel mismatch")
            visual_matches[str(index)] = True
    for arm,matrix in matrices.items():
        recomputed = evaluate.metrics(matrix)
        require(all(summary["model_results"][arm][key] == value for key,value in recomputed.items()),
                "aggregate confusion or support/IoU differs")
        require(int(matrix.sum()) == 240*360*640,"whole sequence pixel count differs")
    result = {"schema":"portable_e2e.ctrack_fixed_maps_prediction_audit.v1","status":"PASS",
        "created_at_utc":datetime.now(timezone.utc).isoformat(),"scenario":scenario,
        "frames_per_model":240,"predicted_pixels_per_model":240*360*640,
        "all_per_frame_and_aggregate_confusion_counts_match":True,
        "all_fixed_native_full_fov_panels_match":visual_matches,
        "summary_sha256":evaluate.digest(run/"summary.json"),
        "source_qa_status":qa["status"],"original_owner_status":summary["original_owner_status"],
        "original_live_replay_exact_match":summary["original_live_replay_exact_match"],
        "inference_repeated":False,"train_eligible":False,"dataset_admitted":False,
        "model_promotion":False,"source_qa_replayed":True,
        "review_note":started["render_review"]["note"]}
    evaluate.write_new(run/"prediction_audit.json",result)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario",choices=("left","straight"),required=True)
    print(json.dumps(verify(parser.parse_args().scenario),sort_keys=True))
