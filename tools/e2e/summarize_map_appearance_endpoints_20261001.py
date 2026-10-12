#!/usr/bin/env python3
"""HH_261001 - Report all preregistered endpoints only after all independent recounts."""

from datetime import datetime, timezone
import json
from pathlib import Path
import sys

sys.dont_write_bytecode = True
try:
    from . import evaluate_map_appearance_endpoints_20261001 as evaluate
except ImportError:
    import evaluate_map_appearance_endpoints_20261001 as evaluate

FIT_SHA = "463741ec65e9fce501bcb06dbaecc6c02d5b46bd566d6510ca0a7050b54e5253"
PREREG_SHA = "dbd037ee055103e53b40d43e8aec57247386929bf990beac20cca6f414438616"


def run():
    c = evaluate.contract
    registration = evaluate.validate_registration()
    c.require(c.digest(evaluate.PREREG) == PREREG_SHA,"registered evaluation policy changed")
    binding = evaluate.fit_binding(FIT_SHA)
    scenarios,total_pixels,total_panels = {},0,0
    for scenario in evaluate.SCENARIOS:
        output = evaluate.output_path(scenario)
        summary,audit = (c.read_json(output/name) for name in ("summary.json","prediction_audit.json"))
        c.require(audit["status"] == "PASS" and audit["summary_sha256"] == c.digest(output/"summary.json")
            and audit["fit_summary_sha256"] == FIT_SHA and summary["fit_artifact_sha256"] == binding
            and summary["preregistration_sha256"] == PREREG_SHA,"complete independent scenario audit required")
        scenarios[scenario] = {"output":str(output),"summary_sha256":c.digest(output/"summary.json"),
            "recount_sha256":c.digest(output/"prediction_audit.json"),"results":summary["model_results"],
            "appearance_minus_control_iou":summary["iou_delta_appearance_minus_control"],
            "native_panels":summary["full_fov_visuals"],"role":summary["evaluation_role"],
            "source_qa_status":summary["source_qa_status"],"original_owner_status":summary["original_owner_status"],
            "original_live_replay_exact_match":summary["original_live_replay_exact_match"]}
        total_pixels += audit["predicted_pixels_recounted"]
        total_panels += audit["native_full_fov_panels_verified"]
    c.require(total_pixels == 663552000 and total_panels == 72,"all preregistered model/frame/condition data required")
    known_receipt = evaluate.DOCS/"known_split_independent_recount.json"
    known = c.read_json(known_receipt)
    c.require(known["status"] == "PASS" and known["summary_sha256"] == FIT_SHA
        and known["total_recounted_pixels"] == 161280000,"root's known-split independent recount missing")
    fit_summary = c.read_json(evaluate.FIT/"summary.json")
    known_results = fit_summary["known_evaluation"]
    for key,check in known["checks"].items():
        split,arm = key.split("/")
        c.require(check["all_pixel_confusion_exact"] is True
            and c.digest(evaluate.FIT/arm/(split+"_predictions.npz")) == check["prediction_sha256"],"known-split prediction receipt differs")
    value = {"schema":"portable_e2e.map_appearance_all_endpoint_results.v1","status":"ALL_FOUR_DIAGNOSTICS_RECOUNTED",
        "created_at_utc":datetime.now(timezone.utc).isoformat(),"fit_summary_sha256":FIT_SHA,"preregistration_sha256":PREREG_SHA,
        "predicted_pixels_recounted":total_pixels,"native_panels_verified":total_panels,"scenarios":scenarios,
        "fit_artifacts":binding,"known_final_only_results":known_results,
        "known_split_independent_recount_sha256":c.digest(known_receipt),"known_split_pixels_recounted_separately":161280000,
        "model_promotion":False,"ctrack_train_eligible":False,"independent_validation":False,
        "limitations":registration["policy"]["limitations"],"postprocessing_source_sha256":c.digest(Path(__file__).resolve())}
    lines = ["# Matched appearance: all fixed endpoint results", "",
        "<!-- HH_261001 - Complete preregistered outputs; no model selection or promotion. -->", "",
        "All four scenarios completed and independently recounted: 2,880 predictions, 663,552,000 pixels, and 72 native full-FOV panels. No frames were filtered or selected by performance.", "",
        "Both models use the same B architecture, initialization, 10,000 steps, sample/crop exposure, labels, optimizer and common weights. The sole paired TRAIN-input factor is RGB postprocess=false (control) versus true (appearance).", "",
        "CTrack is exposed external diagnostic reuse, not independent certification. Paired Town01/Town02 scores below are exact TRAIN-frame resubstitution, not held-out validation. LEFT's original owner FAIL and unavailable live-map equality remain unchanged.", "",
        f"Preregistration SHA: `{PREREG_SHA}`. Final fit summary SHA: `{FIT_SHA}`.", "",
        "All IoUs and deltas below are percentages/percentage points. A null IoU means no GT pixels; false-positive counts remain visible.", ""]
    latency = []
    for scenario,result in scenarios.items():
        lines.extend(["## "+scenario,"",f"Role: `{result['role']}`. QA: `{result['source_qa_status']}`; original live-map equality: `{result['original_live_replay_exact_match']}`.",""])
        for condition in evaluate.locations(scenario)[2]:
            lines.extend(["### RGB postprocess="+condition,"",
                "| Class | Control IoU % | Appearance IoU % | Delta pp | GT pixels | Control FP | Appearance FP |",
                "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"])
            control,appearance = (result["results"][arm+"_"+condition] for arm in evaluate.ARMS)
            for label in evaluate.frozen.LABELS:
                a,b = control["class_metrics"][label],appearance["class_metrics"][label]
                c.require(a["ground_truth_pixels"] == b["ground_truth_pixels"],"matched targets differ")
                left = "null" if a["iou"] is None else f"{100*a['iou']:.4f}"
                right = "null" if b["iou"] is None else f"{100*b['iou']:.4f}"
                delta = "null" if a["iou"] is None else f"{100*(b['iou']-a['iou']):+.4f}"
                lines.append(f"| {label} | {left} | {right} | {delta} | {a['ground_truth_pixels']:,} | {a['false_positive_pixels']:,} | {b['false_positive_pixels']:,} |")
            lines.append("")
            latency.append((scenario,condition,control["inference_p95_ms"],appearance["inference_p95_ms"]))
        lines.extend(["Fixed native panels (indices 0/119/239):",""])
        for name in sorted(result["native_panels"]):
            lines.append(f"- [{name}]({result['output']}/{name})")
        lines.append("")
    lines.extend(["## External road error direction","",
        "Complete-sequence raw-target confusion, not selected frames. The appearance model's STRAIGHT road loss is primarily road pixels becoming sidewalk.","",
        "| Scenario | Model | GT road pixels | Road predicted sidewalk | Fraction % |","| --- | --- | ---: | ---: | ---: |"])
    for scenario in ("ctrack_left","ctrack_straight"):
        for arm in evaluate.ARMS:
            row = scenarios[scenario]["results"][arm+"_true"]["confusion"][1]
            lines.append(f"| {scenario} | {arm} | {sum(row):,} | {row[2]:,} | {100*row[2]/sum(row):.4f} |")
    lines.extend(["","## Known final-only DEV/holdout retention","",
        "These unchanged splits were evaluated after both endpoints were saved. Root separately verified all 161,280,000 prediction pixels; receipt: `known_split_independent_recount.json`. They are exposed known splits, not new independent certification. All seven class IoUs are lower for appearance than control on both splits.","",
        "| Split | Class | Control IoU % | Appearance IoU % | Delta pp |","| --- | --- | ---: | ---: | ---: |"])
    for split,result in known_results.items():
        for label in evaluate.frozen.LABELS:
            a,b = (result[arm]["iou"][label] for arm in evaluate.ARMS)
            lines.append(f"| {split} | {label} | {100*a:.4f} | {100*b:.4f} | {100*(b-a):+.4f} |")
    lines.extend(["","## CPU2 diagnostic latency","",
        "Forward+argmax only, including first cold inference. Other local work may overlap; not an unloaded-runtime or complete-pipeline10Hz claim.","",
        "| Scenario | RGB | Control p95 ms | Appearance p95 ms |","| --- | --- | ---: | ---: |"])
    lines.extend(f"| {scenario} | {condition} | {a:.3f} | {b:.3f} |" for scenario,condition,a,b in latency)
    lines.extend(["","No promotion thresholds, adaptive checkpoint selection, new DEV tuning, CTrack TRAIN admission or broad dataset admission were performed.",""])
    evaluate.frozen.write_new(evaluate.DOCS/"RESULTS.json",value)
    with (evaluate.DOCS/"RESULTS.md").open("x") as stream:
        stream.write("\n".join(lines))
    return {"status":value["status"],"predicted_pixels_recounted":total_pixels,"native_panels_verified":total_panels,
        "json_sha256":c.digest(evaluate.DOCS/"RESULTS.json"),"report":str(evaluate.DOCS/"RESULTS.md")}


if __name__ == "__main__":
    print(json.dumps(run(),sort_keys=True),flush=True)
