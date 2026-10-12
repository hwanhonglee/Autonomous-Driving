#!/usr/bin/env python3
"""HH_261001 - TRAIN-only previews then thin registered package; never train/launch."""

from datetime import datetime, timezone
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys

sys.dont_write_bytecode = True
try:
    from . import map_appearance_matched_contract_20261001 as base
    from . import map_photometric_matched_contract_20261001 as photo
except ImportError:
    import map_appearance_matched_contract_20261001 as base
    import map_photometric_matched_contract_20261001 as photo
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
REFERENCE = ROOT/"storage/experiments/training/2026-10-01"/base.NAME
OUTPUT = ROOT/"storage/experiments/training/2026-10-01"/photo.NAME
DOCS = ROOT/"docs/assets/validation/2026-10-01/map_photometric_matched"
PREVIEW = DOCS/"train_previews_v1"
PRIOR = ROOT/"docs/assets/validation/2026-09-29/map_photometric_pilot"
INDICES = (0,119,239)


def write_new(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("x") as stream:
        json.dump(value,stream,indent=2,sort_keys=True,allow_nan=False); stream.write("\n")


def copy_new(source,target):
    target.parent.mkdir(parents=True,exist_ok=True)
    with source.open("rb") as incoming,target.open("xb") as outgoing:
        shutil.copyfileobj(incoming,outgoing,1024*1024)
    base.require(base.digest(source) == base.digest(target),"archival copy differs")


def load_policy():
    path = REFERENCE/"source/train_map_photometric_pilot.py"
    base.require(base.digest(path) == photo.POLICY_SOURCE_SHA,"archived augmentation changed")
    spec = importlib.util.spec_from_file_location("_archived_photo_policy_20261001",path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def reference_binding():
    base.require(base.digest(REFERENCE/"manifest.json") == photo.REFERENCE_MANIFEST_SHA
        and base.digest(REFERENCE/"study_train_registry.json") == photo.REFERENCE_REGISTRY_SHA
        and base.digest(REFERENCE/"source/bundle.json") == photo.REFERENCE_SOURCE_SHA,"original study changed")
    manifest = base.read_json(REFERENCE/"manifest.json")
    base.validate_manifest(manifest,base.read_json(REFERENCE/"study_train_registry.json"))
    for name,ref in manifest["files"].items():
        base.require(base.digest(REFERENCE/name) == ref["sha256"],"immutable source arrays/parent changed")
    base.require(base.digest(REFERENCE/"paired_samples.json") == manifest["paired_samples_sha256"],"paired provenance changed")
    return manifest


def prior_evidence():
    paths = [PRIOR/"README.md",PRIOR/"PLAN.md",PRIOR/"SPATIAL_DIAGNOSTIC.md",
        PRIOR/"remote_run_v1/summary.json",REFERENCE/"source/train_map_photometric_pilot.py"]
    return {"files":{str(path):base.digest(path) for path in paths},
        "prior_status":"retention gate FAIL; not promoted",
        "town05_lane_delta_pp":6.158,"town01_lane_delta_pp":-5.485,
        "spatial_note":"Town01 far-field lane false positives increased; prior policy was not a general solution.",
        "difference":"Prior all729 scratch2000; this B-context continuation10000 applies only TRUE480, original729 unchanged.",
        "scope":"Ranges are reused unchanged; prior repeatedly observed DEV and current CTrack motivated exploration, not independent certification."}


def tensor_digest(path):
    sha = hashlib.sha256()
    with np.load(path,allow_pickle=False) as data:
        base.require(len(data.files) == len(set(data.files)),"duplicate tensor keys")
        for name in sorted(data.files):
            array = data[name]
            base.require(array.dtype == np.float32 and bool(np.isfinite(array).all()),"finite float32 tensors required")
            header = json.dumps([name,"torch.float32",list(array.shape)]).encode("utf-8")
            sha.update(len(header).to_bytes(8,"big")); sha.update(header); sha.update(array.tobytes())
    return sha.hexdigest()


class ConditionalPreviewRng:
    """HH_261001 - Declared policy-ON boundary example, never a training RNG."""
    def __init__(self): self.draw = 0
    def random(self): return 0.
    def uniform(self,low,high):
        self.draw += 1
        return .5 if self.draw == 1 else .35


def preview():
    base.require(not PREVIEW.exists(),"fresh TRAIN preview directory required")
    manifest = reference_binding(); guard = load_policy()
    PREVIEW.mkdir(parents=True)
    rng = np.random.default_rng(photo.PHOTO_SEED)
    rows = []
    colors = np.asarray(((30,30,40),(67,92,123),(141,88,147),(246,222,90),
        (246,100,70),(60,220,180),(179,190,199)),np.uint8)
    with np.load(REFERENCE/"paired_train480.npz",allow_pickle=False) as data:
        rgb,target = data["rgb_true"],data["target"]
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",16)
    for route_index,route in enumerate(("town01_p01","town02_p00")):
        for index in INDICES:
            position = route_index*240+index
            original = rgb[position].copy()
            transformed,applied = guard.photometric(original,rng)
            low,low_applied = guard.photometric(original,ConditionalPreviewRng())
            base.require(low_applied and np.array_equal(original,rgb[position]),"preview mutated original")
            canvas = Image.new("RGB",(1280,860),(18,27,40)); draw = ImageDraw.Draw(canvas)
            draw.text((8,7),f"TRAIN-only {route} frame{index:03d}/239 | archived policy input QA | no model predictions",font=font,fill="white")
            panels = ((0,57,"Original TRUE RGB",original),(640,57,f"Seeded exact policy draw; applied={applied}",transformed),
                (0,442,"Conditional ON example: blend=.5 brightness=.35",low),(640,442,"Shared unchanged semantic target",colors[target[position]]))
            for x,y,label,pixels in panels:
                draw.text((x+8,y-22),label,font=font,fill="white"); canvas.paste(Image.fromarray(pixels),(x,y))
            draw.text((8,815),"Fixed first/mid/last; no CTrack input, score tuning, geometry changes or label transforms.",font=font,fill="white")
            path = PREVIEW/f"{route}_{index:03d}_photo_inputs.png"; canvas.save(path)
            rows.append({"route":route,"index":index,"pair_id":position,"seeded_applied":bool(applied),
                "rgb_sha256":hashlib.sha256(original.tobytes()).hexdigest(),"target_sha256":hashlib.sha256(target[position].tobytes()).hexdigest(),
                "seeded_rgb_sha256":hashlib.sha256(transformed.tobytes()).hexdigest(),"conditional_rgb_sha256":hashlib.sha256(low.tobytes()).hexdigest(),
                "png":path.name,"png_sha256":base.digest(path)})
    receipt = {"schema":"portable_e2e.map_photometric_train_preview.v1","created_at_utc":datetime.now(timezone.utc).isoformat(),
        "reference_manifest_sha256":photo.REFERENCE_MANIFEST_SHA,"photometric_policy":photo.policy(),"indices":list(INDICES),
        "preview_seed":photo.PHOTO_SEED,"conditional_example":{"blend":.5,"brightness":.35,"gate":"explicit ON for input QA only"},
        "samples":rows,"only_train_inputs":True,"ctrack_inputs_used":False,"model_predictions_made":False,
        "prior_evidence":prior_evidence(),"builder_source_sha256":base.digest(Path(__file__).resolve())}
    write_new(PREVIEW/"receipt.json",receipt)
    reference_binding()
    return {"status":"TRAIN_INPUT_PREVIEWS_READY","path":str(PREVIEW),"receipt_sha256":base.digest(PREVIEW/"receipt.json")}


def build(review_note):
    base.require(isinstance(review_note,str) and len(review_note.strip()) >= 60,"explicit review of all six fixed TRAIN previews required")
    base.require(not OUTPUT.exists() and not OUTPUT.is_symlink(),"fresh thin study package required")
    reference = reference_binding(); guard = load_policy()
    preview_receipt = base.read_json(PREVIEW/"receipt.json")
    base.require(preview_receipt["photometric_policy"] == photo.policy() and preview_receipt["indices"] == list(INDICES)
        and preview_receipt["only_train_inputs"] is True and preview_receipt["ctrack_inputs_used"] is False
        and preview_receipt["builder_source_sha256"] == base.digest(Path(__file__).resolve()),"TRAIN preview contract changed")
    for row in preview_receipt["samples"]:
        base.require(base.digest(PREVIEW/row["png"]) == row["png_sha256"],"fixed TRAIN preview changed")
    source_files = {name:ROOT/"tools/e2e"/name for name in
        ("train_map_photometric_matched_20261001.py","map_photometric_matched_contract_20261001.py")}
    source_files["plan.md"] = DOCS/"PLAN.md"
    for name in ("map_appearance_matched_contract_20261001.py","map_perception.py","map_context_recovery.py","train_map_photometric_pilot.py"):
        source_files[name] = REFERENCE/"source"/name
    source_hashes = {name:base.digest(path) for name,path in source_files.items()}
    base.require(base.digest(REFERENCE/"fit/summary.json") == photo.PREDECESSOR_FIT_SHA,"predecessor final fit changed")
    predecessor = base.read_json(REFERENCE/"fit/summary.json")
    weights = REFERENCE/"fit/appearance/weights.npz"
    base.require(base.digest(weights) == predecessor["arms"]["appearance"]["weights_sha256"],"identical-reference historical weights changed")
    with np.load(REFERENCE/"training_schedule.npz",allow_pickle=False) as data:
        ids = data["indices"]
    trace = photo.expected_trace(ids,guard.photometric)
    review = {"schema":"portable_e2e.map_photometric_train_review.v1","review_note":review_note,
        "reviewer":"assistant visual input QA; not independent human certification",
        "preview_receipt_sha256":base.digest(PREVIEW/"receipt.json"),"preview_samples":preview_receipt["samples"],
        "prior_adverse_evidence_reviewed":prior_evidence(),"no_ctrack_tuning":True}
    OUTPUT.mkdir(parents=True)
    write_new(OUTPUT/"preview_review.json",review)
    shared = {"study_id":photo.NAME,"reference_package":{"study_id":base.NAME,"manifest_sha256":photo.REFERENCE_MANIFEST_SHA,
        "registry_sha256":photo.REFERENCE_REGISTRY_SHA,"source_bundle_sha256":photo.REFERENCE_SOURCE_SHA},
        "counts":{"original":729,"paired_true":480,"per_arm":1209},
        "arms":{"control":"original729+TRUE480; no photo","photometric":"same pool; archived photo on TRUE480 only"},
        "photometric_policy":photo.policy(),"parent_weights_sha256":base.PARENT_SHA,"fresh_fits":2,
        "ctrack_train_eligible":False,"broad_dataset_admitted":False,"preview_review_sha256":base.digest(OUTPUT/"preview_review.json")}
    registry = {"schema":"portable_e2e.study_train_registration.v1",**shared,
        "registered_at_utc":datetime.now(timezone.utc).isoformat(),"study_train_authorized":True,
        "authorization":"Parent approved preparation of this bounded pure-photometric fresh A/B; root owns launch after review and W pair completion.",
        "source_diagnostic_receipts_preserved":True,"source_study_registry_modified":False,"dev_or_test_used_for_fit":False,
        "optimization":{"steps_per_arm":10000,"batch":8,"seed":base.SEED,"learning_rate":base.LR,
            "weight_decay":base.WEIGHT_DECAY,"optimizer":"fresh AdamW in both arms"},
        "training_plan_sha256":source_hashes["plan.md"],"prior_adverse_evidence":prior_evidence()}
    write_new(OUTPUT/"study_train_registry.json",registry)
    manifest = {"schema":"portable_e2e.map_photometric_reference_data.v1",**shared,
        "local_data_files":{},"study_registry_sha256":base.digest(OUTPUT/"study_train_registry.json"),
        "expected_photo_trace":trace,"predecessor_fit_summary_sha256":photo.PREDECESSOR_FIT_SHA,
        "predecessor_appearance_weights_sha256":base.digest(weights),"predecessor_appearance_state_sha256":tensor_digest(weights),
        "reference_paths":{"local":str(REFERENCE),"remote":"/home/pro6000/personal/hwanhong/dataset/"+base.NAME},
        **{"reference_"+key:reference[key] for key in ("files","schedule_contract","common_class_pixels","common_class_weights","paired_samples_sha256")},
        "builder_source_sha256":base.digest(Path(__file__).resolve())}
    photo.validate_study(manifest,registry,reference)
    write_new(OUTPUT/"manifest.json",manifest)
    for name,path in source_files.items():
        base.require(base.digest(path) == source_hashes[name],"new source changed during preparation")
        copy_new(path,OUTPUT/"source"/name)
    copy_new(Path(__file__).resolve(),OUTPUT/"provenance"/Path(__file__).name)
    copy_new(ROOT/"learning/tests/test_map_photometric_matched_20261001.py",OUTPUT/"provenance/test_map_photometric_matched_20261001.py")
    copy_new(PREVIEW/"receipt.json",OUTPUT/"provenance/train_preview_receipt.json")
    for row in preview_receipt["samples"]:
        copy_new(PREVIEW/row["png"],OUTPUT/"provenance/train_previews"/row["png"])
    for index,path in enumerate(prior_evidence()["files"]):
        copy_new(Path(path),OUTPUT/"provenance/prior_policy"/(str(index)+"_"+Path(path).name))
    bundle = {"schema":"portable_e2e.map_photometric_matched_source.v1","study_id":photo.NAME,
        "files":source_hashes,"data_manifest_sha256":base.digest(OUTPUT/"manifest.json"),
        "study_registry_sha256":base.digest(OUTPUT/"study_train_registry.json")}
    write_new(OUTPUT/"source/bundle.json",bundle)
    reference_binding()
    result = {"status":"THIN_PHOTOMETRIC_STUDY_READY_NOT_FITTED","output":str(OUTPUT),
        "manifest_sha256":bundle["data_manifest_sha256"],"registry_sha256":bundle["study_registry_sha256"],
        "source_bundle_sha256":base.digest(OUTPUT/"source/bundle.json"),"large_arrays_copied":False,"gpu_used":False,
        "photo_trace":trace,"fresh_fits":2,"ctrack_train_eligible":False}
    write_new(OUTPUT/"build_result.json",result)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--preview",action="store_true"); mode.add_argument("--execute",action="store_true")
    parser.add_argument("--review-note")
    args = parser.parse_args()
    print(json.dumps(preview() if args.preview else build(args.review_note),sort_keys=True),flush=True)
