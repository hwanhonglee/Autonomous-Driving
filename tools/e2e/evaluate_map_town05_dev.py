"""HH_260906 - Compare two frozen map models on the pre-registered Town05 Epic DEV capture."""
from __future__ import annotations

import hashlib
import inspect
import io
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from scripts.e2e import audit_map_town05_dev_capture as capture_audit


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/map_town05_dev_eval"
PLAN = OUTPUT / "PLAN.md"
PLAN_SHA256 = "1c6ecba4f3cf85df1df833414cd53810b8b376c227c687a4345997683ae2644e"
AUDIT = ROOT / "docs/assets/validation/2026-09-27/map_town05_dev_capture/epic_v2_audit.json"
AUDIT_SHA256 = "d64d77bf4f1d02892f59ac9debd18489c5acbf38842ec81ac4398f7052b950d1"
CAPTURE = ROOT / "storage/experiments/training/2026-09-27/map_town05_independent_dev_capture_epic_v2/mapping"
PARENT = ROOT / "storage/experiments/training/2026-09-26/dual_data_ai_v1/mapping_ai_v2_checkpoint/checkpoint.pt"
PARENT_SHA256 = "ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380"
C_ROOT = ROOT / "docs/assets/validation/2026-09-27/map_town_rehearsal_c/remote_run_v2"
C_CHECKPOINT = C_ROOT / "checkpoint.pt"
C_SHA256 = "864de725655ef7219218f2b4a9da059630e6e3c30a18a010f74238938aa4a7f6"
MODEL_SOURCE = ROOT / "learning/portable_e2e/map_perception.py"
MODEL_SOURCE_SHA256 = "1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b"
LABELS = ("other", "road", "sidewalk", "lane_marking", "traffic_light_fixture", "traffic_sign", "pole")
TAG_TABLE = {1: "road", 24: "lane_marking", 2: "sidewalk", 7: "traffic_light_fixture", 8: "traffic_sign", 6: "pole"}
COLORS = np.array([[30, 30, 40], [67, 92, 123], [141, 88, 147], [246, 222, 90],
                   [246, 100, 70], [60, 220, 180], [179, 190, 199]], dtype=np.uint8)
SELECTED = ("frames/0000", "frames/0123", "frames/0239")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def digest(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    output = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            output.update(block)
    return output.hexdigest()


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def preflight() -> tuple[dict, list[int]]:
    """HH_260906 - Fail closed on any source, audit, model or split deviation before inference."""
    require(OUTPUT.is_dir() and not OUTPUT.is_symlink(), "pre-registered output directory required")
    require(digest(PLAN) == PLAN_SHA256, "pre-registered PLAN changed")
    require(digest(AUDIT) == AUDIT_SHA256, "Town05 Epic audit changed")
    require(CAPTURE.is_dir() and not CAPTURE.is_symlink(), "Town05 Epic mapping source missing")
    require(all(not (OUTPUT / name).exists() for name in
                ("summary.json", "predictions.npz", "full_fov_0000.png", "full_fov_0123.png", "full_fov_0239.png")),
            "evaluation output already exists; never overwrite it")
    recorded = read_json(AUDIT)
    fresh = capture_audit.build_summary()
    require(fresh == recorded, "source has drifted from the complete Epic audit")
    require(recorded["schema"] == "portable_e2e.map_town05_dev_capture_audit.v1"
            and recorded["status"] == "CAPTURE_AUDIT_PASS_NOT_ADMITTED"
            and recorded["provenance"]["town"] == "Town05_Opt"
            and recorded["provenance"]["quality"] == "Epic"
            and recorded["raw_frames"] == len(recorded["frames"]) == 240
            and recorded["retained_frames_0p25m"] == len(recorded["retained_stems"]) == 232
            and recorded["retained_class_pixels"][LABELS.index("traffic_sign")] == 256
            and all(row["class_pixels"][LABELS.index("traffic_sign")] in (1, 2)
                    for row in recorded["frames"])
            and tuple(recorded["labels"]) == LABELS
            and recorded["native_frame_wh"] == [640, 360]
            and recorded["dataset_admitted"] is False,
            "pre-registered Town05 DEV scope differs")
    retained = set(recorded["retained_stems"])
    require(len(retained) == 232 and all(name in retained for name in SELECTED)
            and tuple(recorded["retained_stems"][i] for i in (0, 115, 231)) == SELECTED,
            "preselected full-FOV examples or retention changed")
    require(digest(PARENT) == PARENT_SHA256 and digest(C_CHECKPOINT) == C_SHA256
            and digest(MODEL_SOURCE) == MODEL_SOURCE_SHA256
            and digest(C_ROOT / "source/map_perception.py") == MODEL_SOURCE_SHA256,
            "frozen model bytes changed")
    candidate_started = read_json(C_ROOT / "started.json")
    candidate_summary = read_json(C_ROOT / "summary.json")
    require(candidate_started["parent_checkpoint_sha256"] == PARENT_SHA256
            and candidate_started["steps"] == 2000
            and candidate_started["automatic_promotion"] is False
            and candidate_summary["checkpoint_sha256"] == C_SHA256
            and candidate_summary["deployment_approved"] is False,
            "rejected C provenance differs")
    positions = {row["stem"]: index for index, row in enumerate(recorded["frames"])}
    require(len(positions) == 240 and all(stem in positions for stem in retained),
            "raw/retained stem mismatch")
    return recorded, sorted(positions[stem] for stem in retained)


def semantic_target(semantic: np.ndarray) -> np.ndarray:
    """HH_260906 - Preserve the seven-label CARLA tag table, treating all other tags as other."""
    require(semantic.shape == (360, 640, 4) and semantic.dtype == np.uint8,
            "native uint8 semantic frame required")
    lut = np.zeros(256, dtype=np.uint8)
    for tag, name in TAG_TABLE.items():
        lut[tag] = LABELS.index(name)
    return lut[semantic[..., 2]]


def confusion_for(target: np.ndarray, prediction: np.ndarray) -> np.ndarray:
    """HH_260906 - Put truth on rows and predictions on columns without resampling pixels."""
    require(target.shape == prediction.shape == (360, 640)
            and target.dtype == prediction.dtype == np.uint8
            and int(target.max()) < len(LABELS) and int(prediction.max()) < len(LABELS),
            "native seven-class masks required")
    return np.bincount((target.astype(np.int64) * len(LABELS) + prediction).ravel(),
                       minlength=len(LABELS) ** 2).reshape(len(LABELS), len(LABELS))


def class_metrics(confusion: np.ndarray) -> dict:
    """HH_260906 - Report exact IoU/precision/recall and TP/FP/FN for each frozen class."""
    require(confusion.shape == (len(LABELS), len(LABELS))
            and (confusion >= 0).all() and int(confusion.sum()) > 0,
            "nonempty seven-class confusion required")
    classes = {}
    for index, name in enumerate(LABELS):
        tp = int(confusion[index, index])
        target = int(confusion[index].sum())
        predicted = int(confusion[:, index].sum())
        fp, fn = predicted - tp, target - tp
        union = tp + fp + fn
        classes[name] = dict(target_pixels=target, predicted_pixels=predicted,
                             true_positive_pixels=tp, false_positive_pixels=fp,
                             false_negative_pixels=fn,
                             precision=tp / predicted if predicted else None,
                             recall=tp / target if target else None,
                             iou=tp / union if union else None)
    present = [row["iou"] for row in classes.values() if row["iou"] is not None]
    return dict(classes=classes, confusion=confusion.tolist(),
                mean_iou_present=float(np.mean(present)),
                pixel_accuracy=float(np.trace(confusion) / confusion.sum()))


def delta_and_gate(parent: dict, candidate: dict) -> dict:
    """HH_260906 - A directional Town05 diagnostic cannot overturn C's earlier DEV rejection."""
    delta = {}
    for name in ("road", "lane_marking"):
        a = parent["classes"][name]["iou"]
        c = candidate["classes"][name]["iou"]
        require(a is not None and c is not None and math.isfinite(a) and math.isfinite(c),
                f"{name} IoU unavailable")
        delta[name] = 100 * (c - a)
    nonregression = delta["road"] >= -1.0 and delta["lane_marking"] >= -2.0
    return dict(road_iou_delta_c_minus_a_pp=delta["road"],
                lane_iou_delta_c_minus_a_pp=delta["lane_marking"],
                town05_directional_nonregression=nonregression,
                town01_dev_protection_passed=False,
                automatic_promotion=False, deployment_approved=False,
                sign_used_for_gate=False)


def load_model(path: Path, expected_sha256: str):
    """HH_260906 - CPU-load only a hash-pinned owned checkpoint with strict model ABI checks."""
    import torch
    from portable_e2e.map_perception import MapPerception

    require(digest(path) == expected_sha256, "checkpoint changed before load")
    options = {"map_location": "cpu"}
    if "weights_only" in inspect.signature(torch.load).parameters:
        options["weights_only"] = True
    checkpoint = torch.load(path, **options)
    require(tuple(checkpoint["labels"]) == LABELS
            and checkpoint["input_shape"] == [3, 360, 640]
            and checkpoint["steps"] == 2000
            and checkpoint["deployable"] is False,
            "frozen checkpoint ABI differs")
    model = MapPerception().cpu().eval()
    model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    require(sum(parameter.numel() for parameter in model.parameters()) == 130503,
            "frozen architecture differs")
    return model


def full_fov_panel(stem: str, rgb: np.ndarray, target: np.ndarray,
                   parent: np.ndarray, candidate: np.ndarray) -> Image.Image:
    """HH_260906 - Preserve all four 640x360 fields of view without selecting on model success."""
    require(all(array.shape[:2] == (360, 640) for array in (rgb, target, parent, candidate)),
            "native full FOV required")
    font_path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    font = ImageFont.truetype(str(font_path), 16) if font_path.is_file() else ImageFont.load_default()
    canvas = Image.new("RGB", (1280, 816), (18, 23, 32))
    draw = ImageDraw.Draw(canvas)
    draw.text((12, 8), f"Town05_Opt Epic DEV {stem} | CPU A vs rejected C | NOT blind TEST or HD-map approval",
              fill="white", font=font)
    panels = (("Original RGB input", rgb), ("CARLA oracle, metrics only", COLORS[target]),
              ("AI v2 A, frozen", COLORS[parent]), ("C, rejected", COLORS[candidate]))
    for index, (title, image) in enumerate(panels):
        x = 640 * (index % 2)
        y = 45 + 385 * (index // 2)
        draw.text((x + 10, y), title, fill="white", font=font)
        canvas.paste(Image.fromarray(image), (x, y + 24))
    return canvas


def write_new(path: Path, payload: bytes) -> None:
    """HH_260906 - Create output files exclusively; never replace a source or previous result."""
    require(path.parent == OUTPUT and not path.exists(), f"output already exists: {path}")
    with path.open("xb") as stream:
        stream.write(payload)


def main() -> None:
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == "", "explicit CUDA_VISIBLE_DEVICES='' required")
    recorded, retained = preflight()
    import torch

    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    require(not torch.cuda.is_available(), "CPU-only execution required")
    models = {"ai_v2": load_model(PARENT, PARENT_SHA256),
              "candidate_c_rejected": load_model(C_CHECKPOINT, C_SHA256)}
    predictions = {name: np.empty((240, 360, 640), dtype=np.uint8) for name in models}
    raw = {name: np.zeros((7, 7), dtype=np.int64) for name in models}
    kept = {name: np.zeros((7, 7), dtype=np.int64) for name in models}
    retained_set = set(retained)
    examples = {}
    started = time.monotonic()
    with torch.no_grad():
        for index, row in enumerate(recorded["frames"]):
            stem = row["stem"]
            rgb_path = CAPTURE / (stem + ".png")
            semantic_path = CAPTURE / (stem + ".npz")
            require(digest(rgb_path) == row["rgb_sha256"]
                    and digest(semantic_path) == row["semantic_npz_sha256"],
                    f"input changed during inference: {stem}")
            with Image.open(rgb_path) as image:
                require(image.mode == "RGB" and image.size == (640, 360), "native RGB required")
                rgb = np.asarray(image).copy()
            with np.load(semantic_path, allow_pickle=False) as source:
                target = semantic_target(source["semantic"])
            require(np.bincount(target.ravel(), minlength=7).tolist() == row["class_pixels"],
                    f"oracle pixels changed during inference: {stem}")
            tensor = torch.from_numpy(rgb.transpose(2, 0, 1).copy()).unsqueeze(0).float().div_(255)
            if stem in SELECTED:
                examples[stem] = {"rgb": rgb, "target": target}
            for name, model in models.items():
                output = model(tensor)
                require(tuple(output.shape) == (1, 7, 360, 640), "model output ABI differs")
                predicted = output.argmax(1).squeeze(0).numpy().astype(np.uint8)
                predictions[name][index] = predicted
                confusion = confusion_for(target, predicted)
                raw[name] += confusion
                if index in retained_set:
                    kept[name] += confusion
                if stem in SELECTED:
                    examples[stem][name] = predicted.copy()
            if (index + 1) % 40 == 0:
                print(f"CPU_TOWN05_FRAMES {index + 1}/240", flush=True)
    require(set(examples) == set(SELECTED), "preselected full-FOV examples missing")
    require(all(int(raw[name].sum()) == 240 * 360 * 640
                and int(kept[name].sum()) == 232 * 360 * 640 for name in models),
            "confusion denominator changed")
    summaries = {name: {"raw_240": class_metrics(raw[name]),
                        "retained_232": class_metrics(kept[name])} for name in models}
    parent = summaries["ai_v2"]["retained_232"]
    candidate = summaries["candidate_c_rejected"]["retained_232"]
    gate = delta_and_gate(parent, candidate)
    sign_index = LABELS.index("traffic_sign")
    require(parent["classes"]["traffic_sign"]["target_pixels"] == 256
            and candidate["classes"]["traffic_sign"]["target_pixels"] == 256,
            "sign diagnostic denominator changed")
    npz = io.BytesIO()
    np.savez_compressed(npz, ai_v2=predictions["ai_v2"],
                        candidate_c_rejected=predictions["candidate_c_rejected"],
                        retained_indices=np.asarray(retained, dtype=np.int64),
                        stems=np.asarray([row["stem"] for row in recorded["frames"]]))
    pictures = {}
    for stem in SELECTED:
        frame = examples[stem]
        panel = full_fov_panel(stem, frame["rgb"], frame["target"],
                               frame["ai_v2"], frame["candidate_c_rejected"])
        buffer = io.BytesIO()
        panel.save(buffer, format="PNG")
        pictures[f"full_fov_{stem.split('/')[-1]}.png"] = buffer.getvalue()
    write_new(OUTPUT / "predictions.npz", npz.getvalue())
    for filename, payload in pictures.items():
        write_new(OUTPUT / filename, payload)
    summary = {
        "schema": "portable_e2e.map_town05_dev_eval.v1",
        "status": "EXPLORATORY_DEV_C_REMAINS_REJECTED",
        "town": "Town05_Opt", "quality": "Epic",
        "scope": "observed first 94.305m of one selected 209.773m straight route",
        "input": "original 640x360 RGB / 255; CARLA semantic only for metrics",
        "raw_frames": 240, "retained_frames": 232,
        "retention_rule": recorded["retention_rule"],
        "labels": LABELS, "models": summaries, "gate": gate,
        "sign_diagnostic_only": True, "sign_positive_pixels_retained": 256,
        "sign_pixels_per_raw_frame": "1 or 2",
        "selected_full_fov_stems": SELECTED,
        "source_sha256": {"plan": PLAN_SHA256, "capture_audit": AUDIT_SHA256,
                          "route": recorded["provenance"]["route_sha256"],
                          "owner_plan": recorded["provenance"]["owner_plan_sha256"],
                          "mapping_summary": recorded["provenance"]["mapping_summary_sha256"],
                          "ai_v2_checkpoint": PARENT_SHA256, "candidate_c_checkpoint": C_SHA256,
                          "model_source": MODEL_SOURCE_SHA256, "evaluator": digest(Path(__file__))},
        "predictions_sha256": digest(OUTPUT / "predictions.npz"),
        "full_fov_sha256": {name: digest(OUTPUT / name) for name in pictures},
        "wall_seconds_inference_and_metrics": time.monotonic() - started,
        "cpu_threads": 2, "gpu_used": False, "training_executed": False,
        "blind_independent_test": False, "automatic_promotion": False,
        "deployment_approved": False, "hd_map_approved": False,
        "limitations": "One selected Town05 DEV route/weather, correlated frames, incomplete route, minute oracle sign pixels; no Town07 TEST, HD-map, learned-control or real-vehicle approval",
    }
    write_new(OUTPUT / "summary.json", (json.dumps(summary, indent=2, ensure_ascii=False) + "\n").encode())
    print("TOWN05_DEV_EXPLORATORY_COMPLETE_NO_PROMOTION", flush=True)


if __name__ == "__main__":
    main()
