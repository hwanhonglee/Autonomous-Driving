"""Compare frozen A and rejected C on the pinned Town05_Opt right p00 Epic DEV capture.

The right-turn sign scout failed its sign-capable gate. This run reports road and
lane diagnostics only; it cannot promote a model or admit a dataset.
"""
from __future__ import annotations

import argparse
import io
import json
import math
import os
from pathlib import Path
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from scripts.e2e import audit_map_town05_sign_dev_scout as capture_audit
from scripts.e2e import evaluate_map_town05_dev as prior


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/map_town05_right_dev_eval"
PLAN = OUTPUT / "PLAN.md"
PLAN_SHA256 = "29608b3276a022a7dbdb762af44176d47ec113b4da058222066c3efc9b63d9c2"
AUDIT = ROOT / "docs/assets/validation/2026-09-27/map_town05_sign_dev_scout/summary.json"
AUDIT_SHA256 = "201d9e3dc6a7ecff46da485682afcc73682cdb3d045b7579920d6b94670d5050"
AUDIT_SOURCE_SHA256 = "8ab683faf85c096724f803c7dec28e0391a18a22468d1c2490af9d9152b63363"
AUDIT_HELPER_SHA256 = "7d2d50ccc9191da44c50162f924c1cbbccfbf9eb06dbac88a7e124fd7e5dc81a"
PRIOR_SOURCE_SHA256 = "ca06d9add59695e5e668147fc3600e9e068635d6599b9249a6d6c4b6d06462ef"
CAPTURE = ROOT / "storage/experiments/training/2026-09-27/map_town05_right_sign_dev_scout_epic_v1/mapping"
PARENT = prior.PARENT
PARENT_SHA256 = prior.PARENT_SHA256
C_ROOT = prior.C_ROOT
C_CHECKPOINT = prior.C_CHECKPOINT
C_SHA256 = prior.C_SHA256
MODEL_SOURCE = prior.MODEL_SOURCE
MODEL_SOURCE_SHA256 = prior.MODEL_SOURCE_SHA256
C_STARTED_SHA256 = "7e11781ab16240543e93a475a39e571fd0062279cadf447c0eaecb87712811be"
C_SUMMARY_SHA256 = "37d6ab4efcf05ba8bd604bf888314dd07d9d1a8d5d3ee3fed7f632d9fa82eac3"
LABELS = prior.LABELS
SELECTED = ("frames/0000", "frames/0105", "frames/0203")
MODEL_NAMES = ("ai_v2", "candidate_c_rejected")
RESULT_NAMES = ("predictions.npz", "full_fov_0000.png", "full_fov_0105.png",
                "full_fov_0203.png", "summary.json")


def require(condition: bool, message: str) -> None:
    prior.require(condition, message)


def fixed_source_checks() -> None:
    """Check the registered plan and every reused code/model/receipt byte source."""
    require(OUTPUT.is_dir() and not OUTPUT.is_symlink(), "registered output directory missing")
    require(prior.digest(PLAN) == PLAN_SHA256, "registered PLAN changed")
    fixed = (
        (AUDIT, AUDIT_SHA256),
        (ROOT / "tools/e2e/audit_map_town05_sign_dev_scout.py", AUDIT_SOURCE_SHA256),
        (ROOT / "tools/e2e/audit_map_generalization_town03.py", AUDIT_HELPER_SHA256),
        (ROOT / "tools/e2e/evaluate_map_town05_dev.py", PRIOR_SOURCE_SHA256),
        (PARENT, PARENT_SHA256), (C_CHECKPOINT, C_SHA256),
        (MODEL_SOURCE, MODEL_SOURCE_SHA256),
        (C_ROOT / "source/map_perception.py", MODEL_SOURCE_SHA256),
        (C_ROOT / "started.json", C_STARTED_SHA256),
        (C_ROOT / "summary.json", C_SUMMARY_SHA256),
    )
    for path, expected in fixed:
        require(prior.digest(path) == expected, f"fixed source changed: {path}")
    require(CAPTURE.is_dir() and not CAPTURE.is_symlink(), "right p00 capture missing")
    started = prior.read_json(C_ROOT / "started.json")
    completed = prior.read_json(C_ROOT / "summary.json")
    require(started["parent_checkpoint_sha256"] == PARENT_SHA256
            and started["steps"] == 2000
            and started["automatic_promotion"] is False
            and completed["checkpoint_sha256"] == C_SHA256
            and completed["deployment_approved"] is False,
            "rejected C provenance changed")


def source_audit() -> tuple[dict, list[int]]:
    """Recompute the complete owner/native-frame audit before any model inference."""
    fixed_source_checks()
    recorded = prior.read_json(AUDIT)
    require(capture_audit.build_summary() == recorded, "right p00 capture drifted from audit")
    require(recorded["schema"] == "portable_e2e.map_town05_sign_dev_scout_audit.v1"
            and recorded["status"] == "REJECTED_SIGN_CAPABLE_DEV"
            and recorded["provenance"]["town"] == "Town05_Opt"
            and recorded["provenance"]["quality"] == "Epic"
            and recorded["provenance"]["route_sha256"]
            == "2f7a25e8ace753bfdd412ae7ef65710a001d1f4b1ecd5765f3ee7b021ced29e3"
            and recorded["provenance"]["owner_plan_sha256"]
            == "caf6c1d24819b08d3829e80d893e03f4ebe2903245336044d6b3aa1f57afd8a7"
            and recorded["provenance"]["mapping_summary_sha256"]
            == "9c6b66385164a7df5dbb85ba973047e89ccb7d44cbfea1ba84f28952674686f8"
            and recorded["raw_frames"] == len(recorded["frames"]) == 240
            and recorded["retained_frames_0p25m"] == len(recorded["retained_stems"]) == 195
            and recorded["native_frame_wh"] == [640, 360]
            and tuple(recorded["labels"]) == LABELS
            and recorded["retained_class_pixels"][LABELS.index("traffic_sign")] == 57
            and recorded["retained_max_whole_frame_tag8_pixels"] == 3
            and recorded["sign_dev_gate_passed"] is False
            and recorded["dataset_admitted"] is False,
            "right p00 DEV diagnostic scope changed")
    retained_stems = recorded["retained_stems"]
    require(tuple(retained_stems[i] for i in (0, 97, 194)) == SELECTED,
            "preselected full-FOV frame ordinals changed")
    positions = {row["stem"]: index for index, row in enumerate(recorded["frames"])}
    require(len(positions) == 240 and all(stem in positions for stem in retained_stems),
            "raw/retained stem mismatch")
    retained = sorted(positions[stem] for stem in retained_stems)
    require(len(retained) == 195, "retained frame count changed")
    return recorded, retained


def panel(stem: str, rgb: np.ndarray, target: np.ndarray,
          parent: np.ndarray, candidate: np.ndarray) -> Image.Image:
    """Display four native 640x360 fields of view with no crop or resize."""
    require(rgb.shape == (360, 640, 3) and all(mask.shape == (360, 640)
            for mask in (target, parent, candidate)), "native full FOV required")
    font_path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    font = ImageFont.truetype(str(font_path), 16) if font_path.is_file() else ImageFont.load_default()
    canvas = Image.new("RGB", (1280, 816), (18, 23, 32))
    draw = ImageDraw.Draw(canvas)
    draw.text((12, 8), f"Town05_Opt RIGHT p00 Epic DEV {stem} | CPU A vs rejected C | road/lane only",
              fill="white", font=font)
    panels = (("Original RGB input", rgb), ("CARLA oracle, metrics only", prior.COLORS[target]),
              ("AI v2 A, frozen", prior.COLORS[parent]),
              ("C, rejected; no promotion", prior.COLORS[candidate]))
    for index, (title, image) in enumerate(panels):
        x = 640 * (index % 2)
        y = 45 + 385 * (index // 2)
        draw.text((x + 10, y), title, fill="white", font=font)
        canvas.paste(Image.fromarray(image), (x, y + 24))
    return canvas


def write_new(path: Path, payload: bytes) -> None:
    require(path.parent == OUTPUT and not path.exists(), f"output already exists: {path}")
    with path.open("xb") as stream:
        stream.write(payload)


def road_lane_gate(parent: dict, candidate: dict) -> dict:
    """Apply the registered inclusive pp limits with floating-point boundary tolerance."""
    gate = prior.delta_and_gate(parent, candidate)
    road = gate["road_iou_delta_c_minus_a_pp"]
    lane = gate["lane_iou_delta_c_minus_a_pp"]
    gate["town05_directional_nonregression"] = (
        (road >= -1.0 or math.isclose(road, -1.0, rel_tol=0, abs_tol=1e-12))
        and (lane >= -2.0 or math.isclose(lane, -2.0, rel_tol=0, abs_tol=1e-12))
    )
    return gate


def evaluate() -> None:
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == "", "CUDA_VISIBLE_DEVICES='' required")
    require(all(not (OUTPUT / name).exists() for name in RESULT_NAMES),
            "evaluation output exists; never overwrite it")
    recorded, retained = source_audit()
    print("TOWN05_RIGHT_PREFLIGHT_PASS_CPU_ONLY", flush=True)
    import torch

    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    require(not torch.cuda.is_available(), "CPU-only execution required")
    models = {"ai_v2": prior.load_model(PARENT, PARENT_SHA256),
              "candidate_c_rejected": prior.load_model(C_CHECKPOINT, C_SHA256)}
    predictions = {name: np.empty((240, 360, 640), dtype=np.uint8) for name in models}
    raw = {name: np.zeros((7, 7), dtype=np.int64) for name in models}
    kept = {name: np.zeros((7, 7), dtype=np.int64) for name in models}
    retained_set = set(retained)
    examples: dict[str, dict[str, np.ndarray]] = {}
    started = time.monotonic()
    print("TOWN05_RIGHT_INFERENCE_START", flush=True)
    with torch.no_grad():
        for index, row in enumerate(recorded["frames"]):
            stem = row["stem"]
            rgb_path, semantic_path = CAPTURE / (stem + ".png"), CAPTURE / (stem + ".npz")
            require(prior.digest(rgb_path) == row["rgb_sha256"]
                    and prior.digest(semantic_path) == row["semantic_npz_sha256"],
                    f"frame bytes changed during inference: {stem}")
            with Image.open(rgb_path) as image:
                require(image.mode == "RGB" and image.size == (640, 360), "native RGB required")
                rgb = np.asarray(image).copy()
            with np.load(semantic_path, allow_pickle=False) as sensor:
                target = prior.semantic_target(sensor["semantic"])
            require(np.bincount(target.ravel(), minlength=7).tolist() == row["class_pixels"],
                    f"oracle pixels changed: {stem}")
            tensor = torch.from_numpy(rgb.transpose(2, 0, 1).copy()).unsqueeze(0).float().div_(255)
            if stem in SELECTED:
                examples[stem] = {"rgb": rgb, "target": target}
            for name, model in models.items():
                output = model(tensor)
                require(tuple(output.shape) == (1, 7, 360, 640), "model output ABI changed")
                prediction = output.argmax(1).squeeze(0).numpy().astype(np.uint8)
                predictions[name][index] = prediction
                confusion = prior.confusion_for(target, prediction)
                raw[name] += confusion
                if index in retained_set:
                    kept[name] += confusion
                if stem in SELECTED:
                    examples[stem][name] = prediction.copy()
            if (index + 1) % 40 == 0:
                print(f"CPU_TOWN05_RIGHT_FRAMES {index + 1}/240", flush=True)
    require(set(examples) == set(SELECTED), "selected examples missing")
    require(all(int(raw[name].sum()) == 240 * 360 * 640
                and int(kept[name].sum()) == 195 * 360 * 640 for name in MODEL_NAMES),
            "confusion denominator changed")
    summaries = {name: {"raw_240": prior.class_metrics(raw[name]),
                        "retained_195": prior.class_metrics(kept[name])} for name in MODEL_NAMES}
    gate = road_lane_gate(summaries["ai_v2"]["retained_195"],
                          summaries["candidate_c_rejected"]["retained_195"])
    require(gate["sign_used_for_gate"] is False and gate["automatic_promotion"] is False
            and gate["deployment_approved"] is False, "nonpromotion gate changed")
    npz = io.BytesIO()
    np.savez_compressed(npz, ai_v2=predictions["ai_v2"],
                        candidate_c_rejected=predictions["candidate_c_rejected"],
                        retained_indices=np.asarray(retained, dtype=np.int64),
                        stems=np.asarray([row["stem"] for row in recorded["frames"]]))
    pictures = {}
    for stem in SELECTED:
        row = examples[stem]
        output = panel(stem, row["rgb"], row["target"], row["ai_v2"],
                       row["candidate_c_rejected"])
        buffer = io.BytesIO()
        output.save(buffer, format="PNG")
        pictures[f"full_fov_{stem.split('/')[-1]}.png"] = buffer.getvalue()
    write_new(OUTPUT / "predictions.npz", npz.getvalue())
    for filename, payload in pictures.items():
        write_new(OUTPUT / filename, payload)
    summary = {
        "schema": "portable_e2e.map_town05_right_dev_eval.v1",
        "status": "ROAD_LANE_DEV_DIAGNOSTIC_C_REMAINS_REJECTED",
        "town": "Town05_Opt", "route": "right p00", "quality": "Epic",
        "capture_travel_m": recorded["provenance"]["travel_m"],
        "capture_goal_reached_by": "CARLA BasicAgent",
        "input": "native 640x360 RGB / 255; CARLA semantic only for metrics",
        "raw_frames": 240, "retained_frames": 195,
        "retention_rule": recorded["retention_rule"],
        "labels": LABELS, "models": summaries, "gate": gate,
        "sign_diagnostic_only": True,
        "sign_positive_pixels_retained": 57,
        "sign_max_whole_frame_pixels_retained": 3,
        "sign_dev_gate_passed": False,
        "selected_retained_ordinals": (0, 97, 194),
        "selected_full_fov_stems": SELECTED,
        "source_sha256": {
            "plan": PLAN_SHA256, "capture_audit": AUDIT_SHA256,
            "capture_audit_source": AUDIT_SOURCE_SHA256,
            "capture_audit_helper": AUDIT_HELPER_SHA256,
            "prior_evaluator": PRIOR_SOURCE_SHA256,
            "route": recorded["provenance"]["route_sha256"],
            "owner_plan": recorded["provenance"]["owner_plan_sha256"],
            "mapping_summary": recorded["provenance"]["mapping_summary_sha256"],
            "ai_v2_checkpoint": PARENT_SHA256, "candidate_c_checkpoint": C_SHA256,
            "c_started": C_STARTED_SHA256, "c_summary": C_SUMMARY_SHA256,
            "model_source": MODEL_SOURCE_SHA256, "evaluator": prior.digest(Path(__file__)),
        },
        "predictions_sha256": prior.digest(OUTPUT / "predictions.npz"),
        "full_fov_sha256": {name: prior.digest(OUTPUT / name) for name in pictures},
        "wall_seconds_inference_and_metrics": time.monotonic() - started,
        "cpu_threads": 2, "gpu_used": False, "training_executed": False,
        "blind_independent_test": False, "automatic_promotion": False,
        "deployment_approved": False, "hd_map_approved": False,
        "limitations": "One correlated Town05 right p00 ClearNoon DEV route; tiny sign truth (max 3 px/frame); no blind TEST, map, learned-control, or real-vehicle approval",
    }
    write_new(OUTPUT / "summary.json", (json.dumps(summary, indent=2, ensure_ascii=False) + "\n").encode())
    print("TOWN05_RIGHT_ROAD_LANE_DEV_COMPLETE_NO_PROMOTION", flush=True)


def verify_saved() -> None:
    """Independently reconstruct both frame sets' confusion from saved masks and oracle."""
    recorded, retained = source_audit()
    require(all((OUTPUT / name).is_file() and not (OUTPUT / name).is_symlink()
                for name in RESULT_NAMES), "completed immutable artifacts required")
    summary = prior.read_json(OUTPUT / "summary.json")
    require(summary["source_sha256"]["evaluator"] == prior.digest(Path(__file__))
            and summary["source_sha256"]["plan"] == PLAN_SHA256
            and summary["predictions_sha256"] == prior.digest(OUTPUT / "predictions.npz")
            and summary["status"] == "ROAD_LANE_DEV_DIAGNOSTIC_C_REMAINS_REJECTED",
            "saved summary provenance changed")
    for filename, expected in summary["full_fov_sha256"].items():
        require(prior.digest(OUTPUT / filename) == expected, "saved full-FOV image changed")
    with np.load(OUTPUT / "predictions.npz", allow_pickle=False) as saved:
        require(set(saved.files) == {*MODEL_NAMES, "retained_indices", "stems"},
                "prediction archive keys changed")
        require(np.array_equal(saved["retained_indices"], np.asarray(retained))
                and saved["stems"].tolist() == [row["stem"] for row in recorded["frames"]],
                "prediction frame order or retention changed")
        matrices = {name: {"raw_240": np.zeros((7, 7), dtype=np.int64),
                           "retained_195": np.zeros((7, 7), dtype=np.int64)}
                    for name in MODEL_NAMES}
        retained_set = set(retained)
        for index, row in enumerate(recorded["frames"]):
            with np.load(CAPTURE / (row["stem"] + ".npz"), allow_pickle=False) as source:
                target = prior.semantic_target(source["semantic"])
            for name in MODEL_NAMES:
                prediction = saved[name][index]
                confusion = prior.confusion_for(target, prediction)
                matrices[name]["raw_240"] += confusion
                if index in retained_set:
                    matrices[name]["retained_195"] += confusion
        for name in MODEL_NAMES:
            require(saved[name].shape == (240, 360, 640)
                    and saved[name].dtype == np.uint8, "saved native masks changed")
            for part in ("raw_240", "retained_195"):
                require(matrices[name][part].tolist() == summary["models"][name][part]["confusion"],
                        f"saved {name} {part} confusion mismatch")
        for stem in SELECTED:
            index = next(i for i, row in enumerate(recorded["frames"]) if row["stem"] == stem)
            with Image.open(CAPTURE / (stem + ".png")) as raw:
                rgb = np.asarray(raw)
            with np.load(CAPTURE / (stem + ".npz"), allow_pickle=False) as source:
                target = prior.semantic_target(source["semantic"])
            with Image.open(OUTPUT / f"full_fov_{stem[-4:]}.png") as image:
                display = np.asarray(image)
            require(display.shape == (816, 1280, 3)
                    and np.array_equal(display[69:429, 0:640], rgb)
                    and np.array_equal(display[69:429, 640:1280], prior.COLORS[target])
                    and np.array_equal(display[454:814, 0:640], prior.COLORS[saved["ai_v2"][index]])
                    and np.array_equal(display[454:814, 640:1280],
                                       prior.COLORS[saved["candidate_c_rejected"][index]]),
                    f"saved full-FOV panel pixels changed: {stem}")
    require(summary["gate"]["sign_used_for_gate"] is False
            and summary["automatic_promotion"] is False
            and summary["deployment_approved"] is False,
            "saved nonpromotion status changed")
    print("TOWN05_RIGHT_SAVED_MASKS_CONFUSIONS_AND_FOV_VERIFY_PASS", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()
    require(not (args.preflight_only and args.verify_only), "choose one verification mode")
    if args.verify_only:
        verify_saved()
    elif args.preflight_only:
        require(all(not (OUTPUT / name).exists() for name in RESULT_NAMES),
                "evaluation outputs already exist")
        source_audit()
        print("TOWN05_RIGHT_PREFLIGHT_PASS_NO_INFERENCE", flush=True)
    else:
        evaluate()


if __name__ == "__main__":
    main()
