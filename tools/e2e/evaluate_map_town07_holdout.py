"""HH_260906 - Single-use sealed Town07 RGB map A/B, never a promotion or driving test."""

from __future__ import annotations

import argparse
import hashlib
import inspect
import json
import math
import os
from pathlib import Path
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
CATALOG = ROOT / ("storage/experiments/validation/2026-09-01/"
                  "autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/town07/catalog")
OWNER_ROOT = ROOT / "storage/experiments/validation/2026-09-27"
ROUTES = {
    "straight": {
        "owner": OWNER_ROOT / "map_generalization_town07_straight_p00_sealed",
        "route": CATALOG / "straight/routes/town07/straight/town07_straight_s0000_p00.json",
        "sha256": "804dc69cec5ebbc2edba70946694f312c77328b5cb5d7260b096af0775f34848",
    },
    "right": {
        "owner": OWNER_ROOT / "map_generalization_town07_right_p00_sealed",
        "route": CATALOG / "turn/routes/town07/right/town07_right_s0000_p00.json",
        "sha256": "384d0aca89517471aa58947bb6256edf8e18180138f276b935cd8e849a5e8410",
    },
}
PLAN = ROOT / "docs/assets/validation/2026-09-27/map_next_factor/PLAN.md"
FROZEN_CONTRACT = PLAN.parent / "town07_freeze.json"
PARENT = ROOT / "storage/experiments/training/2026-09-26/dual_data_ai_v1/mapping_ai_v2_checkpoint/checkpoint.pt"
PARENT_RESULT = ROOT / "docs/assets/validation/2026-09-26/dual_data_ai/results/mapping_ai_v2"
MODEL_SOURCE = ROOT / "learning/portable_e2e/map_perception.py"
DEFAULT_OUTPUT = ROOT / "docs/assets/validation/2026-09-27/map_town07_holdout_eval"
PARENT_SHA256 = "ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380"
PARENT_SUMMARY_SHA256 = "9f147a764a560a99d185eeffcb1b8e73c3bbb1ce60cdd058328e050c32dc0e78"
PARENT_STARTED_SHA256 = "f87cd848f03dacf86fabf3e223fb0118d8d09aa653160f089a3a894e6cd43c6f"
PARENT_MANIFEST_SHA256 = "e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c"
MODEL_SOURCE_SHA256 = "1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b"
LABELS = ("other", "road", "sidewalk", "lane_marking", "traffic_light_fixture", "traffic_sign", "pole")
TAG_TABLE = {1: "road", 24: "lane_marking", 2: "sidewalk", 7: "traffic_light_fixture",
             8: "traffic_sign", 6: "pole"}
COLORS = np.asarray([(30, 42, 57), (122, 129, 138), (159, 133, 173), (248, 211, 70),
                     (237, 93, 79), (229, 65, 185), (81, 190, 205)], dtype=np.uint8)
OWNER_HASH_KEYS = ("plan", "started", "result", "summary", "records", "ready", "after_capture", "stopped")


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise ValueError(reason)


def digest(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    hasher = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def scoped_relative_path(value: str) -> Path:
    """HH_260906 - Freeze receipts may only name explicit files within this workspace."""
    relative = Path(value)
    require(not relative.is_absolute() and ".." not in relative.parts,
            "freeze receipt path must be workspace-relative")
    path = ROOT / relative
    require(path.resolve().is_relative_to(ROOT.resolve()), "freeze receipt path escapes workspace")
    return path


def owner_metadata_paths(owner: Path) -> dict[str, Path]:
    mapping = owner / "mapping"
    return {
        "plan": owner / "owner_plan.json", "started": owner / "owner_started.json",
        "result": owner / "owner_result.json", "summary": mapping / "summary.json",
        "records": mapping / "records.json",
        "ready": owner / "lifecycle/ready.json",
        "after_capture": owner / "lifecycle/after_capture.json",
        "stopped": owner / "lifecycle/stopped.json",
    }


def require_hash_ledger(paths: dict[str, Path], ledger: dict[str, str], label: str) -> None:
    require(set(ledger) == set(paths), f"{label}: freeze hash inventory differs")
    for key, path in paths.items():
        require(digest(path) == ledger[key], f"{label}: frozen {key} SHA256 differs")


def verify_freeze_receipt(path: Path, *, unseal: bool) -> dict:
    """HH_260906 - No Town07 metadata or pixels are opened before this exact freeze gate."""
    require(unseal, "explicit --unseal is required before any Town07 access")
    require(path.is_file() and not path.is_symlink(), "regular frozen receipt required")
    contract = read_json(path)
    require(contract.get("schema") == "portable_e2e.map_town07_holdout_freeze.v1"
            and contract.get("status") == "FROZEN_BEFORE_HOLDOUT"
            and contract.get("unseal_authorized") is True
            and contract.get("automatic_promotion") is False,
            "Town07 holdout is not authorized by a frozen no-promotion receipt")
    pinned = contract["pinned"]
    require(digest(Path(__file__)) == pinned["evaluator_sha256"]
            and digest(PLAN) == pinned["plan_sha256"]
            and digest(MODEL_SOURCE) == pinned["model_source_sha256"] == MODEL_SOURCE_SHA256,
            "frozen evaluator, plan or model source changed")
    require(digest(PARENT) == pinned["parent_checkpoint_sha256"] == PARENT_SHA256
            and digest(PARENT_RESULT / "summary.json") == PARENT_SUMMARY_SHA256
            and digest(PARENT_RESULT / "started.json") == PARENT_STARTED_SHA256
            and digest(PARENT_RESULT / "data_manifest.json") == PARENT_MANIFEST_SHA256,
            "fixed AI v2 parent source changed")
    candidate = scoped_relative_path(pinned["candidate_run_relpath"])
    dataset = scoped_relative_path(pinned["candidate_dataset_relpath"])
    require(candidate.is_dir() and not candidate.is_symlink()
            and dataset.is_dir() and not dataset.is_symlink(),
            "candidate run and TRAIN dataset must be owned workspace directories")
    candidate_paths = {
        "checkpoint": candidate / "checkpoint.pt", "summary": candidate / "summary.json",
        "started": candidate / "started.json", "data_manifest": candidate / "data_manifest.json",
        "model_source": candidate / "source/map_perception.py",
        "training_source": candidate / "source/train_map_native_ah.py",
        "dataset_manifest": dataset / "manifest.json",
    }
    require_hash_ledger(candidate_paths, pinned["candidate_sha256"], "candidate")
    require(pinned["candidate_sha256"]["model_source"] == MODEL_SOURCE_SHA256
            and pinned["candidate_sha256"]["data_manifest"] == pinned["candidate_sha256"]["dataset_manifest"],
            "candidate architecture or dataset manifest differs")
    require(set(pinned["owner_metadata_sha256"]) == set(ROUTES),
            "both sealed owner receipt ledgers required")
    # HH_260906 - This is the first place Town07 owner metadata may be read; no frame bytes yet.
    for name, spec in ROUTES.items():
        require_hash_ledger(owner_metadata_paths(spec["owner"]),
                            pinned["owner_metadata_sha256"][name], f"Town07 {name}")
    parent_summary, parent_started = (read_json(PARENT_RESULT / filename)
                                      for filename in ("summary.json", "started.json"))
    parent_manifest = read_json(PARENT_RESULT / "data_manifest.json")
    candidate_summary = read_json(candidate_paths["summary"])
    candidate_started = read_json(candidate_paths["started"])
    manifest = read_json(candidate_paths["data_manifest"])
    require(parent_summary["checkpoint_sha256"] == PARENT_SHA256
            and parent_summary["deployment_approved"] is False
            and candidate_summary["checkpoint_sha256"] == pinned["candidate_sha256"]["checkpoint"]
            and candidate_summary["deployment_approved"] is False
            and candidate_started["parent_checkpoint_sha256"] == PARENT_SHA256
            and candidate_started["data_manifest_sha256"] == pinned["candidate_sha256"]["dataset_manifest"]
            and candidate_started["source_sha256"] == pinned["candidate_sha256"]["training_source"]
            and candidate_started["new_physical_routes"] is True
            and candidate_started["initialization"] == "scratch"
            and candidate_started["steps"] == 2000
            and candidate_started["seed"] == parent_started["seed"]
            and candidate_started["batch_size"] == parent_started["batch_size"]
            and candidate_started["input_shape"] == [3, 360, 640]
            and manifest["labels"] == parent_manifest["labels"] == list(LABELS)
            and manifest["image_size_wh"] == [640, 360]
            and manifest["splits"]["train"]["count"] == 913
            and manifest["splits"]["dev"] == parent_manifest["splits"]["dev"]
            and manifest["splits"]["train"]["source_records"][:497]
                == parent_manifest["splits"]["train"]["source_records"],
            "candidate fixed training/development contract differs")
    require(digest(dataset / manifest["splits"]["train"]["file"])
            == manifest["splits"]["train"]["sha256"]
            and digest(dataset / manifest["splits"]["dev"]["file"])
            == manifest["splits"]["dev"]["sha256"],
            "candidate TRAIN or frozen DEV array bytes differ")
    return {
        "contract": contract, "contract_sha256": digest(path), "candidate": candidate,
        "candidate_summary": candidate_summary, "parent_summary": parent_summary,
        "candidate_checkpoint_sha256": pinned["candidate_sha256"]["checkpoint"],
        "candidate_manifest_sha256": pinned["candidate_sha256"]["dataset_manifest"],
    }


def option_value(argv: list[str], name: str) -> str:
    require(argv.count(name) == 1 and argv.index(name) + 1 < len(argv),
            f"owner collector option differs: {name}")
    return argv[argv.index(name) + 1]


def verify_owner_metadata(name: str, spec: dict) -> tuple[list[dict], dict]:
    """HH_260906 - Recheck exact route/world/source evidence after freeze, before sensor reads."""
    owner, route_path, route_sha = spec["owner"], spec["route"], spec["sha256"]
    require(owner.is_dir() and not owner.is_symlink() and digest(route_path) == route_sha,
            f"Town07 {name}: owner or pinned route differs")
    route = read_json(route_path)
    require(route["town"] == "Town07" and route["coordinate_reference"] == "base_link"
            and len(route["route"]) >= 2, f"Town07 {name}: route schema differs")
    paths = owner_metadata_paths(owner)
    plan, started, result, capture, records = (read_json(paths[key]) for key in
                                               ("plan", "started", "result", "summary", "records"))
    require(Path(plan["route_path"]).resolve() == route_path.resolve()
            and plan["route_sha256"] == started["route_sha256"] == capture["route_sha256"] == route_sha
            and plan["map"] == started["map"] == capture["town"] == "Town07"
            and plan["capture_mode"] == result["capture_mode"] == "mapping-probe"
            and plan["quality"] == started["quality"] == "Epic"
            and plan.get("source_bytes_archived") is True
            and result["exit_code"] == 0
            and result.get("source_bytes_unchanged_and_archived") is True
            and plan.get("learned_model_control") is False
            and result.get("learned_model_control") is False
            and capture["status"] == "COMPLETE_ORACLE_PLUMBING_ONLY"
            and capture.get("research_split_intent") == "town07_sealed_evaluation_candidate"
            and capture["frame_count"] == len(records) == 240
            and capture["physics_driving"] is True and capture["motion_mode"] == "basic-agent"
            and capture.get("learned_model_control") is False
            and capture["ignores_traffic_lights"] is True
            and capture["collision_events"] == 0
            and capture["live_replay_exact_match"] is True
            and not capture["cleanup_errors"]
            and capture["oracle_semantics"] is True and capture["oracle_pose"] is True
            and capture.get("hd_map_approved") is False
            and {int(k): v for k, v in capture["label_table"].items()} == TAG_TABLE,
            f"Town07 {name}: physical owner proof differs")
    argv = plan["collector_argv"]
    require(Path(argv[0]).resolve() == (owner / "mapping").resolve()
            and Path(argv[1]).resolve() == route_path.resolve()
            and option_value(argv, "--motion-mode") == "basic-agent"
            and option_value(argv, "--frames") == "240"
            and argv.count("--ignore-traffic-lights") == 1
            and argv.count("--town07-generalization-test") == 1
            and "--town03-generalization-train" not in argv
            and "--town06-sign-scout" not in argv
            and option_value(argv, "--host") == plan["host"] == "127.0.0.1"
            and int(option_value(argv, "--port")) == plan["port"] == started["port"],
            f"Town07 {name}: scoped collector receipt differs")
    for stage in ("ready", "after_capture", "stopped"):
        event = read_json(paths[stage])
        require(event["status"] == "PASS" and event["stage"] == stage
                and event["expected_map"] == "Town07"
                and event["owner_pid"] == started["server_pid"]
                and (stage == "stopped" or event.get("active_map_basename") == "Town07")
                and (stage != "stopped" or event.get("port_released") is True),
                f"Town07 {name}: owner lifecycle differs: {stage}")
    source_hashes = plan["source_sha256"]
    require(bool(source_hashes) and set(source_hashes) == set(result["source_checks"])
            and all(result["source_checks"].values())
            and all(result.get("external_source_checks", {}).values()),
            f"Town07 {name}: archived source inventory differs")
    for relative, expected in source_hashes.items():
        part = Path(relative)
        archive = owner / "provenance" / part
        require(not part.is_absolute() and ".." not in part.parts
                and archive.resolve().is_relative_to((owner / "provenance").resolve())
                and digest(archive) == expected,
                f"Town07 {name}: archived source SHA differs: {relative}")
    prior_frame = prior_time = None
    for index, record in enumerate(records):
        require(record["stem"] == f"frames/{index:04d}"
                and isinstance(record["frame"], int)
                and (prior_frame is None or record["frame"] > prior_frame)
                and math.isfinite(record["timestamp_s"])
                and (prior_time is None or record["timestamp_s"] > prior_time)
                and all(math.isfinite(record[key]) for key in ("ego_x", "ego_y")),
                f"Town07 {name}: record order or pose differs")
        prior_frame, prior_time = record["frame"], record["timestamp_s"]
    return records, {"route_sha256": route_sha, "owner_metadata_sha256":
                     {key: digest(value) for key, value in paths.items()},
                     "archived_source_count": len(source_hashes),
                     "collision_events": 0, "replay_exact": True, "cleanup_errors": []}


def retain_indices(rows: list[dict], prior_xy: list[tuple[float, float]] | None = None,
                   minimum_m: float = 0.25) -> tuple[list[int], list[tuple[float, float]]]:
    """HH_260906 - One Town, route-order dedup against all previously kept XY positions."""
    require(minimum_m > 0 and math.isfinite(minimum_m), "positive position spacing required")
    chosen_xy = list(prior_xy or [])
    kept = []
    for index, row in enumerate(rows):
        xy = (float(row["ego_x"]), float(row["ego_y"]))
        require(all(math.isfinite(value) for value in xy), "finite Town07 XY required")
        if all(math.hypot(xy[0] - x, xy[1] - y) >= minimum_m for x, y in chosen_xy):
            kept.append(index)
            chosen_xy.append(xy)
    return kept, chosen_xy


def semantic_target(semantic: np.ndarray, frame_wh: tuple[int, int] = (640, 360)) -> np.ndarray:
    """HH_260906 - CARLA oracle tags become seven-class targets, never model inputs."""
    width, height = frame_wh
    require(semantic.shape == (height, width, 4) and semantic.dtype == np.uint8,
            "native uint8 CARLA semantic BGRA required")
    lut = np.zeros(256, dtype=np.uint8)
    for tag, label in TAG_TABLE.items():
        lut[tag] = LABELS.index(label)
    return lut[semantic[..., 2]]


def confusion_matrix(target: np.ndarray, prediction: np.ndarray) -> np.ndarray:
    require(target.shape == prediction.shape and target.dtype == prediction.dtype == np.uint8
            and target.ndim == 2 and target.max() < len(LABELS)
            and prediction.max() < len(LABELS), "seven-class native pixel prediction required")
    return np.bincount((target.astype(np.int64) * len(LABELS) + prediction).ravel(),
                       minlength=len(LABELS) ** 2).reshape(len(LABELS), len(LABELS))


def class_metrics(confusion: np.ndarray) -> dict:
    require(confusion.shape == (len(LABELS), len(LABELS)) and np.all(confusion >= 0)
            and int(confusion.sum()) > 0, "nonempty seven-class confusion required")
    classes = {}
    for index, label in enumerate(LABELS):
        target = int(confusion[index].sum())
        predicted = int(confusion[:, index].sum())
        tp = int(confusion[index, index])
        union = target + predicted - tp
        classes[label] = {
            "target_pixels": target, "predicted_pixels": predicted,
            "true_positive_pixels": tp,
            "precision": tp / predicted if predicted else None,
            "recall": tp / target if target else None,
            "iou": tp / union if union else None,
        }
    present = [row["iou"] for row in classes.values() if row["iou"] is not None]
    return {
        "classes": classes, "confusion": confusion.tolist(),
        "mean_iou_present": float(np.mean(present)),
        "mean_iou_7_zero_if_absent": float(sum((row["iou"] or 0.0) for row in classes.values()) / len(LABELS)),
        "pixel_accuracy": float(np.trace(confusion) / confusion.sum()),
        "gt_lane_pred_sign_pixels": int(confusion[LABELS.index("lane_marking"), LABELS.index("traffic_sign")]),
        "gt_sign_pred_other_pixels": int(confusion[LABELS.index("traffic_sign"), LABELS.index("other")]),
    }


def score_gate(parent: dict, candidate: dict, parent_dev: dict, candidate_dev: dict) -> dict:
    """HH_260906 - These thresholds precede any Town07 image or prediction access."""
    def iou(result, label):
        return result["classes"][label]["iou"] or 0.0

    routes = ("straight", "right")
    route_nonregression = {
        route: (iou(candidate[route], "road") >= iou(parent[route], "road")
                and iou(candidate[route], "lane_marking") >= iou(parent[route], "lane_marking"))
        for route in routes
    }
    route_floors = {
        route: (iou(candidate[route], "road") >= 0.80
                and iou(candidate[route], "lane_marking") >= 0.50)
        for route in routes
    }
    combined = {
        "road_improvement_5pp": iou(candidate["combined"], "road") - iou(parent["combined"], "road") >= 0.05,
        "lane_improvement_5pp": iou(candidate["combined"], "lane_marking")
            - iou(parent["combined"], "lane_marking") >= 0.05,
        "seven_class_mean_improvement_3pp": candidate["combined"]["mean_iou_7_zero_if_absent"]
            - parent["combined"]["mean_iou_7_zero_if_absent"] >= 0.03,
    }
    dev_guard = {
        "road_within_1pp": iou(candidate_dev, "road") >= iou(parent_dev, "road") - 0.01,
        "lane_within_2pp": iou(candidate_dev, "lane_marking") >= iou(parent_dev, "lane_marking") - 0.02,
        "mean_within_2pp": candidate_dev["mean_iou_7_zero_if_absent"]
            >= parent_dev["mean_iou_7_zero_if_absent"] - 0.02,
        "sign_iou_nonregression": iou(candidate_dev, "traffic_sign") >= iou(parent_dev, "traffic_sign"),
        "sign_precision_nonregression": (candidate_dev["classes"]["traffic_sign"]["precision"] or 0)
            >= (parent_dev["classes"]["traffic_sign"]["precision"] or 0),
    }
    improvement_signal = all(route_nonregression.values()) and all(combined.values())
    basic_quality = all(route_floors.values())
    research_gate_passed = improvement_signal and basic_quality and all(dev_guard.values())
    return {
        "route_road_lane_nonregression": route_nonregression,
        "route_absolute_road_80_lane_50": route_floors,
        "combined_relative_improvement": combined,
        "repeated_town01_dev_guard": dev_guard,
        "cross_town_improvement_signal": improvement_signal,
        "basic_recognition_quality_passed": basic_quality,
        "research_gate_passed": research_gate_passed,
        "automatic_promotion": False,
        "deployment_approved": False,
        "hd_map_approved": False,
    }


def load_model(path: Path, expected_sha256: str):
    """HH_260906 - Torch 1.8 has no weights_only; only SHA-pinned owned CPU checkpoints may load."""
    import torch
    from portable_e2e.map_perception import MapPerception

    require(digest(path) == expected_sha256, "checkpoint changed before CPU model load")
    kwargs = {"map_location": "cpu"}
    if "weights_only" in inspect.signature(torch.load).parameters:
        kwargs["weights_only"] = True
    checkpoint = torch.load(path, **kwargs)
    require(tuple(checkpoint["labels"]) == LABELS and checkpoint["input_shape"] == [3, 360, 640]
            and checkpoint["steps"] == 2000 and checkpoint["deployable"] is False,
            "frozen checkpoint schema differs")
    model = MapPerception().cpu().eval()
    model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    require(sum(parameter.numel() for parameter in model.parameters()) == 130503,
            "map architecture parameter count differs")
    return model


def font(size: int):
    font_path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(font_path), size) if font_path.is_file() else ImageFont.load_default()


def render_panel(path: Path, route: str, stem: str, rgb: np.ndarray, target: np.ndarray,
                 parent: np.ndarray, candidate: np.ndarray) -> None:
    """HH_260906 - Deterministic native full-FOV panels, not cropped score-selected examples."""
    require(rgb.shape == (360, 640, 3)
            and all(array.shape == (360, 640) for array in (target, parent, candidate)),
            "full native frame panels required")
    canvas = Image.new("RGB", (1280, 812), (16, 23, 33))
    draw = ImageDraw.Draw(canvas)
    draw.text((10, 5), f"Town07 {route} {stem} | sealed single-use map evaluation | NOT HD-map/driving approval",
              fill="white", font=font(17))
    panels = (("RGB input", rgb), ("CARLA oracle target", COLORS[target]),
              ("AI v2 frozen prediction", COLORS[parent]), ("Town03-data candidate prediction", COLORS[candidate]))
    for index, (title, image) in enumerate(panels):
        left, top = 640 * (index % 2), 42 + 385 * (index // 2)
        draw.text((left + 8, top), title, fill="white", font=font(17))
        canvas.paste(Image.fromarray(image), (left, top + 25))
    canvas.save(path)


def evaluate_frames(records_by_route: dict, retained_by_route: dict, models: dict) -> tuple[dict, dict, dict]:
    """HH_260906 - Both models see identical RGB float32/255 tensors on one CPU backend."""
    import torch

    raw = {name: {model: np.zeros((7, 7), dtype=np.int64) for model in models}
           for name in ROUTES}
    kept = {name: {model: np.zeros((7, 7), dtype=np.int64) for model in models}
            for name in ROUTES}
    selected_frames = {}
    predictions = {}
    frame_hashes = {}
    total = 0
    with torch.no_grad():
        for name in ("straight", "right"):
            source = ROUTES[name]["owner"] / "mapping"
            records = records_by_route[name]
            retained = retained_by_route[name]
            retained_set = set(retained)
            representative = {retained[0], retained[len(retained) // 2], retained[-1]}
            route_predictions = {key: np.empty((len(records), 360, 640), dtype=np.uint8) for key in models}
            hashes = {}
            for index, row in enumerate(records):
                stem = row["stem"]
                rgb_path, semantic_path = source / (stem + ".png"), source / (stem + ".npz")
                hashes[stem] = {"rgb_sha256": digest(rgb_path), "semantic_npz_sha256": digest(semantic_path)}
                with Image.open(rgb_path) as image:
                    require(image.mode == "RGB" and image.size == (640, 360),
                            "native RGB source differs")
                    rgb = np.asarray(image).copy()
                with np.load(semantic_path, allow_pickle=False) as source_arrays:
                    target = semantic_target(source_arrays["semantic"])
                image_tensor = torch.from_numpy(rgb.transpose(2, 0, 1).copy()).unsqueeze(0).float().div_(255)
                if index in representative:
                    selected_frames[(name, stem)] = {"rgb": rgb, "target": target}
                for model_name, model in models.items():
                    pred = model(image_tensor).argmax(1).squeeze(0).numpy().astype(np.uint8)
                    route_predictions[model_name][index] = pred
                    matrix = confusion_matrix(target, pred)
                    raw[name][model_name] += matrix
                    if index in retained_set:
                        kept[name][model_name] += matrix
                    if index in representative:
                        selected_frames[(name, stem)][model_name] = pred.copy()
                total += 1
                if total % 40 == 0:
                    print(f"CPU_TOWN07_FRAMES {total}/480", flush=True)
            predictions[name] = route_predictions
            frame_hashes[name] = hashes
    metrics = {}
    for model_name in models:
        metrics[model_name] = {}
        combined = np.zeros((7, 7), dtype=np.int64)
        for name in ("straight", "right"):
            metrics[model_name][name] = {
                "raw_240": class_metrics(raw[name][model_name]),
                "retained_global": class_metrics(kept[name][model_name]),
            }
            combined += kept[name][model_name]
        metrics[model_name]["combined_retained"] = class_metrics(combined)
    return metrics, selected_frames, {"predictions": predictions, "frame_hashes": frame_hashes}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frozen-contract", type=Path, required=True)
    parser.add_argument("--unseal", action="store_true")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    require(args.unseal, "--unseal required; no Town07 access without frozen authorization")
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == "", "explicit empty CUDA_VISIBLE_DEVICES required")
    require(args.frozen_contract.absolute() == FROZEN_CONTRACT.absolute(),
            "exact dated Town07 frozen receipt path required")
    output = args.output.absolute()
    require(output == DEFAULT_OUTPUT.absolute() and not output.exists(),
            "single exact fresh Town07 validation output required")
    frozen = verify_freeze_receipt(args.frozen_contract, unseal=args.unseal)
    records_by_route = {}
    proof_by_route = {}
    retained_by_route = {}
    global_xy: list[tuple[float, float]] = []
    for name in ("straight", "right"):
        records, proof = verify_owner_metadata(name, ROUTES[name])
        retained, global_xy = retain_indices(records, global_xy)
        require(len(retained) >= 120, f"Town07 {name}: insufficient globally distinct frames")
        records_by_route[name], proof_by_route[name], retained_by_route[name] = records, proof, retained
    import torch
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    require(not torch.cuda.is_available(), "CPU-only equal backend required")
    models = {"ai_v2": load_model(PARENT, PARENT_SHA256),
              "town03_data_candidate": load_model(frozen["candidate"] / "checkpoint.pt",
                                                   frozen["candidate_checkpoint_sha256"])}
    started = time.monotonic()
    metrics, selected, artifacts = evaluate_frames(records_by_route, retained_by_route, models)
    parent_dev = class_metrics(np.asarray(frozen["parent_summary"]["confusion"], dtype=np.int64))
    candidate_dev = class_metrics(np.asarray(frozen["candidate_summary"]["confusion"], dtype=np.int64))
    parent = {name: metrics["ai_v2"][name]["retained_global"] for name in ("straight", "right")}
    parent["combined"] = metrics["ai_v2"]["combined_retained"]
    candidate = {name: metrics["town03_data_candidate"][name]["retained_global"]
                 for name in ("straight", "right")}
    candidate["combined"] = metrics["town03_data_candidate"]["combined_retained"]
    gate = score_gate(parent, candidate, parent_dev, candidate_dev)
    output.mkdir(parents=False, exist_ok=False)
    prediction_path = output / "predictions.npz"
    with prediction_path.open("xb") as stream:
        np.savez_compressed(stream,
                            straight_ai_v2=artifacts["predictions"]["straight"]["ai_v2"],
                            straight_candidate=artifacts["predictions"]["straight"]["town03_data_candidate"],
                            right_ai_v2=artifacts["predictions"]["right"]["ai_v2"],
                            right_candidate=artifacts["predictions"]["right"]["town03_data_candidate"],
                            straight_retained_indices=np.asarray(retained_by_route["straight"], dtype=np.int64),
                            right_retained_indices=np.asarray(retained_by_route["right"], dtype=np.int64))
    for (name, stem), frame in selected.items():
        render_panel(output / f"{name}_{Path(stem).name}_full_fov.png", name, stem,
                     frame["rgb"], frame["target"], frame["ai_v2"],
                     frame["town03_data_candidate"])
    with (output / "input_frames.json").open("x", encoding="utf-8") as stream:
        json.dump(artifacts["frame_hashes"], stream, indent=2)
    summary = {
        "schema": "portable_e2e.map_town07_holdout_eval.v1",
        "status": "SINGLE_USE_RESEARCH_EVALUATION_ONLY",
        "town": "Town07", "route_order": ["straight", "right"],
        "labels": list(LABELS), "input": "original 640x360 RGB float32/255; oracle CARLA semantics only as target",
        "raw_frames_per_route": 240,
        "global_0p25m_retained": {name: len(retained_by_route[name]) for name in ("straight", "right")},
        "retention_rule": "straight then right, >=0.25m from every previously kept Town07 ego XY",
        "models": metrics, "gate": gate,
        "source_sha256": {"frozen_contract": frozen["contract_sha256"],
                          "evaluator": digest(Path(__file__)), "plan": digest(PLAN),
                          "model_source": MODEL_SOURCE_SHA256,
                          "parent_checkpoint": PARENT_SHA256,
                          "candidate_checkpoint": frozen["candidate_checkpoint_sha256"],
                          "candidate_manifest": frozen["candidate_manifest_sha256"],
                          "owner": proof_by_route,
                          "input_frames": digest(output / "input_frames.json"),
                          "predictions": digest(prediction_path)},
        "wall_seconds": time.monotonic() - started, "cpu_threads": 2, "gpu_used": False,
        "training_executed_in_evaluator": False, "automatic_promotion": False,
        "deployment_approved": False, "hd_map_approved": False,
        "limitations": "Two predeclared Town07 routes but one Town/weather; correlated frames and CARLA oracle targets; no complete HD map, closed-loop driving or real vehicle validation",
    }
    with (output / "summary.json").open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=False)
    links = " · ".join(f"[{name} {Path(stem).name} 전체화각](./{name}_{Path(stem).name}_full_fov.png)"
                       for name, stem in selected)
    def percent(value):
        return f"{100 * (value or 0.0):.2f}%"
    rows = []
    for name in ("straight", "right", "combined_retained"):
        a = metrics["ai_v2"][name]["retained_global"] if name != "combined_retained" else metrics["ai_v2"][name]
        b = (metrics["town03_data_candidate"][name]["retained_global"]
             if name != "combined_retained" else metrics["town03_data_candidate"][name])
        rows.append(f"| {name} | {percent(a['classes']['road']['iou'])} → {percent(b['classes']['road']['iou'])} | "
                    f"{percent(a['classes']['lane_marking']['iou'])} → {percent(b['classes']['lane_marking']['iou'])} | "
                    f"{percent(a['mean_iou_7_zero_if_absent'])} → {percent(b['mean_iou_7_zero_if_absent'])} |")
    report = f"""# Town07 지도 모델 단회 홀드아웃 A/B 평가

<!-- HH_260906 - The sealed holdout evaluates RGB semantics only; it cannot approve HD mapping or driving. -->

**결론: 연구 판정 조건 {'통과' if gate['research_gate_passed'] else '미통과'}; 자동 채택·배포 없음.** 사전 봉인된 체크포인트·코드·판정 기준과 두 Town07 원본을 같은 CPU에서 원본 640×360 RGB로 비교했습니다. 실제 원본은 CARLA BasicAgent가 수집했고 학습된 모델이 차를 운전한 결과가 아닙니다.

| 0.25m 전역 중복 제거 기준 | 도로 IoU A→B | 차선 IoU A→B | 7-class mIoU A→B |
|---|---:|---:|---:|
{chr(10).join(rows)}

보존 프레임: 직진 **{len(retained_by_route['straight'])}장**, 우회전 **{len(retained_by_route['right'])}장**. 정확한 클래스별 IoU·정밀도·재현율, 혼동행렬, 실제 차선을 표지판으로 잘못 예측한 픽셀과 실제 표지판을 `other`로 놓친 픽셀은 [기계 판독 결과](summary.json)에 있습니다. [원본별 입력 SHA](input_frames.json) · [두 모델 원본 예측](predictions.npz) · {links}

사전에 정한 관문은 두 경로 각각 도로·차선 비퇴행 및 도로≥80%·차선≥50%, 합계 도로+5%p·차선+5%p·mIoU+3%p, 기존 Town01 DEV 보호입니다. [동결 계약 및 우선순위](../map_next_factor/PLAN.md)를 평가 전에 해시로 확인했습니다. 조건을 통과해도 한 Town·한 날씨와 상관된 프레임만으로 HD 지도 품질·10Hz 시스템·폐루프 주행·실차 안전을 승인하지 않습니다. Town07를 다시 조정용으로 쓰면 더 이상 단회 평가가 아닙니다.
"""
    with (output / "README.md").open("x", encoding="utf-8") as stream:
        stream.write(report)
    print("TOWN07_SINGLE_USE_RESEARCH_EVALUATION_COMPLETE_NO_PROMOTION", flush=True)


if __name__ == "__main__":
    main()
