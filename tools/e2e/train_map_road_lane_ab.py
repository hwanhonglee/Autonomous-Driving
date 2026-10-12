#!/usr/bin/env python3
"""HH_260929 - Run a non-promotable Town02 data-composition A/B on personal GPU0."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time


EXPECTED_DATA_MANIFEST_SHA256 = "313b41ed4fb897f01fb4fb433c1bafb234ef9b234e62f96a462ef2188e9dccc6"
EXPECTED_PARENT_CHECKPOINT_SHA256 = "ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380"
EXPECTED_PARENT_MANIFEST_SHA256 = "e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c"
SUPPORT_SHA256 = {
    "map_perception.py": "1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b",
    "map_learning_balance.py": "57c9392d8b700f5cf628865042b66ae630961f35fb8858a5d0ead77fe5a7c73b",
    "dual_recovery_objectives.py": "d10bcad9060c1f4bde40b20773fd4c18f1d4b6ba3861dd7cef55cd36cff4ad15",
}
SPLITS = {"train_d0": 497, "train_d1": 729, "dev_town01": 118, "dev_town05": 232}
IOU_REPRODUCTION_TOLERANCE = 0.005
STEPS = 2000
BATCH_SIZE = 8
SEED = 20260922
LEARNING_RATE = 3e-4
WEIGHT_DECAY = 1e-4


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def validate_package(data_root, manifest, digest, np, labels):
    """HH_260929 - Reject a changed split before any personal GPU lease is acquired."""
    require(manifest.get("schema") == "portable_e2e.map_road_lane_ab_data.v1",
            "data package schema changed")
    files = manifest.get("files")
    require(isinstance(files, dict) and set(files) == set(SPLITS),
            "four fixed A/B arrays required")
    arrays = {}
    for name, count in SPLITS.items():
        ref = files[name]
        require(isinstance(ref, dict) and ref.get("count") == count
                and ref.get("file") == name + ".npz", f"{name} metadata changed")
        path = data_root / ref["file"]
        require(path.is_file() and not path.is_symlink()
                and path.resolve().is_relative_to(data_root.resolve())
                and ref.get("sha256") == digest(path), f"{name} bytes changed")
        with np.load(path, allow_pickle=False) as item:
            require(set(item.files) == {"rgb", "target"}, f"{name} array keys changed")
            rgb, target = item["rgb"].copy(), item["target"].copy()
        require(rgb.shape == (count, 360, 640, 3)
                and target.shape == (count, 360, 640)
                and rgb.dtype == np.uint8 and target.dtype == np.uint8
                and int(target.max()) < len(labels), f"{name} array contract changed")
        arrays[name] = (rgb, target)
    for index in (0, 1):
        require(np.array_equal(arrays["train_d1"][index][:SPLITS["train_d0"]],
                               arrays["train_d0"][index]),
                "D1 no longer has exact D0 TRAIN prefix")
    require(manifest.get("parent_manifest_sha256") == EXPECTED_PARENT_MANIFEST_SHA256,
            "AI v2 TRAIN/DEV provenance changed")
    require(manifest.get("town05_split") == "DEV_ONLY",
            "Town05 independent development split changed")
    require(manifest.get("town02_split") == "TRAIN_EXPLORATORY_ROAD_LANE_ONLY",
            "Town02 TRAIN exploratory scope changed")
    return arrays


def metrics_from_confusion(confusion, labels, np):
    """HH_260929 - Publish every semantic class even though this A/B targets road/lane."""
    intersection = np.diag(confusion)
    union = confusion.sum(0) + confusion.sum(1) - intersection
    predicted = confusion.sum(0)
    iou = {name: float(intersection[i] / union[i]) if union[i] else None
           for i, name in enumerate(labels)}
    precision = {name: float(intersection[i] / predicted[i]) if predicted[i] else None
                 for i, name in enumerate(labels)}
    return {
        "iou": iou,
        "precision": precision,
        "pixel_accuracy": float(intersection.sum() / confusion.sum()),
        "confusion": confusion.tolist(),
    }


def reproduction_gate(reference, replay):
    """HH_260929 - Stop before D1 if D0 fails the predeclared seven-class replay."""
    missing = [name for name, value in reference["iou"].items()
               if value is None or replay["iou"].get(name) is None]
    require(not missing, "D0 class absent from the frozen Town01 reference")
    differences = {name: abs(replay["iou"][name] - value)
                   for name, value in reference["iou"].items()}
    return differences, all(value <= IOU_REPRODUCTION_TOLERANCE
                            for value in differences.values())


def development_gates(d0, d1, reference):
    """HH_260929 - Keep the road/lane hypothesis separate from all-class approval."""
    one = d1["town01"]["iou"]
    five0, five1 = d0["town05"]["iou"], d1["town05"]["iou"]
    road_lane = {
        "town05_road_plus_5pp": five1["road"] >= five0["road"] + 0.05,
        "town05_lane_plus_5pp": five1["lane_marking"] >= five0["lane_marking"] + 0.05,
        "town05_road_at_least_80pct": five1["road"] >= 0.80,
        "town05_lane_at_least_50pct": five1["lane_marking"] >= 0.50,
        "town01_road_at_least_97p751pct": one["road"] >= 0.97751,
        "town01_lane_at_least_89p233pct": one["lane_marking"] >= 0.89233,
    }
    class_delta = {name: (None if one[name] is None or reference["iou"][name] is None
                           else one[name] - reference["iou"][name])
                   for name in reference["iou"]}
    return {
        "road_lane_gates": road_lane,
        "road_lane_development_target_met": all(road_lane.values()),
        "town01_class_iou_delta_vs_ai_v2": class_delta,
        "all_town01_classes_non_regressing": all(value is not None and value >= 0
                                                  for value in class_delta.values()),
        "automatic_model_promotion": False,
        "independent_test_used": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("frozen-repo", "data", "parent", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    sys.path.insert(0, str(args.frozen_repo))
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import numpy as np
    import torch
    from map_perception import MapPerception, LABELS, semantic_loss
    from map_learning_balance import class_weights
    from dual_recovery_objectives import paired_crop
    from scripts.e2e import run_moving_multiroute_recovery_k as guard

    workspace = guard.check_run_environment({"repo_root": str(args.frozen_repo),
                                             "output_root": str(args.output)})
    require(not args.output.exists() and not args.output.is_symlink(),
            "fresh A/B output required")
    require(args.data.resolve().is_relative_to((workspace.parent / "dataset").resolve())
            and not args.data.is_symlink(), "personal dataset required")
    require(args.parent.resolve().is_relative_to((workspace / "runs/diagnostics").resolve())
            and not args.parent.is_symlink(), "personal AI v2 checkpoint required")
    source = Path(__file__).resolve().parent
    for name, expected in SUPPORT_SHA256.items():
        require(guard.digest(source / name) == expected, f"frozen support changed: {name}")
    manifest_path = args.data / "manifest.json"
    require(guard.digest(manifest_path) == EXPECTED_DATA_MANIFEST_SHA256,
            "A/B data manifest differs from the preregistered package")
    manifest = guard.read_json(manifest_path)
    arrays = validate_package(args.data, manifest, guard.digest, np, LABELS)
    parent_summary = guard.read_json(args.parent / "summary.json")
    parent_started = guard.read_json(args.parent / "started.json")
    parent_checkpoint = args.parent / "checkpoint.pt"
    require(parent_summary.get("checkpoint_sha256") == EXPECTED_PARENT_CHECKPOINT_SHA256
            and guard.digest(parent_checkpoint) == EXPECTED_PARENT_CHECKPOINT_SHA256
            and parent_started.get("data_manifest_sha256") == EXPECTED_PARENT_MANIFEST_SHA256
            and parent_started.get("train_count") == 497
            and parent_started.get("dev_count") == 118,
            "published AI v2 parent changed")
    require(set(parent_summary["iou"]) == set(LABELS), "AI v2 class reference changed")
    pixels = np.bincount(arrays["train_d0"][1].ravel(), minlength=len(LABELS))
    weights_np = class_weights(pixels, "sqrt_frequency")
    if args.validate_only:
        print(json.dumps({"status": "CPU_PREFLIGHT_PASS", "counts": SPLITS,
                          "parent_sha256": EXPECTED_PARENT_CHECKPOINT_SHA256,
                          "class_weights": weights_np.tolist()}), flush=True)
        return

    with guard.gpu0_lease(workspace):
        gpu = guard.preflight_gpu()
        device = torch.device("cuda:0")
        args.output.mkdir(parents=True, exist_ok=False)
        for name in (Path(__file__).name, *SUPPORT_SHA256):
            guard.copy_new(source / name, args.output / "source" / name)
        guard.copy_new(manifest_path, args.output / "data_manifest.json")
        started = {
            "schema": "portable_e2e.map_road_lane_ab_run.v1", "started_at_utc": guard.utc_now(),
            "gpu": gpu, "data_manifest_sha256": EXPECTED_DATA_MANIFEST_SHA256,
            "parent_checkpoint_sha256": EXPECTED_PARENT_CHECKPOINT_SHA256,
            "training_source_sha256": guard.digest(Path(__file__)),
            "support_sha256": SUPPORT_SHA256, "seed": SEED, "steps_per_arm": STEPS,
            "batch_size": BATCH_SIZE, "learning_rate": LEARNING_RATE,
            "weight_decay": WEIGHT_DECAY, "initialization": "scratch_both_arms",
            "augmentation": "TRAIN only: probability 0.5 aligned crop, scale 0.65..1.0, resize to native; no flip",
            "class_weights_source": "Town01 D0 TRAIN only; fixed for D1",
            "class_pixels": pixels.tolist(), "class_weights": weights_np.tolist(),
            "single_factor": "D1 adds 232 Town02_Opt p02 TRAIN frames; all else unchanged",
            "automatic_promotion": False, "sign_dev_qualified": False,
        }
        guard.write_new(args.output / "started.json", started)
        weights = torch.tensor(weights_np, dtype=torch.float32, device=device)

        def evaluate(network, name):
            rgb, target = arrays[name]
            confusion = np.zeros((len(LABELS), len(LABELS)), dtype=np.int64)
            predictions = []
            network.eval()
            with torch.no_grad():
                for begin in range(0, len(rgb), BATCH_SIZE):
                    batch = torch.tensor(rgb[begin:begin + BATCH_SIZE].transpose(0, 3, 1, 2).copy(),
                                         device=device, dtype=torch.float32) / 255
                    pred = network(batch).argmax(1).cpu().numpy().astype(np.uint8)
                    predictions.append(pred)
                    encoded = target[begin:begin + BATCH_SIZE].astype(np.int64) * len(LABELS) + pred
                    confusion += np.bincount(encoded.ravel(), minlength=len(LABELS) ** 2).reshape(
                        len(LABELS), len(LABELS))
            return metrics_from_confusion(confusion, LABELS, np), np.concatenate(predictions)

        results = {}
        for arm, train_name in (("D0", "train_d0"), ("D1", "train_d1")):
            guard._seed_everything(SEED, device)
            model = MapPerception().to(device)
            optimizer = torch.optim.AdamW(model.parameters(), lr=LEARNING_RATE,
                                          weight_decay=WEIGHT_DECAY)
            sampler = np.random.default_rng(SEED)
            augmentation = np.random.default_rng(20260926)
            rgb, target = arrays[train_name]
            arm_dir = args.output / arm
            arm_dir.mkdir()
            start = time.monotonic()
            model.train()
            with (arm_dir / "history.jsonl").open("x") as log:
                for step in range(1, STEPS + 1):
                    ids = sampler.integers(0, len(rgb), size=BATCH_SIZE)
                    pairs = [paired_crop(rgb[i], target[i], augmentation) for i in ids]
                    images = torch.tensor(np.stack([pair[0] for pair in pairs]).transpose(0, 3, 1, 2).copy(),
                                          device=device, dtype=torch.float32) / 255
                    labels = torch.tensor(np.stack([pair[1] for pair in pairs]).astype(np.int64),
                                          device=device)
                    optimizer.zero_grad(set_to_none=True)
                    loss = semantic_loss(model(images), labels, weights)
                    require(bool(torch.isfinite(loss)), "nonfinite map loss")
                    loss.backward()
                    norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
                    require(bool(torch.isfinite(norm)), "nonfinite map gradient")
                    optimizer.step()
                    if step % 100 == 0:
                        row = {"step": step, "loss": float(loss.detach()),
                               "wall_seconds": time.monotonic() - start}
                        log.write(json.dumps(row) + "\n")
                        log.flush()
                        print(json.dumps({"arm": arm, **row}), flush=True)
            training_seconds = time.monotonic() - start
            guard.save_new(arm_dir / "checkpoint.pt", {
                "model_state_dict": model.state_dict(), "labels": LABELS,
                "model_id": "portable_e2e.map_semantics.town02_road_lane_" + arm.lower(),
                "optimizer_state_dict": optimizer.state_dict(), "steps": STEPS,
                "deployable": False, "research_contract": started,
            })
            town01, pred01 = evaluate(model, "dev_town01")
            town05, pred05 = evaluate(model, "dev_town05")
            np.savez_compressed(arm_dir / "town01_predictions.npz", prediction=pred01)
            np.savez_compressed(arm_dir / "town05_predictions.npz", prediction=pred05)
            metrics = {"train_count": len(rgb), "training_wall_seconds": training_seconds,
                       "checkpoint_sha256": guard.digest(arm_dir / "checkpoint.pt"),
                       "town01": town01, "town05": town05, "deployment_approved": False}
            guard.write_new(arm_dir / "metrics.json", metrics)
            results[arm] = metrics
            if arm == "D0":
                difference, replay_ok = reproduction_gate(parent_summary, town01)
                guard.write_new(args.output / "d0_reproduction.json", {
                    "iou_absolute_difference": difference, "tolerance": IOU_REPRODUCTION_TOLERANCE,
                    "pass": replay_ok,
                })
                if not replay_ok:
                    guard.write_new(args.output / "summary.json", {
                        "status": "D0_REPRODUCTION_FAILED_D1_NOT_RUN", "D0": metrics,
                        "d0_iou_absolute_difference": difference,
                        "model_promotion": False, "gpu_training_executed": True,
                    })
                    print("D0_REPRODUCTION_FAILED_D1_NOT_RUN", flush=True)
                    return
        gates = development_gates(results["D0"], results["D1"], parent_summary)
        guard.write_new(args.output / "summary.json", {
            "status": "A_B_COMPLETE_EXPLORATORY_NOT_PROMOTED", "D0": results["D0"],
            "D1": results["D1"], "gates": gates, "gpu_training_executed": True,
            "model_promotion": False, "sign_dev_qualified": False,
            "hd_map_complete": False, "learned_closed_loop_driving": False,
            "completed_at_utc": guard.utc_now(),
        })
        print("A_B_COMPLETE_EXPLORATORY_NOT_PROMOTED", flush=True)


if __name__ == "__main__":
    main()
