#!/usr/bin/env python3
"""HH_260929 - Train a non-promotable Town02 p00 versus p02 route-holdout map A/B."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time


# HH_260929 - Bind the full source-reconstructed package receipt before opening GPU0.
EXPECTED_DATA_MANIFEST_SHA256 = "f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834"
EXPECTED_PARENT_CHECKPOINT_SHA256 = "ae489cd99e48f6f68da3241e8296c705847398c23b8ba69b71ec024cef4ef380"
EXPECTED_PARENT_MANIFEST_SHA256 = "e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c"
SUPPORT_SHA256 = {
    "map_perception.py": "1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b",
    "map_learning_balance.py": "57c9392d8b700f5cf628865042b66ae630961f35fb8858a5d0ead77fe5a7c73b",
    "dual_recovery_objectives.py": "d10bcad9060c1f4bde40b20773fd4c18f1d4b6ba3861dd7cef55cd36cff4ad15",
}
# HH_260929 - Freeze the complete imported K helper snapshot before Python imports it.
FROZEN_HELPER_SHA256 = {
    "scripts/e2e/run_moving_multiroute_recovery_k.py": "9044dd3fc421e5738ef58b2fdae189402db76db60b2d305d82cc897bdfc84944",
    "portable_e2e/audit_runtime.py": "c4c68e888fb89bb4e78052e3167f4167d89c7e7da952e3a0da627d677a776647",
    "portable_e2e/contract.py": "e2e3a2669fa94aa98cb7e50f20ffa7db433b99eead9a12c654392e4b293a5c09",
    "portable_e2e/dataset.py": "3b30a7b4db1907a10ad94002bb880deaa4bb545a978bad3beec1b0aacf3b9c95",
    "portable_e2e/evaluate.py": "9d30d3be7e114ee80f31355b2c2780c840ef48328e358935ec33944dc8d14887",
    "portable_e2e/losses.py": "cdf37262024435c3a04888bf816c24db1b769b87034f08e1541c02d22da80973",
    "portable_e2e/model.py": "fbbedef10b97add830e0d887400107b2bcd6dfb5f0c5c4901e399d2e4717fd6e",
    "portable_e2e/runtime_contract.py": "38e993278ef84b149efc90931423cd90b1562d86c9eb1585260d50e03b2ae0d3",
    "portable_e2e/stop_primitive_research.py": "4021eb2ba6d84260b6b658114fbe31a1e7ef826121e4d07690bfd4154919a92c",
    "portable_e2e/torch_dataset.py": "a81d26b1c9aaf67db56cf993a41857133dccac78e42047754e106cff4c8987c9",
    "portable_e2e/train.py": "c5ee8ab4533c3feedb0c60fe270fcb4a5e42247e186cfc8fe1ac9ec2fb7e0247",
}
EXPECTED_RESEARCH_SCOPE_SHA256 = "c4870d042a074f559be7ed841ba0351be007d2d562d9a0b7567db2ac95b719b0"
COUNTS = {"train_a": 497, "train_b": 729, "holdout_town02_p02": 232, "dev_town01": 118}
SCHEMA = "portable_e2e.map_town02_p00_holdout_data.v1"
SEED = 20260922
STEPS = 2000
BATCH_SIZE = 8
LEARNING_RATE = 3e-4
WEIGHT_DECAY = 1e-4
REPRODUCTION_TOLERANCE = 0.005
# HH_260929 - Ignore only binary floating-point roundoff at exact percentage-point gates.
GATE_EPSILON = 1e-12


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _plain_digest(path: Path) -> str:
    """HH_260929 - Hash imported source before executing anything from that source tree."""
    require(path.is_file() and not path.is_symlink(), f"regular source required: {path}")
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(block)
    return sha.hexdigest()


def validate_personal_paths(workspace: Path, frozen_repo: Path, data: Path,
                            parent: Path, output: Path) -> None:
    """HH_260929 - Reject lexical traversal and symlink ancestors before any output write."""
    workspace = workspace.absolute()
    diagnostics = workspace / "runs/diagnostics"
    dataset = workspace.parent / "dataset"
    expected_repo = workspace / "source_worktrees/moving_recovery_k_20260922_v1"
    require(workspace.resolve() == workspace and diagnostics.resolve() == diagnostics
            and dataset.resolve() == dataset and expected_repo.resolve() == expected_repo,
            "personal workspace ancestor is a symlink")
    require(frozen_repo.absolute() == expected_repo
            and data.parent.absolute() == dataset
            and parent.parent.absolute() == diagnostics
            and output.parent.absolute() == diagnostics,
            "source, dataset, or output escaped its exact personal parent")
    require(not any(path.is_symlink() for path in (frozen_repo, data, parent, output))
            and frozen_repo.resolve() == expected_repo
            and data.resolve().is_relative_to(dataset)
            and parent.resolve().is_relative_to(diagnostics)
            and output.resolve().is_relative_to(diagnostics)
            and output.resolve() != diagnostics,
            "personal path resolves outside its intended directory")


def validate_package(data_root, manifest, digest, np, labels):
    """HH_260929 - Bind every split and verify that B changes only the added p00 suffix."""
    require(manifest.get("schema") == SCHEMA, "route-holdout package schema differs")
    files = manifest.get("files")
    require(isinstance(files, dict) and set(files) == set(COUNTS),
            "four exact route-holdout splits required")
    require(manifest.get("parent_manifest_sha256") == EXPECTED_PARENT_MANIFEST_SHA256,
            "Town01 parent provenance changed")
    require(manifest.get("independent_dev_approved") is False
            and manifest.get("dataset_admitted") is False
            and manifest.get("exploratory_ab_train_authorized") is True
            and manifest.get("provenance", {}).get("research_scope_sha256")
            == EXPECTED_RESEARCH_SCOPE_SHA256,
            "exploratory split claimed approval")
    arrays = {}
    for name, count in COUNTS.items():
        ref = files[name]
        require(isinstance(ref, dict) and ref.get("count") == count
                and ref.get("file") == f"{name}.npz", f"{name} metadata differs")
        path = data_root / ref["file"]
        require(path.is_file() and not path.is_symlink()
                and path.resolve().is_relative_to(data_root.resolve())
                and ref.get("sha256") == digest(path), f"{name} bytes differ")
        with np.load(path, allow_pickle=False) as item:
            require(set(item.files) == {"rgb", "target"}, f"{name} array keys differ")
            rgb, target = item["rgb"].copy(), item["target"].copy()
        require(rgb.shape == (count, 360, 640, 3)
                and target.shape == (count, 360, 640)
                and rgb.dtype == np.uint8 and target.dtype == np.uint8
                and int(target.max()) < len(labels), f"{name} array ABI differs")
        class_pixels = np.bincount(target.ravel(), minlength=len(labels)).tolist()
        require(ref.get("class_pixels") == class_pixels,
                f"{name} semantic histogram differs")
        arrays[name] = (rgb, target)
    for part in (0, 1):
        require(np.array_equal(arrays["train_b"][part][:COUNTS["train_a"]],
                               arrays["train_a"][part]),
                "B changed the frozen Town01 TRAIN prefix")
    return arrays


def metrics_from_confusion(confusion, labels, np):
    """HH_260929 - Keep undefined IoU explicit and publish the complete 7-class matrix."""
    intersection = np.diag(confusion)
    union = confusion.sum(0) + confusion.sum(1) - intersection
    predicted = confusion.sum(0)
    return {
        "iou": {name: float(intersection[i] / union[i]) if union[i] else None
                for i, name in enumerate(labels)},
        "precision": {name: float(intersection[i] / predicted[i]) if predicted[i] else None
                      for i, name in enumerate(labels)},
        "pixel_accuracy": float(intersection.sum() / confusion.sum()),
        "confusion": confusion.tolist(),
        "true_class_pixels": confusion.sum(1).tolist(),
    }


def reproduction_gate(reference, replay):
    """HH_260929 - Stop before B when A does not reproduce the published AI v2 DEV."""
    names = set(reference.get("iou", {}))
    require(names and names == set(replay.get("iou", {})), "seven-class reference changed")
    differences = {}
    for name in sorted(names):
        first, second = reference["iou"][name], replay["iou"][name]
        require(first is not None and second is not None,
                f"Town01 reference class {name} has undefined IoU")
        differences[name] = abs(first - second)
    return differences, all(value <= REPRODUCTION_TOLERANCE
                            for value in differences.values())


def probe_gates(a, b):
    """HH_260929 - Apply fixed p02 and Town01 gates without independent-DEV claims."""
    p02_a, p02_b = a["p02"]["iou"], b["p02"]["iou"]
    one_a, one_b = a["town01"]["iou"], b["town01"]["iou"]
    names = set(one_a)
    require(names == set(one_b) == set(p02_a) == set(p02_b),
            "A/B seven-class metric keys differ")
    delta_p02 = {name: (None if p02_a[name] is None or p02_b[name] is None
                        else p02_b[name] - p02_a[name]) for name in sorted(names)}
    delta_town01 = {name: (None if one_a[name] is None or one_b[name] is None
                           else one_b[name] - one_a[name]) for name in sorted(names)}
    lane = delta_p02["lane_marking"]
    road = delta_p02["road"]
    gate = {
        "p02_lane_gain_at_least_2pp": lane is not None and lane + GATE_EPSILON >= 0.02,
        "p02_road_drop_at_most_1pp": road is not None and road + GATE_EPSILON >= -0.01,
        "town01_road_drop_at_most_0p5pp": delta_town01["road"] is not None
        and delta_town01["road"] + GATE_EPSILON >= -0.005,
        "town01_lane_drop_at_most_0p5pp": delta_town01["lane_marking"] is not None
        and delta_town01["lane_marking"] + GATE_EPSILON >= -0.005,
        "town01_all_seven_drop_at_most_0p5pp": all(
            value is not None and value + GATE_EPSILON >= -0.005
            for value in delta_town01.values()),
    }
    return {"gates": gate, "all_gates_passed": all(gate.values()),
            "p02_iou_delta_b_minus_a": delta_p02,
            "town01_iou_delta_b_minus_a": delta_town01,
            "independent_dev_sites": 0, "automatic_model_promotion": False}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("frozen-repo", "data", "parent", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    workspace_root = Path.home() / "personal/hwanhong/portable_e2e"
    require(Path(sys.prefix).absolute() == workspace_root / "venvs/py312",
            "personal py312 venv required before imports")
    validate_personal_paths(workspace_root, args.frozen_repo, args.data,
                            args.parent, args.output)
    source = Path(__file__).resolve().parent
    for name, expected in SUPPORT_SHA256.items():
        require(_plain_digest(source / name) == expected,
                f"frozen map support changed before import: {name}")
    for name, expected in FROZEN_HELPER_SHA256.items():
        require(_plain_digest(args.frozen_repo / name) == expected,
                f"frozen K helper changed before import: {name}")
    sys.path.insert(0, str(args.frozen_repo))
    sys.path.insert(0, str(source))
    import numpy as np
    import torch
    import map_perception as map_module
    import map_learning_balance as balance_module
    import dual_recovery_objectives as crop_module
    from map_perception import MapPerception, LABELS, semantic_loss
    from map_learning_balance import class_weights
    from dual_recovery_objectives import paired_crop
    from scripts.e2e import run_moving_multiroute_recovery_k as guard

    # HH_260929 - A hash on a file is insufficient if imports resolve elsewhere.
    for module, path in (
        (map_module, source / "map_perception.py"),
        (balance_module, source / "map_learning_balance.py"),
        (crop_module, source / "dual_recovery_objectives.py"),
        (guard, args.frozen_repo / "scripts/e2e/run_moving_multiroute_recovery_k.py"),
    ):
        require(Path(module.__file__).resolve() == path.resolve(),
                f"imported module differs from frozen source: {module.__name__}")
    for relative in FROZEN_HELPER_SHA256:
        if not relative.startswith("portable_e2e/"):
            continue
        module_name = relative.removesuffix(".py").replace("/", ".")
        loaded = sys.modules.get(module_name)
        if loaded is not None:
            require(Path(loaded.__file__).resolve() == (args.frozen_repo / relative).resolve(),
                    f"imported K helper differs from frozen source: {module_name}")
    require(sys.modules.get("portable_e2e.train") is not None,
            "frozen seed helper did not load")

    workspace = guard.check_run_environment({"repo_root": str(args.frozen_repo),
                                             "output_root": str(args.output)})
    require(not args.output.exists() and not args.output.is_symlink(),
            "fresh output required")
    validate_personal_paths(workspace, args.frozen_repo, args.data,
                            args.parent, args.output)
    for name, expected in SUPPORT_SHA256.items():
        require(guard.digest(source / name) == expected,
                f"frozen support changed: {name}")
    require(len(EXPECTED_DATA_MANIFEST_SHA256) == 64,
            "dataset manifest hash not frozen before GPU run")
    manifest_path = args.data / "manifest.json"
    require(guard.digest(manifest_path) == EXPECTED_DATA_MANIFEST_SHA256,
            "preregistered dataset manifest bytes differ")
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
            "published AI v2 parent differs")
    pixels = np.bincount(arrays["train_a"][1].ravel(), minlength=len(LABELS))
    weights_np = class_weights(pixels, "sqrt_frequency")
    if args.validate_only:
        print(json.dumps({"status": "CPU_PREFLIGHT_PASS", "counts": COUNTS,
                          "manifest_sha256": EXPECTED_DATA_MANIFEST_SHA256,
                          "weights": weights_np.tolist()}), flush=True)
        return

    with guard.gpu0_lease(workspace):
        gpu = guard.preflight_gpu()
        device = torch.device("cuda:0")
        args.output.mkdir(parents=True, exist_ok=False)
        for name in (Path(__file__).name, *SUPPORT_SHA256):
            guard.copy_new(source / name, args.output / "source" / name)
        for name in FROZEN_HELPER_SHA256:
            guard.copy_new(args.frozen_repo / name, args.output / "frozen_helper" / name)
        guard.copy_new(manifest_path, args.output / "data_manifest.json")
        started = {
            "schema": "portable_e2e.map_town02_p00_holdout_run.v1",
            "started_at_utc": guard.utc_now(), "gpu": gpu,
            "dataset_manifest_sha256": EXPECTED_DATA_MANIFEST_SHA256,
            "training_source_sha256": guard.digest(Path(__file__)),
            "support_sha256": SUPPORT_SHA256,
            "frozen_helper_sha256": FROZEN_HELPER_SHA256,
            "research_scope_sha256": EXPECTED_RESEARCH_SCOPE_SHA256,
            "parent_checkpoint_sha256": EXPECTED_PARENT_CHECKPOINT_SHA256,
            "seed": SEED, "steps_per_arm": STEPS, "batch_size": BATCH_SIZE,
            "learning_rate": LEARNING_RATE, "weight_decay": WEIGHT_DECAY,
            "initialization": "scratch_both_arms",
            "augmentation": "TRAIN only: probability 0.5 aligned crop, scale 0.65..1.0, resize to native; no flip",
            "class_weights_source": "Town01 A TRAIN 497 only; fixed for B",
            "class_pixels": pixels.tolist(), "class_weights": weights_np.tolist(),
            "single_factor": "B adds retained 232 Town02_Opt p00 TRAIN frames to exact A prefix",
            "holdout": "Town02_Opt p02 whole route, historically exposed internal diagnostic only",
            "independent_dev_sites": 0, "automatic_promotion": False,
        }
        guard.write_new(args.output / "started.json", started)
        weights = torch.tensor(weights_np, dtype=torch.float32, device=device)

        def evaluate(network, name):
            rgb, target = arrays[name]
            confusion = np.zeros((len(LABELS), len(LABELS)), dtype=np.int64)
            predictions = []
            frame_iou = []
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
                    for sample in range(len(pred)):
                        code = target[begin + sample].astype(np.int64) * len(LABELS) + pred[sample]
                        matrix = np.bincount(code.ravel(), minlength=len(LABELS) ** 2).reshape(
                            len(LABELS), len(LABELS))
                        frame_iou.append(metrics_from_confusion(matrix, LABELS, np)["iou"])
            return metrics_from_confusion(confusion, LABELS, np), np.concatenate(predictions), frame_iou

        results = {}
        for arm, train_name in (("A", "train_a"), ("B", "train_b")):
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
                "model_id": "portable_e2e.map_semantics.town02_p00_holdout_" + arm.lower(),
                "optimizer_state_dict": optimizer.state_dict(), "steps": STEPS,
                "deployable": False, "research_contract": started,
            })
            town01, pred01, frame01 = evaluate(model, "dev_town01")
            p02, pred02, frame02 = evaluate(model, "holdout_town02_p02")
            np.savez_compressed(arm_dir / "town01_predictions.npz", prediction=pred01)
            np.savez_compressed(arm_dir / "p02_predictions.npz", prediction=pred02)
            guard.write_new(arm_dir / "frame_iou.json", {"town01": frame01, "p02": frame02})
            metrics = {"train_count": len(rgb), "training_wall_seconds": training_seconds,
                       "checkpoint_sha256": guard.digest(arm_dir / "checkpoint.pt"),
                       "town01": town01, "p02": p02, "deployment_approved": False}
            guard.write_new(arm_dir / "metrics.json", metrics)
            results[arm] = metrics
            if arm == "A":
                differences, reproduced = reproduction_gate(parent_summary, town01)
                guard.write_new(args.output / "a_reproduction.json", {
                    "iou_absolute_difference": differences,
                    "tolerance": REPRODUCTION_TOLERANCE, "pass": reproduced,
                })
                if not reproduced:
                    guard.write_new(args.output / "summary.json", {
                        "status": "A_REPRODUCTION_FAILED_B_NOT_RUN", "A": metrics,
                        "gpu_training_executed": True, "model_promotion": False,
                    })
                    print("A_REPRODUCTION_FAILED_B_NOT_RUN", flush=True)
                    return
        gates = probe_gates(results["A"], results["B"])
        guard.write_new(args.output / "summary.json", {
            "status": "A_B_INTERNAL_ROUTE_HOLDOUT_COMPLETE_NOT_PROMOTED",
            "A": results["A"], "B": results["B"], "gates": gates,
            "gpu_training_executed": True, "model_promotion": False,
            "independent_dev_sites": 0, "hd_map_complete": False,
            "learned_closed_loop_driving": False, "completed_at_utc": guard.utc_now(),
        })
        print("A_B_INTERNAL_ROUTE_HOLDOUT_COMPLETE_NOT_PROMOTED", flush=True)


if __name__ == "__main__":
    main()
