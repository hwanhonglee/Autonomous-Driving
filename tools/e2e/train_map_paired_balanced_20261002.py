#!/usr/bin/env python3
"""HH_261002 - Two fixed same-parent map fits; only matched paired OFF/ON exposure differs."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time

sys.dont_write_bytecode = True
try:
    from . import map_paired_balanced_contract_20261002 as contract
except ImportError:
    import map_paired_balanced_contract_20261002 as contract
import numpy as np

WORKSPACE = Path("/home/pro6000/personal/hwanhong/portable_e2e")
SOURCE = WORKSPACE / "source_worktrees" / contract.NAME
DATA = WORKSPACE.parent / "dataset" / contract.NAME
OUTPUT = WORKSPACE / "runs/diagnostics" / contract.NAME
EVAL_DATA = WORKSPACE.parent / "dataset/map_town02_p00_route_holdout_20260929_v1"
EVAL_MANIFEST_SHA = "f4764b9aa9d015f3d136a145f6194dd60bda8900c27dd2423de6145b9f472834"
EVAL_FILES = {"dev_town01": (118, "417c60f924abb044617e2aba5b1afd70b83f5da7f905e54513e9c220172ba4b3"),
              "holdout_town02_p02": (232, "858f92ccd874c154576d3cf6a85132508801a22671baf6d0c07abe3936eb16cb")}
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
PAIR_BUDGET_SECONDS = 1800
MIN_FREE_BYTES = 5 * 1024**3
WRITE_RESERVE_BYTES = 512 * 1024**2


def reserve(path: Path) -> None:
    contract.canonical(path)
    parent = next(item for item in (path, *path.parents) if item.exists())
    contract.require(shutil.disk_usage(parent).free >= MIN_FREE_BYTES + WRITE_RESERVE_BYTES,
                     "5 GiB free plus 512 MiB checkpoint write reserve required")


def environment_contract() -> None:
    contract.require(os.uname().nodename == "pro6000" and Path(sys.prefix) == WORKSPACE / "venvs/py312",
                     "existing personal py312 venv on pro6000 required")
    contract.require(Path.cwd() == SOURCE and Path(__file__).resolve().parent == SOURCE,
                     "exact declared personal source directory required")
    required = {"CUDA_VISIBLE_DEVICES": GPU_UUID, "CUBLAS_WORKSPACE_CONFIG": ":4096:8",
                "PYTHONNOUSERSITE": "1", "OMP_NUM_THREADS": "16", "MKL_NUM_THREADS": "16"}
    contract.require(all(os.environ.get(key) == value for key, value in required.items()),
                     "single physical GPU0/deterministic environment differs")
    contract.require(not any(os.environ.get(key) for key in
                             ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP", "PYTHONUSERBASE")), "Python path override forbidden")
    for path in (WORKSPACE, SOURCE, DATA, EVAL_DATA, OUTPUT.parent):
        contract.canonical(path)
        contract.require(path.is_dir(), "canonical personal directory required: " + str(path))
    contract.canonical(OUTPUT)
    contract.require(not OUTPUT.exists(), "fresh output required; no resume, retry, or best-checkpoint selection")
    reserve(OUTPUT)


def source_contract(root, source_sha, data_sha, registry_sha):
    bundle = contract.read_json(root / "bundle.json", source_sha)
    contract.require(bundle.get("schema") == contract.SOURCE_SCHEMA and bundle.get("study_id") == contract.NAME
                     and set(bundle.get("files", {})) == contract.SOURCE_FILES
                     and bundle.get("data_manifest_sha256") == data_sha
                     and bundle.get("study_registry_sha256") == registry_sha, "source/data/registration binding differs")
    for name, sha in bundle["files"].items():
        contract.require(contract.digest(root / name) == sha, "declared source changed: " + name)
    for name, sha in contract.FROZEN_SOURCE_SHA.items():
        contract.require(bundle["files"][name] == sha, "frozen architecture/helper/guard changed")
    contract.require(bundle["files"]["plan.md"] == bundle.get("training_plan_sha256"), "source plan binding differs")
    return bundle


def load_weights(model, path, expected_sha):
    """HH_261002 - Exact numeric NPZ payload; checkpoint pickle is never read."""
    import torch
    template, data = model.state_dict(), contract.read_npz(path, expected_sha)
    contract.require(set(data) == set(template), "model learned-tensor keys differ")
    state = {}
    for name, tensor in template.items():
        value = data[name]
        contract.require(value.shape == tuple(tensor.shape) and value.dtype == np.float32
                         and bool(np.isfinite(value).all()), "finite float32 model ABI differs: " + name)
        state[name] = torch.from_numpy(value.copy())
    model.load_state_dict(state, strict=True)
    contract.require(all(value.requires_grad for value in model.parameters()), "all original245831 parameters must train")


def batch_arrays(arrays, indices, boxes, bits, arm):
    contract.require(arm in contract.ARMS and len(indices) == len(boxes) == len(bits), "unknown arm/batch ABI")
    contract.require(bool(np.isin(bits, (0, 1)).all()), "view bits must be exactly binary")
    original_rgb, original_target, pair_false, pair_true, pair_target = arrays
    pairs = []
    actual_bits = np.zeros(len(indices), dtype=np.uint8)
    for position, (index, box, bit) in enumerate(zip(indices, boxes, bits)):
        contract.require(type(index) in (int, np.int64) and 0 <= int(index) < contract.TOTAL_COUNT, "physical slot out of bounds")
        if index < contract.ORIGINAL_COUNT:
            contract.require(int(bit) == 0, "original replay view must remain unchanged")
            rgb, target = original_rgb[index], original_target[index]
        else:
            pair = int(index) - contract.ORIGINAL_COUNT
            actual_bits[position] = int(bit) if arm == "balanced" else 0
            rgb = (pair_true if actual_bits[position] else pair_false)[pair]
            target = pair_target[pair]
        pairs.append(contract.apply_crop(rgb, target, box))
    return np.stack([row[0] for row in pairs]), np.stack([row[1] for row in pairs]), actual_bits


def confusion_metrics(matrix):
    actual, predicted, diagonal = matrix.sum(1), matrix.sum(0), np.diag(matrix)
    union = actual + predicted - diagonal
    return {"confusion": matrix.tolist(), "target_pixels": dict(zip(contract.LABELS, actual.tolist())),
            "predicted_pixels": dict(zip(contract.LABELS, predicted.tolist())),
            "false_positive_pixels": dict(zip(contract.LABELS, (predicted-diagonal).tolist())),
            "iou": {label: float(diagonal[i]/union[i]) if actual[i] else None for i, label in enumerate(contract.LABELS)},
            "pixel_accuracy": float(diagonal.sum()/matrix.sum()), "absent_target_class_iou": "null; false positives retained"}


def evaluate_endpoints(arm_info, model_class, guard):
    """HH_261002 - Known exposed diagnostic arrays open only after BOTH immutable final10000 exports."""
    import torch
    contract.require(set(arm_info) == set(contract.ARMS), "both fixed final endpoints required before DEV")
    for arm, info in arm_info.items():
        contract.require(info["steps"] == contract.STEPS
                         and contract.digest(OUTPUT / arm / "weights.npz") == info["weights_sha256"], "endpoint incomplete or changed")
    contract.read_json(EVAL_DATA / "manifest.json", EVAL_MANIFEST_SHA)
    results = {}
    for split, (count, sha) in EVAL_FILES.items():
        path = EVAL_DATA / (split + ".npz")
        data = contract.read_npz(path, sha)
        contract.require(set(data) == {"rgb", "target"}, "known evaluation tensor keys differ")
        rgb, target = data["rgb"], data["target"]
        contract.require(rgb.shape == (count, 360, 640, 3) and target.shape == (count, 360, 640)
                         and rgb.dtype == target.dtype == np.uint8 and int(target.max()) < 7, "known diagnostic data ABI differs")
        results[split] = {}
        for arm, info in arm_info.items():
            model = model_class()
            load_weights(model, OUTPUT / arm / "weights.npz", info["weights_sha256"])
            model.to("cuda:0").eval()
            predictions, matrix = np.empty_like(target), np.zeros((7, 7), dtype=np.int64)
            with torch.no_grad():
                for begin in range(0, count, contract.BATCH):
                    x = torch.tensor(rgb[begin:begin+contract.BATCH].transpose(0, 3, 1, 2).copy(), dtype=torch.float32, device="cuda:0")/255
                    pred = model(x).argmax(1).cpu().numpy().astype(np.uint8)
                    predictions[begin:begin+len(pred)] = pred
                    encoded = target[begin:begin+len(pred)].astype(np.int64)*7 + pred
                    matrix += np.bincount(encoded.ravel(), minlength=49).reshape(7, 7)
            reserve(OUTPUT)
            with (OUTPUT / arm / (split + "_predictions.npz")).open("xb") as stream:
                np.savez_compressed(stream, prediction=predictions)
            results[split][arm] = {"count": count, "source_sha256": sha, "known_exposed_diagnostic_not_independent": True,
                                  **confusion_metrics(matrix)}
            del model
        contract.require(contract.digest(path) == sha, "known diagnostic bytes changed during evaluation")
    guard.write_new(OUTPUT / "known_evaluation.json", results)
    return results


def postcheck(bundle, manifest, args):
    source_contract(SOURCE, args.expected_source_sha256, args.expected_data_sha256, args.expected_registry_sha256)
    for name, ref in manifest["files"].items():
        contract.require(contract.digest(DATA / name) == ref["sha256"], "training bytes changed: " + name)
    contract.require(contract.digest(DATA / "manifest.json") == args.expected_data_sha256
                     and contract.digest(DATA / "study_train_registry.json") == args.expected_registry_sha256
                     and contract.digest(DATA / "paired_samples.json") == manifest["paired_samples_sha256"], "registration changed during fit")
    for name, sha in bundle["files"].items():
        contract.require(contract.digest(OUTPUT / "source" / name) == sha, "source archive changed")
    contract.require(contract.digest(OUTPUT / "source/bundle.json") == args.expected_source_sha256,
                     "source bundle archive changed")
    for name, sha in (("manifest.json", args.expected_data_sha256),
                      ("study_train_registry.json", args.expected_registry_sha256),
                      ("paired_samples.json", manifest["paired_samples_sha256"])):
        contract.require(contract.digest(OUTPUT / "inputs" / name) == sha, "input receipt archive changed")
    registry = contract.read_json(DATA / "study_train_registry.json", args.expected_registry_sha256)
    for filename, sha in registry["copied_provenance_files"].items():
        contract.require(contract.digest(DATA / filename) == sha, "source evidence changed during fitting")


def run(args):
    environment_contract()
    bundle = source_contract(SOURCE, args.expected_source_sha256, args.expected_data_sha256, args.expected_registry_sha256)
    manifest, arrays, indices, crops, bits = contract.load_package(DATA, args.expected_data_sha256, args.expected_registry_sha256)
    registry = contract.read_json(DATA / "study_train_registry.json", args.expected_registry_sha256)
    contract.require(bundle["training_plan_sha256"] == registry["training_plan_sha256"], "registered/source plan differs")
    receipt = {"study_id": contract.NAME, "data_manifest_sha256": args.expected_data_sha256,
               "source_bundle_sha256": args.expected_source_sha256, "study_registry_sha256": args.expected_registry_sha256,
               "parent_weights_sha256": contract.PARENT_SHA, "schedule_contract": manifest["schedule_contract"],
               "counts": contract.COUNTS, "arms": contract.ARMS, "optimization": contract.optimization(),
               "common_class_weights": manifest["common_class_weights"],
               "sole_factor": "same480 physical slots: alwaysFALSE versus predeclared balanced FALSE/TRUE",
               "native_diagnostic_denials_preserved": True, "ctrack_train_eligible": False,
               "model_promoted": False, "vehicle_control_approved": False, "independent_new_dev_sites": 0}
    if args.validate_only:
        print(json.dumps({"status": "CPU_PREFLIGHT_PASS_NO_GPU_OR_OUTPUT", **receipt}), flush=True)
        return
    reserve(OUTPUT)
    # HH_261002 - Import original model/guard only after full immutable source checks; no installation.
    import torch
    import train_map_photometric_pilot as guard
    import map_context_recovery as model_module
    import map_perception as perception
    for module in (guard, model_module, perception):
        contract.require(Path(module.__file__).resolve().parent == SOURCE, "model/guard provider escaped declared source")
    MapContextRecovery, semantic_loss = model_module.MapContextRecovery, perception.semantic_loss
    torch.set_num_threads(16); torch.set_num_interop_threads(1)
    torch.use_deterministic_algorithms(True)
    torch.backends.cuda.matmul.allow_tf32 = False; torch.backends.cudnn.allow_tf32 = False
    with guard.gpu0_lease():
        reserve(OUTPUT)
        gpu = guard.preflight_gpu()
        OUTPUT.mkdir()
        for name in (*bundle["files"], "bundle.json"):
            guard.copy_new(SOURCE / name, OUTPUT / "source" / name)
        for name in ("manifest.json", "study_train_registry.json", "paired_samples.json"):
            guard.copy_new(DATA / name, OUTPUT / "inputs" / name)
        guard.write_new(OUTPUT / "started.json", {**receipt, "gpu": gpu, "created_at_utc": datetime.now(timezone.utc).isoformat()})
        deadline, arm_info = time.monotonic() + PAIR_BUDGET_SECONDS, {}
        weights = torch.tensor(manifest["common_class_weights"], dtype=torch.float32, device="cuda:0")
        for arm in contract.ARMS:
            guard.seed_everything(contract.SEED)
            model = MapContextRecovery()
            load_weights(model, DATA / "parent_weights.npz", contract.PARENT_SHA)
            initial_sha = guard.state_digest(model)
            contract.require(sum(value.numel() for value in model.parameters()) == 245831, "unchanged seven-label B architecture required")
            model.to("cuda:0").train()
            optimizer = torch.optim.AdamW(model.parameters(), lr=contract.LR, weight_decay=contract.WEIGHT_DECAY)
            contract.require(len(optimizer.state) == 0, "fresh optimizer required in both arms")
            arm_dir = OUTPUT / arm
            arm_dir.mkdir()
            started, total = time.monotonic(), 0.
            samples_seen, crops_seen, views_seen = hashlib.sha256(), hashlib.sha256(), hashlib.sha256()
            observed_on, observed_off = np.zeros(480, dtype=np.int64), np.zeros(480, dtype=np.int64)
            with (arm_dir / "history.jsonl").open("x") as log, (arm_dir / "exposures.jsonl").open("x") as exposure_log:
                for step in range(contract.STEPS):
                    contract.require(time.monotonic() < deadline, "bounded30minute paired fit wall budget exhausted")
                    ids, boxes = indices[step], crops[step]
                    rgb, target, actual_views = batch_arrays(arrays, ids, boxes, bits[step], arm)
                    samples_seen.update(ids.tobytes()); crops_seen.update(boxes.tobytes()); views_seen.update(actual_views.tobytes())
                    for index, bit in zip(ids, actual_views):
                        if index >= 729: (observed_on if bit else observed_off)[int(index)-729] += 1
                    x = torch.tensor(rgb.transpose(0, 3, 1, 2).copy(), dtype=torch.float32, device="cuda:0")/255
                    y = torch.tensor(target.astype(np.int64), device="cuda:0")
                    optimizer.zero_grad(set_to_none=True)
                    loss = semantic_loss(model(x), y, weights)
                    contract.require(bool(torch.isfinite(loss)), "nonfinite training loss")
                    loss.backward()
                    norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 5.)
                    contract.require(bool(torch.isfinite(norm)), "nonfinite aggregate training gradient")
                    optimizer.step()
                    exposure_log.write(json.dumps({"step": step+1, "physical_slot_indices": ids.tolist(),
                                                   "shared_crop_boxes": boxes.tolist(), "actual_paired_TRUE_bits": actual_views.tolist()}) + "\n")
                    total += float(loss.detach())
                    if (step+1) % 100 == 0:
                        row = {"arm": arm, "step": step+1, "mean_loss_100": total/100, "wall_seconds": time.monotonic()-started}
                        log.write(json.dumps(row, allow_nan=False) + "\n"); log.flush(); exposure_log.flush()
                        print(json.dumps(row), flush=True); total = 0.
            torch.cuda.synchronize()
            reserve(OUTPUT)
            state = {name: value.detach().cpu().numpy().copy() for name, value in model.state_dict().items()}
            contract.require(all(value.dtype == np.float32 and bool(np.isfinite(value).all()) for value in state.values()),
                             "nonfinite final learned state")
            checkpoint = {"model_state_dict": model.state_dict(), "optimizer_state_dict": optimizer.state_dict(),
                          "labels": contract.LABELS, "steps": contract.STEPS, "study_id": contract.NAME, "arm": arm,
                          "parent_weights_sha256": contract.PARENT_SHA, "deployable": False}
            with (arm_dir / "checkpoint.pt").open("xb") as stream: torch.save(checkpoint, stream)
            reserve(OUTPUT)
            with (arm_dir / "weights.npz").open("xb") as stream: np.savez_compressed(stream, **state)
            info = {"steps": contract.STEPS, "examples_seen": contract.STEPS*contract.BATCH,
                    "initial_state_sha256": initial_sha, "parameter_count": 245831,
                    "training_wall_seconds": time.monotonic()-started, "sample_sequence_sha256": samples_seen.hexdigest(),
                    "crop_sequence_sha256": crops_seen.hexdigest(), "view_sequence_sha256": views_seen.hexdigest(),
                    "paired_FALSE_exposures_per_pair": observed_off.tolist(), "paired_TRUE_exposures_per_pair": observed_on.tolist(),
                    "weights_sha256": contract.digest(arm_dir / "weights.npz"), "checkpoint_sha256": contract.digest(arm_dir / "checkpoint.pt"),
                    "full_exposure_journal_sha256": contract.digest(arm_dir / "exposures.jsonl"), "optimizer_initial_state_entries": 0}
            proof = manifest["schedule_contract"]
            contract.require(info["sample_sequence_sha256"] == proof["indices_sha256"]
                             and info["crop_sequence_sha256"] == proof["crops_sha256"]
                             and info["view_sequence_sha256"] == proof[arm + "_view_bits_sha256"], "actual complete exposure sequence differs")
            if arm == "balanced":
                contract.require(observed_off.tolist() == proof["balanced_FALSE_exposures_per_pair"]
                                 and observed_on.tolist() == proof["balanced_TRUE_exposures_per_pair"], "actual mixed view counts differ")
            else:
                contract.require(not observed_on.any()
                                 and observed_off.tolist() == proof["sample_exposures"][729:], "control physical slots differ")
            guard.write_new(arm_dir / "training.json", info)
            arm_info[arm] = info
            del model, optimizer, checkpoint, x, y, loss, state
            torch.cuda.empty_cache()
        contract.require(arm_info["control"]["initial_state_sha256"] == arm_info["balanced"]["initial_state_sha256"], "same parent initialization differs")
        guard.write_new(OUTPUT / "fit_complete.json", {"status": "BOTH_FIXED_FINAL10000_ENDPOINTS_SAVED_BEFORE_DEV", "arms": arm_info})
        evaluated = evaluate_endpoints(arm_info, MapContextRecovery, guard)
        postcheck(bundle, manifest, args)
        result = {"status": "MATCHED_BALANCED_PAIR_COMPLETE_DIAGNOSTIC_NOT_PROMOTED", **receipt, "arms_result": arm_info,
                  "known_evaluation": evaluated, "dev_used_during_optimization": False, "source_and_data_unchanged": True}
        guard.write_new(OUTPUT / "summary.json", result)
        print(json.dumps({"status": result["status"], "output": str(OUTPUT)}), flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    for name in ("data", "source", "registry"):
        parser.add_argument("--expected-" + name + "-sha256", required=True)
    parser.add_argument("--validate-only", action="store_true")
    run(parser.parse_args(argv))
    return 0


if __name__ == "__main__": raise SystemExit(main())
