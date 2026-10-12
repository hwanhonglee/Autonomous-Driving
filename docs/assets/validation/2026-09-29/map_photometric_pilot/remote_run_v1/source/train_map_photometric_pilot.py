#!/usr/bin/env python3
"""HH_260929 - Train one non-promotable D1 image-only policy on personal GPU0."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
import time

import numpy as np
import torch


WORKSPACE = Path("/home/pro6000/personal/hwanhong/portable_e2e")
SOURCE_NAME = "map_photometric_pilot_20260929_v1"
SOURCE = WORKSPACE / "source_worktrees" / SOURCE_NAME
DATA = WORKSPACE.parent / "dataset/map_road_lane_ab_20260929_v1"
BASELINE = WORKSPACE / "runs/diagnostics/map_road_lane_ab_20260929_v1"
OUTPUT = WORKSPACE / "runs/diagnostics" / SOURCE_NAME
GPU0_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
PLAN_SHA256 = "79c14b50b268fa3a51f023baf34f602faa95cd687b7d90158911ca1372609579"
DATA_SHA256 = "313b41ed4fb897f01fb4fb433c1bafb234ef9b234e62f96a462ef2188e9dccc6"
D0_SHA256 = "69dae433616cf27690310af9ff1923ef3b2e67641fa935e2a2f77116eea02ace"
D1_SHA256 = "4e2f7df61814a7e8aade88877482506a30e41c9fff0b5e6b7c3182193ff7d8f8"
PARENT_SHA256 = "e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c"
BASELINE_SHA256 = {
    "started.json": "05b29737a73c258444931b9dfd7a6ff7ebd462f7c48dee3486b0f87c2f90eb20",
    "summary.json": "cce54a989c09a205dc34f97ac3b89b75df43f38e6648717dd660674081888ff0",
    "D1/metrics.json": "32678cd5225532b1700bb90ddc385d44a516db3fd6ad936e798539231cb48624",
    "D1/checkpoint.pt": "c583be21a082fbfaa0a9ed1346dc667f032ef928582bd8c537b949039404c38f",
    "source/train_map_road_lane_ab.py": "ac3c71c071cc9db9f6b36fe0948ece9d034313c00853f986f843bb13a40e3c12",
}
SUPPORT_SHA256 = {
    "map_perception.py": "1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b",
    "map_learning_balance.py": "57c9392d8b700f5cf628865042b66ae630961f35fb8858a5d0ead77fe5a7c73b",
    "dual_recovery_objectives.py": "d10bcad9060c1f4bde40b20773fd4c18f1d4b6ba3861dd7cef55cd36cff4ad15",
}
LABELS = ("other", "road", "sidewalk", "lane_marking",
          "traffic_light_fixture", "traffic_sign", "pole")
INITIAL_STATE_SHA256 = "e78098947d72df35bcee1eb8719ad0e9b0acd67ec6a1e393197d477823da2058"
SEED = 20260922
CROP_SEED = 20260926
PHOTO_SEED = 20260929
STEPS = 2000
BATCH_SIZE = 8
LEARNING_RATE = 3e-4
WEIGHT_DECAY = 1e-4


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def digest(path: Path) -> str:
    """HH_260929 - Hash regular immutable inputs before work begins."""
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(block)
    return sha.hexdigest()


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_new(path: Path, value: dict) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def copy_new(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as original, target.open("xb") as archived:
        shutil.copyfileobj(original, archived, 1024 * 1024)
        archived.flush()
        os.fsync(archived.fileno())
    require(digest(target) == digest(source), "source archive bytes changed")


def photometric(image: np.ndarray, rng: np.random.Generator) -> tuple[np.ndarray, bool]:
    """HH_260929 - Apply one jointly gated grayscale-and-darkness TRAIN policy."""
    require(image.ndim == 3 and image.shape[2] == 3 and image.dtype == np.uint8,
            "uint8 RGB crop required")
    if rng.random() >= 0.5:
        return image.copy(), False
    blend = float(rng.uniform(0.0, 1.0))
    scale = float(rng.uniform(0.35, 1.0))
    source = image.astype(np.float32)
    gray = (0.299 * source[..., 0:1] + 0.587 * source[..., 1:2]
            + 0.114 * source[..., 2:3])
    transformed = np.rint(np.clip(scale * ((1.0 - blend) * source + blend * gray),
                                  0.0, 255.0)).astype(np.uint8)
    return transformed, True


def exploratory_gate(baseline: dict, candidate: dict) -> dict:
    """HH_260929 - Feasible repeated-DEV diagnostics, never a release decision."""
    deltas = {}
    for town in ("town01", "town05"):
        base, trial = baseline[town]["iou"], candidate[town]["iou"]
        require(set(base) == set(LABELS) and set(trial) == set(LABELS),
                "seven-class metrics required")
        deltas[town] = {name: None if base[name] is None or trial[name] is None
                        else trial[name] - base[name] for name in LABELS}
    require(all(deltas[town][name] is not None for town, name in
                (("town05", "lane_marking"), ("town05", "road"),
                 ("town01", "road"), ("town01", "lane_marking"))),
            "road/lane IoU must be defined")
    checks = {
        "town05_lane_plus_5pp_vs_d1": deltas["town05"]["lane_marking"] >= 0.05,
        "town05_road_drop_at_most_1pp_vs_d1": deltas["town05"]["road"] >= -0.01,
        "town01_road_drop_at_most_1pp_vs_d1": deltas["town01"]["road"] >= -0.01,
        "town01_lane_drop_at_most_1pp_vs_d1": deltas["town01"]["lane_marking"] >= -0.01,
    }
    return {"iou_delta_vs_frozen_d1": deltas, "checks": checks,
            "passed_exploratory_only": all(checks.values()),
            "independent_validation": False, "model_promotion": False}


def validate_environment() -> None:
    require(os.uname().nodename == "pro6000", "personal training host required")
    require(Path(sys.prefix).absolute() == WORKSPACE / "venvs/py312",
            "personal py312 venv required")
    require(Path.cwd().resolve() == SOURCE.resolve()
            and Path(__file__).resolve().parent == SOURCE.resolve(),
            "frozen personal source directory required")
    expected = {"CUDA_VISIBLE_DEVICES": GPU0_UUID, "CUBLAS_WORKSPACE_CONFIG": ":4096:8",
                "PYTHONNOUSERSITE": "1", "OMP_NUM_THREADS": "16", "MKL_NUM_THREADS": "16"}
    require(all(os.environ.get(name) == value for name, value in expected.items()),
            "deterministic GPU0 environment differs")
    require(not any(os.environ.get(name) for name in
                    ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP", "PYTHONUSERBASE")),
            "inherited Python path overrides forbidden")
    for path in (WORKSPACE, SOURCE, DATA, BASELINE, OUTPUT.parent):
        require(path.is_dir() and not path.is_symlink(), f"personal path missing: {path}")
    require(not OUTPUT.exists() and not OUTPUT.is_symlink(), "fresh output required")


def load_train(path: Path, expected_sha: str, count: int) -> tuple[np.ndarray, np.ndarray]:
    require(digest(path) == expected_sha, "TRAIN archive bytes changed")
    with np.load(path, allow_pickle=False) as archive:
        require(set(archive.files) == {"rgb", "target"}, "TRAIN archive keys changed")
        rgb, target = archive["rgb"], archive["target"]
    require(rgb.shape == (count, 360, 640, 3) and target.shape == (count, 360, 640)
            and rgb.dtype == target.dtype == np.uint8 and int(target.max()) < 7,
            "TRAIN array contract changed")
    return rgb, target


def preflight() -> tuple[dict, np.ndarray, np.ndarray, np.ndarray]:
    """HH_260929 - Inspect only TRAIN arrays and frozen run metadata before GPU."""
    validate_environment()
    source_sha = digest(Path(__file__))
    require(digest(SOURCE / "plan.md") == PLAN_SHA256, "preregistered plan changed")
    for name, expected in SUPPORT_SHA256.items():
        require(digest(SOURCE / name) == expected, f"frozen support changed: {name}")
    for name, expected in BASELINE_SHA256.items():
        require(digest(BASELINE / name) == expected, f"frozen D1 baseline changed: {name}")
    require(digest(DATA / "manifest.json") == DATA_SHA256, "dataset manifest changed")
    manifest = read_json(DATA / "manifest.json")
    require(manifest.get("schema") == "portable_e2e.map_road_lane_ab_data.v1"
            and manifest.get("parent_manifest_sha256") == PARENT_SHA256
            and tuple(manifest.get("labels", ())) == LABELS
            and manifest.get("town02_split") == "TRAIN_EXPLORATORY_ROAD_LANE_ONLY"
            and manifest.get("town05_split") == "DEV_ONLY"
            and manifest["split_contract"]["dev_used_for_training"] is False
            and manifest["split_contract"]["test_used"] is False,
            "dataset split contract changed")
    files = manifest["files"]
    require(set(files) == {"train_d0", "train_d1", "dev_town01", "dev_town05"}
            and files["train_d0"]["sha256"] == D0_SHA256
            and files["train_d1"]["sha256"] == D1_SHA256
            and files["train_d0"]["count"] == 497
            and files["train_d1"]["count"] == 729,
            "dataset TRAIN identity changed")
    d0_rgb, d0_target = load_train(DATA / "train_d0.npz", D0_SHA256, 497)
    d1_rgb, d1_target = load_train(DATA / "train_d1.npz", D1_SHA256, 729)
    require(np.array_equal(d1_rgb[:497], d0_rgb)
            and np.array_equal(d1_target[:497], d0_target), "D1 D0 prefix changed")
    for name, target in (("train_d0", d0_target), ("train_d1", d1_target)):
        require(np.bincount(target.ravel(), minlength=7).tolist()
                == files[name]["class_pixels"], f"{name} class counts changed")
    sys.path.insert(0, str(SOURCE))
    from map_learning_balance import class_weights
    weights = class_weights(np.bincount(d0_target.ravel(), minlength=7), "sqrt_frequency")
    started = read_json(BASELINE / "started.json")
    baseline = read_json(BASELINE / "D1/metrics.json")
    summary = read_json(BASELINE / "summary.json")
    require(started.get("schema") == "portable_e2e.map_road_lane_ab_run.v1"
            and started.get("training_source_sha256")
            == BASELINE_SHA256["source/train_map_road_lane_ab.py"]
            and started.get("data_manifest_sha256") == DATA_SHA256
            and started.get("seed") == SEED and started.get("steps_per_arm") == STEPS
            and started.get("batch_size") == BATCH_SIZE
            and started.get("learning_rate") == LEARNING_RATE
            and started.get("weight_decay") == WEIGHT_DECAY
            and started.get("initialization") == "scratch_both_arms"
            and started.get("class_weights_source") == "Town01 D0 TRAIN only; fixed for D1"
            and np.array_equal(np.asarray(started.get("class_weights")), weights)
            and started.get("support_sha256") == SUPPORT_SHA256,
            "frozen D1 training contract changed")
    require(summary.get("status") == "A_B_COMPLETE_EXPLORATORY_NOT_PROMOTED"
            and summary.get("D1") == baseline and baseline.get("train_count") == 729
            and baseline.get("checkpoint_sha256") == BASELINE_SHA256["D1/checkpoint.pt"]
            and baseline.get("deployment_approved") is False
            and summary.get("model_promotion") is False,
            "frozen D1 result contract changed")
    receipt = {"status": "CPU_PREFLIGHT_PASS", "runner_sha256": source_sha,
               "plan_sha256": PLAN_SHA256, "data_manifest_sha256": DATA_SHA256,
               "train_d0_sha256": D0_SHA256, "train_d1_sha256": D1_SHA256,
               "train_frames": 729, "class_weights": weights.tolist(),
               "baseline_file_sha256": BASELINE_SHA256,
               "baseline_town01_iou": baseline["town01"]["iou"],
               "baseline_town05_iou": baseline["town05"]["iou"],
               "dev_arrays_opened_before_checkpoint": False, "test_used": False}
    return receipt, d1_rgb, d1_target, weights


@contextmanager
def gpu0_lease():
    """HH_260929 - Cooperate with all personal GPU0 runs through the shared lock."""
    lock = WORKSPACE / "runs/campaigns/.gpu0_training.lock"
    require(lock.parent.is_dir() and not lock.parent.is_symlink(), "GPU lease parent missing")
    flags = os.O_RDWR | os.O_CREAT | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(lock, flags, 0o600)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ValueError("physical GPU0 is leased by another training run") from error
        yield
    finally:
        os.close(descriptor)


def preflight_gpu() -> dict:
    """HH_260929 - Never select a busy GPU or fall back to physical GPU1."""
    rows = subprocess.check_output(
        ["nvidia-smi", "--query-gpu=index,uuid,memory.used,utilization.gpu",
         "--format=csv,noheader,nounits"], timeout=20, text=True).splitlines()
    matches = [[field.strip() for field in row.split(",")]
               for row in rows if GPU0_UUID in row]
    require(len(matches) == 1 and matches[0][0] == "0"
            and matches[0][1] == GPU0_UUID
            and int(matches[0][2]) == 0 and int(matches[0][3]) == 0,
            "approved physical GPU0 absent or occupied")
    require(torch.cuda.is_available() and torch.cuda.device_count() == 1,
            "exactly one CUDA device must be visible")
    properties = torch.cuda.get_device_properties(torch.device("cuda:0"))
    require(str(getattr(properties, "uuid", "")) in
            (GPU0_UUID, GPU0_UUID.removeprefix("GPU-")),
            "PyTorch visible GPU UUID differs")
    return {"physical_index": 0, "uuid": GPU0_UUID, "memory_used_mib_before_torch": 0,
            "utilization_percent_before_torch": 0, "name": properties.name}


def seed_everything(seed: int) -> None:
    """HH_260929 - Mirror the frozen baseline's exact deterministic seed calls."""
    random.seed(seed)
    np.random.seed(seed % (2**32))
    torch.random.default_generator.manual_seed(seed)
    with torch.cuda.device(torch.device("cuda:0")):
        torch.cuda.manual_seed(seed)
    torch.use_deterministic_algorithms(True)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True


def state_digest(model: torch.nn.Module) -> str:
    sha = hashlib.sha256()
    for name, tensor in sorted(model.state_dict().items()):
        value = tensor.detach().cpu().contiguous()
        header = json.dumps([name, str(value.dtype), list(value.shape)]).encode("utf-8")
        sha.update(len(header).to_bytes(8, "big"))
        sha.update(header)
        sha.update(value.numpy().tobytes())
    return sha.hexdigest()


def metrics_from_confusion(confusion: np.ndarray) -> dict:
    intersection = np.diag(confusion)
    union = confusion.sum(0) + confusion.sum(1) - intersection
    predicted = confusion.sum(0)
    return {"confusion": confusion.tolist(),
            "iou": {name: float(intersection[i] / union[i]) if union[i] else None
                    for i, name in enumerate(LABELS)},
            "precision": {name: float(intersection[i] / predicted[i]) if predicted[i] else None
                          for i, name in enumerate(LABELS)},
            "pixel_accuracy": float(intersection.sum() / confusion.sum())}


def evaluate(network: torch.nn.Module, split: str, manifest: dict) -> tuple[dict, np.ndarray]:
    """HH_260929 - This is called only after the trained checkpoint is sealed."""
    ref = manifest["files"][split]
    path = DATA / ref["file"]
    require(digest(path) == ref["sha256"] and ref["count"] in (118, 232),
            "declared DEV array changed")
    with np.load(path, allow_pickle=False) as archive:
        require(set(archive.files) == {"rgb", "target"}, "DEV array keys changed")
        rgb, target = archive["rgb"], archive["target"]
    require(rgb.shape == (ref["count"], 360, 640, 3)
            and target.shape == (ref["count"], 360, 640)
            and rgb.dtype == target.dtype == np.uint8,
            "DEV array contract changed")
    confusion = np.zeros((7, 7), dtype=np.int64)
    predictions = []
    network.eval()
    with torch.no_grad():
        for begin in range(0, len(rgb), BATCH_SIZE):
            images = torch.tensor(rgb[begin:begin + BATCH_SIZE].transpose(0, 3, 1, 2).copy(),
                                  device="cuda:0", dtype=torch.float32) / 255
            prediction = network(images).argmax(1).cpu().numpy().astype(np.uint8)
            predictions.append(prediction)
            encoded = target[begin:begin + BATCH_SIZE].astype(np.int64) * 7 + prediction
            confusion += np.bincount(encoded.ravel(), minlength=49).reshape(7, 7)
    return metrics_from_confusion(confusion), np.concatenate(predictions)


def run() -> dict:
    contract, rgb, target, weights_np = preflight()
    require(read_json(SOURCE / "cpu_preflight_receipt.json") == contract,
            "CPU preflight receipt absent or changed")
    from dual_recovery_objectives import paired_crop
    from map_perception import MapPerception, semantic_loss
    with gpu0_lease():
        gpu = preflight_gpu()
        OUTPUT.mkdir(parents=True, exist_ok=False)
        for name in (Path(__file__).name, "plan.md", "cpu_preflight_receipt.json",
                     *SUPPORT_SHA256):
            copy_new(SOURCE / name, OUTPUT / "source" / name)
        copy_new(DATA / "manifest.json", OUTPUT / "data_manifest.json")
        write_new(OUTPUT / "cpu_preflight.json", contract)
        seed_everything(SEED)
        model = MapPerception().to("cuda:0")
        initial_sha = state_digest(model)
        require(initial_sha == INITIAL_STATE_SHA256, "frozen scratch initialization changed")
        optimizer = torch.optim.AdamW(model.parameters(), lr=LEARNING_RATE,
                                      weight_decay=WEIGHT_DECAY)
        sampler = np.random.default_rng(SEED)
        crop_rng = np.random.default_rng(CROP_SEED)
        photo_rng = np.random.default_rng(PHOTO_SEED)
        weights = torch.tensor(weights_np, dtype=torch.float32, device="cuda:0")
        started = {"schema": "portable_e2e.map_photometric_pilot_run.v1",
                   "started_at_utc": datetime.now(timezone.utc).isoformat(),
                   "gpu": gpu, "runner_sha256": contract["runner_sha256"],
                   "plan_sha256": PLAN_SHA256, "data_manifest_sha256": DATA_SHA256,
                   "baseline_file_sha256": BASELINE_SHA256,
                   "seed": SEED, "crop_seed": CROP_SEED, "photometric_seed": PHOTO_SEED,
                   "steps": STEPS, "batch_size": BATCH_SIZE,
                   "learning_rate": LEARNING_RATE, "weight_decay": WEIGHT_DECAY,
                   "class_weights": weights_np.tolist(), "initial_state_sha256": initial_sha,
                   "policy": "sample p=0.5; joint BT.601 grayscale blend U[0,1), brightness U[0.35,1); uint8 nearest-even; labels unchanged",
                   "baseline_retrained": False, "model_promotion": False,
                   "dev_arrays_opened_before_checkpoint": False, "test_used": False}
        write_new(OUTPUT / "started.json", started)
        enabled_count = 0
        start = time.monotonic()
        model.train()
        with (OUTPUT / "history.jsonl").open("x", encoding="utf-8") as log:
            for step in range(1, STEPS + 1):
                ids = sampler.integers(0, len(rgb), size=BATCH_SIZE)
                pairs = [paired_crop(rgb[index], target[index], crop_rng) for index in ids]
                adjusted = [photometric(pair[0], photo_rng) for pair in pairs]
                enabled_count += sum(int(flag) for _, flag in adjusted)
                images = torch.tensor(np.stack([item[0] for item in adjusted])
                                      .transpose(0, 3, 1, 2).copy(),
                                      device="cuda:0", dtype=torch.float32) / 255
                labels = torch.tensor(np.stack([pair[1] for pair in pairs]).astype(np.int64),
                                      device="cuda:0")
                optimizer.zero_grad(set_to_none=True)
                loss = semantic_loss(model(images), labels, weights)
                require(bool(torch.isfinite(loss)), "nonfinite map loss")
                loss.backward()
                norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
                require(bool(torch.isfinite(norm)), "nonfinite map gradient")
                optimizer.step()
                if step % 100 == 0:
                    row = {"step": step, "loss": float(loss.detach()),
                           "wall_seconds": time.monotonic() - start,
                           "policy_enabled_samples": enabled_count}
                    log.write(json.dumps(row, allow_nan=False) + "\n")
                    log.flush()
                    print(json.dumps(row, allow_nan=False), flush=True)
        training_seconds = time.monotonic() - start
        with (OUTPUT / "checkpoint.pt").open("xb") as stream:
            torch.save({"model_state_dict": model.state_dict(), "labels": LABELS,
                        "model_id": "portable_e2e.map_semantics.photo_pilot_not_deployable",
                        "optimizer_state_dict": optimizer.state_dict(), "steps": STEPS,
                        "deployable": False, "research_contract": started}, stream)
            stream.flush()
            os.fsync(stream.fileno())
        checkpoint_sha = digest(OUTPUT / "checkpoint.pt")
        # HH_260929 - Open repeatedly seen DEV arrays only after checkpoint is sealed.
        manifest = read_json(DATA / "manifest.json")
        evaluated = {}
        for split, town in (("dev_town01", "town01"), ("dev_town05", "town05")):
            metrics, predictions = evaluate(model, split, manifest)
            pred_path = OUTPUT / f"{town}_predictions.npz"
            require(not pred_path.exists(), "fresh prediction archive required")
            np.savez_compressed(pred_path, prediction=predictions)
            evaluated[town] = metrics
        candidate = {"train_count": 729, "training_wall_seconds": training_seconds,
                     "policy_enabled_samples": enabled_count,
                     "policy_total_samples": STEPS * BATCH_SIZE,
                     "checkpoint_sha256": checkpoint_sha,
                     "town01": evaluated["town01"], "town05": evaluated["town05"],
                     "deployment_approved": False}
        write_new(OUTPUT / "metrics.json", candidate)
        baseline = read_json(BASELINE / "D1/metrics.json")
        gate = exploratory_gate(baseline, candidate)
        result = {"status": "ONE_CANDIDATE_COMPLETE_EXPLORATORY_NOT_PROMOTED",
                  "baseline": {"run": str(BASELINE), "file_sha256": BASELINE_SHA256,
                               "town01": baseline["town01"], "town05": baseline["town05"]},
                  "candidate": candidate, "gate": gate,
                  "file_sha256": {name: digest(OUTPUT / name) for name in
                                  ("checkpoint.pt", "history.jsonl", "started.json",
                                   "metrics.json", "town01_predictions.npz",
                                   "town05_predictions.npz")},
                  "independent_dev_count": 0, "test_used": False,
                  "model_promotion": False, "hd_map_complete": False,
                  "completed_at_utc": datetime.now(timezone.utc).isoformat()}
        write_new(OUTPUT / "summary.json", result)
        print(result["status"], flush=True)
        return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    if args.validate_only:
        contract = preflight()[0]
        write_new(SOURCE / "cpu_preflight_receipt.json", contract)
        print(json.dumps(contract, sort_keys=True, allow_nan=False), flush=True)
    else:
        run()


if __name__ == "__main__":
    main()
