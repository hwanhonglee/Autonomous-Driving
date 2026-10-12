#!/usr/bin/env python3
"""HH_260929 - Run one frozen TRAIN-route B/W2 pair on personal physical GPU0."""

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
SOURCE_NAME = "map_lane_weight_route_cv_20260929_v1"
SOURCE = WORKSPACE / "source_worktrees" / SOURCE_NAME
DATA = WORKSPACE.parent / "dataset/map_road_lane_ab_20260929_v1"
OUTPUT = WORKSPACE / "runs/diagnostics" / SOURCE_NAME
GPU0_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
DATA_SHA256 = "313b41ed4fb897f01fb4fb433c1bafb234ef9b234e62f96a462ef2188e9dccc6"
D0_SHA256 = "69dae433616cf27690310af9ff1923ef3b2e67641fa935e2a2f77116eea02ace"
D1_SHA256 = "4e2f7df61814a7e8aade88877482506a30e41c9fff0b5e6b7c3182193ff7d8f8"
PARENT_SHA256 = "e9805574a252e72f8e81fcd434156da7782b25e8e965b8078f976cf4f234892c"
AUDIT_SHA256 = "9dd002dee9ae4829724a2b073163b22c35cb97c25453652d384374f56a065375"
PLAN_SHA256 = "247e239777eb1159ad97aa5fb643585c8c71b32808dc91e307e594963c9abcdf"
SUPPORT_SHA256 = {
    "map_perception.py": "1b0076c8f9d34192ab5aedf6cdf96ce2c14458e2bb49986685a6811376b0750b",
    "map_learning_balance.py": "57c9392d8b700f5cf628865042b66ae630961f35fb8858a5d0ead77fe5a7c73b",
    "dual_recovery_objectives.py": "d10bcad9060c1f4bde40b20773fd4c18f1d4b6ba3861dd7cef55cd36cff4ad15",
    "parent_manifest.json": PARENT_SHA256,
    "audit.md": AUDIT_SHA256,
    "plan.md": PLAN_SHA256,
}
LABELS = ("other", "road", "sidewalk", "lane_marking",
          "traffic_light_fixture", "traffic_sign", "pole")
FOLDS = (("historical", 0, 114), ("p03", 114, 301), ("p01", 301, 497))
SEEDS = (20260922, 20260923, 20260924)
STEPS = 2000
BATCH_SIZE = 8
LEARNING_RATE = 3e-4
WEIGHT_DECAY = 1e-4
LANE_MULTIPLIER = 2.0
LANE_GAIN_MINIMUM = 0.05
ROAD_DROP_MINIMUM = -0.01


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def digest(path: Path) -> str:
    """HH_260929 - Bind regular source and TRAIN bytes before any GPU lease."""
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_new(path: Path, value: dict) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def copy_new(source: Path, target: Path) -> None:
    with source.open("rb") as original, target.open("xb") as archived:
        shutil.copyfileobj(original, archived, 1024 * 1024)
        archived.flush()
        os.fsync(archived.fileno())
    require(digest(target) == digest(source), "source archive bytes changed")


def pair_contract(pair_index: int) -> dict:
    """HH_260929 - Fix fold-major order before inspecting heldout predictions."""
    require(0 <= pair_index < 9, "pair index must be 0..8")
    fold_name, begin, end = FOLDS[pair_index // 3]
    return {"pair_index": pair_index, "fold": fold_name,
            "holdout_start": begin, "holdout_end": end,
            "seed": SEEDS[pair_index % 3]}


def fold_indices(pair_index: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """HH_260929 - No heldout Town01 index enters sampling or class weighting."""
    pair = pair_contract(pair_index)
    begin, end = pair["holdout_start"], pair["holdout_end"]
    kept_town01 = np.r_[0:begin, end:497].astype(np.int64)
    train = np.r_[kept_town01, 497:729].astype(np.int64)
    holdout = np.arange(begin, end, dtype=np.int64)
    require(len(train) + len(holdout) == 729
            and len(np.unique(train)) == len(train)
            and not np.intersect1d(train, holdout).size
            and np.array_equal(train[-232:], np.arange(497, 729)),
            "TRAIN/holdout fold partition changed")
    return train, holdout, kept_town01


def lane_weights(base: np.ndarray) -> np.ndarray:
    """HH_260929 - Change exactly one weighted-CE coefficient."""
    require(base.shape == (7,) and np.isfinite(base).all() and np.all(base > 0),
            "seven positive class weights required")
    changed = base.copy()
    changed[3] *= LANE_MULTIPLIER
    require(np.array_equal(changed[[0, 1, 2, 4, 5, 6]],
                           base[[0, 1, 2, 4, 5, 6]]), "non-lane class weight changed")
    return changed


def gate(baseline: dict, candidate: dict) -> dict:
    """HH_260929 - Stop the sequence on the first failed preregistered pair."""
    base, trial = baseline["iou"], candidate["iou"]
    require(all(base[name] is not None and trial[name] is not None
                for name in ("lane_marking", "road")),
            "road and lane must be present in heldout route")
    lane_delta = trial["lane_marking"] - base["lane_marking"]
    road_delta = trial["road"] - base["road"]
    return {"lane_iou_delta": lane_delta, "road_iou_delta": road_delta,
            "lane_plus_5pp": lane_delta >= LANE_GAIN_MINIMUM,
            "road_drop_at_most_1pp": road_delta >= ROAD_DROP_MINIMUM,
            "passed": lane_delta >= LANE_GAIN_MINIMUM and road_delta >= ROAD_DROP_MINIMUM}


def validate_environment() -> None:
    """HH_260929 - Keep execution inside the existing personal venv and source tree."""
    require(os.uname().nodename == "pro6000", "personal training host required")
    require(Path(sys.prefix).absolute() == WORKSPACE / "venvs/py312",
            "personal py312 venv required")
    require(Path.cwd().resolve() == SOURCE.resolve()
            and Path(__file__).resolve().parent == SOURCE.resolve(),
            "frozen personal source working directory required")
    required = {"CUDA_VISIBLE_DEVICES": GPU0_UUID,
                "CUBLAS_WORKSPACE_CONFIG": ":4096:8", "PYTHONNOUSERSITE": "1",
                "OMP_NUM_THREADS": "16", "MKL_NUM_THREADS": "16"}
    require(all(os.environ.get(name) == value for name, value in required.items()),
            "deterministic physical GPU0 environment differs")
    require(not any(os.environ.get(name) for name in
                    ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP", "PYTHONUSERBASE")),
            "inherited Python path overrides forbidden")
    for path in (WORKSPACE, SOURCE, DATA, OUTPUT.parent):
        require(path.is_dir() and not path.is_symlink(), f"personal directory missing: {path}")


def load_train(path: Path, expected_sha: str, count: int) -> tuple[np.ndarray, np.ndarray]:
    require(digest(path) == expected_sha, "TRAIN archive hash changed")
    with np.load(path, allow_pickle=False) as archive:
        require(set(archive.files) == {"rgb", "target"}, "TRAIN archive keys changed")
        rgb, target = archive["rgb"], archive["target"]
    require(rgb.shape == (count, 360, 640, 3)
            and target.shape == (count, 360, 640)
            and rgb.dtype == target.dtype == np.uint8
            and int(target.max()) < 7, "TRAIN array ABI changed")
    return rgb, target


def validate_previous(pair_index: int, runner_sha: str) -> None:
    """HH_260929 - A failed or incomplete prior pair forbids any later pair."""
    if pair_index == 0:
        require(not OUTPUT.exists() and not OUTPUT.is_symlink(), "first output must be fresh")
        return
    require(OUTPUT.is_dir() and not OUTPUT.is_symlink(), "prior output root absent")
    for earlier in range(pair_index):
        pair = pair_contract(earlier)
        directory = OUTPUT / f"pair_{earlier:02d}_{pair['fold']}_{pair['seed']}"
        summary_path = directory / "summary.json"
        require(summary_path.is_file() and not summary_path.is_symlink(),
                "prior pair summary missing")
        summary = read_json(summary_path)
        require(summary.get("pair") == pair and summary.get("status") == "PAIR_PASS"
                and summary.get("gate", {}).get("passed") is True
                and summary.get("runner_sha256") == runner_sha
                and summary.get("data_manifest_sha256") == DATA_SHA256
                and summary.get("model_promotion") is False,
                "prior pair failed or source contract changed")
        for file, expected in summary["file_sha256"].items():
            require(digest(directory / file) == expected,
                    f"prior pair output changed: {file}")
    require(not (OUTPUT / f"pair_{pair_index:02d}_{pair_contract(pair_index)['fold']}_"
                 f"{pair_contract(pair_index)['seed']}").exists(),
            "next pair output already exists")


def preflight(pair_index: int) -> tuple[dict, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """HH_260929 - Load only the two pinned TRAIN arrays before any GPU action."""
    validate_environment()
    source_sha = digest(Path(__file__))
    for name, expected in SUPPORT_SHA256.items():
        require(digest(SOURCE / name) == expected, f"frozen support changed: {name}")
    validate_previous(pair_index, source_sha)
    require(digest(DATA / "manifest.json") == DATA_SHA256, "package manifest changed")
    manifest = read_json(DATA / "manifest.json")
    parent = read_json(SOURCE / "parent_manifest.json")
    require(manifest.get("schema") == "portable_e2e.map_road_lane_ab_data.v1"
            and manifest.get("parent_manifest_sha256") == PARENT_SHA256
            and tuple(manifest.get("labels", ())) == tuple(LABELS)
            and manifest.get("town02_split") == "TRAIN_EXPLORATORY_ROAD_LANE_ONLY"
            and manifest.get("town05_split") == "DEV_ONLY"
            and manifest["split_contract"]["dev_used_for_training"] is False
            and manifest["split_contract"]["test_used"] is False,
            "TRAIN-only data scope changed")
    files = manifest["files"]
    require(set(files) == {"train_d0", "train_d1", "dev_town01", "dev_town05"}
            and files["train_d0"]["sha256"] == D0_SHA256
            and files["train_d1"]["sha256"] == D1_SHA256
            and files["train_d0"]["count"] == 497
            and files["train_d1"]["count"] == 729
            and files["dev_town01"]["sha256"] == parent["splits"]["dev"]["sha256"],
            "declared split identity changed")
    records = parent["splits"]["train"]["source_records"]
    require(len(records) == 497
            and all("source_collection" not in row for row in records[:114])
            and all("mapping_town01_p03" in row.get("source_collection", "")
                    for row in records[114:301])
            and all("mapping_town01_p01" in row.get("source_collection", "")
                    for row in records[301:497]),
            "Town01 route boundaries changed")
    d0_rgb, d0_target = load_train(DATA / "train_d0.npz", D0_SHA256, 497)
    d1_rgb, d1_target = load_train(DATA / "train_d1.npz", D1_SHA256, 729)
    require(np.array_equal(d1_rgb[:497], d0_rgb)
            and np.array_equal(d1_target[:497], d0_target), "D1 lost exact D0 prefix")
    for name, target in (("train_d0", d0_target), ("train_d1", d1_target)):
        require(np.bincount(target.ravel(), minlength=7).tolist()
                == files[name]["class_pixels"], f"{name} class pixels changed")
    train, holdout, weight_ids = fold_indices(pair_index)
    sys.path.insert(0, str(SOURCE))
    from map_learning_balance import class_weights
    base = class_weights(np.bincount(d1_target[weight_ids].ravel(), minlength=7),
                         "sqrt_frequency")
    changed = lane_weights(base)
    require(np.array_equal(d1_target[holdout], d0_target[holdout])
            and np.array_equal(d1_rgb[holdout], d0_rgb[holdout]),
            "heldout route no longer matches Town01 D0")
    contract = {"status": "CPU_PREFLIGHT_PASS", "pair": pair_contract(pair_index),
                "train_frames": len(train), "heldout_frames": len(holdout),
                "weight_source_frames": len(weight_ids),
                "train_index_sha256": hashlib.sha256(train.tobytes()).hexdigest(),
                "holdout_index_sha256": hashlib.sha256(holdout.tobytes()).hexdigest(),
                "base_weights": base.tolist(), "candidate_weights": changed.tolist(),
                "runner_sha256": source_sha, "data_manifest_sha256": DATA_SHA256,
                "plan_sha256": PLAN_SHA256, "audit_sha256": AUDIT_SHA256,
                "dev_arrays_opened": False, "test_used": False}
    return contract, d1_rgb, d1_target, train, holdout, changed


@contextmanager
def gpu0_lease():
    """HH_260929 - Cooperate with all prior personal training through one lock."""
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
    """HH_260929 - Require one idle physical GPU0 and no GPU1 fallback."""
    rows = subprocess.check_output(
        ["nvidia-smi", "--query-gpu=index,uuid,memory.used,utilization.gpu",
         "--format=csv,noheader,nounits"], timeout=20, text=True).splitlines()
    matches = [[field.strip() for field in row.split(",")]
               for row in rows if GPU0_UUID in row]
    require(len(matches) == 1 and matches[0][0] == "0"
            and matches[0][1] == GPU0_UUID
            and int(matches[0][2]) == 0 and int(matches[0][3]) == 0,
            "approved physical GPU0 is absent or occupied")
    require(torch.cuda.is_available() and torch.cuda.device_count() == 1,
            "exactly one CUDA device must be visible")
    properties = torch.cuda.get_device_properties(torch.device("cuda:0"))
    require(str(getattr(properties, "uuid", "")) in
            (GPU0_UUID, GPU0_UUID.removeprefix("GPU-")),
            "PyTorch visible GPU UUID differs")
    return {"physical_index": 0, "uuid": GPU0_UUID,
            "memory_used_mib_before_torch": 0, "utilization_percent_before_torch": 0,
            "name": properties.name}


def seed_everything(seed: int) -> None:
    """HH_260929 - Match the prior deterministic scratch initialization policy."""
    random.seed(seed)
    np.random.seed(seed % (2**32))
    torch.random.default_generator.manual_seed(seed)
    with torch.cuda.device(torch.device("cuda:0")):
        torch.cuda.manual_seed(seed)
    torch.use_deterministic_algorithms(True)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True


def state_digest(model: MapPerception) -> str:
    sha = hashlib.sha256()
    for name, tensor in sorted(model.state_dict().items()):
        value = tensor.detach().cpu().contiguous()
        header = json.dumps([name, str(value.dtype), list(value.shape)]).encode("utf-8")
        sha.update(len(header).to_bytes(8, "big"))
        sha.update(header)
        sha.update(value.numpy().tobytes())
    return sha.hexdigest()


def metrics(confusion: np.ndarray) -> dict:
    """HH_260929 - Publish every heldout semantic class, including absent classes."""
    intersection = np.diag(confusion)
    union = confusion.sum(0) + confusion.sum(1) - intersection
    return {"confusion": confusion.tolist(),
            "iou": {name: float(intersection[i] / union[i]) if union[i] else None
                    for i, name in enumerate(LABELS)},
            "pixel_accuracy": float(intersection.sum() / confusion.sum())}


def evaluate(model: MapPerception, rgb: np.ndarray, target: np.ndarray,
             holdout: np.ndarray) -> tuple[dict, np.ndarray]:
    confusion = np.zeros((7, 7), dtype=np.int64)
    predictions = []
    model.eval()
    with torch.no_grad():
        for start in range(0, len(holdout), BATCH_SIZE):
            ids = holdout[start:start + BATCH_SIZE]
            images = torch.tensor(rgb[ids].transpose(0, 3, 1, 2).copy(),
                                  device="cuda:0", dtype=torch.float32) / 255
            prediction = model(images).argmax(1).cpu().numpy().astype(np.uint8)
            predictions.append(prediction)
            encoded = target[ids].astype(np.int64) * 7 + prediction
            confusion += np.bincount(encoded.ravel(), minlength=49).reshape(7, 7)
    return metrics(confusion), np.concatenate(predictions)


def run_arm(arm: str, contract: dict, rgb: np.ndarray, target: np.ndarray,
            train: np.ndarray, holdout: np.ndarray, weights_np: np.ndarray,
            directory: Path) -> dict:
    """HH_260929 - Recreate sampling, crop, and initial model state for each paired arm."""
    from dual_recovery_objectives import paired_crop
    from map_perception import MapPerception, LABELS as model_labels, semantic_loss
    require(tuple(model_labels) == LABELS, "frozen model label order changed")
    seed_everything(contract["pair"]["seed"])
    model = MapPerception().to(torch.device("cuda:0"))
    initial_sha = state_digest(model)
    optimizer = torch.optim.AdamW(model.parameters(), lr=LEARNING_RATE,
                                  weight_decay=WEIGHT_DECAY)
    sampler = np.random.default_rng(contract["pair"]["seed"])
    augmentation = np.random.default_rng(20260926)
    weights = torch.tensor(weights_np, device="cuda:0", dtype=torch.float32)
    model.train()
    start_time = time.monotonic()
    history_path = directory / f"{arm}_history.jsonl"
    with history_path.open("x", encoding="utf-8") as log:
        for step in range(1, STEPS + 1):
            batch_ids = train[sampler.integers(0, len(train), size=BATCH_SIZE)]
            pairs = [paired_crop(rgb[index], target[index], augmentation)
                     for index in batch_ids]
            images = torch.tensor(np.stack([item[0] for item in pairs])
                                  .transpose(0, 3, 1, 2).copy(),
                                  device="cuda:0", dtype=torch.float32) / 255
            labels = torch.tensor(np.stack([item[1] for item in pairs]).astype(np.int64),
                                  device="cuda:0")
            optimizer.zero_grad(set_to_none=True)
            loss = semantic_loss(model(images), labels, weights)
            require(bool(torch.isfinite(loss)), "nonfinite map loss")
            loss.backward()
            norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            require(bool(torch.isfinite(norm)), "nonfinite map gradient")
            optimizer.step()
            if step % 100 == 0:
                row = {"arm": arm, "step": step, "loss": float(loss.detach()),
                       "wall_seconds": time.monotonic() - start_time}
                log.write(json.dumps(row, sort_keys=True) + "\n")
                log.flush()
                print(json.dumps({"pair": contract["pair"]["pair_index"], **row}), flush=True)
    training_seconds = time.monotonic() - start_time
    checkpoint_path = directory / f"{arm}_checkpoint.pt"
    with checkpoint_path.open("xb") as stream:
        torch.save({"model_state_dict": model.state_dict(),
                    "optimizer_state_dict": optimizer.state_dict(),
                    "labels": tuple(LABELS), "steps": STEPS,
                    "seed": contract["pair"]["seed"], "fold": contract["pair"]["fold"],
                    "deployable": False, "research_contract": contract}, stream)
    result, prediction = evaluate(model, rgb, target, holdout)
    prediction_path = directory / f"{arm}_holdout_predictions.npz"
    with prediction_path.open("xb") as stream:
        np.savez_compressed(stream, prediction=prediction)
    result.update({"arm": arm, "initial_state_sha256": initial_sha,
                   "checkpoint_sha256": digest(checkpoint_path),
                   "prediction_sha256": digest(prediction_path),
                   "training_wall_seconds": training_seconds,
                   "train_frames": len(train), "holdout_frames": len(holdout)})
    write_new(directory / f"{arm}_metrics.json", result)
    return result


def run(pair_index: int) -> dict:
    contract, rgb, target, train, holdout, candidate_weights = preflight(pair_index)
    base_weights = np.asarray(contract["base_weights"])
    with gpu0_lease():
        gpu = preflight_gpu()
        if pair_index == 0:
            OUTPUT.mkdir(exist_ok=False)
            snapshot = OUTPUT / "source"
            snapshot.mkdir(exist_ok=False)
            for name in (Path(__file__).name, *SUPPORT_SHA256):
                copy_new(SOURCE / name, snapshot / name)
        pair = contract["pair"]
        directory = OUTPUT / f"pair_{pair_index:02d}_{pair['fold']}_{pair['seed']}"
        directory.mkdir(exist_ok=False)
        started = {**contract, "schema": "portable_e2e.map_lane_weight_route_pair.v1",
                   "started_at_utc": datetime.now(timezone.utc).isoformat(),
                   "gpu": gpu, "steps_per_arm": STEPS, "batch_size": BATCH_SIZE,
                   "learning_rate": LEARNING_RATE, "weight_decay": WEIGHT_DECAY,
                   "augmentation_seed": 20260926, "model_promotion": False}
        write_new(directory / "started.json", started)
        baseline = run_arm("B", contract, rgb, target, train, holdout,
                           base_weights, directory)
        candidate = run_arm("W2", contract, rgb, target, train, holdout,
                            candidate_weights, directory)
        require(baseline["initial_state_sha256"] == candidate["initial_state_sha256"],
                "paired scratch initial state differs")
        decision = gate(baseline, candidate)
        files = ("started.json", "B_history.jsonl", "W2_history.jsonl",
                 "B_checkpoint.pt", "W2_checkpoint.pt",
                 "B_holdout_predictions.npz", "W2_holdout_predictions.npz",
                 "B_metrics.json", "W2_metrics.json")
        summary = {"schema": "portable_e2e.map_lane_weight_route_pair.v1",
                   "status": "PAIR_PASS" if decision["passed"] else "PAIR_FAIL_STOP",
                   "pair": pair, "gate": decision,
                   "baseline_iou": baseline["iou"], "candidate_iou": candidate["iou"],
                   "initial_state_sha256": baseline["initial_state_sha256"],
                   "runner_sha256": contract["runner_sha256"],
                   "data_manifest_sha256": DATA_SHA256,
                   "file_sha256": {file: digest(directory / file) for file in files},
                   "next_pair_authorized": bool(decision["passed"] and pair_index < 8),
                   "all_nine_passed": bool(decision["passed"] and pair_index == 8),
                   "model_promotion": False, "test_used": False,
                   "deployment_approved": False,
                   "completed_at_utc": datetime.now(timezone.utc).isoformat()}
        write_new(directory / "summary.json", summary)
        print(json.dumps({"status": summary["status"], "pair": pair,
                          "gate": decision, "output": str(directory)}), flush=True)
        return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--pair-index", type=int, required=True)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()
    if args.validate_only:
        contract, *_ = preflight(args.pair_index)
        print(json.dumps(contract, sort_keys=True), flush=True)
    else:
        run(args.pair_index)


if __name__ == "__main__":
    main()
