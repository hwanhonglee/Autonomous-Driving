"""HH_261001 - Profile real GPU0 raw-spatial training without calling it complete model training."""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import time

import torch
from torch.utils.data.dataloader import default_collate

from portable_e2e.dataset import load_training_examples
from portable_e2e.temporal_dataset_v2 import (TemporalCommon10TorchDataset,
    load_common10_temporal_metadata, spatial_expert_labels)
from portable_e2e.temporal_losses_v2 import temporal_trajectory_loss_v2
from portable_e2e.temporal_model_v2 import TemporalModelV2, parameter_count


ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = Path("/home/pro6000/personal/hwanhong/portable_e2e")
PERSONAL = Path("/home/pro6000/personal/hwanhong")
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
FIELDS = ("images", "calibration", "ego_history", "ego_history_mask", "route_xy", "route_mask",
          "time_valid", "relative_pose", "time_offsets_s")
TRAIN_MANIFEST_SHA = "8c71781052d0a2f355dee35dacae21f8e630f197ab7f6d6fd1fe14ac0d442957"
DEV_MANIFEST_SHA = "70fceaf0a6fea1b6ac455154bdc7b1113cfef15abc1a26ca7327ac2a186799e6"


def require(value, message):
    if not value:
        raise ValueError(message)


def digest(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), "regular pinned file required: " + str(path))
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024**2), b""):
            value.update(block)
    return value.hexdigest()


def write_new(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def source_snapshot():
    # HH_261001 - New and legacy dependencies are byte-pinned, never rewritten to fit old checkpoints.
    paths = [*ROOT.glob("learning/portable_e2e/*.py"),
             *ROOT.glob("learning/portable_e2e/config/*.json"), Path(__file__),
             ROOT / "tools/workspace_python.py", ROOT / "tools/canonical_workspace.py"]
    return {str(path.relative_to(ROOT)): digest(path) for path in sorted(set(paths))}


def load_view(root, split, manifest_sha):
    root = root.resolve(strict=True)
    require(root.is_relative_to(PERSONAL.resolve()), "dataset escaped the personal folder")
    require(digest(root / "dataset.json") == manifest_sha, "existing dataset manifest differs")
    loaded = load_training_examples(root, split=split, mode="planning", check_image_hashes=True)
    metadata = load_common10_temporal_metadata(root, split=split,
                                              expected_tokens=[row.token for row in loaded.examples])
    dataset = TemporalCommon10TorchDataset(loaded.examples, raw_metadata=metadata,
                                          split=split, verify_image_sha256=True)
    return dataset, dict(path=str(root), split=split, manifest_sha256=manifest_sha,
        validated_fingerprint_sha256=loaded.fingerprint_sha256,
        temporal_fingerprint_sha256=dataset.fingerprint_sha256,
        sample_count=len(dataset), episode_count=len({row.episode_id for row in dataset.examples}))


def eligible_indices(dataset):
    # HH_261001 - This raw-spatial diagnostic excludes stationary preparation and empty spatial labels.
    # It does not filter to favorable model predictions or clip existing 30 km/h target values.
    return [index for index, row in enumerate(dataset.examples)
            if row.features[1] > .05 and int(spatial_expert_labels(row)["spatial_valid"].sum()) >= 2]


def direction(row):
    if row.features[7] > .5:
        return "left"
    if row.features[8] > .5:
        return "right"
    return "straight_or_lane_follow"


def fixed_probe(dataset, indices, maximum_per_direction=4):
    groups = {}
    for index in indices:
        groups.setdefault(direction(dataset.examples[index]), []).append(index)
    # HH_261001 - Predeclare uniformly spaced known-DEV probes before any neural predictions.
    selected = []
    for name in sorted(groups):
        values = groups[name]
        count = min(maximum_per_direction, len(values))
        selected.extend(values[round(i * (len(values)-1) / max(1, count-1))] for i in range(count))
    return selected


def move(batch, device):
    return {key: value.to(device) if isinstance(value, torch.Tensor) else value for key, value in batch.items()}


def forward(model, batch):
    return model(**{name: batch[name] for name in FIELDS}, history_mode="temporal")


def evaluate(model, dataset, indices, device):
    # HH_261001 - This is a small previously-used DEV raw-output diagnostic, not sealed TEST evaluation.
    model.eval()
    values = []
    with torch.no_grad():
        for index in indices:
            batch = move(default_collate([dataset[index]]), device)
            losses = temporal_trajectory_loss_v2(forward(model, batch), batch)
            values.append(dict(index=index, token=dataset.examples[index].token,
                raw_curvature=float(losses["raw_curvature"]),
                raw_preferred_speed=float(losses["raw_preferred_speed"]), total=float(losses["total"]),
                valid_spatial_knots=int(batch["spatial_valid"].sum())))
    require(values, "at least one known-DEV raw probe required")
    return dict(rows=values, mean_total=sum(row["total"] for row in values)/len(values),
                independent_validation=False, temporal_trajectory_evaluated=False)


@contextmanager
def gpu0_lease():
    require(Path(sys.prefix).resolve() == (WORKSPACE / "venvs/py312").resolve(), "existing personal venv required")
    require(ROOT.resolve().is_relative_to(WORKSPACE.resolve()), "source must remain in personal workspace")
    path = WORKSPACE / "runs/campaigns/.gpu0_training.lock"
    require(path.parent.is_dir() and not path.parent.is_symlink(), "existing safe GPU0 lease parent required")
    descriptor = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield
    finally:
        os.close(descriptor)


def gpu_preflight():
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == GPU_UUID, "only the physical GPU0 UUID may be visible")
    line = subprocess.check_output(["nvidia-smi", "-i", "0",
        "--query-gpu=index,uuid,memory.used,utilization.gpu", "--format=csv,noheader,nounits"],
        text=True, timeout=15).strip()
    index, uuid, memory, utilization = [field.strip() for field in line.split(",")]
    require(index == "0" and uuid == GPU_UUID and int(memory) == int(utilization) == 0, "physical GPU0 must be idle")
    require(torch.cuda.is_available() and torch.cuda.device_count() == 1, "exactly one approved CUDA GPU required")
    properties = torch.cuda.get_device_properties(0)
    require(str(properties.uuid) in (GPU_UUID, GPU_UUID.removeprefix("GPU-")), "Torch physical GPU UUID differs")
    return dict(physical_index=0, uuid=GPU_UUID, name=properties.name, idle_before_torch=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--train", type=Path, required=True)
    parser.add_argument("--known-dev", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args(argv)
    require(1 <= args.steps <= 200 and 1 <= args.batch_size <= 4, "bounded wiring profile only; main training needs a new plan")
    require(not args.output.exists() and not args.output.is_symlink(), "fresh profile output required")
    # HH_261001 - Resolve parent components so a symlink cannot redirect personal output outside the scope.
    require(args.output.resolve().is_relative_to((WORKSPACE / "runs/diagnostics").resolve()), "personal diagnostic output required")
    torch.set_num_threads(4)
    train, train_receipt = load_view(args.train, "train", TRAIN_MANIFEST_SHA)
    dev, dev_receipt = load_view(args.known_dev, "val", DEV_MANIFEST_SHA)
    require(train_receipt["sample_count"] == 2787 and dev_receipt["sample_count"] == 665, "fixed legacy view counts differ")
    require(not ({row.episode_id for row in train.examples} & {row.episode_id for row in dev.examples}), "episode split overlap")
    indices, dev_indices = eligible_indices(train), eligible_indices(dev)
    require(len(indices) >= args.batch_size, "insufficient raw-spatial TRAIN cohort")
    probes = fixed_probe(dev, dev_indices)
    sources = source_snapshot()
    declaration = dict(schema="portable_e2e.temporal_raw_bootstrap.20261001.v1", train=train_receipt,
        known_dev=dev_receipt, eligible_training_count=len(indices),
        eligible_episode_count=len({train.examples[i].episode_id for i in indices}),
        eligible_direction_counts=dict(Counter(direction(train.examples[i]) for i in indices)),
        cohort="observed current vx>.05 and at least two actually recorded spatial knots",
        steps=args.steps, batch_size=args.batch_size, seed=20261001, learning_rate=1e-4,
        optimizer="fresh AdamW", weight_decay=1e-4, gradient_clip=5.,
        objective="raw curvature + raw preferred speed only; intent/stop masked; no projected temporal loss",
        initialization="fresh random weights, not adapted W1", history_mode="temporal",
        known_dev_probe_indices=probes, source_sha256=sources,
        data_labels_clipped=False, new_data_approved=False, full_future_training=False,
        main_training=False, vehicle_control_approved=False, model_promoted=False)
    if args.validate_only:
        print(json.dumps(dict(status="VALIDATED_NO_TRAINING", **declaration), indent=2))
        return 0
    with gpu0_lease():
        gpu = gpu_preflight()
        require(os.environ.get("CUBLAS_WORKSPACE_CONFIG") == ":4096:8", "deterministic cuBLAS environment required")
        random.seed(20261001); torch.manual_seed(20261001); torch.cuda.manual_seed_all(20261001)
        torch.use_deterministic_algorithms(True)
        # HH_261001 - Use deterministic math attention in modern Torch, with no new dependency.
        if hasattr(torch.backends.cuda, "enable_flash_sdp"):
            torch.backends.cuda.enable_flash_sdp(False); torch.backends.cuda.enable_mem_efficient_sdp(False)
        args.output.mkdir(parents=True, exist_ok=False)
        write_new(args.output / "declaration.json", declaration)
        for name, expected in sources.items():
            require(digest(ROOT / name) == expected, "source changed before archive")
            target = args.output / "source" / name; target.parent.mkdir(parents=True, exist_ok=True)
            with target.open("xb") as stream:
                stream.write((ROOT / name).read_bytes())
        device = torch.device("cuda:0")
        model = TemporalModelV2().to(device)
        optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4, weight_decay=1e-4)
        write_new(args.output / "started.json", dict(gpu=gpu, pid=os.getpid(),
            started_at_utc=datetime.now(timezone.utc).isoformat(), parameters=parameter_count(model)))
        before = evaluate(model, dev, probes, device)
        write_new(args.output / "known_dev_raw_before.json", before)
        order = list(indices); random.shuffle(order)
        cursor, seen, unique, step_times = 0, 0, set(), []
        started = time.monotonic()
        with (args.output / "history.jsonl").open("x") as journal:
            for step in range(1, args.steps+1):
                tick = time.monotonic()
                if cursor+args.batch_size > len(order):
                    random.shuffle(order); cursor = 0
                chosen = order[cursor:cursor+args.batch_size]; cursor += args.batch_size
                batch = move(default_collate([train[index] for index in chosen]), device)
                model.train(); optimizer.zero_grad(set_to_none=True)
                losses = temporal_trajectory_loss_v2(forward(model, batch), batch)
                losses["total"].backward()
                norm = torch.nn.utils.clip_grad_norm_(model.parameters(), 5.)
                require(bool(torch.isfinite(norm)), "nonfinite gradient")
                optimizer.step()
                require(all(bool(torch.isfinite(value).all()) for value in model.state_dict().values()), "nonfinite updated parameters")
                torch.cuda.synchronize()
                elapsed = time.monotonic()-tick
                step_times.append(elapsed); seen += len(chosen); unique.update(chosen)
                row = dict(step=step, optimizer_updates=step, examples_seen=seen, indices=chosen,
                    loss=float(losses["total"]), raw_curvature=float(losses["raw_curvature"]),
                    raw_preferred_speed=float(losses["raw_preferred_speed"]), gradient_norm=float(norm),
                    step_wall_seconds=elapsed, elapsed_fit_seconds=time.monotonic()-started)
                journal.write(json.dumps(row, allow_nan=False)+"\n"); journal.flush()
                if step == 1 or step % 25 == 0:
                    print(json.dumps(row), flush=True)
        after = evaluate(model, dev, probes, device)
        write_new(args.output / "known_dev_raw_after.json", after)
        checkpoint = args.output / "raw_geometry_speed_checkpoint.pt"
        state = {name: value.detach().cpu() for name, value in model.state_dict().items()}
        with checkpoint.open("xb") as stream:
            torch.save(dict(schema=declaration["schema"], model_config=model.config.to_dict(),
                model_state_dict=state, optimizer_updates=args.steps, raw_spatial_only=True,
                stop_intent_trained=False, decoder_integrated=False, vehicle_control_approved=False,
                source_sha256=sources, training_dataset_fingerprint_sha256=train.fingerprint_sha256), stream)
        require(all(digest(ROOT / name) == expected for name, expected in sources.items()), "source changed during profile")
        summary = dict(status="RAW_SPATIAL_BOOTSTRAP_COMPLETE_NOT_PROMOTED", optimizer_updates=args.steps,
            examples_seen=seen, unique_training_anchors=len(unique),
            unique_training_episodes=len({train.examples[i].episode_id for i in unique}),
            fit_wall_seconds=sum(step_times), mean_step_seconds=sum(step_times)/len(step_times),
            nearest_rank_p95_step_seconds=sorted(step_times)[math.ceil(.95*len(step_times))-1],
            peak_GPU_allocated_bytes=torch.cuda.max_memory_allocated(),
            parameters=parameter_count(model), known_dev_probe_count=len(probes),
            known_dev_before=before["mean_total"], known_dev_after=after["mean_total"],
            checkpoint_sha256=digest(checkpoint), source_hashes_match=True,
            sensor_to_actuator_latency_measured=False, temporal_trajectory_evaluated=False,
            model_promoted=False, vehicle_control_approved=False, main_training=False)
        write_new(args.output / "summary.json", summary)
        print(json.dumps(summary), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
