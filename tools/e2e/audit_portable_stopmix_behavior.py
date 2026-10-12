#!/usr/bin/env python3
"""HH_260906 - Diagnose target-motion subgroups on the complete held-out validation split, never vehicle control."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys

import torch
from torch.utils.data import DataLoader

from portable_e2e.audit_runtime import (
    _AuditAccumulator, _atomic_new_json, _read_checkpoint_for_audit,
    _validate_checkpoint_and_model, audit_prediction,
)
from portable_e2e.contract import ContractError
from portable_e2e.dataset import FEATURE_NAMES, load_training_examples
from portable_e2e.model import ModelConfig, PHYSICAL_MODEL_ID, PHYSICAL_STOPMIX_MODEL_ID
from portable_e2e.runtime import _select_runtime_device
from portable_e2e.runtime_contract import RuntimeGateConfig, RUNTIME_GATE_ID
from portable_e2e.torch_dataset import Common10TorchDataset
from portable_e2e.train import _seed_everything
from portable_e2e.visualize import render_trajectory_png


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = "portable_e2e.stopmix_behavior_audit.v1"
DATASET_SHA256 = "18262e5aa4abbb3e03e35e379b5da1e5ce7fd339a9a8942e02b58ca737f7242c"
GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
EXPECTED_SAMPLES = 337
RENDER_INDICES = (0, 31, 61, 92, 122, 153, 183, 214, 244, 275, 305, 336)
GROUPS = ("unavailable_masks", "stationary_hold", "moving_to_stop", "continuing_motion", "other_motion")
CONFIGS = {
    PHYSICAL_MODEL_ID: "portable_e2e/config/perspective_trajectory_physical_v1.model.json",
    PHYSICAL_STOPMIX_MODEL_ID: "portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json",
}
SOURCE_PATHS = (
    "scripts/e2e/audit_portable_stopmix_behavior.py", "portable_e2e/model.py",
    "portable_e2e/stop_primitive_research.py", "portable_e2e/contract.py",
    "portable_e2e/dataset.py", "portable_e2e/torch_dataset.py", "portable_e2e/train.py",
    "portable_e2e/evaluate.py", "portable_e2e/losses.py", "portable_e2e/audit_runtime.py",
    "portable_e2e/runtime.py", "portable_e2e/runtime_contract.py", "portable_e2e/visualize.py",
    *CONFIGS.values(),
)
INPUT_KEYS = ("images", "calibration", "ego_history", "ego_history_mask", "route_xy", "route_mask")


def require(condition, message):
    if not condition:
        raise ContractError(message)


def digest(path):
    path = Path(path)
    require(path.is_file() and not path.is_symlink(), "input must be a regular nonsymlink file")
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def utc():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def source_identity(commit):
    require(isinstance(commit, str) and re.fullmatch(r"[0-9a-f]{40}", commit), "source commit must be full lowercase SHA1")
    environment = dict(os.environ, GIT_NO_LAZY_FETCH="1", GIT_ALLOW_PROTOCOL="", GIT_TERMINAL_PROMPT="0")
    def git(*arguments):
        return subprocess.run(["git", "-c", "protocol.allow=never", *arguments], cwd=ROOT,
            env=environment, capture_output=True, check=True, timeout=15).stdout
    require(git("rev-parse", "HEAD").decode().strip() == commit, "source HEAD differs from the declared commit")
    hashes = {}
    for name in SOURCE_PATHS:
        if name.startswith("portable_e2e/") and name.endswith(".py"):
            module = sys.modules.get(name[:-3].replace("/", "."))
            require(module is not None and Path(module.__file__).resolve() == ROOT / name,
                    "loaded Python module must come from the declared repository")
        hashes[name] = digest(ROOT / name)
        require(hashlib.sha256(git("show", f"{commit}:{name}")).hexdigest() == hashes[name],
                "executed source differs from declared Git bytes")
    return hashes


def checked_output(output, dataset, checkpoint):
    lexical = Path(output).absolute()
    require(not any(p.is_symlink() for p in (lexical, *lexical.parents)), "output symlinks are forbidden")
    resolved = lexical.resolve()
    forbidden = (Path(dataset).absolute(), Path(dataset).resolve(), ROOT / "datasets", (ROOT / "datasets").resolve(),
                 Path(checkpoint).absolute().parent, Path(checkpoint).resolve().parent)
    require(not any(resolved == p or p in resolved.parents for p in forbidden), "output must be outside dataset and checkpoint input trees")
    require(not lexical.exists(), "output must be a fresh directory")
    return lexical


def checked_targets(current_vx, target_speed, valid):
    require(type(current_vx) in (int, float) and math.isfinite(current_vx), "current raw vx must be finite")
    require(len(target_speed) == len(valid) == 64 and all(type(x) is bool for x in valid), "target mask must contain 64 booleans")
    require(any(valid) and valid == sorted(valid, reverse=True), "target mask must be a nonempty prefix")
    require(all(type(v) in (int, float) and math.isfinite(v) and v >= 0
                for v, ok in zip(target_speed, valid) if ok), "valid target speeds must be finite nonnegative numbers")


def motion_group(current_vx, target_speed, valid):
    # HH_260906 - Use canonical pre-cast numbers for fixed diagnostic thresholds, not rounded float32 comparisons.
    checked_targets(current_vx, target_speed, valid)
    if not all(valid):
        return "unavailable_masks"
    if abs(current_vx) <= .1 and all(v <= .1 for v in target_speed):
        return "stationary_hold"
    if current_vx > .1 and all(v <= .1 for v in target_speed[-10:]):
        return "moving_to_stop"
    if all(v >= .5 for v in target_speed):
        return "continuing_motion"
    return "other_motion"


def speed_behavior(speed):
    require(len(speed) == 64 and all(type(v) in (int, float) and math.isfinite(v) for v in speed),
            "predicted speed must contain 64 finite numbers")
    first_zero = next((i for i, v in enumerate(speed) if v == 0.0), None)
    # HH_260906 - The current anchor is excluded: an ordinary launch from rest is not future reacceleration.
    return {"terminal_speed_mps": speed[-1], "terminal_exact_zero": speed[-1] == 0.0,
        "terminal_at_or_below_0p1_mps": speed[-1] <= .1,
        "first_exact_zero_future_index": first_zero,
        "reacceleration_after_exact_future_zero": first_zero is not None and any(v > 0 for v in speed[first_zero + 1:]),
        "nonincreasing_speed_exact": all(b <= a for a, b in zip(speed, speed[1:]))}


def analyze_sample(*, index, example, xy, speed, logits, target_xy, target_speed, valid, model_id):
    require(model_id in CONFIGS, "only the declared physical or stopmix research models are supported")
    count = 12 if model_id == PHYSICAL_STOPMIX_MODEL_ID else 6
    require(len(xy) == len(speed) == len(logits) == count, "candidate count differs from the explicit model contract")
    require(all(type(v) in (int, float) and math.isfinite(v) for v in logits), "candidate logits must be finite")
    require(len(target_xy) == 64 and all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in target_xy),
            "target XY must contain 64 finite planar points")
    raw_vx = example.features[FEATURE_NAMES.index("velocity_x_mps")]
    raw_valid = [p is not None for p in example.targets_xy]
    require(valid == raw_valid, "tensor target mask differs from canonical raw targets")
    group = motion_group(raw_vx, list(example.target_speed_mps), valid)
    checked_targets(raw_vx, target_speed, valid)
    selected = max(range(count), key=logits.__getitem__)
    last = sum(valid) - 1
    candidates = []
    for candidate_index in range(count):
        path, velocities = xy[candidate_index], speed[candidate_index]
        require(len(path) == 64 and all(len(p) == 2 and all(math.isfinite(v) for v in p) for p in path),
                "candidate XY must contain 64 finite planar points")
        distances = [math.hypot(p[0] - t[0], p[1] - t[1]) for p, t in zip(path[:last + 1], target_xy[:last + 1])]
        behavior = speed_behavior(velocities)
        candidates.append({"candidate_index": candidate_index,
            "family": "STOP" if count == 12 and candidate_index >= 6 else "DRIVE",
            "ade_m": math.fsum(distances) / (last + 1), "fde_m": distances[-1],
            "speed_mae_mps": math.fsum(abs(a - b) for a, b in zip(velocities[:last + 1], target_speed[:last + 1])) / (last + 1),
            **behavior})
    gate = audit_prediction(xy, speed, logits, RuntimeGateConfig(candidate_count=count, future_points=64), current_speed_mps=raw_vx)
    require(gate["selected_candidate_index"] == selected, "runtime and diagnostic selections disagree")
    chosen = candidates[selected]
    oracle = min(row["ade_m"] for row in candidates)
    row = {"index": index, "sample_id": example.token, "episode_id": example.episode_id,
        "sequence_index": example.sequence_index, "anchor_timestamp_ns": example.anchor_timestamp_ns,
        "camera_sha256": list(example.camera_sha256), "source_manifest_sha256": example.source_manifest_sha256,
        "target_motion_group": group, "valid_future_points": last + 1, "raw_current_vx_mps": raw_vx,
        "candidate_count": count, "selected_candidate_index": selected,
        "selected_stop_candidate": selected >= 6 if count == 12 else None,
        "selected_ade_m": chosen["ade_m"], "selected_fde_m": chosen["fde_m"],
        "selected_speed_mae_mps": chosen["speed_mae_mps"], "oracle_ade_m": oracle,
        "ade_selection_regret_m": chosen["ade_m"] - oracle,
        "selected_speed_behavior": {key: chosen[key] for key in speed_behavior(speed[selected])},
        "candidates": candidates, "runtime_geometry": gate}
    return row


def summarize_rows(rows, candidate_count):
    require(rows and [r["index"] for r in rows] == list(range(len(rows))), "sample rows must be complete and canonically ordered")
    require(len({r["sample_id"] for r in rows}) == len(rows), "duplicate sample IDs are forbidden")
    require(all(r["candidate_count"] == candidate_count and r["target_motion_group"] in GROUPS for r in rows),
            "row candidate/group contract differs")
    def aggregate(subset):
        if not subset:
            return {"sample_count": 0, "metrics": None, "geometry": None, "stop_selection_count": None}
        geometry = _AuditAccumulator(RuntimeGateConfig(candidate_count=candidate_count, future_points=64))
        for row in subset:
            geometry.add(row["runtime_geometry"])
        return {"sample_count": len(subset),
            "metrics": {key: math.fsum(row[key] for row in subset) / len(subset) for key in
                ("selected_ade_m", "selected_fde_m", "selected_speed_mae_mps", "oracle_ade_m", "ade_selection_regret_m")},
            "geometry": geometry.report(),
            "stop_selection_count": sum(row["selected_stop_candidate"] is True for row in subset) if candidate_count == 12 else None,
            "selected_terminal_exact_zero_count": sum(row["selected_speed_behavior"]["terminal_exact_zero"] for row in subset),
            "selected_terminal_at_or_below_0p1_count": sum(row["selected_speed_behavior"]["terminal_at_or_below_0p1_mps"] for row in subset),
            "selected_reacceleration_after_exact_future_zero_count": sum(row["selected_speed_behavior"]["reacceleration_after_exact_future_zero"] for row in subset)}
    return {"all_samples": aggregate(rows), "target_motion_groups": {
        group: aggregate([r for r in rows if r["target_motion_group"] == group]) for group in GROUPS}}


def audit(args):
    require(args.expected_model_id in CONFIGS, "unsupported expected model ID")
    require(re.fullmatch(r"[0-9a-f]{64}", args.checkpoint_sha256 or ""), "checkpoint SHA256 is required")
    require(type(args.batch_size) is int and 1 <= args.batch_size <= 32, "batch size must be in [1,32]")
    require(args.device in ("cpu", "cuda:0"), "only CPU or the assigned logical GPU0 is supported")
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == ("" if args.device == "cpu" else GPU_UUID),
            "explicit empty CPU visibility or exact assigned GPU0 UUID is required")
    output = checked_output(args.output_dir, args.dataset, args.checkpoint)
    sources = source_identity(args.expected_source_commit)
    require(digest(Path(args.dataset) / "dataset.json") == DATASET_SHA256, "only the frozen development-v3 corpus is supported")
    require(digest(args.checkpoint) == args.checkpoint_sha256, "checkpoint bytes differ from required SHA")
    loaded = load_training_examples(args.dataset, split="val", mode="planning", check_image_hashes=True)
    require(len(loaded.examples) == EXPECTED_SAMPLES and len({e.episode_id for e in loaded.examples}) == 1,
            "the complete 337-sample single-episode validation split is required")
    corpus = loaded.validation_report["dataset_fingerprint_sha256"]
    payload, config, train_ids, checkpoint_provenance = _read_checkpoint_for_audit(
        checkpoint_path=args.checkpoint, expected_checkpoint_sha256=args.checkpoint_sha256,
        corpus_fingerprint_sha256=corpus)
    expected = ModelConfig.from_mapping(json.loads((ROOT / CONFIGS[args.expected_model_id]).read_text()))
    require(config == expected and len(train_ids) == 3, "checkpoint model config or training episode count differs from the fixed study")
    dataset = Common10TorchDataset(loaded.examples, config, verify_image_sha256=True, split="val")
    device = _select_runtime_device(args.device)
    _seed_everything(0, device)
    model, provenance = _validate_checkpoint_and_model(dataset, payload=payload, model_config=config,
        training_episode_ids=train_ids, checkpoint_provenance=checkpoint_provenance, device=device)
    output.mkdir(parents=True, exist_ok=False)
    (output / "trajectories").mkdir()
    started = {"schema": SCHEMA, "status": "RUNNING", "started_at_utc": utc(), "source_commit": args.expected_source_commit,
        "source_sha256": sources, "dataset_manifest_sha256": DATASET_SHA256, "corpus_fingerprint_sha256": corpus,
        "dataset_fingerprint_sha256": dataset.fingerprint_sha256, "checkpoint_sha256": args.checkpoint_sha256,
        "model_id": config.model_id, "candidate_count": config.candidate_count, "evaluation_split": "val",
        "expected_sample_count": EXPECTED_SAMPLES, "fixed_render_indices": list(RENDER_INDICES),
        "device": str(device), "batch_size": args.batch_size,
        "runtime": {"torch_version": torch.__version__, "torch_threads": torch.get_num_threads()},
        "source_commit_scope": "Verified live analysis source only. Checkpoints do not embed a training source commit; the external campaign receipt must bind this checkpoint SHA to its training source/stage.",
        "test_inference_or_optimization_or_selection": False,
        "integrity_scope": "Full corpus contract/hash validation may read test files; model inputs and motion diagnostics use val only.",
        "model_forward_inputs": list(INPUT_KEYS), "vehicle_control_approved": False, "training_data_approved": False}
    _atomic_new_json(output / "started.json", started)
    rows, renders = [], []
    try:
        with (output / "samples.jsonl").open("x", encoding="utf-8") as stream, torch.no_grad():
            for batch in DataLoader(dataset, batch_size=args.batch_size, shuffle=False, num_workers=0, pin_memory=False):
                inputs = [batch[key].to(device) for key in INPUT_KEYS]
                predictions = model(*inputs)
                size = len(batch["sample_id"])
                expected_shapes = ((size, config.candidate_count, 64, 2), (size, config.candidate_count, 64), (size, config.candidate_count))
                require(len(predictions) == 3 and all(isinstance(t, torch.Tensor) and tuple(t.shape) == shape
                    for t, shape in zip(predictions, expected_shapes)), "model prediction tensor ABI differs")
                xy_rows, speed_rows, logits_rows = [t.detach().cpu().tolist() for t in predictions]
                for position in range(size):
                    index = len(rows)
                    require(index < len(dataset) and batch["sample_id"][position] == dataset.examples[index].token,
                            "validation loader order or sample identity changed")
                    target = batch["target_xy"][position].tolist()
                    valid = batch["target_valid"][position].tolist()
                    row = analyze_sample(index=index, example=dataset.examples[index], xy=xy_rows[position],
                        speed=speed_rows[position], logits=logits_rows[position], target_xy=target,
                        target_speed=batch["target_speed_mps"][position].tolist(), valid=valid, model_id=config.model_id)
                    stream.write(json.dumps(row, sort_keys=True, allow_nan=False) + "\n")
                    stream.flush()
                    rows.append(row)
                    if index in RENDER_INDICES:
                        name = f"trajectories/val_{index:03d}.png"
                        render_trajectory_png(output / name, route_xy=batch["route_xy"][position][batch["route_mask"][position]].tolist(),
                            target_xy=target, target_valid=valid, candidate_xy=xy_rows[position], candidate_logits=logits_rows[position],
                            title=f"OPEN-LOOP PREDICTIONS ONLY | val {index} | K={config.candidate_count} | {row['target_motion_group']}")
                        renders.append({"index": index, "sample_id": row["sample_id"], "anchor_timestamp_ns": row["anchor_timestamp_ns"],
                            "camera_sha256": row["camera_sha256"], "file": name, "sha256": digest(output / name)})
            os.fsync(stream.fileno())
        require(len(rows) == EXPECTED_SAMPLES and [r["index"] for r in renders] == list(RENDER_INDICES), "validation or fixed render denominator is incomplete")
        require(source_identity(args.expected_source_commit) == sources and digest(args.checkpoint) == args.checkpoint_sha256
                and digest(Path(args.dataset) / "dataset.json") == DATASET_SHA256, "source, checkpoint or dataset manifest changed during diagnosis")
        post = load_training_examples(args.dataset, split="val", mode="planning", check_image_hashes=True)
        require(post.fingerprint_sha256 == loaded.fingerprint_sha256 and post.validation_report == loaded.validation_report,
                "corpus integrity changed during diagnosis")
        require(source_identity(args.expected_source_commit) == sources and digest(args.checkpoint) == args.checkpoint_sha256
                and digest(Path(args.dataset) / "dataset.json") == DATASET_SHA256,
                "source or primary inputs changed during the final corpus integrity scan")
        report = {**started, "checkpoint_validation": dict(provenance), "status": "COMPLETE_NOT_PROMOTED", "ended_at_utc": utc(),
            "source_checkpoint_and_corpus_postcheck_pass": True,
            "group_rule_priority": list(GROUPS), "group_rules": {
                "unavailable_masks": "Not all 64 future points valid; at least one valid prefix point is required.",
                "stationary_hold": "abs(raw current vx)<=0.1 and all 64 canonical target speeds<=0.1 m/s.",
                "moving_to_stop": "raw current vx>0.1 and all last 10 canonical target speeds<=0.1 m/s.",
                "continuing_motion": "All 64 canonical target speeds>=0.5 m/s.", "other_motion": "Remaining fully valid windows."},
            "metric_scope": "Float64 scalar arithmetic on stored tensorized predictions/targets; small reduction differences from GPU evaluation are possible.",
            "limitations": ["Target-derived groups describe motion, not traffic-rule intent or stop reason.",
                "Zero-count groups provide no behavior evidence; overlapping windows are not independent stopping events.",
                "STOP family exists only at indices 6-11 for K12; K6 STOP selection is not applicable, not failure to stop.",
                "Exact future-zero reacceleration excludes the current anchor; terminal-low-speed is separate from exact zero.",
                "Raw current vx is passed unchanged to runtime v8; decoder speed clamping does not remove raw-input gate failures.",
                "Trajectory PNGs are open-loop predictions, not camera footage, closed-loop driving or safe control evidence."],
            "gate": {"source": RUNTIME_GATE_ID, "thresholds": asdict(RuntimeGateConfig(candidate_count=config.candidate_count, future_points=64)),
                     "threshold_overrides": False},
            **summarize_rows(rows, config.candidate_count), "renders": renders, "samples_sha256": digest(output / "samples.jsonl")}
        _atomic_new_json(output / "summary.json", report)
        paths = [output / "started.json", output / "samples.jsonl", output / "summary.json", *[output / r["file"] for r in renders]]
        with (output / "SHA256SUMS").open("x", encoding="utf-8") as stream:
            stream.write("".join(f"{digest(path)}  {path.relative_to(output).as_posix()}\n" for path in paths))
        return report
    except BaseException as error:
        # HH_260906 - Retain completed numeric rows and partial plots; never create a completed summary for an interrupted audit.
        _atomic_new_json(output / "failed.json", {**started, "status": "INCOMPLETE", "ended_at_utc": utc(),
            "completed_sample_count": len(rows), "completed_render_count": len(renders), "failure_type": type(error).__name__})
        raise


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--checkpoint-sha256", required=True)
    parser.add_argument("--expected-source-commit", required=True)
    parser.add_argument("--expected-model-id", choices=tuple(CONFIGS), required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--device", choices=("cpu", "cuda:0"), default="cpu")
    parser.add_argument("--batch-size", type=int, default=4)
    return parser.parse_args(argv)


def main(argv=None):
    try:
        result = audit(parse_args(argv))
    except (ContractError, OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        print(json.dumps({"status": "INCOMPLETE", "error_type": type(error).__name__}))
        return 1
    print(json.dumps({"status": result["status"], "sample_count": result["all_samples"]["sample_count"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
