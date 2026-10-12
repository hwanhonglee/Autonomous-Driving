#!/usr/bin/env python3
"""HH_260906 - Evaluate the sealed F checkpoint on the two fixed DEV episodes only."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace


SUMMARY_SCHEMA = "portable_e2e.moving_direction_balanced_replay_subgroup_evaluation.v1"
DENIALS = {"test_read_or_use": False, "training_executed_by_evaluator": False,
    "closed_loop_evaluation": False, "vehicle_control_approved": False,
    "automatic_promotion": False, "model_exported": False}


def require(value, message):
    if not value:
        raise ValueError(message)


def local_regular(path):
    # HH_260906 - Authenticate the external runner before importing any of its code.
    path = Path(path).absolute()
    require(".." not in path.parts and path.is_file() and path.stat().st_nlink == 1
            and not any(parent.is_symlink() for parent in (path, *path.parents))
            and path == path.resolve(), "regular canonical nonsymlink file required")
    return path


def load_module(path, expected, name, digest):
    path = Path(path).absolute(); require(path.is_file() and digest(path) == expected, "helper SHA differs")
    spec = importlib.util.spec_from_file_location(name, path); require(spec and spec.loader, "helper import spec failed")
    module = importlib.util.module_from_spec(spec); sys.modules[name] = module; spec.loader.exec_module(module)
    require(digest(path) == expected, "helper changed during import"); return module


def utc():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def write_new(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write((json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode())
        stream.flush(); os.fsync(stream.fileno())


def json_compatible(value):
    # HH_260906 - Compare checkpoint tuples with their immutable JSON-plan list representation.
    return json.loads(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False))


def run(args):
    runner_path = local_regular(args.runner)
    preliminary = __import__("hashlib").sha256(runner_path.read_bytes()).hexdigest()
    require(preliminary == args.runner_sha256, "runner SHA argument differs")
    runner = load_module(runner_path, args.runner_sha256, "_turn_replay_subgroup_runner", lambda path:
        __import__("hashlib").sha256(Path(path).read_bytes()).hexdigest())
    plan_path = runner.regular(args.plan); require(runner.digest(plan_path) == args.plan_sha256, "plan SHA differs")
    plan = runner.validate_plan(runner.read(plan_path), runner_sha256=args.runner_sha256)
    evaluator_path = runner.regular(__file__)
    require(runner.digest(evaluator_path) == plan["evaluator_sha256"], "evaluator SHA differs")
    repo = runner.canonical(args.repo_root); runner.verify_sources(plan, repo)
    require(os.environ.get("CUDA_VISIBLE_DEVICES") == plan["gpu"]["uuid"],
            "CUDA_VISIBLE_DEVICES must be the pinned physical GPU0 UUID")
    output = Path(args.output_dir).absolute()
    require(not output.exists() and output.parent.is_dir() and output.parent == output.parent.resolve()
            and not output.is_relative_to(repo), "fresh external subgroup output required")
    checkpoint = runner.regular(args.checkpoint); checkpoint_sha = runner.sha256(args.checkpoint_sha256)
    require(runner.digest(checkpoint) == checkpoint_sha, "checkpoint SHA differs")
    helper_path = repo / runner.LEGACY_EVALUATOR_PATH
    helper = load_module(helper_path, plan["source"]["added_sha256"][runner.LEGACY_EVALUATOR_PATH],
                         "_turn_replay_subgroup_primitives", runner.digest)
    data = plan["arm"]["dataset"]
    spec = helper.DatasetSpec(path=data["path"], manifest_sha256=data["manifest_sha256"],
        corpus_fingerprint_sha256=data["corpus_fingerprint_sha256"],
        train_fingerprint_sha256=data["split_fingerprint_sha256"]["train"],
        val_fingerprint_sha256=data["split_fingerprint_sha256"]["val"],
        split_examples_sha256=data["split_examples_sha256"], counts=data["counts"],
        episode_ids={key: tuple(value) for key, value in data["episode_ids"].items()})
    dataset_root = runner.resolve_dataset(Path(args.workspace).absolute(), data["path"])
    require(not output.is_relative_to(dataset_root), "output cannot be inside dataset")
    metadata_hashes = helper.preflight_view_metadata(dataset_root, spec)
    core = helper.load_core(repo)
    model_config, payload = helper._checkpoint_model_config(core, checkpoint, checkpoint_sha, spec)
    require(payload.get("sampling_plan") == plan["arm"]["sampling_plan"]
            and payload.get("sampling_plan_sha256") == plan["arm"]["sampling_plan_sha256"]
            and json_compatible(payload.get("train_config")) == plan["arm"]["train_config"]
            and payload.get("sampling_metrics", {}).get("sampling_stream_samples_seen")
                == plan["arm"]["expected_sampling_stream_samples_seen"],
            "checkpoint replay contract/accounting differs")
    loaded = core.load_training_examples(dataset_root, split="val", mode="planning", check_image_hashes=True)
    examples = tuple(loaded.examples)
    require(len(examples) == 665
            and loaded.validation_report.get("dataset_manifest_sha256") == spec.manifest_sha256
            and loaded.validation_report.get("dataset_fingerprint_sha256") == spec.corpus_fingerprint_sha256
            and core.fingerprint_examples(examples) == plan["validation"]["fingerprint_sha256"],
            "loaded shared VAL identity differs")
    order, counts = helper._episode_order(examples)
    episodes = tuple(helper.EpisodeSpec(row["label"], row["episode_id"], row["count"])
                     for row in plan["validation"]["episodes"])
    require(order == tuple(row.episode_id for row in episodes)
            and counts == {row.episode_id: row.count for row in episodes}, "VAL episode order/count differs")
    full = core.Common10TorchDataset(examples, model_config, verify_image_sha256=True, split="val")
    require(full.fingerprint_sha256 == spec.val_fingerprint_sha256, "VAL tensor fingerprint differs")
    output.mkdir(parents=False, exist_ok=False); episode_results = {}
    for episode in episodes:
        subset_examples = tuple(value for value in examples if value.episode_id == episode.episode_id)
        require(len(subset_examples) == episode.count, "episode subset count differs")
        subset = core.Common10TorchDataset(subset_examples, model_config, verify_image_sha256=True, split="val")
        child = output / "episodes" / episode.label
        report = core.evaluate_model(subset, checkpoint_path=checkpoint, output_dir=child,
            dataset_fingerprint_sha256=subset.fingerprint_sha256,
            corpus_fingerprint_sha256=spec.corpus_fingerprint_sha256, device_name="cuda:0",
            batch_size=4, num_workers=0, render_count=6, evaluation_split="val")
        episode_results[episode.label] = helper._verify_episode_output(child, report, episode=episode,
            checkpoint_sha256=checkpoint_sha, subset_fingerprint=subset.fingerprint_sha256,
            corpus_fingerprint=spec.corpus_fingerprint_sha256,
            train_fingerprint=spec.train_fingerprint_sha256)
    helper._verify_metadata_unchanged(metadata_hashes)
    require(runner.digest(checkpoint) == checkpoint_sha and runner.digest(plan_path) == args.plan_sha256
            and runner.digest(evaluator_path) == plan["evaluator_sha256"]
            and runner.digest(runner_path) == args.runner_sha256, "immutable input changed")
    summary = {"schema": SUMMARY_SCHEMA, "status": "COMPLETE_NOT_PROMOTED", "created_at_utc": utc(),
        "arm": runner.ARM, "plan_path": str(plan_path), "plan_sha256": args.plan_sha256,
        "base_commit": runner.BASE_COMMIT, "gpu_uuid": plan["gpu"]["uuid"],
        "runner_sha256": args.runner_sha256, "evaluator_sha256": plan["evaluator_sha256"],
        "source": plan["source"], "dataset": {"path": str(dataset_root),
            "manifest_sha256": spec.manifest_sha256,
            "corpus_fingerprint_sha256": spec.corpus_fingerprint_sha256,
            "train_fingerprint_sha256": spec.train_fingerprint_sha256,
            "combined_val_fingerprint_sha256": spec.val_fingerprint_sha256,
            "combined_val_example_fingerprint_sha256": plan["validation"]["fingerprint_sha256"],
            "counts": dict(spec.counts)},
        "checkpoint": {"path": str(checkpoint), "sha256": checkpoint_sha,
            "training_split": payload["training_split"], "training_episode_ids": payload["training_episode_ids"],
            "training_dataset_fingerprint_sha256": payload["dataset_fingerprint_sha256"],
            "corpus_fingerprint_sha256": payload["corpus_fingerprint_sha256"],
            "model_config_sha256": payload["model_config_sha256"],
            "sampling_plan_sha256": payload["sampling_plan_sha256"]},
        "checkpoint_sha256": checkpoint_sha, "corpus_fingerprint_sha256": spec.corpus_fingerprint_sha256,
        "combined_val_count": 665, "episodes": episode_results,
        "interpretation": "Fixed-VAL open-loop subgroup evidence only; no TEST, closed-loop, map-wide, export or vehicle claim.",
        **DENIALS}
    write_new(output / "summary.json", summary)
    require(runner.read(output / "summary.json") == summary, "persisted summary differs")
    return summary


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--runner", type=Path, required=True); parser.add_argument("--runner-sha256", required=True)
    parser.add_argument("--plan", type=Path, required=True); parser.add_argument("--plan-sha256", required=True)
    parser.add_argument("--checkpoint", type=Path, required=True); parser.add_argument("--checkpoint-sha256", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv=None):
    try:
        summary = run(parse_args(argv))
    except (ValueError, OSError, RuntimeError, subprocess.SubprocessError) as error:
        print("MOVING_DIRECTION_BALANCED_REPLAY_SUBGROUP_ERROR: " + str(error), file=sys.stderr); return 2
    print(json.dumps({"status": summary["status"], "arm": summary["arm"],
                      "checkpoint_sha256": summary["checkpoint_sha256"]},
                     sort_keys=True, separators=(",", ":"), allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
