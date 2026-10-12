#!/usr/bin/env python3
"""HH_260906 - Compare fresh C/D fits at 7,380 steps each; preserve the completed lower-budget A/B experiment."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time


# HH_260906 - Preserve the evaluator's existing plan projection; campaign ID, budgets and decision distinguish this fresh experiment.
PLAN_SCHEMA = "portable_e2e.moving_data_ab_seed_pilot_plan.v1"
VIEW_SCHEMA = "portable_e2e.moving_data_ab_views.v1"
CAMPAIGN_ID = "hh260921-moving-data-high-budget-ab-seed20260906-v1"
SOURCE_COMMIT = "b478f02e42b94bf04bffec5c8170e05edc33b0f8"
GPU = {"physical_index": 0, "uuid": "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"}
MODEL_PATH = "portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json"
MODEL_SHA256 = "29e537e8b216cff0216772624f670ece0f0849f0f2ec55c0b726f11c1d74b602"
MODEL_CANONICAL_SHA256 = "c17b49ea680b5cae0d9fc35dc6c457af8cc32d1b7171863534f70e9b158cfb7a"
MODEL_ID = "portable_e2e.perspective_trajectory.physical_stopmix.v1"
MODEL_PARAMETER_COUNT = 1_056_362
OWNER_PATH = "benchmarks/training_profile_v1/profile_portable_training.py"
OWNER_SHA256 = "a995722f1a60f192a059433afb44e6297937d97166877c5cc4de23220d15b436"
PIPELINE_PATH = "scripts/e2e/run_portable_training_campaign.py"
OUTPUT_PATH = "runs/diagnostics/moving_data_high_budget_ab_seed20260906_20260921_v1"
ARMS = ("A_baseline_train", "B_town07_augmented_train")
EXPERIMENTAL_LABELS = {ARMS[0]: "C_old_train_7380_steps", ARMS[1]: "D_augmented_train_7380_steps"}
ARM_COUNTS = {
    "A_baseline_train": {"train": 1147, "val": 665},
    "B_town07_augmented_train": {"train": 1475, "val": 665},
}
EXPECTED_TRAIN_STATE = {
    "A_baseline_train": {"epoch": 25, "next_batch_index": 205, "global_step": 7380,
                         "samples_seen": 29495, "domain_samples_seen": {"carla": 29495}},
    "B_town07_augmented_train": {"epoch": 20, "next_batch_index": 0, "global_step": 7380,
                                 "samples_seen": 29500, "domain_samples_seen": {"carla": 29500}},
}
VALIDATION_EPISODES = (
    {"episode_id": "carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9",
     "count": 337, "label": "town03_right_turn_dev"},
    {"episode_id": "carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320",
     "count": 328, "label": "town03_straight_dev"},
)
TRAIN_CONFIG = {"seed": 20260906, "batch_size": 4, "learning_rate": 1e-4,
    "weight_decay": 1e-4, "checkpoint_interval": 154,
    "num_workers": 0, "maximum_gradient_norm": 5.0, "verify_image_sha256": True,
    "sampling_policy": "uniform_without_replacement", "domain_ratios": []}
LOSS_CONFIG = {"xy_weight": 1.0, "speed_weight": 0.2, "yaw_weight": 0.1,
    "kinematic_speed_weight": 0.05, "final_displacement_weight": 0.5,
    "candidate_score_weight": 0.1, "candidate_regret_weight": 0.1}
ARM_TRAIN_CONFIG = {arm: dict(TRAIN_CONFIG, max_steps=EXPECTED_TRAIN_STATE[arm]["global_step"]) for arm in ARMS}
TIMEOUTS = {"train": 5400, "evaluate": 600, "audit": 600, "subgroup": 900}
WHOLE_TIMEOUT_SECONDS = 18000
STAGES = tuple(TIMEOUTS)
SOURCE_PATHS = tuple("portable_e2e/" + name + ".py" for name in (
    "__init__", "contract", "dataset", "torch_dataset", "model", "losses", "train",
    "evaluate", "audit_runtime", "runtime", "runtime_contract", "runtime_weight_bundle",
    "visualize", "stop_primitive_research")) + (
    "portable_e2e/config/common_10hz_v1.contract.json", MODEL_PATH,
    "scripts/e2e/audit_portable_stopmix_behavior.py", PIPELINE_PATH)
DENIALS = {"resume": False, "automatic_retry": False, "test_read_or_use": False,
    "model_export": False, "automatic_promotion": False, "training_data_approved": False,
    "vehicle_control_approved": False, "closed_loop_claim": False,
    "all_map_claim": False, "deployment_approved": False}
DECISION = {"single_seed_exploratory_only": True, "final_checkpoint_only": True,
    "same_optimizer_steps": True, "optimizer_steps_per_arm": 7380,
    "same_total_sample_exposures": False, "same_complete_epochs": False,
    "experimental_arm_labels": EXPERIMENTAL_LABELS, "fresh_initialization_each_arm": True,
    "no_early_stopping_or_checkpoint_selection": True,
    "compare_each_validation_episode_separately": True,
    "diagnostic_hypothesis": "At the same higher optimizer budget, D improves straight geometry without right-turn ADE/FDE regression versus C.",
    "maximum_D_over_C": {
        "town03_straight_dev": {"ade_6p4s_m": 0.90, "fde_6p4s_m": 0.90},
        "town03_right_turn_dev": {"ade_6p4s_m": 1.0, "fde_6p4s_m": 1.0,
                                 "speed_mae_6p4s_mps": 1.05}},
    "geometry_pass_rule": "D_count >= C_count separately for selected, any-candidate and all-candidates counts over identical 665 DEV examples",
    "planned_later_contrasts": ["D-C at 7380 steps", "prior B-prior A at 5740 steps",
                                "D-prior A", "D-prior B", "C-prior A"],
    "three_seed_confirmation_required": True, "adoption_allowed": False}


def require(value, message):
    if not value:
        raise ValueError(message)


def utc():
    return datetime.now(timezone.utc).isoformat()


def encoded(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def decode(payload):
    def pairs(items):
        result = {}
        for key, value in items:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    return json.loads(payload, object_pairs_hook=pairs,
        parse_constant=lambda value: require(False, "nonfinite JSON: " + value))


def canonical(path, *, must_exist=True):
    path = Path(path).absolute()
    require(".." not in path.parts, "parent traversal is forbidden")
    if must_exist:
        require(path.exists(), "path does not exist: " + str(path))
        require(path == path.resolve(), "canonical path required: " + str(path))
    return path


def regular(path):
    path = canonical(path)
    require(path.is_file() and path.stat().st_nlink == 1
            and not any(parent.is_symlink() for parent in (path, *path.parents)),
            "regular single-link nonsymlink file required: " + str(path))
    return path


def digest(path):
    value = hashlib.sha256()
    with regular(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def read(path):
    return decode(regular(path).read_text())


def write_new(path, value):
    # HH_260906 - Receipts are immutable create-only evidence; failures retain partial artifacts.
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(encoded(value)); stream.flush(); os.fsync(stream.fileno())


def copy_new(source, target, expected):
    source = regular(source); require(digest(source) == expected, "copy source SHA differs")
    target = Path(target); target.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as src, target.open("xb") as dst:
        shutil.copyfileobj(src, dst, 1024 * 1024); dst.flush(); os.fsync(dst.fileno())
    require(digest(source) == expected and digest(target) == expected
            and source.stat().st_ino != target.stat().st_ino, "independent exact copy failed")


def sha256(value, name="SHA256"):
    require(type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value), name + " required")
    return value


def relative(value, name="relative path"):
    require(type(value) is str and value and not Path(value).is_absolute()
            and ".." not in Path(value).parts and Path(value).as_posix() == value, name + " required")
    return value


def timestamp(value):
    require(type(value) is str, "UTC timestamp string required")
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    require(parsed.tzinfo is not None and parsed.utcoffset().total_seconds() == 0, "UTC timestamp required")
    return parsed


def git(repo, *args):
    env = dict(os.environ, GIT_NO_LAZY_FETCH="1", GIT_ALLOW_PROTOCOL="", GIT_TERMINAL_PROMPT="0")
    return subprocess.check_output(["git", "-c", "protocol.allow=never", *args], cwd=repo,
        env=env, timeout=30)


def historical_sources(repo):
    # HH_260906 - Pin the reviewed commit offline; declaration does not bless the current dirty worktree.
    result = {}
    for name in SOURCE_PATHS:
        result[name] = hashlib.sha256(git(repo, "show", SOURCE_COMMIT + ":" + name)).hexdigest()
    require(result[MODEL_PATH] == MODEL_SHA256, "historical STOPMIX config SHA differs")
    return result


def dataset_relative(output_root, workspace):
    output = canonical(output_root)
    workspace = canonical(workspace)
    candidates = [(workspace / "datasets").absolute(), (workspace.parent / "dataset").absolute()]
    for base in candidates:
        try:
            suffix = output.relative_to(base.resolve() if base.exists() else base)
        except ValueError:
            continue
        value = (Path("datasets") / suffix).as_posix()
        require(value.startswith("datasets/prepared/"), "dataset must be inside the personal prepared root")
        return relative(value, "workspace-relative dataset path")
    raise ValueError("view output_root is outside the approved personal dataset roots")


def _view_arm(report, arm, workspace):
    item = report["arms"][arm]
    require(item.get("counts") == ARM_COUNTS[arm], "view split counts differ: " + arm)
    validation = item.get("validation")
    require(type(validation) is dict and validation.get("status") == "PASS", "view validation failed: " + arm)
    corpus = sha256(validation.get("dataset_fingerprint_sha256"), "corpus fingerprint")
    fingerprints = item.get("fingerprints")
    require(type(fingerprints) is dict and set(fingerprints) == {"train", "val"}, "split fingerprints differ")
    split_fingerprint, loader_fingerprint, examples, tokens = {}, {}, {}, {}
    for split in ("train", "val"):
        row = fingerprints[split]
        require(row.get("count") == ARM_COUNTS[arm][split], "loader split count differs")
        examples[split] = sha256(row.get("examples_sha256"), "examples SHA")
        loader_fingerprint[split] = sha256(row.get("loader_sha256"), "loader SHA")
        # HH_260906 - Native checkpoints bind the tensor dataset, not LoadedDataset's manifest-aware loader hash.
        split_fingerprint[split] = hashlib.sha256(
            (examples[split] + "\0" + split).encode()).hexdigest()
        tokens[split] = sha256(row.get("tokens_sha256"), "tokens SHA")
    episodes = item.get("episode_ids")
    require(type(episodes) is dict and set(episodes) == {"train", "val"}
            and all(type(value) is list and value for value in episodes.values()), "episode IDs missing")
    require(episodes["val"] == [row["episode_id"] for row in VALIDATION_EPISODES],
            "ordered shared DEV episode IDs differ")
    require(len(episodes["train"]) == (3 if arm == ARMS[0] else 4)
            and not set(episodes["train"]) & set(episodes["val"]), "TRAIN/DEV episode identities differ")
    tree = item.get("tree")
    require(type(tree) is dict and type(tree.get("files")) is list and tree["files"], "view tree inventory missing")
    files = []
    seen = set()
    for row in tree["files"]:
        require(set(row) >= {"path", "size", "sha256"}, "tree row fields missing")
        name = relative(row["path"], "tree file path")
        require(name not in seen and not any("test" in part.lower() for part in Path(name).parts),
                "duplicate or TEST tree entry")
        require(type(row["size"]) is int and row["size"] >= 0, "tree file size invalid")
        files.append({"path": name, "size": row["size"], "sha256": sha256(row["sha256"])})
        seen.add(name)
    require("dataset.json" in seen and sha256(item.get("dataset_manifest_sha256"))
            == next(row["sha256"] for row in files if row["path"] == "dataset.json"),
            "dataset manifest inventory differs")
    return {"dataset": {"path": dataset_relative(item["output_root"], workspace),
        "manifest_sha256": item["dataset_manifest_sha256"], "corpus_fingerprint_sha256": corpus,
        "split_fingerprint_sha256": split_fingerprint, "split_loader_sha256": loader_fingerprint,
        "split_examples_sha256": examples,
        "split_tokens_sha256": tokens, "counts": dict(item["counts"]),
        "episode_ids": episodes, "tree": {key: tree[key] for key in
            ("manifest_sha256", "file_count", "directory_count", "total_size_bytes")}
            | {"files": files}}, "expected_train_state": EXPECTED_TRAIN_STATE[arm], "train_config": dict(ARM_TRAIN_CONFIG[arm])}


def declare_plan(*, repo_root, workspace, view_report, evaluator, output_plan, runner=None):
    repo = canonical(repo_root); workspace = canonical(workspace); report_path = regular(view_report)
    evaluator = regular(evaluator); runner = regular(runner or __file__)
    report = read(report_path)
    require(report.get("schema") == VIEW_SCHEMA
            and report.get("status") == "VALIDATED_AB_RESEARCH_VIEWS_NOT_NEW_ADMISSION"
            and report.get("shared_val_content_identical") is True, "qualified A/B view report required")
    require(set(report.get("arms", {})) == set(ARMS), "view arm keys differ")
    arms = {arm: _view_arm(report, arm, workspace) for arm in ARMS}
    val_examples = {arms[arm]["dataset"]["split_examples_sha256"]["val"] for arm in ARMS}
    val_tokens = {arms[arm]["dataset"]["split_tokens_sha256"]["val"] for arm in ARMS}
    require(len(val_examples) == len(val_tokens) == 1, "A/B validation content/order differs")
    plan = {"schema": PLAN_SCHEMA, "campaign_id": CAMPAIGN_ID, "declared_at_utc": utc(),
        "source_commit": SOURCE_COMMIT, "gpu": dict(GPU), "source_sha256": historical_sources(repo),
        "runner_sha256": digest(runner), "evaluator_sha256": digest(evaluator),
        "owner": {"path": OWNER_PATH, "sha256": OWNER_SHA256},
        "model_config": MODEL_PATH, "model_config_sha256": MODEL_SHA256,
        "model_config_canonical_sha256": MODEL_CANONICAL_SHA256, "model_id": MODEL_ID,
        "view_report_sha256": digest(report_path), "output": OUTPUT_PATH,
        "arms": arms, "validation": {"episodes": list(VALIDATION_EPISODES), "count": 665,
            "fingerprint_sha256": next(iter(val_examples)), "tokens_sha256": next(iter(val_tokens))},
        "train_config": dict(TRAIN_CONFIG), "loss_config": dict(LOSS_CONFIG),
        "stage_order": list(STAGES), "stage_timeout_seconds": dict(TIMEOUTS),
        "whole_timeout_seconds": WHOLE_TIMEOUT_SECONDS,
        "decision": dict(DECISION), **DENIALS}
    validate_plan(plan, runner_sha256=digest(runner))
    write_new(output_plan, plan)
    return plan


def validate_plan(plan, *, runner_sha256, observed=None):
    require(type(plan) is dict, "plan object required")
    fixed = {"schema": PLAN_SCHEMA, "campaign_id": CAMPAIGN_ID, "source_commit": SOURCE_COMMIT,
        "gpu": GPU, "owner": {"path": OWNER_PATH, "sha256": OWNER_SHA256},
        "model_config": MODEL_PATH, "model_config_sha256": MODEL_SHA256,
        "model_config_canonical_sha256": MODEL_CANONICAL_SHA256, "model_id": MODEL_ID,
        "output": OUTPUT_PATH, "train_config": TRAIN_CONFIG, "loss_config": LOSS_CONFIG,
        "stage_order": list(STAGES), "stage_timeout_seconds": TIMEOUTS,
        "whole_timeout_seconds": WHOLE_TIMEOUT_SECONDS,
        "decision": dict(DECISION), **DENIALS}
    variable = {"declared_at_utc", "source_sha256", "runner_sha256", "evaluator_sha256",
        "view_report_sha256", "arms", "validation"}
    require(set(plan) == set(fixed) | variable, "plan fields differ")
    for key, value in fixed.items():
        require(plan.get(key) == value, "fixed plan field differs: " + key)
    require(plan["runner_sha256"] == runner_sha256, "runner SHA differs")
    for key in ("runner_sha256", "evaluator_sha256", "view_report_sha256"):
        sha256(plan[key], key)
    require(set(plan["source_sha256"]) == set(SOURCE_PATHS), "source closure differs")
    for value in plan["source_sha256"].values(): sha256(value)
    require(plan["source_sha256"][MODEL_PATH] == MODEL_SHA256, "source model config differs")
    stamp = timestamp(plan["declared_at_utc"])
    require(stamp <= (observed or datetime.now(timezone.utc)), "future declaration forbidden")
    require(set(plan["arms"]) == set(ARMS), "plan arms differ")
    for arm in ARMS:
        row = plan["arms"][arm]; data = row.get("dataset")
        require(row.get("expected_train_state") == EXPECTED_TRAIN_STATE[arm]
                and row.get("train_config") == ARM_TRAIN_CONFIG[arm], "arm train state/config differs")
        require(type(data) is dict and data.get("counts") == ARM_COUNTS[arm], "dataset counts differ")
        relative(data.get("path"), "dataset path")
        require(data["path"].startswith("datasets/prepared/"), "prepared dataset required")
        for key in ("manifest_sha256", "corpus_fingerprint_sha256"): sha256(data.get(key), key)
        for field in ("split_fingerprint_sha256", "split_loader_sha256",
                      "split_examples_sha256", "split_tokens_sha256"):
            require(set(data.get(field, {})) == {"train", "val"}, field + " differs")
            for value in data[field].values(): sha256(value)
        for split in ("train", "val"):
            expected_tensor = hashlib.sha256(
                (data["split_examples_sha256"][split] + "\0" + split).encode()).hexdigest()
            require(data["split_fingerprint_sha256"][split] == expected_tensor,
                    "tensor fingerprint does not match example content/split")
        require(data["episode_ids"]["val"] == [x["episode_id"] for x in VALIDATION_EPISODES],
                "validation episode order differs")
        files = data.get("tree", {}).get("files")
        require(type(files) is list and len(files) == data["tree"].get("file_count"), "tree count differs")
        require(len({row["path"] for row in files}) == len(files), "tree duplicate path")
    expected_validation = {"episodes": list(VALIDATION_EPISODES), "count": 665,
        "fingerprint_sha256": plan["arms"][ARMS[0]]["dataset"]["split_examples_sha256"]["val"],
        "tokens_sha256": plan["arms"][ARMS[0]]["dataset"]["split_tokens_sha256"]["val"]}
    require(plan["validation"] == expected_validation
            and all(plan["arms"][arm]["dataset"]["split_examples_sha256"]["val"]
                    == expected_validation["fingerprint_sha256"]
                    and plan["arms"][arm]["dataset"]["split_tokens_sha256"]["val"]
                    == expected_validation["tokens_sha256"] for arm in ARMS),
            "shared validation contract differs")
    return plan


def verify_sources(plan, repo):
    # HH_260906 - Every stage rechecks clean reviewed Git bytes without fetching or changing the repository.
    require(git(repo, "rev-parse", "HEAD").decode().strip() == SOURCE_COMMIT, "repo HEAD differs")
    require(not git(repo, "status", "--porcelain", "--untracked-files=all").decode().strip(), "repo is not clean")
    for name, expected in plan["source_sha256"].items():
        require(digest(repo / name) == expected
                and hashlib.sha256(git(repo, "show", SOURCE_COMMIT + ":" + name)).hexdigest() == expected,
                "source bytes differ: " + name)


def resolve_dataset(workspace, name):
    relative(name); lexical = workspace / name
    datasets = workspace / "datasets"
    if datasets.is_symlink():
        require(datasets.resolve() == workspace.parent / "dataset", "unapproved datasets alias")
        current = datasets
        for part in Path(name).parts[1:]:
            current /= part; require(not current.is_symlink(), "symlink below datasets alias")
        root = lexical.resolve()
        require(root.is_relative_to((workspace.parent / "dataset/prepared").resolve()), "dataset escaped prepared root")
    else:
        root = lexical
        require(root == root.resolve(), "canonical dataset required")
    require(root.is_dir() and not root.is_symlink(), "dataset directory required")
    return root


def verify_tree(root, expected):
    observed = {}
    for parent, directories, files in os.walk(root, followlinks=False):
        for name in [*directories, *files]:
            path = Path(parent) / name
            require(not path.is_symlink() and "test" not in name.lower(), "symlink/TEST entry in view")
        for name in files:
            path = Path(parent) / name; key = path.relative_to(root).as_posix()
            require(key in expected, "unplanned view file: " + key)
            row = expected[key]
            require(path.stat().st_size == row["size"] and digest(path) == row["sha256"], "view file differs: " + key)
            observed[key] = row
    require(observed == expected, "view full inventory differs")


def preflight_metadata(root, data):
    # HH_260906 - Inspect every episode split/count before any loader can resolve camera or target payloads.
    manifest = read(root / "dataset.json"); references = manifest.get("episodes")
    require(type(references) is list and references, "dataset episode references missing")
    observed_ids = {"train": [], "val": []}; observed_counts = {"train": 0, "val": 0}
    for reference in references:
        episode_id = reference.get("episode_id")
        require(type(episode_id) is str and episode_id
                and reference.get("manifest") == f"episodes/{episode_id}/episode.json",
                "episode reference identity/path differs")
        path = root / reference["manifest"]
        require(path.is_relative_to(root) and digest(path) == reference.get("sha256"),
                "episode manifest SHA differs")
        episode = read(path); split = episode.get("split")
        require(split in observed_ids, "TEST/unknown split forbidden before payload loading")
        require(episode.get("episode_id") == episode_id and type(episode.get("sample_count")) is int
                and episode["sample_count"] > 0, "episode identity/count differs")
        observed_ids[split].append(episode_id); observed_counts[split] += episode["sample_count"]
    require(observed_ids == data["episode_ids"] and observed_counts == data["counts"],
            "dataset metadata split identity/count differs")


LOADER_PROBE = r'''
import hashlib,json,sys
from pathlib import Path
sys.path.insert(0,sys.argv[1])
from portable_e2e import dataset as data
from portable_e2e.model import ModelConfig
from portable_e2e.torch_dataset import Common10TorchDataset
root=Path(sys.argv[2])
result={}
for split in ('train','val'):
    loaded=data.load_training_examples(root,split=split,mode='planning',check_image_hashes=True)
    ids=[]
    for example in loaded.examples:
        if example.episode_id not in ids: ids.append(example.episode_id)
    token_bytes=(json.dumps([e.token for e in loaded.examples],indent=2,sort_keys=True,allow_nan=False)+'\n').encode()
    config=ModelConfig.from_mapping(json.loads(Path(sys.argv[3]).read_text()))
    tensor=Common10TorchDataset(loaded.examples,config,verify_image_sha256=True,split=split)
    result[split]={'count':len(loaded.examples),'loader_sha256':loaded.fingerprint_sha256,
        'tensor_sha256':tensor.fingerprint_sha256,
        'examples_sha256':data.fingerprint_examples(loaded.examples),
        'tokens_sha256':hashlib.sha256(token_bytes).hexdigest(),'episode_ids':ids,
        'corpus_fingerprint_sha256':loaded.validation_report['dataset_fingerprint_sha256']}
print(json.dumps(result,sort_keys=True,separators=(',',':'),allow_nan=False))
'''


def probe_view(workspace, repo, root):
    env = dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1")
    for name in ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP", "PYTHONUSERBASE"): env.pop(name, None)
    raw = subprocess.check_output([str(workspace / "venvs/py312/bin/python"), "-B", "-c", LOADER_PROBE,
        str(repo), str(root), str(repo / MODEL_PATH)], cwd=repo, env=env, text=True, timeout=900)
    return decode(raw)


def verify_views(plan, workspace, repo, view_report):
    report = read(view_report)
    require(digest(view_report) == plan["view_report_sha256"] and report.get("schema") == VIEW_SCHEMA
            and report.get("status") == "VALIDATED_AB_RESEARCH_VIEWS_NOT_NEW_ADMISSION",
            "view report differs")
    require(set(report.get("arms", {})) == set(ARMS), "view report arms differ")
    roots = {}
    for arm in ARMS:
        data = plan["arms"][arm]["dataset"]; proof = report["arms"][arm]
        root = resolve_dataset(workspace, data["path"]); roots[arm] = root
        # HH_260906 - The sealed report may have been declared on the transfer source; the plan carries its portable dataset path.
        require(Path(proof["output_root"]).is_absolute() and Path(proof["output_root"]).name == Path(data["path"]).name
                and proof["counts"] == data["counts"]
                and proof["dataset_manifest_sha256"] == data["manifest_sha256"]
                and proof["validation"]["dataset_fingerprint_sha256"] == data["corpus_fingerprint_sha256"]
                and proof["episode_ids"] == data["episode_ids"]
                and proof["tree"] == data["tree"]
                and all(proof["fingerprints"][split]["loader_sha256"] == data["split_loader_sha256"][split]
                        and proof["fingerprints"][split]["examples_sha256"] == data["split_examples_sha256"][split]
                        and proof["fingerprints"][split]["tokens_sha256"] == data["split_tokens_sha256"][split]
                        for split in ("train", "val")),
                "view report arm provenance differs")
        preflight_metadata(root, data)
        expected = {row["path"]: row for row in data["tree"]["files"]}; verify_tree(root, expected)
        observed = probe_view(workspace, repo, root)
        for split in ("train", "val"):
            require(observed[split] == {"count": data["counts"][split],
                "loader_sha256": data["split_loader_sha256"][split],
                "tensor_sha256": data["split_fingerprint_sha256"][split],
                "examples_sha256": data["split_examples_sha256"][split],
                "tokens_sha256": data["split_tokens_sha256"][split],
                "episode_ids": data["episode_ids"][split],
                "corpus_fingerprint_sha256": data["corpus_fingerprint_sha256"]},
                "live loader proof differs: " + arm + "/" + split)
    return roots


def load_module(path, expected, name):
    path = regular(path); require(digest(path) == expected, "helper SHA differs")
    spec = importlib.util.spec_from_file_location(name, path)
    require(spec is not None and spec.loader is not None, "helper import spec failed")
    module = importlib.util.module_from_spec(spec); sys.modules[name] = module; spec.loader.exec_module(module)
    require(digest(path) == expected, "helper changed during import")
    return module


def stage_command(plan, workspace, repo, output, plan_path, evaluator, arm, stage):
    item = output / arm; data = str(workspace / plan["arms"][arm]["dataset"]["path"])
    python = str(workspace / "venvs/py312/bin/python"); checkpoint = item / "training/checkpoints/latest.pt"
    if stage == "train":
        return [python, "-m", "portable_e2e.train", data, "--run-dir", str(item / "training"),
            "--model-config", str(repo / MODEL_PATH), "--split", "train", "--device", "cuda:0",
            "--batch-size", "4", "--seed", "20260906", "--learning-rate", "0.0001",
            "--weight-decay", "0.0001", "--candidate-score-weight", "0.1",
            "--candidate-regret-weight", "0.1", "--max-steps", str(plan["arms"][arm]["train_config"]["max_steps"]),
            "--checkpoint-interval", "154", "--num-workers", "0", "--maximum-gradient-norm", "5",
            "--sampling-policy", "uniform_without_replacement"]
    if stage == "evaluate":
        return [python, "-m", "portable_e2e.evaluate", data, "--checkpoint", str(checkpoint),
            "--output-dir", str(item / "evaluation"), "--split", "val", "--device", "cuda:0",
            "--batch-size", "4", "--num-workers", "0", "--render-count", "12"]
    if stage == "audit":
        return [python, "-m", "portable_e2e.audit_runtime", data, "--checkpoint", str(checkpoint),
            "--checkpoint-sha256", digest(checkpoint), "--output-json", str(item / "gate_v8.json"),
            "--split", "val", "--device", "cuda:0", "--batch-size", "4"]
    require(stage == "subgroup", "unknown stage")
    return [python, str(evaluator), "--repo-root", str(repo), "--plan", str(plan_path),
        "--plan-sha256", digest(plan_path), "--arm", arm, "--checkpoint", str(checkpoint),
        "--checkpoint-sha256", digest(checkpoint), "--output-dir", str(item / "subgroup")]


def verify_history(path, state):
    count = 0; last = None
    with regular(path).open() as stream:
        for line in stream:
            count += 1; last = decode(line)
            require(last.get("global_step") == count, "training history step differs")
    require(count == state["global_step"] and last is not None and last.get("samples_seen") == state["samples_seen"],
            "training history endpoint differs")
    return digest(path)


def _stage_common(report, plan, arm, checkpoint_sha):
    data = plan["arms"][arm]["dataset"]
    require(report.get("checkpoint_sha256") == checkpoint_sha
            and report.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"],
            "stage checkpoint/corpus differs")
    require(report.get("vehicle_control_approved") is False, "stage cannot approve vehicle control")


def verify_stage(plan, output, arm, stage):
    item = output / arm; checkpoint = item / "training/checkpoints/latest.pt"; checkpoint_sha = digest(checkpoint)
    paths = {"train": item / "training/run.json", "evaluate": item / "evaluation/metrics.json",
        "audit": item / "gate_v8.json", "subgroup": item / "subgroup/summary.json"}
    path = paths[stage]; report = read(path); data = plan["arms"][arm]["dataset"]
    config = read(output / "source" / MODEL_PATH)
    require(digest(output / "source" / MODEL_PATH) == MODEL_SHA256, "archived model differs")
    if stage == "train":
        state = EXPECTED_TRAIN_STATE[arm]
        require(report.get("status") == "TRAINING_TARGET_REACHED"
                and report.get("warning") == "RESEARCH CHECKPOINT — NOT APPROVED FOR VEHICLE CONTROL"
                and "resumed_at_utc" not in report, "fresh training completion required")
        require(report.get("device") == "cuda:0"
                and report.get("hardware", {}).get("device_uuid") == GPU["uuid"].removeprefix("GPU-"),
                "training GPU differs")
        require(report.get("dataset_size") == data["counts"]["train"]
                and report.get("dataset_fingerprint_sha256") == data["split_fingerprint_sha256"]["train"]
                and report.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"],
                "training dataset differs")
        require(report.get("training_episode_ids") == sorted(data["episode_ids"]["train"]),
                "training episode IDs differ")
        require(report.get("state") == state and report.get("train_config") == ARM_TRAIN_CONFIG[arm]
                and report.get("loss_config") == LOSS_CONFIG and report.get("model_config") == config
                and report.get("model_parameter_count") == MODEL_PARAMETER_COUNT,
                "training state/config/model differs")
        batches = 287 if arm == ARMS[0] else 369
        require(report.get("sampling_plan", {}).get("batches_per_epoch") == batches
                and report["sampling_plan"].get("epoch_sample_count") == data["counts"]["train"],
                "training sampling plan differs")
        history_sha = verify_history(item / "training/metrics.jsonl", state)
        return {"path": path.relative_to(output).as_posix(), "sha256": digest(path),
            "checkpoint_sha256": checkpoint_sha, "history_sha256": history_sha}
    _stage_common(report, plan, arm, checkpoint_sha)
    if stage in ("evaluate", "audit"):
        count = report.get("sample_count") if stage == "evaluate" else report.get("geometry", {}).get("sample_count")
        require(report.get("evaluation_split") == "val" and count == 665
                and report.get("evaluation_episode_count") == 2
                and report.get("dataset_fingerprint_sha256") == data["split_fingerprint_sha256"]["val"]
                and report.get("training_dataset_fingerprint_sha256") == data["split_fingerprint_sha256"]["train"]
                and report.get("model_config_sha256") == MODEL_CANONICAL_SHA256
                and report.get("model_parameter_count") == MODEL_PARAMETER_COUNT,
                "evaluation/audit provenance differs")
        if stage == "evaluate":
            require(report.get("status") == "OPEN_LOOP_EVALUATION_COMPLETE"
                    and report.get("loss_config") == LOSS_CONFIG
                    and report.get("auxiliary_loss_metric_counts") == {"candidate_regret_loss": 665}
                    and all(type(value) in (int, float) and math.isfinite(value)
                            for value in report.get("metrics", {}).values()), "evaluation metrics differ")
        else:
            require(report.get("status") == "RUNTIME_GEOMETRY_AUDIT_COMPLETE"
                    and report.get("gate", {}).get("source") == "portable_e2e.runtime_geometry_gate.v8"
                    and report["gate"].get("threshold_overrides") is False
                    and report["gate"].get("thresholds", {}).get("candidate_count") == 12,
                    "runtime audit gate differs")
    else:
        require(report.get("status") == "COMPLETE_NOT_PROMOTED" and report.get("arm") == arm
                and report.get("checkpoint_sha256") == checkpoint_sha
                and report.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"],
                "subgroup summary provenance differs")
        episodes = report.get("episodes")
        require(type(episodes) in (dict, list), "subgroup episode results missing")
    return {"path": path.relative_to(output).as_posix(), "sha256": digest(path),
        "checkpoint_sha256": checkpoint_sha}


def _episode_rows(summary):
    value = summary["episodes"]
    rows = list(value.values()) if isinstance(value, dict) else value
    result = {}
    for row in rows:
        episode_id = row.get("episode_id")
        metrics = row.get("metrics", row.get("all_samples", {}).get("metrics"))
        require(episode_id and isinstance(metrics, dict), "subgroup episode metrics missing")
        result[episode_id] = metrics
    require(set(result) == {row["episode_id"] for row in VALIDATION_EPISODES}, "subgroup episodes differ")
    return result


def diagnostic_gate(reports, audits):
    # HH_260906 - These preregistered relative checks report this seed only; a pass never promotes a model.
    metrics = {arm: _episode_rows(reports[arm]) for arm in ARMS}
    checks = {}
    for episode in VALIDATION_EPISODES:
        label, eid, count = episode["label"], episode["episode_id"], episode["count"]
        for arm in ARMS:
            values = reports[arm]["episodes"]
            rows = list(values.values()) if isinstance(values, dict) else values
            matched = [row for row in rows if row.get("episode_id") == eid]
            require(len(matched) == 1 and matched[0].get("count") == count,
                    "diagnostic gate requires the complete fixed episode")
            require(all(matched[0].get("metric_counts", {}).get(key) == count
                        for key in DECISION["maximum_D_over_C"][label]),
                    "diagnostic gate metric denominator differs")
        checks[label] = {}
        for key, multiplier in DECISION["maximum_D_over_C"][label].items():
            a, b = metrics[ARMS[0]][eid].get(key), metrics[ARMS[1]][eid].get(key)
            require(all(type(value) in (int, float) and math.isfinite(value) and value >= 0 for value in (a, b)),
                    "diagnostic gate requires finite nonnegative metrics")
            checks[label][key] = {"C": a, "D": b, "maximum_D_over_C": multiplier,
                "maximum_D": multiplier * a, "observed_D_over_C": b / a if a else None,
                "pass": b <= multiplier * a}
    geometry_counts = {}
    for arm in ARMS:
        geometry = audits[arm].get("geometry", {})
        value = geometry.get("selected_result", {})
        counts = {"selected": value.get("geometry_pass_count"),
                  "any_candidate": geometry.get("any_candidate_geometry_pass_count"),
                  "all_candidates": geometry.get("all_candidates_geometry_pass_count")}
        require(geometry.get("sample_count") == value.get("sample_count") == 665
                and all(type(count) is int and 0 <= count <= 665 for count in counts.values()),
                "diagnostic gate geometry denominator/count differs")
        geometry_counts[arm] = counts
    geometry_checks = {kind: {"C_pass_count": geometry_counts[ARMS[0]][kind],
                             "D_pass_count": geometry_counts[ARMS[1]][kind],
                             "pass": geometry_counts[ARMS[1]][kind] >= geometry_counts[ARMS[0]][kind]}
                       for kind in ("selected", "any_candidate", "all_candidates")}
    geometry_pass = all(row["pass"] for row in geometry_checks.values())
    passed = geometry_pass and all(item["pass"] for episode in checks.values() for item in episode.values())
    return {"status": "PASS_SINGLE_SEED_DIAGNOSTIC" if passed else "FAIL_SINGLE_SEED_DIAGNOSTIC",
        "pass": passed, "episode_metric_checks": checks,
        "geometry": {"sample_count": 665, "checks": geometry_checks, "pass": geometry_pass},
        "three_seed_confirmation_required": True, "adoption_allowed": False,
        "automatic_promotion": False}


def comparison(output):
    reports = {arm: read(output / arm / "subgroup/summary.json") for arm in ARMS}
    episodes = {arm: _episode_rows(report) for arm, report in reports.items()}
    deltas = {}
    for episode in VALIDATION_EPISODES:
        eid = episode["episode_id"]; left, right = episodes[ARMS[0]][eid], episodes[ARMS[1]][eid]
        keys = sorted(set(left) & set(right))
        deltas[episode["label"]] = {key: right[key] - left[key] for key in keys
            if type(left[key]) in (int, float) and type(right[key]) in (int, float)
            and math.isfinite(left[key]) and math.isfinite(right[key])}
    return {"arms": {arm: {"subgroup_summary_sha256": digest(output / arm / "subgroup/summary.json")}
        for arm in ARMS}, "experimental_arm_labels": EXPERIMENTAL_LABELS,
        "metric_delta_D_minus_C_by_episode": deltas,
        "diagnostic_gate": diagnostic_gate(reports, {arm: read(output / arm / "gate_v8.json") for arm in ARMS}),
        "interpretation": "Fresh C/D fits use 7380 optimizer steps each and 29495/29500 sample exposures, respectively. The older A/B5740 results remain separate evidence. Single-seed open-loop diagnostic only; three-seed confirmation is required before any separate adoption review."}


def _archive(plan, output, repo, runner, evaluator, owner, view_report):
    archive = output / "source"
    for name, expected in plan["source_sha256"].items(): copy_new(repo / name, archive / name, expected)
    copy_new(runner, archive / "run_moving_data_exposure_match_pilot.py", plan["runner_sha256"])
    copy_new(evaluator, archive / "evaluate_portable_moving_data_subgroups.py", plan["evaluator_sha256"])
    copy_new(owner, archive / "profile_portable_training.py", OWNER_SHA256)
    copy_new(view_report, output / "view_report.json", plan["view_report_sha256"])


def run(*, plan_path, repo_root, workspace, view_report, evaluator, runner=None):
    runner = regular(runner or __file__); repo = canonical(repo_root); workspace = canonical(workspace)
    plan_path = regular(plan_path); view_report = regular(view_report); evaluator = regular(evaluator)
    require(not runner.is_relative_to(repo) and not evaluator.is_relative_to(repo),
            "main execution requires archived runner/evaluator outside the clean repo")
    plan_sha = digest(plan_path); plan = validate_plan(read(plan_path), runner_sha256=digest(runner))
    require(digest(evaluator) == plan["evaluator_sha256"] and digest(view_report) == plan["view_report_sha256"],
            "external execution inputs differ")
    output = workspace / plan["output"]
    require(output == workspace / OUTPUT_PATH and not output.exists() and not output.is_symlink(),
            "fresh fixed output required; resume/rerun forbidden")
    verify_sources(plan, repo); verify_views(plan, workspace, repo, view_report)
    owner_path = regular(workspace / OWNER_PATH)
    require(digest(owner_path) == OWNER_SHA256, "GPU owner helper differs")
    pipeline_path = regular(repo / PIPELINE_PATH)
    require(digest(pipeline_path) == plan["source_sha256"][PIPELINE_PATH], "stage owner differs")
    owner = load_module(owner_path, OWNER_SHA256, "_moving_ab_gpu_owner")
    pipeline = load_module(pipeline_path, plan["source_sha256"][PIPELINE_PATH], "_moving_ab_stage_owner")
    owner.check_environment(workspace, repo)
    with owner.gpu_lease(workspace):
        idle = owner.gpu_idle(repo); output.mkdir(parents=True, exist_ok=False)
        copy_new(plan_path, output / "plan.json", plan_sha); _archive(plan, output, repo, runner, evaluator, owner_path, view_report)
        status = {"schema": "portable_e2e.moving_data_high_budget_ab_pilot.v1", "status": "RUNNING",
            "started_at_utc": utc(), "plan_sha256": plan_sha, "pre_cuda_gpu": idle,
            "experimental_arm_labels": EXPERIMENTAL_LABELS,
            "arms": {arm: "NOT_RUN" for arm in ARMS}, "stages": [], **DENIALS}
        write_new(output / "started.json", status); started = time.monotonic()
        try:
            with owner.bounded_signals(WHOLE_TIMEOUT_SECONDS):
                for arm in ARMS:
                    item = output / arm; item.mkdir(); status["arms"][arm] = "RUNNING"; checkpoint_sha = None
                    for stage in STAGES:
                        require(time.monotonic() - started + TIMEOUTS[stage] + 30 < WHOLE_TIMEOUT_SECONDS,
                                "insufficient whole-run reserve")
                        verify_sources(plan, repo); verify_views(plan, workspace, repo, view_report)
                        require(digest(runner) == plan["runner_sha256"] and digest(evaluator) == plan["evaluator_sha256"]
                                and digest(plan_path) == plan_sha, "runner/evaluator/plan changed")
                        if checkpoint_sha is not None:
                            require(digest(item / "training/checkpoints/latest.pt") == checkpoint_sha,
                                    "checkpoint changed before stage")
                        args = stage_command(plan, workspace, repo, output, output / "plan.json", evaluator, arm, stage)
                        receipt = {"arm": arm, "stage": stage, "command": args,
                            "timeout_seconds": TIMEOUTS[stage], "started_at_utc": utc(), "status": "RUNNING",
                            "owner_pid": os.getpid(), "plan_sha256": plan_sha,
                            "input_checkpoint_sha256": checkpoint_sha, "pre_cuda_gpu": owner.gpu_idle(repo)}
                        write_new(item / (stage + "_started.json"), receipt)
                        env = dict(os.environ, PORTABLE_MOVING_DATA_EXPOSURE_MATCH_OWNER_PID=str(os.getpid()))
                        with (item / (stage + ".log")).open("xb") as log:
                            rc = pipeline.run_owned_stage(args, repo, env, log, timeout=TIMEOUTS[stage])
                        require(rc == 0, arm + " " + stage + " exited " + str(rc))
                        if checkpoint_sha is not None:
                            require(digest(item / "training/checkpoints/latest.pt") == checkpoint_sha,
                                    "checkpoint changed during read-only stage")
                        proof = verify_stage(plan, output, arm, stage); checkpoint_sha = proof["checkpoint_sha256"]
                        receipt.update(status="COMPLETE", ended_at_utc=utc(), **proof)
                        write_new(item / (stage + "_completed.json"), receipt); status["stages"].append(receipt)
                    status["arms"][arm] = "COMPLETE"
                verify_sources(plan, repo); verify_views(plan, workspace, repo, view_report)
                status.update(status="COMPLETE_NOT_PROMOTED", comparison=comparison(output),
                    source_and_views_postcheck_pass=True, ended_at_utc=utc(), final_gpu=owner.gpu_idle(repo))
                write_new(output / "summary.json", status)
        except BaseException as error:
            for arm in ARMS:
                if status["arms"][arm] == "RUNNING": status["arms"][arm] = "FAIL"
            status.update(status="FAIL", error_type=type(error).__name__, error=str(error), ended_at_utc=utc())
            with owner.finalization_signals(): write_new(output / "failed.json", status)
            raise
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    sub = parser.add_subparsers(dest="mode", required=True)
    for name in ("declare", "run"):
        item = sub.add_parser(name, allow_abbrev=False)
        item.add_argument("--repo-root", type=Path, required=True)
        item.add_argument("--workspace", type=Path, required=True)
        item.add_argument("--view-report", type=Path, required=True)
        item.add_argument("--evaluator", type=Path, required=True)
    sub.choices["declare"].add_argument("--output-plan", type=Path, required=True)
    sub.choices["run"].add_argument("--plan", type=Path, required=True)
    args = parser.parse_args(argv)
    common = dict(repo_root=args.repo_root, workspace=args.workspace,
                  view_report=args.view_report, evaluator=args.evaluator)
    if args.mode == "declare": declare_plan(output_plan=args.output_plan, **common)
    else: run(plan_path=args.plan, **common)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
