#!/usr/bin/env python3
"""HH_260906 - Run one sealed F direction-balanced replay performance diagnostic."""
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


PLAN_SCHEMA = "portable_e2e.moving_direction_balanced_replay_seed_pilot_plan.v1"
SUMMARY_SCHEMA = "portable_e2e.moving_direction_balanced_replay_seed_pilot_summary.v1"
CAMPAIGN_ID = "hh260921-moving-direction-balanced-replay-seed20260906-v1"
BASE_COMMIT = "b478f02e42b94bf04bffec5c8170e05edc33b0f8"
GPU = {"physical_index": 0, "uuid": "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"}
ARM = "F_direction_balanced_replay_train"
VIEW_ARM = "B_town07_augmented_train"
OUTPUT_PATH = "runs/diagnostics/moving_direction_balanced_replay_seed20260906_20260921_v1"
MODEL_PATH = "portable_e2e/config/perspective_trajectory_physical_stopmix_v1.model.json"
MODEL_SHA256 = "29e537e8b216cff0216772624f670ece0f0849f0f2ec55c0b726f11c1d74b602"
MODEL_CANONICAL_SHA256 = "c17b49ea680b5cae0d9fc35dc6c457af8cc32d1b7171863534f70e9b158cfb7a"
MODEL_ID = "portable_e2e.perspective_trajectory.physical_stopmix.v1"
MODEL_PARAMETER_COUNT = 1_056_362
OWNER_PATH = "benchmarks/training_profile_v1/profile_portable_training.py"
OWNER_SHA256 = "a995722f1a60f192a059433afb44e6297937d97166877c5cc4de23220d15b436"
PIPELINE_PATH = "scripts/e2e/run_portable_training_campaign.py"
LEGACY_EVALUATOR_PATH = "scripts/e2e/evaluate_portable_moving_data_subgroups.py"
OVERLAY_PATHS = ("portable_e2e/evaluate.py", "portable_e2e/train.py")
BASE_SOURCE_PATHS = tuple("portable_e2e/" + name + ".py" for name in (
    "__init__", "contract", "dataset", "torch_dataset", "model", "losses", "train",
    "evaluate", "audit_runtime", "runtime", "runtime_contract", "runtime_weight_bundle",
    "visualize", "stop_primitive_research")) + (
    "portable_e2e/config/common_10hz_v1.contract.json", MODEL_PATH,
    "scripts/e2e/audit_portable_stopmix_behavior.py", PIPELINE_PATH)
ADDED_SOURCE_PATHS = (LEGACY_EVALUATOR_PATH,)
SOURCE_PATHS = BASE_SOURCE_PATHS + ADDED_SOURCE_PATHS
VALIDATION_EPISODES = (
    {"episode_id": "carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9",
     "count": 337, "label": "town03_right_turn_dev"},
    {"episode_id": "carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320",
     "count": 328, "label": "town03_straight_dev"},
)
TRAIN_CONFIG = {"seed": 20260906, "batch_size": 4, "learning_rate": 1e-4,
    "weight_decay": 1e-4, "max_steps": 8280, "checkpoint_interval": 414,
    "num_workers": 0, "maximum_gradient_norm": 5.0, "verify_image_sha256": True,
    "sampling_policy": "direction_balanced_cohort_replay", "domain_ratios": []}
LOSS_CONFIG = {"xy_weight": 1.0, "speed_weight": 0.2, "yaw_weight": 0.1,
    "kinematic_speed_weight": 0.05, "final_displacement_weight": 0.5,
    "candidate_score_weight": 0.1, "candidate_regret_weight": 0.1}
EXPECTED_TRAIN_STATE = {"epoch": 20, "next_batch_index": 0, "global_step": 8280,
    "samples_seen": 33120, "domain_samples_seen": {"carla": 33120}}
EXPECTED_STREAM_SAMPLES = {"base": 29500, "replay": 3620}
EXPECTED_SAMPLING_PLAN_SHA256 = "abb0e655b174844cc20f293bc12c5156fe33b3a00741ac1cfe8b703367e41c7b"
EXPECTED_EXECUTION_SCHEDULE_SHA256 = "d340a10c42f2a733988b69ae6571ec3c37b7e8fa1601c43e6111192cd35c55b0"
EXPECTED_BASE_OCCURRENCES_SHA256 = "0791287eda5a5b5281bf609206e6db8b2686c05abd78b5f40e89e3eba2da462b"
EXPECTED_DIRECTION_MEMBERSHIP_SHA256 = {
    "left": "d905521dc7573692fbd7910aeba4e54a47e2a6b62196e83ee582861c3776d11c",
    "right": "d4b2b7b110b2f8c71f54b55c17f18e8e47078fdb92f0d40f8dcf68e051901f23",
}
COHORT_CRITERION = {"split": "train", "field": "target_yaw_rad",
    "reduction": "max_abs_non_null", "comparison": "strictly_greater_than",
    "threshold_rad": 0.2, "direction_source": "scenario_tags_exact_membership",
    "direction_tags": ["left", "right"],
    "balancing": "minority_high_curvature_to_majority_count"}
REFERENCE_METRICS = {
    "town03_right_turn_dev": {"ade_6p4s_m": 2.9353143748791353,
        "fde_6p4s_m": 7.793866114022117, "speed_mae_6p4s_mps": 1.1076154233086357},
    "town03_straight_dev": {"ade_6p4s_m": 8.090865013076037,
        "fde_6p4s_m": 19.32532976894844},
}
REFERENCE_D_METRICS = {
    "town03_straight_dev": {"ade_6p4s_m": 3.764740187947343,
        "fde_6p4s_m": 11.027441243572932, "speed_mae_6p4s_mps": 1.7877504960065935},
    "combined_val": {"ade_6p4s_m": 3.5632455477589056,
        "fde_6p4s_m": 10.353536791729749, "speed_mae_6p4s_mps": 1.6350220532345592},
}
MAXIMUM_F = {
    "town03_right_turn_dev": {"ade_6p4s_m": 2.9353143748791353,
        "fde_6p4s_m": 7.793866114022117, "speed_mae_6p4s_mps": 1.1076154233086357},
    "town03_straight_dev": dict(REFERENCE_D_METRICS["town03_straight_dev"]),
    "combined_val": dict(REFERENCE_D_METRICS["combined_val"]),
}
REFERENCE_GEOMETRY = {"selected": 664, "any_candidate": 664, "all_candidates": 664}
REFERENCE_FILES = {
    "plan": "plan.json", "summary": "summary.json",
    "checkpoint": "A_baseline_train/training/checkpoints/latest.pt",
    "training_run": "A_baseline_train/training/run.json",
    "subgroup": "A_baseline_train/subgroup/summary.json",
    "gate": "A_baseline_train/gate_v8.json",
}
REFERENCE_D_FILES = {
    "checkpoint": "B_town07_augmented_train/training/checkpoints/latest.pt",
    "training_run": "B_town07_augmented_train/training/run.json",
    "evaluation": "B_town07_augmented_train/evaluation/metrics.json",
    "subgroup": "B_town07_augmented_train/subgroup/summary.json",
    "gate": "B_town07_augmented_train/gate_v8.json",
}
TIMEOUTS = {"train": 7200, "evaluate": 900, "audit": 900, "subgroup": 1200}
WHOLE_TIMEOUT_SECONDS = 12600
STAGES = tuple(TIMEOUTS)
DENIALS = {"resume": False, "automatic_retry": False, "test_read_or_use": False,
    "model_export": False, "automatic_promotion": False, "training_data_approved": False,
    "vehicle_control_approved": False, "closed_loop_claim": False,
    "all_map_claim": False, "deployment_approved": False}
DECISION = {"single_seed_exploratory_only": True, "final_checkpoint_only": True,
    "fresh_initialization": True, "no_early_stopping_or_checkpoint_selection": True,
    "reference_arm": "pareto_envelope_C_right_D_straight_combined", "experimental_arm": ARM,
    "maximum_F": MAXIMUM_F, "minimum_geometry_pass_counts": REFERENCE_GEOMETRY,
    "diagnostic_hypothesis": "Direction-balanced high-curvature exposure restores the C right-turn envelope without losing the D straight and combined envelopes.",
    "unseen_confirmation_seeds": [20260907, 20260908, 20260909],
    "confirmation_runs_allowed_only_after_seed20260906_pass": True,
    "adoption_allowed": False}


def require(value, message):
    if not value:
        raise ValueError(message)


def utc():
    return datetime.now(timezone.utc).isoformat()


def encoded(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def compact(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def canonical_sha256(value):
    return hashlib.sha256(compact(value)).hexdigest()


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
        require(path.exists() and path == path.resolve(), "canonical existing path required: " + str(path))
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
    # HH_260906 - Every receipt is create-only so a failed attempt cannot rewrite its history.
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(encoded(value)); stream.flush(); os.fsync(stream.fileno())


def copy_new(source, target, expected):
    source = regular(source); require(digest(source) == expected, "copy source SHA differs")
    target = Path(target); target.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as src, target.open("xb") as dst:
        shutil.copyfileobj(src, dst, 1024 * 1024); dst.flush(); os.fsync(dst.fileno())
    require(digest(target) == expected and source.stat().st_ino != target.stat().st_ino,
            "independent exact copy failed")


def sha256(value, name="SHA256"):
    require(type(value) is str and re.fullmatch(r"[0-9a-f]{64}", value), name + " required")
    return value


def relative(value, name="relative path"):
    require(type(value) is str and value and not Path(value).is_absolute()
            and ".." not in Path(value).parts and Path(value).as_posix() == value,
            name + " required")
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


def source_identity(repo):
    # HH_260906 - The detached base stays fixed; only the explicitly reviewed trainer/evaluator overlay may differ.
    require(git(repo, "rev-parse", "HEAD").decode().strip() == BASE_COMMIT, "base commit differs")
    current, base = {}, {}
    for name in BASE_SOURCE_PATHS:
        current[name] = digest(repo / name)
        base[name] = hashlib.sha256(git(repo, "show", BASE_COMMIT + ":" + name)).hexdigest()
    observed = tuple(sorted(name for name in BASE_SOURCE_PATHS if current[name] != base[name]))
    require(observed == OVERLAY_PATHS, "runtime overlay source set differs")
    require(current[MODEL_PATH] == base[MODEL_PATH] == MODEL_SHA256, "model config differs")
    added = {}
    env = dict(os.environ, GIT_NO_LAZY_FETCH="1", GIT_ALLOW_PROTOCOL="", GIT_TERMINAL_PROMPT="0")
    for name in ADDED_SOURCE_PATHS:
        result = subprocess.run(["git", "-c", "protocol.allow=never", "cat-file", "-e",
            BASE_COMMIT + ":" + name], cwd=repo, env=env, stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
        require(result.returncode != 0, "declared added source unexpectedly exists at base commit: " + name)
        added[name] = digest(repo / name)
    return {"base_commit": BASE_COMMIT, "base_sha256": base,
        "current_sha256": current, "overlay_paths": list(observed),
        "added_paths": list(ADDED_SOURCE_PATHS), "added_sha256": added}


def verify_sources(plan, repo):
    observed = source_identity(repo)
    require(observed == plan["source"], "source/overlay identity differs")


def dataset_relative(output_root, workspace):
    output = canonical(output_root); workspace = canonical(workspace)
    for base in ((workspace / "datasets").absolute(), (workspace.parent / "dataset").absolute()):
        try:
            suffix = output.relative_to(base.resolve() if base.exists() else base)
        except ValueError:
            continue
        value = (Path("datasets") / suffix).as_posix()
        require(value.startswith("datasets/prepared/"), "dataset must be inside personal prepared storage")
        return relative(value)
    raise ValueError("view output_root is outside approved personal dataset roots")


def _view_dataset(report, workspace):
    require(report.get("schema") == "portable_e2e.moving_data_ab_views.v1"
            and report.get("status") == "VALIDATED_AB_RESEARCH_VIEWS_NOT_NEW_ADMISSION"
            and report.get("shared_val_content_identical") is True, "qualified A/B view report required")
    require(set(report.get("arms", {})) == {"A_baseline_train", VIEW_ARM}, "view arms differ")
    item = report["arms"][VIEW_ARM]
    require(item.get("counts") == {"train": 1475, "val": 665}, "augmented split counts differ")
    require(item.get("validation", {}).get("status") == "PASS", "view validation failed")
    fingerprints = item.get("fingerprints")
    require(type(fingerprints) is dict and set(fingerprints) == {"train", "val"}, "fingerprints missing")
    split_examples, split_tokens, split_loader, split_tensor = {}, {}, {}, {}
    for split, count in (("train", 1475), ("val", 665)):
        row = fingerprints[split]; require(row.get("count") == count, "view count differs")
        split_examples[split] = sha256(row.get("examples_sha256")); split_tokens[split] = sha256(row.get("tokens_sha256"))
        split_loader[split] = sha256(row.get("loader_sha256"))
        split_tensor[split] = hashlib.sha256((split_examples[split] + "\0" + split).encode()).hexdigest()
    ids = item.get("episode_ids")
    require(type(ids) is dict and len(ids.get("train", [])) == 4
            and ids.get("val") == [row["episode_id"] for row in VALIDATION_EPISODES]
            and not set(ids["train"]) & set(ids["val"]), "episode identity/split differs")
    tree = item.get("tree"); require(type(tree) is dict and type(tree.get("files")) is list, "view tree missing")
    files, seen = [], set()
    for row in tree["files"]:
        name = relative(row.get("path"), "tree path")
        require(name not in seen and not any("test" in part.lower() for part in Path(name).parts),
                "duplicate/TEST tree entry")
        require(type(row.get("size")) is int and row["size"] >= 0, "tree size invalid")
        files.append({"path": name, "size": row["size"], "sha256": sha256(row.get("sha256"))}); seen.add(name)
    manifest_sha = sha256(item.get("dataset_manifest_sha256"))
    require("dataset.json" in seen and manifest_sha == next(x["sha256"] for x in files if x["path"] == "dataset.json"),
            "manifest inventory differs")
    return {"path": dataset_relative(item["output_root"], workspace),
        "manifest_sha256": manifest_sha,
        "corpus_fingerprint_sha256": sha256(item["validation"]["dataset_fingerprint_sha256"]),
        "split_fingerprint_sha256": split_tensor, "split_loader_sha256": split_loader,
        "split_examples_sha256": split_examples, "split_tokens_sha256": split_tokens,
        "counts": dict(item["counts"]), "episode_ids": ids,
        "tree": {key: tree[key] for key in ("manifest_sha256", "file_count", "directory_count", "total_size_bytes")}
            | {"files": files}}


def resolve_dataset(workspace, name):
    relative(name); lexical = workspace / name; datasets = workspace / "datasets"
    if datasets.is_symlink():
        require(datasets.resolve() == workspace.parent / "dataset", "unapproved datasets alias")
        current = datasets
        for part in Path(name).parts[1:]:
            current /= part; require(not current.is_symlink(), "symlink below datasets alias")
        root = lexical.resolve(); require(root.is_relative_to((workspace.parent / "dataset/prepared").resolve()),
                                        "dataset escaped prepared root")
    else:
        root = lexical; require(root == root.resolve(), "canonical dataset required")
    require(root.is_dir() and not root.is_symlink(), "dataset directory required")
    return root


def verify_tree(root, expected):
    expected = {row["path"]: row for row in expected}; observed = {}
    for parent, directories, files in os.walk(root, followlinks=False):
        for name in [*directories, *files]:
            path = Path(parent) / name
            require(not path.is_symlink() and "test" not in name.lower(), "symlink/TEST view entry")
        for name in files:
            path = Path(parent) / name; key = path.relative_to(root).as_posix()
            require(key in expected, "unplanned view file: " + key)
            row = expected[key]
            require(path.stat().st_size == row["size"] and digest(path) == row["sha256"], "view file differs: " + key)
            observed[key] = row
    require(observed == expected, "view full inventory differs")


def preflight_metadata(root, data):
    manifest = read(root / "dataset.json"); refs = manifest.get("episodes")
    require(type(refs) is list and refs, "dataset episode references missing")
    ids = {"train": [], "val": []}; counts = {"train": 0, "val": 0}
    for ref in refs:
        episode_id = ref.get("episode_id"); expected = f"episodes/{episode_id}/episode.json"
        require(type(episode_id) is str and ref.get("manifest") == expected, "episode reference differs")
        path = root / expected; require(path.is_relative_to(root) and digest(path) == ref.get("sha256"), "episode SHA differs")
        episode = read(path); split = episode.get("split")
        require(split in ids, "TEST/unknown split forbidden before payload loading")
        require(episode.get("episode_id") == episode_id and type(episode.get("sample_count")) is int
                and episode["sample_count"] > 0, "episode identity/count differs")
        ids[split].append(episode_id); counts[split] += episode["sample_count"]
    require(ids == data["episode_ids"] and counts == data["counts"], "dataset metadata differs")


COHORT_PROBE = r'''
import hashlib,json,sys
from pathlib import Path
sys.path.insert(0,sys.argv[1])
from portable_e2e import dataset as data
from portable_e2e.model import ModelConfig
from portable_e2e.torch_dataset import Common10TorchDataset
from portable_e2e.train import (TrainConfig,_cohort_replay_epoch_occurrences,
 _dataset_domain_indices,_sampling_plan)
root=Path(sys.argv[2]); model=ModelConfig.from_mapping(json.loads(Path(sys.argv[3]).read_text()))
result={}
for split in ('train','val'):
 loaded=data.load_training_examples(root,split=split,mode='planning',check_image_hashes=True)
 tokens=[e.token for e in loaded.examples]
 token_bytes=(json.dumps(tokens,indent=2,sort_keys=True,allow_nan=False)+'\n').encode()
 tensor=Common10TorchDataset(loaded.examples,model,verify_image_sha256=True,split=split)
 result[split]={'count':len(loaded.examples),'loader_sha256':loaded.fingerprint_sha256,
  'tensor_sha256':tensor.fingerprint_sha256,'examples_sha256':data.fingerprint_examples(loaded.examples),
  'tokens_sha256':hashlib.sha256(token_bytes).hexdigest(),'episode_ids':list(dict.fromkeys(e.episode_id for e in loaded.examples)),
  'corpus_fingerprint_sha256':loaded.validation_report['dataset_fingerprint_sha256']}
 if split=='train':
  cfg=TrainConfig(seed=20260906,batch_size=4,learning_rate=1e-4,weight_decay=1e-4,max_steps=8280,
   checkpoint_interval=414,num_workers=0,maximum_gradient_norm=5.0,verify_image_sha256=True,
   sampling_policy='direction_balanced_cohort_replay',domain_ratios=())
  domain_indices=_dataset_domain_indices(tensor)
  result['sampling_plan']=_sampling_plan(domain_indices,cfg,dataset=tensor)
  rows=[]; intra_batch_collision_count=0
  for epoch in range(20):
   occurrences=_cohort_replay_epoch_occurrences(result['sampling_plan'],cfg,epoch=epoch,domain_indices=domain_indices)
   for offset in range(0,len(occurrences),4):
    indices=[row['dataset_index'] for row in occurrences[offset:offset+4]]
    intra_batch_collision_count += len(indices)-len(set(indices))
   for position,row in enumerate(occurrences):
    rows.append({'epoch':epoch,'position':position,'stream':row['sampling_stream'],
     'occurrence_index':row['occurrence_index'],'dataset_index':row['dataset_index']})
  result['execution_schedule']={'schema':'portable_e2e.cohort_replay_execution_schedule.v1',
   'epoch_count':20,'epoch_sample_count':1656,'row_count':len(rows),
   'intra_batch_collision_count':intra_batch_collision_count,
   'hash_encoding':'sha256_canonical_json_v1','sha256':hashlib.sha256(json.dumps(rows,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest(),
   'rows':rows}
print(json.dumps(result,sort_keys=True,separators=(',',':'),allow_nan=False))
'''


def probe_dataset(workspace, repo, root):
    env = dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1")
    for name in ("PYTHONPATH", "PYTHONHOME", "PYTHONSTARTUP", "PYTHONUSERBASE"): env.pop(name, None)
    raw = subprocess.check_output([str(workspace / "venvs/py312/bin/python"), "-B", "-c", COHORT_PROBE,
        str(repo), str(root), str(repo / MODEL_PATH)], cwd=repo, env=env, text=True, timeout=1200)
    return decode(raw)


def validate_sampling_plan(value):
    require(type(value) is dict, "sampling plan object required")
    fixed = {"policy": "direction_balanced_cohort_replay",
        "order_algorithm": "smooth_collision_avoiding_direction_balanced_replay_interleave_v1",
        "seed_derivation": "sha256_seed_epoch_named_stream_v1", "known_domains": ["carla", "real"],
        "domain_ratios": {}, "replacement": False, "cross_stream_sample_reuse": True,
        "dataset_domain_counts": {"carla": 1475}, "epoch_domain_sample_counts": {"carla": 1656},
        "epoch_replayed_domain_sample_counts": {"carla": 181},
        "epoch_discarded_sample_counts": {"carla": 0}, "epoch_sample_count": 1656,
        "batch_size": 4, "batches_per_epoch": 414}
    require(set(value) == set(fixed) | {"cohort_replay"}, "sampling plan fields differ")
    for key, expected in fixed.items(): require(value.get(key) == expected, "sampling plan differs: " + key)
    cohort = value["cohort_replay"]
    scalar = {"schema": "portable_e2e.direction_balanced_cohort_replay.v1", "criterion": COHORT_CRITERION,
        "occurrence_order_hash_encoding": "sha256_canonical_json_array_v1",
        "rotation_algorithm": "cyclic_concatenated_remainder_v1",
        "base_sample_count": 1475, "membership_sample_count": 327,
        "direction_membership_counts": {"left": 254, "right": 73},
        "direction_memberships_sha256": EXPECTED_DIRECTION_MEMBERSHIP_SHA256,
        "dominant_direction": "left", "replay_direction": "right",
        "target_direction_count": 254, "replay_unique_member_count": 73,
        "base_replay_copies_per_member": 2, "rotating_extra_count": 35,
        "replay_occurrence_count": 181, "replay_domain": "carla",
        "base_occurrences_sha256": EXPECTED_BASE_OCCURRENCES_SHA256}
    require(type(cohort) is dict and set(cohort) == set(scalar) | {
            "base_occurrences", "direction_memberships"}, "cohort replay fields differ")
    for key, expected in scalar.items(): require(cohort.get(key) == expected, "cohort replay differs: " + key)
    base, memberships = cohort["base_occurrences"], cohort["direction_memberships"]
    require(type(base) is list and len(base) == 1475 and type(memberships) is dict
            and tuple(memberships) == ("left", "right"), "cohort occurrence counts differ")
    require(canonical_sha256(base) == EXPECTED_BASE_OCCURRENCES_SHA256, "base occurrence SHA differs")
    base_by_index = {}
    for index, row in enumerate(base):
        require(row == {"base_occurrence_index": index, "dataset_index": row.get("dataset_index"),
                "sample_token": row.get("sample_token"), "domain": "carla"}, "base occurrence differs")
        require(row["dataset_index"] == index and type(row["sample_token"]) is str and row["sample_token"],
                "base occurrence identity differs")
        base_by_index[index] = row["sample_token"]
    seen = set()
    for direction, expected_count in (("left", 254), ("right", 73)):
        rows = memberships[direction]
        require(type(rows) is list and len(rows) == expected_count
                and canonical_sha256(rows) == EXPECTED_DIRECTION_MEMBERSHIP_SHA256[direction],
                "direction membership count/SHA differs")
        for index, row in enumerate(rows):
            require(row == {"membership_index": index, "dataset_index": row.get("dataset_index"),
                    "sample_token": row.get("sample_token"), "domain": "carla", "direction": direction},
                    "direction membership row differs")
            require(row["dataset_index"] not in seen
                    and base_by_index.get(row["dataset_index"]) == row["sample_token"],
                    "direction membership is not a unique base member")
            seen.add(row["dataset_index"])
    require(canonical_sha256(value) == EXPECTED_SAMPLING_PLAN_SHA256,
            "sampling plan sealed SHA differs")
    return value


def validate_execution_schedule(value, sampling_plan):
    require(type(value) is dict and set(value) == {"schema", "epoch_count", "epoch_sample_count",
            "row_count", "intra_batch_collision_count", "hash_encoding", "sha256", "rows"},
            "execution schedule fields differ")
    require(value["schema"] == "portable_e2e.cohort_replay_execution_schedule.v1"
            and value["epoch_count"] == 20 and value["epoch_sample_count"] == 1656
            and value["row_count"] == 33120 and value["intra_batch_collision_count"] == 0
            and value["hash_encoding"] == "sha256_canonical_json_v1",
            "execution schedule accounting differs")
    rows = value["rows"]
    require(type(rows) is list and len(rows) == 33120 and value["sha256"] == canonical_sha256(rows)
            and value["sha256"] == EXPECTED_EXECUTION_SCHEDULE_SHA256,
            "execution schedule SHA differs")
    base_count = sampling_plan["cohort_replay"]["base_sample_count"]
    replay_count = sampling_plan["cohort_replay"]["replay_occurrence_count"]
    base_dataset = [row["dataset_index"] for row in sampling_plan["cohort_replay"]["base_occurrences"]]
    right_dataset = [row["dataset_index"]
        for row in sampling_plan["cohort_replay"]["direction_memberships"]["right"]]
    observed_streams = {"base": 0, "replay": 0}
    epoch_occurrences = {"base": set(), "replay": set()}
    for offset, row in enumerate(rows):
        epoch, position = divmod(offset, 1656)
        if position == 0:
            epoch_occurrences = {"base": set(), "replay": set()}
            start = (epoch * 35) % 73
            replay_dataset = right_dataset * 2 + [right_dataset[(start + index) % 73] for index in range(35)]
        require(row == {"epoch": epoch, "position": position, "stream": row.get("stream"),
                "occurrence_index": row.get("occurrence_index"), "dataset_index": row.get("dataset_index")},
                "execution schedule row fields/order differ")
        stream = row["stream"]
        require(stream in observed_streams and type(row["occurrence_index"]) is int
                and 0 <= row["occurrence_index"] < (base_count if stream == "base" else replay_count)
                and type(row["dataset_index"]) is int and 0 <= row["dataset_index"] < 1475,
                "execution schedule occurrence differs")
        occurrence_dataset = base_dataset if stream == "base" else replay_dataset
        require(row["dataset_index"] == occurrence_dataset[row["occurrence_index"]]
                and row["occurrence_index"] not in epoch_occurrences[stream],
                "execution schedule occurrence identity/reuse differs")
        epoch_occurrences[stream].add(row["occurrence_index"])
        observed_streams[stream] += 1
        if position == 1655:
            require(epoch_occurrences == {"base": set(range(base_count)), "replay": set(range(replay_count))},
                    "execution schedule epoch does not cover each occurrence once")
        batch_start = offset - (position % 4)
        if position % 4 == 3 or position == 1655:
            batch = rows[batch_start:offset + 1]
            require(len({item["dataset_index"] for item in batch}) == len(batch),
                    "execution schedule contains an intra-batch duplicate")
    require(observed_streams == EXPECTED_STREAM_SAMPLES, "execution schedule stream totals differ")
    return value


def verify_dataset(plan, workspace, repo, view_report):
    report = read(view_report); require(digest(view_report) == plan["view_report_sha256"], "view report SHA differs")
    declared = _view_dataset(report, workspace); require(declared == plan["arm"]["dataset"], "view dataset differs")
    root = resolve_dataset(workspace, declared["path"]); preflight_metadata(root, declared)
    verify_tree(root, declared["tree"]["files"])
    observed = probe_dataset(workspace, repo, root)
    for split in ("train", "val"):
        require(observed[split] == {"count": declared["counts"][split],
            "loader_sha256": declared["split_loader_sha256"][split],
            "tensor_sha256": declared["split_fingerprint_sha256"][split],
            "examples_sha256": declared["split_examples_sha256"][split],
            "tokens_sha256": declared["split_tokens_sha256"][split],
            "episode_ids": declared["episode_ids"][split],
            "corpus_fingerprint_sha256": declared["corpus_fingerprint_sha256"]},
            "live loader proof differs: " + split)
    validate_sampling_plan(observed["sampling_plan"])
    validate_execution_schedule(observed["execution_schedule"], observed["sampling_plan"])
    require(observed["sampling_plan"] == plan["arm"]["sampling_plan"]
            and observed["execution_schedule"] == plan["arm"]["execution_schedule"],
            "live sampling plan/schedule differs")
    return root


def _reference_geometry(gate):
    geometry = gate.get("geometry", {}); selected = geometry.get("selected_result", {})
    result = {"selected": selected.get("geometry_pass_count"),
        "any_candidate": geometry.get("any_candidate_geometry_pass_count"),
        "all_candidates": geometry.get("all_candidates_geometry_pass_count")}
    require(geometry.get("sample_count") == selected.get("sample_count") == 665
            and result == REFERENCE_GEOMETRY, "reference C geometry differs")
    return result


def _episode_rows(summary):
    raw = summary.get("episodes"); rows = list(raw.values()) if isinstance(raw, dict) else raw
    require(type(rows) is list, "subgroup episodes missing")
    result = {}
    for row in rows:
        label = next((x["label"] for x in VALIDATION_EPISODES if x["episode_id"] == row.get("episode_id")), None)
        require(label and row.get("count") == next(x["count"] for x in VALIDATION_EPISODES if x["label"] == label),
                "subgroup episode identity/count differs")
        result[label] = row
    require(set(result) == {x["label"] for x in VALIDATION_EPISODES}, "subgroup episode set differs")
    return result


def reference_binding(reference_root, workspace):
    reference_root = canonical(reference_root); workspace = canonical(workspace)
    require(reference_root.is_dir() and reference_root.is_relative_to(workspace / "runs/diagnostics"),
            "C reference must be an existing personal diagnostic run")
    files = {name: {"path": path, "sha256": digest(reference_root / path)}
             for name, path in REFERENCE_FILES.items()}
    plan = read(reference_root / REFERENCE_FILES["plan"]); summary = read(reference_root / REFERENCE_FILES["summary"])
    subgroup = read(reference_root / REFERENCE_FILES["subgroup"]); gate = read(reference_root / REFERENCE_FILES["gate"])
    training = read(reference_root / REFERENCE_FILES["training_run"])
    require(plan.get("campaign_id") == "hh260921-moving-data-high-budget-ab-seed20260906-v1"
            and plan.get("source_commit") == BASE_COMMIT, "reference C plan identity differs")
    require(summary.get("status") == "COMPLETE_NOT_PROMOTED"
            and summary.get("plan_sha256") == files["plan"]["sha256"], "reference C summary differs")
    require(subgroup.get("arm") == "A_baseline_train" and subgroup.get("combined_val_count") == 665,
            "reference C subgroup differs")
    require(training.get("state") == {"epoch": 25, "next_batch_index": 205, "global_step": 7380,
            "samples_seen": 29495, "domain_samples_seen": {"carla": 29495}}, "reference C train state differs")
    require(files["checkpoint"]["sha256"] == subgroup.get("checkpoint_sha256")
            == gate.get("checkpoint_sha256") == training.get("last_metrics", {}).get("checkpoint_sha256", files["checkpoint"]["sha256"]),
            "reference C checkpoint binding differs")
    rows = _episode_rows(subgroup)
    for label, metrics in REFERENCE_METRICS.items():
        require(all(rows[label].get("metrics", {}).get(key) == value for key, value in metrics.items()),
                "reference C metrics differ: " + label)
    _reference_geometry(gate)
    return {"root": reference_root.relative_to(workspace).as_posix(),
        "campaign_id": plan["campaign_id"], "arm": "A_baseline_train", "files": files,
        "metrics": REFERENCE_METRICS, "geometry_pass_counts": REFERENCE_GEOMETRY}


def verify_reference(plan, workspace):
    binding = plan["reference_C"]; root = canonical(workspace / relative(binding["root"]))
    observed = reference_binding(root, workspace)
    require(observed == binding, "reference C evidence differs")
    return root


def reference_D_binding(reference_root, workspace):
    reference_root = canonical(reference_root); workspace = canonical(workspace)
    require(reference_root.is_dir() and reference_root.is_relative_to(workspace / "runs/diagnostics"),
            "D reference must be an existing personal diagnostic run")
    files = {name: {"path": path, "sha256": digest(reference_root / path)}
             for name, path in REFERENCE_D_FILES.items()}
    subgroup = read(reference_root / REFERENCE_D_FILES["subgroup"])
    gate = read(reference_root / REFERENCE_D_FILES["gate"])
    training = read(reference_root / REFERENCE_D_FILES["training_run"])
    evaluation = read(reference_root / REFERENCE_D_FILES["evaluation"])
    require(subgroup.get("arm") == "B_town07_augmented_train"
            and subgroup.get("combined_val_count") == 665, "reference D subgroup differs")
    require(training.get("state") == {"epoch": 20, "next_batch_index": 0, "global_step": 7380,
            "samples_seen": 29500, "domain_samples_seen": {"carla": 29500}},
            "reference D train state differs")
    checkpoint_sha = files["checkpoint"]["sha256"]
    require(checkpoint_sha == subgroup.get("checkpoint_sha256") == gate.get("checkpoint_sha256")
            == evaluation.get("checkpoint_sha256"), "reference D checkpoint binding differs")
    rows = _episode_rows(subgroup)
    straight = rows["town03_straight_dev"]
    require(all(straight.get("metrics", {}).get(key) == value
            for key, value in REFERENCE_D_METRICS["town03_straight_dev"].items()),
            "reference D straight metrics differ")
    require(evaluation.get("sample_count") == 665
            and all(evaluation.get("metrics", {}).get(key) == value
                for key, value in REFERENCE_D_METRICS["combined_val"].items()),
            "reference D combined metrics differ")
    _reference_geometry(gate)
    return {"root": reference_root.relative_to(workspace).as_posix(),
        "campaign_id": "hh260921-moving-data-high-budget-ab-seed20260906-v1",
        "arm": "B_town07_augmented_train", "files": files,
        "metrics": REFERENCE_D_METRICS, "geometry_pass_counts": REFERENCE_GEOMETRY}


def verify_reference_D(plan, workspace):
    binding = plan["reference_D"]; root = canonical(workspace / relative(binding["root"]))
    observed = reference_D_binding(root, workspace)
    require(observed == binding, "reference D evidence differs")
    return root


def declare_plan(*, repo_root, workspace, view_report, reference_root, evaluator, output_plan, runner=None):
    repo = canonical(repo_root); workspace = canonical(workspace); view_report = regular(view_report)
    evaluator = regular(evaluator); runner = regular(runner or __file__)
    dataset = _view_dataset(read(view_report), workspace); root = resolve_dataset(workspace, dataset["path"])
    preflight_metadata(root, dataset); observed = probe_dataset(workspace, repo, root)
    validate_sampling_plan(observed["sampling_plan"])
    validate_execution_schedule(observed["execution_schedule"], observed["sampling_plan"])
    for split in ("train", "val"):
        require(observed[split]["examples_sha256"] == dataset["split_examples_sha256"][split]
                and observed[split]["tensor_sha256"] == dataset["split_fingerprint_sha256"][split],
                "declaration live dataset differs")
    validation = {"episodes": list(VALIDATION_EPISODES), "count": 665,
        "fingerprint_sha256": dataset["split_examples_sha256"]["val"],
        "tokens_sha256": dataset["split_tokens_sha256"]["val"]}
    plan = {"schema": PLAN_SCHEMA, "campaign_id": CAMPAIGN_ID, "declared_at_utc": utc(),
        "base_commit": BASE_COMMIT, "gpu": dict(GPU), "source": source_identity(repo),
        "runner_sha256": digest(runner), "evaluator_sha256": digest(evaluator),
        "owner": {"path": OWNER_PATH, "sha256": OWNER_SHA256},
        "model_config": MODEL_PATH, "model_config_sha256": MODEL_SHA256,
        "model_config_canonical_sha256": MODEL_CANONICAL_SHA256, "model_id": MODEL_ID,
        "view_report_sha256": digest(view_report), "output": OUTPUT_PATH,
        "arm": {"label": ARM, "dataset_role": VIEW_ARM, "dataset": dataset,
            "expected_train_state": EXPECTED_TRAIN_STATE, "expected_sampling_stream_samples_seen": EXPECTED_STREAM_SAMPLES,
            "train_config": TRAIN_CONFIG, "sampling_plan": observed["sampling_plan"],
            "sampling_plan_sha256": canonical_sha256(observed["sampling_plan"]),
            "execution_schedule": observed["execution_schedule"]},
        "validation": validation, "reference_C": reference_binding(reference_root, workspace),
        "reference_D": reference_D_binding(reference_root, workspace),
        "train_config": TRAIN_CONFIG, "loss_config": LOSS_CONFIG, "stage_order": list(STAGES),
        "stage_timeout_seconds": TIMEOUTS, "whole_timeout_seconds": WHOLE_TIMEOUT_SECONDS,
        "decision": DECISION, **DENIALS}
    validate_plan(plan, runner_sha256=digest(runner)); write_new(output_plan, plan); return plan


def validate_plan(plan, *, runner_sha256, observed=None):
    require(type(plan) is dict, "plan object required")
    fixed = {"schema": PLAN_SCHEMA, "campaign_id": CAMPAIGN_ID, "base_commit": BASE_COMMIT,
        "gpu": GPU, "owner": {"path": OWNER_PATH, "sha256": OWNER_SHA256},
        "model_config": MODEL_PATH, "model_config_sha256": MODEL_SHA256,
        "model_config_canonical_sha256": MODEL_CANONICAL_SHA256, "model_id": MODEL_ID,
        "output": OUTPUT_PATH, "train_config": TRAIN_CONFIG, "loss_config": LOSS_CONFIG,
        "stage_order": list(STAGES), "stage_timeout_seconds": TIMEOUTS,
        "whole_timeout_seconds": WHOLE_TIMEOUT_SECONDS, "decision": DECISION, **DENIALS}
    variable = {"declared_at_utc", "source", "runner_sha256", "evaluator_sha256",
        "view_report_sha256", "arm", "validation", "reference_C", "reference_D"}
    require(set(plan) == set(fixed) | variable, "plan fields differ")
    for key, value in fixed.items(): require(plan.get(key) == value, "fixed plan field differs: " + key)
    require(plan["runner_sha256"] == runner_sha256, "runner SHA differs")
    for key in ("runner_sha256", "evaluator_sha256", "view_report_sha256"): sha256(plan[key], key)
    require(timestamp(plan["declared_at_utc"]) <= (observed or datetime.now(timezone.utc)), "future declaration forbidden")
    source = plan["source"]
    require(type(source) is dict and set(source) == {"base_commit", "base_sha256", "current_sha256",
            "overlay_paths", "added_paths", "added_sha256"}
            and source["base_commit"] == BASE_COMMIT and source["overlay_paths"] == list(OVERLAY_PATHS)
            and source["added_paths"] == list(ADDED_SOURCE_PATHS),
            "source overlay contract differs")
    require(set(source["base_sha256"]) == set(source["current_sha256"]) == set(BASE_SOURCE_PATHS)
            and set(source["added_sha256"]) == set(ADDED_SOURCE_PATHS), "source closure differs")
    for name in BASE_SOURCE_PATHS:
        sha256(source["base_sha256"][name]); sha256(source["current_sha256"][name])
        require((source["base_sha256"][name] != source["current_sha256"][name]) == (name in OVERLAY_PATHS),
                "overlay byte distinction differs: " + name)
    for name in ADDED_SOURCE_PATHS: sha256(source["added_sha256"][name], "added source SHA")
    arm = plan["arm"]
    require(set(arm) == {"label", "dataset_role", "dataset", "expected_train_state",
            "expected_sampling_stream_samples_seen", "train_config", "sampling_plan", "sampling_plan_sha256",
            "execution_schedule"}
            and arm["label"] == ARM and arm["dataset_role"] == VIEW_ARM
            and arm["expected_train_state"] == EXPECTED_TRAIN_STATE
            and arm["expected_sampling_stream_samples_seen"] == EXPECTED_STREAM_SAMPLES
            and arm["train_config"] == TRAIN_CONFIG, "arm contract differs")
    data = arm["dataset"]
    require(type(data) is dict and data.get("counts") == {"train": 1475, "val": 665}, "dataset counts differ")
    relative(data.get("path")); require(data["path"].startswith("datasets/prepared/"), "prepared dataset required")
    for key in ("manifest_sha256", "corpus_fingerprint_sha256"): sha256(data.get(key), key)
    for field in ("split_fingerprint_sha256", "split_loader_sha256", "split_examples_sha256", "split_tokens_sha256"):
        require(set(data.get(field, {})) == {"train", "val"}, field + " differs")
        for value in data[field].values(): sha256(value)
    require(data["episode_ids"]["val"] == [x["episode_id"] for x in VALIDATION_EPISODES]
            and len(data["episode_ids"]["train"]) == 4, "dataset episode IDs differ")
    require(type(data.get("tree", {}).get("files")) is list
            and len(data["tree"]["files"]) == data["tree"].get("file_count")
            and len({row.get("path") for row in data["tree"]["files"]}) == data["tree"]["file_count"],
            "dataset tree inventory differs")
    validate_sampling_plan(arm["sampling_plan"])
    require(arm["sampling_plan_sha256"] == canonical_sha256(arm["sampling_plan"]), "sampling plan SHA differs")
    validate_execution_schedule(arm["execution_schedule"], arm["sampling_plan"])
    expected_validation = {"episodes": list(VALIDATION_EPISODES), "count": 665,
        "fingerprint_sha256": data["split_examples_sha256"]["val"],
        "tokens_sha256": data["split_tokens_sha256"]["val"]}
    require(plan["validation"] == expected_validation, "validation contract differs")
    reference = plan["reference_C"]
    require(type(reference) is dict and set(reference) == {"root", "campaign_id", "arm", "files", "metrics", "geometry_pass_counts"}
            and reference["campaign_id"] == "hh260921-moving-data-high-budget-ab-seed20260906-v1"
            and reference["arm"] == "A_baseline_train" and reference["metrics"] == REFERENCE_METRICS
            and reference["geometry_pass_counts"] == REFERENCE_GEOMETRY, "reference C contract differs")
    relative(reference["root"], "reference root")
    require(set(reference["files"]) == set(REFERENCE_FILES), "reference file bindings differ")
    for name, path in REFERENCE_FILES.items():
        require(reference["files"][name]["path"] == path, "reference file path differs")
        sha256(reference["files"][name]["sha256"], "reference file SHA")
    reference_D = plan["reference_D"]
    require(type(reference_D) is dict and set(reference_D) == {
            "root", "campaign_id", "arm", "files", "metrics", "geometry_pass_counts"}
            and reference_D["campaign_id"] == "hh260921-moving-data-high-budget-ab-seed20260906-v1"
            and reference_D["arm"] == "B_town07_augmented_train"
            and reference_D["metrics"] == REFERENCE_D_METRICS
            and reference_D["geometry_pass_counts"] == REFERENCE_GEOMETRY,
            "reference D contract differs")
    relative(reference_D["root"], "reference D root")
    require(set(reference_D["files"]) == set(REFERENCE_D_FILES), "reference D file bindings differ")
    for name, path in REFERENCE_D_FILES.items():
        require(reference_D["files"][name]["path"] == path, "reference D file path differs")
        sha256(reference_D["files"][name]["sha256"], "reference D file SHA")
    return plan


def load_module(path, expected, name):
    path = regular(path); require(digest(path) == expected, "helper SHA differs")
    spec = importlib.util.spec_from_file_location(name, path); require(spec and spec.loader, "helper spec failed")
    module = importlib.util.module_from_spec(spec); sys.modules[name] = module; spec.loader.exec_module(module)
    require(digest(path) == expected, "helper changed during import"); return module


def stage_command(plan, workspace, repo, output, plan_path, evaluator, runner, stage):
    item = output / ARM; data = str(workspace / plan["arm"]["dataset"]["path"])
    python = str(workspace / "venvs/py312/bin/python"); checkpoint = item / "training/checkpoints/latest.pt"
    if stage == "train":
        return [python, "-m", "portable_e2e.train", data, "--run-dir", str(item / "training"),
            "--model-config", str(repo / MODEL_PATH), "--split", "train", "--device", "cuda:0",
            "--batch-size", "4", "--seed", "20260906", "--learning-rate", "0.0001", "--weight-decay", "0.0001",
            "--candidate-score-weight", "0.1", "--candidate-regret-weight", "0.1", "--max-steps", "8280",
            "--checkpoint-interval", "414", "--num-workers", "0", "--maximum-gradient-norm", "5",
            "--sampling-policy", "direction_balanced_cohort_replay"]
    if stage == "evaluate":
        return [python, "-m", "portable_e2e.evaluate", data, "--checkpoint", str(checkpoint),
            "--output-dir", str(item / "evaluation"), "--split", "val", "--device", "cuda:0",
            "--batch-size", "4", "--num-workers", "0", "--render-count", "12"]
    if stage == "audit":
        return [python, "-m", "portable_e2e.audit_runtime", data, "--checkpoint", str(checkpoint),
            "--checkpoint-sha256", digest(checkpoint), "--output-json", str(item / "gate_v8.json"),
            "--split", "val", "--device", "cuda:0", "--batch-size", "4"]
    require(stage == "subgroup", "unknown stage")
    return [python, str(evaluator), "--repo-root", str(repo), "--workspace", str(workspace),
        "--runner", str(runner),
        "--runner-sha256", digest(runner), "--plan", str(plan_path), "--plan-sha256", digest(plan_path),
        "--checkpoint", str(checkpoint), "--checkpoint-sha256", digest(checkpoint),
        "--output-dir", str(item / "subgroup")]


def verify_history(path):
    count = 0; last = None
    with regular(path).open() as stream:
        for line in stream:
            count += 1; last = decode(line); require(last.get("global_step") == count, "history step differs")
    require(count == 8280 and last and last.get("samples_seen") == 33120
            and last.get("domain_samples_seen") == {"carla": 33120}
            and last.get("sampling_stream_samples_seen") == EXPECTED_STREAM_SAMPLES,
            "history endpoint differs")
    return digest(path)


def verify_stage(plan, output, stage):
    item = output / ARM; checkpoint = item / "training/checkpoints/latest.pt"; checkpoint_sha = digest(checkpoint)
    paths = {"train": item / "training/run.json", "evaluate": item / "evaluation/metrics.json",
        "audit": item / "gate_v8.json", "subgroup": item / "subgroup/summary.json"}
    path = paths[stage]; report = read(path); data = plan["arm"]["dataset"]
    model_config = read(output / "source" / MODEL_PATH)
    require(digest(output / "source" / MODEL_PATH) == MODEL_SHA256, "archived model config differs")
    if stage == "train":
        require(report.get("status") == "TRAINING_TARGET_REACHED"
                and report.get("warning") == "RESEARCH CHECKPOINT — NOT APPROVED FOR VEHICLE CONTROL"
                and "resumed_at_utc" not in report, "fresh training completion required")
        require(report.get("device") == "cuda:0"
                and report.get("hardware", {}).get("device_uuid") == GPU["uuid"].removeprefix("GPU-"), "GPU differs")
        require(report.get("dataset_size") == 1475
                and report.get("dataset_fingerprint_sha256") == data["split_fingerprint_sha256"]["train"]
                and report.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"], "TRAIN data differs")
        require(report.get("training_episode_ids") == sorted(data["episode_ids"]["train"])
                and report.get("state") == EXPECTED_TRAIN_STATE and report.get("train_config") == TRAIN_CONFIG
                and report.get("loss_config") == LOSS_CONFIG and report.get("model_config") == model_config
                and report.get("model_parameter_count") == MODEL_PARAMETER_COUNT,
                "training state/config differs")
        require(report.get("sampling_plan") == plan["arm"]["sampling_plan"]
                and report.get("sampling_plan_sha256") == plan["arm"]["sampling_plan_sha256"]
                and report.get("sampling_execution_schedule_sha256") == plan["arm"]["execution_schedule"]["sha256"]
                and report.get("sampling_execution_schedule_row_count") == 33120
                and report.get("sampling_metrics", {}).get("sampling_stream_samples_seen") == EXPECTED_STREAM_SAMPLES
                and report.get("last_metrics", {}).get("sampling_stream_samples_seen") == EXPECTED_STREAM_SAMPLES,
                "training replay accounting differs")
        return {"path": path.relative_to(output).as_posix(), "sha256": digest(path),
            "checkpoint_sha256": checkpoint_sha, "history_sha256": verify_history(item / "training/metrics.jsonl"),
            "sampling_execution_schedule_row_count": 33120,
            "sampling_execution_schedule_sha256": plan["arm"]["execution_schedule"]["sha256"]}
    require(report.get("checkpoint_sha256") == checkpoint_sha
            and report.get("corpus_fingerprint_sha256") == data["corpus_fingerprint_sha256"], "stage checkpoint/corpus differs")
    require(report.get("vehicle_control_approved") is False, "stage cannot approve vehicle control")
    if stage in ("evaluate", "audit"):
        count = report.get("sample_count") if stage == "evaluate" else report.get("geometry", {}).get("sample_count")
        require(report.get("evaluation_split") == "val" and count == 665
                and report.get("evaluation_episode_count") == 2
                and report.get("dataset_fingerprint_sha256") == data["split_fingerprint_sha256"]["val"]
                and report.get("training_dataset_fingerprint_sha256") == data["split_fingerprint_sha256"]["train"]
                and report.get("training_sampling_plan_sha256") == plan["arm"]["sampling_plan_sha256"]
                and report.get("training_sampling_policy") == "direction_balanced_cohort_replay"
                and report.get("model_config_sha256") == MODEL_CANONICAL_SHA256
                and report.get("model_parameter_count") == MODEL_PARAMETER_COUNT, "evaluation/audit provenance differs")
        if stage == "evaluate":
            require(report.get("status") == "OPEN_LOOP_EVALUATION_COMPLETE"
                    and report.get("training_sampling_stream_samples_seen") == EXPECTED_STREAM_SAMPLES
                    and report.get("loss_config") == LOSS_CONFIG
                    and report.get("auxiliary_loss_metric_counts") == {"candidate_regret_loss": 665}
                    and all(type(value) in (int, float) and math.isfinite(value)
                            for value in report.get("metrics", {}).values()), "evaluation metrics differ")
        else:
            require(report.get("status") == "RUNTIME_GEOMETRY_AUDIT_COMPLETE"
                    and report.get("gate", {}).get("source") == "portable_e2e.runtime_geometry_gate.v8"
                    and report["gate"].get("threshold_overrides") is False, "runtime audit differs")
    else:
        require(report.get("status") == "COMPLETE_NOT_PROMOTED" and report.get("arm") == ARM
                and report.get("combined_val_count") == 665
                and report.get("plan_sha256") == digest(output / "plan.json")
                and report.get("runner_sha256") == plan["runner_sha256"]
                and report.get("evaluator_sha256") == plan["evaluator_sha256"]
                and report.get("checkpoint", {}).get("sampling_plan_sha256") == plan["arm"]["sampling_plan_sha256"],
                "subgroup summary differs")
    return {"path": path.relative_to(output).as_posix(), "sha256": digest(path), "checkpoint_sha256": checkpoint_sha}


def diagnostic_gate(subgroup, evaluation, audit):
    # HH_260906 - Require the C right-turn and D straight/combined Pareto envelope before any new head work.
    rows = _episode_rows(subgroup); checks = {}
    for episode in VALIDATION_EPISODES:
        label, count = episode["label"], episode["count"]; row = rows[label]; checks[label] = {}
        reference_metrics = (REFERENCE_METRICS[label]
            if label == "town03_right_turn_dev" else REFERENCE_D_METRICS[label])
        reference_arm = "C" if label == "town03_right_turn_dev" else "D"
        for key, maximum in MAXIMUM_F[label].items():
            value = row.get("metrics", {}).get(key)
            require(row.get("metric_counts", {}).get(key) == count
                    and type(value) in (int, float) and math.isfinite(value) and value >= 0,
                    "diagnostic metric evidence incomplete")
            checks[label][key] = {"reference_arm": reference_arm,
                "reference": reference_metrics[key], "maximum_F": maximum,
                "observed_F": value, "pass": value <= maximum}
    combined_checks = {}
    require(evaluation.get("sample_count") == 665, "combined evaluation count differs")
    for key, maximum in MAXIMUM_F["combined_val"].items():
        value = evaluation.get("metrics", {}).get(key)
        require(evaluation.get("metric_counts", {}).get(key) == 665
                and type(value) in (int, float) and math.isfinite(value) and value >= 0,
                "combined diagnostic metric evidence incomplete")
        combined_checks[key] = {"reference_arm": "D",
            "reference": REFERENCE_D_METRICS["combined_val"][key],
            "maximum_F": maximum, "observed_F": value, "pass": value <= maximum}
    geometry = audit.get("geometry", {}); selected = geometry.get("selected_result", {})
    observed = {"selected": selected.get("geometry_pass_count"),
        "any_candidate": geometry.get("any_candidate_geometry_pass_count"),
        "all_candidates": geometry.get("all_candidates_geometry_pass_count")}
    require(geometry.get("sample_count") == selected.get("sample_count") == 665
            and all(type(value) is int and 0 <= value <= 665 for value in observed.values()),
            "diagnostic geometry evidence incomplete")
    geometry_checks = {key: {"minimum": REFERENCE_GEOMETRY[key], "observed_F": value,
        "pass": value >= REFERENCE_GEOMETRY[key]} for key, value in observed.items()}
    passed = all(x["pass"] for group in checks.values() for x in group.values()) \
        and all(x["pass"] for x in combined_checks.values()) \
        and all(x["pass"] for x in geometry_checks.values())
    return {"status": "PASS_SINGLE_SEED_DIAGNOSTIC" if passed else "FAIL_SINGLE_SEED_DIAGNOSTIC",
        "pass": passed, "episode_metric_checks": checks,
        "combined_metric_checks": combined_checks,
        "geometry": {"sample_count": 665, "checks": geometry_checks,
            "pass": all(x["pass"] for x in geometry_checks.values())},
        "next_action": "run_unseen_confirmation_seeds_20260907_20260908_20260909" if passed
            else "do_not_confirm_or_promote_revise_training_data_or_sampling",
        "adoption_allowed": False, "automatic_promotion": False}


def _archive(plan, output, repo, runner, evaluator, owner, view_report, reference_root):
    archive = output / "source"
    for name, expected in plan["source"]["current_sha256"].items(): copy_new(repo / name, archive / name, expected)
    for name, expected in plan["source"]["added_sha256"].items(): copy_new(repo / name, archive / name, expected)
    copy_new(runner, archive / Path(runner).name, plan["runner_sha256"])
    copy_new(evaluator, archive / Path(evaluator).name, plan["evaluator_sha256"])
    copy_new(owner, archive / "profile_portable_training.py", OWNER_SHA256)
    copy_new(view_report, output / "view_report.json", plan["view_report_sha256"])
    for name in ("plan", "summary", "training_run", "subgroup", "gate"):
        row = plan["reference_C"]["files"][name]
        copy_new(reference_root / row["path"], output / "reference_C" / row["path"], row["sha256"])
    for row in plan["reference_D"]["files"].values():
        copy_new(reference_root / row["path"], output / "reference_D" / row["path"], row["sha256"])


def run(*, plan_path, repo_root, workspace, view_report, reference_root, evaluator, runner=None):
    runner = regular(runner or __file__); repo = canonical(repo_root); workspace = canonical(workspace)
    plan_path = regular(plan_path); view_report = regular(view_report); evaluator = regular(evaluator)
    require(not runner.is_relative_to(repo) and not evaluator.is_relative_to(repo),
            "execution requires archived runner/evaluator outside the overlay worktree")
    plan_sha = digest(plan_path); plan = validate_plan(read(plan_path), runner_sha256=digest(runner))
    require(digest(evaluator) == plan["evaluator_sha256"] and digest(view_report) == plan["view_report_sha256"],
            "execution input differs")
    output = workspace / plan["output"]
    require(output == workspace / OUTPUT_PATH and not output.exists() and not output.is_symlink(), "fresh output required")
    verify_sources(plan, repo); verify_dataset(plan, workspace, repo, view_report)
    expected_reference = canonical(workspace / plan["reference_C"]["root"])
    require(canonical(reference_root) == expected_reference, "reference C root argument differs")
    verify_reference(plan, workspace); verify_reference_D(plan, workspace)
    owner_path = regular(workspace / OWNER_PATH); require(digest(owner_path) == OWNER_SHA256, "GPU owner differs")
    pipeline_path = regular(repo / PIPELINE_PATH)
    owner = load_module(owner_path, OWNER_SHA256, "_turn_replay_gpu_owner")
    pipeline = load_module(pipeline_path, plan["source"]["current_sha256"][PIPELINE_PATH], "_turn_replay_stage_owner")
    owner.check_environment(workspace, repo)
    with owner.gpu_lease(workspace):
        idle = owner.gpu_idle(repo); output.mkdir(parents=True, exist_ok=False)
        copy_new(plan_path, output / "plan.json", plan_sha)
        _archive(plan, output, repo, runner, evaluator, owner_path, view_report, expected_reference)
        status = {"schema": SUMMARY_SCHEMA, "status": "RUNNING", "started_at_utc": utc(),
            "plan_sha256": plan_sha, "pre_cuda_gpu": idle, "arm": "NOT_RUN", "stages": [], **DENIALS}
        write_new(output / "started.json", status); started = time.monotonic()
        try:
            with owner.bounded_signals(WHOLE_TIMEOUT_SECONDS):
                item = output / ARM; item.mkdir(); status["arm"] = "RUNNING"; checkpoint_sha = None
                for stage in STAGES:
                    require(time.monotonic() - started + TIMEOUTS[stage] + 30 < WHOLE_TIMEOUT_SECONDS,
                            "insufficient whole-run reserve")
                    verify_sources(plan, repo); verify_dataset(plan, workspace, repo, view_report)
                    verify_reference(plan, workspace); verify_reference_D(plan, workspace)
                    require(digest(runner) == plan["runner_sha256"] and digest(evaluator) == plan["evaluator_sha256"]
                            and digest(plan_path) == plan_sha, "runner/evaluator/plan changed")
                    if checkpoint_sha is not None:
                        require(digest(item / "training/checkpoints/latest.pt") == checkpoint_sha, "checkpoint changed")
                    args = stage_command(plan, workspace, repo, output, output / "plan.json", evaluator, runner, stage)
                    receipt = {"arm": ARM, "stage": stage, "command": args, "timeout_seconds": TIMEOUTS[stage],
                        "started_at_utc": utc(), "status": "RUNNING", "owner_pid": os.getpid(),
                        "plan_sha256": plan_sha, "input_checkpoint_sha256": checkpoint_sha,
                        "pre_cuda_gpu": owner.gpu_idle(repo)}
                    write_new(item / (stage + "_started.json"), receipt)
                    env = dict(os.environ, PORTABLE_MOVING_TURN_REPLAY_OWNER_PID=str(os.getpid()))
                    with (item / (stage + ".log")).open("xb") as log:
                        rc = pipeline.run_owned_stage(args, repo, env, log, timeout=TIMEOUTS[stage])
                    require(rc == 0, ARM + " " + stage + " exited " + str(rc))
                    proof = verify_stage(plan, output, stage); checkpoint_sha = proof["checkpoint_sha256"]
                    if receipt["input_checkpoint_sha256"] is not None:
                        require(checkpoint_sha == receipt["input_checkpoint_sha256"],
                                "read-only stage changed checkpoint")
                    receipt.update(status="COMPLETE", ended_at_utc=utc(), **proof)
                    write_new(item / (stage + "_completed.json"), receipt); status["stages"].append(receipt)
                gate = diagnostic_gate(read(item / "subgroup/summary.json"),
                    read(item / "evaluation/metrics.json"), read(item / "gate_v8.json"))
                status.update(status="COMPLETE_NOT_PROMOTED", arm="COMPLETE", diagnostic_gate=gate,
                    sampling_execution_schedule={"row_count": 33120,
                        "sha256": plan["arm"]["execution_schedule"]["sha256"]},
                    source_dataset_reference_postcheck_pass=True, ended_at_utc=utc(), final_gpu=owner.gpu_idle(repo))
                write_new(output / "summary.json", status)
        except BaseException as error:
            if status["arm"] == "RUNNING": status["arm"] = "FAIL"
            status.update(status="FAIL", error_type=type(error).__name__, error=str(error), ended_at_utc=utc())
            with owner.finalization_signals(): write_new(output / "failed.json", status)
            raise
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    sub = parser.add_subparsers(dest="mode", required=True)
    for name in ("declare", "run"):
        item = sub.add_parser(name, allow_abbrev=False)
        item.add_argument("--repo-root", type=Path, required=True); item.add_argument("--workspace", type=Path, required=True)
        item.add_argument("--view-report", type=Path, required=True); item.add_argument("--reference-root", type=Path, required=True)
        item.add_argument("--evaluator", type=Path, required=True)
    sub.choices["declare"].add_argument("--output-plan", type=Path, required=True)
    sub.choices["run"].add_argument("--plan", type=Path, required=True)
    args = parser.parse_args(argv)
    common = dict(repo_root=args.repo_root, workspace=args.workspace, view_report=args.view_report,
                  reference_root=args.reference_root, evaluator=args.evaluator)
    if args.mode == "declare": declare_plan(output_plan=args.output_plan, **common)
    else: run(plan_path=args.plan, **common)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
