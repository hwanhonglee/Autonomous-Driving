#!/usr/bin/env python3
"""HH_260906 - Run the create-only paired H selector sampling recalibration on F."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import traceback


GPU_UUID = "GPU-59f374a4-53f5-c050-34b2-56aab0e3c7e5"
F_CHECKPOINT_SHA256 = (
    "a551570aa6fc1697e9dcfd9fa714b2c1066f95e2194a5fefa9b0d9f3c20baf8c"
)
F_PLAN_SHA256 = "a2fdf6dcf66234c252d4289f2f979313902b8bbeb92f9796d07675e1b1eedc8d"
F_GATE_SHA256 = "3c75b78ec72f9e9f7d3851f62f6415feb514391fcb9b3a6d33859ebc8c7baa41"
OUTPUT_RELATIVE = (
    "runs/diagnostics/"
    "moving_selector_sampling_recalibration_seed20260906_20260921_v1"
)
REPO_RELATIVE = (
    "source_worktrees/"
    "moving_selector_recalibration_h_seed20260906_20260921_v1"
)
F_ROOT_RELATIVE = (
    "runs/diagnostics/"
    "moving_direction_balanced_replay_seed20260906_20260921_v1"
)
F_ARM = "F_direction_balanced_replay_train"
RIGHT_EPISODE = (
    "carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9"
)
STRAIGHT_EPISODE = (
    "carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320"
)
SOURCE_PATHS = (
    "portable_e2e/frozen_stopmix_selector_recalibration.py",
    "portable_e2e/frozen_stopmix_selector.py",
    "portable_e2e/frozen_stopmix_drive_rank.py",
    "portable_e2e/frozen_stopmix_residual.py",
    "portable_e2e/losses.py",
    "portable_e2e/model.py",
    "portable_e2e/dataset.py",
    "portable_e2e/torch_dataset.py",
    "portable_e2e/train.py",
    "portable_e2e/evaluate.py",
    "portable_e2e/audit_runtime.py",
    "portable_e2e/contract.py",
    "portable_e2e/stop_primitive_research.py",
    "scripts/e2e/diagnose_stopmix_training_oracles.py",
    "scripts/e2e/evaluate_portable_moving_data_subgroups.py",
    "scripts/e2e/run_moving_selector_sampling_recalibration.py",
)
RIGHT_MAXIMUM = {
    "ade_6p4s_m": 2.9353143748791353,
    "fde_6p4s_m": 7.793866114022117,
    "speed_mae_6p4s_mps": 1.1076154233086357,
}
STRAIGHT_MAXIMUM = {
    "ade_6p4s_m": 3.764740187947343,
    "fde_6p4s_m": 11.027441243572932,
    "speed_mae_6p4s_mps": 1.7877504960065935,
}
COMBINED_MAXIMUM = {
    "ade_6p4s_m": 3.5632455477589056,
    "fde_6p4s_m": 10.353536791729749,
    "speed_mae_6p4s_mps": 1.6350220532345592,
}
GEOMETRY_MINIMUM = {
    "selected_geometry_pass_count": 664,
    "any_candidate_geometry_pass_count": 664,
    "all_candidates_geometry_pass_count": 664,
}
DENIALS = {
    "automatic_promotion": False,
    "runtime_checkpoint_exported": False,
    "test_read_or_use": False,
    "closed_loop_evaluation": False,
    "all_map_claim": False,
    "vehicle_control_approved": False,
    "deployment_approved": False,
    "new_function_head_added": False,
}


def require(value, message):
    if not value:
        raise ValueError(message)


def utc():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def encoded(value):
    return (
        json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + "\n"
    ).encode()


def compact(value):
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode()


def sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def regular(path):
    path = Path(path).absolute()
    require(path.is_file() and not path.is_symlink(), "regular source file required")
    require(
        not any(parent.is_symlink() for parent in path.parents),
        "symlink parent is forbidden",
    )
    return path


def digest(path):
    value = hashlib.sha256()
    with regular(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def read_json(path):
    def pairs(values):
        result = {}
        for key, value in values:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result

    return json.loads(
        regular(path).read_text(),
        object_pairs_hook=pairs,
        parse_constant=lambda value: require(False, "nonfinite JSON " + value),
    )


def write_new(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(encoded(value))
        stream.flush()
        os.fsync(stream.fileno())


def save_new(path, value, torch):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        torch.save(value, stream)
        stream.flush()
        os.fsync(stream.fileno())


def copy_new(source, target):
    source = regular(source)
    target = Path(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as src, target.open("xb") as dst:
        shutil.copyfileobj(src, dst, 1024 * 1024)
        dst.flush()
        os.fsync(dst.fileno())
    require(digest(source) == digest(target), "archived source differs")


def tensor_digest(values):
    result = hashlib.sha256()
    for name, tensor in sorted(values.items()):
        value = tensor.detach().cpu().contiguous()
        header = compact([name, str(value.dtype), list(value.shape)])
        result.update(len(header).to_bytes(8, "big"))
        result.update(header)
        result.update(value.numpy().tobytes())
    return result.hexdigest()


def preflight_gpu():
    require(
        os.environ.get("CUDA_VISIBLE_DEVICES") == GPU_UUID,
        "CUDA_VISIBLE_DEVICES must bind only the approved physical GPU0 UUID",
    )
    query = subprocess.check_output(
        [
            "nvidia-smi",
            "--query-gpu=index,uuid,memory.used,utilization.gpu",
            "--format=csv,noheader,nounits",
        ],
        timeout=20,
        text=True,
    ).splitlines()
    rows = [row for row in query if GPU_UUID in row]
    require(len(rows) == 1 and rows[0].split(",")[0].strip() == "0", "GPU0 UUID differs")
    fields = [value.strip() for value in rows[0].split(",")]
    memory_used_mib = int(fields[2])
    utilization_percent = int(fields[3])
    require(
        memory_used_mib == 0 and utilization_percent == 0,
        "approved GPU0 must be completely idle before H starts",
    )
    return {
        "physical_index": int(fields[0]),
        "uuid": fields[1],
        "memory_used_mib_before_torch": memory_used_mib,
        "utilization_percent_before_torch": utilization_percent,
    }


def extract_cache(model, dataset, core, corpus_sha, device, torch):
    from torch.utils.data import DataLoader

    fields = (
        "fused",
        "candidate_xy",
        "candidate_speed",
        "target_xy",
        "target_speed_mps",
        "target_yaw_rad",
        "target_valid",
        "original_logits",
    )
    inputs = (
        "images",
        "calibration",
        "ego_history",
        "ego_history_mask",
        "route_xy",
        "route_mask",
    )
    arrays = {name: [] for name in fields}
    observed = []
    hook = model.fusion.register_forward_hook(
        lambda _module, _inputs, output: observed.append(output.detach().clone())
    )
    before = tensor_digest(model.state_dict())
    try:
        with torch.no_grad():
            for batch in DataLoader(
                dataset,
                batch_size=4,
                shuffle=False,
                num_workers=0,
                pin_memory=False,
            ):
                observed.clear()
                tensors = {name: batch[name].to(device) for name in inputs}
                xy, speed, logits = model(*(tensors[name] for name in inputs))
                require(len(observed) == 1, "fusion output capture count differs")
                values = (
                    observed[0],
                    xy,
                    speed,
                    batch["target_xy"],
                    batch["target_speed_mps"],
                    batch["target_yaw_rad"],
                    batch["target_valid"],
                    logits,
                )
                for name, value in zip(fields, values):
                    arrays[name].append(value.detach().cpu().clone())
    finally:
        hook.remove()
    require(tensor_digest(model.state_dict()) == before, "F model changed during cache")
    result = core.CachedStopmixSelectorData(
        split=dataset.split,
        sample_ids=tuple(example.token for example in dataset.examples),
        episode_ids=tuple(example.episode_id for example in dataset.examples),
        source_checkpoint_sha256=F_CHECKPOINT_SHA256,
        corpus_fingerprint_sha256=corpus_sha,
        dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        **{name: torch.cat(values) for name, values in arrays.items()},
    )
    require(len(result) == len(dataset), "cache row count differs")
    return result


def subset_cache(data, indices, core):
    indices = list(indices)
    values = {
        name: getattr(data, name)
        for name in (
            "split",
            "source_checkpoint_sha256",
            "corpus_fingerprint_sha256",
            "dataset_fingerprint_sha256",
        )
    }
    values["sample_ids"] = tuple(data.sample_ids[index] for index in indices)
    values["episode_ids"] = tuple(data.episode_ids[index] for index in indices)
    for name in core.cache_core._TENSOR_FIELDS:
        values[name] = getattr(data, name)[indices]
    return core.CachedStopmixSelectorData(**values)


def standard_metrics(data, logits, core):
    report = core.cache_core.evaluate_logits(data, logits)
    return {
        "sample_count": report["sample_count"],
        "metrics": report["metrics"],
        "metric_counts": report["metric_counts"],
        "selected_histogram": report["selected_histogram"],
        "composite_oracle_histogram": report["composite_oracle_histogram"],
        "selected_composite_oracle_agreement_count": report[
            "selected_composite_oracle_agreement_count"
        ],
    }


def subgroup_metrics(dev, logits, core, combined_report):
    groups = {
        "town03_right_turn_dev": [
            index
            for index, episode in enumerate(dev.episode_ids)
            if episode == RIGHT_EPISODE
        ],
        "town03_straight_dev": [
            index
            for index, episode in enumerate(dev.episode_ids)
            if episode == STRAIGHT_EPISODE
        ],
        "combined_val": list(range(len(dev))),
    }
    require(
        len(groups["town03_right_turn_dev"]) == 337
        and len(groups["town03_straight_dev"]) == 328
        and len(groups["combined_val"]) == 665,
        "fixed DEV subgroup counts differ",
    )
    result = {
        name: standard_metrics(
            subset_cache(dev, indices, core), logits[indices], core
        )
        for name, indices in groups.items()
        if name != "combined_val"
    }
    # HH_260906 - Reuse the core's sole full-DEV final evaluation instead of evaluating the combined set twice.
    result["combined_val"] = {
        name: combined_report[name]
        for name in (
            "sample_count",
            "metrics",
            "metric_counts",
            "selected_histogram",
            "composite_oracle_histogram",
            "selected_composite_oracle_agreement_count",
        )
    }
    return result


def strict_gate(metrics, geometry):
    thresholds = {
        "town03_right_turn_dev": RIGHT_MAXIMUM,
        "town03_straight_dev": STRAIGHT_MAXIMUM,
        "combined_val": COMBINED_MAXIMUM,
    }
    checks = []
    for group, maximum in thresholds.items():
        for metric, limit in maximum.items():
            value = metrics[group]["metrics"][metric]
            checks.append(
                {
                    "group": group,
                    "metric": metric,
                    "observed": value,
                    "maximum": limit,
                    "pass": value <= limit,
                }
            )
    geometry_checks = [
        {
            "metric": name,
            "observed": geometry[name],
            "minimum": minimum,
            "pass": geometry[name] >= minimum,
        }
        for name, minimum in GEOMETRY_MINIMUM.items()
    ]
    pass_count = sum(row["pass"] for row in checks)
    geometry_pass_count = sum(row["pass"] for row in geometry_checks)
    passed = pass_count == 9 and geometry_pass_count == 3
    return {
        "status": (
            "PASS_STRICT_9_METRICS_AND_GEOMETRY_NOT_PROMOTED"
            if passed
            else "FAIL_STRICT_PERFORMANCE_GATE_NOT_PROMOTED"
        ),
        "pass": passed,
        "metric_pass_count": pass_count,
        "metric_check_count": 9,
        "geometry_pass_count": geometry_pass_count,
        "geometry_check_count": 3,
        "metric_checks": checks,
        "geometry_checks": geometry_checks,
        "stop_guard": {
            "status": "NOT_EVALUABLE_NO_SEALED_STATIONARY_DEV_EPISODE",
            "sample_count": 0,
            "required_for_promotion": True,
            "pass": None,
            "reason": "The fixed DEV contract contains only Town03 right-turn and straight episodes; no stationary episode may be inferred, substituted, or selected post hoc.",
        },
        "promotion_eligible": False,
        "interpretation": "The nine moving DEV metrics and frozen geometry are strict screens. The missing stop guard independently prevents promotion even if all screens pass.",
    }


def paired_h1_vs_h0_gate(h0_metrics, h1_metrics):
    checks = []
    for group in ("town03_straight_dev", "combined_val"):
        for metric in ("ade_6p4s_m", "fde_6p4s_m", "speed_mae_6p4s_mps"):
            reference = h0_metrics[group]["metrics"][metric]
            observed = h1_metrics[group]["metrics"][metric]
            checks.append(
                {
                    "group": group,
                    "metric": metric,
                    "H0": reference,
                    "H1": observed,
                    "delta_H1_minus_H0": observed - reference,
                    "pass": observed <= reference,
                }
            )
    count = sum(row["pass"] for row in checks)
    return {
        "status": (
            "PASS_H1_NONREGRESSION_ON_STRAIGHT_AND_COMBINED"
            if count == 6
            else "FAIL_H1_PAIRED_NONREGRESSION_GATE"
        ),
        "pass": count == 6,
        "pass_count": count,
        "check_count": 6,
        "checks": checks,
        "scope": "Paired H1 versus H0 final DEV only; same F head initialization, base order/positions, steps, optimizer and objective.",
    }


def git_identity(repo):
    environment = dict(
        os.environ,
        GIT_NO_LAZY_FETCH="1",
        GIT_ALLOW_PROTOCOL="",
        GIT_TERMINAL_PROMPT="0",
    )
    head = subprocess.check_output(
        ["git", "-c", "protocol.allow=never", "rev-parse", "HEAD"],
        cwd=repo,
        env=environment,
        timeout=20,
        text=True,
    ).strip()
    status = subprocess.check_output(
        ["git", "-c", "protocol.allow=never", "status", "--porcelain", "--untracked-files=all"],
        cwd=repo,
        env=environment,
        timeout=20,
        text=True,
    ).splitlines()
    require(head == "b478f02e42b94bf04bffec5c8170e05edc33b0f8", "detached base commit differs")
    expected = {
        "portable_e2e/evaluate.py",
        "portable_e2e/frozen_stopmix_drive_rank.py",
        "portable_e2e/frozen_stopmix_residual.py",
        "portable_e2e/frozen_stopmix_selector.py",
        "portable_e2e/frozen_stopmix_selector_recalibration.py",
        "portable_e2e/train.py",
        "scripts/e2e/diagnose_stopmix_training_oracles.py",
        "scripts/e2e/evaluate_portable_moving_data_subgroups.py",
        "scripts/e2e/run_moving_selector_sampling_recalibration.py",
    }
    observed = {row[3:] for row in status}
    require(observed == expected, "H source worktree overlay set differs")
    return {"base_commit": head, "porcelain": status, "overlay_paths": sorted(observed)}


def manifest(output):
    rows = []
    for path in sorted(output.rglob("*")):
        if path.is_file() and path.name != "SHA256SUMS":
            require(not path.is_symlink(), "symlink output forbidden")
            rows.append((path.relative_to(output).as_posix(), digest(path)))
    with (output / "SHA256SUMS").open("xb") as stream:
        stream.write(
            "".join(f"{value}  {name}\n" for name, value in rows).encode()
        )
        stream.flush()
        os.fsync(stream.fileno())
    return {"file_count_excluding_manifest": len(rows), "sha256": digest(output / "SHA256SUMS")}


def run(workspace, repo):
    workspace = Path(workspace).absolute()
    repo = Path(repo).absolute()
    output = workspace / OUTPUT_RELATIVE
    f_root = workspace / F_ROOT_RELATIVE
    require(
        repo == workspace / REPO_RELATIVE
        and repo.is_dir()
        and not repo.is_symlink(),
        "fixed detached H source worktree required",
    )
    require(output == Path.home() / "personal/hwanhong/portable_e2e" / OUTPUT_RELATIVE,
            "fixed personal output root required")
    require(not output.exists(), "create-only output already exists")
    parent_plan_path = f_root / "plan.json"
    checkpoint_path = f_root / F_ARM / "training/checkpoints/latest.pt"
    parent_gate_path = f_root / F_ARM / "gate_v8.json"
    require(digest(parent_plan_path) == F_PLAN_SHA256, "F plan bytes differ")
    require(digest(checkpoint_path) == F_CHECKPOINT_SHA256, "F checkpoint bytes differ")
    require(digest(parent_gate_path) == F_GATE_SHA256, "F geometry gate bytes differ")
    gpu_preflight = preflight_gpu()
    source = {name: digest(repo / name) for name in SOURCE_PATHS}
    parent_plan = read_json(parent_plan_path)
    git_proof = git_identity(repo)
    require(
        source["portable_e2e/train.py"]
        == parent_plan["source"]["current_sha256"]["portable_e2e/train.py"]
        and source["portable_e2e/evaluate.py"]
        == parent_plan["source"]["current_sha256"]["portable_e2e/evaluate.py"]
        and source["scripts/e2e/evaluate_portable_moving_data_subgroups.py"]
        == parent_plan["source"]["added_sha256"]["scripts/e2e/evaluate_portable_moving_data_subgroups.py"],
        "F overlay bytes differ",
    )
    data_contract = parent_plan["arm"]["dataset"]
    dataset_root = (workspace / data_contract["path"]).resolve(strict=True)
    require(
        dataset_root.resolve().is_relative_to(
            (workspace.parent / "dataset/prepared").resolve()
        ),
        "dataset escaped personal prepared storage",
    )
    require(
        digest(dataset_root / "dataset.json") == data_contract["manifest_sha256"],
        "dataset manifest differs",
    )
    plan = {
        "schema": "portable_e2e.moving_selector_sampling_recalibration_plan.v1",
        "experiment_id": "hh260921-moving-selector-sampling-recalibration-seed20260906-v1",
        "declared_at_utc": utc(),
        "output": OUTPUT_RELATIVE,
        "repo_root": REPO_RELATIVE,
        "git": git_proof,
        "gpu": {"physical_index": 0, "uuid": GPU_UUID},
        "source_sha256": source,
        "parent": {
            "f_plan": F_ROOT_RELATIVE + "/plan.json",
            "f_plan_sha256": F_PLAN_SHA256,
            "f_checkpoint": F_ROOT_RELATIVE
            + "/"
            + F_ARM
            + "/training/checkpoints/latest.pt",
            "f_checkpoint_sha256": F_CHECKPOINT_SHA256,
            "f_gate": F_ROOT_RELATIVE + "/" + F_ARM + "/gate_v8.json",
            "f_gate_sha256": F_GATE_SHA256,
        },
        "dataset": data_contract,
        "arms": ["H0_f_right_replay", "H1_train_cyclic_uniform_replay"],
        "cache": "One immutable TRAIN1475 plus DEV665 generator/context/candidate cache shared by both arms.",
        "fit": {
            "epochs": 20,
            "steps": 8280,
            "batch_size": 4,
            "samples_seen": 33120,
            "base_exposures": 29500,
            "extra_exposures": 3620,
            "learning_rate": 0.0001,
            "weight_decay": 0.0001,
            "maximum_gradient_norm": 5.0,
            "objective": "0.1*CE + 0.1*expected detached composite regret",
            "trainable_parameter_count": 3084,
        },
        "paired_sampling": {
            "H0_f_right_replay": "Exact F direction-balanced right replay identities.",
            "H1_train_cyclic_uniform_replay": "Same base identities/order, stream positions and steps; the 181 extra identities per epoch are cyclic-uniform over all TRAIN with zero within-batch duplicates.",
        },
        "dev_policy": "Town03 DEV is cached once but never used for fitting or intermediate selection; targets and metrics are evaluated only after the final fixed update.",
        "strict_gate": {
            "right_C_maximum": RIGHT_MAXIMUM,
            "straight_D_maximum": STRAIGHT_MAXIMUM,
            "combined_D_maximum": COMBINED_MAXIMUM,
            "geometry_minimum": GEOMETRY_MINIMUM,
            "stop_guard": "NOT_EVALUABLE without a sealed stationary DEV episode; never treated as pass.",
        },
        **DENIALS,
    }
    output.mkdir(parents=True, exist_ok=False)
    write_new(output / "plan.json", plan)
    for name in SOURCE_PATHS:
        copy_new(repo / name, output / "source" / name)
    copy_new(parent_plan_path, output / "parent/F_plan.json")
    copy_new(parent_gate_path, output / "parent/F_gate_v8.json")
    started = {"schema": "portable_e2e.moving_selector_sampling_recalibration_started.v1",
               "started_at_utc": utc(), "gpu_preflight": gpu_preflight,
               "plan_sha256": digest(output / "plan.json"), **DENIALS}
    write_new(output / "started.json", started)

    sys.path.insert(0, str(repo))
    import torch
    from portable_e2e import frozen_stopmix_selector_recalibration as core
    from portable_e2e import losses as loss_module
    from portable_e2e.audit_runtime import _read_checkpoint_for_audit
    from portable_e2e.dataset import load_training_examples
    from portable_e2e.model import PerspectiveTrajectoryModel, parameter_count
    from portable_e2e.torch_dataset import Common10TorchDataset
    from portable_e2e.train import TrainConfig, _nested_tensors_are_finite
    from scripts.e2e import diagnose_stopmix_training_oracles as oracle_helper

    require(torch.cuda.device_count() == 1, "exactly one visible CUDA device required")
    device = torch.device("cuda:0")
    properties = torch.cuda.get_device_properties(0)
    observed_uuid = str(getattr(properties, "uuid", ""))
    require(
        observed_uuid in (GPU_UUID, GPU_UUID.removeprefix("GPU-")),
        "PyTorch visible UUID differs",
    )
    payload, config, training_ids, checkpoint_proof = _read_checkpoint_for_audit(
        checkpoint_path=checkpoint_path,
        expected_checkpoint_sha256=F_CHECKPOINT_SHA256,
        corpus_fingerprint_sha256=data_contract["corpus_fingerprint_sha256"],
    )
    require(parameter_count(PerspectiveTrajectoryModel(config)) == 1056362,
            "F architecture parameter count differs")
    require(payload["sampling_plan"] == parent_plan["arm"]["sampling_plan"],
            "checkpoint and F-plan sampling contracts differ")
    train_config = TrainConfig(**dict(payload["train_config"]))
    require(tuple(training_ids) == tuple(sorted(data_contract["episode_ids"]["train"])),
            "F training episode IDs differ")
    datasets = {}
    for split, count in (("train", 1475), ("val", 665)):
        loaded = load_training_examples(
            dataset_root, split=split, mode="planning", check_image_hashes=True
        )
        require(
            loaded.validation_report["dataset_fingerprint_sha256"]
            == data_contract["corpus_fingerprint_sha256"],
            "corpus fingerprint differs",
        )
        dataset = Common10TorchDataset(
            loaded.examples,
            config,
            verify_image_sha256=True,
            split=split,
        )
        require(
            len(dataset) == count
            and dataset.fingerprint_sha256
            == data_contract["split_fingerprint_sha256"][split],
            "split count or fingerprint differs",
        )
        require(
            set(example.episode_id for example in dataset.examples)
            == set(data_contract["episode_ids"][split]),
            "split episode identity differs",
        )
        datasets[split] = dataset
    model = PerspectiveTrajectoryModel(config).to(device)
    model.load_state_dict(payload["model_state_dict"], strict=True)
    require(_nested_tensors_are_finite(model.state_dict()), "F model contains nonfinite state")
    model.requires_grad_(False)
    model.eval()
    del payload
    generator_before = tensor_digest(model.state_dict())
    caches = {
        split: extract_cache(
            model,
            datasets[split],
            core,
            data_contract["corpus_fingerprint_sha256"],
            device,
            torch,
        )
        for split in ("train", "val")
    }
    core.validate_pair(caches["train"], caches["val"])
    require(tensor_digest(model.state_dict()) == generator_before,
            "F generator changed after shared cache")
    cache_proof = {}
    for split, cache in caches.items():
        path = output / "cache" / (split + ".pt")
        save_new(
            path,
            {
                "artifact_id": "portable_e2e.frozen_f_cache.v1",
                "split": split,
                "cache_digest": cache.digest(),
                "sample_ids": list(cache.sample_ids),
                "episode_ids": list(cache.episode_ids),
                "tensors": {
                    name: getattr(cache, name)
                    for name in core.cache_core._TENSOR_FIELDS
                },
                **DENIALS,
            },
            torch,
        )
        cache_proof[split] = {
            "count": len(cache),
            "cache_digest": cache.digest(),
            "file_sha256": digest(path),
        }
    projected_costs_fn, projection_proof = oracle_helper.projected_costs(loss_module)
    parent_gate = read_json(parent_gate_path)
    parent_geometry = parent_gate["geometry"]
    geometry = {
        "selected_geometry_pass_count": parent_geometry["selected_result"][
            "geometry_pass_count"
        ],
        "any_candidate_geometry_pass_count": parent_geometry[
            "any_candidate_geometry_pass_count"
        ],
        "all_candidates_geometry_pass_count": parent_geometry[
            "all_candidates_geometry_pass_count"
        ],
        "sample_count": parent_geometry["sample_count"],
        "inheritance": "Bit-identical cached F candidates; every candidate has the same 664/665 pass mask, so selector identity cannot change these counts.",
        "parent_gate_sha256": F_GATE_SHA256,
    }
    arm_results = {}
    paired_initial_sha = None
    for arm in core.ARMS:
        arm_dir = output / arm
        arm_dir.mkdir()
        with (arm_dir / "history.jsonl").open("x") as history:
            def progress(row):
                history.write(json.dumps(row, sort_keys=True, allow_nan=False) + "\n")
                history.flush()
                if row["global_step"] % 414 == 0:
                    print(json.dumps({"event": "H_PROGRESS", "arm": arm,
                                      "step": row["global_step"],
                                      "samples_seen": row["samples_seen"]}), flush=True)

            result = core.fit_recalibrated_selector(
                caches["train"],
                caches["val"],
                original_drive_head=model.candidate_head,
                original_stop_head=model.stop_candidate_head,
                sampling_plan=parent_plan["arm"]["sampling_plan"],
                train_config=train_config,
                projected_costs_fn=projected_costs_fn,
                projection_proof=projection_proof,
                arm=arm,
                device=device,
                callback=progress,
            )
        require(len(result["report"]["history"]) == 8280,
                "complete fixed history required")
        if paired_initial_sha is None:
            paired_initial_sha = result["report"]["initial_head_sha256"]
        require(result["report"]["initial_head_sha256"] == paired_initial_sha,
                "paired arms did not start from identical F selector bytes")
        metrics = subgroup_metrics(
            caches["val"],
            result["final_logits_cpu"]["val"],
            core,
            result["report"]["final_dev_metrics"],
        )
        gate = strict_gate(metrics, geometry)
        result["report"]["history_sha256"] = digest(arm_dir / "history.jsonl")
        result["report"]["strict_gate"] = gate
        result["report"]["subgroup_metrics"] = metrics
        result["report"]["geometry"] = geometry
        result["report"]["final_dev_metric_evaluation_calls"] = {
            "full_dev": 1,
            "right_subgroup": 1,
            "straight_subgroup": 1,
            "all_after_final_step": True,
        }
        result["report"]["history"] = {
            "path": "history.jsonl",
            "row_count": 8280,
            "sha256": result["report"]["history_sha256"],
        }
        write_new(arm_dir / "report.json", result["report"])
        save_new(
            arm_dir / "head_only.pt",
            {
                "artifact_id": core.ARTIFACT_ID,
                "source_checkpoint_sha256": F_CHECKPOINT_SHA256,
                "head_state_dict": result["head_state_dict"],
                "runtime_checkpoint": False,
                **DENIALS,
            },
            torch,
        )
        save_new(
            arm_dir / "final_logits.pt",
            {
                "artifact_id": "portable_e2e.frozen_f_selector_logits.v1",
                "source_checkpoint_sha256": F_CHECKPOINT_SHA256,
                "logits": result["final_logits_cpu"],
                **DENIALS,
            },
            torch,
        )
        arm_results[arm] = {
            "status": result["report"]["status"],
            "initial_head_sha256": result["report"]["initial_head_sha256"],
            "final_head_sha256": result["report"]["final_head_sha256"],
            "arm_schedule_sha256": result["report"]["arm_schedule_sha256"],
            "paired_base_position_order_sha256": result["report"][
                "paired_base_position_order_sha256"
            ],
            "strict_gate": gate,
            "subgroup_metrics": metrics,
            "files": {
                name: digest(arm_dir / name)
                for name in (
                    "history.jsonl",
                    "report.json",
                    "head_only.pt",
                    "final_logits.pt",
                )
            },
        }
        del result
        torch.cuda.empty_cache()
    require(
        arm_results[core.ARMS[0]]["paired_base_position_order_sha256"]
        == arm_results[core.ARMS[1]]["paired_base_position_order_sha256"],
        "paired base schedule proof differs",
    )
    paired_gate = paired_h1_vs_h0_gate(
        arm_results[core.ARMS[0]]["subgroup_metrics"],
        arm_results[core.ARMS[1]]["subgroup_metrics"],
    )
    h1_absolute_pass = arm_results[core.ARMS[1]]["strict_gate"]["pass"]
    recovery_signal = h1_absolute_pass and paired_gate["pass"]
    require(tensor_digest(model.state_dict()) == generator_before,
            "F generator changed after both head-only fits")
    summary = {
        "schema": "portable_e2e.moving_selector_sampling_recalibration_summary.v1",
        "status": "PAIRED_HEAD_ONLY_FITS_COMPLETE_NOT_PROMOTED",
        "completed_at_utc": utc(),
        "plan_sha256": digest(output / "plan.json"),
        "source_sha256": source,
        "gpu_preflight": gpu_preflight,
        "torch": {
            "version": str(torch.__version__),
            "visible_device_count": torch.cuda.device_count(),
            "device_name": properties.name,
            "device_uuid": observed_uuid,
        },
        "checkpoint_proof": checkpoint_proof,
        "generator_state_sha256_before": generator_before,
        "generator_state_sha256_after": tensor_digest(model.state_dict()),
        "cache": cache_proof,
        "projection_proof": projection_proof,
        "paired_initial_head_sha256": paired_initial_sha,
        "arms": arm_results,
        "paired_h1_vs_h0_gate": paired_gate,
        "decision": {
            "eligible_arms": [],
            "status": (
                "SELECTOR_SAMPLING_RECOVERY_SIGNAL"
                if recovery_signal
                else "FAIL_SELECTOR_SAMPLING_RECOVERY_SIGNAL"
            ),
            "recovery_signal": recovery_signal,
            "h1_absolute_9_metric_and_geometry_gate_pass": h1_absolute_pass,
            "h1_vs_h0_paired_6_metric_gate_pass": paired_gate["pass"],
            "stop_guard_status": "NOT_EVALUABLE_NO_SEALED_STATIONARY_DEV_EPISODE",
            "reason": "No arm is promotable: a recovery signal requires H1 absolute 9+geometry and paired 6-metric nonregression, while TEST and closed loop remain forbidden and the sealed stop guard is NOT_EVALUABLE.",
            "next_action": "Use the arm comparison only to decide whether selector sampling deserves a separately sealed confirmation study.",
        },
        **DENIALS,
    }
    write_new(output / "summary.json", summary)
    manifest_proof = manifest(output)
    print(json.dumps({"event": "H_COMPLETE", "output": str(output),
                      "summary_sha256": digest(output / "summary.json"),
                      "manifest": manifest_proof}, sort_keys=True), flush=True)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path.home() / "personal/hwanhong/portable_e2e",
    )
    parser.add_argument("--repo-root", type=Path, required=True)
    args = parser.parse_args(argv)
    output = args.workspace.absolute() / OUTPUT_RELATIVE
    try:
        return run(args.workspace, args.repo_root)
    except BaseException as error:
        if output.is_dir() and not (output / "failed.json").exists():
            try:
                write_new(
                    output / "failed.json",
                    {
                        "schema": "portable_e2e.moving_selector_sampling_recalibration_failure.v1",
                        "failed_at_utc": utc(),
                        "error_type": type(error).__name__,
                        "error": str(error),
                        "traceback": traceback.format_exc(),
                        **DENIALS,
                    },
                )
            except BaseException:
                pass
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
