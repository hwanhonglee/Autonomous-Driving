#!/usr/bin/env python3
"""HH_260906 - Run paired DRIVE-mode residual recovery from exact H caches."""

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
H_ROOT_RELATIVE = (
    "runs/diagnostics/"
    "moving_selector_sampling_recalibration_seed20260906_20260921_v1"
)
H_PLAN_SHA256 = "3bf5bd5e63e3c637b47efab52e5c4e98b83a35fd337059bc4028962a80c82542"
H_SUMMARY_SHA256 = "e916adbb57fc2dddd21b77da6024daad9519e0e0184d944bb6677701d0666b7e"
H_MANIFEST_SHA256 = "fba179f8c4670da20b0648f121a4358cfb2582d9f672ed29f04ed809e31e00ba"
H_TRAIN_CACHE_SHA256 = "a586abd602ca410e26b37785d30b371b3310e4ee31644870838c3569cc077ca0"
H_DEV_CACHE_SHA256 = "cbc9040ea13f32826a701a3ccc112ffc79cdb9e6e47f3db6600f96fd83728e50"
H1_HISTORY_SHA256 = "0bdbfeff48187e804c2639f9838674a398461c965bcbfb1ddac635f576f5cf92"
H1_REPORT_SHA256 = "a91db88bfcb4cc309063632667bd0012d1eeb7eaacb08955bbac314cc7beb115"
H1_ARM = "H1_train_cyclic_uniform_replay"
OUTPUT_RELATIVE = (
    "runs/diagnostics/"
    "moving_drive_mode_residual_recovery_seed20260906_20260921_v1"
)
REPO_RELATIVE = (
    "source_worktrees/"
    "moving_drive_mode_residual_recovery_i_seed20260906_20260921_v1"
)
RIGHT_EPISODE = (
    "carla_town03_town03_right_s0000_p00_ClearNoon_s0000_1cc631b7eac9"
)
STRAIGHT_EPISODE = (
    "carla_town03_town03_straight_s0000_p07_ClearNoon_s0201_26b281301320"
)
SOURCE_PATHS = (
    "portable_e2e/__init__.py",
    "portable_e2e/dataset.py",
    "portable_e2e/torch_dataset.py",
    "portable_e2e/visualize.py",
    "portable_e2e/frozen_selector.py",
    "portable_e2e/frozen_drive_mode_residual_recovery.py",
    "portable_e2e/frozen_stopmix_selector_recalibration.py",
    "portable_e2e/frozen_stopmix_selector.py",
    "portable_e2e/frozen_stopmix_drive_rank.py",
    "portable_e2e/frozen_stopmix_residual.py",
    "portable_e2e/losses.py",
    "portable_e2e/model.py",
    "portable_e2e/train.py",
    "portable_e2e/evaluate.py",
    "portable_e2e/contract.py",
    "portable_e2e/stop_primitive_research.py",
    "scripts/e2e/diagnose_stopmix_training_oracles.py",
    "scripts/e2e/run_moving_drive_mode_residual_recovery.py",
)
BASE_SOURCE_PATHS = (
    "portable_e2e/__init__.py",
    "portable_e2e/dataset.py",
    "portable_e2e/torch_dataset.py",
    "portable_e2e/visualize.py",
    "portable_e2e/frozen_selector.py",
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
    "new_autonomous_function_head_added": False,
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


def regular(path):
    path = Path(path).absolute()
    require(path.is_file() and not path.is_symlink(), "regular file required")
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


def read_jsonl(path):
    path = regular(path)
    rows = []
    for number, line in enumerate(path.read_text().splitlines(), start=1):
        require(line.strip(), f"blank JSONL row {number}")
        rows.append(json.loads(line, parse_constant=lambda value: require(False, value)))
    return rows


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
    with source.open("rb") as source_stream, target.open("xb") as target_stream:
        shutil.copyfileobj(source_stream, target_stream, 1024 * 1024)
        target_stream.flush()
        os.fsync(target_stream.fileno())
    require(digest(source) == digest(target), "archived source differs")


def verify_exact_h_manifest(root):
    root = Path(root).absolute()
    manifest_path = root / "SHA256SUMS"
    require(digest(manifest_path) == H_MANIFEST_SHA256, "H manifest bytes differ")
    rows = []
    for line in regular(manifest_path).read_text().splitlines():
        require(len(line) > 66 and line[64:66] == "  ", "invalid H manifest row")
        value, name = line[:64], line[66:]
        require(
            len(value) == 64
            and all(character in "0123456789abcdef" for character in value)
            and name
            and not Path(name).is_absolute()
            and ".." not in Path(name).parts,
            "unsafe H manifest row",
        )
        rows.append((name, value))
    require(
        [name for name, _value in rows] == sorted(name for name, _value in rows)
        and len({name for name, _value in rows}) == len(rows),
        "H manifest paths are not unique and sorted",
    )
    for name, value in rows:
        require(digest(root / name) == value, "H artifact differs: " + name)
    actual = sorted(
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file() and path.name != "SHA256SUMS"
    )
    require(actual == [name for name, _value in rows], "H artifact inventory differs")
    return {
        "sha256": H_MANIFEST_SHA256,
        "verified_file_count": len(rows),
        "all_files_verified": True,
    }


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
        "CUDA_VISIBLE_DEVICES must bind only approved physical GPU0 UUID",
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
    require(len(rows) == 1, "approved GPU UUID missing or duplicated")
    fields = [value.strip() for value in rows[0].split(",")]
    require(fields[0] == "0", "approved UUID is not physical GPU0")
    require(
        int(fields[2]) == 0 and int(fields[3]) == 0,
        "approved GPU0 must be completely idle before I starts",
    )
    return {
        "physical_index": 0,
        "uuid": fields[1],
        "memory_used_mib_before_torch": int(fields[2]),
        "utilization_percent_before_torch": int(fields[3]),
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
        [
            "git",
            "-c",
            "protocol.allow=never",
            "status",
            "--porcelain",
            "--untracked-files=all",
        ],
        cwd=repo,
        env=environment,
        timeout=20,
        text=True,
    ).splitlines()
    require(
        head == "b478f02e42b94bf04bffec5c8170e05edc33b0f8",
        "detached base commit differs",
    )
    expected = {
        "portable_e2e/evaluate.py",
        "portable_e2e/frozen_drive_mode_residual_recovery.py",
        "portable_e2e/frozen_stopmix_drive_rank.py",
        "portable_e2e/frozen_stopmix_residual.py",
        "portable_e2e/frozen_stopmix_selector.py",
        "portable_e2e/frozen_stopmix_selector_recalibration.py",
        "portable_e2e/train.py",
        "scripts/e2e/diagnose_stopmix_training_oracles.py",
        "scripts/e2e/run_moving_drive_mode_residual_recovery.py",
    }
    observed = {row[3:] for row in status}
    require(observed == expected, "I source worktree overlay set differs")
    base_source_sha256 = {}
    for name in BASE_SOURCE_PATHS:
        committed = subprocess.check_output(
            ["git", "-c", "protocol.allow=never", "show", f"HEAD:{name}"],
            cwd=repo,
            env=environment,
            timeout=20,
        )
        observed_sha256 = digest(repo / name)
        committed_sha256 = hashlib.sha256(committed).hexdigest()
        require(
            observed_sha256 == committed_sha256,
            "base transitive source differs from detached commit: " + name,
        )
        base_source_sha256[name] = committed_sha256
    return {
        "base_commit": head,
        "porcelain": status,
        "overlay_paths": sorted(observed),
        "base_transitive_source_sha256": base_source_sha256,
    }


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
    return {
        "file_count_excluding_manifest": len(rows),
        "sha256": digest(output / "SHA256SUMS"),
    }


def load_cache(path, split, plan, summary, core, torch):
    expected_sha = (
        H_TRAIN_CACHE_SHA256 if split == "train" else H_DEV_CACHE_SHA256
    )
    require(digest(path) == expected_sha, "exact H cache bytes differ")
    payload = torch.load(path, map_location="cpu")
    require(
        isinstance(payload, dict)
        and payload.get("artifact_id") == "portable_e2e.frozen_f_cache.v1"
        and payload.get("split") == split
        and payload.get("test_read_or_use") is False,
        "H cache payload contract differs",
    )
    dataset = plan["dataset"]
    result = core.CachedStopmixSelectorData(
        split=split,
        sample_ids=tuple(payload["sample_ids"]),
        episode_ids=tuple(payload["episode_ids"]),
        source_checkpoint_sha256=F_CHECKPOINT_SHA256,
        corpus_fingerprint_sha256=dataset["corpus_fingerprint_sha256"],
        dataset_fingerprint_sha256=dataset["split_fingerprint_sha256"][split],
        **payload["tensors"],
    )
    proof = summary["cache"][split]
    require(
        len(result) == dataset["counts"][split] == proof["count"]
        and result.digest() == payload["cache_digest"] == proof["cache_digest"]
        and proof["file_sha256"] == expected_sha,
        "reconstructed H cache proof differs",
    )
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
        name: report[name]
        for name in (
            "sample_count",
            "metrics",
            "metric_counts",
            "selected_histogram",
            "composite_oracle_histogram",
            "selected_composite_oracle_agreement_count",
        )
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
    }
    require(
        len(groups["town03_right_turn_dev"]) == 337
        and len(groups["town03_straight_dev"]) == 328
        and len(dev) == 665,
        "fixed DEV subgroup counts differ",
    )
    result = {
        name: standard_metrics(
            subset_cache(dev, indices, core), logits[indices], core
        )
        for name, indices in groups.items()
    }
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


def strict_gate(metrics, geometry, modes):
    thresholds = {
        "town03_right_turn_dev": RIGHT_MAXIMUM,
        "town03_straight_dev": STRAIGHT_MAXIMUM,
        "combined_val": COMBINED_MAXIMUM,
    }
    metric_checks = []
    for group, maximum in thresholds.items():
        for metric, limit in maximum.items():
            observed = metrics[group]["metrics"][metric]
            metric_checks.append(
                {
                    "group": group,
                    "metric": metric,
                    "observed": observed,
                    "maximum": limit,
                    "pass": observed <= limit,
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
    mode_checks = [
        {
            "split": split,
            "mode_change_count": proof["mode_change_count"],
            "original_stop_selected_index_mismatch_count": proof[
                "original_stop_selected_index_mismatch_count"
            ],
            "original_stop_row_logit_mismatch_count": proof[
                "original_stop_row_logit_mismatch_count"
            ],
            "original_drive_group_max_mismatch_count": proof[
                "original_drive_group_max_mismatch_count"
            ],
            "pass": proof["pass"],
        }
        for split, proof in (("train", modes["train"]), ("val", modes["val"]))
    ]
    metric_count = sum(row["pass"] for row in metric_checks)
    geometry_count = sum(row["pass"] for row in geometry_checks)
    mode_count = sum(row["pass"] for row in mode_checks)
    passed = metric_count == 9 and geometry_count == 3 and mode_count == 2
    return {
        "status": (
            "PASS_STRICT_9_METRICS_GEOMETRY_AND_MODE_NOT_PROMOTED"
            if passed
            else "FAIL_STRICT_PERFORMANCE_OR_MODE_GATE_NOT_PROMOTED"
        ),
        "pass": passed,
        "metric_pass_count": metric_count,
        "metric_check_count": 9,
        "geometry_pass_count": geometry_count,
        "geometry_check_count": 3,
        "mode_pass_count": mode_count,
        "mode_check_count": 2,
        "metric_checks": metric_checks,
        "geometry_checks": geometry_checks,
        "mode_checks": mode_checks,
        "stop_guard": {
            "status": "NOT_EVALUABLE_NO_SEALED_STATIONARY_DEV_EPISODE",
            "sample_count": 0,
            "f_stop_mode_preservation_pass": mode_count == 2,
            "independent_stationary_performance_pass": None,
            "required_for_promotion": True,
            "pass": None,
            "reason": "Exact preservation of F STOP rows is not an independent stationary-performance evaluation.",
        },
        "promotion_eligible": False,
    }


def paired_i1_vs_i0_gate(i0_metrics, i1_metrics):
    checks = []
    for group in ("town03_straight_dev", "combined_val"):
        for metric in ("ade_6p4s_m", "fde_6p4s_m", "speed_mae_6p4s_mps"):
            reference = i0_metrics[group]["metrics"][metric]
            observed = i1_metrics[group]["metrics"][metric]
            checks.append(
                {
                    "group": group,
                    "metric": metric,
                    "I0": reference,
                    "I1": observed,
                    "delta_I1_minus_I0": observed - reference,
                    "pass": observed <= reference,
                }
            )
    count = sum(row["pass"] for row in checks)
    return {
        "status": (
            "PASS_I1_NONREGRESSION_ON_STRAIGHT_AND_COMBINED"
            if count == 6
            else "FAIL_I1_PAIRED_NONREGRESSION_DIAGNOSTIC"
        ),
        "pass": count == 6,
        "pass_count": count,
        "check_count": 6,
        "checks": checks,
        "scope": "Paired I1 versus I0 final DEV diagnostic only; identical H1 schedule, seed, initialization, optimizer, steps and DRIVE-only objective.",
    }


def run(workspace, repo):
    workspace = Path(workspace).absolute()
    repo = Path(repo).absolute()
    output = workspace / OUTPUT_RELATIVE
    h_root = workspace / H_ROOT_RELATIVE
    require(
        repo == workspace / REPO_RELATIVE
        and repo.is_dir()
        and not repo.is_symlink(),
        "fixed detached I source worktree required",
    )
    require(
        output == Path.home() / "personal/hwanhong/portable_e2e" / OUTPUT_RELATIVE,
        "fixed personal output root required",
    )
    require(not output.exists(), "create-only output already exists")
    require(not (h_root / "failed.json").exists(), "failed H run cannot be a parent")
    h_manifest = verify_exact_h_manifest(h_root)
    h_plan_path = h_root / "plan.json"
    h_summary_path = h_root / "summary.json"
    h_history_path = h_root / H1_ARM / "history.jsonl"
    h_report_path = h_root / H1_ARM / "report.json"
    require(digest(h_plan_path) == H_PLAN_SHA256, "exact H plan bytes differ")
    require(digest(h_summary_path) == H_SUMMARY_SHA256, "exact H summary bytes differ")
    require(digest(h_history_path) == H1_HISTORY_SHA256, "exact H1 history bytes differ")
    require(digest(h_report_path) == H1_REPORT_SHA256, "exact H1 report bytes differ")
    h_plan = read_json(h_plan_path)
    h_summary = read_json(h_summary_path)
    h_report = read_json(h_report_path)
    require(
        h_plan["schema"]
        == "portable_e2e.moving_selector_sampling_recalibration_plan.v1"
        and h_summary["schema"]
        == "portable_e2e.moving_selector_sampling_recalibration_summary.v1"
        and h_summary["status"] == "PAIRED_HEAD_ONLY_FITS_COMPLETE_NOT_PROMOTED"
        and h_summary["decision"]["status"]
        == "FAIL_SELECTOR_SAMPLING_RECOVERY_SIGNAL"
        and h_summary["decision"]["recovery_signal"] is False
        and h_report["arm"] == H1_ARM
        and h_report["steps"] == 8280
        and h_report["samples_seen"] == 33120
        and h_report["source_checkpoint_sha256"] == F_CHECKPOINT_SHA256,
        "H parent semantic contract differs",
    )
    gpu_preflight = preflight_gpu()
    source = {name: digest(repo / name) for name in SOURCE_PATHS}
    shared_parent_sources = [
        name
        for name in SOURCE_PATHS
        if (h_root / "source" / name).is_file()
    ]
    require(
        all(
            source[name] == digest(h_root / "source" / name)
            for name in shared_parent_sources
        ),
        "I shared source differs from exact H source",
    )
    git_proof = git_identity(repo)
    plan = {
        "schema": "portable_e2e.moving_drive_mode_residual_recovery_plan.v1",
        "experiment_id": "hh260921-moving-drive-mode-residual-recovery-seed20260906-v1",
        "declared_at_utc": utc(),
        "output": OUTPUT_RELATIVE,
        "repo_root": REPO_RELATIVE,
        "git": git_proof,
        "gpu": {"physical_index": 0, "uuid": GPU_UUID},
        "source_sha256": source,
        "parent_h": {
            "root": H_ROOT_RELATIVE,
            "plan_sha256": H_PLAN_SHA256,
            "summary_sha256": H_SUMMARY_SHA256,
            "manifest": h_manifest,
            "train_cache_sha256": H_TRAIN_CACHE_SHA256,
            "dev_cache_sha256": H_DEV_CACHE_SHA256,
            "h1_history_sha256": H1_HISTORY_SHA256,
            "h1_report_sha256": H1_REPORT_SHA256,
        },
        "dataset": h_plan["dataset"],
        "arms": ["I0_prior_context_residual", "I1_candidate_geometry_residual"],
        "paired_ablation": {
            "I0_prior_context_residual": "F prior/context with all 192 geometry features zeroed.",
            "I1_candidate_geometry_residual": "F prior/context plus immutable candidate XY and speed geometry.",
        },
        "fit": {
            "seed": 20260906,
            "epochs": 20,
            "steps": 8280,
            "batch_size": 4,
            "samples_seen": 33120,
            "base_exposures": 29500,
            "extra_exposures": 3620,
            "schedule": "Exact token-bound H1 cyclic-uniform history for both arms.",
            "learning_rate": 0.0001,
            "weight_decay": 0.0001,
            "maximum_gradient_norm": 5.0,
            "objective": "0.1*DRIVE6 projected-cost argmin CE + 0.1*expected detached DRIVE6 projected regret",
            "selector_shape": "449-256-1",
            "trainable_parameter_count": 115457,
            "initialization": "Identical seed and zero final layer for exact F logit/selection parity.",
        },
        "mode_contract": "Original F STOP rows remain exact K12 logits and selection. Original F DRIVE rows retain unchanged STOP logits and exact F DRIVE-group maximum while candidates 0..5 are reranked.",
        "dev_policy": "Town03 DEV labels, costs and metrics are used only after the final fixed update; no intermediate selection.",
        "strict_gate": {
            "right_C_maximum": RIGHT_MAXIMUM,
            "straight_D_maximum": STRAIGHT_MAXIMUM,
            "combined_D_maximum": COMBINED_MAXIMUM,
            "geometry_minimum": GEOMETRY_MINIMUM,
            "mode_preservation": "Exact on TRAIN and DEV.",
            "stop_guard": "NOT_EVALUABLE; F STOP preservation is not independent stationary validation.",
        },
        **DENIALS,
    }
    output.mkdir(parents=True, exist_ok=False)
    write_new(output / "plan.json", plan)
    for name in SOURCE_PATHS:
        copy_new(repo / name, output / "source" / name)
    for source_path, name in (
        (h_plan_path, "H_plan.json"),
        (h_summary_path, "H_summary.json"),
        (h_report_path, "H1_report.json"),
        (h_root / "SHA256SUMS", "H_SHA256SUMS"),
    ):
        copy_new(source_path, output / "parent" / name)
    write_new(
        output / "started.json",
        {
            "schema": "portable_e2e.moving_drive_mode_residual_recovery_started.v1",
            "started_at_utc": utc(),
            "gpu_preflight": gpu_preflight,
            "plan_sha256": digest(output / "plan.json"),
            **DENIALS,
        },
    )

    sys.path.insert(0, str(repo))
    import torch
    from portable_e2e import frozen_drive_mode_residual_recovery as core
    from portable_e2e import losses as loss_module
    from scripts.e2e import diagnose_stopmix_training_oracles as oracle_helper

    require(torch.cuda.device_count() == 1, "exactly one visible CUDA device required")
    device = torch.device("cuda:0")
    properties = torch.cuda.get_device_properties(0)
    observed_uuid = str(getattr(properties, "uuid", ""))
    require(
        observed_uuid in (GPU_UUID, GPU_UUID.removeprefix("GPU-")),
        "PyTorch visible UUID differs",
    )
    caches = {
        split: load_cache(
            h_root / "cache" / f"{split}.pt",
            split,
            h_plan,
            h_summary,
            core,
            torch,
        )
        for split in ("train", "val")
    }
    core.validate_pair(caches["train"], caches["val"])
    cache_before = {split: data.digest() for split, data in caches.items()}
    schedule = core.validate_h1_schedule(
        read_jsonl(h_history_path), caches["train"]
    )
    normalized_schedule_sha256 = core.schedule_digest(schedule)
    projected_costs_fn, projection_proof = oracle_helper.projected_costs(loss_module)
    geometry = dict(h_report["geometry"])
    require(
        all(geometry[name] >= minimum for name, minimum in GEOMETRY_MINIMUM.items())
        and geometry["sample_count"] == 665,
        "inherited F candidate geometry contract differs",
    )
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
                    print(
                        json.dumps(
                            {
                                "event": "I_PROGRESS",
                                "arm": arm,
                                "step": row["global_step"],
                                "samples_seen": row["samples_seen"],
                            }
                        ),
                        flush=True,
                    )

            result = core.fit_drive_mode_residual(
                caches["train"],
                caches["val"],
                schedule=schedule,
                projected_costs_fn=projected_costs_fn,
                projection_proof=projection_proof,
                arm=arm,
                device=device,
                callback=progress,
            )
        require(len(result["report"]["history"]) == 8280, "complete I history required")
        if paired_initial_sha is None:
            paired_initial_sha = result["report"]["initial_head_sha256"]
        require(
            result["report"]["initial_head_sha256"] == paired_initial_sha
            and result["report"]["normalized_h1_schedule_sha256"]
            == normalized_schedule_sha256,
            "paired I initialization or schedule differs",
        )
        metrics = subgroup_metrics(
            caches["val"],
            result["final_logits_cpu"]["val"],
            core,
            result["report"]["final_dev_metrics"],
        )
        modes = result["report"]["final_mode_preservation"]
        gate = strict_gate(metrics, geometry, modes)
        result["report"]["history_sha256"] = digest(arm_dir / "history.jsonl")
        result["report"]["strict_gate"] = gate
        result["report"]["subgroup_metrics"] = metrics
        result["report"]["geometry"] = geometry
        result["report"]["history"] = {
            "path": "history.jsonl",
            "row_count": 8280,
            "sha256": result["report"]["history_sha256"],
        }
        write_new(arm_dir / "report.json", result["report"])
        save_new(
            arm_dir / "selector_only.pt",
            {
                "artifact_id": core.ARTIFACT_ID,
                "source_checkpoint_sha256": F_CHECKPOINT_SHA256,
                "head_state_dict": result["head_state_dict"],
                "runtime_checkpoint": False,
                "selector_replacement_diagnostic_only": True,
                **DENIALS,
            },
            torch,
        )
        save_new(
            arm_dir / "final_logits.pt",
            {
                "artifact_id": "portable_e2e.frozen_f_drive_mode_residual_logits.v1",
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
            "normalized_h1_schedule_sha256": result["report"][
                "normalized_h1_schedule_sha256"
            ],
            "strict_gate": gate,
            "mode_preservation": modes,
            "subgroup_metrics": metrics,
            "files": {
                name: digest(arm_dir / name)
                for name in (
                    "history.jsonl",
                    "report.json",
                    "selector_only.pt",
                    "final_logits.pt",
                )
            },
        }
        del result
        torch.cuda.empty_cache()
    require(
        all(data.digest() == cache_before[split] for split, data in caches.items()),
        "shared H cache changed after paired I fits",
    )
    require(
        len({value["initial_head_sha256"] for value in arm_results.values()}) == 1
        and len(
            {
                value["normalized_h1_schedule_sha256"]
                for value in arm_results.values()
            }
        )
        == 1,
        "paired I initialization or schedule proof differs",
    )
    paired_gate = paired_i1_vs_i0_gate(
        arm_results[core.ARMS[0]]["subgroup_metrics"],
        arm_results[core.ARMS[1]]["subgroup_metrics"],
    )
    i1_absolute_pass = arm_results[core.ARMS[1]]["strict_gate"]["pass"]
    recovery_signal = i1_absolute_pass and paired_gate["pass"]
    summary = {
        "schema": "portable_e2e.moving_drive_mode_residual_recovery_summary.v1",
        "status": "PAIRED_DRIVE_MODE_RESIDUAL_FITS_COMPLETE_NOT_PROMOTED",
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
        "parent_h": plan["parent_h"],
        "cache_sha256_before": cache_before,
        "cache_sha256_after": {
            split: data.digest() for split, data in caches.items()
        },
        "projection_proof": projection_proof,
        "paired_initial_head_sha256": paired_initial_sha,
        "paired_normalized_h1_schedule_sha256": normalized_schedule_sha256,
        "arms": arm_results,
        "paired_i1_vs_i0_gate": paired_gate,
        "decision": {
            "eligible_arms": [],
            "status": (
                "DRIVE_MODE_RESIDUAL_RECOVERY_SIGNAL"
                if recovery_signal
                else "FAIL_DRIVE_MODE_RESIDUAL_RECOVERY_SIGNAL"
            ),
            "recovery_signal": recovery_signal,
            "i1_absolute_9_metric_geometry_mode_gate_pass": i1_absolute_pass,
            "i1_vs_i0_paired_6_metric_gate_pass": paired_gate["pass"],
            "stop_guard_status": "NOT_EVALUABLE_NO_SEALED_STATIONARY_DEV_EPISODE",
            "reason": "No arm is promotable. A diagnostic signal requires I1 absolute 9-metric, geometry and exact mode gates plus paired 6-metric nonregression; stationary, TEST and closed-loop evidence remain absent.",
            "next_action": "If recovery fails, collect new unique 15/30-kph straight routes across TRAIN-role Towns before any autonomous-function head; defer right-turn-only G because the right-turn gate already passes. If recovery succeeds, seal independent stationary and new-seed confirmation studies.",
        },
        **DENIALS,
    }
    write_new(output / "summary.json", summary)
    manifest_proof = manifest(output)
    print(
        json.dumps(
            {
                "event": "I_COMPLETE",
                "output": str(output),
                "summary_sha256": digest(output / "summary.json"),
                "manifest": manifest_proof,
            },
            sort_keys=True,
        ),
        flush=True,
    )
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
                        "schema": "portable_e2e.moving_drive_mode_residual_recovery_failure.v1",
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
