#!/usr/bin/env python3
"""HH_260906 - Run the explicit causal route-conditioned J research ranker."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import traceback
from typing import Any, Mapping

REPO_IMPORT_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_IMPORT_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_IMPORT_ROOT))

from scripts.e2e import run_moving_drive_mode_residual_recovery as i_base


GPU_UUID = i_base.GPU_UUID
F_CHECKPOINT_SHA256 = i_base.F_CHECKPOINT_SHA256
H_ROOT_RELATIVE = i_base.H_ROOT_RELATIVE
H_PLAN_SHA256 = i_base.H_PLAN_SHA256
H_SUMMARY_SHA256 = i_base.H_SUMMARY_SHA256
H_MANIFEST_SHA256 = i_base.H_MANIFEST_SHA256
H_TRAIN_CACHE_SHA256 = i_base.H_TRAIN_CACHE_SHA256
H_DEV_CACHE_SHA256 = i_base.H_DEV_CACHE_SHA256
H1_REPORT_SHA256 = i_base.H1_REPORT_SHA256
H1_ARM = i_base.H1_ARM
I_ROOT_RELATIVE = (
    "runs/diagnostics/"
    "moving_drive_mode_residual_recovery_seed20260906_20260921_v1"
)
I_PLAN_SHA256 = "1fee39ef45d008f8082ad0e501ef1467e793c869469854e29d78c85495788a82"
I_SUMMARY_SHA256 = "67d13d580c43bc42bcee818339c64e207a84d7ed66d848176c3dac895126b0cf"
I_MANIFEST_SHA256 = "72b741f9bfe815e88cdcc85a0b7067e965230d9e3581b845226bc57926c1ad08"
RAW_DATASET_SHA256 = (
    "70fceaf0a6fea1b6ac455154bdc7b1113cfef15abc1a26ca7327ac2a186799e6"
)
OUTPUT_RELATIVE = (
    "runs/diagnostics/"
    "moving_causal_route_ranker_seed20260906_20260921_v1"
)
REPO_RELATIVE = (
    "source_worktrees/"
    "moving_causal_route_ranker_j_seed20260906_20260921_v1"
)
BASE_COMMIT = "b478f02e42b94bf04bffec5c8170e05edc33b0f8"
RIGHT_EPISODE = i_base.RIGHT_EPISODE
STRAIGHT_EPISODE = i_base.STRAIGHT_EPISODE
RIGHT_MAXIMUM = dict(i_base.RIGHT_MAXIMUM)
STRAIGHT_MAXIMUM = dict(i_base.STRAIGHT_MAXIMUM)
COMBINED_MAXIMUM = dict(i_base.COMBINED_MAXIMUM)
GEOMETRY_MINIMUM = dict(i_base.GEOMETRY_MINIMUM)
EPISODE_CONTRACT = {
    "carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_dfc36d890f2a": {
        "split": "train",
        "route_group": "town07_same_physical_route",
        "sample_count": 309,
        "sample_jsonl_sha256": "781ca35661ec018d9ca8dd228f4a9bda41ef3ebb11a21ee7958f2eaa09c660ca",
    },
    "carla_town07_town07_straight_s0000_p00_ClearNoon_s0000_f3f1a4d7b240": {
        "split": "train",
        "route_group": "town07_same_physical_route",
        "sample_count": 328,
        "sample_jsonl_sha256": "a73f1cd63bc09e55be8c4cf7bd05dfd23f1bf5cee935cba597730865a77da28b",
    },
    "carla_c_track_1_0_7_c_track_1_0_7_left_s0000_p01_ClearNoon_s0000_9b5bfc42d8e7": {
        "split": "train",
        "route_group": "c_track",
        "sample_count": 304,
        "sample_jsonl_sha256": "fc60a820dc87b1bcf8bf96de08b7c0de6177d8f24ee04fb44be248ac5d854adb",
    },
    "carla_town01_town01_right_s0001_p00_ClearNoon_s0001_1af7c32f1f1f": {
        "split": "train",
        "route_group": "town01",
        "sample_count": 534,
        "sample_jsonl_sha256": "4ec758139cc54f55f25a071a723298045648c165e864bd34e1b8604b17a71318",
    },
    RIGHT_EPISODE: {
        "split": "val",
        "route_group": "town03_dev_right",
        "sample_count": 337,
        "sample_jsonl_sha256": "7c27d8e05c15bd8cc9b4558ccd06bf2415f00a18795578abc6c933c4ba5e3ca1",
    },
    STRAIGHT_EPISODE: {
        "split": "val",
        "route_group": "town03_dev_straight",
        "sample_count": 328,
        "sample_jsonl_sha256": "9f4336dc580cefb05c0323bb8082c14b26ede9f703416cd0805f1b4b99e634ff",
    },
}
EXPECTED_TRAIN_DRIVE_BY_GROUP = {
    "town07_same_physical_route": 625,
    "c_track": 289,
    "town01": 318,
}
EXPECTED_TRAIN_POLICY_DRIVE_BY_GROUP = {
    "town07_same_physical_route": 625,
    "c_track": 0,
    "town01": 156,
}
EXPECTED_DEV_POLICY_DRIVE_BY_GROUP = {
    "town03_dev_right": 129,
    "town03_dev_straight": 328,
}
SOURCE_PATHS = tuple(
    dict.fromkeys(
        i_base.SOURCE_PATHS
        + (
            "portable_e2e/frozen_causal_route_ranker.py",
            "scripts/e2e/run_moving_causal_route_ranker.py",
        )
    )
)
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
EXPECTED_OVERLAYS = {
    "portable_e2e/evaluate.py",
    "portable_e2e/frozen_causal_route_ranker.py",
    "portable_e2e/frozen_drive_mode_residual_recovery.py",
    "portable_e2e/frozen_stopmix_drive_rank.py",
    "portable_e2e/frozen_stopmix_residual.py",
    "portable_e2e/frozen_stopmix_selector.py",
    "portable_e2e/frozen_stopmix_selector_recalibration.py",
    "portable_e2e/train.py",
    "scripts/e2e/diagnose_stopmix_training_oracles.py",
    "scripts/e2e/run_moving_causal_route_ranker.py",
    "scripts/e2e/run_moving_drive_mode_residual_recovery.py",
}
_NUMBER = re.compile(r"-?(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?")
RAW_PERSONAL_RELATIVE = (
    "dataset/prepared/moving-data-ab-seed20260906-20260921-v1/"
    "B_town07_augmented_train"
)
H_DATASET_RELATIVE = (
    "datasets/prepared/moving-data-ab-seed20260906-20260921-v1/"
    "B_town07_augmented_train"
)
GPU_LOCK_RELATIVE = "runs/campaigns/.gpu0_training.lock"


def require(value: Any, message: str) -> None:
    """HH_260906 - Raise one fail-closed runner contract error."""
    if not value:
        raise ValueError(message)


def utc() -> str:
    """HH_260906 - Return one explicit UTC evidence timestamp."""
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def encoded(value: Any) -> bytes:
    """HH_260906 - Encode canonical readable finite JSON with a final newline."""
    return (
        json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + "\n"
    ).encode()


def regular(path: Path) -> Path:
    """HH_260906 - Resolve one existing nonsymlink regular input file."""
    path = Path(path).absolute()
    require(path.is_file() and not path.is_symlink(), "regular file required")
    require(
        not any(parent.is_symlink() for parent in path.parents),
        "symlink parent is forbidden",
    )
    return path


def digest(path: Path) -> str:
    """HH_260906 - Stream one regular file into SHA256."""
    result = hashlib.sha256()
    with regular(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def _pairs(values):
    """HH_260906 - Reject duplicate JSON object keys while parsing evidence."""
    result = {}
    for key, value in values:
        require(key not in result, "duplicate JSON key: " + key)
        result[key] = value
    return result


def _finite_json_float(value: str, context: str) -> float:
    """HH_260906 - Decode one JSON float only when its binary value remains finite."""
    try:
        result = float(value)
    except (OverflowError, ValueError) as error:
        raise ValueError(context + " contains an invalid float") from error
    require(math.isfinite(result), context + " contains a nonfinite float")
    return result


def read_json(path: Path) -> dict[str, Any]:
    """HH_260906 - Parse one finite duplicate-key-free JSON document."""
    path = regular(path)
    return json.loads(
        path.read_text(),
        object_pairs_hook=_pairs,
        parse_float=lambda value: _finite_json_float(value, str(path)),
        parse_constant=lambda value: require(False, "nonfinite JSON " + value),
    )


def _space(text: str, position: int) -> int:
    """HH_260906 - Advance over JSON whitespace without allocating a value."""
    while position < len(text) and text[position] in " \t\r\n":
        position += 1
    return position


def _scan_string(text: str, position: int, context: str) -> int:
    """HH_260906 - Validate and skip one JSON string without decoding its payload."""
    require(position < len(text) and text[position] == '"', context + " string required")
    position += 1
    while position < len(text):
        character = text[position]
        code = ord(character)
        require(code >= 0x20, context + " contains a control character")
        if character == '"':
            return position + 1
        if character == "\\":
            position += 1
            require(position < len(text), context + " has a truncated escape")
            escape = text[position]
            require(escape in '"\\/bfnrtu', context + " has an invalid escape")
            if escape == "u":
                digits = text[position + 1 : position + 5]
                require(
                    len(digits) == 4
                    and all(value in "0123456789abcdefABCDEF" for value in digits),
                    context + " has an invalid unicode escape",
                )
                position += 4
        position += 1
    raise ValueError(context + " has an unterminated string")


def _scan_value(text: str, position: int, context: str) -> int:
    """HH_260906 - Validate and skip any JSON value without materializing it."""
    position = _space(text, position)
    require(position < len(text), context + " value is missing")
    character = text[position]
    if character == '"':
        return _scan_string(text, position, context)
    if character == "{":
        return _skip_object(text, position, context)
    if character == "[":
        position = _space(text, position + 1)
        if position < len(text) and text[position] == "]":
            return position + 1
        index = 0
        while True:
            position = _scan_value(text, position, f"{context}[{index}]")
            position = _space(text, position)
            require(position < len(text), context + " array is unterminated")
            if text[position] == "]":
                return position + 1
            require(text[position] == ",", context + " array separator required")
            position = _space(text, position + 1)
            index += 1
    for literal in ("true", "false", "null"):
        if text.startswith(literal, position):
            return position + len(literal)
    match = _NUMBER.match(text, position)
    require(match is not None, context + " contains an invalid or nonfinite value")
    return match.end()


def _skip_object(text: str, position: int, context: str) -> int:
    """HH_260906 - Validate an object and duplicate keys without retaining value spans."""
    require(position < len(text) and text[position] == "{", context + " object required")
    keys = set()
    position = _space(text, position + 1)
    if position < len(text) and text[position] == "}":
        return position + 1
    while True:
        key_start = position
        key_end = _scan_string(text, key_start, context + " key")
        key = _decode_key(text, key_start, key_end, context)
        require(key not in keys, context + " duplicates key " + key)
        keys.add(key)
        position = _space(text, key_end)
        require(position < len(text) and text[position] == ":", context + " colon required")
        position = _scan_value(text, _space(text, position + 1), context + "." + key)
        position = _space(text, position)
        require(position < len(text), context + " object is unterminated")
        if text[position] == "}":
            return position + 1
        require(text[position] == ",", context + " object separator required")
        position = _space(text, position + 1)


def _decode_key(text: str, start: int, end: int, context: str) -> str:
    """HH_260906 - Decode only an object key after its string syntax is validated."""
    value = json.loads(text[start:end])
    require(isinstance(value, str), context + " key must decode to text")
    return value


def _object_spans(
    text: str, position: int, context: str
) -> tuple[dict[str, tuple[int, int]], int]:
    """HH_260906 - Index object value spans while rejecting every duplicate key recursively."""
    require(position < len(text) and text[position] == "{", context + " object required")
    spans = {}
    position = _space(text, position + 1)
    if position < len(text) and text[position] == "}":
        return spans, position + 1
    while True:
        key_start = position
        key_end = _scan_string(text, key_start, context + " key")
        key = _decode_key(text, key_start, key_end, context)
        require(key not in spans, context + " duplicates key " + key)
        position = _space(text, key_end)
        require(position < len(text) and text[position] == ":", context + " colon required")
        value_start = _space(text, position + 1)
        value_end = _scan_value(text, value_start, context + "." + key)
        spans[key] = (value_start, value_end)
        position = _space(text, value_end)
        require(position < len(text), context + " object is unterminated")
        if text[position] == "}":
            return spans, position + 1
        require(text[position] == ",", context + " object separator required")
        position = _space(text, position + 1)


def _decode_slice(text: str, span: tuple[int, int], context: str) -> Any:
    """HH_260906 - Materialize one explicitly allowlisted projected JSON value only."""
    return json.loads(
        text[span[0] : span[1]],
        object_pairs_hook=_pairs,
        parse_float=lambda value: _finite_json_float(value, context),
        parse_constant=lambda value: require(False, context + " nonfinite " + value),
    )


def read_projected_json_line(line: str, context: str) -> dict[str, Any]:
    """HH_260906 - Project causal identity, ego and navigation fields without loading labels."""
    require(bool(line.strip()), context + " is blank")
    start = _space(line, 0)
    top, end = _object_spans(line, start, context)
    require(_space(line, end) == len(line), context + " has trailing JSON data")
    top_required = {
        "sample_id",
        "episode_id",
        "sequence_index",
        "anchor_timestamp_ns",
        "ego",
        "navigation",
    }
    require(top_required.issubset(top), context + " misses a projected top-level field")
    ego_spans, ego_end = _object_spans(line, top["ego"][0], context + ".ego")
    require(ego_end == top["ego"][1], context + ".ego projection boundary differs")
    navigation_spans, navigation_end = _object_spans(
        line, top["navigation"][0], context + ".navigation"
    )
    require(
        navigation_end == top["navigation"][1],
        context + ".navigation projection boundary differs",
    )
    ego_required = {
        "timestamp_ns",
        "linear_velocity_base_mps",
        "linear_acceleration_base_mps2",
    }
    navigation_required = {
        "command",
        "command_timestamp_ns",
        "route_source_timestamp_ns",
        "route_polyline_base_m",
    }
    require(
        ego_required.issubset(ego_spans)
        and navigation_required.issubset(navigation_spans),
        context + " misses a projected causal field",
    )
    return {
        "sample_id": _decode_slice(line, top["sample_id"], context + ".sample_id"),
        "episode_id": _decode_slice(line, top["episode_id"], context + ".episode_id"),
        "sequence_index": _decode_slice(
            line, top["sequence_index"], context + ".sequence_index"
        ),
        "anchor_timestamp_ns": _decode_slice(
            line, top["anchor_timestamp_ns"], context + ".anchor_timestamp_ns"
        ),
        "ego": {
            key: _decode_slice(line, ego_spans[key], context + ".ego." + key)
            for key in sorted(ego_required)
        },
        "navigation": {
            key: _decode_slice(
                line, navigation_spans[key], context + ".navigation." + key
            )
            for key in sorted(navigation_required)
        },
    }


def write_new(path: Path, value: Any) -> None:
    """HH_260906 - Create one JSON artifact without overwriting prior evidence."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(encoded(value))
        stream.flush()
        os.fsync(stream.fileno())


def save_new(path: Path, value: Any, torch) -> None:
    """HH_260906 - Create one torch artifact without overwriting prior evidence."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        torch.save(value, stream)
        stream.flush()
        os.fsync(stream.fileno())


def copy_new(source: Path, target: Path) -> None:
    """HH_260906 - Archive one source file and verify identical transferred bytes."""
    source = regular(source)
    target = Path(target)
    target.parent.mkdir(parents=True, exist_ok=True)
    with source.open("rb") as source_stream, target.open("xb") as target_stream:
        shutil.copyfileobj(source_stream, target_stream, 1024 * 1024)
        target_stream.flush()
        os.fsync(target_stream.fileno())
    require(digest(source) == digest(target), "archived source differs")


def manifest(output: Path) -> dict[str, Any]:
    """HH_260906 - Seal every create-only output file in a sorted SHA256 manifest."""
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


def verify_exact_manifest(
    root: Path, expected_sha256: str, label: str
) -> dict[str, Any]:
    """HH_260906 - Verify one exact sorted artifact manifest and its full inventory."""
    root = Path(root).absolute()
    require(root.is_dir() and not root.is_symlink(), label + " root differs")
    manifest_path = root / "SHA256SUMS"
    require(digest(manifest_path) == expected_sha256, label + " manifest bytes differ")
    rows = []
    for line in regular(manifest_path).read_text().splitlines():
        require(
            len(line) > 66 and line[64:66] == "  ",
            label + " manifest row is invalid",
        )
        value, name = line[:64], line[66:]
        require(
            len(value) == 64
            and all(character in "0123456789abcdef" for character in value)
            and bool(name)
            and not Path(name).is_absolute()
            and ".." not in Path(name).parts,
            label + " manifest row is unsafe",
        )
        rows.append((name, value))
    require(
        [name for name, _value in rows]
        == sorted(name for name, _value in rows)
        and len({name for name, _value in rows}) == len(rows),
        label + " manifest paths are not unique and sorted",
    )
    for name, value in rows:
        require(digest(root / name) == value, label + " artifact differs: " + name)
    actual = sorted(
        path.relative_to(root).as_posix()
        for path in root.rglob("*")
        if path.is_file() and path.name != "SHA256SUMS"
    )
    require(
        actual == [name for name, _value in rows],
        label + " artifact inventory differs",
    )
    return {
        "sha256": expected_sha256,
        "verified_file_count": len(rows),
        "all_files_verified": True,
    }


def git_identity(repo: Path) -> dict[str, Any]:
    """HH_260906 - Pin the detached base, exact overlays and committed transitive bytes."""
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
    observed = {row[3:] for row in status}
    require(head == BASE_COMMIT, "detached J base commit differs")
    require(observed == EXPECTED_OVERLAYS, "J source worktree overlay set differs")
    committed_source_sha256 = {}
    for name in SOURCE_PATHS:
        if name in EXPECTED_OVERLAYS:
            continue
        committed = subprocess.check_output(
            ["git", "-c", "protocol.allow=never", "show", f"HEAD:{name}"],
            cwd=repo,
            env=environment,
            timeout=20,
        )
        committed_sha256 = hashlib.sha256(committed).hexdigest()
        require(
            digest(repo / name) == committed_sha256,
            "base transitive source differs from detached commit: " + name,
        )
        committed_source_sha256[name] = committed_sha256
    return {
        "base_commit": head,
        "porcelain": status,
        "overlay_paths": sorted(observed),
        "base_transitive_source_sha256": committed_source_sha256,
    }


def verify_external_gpu_lock(workspace: Path) -> dict[str, Any]:
    """HH_260906 - Require a caller-held GPU0 flock before any CUDA import or output."""
    lock_path = (workspace / GPU_LOCK_RELATIVE).absolute()
    expected = os.environ.get("PORTABLE_E2E_EXTERNAL_FLOCK_PATH")
    require(expected == str(lock_path), "external GPU0 flock path declaration differs")
    require(lock_path.is_file() and not lock_path.is_symlink(), "regular GPU0 lock file required")
    with lock_path.open("rb") as stream:
        try:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            held = True
        else:
            held = False
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
    require(held, "GPU0 lock is not externally held")
    return {
        "path": str(lock_path),
        "declared_by_environment": True,
        "independent_nonblocking_acquire_failed": True,
        "external_lock_held": True,
    }


def safe_relative(root: Path, raw: Any, context: str) -> Path:
    """HH_260906 - Resolve one manifest path strictly beneath its dataset root."""
    require(
        isinstance(raw, str)
        and bool(raw)
        and not Path(raw).is_absolute()
        and ".." not in Path(raw).parts,
        context + " is unsafe",
    )
    result = (root / raw).absolute()
    require(result.is_relative_to(root.absolute()), context + " escaped root")
    return regular(result)


def sidecar_payload(value, core) -> dict[str, Any]:
    """HH_260906 - Serialize only the input sidecar ABI and non-input fold metadata."""
    return {
        "schema": "portable_e2e.causal_route_input_sidecar.v1",
        "split": value.split,
        "sample_ids": list(value.sample_ids),
        "route_groups_non_ranker_metadata": list(value.route_groups),
        "source_dataset_sha256": value.source_dataset_sha256,
        "digest": value.digest(),
        "ranker_input_tensor_keys": [
            "route_xy",
            "route_mask",
            "ego_speed_mps",
            "ego_acceleration_mps2",
        ],
        "policy_input_tensor_keys": [
            "command",
            "straight_ranker_mask",
            "route_heading_change_rad",
        ],
        "tensors": {
            name: getattr(value, name).detach().cpu().clone()
            for name in core._TENSOR_FIELDS
        },
        "contains_target_label_future_ego_image_fused_map_or_episode_tensor": False,
        **DENIALS,
    }


def build_causal_sidecars(
    dataset_root: Path,
    caches: Mapping[str, Any],
    core,
    *,
    expected_dataset_sha256: str = RAW_DATASET_SHA256,
    episode_contract: Mapping[str, Mapping[str, Any]] = EPISODE_CONTRACT,
    expected_total_count: int = 2140,
):
    """HH_260906 - Verify the full manifest chain and build exact 2140-row sidecars."""
    dataset_root = Path(dataset_root).absolute()
    require(
        type(expected_total_count) is int
        and expected_total_count > 0
        and dataset_root.is_dir()
        and not dataset_root.is_symlink(),
        "regular raw dataset directory required",
    )
    dataset_path = dataset_root / "dataset.json"
    require(
        digest(dataset_path) == expected_dataset_sha256,
        "exact raw dataset.json SHA256 differs",
    )
    dataset = read_json(dataset_path)
    references = dataset.get("episodes")
    require(
        dataset.get("schema_id") == "autoware-e2e.common10.dataset.v1"
        and dataset.get("contract_id") == "common_10hz_v1"
        and isinstance(references, list)
        and len(references) == len(episode_contract),
        "raw dataset schema, contract or episode count differs",
    )
    rows_by_split = {"train": {}, "val": {}}
    ordered_ids_by_split = {"train": [], "val": []}
    group_by_split = {"train": {}, "val": {}}
    policy_by_split = {"train": {}, "val": {}}
    episode_proofs = []
    reason_counts = Counter()
    route_valid_counts = Counter()
    route_source_age_ns = []
    seen_episodes = set()
    for reference in references:
        require(isinstance(reference, dict), "episode reference must be a mapping")
        episode_path = safe_relative(
            dataset_root, reference.get("manifest"), "episode manifest path"
        )
        episode_sha256 = digest(episode_path)
        require(
            reference.get("sha256") == episode_sha256,
            "dataset to episode manifest SHA256 chain differs",
        )
        episode = read_json(episode_path)
        episode_id = episode.get("episode_id")
        require(
            episode_id in episode_contract and episode_id not in seen_episodes,
            "unknown or duplicate sealed episode identity",
        )
        seen_episodes.add(episode_id)
        expected = episode_contract[episode_id]
        sample_count = expected["sample_count"]
        require(
            reference.get("episode_id") == episode_id
            and episode.get("schema_id") == "autoware-e2e.common10.episode.v1"
            and episode.get("split") == expected["split"]
            and episode.get("sample_count") == sample_count
            and episode.get("sample_jsonl_sha256")
            == expected["sample_jsonl_sha256"],
            "sealed episode split, count or sample SHA256 differs",
        )
        samples_path = safe_relative(
            episode_path.parent,
            episode.get("sample_jsonl"),
            "sample JSONL path",
        )
        sample_sha256 = digest(samples_path)
        require(
            sample_sha256
            == episode.get("sample_jsonl_sha256")
            == expected["sample_jsonl_sha256"],
            "episode to sample JSONL SHA256 chain differs",
        )
        split = expected["split"]
        require(split in ("train", "val"), "TEST or unknown split is forbidden")
        turn_latched = False
        observed_count = 0
        with samples_path.open() as stream:
            for index, line in enumerate(stream):
                raw = read_projected_json_line(
                    line, f"{episode_id}.samples[{index}]"
                )
                require(
                    raw.get("episode_id") == episode_id
                    and raw.get("sequence_index") == index,
                    "raw episode or contiguous sequence identity differs",
                )
                causal = core.extract_causal_row(raw)
                causality = causal["causality"]
                route_source_age_ns.append(
                    causality["anchor_timestamp_ns"]
                    - causality["route_source_timestamp_ns"]
                )
                sample_id = causal["sample_id"]
                require(
                    sample_id not in rows_by_split[split],
                    "duplicate raw sample ID within split",
                )
                use_ranker, turn_latched, reason = core.causal_straight_decision(
                    causal["command"],
                    causal["route_heading_change_rad"],
                    turn_latched,
                )
                rows_by_split[split][sample_id] = causal
                ordered_ids_by_split[split].append(sample_id)
                group_by_split[split][sample_id] = expected["route_group"]
                policy_by_split[split][sample_id] = use_ranker
                reason_counts[(split, expected["route_group"], reason)] += 1
                route_valid_counts[len(causal["route_xy"])] += 1
                observed_count += 1
        require(observed_count == sample_count, "sample JSONL row count differs")
        episode_proofs.append(
            {
                "episode_id": episode_id,
                "split": split,
                "route_group_non_ranker_metadata": expected["route_group"],
                "episode_manifest": episode_path.relative_to(dataset_root).as_posix(),
                "episode_manifest_sha256": episode_sha256,
                "sample_jsonl": samples_path.relative_to(dataset_root).as_posix(),
                "sample_jsonl_sha256": sample_sha256,
                "sample_count": observed_count,
            }
        )
    require(set(episode_contract) == seen_episodes, "sealed episode set differs")
    require(
        len(route_source_age_ns) == expected_total_count
        and all(value >= 0 for value in route_source_age_ns),
        "route source timestamp causality coverage differs",
    )
    expected_all = set(caches["train"].sample_ids) | set(caches["val"].sample_ids)
    raw_all = set(rows_by_split["train"]) | set(rows_by_split["val"])
    require(
        len(expected_all) == len(raw_all) == expected_total_count
        and expected_all == raw_all
        and set(rows_by_split["train"]) == set(caches["train"].sample_ids)
        and set(rows_by_split["val"]) == set(caches["val"].sample_ids)
        and tuple(ordered_ids_by_split["train"])
        == tuple(caches["train"].sample_ids)
        and tuple(ordered_ids_by_split["val"])
        == tuple(caches["val"].sample_ids)
        and not (set(rows_by_split["train"]) & set(rows_by_split["val"])),
        "raw/H cache ordered sample coverage is not exact expected total",
    )
    sidecars = {
        split: core.make_sidecar(
            split,
            caches[split].sample_ids,
            rows_by_split[split],
            group_by_split[split],
            policy_by_split[split],
        )
        for split in ("train", "val")
    }
    proof = {
        "schema": "portable_e2e.causal_route_sidecar_build_proof.v1",
        "dataset_json_sha256": expected_dataset_sha256,
        "dataset_to_episode_to_samples_chain_verified": True,
        "episode_proofs": episode_proofs,
        "cache_sample_count": expected_total_count,
        "raw_sample_count": expected_total_count,
        "ordered_sample_coverage": {
            split: tuple(ordered_ids_by_split[split])
            == tuple(caches[split].sample_ids)
            for split in ("train", "val")
        },
        "sidecar_sha256": {
            split: sidecars[split].digest() for split in ("train", "val")
        },
        "route_valid_point_histogram": {
            str(key): value for key, value in sorted(route_valid_counts.items())
        },
        "route_source_timestamp_causality": {
            "verified_count": len(route_source_age_ns),
            "future_count": 0,
            "anchor_minus_route_source_minimum_ns": min(route_source_age_ns),
            "anchor_minus_route_source_maximum_ns": max(route_source_age_ns),
        },
        "policy_reason_counts": [
            {
                "split": key[0],
                "route_group_non_ranker_metadata": key[1],
                "reason": key[2],
                "count": count,
            }
            for key, count in sorted(reason_counts.items())
        ],
        "ranker_input_field_allowlist": [
            "navigation.route_polyline_base_m",
            "ego.linear_velocity_base_mps[current]",
            "ego.linear_acceleration_base_mps2[current]",
        ],
        "causality_validation_field_allowlist": [
            "anchor_timestamp_ns",
            "ego.timestamp_ns",
            "navigation.command_timestamp_ns",
            "navigation.route_source_timestamp_ns",
        ],
        "forbidden_ranker_fields_serialized": [],
        "test_read_or_used": False,
    }
    return sidecars, proof


def subset_cache(data, indices, core):
    """HH_260906 - Construct one immutable metric-only cache subset."""
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
    """HH_260906 - Keep only the fixed evaluation fields needed by the gate."""
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
    """HH_260906 - Evaluate the fixed Town03 right, straight and combined DEV groups."""
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
        name: standard_metrics(subset_cache(dev, indices, core), logits[indices], core)
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


def strict_gate(metrics, geometry, preservation):
    """HH_260906 - Apply the unchanged nine metrics, geometry and exact preservation gate."""
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
    preservation_checks = [
        {
            "split": split,
            "mode_change_count": value["mode_change_count"],
            "fallback_row_logit_mismatch_count": value[
                "fallback_row_logit_mismatch_count"
            ],
            "reranked_drive_group_maximum_mismatch_count": value[
                "reranked_drive_group_maximum_mismatch_count"
            ],
            "pass": value["pass"],
        }
        for split, value in (("train", preservation["train"]), ("val", preservation["val"]))
    ]
    metric_count = sum(row["pass"] for row in metric_checks)
    geometry_count = sum(row["pass"] for row in geometry_checks)
    preservation_count = sum(row["pass"] for row in preservation_checks)
    passed = metric_count == 9 and geometry_count == 3 and preservation_count == 2
    return {
        "status": (
            "PASS_STRICT_9_METRICS_GEOMETRY_AND_PRESERVATION_NOT_PROMOTED"
            if passed
            else "FAIL_STRICT_PERFORMANCE_OR_PRESERVATION_GATE_NOT_PROMOTED"
        ),
        "pass": passed,
        "metric_pass_count": metric_count,
        "metric_check_count": 9,
        "geometry_pass_count": geometry_count,
        "geometry_check_count": 3,
        "preservation_pass_count": preservation_count,
        "preservation_check_count": 2,
        "metric_checks": metric_checks,
        "geometry_checks": geometry_checks,
        "preservation_checks": preservation_checks,
        "promotion_eligible": False,
        "stop_guard": {
            "status": "NOT_EVALUABLE_NO_SEALED_STATIONARY_DEV_EPISODE",
            "pass": None,
            "required_for_promotion": True,
        },
    }


def validate_drive_group_counts(caches, sidecars) -> dict[str, Any]:
    """HH_260906 - Prove all 1232 F-DRIVE rows and zero STOP rows enter group loss."""
    import torch

    selected = caches["train"].original_logits.argmax(dim=1) < 6
    train_drive = {
        group: int(
            (
                selected
                & torch.tensor(
                    [value == group for value in sidecars["train"].route_groups],
                    dtype=torch.bool,
                )
            ).sum()
        )
        for group in EXPECTED_TRAIN_DRIVE_BY_GROUP
    }
    require(
        train_drive == EXPECTED_TRAIN_DRIVE_BY_GROUP
        and int(selected.sum()) == 1232
        and int((~selected).sum()) == 243,
        "F-DRIVE physical group or STOP exclusion partition differs",
    )
    train_policy = {
        group: int(
            (
                selected
                & sidecars["train"].straight_ranker_mask
                & torch.tensor(
                    [value == group for value in sidecars["train"].route_groups],
                    dtype=torch.bool,
                )
            ).sum()
        )
        for group in EXPECTED_TRAIN_POLICY_DRIVE_BY_GROUP
    }
    dev_drive = caches["val"].original_logits.argmax(dim=1) < 6
    dev_policy = {
        group: int(
            (
                dev_drive
                & sidecars["val"].straight_ranker_mask
                & torch.tensor(
                    [value == group for value in sidecars["val"].route_groups],
                    dtype=torch.bool,
                )
            ).sum()
        )
        for group in EXPECTED_DEV_POLICY_DRIVE_BY_GROUP
    }
    require(
        train_policy == EXPECTED_TRAIN_POLICY_DRIVE_BY_GROUP
        and dev_policy == EXPECTED_DEV_POLICY_DRIVE_BY_GROUP,
        "causal policy-applied TRAIN or DEV DRIVE count differs",
    )
    return {
        "loss_drive_count_by_train_physical_route_group": train_drive,
        "policy_applied_drive_count_by_train_physical_route_group": train_policy,
        "policy_applied_drive_count_by_dev_maneuver_metadata": dev_policy,
        "limitation": "C-track has zero policy-applied held DRIVE rows, so three folds validate the shared scorer training protocol, not three-route applied-policy performance.",
    }


def run(workspace: Path, repo: Path) -> int:
    """HH_260906 - Execute one create-only GPU0 J study without TEST or promotion."""
    workspace = Path(workspace).absolute()
    repo = Path(repo).absolute()
    output = workspace / OUTPUT_RELATIVE
    h_root = workspace / H_ROOT_RELATIVE
    i_root = workspace / I_ROOT_RELATIVE
    python_prefix = Path(sys.prefix).absolute()
    python_executable = Path(sys.executable).absolute()
    require(
        repo == workspace / REPO_RELATIVE
        and REPO_IMPORT_ROOT == repo
        and repo.is_dir()
        and not repo.is_symlink(),
        "fixed regular J source worktree required",
    )
    require(
        python_prefix == workspace / "venvs/py312"
        and Path(sys.base_prefix).absolute() != python_prefix
        and python_executable.is_relative_to(python_prefix)
        and os.environ.get("PYTHONNOUSERSITE") == "1"
        and os.environ.get("PYTHONDONTWRITEBYTECODE") == "1"
        and sys.dont_write_bytecode
        and os.environ.get("CUDA_VISIBLE_DEVICES") == GPU_UUID,
        "fixed isolated personal venv or GPU0 process environment differs",
    )
    python_environment = {
        "prefix": str(python_prefix),
        "base_prefix": str(Path(sys.base_prefix).absolute()),
        "executable": str(python_executable),
        "python_no_user_site": os.environ["PYTHONNOUSERSITE"],
        "python_dont_write_bytecode": os.environ["PYTHONDONTWRITEBYTECODE"],
        "cuda_visible_devices": os.environ["CUDA_VISIBLE_DEVICES"],
        "isolated_personal_venv": True,
    }
    require(
        output
        == Path.home() / "personal/hwanhong/portable_e2e" / OUTPUT_RELATIVE,
        "fixed personal create-only J output root required",
    )
    require(not output.exists(), "create-only J output already exists")
    require(not (h_root / "failed.json").exists(), "failed H run cannot be a parent")
    require(
        not (i_root / "failed.json").exists(),
        "failed I run cannot be reproducibility context",
    )
    h_manifest = i_base.verify_exact_h_manifest(h_root)
    i_manifest = verify_exact_manifest(i_root, I_MANIFEST_SHA256, "I")
    h_plan_path = h_root / "plan.json"
    h_summary_path = h_root / "summary.json"
    h_report_path = h_root / H1_ARM / "report.json"
    i_plan_path = i_root / "plan.json"
    i_summary_path = i_root / "summary.json"
    require(digest(h_plan_path) == H_PLAN_SHA256, "exact H plan bytes differ")
    require(digest(h_summary_path) == H_SUMMARY_SHA256, "exact H summary bytes differ")
    require(digest(h_report_path) == H1_REPORT_SHA256, "exact H1 report bytes differ")
    require(digest(i_plan_path) == I_PLAN_SHA256, "exact I plan bytes differ")
    require(digest(i_summary_path) == I_SUMMARY_SHA256, "exact I summary bytes differ")
    h_plan = read_json(h_plan_path)
    h_summary = read_json(h_summary_path)
    h_report = read_json(h_report_path)
    i_plan = read_json(i_plan_path)
    i_summary = read_json(i_summary_path)
    require(
        h_plan.get("schema")
        == "portable_e2e.moving_selector_sampling_recalibration_plan.v1"
        and h_summary.get("schema")
        == "portable_e2e.moving_selector_sampling_recalibration_summary.v1"
        and h_summary.get("status")
        == "PAIRED_HEAD_ONLY_FITS_COMPLETE_NOT_PROMOTED"
        and h_summary.get("decision", {}).get("status")
        == "FAIL_SELECTOR_SAMPLING_RECOVERY_SIGNAL"
        and h_summary.get("decision", {}).get("recovery_signal") is False
        and h_report.get("arm") == H1_ARM
        and h_report.get("steps") == 8280
        and h_report.get("samples_seen") == 33120
        and h_report.get("source_checkpoint_sha256") == F_CHECKPOINT_SHA256,
        "H semantic parent contract differs",
    )
    require(
        i_plan.get("schema")
        == "portable_e2e.moving_drive_mode_residual_recovery_plan.v1"
        and i_summary.get("schema")
        == "portable_e2e.moving_drive_mode_residual_recovery_summary.v1"
        and i_summary.get("status")
        == "PAIRED_DRIVE_MODE_RESIDUAL_FITS_COMPLETE_NOT_PROMOTED"
        and i_summary.get("decision", {}).get("status")
        == "FAIL_DRIVE_MODE_RESIDUAL_RECOVERY_SIGNAL"
        and i_summary.get("decision", {}).get("recovery_signal") is False,
        "I reproducibility-context semantic contract differs",
    )
    require(
        h_plan.get("dataset", {}).get("path") == H_DATASET_RELATIVE,
        "exact H historical dataset path declaration differs",
    )
    raw_root = (workspace.parent / RAW_PERSONAL_RELATIVE).resolve(strict=True)
    require(
        raw_root.is_dir() and not raw_root.is_symlink(),
        "canonical raw prepared dataset path differs",
    )
    external_lock = verify_external_gpu_lock(workspace)
    gpu_preflight = i_base.preflight_gpu()
    source = {name: digest(repo / name) for name in SOURCE_PATHS}
    git_proof = git_identity(repo)
    shared_h_source = {
        name: digest(h_root / "source" / name)
        for name in SOURCE_PATHS
        if (h_root / "source" / name).is_file()
    }
    require(
        all(source[name] == value for name, value in shared_h_source.items()),
        "J shared source differs from exact H parent source",
    )
    shared_i_source = {
        name: digest(i_root / "source" / name)
        for name in SOURCE_PATHS
        if (i_root / "source" / name).is_file()
    }
    require(
        set(shared_i_source) == set(i_base.SOURCE_PATHS)
        and all(source[name] == value for name, value in shared_i_source.items()),
        "J inherited source differs from exact I reproducibility context",
    )
    sys.path.insert(0, str(repo))
    import torch
    from portable_e2e import frozen_causal_route_ranker as core
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
        split: i_base.load_cache(
            h_root / "cache" / f"{split}.pt",
            split,
            h_plan,
            h_summary,
            core,
            torch,
        )
        for split in ("train", "val")
    }
    sidecars, sidecar_proof = build_causal_sidecars(raw_root, caches, core)
    core.validate_pair_and_sidecars(
        caches["train"], caches["val"], sidecars["train"], sidecars["val"]
    )
    drive_count_proof = validate_drive_group_counts(caches, sidecars)
    cache_before = {split: value.digest() for split, value in caches.items()}
    sidecar_before = {split: value.digest() for split, value in sidecars.items()}
    geometry = dict(h_report["geometry"])
    require(
        all(geometry[name] >= minimum for name, minimum in GEOMETRY_MINIMUM.items())
        and geometry["sample_count"] == 665,
        "inherited F candidate geometry contract differs",
    )
    plan = {
        "schema": "portable_e2e.moving_causal_route_ranker_plan.v1",
        "experiment_id": "hh260921-moving-causal-route-ranker-seed20260906-v1",
        "declared_at_utc": utc(),
        "output": OUTPUT_RELATIVE,
        "repo_root": str(repo.relative_to(workspace)),
        "git": git_proof,
        "gpu": {"physical_index": 0, "uuid": GPU_UUID},
        "python_environment": python_environment,
        "external_gpu_lock": external_lock,
        "source_sha256": source,
        "shared_h_source_sha256": shared_h_source,
        "shared_i_source_sha256": shared_i_source,
        "parent_h": {
            "root": H_ROOT_RELATIVE,
            "plan_sha256": H_PLAN_SHA256,
            "summary_sha256": H_SUMMARY_SHA256,
            "manifest": h_manifest,
            "train_cache_sha256": H_TRAIN_CACHE_SHA256,
            "dev_cache_sha256": H_DEV_CACHE_SHA256,
        },
        "parent_i_reproducibility_context": {
            "root": I_ROOT_RELATIVE,
            "plan_sha256": I_PLAN_SHA256,
            "summary_sha256": I_SUMMARY_SHA256,
            "manifest": i_manifest,
            "data_or_weight_dependency": False,
            "purpose": "Seal the immediately preceding failed I study as context; J data and frozen F candidates remain bound through exact H.",
        },
        "raw_sidecar": sidecar_proof,
        "drive_and_policy_count_proof": drive_count_proof,
        "fit": {
            "seed": 20260906,
            "leave_one_route_group_out": list(core.ROUTE_GROUPS),
            "fold_fit_steps": core.FIT_STEPS,
            "final_fit_steps": core.FIT_STEPS,
            "learning_rate": core.LEARNING_RATE,
            "shared_nonnegative_weight_count": core.WEIGHT_COUNT,
            "features": list(core.FEATURE_NAMES),
            "loss_rows": "All and only original-F DRIVE1232 rows; route-group balanced; STOP243 excluded.",
        },
        "selection_policy": {
            "straight_heading_maximum_rad": core.STRAIGHT_HEADING_MAXIMUM_RAD,
            "turn_latch_heading_minimum_rad": core.TURN_LATCH_HEADING_MINIMUM_RAD,
            "turn_evidence_fallback": "Exact full K12 F logits from first current command/route turn evidence onward.",
            "causal_prefix_limitation": "A causally indistinguishable pre-turn straight prefix may be reranked; no episode/map identity is used to rewrite the past.",
            "threshold_origin": "Predeclared exploratory TRAIN-input heuristic; no DEV outcome or TEST selected it.",
        },
        "strict_gate": {
            "right_C_maximum": RIGHT_MAXIMUM,
            "straight_D_maximum": STRAIGHT_MAXIMUM,
            "combined_D_maximum": COMBINED_MAXIMUM,
            "geometry_minimum": GEOMETRY_MINIMUM,
            "preservation": "Exact fallback K12 rows, DRIVE/STOP mode and reranked DRIVE group maximum on TRAIN and DEV.",
        },
        "dev_policy": "The sealed CPU DEV cache is loaded before fit only for integrity, identity coverage and F-mode counts. No DEV outcome or metric drives fit or selection. Device transfer and one DEV evaluation phase follow all three fixed TRAIN folds and the final all-TRAIN fit; that phase performs one combined full-split reduction plus two sealed subgroup reductions over the same final logits.",
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
    for source_path, name in (
        (i_plan_path, "I_plan.json"),
        (i_summary_path, "I_summary.json"),
        (i_root / "SHA256SUMS", "I_SHA256SUMS"),
    ):
        copy_new(source_path, output / "parent_i" / name)
    for split in ("train", "val"):
        save_new(
            output / "sidecar" / f"{split}.pt",
            sidecar_payload(sidecars[split], core),
            torch,
        )
    write_new(
        output / "started.json",
        {
            "schema": "portable_e2e.moving_causal_route_ranker_started.v1",
            "started_at_utc": utc(),
            "gpu_preflight": gpu_preflight,
            "python_environment": python_environment,
            "external_gpu_lock": external_lock,
            "plan_sha256": digest(output / "plan.json"),
            **DENIALS,
        },
    )
    projected_costs_fn, projection_proof = oracle_helper.projected_costs(loss_module)
    history_path = output / "history.jsonl"
    with history_path.open("x") as history:
        def progress(phase, row):
            """HH_260906 - Append one phase-bound optimizer row to durable evidence."""
            history.write(
                json.dumps(
                    {"phase": phase, **row}, sort_keys=True, allow_nan=False
                )
                + "\n"
            )
            history.flush()

        result = core.fit_causal_route_ranker(
            caches["train"],
            caches["val"],
            sidecars["train"],
            sidecars["val"],
            projected_costs_fn=projected_costs_fn,
            projection_proof=projection_proof,
            device=device,
            callback=progress,
        )
    require(
        result["report"]["train_partition"]["drive_count_by_physical_route_group"]
        == EXPECTED_TRAIN_DRIVE_BY_GROUP,
        "core/runner TRAIN group partition differs",
    )
    require(
        [
            (row["held_route_group"], row["fit_drive_count"])
            for row in result["report"]["train_only_leave_one_route_group_out"]
        ]
        == [
            ("town07_same_physical_route", 607),
            ("c_track", 943),
            ("town01", 914),
        ]
        and [
            (row["held_route_group"], row["held_policy_straight_drive_count"])
            for row in result["report"]["train_only_leave_one_route_group_out"]
        ]
        == [
            ("town07_same_physical_route", 625),
            ("c_track", 0),
            ("town01", 156),
        ]
        and result["report"]["train_partition"][
            "all_fold_and_final_optimization_row_exposures"
        ]
        == 946176
        and result["report"]["train_partition"][
            "stop_optimization_row_exposures"
        ]
        == 0,
        "LOO/final DRIVE exposure or STOP exclusion proof differs",
    )
    metrics = subgroup_metrics(
        caches["val"],
        result["final_logits_cpu"]["val"],
        core,
        result["report"]["final_dev_metrics"],
    )
    gate = strict_gate(metrics, geometry, result["report"]["final_preservation"])
    result["report"]["history"] = {
        "path": "history.jsonl",
        "row_count": 4 * core.FIT_STEPS,
        "sha256": digest(history_path),
    }
    result["report"]["subgroup_metrics"] = metrics
    result["report"]["geometry"] = geometry
    result["report"]["strict_gate"] = gate
    result["report"]["dev_post_freeze_evaluation_phase"] = {
        "phase_count": 1,
        "combined_full_split_reduction_count": 1,
        "sealed_subgroup_reduction_count": 2,
        "total_metric_reduction_count": 3,
        "same_final_logits_reused": True,
        "intermediate_selection_count": 0,
    }
    write_new(output / "report.json", result["report"])
    save_new(
        output / "scorer_only.pt",
        {
            "artifact_id": core.ARTIFACT_ID,
            "source_checkpoint_sha256": F_CHECKPOINT_SHA256,
            "scorer_state_dict": result["scorer_state_dict"],
            "runtime_checkpoint": False,
            "research_ranker_only": True,
            **DENIALS,
        },
        torch,
    )
    save_new(
        output / "final_logits.pt",
        {
            "artifact_id": "portable_e2e.frozen_f_causal_route_ranker_logits.v1",
            "source_checkpoint_sha256": F_CHECKPOINT_SHA256,
            "logits": result["final_logits_cpu"],
            **DENIALS,
        },
        torch,
    )
    require(
        {split: value.digest() for split, value in caches.items()} == cache_before
        and {split: value.digest() for split, value in sidecars.items()}
        == sidecar_before,
        "shared cache or causal sidecar changed",
    )
    summary = {
        "schema": "portable_e2e.moving_causal_route_ranker_summary.v1",
        "status": "CAUSAL_ROUTE_RANKER_FIT_COMPLETE_NOT_PROMOTED",
        "completed_at_utc": utc(),
        "plan_sha256": digest(output / "plan.json"),
        "report_sha256": digest(output / "report.json"),
        "source_sha256": source,
        "shared_i_source_sha256": shared_i_source,
        "gpu_preflight": gpu_preflight,
        "python_environment": python_environment,
        "external_gpu_lock": external_lock,
        "torch": {
            "version": str(torch.__version__),
            "visible_device_count": torch.cuda.device_count(),
            "device_name": properties.name,
            "device_uuid": observed_uuid,
        },
        "raw_sidecar_proof": sidecar_proof,
        "parent_i_reproducibility_context": plan[
            "parent_i_reproducibility_context"
        ],
        "cache_sha256_before": cache_before,
        "cache_sha256_after": {
            split: value.digest() for split, value in caches.items()
        },
        "sidecar_sha256_before": sidecar_before,
        "sidecar_sha256_after": {
            split: value.digest() for split, value in sidecars.items()
        },
        "strict_gate": gate,
        "decision": {
            "status": (
                "CAUSAL_ROUTE_RANKER_RECOVERY_SIGNAL_EXPLORATORY"
                if gate["pass"]
                else "FAIL_CAUSAL_ROUTE_RANKER_RECOVERY_SIGNAL"
            ),
            "recovery_signal": gate["pass"],
            "promotion_eligible": False,
            "reason": "Even a strict DEV pass remains exploratory because Town03 DEV has been repeatedly observed and stationary, TEST and closed-loop evidence are absent.",
        },
        **DENIALS,
    }
    write_new(output / "summary.json", summary)
    manifest_proof = manifest(output)
    print(
        json.dumps(
            {
                "event": "J_COMPLETE",
                "output": str(output),
                "summary_sha256": digest(output / "summary.json"),
                "manifest": manifest_proof,
            },
            sort_keys=True,
        ),
        flush=True,
    )
    return 0


def main(argv=None) -> int:
    """HH_260906 - Parse the fixed workspace and source-worktree runner arguments."""
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
                        "schema": "portable_e2e.moving_causal_route_ranker_failure.v1",
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
