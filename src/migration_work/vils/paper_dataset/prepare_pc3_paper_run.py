#!/usr/bin/env python3
"""Create one non-overwriting PC3 paper-run directory and freeze its acquisition profile."""

# HH_260814 - Add paper trial identity without changing the existing evidence collector or its rollback behavior.

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
from typing import Any, Sequence


SAFE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}")
STAGES = ("1", "1.5", "2", "3", "4", "5", "sil")
MODES = (
    "real_only",
    "shadow",
    "validated_candidate",
    "hybrid_optional",
    "hybrid_required",
    "sil",
)
# HH_260814 - Keep preparation aligned with the staged safety table; required mode
# remains intentionally unavailable until its accepted-status policy is reviewed.
STAGE_MODE_MATRIX = {
    "1": frozenset(("real_only",)),
    "1.5": frozenset(("real_only",)),
    "2": frozenset(("shadow",)),
    "3": frozenset(("validated_candidate",)),
    "4": frozenset(("hybrid_optional",)),
    "5": frozenset(("hybrid_optional",)),
    "sil": frozenset(("sil",)),
}


class PreparationError(RuntimeError):
    """Raised when a paper run cannot be prepared without ambiguity."""


def utc_text() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def validate_id(label: str, value: str) -> str:
    if not SAFE_ID.fullmatch(value):
        raise PreparationError(
            f"{label} must be 1-64 characters using letters, digits, dot, underscore, or hyphen"
        )
    return value


def validate_stage_mode(stage: str, mode: str) -> tuple[str, str]:
    allowed = STAGE_MODE_MATRIX.get(stage)
    if allowed is None:
        raise PreparationError(f"unsupported stage: {stage}")
    if mode not in MODES:
        raise PreparationError(f"unsupported mode: {mode}")
    if mode not in allowed:
        expected = ", ".join(sorted(allowed))
        raise PreparationError(
            f"mode {mode} is not permitted for stage {stage}; expected: {expected}"
        )
    return stage, mode


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def exclusive_copy(source: Path, destination: Path) -> None:
    payload = source.read_bytes()
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(destination, flags, 0o600)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        destination.unlink(missing_ok=True)
        raise


def exclusive_json(destination: Path, payload: dict[str, object]) -> None:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(destination, flags, 0o600)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        destination.unlink(missing_ok=True)
        raise


def _manifest_run_id(candidate: Path) -> str | None:
    manifest = candidate / "manifest" / "run.json"
    if not manifest.is_file() or manifest.is_symlink():
        return None
    try:
        payload = json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    value = payload.get("run_id")
    return value if isinstance(value, str) else None


def find_duplicate_run(evidence_root: Path, run_id: str) -> Path | None:
    if not evidence_root.exists():
        return None
    root = evidence_root.resolve(strict=True)
    if not root.is_dir() or evidence_root.is_symlink():
        raise PreparationError(f"evidence root is not a real directory: {evidence_root}")
    for candidate in sorted(root.iterdir()):
        if candidate.is_symlink() or not candidate.is_dir():
            continue
        if candidate.name.endswith(f"-{run_id}") or _manifest_run_id(candidate) == run_id:
            return candidate
    return None


def validate_collector_run(run_dir: Path, evidence_root: Path, run_id: str) -> dict[str, Any]:
    root = evidence_root.resolve(strict=True)
    resolved_run = run_dir.resolve(strict=True)
    if resolved_run.parent != root or not resolved_run.is_dir() or run_dir.is_symlink():
        raise PreparationError(
            f"collector returned a run outside the requested evidence root: {resolved_run}"
        )
    manifest_path = resolved_run / "manifest" / "run.json"
    if not manifest_path.is_file() or manifest_path.is_symlink():
        raise PreparationError("collector run.json is missing or unsafe")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise PreparationError(f"collector run.json is invalid: {error}") from error
    if manifest.get("run_id") != run_id:
        raise PreparationError("collector run.json run_id does not match the requested run_id")
    manifest_run_directory = manifest.get("run_directory")
    if not isinstance(manifest_run_directory, str):
        raise PreparationError("collector run.json has no valid run_directory")
    try:
        recorded_run = Path(manifest_run_directory).resolve(strict=True)
    except OSError as error:
        raise PreparationError(f"collector run.json directory is invalid: {error}") from error
    if recorded_run != resolved_run:
        raise PreparationError("collector run.json directory does not match the returned directory")
    return manifest


def verify_collector_inputs(
    run_dir: Path, original_contract: Path, original_topics: Path
) -> None:
    expected = {
        "pc3_vils_contract.yaml": original_contract,
        "pc3_vils_bag_topics.txt": original_topics,
    }
    for name, source in expected.items():
        frozen = run_dir / "manifest" / name
        if not frozen.is_file() or frozen.is_symlink():
            raise PreparationError(f"collector did not preserve its original {name}")
        if sha256_file(frozen) != sha256_file(source):
            raise PreparationError(f"collector changed its original {name}")


def best_effort_mark_preparation_fail(
    run_dir: Path | None, evidence_root: Path, run_id: str, error: Exception
) -> None:
    if run_dir is None:
        return
    try:
        root = evidence_root.resolve(strict=True)
        resolved_run = run_dir.resolve(strict=True)
        if resolved_run.parent != root or not resolved_run.is_dir():
            return
        manifest_dir = resolved_run / "manifest"
        if not manifest_dir.is_dir() or manifest_dir.is_symlink():
            return
        abort_path = manifest_dir / "abort.json"
        if abort_path.exists():
            return
        # HH_260814 - Preserve a failed preparation as evidence and never repair it in place.
        exclusive_json(
            abort_path,
            {
                "schema_version": 1,
                "status": "PREPARATION_FAIL",
                "paper_sample_eligible": False,
                "recorded_utc": utc_text(),
                "run_id": run_id,
                "reason": f"{type(error).__name__}: {error}",
            },
        )
    except (OSError, ValueError):
        pass


def frozen_record(run_dir: Path, relative: str) -> dict[str, str]:
    path = run_dir / relative
    return {"path": relative, "sha256": sha256_file(path)}


def prepare(args: argparse.Namespace) -> Path:
    script_dir = Path(__file__).resolve().parent
    vils_dir = script_dir.parent
    collector = vils_dir / "collect_pc3_vils_evidence.py"
    original_contract = vils_dir / "pc3_vils_contract.yaml"
    original_topics = vils_dir / "pc3_vils_bag_topics.txt"
    dds_config = vils_dir / "config" / "cyclonedds_vehicle_domain.xml"
    contract = script_dir / "paper_dataset_contract.yaml"
    profiles = {
        "core_topics": script_dir / "pc3_paper_core_topics.txt",
        "lidar_topics": script_dir / "pc3_paper_lidar_topics.txt",
        "core_qos": script_dir / "pc3_paper_core_qos.yaml",
        "lidar_qos": script_dir / "pc3_paper_lidar_qos.yaml",
    }
    for path in (
        collector,
        original_contract,
        original_topics,
        dds_config,
        contract,
        *profiles.values(),
    ):
        if not path.is_file() or path.is_symlink():
            raise PreparationError(f"required acquisition file is missing or unsafe: {path}")

    run_id = validate_id("run_id", args.run_id)
    scenario_id = validate_id("scenario_id", args.scenario_id)
    condition_id = validate_id("condition_id", args.condition_id)
    stage, mode = validate_stage_mode(args.stage, args.mode)
    if args.replicate_id < 1:
        raise PreparationError("replicate_id must be at least 1")
    if args.planned_duration_sec < 1 or args.planned_duration_sec > 3600:
        raise PreparationError("planned_duration_sec must be from 1 through 3600")
    if args.random_seed < 0 or args.random_seed > 2**32 - 1:
        raise PreparationError("random_seed must be an unsigned 32-bit integer")

    duplicate = find_duplicate_run(args.evidence_root, run_id)
    if duplicate is not None:
        raise PreparationError(f"run_id already exists under the evidence root: {duplicate}")

    command = (
        sys.executable,
        str(collector),
        "init",
        "--run-id",
        run_id,
        "--evidence-root",
        str(args.evidence_root),
        "--map-dir",
        str(args.map_dir),
        "--autoware-root",
        str(args.autoware_root),
        "--ros2-root",
        str(args.ros2_root),
        "--contract",
        str(original_contract),
        "--topics",
        str(original_topics),
    )
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    if completed.returncode != 0:
        raise PreparationError(completed.stderr.strip() or "evidence collector init failed")
    output_lines = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    run_dir: Path | None = None
    try:
        if not output_lines:
            raise PreparationError("evidence collector did not return a run directory")
        returned_run = Path(output_lines[-1])
        run_dir = returned_run.resolve(strict=True)
        validate_collector_run(returned_run, args.evidence_root, run_id)
        verify_collector_inputs(run_dir, original_contract, original_topics)
        manifest_dir = run_dir / "manifest"

        # HH_260814 - Freeze paper inputs separately so the original evidence contract remains intact.
        exclusive_copy(profiles["core_topics"], manifest_dir / "pc3_paper_core_topics.txt")
        exclusive_copy(profiles["lidar_topics"], manifest_dir / "pc3_paper_lidar_topics.txt")
        exclusive_copy(profiles["core_qos"], manifest_dir / "pc3_paper_core_qos.yaml")
        exclusive_copy(profiles["lidar_qos"], manifest_dir / "pc3_paper_lidar_qos.yaml")
        exclusive_copy(contract, manifest_dir / "paper_dataset_contract.yaml")
        exclusive_copy(dds_config, manifest_dir / "cyclonedds_vehicle_domain.xml")

        boot_id_path = Path("/proc/sys/kernel/random/boot_id")
        trial = {
            "schema_version": 1,
            "status": "PREPARED_NOT_STARTED",
            "paper_sample_eligible": "pending",
            "created_utc": utc_text(),
            "run_id": run_id,
            "scenario_id": scenario_id,
            "condition_id": condition_id,
            "replicate_id": args.replicate_id,
            "stage": stage,
            "mode": mode,
            "random_seed": args.random_seed,
            "planned_duration_sec": args.planned_duration_sec,
            "host": socket.gethostname(),
            "host_role": "PC3_MAP_LOCALIZATION_PHYSICAL_LIDAR",
            "boot_id": boot_id_path.read_text(encoding="utf-8").strip()
            if boot_id_path.is_file()
            else None,
            "expected_host_manifests": ["PC1", "PC2", "PC3", "PC4"],
            "expected_recorder_profiles": ["core"],
            "recorder_profiles": {
                "core": "manifest/pc3_paper_core_topics.txt",
                "lidar": "manifest/pc3_paper_lidar_topics.txt",
            },
            "profile_hashes": {
                "paper_contract": frozen_record(
                    run_dir, "manifest/paper_dataset_contract.yaml"
                ),
                "dds": frozen_record(run_dir, "manifest/cyclonedds_vehicle_domain.xml"),
                "core": {
                    "topics": frozen_record(
                        run_dir, "manifest/pc3_paper_core_topics.txt"
                    ),
                    "qos": frozen_record(run_dir, "manifest/pc3_paper_core_qos.yaml"),
                },
                "lidar": {
                    "topics": frozen_record(
                        run_dir, "manifest/pc3_paper_lidar_topics.txt"
                    ),
                    "qos": frozen_record(run_dir, "manifest/pc3_paper_lidar_qos.yaml"),
                },
            },
            "ground_truth_limit": (
                "PC3 localization is an operational reference, not independent position ground truth."
            ),
        }
        exclusive_json(manifest_dir / "paper_trial.json", trial)
        (run_dir / "events" / "recording_locks").mkdir(mode=0o700, exist_ok=False)
        return run_dir
    except Exception as error:
        if run_dir is None:
            # HH_260814 - A zero-exit collector may still have created evidence before
            # malformed output; locate only the matching in-root run for fail marking.
            try:
                run_dir = find_duplicate_run(args.evidence_root, run_id)
            except (OSError, PreparationError):
                run_dir = None
        best_effort_mark_preparation_fail(run_dir, args.evidence_root, run_id, error)
        if isinstance(error, PreparationError):
            raise
        raise PreparationError(f"post-init preparation failed: {error}") from error


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--scenario-id", required=True)
    parser.add_argument("--condition-id", required=True)
    parser.add_argument("--replicate-id", required=True, type=int)
    parser.add_argument("--stage", required=True, choices=STAGES)
    parser.add_argument("--mode", required=True, choices=MODES)
    parser.add_argument("--random-seed", required=True, type=int)
    parser.add_argument("--planned-duration-sec", required=True, type=int)
    parser.add_argument("--evidence-root", type=Path, default=Path("/home/a/pc3_vils_runs"))
    parser.add_argument("--map-dir", type=Path, default=Path("/home/a/Autoware_Map/C_track"))
    parser.add_argument("--autoware-root", type=Path, default=Path("/home/a/autoware"))
    parser.add_argument("--ros2-root", type=Path, default=Path("/home/a/ros2_ws"))
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        run_dir = prepare(build_parser().parse_args(argv))
    except (PreparationError, OSError, subprocess.SubprocessError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    print(run_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
