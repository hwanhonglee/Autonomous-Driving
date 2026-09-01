#!/usr/bin/env python3
"""Capture or verify the patched VAD-object/AEB runtime build contract."""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "patches/autoware_vad_object_safety.manifest.json"
PROVENANCE = (
    ROOT
    / "build/autoware_tensorrt_vad/e2e_object_safety_build_provenance.json"
)
WORKSPACE_LOCK = ROOT / "data/locks/autoware_e2e_runtime.lock"


class ProvenanceError(RuntimeError):
    pass


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ProvenanceError(f"cannot read {label}: {path}: {error}") from error
    if not isinstance(value, dict):
        raise ProvenanceError(f"{label} must be a JSON object: {path}")
    return value


def require_file(path: Path, label: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise ProvenanceError(f"{label} is missing or a symlink: {path}")


def acquire_lock() -> Any:
    inherited = os.environ.get("AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD", "")
    if inherited.isdigit():
        descriptor = Path(f"/proc/self/fd/{inherited}")
        if descriptor.exists() and descriptor.resolve() == WORKSPACE_LOCK.resolve():
            return None
        raise ProvenanceError("invalid inherited workspace lock descriptor")
    WORKSPACE_LOCK.parent.mkdir(parents=True, exist_ok=True)
    stream = WORKSPACE_LOCK.open("a+b")
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        stream.close()
        raise ProvenanceError("another build or matrix owns shared workspace state") from error
    return stream


def git_head(repository: Path) -> str:
    return subprocess.run(
        ["git", "-C", str(repository), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def package_prefix(package: str) -> str:
    actual = Path(
        subprocess.run(
            ["ros2", "pkg", "prefix", package],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    ).resolve()
    expected = (ROOT / "install" / package).resolve()
    if actual != expected:
        raise ProvenanceError(
            f"ROS package prefix does not select workspace {package}: "
            f"expected={expected} actual={actual}"
        )
    return str(actual.relative_to(ROOT))


def current_contract() -> dict[str, Any]:
    require_file(MANIFEST, "VAD object-safety manifest")
    manifest = read_object(MANIFEST, "VAD object-safety manifest")
    if manifest.get("schema_version") != 1:
        raise ProvenanceError("VAD object-safety manifest schema_version must be 1")
    repository_contract = manifest.get("repository")
    if not isinstance(repository_contract, dict):
        raise ProvenanceError("repository contract is missing")
    repository = (ROOT / str(repository_contract.get("workspace_path", ""))).resolve()
    expected_head = repository_contract.get("git_commit")
    if git_head(repository) != expected_head:
        raise ProvenanceError("autoware_universe HEAD differs from manifest")

    patches: list[dict[str, Any]] = []
    sources: list[dict[str, Any]] = []
    source_mtimes: dict[str, int] = {}
    for patch_contract in manifest.get("patches") or []:
        if not isinstance(patch_contract, dict):
            raise ProvenanceError("patch contract is invalid")
        patch = (ROOT / str(patch_contract.get("path", ""))).resolve()
        require_file(patch, "object-safety patch")
        if sha256(patch) != patch_contract.get("sha256"):
            raise ProvenanceError(f"patch SHA256 differs: {patch}")
        patches.append(
            {
                "name": patch_contract.get("name"),
                "path": str(patch.relative_to(ROOT)),
                "sha256": sha256(patch),
            }
        )
        for item in patch_contract.get("files") or []:
            if not isinstance(item, dict):
                raise ProvenanceError("source contract is invalid")
            source = (repository / str(item.get("path", ""))).resolve()
            require_file(source, "patched source")
            digest = sha256(source)
            if digest != item.get("patched_sha256"):
                raise ProvenanceError(f"patched source SHA256 differs: {source}")
            relative = str(source.relative_to(ROOT))
            mtime_ns = source.stat().st_mtime_ns
            source_mtimes[relative] = mtime_ns
            sources.append(
                {"path": relative, "sha256": digest, "mtime_ns": mtime_ns}
            )

    runtimes: list[dict[str, Any]] = []
    for item in manifest.get("runtime_libraries") or []:
        if not isinstance(item, dict):
            raise ProvenanceError("runtime library contract is invalid")
        source_paths = item.get("source_paths")
        if (
            not isinstance(source_paths, list)
            or not source_paths
            or any(not isinstance(path, str) for path in source_paths)
            or len(source_paths) != len(set(source_paths))
        ):
            raise ProvenanceError("runtime library source_paths contract is invalid")
        missing_sources = [path for path in source_paths if path not in source_mtimes]
        if missing_sources:
            raise ProvenanceError(
                f"runtime library references unpinned sources: {missing_sources}"
            )
        build = (ROOT / str(item.get("build_path", ""))).resolve()
        install = ROOT / str(item.get("install_path", ""))
        require_file(build, "runtime build library")
        if not install.is_symlink() or install.resolve() != build:
            raise ProvenanceError(
                f"runtime install library does not resolve to workspace build: {install}"
            )
        newest_runtime_source_mtime = max(source_mtimes[path] for path in source_paths)
        if build.stat().st_mtime_ns < newest_runtime_source_mtime:
            raise ProvenanceError(
                f"runtime library is older than its patched sources: {build}"
            )
        marker = item.get("required_marker")
        if isinstance(marker, str) and marker.encode() not in build.read_bytes():
            raise ProvenanceError(f"runtime source-contract marker is absent: {build}")
        runtimes.append(
            {
                "package": item.get("package"),
                "build_path": str(build.relative_to(ROOT)),
                "install_path": str(install.relative_to(ROOT)),
                "sha256": sha256(build),
                "mtime_ns": build.stat().st_mtime_ns,
                "ros_package_prefix": package_prefix(str(item.get("package"))),
                "required_marker": marker,
                "source_paths": source_paths,
            }
        )

    deterministic = {
        "schema_version": 1,
        "contract_id": manifest.get("contract_id"),
        "manifest": {
            "path": str(MANIFEST.relative_to(ROOT)),
            "sha256": sha256(MANIFEST),
        },
        "repository": {
            "path": str(repository.relative_to(ROOT)),
            "git_commit": expected_head,
        },
        "patches": patches,
        "sources": sources,
        "runtime_libraries": runtimes,
    }
    normalized = json.dumps(deterministic, sort_keys=True, separators=(",", ":"))
    deterministic["source_runtime_contract_sha256"] = hashlib.sha256(
        normalized.encode()
    ).hexdigest()
    return deterministic


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("capture", "verify"))
    args = parser.parse_args()
    workspace_lock = acquire_lock()
    current = current_contract()
    if args.command == "capture":
        PROVENANCE.parent.mkdir(parents=True, exist_ok=True)
        PROVENANCE.write_text(
            json.dumps(current, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(f"Captured VAD object-safety build provenance: {PROVENANCE}")
        if workspace_lock is not None:
            workspace_lock.close()
        return 0
    require_file(PROVENANCE, "VAD object-safety build provenance")
    captured = read_object(PROVENANCE, "VAD object-safety build provenance")
    if captured != current:
        raise ProvenanceError("VAD object-safety build provenance differs from runtime")
    print(f"Verified VAD object-safety build provenance: {PROVENANCE}")
    if workspace_lock is not None:
        workspace_lock.close()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, subprocess.CalledProcessError, ProvenanceError) as error:
        print(f"VAD object-safety provenance error: {error}", file=sys.stderr)
        raise SystemExit(2)
