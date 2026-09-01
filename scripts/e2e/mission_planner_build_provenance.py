#!/usr/bin/env python3
"""Capture and verify the exact Town05 finite-route runtime build."""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "patches/autoware_mission_planner_lane_only_no_area.manifest.json"
PROVENANCE = (
    ROOT
    / "build/autoware_mission_planner_universe/"
    "e2e_lane_only_build_provenance.json"
)
WORKSPACE_LOCK = ROOT / "data/locks/autoware_e2e_runtime.lock"
EXPECTED_REPOSITORIES = (
    "autoware_lanelet2_extension",
    "autoware_core",
    "autoware_universe",
)
RUNTIME_MARKER_TEMPLATE = (
    "Mission planner lane-only fallback active (no routable areas; "
    "source-contract={contract})."
)
RUNTIME_LIBRARIES = {
    "autoware_lanelet2_extension": {
        "package": "autoware_lanelet2_extension",
        "build": ROOT
        / "build/autoware_lanelet2_extension/"
        "libautoware_lanelet2_extension_lib.so",
        "install": ROOT
        / "install/autoware_lanelet2_extension/lib/"
        "libautoware_lanelet2_extension_lib.so",
    },
    "autoware_route_handler": {
        "package": "autoware_route_handler",
        "build": ROOT / "build/autoware_route_handler/libautoware_route_handler.so",
        "install": ROOT
        / "install/autoware_route_handler/lib/libautoware_route_handler.so",
    },
    "autoware_mission_planner_universe": {
        "package": "autoware_mission_planner_universe",
        "build": ROOT
        / "build/autoware_mission_planner_universe/"
        "libautoware_mission_planner_universe_lanelet2_plugins.so",
        "install": ROOT
        / "install/autoware_mission_planner_universe/lib/"
        "libautoware_mission_planner_universe_lanelet2_plugins.so",
    },
    "autoware_map_loader": {
        "package": "autoware_map_loader",
        "build": ROOT / "build/autoware_map_loader/liblanelet2_map_loader_node.so",
        "install": ROOT
        / "install/autoware_map_loader/lib/liblanelet2_map_loader_node.so",
    },
}


class ProvenanceError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ProvenanceError(f"cannot read {label}: {path}: {error}") from error
    if not isinstance(value, dict):
        raise ProvenanceError(f"{label} must be a JSON object: {path}")
    return value


def require_regular_file(path: Path, label: str) -> None:
    if path.is_symlink() or not path.is_file():
        raise ProvenanceError(f"{label} is missing or a symlink: {path}")


def require_install_link(path: Path, target: Path, label: str) -> None:
    if not path.is_symlink() or path.resolve() != target.resolve():
        actual = path.resolve() if path.exists() else "<missing>"
        raise ProvenanceError(
            f"{label} does not resolve to the expected workspace build: "
            f"{path} -> {actual}"
        )


def acquire_workspace_lock() -> Any:
    inherited = os.environ.get("AUTOWARE_E2E_WORKSPACE_RUNTIME_LOCK_FD", "")
    if inherited.isdigit():
        descriptor_path = Path(f"/proc/self/fd/{inherited}")
        if (
            descriptor_path.exists()
            and descriptor_path.resolve() == WORKSPACE_LOCK.resolve()
        ):
            return None
        raise ProvenanceError("invalid inherited Autoware E2E workspace lock descriptor")
    WORKSPACE_LOCK.parent.mkdir(parents=True, exist_ok=True)
    stream = WORKSPACE_LOCK.open("a+b")
    try:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        stream.close()
        raise ProvenanceError(
            "another build or matrix owns shared Autoware E2E state"
        ) from error
    return stream


def git_head(repository: Path) -> str:
    return subprocess.run(
        ["git", "-C", str(repository), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def resolve_dependency(library: Path, soname: str, target: Path) -> str:
    output = subprocess.run(
        ["ldd", str(library)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    resolved: Path | None = None
    for line in output.splitlines():
        match = re.match(rf"\s*{re.escape(soname)}\s+=>\s+(\S+)", line)
        if match:
            resolved = Path(match.group(1))
            break
    if resolved is None or resolved.resolve() != target.resolve():
        raise ProvenanceError(
            f"{library.name} ldd does not resolve workspace {soname}: "
            f"actual={resolved} expected={target}"
        )
    return str(resolved.relative_to(ROOT))


def package_prefix(package: str) -> str:
    prefix = subprocess.run(
        ["ros2", "pkg", "prefix", package],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    expected = ROOT / "install" / package
    if Path(prefix).resolve() != expected.resolve():
        raise ProvenanceError(
            f"ROS package prefix does not select workspace {package}: "
            f"expected={expected} actual={prefix}"
        )
    return prefix


def current_contract() -> dict[str, Any]:
    require_regular_file(MANIFEST, "Town05 finite-route patch manifest")
    manifest = read_object(MANIFEST, "Town05 finite-route patch manifest")
    if manifest.get("schema_version") != 2:
        raise ProvenanceError(
            "Town05 finite-route patch manifest schema_version must be 2"
        )
    repositories = manifest.get("repositories")
    names = [
        item.get("name") if isinstance(item, dict) else None
        for item in repositories or []
    ]
    if names != list(EXPECTED_REPOSITORIES):
        raise ProvenanceError(
            f"Town05 finite-route repositories must be {EXPECTED_REPOSITORIES}"
        )
    town05_contract = manifest.get("town05_regression_contract")
    if (
        not isinstance(town05_contract, dict)
        or town05_contract.get("map") != "Town05_Opt"
        or town05_contract.get("lanelet_path") != [22775, 28467, 1850]
        or re.fullmatch(
            r"[0-9a-f]{64}", str(town05_contract.get("lanelet2_map_sha256", ""))
        )
        is None
    ):
        raise ProvenanceError("Town05 regression contract is invalid")

    admitted_repositories: list[dict[str, Any]] = []
    source_paths: dict[str, list[Path]] = {}
    repository_roots: dict[str, Path] = {}
    for index, repository_contract in enumerate(repositories):
        name = repository_contract["name"]
        workspace_relative = repository_contract.get("workspace_path")
        expected_commit = repository_contract.get("git_commit")
        patch_relative = repository_contract.get("patch")
        if (
            not isinstance(workspace_relative, str)
            or not isinstance(expected_commit, str)
            or re.fullmatch(r"[0-9a-f]{40}", expected_commit) is None
            or not isinstance(patch_relative, str)
        ):
            raise ProvenanceError(f"repositories[{index}] contract is invalid")
        repository = (ROOT / workspace_relative).resolve()
        patch = (ROOT / patch_relative).resolve()
        for path, label in (
            (repository, f"{name} repository"),
            (patch, f"{name} patch"),
        ):
            try:
                path.relative_to(ROOT.resolve())
            except ValueError as error:
                raise ProvenanceError(f"{label} escapes workspace: {path}") from error
        require_regular_file(patch, f"{name} patch")
        actual_commit = git_head(repository)
        if actual_commit != expected_commit:
            raise ProvenanceError(
                f"{name} revision differs from pinned build: "
                f"expected={expected_commit} actual={actual_commit}"
            )
        files = repository_contract.get("files")
        if not isinstance(files, list) or not files:
            raise ProvenanceError(f"{name} files must be a non-empty list")
        sources: list[dict[str, Any]] = []
        resolved_sources: list[Path] = []
        for file_index, item in enumerate(files):
            relative = item.get("path") if isinstance(item, dict) else None
            expected_sha = item.get("patched_sha256") if isinstance(item, dict) else None
            if (
                not isinstance(relative, str)
                or not isinstance(expected_sha, str)
                or re.fullmatch(r"[0-9a-f]{64}", expected_sha) is None
            ):
                raise ProvenanceError(
                    f"{name} files[{file_index}] patched contract is invalid"
                )
            source = (repository / relative).resolve()
            try:
                source.relative_to(repository)
            except ValueError as error:
                raise ProvenanceError(
                    f"{name} source path escapes repository: {relative}"
                ) from error
            require_regular_file(source, f"{name} patched source")
            actual_sha = sha256_file(source)
            if actual_sha != expected_sha:
                raise ProvenanceError(
                    f"{name} patched source SHA256 differs: "
                    f"{relative}: {actual_sha}"
                )
            resolved_sources.append(source)
            sources.append(
                {
                    "path": str(source.relative_to(ROOT)),
                    "sha256": actual_sha,
                    "mtime_ns": source.stat().st_mtime_ns,
                }
            )
        source_paths[name] = resolved_sources
        repository_roots[name] = repository
        admitted_repositories.append(
            {
                "name": name,
                "git_commit": actual_commit,
                "patch": {
                    "path": str(patch.relative_to(ROOT)),
                    "sha256": sha256_file(patch),
                },
                "sources": sources,
            }
        )

    runtime_contract = manifest.get("runtime_source_contract")
    if not isinstance(runtime_contract, dict):
        raise ProvenanceError("runtime_source_contract is missing")
    runtime_repository_name = runtime_contract.get("repository")
    runtime_relative = runtime_contract.get("path")
    marker_prefix = runtime_contract.get("marker_prefix")
    normalized_sha = runtime_contract.get("normalized_sha256")
    if (
        runtime_repository_name != "autoware_universe"
        or not isinstance(runtime_relative, str)
        or marker_prefix != "source-contract="
        or not isinstance(normalized_sha, str)
        or re.fullmatch(r"[0-9a-f]{64}", normalized_sha) is None
    ):
        raise ProvenanceError("runtime_source_contract is invalid")
    runtime_source = (
        repository_roots[runtime_repository_name] / runtime_relative
    ).resolve()
    require_regular_file(runtime_source, "mission planner runtime source")
    runtime_bytes = runtime_source.read_bytes()
    matches = re.findall(rb"source-contract=([0-9a-f]{64})", runtime_bytes)
    if len(matches) != 1:
        raise ProvenanceError("mission planner runtime source marker is not unique")
    embedded_contract = matches[0].decode("ascii")
    computed_normalized_sha = hashlib.sha256(
        runtime_bytes.replace(matches[0], b"0" * 64)
    ).hexdigest()
    if embedded_contract != normalized_sha or computed_normalized_sha != normalized_sha:
        raise ProvenanceError(
            "mission planner runtime source-contract marker is stale: "
            f"embedded={embedded_contract} computed={computed_normalized_sha} "
            f"expected={normalized_sha}"
        )
    runtime_marker = RUNTIME_MARKER_TEMPLATE.format(contract=normalized_sha).encode(
        "ascii"
    )

    libraries: dict[str, dict[str, Any]] = {}
    for name, library_contract in RUNTIME_LIBRARIES.items():
        build = library_contract["build"]
        install = library_contract["install"]
        require_regular_file(build, f"{name} runtime library")
        require_install_link(install, build, f"installed {name} runtime library")
        libraries[name] = {
            "path": str(build.relative_to(ROOT)),
            "sha256": sha256_file(build),
            "mtime_ns": build.stat().st_mtime_ns,
            "installed_path": str(install.relative_to(ROOT)),
            "ros_package_prefix": package_prefix(str(library_contract["package"])),
        }

    mission_library = RUNTIME_LIBRARIES["autoware_mission_planner_universe"]["build"]
    route_handler_library = RUNTIME_LIBRARIES["autoware_route_handler"]["build"]
    extension_library = RUNTIME_LIBRARIES["autoware_lanelet2_extension"]["build"]
    map_loader_library = RUNTIME_LIBRARIES["autoware_map_loader"]["build"]
    if runtime_marker not in mission_library.read_bytes():
        raise ProvenanceError(
            "mission planner runtime library lacks the current source-contract marker"
        )
    freshness_contract = {
        "autoware_lanelet2_extension": source_paths["autoware_lanelet2_extension"],
        "autoware_route_handler": source_paths["autoware_core"],
        "autoware_mission_planner_universe": source_paths["autoware_universe"],
    }
    for library_name, sources in freshness_contract.items():
        library_mtime = RUNTIME_LIBRARIES[library_name]["build"].stat().st_mtime_ns
        newest_source = max(source.stat().st_mtime_ns for source in sources)
        if library_mtime < newest_source:
            raise ProvenanceError(
                f"{library_name} runtime library predates its patched sources"
            )

    resolution = {
        "mission_planner_to_route_handler": resolve_dependency(
            mission_library,
            "libautoware_route_handler.so",
            route_handler_library,
        ),
        "route_handler_to_lanelet2_extension": resolve_dependency(
            route_handler_library,
            "libautoware_lanelet2_extension_lib.so",
            extension_library,
        ),
        "map_loader_to_lanelet2_extension": resolve_dependency(
            map_loader_library,
            "libautoware_lanelet2_extension_lib.so",
            extension_library,
        ),
    }
    return {
        "schema_version": 2,
        "status": "INTEGRITY_PASS",
        "validation_scope": "Town05 finite-centerline and fail-closed route runtime",
        "manifest": {
            "path": str(MANIFEST.relative_to(ROOT)),
            "sha256": sha256_file(MANIFEST),
        },
        "town05_regression_contract": town05_contract,
        "repositories": admitted_repositories,
        "runtime_source_contract": {
            "path": str(runtime_source.relative_to(ROOT)),
            "normalized_sha256": normalized_sha,
        },
        "runtime_libraries": libraries,
        "runtime_resolution": resolution,
    }


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    staged = path.with_name(f".{path.name}.staged.{os.getpid()}")
    staged.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
        encoding="utf-8",
    )
    os.replace(staged, path)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Bind all patched Town05 route sources to workspace libraries."
    )
    parser.add_argument("command", choices=("capture", "verify"))
    args = parser.parse_args()
    workspace_lock = None
    try:
        workspace_lock = acquire_workspace_lock()
        current = current_contract()
        if args.command == "capture":
            atomic_json(PROVENANCE, current)
            print(f"MISSION_PLANNER_BUILD_CAPTURED path={PROVENANCE}")
        else:
            admitted = read_object(PROVENANCE, "mission planner build provenance")
            if admitted != current:
                raise ProvenanceError(
                    "mission planner runtime/source provenance differs from captured build"
                )
            print(f"MISSION_PLANNER_BUILD_VERIFIED path={PROVENANCE}")
    except (OSError, ProvenanceError, subprocess.CalledProcessError) as error:
        print(f"MISSION_PLANNER_BUILD_FAILED: {error}", file=sys.stderr)
        return 1
    finally:
        if workspace_lock is not None:
            workspace_lock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
