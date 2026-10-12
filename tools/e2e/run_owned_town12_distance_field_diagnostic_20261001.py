#!/usr/bin/env python3
"""HH_261001 - Own a single-factor Town12 distance-field startup diagnostic."""

from __future__ import annotations

from contextlib import contextmanager
import hashlib
from pathlib import Path
import re
from typing import Any, Iterator

try:
    from . import run_owned_town12_source_editor_probe as owner
except ImportError:
    from scripts.e2e import run_owned_town12_source_editor_probe as owner


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/town12_generate_distance_fields_zero_v1"
CVAR = "r.GenerateMeshDistanceFields"
OVERRIDE = f"-ini:Engine:[/Script/Engine.RendererSettings]:{CVAR}=0"
QUERY = f"-ExecCmds={CVAR}"
SOURCE_FILES = tuple(dict.fromkeys((
    *owner.SOURCE_FILES,
    *owner.shared_owner.SOURCE_FILES,
    "tools/e2e/run_owned_town12_distance_field_diagnostic_20261001.py",
    "learning/tests/test_run_owned_town12_distance_field_diagnostic_20261001.py",
)))
ENGINE_ROOT = Path("/home/a/UnrealEngine_4.26")
ENGINE_EVIDENCE = (
    "Engine/Source/Runtime/Core/Private/Misc/ConfigCacheIni.cpp",
    "Engine/Source/Runtime/Core/Private/HAL/ConsoleManager.cpp",
    "Engine/Source/Runtime/Launch/Private/LaunchEngineLoop.cpp",
    "Engine/Source/Runtime/Engine/Private/UnrealEngine.cpp",
    "Engine/Source/Runtime/Engine/Private/DistanceFieldAtlas.cpp",
    "Engine/Source/Runtime/Engine/Private/StaticMesh.cpp",
    "Engine/Source/Runtime/Engine/Private/StaticMeshRender.cpp",
    "Engine/Source/Runtime/Renderer/Private/GPUScene.cpp",
)
PROJECT_CONFIG = Path("/home/a/carla/Unreal/CarlaUE4/Config/DefaultEngine.ini")


def diagnostic_contract() -> dict[str, Any]:
    return {
        "schema": "portable_e2e.town12_distance_field_startup_diagnostic.v1",
        "changed_renderer_cvar": CVAR,
        "expected_effective_integer": 0,
        "startup_override": OVERRIDE,
        "query_only_exec_command": QUERY,
        "shared_configuration_edit_requested": False,
        "hypothesis": "Explicit numeric zero tests the previous inline-comment setting.",
        "limitation": (
            "Mesh-level bGenerateMeshDistanceField can still build distance fields. "
            "This is not a proven crash fix or a capture-quality validation."
        ),
        "capture_authorized": False,
        "training_data_admitted": False,
    }


def source_paths() -> dict[str, Path]:
    """HH_261001 - Freeze inherited transitive sources and local engine evidence."""
    paths = {name: ROOT / name for name in SOURCE_FILES}
    paths.update({f"external/UnrealEngine_4.26/{name}": ENGINE_ROOT / name
                  for name in ENGINE_EVIDENCE})
    paths["external/carla/Config/DefaultEngine.ini"] = PROJECT_CONFIG
    return paths


def archive_sources() -> dict[str, str]:
    hashes = {}
    for name, source in source_paths().items():
        if source.is_symlink() or not source.is_file():
            raise owner.Town12OwnerError(f"diagnostic input missing or symlinked: {source}")
        payload = source.read_bytes()
        archived = OUTPUT / "provenance" / name
        archived.parent.mkdir(parents=True, exist_ok=True)
        with archived.open("xb") as stream:
            stream.write(payload)
        hashes[name] = hashlib.sha256(payload).hexdigest()
    return hashes


def verify_sources(hashes: dict[str, str]) -> bool:
    try:
        paths = source_paths()
        return set(hashes) == set(paths) and bool(hashes) and all(
            not paths[name].is_symlink()
            and not (OUTPUT / "provenance" / name).is_symlink()
            and owner._sha256(paths[name]) == expected
            and owner._sha256(OUTPUT / "provenance" / name) == expected
            for name, expected in hashes.items()
        )
    except (OSError, KeyError):
        return False


def renderer_receipt(log: Path) -> dict[str, Any]:
    """HH_261001 - Require queried integer state; an echoed ini string is insufficient."""
    receipt: dict[str, Any] = {
        **diagnostic_contract(), "status": "INCONCLUSIVE", "observations": [],
        "numeric_override_logged": False, "server_log_sha256": None,
    }
    if not log.is_file() or log.is_symlink():
        receipt["error"] = "owned server log is absent or symlinked"
        return receipt
    payload = log.read_bytes()
    receipt["server_log_sha256"] = hashlib.sha256(payload).hexdigest()
    content = payload.decode("utf-8", errors="replace")
    receipt["numeric_override_logged"] = (
        f"Setting CVar [[{CVAR}:0]]" in content
    )
    pattern = re.compile(
        rf'(?<![\w.]){re.escape(CVAR)}\s*=\s*"([^"]*)"\s+LastSetBy:\s*(\w+)'
    )
    for number, line in enumerate(content.splitlines(), start=1):
        match = pattern.search(line)
        if match:
            value, source = match.groups()
            receipt["observations"].append({
                "line": number, "value": value, "last_set_by": source,
                "effective_integer": int(value) if re.fullmatch(r"-?\d+", value) else None,
            })
    observations = receipt["observations"]
    if not observations:
        receipt["error"] = "effective CVar query was not observed before startup ended"
    elif any(item["effective_integer"] != 0 or item["last_set_by"] != "ProjectSetting"
             for item in observations):
        receipt.update(status="FAIL", error="effective CVar differs from the startup override")
    elif not receipt["numeric_override_logged"]:
        receipt.update(status="FAIL", error="numeric startup override application was not logged")
    else:
        receipt.update(status="PASS", error=None)
    return receipt


@contextmanager
def configured_owner() -> Iterator[None]:
    """HH_261001 - Reuse every ownership gate and restore the original module on exit."""
    names = ("OUTPUT", "SOURCE_FILES", "editor_command", "preflight",
             "_archive_sources", "_verify_sources", "_wait_ready", "_write_new_json")
    previous = {name: getattr(owner, name) for name in names}

    def editor_command(profile: dict[str, Any], entry: dict[str, Any]) -> list[str]:
        return [*previous["editor_command"](profile, entry), OVERRIDE, QUERY]

    def preflight():
        binding = previous["preflight"]()
        for source in source_paths().values():
            if source.is_symlink() or not source.is_file():
                raise owner.Town12OwnerError(f"diagnostic input missing or symlinked: {source}")
        return binding

    def wait_ready(editor, environment):
        previous["_wait_ready"](editor, environment)
        receipt = renderer_receipt(OUTPUT / "server.log")
        previous["_write_new_json"](OUTPUT / "effective_renderer.json", receipt)
        if receipt["status"] != "PASS":
            raise owner.Town12OwnerError(str(receipt["error"]))

    def write_new_json(path: Path, value: dict[str, Any]) -> None:
        if path.name == "owner_plan.json":
            value = {**value, "renderer_diagnostic": diagnostic_contract()}
        elif path.name == "owner_result.json":
            receipt = renderer_receipt(OUTPUT / "server.log")
            previous["_write_new_json"](OUTPUT / "renderer_diagnostic.json", receipt)
            value = {**value, "renderer_diagnostic": receipt}
        previous["_write_new_json"](path, value)

    try:
        owner.OUTPUT = OUTPUT
        owner.SOURCE_FILES = SOURCE_FILES
        owner.editor_command = editor_command
        owner.preflight = preflight
        owner._archive_sources = archive_sources
        owner._verify_sources = verify_sources
        owner._wait_ready = wait_ready
        owner._write_new_json = write_new_json
        yield
    finally:
        for name, value in previous.items():
            setattr(owner, name, value)


def main(argv: list[str] | None = None) -> int:
    with configured_owner():
        return owner.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
