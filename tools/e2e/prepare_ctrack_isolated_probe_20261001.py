#!/usr/bin/env python3
"""HH_261001 - Prepare and verify a private CTrack source-editor sandbox.

This tool never starts Unreal or CARLA. The emitted launch argv is for a runtime
owner that acquires the workspace lease, checks ports, bounds runtime/disk use,
and owns the bubblewrap process group. Content and binaries stay kernel-readonly.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
PROJECT = Path("/home/a/carla/Unreal/CarlaUE4")
ENGINE = Path("/home/a/UnrealEngine_4.26/Engine")
EDITOR = ENGINE / "Binaries/Linux/UE4Editor"
MAP = "/Game/map_package/Maps/C_track_1_0_7/C_track_1_0_7"
LEVEL = PROJECT / "Content" / (MAP.removeprefix("/Game/") + ".umap")
XODR = LEVEL.parent / "OpenDrive/C_track_1_0_7.xodr"
GAME_MODE = "/Game/Carla/Blueprints/Game/CarlaGameMode.CarlaGameMode_C"
DDC_GRAPH = "CTrackReadOnlyShared20261001"
PINNED = {
    LEVEL: "775739b54fe5b6eecf2b2cae6f463d3c9583336e89911b882b9a9281d27407fe",
    XODR: "3f760b8ec8cb117a975c9631a74948b620c991aeaef5c1b07497ebb37ec7be27",
    EDITOR: "331f8ece41d8e09c85e927046a9afa522de2dc0a56affa560849d677b9ccb9e9",
    PROJECT / "CarlaUE4.uproject": "11b5ca8a37d550f45fcf5b93d15e450b7df944746ea599518cca02d6e8eddde2",
    PROJECT / "Plugins/Carla/Source/Carla/Game/Tagger.h": "ba5dd4bc118e06a4e595c4295d9ca4cb53c984ff97da7e5de342e53d1d58cd17",
    PROJECT / "Plugins/Carla/Source/Carla/Game/Tagger.cpp": "fb71f676e987506e41edcbc6309323d1e2dbb5408fb8ee20addeb0b5708b372b",
}
DEFAULT_OUTPUT = ROOT / "storage/experiments/validation/2026-10-01/ctrack_isolated_source_20261001"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def fingerprint(path: Path) -> dict:
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"regular source required: {path}")
    st = path.stat()
    return {"sha256": sha256(path), "size_bytes": st.st_size,
            "device": st.st_dev, "inode": st.st_ino, "mtime_ns": st.st_mtime_ns}


def package_candidates(data: bytes) -> list[str]:
    # HH_261001 - These are printable package candidates, not an import-table parser.
    return sorted({token.decode("ascii") for token in re.findall(rb"[ -~]{4,}", data)
                   if re.fullmatch(rb"/(Game|Engine)/[A-Za-z0-9_/]+", token)})


def resolve_package(package: str) -> Path | None:
    prefix, base = (("/Game/", PROJECT / "Content") if package.startswith("/Game/")
                    else ("/Engine/", ENGINE / "Content"))
    for suffix in (".uasset", ".umap"):
        path = base / (package.removeprefix(prefix) + suffix)
        if path.is_file():
            return path
    return None


def audit_sources() -> dict:
    """HH_261001 - Pin the map, semantic meshes, configs and loaded module build IDs."""
    fingerprints = {str(path): fingerprint(path) for path in PINNED}
    for path, expected in PINNED.items():
        if fingerprints[str(path)]["sha256"] != expected:
            raise ValueError(f"source SHA256 changed: {path}")
    references = []
    meshes = {name: [] for name in ("Road", "RoadLine", "SideWalk")}
    for package in package_candidates(LEVEL.read_bytes()):
        path = resolve_package(package)
        reference = {"package": package, "path": str(path) if path else None}
        references.append(reference)
        pieces = package.split("/")
        if package.startswith("/Game/map_package/Static/") and pieces[4] in meshes:
            if path is None or b"StaticMesh" not in path.read_bytes():
                raise ValueError(f"semantic mesh is absent or not StaticMesh: {package}")
            fingerprints[str(path)] = fingerprint(path)
            meshes[pieces[4]].append(reference)
    if {name: len(rows) for name, rows in meshes.items()} != {
            "Road": 10, "RoadLine": 10, "SideWalk": 30}:
        raise ValueError("staging semantic mesh count changed")
    for path in sorted((PROJECT / "Config").rglob("*")):
        if path.is_file():
            fingerprints[str(path)] = fingerprint(path)
    module_files = [ENGINE / "Binaries/Linux/UE4Editor.modules",
                    PROJECT / "Binaries/Linux/UE4Editor.modules"]
    module_files.extend(sorted((PROJECT / "Plugins").glob("*/Binaries/Linux/UE4Editor.modules")))
    module_records = []
    for path in module_files:
        document = json.loads(path.read_text())
        fingerprints[str(path)] = fingerprint(path)
        binaries = []
        for name, filename in document["Modules"].items():
            binary = path.parent / filename
            if not binary.is_file():
                raise ValueError(f"module binary absent: {binary}")
            # The engine has hundreds of libraries; fingerprint every project/plugin module.
            if path != module_files[0]:
                fingerprints[str(binary)] = fingerprint(binary)
            binaries.append({"name": name, "path": str(binary)})
        module_records.append({"path": str(path), "build_id": document["BuildId"],
                               "binaries": binaries})
    build_ids = {row["build_id"] for row in module_records}
    if len(build_ids) != 1:
        raise ValueError(f"engine/project/plugin module BuildIds differ: {build_ids}")
    for path in (PROJECT / "Plugins/Carla/Source/Carla/OpenDrive/OpenDrive.cpp",
                 PROJECT / "Plugins/Carla/Source/Carla/Game/CarlaGameModeBase.cpp",
                 ENGINE / "Source/Runtime/Core/Private/Misc/Paths.cpp",
                 ENGINE / "Source/Runtime/Core/Private/Unix/UnixPlatformProcess.cpp",
                 ENGINE / "Config/BaseEngine.ini"):
        fingerprints[str(path)] = fingerprint(path)
    # The full trees remain mounted, so no dependency subset is guessed or copied.
    return {"source_fingerprints": fingerprints, "module_build_id": next(iter(build_ids)),
            "modules": module_records, "map_package": MAP,
            "semantic_meshes": meshes, "printable_package_candidates": references,
            "unresolved_printable_candidates": [r["package"] for r in references if not r["path"]],
            "reference_method": "printable candidates; does not prove serialized dependency closure",
            "content_strategy": "entire existing project and engine trees mounted read-only",
            "runtime_validated": False, "dataset_admitted": False}


def validate_run_root(path: Path, *, new: bool) -> Path:
    path = path.absolute()
    if path.resolve() != path or any(part.is_symlink() for part in (path, *path.parents)):
        raise ValueError("run directory or an ancestor is symlinked")
    if path == ROOT or not path.is_relative_to(ROOT / "storage/experiments"):
        raise ValueError("run directory must be below workspace storage/experiments")
    if new and path.exists():
        raise ValueError(f"refusing to reuse an existing run: {path}")
    return path


def private_bindings(run_root: Path) -> list[tuple[Path, Path]]:
    bindings = [(run_root, run_root)]
    bindings.extend((run_root / name, PROJECT / name)
                    for name in ("Config", "Saved", "Intermediate", "DerivedDataCache"))
    bindings.extend((run_root / name, destination) for name, destination in (
        ("Tmp", Path("/tmp")),
        ("UserCache", Path("/home/a/.cache")),
        ("UserConfig", Path("/home/a/.config")),
        ("UserData", Path("/home/a/.local/share")),
        ("EngineSaved", ENGINE / "Saved"),
        ("EngineIntermediate", ENGINE / "Intermediate")))
    return bindings


def sandbox_argv(run_root: Path) -> list[str]:
    """HH_261001 - Kernel-enforce RO shared inputs, including absolute cache fallbacks."""
    argv = ["/usr/bin/bwrap", "--die-with-parent", "--unshare-user", "--unshare-pid",
            "--unshare-ipc", "--ro-bind", "/", "/", "--proc", "/proc",
            "--dev-bind", "/dev", "/dev"]
    for source, destination in private_bindings(run_root):
        argv.extend(("--bind", str(source), str(destination)))
    # Hide host shared-memory objects, keep GPU character devices from /dev available.
    argv.extend(("--tmpfs", "/dev/shm", "--chdir", str(run_root)))
    environment = {
        "XDG_CACHE_HOME": str(run_root / "UserCache"),
        "XDG_CONFIG_HOME": str(run_root / "UserConfig"),
        "XDG_DATA_HOME": str(run_root / "UserData"),
        "XDG_RUNTIME_DIR": str(run_root / "Runtime"),
        "TMPDIR": "/tmp", "UE-LocalDataCachePath": str(run_root / "DerivedDataCache"),
        "UE-SharedDataCachePath": "None", "UE-SharedDataCachePath2": "None",
        "__GL_SHADER_DISK_CACHE_PATH": str(run_root / "UserCache/NVIDIA"),
        "CUDA_CACHE_PATH": str(run_root / "UserCache/CUDA"),
        "PYTHONDONTWRITEBYTECODE": "1",
    }
    for name, value in environment.items():
        argv.extend(("--setenv", name, value))
    return argv


def editor_argv(run_root: Path, port: int) -> list[str]:
    if not 1024 <= port <= 65532:
        raise ValueError("port must leave room for three unprivileged CARLA ports")
    return [str(EDITOR), str(PROJECT / "CarlaUE4.uproject"), MAP,
            "-game", "-vulkan", "-prefernvidia", "-quality-level=Epic",
            f"-carla-port={port}", "-RenderOffScreen", "-nosound", "-unattended",
            f"-userdir={run_root}", f"-ddc={DDC_GRAPH}",
            f"-abslog={run_root / 'Saved/Logs/CarlaUE4.log'}",
            "-ini:EditorSettings:[/Script/UnrealEd.CrashReportsPrivacySettings]:bSendUnattendedBugReports=False"]


def private_engine_config(original: str, run_root: Path) -> str:
    # HH_261001 - Override all startup/transition maps in PRIVATE config, before URL startup.
    maps = "\n[/Script/EngineSettings.GameMapsSettings]\n" + "\n".join(
        f"{key}={MAP}.C_track_1_0_7" for key in
        ("EditorStartupMap", "GameDefaultMap", "ServerDefaultMap", "TransitionMap"))
    maps += (f"\nGlobalDefaultGameMode={GAME_MODE}\nGlobalDefaultServerGameMode={GAME_MODE}"
             "\nGameInstanceClass=/Script/Carla.CarlaGameInstance\n")
    ddc = f'''
[{DDC_GRAPH}]
Root=(Type=KeyLength, Length=120, Inner=AsyncPut)
AsyncPut=(Type=AsyncPut, Inner=Hierarchy)
Hierarchy=(Type=Hierarchical, Inner=Local, Inner=Shared)
Local=(Type=FileSystem, ReadOnly=false, Clean=false, Flush=false, PurgeTransient=false, DeleteUnused=false, Path="{run_root / 'DerivedDataCache'}")
Shared=(Type=FileSystem, ReadOnly=true, Clean=false, Flush=false, PurgeTransient=false, DeleteUnused=false, Path="{ENGINE / 'DerivedDataCache'}")
'''
    return original + "\n; HH_261001 - Private sandbox startup and read-only shared DDC.\n" + maps + ddc


def write_json_new(path: Path, document: dict) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, sort_keys=True)
        stream.write("\n")


def prepare(run_root: Path, port: int) -> dict:
    run_root = validate_run_root(run_root, new=True)
    audit = audit_sources()
    launch = sandbox_argv(run_root) + ["--"] + editor_argv(run_root, port)
    if shutil.disk_usage(run_root.parent if run_root.parent.exists() else ROOT).free < 5 * 1024**3:
        raise ValueError("at least 5 GiB free disk is required before staging")
    run_root.mkdir(parents=True)
    for source, _ in private_bindings(run_root):
        source.mkdir(exist_ok=True)
    (run_root / "Runtime").mkdir(mode=0o700)
    (run_root / "Saved/Logs").mkdir(parents=True)
    for source in sorted((PROJECT / "Config").rglob("*")):
        target = run_root / "Config" / source.relative_to(PROJECT / "Config")
        if source.is_symlink():
            raise ValueError(f"refuse symlinked source config: {source}")
        if source.is_dir():
            target.mkdir(exist_ok=True)
        else:
            raw = source.read_bytes()
            if source.name == "DefaultEngine.ini":
                raw = private_engine_config(raw.decode("utf-8"), run_root).encode("utf-8")
            with target.open("xb") as stream:
                stream.write(raw)
            if source.stat().st_ino == target.stat().st_ino:
                raise ValueError("private config unexpectedly shares its source inode")
    manifest = {"schema": "portable_e2e.ctrack_isolated_source_plan.v1", **audit,
                "run_root": str(run_root), "port_triplet": [port, port + 1, port + 2],
                "editor_argv": editor_argv(run_root, port), "launch_argv": launch,
                "prelaunch_requirements": ["workspace runtime lease", "ports free", "GPU/runtime budget",
                                           "verify source fingerprints and sandbox smoke"],
                "ownership_note": "Popen PID is bubblewrap; resolve UE4 descendant, own whole PGID",
                "simulator_launched": False,
                "private_config_sha256": sha256(run_root / "Config/DefaultEngine.ini")}
    write_json_new(run_root / "plan.json", manifest)
    return manifest


# HH_261001 - Opening protected sources O_WRONLY without O_TRUNC never changes their bytes.
# Reject an unexpectedly writable source even when Unix mode bits would permit writing.
SMOKE_CODE = r'''
import errno, json, os, pathlib, sys
plan = json.loads(pathlib.Path(sys.argv[1]).read_text())
root = pathlib.Path(plan["run_root"])
protected = [p for p in plan["source_fingerprints"] if "/Config/" not in p]
denied = []
for path in protected:
    try:
        fd = os.open(path, os.O_WRONLY)
    except OSError as exc:
        if exc.errno != errno.EROFS:
            raise
        denied.append(path)
    else:
        os.close(fd)
        raise RuntimeError("source writable inside sandbox: " + path)
source_config = pathlib.Path("/home/a/carla/Unreal/CarlaUE4/Config/DefaultEngine.ini")
private_config = root / "Config/DefaultEngine.ini"
assert source_config.stat().st_ino == private_config.stat().st_ino
assert source_config.stat().st_ino != plan["source_fingerprints"][str(source_config)]["inode"]
assert "CTrackReadOnlyShared20261001" in source_config.read_text()
readonly_cache = pathlib.Path("/home/a/UnrealEngine_4.26/Engine/DerivedDataCache")
assert os.statvfs(readonly_cache).f_flag & os.ST_RDONLY
paths = [root / name for name in ("Config", "Saved", "Intermediate", "DerivedDataCache",
                                 "Tmp", "UserCache", "UserConfig", "UserData", "EngineSaved",
                                 "EngineIntermediate", "Runtime")]
for directory in paths:
    with (directory / "HH_261001_sandbox_write_probe").open("x") as stream:
        stream.write("private writable mount verified\n")
print(json.dumps({"status": "PASS", "readonly_sources": len(denied),
                  "private_writable_directories": len(paths), "engine_ddc_readonly": True,
                  "private_config_inode_distinct": True, "simulator_launched": False}))
'''


def verify(run_root: Path, *, smoke: bool) -> dict:
    run_root = validate_run_root(run_root, new=False)
    plan = json.loads((run_root / "plan.json").read_text())
    if plan["run_root"] != str(run_root):
        raise ValueError("plan run root mismatch")
    before = {path: fingerprint(Path(path)) for path in plan["source_fingerprints"]}
    if before != plan["source_fingerprints"]:
        raise ValueError("source fingerprint changed since staging")
    if sha256(run_root / "Config/DefaultEngine.ini") != plan["private_config_sha256"]:
        raise ValueError("staged startup/cache config changed")
    if plan["launch_argv"] != sandbox_argv(run_root) + ["--"] + editor_argv(run_root, plan["port_triplet"][0]):
        raise ValueError("launch argv changed")
    result = {"status": "PASS", "source_fingerprints_unchanged": len(before),
              "simulator_launched": False}
    if smoke:
        completed = subprocess.run(sandbox_argv(run_root) + ["--", sys.executable, "-c", SMOKE_CODE,
                                   str(run_root / "plan.json")],
                                   text=True, capture_output=True, timeout=30)
        if completed.returncode:
            raise ValueError("sandbox smoke failed: " + completed.stderr[-2000:])
        result["sandbox_smoke"] = json.loads(completed.stdout)
        after = {path: fingerprint(Path(path)) for path in before}
        if after != before:
            raise ValueError("source fingerprint changed across smoke")
        write_json_new(run_root / "sandbox_smoke.json", result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--port", type=int, default=2400)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--prepare", action="store_true")
    action.add_argument("--verify", action="store_true")
    action.add_argument("--sandbox-smoke", action="store_true")
    args = parser.parse_args()
    try:
        if args.prepare:
            plan = prepare(args.output, args.port)
            counts = {name: len(rows) for name, rows in plan["semantic_meshes"].items()}
            result = {"status": "PREPARED", "plan": str(args.output / "plan.json"),
                      "semantic_mesh_counts": counts, "module_build_id": plan["module_build_id"],
                      "unresolved_printable_candidates": plan["unresolved_printable_candidates"],
                      "simulator_launched": False}
        else:
            result = verify(args.output, smoke=args.sandbox_smoke)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        print(json.dumps({"status": "FAIL", "error": str(exc)}), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
