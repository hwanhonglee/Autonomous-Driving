#!/usr/bin/env python3

import argparse
import json
import os
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST = Path(__file__).with_name("carla_expert_suite.yaml")
DEFAULT_OUTPUT = ROOT / "artifacts/inventory/carla_training_maps.json"

PROFILE_CONTRACT = {
    "packaged_0915": {
        "kind": "packaged",
        "version": "0.9.15-dev-dirty",
        "executable": "/home/hong/carla-autoware-universe/CARLA_0.9.15/CarlaUE4.sh",
        "project": None,
        "python_api": "/home/hong/carla-autoware-universe/CARLA_0.9.15/"
        "PythonAPI/carla/dist/carla-0.9.15-py3.10-linux-x86_64.egg",
        "python_api_sha256": "ee7209c5570cb9201e9fbd3cbcefc5b176dd4f6b2289df52bb0bd7dc2c56800b",
    },
    "source_editor_0915_4ws": {
        "kind": "source_editor",
        "version": "0.9.15-dev-dirty",
        "executable": "/home/hong/UnrealEngine_4.26/Engine/Binaries/Linux/UE4Editor",
        "project": "/home/hong/carla/Unreal/CarlaUE4/CarlaUE4.uproject",
        "python_api": "/home/hong/carla/PythonAPI/carla/dist/"
        "carla-0.9.15-py3.10-linux-x86_64.egg",
        "python_api_sha256": "328e90242236b8f0529e6c9526694934bf20c27dc1e7348ba46eaf79061e6617",
    },
}
MAP_CONTRACT = {
    "town01": ("Town01", "/Game/Carla/Maps/Town01", "ready", "packaged_0915"),
    "town02": ("Town02", "/Game/Carla/Maps/Town02", "unavailable", None),
    "town02_opt": (
        "Town02_Opt",
        "/Game/Carla/Maps/Town02_Opt",
        "ready",
        "packaged_0915",
    ),
    "town03": (
        "Town03",
        "/Game/Carla/Maps/Town03",
        "ready",
        "packaged_0915",
    ),
    "town04": ("Town04", "/Game/Carla/Maps/Town04", "ready", "packaged_0915"),
    "town05": ("Town05", "/Game/Carla/Maps/Town05", "unavailable", None),
    "town05_opt": (
        "Town05_Opt",
        "/Game/Carla/Maps/Town05_Opt",
        "ready",
        "packaged_0915",
    ),
    "town06": ("Town06", "/Game/Carla/Maps/Town06", "ready", "packaged_0915"),
    "town07": ("Town07", "/Game/Carla/Maps/Town07", "ready", "packaged_0915"),
    "town08": ("Town08", "/Game/Carla/Maps/Town08", "unavailable", None),
    "town09": ("Town09", "/Game/Carla/Maps/Town09", "unavailable", None),
    "town10hd": (
        "Town10HD",
        "/Game/Carla/Maps/Town10HD",
        "unavailable",
        None,
    ),
    "town10hd_opt": (
        "Town10HD_Opt",
        "/Game/Carla/Maps/Town10HD_Opt",
        "ready",
        "packaged_0915",
    ),
    "town11": (
        "Town11",
        "/Game/Carla/Maps/Town11/Town11",
        "source_editor_required",
        "source_editor_0915_4ws",
    ),
    "town12": (
        "Town12",
        "/Game/Carla/Maps/Town12/Town12",
        "source_editor_required",
        "source_editor_0915_4ws",
    ),
    "town13": (
        "Town13",
        "/Game/Carla/Maps/Town13/Town13",
        "source_editor_required",
        "source_editor_0915_4ws",
    ),
    "town15": (
        "Town15",
        "/Game/Carla/Maps/Town15/Town15",
        "source_editor_required",
        "source_editor_0915_4ws",
    ),
    "c_track_1_0_7": (
        "C_track_1_0_7",
        "/Game/Carla/Maps/C_track_1_0_7",
        "ready",
        "packaged_0915",
    ),
    "woraksan_1_0_3": (
        "Woraksan_v1_0_3_parking_lot_hegiht_fit",
        "/Game/map_package/Maps/Woraksan_v1_0_3_parking_lot_hegiht_fit/"
        "Woraksan_v1_0_3_parking_lot_hegiht_fit",
        "ready",
        "packaged_0915",
    ),
}
SPLIT_CONTRACT = {
    "town01": "train",
    "town02": "excluded",
    "town02_opt": "train",
    "town03": "train",
    "town04": "train",
    "town05": "excluded",
    "town05_opt": "train",
    "town06": "train",
    "town07": "train",
    "town08": "excluded",
    "town09": "excluded",
    "town10hd": "excluded",
    "town10hd_opt": "train",
    "town11": "excluded",
    "town12": "train",
    "town13": "validation",
    "town15": "test",
    "c_track_1_0_7": "train",
    "woraksan_1_0_3": "train",
}
SPLITS = {"train", "validation", "test", "excluded"}
MAP_FIELDS = {
    "id",
    "canonical_name",
    "server_profile",
    "load_name",
    "split",
    "status",
    "reason",
    "level_path",
    "opendrive_path",
}
PROFILE_FIELDS = {
    "kind",
    "version",
    "executable",
    "project",
    "python_api",
    "python_api_sha256",
}


class ManifestError(RuntimeError):
    pass


def _require_mapping(value, label):
    if not isinstance(value, dict):
        raise ManifestError(f"{label} must be a mapping")
    return value


def _require_nonempty_string(value, label):
    if not isinstance(value, str) or not value.strip():
        raise ManifestError(f"{label} must be a non-empty string")


def load_manifest(path=DEFAULT_MANIFEST):
    path = Path(path).expanduser().resolve()
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ManifestError(f"failed to read {path}: {error}") from error
    return validate_manifest(document), path


def validate_manifest(document):
    document = _require_mapping(document, "manifest")
    required_root = {
        "schema_version",
        "suite_id",
        "description",
        "server_profiles",
        "maps",
    }
    if set(document) != required_root:
        raise ManifestError(
            "manifest root fields must be exactly: " + ", ".join(sorted(required_root))
        )
    if document["schema_version"] != 1:
        raise ManifestError("schema_version must be 1")
    _require_nonempty_string(document["suite_id"], "suite_id")
    _require_nonempty_string(document["description"], "description")

    profiles = _require_mapping(document["server_profiles"], "server_profiles")
    if set(profiles) != set(PROFILE_CONTRACT):
        raise ManifestError("server_profiles must contain the two canonical profiles")
    for profile_id, expected_profile in PROFILE_CONTRACT.items():
        profile = _require_mapping(profiles[profile_id], f"profile {profile_id}")
        if set(profile) != PROFILE_FIELDS:
            raise ManifestError(f"profile {profile_id} has an invalid field set")
        if profile != expected_profile:
            raise ManifestError(f"profile {profile_id} canonical contract mismatch")
        for field in ("version", "executable", "python_api", "python_api_sha256"):
            _require_nonempty_string(profile[field], f"profile {profile_id}.{field}")
        if not Path(profile["executable"]).is_absolute():
            raise ManifestError(f"profile {profile_id}.executable must be absolute")
        if not Path(profile["python_api"]).is_absolute():
            raise ManifestError(f"profile {profile_id}.python_api must be absolute")
        project = profile["project"]
        if profile["kind"] == "source_editor":
            _require_nonempty_string(project, f"profile {profile_id}.project")
            if not Path(project).is_absolute():
                raise ManifestError(f"profile {profile_id}.project must be absolute")
        elif project is not None:
            raise ManifestError(f"profile {profile_id}.project must be null")

    maps = document["maps"]
    if not isinstance(maps, list):
        raise ManifestError("maps must be a list")
    by_id = {}
    for index, map_entry in enumerate(maps):
        map_entry = _require_mapping(map_entry, f"maps[{index}]")
        if set(map_entry) != MAP_FIELDS:
            raise ManifestError(f"maps[{index}] has an invalid field set")
        map_id = map_entry.get("id")
        _require_nonempty_string(map_id, f"maps[{index}].id")
        if map_id in by_id:
            raise ManifestError(f"duplicate map id: {map_id}")
        by_id[map_id] = map_entry

    if set(by_id) != set(MAP_CONTRACT):
        missing = sorted(set(MAP_CONTRACT) - set(by_id))
        extra = sorted(set(by_id) - set(MAP_CONTRACT))
        raise ManifestError(f"canonical map set mismatch; missing={missing}, extra={extra}")

    for map_id, expected in MAP_CONTRACT.items():
        entry = by_id[map_id]
        canonical_name, load_name, status, profile_id = expected
        actual = (
            entry["canonical_name"],
            entry["load_name"],
            entry["status"],
            entry["server_profile"],
        )
        if actual != expected:
            raise ManifestError(f"map {map_id} canonical contract mismatch")
        if not load_name.startswith("/Game/"):
            raise ManifestError(f"map {map_id}.load_name must be a full Unreal path")
        if entry["split"] not in SPLITS:
            raise ManifestError(f"map {map_id}.split is invalid")
        if entry["split"] != SPLIT_CONTRACT[map_id]:
            raise ManifestError(f"map {map_id}.split canonical contract mismatch")
        _require_nonempty_string(entry["reason"], f"map {map_id}.reason")
        if status == "unavailable":
            if entry["split"] != "excluded":
                raise ManifestError(f"unavailable map {map_id} must be excluded")
            if entry["level_path"] is not None or entry["opendrive_path"] is not None:
                raise ManifestError(f"unavailable map {map_id} cannot bind assets")
        else:
            for field in ("level_path", "opendrive_path"):
                _require_nonempty_string(entry[field], f"map {map_id}.{field}")
                if not Path(entry[field]).is_absolute():
                    raise ManifestError(f"map {map_id}.{field} must be absolute")
            if profile_id not in profiles:
                raise ManifestError(f"map {map_id} references an unknown profile")
    return document


def _canonical_map_name(value):
    value = str(value).strip().rstrip("/")
    parts = value.split("/")
    if parts:
        parts[-1] = parts[-1].removeprefix("UEDPIE_0_")
    value = "/".join(parts)
    if value.startswith("Game/"):
        value = "/" + value
    elif value.startswith(("Carla/", "map_package/")):
        value = "/Game/" + value
    return value


def collect_live_inventory(manifest, host, port, timeout):
    try:
        import carla
    except ImportError as error:
        raise RuntimeError(
            "CARLA Python API is required only for --live; prepend the selected "
            "server profile's python_api to PYTHONPATH"
        ) from error

    client = carla.Client(host, port)
    client.set_timeout(timeout)
    available_maps = sorted(
        {_canonical_map_name(value) for value in client.get_available_maps()}
    )
    world = client.get_world()
    current_map = world.get_map()
    current_name = _canonical_map_name(current_map.name)
    topology_count = len(current_map.get_topology())
    spawn_count = len(current_map.get_spawn_points())

    maps = []
    for entry in manifest["maps"]:
        load_name = _canonical_map_name(entry["load_name"])
        is_current = load_name == current_name
        maps.append(
            {
                "id": entry["id"],
                "load_name": entry["load_name"],
                "advertised_by_server": load_name in available_maps,
                "is_current_map": is_current,
                "topology_count": topology_count if is_current else None,
                "spawn_point_count": spawn_count if is_current else None,
                "inspection": "current_map" if is_current else "not_loaded_by_inventory",
            }
        )
    return {
        "success": True,
        "host": host,
        "port": port,
        "client_version": client.get_client_version(),
        "server_version": client.get_server_version(),
        "available_maps": available_maps,
        "current_map": current_name,
        "current_topology_count": topology_count,
        "current_spawn_point_count": spawn_count,
        "maps": maps,
        "map_loading_performed": False,
    }


def atomic_write_json(path, payload):
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise
    return path


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Validate the CARLA expert-map manifest and optionally inspect a live server."
    )
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args(argv)
    if not 0 < args.port <= 65535:
        parser.error("--port must be in [1, 65535]")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    return args


def main(argv=None):
    args = parse_args(argv)
    try:
        manifest, manifest_path = load_manifest(args.manifest)
    except ManifestError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2

    report = {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "manifest": str(manifest_path),
        "suite_id": manifest["suite_id"],
        "static": {
            "success": True,
            "server_profile_count": len(manifest["server_profiles"]),
            "map_count": len(manifest["maps"]),
            "status_counts": {
                status: sum(1 for item in manifest["maps"] if item["status"] == status)
                for status in ("ready", "source_editor_required", "unavailable")
            },
        },
        "live": None,
    }
    exit_code = 0
    if args.live:
        try:
            report["live"] = collect_live_inventory(
                manifest, args.host, args.port, args.timeout
            )
        except Exception as error:
            report["live"] = {
                "success": False,
                "host": args.host,
                "port": args.port,
                "error": f"{type(error).__name__}: {error}",
                "map_loading_performed": False,
            }
            exit_code = 1

    output = atomic_write_json(args.output, report)
    print(f"inventory={output} static=PASS live={'PASS' if args.live and exit_code == 0 else 'SKIPPED' if not args.live else 'FAIL'}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
