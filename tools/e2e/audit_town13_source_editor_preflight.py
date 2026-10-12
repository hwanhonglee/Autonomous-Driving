#!/usr/bin/env python3
"""Read-only Town13 source-editor prerequisites; never authorize a launch."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

try:
    from .inventory_carla_training_maps import DEFAULT_MANIFEST, ManifestError, load_manifest
except ImportError:
    from inventory_carla_training_maps import DEFAULT_MANIFEST, ManifestError, load_manifest


EXPECTED_MAP = {
    "id": "town13",
    "canonical_name": "Town13",
    "load_name": "/Game/Carla/Maps/Town13/Town13",
    "split": "validation",
    "status": "source_editor_required",
    "server_profile": "source_editor_0915_4ws",
}
EXPECTED_SHA256 = {
    "executable": "331f8ece41d8e09c85e927046a9afa522de2dc0a56affa560849d677b9ccb9e9",
    "project": "11b5ca8a37d550f45fcf5b93d15e450b7df944746ea599518cca02d6e8eddde2",
    "python_api": "ee7209c5570cb9201e9fbd3cbcefc5b176dd4f6b2289df52bb0bd7dc2c56800b",
    "level_path": "390179369aa348ab97a7dc6a98e31123ecba23bfdd9b4a34f2d87d6061c097b7",
    "opendrive_path": "5354326f2fe4d26f0dcb5c82acf462257e638278363ab7adecb97ec3aa1f426f",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def inspect_file(path: Path, expected_sha256: str) -> dict[str, Any]:
    """HH_260906 - Reject missing, symlinked, or changed source assets."""
    result: dict[str, Any] = {"path": str(path), "expected_sha256": expected_sha256, "ok": False}
    if not path.is_absolute() or path.is_symlink() or not path.is_file():
        result["reason"] = "missing_nonregular_or_symlink"
        return result
    try:
        result["size_bytes"] = path.stat().st_size
        result["sha256"] = sha256_file(path)
    except OSError as error:
        result["reason"] = f"read_error:{type(error).__name__}"
        return result
    result["ok"] = result["sha256"] == expected_sha256
    if not result["ok"]:
        result["reason"] = "sha256_mismatch"
    return result


def inspect_route(path: Path | None, declared_sha256: str | None) -> dict[str, Any]:
    """HH_260906 - Require a byte-pinned Town13 route before even considering a scout."""
    result: dict[str, Any] = {"ok": False, "path": str(path) if path else None}
    if path is None or declared_sha256 is None:
        result["reason"] = "exact_route_and_sha256_not_declared"
        return result
    if not path.is_absolute() or path.is_symlink() or not path.is_file():
        result["reason"] = "route_missing_nonregular_or_symlink"
        return result
    if len(declared_sha256) != 64 or any(c not in "0123456789abcdef" for c in declared_sha256):
        result["reason"] = "route_sha256_invalid"
        return result
    try:
        payload = path.read_bytes()
        result["sha256"] = hashlib.sha256(payload).hexdigest()
        route = json.loads(payload)
    except (OSError, ValueError, UnicodeDecodeError) as error:
        result["reason"] = f"route_read_or_json_error:{type(error).__name__}"
        return result
    if result["sha256"] != declared_sha256:
        result["reason"] = "route_sha256_mismatch"
        return result
    if not isinstance(route, dict) or route.get("town") != "Town13" or route.get("coordinate_reference") != "base_link":
        result["reason"] = "route_town_or_coordinate_mismatch"
        return result
    if not isinstance(route.get("route"), list) or len(route["route"]) < 2:
        result["reason"] = "route_points_missing"
        return result
    if not all(isinstance(route.get(key), int) and route[key] >= 0 for key in ("start_spawn_index", "goal_spawn_index")):
        result["reason"] = "route_spawn_indices_missing"
        return result
    result.update(ok=True, start_spawn_index=route["start_spawn_index"],
                  goal_spawn_index=route["goal_spawn_index"], route_points=len(route["route"]))
    return result


def audit_town13(
    manifest: dict[str, Any], route_path: Path | None = None,
    route_sha256: str | None = None,
    expected_sha256: dict[str, str] = EXPECTED_SHA256,
) -> dict[str, Any]:
    """HH_260906 - Separate static prerequisites from owner and live-world authority."""
    blockers: list[str] = []
    map_entry = next((item for item in manifest["maps"] if item["id"] == "town13"), None)
    if map_entry is None or any(map_entry.get(key) != value for key, value in EXPECTED_MAP.items()):
        blockers.append("town13_suite_binding_mismatch")
        assets: dict[str, dict[str, Any]] = {}
    else:
        profile = manifest["server_profiles"].get(EXPECTED_MAP["server_profile"])
        if profile is None or profile.get("kind") != "source_editor" or profile.get("python_api_sha256") != expected_sha256["python_api"]:
            blockers.append("town13_source_profile_mismatch")
            assets = {}
        else:
            paths = {key: profile[key] for key in ("executable", "project", "python_api")}
            paths.update({key: map_entry[key] for key in ("level_path", "opendrive_path")})
            assets = {key: inspect_file(Path(value), expected_sha256[key]) for key, value in paths.items()}
            blockers.extend(f"{key}_{asset.get('reason', 'not_verified')}" for key, asset in assets.items() if not asset["ok"])
    route = inspect_route(route_path, route_sha256)
    if not route["ok"]:
        blockers.append(route["reason"])
    # HH_260906 - The catalog-only owner exists, but its live start and the camera scout remain unverified.
    blockers.extend(("source_editor_catalog_owner_live_start_unverified",
                     "town13_mapping_probe_scope_not_implemented",
                     "owned_live_world_and_camera_visibility_unverified"))
    return {
        "schema": "portable_e2e.town13_static_preflight.v1",
        "status": "STOP_NOT_EXECUTABLE",
        "read_only": True,
        "simulator_started": False,
        "execution_authorized": False,
        "dataset_admitted": False,
        "map_model_validation_passed": False,
        "route": route,
        "assets": assets,
        "blockers": blockers,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--route", type=Path)
    parser.add_argument("--route-sha256")
    args = parser.parse_args(argv)
    if (args.route is None) != (args.route_sha256 is None):
        parser.error("--route and --route-sha256 must be provided together")
    try:
        manifest, _ = load_manifest(args.manifest)
        report = audit_town13(manifest, args.route, args.route_sha256)
    except (ManifestError, OSError, KeyError, TypeError, ValueError) as error:
        report = {"schema": "portable_e2e.town13_static_preflight.v1",
                  "status": "STOP_NOT_EXECUTABLE", "read_only": True,
                  "execution_authorized": False, "dataset_admitted": False,
                  "blockers": [f"manifest_or_preflight_error:{type(error).__name__}:{error}"]}
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
