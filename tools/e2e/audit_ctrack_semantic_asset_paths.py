#!/usr/bin/env python3
"""HH_260929 - Audit C-track mesh paths against this host's CARLA tagger source.

This reads local source and packaged assets only. It does not run CARLA or
infer semantic ground truth from RGB images.
"""

from __future__ import annotations

from collections import Counter
import hashlib
import json
from pathlib import Path
import re


CARLA_CONTENT = Path("/home/a/carla-autoware-universe/CARLA_0.9.15/CarlaUE4/Content")
TAGGER_DIR = Path("/home/a/carla/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Game")
PINNED_SHA256 = {
    "Tagger.h": "ba5dd4bc118e06a4e595c4295d9ca4cb53c984ff97da7e5de342e53d1d58cd17",
    "Tagger.cpp": "fb71f676e987506e41edcbc6309323d1e2dbb5408fb8ee20addeb0b5708b372b",
    "C_track_1_0_7.umap": "d44c26a3e7fae4e05cd01cc12022e96daf7da5b4650797718442b143743e72a7",
    "Town05_Opt.umap": "89596dd951846a719a7dd71a342a40f090119ff32941337b2f4cf0d4f06a0a82",
    "T05_Ground.umap": "49f21962c5bcd7b604d00e62c8d9ac2f79936859f8e313a6ba713c48cbaa7893",
    "C_track_1_0_7_Roads_Road_Layer0Node.uasset": "c8cd7d0940d073ed912815d740832f2f91406761038d8477b76c2a1febbe8106",
    "C_track_1_0_7_Roads_Marking_Layer0Node.uasset": "271403687def81cd3cb828d0dbaafa51692121b7e9aebe47f0e9301e2157bdaa",
    "C_track_1_0_7_Roads_Sidewalk_Layer0Node.uasset": "f56e6950e9161805de757ad0cca931aa62cedfe4e20fc96934c0ad6a05114874",
}
FAMILIES = {
    "road": ("_Roads_Road_Layer0", "Road"),
    "lane_marking": ("_Roads_Marking_Layer0", "RoadLine"),
    "sidewalk": ("_Roads_Sidewalk_Layer0", "SideWalk"),
}


def printable_game_paths(data: bytes) -> set[str]:
    """HH_260929 - Match printable Unreal package strings, not binary offsets."""
    return {
        token.decode("ascii")
        for token in re.findall(rb"[ -~]{4,}", data)
        if token.startswith(b"/Game/Carla/") and b" " not in token
    }


def tagger_folder(path: str) -> str:
    """HH_260929 - Preserve the leading empty token used by Split('/', false)."""
    pieces = path.split("/")
    return pieces[4] if len(pieces) > 4 else ""


def classify_ctrack_paths(paths: set[str], content: Path) -> dict:
    base = "/Game/Carla/Maps/C_track_1_0_7/"
    package_dir = content / "Carla/Maps/C_track_1_0_7"
    rows = {}
    for family, (fragment, expected_folder) in FAMILIES.items():
        matched = sorted(path for path in paths
                         if path.startswith(base) and fragment in path[len(base):])
        assets = []
        for path in matched:
            name = path[len(base):]
            asset = package_dir / f"{name}.uasset"
            if not asset.is_file() or asset.is_symlink():
                raise ValueError(f"referenced packaged mesh missing: {asset}")
            raw = asset.read_bytes()
            if b"StaticMesh" not in raw:
                raise ValueError(f"packaged asset lacks StaticMesh marker: {asset}")
            assets.append((str(asset), hashlib.sha256(raw).hexdigest()))
        if not matched:
            raise ValueError(f"no C-track {family} references found")
        rows[family] = {
            "reference_count_unique": len(matched),
            "matching_tagger_folder_count": sum(
                tagger_folder(path) == expected_folder for path in matched),
            "actual_tagger_folders": dict(sorted(Counter(
                tagger_folder(path) for path in matched).items())),
            "companion_static_mesh_assets_confirmed": len(assets),
            "example_reference": matched[0],
            "example_asset_sha256": assets[0][1],
        }
    return rows


def classify_town05_paths(paths: set[str]) -> dict:
    rows = {}
    for family, (_, expected_folder) in FAMILIES.items():
        matched = sorted(path for path in paths if path.startswith(
            f"/Game/Carla/Static/{expected_folder}/"))
        if not matched:
            raise ValueError(f"no Town05 {family} reference found")
        rows[family] = {
            "reference_count_unique": len(matched),
            "matching_tagger_folder_count": sum(
                tagger_folder(path) == expected_folder for path in matched),
            "example_reference": matched[0],
        }
    return rows


def _read_pinned(path: Path) -> bytes:
    if not path.is_file() or path.is_symlink():
        raise ValueError(f"regular source file required: {path}")
    raw = path.read_bytes()
    expected = PINNED_SHA256[path.name]
    actual = hashlib.sha256(raw).hexdigest()
    if actual != expected:
        raise ValueError(f"pinned source changed: {path}: {actual} != {expected}")
    return raw


def audit() -> dict:
    header_path = TAGGER_DIR / "Tagger.h"
    source_path = TAGGER_DIR / "Tagger.cpp"
    ctrack_path = CARLA_CONTENT / "Carla/Maps/C_track_1_0_7.umap"
    town05_path = CARLA_CONTENT / "Carla/Maps/Town05_Opt.umap"
    ground_path = CARLA_CONTENT / "Carla/Maps/Sublevels/Town05_Opt/T05_Ground.umap"
    header = _read_pinned(header_path)
    source = _read_pinned(source_path)
    if b"StringArray[4]" not in header or b"GetStaticMesh()" not in source:
        raise ValueError("tagger path selection contract changed")
    for folder in (b"Road", b"RoadLine", b"SideWalk"):
        if b'String == "' + folder + b'"' not in source:
            raise ValueError(f"tagger folder classification changed: {folder!r}")
    ctrack = classify_ctrack_paths(printable_game_paths(_read_pinned(ctrack_path)),
                                   CARLA_CONTENT)
    town05_level = printable_game_paths(_read_pinned(town05_path))
    if "/Game/Carla/Maps/Sublevels/Town05_Opt/T05_Ground" not in town05_level:
        raise ValueError("Town05_Opt no longer references the T05_Ground sublevel")
    town05 = classify_town05_paths(printable_game_paths(_read_pinned(ground_path)))
    mesh_dir = CARLA_CONTENT / "Carla/Maps/C_track_1_0_7"
    examples = {}
    for name in PINNED_SHA256:
        if name.endswith(".uasset"):
            path = mesh_dir / name
            _read_pinned(path)
            examples[name] = {"path": str(path), "sha256": PINNED_SHA256[name]}
    return {
        "schema": "portable_e2e.ctrack_semantic_asset_path_audit.v1",
        "method": "read-only printable package-reference and static-mesh marker audit",
        "carla_runtime_used": False,
        "runtime_root_cause_proven": False,
        "dataset_admitted": False,
        "tagger_source": {name: {"path": str(TAGGER_DIR / name),
                                 "sha256": PINNED_SHA256[name]}
                          for name in ("Tagger.h", "Tagger.cpp")},
        "packaged_levels": {
            name: {"path": str(path), "sha256": PINNED_SHA256[name]}
            for name, path in (
                ("C_track_1_0_7.umap", ctrack_path),
                ("Town05_Opt.umap", town05_path),
                ("T05_Ground.umap", ground_path),
            )
        },
        "sample_packaged_mesh_assets": examples,
        "ctrack": ctrack,
        "town05_opt_ground_control": town05,
    }


if __name__ == "__main__":
    print(json.dumps(audit(), indent=2, sort_keys=True))
