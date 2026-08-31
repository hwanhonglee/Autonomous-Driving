#!/usr/bin/env python3

"""Inventory and prepare pinned Autoware full-map bundles for packaged CARLA Towns."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
from typing import Any
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST = Path(__file__).with_name("packaged_town_full_maps.yaml")
DEFAULT_GENERATED_ROOT = ROOT / "data/generated/packaged_town_full_maps"
DEFAULT_TARGET_ROOT = ROOT / "data/maps"
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
REFLECTION_MATRIX = (
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    -1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    0.0,
    1.0,
)
# Keep correspondence selection inside the existing maximum planar admission
# threshold.  Within that gate, full XYZ distance disambiguates bridges,
# tunnels, and other multi-height surfaces without making the threshold more
# permissive.
ROUTE_PCD_XY_RADIUS_M = 1.0


class TownMapError(RuntimeError):
    """Raised when a source or generated bundle violates its pinned contract."""


def _load_helper(filename: str, module_name: str) -> Any:
    path = Path(__file__).with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise TownMapError(f"cannot load helper module {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(module_name, None)
        raise
    return module


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise TownMapError(f"{label} must be a mapping")
    return value


def _require_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise TownMapError(f"{label} must be a non-empty string")
    return value.strip()


def _validate_digest(value: Any, label: str) -> str:
    digest = _require_string(value, label)
    if not SHA256_PATTERN.fullmatch(digest):
        raise TownMapError(f"{label} must be a lowercase SHA-256 digest")
    return digest


def load_manifest(path: Path) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise TownMapError(f"cannot read manifest {path}: {error}") from error
    document = _require_mapping(document, "manifest")
    expected_root = {
        "schema_version",
        "suite_id",
        "description",
        "source_contracts",
        "pointcloud_transform",
        "maps",
    }
    if set(document) != expected_root:
        raise TownMapError(
            "manifest root fields differ from the strict contract: "
            f"expected={sorted(expected_root)}, actual={sorted(document)}"
        )
    if document["schema_version"] != 1:
        raise TownMapError("manifest.schema_version must be 1")
    _require_string(document["suite_id"], "manifest.suite_id")
    _require_string(document["description"], "manifest.description")
    transform = _require_mapping(document["pointcloud_transform"], "pointcloud_transform")
    matrix = transform.get("matrix_4x4_row_major")
    if not isinstance(matrix, list) or tuple(float(value) for value in matrix) != REFLECTION_MATRIX:
        raise TownMapError("pointcloud_transform must be the pinned CARLA-to-ROS Y reflection")
    if transform.get("command") != "pcl_transform_point_cloud":
        raise TownMapError("pointcloud_transform.command must be pcl_transform_point_cloud")

    source_contracts = _require_mapping(document["source_contracts"], "source_contracts")
    if set(source_contracts) != {"lanelet2", "carla"}:
        raise TownMapError("source_contracts must contain lanelet2 and carla")
    for contract_id, contract_value in source_contracts.items():
        contract = _require_mapping(contract_value, f"source_contracts.{contract_id}")
        if set(contract) != {"environment", "candidates", "provenance"}:
            raise TownMapError(f"source_contracts.{contract_id} fields are invalid")
        _require_string(contract["environment"], f"source_contracts.{contract_id}.environment")
        _require_string(contract["provenance"], f"source_contracts.{contract_id}.provenance")
        if not isinstance(contract["candidates"], list) or not contract["candidates"]:
            raise TownMapError(f"source_contracts.{contract_id}.candidates must not be empty")
        for index, candidate in enumerate(contract["candidates"]):
            candidate_text = _require_string(
                candidate, f"source_contracts.{contract_id}.candidates[{index}]"
            )
            if not Path(candidate_text).is_absolute():
                raise TownMapError(
                    f"source_contracts.{contract_id}.candidates[{index}] must be absolute"
                )

    maps = document["maps"]
    if not isinstance(maps, list) or not maps:
        raise TownMapError("manifest.maps must be a non-empty list")
    ids: set[str] = set()
    targets: set[str] = set()
    expected_map_fields = {
        "id",
        "canonical_carla_map",
        "lanelet_source_town",
        "pcd_source_town",
        "target_name",
        "lanelet2",
        "pointcloud",
        "runtime",
    }
    for index, raw_map in enumerate(maps):
        entry = _require_mapping(raw_map, f"maps[{index}]")
        allowed_fields = expected_map_fields | {"blocker"}
        if set(entry) - allowed_fields or expected_map_fields - set(entry):
            raise TownMapError(f"maps[{index}] fields are invalid: {sorted(entry)}")
        map_id = _require_string(entry["id"], f"maps[{index}].id")
        if map_id in ids:
            raise TownMapError(f"duplicate map id: {map_id}")
        ids.add(map_id)
        canonical = _require_string(
            entry["canonical_carla_map"], f"maps[{index}].canonical_carla_map"
        )
        if not canonical.startswith("/Game/Carla/Maps/"):
            raise TownMapError(f"maps[{index}].canonical_carla_map is not packaged CARLA")
        target_name = _require_string(entry["target_name"], f"maps[{index}].target_name")
        if Path(target_name).name != target_name or target_name in targets:
            raise TownMapError(f"maps[{index}].target_name is unsafe or duplicated")
        targets.add(target_name)
        lanelet = _require_mapping(entry["lanelet2"], f"maps[{index}].lanelet2")
        expected_lanelet_fields = {
            "relative_path",
            "sha256",
            "size_bytes",
            "nodes",
            "ways",
            "road_lanelets",
        }
        if set(lanelet) != expected_lanelet_fields:
            raise TownMapError(f"maps[{index}].lanelet2 fields are invalid")
        _validate_digest(lanelet["sha256"], f"maps[{index}].lanelet2.sha256")
        pointcloud = entry["pointcloud"]
        if pointcloud is None:
            if entry.get("pcd_source_town") is not None or "blocker" not in entry:
                raise TownMapError(f"maps[{index}] missing pointcloud must include a blocker")
        else:
            pointcloud = _require_mapping(pointcloud, f"maps[{index}].pointcloud")
            expected_pcd_fields = {
                "relative_path",
                "sha256",
                "size_bytes",
                "points",
                "transformed_sha256",
                "transformed_size_bytes",
            }
            if set(pointcloud) != expected_pcd_fields:
                raise TownMapError(f"maps[{index}].pointcloud fields are invalid")
            _validate_digest(pointcloud["sha256"], f"maps[{index}].pointcloud.sha256")
            _validate_digest(
                pointcloud["transformed_sha256"],
                f"maps[{index}].pointcloud.transformed_sha256",
            )
        runtime = _require_mapping(entry["runtime"], f"maps[{index}].runtime")
        expected_runtime_fields = {
            "level_relative_path",
            "level_sha256",
            "opendrive_relative_path",
            "opendrive_sha256",
        }
        if set(runtime) != expected_runtime_fields:
            raise TownMapError(f"maps[{index}].runtime fields are invalid")
        _validate_digest(runtime["level_sha256"], f"maps[{index}].runtime.level_sha256")
        _validate_digest(
            runtime["opendrive_sha256"], f"maps[{index}].runtime.opendrive_sha256"
        )
    return document


def resolve_source_root(contract: dict[str, Any], marker: str) -> tuple[Path | None, list[str]]:
    environment = contract["environment"]
    configured = os.environ.get(environment)
    candidates = [configured] if configured else contract["candidates"]
    inspected: list[str] = []
    for raw_candidate in candidates:
        candidate = Path(raw_candidate)
        if not candidate.is_absolute():
            raise TownMapError(f"{environment} must be an absolute path, got {candidate}")
        inspected.append(str(candidate))
        if (candidate / marker).is_file():
            return candidate.resolve(), inspected
    return None, inspected


def inspect_file(path: Path, expected_size: int | None, expected_sha256: str) -> dict[str, Any]:
    result: dict[str, Any] = {
        "path": str(path),
        "exists": path.is_file(),
        "expected_size_bytes": expected_size,
        "expected_sha256": expected_sha256,
    }
    if not result["exists"]:
        result.update({"size_bytes": None, "sha256": None, "match": False})
        return result
    size = path.stat().st_size
    digest = sha256_file(path)
    result.update(
        {
            "size_bytes": size,
            "sha256": digest,
            "match": digest == expected_sha256
            and (expected_size is None or size == expected_size),
        }
    )
    return result


def _osm_xyz(path: Path) -> Any:
    import numpy as np

    points = []
    for element in ET.parse(path).getroot().findall("node"):
        tags = {child.get("k"): child.get("v") for child in element.findall("tag")}
        points.append(
            (
                float(tags["local_x"]),
                float(tags["local_y"]),
                float(tags["ele"]),
            )
        )
    return np.asarray(points, dtype=float)


def deep_alignment(lanelet_path: Path, source_pcd_path: Path) -> dict[str, Any]:
    try:
        import numpy as np
        import open3d as o3d
        from scipy.spatial import cKDTree
    except ImportError as error:
        raise TownMapError(f"deep alignment requires numpy, open3d, and scipy: {error}") from error

    osm = _osm_xyz(lanelet_path)
    cloud = o3d.io.read_point_cloud(str(source_pcd_path))
    source = np.asarray(cloud.points)
    if source.shape[0] == 0 or not np.isfinite(source).all():
        raise TownMapError(f"PCD is empty or non-finite: {source_pcd_path}")
    downsampled = cloud.voxel_down_sample(0.5)
    sample = np.asarray(downsampled.points)
    transformed_xy = sample[:, :2].copy()
    transformed_xy[:, 1] *= -1.0
    distances, indices = cKDTree(transformed_xy).query(osm[:, :2], k=1, workers=-1)
    raw_distances, _ = cKDTree(sample[:, :2]).query(osm[:, :2], k=1, workers=-1)
    z_distances = np.abs(sample[indices, 2] - osm[:, 2])
    transformed_min = source.min(axis=0).copy()
    transformed_max = source.max(axis=0).copy()
    transformed_min[1], transformed_max[1] = -source[:, 1].max(), -source[:, 1].min()
    result = {
        "method": "0.5 m voxel cloud; nearest XY to every Lanelet2 node",
        "osm_nodes": int(osm.shape[0]),
        "source_pcd_points": int(source.shape[0]),
        "downsampled_pcd_points": int(sample.shape[0]),
        "source_pcd_bounds_xyz_m": {
            "min": [float(value) for value in source.min(axis=0)],
            "max": [float(value) for value in source.max(axis=0)],
        },
        "transformed_pcd_bounds_xyz_m": {
            "min": [float(value) for value in transformed_min],
            "max": [float(value) for value in transformed_max],
        },
        "nearest_xy_without_reflection_m": {
            "median": float(np.median(raw_distances)),
            "p95": float(np.percentile(raw_distances, 95)),
            "max": float(raw_distances.max()),
        },
        "nearest_xy_with_y_reflection_m": {
            "median": float(np.median(distances)),
            "p95": float(np.percentile(distances, 95)),
            "max": float(distances.max()),
            "within_1m_fraction": float(np.mean(distances <= 1.0)),
        },
        "nearest_abs_z_with_y_reflection_m": {
            "median": float(np.median(z_distances)),
            "p95": float(np.percentile(z_distances, 95)),
        },
    }
    metrics = result["nearest_xy_with_y_reflection_m"]
    z_metrics = result["nearest_abs_z_with_y_reflection_m"]
    thresholds_pass = (
        metrics["median"] <= 0.5
        and metrics["p95"] <= 1.0
        and metrics["within_1m_fraction"] >= 0.95
        and z_metrics["p95"] <= 1.0
    )
    if not thresholds_pass:
        result["status"] = "FAIL"
    elif metrics["within_1m_fraction"] < 0.999 or metrics["max"] > 1.0:
        result["status"] = "PASS_WITH_OUTLIERS"
    else:
        result["status"] = "PASS"
    return result


def transformed_path(generated_root: Path, entry: dict[str, Any]) -> Path:
    return generated_root / entry["pcd_source_town"] / "pointcloud_map.pcd"


def inspect_installed_semantic_lidar_bundle(
    entry: dict[str, Any],
    target_path: Path,
    lanelet_file: dict[str, Any],
    helper: Any,
) -> dict[str, Any] | None:
    metadata_path = target_path / "map_bundle.json"
    if not metadata_path.is_file():
        return None
    errors: list[str] = []
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        return {"errors": [f"cannot read installed semantic bundle metadata: {error}"]}
    if not isinstance(metadata, dict) or not isinstance(
        metadata.get("semantic_lidar_provenance"), dict
    ):
        return {
            "errors": [
                "Town10HD target metadata is not an audited semantic-LiDAR bundle"
            ]
        }
    if metadata.get("profile") != entry["id"]:
        errors.append("semantic bundle profile differs from the manifest map id")
    if metadata.get("canonical_carla_map") != entry["canonical_carla_map"]:
        errors.append("semantic bundle canonical CARLA map differs from the manifest")
    if metadata.get("status") != "full_map_structurally_ready_not_vad_validated":
        errors.append("semantic bundle status is not a structural admission candidate")

    provenance_record = metadata["semantic_lidar_provenance"]
    provenance_path = Path(str(provenance_record.get("path", ""))).expanduser()
    if not provenance_path.is_absolute():
        errors.append("semantic provenance path must be absolute")
        provenance = None
    elif not provenance_path.is_file():
        errors.append("semantic provenance file is absent")
        provenance = None
    elif sha256_file(provenance_path) != provenance_record.get("sha256"):
        errors.append("semantic provenance differs from its bundle hash")
        provenance = None
    else:
        try:
            provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            errors.append(f"cannot read semantic provenance: {error}")
            provenance = None
    if not isinstance(provenance, dict) or provenance.get("status") != "COMPLETE_QA_PASS":
        errors.append("semantic provenance is not COMPLETE_QA_PASS")
    elif provenance.get("qa", {}).get("status") != "PASS":
        errors.append("semantic provenance global QA is not PASS")

    generated_record = metadata.get("pointcloud_generated")
    if not isinstance(generated_record, dict):
        errors.append("semantic bundle has no generated pointcloud record")
        generated_file = None
        pointcloud_structure = None
    else:
        expected_sha256 = generated_record.get("sha256") or generated_record.get(
            "expected_sha256"
        )
        expected_size = generated_record.get("size_bytes") or generated_record.get(
            "expected_size_bytes"
        )
        pcd_path = target_path / "pointcloud_map.pcd"
        if not isinstance(expected_sha256, str):
            errors.append("semantic pointcloud record has no SHA-256")
            generated_file = None
            pointcloud_structure = None
        else:
            generated_file = inspect_file(pcd_path, expected_size, expected_sha256)
            if not generated_file["match"]:
                errors.append("installed semantic pointcloud differs from its bundle pin")
                pointcloud_structure = None
            else:
                try:
                    pointcloud_structure = helper.inspect_pcd(pcd_path)
                except Exception as error:
                    errors.append(f"installed semantic PCD inspection failed: {error}")
                    pointcloud_structure = None
                else:
                    generated_file["inspection"] = pointcloud_structure
                    if pointcloud_structure["points"] != generated_record.get(
                        "inspection", {}
                    ).get("points"):
                        errors.append("semantic pointcloud count differs from its bundle pin")
            source_path = Path(str(generated_record.get("path", ""))).expanduser()
            if (
                not source_path.is_absolute()
                or not source_path.is_file()
                or pcd_path.resolve() != source_path.resolve()
            ):
                errors.append("bundle pointcloud symlink differs from semantic source")

    target_lanelet = target_path / "lanelet2_map.osm"
    if (
        not target_lanelet.is_file()
        or not lanelet_file.get("match")
        or sha256_file(target_lanelet) != lanelet_file.get("sha256")
    ):
        errors.append("installed semantic bundle Lanelet2 differs from its source pin")
    alignment = metadata.get("alignment")
    if not isinstance(alignment, dict) or alignment.get("status") != "PASS":
        errors.append("installed semantic bundle alignment is not PASS")
    return {
        "errors": errors,
        "metadata": metadata,
        "pointcloud_file": generated_file,
        "pointcloud_structure": pointcloud_structure,
        "alignment": alignment,
    }


def inspect_entry(
    entry: dict[str, Any],
    lanelet_root: Path | None,
    carla_root: Path | None,
    generated_root: Path,
    target_root: Path,
    helper: Any,
    run_deep_alignment: bool,
) -> dict[str, Any]:
    errors: list[str] = []
    target_path = target_root / entry["target_name"]
    lanelet_path = (
        lanelet_root / entry["lanelet2"]["relative_path"]
        if lanelet_root is not None
        else Path("/<unresolved-lanelet-root>") / entry["lanelet2"]["relative_path"]
    )
    lanelet_file = inspect_file(
        lanelet_path,
        entry["lanelet2"]["size_bytes"],
        entry["lanelet2"]["sha256"],
    )
    lanelet_structure = None
    if lanelet_file["match"]:
        try:
            lanelet_structure = helper.inspect_lanelet2(lanelet_path)
        except Exception as error:  # helper owns its domain-specific exception
            errors.append(f"Lanelet2 structural inspection failed: {error}")
        else:
            for key in ("nodes", "ways", "road_lanelets"):
                if lanelet_structure[key] != entry["lanelet2"][key]:
                    errors.append(
                        f"Lanelet2 {key} mismatch: expected {entry['lanelet2'][key]}, "
                        f"got {lanelet_structure[key]}"
                    )
    else:
        errors.append("Lanelet2 source is absent or differs from its size/hash pin")

    runtime_result: dict[str, Any] = {}
    for kind in ("level", "opendrive"):
        relative_path = entry["runtime"][f"{kind}_relative_path"]
        path = (
            carla_root / relative_path
            if carla_root is not None
            else Path("/<unresolved-carla-root>") / relative_path
        )
        runtime_result[kind] = inspect_file(
            path, None, entry["runtime"][f"{kind}_sha256"]
        )
        if not runtime_result[kind]["match"]:
            errors.append(f"runtime {kind} is absent or differs from its hash pin")

    pointcloud_file = None
    pointcloud_structure = None
    generated_file = None
    alignment = None
    semantic_bundle = None
    required_transform = "reflect Y: (x, y, z) -> (x, -y, z)"
    if entry["pointcloud"] is None:
        semantic_bundle = inspect_installed_semantic_lidar_bundle(
            entry, target_path, lanelet_file, helper
        )
        if semantic_bundle is None:
            errors.append(entry["blocker"])
        else:
            errors.extend(semantic_bundle["errors"])
            pointcloud_file = semantic_bundle.get("pointcloud_file")
            pointcloud_structure = semantic_bundle.get("pointcloud_structure")
            generated_file = semantic_bundle.get("pointcloud_file")
            alignment = semantic_bundle.get("alignment")
            required_transform = "none; semantic collector output is already ROS Local"
    else:
        source_pcd = (
            carla_root / entry["pointcloud"]["relative_path"]
            if carla_root is not None
            else Path("/<unresolved-carla-root>") / entry["pointcloud"]["relative_path"]
        )
        pointcloud_file = inspect_file(
            source_pcd,
            entry["pointcloud"]["size_bytes"],
            entry["pointcloud"]["sha256"],
        )
        if pointcloud_file["match"]:
            try:
                pointcloud_structure = helper.inspect_pcd(source_pcd)
            except Exception as error:
                errors.append(f"source PCD structural inspection failed: {error}")
            else:
                if pointcloud_structure["points"] != entry["pointcloud"]["points"]:
                    errors.append("source PCD point count differs from the manifest")
            if run_deep_alignment and lanelet_file["match"]:
                try:
                    alignment = deep_alignment(lanelet_path, source_pcd)
                except Exception as error:
                    errors.append(f"deep alignment failed: {error}")
                else:
                    if alignment["status"] == "FAIL":
                        errors.append("deep Lanelet2/PCD alignment thresholds failed")
        else:
            errors.append("source PCD is absent or differs from its size/hash pin")

        output_path = transformed_path(generated_root, entry)
        generated_file = inspect_file(
            output_path,
            entry["pointcloud"]["transformed_size_bytes"],
            entry["pointcloud"]["transformed_sha256"],
        )
        if generated_file["exists"] and generated_file["match"]:
            try:
                generated_structure = helper.inspect_pcd(output_path)
            except Exception as error:
                errors.append(f"generated PCD structural inspection failed: {error}")
            else:
                generated_file["inspection"] = generated_structure
                if generated_structure["points"] != entry["pointcloud"]["points"]:
                    errors.append("generated PCD point count differs from the manifest")
        elif generated_file["exists"]:
            errors.append("generated PCD exists but differs from its pinned transform output")

    bundle_files = {
        name: str(target_path / name)
        for name in (
            "lanelet2_map.osm",
            "pointcloud_map.pcd",
            "map_projector_info.yaml",
            "map_bundle.json",
        )
    }
    bundle_complete = all(Path(path).is_file() for path in bundle_files.values())
    blocking_source_error = bool(errors)
    if entry["pointcloud"] is None and semantic_bundle is None:
        status = "BLOCKED_MISSING_POINTCLOUD"
    elif entry["pointcloud"] is None and errors:
        status = "FAIL"
    elif entry["pointcloud"] is None and bundle_complete and generated_file:
        status = "FULL_MAP_READY"
    elif blocking_source_error:
        status = "FAIL"
    elif bundle_complete and generated_file and generated_file["match"]:
        status = "FULL_MAP_READY"
    else:
        status = "READY_TO_PREPARE"
    return {
        "id": entry["id"],
        "canonical_carla_map": entry["canonical_carla_map"],
        "target_name": entry["target_name"],
        "status": status,
        "errors": errors,
        "lanelet2": {"file": lanelet_file, "inspection": lanelet_structure},
        "pointcloud_source": {
            "file": pointcloud_file,
            "inspection": pointcloud_structure,
            "required_transform": required_transform,
        },
        "pointcloud_generated": generated_file,
        "alignment": alignment,
        "runtime": runtime_result,
        "bundle": {
            "path": str(target_path),
            "complete": bundle_complete,
            "files": bundle_files,
        },
        "blocker": entry.get("blocker") if semantic_bundle is None else None,
    }


def _atomic_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, prefix=f".{path.name}.", delete=False
    ) as stream:
        stream.write(content)
        temp_path = Path(stream.name)
    os.replace(temp_path, path)


def _ensure_symlink(link: Path, source: Path) -> None:
    if link.is_symlink():
        if link.resolve() == source.resolve():
            return
        raise TownMapError(f"refusing to replace symlink with another target: {link}")
    if link.exists():
        raise TownMapError(f"refusing to replace regular bundle asset: {link}")
    temporary = link.with_name(f".{link.name}.{os.getpid()}.tmp")
    temporary.symlink_to(source.resolve())
    os.replace(temporary, link)


def prepare_entry(
    entry: dict[str, Any],
    inspection: dict[str, Any],
    lanelet_root: Path,
    carla_root: Path,
    generated_root: Path,
    target_root: Path,
    helper: Any,
) -> None:
    if entry["pointcloud"] is None:
        return
    if inspection["status"] == "FAIL":
        raise TownMapError(f"cannot prepare {entry['id']}: source inspection failed")
    source_pcd = carla_root / entry["pointcloud"]["relative_path"]
    output_pcd = transformed_path(generated_root, entry)
    existing_output = inspection["pointcloud_generated"]
    if existing_output is None or not existing_output["match"]:
        if existing_output and existing_output["exists"]:
            raise TownMapError(f"refusing to overwrite unpinned generated PCD: {output_pcd}")
        output_pcd.parent.mkdir(parents=True, exist_ok=True)
        temporary = output_pcd.with_name(f".{output_pcd.stem}.{os.getpid()}.tmp.pcd")
        command = [
            "pcl_transform_point_cloud",
            str(source_pcd),
            str(temporary),
            "-matrix",
            ",".join(str(value) for value in REFLECTION_MATRIX),
        ]
        try:
            subprocess.run(command, check=True)
            generated = inspect_file(
                temporary,
                entry["pointcloud"]["transformed_size_bytes"],
                entry["pointcloud"]["transformed_sha256"],
            )
            structure = helper.inspect_pcd(temporary)
            if not generated["match"] or structure["points"] != entry["pointcloud"]["points"]:
                raise TownMapError(f"generated PCD verification failed for {entry['id']}")
            os.replace(temporary, output_pcd)
        finally:
            if temporary.exists():
                temporary.unlink()

    target = target_root / entry["target_name"]
    target.mkdir(parents=True, exist_ok=True)
    lanelet_path = lanelet_root / entry["lanelet2"]["relative_path"]
    _ensure_symlink(target / "lanelet2_map.osm", lanelet_path)
    _ensure_symlink(target / "pointcloud_map.pcd", output_pcd)
    _atomic_text(target / "map_projector_info.yaml", "projector_type: Local\n")
    metadata = {
        "schema_version": 1,
        "profile": entry["id"],
        "status": "full_map_structurally_ready_not_vad_validated",
        "canonical_carla_map": entry["canonical_carla_map"],
        "projector_type": "Local",
        "carla_to_map_transform": {
            "kind": "identity_after_carla_interface_ros_handedness_conversion",
            "x_m": 0.0,
            "y_m": 0.0,
            "z_m": 0.0,
            "yaw_rad": 0.0,
            "confidence": "lanelet_route_and_reflected_pcd_alignment_verified",
            "source": "packaged Town route/Lanelet2/HDMaps audit on 2026-08-31",
        },
        "pointcloud_source_transform": {
            "kind": "carla_unreal_to_ros_y_reflection",
            "matrix_4x4_row_major": list(REFLECTION_MATRIX),
        },
        "lanelet2": inspection["lanelet2"],
        "pointcloud_source": inspection["pointcloud_source"],
        "pointcloud_generated": inspect_file(
            output_pcd,
            entry["pointcloud"]["transformed_size_bytes"],
            entry["pointcloud"]["transformed_sha256"],
        ),
        "alignment": inspection["alignment"],
        "runtime": inspection["runtime"],
    }
    _atomic_text(target / "map_bundle.json", json.dumps(metadata, indent=2, sort_keys=True) + "\n")


def _height_aware_route_correspondence(
    route_xyz: Any, points: Any, xy_tree: Any, xy_radius_m: float
) -> tuple[Any, Any, Any, int]:
    """Match route poses to nearby cloud points using full XYZ distance.

    An XY-only nearest-neighbor query is ambiguous when a bridge or roof is
    vertically above the road.  Restricting candidates to the existing planar
    admission radius retains the route coverage contract; selecting the
    minimum full XYZ distance then chooses the surface at the route height.
    Poses with no candidate are represented by infinite distances and counted
    explicitly so they cannot silently pass.
    """
    import numpy as np

    candidate_lists = xy_tree.query_ball_point(
        route_xyz[:, :2], r=xy_radius_m, workers=-1
    )
    xy_distances = np.full(route_xyz.shape[0], np.inf, dtype=float)
    z_distances = np.full(route_xyz.shape[0], np.inf, dtype=float)
    selected_indices = np.full(route_xyz.shape[0], -1, dtype=int)

    for pose_index, candidates in enumerate(candidate_lists):
        if not candidates:
            continue
        candidate_indices = np.asarray(candidates, dtype=int)
        deltas = points[candidate_indices] - route_xyz[pose_index]
        distance_squared = np.einsum("ij,ij->i", deltas, deltas)
        # cKDTree does not promise candidate-list ordering.  Use point index
        # as a secondary key to make equal-distance correspondence stable.
        order = np.lexsort((candidate_indices, distance_squared))
        selected = candidate_indices[order[0]]
        selected_indices[pose_index] = selected
        selected_delta = points[selected] - route_xyz[pose_index]
        xy_distances[pose_index] = float(np.linalg.norm(selected_delta[:2]))
        z_distances[pose_index] = float(abs(selected_delta[2]))

    unmatched_count = int(np.count_nonzero(selected_indices < 0))
    return xy_distances, z_distances, selected_indices, unmatched_count


def validate_route_pcd_proximity(
    route_paths: list[Path], map_path: Path
) -> dict[str, dict[str, Any]]:
    try:
        import numpy as np
        import open3d as o3d
        from scipy.spatial import cKDTree
    except ImportError as error:
        raise TownMapError(f"route/PCD validation requires numpy, open3d, scipy: {error}") from error
    pcd_path = map_path / "pointcloud_map.pcd"
    cloud = o3d.io.read_point_cloud(str(pcd_path)).voxel_down_sample(0.5)
    points = np.asarray(cloud.points)
    if points.shape[0] == 0 or not np.isfinite(points).all():
        raise TownMapError(f"bundle PCD is empty or non-finite: {pcd_path}")
    tree = cKDTree(points[:, :2])
    results: dict[str, dict[str, Any]] = {}
    for route_path in route_paths:
        try:
            payload = json.loads(route_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise TownMapError(f"cannot read route {route_path}: {error}") from error
        poses = [payload.get("start_ros_pose"), *payload.get("route", [])]
        route_xyz = np.asarray(
            [
                [float(pose["x"]), float(pose["y"]), float(pose["z"])]
                for pose in poses
                if isinstance(pose, dict)
            ],
            dtype=float,
        )
        if route_xyz.shape[0] == 0 or not np.isfinite(route_xyz).all():
            raise TownMapError(f"route contains no finite ROS poses: {route_path}")
        distances, z_distances, _indices, unmatched_count = (
            _height_aware_route_correspondence(
                route_xyz, points, tree, ROUTE_PCD_XY_RADIUS_M
            )
        )
        status = (
            "PASS"
            if unmatched_count == 0
            and float(distances.max()) <= ROUTE_PCD_XY_RADIUS_M
            and float(np.percentile(distances, 95)) <= 0.75
            and float(np.percentile(z_distances, 95)) <= 5.0
            else "FAIL"
        )
        results[str(route_path)] = {
            "status": status,
            "method": (
                "minimum-3D point among candidates within "
                f"{ROUTE_PCD_XY_RADIUS_M:.1f} m XY in 0.5 m voxel-downsampled "
                "transformed bundle PCD"
            ),
            "xy_candidate_radius_m": ROUTE_PCD_XY_RADIUS_M,
            "pose_count": int(route_xyz.shape[0]),
            "unmatched_pose_count": unmatched_count,
            "nearest_xy_m": {
                "median": float(np.median(distances)),
                "p95": float(np.percentile(distances, 95)),
                "max": float(distances.max()),
            },
            "nearest_abs_z_m": {
                "median": float(np.median(z_distances)),
                "p95": float(np.percentile(z_distances, 95)),
                "max": float(z_distances.max()),
            },
        }
    return results


def validate_catalog_routes(
    report_entry: dict[str, Any], route_root: Path, validator: Any
) -> dict[str, Any]:
    map_id = report_entry["id"]
    route_dir = route_root / map_id / "catalog/routes" / map_id
    route_paths = sorted(route_dir.glob("*/*.json")) if route_dir.is_dir() else []
    try:
        pcd_results = validate_route_pcd_proximity(
            route_paths, Path(report_entry["bundle"]["path"])
        )
    except Exception as error:
        pcd_results = {str(path): {"status": "FAIL", "error": str(error)} for path in route_paths}
    cases = []
    for route_path in route_paths:
        scenario = route_path.parent.name
        try:
            result = validator.validate_route_map(
                route_path,
                Path(report_entry["bundle"]["path"]),
                tolerance_m=0.75,
                vertical_tolerance_m=5.0,
            )
        except Exception as error:
            cases.append(
                {
                    "scenario": scenario,
                    "route": str(route_path),
                    "status": "FAIL",
                    "error": str(error),
                }
            )
        else:
            pcd_result = pcd_results[str(route_path)]
            cases.append(
                {
                    "scenario": scenario,
                    "route": str(route_path),
                    "status": "PASS" if pcd_result["status"] == "PASS" else "FAIL",
                    "maximum_lanelet_distance_m": result["maximum_lanelet_distance_m"],
                    "maximum_vertical_distance_m": result[
                        "maximum_lanelet_vertical_distance_m"
                    ],
                    "route_pose_count": result["route_pose_count"],
                    "pointcloud_proximity": pcd_result,
                }
            )
    scenario_status = {
        scenario: "PASS"
        if any(case["scenario"] == scenario and case["status"] == "PASS" for case in cases)
        else "MISSING_OR_FAIL"
        for scenario in ("straight", "left", "right")
    }
    status = (
        "PASS"
        if cases
        and all(case["status"] == "PASS" for case in cases)
        and scenario_status["straight"] == "PASS"
        and any(scenario_status[name] == "PASS" for name in ("left", "right"))
        else "FAIL"
    )
    return {
        "status": status,
        "route_directory": str(route_dir),
        "case_count": len(cases),
        "pass_count": sum(case["status"] == "PASS" for case in cases),
        "scenario_status": scenario_status,
        "cases": cases,
    }


def render_markdown(report: dict[str, Any]) -> str:
    lines = [
        "# Packaged CARLA Town full-map readiness",
        "",
        "This report distinguishes map/route structural preflight from an actual Autoware VAD run. "
        "`TEST_ROUTES_MAP_PREFLIGHT_PASS` does **not** claim that VAD drove the route.",
        "",
        "| Map | Result | Lanelet2 | ROS PCD | global XY p95 | global <=1m | straight | turn | Blocker |",
        "|---|---|---:|---:|---:|---:|---:|---:|---|",
    ]
    for entry in report["maps"]:
        lanelet = entry["lanelet2"]["inspection"]
        pcd = entry["pointcloud_generated"]
        alignment = entry["alignment"]
        routes = entry.get("route_preflight")
        lanelet_text = str(lanelet["road_lanelets"]) if lanelet else "FAIL"
        pcd_text = str(pcd["inspection"]["points"]) if pcd and pcd.get("inspection") else "—"
        p95 = (
            f"{alignment['nearest_xy_with_y_reflection_m']['p95']:.3f} m"
            if alignment
            else "—"
        )
        within_1m = (
            f"{100.0 * alignment['nearest_xy_with_y_reflection_m']['within_1m_fraction']:.2f}%"
            if alignment
            else "—"
        )
        straight = routes["scenario_status"]["straight"] if routes else "—"
        turn = (
            "PASS"
            if routes
            and any(
                routes["scenario_status"][name] == "PASS" for name in ("left", "right")
            )
            else "—"
        )
        blocker = entry.get("blocker") or "; ".join(entry["errors"])
        lines.append(
            f"| {entry['id']} | {entry['status']} | {lanelet_text} | {pcd_text} | "
            f"{p95} | {within_1m} | {straight} | {turn} | {blocker or ''} |"
        )
    lines.extend(
        [
            "",
            "## Coordinate finding",
            "",
            "The packaged `HDMaps/*.pcd` files use CARLA/Unreal handedness. The ROS route and "
            "Lanelet2 maps use `(x, -y)`. Each ready bundle therefore contains a pinned PCD "
            "created with the matrix `diag(1, -1, 1, 1)`; raw PCD symlinks would be wrong.",
            "",
        ]
    )

    town10 = next((entry for entry in report["maps"] if entry["id"] == "town10hd_opt"), None)
    if town10 is not None:
        status = town10["status"]
        recovered = status == "FULL_MAP_READY" or status.startswith(
            "TEST_ROUTES_MAP_PREFLIGHT_PASS"
        )
        lines.extend(["## Town10HD_Opt", ""])
        if recovered:
            pointcloud = town10.get("pointcloud_generated") or {}
            point_count = (pointcloud.get("inspection") or {}).get("points")
            point_count_text = (
                f" with {point_count:,} points" if isinstance(point_count, int) else ""
            )
            lines.append(
                "An audited semantic-LiDAR PCD is installed and pinned for Town10HD_Opt"
                f"{point_count_text}. Its collector provenance is `COMPLETE_QA_PASS`, and the "
                f"current readiness result is `{status}`. The cloud is already in ROS Local "
                "coordinates and must not be reflected a second time."
            )
            routes = town10.get("route_preflight")
            if routes and routes.get("status") == "PASS":
                lines.append(
                    "The requested catalog's straight and turn route-to-Lanelet/PCD preflights "
                    "also pass. This establishes full-map structural readiness; it does not by "
                    "itself claim that Autoware VAD drove either route."
                )
            else:
                lines.append(
                    "No route preflight was requested in this report. Full-map structural "
                    "readiness does not by itself claim an Autoware VAD drive."
                )
        elif status == "BLOCKED_MISSING_POINTCLOUD":
            lines.extend(
                [
                    "The Lanelet2 map, cooked level, and OpenDRIVE are pinned, but no Town10HD "
                    "point cloud was found. It remains blocked for full Autoware validation "
                    "until a PCD is generated and aligned. CARLA-only driving evidence is a "
                    "different result.",
                    "The practical recovery path on this host is a deterministic synchronous "
                    "semantic-LiDAR sweep over the complete OpenDRIVE waypoint graph (or an "
                    "Unreal Editor collision-mesh export), followed by world-frame "
                    "accumulation, static-class filtering, voxelization, Y reflection, and "
                    "Lanelet2/route QA. A cloud synthesized only from Lanelet boundaries is not "
                    "acceptable localization-map evidence.",
                    "That recovery workflow is implemented by "
                    "`scripts/e2e/generate_carla_semantic_lidar_map.py`; Town10HD_Opt stays "
                    "blocked until the collector reports `COMPLETE_QA_PASS` and the generated "
                    "PCD is pinned.",
                ]
            )
        else:
            lines.append(
                f"The Town10HD_Opt candidate is not admitted: its current result is `{status}`. "
                "The table's blocker and errors are authoritative. Recovery requires audited "
                "`COMPLETE_QA_PASS` semantic-LiDAR provenance, a pinned ROS-frame PCD, and "
                "passing alignment checks."
            )
        lines.append("")
    return "\n".join(lines)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--generated-root", type=Path, default=DEFAULT_GENERATED_ROOT)
    parser.add_argument("--target-root", type=Path, default=DEFAULT_TARGET_ROOT)
    parser.add_argument("--map", dest="map_ids", action="append", default=[])
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--deep-alignment", action="store_true")
    parser.add_argument(
        "--route-root",
        type=Path,
        help="optional maps root containing <id>/catalog/routes",
    )
    parser.add_argument("--json-output", type=Path)
    parser.add_argument("--markdown-output", type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        document = load_manifest(args.manifest)
        requested = set(args.map_ids)
        known = {entry["id"] for entry in document["maps"]}
        unknown = sorted(requested - known)
        if unknown:
            raise TownMapError(f"unknown map ids: {', '.join(unknown)}")
        selected = [
            entry for entry in document["maps"] if not requested or entry["id"] in requested
        ]
        contracts = document["source_contracts"]
        lanelet_root, lanelet_candidates = resolve_source_root(
            contracts["lanelet2"], "Town01/lanelet2_map.osm"
        )
        carla_root, carla_candidates = resolve_source_root(
            contracts["carla"], "CarlaUE4/Content/Carla/Maps/Town01.umap"
        )
        helper = _load_helper("setup_custom_full_map.py", "town_setup_custom_full_map")
        validator = _load_helper("validate_route_map.py", "town_validate_route_map")
        effective_deep_alignment = args.deep_alignment or args.prepare
        inspections = [
            inspect_entry(
                entry,
                lanelet_root,
                carla_root,
                args.generated_root,
                args.target_root,
                helper,
                effective_deep_alignment,
            )
            for entry in selected
        ]
        if args.prepare:
            if lanelet_root is None or carla_root is None:
                raise TownMapError("cannot prepare without resolved Lanelet2 and CARLA roots")
            by_id = {entry["id"]: entry for entry in inspections}
            for entry in selected:
                prepare_entry(
                    entry,
                    by_id[entry["id"]],
                    lanelet_root,
                    carla_root,
                    args.generated_root,
                    args.target_root,
                    helper,
                )
            inspections = [
                inspect_entry(
                    entry,
                    lanelet_root,
                    carla_root,
                    args.generated_root,
                    args.target_root,
                    helper,
                    effective_deep_alignment,
                )
                for entry in selected
            ]

        if args.route_root:
            for inspection in inspections:
                if inspection["status"] == "FULL_MAP_READY":
                    inspection["route_preflight"] = validate_catalog_routes(
                        inspection, args.route_root, validator
                    )
                    if inspection["route_preflight"]["status"] == "PASS":
                        inspection["status"] = "TEST_ROUTES_MAP_PREFLIGHT_PASS"
                        alignment = inspection.get("alignment")
                        if alignment and alignment["status"] == "PASS_WITH_OUTLIERS":
                            inspection["status"] += "_WITH_GLOBAL_PCD_OUTLIERS"
                    else:
                        inspection["status"] = "FAIL"
                        inspection["errors"].append("route/map catalog preflight failed")

        report = {
            "schema_version": 1,
            "suite_id": document["suite_id"],
            "scope": (
                "Packaged CARLA Town full-map assets and straight/turn route-map structural "
                "preflight; not an Autoware VAD execution claim."
            ),
            "source_roots": {
                "lanelet2": str(lanelet_root) if lanelet_root else None,
                "lanelet2_candidates_inspected": lanelet_candidates,
                "carla": str(carla_root) if carla_root else None,
                "carla_candidates_inspected": carla_candidates,
            },
            "maps": inspections,
        }
        report["summary"] = {
            "map_count": len(inspections),
            "test_routes_map_preflight_pass": sum(
                entry["status"].startswith("TEST_ROUTES_MAP_PREFLIGHT_PASS")
                for entry in inspections
            ),
            "full_map_ready": sum(entry["status"] == "FULL_MAP_READY" for entry in inspections),
            "ready_to_prepare": sum(
                entry["status"] == "READY_TO_PREPARE" for entry in inspections
            ),
            "blocked_missing_pointcloud": sum(
                entry["status"] == "BLOCKED_MISSING_POINTCLOUD" for entry in inspections
            ),
            "failed": sum(entry["status"] == "FAIL" for entry in inspections),
        }
        if report["summary"]["failed"]:
            report["summary"]["overall_status"] = "FAIL"
        elif report["summary"]["blocked_missing_pointcloud"]:
            report["summary"]["overall_status"] = "PARTIAL_BLOCKED"
        else:
            report["summary"]["overall_status"] = "PASS"
        json_text = json.dumps(report, indent=2, sort_keys=True) + "\n"
        markdown_text = render_markdown(report)
        if args.json_output:
            _atomic_text(args.json_output, json_text)
        if args.markdown_output:
            _atomic_text(args.markdown_output, markdown_text)
        print(json_text, end="")
        return 1 if report["summary"]["failed"] else 0
    except TownMapError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
