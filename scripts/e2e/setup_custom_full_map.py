#!/usr/bin/env python3

"""Inspect and create pinned Autoware bundles for local custom CARLA maps."""

from __future__ import annotations

import argparse
import copy
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import struct
import sys
import tempfile
from typing import Any
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST = Path(__file__).with_name("custom_map_bundles.yaml")
DEFAULT_TARGET_ROOT = ROOT / "data/maps"

ROOT_FIELDS = {"schema_version", "suite_id", "description", "profiles"}
PROFILE_FIELDS = {
    "display_name",
    "status",
    "canonical_carla_map",
    "target_name",
    "selection_reason",
    "projector",
    "carla_to_map_transform",
    "bundle_sources",
    "lanelet2_derivation",
    "expected",
    "reference_assets",
    "rejected_alternatives",
}
ASSET_FIELDS = {"path", "sha256", "size_bytes", "provenance"}
ALIGNMENT_FIELDS = {"kind", "x_m", "y_m", "z_m", "yaw_rad", "confidence", "source"}
EXPECTED_FIELDS = {"osm_nodes", "osm_ways", "osm_road_lanelets", "pcd_points"}
REJECTED_FIELDS = {"id", "paths", "reason"}
LANELET2_DERIVATION_FIELDS = {
    "schema_version",
    "kind",
    "relation_id",
    "expected_centerline_way_id",
    "boundary_way_clones",
    "expected_output_sha256",
    "expected_output_size_bytes",
    "provenance",
}
BOUNDARY_WAY_CLONE_FIELDS = {
    "role",
    "source_way_id",
    "derived_way_id",
    "expected_source_first_node_id",
    "expected_source_last_node_id",
}
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
OSM_ID_PATTERN = re.compile(r"[1-9][0-9]*")


class BundleError(RuntimeError):
    """Raised when a manifest, source asset, or target bundle is unsafe."""


def _require_mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise BundleError(f"{label} must be a mapping")
    return value


def _require_exact_fields(value: dict[str, Any], fields: set[str], label: str) -> None:
    actual = set(value)
    if actual != fields:
        missing = sorted(fields - actual)
        extra = sorted(actual - fields)
        raise BundleError(f"{label} fields are invalid; missing={missing}, extra={extra}")


def _require_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise BundleError(f"{label} must be a non-empty string")
    return value.strip()


def _require_finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BundleError(f"{label} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise BundleError(f"{label} must be finite")
    return result


def _validate_asset(asset: Any, label: str) -> None:
    asset = _require_mapping(asset, label)
    _require_exact_fields(asset, ASSET_FIELDS, label)
    _require_string(asset["path"], f"{label}.path")
    digest = _require_string(asset["sha256"], f"{label}.sha256")
    if not SHA256_PATTERN.fullmatch(digest):
        raise BundleError(f"{label}.sha256 must be 64 lowercase hexadecimal characters")
    size = asset["size_bytes"]
    if isinstance(size, bool) or not isinstance(size, int) or size <= 0:
        raise BundleError(f"{label}.size_bytes must be a positive integer")
    _require_string(asset["provenance"], f"{label}.provenance")


def _require_osm_id(value: Any, label: str) -> str:
    result = _require_string(value, label)
    if not OSM_ID_PATTERN.fullmatch(result):
        raise BundleError(f"{label} must be a positive decimal OSM id string")
    return result


def _validate_lanelet2_derivation(value: Any, label: str) -> None:
    if value is None:
        return
    derivation = _require_mapping(value, label)
    _require_exact_fields(derivation, LANELET2_DERIVATION_FIELDS, label)
    if derivation["schema_version"] != 1:
        raise BundleError(f"{label}.schema_version must be 1")
    if derivation["kind"] != "reverse_lanelet_boundary_ways":
        raise BundleError(
            f"{label}.kind must be 'reverse_lanelet_boundary_ways'"
        )
    _require_osm_id(derivation["relation_id"], f"{label}.relation_id")
    _require_osm_id(
        derivation["expected_centerline_way_id"],
        f"{label}.expected_centerline_way_id",
    )
    digest = _require_string(
        derivation["expected_output_sha256"],
        f"{label}.expected_output_sha256",
    )
    if not SHA256_PATTERN.fullmatch(digest):
        raise BundleError(
            f"{label}.expected_output_sha256 must be 64 lowercase hexadecimal characters"
        )
    size = derivation["expected_output_size_bytes"]
    if isinstance(size, bool) or not isinstance(size, int) or size <= 0:
        raise BundleError(f"{label}.expected_output_size_bytes must be positive")
    _require_string(derivation["provenance"], f"{label}.provenance")

    clones = derivation["boundary_way_clones"]
    if not isinstance(clones, list) or len(clones) != 2:
        raise BundleError(f"{label}.boundary_way_clones must contain left and right")
    roles: set[str] = set()
    source_ids: set[str] = set()
    derived_ids: set[str] = set()
    for index, raw_clone in enumerate(clones):
        clone_label = f"{label}.boundary_way_clones[{index}]"
        clone = _require_mapping(raw_clone, clone_label)
        _require_exact_fields(clone, BOUNDARY_WAY_CLONE_FIELDS, clone_label)
        role = _require_string(clone["role"], f"{clone_label}.role")
        if role not in {"left", "right"}:
            raise BundleError(f"{clone_label}.role must be left or right")
        if role in roles:
            raise BundleError(f"{label}.boundary_way_clones contains duplicate role {role}")
        roles.add(role)
        source_id = _require_osm_id(
            clone["source_way_id"], f"{clone_label}.source_way_id"
        )
        derived_id = _require_osm_id(
            clone["derived_way_id"], f"{clone_label}.derived_way_id"
        )
        _require_osm_id(
            clone["expected_source_first_node_id"],
            f"{clone_label}.expected_source_first_node_id",
        )
        _require_osm_id(
            clone["expected_source_last_node_id"],
            f"{clone_label}.expected_source_last_node_id",
        )
        if source_id == derived_id:
            raise BundleError(f"{clone_label} must use a fresh derived_way_id")
        if source_id in source_ids:
            raise BundleError(f"{label}.boundary_way_clones reuses source way {source_id}")
        if derived_id in derived_ids:
            raise BundleError(f"{label}.boundary_way_clones reuses derived way {derived_id}")
        source_ids.add(source_id)
        derived_ids.add(derived_id)
    if roles != {"left", "right"}:
        raise BundleError(f"{label}.boundary_way_clones must contain left and right")
    if source_ids & derived_ids:
        raise BundleError(
            f"{label}.boundary_way_clones source and derived ids must not overlap"
        )


def validate_manifest(document: Any) -> dict[str, Any]:
    document = _require_mapping(document, "manifest")
    _require_exact_fields(document, ROOT_FIELDS, "manifest")
    if document["schema_version"] != 1:
        raise BundleError("manifest.schema_version must be 1")
    _require_string(document["suite_id"], "manifest.suite_id")
    _require_string(document["description"], "manifest.description")

    profiles = _require_mapping(document["profiles"], "manifest.profiles")
    if not profiles:
        raise BundleError("manifest.profiles must not be empty")
    target_name_owners: dict[str, str] = {}
    for profile_id, raw_profile in profiles.items():
        _require_string(profile_id, "profile id")
        profile = _require_mapping(raw_profile, f"profiles.{profile_id}")
        _require_exact_fields(profile, PROFILE_FIELDS, f"profiles.{profile_id}")
        for field in (
            "display_name",
            "status",
            "canonical_carla_map",
            "target_name",
            "selection_reason",
        ):
            _require_string(profile[field], f"profiles.{profile_id}.{field}")
        if not profile["canonical_carla_map"].startswith("/Game/"):
            raise BundleError(
                f"profiles.{profile_id}.canonical_carla_map must start with /Game/"
            )
        target_name = profile["target_name"]
        if Path(target_name).name != target_name or target_name in {".", ".."}:
            raise BundleError(f"profiles.{profile_id}.target_name must be one directory name")
        existing_owner = target_name_owners.get(target_name)
        if existing_owner is not None:
            raise BundleError(
                f"profiles.{profile_id}.target_name duplicates "
                f"profiles.{existing_owner}.target_name: {target_name!r}; "
                "target_name values must be unique"
            )
        target_name_owners[target_name] = profile_id

        projector = _require_mapping(profile["projector"], f"profiles.{profile_id}.projector")
        _require_exact_fields(projector, {"projector_type"}, f"profiles.{profile_id}.projector")
        if projector["projector_type"] != "Local":
            raise BundleError(
                f"profiles.{profile_id}.projector.projector_type must be 'Local'"
            )

        alignment = _require_mapping(
            profile["carla_to_map_transform"],
            f"profiles.{profile_id}.carla_to_map_transform",
        )
        _require_exact_fields(
            alignment, ALIGNMENT_FIELDS, f"profiles.{profile_id}.carla_to_map_transform"
        )
        _require_string(
            alignment["kind"], f"profiles.{profile_id}.carla_to_map_transform.kind"
        )
        _require_string(
            alignment["confidence"],
            f"profiles.{profile_id}.carla_to_map_transform.confidence",
        )
        _require_string(
            alignment["source"], f"profiles.{profile_id}.carla_to_map_transform.source"
        )
        for field in ("x_m", "y_m", "z_m", "yaw_rad"):
            _require_finite(
                alignment[field], f"profiles.{profile_id}.carla_to_map_transform.{field}"
            )

        sources = _require_mapping(
            profile["bundle_sources"], f"profiles.{profile_id}.bundle_sources"
        )
        _require_exact_fields(
            sources,
            {"lanelet2_map", "pointcloud_map"},
            f"profiles.{profile_id}.bundle_sources",
        )
        for asset_id, asset in sources.items():
            _validate_asset(asset, f"profiles.{profile_id}.bundle_sources.{asset_id}")

        _validate_lanelet2_derivation(
            profile["lanelet2_derivation"],
            f"profiles.{profile_id}.lanelet2_derivation",
        )

        expected = _require_mapping(profile["expected"], f"profiles.{profile_id}.expected")
        _require_exact_fields(expected, EXPECTED_FIELDS, f"profiles.{profile_id}.expected")
        for field, value in expected.items():
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise BundleError(f"profiles.{profile_id}.expected.{field} must be positive")

        references = _require_mapping(
            profile["reference_assets"], f"profiles.{profile_id}.reference_assets"
        )
        for asset_id, asset in references.items():
            _require_string(asset_id, f"profiles.{profile_id}.reference asset id")
            _validate_asset(asset, f"profiles.{profile_id}.reference_assets.{asset_id}")

        rejected = profile["rejected_alternatives"]
        if not isinstance(rejected, list):
            raise BundleError(f"profiles.{profile_id}.rejected_alternatives must be a list")
        selected_paths = {asset["path"] for asset in sources.values()}
        for index, item in enumerate(rejected):
            label = f"profiles.{profile_id}.rejected_alternatives[{index}]"
            item = _require_mapping(item, label)
            _require_exact_fields(item, REJECTED_FIELDS, label)
            _require_string(item["id"], f"{label}.id")
            _require_string(item["reason"], f"{label}.reason")
            if not isinstance(item["paths"], list) or not item["paths"]:
                raise BundleError(f"{label}.paths must be a non-empty list")
            for path in item["paths"]:
                path = _require_string(path, f"{label}.paths[]")
                if path in selected_paths:
                    raise BundleError(f"{label} rejects a selected bundle source: {path}")
    return document


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(8 * 1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def load_manifest(path: Path | str = DEFAULT_MANIFEST) -> tuple[dict[str, Any], Path]:
    manifest_path = Path(path).expanduser().resolve()
    try:
        document = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise BundleError(f"cannot read manifest {manifest_path}: {error}") from error
    return validate_manifest(document), manifest_path


def _resolve_source(declared_path: str, manifest_path: Path) -> Path:
    path = Path(declared_path).expanduser()
    if not path.is_absolute():
        path = manifest_path.parent / path
    try:
        resolved = path.resolve(strict=True)
    except OSError as error:
        raise BundleError(f"source does not exist: {path} ({error})") from error
    if not resolved.is_file():
        raise BundleError(f"source is not a regular file: {resolved}")
    if not os.access(resolved, os.R_OK):
        raise BundleError(f"source is not readable: {resolved}")
    return resolved


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _inspect_lanelet2_root(root: ET.Element) -> dict[str, Any]:
    nodes = ways = road_lanelets = local_nodes = 0
    x_min = y_min = math.inf
    x_max = y_max = -math.inf
    for element in root:
        name = _local_name(element.tag)
        if name == "node":
            nodes += 1
            tags = {
                child.attrib.get("k"): child.attrib.get("v")
                for child in element
                if _local_name(child.tag) == "tag"
            }
            if "local_x" not in tags or "local_y" not in tags:
                continue
            try:
                x = float(tags["local_x"])
                y = float(tags["local_y"])
            except (TypeError, ValueError) as error:
                raise BundleError(f"Lanelet2 node has invalid local_x/local_y: {error}") from error
            if not (math.isfinite(x) and math.isfinite(y)):
                raise BundleError("Lanelet2 node has non-finite local_x/local_y")
            local_nodes += 1
            x_min, x_max = min(x_min, x), max(x_max, x)
            y_min, y_max = min(y_min, y), max(y_max, y)
        elif name == "way":
            ways += 1
        elif name == "relation":
            tags = {
                child.attrib.get("k"): child.attrib.get("v")
                for child in element
                if _local_name(child.tag) == "tag"
            }
            if tags.get("type") == "lanelet" and tags.get("subtype") == "road":
                road_lanelets += 1

    if nodes == 0 or ways == 0 or road_lanelets == 0:
        raise BundleError(
            f"Lanelet2 OSM has insufficient topology: nodes={nodes}, ways={ways}, "
            f"road_lanelets={road_lanelets}"
        )
    if local_nodes != nodes:
        raise BundleError(
            "Lanelet2 Local bundle requires local_x/local_y on every node; "
            f"found {local_nodes}/{nodes}"
        )
    return {
        "nodes": nodes,
        "ways": ways,
        "road_lanelets": road_lanelets,
        "local_nodes": local_nodes,
        "bounds_xy_m": {"x_min": x_min, "x_max": x_max, "y_min": y_min, "y_max": y_max},
    }


def inspect_lanelet2(path: Path) -> dict[str, Any]:
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as error:
        raise BundleError(f"invalid Lanelet2 OSM {path}: {error}") from error
    return _inspect_lanelet2_root(root)


def _indexed_osm_elements(root: ET.Element, kind: str) -> dict[str, ET.Element]:
    result: dict[str, ET.Element] = {}
    for element in root:
        if _local_name(element.tag) != kind:
            continue
        element_id = element.attrib.get("id")
        if not element_id:
            raise BundleError(f"Lanelet2 {kind} is missing an id")
        if element_id in result:
            raise BundleError(f"Lanelet2 contains duplicate {kind} id {element_id}")
        result[element_id] = element
    return result


def _way_node_refs(way: ET.Element, label: str) -> list[str]:
    refs = [
        child.attrib.get("ref", "")
        for child in way
        if _local_name(child.tag) == "nd"
    ]
    if len(refs) < 2 or any(not ref for ref in refs):
        raise BundleError(f"{label} must contain at least two valid node references")
    return refs


def _relation_way_member(
    relation: ET.Element, role: str, expected_ref: str, label: str
) -> ET.Element:
    members = [
        child
        for child in relation
        if _local_name(child.tag) == "member"
        and child.attrib.get("type") == "way"
        and child.attrib.get("role") == role
    ]
    if len(members) != 1:
        raise BundleError(f"{label} must contain exactly one {role} way member")
    if members[0].attrib.get("ref") != expected_ref:
        raise BundleError(
            f"{label} {role} way mismatch: expected {expected_ref}, "
            f"got {members[0].attrib.get('ref')!r}"
        )
    return members[0]


def derive_lanelet2_map(
    source_path: Path,
    derivation: dict[str, Any],
    *,
    source_sha256: str | None = None,
    verify_expected_output: bool = True,
) -> tuple[str, dict[str, Any], dict[str, Any]]:
    """Create one deterministic, provenance-rich Lanelet2 derivative in memory."""
    try:
        tree = ET.parse(source_path)
    except (OSError, ET.ParseError) as error:
        raise BundleError(f"invalid Lanelet2 OSM {source_path}: {error}") from error
    root = tree.getroot()
    nodes = _indexed_osm_elements(root, "node")
    ways = _indexed_osm_elements(root, "way")
    relations = _indexed_osm_elements(root, "relation")

    relation_id = derivation["relation_id"]
    relation = relations.get(relation_id)
    if relation is None:
        raise BundleError(f"Lanelet2 derivation relation {relation_id} does not exist")
    relation_tags = {
        child.attrib.get("k"): child.attrib.get("v")
        for child in relation
        if _local_name(child.tag) == "tag"
    }
    if relation_tags.get("type") != "lanelet" or relation_tags.get("subtype") != "road":
        raise BundleError(f"Lanelet2 derivation relation {relation_id} is not a road lanelet")
    _relation_way_member(
        relation,
        "centerline",
        derivation["expected_centerline_way_id"],
        f"relation {relation_id}",
    )

    derived_ids = {item["derived_way_id"] for item in derivation["boundary_way_clones"]}
    collisions = sorted(derived_ids & set(ways), key=int)
    if collisions:
        raise BundleError(
            "Lanelet2 derivation derived way id collision: " + ", ".join(collisions)
        )

    relation_usages: dict[str, list[dict[str, str]]] = {}
    for source_id in {item["source_way_id"] for item in derivation["boundary_way_clones"]}:
        relation_usages[source_id] = sorted(
            (
                {
                    "relation_id": candidate.attrib["id"],
                    "role": member.attrib.get("role", ""),
                }
                for candidate in relations.values()
                for member in candidate
                if _local_name(member.tag) == "member"
                and member.attrib.get("type") == "way"
                and member.attrib.get("ref") == source_id
            ),
            key=lambda item: (int(item["relation_id"]), item["role"]),
        )

    clones: list[ET.Element] = []
    clone_records: list[dict[str, Any]] = []
    for clone_spec in derivation["boundary_way_clones"]:
        role = clone_spec["role"]
        source_id = clone_spec["source_way_id"]
        derived_id = clone_spec["derived_way_id"]
        source_way = ways.get(source_id)
        if source_way is None:
            raise BundleError(f"Lanelet2 derivation source way {source_id} does not exist")
        source_refs = _way_node_refs(source_way, f"source way {source_id}")
        missing_nodes = sorted(set(source_refs) - set(nodes), key=int)
        if missing_nodes:
            raise BundleError(
                f"source way {source_id} references missing nodes: {missing_nodes}"
            )
        if source_refs[0] != clone_spec["expected_source_first_node_id"]:
            raise BundleError(
                f"source way {source_id} first node mismatch: expected "
                f"{clone_spec['expected_source_first_node_id']}, got {source_refs[0]}"
            )
        if source_refs[-1] != clone_spec["expected_source_last_node_id"]:
            raise BundleError(
                f"source way {source_id} last node mismatch: expected "
                f"{clone_spec['expected_source_last_node_id']}, got {source_refs[-1]}"
            )
        target_member = _relation_way_member(
            relation, role, source_id, f"relation {relation_id}"
        )

        clone = copy.deepcopy(source_way)
        clone.attrib["id"] = derived_id
        clone_nds = [child for child in clone if _local_name(child.tag) == "nd"]
        for child, node_ref in zip(clone_nds, reversed(source_refs)):
            child.attrib["ref"] = node_ref
        derived_refs = _way_node_refs(clone, f"derived way {derived_id}")
        if derived_refs != list(reversed(source_refs)):
            raise BundleError(f"internal error reversing source way {source_id}")
        target_member.attrib["ref"] = derived_id
        clones.append(clone)
        clone_records.append(
            {
                **clone_spec,
                "source_node_count": len(source_refs),
                "derived_first_node_id": derived_refs[0],
                "derived_last_node_id": derived_refs[-1],
                "source_relation_usages": relation_usages[source_id],
            }
        )

    first_relation_index = next(
        (index for index, element in enumerate(root) if _local_name(element.tag) == "relation"),
        len(root),
    )
    for offset, clone in enumerate(clones):
        root.insert(first_relation_index + offset, clone)

    ET.indent(tree, space="  ")
    buffer = io.BytesIO()
    tree.write(buffer, encoding="utf-8", xml_declaration=True, short_empty_elements=True)
    output_bytes = buffer.getvalue()
    if not output_bytes.endswith(b"\n"):
        output_bytes += b"\n"
    content = output_bytes.decode("utf-8")
    output_sha256 = hashlib.sha256(output_bytes).hexdigest()
    output_size = len(output_bytes)
    if verify_expected_output:
        if output_sha256 != derivation["expected_output_sha256"]:
            raise BundleError(
                "Lanelet2 derived output SHA-256 mismatch: expected "
                f"{derivation['expected_output_sha256']}, got {output_sha256}. "
                "Review the derivation and pin the output only after topology validation."
            )
        if output_size != derivation["expected_output_size_bytes"]:
            raise BundleError(
                "Lanelet2 derived output size mismatch: expected "
                f"{derivation['expected_output_size_bytes']}, got {output_size}"
            )

    derived_root = ET.fromstring(output_bytes)
    derived_inspection = _inspect_lanelet2_root(derived_root)
    provenance = {
        "schema_version": derivation["schema_version"],
        "kind": derivation["kind"],
        "provenance": derivation["provenance"],
        "source_path": str(source_path),
        "source_sha256": source_sha256 or sha256_file(source_path),
        "relation_id": relation_id,
        "expected_centerline_way_id": derivation["expected_centerline_way_id"],
        "boundary_way_clones": clone_records,
        "output_sha256": output_sha256,
        "output_size_bytes": output_size,
        "polygon_geometry_preserved": True,
    }
    return content, provenance, derived_inspection


def inspect_pcd(path: Path) -> dict[str, Any]:
    header: dict[str, str] = {}
    data_offset = 0
    try:
        with path.open("rb") as stream:
            while stream.tell() < 65_536:
                raw_line = stream.readline()
                if not raw_line:
                    break
                try:
                    line = raw_line.decode("ascii").strip()
                except UnicodeDecodeError as error:
                    raise BundleError("PCD binary payload starts before a DATA header") from error
                if not line or line.startswith("#"):
                    continue
                parts = line.split(None, 1)
                key = parts[0].upper()
                if key in header:
                    raise BundleError(f"PCD header contains duplicate {key}")
                header[key] = parts[1].strip() if len(parts) == 2 else ""
                if key == "DATA":
                    data_offset = stream.tell()
                    break
    except OSError as error:
        raise BundleError(f"cannot inspect PCD {path}: {error}") from error

    required = {"VERSION", "FIELDS", "SIZE", "TYPE", "COUNT", "WIDTH", "HEIGHT", "POINTS", "DATA"}
    missing = sorted(required - set(header))
    if missing:
        raise BundleError(f"PCD header is missing: {', '.join(missing)}")
    fields = header["FIELDS"].split()
    if not {"x", "y", "z"}.issubset(fields):
        raise BundleError(f"PCD FIELDS must contain x y z, got {fields}")
    try:
        sizes = [int(value) for value in header["SIZE"].split()]
        counts = [int(value) for value in header["COUNT"].split()]
        width = int(header["WIDTH"])
        height = int(header["HEIGHT"])
        points = int(header["POINTS"])
    except ValueError as error:
        raise BundleError(f"PCD header has an invalid numeric value: {error}") from error
    types = header["TYPE"].split()
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise BundleError("PCD FIELDS/SIZE/TYPE/COUNT column counts disagree")
    if any(value <= 0 for value in sizes + counts):
        raise BundleError("PCD SIZE and COUNT values must be positive")
    if any(value not in {"F", "I", "U"} for value in types):
        raise BundleError(f"PCD TYPE values are invalid: {types}")
    if width <= 0 or height <= 0 or points <= 0 or width * height != points:
        raise BundleError(
            f"PCD dimensions are invalid: width={width}, height={height}, points={points}"
        )
    encoding = header["DATA"].lower()
    if encoding not in {"ascii", "binary", "binary_compressed"}:
        raise BundleError(f"unsupported PCD DATA encoding: {header['DATA']!r}")
    file_size = path.stat().st_size
    if data_offset == 0 or file_size <= data_offset:
        raise BundleError("PCD has no point payload")
    point_stride = sum(size * count for size, count in zip(sizes, counts))
    if encoding == "binary" and file_size < data_offset + point_stride * points:
        raise BundleError(
            f"binary PCD is truncated: {file_size} bytes; expected at least "
            f"{data_offset + point_stride * points}"
        )
    if encoding == "binary_compressed" and file_size >= data_offset + 8:
        with path.open("rb") as stream:
            stream.seek(data_offset)
            compressed_size, uncompressed_size = struct.unpack("<II", stream.read(8))
        if compressed_size <= 0 or uncompressed_size != point_stride * points:
            raise BundleError("binary_compressed PCD size metadata is inconsistent")
    return {"points": points, "encoding": encoding, "fields": fields, "point_stride": point_stride}


def _inspect_asset(
    asset_id: str, asset: dict[str, Any], manifest_path: Path, category: str
) -> dict[str, Any]:
    path = _resolve_source(asset["path"], manifest_path)
    size = path.stat().st_size
    if size != asset["size_bytes"]:
        raise BundleError(
            f"{category}.{asset_id} size mismatch for {path}: "
            f"expected {asset['size_bytes']}, got {size}"
        )
    digest = sha256_file(path)
    if digest != asset["sha256"]:
        raise BundleError(
            f"{category}.{asset_id} SHA-256 mismatch for {path}: "
            f"expected {asset['sha256']}, got {digest}. "
            "Do not silently mix map generations; update the manifest only after revalidation."
        )
    return {
        "declared_path": asset["path"],
        "resolved_path": str(path),
        "sha256": digest,
        "size_bytes": size,
        "provenance": asset["provenance"],
    }


def alignment_warnings(alignment: dict[str, Any]) -> list[str]:
    warnings = []
    confidence = alignment["confidence"]
    if "unvalidated" in confidence or "placeholder" in alignment["kind"]:
        warnings.append(
            "reference-only alignment contains an unvalidated placeholder; "
            "do not use it for closed-loop CARLA driving or LiDAR localization"
        )
    if "empirical" in confidence or "single_anchor" in confidence:
        warnings.append(
            f"runtime alignment confidence is {confidence}; validate it with multiple 3D anchors"
        )
    if abs(float(alignment["z_m"])) > 1.0e-9:
        warnings.append(
            f"3D preflight required: empirical z_m={float(alignment['z_m']):.3f}; "
            "compare CARLA pose/route Z with Lanelet elevation and the PCD before closed loop"
        )
    return warnings


def inspect_profile(
    document: dict[str, Any],
    profile_id: str,
    manifest_path: Path,
    *,
    inspect_references: bool = True,
) -> dict[str, Any]:
    try:
        profile = document["profiles"][profile_id]
    except KeyError as error:
        choices = ", ".join(sorted(document["profiles"]))
        raise BundleError(f"unknown profile {profile_id!r}; choose one of: {choices}") from error

    bundle_assets = {
        asset_id: _inspect_asset(asset_id, asset, manifest_path, "bundle_sources")
        for asset_id, asset in profile["bundle_sources"].items()
    }
    reference_assets = (
        {
            asset_id: _inspect_asset(asset_id, asset, manifest_path, "reference_assets")
            for asset_id, asset in profile["reference_assets"].items()
        }
        if inspect_references
        else {}
    )

    lanelet_source_path = Path(bundle_assets["lanelet2_map"]["resolved_path"])
    lanelet_source = inspect_lanelet2(lanelet_source_path)
    derivation_spec = profile["lanelet2_derivation"]
    lanelet_derivation = None
    if derivation_spec is None:
        lanelet = lanelet_source
    else:
        _, derivation_result, lanelet = derive_lanelet2_map(
            lanelet_source_path,
            derivation_spec,
            source_sha256=bundle_assets["lanelet2_map"]["sha256"],
        )
        lanelet_derivation = {
            "config": copy.deepcopy(derivation_spec),
            "result": derivation_result,
        }
    pcd = inspect_pcd(Path(bundle_assets["pointcloud_map"]["resolved_path"]))
    expected = profile["expected"]
    actual_expected = {
        "osm_nodes": lanelet["nodes"],
        "osm_ways": lanelet["ways"],
        "osm_road_lanelets": lanelet["road_lanelets"],
        "pcd_points": pcd["points"],
    }
    differences = {
        key: (expected[key], actual)
        for key, actual in actual_expected.items()
        if expected[key] != actual
    }
    if differences:
        detail = ", ".join(
            f"{key}=expected {wanted}, got {actual}"
            for key, (wanted, actual) in differences.items()
        )
        raise BundleError(f"profile {profile_id} structural expectation mismatch: {detail}")

    opendrive_hashes = {
        details["sha256"]
        for asset_id, details in reference_assets.items()
        if asset_id.endswith("opendrive")
    }
    if len(opendrive_hashes) > 1:
        raise BundleError(
            f"profile {profile_id} RoadRunner/source/runtime OpenDRIVE hashes disagree: "
            f"{sorted(opendrive_hashes)}"
        )

    warnings = alignment_warnings(profile["carla_to_map_transform"])
    if not inspect_references and profile["reference_assets"]:
        warnings.append(
            "authoring/provenance reference assets were not inspected; "
            "bundle source hashes and structure were still validated"
        )

    return {
        "profile_id": profile_id,
        "display_name": profile["display_name"],
        "status": profile["status"],
        "canonical_carla_map": profile["canonical_carla_map"],
        "target_name": profile["target_name"],
        "selection_reason": profile["selection_reason"],
        "projector": dict(profile["projector"]),
        "carla_to_map_transform": dict(profile["carla_to_map_transform"]),
        "bundle_sources": bundle_assets,
        "lanelet2_derivation": lanelet_derivation,
        "reference_assets": reference_assets,
        "rejected_alternatives": profile["rejected_alternatives"],
        "lanelet2": lanelet,
        "lanelet2_source": lanelet_source,
        "pcd": pcd,
        "warnings": warnings,
    }


def _metadata_content(inspection: dict[str, Any], manifest_path: Path) -> str:
    alignment = inspection["carla_to_map_transform"]
    payload = {
        "schema_version": 1,
        "profile": inspection["profile_id"],
        "display_name": inspection["display_name"],
        "status": inspection["status"],
        "canonical_carla_map": inspection["canonical_carla_map"],
        "projector_type": inspection["projector"]["projector_type"],
        "carla_to_map_transform": {
            key: alignment[key] for key in ("x_m", "y_m", "z_m", "yaw_rad")
        },
        "alignment_provenance": {
            "kind": alignment["kind"],
            "confidence": alignment["confidence"],
            "source": alignment["source"],
            "applies_to": "CARLA poses and routes before publication in the Autoware map frame; not baked into OSM or PCD",
        },
        "bundle_sources": inspection["bundle_sources"],
        "lanelet2_derivation": inspection["lanelet2_derivation"],
        "reference_assets": inspection["reference_assets"],
        "rejected_alternatives": inspection["rejected_alternatives"],
        "structural_inspection": {
            "lanelet2": inspection["lanelet2"],
            "lanelet2_source": inspection["lanelet2_source"],
            "pcd": inspection["pcd"],
        },
        "warnings": inspection["warnings"],
        "manifest": {
            "path": str(manifest_path),
            "sha256": sha256_file(manifest_path),
        },
    }
    return json.dumps(payload, indent=2, sort_keys=True) + "\n"


def _absolute_without_resolving(path: Path) -> Path:
    return Path(os.path.abspath(os.fspath(path.expanduser())))


def _symlink_action(target: Path, source: Path) -> str:
    if target.is_symlink():
        current = Path(os.path.realpath(target))
        return "keep" if current == source else "update"
    if os.path.lexists(target):
        raise BundleError(
            f"refusing to replace existing non-symlink map asset: {target}. "
            "Move it aside or select a different --target-dir."
        )
    return "create"


def _generated_action(
    target: Path, content: str, refresh_generated: bool
) -> str:
    if target.is_symlink():
        raise BundleError(f"refusing to replace generated-file symlink: {target}")
    if not os.path.lexists(target):
        return "create"
    if not target.is_file():
        raise BundleError(f"generated target exists but is not a regular file: {target}")
    try:
        existing = target.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise BundleError(f"cannot inspect existing generated file {target}: {error}") from error
    if existing == content:
        return "keep"
    if refresh_generated:
        return "update"
    raise BundleError(
        f"refusing to replace generated file with different content: {target}. "
        "Review the difference, then rerun with --refresh-generated."
    )


def _derived_lanelet_action(
    target: Path,
    content: str,
    source: Path,
    refresh_generated: bool,
) -> str:
    if target.is_symlink():
        current = Path(os.path.realpath(target))
        if current != source:
            raise BundleError(
                f"refusing to replace lanelet2_map.osm symlink to an unpinned source: "
                f"{target} -> {current}"
            )
        if not refresh_generated:
            raise BundleError(
                f"lanelet2_map.osm must migrate from the pinned source symlink to a "
                f"derived regular file: {target}. Review the derivation, then rerun with "
                "--refresh-generated."
            )
        return "update"
    return _generated_action(target, content, refresh_generated)


def _atomic_symlink(source: Path, target: Path) -> None:
    temporary = target.parent / f".{target.name}.tmp-{os.getpid()}"
    try:
        if os.path.lexists(temporary):
            temporary.unlink()
        temporary.symlink_to(source)
        os.replace(temporary, target)
    finally:
        if os.path.lexists(temporary):
            temporary.unlink()


def _atomic_text(target: Path, content: str) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{target.name}.tmp-", dir=target.parent, text=True
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.chmod(0o644)
        os.replace(temporary, target)
    finally:
        if os.path.lexists(temporary):
            temporary.unlink()


def prepare_bundle(
    inspection: dict[str, Any],
    manifest_path: Path,
    *,
    target_root: Path = DEFAULT_TARGET_ROOT,
    target_dir: Path | None = None,
    dry_run: bool = False,
    refresh_generated: bool = False,
) -> dict[str, Any]:
    if target_dir is None:
        target = _absolute_without_resolving(Path(target_root) / inspection["target_name"])
    else:
        target = _absolute_without_resolving(Path(target_dir))
    if target in {Path("/"), ROOT}:
        raise BundleError(f"unsafe target directory: {target}")
    if target.is_symlink():
        raise BundleError(f"refusing to use a symlink as the bundle directory: {target}")
    if os.path.lexists(target) and not target.is_dir():
        raise BundleError(f"bundle target exists but is not a directory: {target}")

    lanelet_target = target / "lanelet2_map.osm"
    pointcloud_target = target / "pointcloud_map.pcd"
    projector_target = target / "map_projector_info.yaml"
    metadata_target = target / "map_bundle.json"
    lanelet_source = Path(inspection["bundle_sources"]["lanelet2_map"]["resolved_path"])
    pointcloud_source = Path(inspection["bundle_sources"]["pointcloud_map"]["resolved_path"])
    lanelet_derivation = inspection["lanelet2_derivation"]
    lanelet_content = None
    if lanelet_derivation is not None:
        lanelet_content, regenerated_result, regenerated_inspection = derive_lanelet2_map(
            lanelet_source,
            lanelet_derivation["config"],
            source_sha256=inspection["bundle_sources"]["lanelet2_map"]["sha256"],
        )
        if regenerated_result != lanelet_derivation["result"]:
            raise BundleError("Lanelet2 derivation provenance changed after inspection")
        if regenerated_inspection != inspection["lanelet2"]:
            raise BundleError("Lanelet2 derived topology changed after inspection")
    projector_content = "projector_type: Local\n"
    metadata_content = _metadata_content(inspection, manifest_path)

    actions = {
        str(lanelet_target): (
            _symlink_action(lanelet_target, lanelet_source)
            if lanelet_content is None
            else _derived_lanelet_action(
                lanelet_target,
                lanelet_content,
                lanelet_source,
                refresh_generated,
            )
        ),
        str(pointcloud_target): _symlink_action(pointcloud_target, pointcloud_source),
        str(projector_target): _generated_action(
            projector_target, projector_content, refresh_generated
        ),
        str(metadata_target): _generated_action(
            metadata_target, metadata_content, refresh_generated
        ),
    }
    if not dry_run:
        target.mkdir(parents=True, exist_ok=True)
        if actions[str(lanelet_target)] != "keep":
            if lanelet_content is None:
                _atomic_symlink(lanelet_source, lanelet_target)
            else:
                _atomic_text(lanelet_target, lanelet_content)
        if actions[str(pointcloud_target)] != "keep":
            _atomic_symlink(pointcloud_source, pointcloud_target)
        if actions[str(projector_target)] != "keep":
            _atomic_text(projector_target, projector_content)
        if actions[str(metadata_target)] != "keep":
            _atomic_text(metadata_target, metadata_content)

        if lanelet_content is None:
            if lanelet_target.resolve(strict=True) != lanelet_source:
                raise BundleError("post-write verification failed for lanelet2_map.osm")
        else:
            if lanelet_target.is_symlink() or not lanelet_target.is_file():
                raise BundleError("derived lanelet2_map.osm is not a regular file")
            if lanelet_target.read_text(encoding="utf-8") != lanelet_content:
                raise BundleError("post-write verification failed for derived lanelet2_map.osm")
        if pointcloud_target.resolve(strict=True) != pointcloud_source:
            raise BundleError("post-write verification failed for pointcloud_map.pcd")
        if projector_target.read_text(encoding="utf-8") != projector_content:
            raise BundleError("post-write verification failed for map_projector_info.yaml")
        if metadata_target.read_text(encoding="utf-8") != metadata_content:
            raise BundleError("post-write verification failed for map_bundle.json")
    return {"target_dir": str(target), "dry_run": dry_run, "actions": actions}


def _human_bytes(value: int) -> str:
    units = ("B", "KiB", "MiB", "GiB", "TiB")
    number = float(value)
    for unit in units:
        if abs(number) < 1024.0 or unit == units[-1]:
            return f"{number:.1f} {unit}"
        number /= 1024.0
    raise AssertionError("unreachable")


def print_inspection(inspection: dict[str, Any]) -> None:
    print(f"Profile: {inspection['profile_id']} ({inspection['display_name']})")
    print(f"  CARLA map: {inspection['canonical_carla_map']}")
    for asset_id, asset in inspection["bundle_sources"].items():
        print(
            f"  [ok] {asset_id}: {asset['resolved_path']} "
            f"({_human_bytes(asset['size_bytes'])}, sha256={asset['sha256']})"
        )
    print(
        "  [ok] Lanelet2: "
        f"nodes={inspection['lanelet2']['nodes']}, ways={inspection['lanelet2']['ways']}, "
        f"road_lanelets={inspection['lanelet2']['road_lanelets']}"
    )
    if inspection["lanelet2_derivation"] is not None:
        result = inspection["lanelet2_derivation"]["result"]
        print(
            "  [ok] Lanelet2 derivative: "
            f"{result['kind']} relation={result['relation_id']}, "
            f"sha256={result['output_sha256']}"
        )
    print(
        "  [ok] PCD: "
        f"points={inspection['pcd']['points']}, encoding={inspection['pcd']['encoding']}"
    )
    print(f"  [ok] provenance references: {len(inspection['reference_assets'])}")
    alignment = inspection["carla_to_map_transform"]
    print(
        "  Runtime CARLA -> map alignment: "
        f"x={alignment['x_m']:.6f} m, y={alignment['y_m']:.6f} m, "
        f"z={alignment['z_m']:.6f} m, yaw={alignment['yaw_rad']:.9f} rad"
    )
    for warning in inspection["warnings"]:
        print(f"  WARNING: {warning}")


def apply_bundle_source_overrides(
    document: dict[str, Any], profile_id: str, values: list[str]
) -> dict[str, Any]:
    """Override only source locations while retaining every pinned asset contract."""
    if not values:
        return document
    try:
        profile = document["profiles"][profile_id]
    except KeyError as error:
        choices = ", ".join(sorted(document["profiles"]))
        raise BundleError(
            f"unknown profile {profile_id!r}; choose one of: {choices}"
        ) from error

    result = copy.deepcopy(document)
    sources = result["profiles"][profile_id]["bundle_sources"]
    seen: set[str] = set()
    for value in values:
        asset_id, separator, path = value.partition("=")
        if not separator or not asset_id or not path:
            raise BundleError(
                "--source must use ASSET_ID=PATH, for example "
                "pointcloud_map=/data/maps/custom.pcd"
            )
        if asset_id not in sources:
            choices = ", ".join(sorted(sources))
            raise BundleError(
                f"unknown bundle source {asset_id!r}; choose one of: {choices}"
            )
        if asset_id in seen:
            raise BundleError(f"duplicate --source override for {asset_id}")
        seen.add(asset_id)
        sources[asset_id]["path"] = path
    return result


def add_inspection_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--source",
        action="append",
        default=[],
        metavar="ASSET_ID=PATH",
        help=(
            "Override a bundle source location without changing its pinned hash, size, "
            "or structure; may be repeated"
        ),
    )
    parser.add_argument(
        "--skip-reference-assets",
        action="store_true",
        help=(
            "Validate runtime Lanelet2/PCD sources but skip optional authoring/provenance "
            "files that are not required to create the runtime bundle"
        ),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("list", help="List configured profiles without reading large assets")

    inspect_parser = subparsers.add_parser("inspect", help="Validate every pinned source")
    inspect_parser.add_argument("profile")
    add_inspection_options(inspect_parser)
    inspect_parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")

    setup_parser = subparsers.add_parser("setup", help="Validate and create a map bundle")
    setup_parser.add_argument("profile")
    add_inspection_options(setup_parser)
    target_group = setup_parser.add_mutually_exclusive_group()
    target_group.add_argument("--target-root", type=Path, default=DEFAULT_TARGET_ROOT)
    target_group.add_argument("--target-dir", type=Path)
    setup_parser.add_argument("--dry-run", action="store_true")
    setup_parser.add_argument(
        "--refresh-generated",
        action="store_true",
        help="Replace changed projector/metadata files after explicit review; never replaces regular map assets",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        document, manifest_path = load_manifest(args.manifest)
        if args.command == "list":
            for profile_id, profile in document["profiles"].items():
                alignment = profile["carla_to_map_transform"]
                print(
                    f"{profile_id}: status={profile['status']}, target={profile['target_name']}, "
                    f"alignment=({alignment['x_m']}, {alignment['y_m']}, "
                    f"{alignment['z_m']}, {alignment['yaw_rad']})"
                )
            return 0

        document = apply_bundle_source_overrides(document, args.profile, args.source)
        inspection = inspect_profile(
            document,
            args.profile,
            manifest_path,
            inspect_references=not args.skip_reference_assets,
        )
        if args.command == "inspect":
            if args.json:
                print(json.dumps(inspection, indent=2, sort_keys=True))
            else:
                print_inspection(inspection)
            return 0

        print_inspection(inspection)
        result = prepare_bundle(
            inspection,
            manifest_path,
            target_root=args.target_root,
            target_dir=args.target_dir,
            dry_run=args.dry_run,
            refresh_generated=args.refresh_generated,
        )
        print("  Planned bundle changes:" if args.dry_run else "  Applied bundle changes:")
        for path, action in result["actions"].items():
            print(f"    {action}: {path}")
        if args.dry_run:
            print(f"Dry run complete; no files were written under {result['target_dir']}")
        else:
            print(f"Custom full-map bundle is ready: {result['target_dir']}")
        return 0
    except BundleError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
