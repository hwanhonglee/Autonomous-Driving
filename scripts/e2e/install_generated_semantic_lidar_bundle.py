#!/usr/bin/env python3
"""Install a QA-passed semantic-LiDAR collection as a Town full-map bundle."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TARGET = ROOT / "data/maps/Town10HD_Opt_full"
DEFAULT_CANONICAL_MAP = "/Game/Carla/Maps/Town10HD_Opt"
DEFAULT_OPENDRIVE_SHA256 = (
    "5d883b799f634030af92be1e9d79d107845540ba04338e8c60e095be1aef7be7"
)
DEFAULT_LANELET_SHA256 = (
    "cdd0a28664bcf9b5bd8867584a6fa6a02fa0f75a7c3dbb901ca216128ef86a17"
)


class InstallationError(RuntimeError):
    """Raised when collection evidence is insufficient or a target is unsafe."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while block := stream.read(8 * 1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def read_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise InstallationError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(value, dict):
        raise InstallationError(f"{label} must be a JSON object: {path}")
    return value


def _load_helper(filename: str, module_name: str) -> Any:
    path = Path(__file__).with_name(filename)
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise InstallationError(f"cannot load helper module {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(module_name, None)
        raise
    return module


def _safe_collection_file(root: Path, value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise InstallationError(f"{label} path is missing")
    path = Path(value).expanduser().resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise InstallationError(f"{label} escapes collection root: {path}") from error
    if not path.is_file():
        raise InstallationError(f"{label} does not exist: {path}")
    return path


def _require_digest(value: Any, label: str) -> str:
    digest = str(value)
    if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
        raise InstallationError(f"{label} is not a lowercase SHA-256 digest")
    return digest


def validate_collection(
    collection_root: Path,
    lanelet_path: Path,
    canonical_map: str,
    opendrive_sha256: str,
    lanelet_sha256: str,
) -> dict[str, Any]:
    provenance_path = collection_root / "provenance.json"
    provenance = read_object(provenance_path, "semantic-LiDAR provenance")
    if provenance.get("schema_version") != 1 or provenance.get("status") != "COMPLETE_QA_PASS":
        raise InstallationError("semantic-LiDAR provenance is not COMPLETE_QA_PASS")
    qa = provenance.get("qa")
    if not isinstance(qa, dict) or qa.get("status") != "PASS":
        raise InstallationError("semantic-LiDAR global QA did not pass")
    lanelet_qa = qa.get("lanelet2")
    route_qa = qa.get("routes")
    if not isinstance(lanelet_qa, dict) or lanelet_qa.get("status") != "PASS":
        raise InstallationError("semantic-LiDAR Lanelet QA did not pass")
    if not isinstance(route_qa, dict) or route_qa.get("status") != "PASS":
        raise InstallationError("semantic-LiDAR route QA did not pass")
    scenarios = route_qa.get("scenario_status")
    if (
        not isinstance(scenarios, dict)
        or scenarios.get("straight") != "PASS"
        or not any(scenarios.get(name) == "PASS" for name in ("left", "right"))
    ):
        raise InstallationError("route QA lacks a passing straight and turn route")

    scan_plan_record = provenance.get("scan_plan")
    pointcloud_record = provenance.get("pointcloud")
    checkpoint_record = provenance.get("checkpoint")
    if not all(
        isinstance(value, dict)
        for value in (scan_plan_record, pointcloud_record, checkpoint_record)
    ):
        raise InstallationError("provenance is missing scan, checkpoint, or pointcloud records")
    scan_plan_path = _safe_collection_file(
        collection_root, scan_plan_record.get("path"), "scan plan"
    )
    pointcloud_path = _safe_collection_file(
        collection_root, pointcloud_record.get("path"), "pointcloud"
    )
    checkpoint_path = _safe_collection_file(
        collection_root, checkpoint_record.get("path"), "checkpoint"
    )
    for path, record, label in (
        (scan_plan_path, scan_plan_record, "scan plan"),
        (pointcloud_path, pointcloud_record, "pointcloud"),
        (checkpoint_path, checkpoint_record, "checkpoint"),
    ):
        expected = _require_digest(record.get("sha256"), f"{label}.sha256")
        if sha256_file(path) != expected:
            raise InstallationError(f"{label} differs from its provenance hash")

    plan = read_object(scan_plan_path, "scan plan")
    generator = _load_helper(
        "generate_carla_semantic_lidar_map.py", "semantic_lidar_bundle_generator"
    )
    if plan.get("fingerprint") != generator.plan_fingerprint(plan):
        raise InstallationError("scan plan fingerprint is corrupt")
    try:
        generator.verify_pinned_inputs(plan)
    except generator.MapGenerationError as error:
        raise InstallationError(f"pinned collection input verification failed: {error}") from error
    semantic_contract = plan.get("semantic_contract")
    if (
        not isinstance(semantic_contract, dict)
        or semantic_contract.get("carla_0915_city_object_labels")
        != generator.EXPECTED_CITY_OBJECT_LABELS
    ):
        raise InstallationError("scan plan does not pin the audited CARLA 0.9.15 label table")
    implementation = plan.get("implementation")
    if not isinstance(implementation, dict):
        raise InstallationError("scan plan has no implementation pins")
    generator_snapshot = _safe_collection_file(
        collection_root,
        implementation.get("generator_snapshot_path"),
        "generator implementation snapshot",
    )
    validator_snapshot = _safe_collection_file(
        collection_root,
        implementation.get("validator_snapshot_path"),
        "route validator implementation snapshot",
    )
    for path, key, label in (
        (generator_snapshot, "generator_sha256", "generator"),
        (validator_snapshot, "validator_sha256", "route validator"),
    ):
        if sha256_file(path) != implementation.get(key):
            raise InstallationError(f"{label} snapshot differs from its plan pin")
    if sha256_file(Path(generator.__file__).resolve()) != implementation.get(
        "generator_sha256"
    ):
        raise InstallationError(
            "current generator differs from the implementation that collected this PCD"
        )
    cooked = plan.get("cooked_assets")
    if not isinstance(cooked, dict):
        raise InstallationError("scan plan has no cooked-asset inventory pin")
    cooked_inventory_path = _safe_collection_file(
        collection_root, cooked.get("inventory_path"), "cooked asset inventory"
    )
    if sha256_file(cooked_inventory_path) != cooked.get("inventory_file_sha256"):
        raise InstallationError("cooked asset inventory artifact differs from its plan pin")
    current_cooked = generator.cooked_asset_inventory(
        Path(cooked["content_root"]), str(cooked["token"])
    )
    if current_cooked["inventory_sha256"] != cooked.get("inventory_sha256"):
        raise InstallationError("Town-named cooked assets changed after collection")
    map_record = plan.get("map")
    coordinate = plan.get("coordinate_contract")
    semantic = plan.get("semantic_contract")
    if not isinstance(map_record, dict) or map_record.get("expected_name") != canonical_map:
        raise InstallationError("scan plan canonical map differs from installation target")
    if map_record.get("opendrive_sha256") != opendrive_sha256:
        raise InstallationError("scan plan OpenDRIVE hash differs from the pinned runtime")
    if (
        not isinstance(coordinate, dict)
        or coordinate.get("carla_world_to_ros") != "(x, y, z) -> (x, -y, z)"
    ):
        raise InstallationError("scan plan does not prove the ROS Y-reflection contract")
    if (
        not isinstance(semantic, dict)
        or semantic.get("source_geometry")
        != "CARLA semantic-LiDAR ray hits; never Lanelet-synthesized"
    ):
        raise InstallationError("scan plan does not prove semantic-LiDAR source geometry")

    checkpoint = read_object(checkpoint_path, "checkpoint")
    poses = plan.get("scan_plan", {}).get("poses")
    completed = checkpoint.get("completed_scans")
    if (
        checkpoint.get("status") != "COLLECTED"
        or not isinstance(poses, list)
        or not isinstance(completed, list)
        or len(completed) != len(poses)
        or int(checkpoint_record.get("completed_scans", -1)) != len(poses)
    ):
        raise InstallationError("checkpoint does not cover every planned scan pose")
    validity = checkpoint.get("collection_validity")
    sessions = checkpoint.get("sessions")
    baseline_signature = _require_digest(
        checkpoint.get("baseline_actor_type_multiset_sha256"),
        "checkpoint.baseline_actor_type_multiset_sha256",
    )
    if (
        not isinstance(validity, dict)
        or validity.get("status") != "PASS"
        or validity.get("every_scan_has_before_and_after_pinned_actor_set_audits")
        is not True
        or validity.get("cross_session_baseline_actor_type_multiset_sha256")
        != baseline_signature
        or validity.get("all_sessions_restored_world_settings") is not True
        or validity.get("all_sessions_cleanup_error_free") is not True
        or validity.get("all_sessions_finalized") is not True
        or not isinstance(sessions, list)
        or not sessions
        or any(session.get("world_settings_restored") is not True for session in sessions)
        or any(
            session.get("server_attestation_final_verified") is not True
            for session in sessions
        )
        or any(session.get("cleanup_errors") != [] for session in sessions)
        or any(
            session.get("outcome") not in {"PASS", "INTERRUPTED", "FAIL"}
            for session in sessions
        )
    ):
        raise InstallationError("checkpoint lacks passing collection/session audits")
    for session in sessions:
        initial_audit = session.get("initial_actor_audit")
        inventory = initial_audit.get("actor_inventory") if isinstance(initial_audit, dict) else None
        server_attestation = session.get("local_server_attestation")
        server_binary = plan.get("carla_runtime_binaries", {}).get("server_executable")
        if (
            not isinstance(initial_audit, dict)
            or initial_audit.get("status") != "PASS_MAP_BASELINE_ONLY"
            or initial_audit.get("actor_type_multiset_sha256") != baseline_signature
            or not isinstance(inventory, list)
            or generator._actor_inventory_sha256(inventory, include_ids=False)
            != baseline_signature
            or not isinstance(server_attestation, dict)
            or not isinstance(server_binary, dict)
            or server_attestation.get("scope")
            != "local /proc TCP-listener-to-executable attestation"
            or server_attestation.get("host") != plan["server"]["host"]
            or server_attestation.get("port") != plan["server"]["port"]
            or server_attestation.get("executable_path") != server_binary.get("path")
            or server_attestation.get("executable_sha256")
            != server_binary.get("sha256")
        ):
            raise InstallationError(
                "checkpoint session has an invalid actor baseline or server attestation"
            )
    for expected_index, item in enumerate(completed):
        actor_audit = item.get("actor_audit") if isinstance(item, dict) else None
        if (
            not isinstance(item, dict)
            or item.get("scan_index") != expected_index
            or item.get("chunk_file") != f"scan_{expected_index:06d}.npy"
            or not isinstance(actor_audit, dict)
            or any(
                actor_audit.get(stage, {}).get("status") != "PASS_PINNED_ACTOR_SET"
                for stage in ("before", "after")
            )
        ):
            raise InstallationError("checkpoint contains an unaudited scan record")

    inspection = generator.inspect_pcd(pointcloud_path)
    for key in ("sha256", "size_bytes", "points", "encoding"):
        if inspection[key] != pointcloud_record.get(key):
            raise InstallationError(f"pointcloud {key} differs from provenance")
    try:
        qa_recheck = generator.run_complete_qa(pointcloud_path, plan, checkpoint)
    except generator.MapGenerationError as error:
        raise InstallationError(f"fresh semantic-LiDAR QA failed: {error}") from error
    if qa_recheck.get("status") != "PASS":
        raise InstallationError("fresh semantic-LiDAR QA did not pass")
    actual_lanelet_sha256 = sha256_file(lanelet_path)
    if actual_lanelet_sha256 != lanelet_sha256:
        raise InstallationError("Lanelet2 differs from its pinned Town10HD digest")
    if lanelet_qa.get("sha256") != lanelet_sha256:
        raise InstallationError("Lanelet QA used a different Lanelet2 file")
    return {
        "provenance_path": provenance_path,
        "provenance": provenance,
        "scan_plan_path": scan_plan_path,
        "scan_plan": plan,
        "checkpoint_path": checkpoint_path,
        "checkpoint": checkpoint,
        "pointcloud_path": pointcloud_path,
        "pointcloud_inspection": inspection,
        "qa_recheck": qa_recheck,
        "lanelet_path": lanelet_path,
        "lanelet_sha256": actual_lanelet_sha256,
    }


def _atomic_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def _ensure_exact_text(path: Path, content: str) -> None:
    if path.exists() or path.is_symlink():
        if path.is_file() and not path.is_symlink():
            try:
                existing = path.read_text(encoding="utf-8")
            except (OSError, UnicodeError) as error:
                raise InstallationError(f"cannot inspect existing file {path}: {error}") from error
            if existing == content:
                return
        raise InstallationError(f"refusing to replace non-matching bundle file: {path}")
    _atomic_text(path, content)


def _ensure_symlink(link: Path, source: Path) -> None:
    if link.is_symlink():
        if link.resolve() == source.resolve():
            return
        raise InstallationError(f"refusing to replace symlink with another target: {link}")
    if link.exists():
        raise InstallationError(f"refusing to replace regular bundle asset: {link}")
    temporary = link.with_name(f".{link.name}.{os.getpid()}.tmp")
    temporary.symlink_to(source.resolve())
    os.replace(temporary, link)


def _file_record(path: Path, expected_sha256: str) -> dict[str, Any]:
    actual = sha256_file(path)
    return {
        "path": str(path.resolve()),
        "exists": True,
        "expected_size_bytes": path.stat().st_size,
        "expected_sha256": expected_sha256,
        "size_bytes": path.stat().st_size,
        "sha256": actual,
        "match": actual == expected_sha256,
    }


def build_bundle_metadata(
    evidence: Mapping[str, Any],
    target: Path,
    profile: str,
    canonical_map: str,
    lanelet_inspection: Mapping[str, Any],
) -> dict[str, Any]:
    provenance = evidence["provenance"]
    pcd_inspection = dict(evidence["pointcloud_inspection"])
    pcd_path = evidence["pointcloud_path"]
    lanelet_path = evidence["lanelet_path"]
    qa = evidence["qa_recheck"]
    lanelet_qa = qa["lanelet2"]
    alignment = {
        "status": "PASS",
        "method": qa["method"],
        "osm_nodes": lanelet_qa["reference_points"],
        "source_pcd_points": pcd_inspection["points"],
        "downsampled_pcd_points": None,
        "nearest_xy_with_y_reflection_m": lanelet_qa["nearest_xy_m"],
        "nearest_abs_z_with_y_reflection_m": lanelet_qa["nearest_abs_z_m"],
        "coordinate_note": "PCD was reflected to ROS before voxel merge; no second reflection",
    }
    pointcloud_file = _file_record(pcd_path, pcd_inspection["sha256"])
    pointcloud_file["inspection"] = pcd_inspection
    return {
        "schema_version": 1,
        "profile": profile,
        "status": "full_map_structurally_ready_not_vad_validated",
        "canonical_carla_map": canonical_map,
        "projector_type": "Local",
        "carla_to_map_transform": {
            "kind": "carla_world_to_ros_y_reflection_applied_during_semantic_lidar_merge",
            "x_m": 0.0,
            "y_m": 0.0,
            "z_m": 0.0,
            "yaw_rad": 0.0,
            "confidence": "semantic_lidar_global_lanelet_and_route_qa_pass",
            "source": str(evidence["provenance_path"]),
        },
        "pointcloud_source_transform": {
            "kind": "already_ros_local_from_semantic_lidar_world_hits",
            "carla_world_to_ros": "(x, y, z) -> (x, -y, z)",
            "additional_transform_required": False,
        },
        "lanelet2": {
            "file": _file_record(lanelet_path, evidence["lanelet_sha256"]),
            "inspection": dict(lanelet_inspection),
        },
        "pointcloud_source": {
            "file": pointcloud_file,
            "inspection": pcd_inspection,
            "required_transform": "none; collector output is already ROS Local",
            "source_geometry": "CARLA semantic-LiDAR static surface ray hits",
        },
        "pointcloud_generated": pointcloud_file,
        "alignment": alignment,
        "route_qa": qa["routes"],
        "runtime": {
            "map": evidence["scan_plan"]["map"],
            "server": evidence["scan_plan"]["server"],
            "collection_validity": provenance["runtime"]["collection_validity"],
            "session_count": provenance["runtime"]["session_count"],
            "total_wall_seconds": provenance["runtime"]["total_wall_seconds"],
        },
        "semantic_lidar_provenance": {
            "path": str(evidence["provenance_path"]),
            "sha256": sha256_file(evidence["provenance_path"]),
            "status": provenance["status"],
            "scan_plan_fingerprint": evidence["scan_plan"]["fingerprint"],
            "completed_scans": provenance["checkpoint"]["completed_scans"],
            "cooked_asset_inventory_sha256": evidence["scan_plan"]["cooked_assets"][
                "inventory_sha256"
            ],
            "truth_claim": provenance["truth_claim"],
        },
        "bundle_files": {
            "lanelet2_map": str(target / "lanelet2_map.osm"),
            "pointcloud_map": str(target / "pointcloud_map.pcd"),
        },
    }


def install(args: argparse.Namespace) -> dict[str, Any]:
    collection_root = args.collection_root.expanduser().resolve()
    lanelet_path = args.lanelet_map.expanduser().resolve()
    target = args.target.expanduser().resolve()
    if not collection_root.is_dir():
        raise InstallationError(f"collection root is absent: {collection_root}")
    if not lanelet_path.is_file():
        raise InstallationError(f"Lanelet2 map is absent: {lanelet_path}")
    evidence = validate_collection(
        collection_root,
        lanelet_path,
        args.canonical_map,
        args.opendrive_sha256,
        args.lanelet_sha256,
    )
    helper = _load_helper("setup_custom_full_map.py", "semantic_lidar_bundle_setup")
    try:
        lanelet_inspection = helper.inspect_lanelet2(lanelet_path)
    except Exception as error:
        raise InstallationError(f"Lanelet2 structural inspection failed: {error}") from error
    if int(lanelet_inspection.get("road_lanelets", 0)) <= 0:
        raise InstallationError("Lanelet2 has no inspected road lanelets")
    metadata = build_bundle_metadata(
        evidence, target, args.profile, args.canonical_map, lanelet_inspection
    )
    target.mkdir(parents=True, exist_ok=True)
    _ensure_symlink(target / "lanelet2_map.osm", lanelet_path)
    _ensure_symlink(target / "pointcloud_map.pcd", evidence["pointcloud_path"])
    _ensure_exact_text(target / "map_projector_info.yaml", "projector_type: Local\n")
    metadata_text = json.dumps(metadata, indent=2, sort_keys=True, allow_nan=False) + "\n"
    _ensure_exact_text(target / "map_bundle.json", metadata_text)
    return {
        "status": "INSTALLED_FULL_MAP_STRUCTURAL_CANDIDATE",
        "scope": "map/route structural admission only; not an Autoware VAD execution PASS",
        "target": str(target),
        "metadata_sha256": sha256_file(target / "map_bundle.json"),
        "lanelet_sha256": sha256_file(target / "lanelet2_map.osm"),
        "pointcloud_sha256": sha256_file(target / "pointcloud_map.pcd"),
        "pointcloud_points": evidence["pointcloud_inspection"]["points"],
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collection-root", type=Path, required=True)
    parser.add_argument("--lanelet-map", type=Path, required=True)
    parser.add_argument("--target", type=Path, default=DEFAULT_TARGET)
    parser.add_argument("--profile", default="town10hd_opt")
    parser.add_argument("--canonical-map", default=DEFAULT_CANONICAL_MAP)
    parser.add_argument("--opendrive-sha256", default=DEFAULT_OPENDRIVE_SHA256)
    parser.add_argument("--lanelet-sha256", default=DEFAULT_LANELET_SHA256)
    args = parser.parse_args(argv)
    try:
        _require_digest(args.opendrive_sha256, "opendrive-sha256")
        _require_digest(args.lanelet_sha256, "lanelet-sha256")
    except InstallationError as error:
        parser.error(str(error))
    if not args.profile or not args.canonical_map:
        parser.error("profile and canonical-map must be non-empty")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    try:
        result = install(parse_args(argv))
    except (InstallationError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
