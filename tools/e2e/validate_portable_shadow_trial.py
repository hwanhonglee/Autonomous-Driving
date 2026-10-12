#!/usr/bin/env python3
# HH_260906 - Validate immutable inputs and map provenance for one Portable E2E shadow trial.

"""Validate and bind one newly materialized Portable E2E shadow trial."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
from typing import Mapping
import xml.etree.ElementTree as ET

import yaml


SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
CAMERA_ORDER = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)
SHADOW_OUTPUT_TOPICS = (
    "/planning/portable_e2e/latency_ms",
    "/planning/portable_e2e/selected_candidate",
    "/planning/portable_e2e/shadow_path",
    "/planning/portable_e2e/shadow_trajectory",
    "/planning/portable_e2e/status",
)
REQUIRED_LAUNCH_ARGUMENTS = {
    "contract_file",
    "contract_sha256",
    "declared_map_id",
    "runtime_bundle_file",
    "runtime_bundle_sha256",
    "source_checkpoint_sha256",
    "corpus_fingerprint_sha256",
    "model_config_sha256",
    "rig_file",
    "rig_sha256",
    "route_file",
    "route_sha256",
    "device",
    "input_settle_timeout_s",
    "maximum_tf_translation_error_m",
    "maximum_tf_rotation_error_rad",
    "use_sim_time",
    "research_acknowledged",
}


class ValidationError(RuntimeError):
    """Report a fail-closed trial binding violation."""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _regular_file(raw_path: Path, label: str) -> Path:
    path = raw_path.expanduser().absolute()
    if path.is_symlink() or not path.is_file():
        raise ValidationError(f"{label} must be a regular non-symlink file")
    return path.resolve()


def _sha256_value(value: str, label: str) -> str:
    if SHA256_PATTERN.fullmatch(value) is None:
        raise ValidationError(f"{label} must be a lowercase SHA-256")
    return value


def _pinned_file(path: Path, expected_sha256: str, label: str) -> tuple[Path, str]:
    resolved = _regular_file(path, label)
    expected = _sha256_value(expected_sha256, f"{label} SHA-256")
    actual = _sha256(resolved)
    if actual != expected:
        raise ValidationError(f"{label} SHA-256 mismatch")
    return resolved, actual


def _json(path: Path, label: str) -> dict:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValidationError(f"cannot parse {label}: {error}") from error
    if not isinstance(payload, dict):
        raise ValidationError(f"{label} root must be an object")
    return payload


def _finite_transform(
    payload: object,
    label: str,
    *,
    allowed_metadata: frozenset[str] = frozenset(),
) -> dict[str, float]:
    numeric_fields = frozenset({"x_m", "y_m", "z_m", "yaw_rad"})
    if not isinstance(payload, dict) or not numeric_fields.issubset(payload):
        raise ValidationError(f"{label} must contain the exact map transform fields")
    unexpected = set(payload) - numeric_fields - allowed_metadata
    if unexpected:
        raise ValidationError(f"{label} contains unsupported fields: {sorted(unexpected)}")
    for name in allowed_metadata.intersection(payload):
        value = payload[name]
        if not isinstance(value, str) or not value.strip():
            raise ValidationError(f"{label}.{name} must be a non-empty string")
    result: dict[str, float] = {}
    for name in ("x_m", "y_m", "z_m", "yaw_rad"):
        value = payload[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValidationError(f"{label}.{name} must be numeric")
        parsed = float(value)
        if not math.isfinite(parsed):
            raise ValidationError(f"{label}.{name} must be finite")
        result[name] = parsed
    return result


def _validate_route_binding(args: argparse.Namespace) -> dict:
    output_dir = args.output_dir.expanduser().absolute()
    if output_dir.is_symlink() or not output_dir.is_dir():
        raise ValidationError("output_dir must be a regular existing directory")
    output_dir = output_dir.resolve()
    source_route = _regular_file(args.source_route, "source route")
    source_route_copy = _regular_file(
        output_dir / "source_route.json", "trial source route copy"
    )
    aligned_route = _regular_file(args.aligned_route, "aligned route")
    alignment_file = _regular_file(args.route_alignment, "route alignment report")
    map_bundle = _regular_file(args.map_bundle, "map bundle")
    carla_probe = _regular_file(args.carla_probe, "CARLA preflight map probe")
    if aligned_route != output_dir / "aligned_route.json":
        raise ValidationError("aligned route must be this trial's aligned_route.json")
    if alignment_file != output_dir / "route_alignment.json":
        raise ValidationError("alignment report must be this trial's route_alignment.json")
    if map_bundle != output_dir / "map_bundle.json":
        raise ValidationError("map bundle must be this trial's pinned map_bundle.json")
    if carla_probe != output_dir / "carla_preflight_health.json":
        raise ValidationError("CARLA map probe must be this trial's preflight result")

    source = _json(source_route, "source route")
    route = _json(aligned_route, "aligned route")
    alignment = _json(alignment_file, "route alignment report")
    bundle = _json(map_bundle, "map bundle")
    probe = _json(carla_probe, "CARLA preflight map probe")
    source_sha256 = _sha256(source_route)
    source_copy_sha256 = _sha256(source_route_copy)
    if source_copy_sha256 != source_sha256:
        raise ValidationError("trial source route copy does not match the pinned source")
    aligned_sha256 = _sha256(aligned_route)
    alignment_sha256 = _sha256(alignment_file)
    map_bundle_sha256 = _sha256(map_bundle)
    carla_probe_sha256 = _sha256(carla_probe)
    if re.fullmatch(r"[A-Za-z0-9_]{1,64}", args.expected_map) is None:
        raise ValidationError("expected CARLA map must normalize to a safe declared map ID")
    declared_map_id = args.expected_map.lower()
    expected_canonical_map = f"/Game/Carla/Maps/{args.expected_map}"
    observed_map_name = probe.get("active_map_name")
    observed_map_basename = (
        observed_map_name.rstrip("/").rsplit("/", 1)[-1]
        if isinstance(observed_map_name, str)
        else ""
    )
    observed_map_id = observed_map_basename.lower()
    if (
        probe.get("schema_version") != 1
        or probe.get("status") != "PASS"
        or probe.get("mode") != "running"
        or probe.get("stage") != "trial_preflight"
        or probe.get("read_only") is not True
        or probe.get("rpc_sequence")
        != ["get_world", "world.get_map", "world.get_snapshot"]
        or probe.get("expected_map") != args.expected_map
        or probe.get("active_map_basename") != args.expected_map
        or observed_map_basename != args.expected_map
        or observed_map_id != declared_map_id
    ):
        raise ValidationError("independent CARLA API map observation does not match expected map")
    if source.get("town") != args.expected_map or route.get("town") != args.expected_map:
        raise ValidationError("source/aligned route town does not match the owned CARLA map")
    if bundle.get("canonical_carla_map") != expected_canonical_map:
        raise ValidationError("map bundle canonical CARLA map does not match the owned map")
    if route.get("coordinate_reference") != "base_link":
        raise ValidationError("aligned route must declare coordinate_reference=base_link")
    if alignment.get("status") != "PASS":
        raise ValidationError("route alignment report must have PASS status")
    if Path(str(alignment.get("source_route", ""))).resolve() != source_route:
        raise ValidationError("route alignment report names a different source route")
    if Path(str(alignment.get("aligned_route", ""))).resolve() != aligned_route:
        raise ValidationError("route alignment report names a different aligned route")
    if alignment.get("source_route_sha256") != source_sha256:
        raise ValidationError("route alignment source digest mismatch")
    if alignment.get("aligned_route_sha256") != aligned_sha256:
        raise ValidationError("route alignment output digest mismatch")
    if alignment.get("profile") != bundle.get("profile"):
        raise ValidationError("route alignment map profile mismatch")
    route_alignment = route.get("coordinate_alignment")
    if not isinstance(route_alignment, dict):
        raise ValidationError("aligned route lacks coordinate_alignment")
    if (
        route_alignment.get("schema_version") != 1
        or route_alignment.get("source_frame") != "carla_map"
        or route_alignment.get("target_frame") != "map"
        or route_alignment.get("source_route_sha256") != source_sha256
        or route_alignment.get("map_bundle_profile") != bundle.get("profile")
    ):
        raise ValidationError("aligned route coordinate provenance mismatch")
    expected_transform = _finite_transform(
        bundle.get("carla_to_map_transform"),
        "map bundle transform",
        allowed_metadata=frozenset({"kind", "confidence", "source"}),
    )
    report_transform = _finite_transform(
        alignment.get("carla_to_map_transform"), "alignment report transform"
    )
    route_transform = _finite_transform(
        route_alignment.get("carla_to_map_transform"), "aligned route transform"
    )
    if report_transform != expected_transform or route_transform != expected_transform:
        raise ValidationError("route and report do not use the pinned map transform")
    return {
        "expected_map": args.expected_map,
        "declared_map_id": declared_map_id,
        "observed_map_id": observed_map_id,
        "observed_map_name": observed_map_name,
        "observed_map_source": "carla_python_api_world_get_map",
        "observed_map_probe": str(carla_probe),
        "observed_map_probe_sha256": carla_probe_sha256,
        "canonical_carla_map": expected_canonical_map,
        "map_bundle_profile": bundle.get("profile"),
        "map_bundle_sha256": map_bundle_sha256,
        "source_route": str(source_route),
        "source_route_sha256": source_sha256,
        "source_route_copy": str(source_route_copy),
        "aligned_route": str(aligned_route),
        "aligned_route_sha256": aligned_sha256,
        "route_alignment_sha256": alignment_sha256,
        "route_scenario": route.get("scenario"),
        "coordinate_reference": "base_link",
        "map_transform": expected_transform,
        "materialized_by_current_trial": True,
    }


def _validate_sensor_mapping(path: Path) -> dict:
    mapping_path = _regular_file(path, "Portable E2E 10 Hz sensor mapping")
    try:
        payload = yaml.safe_load(mapping_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValidationError(f"cannot parse Portable E2E sensor mapping: {error}") from error
    mappings = payload.get("sensor_mappings") if isinstance(payload, dict) else None
    if not isinstance(mappings, dict):
        raise ValidationError("Portable E2E sensor mapping lacks sensor_mappings")
    cameras: dict[str, dict] = {}
    for value in mappings.values():
        if isinstance(value, dict) and value.get("carla_type") == "sensor.camera.rgb":
            camera = value.get("id")
            if not isinstance(camera, str) or camera in cameras:
                raise ValidationError("Portable E2E sensor mapping has duplicate camera IDs")
            cameras[camera] = value
    if set(cameras) != set(CAMERA_ORDER):
        raise ValidationError("Portable E2E sensor mapping must contain exactly six ABI cameras")
    for camera in CAMERA_ORDER:
        value = cameras[camera]
        ros = value.get("ros_config")
        parameters = value.get("parameters")
        if not isinstance(ros, dict) or not isinstance(parameters, dict):
            raise ValidationError(f"Portable E2E camera {camera} lacks configuration")
        if (
            ros.get("frequency_hz") != 11
            or ros.get("qos_profile") != "reliable"
            or ros.get("image_qos_profile") != "best_effort_depth_1"
            or ros.get("camera_info_qos_profile") != "reliable"
            or ros.get("topic_image") != f"/sensing/camera/{camera}/image_raw"
            or ros.get("topic_info") != f"/sensing/camera/{camera}/camera_info"
            or parameters.get("sensor_tick") != 0.1
        ):
            raise ValidationError(f"Portable E2E camera {camera} violates the 10 Hz QoS contract")
    return {
        "file": str(mapping_path),
        "sha256": _sha256(mapping_path),
        "profile_id": "portable_e2e_exact_bundle_10hz_v2",
        "requested_bridge_publish_cap_hz": 11,
        "effective_camera_rate_hz": 10,
        "camera_count": 6,
        "image_qos": "best_effort_keep_last_depth_1",
        "camera_info_qos": "reliable",
    }


def _validate_shadow_launch_snapshot(path: Path) -> tuple[dict, bytes]:
    launch_path = _regular_file(path, "Portable E2E shadow launch")
    try:
        snapshot = launch_path.read_bytes()
        text = snapshot.decode("utf-8")
        root = ET.fromstring(snapshot)
    except (OSError, UnicodeError, ET.ParseError) as error:
        raise ValidationError(f"cannot parse Portable E2E shadow launch: {error}") from error
    if root.tag != "launch" or root.attrib:
        raise ValidationError(
            "Portable E2E shadow launch root must be an attribute-free launch element"
        )
    if root.findall(".//remap"):
        raise ValidationError("Portable E2E shadow launch must not contain remaps")
    top_level = list(root)
    if (
        len(top_level) != len(REQUIRED_LAUNCH_ARGUMENTS) + 1
        or any(element.tag != "arg" for element in top_level[:-1])
        or top_level[-1].tag != "node"
    ):
        raise ValidationError(
            "Portable E2E shadow launch permits only direct arguments followed by one node"
        )
    node = top_level[-1]
    if node.attrib != {
        "pkg": "autoware_e2e_vad_launch",
        "exec": "portable_e2e_shadow_node.py",
        "name": "portable_e2e_shadow",
        "output": "screen",
    }:
        raise ValidationError("Portable E2E shadow launch node identity changed")
    argument_elements = top_level[:-1]
    argument_names = [item.get("name") for item in argument_elements]
    if (
        set(argument_names) != REQUIRED_LAUNCH_ARGUMENTS
        or len(argument_names) != len(REQUIRED_LAUNCH_ARGUMENTS)
    ):
        raise ValidationError("Portable E2E shadow launch argument contract changed")
    argument_by_name = {item.get("name"): item for item in argument_elements}
    expected_defaults = {
        "device": "cpu",
        "use_sim_time": "true",
        "input_settle_timeout_s": "0.05",
        "maximum_tf_translation_error_m": "0.005",
        "maximum_tf_rotation_error_rad": "0.005",
        "research_acknowledged": "false",
    }
    for name, element in argument_by_name.items():
        if (
            not set(element.attrib).issubset({"name", "default", "description"})
            or list(element)
            or (element.text is not None and element.text.strip())
        ):
            raise ValidationError(
                f"Portable E2E shadow launch argument has non-allowlisted structure: {name}"
            )
        expected_default = expected_defaults.get(name)
        if expected_default is None:
            if "default" in element.attrib:
                raise ValidationError(
                    f"Portable E2E required launch input gained a default: {name}"
                )
        elif element.get("default") != expected_default:
            raise ValidationError(f"Portable E2E launch default changed: {name}")
    parameter_elements = list(node)
    if any(element.tag != "param" for element in parameter_elements):
        raise ValidationError(
            "Portable E2E shadow launch node contains a non-parameter child"
        )
    parameter_names = [item.get("name") for item in parameter_elements]
    if (
        set(parameter_names) != REQUIRED_LAUNCH_ARGUMENTS
        or len(parameter_names) != len(REQUIRED_LAUNCH_ARGUMENTS)
    ):
        raise ValidationError("Portable E2E shadow launch parameter contract changed")
    for element in parameter_elements:
        name = element.get("name")
        if (
            set(element.attrib) != {"name", "value"}
            or list(element)
            or (element.text is not None and element.text.strip())
        ):
            raise ValidationError(
                f"Portable E2E shadow launch parameter has non-allowlisted structure: {name}"
            )
        if element.get("value") != f"$(var {name})":
            raise ValidationError(
                f"Portable E2E launch parameter is not bound to its same-name input: {name}"
            )
    if any(topic in text for topic in ("/planning/trajectory", "/control/", "/vehicle/command/")):
        raise ValidationError("Portable E2E shadow launch references a canonical control topic")
    record = {
        "file": str(launch_path),
        "sha256": hashlib.sha256(snapshot).hexdigest(),
        "node": "/portable_e2e_shadow",
        "remaps_allowed": False,
        "exact_same_name_parameter_bindings": True,
        "pinned_defaults": expected_defaults,
        "allowed_output_topics": list(SHADOW_OUTPUT_TOPICS),
    }
    return record, snapshot


def _validate_shadow_launch(path: Path) -> dict:
    return _validate_shadow_launch_snapshot(path)[0]


def _validate_installed_shadow_launch(path: Path, source: Mapping[str, object]) -> dict:
    """Bind the launch file resolved by ROS to the validated source content."""
    installed = path.expanduser().absolute()
    if not installed.exists() or installed.is_dir():
        raise ValidationError("installed Portable E2E shadow launch is missing")
    try:
        resolved = installed.resolve(strict=True)
    except OSError as error:
        raise ValidationError(
            f"cannot resolve installed Portable E2E shadow launch: {error}"
        ) from error
    # HH_260906 - Reject any symlink target other than the exact validated source path.
    if installed.is_symlink() and resolved != Path(str(source["file"])):
        raise ValidationError(
            "installed Portable E2E shadow launch symlink targets another source"
        )
    try:
        snapshot = resolved.read_bytes()
    except OSError as error:
        raise ValidationError(
            f"cannot read installed Portable E2E shadow launch: {error}"
        ) from error
    digest = hashlib.sha256(snapshot).hexdigest()
    if not resolved.is_file() or digest != source.get("sha256"):
        raise ValidationError(
            "installed Portable E2E shadow launch does not match validated source"
        )
    try:
        resolved_after_snapshot = installed.resolve(strict=True)
    except OSError as error:
        raise ValidationError(
            f"cannot resolve installed Portable E2E shadow launch: {error}"
        ) from error
    if resolved_after_snapshot != resolved:
        raise ValidationError(
            "installed Portable E2E shadow launch changed while it was validated"
        )
    return {
        "file": str(installed),
        "resolved_file": str(resolved),
        "sha256": digest,
        "symlink_install": installed.is_symlink(),
        "matches_validated_source": True,
    }


def _validate_controlling_vad_transport(
    model_override_path: Path, cyclonedds_path: Path
) -> dict:
    model_path = _regular_file(model_override_path, "controlling VAD model override")
    cyclone_path = _regular_file(cyclonedds_path, "CycloneDDS loopback config")
    try:
        model = yaml.safe_load(model_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValidationError(f"cannot parse controlling VAD model override: {error}") from error
    try:
        sync = model["/**"]["ros__parameters"]["sync_params"]
    except (KeyError, TypeError) as error:
        raise ValidationError("controlling VAD model override lacks sync_params") from error
    if (
        not isinstance(sync, dict)
        or sync.get("image_reliability") != "best_effort"
        or sync.get("image_queue_depth") != 1
    ):
        raise ValidationError("controlling VAD must use Best-Effort KEEP_LAST depth 1")
    cyclone_text = cyclone_path.read_text(encoding="utf-8")
    try:
        ET.fromstring(cyclone_text)
    except ET.ParseError as error:
        raise ValidationError(f"cannot parse CycloneDDS config: {error}") from error
    if 'name="lo"' not in cyclone_text:
        raise ValidationError("CycloneDDS config must pin the loopback interface")
    return {
        "vad_model_override_file": str(model_path),
        "vad_model_override_sha256": _sha256(model_path),
        "vad_image_reliability": "best_effort",
        "vad_image_history": "keep_last",
        "vad_image_depth": 1,
        "cyclonedds_file": str(cyclone_path),
        "cyclonedds_sha256": _sha256(cyclone_path),
        "network_interface": "lo",
    }


def _write_exclusive_json(path: Path, payload: dict) -> None:
    target = path.expanduser().absolute()
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.is_symlink() or target.exists():
        raise ValidationError("Portable E2E trial binding output already exists")
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{target.name}.", dir=target.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, target)
        except FileExistsError as error:
            raise ValidationError(
                "Portable E2E trial binding output raced with another writer"
            ) from error
    finally:
        temporary.unlink(missing_ok=True)


def _write_exclusive_bytes(path: Path, payload: bytes) -> Path:
    target = path.expanduser().absolute()
    if not target.name.endswith(".launch.xml"):
        raise ValidationError("Portable E2E launch snapshot must end with .launch.xml")
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.is_symlink() or target.exists():
        raise ValidationError("Portable E2E launch snapshot output already exists")
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{target.name}.", dir=target.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.chmod(0o444)
        try:
            os.link(temporary, target)
        except FileExistsError as error:
            raise ValidationError(
                "Portable E2E launch snapshot output raced with another writer"
            ) from error
    finally:
        temporary.unlink(missing_ok=True)
    return target.resolve()


def _snapshot_binding(path: Path, source_sha256: str) -> dict:
    candidate = path.expanduser().absolute()
    if not candidate.exists():
        raise ValidationError(
            "Portable E2E launch snapshot is missing; expected a regular non-symlink file"
        )
    if candidate.is_symlink() or not candidate.is_file():
        raise ValidationError(
            "Portable E2E launch snapshot must be a regular non-symlink file"
        )
    snapshot = _validate_shadow_launch(path)
    resolved = Path(snapshot["file"])
    mode_octal = format(resolved.stat().st_mode & 0o777, "04o")
    if mode_octal != "0444":
        raise ValidationError("Portable E2E launch snapshot mode must be 0444")
    if snapshot["sha256"] != source_sha256:
        raise ValidationError(
            "Portable E2E launch snapshot does not match source bytes (SHA-256 mismatch)"
        )
    return {
        "file": str(resolved),
        "sha256": snapshot["sha256"],
        "mode_octal": mode_octal,
        "regular_file": True,
        "symlink": False,
        "source_sha256": source_sha256,
        "exact_source_bytes": True,
        "direct_ros_launch": True,
    }


def _materialize_shadow_launch_snapshot(
    path: Path, source: Mapping[str, object]
) -> tuple[Path, dict]:
    current_source, source_snapshot = _validate_shadow_launch_snapshot(
        Path(str(source["file"]))
    )
    initial_source = dict(source)
    initial_source.pop("installed_binding", None)
    initial_source.pop("snapshot_binding", None)
    if current_source != initial_source:
        raise ValidationError(
            "Portable E2E source launch changed before snapshot materialization"
        )
    current_installed = _validate_installed_shadow_launch(
        Path(str(source["installed_binding"]["file"])),
        current_source,
    )
    if current_installed != source.get("installed_binding"):
        raise ValidationError(
            "installed Portable E2E launch changed before snapshot materialization"
        )
    created = _write_exclusive_bytes(path, source_snapshot)
    try:
        binding = _snapshot_binding(created, str(source["sha256"]))
    except BaseException:
        created.unlink(missing_ok=True)
        raise
    return created, binding


def validate(args: argparse.Namespace) -> dict:
    if args.device not in ("cpu", "cuda:0"):
        raise ValidationError("Portable E2E device must be cpu or logical cuda:0")
    route = _validate_route_binding(args)
    sensor_mapping = _validate_sensor_mapping(args.sensor_mapping)
    shadow_launch = _validate_shadow_launch(args.shadow_launch)
    shadow_launch["installed_binding"] = _validate_installed_shadow_launch(
        args.installed_shadow_launch,
        shadow_launch,
    )
    controlling_vad_transport = _validate_controlling_vad_transport(
        args.vad_model_override, args.cyclonedds_config
    )
    runtime_bundle, runtime_bundle_sha256 = _pinned_file(
        args.runtime_bundle, args.runtime_bundle_sha256, "runtime bundle"
    )
    contract_file, contract_sha256 = _pinned_file(
        args.contract_file, args.contract_sha256, "Common10 contract"
    )
    rig_file, rig_sha256 = _pinned_file(args.rig_file, args.rig_sha256, "camera rig")
    payload = {
        "schema_version": 1,
        "status": "PASS",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "profile_id": "portable_e2e_shadow_owned_trial_30kph_10hz_v1",
        "execution_mode": "shadow_only",
        "controlling_planner": "autoware_vad",
        "vehicle_control_approved": False,
        "canonical_or_control_publication_allowed": False,
        "duplicate_node_allowed": False,
        "remaps_allowed": False,
        "device": args.device,
        "route_binding": route,
        "sensor_mapping": sensor_mapping,
        "controlling_vad_transport": controlling_vad_transport,
        "shadow_launch": shadow_launch,
        "model_provenance": {
            "runtime_bundle_file": str(runtime_bundle),
            "runtime_bundle_sha256": runtime_bundle_sha256,
            "source_checkpoint_sha256": _sha256_value(
                args.source_checkpoint_sha256, "source checkpoint SHA-256"
            ),
            "model_config_sha256": _sha256_value(
                args.model_config_sha256, "model config SHA-256"
            ),
            "corpus_fingerprint_sha256": _sha256_value(
                args.corpus_fingerprint_sha256, "corpus fingerprint SHA-256"
            ),
            "contract_file": str(contract_file),
            "contract_sha256": contract_sha256,
            "rig_file": str(rig_file),
            "rig_sha256": rig_sha256,
        },
    }
    return payload


def recheck_launch_binding(args: argparse.Namespace) -> dict:
    binding_path = _regular_file(args.binding, "Portable E2E trial binding")
    try:
        binding_snapshot = binding_path.read_bytes()
    except OSError as error:
        raise ValidationError(f"cannot read Portable E2E trial binding: {error}") from error
    binding_sha256 = hashlib.sha256(binding_snapshot).hexdigest()
    expected_binding_sha256 = _sha256_value(
        args.binding_sha256, "Portable E2E trial binding SHA-256"
    )
    if binding_sha256 != expected_binding_sha256:
        raise ValidationError(
            "Portable E2E trial binding SHA-256 mismatch before ROS launch"
        )
    try:
        binding = json.loads(binding_snapshot)
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ValidationError(f"cannot parse Portable E2E trial binding: {error}") from error
    if not isinstance(binding, dict) or binding.get("status") != "PASS":
        raise ValidationError("Portable E2E trial binding is not a PASS object")
    initial_source = binding.get("shadow_launch")
    if not isinstance(initial_source, dict):
        raise ValidationError("Portable E2E trial binding lacks shadow launch provenance")

    current_source = _validate_shadow_launch(args.shadow_launch)
    initial_source_file = initial_source.get("file")
    if current_source.get("file") != initial_source_file:
        raise ValidationError("validated source launch path changed before ROS launch")
    if current_source.get("sha256") != initial_source.get("sha256"):
        raise ValidationError(
            "validated source launch SHA-256 changed before ROS launch"
        )
    initial_source_without_install = dict(initial_source)
    initial_source_without_install.pop("installed_binding", None)
    initial_source_without_install.pop("snapshot_binding", None)
    if current_source != initial_source_without_install:
        raise ValidationError(
            "validated source launch contract changed before ROS launch"
        )
    current_installed = _validate_installed_shadow_launch(
        args.installed_shadow_launch,
        current_source,
    )
    if current_installed != initial_source.get("installed_binding"):
        raise ValidationError(
            "installed Portable E2E shadow launch binding changed before ROS launch"
        )
    initial_snapshot = initial_source.get("snapshot_binding")
    if not isinstance(initial_snapshot, dict):
        raise ValidationError(
            "Portable E2E trial binding lacks an executable launch snapshot"
        )
    expected_snapshot_file = initial_snapshot.get("file")
    provided_snapshot = args.shadow_launch_snapshot.expanduser().absolute()
    if str(provided_snapshot) != expected_snapshot_file:
        raise ValidationError(
            "Portable E2E executable launch snapshot changed before ROS launch: path mismatch"
        )
    current_snapshot = _snapshot_binding(
        args.shadow_launch_snapshot,
        str(initial_source.get("sha256")),
    )
    if current_snapshot != initial_snapshot:
        raise ValidationError(
            "Portable E2E executable launch snapshot changed before ROS launch"
        )
    snapshot_contract = _validate_shadow_launch(args.shadow_launch_snapshot)
    comparable_snapshot_contract = dict(snapshot_contract)
    comparable_snapshot_contract["file"] = current_source["file"]
    if comparable_snapshot_contract != current_source:
        raise ValidationError(
            "Portable E2E executable launch snapshot contract differs from source"
        )
    return {
        "schema_version": 1,
        "status": "PASS",
        "checked_at": datetime.now(timezone.utc).isoformat(),
        "check_stage": "immediately_before_ros_launch",
        "trial_binding_file": str(binding_path),
        "trial_binding_sha256": binding_sha256,
        "source_launch": current_source,
        "installed_binding": current_installed,
        "runtime_shadow_launch": current_snapshot,
        "matches_initial_binding": True,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--source-route", type=Path, required=True)
    parser.add_argument("--aligned-route", type=Path, required=True)
    parser.add_argument("--route-alignment", type=Path, required=True)
    parser.add_argument("--map-bundle", type=Path, required=True)
    parser.add_argument("--carla-probe", type=Path, required=True)
    parser.add_argument("--expected-map", required=True)
    parser.add_argument("--sensor-mapping", type=Path, required=True)
    parser.add_argument("--vad-model-override", type=Path, required=True)
    parser.add_argument("--cyclonedds-config", type=Path, required=True)
    parser.add_argument("--shadow-launch", type=Path, required=True)
    parser.add_argument("--installed-shadow-launch", type=Path, required=True)
    parser.add_argument("--shadow-launch-snapshot", type=Path, required=True)
    parser.add_argument("--runtime-bundle", type=Path, required=True)
    parser.add_argument("--runtime-bundle-sha256", required=True)
    parser.add_argument("--source-checkpoint-sha256", required=True)
    parser.add_argument("--model-config-sha256", required=True)
    parser.add_argument("--corpus-fingerprint-sha256", required=True)
    parser.add_argument("--contract-file", type=Path, required=True)
    parser.add_argument("--contract-sha256", required=True)
    parser.add_argument("--rig-file", type=Path, required=True)
    parser.add_argument("--rig-sha256", required=True)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def parse_recheck_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Recheck a bound Portable E2E launch immediately before ROS launch."
    )
    parser.add_argument("--binding", type=Path, required=True)
    parser.add_argument("--binding-sha256", required=True)
    parser.add_argument("--shadow-launch", type=Path, required=True)
    parser.add_argument("--installed-shadow-launch", type=Path, required=True)
    parser.add_argument("--shadow-launch-snapshot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    created_snapshot: Path | None = None
    try:
        if arguments[:1] == ["recheck-launch"]:
            args = parse_recheck_args(arguments[1:])
            payload = recheck_launch_binding(args)
        else:
            args = parse_args(arguments)
            payload = validate(args)
            created_snapshot, snapshot_binding = _materialize_shadow_launch_snapshot(
                args.shadow_launch_snapshot,
                payload["shadow_launch"],
            )
            payload["shadow_launch"]["snapshot_binding"] = snapshot_binding
        _write_exclusive_json(args.output, payload)
    except ValidationError as error:
        if created_snapshot is not None:
            created_snapshot.unlink(missing_ok=True)
        raise SystemExit(f"Portable E2E shadow trial validation failed: {error}") from error
    except BaseException:
        if created_snapshot is not None:
            created_snapshot.unlink(missing_ok=True)
        raise
    print(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
