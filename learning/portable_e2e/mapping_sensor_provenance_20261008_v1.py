"""HH_261008 - Preserve actual per-sensor headers before validation; no simulator or numeric imports.

This helper does not connect to CARLA, read old captures, manufacture absent
headers, actuate a vehicle, admit TRAIN data or approve a mapping capability.
Only a separately sealed future collector may pass its actually received objects.
"""
from __future__ import annotations

import hashlib
import math
from pathlib import PurePosixPath

SCHEMA = "portable_e2e.mapping_sensor_provenance.20261008.v1"
ROLES = ("rgb", "depth", "semantic")
TYPES = dict(rgb="sensor.camera.rgb", depth="sensor.camera.depth",
             semantic="sensor.camera.semantic_segmentation")
WIDTH, HEIGHT = 640, 360
RAW_BYTES = WIDTH * HEIGHT * 4
TIMESTAMP_ATOL = 1e-6
POSE_ATOL, POSE_RTOL = 1e-5, 1e-5


def retained_scalar(value):
    """HH_261008 - Invalid values remain explicit evidence, never accepted finite substitutes."""
    if type(value) in (str, int, bool, type(None)):
        return value
    if type(value) is float:
        if math.isfinite(value):
            return value
        return dict(raw_type="float", raw_repr=repr(value), accepted=False)
    if type(value) in (tuple, list):
        return [retained_scalar(item) for item in value]
    if type(value) is dict:
        return {str(key): retained_scalar(item) for key, item in value.items()}
    return dict(raw_type=type(value).__name__, accepted=False)


def integer(value, *, minimum=0):
    return type(value) is int and minimum <= value < 2**63


def finite_number(value, *, positive=False):
    if type(value) not in (int, float):
        return False
    # HH_261008 - Overflowing integer fixtures fail as retained invalid evidence, not as an unhandled validator exception.
    try:
        return math.isfinite(value) and (value > 0 if positive else value >= 0)
    except OverflowError:
        return False


def finite_matrix_number(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def matrix(value):
    return (type(value) is list and len(value) == 4 and
            all(type(row) is list and len(row) == 4 for row in value) and
            all(finite_matrix_number(number) for row in value for number in row))


def relative_artifact(value):
    # HH_261008 - Future artifact references may not escape the owned output or alias absolute/legacy paths.
    if type(value) is not str or not value or "\\" in value:
        return False
    path = PurePosixPath(value)
    return not path.is_absolute() and str(path) == value and all(part not in (".", "..") for part in path.parts)


def sha256_text(value):
    return type(value) is str and len(value) == 64 and all(char in "0123456789abcdef" for char in value)


def observe_headers(snapshot, sensor_actors, received_images, *, ordinal, required_frame):
    """Read already received objects only, retaining extraction failures without any simulator action."""
    observed = dict(ordinal=retained_scalar(ordinal), required_frame=retained_scalar(required_frame),
                    world_snapshot={}, sensors=[], extraction_errors=[])
    errors = observed["extraction_errors"]
    try:
        observed["world_snapshot"] = retained_scalar(dict(frame=snapshot.frame,
            elapsed_seconds=snapshot.timestamp.elapsed_seconds, delta_seconds=snapshot.timestamp.delta_seconds))
    except Exception as failure:
        errors.append(dict(scope="world_snapshot", error=repr(failure)))
    if type(sensor_actors) not in (tuple, list) or type(received_images) not in (tuple, list):
        errors.append(dict(scope="sensor_collection", error="exact list/tuple required"))
        return observed
    if len(sensor_actors) != 3 or len(received_images) != 3:
        errors.append(dict(scope="sensor_collection", error="exactly three ordered actors and received images required",
                           actor_count=len(sensor_actors), image_count=len(received_images)))
        return observed
    for role, actor, image in zip(ROLES, sensor_actors, received_images):
        entry = dict(role=role, actor={}, measurement={})
        observed["sensors"].append(entry)
        try:
            entry["actor"] = retained_scalar(dict(id=actor.id, type_id=actor.type_id))
        except Exception as failure:
            errors.append(dict(scope=role+".actor", error=repr(failure)))
        try:
            entry["measurement"] = retained_scalar(dict(frame=image.frame, timestamp_s=image.timestamp,
                width=image.width, height=image.height, world_from_camera=image.transform.get_matrix()))
        except Exception as failure:
            errors.append(dict(scope=role+".header", error=repr(failure)))
        try:
            raw = image.raw_data
            # HH_261008 - A memoryview hashes the real BGRA buffer without creating an image array or an unbounded copy.
            view = memoryview(raw)
            try:
                if not view.c_contiguous or view.itemsize != 1 or view.ndim != 1:
                    raise ValueError("actual contiguous byte buffer required")
                entry["measurement"]["raw_data_bytes"] = view.nbytes
                if view.nbytes != RAW_BYTES:
                    raise ValueError("declared 640x360 BGRA byte count required before hashing")
                entry["measurement"]["raw_BGRA_sha256"] = hashlib.sha256(view).hexdigest()
            finally:
                view.release()
        except Exception as failure:
            errors.append(dict(scope=role+".raw_bytes", error=repr(failure)))
    return observed


def assess_headers(observed):
    """HH_261008 - Return evidence and every detected failure; a header check never admits a dataset."""
    if type(observed) is not dict:
        raise ValueError("explicit observed header dictionary required")
    # HH_261008 - Copy only bounded scalar evidence, not the received buffers or mutable actor objects.
    actual = retained_scalar(observed)
    failures = []
    def require(condition, name):
        if not condition:
            failures.append(name)
    require(not actual.get("extraction_errors"), "header_extraction_failed")
    require(integer(actual.get("ordinal")), "ordinal_not_strict_nonnegative_int")
    required = actual.get("required_frame")
    require(integer(required), "required_frame_not_strict_nonnegative_int")
    world = actual.get("world_snapshot")
    if type(world) is not dict:
        world = {}
        require(False, "world_snapshot_missing")
    require(integer(world.get("frame")) and world.get("frame") == required, "world_frame_mismatch")
    require(finite_number(world.get("elapsed_seconds")), "world_elapsed_not_finite_nonnegative")
    require(finite_number(world.get("delta_seconds"), positive=True), "world_delta_not_finite_positive")
    sensors = actual.get("sensors")
    if type(sensors) is not list or len(sensors) != 3 or any(type(value) is not dict for value in sensors):
        require(False, "three_ordered_sensor_headers_required")
        sensors = []
    require(tuple(value.get("role") for value in sensors) == ROLES, "sensor_role_order_mismatch")
    times, poses, ids = [], [], []
    for position, entry in enumerate(sensors):
        role = ROLES[position]
        actor, measurement = entry.get("actor"), entry.get("measurement")
        if type(actor) is not dict:
            actor = {}
        if type(measurement) is not dict:
            measurement = {}
        require(integer(actor.get("id"), minimum=1), role+"_actor_id_invalid")
        require(actor.get("type_id") == TYPES[role], role+"_actor_type_mismatch")
        ids.append(actor.get("id"))
        require(integer(measurement.get("frame")) and measurement.get("frame") == required, role+"_frame_mismatch")
        require(type(measurement.get("width")) is int and measurement.get("width") == WIDTH and
                type(measurement.get("height")) is int and measurement.get("height") == HEIGHT, role+"_dimensions_mismatch")
        require(type(measurement.get("raw_data_bytes")) is int and measurement.get("raw_data_bytes") == RAW_BYTES,
                role+"_raw_BGRA_length_mismatch")
        require(sha256_text(measurement.get("raw_BGRA_sha256")), role+"_actual_raw_hash_missing")
        timestamp = measurement.get("timestamp_s")
        valid_time = finite_number(timestamp)
        require(valid_time, role+"_timestamp_not_finite_nonnegative")
        if valid_time:
            times.append(timestamp)
            if finite_number(world.get("elapsed_seconds")):
                require(abs(timestamp-world["elapsed_seconds"]) <= TIMESTAMP_ATOL, role+"_world_timestamp_mismatch")
        pose = measurement.get("world_from_camera")
        valid_pose = matrix(pose)
        require(valid_pose, role+"_pose_not_finite_4x4")
        if valid_pose:
            require(all(abs(pose[3][i]-value) <= 1e-8 for i, value in enumerate((0, 0, 0, 1))), role+"_pose_not_affine")
            poses.append(pose)
    require(all(integer(value, minimum=1) for value in ids) and len(ids) == 3 and len(set(ids)) == 3,
            "three_distinct_actor_ids_required")
    timestamp_spread = max(times)-min(times) if len(times) == 3 else None
    if timestamp_spread is not None:
        require(timestamp_spread <= TIMESTAMP_ATOL, "sensor_timestamp_spread_exceeded")
    pose_delta = None
    if len(poses) == 3:
        pose_delta = max(abs(pose[i][j]-poses[0][i][j]) for pose in poses for i in range(4) for j in range(4))
        # HH_261008 - Match original np.allclose(reference, actual, atol=1e-5, rtol=1e-5) scalar semantics.
        require(all(abs(poses[0][i][j]-pose[i][j]) <= POSE_ATOL+POSE_RTOL*abs(pose[i][j])
                    for pose in poses for i in range(4) for j in range(4)), "sensor_pose_mismatch")
    return dict(schema=SCHEMA, provenance="HH_261008 - Actual provided headers only; no retrospective inferred values.",
        status="HEADERS_CONSISTENT_NOT_CAPTURE_OR_MAP_APPROVAL" if not failures else "FAILED_HEADERS_RETAINED",
        observed=actual, failures=failures, measured_timestamp_spread_s=timestamp_spread,
        measured_pose_max_abs_delta=pose_delta, timestamp_atol_s=TIMESTAMP_ATOL,
        pose_atol=POSE_ATOL, pose_rtol=POSE_RTOL, expected=1, observed_count=1,
        passed=0 if failures else 1, failed=1 if failures else 0, UNEXECUTED=0,
        evidence_origin="caller_provided_objects_not_authenticated_by_helper",
        TRAIN_admitted=False, learned_control=False, oracle_pose=True, oracle_depth=True, oracle_semantics=True,
        sensor_state_or_model_accuracy_verified=False, actual_CARLA_capture_claimed=False, hd_map_approved=False)


def bind_saved_artifacts(receipt, artifacts):
    """Validate explicit future artifact receipts; this helper neither opens nor writes files."""
    if type(receipt) is not dict or receipt.get("schema") != SCHEMA or type(artifacts) is not list or len(artifacts) != 2:
        raise ValueError("own header receipt and exact RGB PNG/oracle NPZ artifact receipts required")
    expected = (("rgb_png", ("rgb",)), ("oracle_npz", ("depth", "semantic")))
    copies = []
    for artifact, (kind, roles) in zip(artifacts, expected):
        if type(artifact) is not dict or set(artifact) != {"kind", "path", "size_bytes", "sha256", "roles"}:
            raise ValueError("exact artifact scalar receipt required")
        if (artifact["kind"] != kind or type(artifact["roles"]) is not list or tuple(artifact["roles"]) != roles
            or not relative_artifact(artifact["path"]) or not sha256_text(artifact["sha256"])
            or not integer(artifact["size_bytes"], minimum=1)):
            raise ValueError("artifact binding differs from future owned capture declaration")
        copies.append(retained_scalar(artifact))
    if copies[0]["path"] == copies[1]["path"]:
        raise ValueError("RGB and oracle artifacts may not alias")
    result = retained_scalar(receipt)
    result.update(saved_artifacts=copies, artifact_byte_hash_verification_by_helper=False,
                  raw_BGRA_to_encoded_artifact_byte_equality_claimed=False)
    return result
