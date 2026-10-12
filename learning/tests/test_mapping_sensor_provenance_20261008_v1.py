"""HH_261008 - Synthetic scalar/header fixtures, never genuine CARLA captures or capability evidence."""
from copy import deepcopy
from types import SimpleNamespace

import pytest
from portable_e2e import mapping_sensor_provenance_20261008_v1 as subject


def fixture():
    pose = [[1., 0., 0., 1.5], [0., 1., 0., 0.], [0., 0., 1., 2.], [0., 0., 0., 1.]]
    snapshot = SimpleNamespace(frame=42, timestamp=SimpleNamespace(elapsed_seconds=4.2, delta_seconds=.1))
    actors = [SimpleNamespace(id=i+1, type_id=subject.TYPES[role]) for i, role in enumerate(subject.ROLES)]
    images = [SimpleNamespace(frame=42, timestamp=4.2, width=640, height=360,
        raw_data=bytes(subject.RAW_BYTES), transform=SimpleNamespace(get_matrix=lambda: deepcopy(pose))) for _ in actors]
    return snapshot, actors, images


def observed():
    return subject.observe_headers(*fixture(), ordinal=0, required_frame=42)


def test_valid_scalar_headers_not_real_capture():
    result = subject.assess_headers(observed())
    assert result["passed"] == 1 and result["failed"] == 0
    assert not result["actual_CARLA_capture_claimed"] and not result["TRAIN_admitted"]
    assert result["evidence_origin"] == "caller_provided_objects_not_authenticated_by_helper"
    assert result["observed"]["sensors"][0]["measurement"]["raw_data_bytes"] == 921600


@pytest.mark.parametrize("bad", [True, -1, 2**63, 42., "42", None])
def test_strict_frame_integer(bad):
    record = observed(); record["sensors"][1]["measurement"]["frame"] = bad
    assert "depth_frame_mismatch" in subject.assess_headers(record)["failures"]


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -1., True, "4.2", None, 2**2048])
def test_invalid_sensor_time_retained(bad):
    record = observed(); record["sensors"][0]["measurement"]["timestamp_s"] = bad
    result = subject.assess_headers(record)
    assert "rgb_timestamp_not_finite_nonnegative" in result["failures"]
    if type(bad) is float and not subject.math.isfinite(bad):
        assert result["observed"]["sensors"][0]["measurement"]["timestamp_s"]["accepted"] is False


@pytest.mark.parametrize("field,bad", [("frame", 41), ("elapsed_seconds", float("nan")), ("delta_seconds", 0), ("delta_seconds", True)])
def test_world_header_failure(field, bad):
    record = observed(); record["world_snapshot"][field] = bad
    assert subject.assess_headers(record)["failed"] == 1


def test_sensor_and_world_timestamp_mismatch():
    record = observed(); record["sensors"][2]["measurement"]["timestamp_s"] += .01
    result = subject.assess_headers(record)
    assert {"semantic_world_timestamp_mismatch", "sensor_timestamp_spread_exceeded"} <= set(result["failures"])


@pytest.mark.parametrize("field,bad", [("width", True), ("width", 1280), ("height", 360.), ("raw_data_bytes", 921599),
    ("raw_BGRA_sha256", "G"*64), ("raw_BGRA_sha256", None)])
def test_image_header_contract(field, bad):
    record = observed(); record["sensors"][0]["measurement"][field] = bad
    assert subject.assess_headers(record)["failed"] == 1


@pytest.mark.parametrize("bad", [True, 0, -1, 2**63, "2"])
def test_actor_id_invalid(bad):
    record = observed(); record["sensors"][1]["actor"]["id"] = bad
    assert subject.assess_headers(record)["failed"] == 1


def test_actor_duplicate_and_role_order():
    record = observed(); record["sensors"][2]["actor"]["id"] = 1
    assert "three_distinct_actor_ids_required" in subject.assess_headers(record)["failures"]
    record = observed(); record["sensors"].reverse()
    assert "sensor_role_order_mismatch" in subject.assess_headers(record)["failures"]


def test_actor_type_wrong():
    record = observed(); record["sensors"][2]["actor"]["type_id"] = "sensor.camera.rgb"
    assert "semantic_actor_type_mismatch" in subject.assess_headers(record)["failures"]


@pytest.mark.parametrize("bad", [[], [[1]*4]*3, [[float("nan")]*4]*4, [[True]*4]*4, [[2**2048]*4]*4, None])
def test_pose_invalid(bad):
    record = observed(); record["sensors"][1]["measurement"]["world_from_camera"] = bad
    assert "depth_pose_not_finite_4x4" in subject.assess_headers(record)["failures"]


def test_pose_tolerance_matches_original_scalar_allclose():
    record = observed(); record["sensors"][1]["measurement"]["world_from_camera"][0][3] += 2e-5
    assert subject.assess_headers(record)["passed"] == 1
    record["sensors"][1]["measurement"]["world_from_camera"][0][3] += 1e-3
    assert "sensor_pose_mismatch" in subject.assess_headers(record)["failures"]


def test_pose_nonaffine():
    record = observed(); record["sensors"][1]["measurement"]["world_from_camera"][3][3] = 0
    assert "depth_pose_not_affine" in subject.assess_headers(record)["failures"]


def test_snapshot_missing_extraction_evidence():
    snap, actors, images = fixture(); del snap.timestamp
    actual = subject.observe_headers(snap, actors, images, ordinal=0, required_frame=42)
    assert actual["extraction_errors"] and subject.assess_headers(actual)["failed"] == 1


def test_raw_wrong_length_not_hashed_or_fixed():
    snap, actors, images = fixture(); images[0].raw_data = b"too short"
    actual = subject.observe_headers(snap, actors, images, ordinal=0, required_frame=42)
    assert actual["sensors"][0]["measurement"]["raw_data_bytes"] == 9
    assert "raw_BGRA_sha256" not in actual["sensors"][0]["measurement"]
    assert subject.assess_headers(actual)["failed"] == 1


def test_missing_collection_does_not_pad_sensors():
    snap, actors, images = fixture()
    actual = subject.observe_headers(snap, actors[:2], images, ordinal=0, required_frame=42)
    assert actual["sensors"] == [] and actual["extraction_errors"]
    assert subject.assess_headers(actual)["failed"] == 1


def artifacts():
    return [dict(kind="rgb_png", path="frames/0000.png", size_bytes=123, sha256="a"*64, roles=["rgb"]),
            dict(kind="oracle_npz", path="frames/0000.npz", size_bytes=234, sha256="b"*64, roles=["depth", "semantic"])]


def test_artifact_binding_retains_no_byte_verification_claim():
    receipt = subject.assess_headers(observed())
    result = subject.bind_saved_artifacts(receipt, artifacts())
    assert len(result["saved_artifacts"]) == 2 and "saved_artifacts" not in receipt
    assert not result["artifact_byte_hash_verification_by_helper"]
    assert not result["raw_BGRA_to_encoded_artifact_byte_equality_claimed"]


@pytest.mark.parametrize("bad", ["../outside.npz", "/tmp/raw", "frames/../raw", "frames//raw", "./raw", "", "frames\\raw"])
def test_escaping_artifact_rejected(bad):
    evidence = artifacts(); evidence[1]["path"] = bad
    with pytest.raises(ValueError):
        subject.bind_saved_artifacts(subject.assess_headers(observed()), evidence)


def test_no_image_array_ros_carla_or_file_open_in_helper():
    import ast
    from pathlib import Path
    tree = ast.parse(Path(subject.__file__).read_bytes())
    imported = {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
    imported |= {name.name for node in ast.walk(tree) if isinstance(node, ast.Import) for name in node.names}
    assert not imported & {"numpy", "torch", "PIL", "carla", "rclpy"}
    assert not any(isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in {"open", "exec", "eval"} for node in ast.walk(tree))
