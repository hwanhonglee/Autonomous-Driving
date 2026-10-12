"""HH_260906 - Exercise bounded SQLite metadata reads using local synthetic memory fixtures, never downloaded DB payloads."""

import copy
import argparse
import base64
import json
import pickle
from pathlib import Path
import sqlite3
import time
import zipfile

import pytest

from scripts.e2e import inspect_nuplan_single_db as audit


def fixture_connection():
    connection = sqlite3.connect(":memory:")
    connection.executescript("""
    CREATE TABLE camera(token BLOB, channel TEXT, model TEXT, width INTEGER, height INTEGER, translation BLOB, rotation BLOB, intrinsic BLOB, distortion BLOB);
    CREATE TABLE image(token BLOB, camera_token BLOB, ego_pose_token BLOB, timestamp INTEGER, filename_jpg TEXT);
    CREATE TABLE ego_pose(token BLOB, timestamp INTEGER, x REAL, y REAL, z REAL, qw REAL, qx REAL, qy REAL, qz REAL, vx REAL, vy REAL, vz REAL, acceleration_x REAL, acceleration_y REAL, acceleration_z REAL, angular_rate_x REAL, angular_rate_y REAL, angular_rate_z REAL);
    CREATE TABLE lidar_pc(token BLOB, ego_pose_token BLOB, scene_token BLOB, timestamp INTEGER);
    CREATE TABLE scene(token BLOB, goal_ego_pose_token BLOB, roadblock_ids TEXT);
    CREATE TABLE log(token BLOB, map_version TEXT);
    """)
    for n in range(3):
        token = bytes([n + 1])
        connection.execute("INSERT INTO ego_pose VALUES(" + ",".join("?" for _ in range(18)) + ")", [token, 1000000 + 100000 * n, *([0.] * 16)])
        connection.execute("INSERT INTO lidar_pc VALUES(?,?,?,?)", (token, token, b"scene", 1000007 + 100000 * n))
    for c, channel in enumerate(audit.CHANNELS):
        token = bytes([c + 10])
        # HH_260906 - These bytes are intentionally not a valid pickle; numeric calibration must never be decoded here.
        connection.execute("INSERT INTO camera VALUES(?,?,?,?,?,?,?,?,?)", (token, channel, "pinhole", 2000, 1200, b"opaque", b"opaque", b"opaque", b"opaque"))
        for n in range(3):
            connection.execute("INSERT INTO image VALUES(?,?,?,?,?)", (bytes([c, n]), token, bytes([n + 1]), 1000000 + 100000 * n + c * 1000, channel + "/frame" + str(n) + ".jpg"))
    connection.execute("INSERT INTO scene VALUES(?,?,?)", (b"scene", b"\x03", "10 20 30"))
    connection.execute("INSERT INTO log VALUES(?,?)", (b"log", "map-version"))
    connection.commit()
    return connection


def configured(connection=None):
    connection = connection or fixture_connection()
    audit.configure(connection, time.monotonic() + 10)
    return connection


def test_complete_plain_metadata_retains_native_time_differences_and_opaque_blobs(monkeypatch):
    monkeypatch.setattr(pickle, "loads", lambda *_: pytest.fail("pickle must never execute"))
    connection = configured()
    result = audit.inspect_connection(connection)
    assert result["counts"] == dict(camera=8, image=24, ego_pose=3, lidar_pc=3, scene=1, log=1)
    assert result["joins"]["lidar_minus_linked_ego_pose_timestamp_us"]["minimum"] == 7
    assert result["joins"]["image_minus_linked_ego_pose_timestamp_us"]["maximum"] == 7000
    assert all(c["effective_hz"] == 10 for c in result["camera_native_cadence"].values())
    assert result["camera_alignment"]["all_eight_bundle_skew_us"]["maximum"] == 7000
    assert result["scene_route_goal_metadata"][0]["goal_timestamp_minus_scene_first_lidar_us"] == 199993
    assert all(field["deserialized"] is False for c in result["calibration_opaque_fields"] for field in c["fields"].values())
    assert result["camera_alignment"]["training_qualified"] is False
    connection.close()


@pytest.mark.parametrize("query", ["ATTACH DATABASE ':memory:' AS other", "CREATE TABLE forbidden(x)", "DROP TABLE image",
    "INSERT INTO log VALUES(NULL, NULL)", "UPDATE log SET map_version='changed'", "DELETE FROM image",
    "PRAGMA query_only=OFF", "PRAGMA writable_schema=ON", "SELECT load_extension('no-file')",
    "SELECT randomblob(100)", "SELECT * FROM sqlite_temp_master"])
def test_authorizer_rejects_file_write_and_unlisted_function_surfaces(query):
    connection = configured()
    with pytest.raises(sqlite3.DatabaseError): connection.execute(query).fetchall()
    assert connection.execute("SELECT count(*) FROM image").fetchone()[0] == 24
    connection.close()


def test_views_are_not_executed_and_rejected():
    connection = fixture_connection()
    connection.execute("CREATE VIEW v AS SELECT 1")
    configured(connection)
    with pytest.raises(ValueError, match="views/virtual"): audit.inspect_connection(connection)
    connection.close()


@pytest.mark.parametrize("declaration", ["CREATE  VIRTUAL TABLE synthetic USING fts5(x)", "CREATE\nVIRTUAL\nTABLE synthetic USING fts5(x)", "CREATE /* marker */ VIRTUAL TABLE synthetic USING fts5(x)"])
def test_virtual_table_declarations_with_nonstandard_spacing_are_rejected(declaration):
    connection = fixture_connection(); connection.execute(declaration); configured(connection)
    with pytest.raises(ValueError, match="views/virtual"): audit.inspect_connection(connection)
    connection.close()


def test_unknown_read_table_is_denied():
    connection = fixture_connection(); connection.execute("CREATE TABLE unknown(x)"); configured(connection)
    with pytest.raises(sqlite3.DatabaseError): connection.execute("SELECT * FROM unknown").fetchall()
    connection.close()


def test_missing_native_pose_join_is_retained_not_synthesized():
    connection = fixture_connection(); connection.execute("DELETE FROM ego_pose WHERE token=?", (b"\x02",)); connection.commit(); configured(connection)
    result = audit.inspect_connection(connection)
    assert result["joins"]["image_missing_ego_pose_count"] == 8
    assert result["joins"]["lidar_missing_ego_pose_count"] == 1
    assert result["joins"]["image_minus_linked_ego_pose_timestamp_us"]["count"] == 16
    connection.close()


def test_invalid_ego_scalar_is_counted_not_replaced():
    connection = fixture_connection(); connection.execute("UPDATE ego_pose SET vx=NULL WHERE token=?", (b"\x02",)); connection.commit(); configured(connection)
    assert audit.inspect_connection(connection)["ego_scalar_invalid_counts"]["vx"] == 1
    connection.close()


def test_null_or_oversized_calibration_is_not_loaded():
    connection = fixture_connection(); connection.execute("UPDATE camera SET intrinsic=?", (b"x" * 65537,)); connection.commit(); configured(connection)
    with pytest.raises(ValueError, match="bounded opaque"): audit.inspect_connection(connection)
    connection.close()


def test_progress_deadline_interrupts_sql():
    connection = fixture_connection(); audit.configure(connection, time.monotonic() - 1)
    connection.set_progress_handler(lambda: 1, 1)
    with pytest.raises(sqlite3.DatabaseError): connection.execute("SELECT count(*) FROM image").fetchall()
    connection.close()


@pytest.mark.parametrize("timestamps", [[], [10], [10, 10], [1000000, 1100000, 1200000]])
def test_cadence_denominators_do_not_retime_or_drop_duplicates(timestamps):
    result = audit.cadence(timestamps)
    assert result["timestamp_count"] == len(timestamps)
    assert result["gap_us"]["count"] == max(0, len(timestamps) - 1)


@pytest.mark.parametrize("timestamps", [[True], [1.0], [None], [float("nan")]])
def test_non_native_timestamp_types_are_rejected(timestamps):
    with pytest.raises(ValueError, match="integers"): audit.cadence(timestamps)


def test_nearest_pair_ties_are_earlier_and_future_camera_offsets_disclosed():
    values = {channel: [90, 110] for channel in audit.CHANNELS}; values["CAM_F0"] = [100]
    result = audit.camera_alignment(values)
    assert result["per_channel_signed_offset_us"]["CAM_B0"]["offset"]["minimum"] == -10
    values["CAM_B0"] = [110]
    assert audit.camera_alignment(values)["per_channel_signed_offset_us"]["CAM_B0"]["later_than_reference_count"] == 1


def test_member_selection_is_deterministic_and_rejects_duplicates():
    infos = []
    for name in audit.LOGS:
        entry = zipfile.ZipInfo("data/cache/mini/" + name + ".db")
        entry.file_size, entry.compress_size, entry.CRC, entry.compress_type = 200000000, 100266146, int("28c49074", 16), 8
        if entry.filename == audit.SELECTED_MEMBER: entry.file_size = 173629440
        infos.append(entry)
    plan = dict(member=audit.SELECTED_MEMBER, uncompressed_bytes=173629440, compressed_bytes=100266146, crc32="28c49074", compression_method=8)
    assert audit.select_member(infos, plan).filename == audit.SELECTED_MEMBER
    with pytest.raises(ValueError, match="uniqueness"): audit.select_member(infos + [infos[0]], plan)
    tampered = copy.deepcopy(plan); tampered["uncompressed_bytes"] -= 1
    with pytest.raises(ValueError, match="frozen"): audit.select_member(infos, tampered)


@pytest.mark.parametrize("version", [0, 2, 3])
def test_wal_database_header_is_rejected_without_repair(version):
    raw = bytearray(512); raw[:16] = b"SQLite format 3\0"; raw[16:18] = (512).to_bytes(2, "big"); raw[18:20] = bytes([version, version])
    with pytest.raises(ValueError, match="WAL"): audit.validate_sqlite_bytes(bytes(raw))
    assert raw[18] == version


def test_valid_header_requires_whole_pages():
    raw = bytearray(512); raw[:16] = b"SQLite format 3\0"; raw[16:18] = (512).to_bytes(2, "big"); raw[18:20] = b"\x01\x01"
    assert audit.validate_sqlite_bytes(bytes(raw))["payload_pages"] == 1
    with pytest.raises(ValueError, match="dimensions"): audit.validate_sqlite_bytes(bytes(raw) + b"x")


def test_symlink_and_metadata_hash_mismatch_are_rejected(tmp_path):
    path = tmp_path / "original.json"; path.write_text("{}\n"); alias = tmp_path / "alias.json"; alias.symlink_to(path)
    with pytest.raises(ValueError, match="nonsymlink"): audit.regular(alias)
    with pytest.raises(ValueError, match="identity mismatch"): audit.metadata(path, "0" * 64)


def test_module_never_imports_pickle_orm_or_model():
    # HH_260906 - Pickle above is test-only and patched to fail; the production worker has no serializer/model import surface.
    source = Path(audit.__file__).read_text()
    assert "import pickle" not in source and "from pickle" not in source
    assert "import torch" not in source and "import sqlalchemy" not in source
    assert "extractall(" not in source and "extract(" not in source


@pytest.mark.parametrize("failure", ["unsupported_deserialize", "bad_header", "archive_disappears", "archive_changes", "proof_disappears", "proof_changes"])
def test_bounded_failure_report_and_original_selection_are_retained_without_extraction(tmp_path, monkeypatch, failure):
    # HH_260906 - Mock only the fixed real-data selection for a tiny local ZIP; production retains its exact reviewed member guard.
    personal = tmp_path / "personal"
    root = personal / "dataset/raw/nuplan/v1.1-mini/research-only-pending-terms-review"
    verification = root / "verification"; verification.mkdir(parents=True)
    archive_path = root / "nuplan-v1.1_mini.zip"
    with zipfile.ZipFile(archive_path, "w") as archive: archive.writestr("synthetic.db", b"not sqlite")
    full = {"status": "PASS", "actual_sha256": audit.sha(archive_path.read_bytes()), "actual_size": archive_path.stat().st_size, "payload_crc": {"status": "PASS"}}
    full_raw = json.dumps(full).encode(); subset_raw = b'{"status":"PASS"}'
    (verification / "nuplan-v1.1_mini.zip.full-audit.json").write_bytes(full_raw)
    (verification / "nuplan-v1.1-mini-db-camera0-subset-inspection.json").write_bytes(subset_raw)
    plan = {"archive_identity": {"name": archive_path.name, **audit.regular(archive_path)},
        "previous_archive_full_audit_sha256": audit.sha(full_raw), "previous_archive_content_sha256": full["actual_sha256"],
        "previous_subset_report_sha256": audit.sha(subset_raw)}
    plan_raw = json.dumps(plan).encode()
    monkeypatch.setattr(audit, "validate_plan", lambda value: None)
    monkeypatch.setattr(audit, "select_member", lambda infos, plan: infos[0])
    connection_type = type("SyntheticConnection", (), {} if failure == "unsupported_deserialize" else {"deserialize": None})
    monkeypatch.setattr(audit.sqlite3, "Connection", connection_type)
    if failure not in ("unsupported_deserialize", "bad_header"):
        def mutate(_):
            path = archive_path if failure.startswith("archive") else verification / "nuplan-v1.1_mini.zip.full-audit.json"
            if failure.endswith("disappears"): path.unlink()
            else: path.write_bytes(b"changed")
            raise ValueError("synthetic parse-time failure retained")
        monkeypatch.setattr(audit, "validate_sqlite_bytes", mutate)
    output = personal / "portable_e2e/runs/diagnostics/failure"
    args = argparse.Namespace(personal_root=str(personal), archive_root=str(root), output_dir=str(output), selection_plan_base64=base64.b64encode(plan_raw).decode())
    assert audit.run(args) == 2
    report = json.loads((output / "report.json").read_text())
    assert report["status"] == "FAILED_METADATA_DIAGNOSTIC"
    assert report["database_payloads_opened"] == (0 if failure == "unsupported_deserialize" else 1)
    assert report["archive_identity_unchanged"] is (not failure.startswith("archive"))
    if failure.startswith("proof") or failure == "archive_disappears": assert report["postcheck_errors"]
    assert (output / "selection_plan.json").read_bytes() == plan_raw
    assert set(p.name for p in output.iterdir()) == {"selection_plan.json", "report.json", "SHA256SUMS"}
    assert not list(personal.rglob("*.db"))
    for line in (output / "SHA256SUMS").read_text().splitlines():
        digest, name = line.split("  ", 1)
        assert name != "SHA256SUMS" and audit.sha((output / name).read_bytes()) == digest
