#!/usr/bin/env python3
"""HH_260906 - Inspect one pinned nuPlan SQLite member in memory; never load calibration pickles or approve data."""

from __future__ import annotations

import argparse
import base64
from bisect import bisect_left
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import signal
import sqlite3
import stat
import sys
import time
import zipfile

LOGS = frozenset(("2021.05.12.22.00.38_veh-35_01008_01518", "2021.05.12.22.28.35_veh-35_00620_01164",
    "2021.05.12.23.36.44_veh-35_00152_00504", "2021.05.12.23.36.44_veh-35_01133_01535",
    "2021.05.12.23.36.44_veh-35_02035_02387", "2021.05.25.14.16.10_veh-35_01690_02183", "2021.06.03.12.02.06_veh-35_00233_00609"))
CHANNELS = ("CAM_B0", "CAM_F0", "CAM_L0", "CAM_L1", "CAM_L2", "CAM_R0", "CAM_R1", "CAM_R2")
TABLES = frozenset(("camera", "image", "ego_pose", "lidar_pc", "scene", "log", "sqlite_master", "sqlite_schema"))
FUNCTIONS = frozenset(("count", "min", "max", "length", "typeof", "hex"))
MAX_ROWS, MAX_DB_BYTES, MAX_METADATA_BYTES = 100000, 512 * 1024**2, 1024**2
SELECTED_MEMBER = "data/cache/mini/2021.06.03.12.02.06_veh-35_00233_00609.db"


def require(value, message):
    if not value:
        raise ValueError(message)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def regular(path):
    path = Path(path).absolute()
    require(path.is_file() and all(not item.is_symlink() for item in (path, *path.parents)), "expected nonsymlink regular file")
    info = path.stat()
    require(stat.S_ISREG(info.st_mode), "special files are forbidden")
    return {"size_bytes": info.st_size, "mtime_ns": info.st_mtime_ns, "ctime_ns": info.st_ctime_ns,
        "device": info.st_dev, "inode": info.st_ino}


def metadata(path, expected):
    identity = regular(path)
    require(identity["size_bytes"] <= MAX_METADATA_BYTES, "oversized metadata")
    raw = path.read_bytes()
    require(sha(raw) == expected and regular(path) == identity, "metadata identity mismatch")
    return json.loads(raw), {"name": path.name, "sha256": sha(raw), "identity": identity}


def validate_plan(plan):
    require(plan.get("schema") == "nuplan.single_db_metadata_selection.v1", "unexpected selection schema")
    require(plan.get("member") == SELECTED_MEMBER and plan.get("uncompressed_bytes") == 173629440
        and plan.get("compressed_bytes") == 100266146 and plan.get("crc32") == "28c49074"
        and plan.get("compression_method") == 8, "unreviewed member selection")
    require(type(plan.get("maximum_db_uncompressed_bytes")) is int and plan["maximum_db_uncompressed_bytes"] == MAX_DB_BYTES,
        "member bound differs from approval")
    for key, value in (("maximum_address_space_bytes", 2 * 1024**3), ("maximum_cpu_seconds", 60),
        ("external_wall_timeout_seconds", 90), ("database_payloads_allowed", 1), ("camera_payloads_allowed", 0), ("map_payloads_allowed", 0)):
        require(type(plan.get(key)) is int and plan[key] == value, "resource/scope limit changed")
    for key in ("terms_consent_provided", "training_data_approved", "conversion_allowed", "training_allowed", "dataset_export_allowed"):
        require(plan.get(key) is False, "metadata audit cannot grant data permission")
    require(datetime.fromisoformat(plan["declared_at_utc"].replace("Z", "+00:00")) < datetime.now(timezone.utc), "selection was not declared before execution")


def select_member(infos, plan):
    """HH_260906 - Recompute the smallest of exactly seven camera-covered members without touching payloads."""
    candidates = [entry for entry in infos if entry.filename in {"data/cache/mini/" + name + ".db" for name in LOGS}]
    require(len(candidates) == len(LOGS) and len({entry.filename for entry in candidates}) == len(LOGS), "seven-member coverage/uniqueness mismatch")
    selected = min(candidates, key=lambda entry: (entry.file_size, entry.filename))
    require(selected.filename == plan["member"] and selected.file_size == plan["uncompressed_bytes"]
        and selected.compress_size == plan["compressed_bytes"] and f"{selected.CRC:08x}" == plan["crc32"]
        and selected.compress_type == plan["compression_method"], "selection differs from frozen central directory")
    require(0 < selected.file_size <= MAX_DB_BYTES and not selected.is_dir() and not selected.flag_bits & 1
        and not stat.S_ISLNK(selected.external_attr >> 16), "encrypted/link/oversized DB member")
    return selected


def validate_sqlite_bytes(raw):
    require(len(raw) >= 100 and raw[:16] == b"SQLite format 3\0", "not a SQLite3 database")
    page_size = int.from_bytes(raw[16:18], "big")
    page_size = 65536 if page_size == 1 else page_size
    require(page_size in {2**power for power in range(9, 17)} and len(raw) % page_size == 0, "invalid SQLite page dimensions")
    require(raw[18] == raw[19] == 1, "WAL/unsupported database image rejected without header repair")
    return {"page_size": page_size, "payload_pages": len(raw) // page_size,
        "header_page_count": int.from_bytes(raw[28:32], "big"), "sqlite_header_write_read_versions": [raw[18], raw[19]]}


def authorizer(action, first, second, database, trigger):
    """HH_260906 - Permit only plain SELECT, reads of named ordinary tables and a small built-in function list."""
    if trigger is not None:
        return sqlite3.SQLITE_DENY
    if action == sqlite3.SQLITE_SELECT:
        return sqlite3.SQLITE_OK
    # HH_260906 - SQLite reports count(*) reads with an empty column and null database; no attach/temp-table creation is allowed.
    if action == sqlite3.SQLITE_READ and first in TABLES and (database == "main" or database is None and second == ""):
        return sqlite3.SQLITE_OK
    if action == sqlite3.SQLITE_FUNCTION and (second or first or "").lower() in FUNCTIONS:
        return sqlite3.SQLITE_OK
    return sqlite3.SQLITE_DENY


def configure(connection, deadline):
    connection.enable_load_extension(False)
    if hasattr(connection, "setconfig") and hasattr(sqlite3, "SQLITE_DBCONFIG_DEFENSIVE"):
        connection.setconfig(sqlite3.SQLITE_DBCONFIG_DEFENSIVE, True)
    for key, value in (("temp_store", "MEMORY"), ("trusted_schema", "OFF"), ("query_only", "ON")):
        connection.execute("PRAGMA " + key + "=" + value)
    require(connection.execute("PRAGMA temp_store").fetchone()[0] == 2
        and connection.execute("PRAGMA trusted_schema").fetchone()[0] == 0
        and connection.execute("PRAGMA query_only").fetchone()[0] == 1, "SQLite safeguards not applied")
    connection.set_authorizer(authorizer)
    connection.set_progress_handler(lambda: int(time.monotonic() >= deadline), 1000)


def rows(connection, query):
    cursor = connection.execute(query)
    result = cursor.fetchmany(MAX_ROWS + 1)
    require(len(result) <= MAX_ROWS, "query exceeds bounded record denominator")
    return result


def stats(values):
    values = sorted(values)
    if not values:
        return {"count": 0, "minimum": None, "maximum": None, "p50": None, "p95": None, "p99": None}
    return {"count": len(values), "minimum": values[0], "maximum": values[-1],
        **{name: values[max(0, math.ceil(fraction * len(values)) - 1)] for name, fraction in (("p50", .5), ("p95", .95), ("p99", .99))}}


def cadence(timestamps):
    require(all(type(value) is int for value in timestamps), "native timestamps must be integers")
    values = sorted(timestamps)
    gaps = [b - a for a, b in zip(values, values[1:])]
    duration = (values[-1] - values[0]) / 1e6 if len(values) > 1 else 0
    return {"timestamp_count": len(values), "minimum_timestamp_us": values[0] if values else None,
        "maximum_timestamp_us": values[-1] if values else None, "duration_seconds": duration,
        "effective_hz": (len(values) - 1) / duration if duration > 0 else None,
        "duplicate_timestamp_count": sum(value == 0 for value in gaps), "gap_us": stats(gaps)}


def camera_alignment(by_channel):
    front = by_channel.get("CAM_F0", [])
    differences, skews = {channel: [] for channel in CHANNELS}, []
    for timestamp in front:
        chosen = []
        for channel in CHANNELS:
            values = by_channel.get(channel, [])
            if not values:
                continue
            index = bisect_left(values, timestamp)
            options = values[max(0, index - 1):index + 1]
            nearest = min(options, key=lambda value: (abs(value - timestamp), value))
            differences[channel].append(nearest - timestamp)
            chosen.append(nearest)
        if len(chosen) == 8:
            skews.append(max(chosen) - min(chosen))
    return {"reference_channel": "CAM_F0", "reference_anchor_count": len(front),
        "method": "Nearest native timestamp independently per all eight channels; ties choose earlier. Offline diagnostic, not a causal six-camera input contract.",
        "per_channel_signed_offset_us": {channel: {"offset": stats(values), "absolute_offset": stats([abs(x) for x in values]),
            "later_than_reference_count": sum(x > 0 for x in values)} for channel, values in differences.items()},
        "all_eight_bundle_skew_us": stats(skews), "all_eight_skew_above_20000us_count": sum(x > 20000 for x in skews),
        "six_camera_mapping_status": "NOT_SELECTED_REQUIRES_NUMERIC_CALIBRATION_AND_FOV_REVIEW", "training_qualified": False}


def inspect_connection(connection):
    schema = rows(connection, "SELECT name, type, sql FROM sqlite_schema ORDER BY name")
    require(not any(kind == "view" or (isinstance(sql, str) and "CREATE VIRTUAL TABLE" in sql.upper()) for _, kind, sql in schema), "views/virtual tables are forbidden")
    actual_tables = {name for name, kind, _ in schema if kind == "table"}
    require(TABLES - {"sqlite_master", "sqlite_schema"} <= actual_tables, "required native tables missing")
    cameras = rows(connection, "SELECT token, channel, model, width, height, translation, rotation, intrinsic, distortion FROM camera")
    images = rows(connection, "SELECT token, camera_token, ego_pose_token, timestamp, filename_jpg FROM image ORDER BY timestamp, token")
    poses = rows(connection, "SELECT token, timestamp, x, y, z, qw, qx, qy, qz, vx, vy, vz, acceleration_x, acceleration_y, acceleration_z, angular_rate_x, angular_rate_y, angular_rate_z FROM ego_pose ORDER BY timestamp, token")
    lidar = rows(connection, "SELECT token, ego_pose_token, scene_token, timestamp FROM lidar_pc ORDER BY timestamp, token")
    scenes = rows(connection, "SELECT token, goal_ego_pose_token, roadblock_ids FROM scene ORDER BY token")
    logs = rows(connection, "SELECT token, map_version FROM log ORDER BY token")
    channel_map = {row[0]: row[1] for row in cameras}
    require(len(channel_map) == len(cameras) and set(channel_map.values()) == set(CHANNELS) and len(cameras) == 8, "expected eight unique native camera channels")
    pose_map = {row[0]: row for row in poses}
    require(len(pose_map) == len(poses) and len({row[0] for row in images}) == len(images), "duplicate pose/image tokens")
    by_channel = {channel: [] for channel in CHANNELS}
    image_pose_offsets, missing_camera, missing_pose = [], 0, 0
    bad_filename_count = 0
    for _, camera_token, ego_token, stamp, filename in images:
        if camera_token not in channel_map:
            missing_camera += 1
        else:
            by_channel[channel_map[camera_token]].append(stamp)
        if ego_token not in pose_map:
            missing_pose += 1
        else:
            image_pose_offsets.append(stamp - pose_map[ego_token][1])
        name = Path(filename) if isinstance(filename, str) else None
        bad_filename_count += int(name is None or name.is_absolute() or ".." in name.parts or name.suffix != ".jpg")
    opaque = []
    for token, channel, model, width, height, *blobs in cameras:
        fields = {}
        for name, blob in zip(("translation", "rotation", "intrinsic", "distortion"), blobs):
            require(blob is None or isinstance(blob, bytes) and len(blob) <= 65536, "calibration field is not a bounded opaque BLOB")
            fields[name] = {"storage_type": "null" if blob is None else "blob", "size_bytes": len(blob) if blob is not None else None,
                "sha256": sha(blob) if blob is not None else None, "deserialized": False}
        opaque.append({"channel": channel, "model": model, "width": width, "height": height, "fields": fields})
    lidar_missing_pose = sum(row[1] not in pose_map for row in lidar)
    lidar_pose_offsets = [row[3] - pose_map[row[1]][1] for row in lidar if row[1] in pose_map]
    scene_ids = {row[0] for row in scenes}
    scene_summary = []
    for token, goal, route in scenes:
        scene_times = [row[3] for row in lidar if row[2] == token]
        require(route is None or isinstance(route, str), "scene route must remain native text or null")
        scene_summary.append({"scene_token_hex": token.hex(), "lidar_count": len(scene_times), "goal_pose_join_available": goal in pose_map,
            "goal_timestamp_minus_scene_first_lidar_us": pose_map[goal][1] - min(scene_times) if goal in pose_map and scene_times else None,
            "roadblock_text_present": bool(route), "roadblock_id_count": len(route.split()) if route else 0,
            "roadblock_text_sha256": sha(route.encode()) if route is not None else None})
    scalar_fields = ("x", "y", "z", "qw", "qx", "qy", "qz", "vx", "vy", "vz", "acceleration_x", "acceleration_y", "acceleration_z", "angular_rate_x", "angular_rate_y", "angular_rate_z")
    scalar_invalid = {name: sum(type(row[i + 2]) not in (int, float) or not math.isfinite(row[i + 2]) for row in poses) for i, name in enumerate(scalar_fields)}
    return {"schema_tables": sorted(actual_tables), "schema_sql_sha256": sha(json.dumps(schema, sort_keys=True).encode()),
        "counts": {"camera": len(cameras), "image": len(images), "ego_pose": len(poses), "lidar_pc": len(lidar), "scene": len(scenes), "log": len(logs)},
        "map_versions": sorted({row[1] for row in logs}), "camera_native_cadence": {key: cadence(value) for key, value in by_channel.items()},
        "ego_pose_native_cadence": cadence([row[1] for row in poses]), "lidar_pc_native_cadence": cadence([row[3] for row in lidar]),
        "joins": {"image_missing_camera_count": missing_camera, "image_missing_ego_pose_count": missing_pose,
            "lidar_missing_ego_pose_count": lidar_missing_pose, "lidar_missing_scene_count": sum(row[2] not in scene_ids for row in lidar),
            "image_minus_linked_ego_pose_timestamp_us": stats(image_pose_offsets), "lidar_minus_linked_ego_pose_timestamp_us": stats(lidar_pose_offsets),
            "unsafe_or_non_jpeg_filename_count": bad_filename_count, "filename_exact_camera_archive_membership": "NOT_RECHECKED_NO_CAMERA_ARCHIVE_PAYLOAD_OR_DIRECTORY_SCAN"},
        "camera_alignment": camera_alignment(by_channel), "calibration_opaque_fields": opaque, "ego_scalar_invalid_counts": scalar_invalid,
        "scene_route_goal_metadata": scene_summary,
        "route_availability": "NATIVE_SCENE_ROADBLOCK_IDS_AND_GOAL_JOIN_ONLY; no availability-before-anchor timestamp or map centerline geometry verified",
        "stop_intent_supervision": "NOT_DEFINED; mission goal is not proof of an observed causal stop instruction",
        "numeric_calibration": "NOT_DECODED; official serializers use pickle; opaque hashes do not establish camera geometry"}


def write_json(path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False); stream.write("\n")


def run(args):
    plan_raw = base64.b64decode(args.selection_plan_base64, validate=True)
    require(len(plan_raw) <= MAX_METADATA_BYTES, "selection plan too large")
    plan = json.loads(plan_raw); validate_plan(plan)
    personal = Path(args.personal_root).absolute()
    archive_root, output = Path(args.archive_root).absolute(), Path(args.output_dir).absolute()
    require(archive_root == personal / "dataset/raw/nuplan/v1.1-mini/research-only-pending-terms-review", "unexpected raw root")
    require(output.is_relative_to(personal / "portable_e2e/runs/diagnostics") and not output.exists()
        and all(not p.is_symlink() for p in (output, *output.parents)), "unsafe or existing output root")
    archive_path = archive_root / plan["archive_identity"]["name"]
    before = regular(archive_path)
    require(before == {k: v for k, v in plan["archive_identity"].items() if k != "name"}, "archive stat changed since selection")
    full, full_proof = metadata(archive_root / "verification/nuplan-v1.1_mini.zip.full-audit.json", plan["previous_archive_full_audit_sha256"])
    subset, subset_proof = metadata(archive_root / "verification/nuplan-v1.1-mini-db-camera0-subset-inspection.json", plan["previous_subset_report_sha256"])
    require(full["status"] == subset["status"] == "PASS" and full["actual_sha256"] == plan["previous_archive_content_sha256"]
        and full["actual_size"] == before["size_bytes"] and full["payload_crc"]["status"] == "PASS", "previous archive verification mismatch")
    source_sha = globals().get("EXECUTED_SOURCE_SHA256")
    if source_sha is None:
        source_sha = sha(Path(__file__).read_bytes())
    output.mkdir(parents=True, exist_ok=False)
    with (output / "selection_plan.json").open("xb") as stream:
        stream.write(plan_raw)
    result = {"schema": "nuplan.single_db_metadata_probe.v1", "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "selection_plan_sha256": sha(plan_raw), "source_sha256": source_sha, "archive_pre_identity": before,
        "prior_verification": [full_proof, subset_proof], "archive_whole_content_rehashed_this_run": False,
        "archive_previous_content_sha256": full["actual_sha256"], "read_only_original_archives": True,
        "extracted": False, "pickle_or_orm_imported": False, "database_payloads_opened": 0, "camera_payloads_read": 0, "map_payloads_read": 0,
        "training_data_approved": False, "terms_consent_provided": False, "dataset_converted": False,
        "trained_model": False, "dataset_exported": False, "python_version": sys.version.split()[0], "sqlite_version": sqlite3.sqlite_version}
    connection = None
    try:
        require(hasattr(sqlite3.Connection, "deserialize"), "SQLite memory deserialize unavailable; no disk fallback")
        with zipfile.ZipFile(archive_path) as archive:
            selected = select_member(archive.infolist(), plan)
            result["member_selection_verified_before_payload"] = {"member": selected.filename, "file_size": selected.file_size,
                "compress_size": selected.compress_size, "crc32": f"{selected.CRC:08x}"}
            result["database_payloads_opened"] = 1
            with archive.open(selected) as stream:
                raw = stream.read(MAX_DB_BYTES + 1)
            require(len(raw) == selected.file_size, "member length mismatch")
        result["member_sha256"] = sha(raw)
        result["member_crc_verified_to_eof"] = True
        result["sqlite_header"] = validate_sqlite_bytes(raw)
        connection = sqlite3.connect(":memory:")
        connection.deserialize(raw)
        del raw
        configure(connection, time.monotonic() + 55)
        result["sqlite_safeguards"] = {"in_memory": True, "query_only": True, "trusted_schema": False,
            "temp_store_memory": True, "extension_loading": False, "select_only_authorizer": True,
            "allowed_tables": sorted(TABLES), "allowed_functions": sorted(FUNCTIONS)}
        result["metadata"] = inspect_connection(connection)
        result["status"] = "METADATA_INSPECTED_NOT_READY"
    except Exception as error:
        result.update(status="FAILED_METADATA_DIAGNOSTIC", error_type=type(error).__name__, error=str(error))
    finally:
        if connection is not None: connection.close()
        result["archive_post_identity"] = regular(archive_path)
        result["archive_identity_unchanged"] = result["archive_post_identity"] == before
        if not result["archive_identity_unchanged"]: result["status"] = "FAILED_METADATA_DIAGNOSTIC"
        metadata(archive_root / "verification/nuplan-v1.1_mini.zip.full-audit.json", full_proof["sha256"])
        metadata(archive_root / "verification/nuplan-v1.1-mini-db-camera0-subset-inspection.json", subset_proof["sha256"])
        result["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
        result["peak_rss_kib"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        write_json(output / "report.json", result)
        payloads = sorted(output.iterdir())
        with (output / "SHA256SUMS").open("x") as stream:
            stream.write("".join(sha(p.read_bytes()) + "  " + p.name + "\n" for p in payloads))
    print(json.dumps({"status": result["status"], "member_sha256": result.get("member_sha256"),
        "report_sha256": sha((output / "report.json").read_bytes()), "peak_rss_kib": result["peak_rss_kib"]}))
    return 0 if result["status"] == "METADATA_INSPECTED_NOT_READY" else 2


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    for name in ("personal-root", "archive-root", "output-dir", "selection-plan-base64"):
        parser.add_argument("--" + name, required=True)
    args = parser.parse_args()
    resource.setrlimit(resource.RLIMIT_AS, (2 * 1024**3, 2 * 1024**3))
    resource.setrlimit(resource.RLIMIT_CPU, (60, 60))
    # HH_260906 - An external 90-second timeout is also required; this earlier alarm can retain a normal failure report.
    def timeout_handler(*_): raise TimeoutError("bounded metadata-probe wall deadline")
    signal.signal(signal.SIGALRM, timeout_handler); signal.alarm(85)
    try: return run(args)
    finally: signal.alarm(0)


if __name__ == "__main__":
    raise SystemExit(main())
