"""HH_260929 - Exercise future capture QA with tiny synthetic archived sensor pairs."""

import hashlib
import json
from dataclasses import replace

import numpy as np
from PIL import Image

from scripts.e2e.audit_ctrack_mapping_capture import (
    Policy, REQUIRED_OWNER_SOURCES, ROUTES, TOWN, _route_geometry,
    _station_regression, audit_capture, digest,
)
from scripts.e2e import collect_carla_mapping_ctrack_v2 as ctrack_collector


def _write_owner_evidence(source, route_path, route_sha, summary, policy):
    """HH_260929 - Simulate retained owned-process files without CARLA or GPU."""
    owner_root = source.parent
    (owner_root / "lifecycle").mkdir()
    source_hashes = {}
    for logical in REQUIRED_OWNER_SOURCES:
        archived = owner_root / "provenance" / logical
        archived.parent.mkdir(parents=True, exist_ok=True)
        payload = f"synthetic source {logical}".encode()
        archived.write_bytes(payload)
        source_hashes[logical] = hashlib.sha256(payload).hexdigest()
    pid, port = 81234, 2100
    scenario = "straight"
    live_name = "Carla/Maps/FixtureTown"
    live_xodr_sha = hashlib.sha256(b"synthetic live XODR").hexdigest()
    plan = {
        "capture_mode": "mapping-ctrack-v2",
        "worker_path": ctrack_collector.WORKER_SOURCE,
        "quality": "Epic", "map": "FixtureTown",
        "route_path": str(route_path), "route_sha256": route_sha,
        "host": "127.0.0.1", "port": port,
        "collector_argv": [str(source), str(route_path), "--host", "127.0.0.1",
                           "--port", str(port), "--motion-mode", "basic-agent",
                           "--frames", str(policy.expected_frames),
                           "--ignore-traffic-lights", "--ctrack-straight"],
        "source_sha256": source_hashes,
        "training_data_admitted": False, "independent_dev_approved": False,
        "test_use_authorized": False,
    }
    started = {"server_pid": pid, "server_pgid": pid, "quality": "Epic",
               "map": "FixtureTown", "route_sha256": route_sha, "port": port}
    ready = {"status": "PASS", "mode": "running", "stage": "ready",
             "generation_id": f"expert_{pid}", "owner_pid": pid, "owner_pgid": pid,
             "port": port, "host": "127.0.0.1", "expected_map": "FixtureTown",
             "active_map_basename": "FixtureTown", "active_map_name": live_name,
             "read_only": True}
    result = {"capture_mode": "mapping-ctrack-v2", "exit_code": 0,
              "source_bytes_unchanged_and_archived": True,
              "source_checks": {key: True for key in source_hashes},
              "training_data_admitted": False, "independent_dev_approved": False,
              "test_use_authorized": False}
    pre_actor = {
        "scenario": scenario, "route_path": str(route_path),
        "route_sha256": route_sha, "owner_pid": pid,
        "owner_generation_id": f"expert_{pid}",
        "owner_plan": str(owner_root / "owner_plan.json"),
        "live_map_name": live_name, "live_xodr_sha256": live_xodr_sha,
        "live_xodr_source": "carla.Map.to_opendrive",
        "site_split_v2_sha256": source_hashes["config/site_split_guard_20260929_v2.json"],
        "frozen_v1_collector_sha256": source_hashes["scripts/e2e/collect_carla_mapping_probe.py"],
        "packaged_assets": {}, "dataset_admitted": False, "train_eligible": False,
    }
    for path, value in ((owner_root / "owner_plan.json", plan),
                        (owner_root / "owner_started.json", started),
                        (owner_root / "lifecycle/ready.json", ready),
                        (owner_root / "owner_result.json", result),
                        (source / "pre_actor_provenance.json", pre_actor)):
        path.write_text(json.dumps(value), encoding="utf-8")
    summary.update({"owner_pid": pid, "owner_generation_id": f"expert_{pid}",
                    "live_map_name": live_name, "live_xodr_sha256": live_xodr_sha,
                    "split_operation": "owned_train_capture_pending_qa",
                    "site_split_v2_sha256": pre_actor["site_split_v2_sha256"],
                    "frozen_v1_collector_sha256": pre_actor["frozen_v1_collector_sha256"],
                    "packaged_assets": {}, "dataset_admitted": False,
                    "train_eligible": False, "independent_dev_approved": False,
                    "test_use_authorized": False, "qa_pending": True,
                    "hd_map_approved": False})


def fixture_episode(tmp_path):
    """HH_260929 - Build a tiny route with before, inside and after evidence."""
    source = tmp_path / "mapping"
    frames = source / "frames"
    frames.mkdir(parents=True)
    route_path = tmp_path / "route.json"
    route = {"town": "FixtureTown", "scenario": "straight",
             "coordinate_reference": "base_link",
             "route": [{"x": float(x), "y": 0.0, "distance_m": float(x),
                        "road_option": "STRAIGHT" if 3 <= x <= 8 else "LANEFOLLOW"}
                       for x in range(13)]}
    route_path.write_text(json.dumps(route), encoding="utf-8")
    route_sha = hashlib.sha256(route_path.read_bytes()).hexdigest()
    positions = [0.0, 1.5, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 11.0, 12.0]
    records = []
    for index, x in enumerate(positions):
        stem = f"frames/{index:04d}"
        Image.new("RGB", (4, 4), (45, 45, 45)).save(source / f"{stem}.png")
        depth = np.zeros((4, 4, 4), dtype=np.uint8)
        encoded = round(2.0 * 16777215 / 1000)
        depth[..., 0], depth[..., 1], depth[..., 2] = (
            encoded >> 16, (encoded >> 8) & 255, encoded & 255)
        semantic = np.zeros_like(depth)
        semantic[..., 2] = 1
        semantic[0, 0, 2] = 24
        pose = np.eye(4)
        pose[0, 3] = x + 1.5
        np.savez_compressed(source / f"{stem}.npz", depth=depth,
                            semantic=semantic, world_from_camera=pose)
        records.append({"stem": stem, "timestamp_s": 7.0 + index * .1,
                        "frame": 100 + index, "ego_x": x, "ego_y": 0.0,
                        "ego_yaw_deg": 0.0, "speed_mps": 2.0, "points": 4})
    summary = {
        "status": "COMPLETE_ORACLE_PLUMBING_ONLY", "live_replay_exact_match": True,
        "collision_events": 0, "frame_count": len(records), "town": "FixtureTown",
        "route_sha256": route_sha, "physics_driving": True,
        "motion_mode": "basic-agent", "controller": "CARLA BasicAgent",
        "learned_model_control": False, "oracle_pose": True,
        "oracle_depth": True, "oracle_semantics": True,
        "label_table": {"1": "road", "24": "lane_marking", "2": "sidewalk",
                        "7": "traffic_light_fixture", "8": "traffic_sign", "6": "pole"},
        "cleanup_errors": [], "ignores_traffic_lights": True,
        "travel_m": 12.0,
    }
    policy = Policy(expected_frames=10, frame_wh=(4, 4),
                    minimum_retained_0p25m=9, minimum_lane_positive_frames=2)
    _write_owner_evidence(source, route_path, route_sha, summary, policy)
    (source / "summary.json").write_text(json.dumps(summary), encoding="utf-8")
    (source / "records.json").write_text(json.dumps(records), encoding="utf-8")
    return source, route_path, route_sha, policy


def test_conditional_qa_requires_real_motion_and_never_admits(tmp_path):
    source, route, route_sha, policy = fixture_episode(tmp_path)
    result = audit_capture(source, route, route_sha, "STRAIGHT",
                           town="FixtureTown", policy=policy)
    assert result["status"] == "QA_CONDITIONAL"
    assert result["fail_reasons"] == []
    assert result["metrics"]["retained_positions_0p25m"] == 10
    assert result["metrics"]["inside_window_frames"] == 6
    assert result["metrics"]["ordered_window_coverage"] is True
    assert result["dataset_admitted"] is False
    assert result["independent_per_sensor_timestamp_pose_replay"] is False


def test_versioned_collector_summary_contract_is_accepted_by_archive_auditor(tmp_path):
    # HH_260929 - A completed capture is still only conditionally QA-valid, never TRAIN-ready.
    source, route, route_sha, policy = fixture_episode(tmp_path)
    summary_path = source / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary.update(ctrack_collector.qa_summary_contract())
    summary["status"] = ctrack_collector.SUCCESS_STATUS
    summary["qa_pending"] = True
    summary_path.write_text(json.dumps(summary), encoding="utf-8")
    result = audit_capture(source, route, route_sha, "STRAIGHT",
                           town="FixtureTown", policy=policy)
    assert result["status"] == "QA_CONDITIONAL"
    assert result["dataset_admitted"] is False
    assert summary["dataset_admitted"] is False


def test_owner_result_exit_and_missing_proof_fail_closed(tmp_path):
    # HH_260929 - A sensor-clean archive is insufficient without completed owner cleanup.
    source, route, route_sha, policy = fixture_episode(tmp_path)
    result_path = source.parent / "owner_result.json"
    result = json.loads(result_path.read_text(encoding="utf-8"))
    result["exit_code"] = 1
    result_path.write_text(json.dumps(result), encoding="utf-8")
    failed = audit_capture(source, route, route_sha, "STRAIGHT",
                           town="FixtureTown", policy=policy)
    assert failed["status"] == "FAIL"
    assert "OWNER_RESULT_NOT_CLEAN" in {row["code"] for row in failed["fail_reasons"]}
    result_path.unlink()
    missing = audit_capture(source, route, route_sha, "STRAIGHT",
                            town="FixtureTown", policy=policy)
    assert missing["status"] == "FAIL"
    assert "OWNER_EVIDENCE_MISSING_OR_UNSAFE" in {
        row["code"] for row in missing["fail_reasons"]}


def test_owner_archived_source_and_pre_actor_identity_fail_closed(tmp_path):
    # HH_260929 - The hash ledger and live-map source proof must survive after capture.
    source, route, route_sha, policy = fixture_episode(tmp_path)
    archived = source.parent / "provenance" / ctrack_collector.WORKER_SOURCE
    archived.write_bytes(archived.read_bytes() + b"tampered")
    changed = audit_capture(source, route, route_sha, "STRAIGHT",
                            town="FixtureTown", policy=policy)
    assert changed["status"] == "FAIL"
    assert "OWNER_SOURCE_ARCHIVE" in {row["code"] for row in changed["fail_reasons"]}

    source, route, route_sha, policy = fixture_episode(tmp_path / "second")
    pre_path = source / "pre_actor_provenance.json"
    pre = json.loads(pre_path.read_text(encoding="utf-8"))
    pre["route_sha256"] = "0" * 64
    pre_path.write_text(json.dumps(pre), encoding="utf-8")
    mismatch = audit_capture(source, route, route_sha, "STRAIGHT",
                             town="FixtureTown", policy=policy)
    assert mismatch["status"] == "FAIL"
    assert "PRE_ACTOR_PROVENANCE" in {row["code"] for row in mismatch["fail_reasons"]}


def test_explicit_collision_pose_and_route_identity_fail_reasons(tmp_path):
    source, route, route_sha, policy = fixture_episode(tmp_path)
    summary_path = source / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["collision_events"] = 1
    summary_path.write_text(json.dumps(summary), encoding="utf-8")
    sensor_path = source / "frames/0003.npz"
    with np.load(sensor_path, allow_pickle=False) as sensor:
        arrays = {key: sensor[key] for key in sensor.files}
    arrays["world_from_camera"][0, 3] += 20.0
    np.savez_compressed(sensor_path, **arrays)
    result = audit_capture(source, route, route_sha, "STRAIGHT",
                           town="FixtureTown", policy=policy)
    codes = {item["code"] for item in result["fail_reasons"]}
    assert result["status"] == "FAIL"
    assert {"COLLISION_EVENTS", "POSE_MOUNT_MISMATCH"} <= codes
    assert result["dataset_admitted"] is False
    ctrack = audit_capture(source, route, route_sha, "STRAIGHT",
                           town=TOWN, policy=policy)
    assert {item["code"] for item in ctrack["fail_reasons"]} == {"CTRACK_ROUTE_NOT_PINNED"}
    pinned = ROUTES["straight"]
    wrong_world = audit_capture(source, pinned["path"], pinned["sha256"],
                                pinned["road_option"], town=TOWN)
    assert "CAPTURE_WORLD_ROUTE" in {item["code"] for item in wrong_world["fail_reasons"]}


def test_ctrack_brand_cannot_relax_fixed_policy(tmp_path):
    # HH_260929 - A tiny fixture policy must never issue C-track QA_CONDITIONAL.
    source, _, _, policy = fixture_episode(tmp_path)
    pinned = ROUTES["straight"]
    result = audit_capture(source, pinned["path"], pinned["sha256"],
                           pinned["road_option"], town=TOWN, policy=policy)
    assert result["status"] == "FAIL"
    assert {row["code"] for row in result["fail_reasons"]} == {"CTRACK_POLICY_OVERRIDE"}


def test_missing_depth_lane_and_actual_window_are_rejected(tmp_path):
    source, route, route_sha, policy = fixture_episode(tmp_path)
    sensor_path = source / "frames/0003.npz"
    with np.load(sensor_path, allow_pickle=False) as sensor:
        arrays = {key: sensor[key] for key in sensor.files}
    arrays["depth"][:] = 0
    arrays["semantic"][0, 0, 2] = 1
    np.savez_compressed(sensor_path, **arrays)
    records_path = source / "records.json"
    records = json.loads(records_path.read_text(encoding="utf-8"))
    records[3]["timestamp_s"] += .03
    records[-2]["ego_x"] = 8.0
    records[-1]["ego_x"] = 8.0
    records_path.write_text(json.dumps(records), encoding="utf-8")
    result = audit_capture(source, route, route_sha, "STRAIGHT", town="FixtureTown",
                           policy=replace(policy, minimum_lane_positive_frames=10))
    codes = {item["code"] for item in result["fail_reasons"]}
    assert result["status"] == "FAIL"
    assert {"TIMESTAMP_CADENCE", "DEPTH_INVALID", "LANE_LABEL_COVERAGE",
            "ROUTE_WINDOW_MISSING_AFTER"} <= codes


def test_two_reviewed_ctrack_route_bytes_still_have_turn_windows():
    for scenario, specification in ROUTES.items():
        route_path = specification["path"]
        assert digest(route_path) == specification["sha256"]
        route = json.loads(route_path.read_text(encoding="utf-8"))
        assert route["town"] == TOWN and route["scenario"] == scenario
        _, _, window = _route_geometry(route, specification["road_option"])
        assert window[1] - window[0] >= 5.0


def test_first_valid_window_then_backward_progress_fails(tmp_path):
    source, route, route_sha, policy = fixture_episode(tmp_path)
    records_path, summary_path = source / "records.json", source / "summary.json"
    records = json.loads(records_path.read_text(encoding="utf-8"))
    records[-1]["ego_x"] = 8.0
    records_path.write_text(json.dumps(records), encoding="utf-8")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["travel_m"] = 14.0
    summary_path.write_text(json.dumps(summary), encoding="utf-8")
    sensor_path = source / "frames/0009.npz"
    with np.load(sensor_path, allow_pickle=False) as sensor:
        arrays = {key: sensor[key] for key in sensor.files}
    arrays["world_from_camera"][0, 3] = 9.5
    np.savez_compressed(sensor_path, **arrays)
    result = audit_capture(source, route, route_sha, "STRAIGHT",
                           town="FixtureTown", policy=policy)
    assert result["metrics"]["ordered_window_coverage"] is True
    assert result["metrics"]["inside_window_distinct_positions_0p25m"] >= 3
    assert result["status"] == "FAIL"
    assert "ROUTE_STATION_REGRESSION" in {item["code"] for item in result["fail_reasons"]}


def test_repeated_small_backward_steps_accumulate_route_drawdown():
    step, drawdown = _station_regression([0.0, 3.0, 6.0, 9.0, 8.0, 6.5, 5.0])
    assert step == 1.5
    assert drawdown == 4.0
    assert step <= Policy().maximum_station_regression_m_per_frame
    assert drawdown > Policy().maximum_station_drawdown_m
