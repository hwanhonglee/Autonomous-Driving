"""HH_260906 - Test the Epic Town05 DEV source audit with small local evidence trees."""

import importlib.util
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools/e2e/audit_map_town05_dev_capture.py"
SPEC = importlib.util.spec_from_file_location("audit_map_town05_dev_capture", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def fixture_capture(tmp_path: Path, positions: list[float]) -> tuple[Path, Path, str]:
    """HH_260906 - Construct a pinned source with real RGB and semantic pair bytes."""
    owner = tmp_path / "epic_dev"
    source = owner / "mapping"
    (source / "frames").mkdir(parents=True)
    route_path = tmp_path / "town05_route.json"
    write_json(route_path, {"town": "Town05_Opt", "scenario": "straight",
                            "coordinate_reference": "base_link", "route": [{"x": 0}, {"x": 1}]})
    route_sha = audit.common.digest(route_path)
    archive = owner / "provenance/scripts/e2e/collect_carla_mapping_probe.py"
    archive.parent.mkdir(parents=True)
    archive.write_text("archived source", encoding="utf-8")
    source_name = "scripts/e2e/collect_carla_mapping_probe.py"
    write_json(owner / "owner_plan.json", {
        "route_path": str(route_path), "route_sha256": route_sha,
        "map": "Town05_Opt", "quality": "Epic", "capture_mode": "mapping-probe",
        "source_bytes_archived": True, "learned_model_control": False,
        "host": "127.0.0.1", "port": 2100,
        "source_sha256": {source_name: audit.common.digest(archive)},
        "collector_argv": [str(source), str(route_path), "--host", "127.0.0.1",
                           "--port", "2100", "--frames", str(len(positions)),
                           "--motion-mode", "basic-agent", "--ignore-traffic-lights",
                           "--town05-generalization-dev"],
    })
    write_json(owner / "owner_started.json", {
        "route_sha256": route_sha, "map": "Town05_Opt", "quality": "Epic",
        "port": 2100, "server_pid": 17, "server_pgid": 17,
        "learned_model_control": False,
    })
    write_json(owner / "owner_result.json", {
        "capture_mode": "mapping-probe", "exit_code": 0,
        "source_bytes_unchanged_and_archived": True,
        "source_checks": {source_name: True}, "external_source_checks": {},
        "learned_model_control": False,
    })
    write_json(source / "summary.json", {
        "route_sha256": route_sha, "town": "Town05_Opt",
        "status": "COMPLETE_ORACLE_PLUMBING_ONLY",
        "research_split_intent": "town05_independent_dev_candidate",
        "frame_count": len(positions), "physics_driving": True,
        "motion_mode": "basic-agent", "learned_model_control": False,
        "collision_events": 0, "live_replay_exact_match": True,
        "cleanup_errors": [], "ignores_traffic_lights": True,
        "oracle_pose": True, "oracle_semantics": True,
        "hd_map_approved": False,
        "label_table": {str(tag): label for tag, label in audit.common.TAG_TO_LABEL.items()},
        "travel_m": 1.0, "agent_goal_reached": False,
    })
    for stage in ("ready", "after_capture", "stopped"):
        write_json(owner / "lifecycle" / f"{stage}.json", {
            "stage": stage, "status": "PASS", "expected_map": "Town05_Opt",
            "owner_pid": 17, "owner_pgid": 17, "host": "127.0.0.1", "port": 2100,
            "active_map_basename": "Town05_Opt" if stage != "stopped" else None,
            "port_released": stage == "stopped",
        })
    records = []
    for index, x in enumerate(positions):
        stem = f"frames/{index:04d}"
        Image.new("RGB", (4, 3), (index, 0, 0)).save(source / f"{stem}.png")
        semantic = np.zeros((3, 4, 4), dtype=np.uint8)
        semantic[0, 0, 2], semantic[0, 1, 2], semantic[0, 2, 2] = 1, 24, 8
        np.savez_compressed(source / f"{stem}.npz", semantic=semantic)
        records.append({"stem": stem, "frame": 100 + index,
                        "timestamp_s": 1.0 + index * 0.1,
                        "ego_x": x, "ego_y": 0.0, "ego_yaw_deg": 0.0,
                        "speed_mps": 1.0})
    write_json(source / "records.json", records)
    return owner, route_path, route_sha


def test_epic_source_pixels_and_retention(tmp_path):
    owner, route, sha = fixture_capture(tmp_path, [0.0, 0.3, 0.6])
    result = audit.build_summary(owner, route, sha, expected_frames=3,
                                 frame_wh=(4, 3), minimum_retained=2)
    assert result["status"] == "CAPTURE_AUDIT_PASS_NOT_ADMITTED"
    assert result["retained_frames_0p25m"] == 3
    assert result["retained_class_pixels"] == [27, 3, 0, 3, 0, 3, 0]
    assert result["retained_sign_positive_frames"] == 3
    assert result["provenance"]["archived_source_count"] == 1
    assert result["provenance"]["agent_goal_reached"] is False
    assert result["dataset_admitted"] is False


@pytest.mark.parametrize("mutation,pattern", [
    ("low", "physical Epic DEV owner proof failed"),
    ("bad_archive", "archived source differs"),
    ("scope", "collector DEV scope differs"),
    ("collision", "physical Epic DEV owner proof failed"),
    ("lifecycle", "CARLA lifecycle failed"),
])
def test_owner_rejects_provenance_regression(tmp_path, mutation, pattern):
    owner, route, sha = fixture_capture(tmp_path, [0.0, 0.3])
    if mutation == "low":
        path = owner / "owner_started.json"
        value = audit.common.read_json(path); value["quality"] = "Low"; write_json(path, value)
    elif mutation == "bad_archive":
        (owner / "provenance/scripts/e2e/collect_carla_mapping_probe.py").write_text("changed")
    elif mutation == "scope":
        path = owner / "owner_plan.json"
        value = audit.common.read_json(path)
        value["collector_argv"][-1] = "--unbound"
        write_json(path, value)
    elif mutation == "collision":
        path = owner / "mapping/summary.json"
        value = audit.common.read_json(path); value["collision_events"] = 1; write_json(path, value)
    else:
        path = owner / "lifecycle/stopped.json"
        value = audit.common.read_json(path); value["port_released"] = False; write_json(path, value)
    with pytest.raises(ValueError, match=pattern):
        audit.audit_owner(owner, route, sha, expected_frames=2)


def test_incomplete_pairs_and_low_distinct_positions(tmp_path):
    owner, route, sha = fixture_capture(tmp_path, [0.0, 0.1, 0.2])
    result = audit.build_summary(owner, route, sha, expected_frames=3,
                                 frame_wh=(4, 3), minimum_retained=2)
    assert result["status"] == "REJECTED_INSUFFICIENT_DISTINCT_POSITIONS"
    assert result["retained_frames_0p25m"] == 1
    (owner / "mapping/frames/0002.npz").unlink()
    with pytest.raises(ValueError, match="native pair file count differs"):
        audit.build_summary(owner, route, sha, expected_frames=3,
                            frame_wh=(4, 3), minimum_retained=2)


def test_bad_native_cadence_is_rejected(tmp_path):
    owner, route, sha = fixture_capture(tmp_path, [0.0, 0.3, 0.6])
    path = owner / "mapping/records.json"
    rows = audit.common.read_json(path)
    rows[2]["timestamp_s"] += .02
    write_json(path, rows)
    with pytest.raises(ValueError, match="native 10 Hz source cadence differs"):
        audit.build_summary(owner, route, sha, expected_frames=3,
                            frame_wh=(4, 3), minimum_retained=2)
