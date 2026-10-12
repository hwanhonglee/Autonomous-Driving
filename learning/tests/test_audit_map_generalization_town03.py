"""HH_260906 - Exercise pinned Town03 mapping evidence without CARLA or GPU."""

import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

from scripts.e2e import audit_map_generalization_town03 as audit


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def fixture_capture(tmp_path: Path, name: str, positions: list[float]) -> dict:
    """HH_260906 - A tiny synthetic owner tree is sufficient to test provenance logic."""
    owner = tmp_path / name
    source = owner / "mapping"
    (source / "frames").mkdir(parents=True)
    route_path = tmp_path / (name + "_route.json")
    write_json(route_path, {"town": "Town03", "coordinate_reference": "base_link",
                            "route": [{"x": 0}, {"x": 1}]})
    route_sha = audit.digest(route_path)
    archived = owner / "provenance/scripts/e2e/collect_carla_mapping_probe.py"
    archived.parent.mkdir(parents=True)
    archived.write_text("archived source", encoding="utf-8")
    source_hash = audit.digest(archived)
    argv = [str(source), str(route_path), "--host", "127.0.0.1", "--port", "2100",
            "--motion-mode", "basic-agent", "--frames", str(len(positions)),
            "--ignore-traffic-lights", "--town03-generalization-train"]
    write_json(owner / "owner_plan.json", {
        "route_path": str(route_path), "route_sha256": route_sha, "map": "Town03",
        "capture_mode": "mapping-probe", "quality": "Epic", "source_bytes_archived": True,
        "learned_model_control": False, "host": "127.0.0.1", "port": 2100,
        "source_sha256": {"scripts/e2e/collect_carla_mapping_probe.py": source_hash},
        "collector_argv": argv,
    })
    write_json(owner / "owner_started.json", {"route_sha256": route_sha, "map": "Town03",
                                               "quality": "Epic", "port": 2100, "server_pid": 27})
    write_json(owner / "owner_result.json", {"capture_mode": "mapping-probe", "exit_code": 0,
                                              "source_bytes_unchanged_and_archived": True,
                                              "source_checks": {"scripts/e2e/collect_carla_mapping_probe.py": True},
                                              "external_source_checks": {},
                                              "learned_model_control": False})
    write_json(source / "summary.json", {
        "route_sha256": route_sha, "town": "Town03", "status": "COMPLETE_ORACLE_PLUMBING_ONLY",
        "research_split_intent": "town03_additive_train_candidate", "frame_count": len(positions),
        "physics_driving": True, "motion_mode": "basic-agent", "learned_model_control": False,
        "collision_events": 0, "live_replay_exact_match": True, "cleanup_errors": [],
        "ignores_traffic_lights": True, "oracle_pose": True, "oracle_semantics": True,
        "hd_map_approved": False, "label_table": {str(k): v for k, v in audit.TAG_TO_LABEL.items()},
        "travel_m": 1.0,
    })
    for stage in ("ready", "after_capture", "stopped"):
        write_json(owner / "lifecycle" / (stage + ".json"), {
            "status": "PASS", "stage": stage, "expected_map": "Town03", "owner_pid": 27,
            "active_map_basename": "Town03" if stage != "stopped" else None,
            "port_released": stage == "stopped",
        })
    records = []
    for index, x in enumerate(positions):
        stem = f"frames/{index:04d}"
        Image.new("RGB", (4, 3), (index, 0, 0)).save(source / (stem + ".png"))
        tags = np.zeros((3, 4, 4), dtype=np.uint8)
        tags[0, 0, 2] = 1
        tags[0, 1, 2] = 24
        tags[0, 2, 2] = 8
        np.savez_compressed(source / (stem + ".npz"), semantic=tags)
        records.append({"stem": stem, "frame": index + 1, "timestamp_s": float(index + 1),
                        "ego_x": x, "ego_y": 0.0, "ego_yaw_deg": 0.0, "speed_mps": 1.0})
    write_json(source / "records.json", records)
    return {"owner": owner, "route": route_path, "sha256": route_sha}


def test_semantic_pixel_counts_and_route_order_global_dedup(tmp_path):
    straight = fixture_capture(tmp_path, "straight", [0.0, 0.3, 0.6])
    right = fixture_capture(tmp_path, "right", [0.1, 0.9, 1.2])
    summary = audit.build_summary({"straight": straight, "right": right}, expected_frames=3,
                                  frame_wh=(4, 3), minimum_retained=2)
    assert summary["status"] == "QUALIFIED_TRAIN_CANDIDATES_ONLY"
    assert summary["combined_retained_frames"] == 5
    assert summary["routes"]["straight"]["retained_stems"] == [
        "frames/0000", "frames/0001", "frames/0002"]
    assert summary["routes"]["right"]["retained_stems"] == ["frames/0001", "frames/0002"]
    assert summary["combined_retained_class_pixels"] == [45, 5, 0, 5, 0, 5, 0]
    assert len(summary["routes"]["right"]["frames"][0]["rgb_sha256"]) == 64


def test_final_global_retained_gate_rejects_overlapping_routes(tmp_path):
    straight = fixture_capture(tmp_path, "straight", [0.0, 0.3, 0.6])
    right = fixture_capture(tmp_path, "right", [0.0, 0.3, 0.6])
    summary = audit.build_summary({"straight": straight, "right": right}, expected_frames=3,
                                  frame_wh=(4, 3), minimum_retained=2)
    assert summary["routes"]["right"]["within_route_retained_frames_0p25m"] == 3
    assert summary["routes"]["right"]["retained_frames_0p25m_global"] == 0
    assert summary["status"] == "REJECTED_INSUFFICIENT_DISTINCT_POSITIONS"


@pytest.mark.parametrize("mutation,pattern", [
    ("route_bytes", "pinned route bytes"),
    ("owner_exit", "physical mapping owner proof"),
    ("archive", "archived source differs"),
    ("lifecycle", "CARLA lifecycle failed"),
    ("test_scope", "collector scope differs"),
])
def test_owner_proof_rejects_changed_source_or_scope(tmp_path, mutation, pattern):
    spec = fixture_capture(tmp_path, "straight", [0.0, 0.3])
    owner = spec["owner"]
    if mutation == "route_bytes":
        spec["route"].write_text(spec["route"].read_text() + "\n")
    elif mutation == "owner_exit":
        path = owner / "owner_result.json"
        value = audit.read_json(path); value["exit_code"] = 1; write_json(path, value)
    elif mutation == "archive":
        (owner / "provenance/scripts/e2e/collect_carla_mapping_probe.py").write_text("changed")
    elif mutation == "lifecycle":
        path = owner / "lifecycle/stopped.json"
        value = audit.read_json(path); value["port_released"] = False; write_json(path, value)
    else:
        path = owner / "owner_plan.json"
        value = audit.read_json(path)
        value["collector_argv"].remove("--town03-generalization-train")
        value["collector_argv"].append("--town07-generalization-test")
        write_json(path, value)
    with pytest.raises(ValueError, match=pattern):
        audit.audit_owner("straight", spec, expected_frames=2)


def test_frame_pair_rejects_corrupt_or_missing_input(tmp_path):
    spec = fixture_capture(tmp_path, "straight", [0.0, 0.3])
    source = spec["owner"] / "mapping"
    (source / "frames/0001.png").write_bytes(b"not a PNG")
    with pytest.raises(Exception):
        audit.audit_frames(source, expected_frames=2, frame_wh=(4, 3))
    (source / "frames/0001.png").unlink()
    with pytest.raises(ValueError, match="missing/unsafe native frame pair"):
        audit.audit_frames(source, expected_frames=2, frame_wh=(4, 3))
