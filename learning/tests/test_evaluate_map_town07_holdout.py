"""HH_260906 - Synthetic freeze/gate tests; never open the real sealed Town07 frames."""

import json
from pathlib import Path
import sys

import numpy as np
import pytest

from scripts.e2e import evaluate_map_town07_holdout as holdout


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def write_bytes(path: Path, value: bytes = b"owned-fixture") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(value)


def test_unseal_is_checked_before_any_contract_or_town07_access(monkeypatch, tmp_path):
    touched = []
    monkeypatch.setattr(holdout, "read_json", lambda path: touched.append(path))
    with pytest.raises(ValueError, match="explicit --unseal"):
        holdout.verify_freeze_receipt(tmp_path / "missing.json", unseal=False)
    assert touched == []


def test_cli_rejects_alternate_receipt_or_output_before_frame_access(monkeypatch, tmp_path):
    frozen = tmp_path / "town07_freeze.json"
    output = tmp_path / "map_town07_holdout_eval"
    monkeypatch.setattr(holdout, "FROZEN_CONTRACT", frozen)
    monkeypatch.setattr(holdout, "DEFAULT_OUTPUT", output)
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "")
    touched = []
    monkeypatch.setattr(holdout, "verify_freeze_receipt", lambda *args, **kwargs: touched.append(args))
    monkeypatch.setattr(sys, "argv", ["holdout", "--frozen-contract", str(tmp_path / "other.json"),
                                     "--unseal", "--output", str(output)])
    with pytest.raises(ValueError, match="exact dated"):
        holdout.main()
    monkeypatch.setattr(sys, "argv", ["holdout", "--frozen-contract", str(frozen),
                                     "--unseal", "--output", str(tmp_path / "another_eval")])
    with pytest.raises(ValueError, match="single exact fresh"):
        holdout.main()
    assert touched == []


def test_global_same_town_spacing_and_tag_lut():
    first = [{"ego_x": x, "ego_y": 0.0} for x in (0.0, 0.1, 0.3)]
    second = [{"ego_x": x, "ego_y": 0.0} for x in (0.05, 0.61, 0.92)]
    kept_a, positions = holdout.retain_indices(first)
    kept_b, _ = holdout.retain_indices(second, positions)
    assert kept_a == [0, 2]
    assert kept_b == [1, 2]
    semantic = np.zeros((2, 4, 4), dtype=np.uint8)
    semantic[..., 2] = np.asarray([[1, 24, 8, 7], [2, 6, 4, 0]], dtype=np.uint8)
    target = holdout.semantic_target(semantic, frame_wh=(4, 2))
    assert target.tolist() == [[1, 3, 5, 4], [2, 6, 0, 0]]
    with pytest.raises(ValueError, match="semantic BGRA"):
        holdout.semantic_target(semantic[..., :3], frame_wh=(4, 2))


def test_confusion_reports_lane_to_sign_and_sign_to_other():
    target = np.asarray([[3, 3, 5, 0], [1, 5, 0, 0]], dtype=np.uint8)
    prediction = np.asarray([[5, 3, 0, 0], [1, 5, 0, 0]], dtype=np.uint8)
    matrix = holdout.confusion_matrix(target, prediction)
    metrics = holdout.class_metrics(matrix)
    assert matrix.sum() == 8
    assert metrics["gt_lane_pred_sign_pixels"] == 1
    assert metrics["gt_sign_pred_other_pixels"] == 1
    assert metrics["classes"]["traffic_sign"]["precision"] == pytest.approx(0.5)
    assert metrics["classes"]["traffic_sign"]["recall"] == pytest.approx(0.5)
    assert metrics["mean_iou_7_zero_if_absent"] <= metrics["mean_iou_present"]
    with pytest.raises(ValueError, match="seven-class"):
        holdout.confusion_matrix(target, np.full_like(target, 7))


def town07_owner_metadata_fixture(tmp_path: Path) -> dict:
    """HH_260906 - Owner proof fixture contains metadata only and no camera images."""
    owner, route = tmp_path / "sealed_owner", tmp_path / "route.json"
    write_json(route, {"town": "Town07", "coordinate_reference": "base_link",
                       "route": [{"x": 0}, {"x": 1}]})
    route_sha = holdout.digest(route)
    source = owner / "mapping"
    write_bytes(owner / "provenance/scripts/e2e/collect_carla_mapping_probe.py", b"archived collector")
    source_rel = "scripts/e2e/collect_carla_mapping_probe.py"
    source_sha = holdout.digest(owner / "provenance" / source_rel)
    argv = [str(source), str(route), "--host", "127.0.0.1", "--port", "2200",
            "--motion-mode", "basic-agent", "--frames", "240",
            "--ignore-traffic-lights", "--town07-generalization-test"]
    write_json(owner / "owner_plan.json", {
        "route_path": str(route), "route_sha256": route_sha, "map": "Town07",
        "capture_mode": "mapping-probe", "quality": "Epic", "source_bytes_archived": True,
        "learned_model_control": False, "host": "127.0.0.1", "port": 2200,
        "collector_argv": argv, "source_sha256": {source_rel: source_sha},
    })
    write_json(owner / "owner_started.json", {"route_sha256": route_sha, "map": "Town07",
                                               "quality": "Epic", "port": 2200, "server_pid": 28})
    write_json(owner / "owner_result.json", {"capture_mode": "mapping-probe", "exit_code": 0,
                                              "source_bytes_unchanged_and_archived": True,
                                              "learned_model_control": False,
                                              "source_checks": {source_rel: True},
                                              "external_source_checks": {}})
    write_json(source / "summary.json", {
        "route_sha256": route_sha, "town": "Town07", "status": "COMPLETE_ORACLE_PLUMBING_ONLY",
        "research_split_intent": "town07_sealed_evaluation_candidate", "frame_count": 240,
        "physics_driving": True, "motion_mode": "basic-agent", "learned_model_control": False,
        "ignores_traffic_lights": True, "collision_events": 0, "live_replay_exact_match": True,
        "cleanup_errors": [], "oracle_semantics": True, "oracle_pose": True,
        "hd_map_approved": False, "label_table": {str(k): v for k, v in holdout.TAG_TABLE.items()},
    })
    write_json(source / "records.json", [
        {"stem": f"frames/{index:04d}", "frame": index + 1, "timestamp_s": float(index + 1),
         "ego_x": index * .3, "ego_y": 0.0} for index in range(240)])
    for stage in ("ready", "after_capture", "stopped"):
        write_json(owner / "lifecycle" / (stage + ".json"), {
            "status": "PASS", "stage": stage, "expected_map": "Town07", "owner_pid": 28,
            "active_map_basename": "Town07" if stage != "stopped" else None,
            "port_released": stage == "stopped",
        })
    return {"owner": owner, "route": route, "sha256": route_sha}


def test_metadata_only_owner_proof_has_no_frame_dependency(tmp_path):
    spec = town07_owner_metadata_fixture(tmp_path)
    records, proof = holdout.verify_owner_metadata("straight", spec)
    assert len(records) == 240 and proof["archived_source_count"] == 1
    assert proof["route_sha256"] == spec["sha256"]
    capture_path = spec["owner"] / "mapping/summary.json"
    capture = holdout.read_json(capture_path)
    capture["collision_events"] = 1
    write_json(capture_path, capture)
    with pytest.raises(ValueError, match="physical owner proof"):
        holdout.verify_owner_metadata("straight", spec)


def metric(road, lane, mean, sign_iou=0.04, sign_precision=0.05):
    classes = {label: {"iou": 0.0, "precision": 0.0} for label in holdout.LABELS}
    classes["road"]["iou"] = road
    classes["lane_marking"]["iou"] = lane
    classes["traffic_sign"] = {"iou": sign_iou, "precision": sign_precision}
    return {"classes": classes, "mean_iou_7_zero_if_absent": mean}


def test_predeclared_gate_requires_both_routes_combined_gain_and_dev_guard():
    parent = {name: metric(0.70, 0.40, 0.40) for name in ("straight", "right", "combined")}
    candidate = {name: metric(0.83, 0.54, 0.48) for name in ("straight", "right", "combined")}
    parent_dev = metric(0.9875, 0.9123, 0.6012, sign_iou=0.03157, sign_precision=0.03293)
    candidate_dev = metric(0.98, 0.90, 0.59, sign_iou=0.032, sign_precision=0.034)
    passed = holdout.score_gate(parent, candidate, parent_dev, candidate_dev)
    assert passed["research_gate_passed"] is True
    assert passed["automatic_promotion"] is False
    candidate["right"] = metric(0.79, 0.54, 0.48)
    failed_floor = holdout.score_gate(parent, candidate, parent_dev, candidate_dev)
    assert failed_floor["research_gate_passed"] is False
    candidate["right"] = metric(0.83, 0.54, 0.48)
    candidate_dev = metric(0.98, 0.90, 0.59, sign_iou=0.030, sign_precision=0.034)
    failed_dev = holdout.score_gate(parent, candidate, parent_dev, candidate_dev)
    assert failed_dev["research_gate_passed"] is False


def freeze_fixture(tmp_path: Path, monkeypatch) -> tuple[Path, dict, Path]:
    """HH_260906 - Replace every source with tiny owned metadata, never a real Town07 sensor file."""
    monkeypatch.setattr(holdout, "ROOT", tmp_path)
    plan = tmp_path / "plan.md"
    parent = tmp_path / "parent/checkpoint.pt"
    parent_result = tmp_path / "parent/result"
    model_source = tmp_path / "model_source.py"
    monkeypatch.setattr(holdout, "PLAN", plan)
    monkeypatch.setattr(holdout, "PARENT", parent)
    monkeypatch.setattr(holdout, "PARENT_RESULT", parent_result)
    monkeypatch.setattr(holdout, "MODEL_SOURCE", model_source)
    write_bytes(plan, b"fixed plan")
    write_bytes(parent, b"parent checkpoint")
    write_bytes(model_source, b"fixed MapPerception source")
    dataset = tmp_path / "dataset"
    write_bytes(dataset / "dev.npz", b"dev-array-fixture")
    monkeypatch.setattr(holdout, "PARENT_SHA256", holdout.digest(parent))
    monkeypatch.setattr(holdout, "MODEL_SOURCE_SHA256", holdout.digest(model_source))
    old_records = [{"old_index": index} for index in range(497)]
    parent_manifest = {"labels": list(holdout.LABELS), "splits": {
        "train": {"source_records": old_records},
        "dev": {"file": "dev.npz", "sha256": holdout.digest(dataset / "dev.npz"), "count": 118},
    }}
    write_json(parent_result / "summary.json", {"checkpoint_sha256": holdout.digest(parent),
                                               "deployment_approved": False})
    write_json(parent_result / "started.json", {"seed": 20260922, "batch_size": 8})
    write_json(parent_result / "data_manifest.json", parent_manifest)
    monkeypatch.setattr(holdout, "PARENT_SUMMARY_SHA256", holdout.digest(parent_result / "summary.json"))
    monkeypatch.setattr(holdout, "PARENT_STARTED_SHA256", holdout.digest(parent_result / "started.json"))
    monkeypatch.setattr(holdout, "PARENT_MANIFEST_SHA256", holdout.digest(parent_result / "data_manifest.json"))
    candidate = tmp_path / "candidate"
    write_bytes(candidate / "checkpoint.pt", b"candidate checkpoint")
    write_bytes(candidate / "source/map_perception.py", model_source.read_bytes())
    write_bytes(candidate / "source/train_map_native_ah.py", b"candidate training source")
    write_bytes(dataset / "train.npz", b"train-array-fixture")
    write_bytes(dataset / "dev.npz", b"dev-array-fixture")
    manifest = {"labels": list(holdout.LABELS), "image_size_wh": [640, 360], "splits": {
        "train": {"file": "train.npz", "sha256": holdout.digest(dataset / "train.npz"),
                  "count": 913, "source_records": old_records + [{"new": True}] * 416},
        "dev": parent_manifest["splits"]["dev"],
    }}
    write_json(dataset / "manifest.json", manifest)
    write_bytes(candidate / "data_manifest.json", (dataset / "manifest.json").read_bytes())
    write_json(candidate / "started.json", {
        "parent_checkpoint_sha256": holdout.digest(parent),
        "data_manifest_sha256": holdout.digest(dataset / "manifest.json"),
        "source_sha256": holdout.digest(candidate / "source/train_map_native_ah.py"),
        "new_physical_routes": True, "initialization": "scratch", "steps": 2000,
        "seed": 20260922, "batch_size": 8, "input_shape": [3, 360, 640],
    })
    write_json(candidate / "summary.json", {
        "checkpoint_sha256": holdout.digest(candidate / "checkpoint.pt"),
        "deployment_approved": False,
    })
    routes = {}
    for name in ("straight", "right"):
        owner = tmp_path / ("owner_" + name)
        paths = holdout.owner_metadata_paths(owner)
        for key, path in paths.items():
            write_bytes(path, (name + key).encode())
        routes[name] = {"owner": owner, "route": tmp_path / (name + ".json"), "sha256": "fixture"}
    monkeypatch.setattr(holdout, "ROUTES", routes)
    pinned = {
        "evaluator_sha256": holdout.digest(Path(holdout.__file__)),
        "plan_sha256": holdout.digest(plan), "model_source_sha256": holdout.digest(model_source),
        "parent_checkpoint_sha256": holdout.digest(parent),
        "candidate_run_relpath": "candidate", "candidate_dataset_relpath": "dataset",
        "candidate_sha256": {
            "checkpoint": holdout.digest(candidate / "checkpoint.pt"),
            "summary": holdout.digest(candidate / "summary.json"),
            "started": holdout.digest(candidate / "started.json"),
            "data_manifest": holdout.digest(candidate / "data_manifest.json"),
            "model_source": holdout.digest(candidate / "source/map_perception.py"),
            "training_source": holdout.digest(candidate / "source/train_map_native_ah.py"),
            "dataset_manifest": holdout.digest(dataset / "manifest.json"),
        },
        "owner_metadata_sha256": {
            name: {key: holdout.digest(path) for key, path in holdout.owner_metadata_paths(spec["owner"]).items()}
            for name, spec in routes.items()
        },
    }
    receipt = tmp_path / "freeze.json"
    write_json(receipt, {"schema": "portable_e2e.map_town07_holdout_freeze.v1",
                         "status": "FROZEN_BEFORE_HOLDOUT", "unseal_authorized": True,
                         "automatic_promotion": False, "pinned": pinned})
    return receipt, pinned, candidate


def test_freeze_receipt_pins_code_parent_candidate_and_owner_metadata(tmp_path, monkeypatch):
    receipt, _, candidate = freeze_fixture(tmp_path, monkeypatch)
    checked = holdout.verify_freeze_receipt(receipt, unseal=True)
    assert checked["candidate"] == candidate
    assert checked["candidate_summary"]["deployment_approved"] is False
    (candidate / "checkpoint.pt").write_bytes(b"changed")
    with pytest.raises(ValueError, match="candidate: frozen checkpoint SHA256 differs"):
        holdout.verify_freeze_receipt(receipt, unseal=True)


def test_freeze_receipt_rejects_changed_owner_or_evaluator(tmp_path, monkeypatch):
    receipt, pinned, _ = freeze_fixture(tmp_path, monkeypatch)
    path = holdout.owner_metadata_paths(holdout.ROUTES["right"]["owner"])["records"]
    path.write_bytes(b"changed records metadata")
    with pytest.raises(ValueError, match="Town07 right: frozen records SHA256 differs"):
        holdout.verify_freeze_receipt(receipt, unseal=True)
    path.write_bytes(b"rightrecords")
    pinned["evaluator_sha256"] = "0" * 64
    contract = holdout.read_json(receipt); contract["pinned"] = pinned; write_json(receipt, contract)
    with pytest.raises(ValueError, match="frozen evaluator"):
        holdout.verify_freeze_receipt(receipt, unseal=True)
