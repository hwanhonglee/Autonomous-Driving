"""HH_260906 - Keep static long-turn review separate from capture/data admission."""

import hashlib
import json
from pathlib import Path

import pytest

from scripts.e2e import audit_new_long_turn_candidates as audit


def _write(path: Path, document: dict) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (json.dumps(document, indent=2) + "\n").encode()
    path.write_bytes(payload)
    return hashlib.sha256(payload).hexdigest()


def _route() -> dict:
    points = []
    for index in range(221):
        points.append(dict(x=float(index), y=0.0, distance_m=float(index),
                           road_option="RIGHT" if 180 <= index <= 195 else "LANEFOLLOW",
                           road_id=10 if index < 180 else 11, section_id=0, lane_id=-1))
    return dict(town="Town01", scenario="right", coordinate_reference="base_link",
                route_length_m=220.0, route=points,
                physical_turn_preflight=dict(status="PASS", scenario="right",
                    directional_block_count=1, additional_maneuver_commands=[],
                    selected_block=dict(start_index=180, end_index=195,
                                        route_lead_distance_m=180.0,
                                        route_tail_distance_m=25.0)))


def _owner_fixture(tmp_path: Path, monkeypatch) -> tuple[Path, Path]:
    monkeypatch.setattr(audit, "ROOT", tmp_path)
    reference = tmp_path / "historical.json"
    reference_sha = _write(reference, _route())
    monkeypatch.setattr(audit, "REFERENCE_SHA256", reference_sha)
    owner = tmp_path / "storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v99"
    _write(owner / "owner_plan.json", dict(map_id="town01", split_intent="TRAIN",
                                          capture_executed=False, training_executed=False))
    route_id = "town01_right_s0002_p00"
    relative = Path("routes/town01/right") / (route_id + ".json")
    route_sha = _write(owner / relative, _route())
    catalog_sha = _write(owner / "route_catalog.json", dict(status="complete", map_id="town01",
        canonical_name="Town01", server=dict(map_load_performed=False),
        routes=[dict(id=route_id, status="ready", scenario="right", route_length_m=220.0,
                     path=relative.as_posix(), sha256=route_sha)]))
    _write(owner / "owner_result.json", dict(exit_code=0, catalog_status="complete",
        source_bytes_unchanged_and_archived=True, lifecycle_receipts_present=True,
        catalog_sha256=catalog_sha, right_candidate_count=1))
    for stage in ("ready", "after_catalog", "stopped"):
        _write(owner / "lifecycle" / f"{stage}.json", dict(stage=stage, status="PASS"))
    return owner, reference


def test_static_audit_flags_historical_duplicate_and_short_15kph_tail(tmp_path, monkeypatch) -> None:
    owner, reference = _owner_fixture(tmp_path, monkeypatch)
    report = audit.audit(owner, reference)
    candidate = report["candidates"][0]
    assert report["status"] == "STATIC_REVIEW_ONLY"
    assert candidate["capture_authorized"] is False
    assert candidate["training_data_approved"] is False
    assert candidate["same_turn_transitions_as_train"] is True
    assert candidate["fraction_waypoints_within_2m_of_train"] == 1.0
    assert candidate["static_budget"]["9"]["turn_tail_pass"] is True
    assert candidate["static_budget"]["15"]["turn_tail_pass"] is False
    assert "SAME_PHYSICAL_TURN_TRANSITIONS_AS_TRAIN" in candidate["review_flags"]


@pytest.mark.parametrize("mutation", ["owner_failed", "missing_stopped", "sha_mismatch", "route_escape", "route_sha_missing"])
def test_audit_rejects_incomplete_or_tampered_owner(tmp_path, monkeypatch, mutation) -> None:
    owner, reference = _owner_fixture(tmp_path, monkeypatch)
    if mutation == "owner_failed":
        path = owner / "owner_result.json"
        record = json.loads(path.read_text())
        record["exit_code"] = 1
        _write(path, record)
    elif mutation == "missing_stopped":
        (owner / "lifecycle/stopped.json").unlink()
    elif mutation == "sha_mismatch":
        path = owner / "route_catalog.json"
        record = json.loads(path.read_text())
        record["server"]["map_load_performed"] = True
        _write(path, record)
    elif mutation in ("route_escape", "route_sha_missing"):
        path = owner / "route_catalog.json"
        record = json.loads(path.read_text())
        if mutation == "route_escape":
            record["routes"][0]["path"] = "../outside.json"
        else:
            record["routes"][0].pop("sha256")
        sha = _write(path, record)
        result_path = owner / "owner_result.json"
        result = json.loads(result_path.read_text())
        result["catalog_sha256"] = sha
        _write(result_path, result)
    with pytest.raises(audit.AuditError):
        audit.audit(owner, reference)


def test_historical_reference_geometry_is_valid_without_carla() -> None:
    historical, digest = audit.read_json(audit.REFERENCE, audit.REFERENCE_SHA256)
    geometry = audit.turn_geometry(historical)
    assert digest == audit.REFERENCE_SHA256
    assert geometry["length_m"] > 200.0
    assert geometry["lead_m"] > 25.0
