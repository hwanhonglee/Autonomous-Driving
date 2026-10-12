"""HH_260906 - Town05_Opt provisional rank must reject unfinished ownership."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from scripts.e2e import audit_town05_opt_long_turn_catalog as audit


def test_incomplete_owner_refuses_to_open_catalog(tmp_path: Path):
    plan = json.loads((audit.OWNER / "owner_plan.json").read_text())
    (tmp_path / "owner_plan.json").write_text(json.dumps(plan))
    with pytest.raises(audit.IncompleteOwner, match="owner_result"):
        audit.completed_owner(tmp_path)


def test_source_check_ledger_must_not_be_missing(tmp_path: Path):
    plan = json.loads((audit.OWNER / "owner_plan.json").read_text())
    (tmp_path / "owner_plan.json").write_text(json.dumps(plan))
    incomplete = {"schema": "portable_e2e.owned_turn_route_catalog_result.v1",
                  "exit_code": 0, "catalog_status": "complete",
                  "source_bytes_unchanged_and_archived": True,
                  "lifecycle_receipts_present": True,
                  "capture_executed": False, "training_executed": False,
                  "dataset_admission_granted": False, "learned_model_control": False,
                  "source_checks": {}}
    (tmp_path / "owner_result.json").write_text(json.dumps(incomplete))
    with pytest.raises(audit.IncompleteOwner, match="source-check ledger"):
        audit.completed_owner(tmp_path)


def test_same_physical_turn_not_double_counted():
    original = (((256, 0, 1), (12, 0, -1)), (100.0, 200.0))
    nearby = (((256, 0, 1), (12, 0, -1)), (100.2, 199.8))
    other = (((208, 0, -1), (12, 0, 1)), (100.0, 200.0))
    assert audit.same_site(original, nearby)
    assert not audit.same_site(original, other)


def test_short_previous_town05_right_route_can_be_fingerprinted():
    assert audit.sha(audit.PREVIOUS_RIGHT) == audit.PREVIOUS_RIGHT_SHA256
    previous = audit.read(audit.PREVIOUS_RIGHT)
    identity, center = audit.turn_fingerprint(previous)
    assert len(identity) >= 1
    assert len(center) == 2
    assert previous["route_length_m"] < 100.0


def test_incomplete_catalog_creates_no_rank_output(tmp_path: Path):
    # HH_260906 - The production owner may finish later; this helper exercises the same guard on a tmp root.
    plan = json.loads((audit.OWNER / "owner_plan.json").read_text())
    (tmp_path / "owner_plan.json").write_text(json.dumps(plan))
    candidate_output = tmp_path / "rank.json"
    with pytest.raises(audit.IncompleteOwner):
        audit.completed_owner(tmp_path)
    assert not candidate_output.exists()


def test_blocked_owner_has_explicit_stop_reason(tmp_path: Path):
    plan = json.loads((audit.OWNER / "owner_plan.json").read_text())
    (tmp_path / "owner_plan.json").write_text(json.dumps(plan))
    blocked = {"exit_code": 1, "catalog_status": "blocked"}
    (tmp_path / "owner_result.json").write_text(json.dumps(blocked))
    with pytest.raises(audit.IncompleteOwner, match="owner search blocked: exit=1 catalog=blocked"):
        audit.completed_owner(tmp_path)
