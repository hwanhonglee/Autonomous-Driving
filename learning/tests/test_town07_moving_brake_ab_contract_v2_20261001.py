"""HH_261001 - Preserve an explicitly exploratory fixed two-attempt declaration."""

from copy import deepcopy
from datetime import datetime, timezone

import pytest

from scripts.e2e import town07_moving_brake_ab_contract_v2_20261001 as contract


def test_fixed_order_is_two_attempts_not_ABBA_or_repeats():
    value = contract.fixed_contract()
    assert contract.SCHEMA == "portable_e2e.town07_w1_moving_brake_exploratory.v2"
    assert contract.RUN_ROOT.name == "town07_w1_moving_brake_exploratory_v2"
    assert contract.PLAN.name == "town07_w1_moving_brake_exploratory_plan_v2.json"
    assert len(contract.NEW_SOURCES) == 6
    assert all("_v2_20261001.py" in name for name in contract.NEW_SOURCES)
    assert [row["arm"] for row in value["trial_order"]] == ["A_original", "B_suppress_normal_moving_brake"]
    assert [row["trial_id"] for row in value["trial_order"]] == ["A1_original", "B1_suppress_normal_moving_brake"]
    assert value["attempts_per_arm"] == 1 and value["ABBA"] is False
    assert value["exploratory"] and value["independent_replication"] is False
    assert value["automatic_retry"] is False and value["model_optimizer_updates"] == 0
    assert value["training_data_approved"] is value["model_promoted"] is False
    assert value["physics_hz"] == 20 and value["capture_hz"] == 10
    assert value["total_capture_seconds"] == 20 and value["teacher_target_kph"] == 9
    assert value["learned_speed_cap_mps"] == 2.5
    assert value["all_original_JPEG_native_ACK_predictions_retained"] is True
    assert value["all_exact_inputs_retained"] is False


@pytest.mark.parametrize("index", [-1, 2, True, None, "0"])
def test_only_fixed_numeric_trial_indices(index):
    with pytest.raises(ValueError):
        contract.trial(index)


def test_total_pair_reserve_and_sampling_allowance_are_distinct():
    required = contract.FLOOR + 2*contract.ARM_RESERVE + contract.SHARED_RESERVE
    assert contract.disk_check(2, free=required)["required_bytes"] == required
    assert contract.ARM_RESERVE == 256*1024**2 and contract.SAMPLE_LIMIT == 128*1024**2
    with pytest.raises(ValueError, match="headroom"):
        contract.disk_check(2, free=required - 1)
    assert contract.disk_check(1, free=required - 1)["reserved_remaining_arms"] == 1
    for arms in (0, 3, 4, True):
        with pytest.raises(ValueError):
            contract.disk_check(arms, free=required)


def test_no_replace_or_symlink_declaration(tmp_path):
    path = tmp_path / "plan.json"
    contract.write_new(path, {"original": True})
    with pytest.raises(FileExistsError):
        contract.write_new(path, {"original": False})
    link = tmp_path / "link.json"
    link.symlink_to(path)
    with pytest.raises(ValueError):
        contract.write_new(link, {})
    with pytest.raises(ValueError):
        contract.read_json(link)
    assert contract.read_json(path) == {"original": True}


def declared_fixture(monkeypatch, tmp_path):
    plan_path = tmp_path / "plan.json"
    monkeypatch.setattr(contract, "PLAN", plan_path)
    monkeypatch.setattr(contract, "runtime_sources", lambda: {"pinned.py": "a"*64})
    monkeypatch.setattr(contract, "assets", lambda: {})
    monkeypatch.setattr(contract, "AGENT_SOURCES", ())
    value = dict(contract.fixed_contract(), declared_at_utc=datetime.now(timezone.utc).isoformat(),
                 source_sha256={"pinned.py": "a"*64}, external_agent_sha256={}, asset_sha256={})
    contract.write_new(plan_path, value)
    return value, contract.digest(plan_path)


def test_exact_declaration_verifies_without_outputs(monkeypatch, tmp_path):
    value, pinned = declared_fixture(monkeypatch, tmp_path)
    contract.verify_plan(value, expected_sha256=pinned)
    assert not contract.RUN_ROOT.exists()
    with pytest.raises(ValueError, match="bytes"):
        contract.verify_plan(value, expected_sha256="0"*64)


@pytest.mark.parametrize("field,value", [("ABBA", True), ("attempts_per_arm", 2),
    ("model_optimizer_updates", 1), ("total_capture_seconds", 21),
    ("physics_hz", 10), ("capture_hz", 20), ("learned_speed_cap_mps", 3.),
    ("feedforward_added", True), ("original_emergency_authority_unchanged", False),
    ("sample_directory_maximum_bytes", 129*1024**2)])
def test_declared_scope_cannot_be_relaxed(monkeypatch, tmp_path, field, value):
    original, pinned = declared_fixture(monkeypatch, tmp_path)
    changed = deepcopy(original)
    changed[field] = value
    with pytest.raises(ValueError, match="contract"):
        contract.verify_plan(changed, expected_sha256=pinned)


def test_source_closure_must_remain_exact(monkeypatch, tmp_path):
    value, pinned = declared_fixture(monkeypatch, tmp_path)
    monkeypatch.setattr(contract, "runtime_sources", lambda: {"pinned.py": "b"*64})
    with pytest.raises(ValueError, match="source closure"):
        contract.verify_plan(value, expected_sha256=pinned)


def test_declaration_cannot_follow_an_attempt(monkeypatch, tmp_path):
    monkeypatch.setattr(contract, "output", lambda index: tmp_path / str(index))
    (tmp_path / "0").mkdir()
    with pytest.raises(ValueError, match="precede"):
        contract.build_plan()
