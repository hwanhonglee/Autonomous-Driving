"""HH_261001 - Isolate a frozen preflight assertion after real campaign outputs exist."""

from pathlib import Path
import runpy


def test_frozen_declaration_check_uses_an_empty_temporary_output(monkeypatch, tmp_path):
    # HH_261001 - Preserve the recorded source bytes and rerun its complete test logic.
    frozen = runpy.run_path(Path(__file__).with_name(
        "test_town07_moving_brake_ab_contract_v2_20261001.py"))
    contract = frozen["contract"]
    original_root = contract.RUN_ROOT
    isolated_root = tmp_path / original_root.name
    monkeypatch.setattr(contract, "RUN_ROOT", isolated_root)
    frozen["test_exact_declaration_verifies_without_outputs"](monkeypatch, tmp_path)
    assert not isolated_root.exists()
    # HH_261001 - Never remove real test evidence to satisfy a preflight-only assertion.
    assert original_root == Path(contract.ROOT) / (
        "storage/experiments/validation/2026-10-01/"
        "town07_w1_moving_brake_exploratory_v2")
