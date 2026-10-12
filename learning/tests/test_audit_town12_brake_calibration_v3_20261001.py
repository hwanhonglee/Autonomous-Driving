"""HH_261001 - Audit the declared low-speed policy without changing inherited replay or source gates."""

import pytest

from scripts.e2e import audit_town12_brake_calibration_v3_20261001 as audit


def test_low_speed_replay_bindings_are_scoped_and_restore_on_error():
    old = audit.inherited.worker, audit.inherited.WORKER_SHA256, audit.inherited.source_audit, audit.inherited.analyze_level
    original_policy = audit.worker.inherited.PulsePolicy
    with pytest.raises(RuntimeError):
        with audit.configured_audit():
            assert audit.inherited.worker is audit.worker
            assert audit.worker.inherited.PulsePolicy is audit.worker.PulsePolicy
            assert audit.inherited.WORKER_SHA256 == audit.WORKER_SHA256
            raise RuntimeError("restore")
    assert (audit.inherited.worker, audit.inherited.WORKER_SHA256, audit.inherited.source_audit, audit.inherited.analyze_level) == old
    assert audit.worker.inherited.PulsePolicy is original_policy


def test_source_closure_selects_v3_without_calling_preflight_or_launch(monkeypatch):
    old = audit.owner.inherited.OUTPUT
    def check(trial, plan):
        assert audit.owner.inherited.OUTPUT == audit.owner.OUTPUT
        return {"status": "checked"}
    monkeypatch.setattr(audit, "ORIGINAL_SOURCE_AUDIT", check)
    assert audit.source_audit(None, None) == {"status": "checked"}
    assert audit.owner.inherited.OUTPUT == old
