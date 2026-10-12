"""HH_261001 - Preserve the v4 audit's policy/owner scope without live simulator calls."""

from scripts.e2e import audit_town12_brake_calibration_v4_20261001 as audit


def test_source_closure_enters_both_versioned_owner_scopes(monkeypatch):
    inherited_owner = audit.owner.inherited.inherited
    previous = inherited_owner.OUTPUT
    def check(trial, plan):
        assert inherited_owner.OUTPUT == audit.owner.OUTPUT
        return {"status": "PASS"}
    monkeypatch.setattr(audit, "ORIGINAL_SOURCE_AUDIT", check)
    assert audit.source_audit(None, None) == {"status": "PASS"}
    assert inherited_owner.OUTPUT == previous


def test_replay_uses_exact_low_speed_policy_and_restores_bindings():
    old = audit.inherited.worker, audit.inherited.WORKER_SHA256, audit.inherited.source_audit, audit.inherited.analyze_level
    with audit.configured_audit():
        assert audit.inherited.worker is audit.worker
        assert audit.inherited.worker.inherited.PulsePolicy is audit.worker.PulsePolicy
        assert audit.inherited.WORKER_SHA256 == audit.WORKER_SHA256
    assert (audit.inherited.worker, audit.inherited.WORKER_SHA256, audit.inherited.source_audit, audit.inherited.analyze_level) == old
