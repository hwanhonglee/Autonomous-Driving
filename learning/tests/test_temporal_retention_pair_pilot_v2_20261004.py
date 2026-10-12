"""HH_261004 - Reject mutation of OLD TRAIN admission pins before launching the revised pilot."""
from argparse import Namespace
from pathlib import Path

import pytest

from scripts.e2e import train_temporal_retention_pair_pilot_v2_20261004 as pilot


def test_revision_preserves_math_and_restores_bindings():
    old = pilot.previous
    values = old.SCHEMA, old.STUDY, old.contract, old.source_snapshot, old.pair.verify_parent
    base_math = old.fit_arm, old.objective, old.retention_term, old.panel_measurements
    with pilot.activated():
        contract = pilot.contract()
        assert contract["study_id"] == pilot.STUDY and contract["schema"] == pilot.SCHEMA
        assert len(contract["sources"]) == 34
        assert contract["old_TRAIN_admission_artifacts_pinned_before_first_fit"] is True
        assert contract["updates_per_arm"] == 1000 and contract["coefficient"] == .25
        assert base_math == (old.fit_arm, old.objective, old.retention_term, old.panel_measurements)
    assert values == (old.SCHEMA, old.STUDY, old.contract, old.source_snapshot, old.pair.verify_parent)


def test_six_admission_artifacts_are_bound_before_fit(monkeypatch):
    args = Namespace(pilot_authority=Path("/private/authority.json"), pilot_authority_sha256="a" * 64,
        recovery_conversion=Path("/private/conversion.json"), recovery_train=Path("/private/recovery"))
    monkeypatch.setattr(pilot, "_verify_parent", lambda *_: ("parent", {}, "screens"))
    monkeypatch.setattr(pilot.previous.frozen, "regular", lambda path, _: path)
    monkeypatch.setattr(pilot, "digest", lambda path: "a" * 64)
    _, pins, _ = pilot.verify_parent(args, [], [])
    assert len(pins) == 6
    assert str(args.pilot_authority) in pins and str(args.recovery_conversion) in pins
    assert str(args.recovery_train / "dataset.json") in pins


def test_authority_changed_after_first_read_is_rejected(monkeypatch):
    args = Namespace(pilot_authority=Path("/private/authority.json"), pilot_authority_sha256="a" * 64,
        recovery_conversion=Path("/private/conversion.json"), recovery_train=Path("/private/recovery"))
    monkeypatch.setattr(pilot, "_verify_parent", lambda *_: ("parent", {}, "screens"))
    monkeypatch.setattr(pilot.previous.frozen, "regular", lambda path, _: path)
    monkeypatch.setattr(pilot, "digest", lambda path: "b" * 64)
    with pytest.raises(ValueError, match="authority changed"):
        pilot.verify_parent(args, [], [])


def test_postfit_pin_verifier_rejects_old_manifest_mutation(monkeypatch, tmp_path):
    # HH_261004 - Exercise the real immutable-input verifier, not a mock postfit success.
    path = tmp_path / "dataset.json"; path.write_text("first")
    expected = pilot.previous.digest(path)
    verifier = pilot.previous.pair.verify_inputs
    monkeypatch.setattr(pilot.previous.pair, "regular", lambda path, _: path)
    verifier({str(path): expected}, {}, [])
    path.write_text("changed")
    with pytest.raises(ValueError):
        verifier({str(path): expected}, {}, [])
