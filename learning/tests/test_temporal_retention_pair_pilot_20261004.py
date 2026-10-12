"""HH_261004 - Synthetic CPU contracts for a non-promoting paired TRAIN-only pilot."""
from argparse import Namespace
import ast
from pathlib import Path

import pytest
import torch

from scripts.e2e import train_temporal_retention_pair_pilot_20261004 as pilot


def test_control_does_not_add_zero_or_replace_the_original_objective():
    base = torch.tensor(3., requires_grad=True)
    extra = torch.tensor(7., requires_grad=True)
    assert pilot.objective(base, extra, "control") is base
    value = pilot.objective(base, extra, "retention")
    assert value.item() == 4.75
    value.backward()
    assert base.grad.item() == 1 and extra.grad.item() == .25


@pytest.mark.parametrize("arm", ["", "candidate", None, True])
def test_unknown_arm_rejected(arm):
    with pytest.raises(ValueError):
        pilot.objective(torch.tensor(0.), torch.tensor(0.), arm)


def test_fixed_panel_covers_whole_eligible_index_span():
    assert pilot.uniform_panel(32) == list(range(32))
    indices = pilot.uniform_panel(697)
    assert len(indices) == len(set(indices)) == 32
    assert indices[0] == 0 and indices[-1] == 696
    eligible = list(range(1, 100, 2))
    assert set(pilot.uniform_panel(100, eligible)) <= set(eligible)


@pytest.mark.parametrize("count,eligible", [(True, None), (31, None), (64, [0] * 32), (64, list(range(31))), (64, list(range(33, 65)))])
def test_panel_cannot_repair_short_denominator_or_out_of_range_membership(count, eligible):
    with pytest.raises(ValueError):
        pilot.uniform_panel(count, eligible)


def test_contract_is_explicit_train_pilot_not_feature_approval():
    value = pilot.contract()
    assert value["updates_per_arm"] == 1000 and value["exposures_per_arm"] == 5000
    assert value["arm_order"] == ["control", "retention"]
    assert value["coefficient"] == .25
    assert value["panel_is_TRAIN_resubstitution_only"] is True
    for name in ("automatic_longer_training", "model_promoted", "simulator_control_authorized", "vehicle_control_approved", "evaluation_only_payloads_opened"):
        assert value[name] is False
    assert value["official_main_features_approved"] == value["official_subfeatures_approved"] == 0
    assert pilot.frozen.source_snapshot().items() <= value["sources"].items()


def test_retention_wiring_passes_actual_masks_sample_id_and_exact_parent():
    batch = dict(sample_id=["synthetic_old_train"], spatial_valid=torch.ones(1, 33, dtype=torch.bool),
        spatial_curvature_valid=torch.ones(1, 33, dtype=torch.bool), spatial_speed_valid=torch.ones(1, 33, dtype=torch.bool))
    raw = dict(curvature_knots=torch.zeros(1, 33, requires_grad=True), preferred_speed_knots=torch.ones(1, 33, requires_grad=True))
    proof = pilot.retention.ReplayProvenance(batch["sample_id"][0], "TRAIN", "OLD_RECOVERY_REPLAY", "a" * 64, "b" * 64, "c" * 64, True, False)
    assert pilot.retention_term(raw, raw, batch, proof)["total"].item() == 0


def test_pilot_never_calls_an_evaluation_only_loader():
    tree = ast.parse(Path(pilot.__file__).read_text())
    called = {node.func.attr for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
    assert "load_evaluation_only" not in called
    assert "load_cohorts" not in called
    assert "load_train" in called and "gpu0_lease" in called


def test_an_early_error_cannot_write_to_existing_or_foreign_output(monkeypatch, tmp_path):
    sentinel = tmp_path / "user_file.txt"; sentinel.write_text("untouched")
    monkeypatch.setattr(pilot, "parser", lambda: Namespace(parse_args=lambda _: Namespace(output=tmp_path)))
    def reject(_): raise RuntimeError("pre-creation guard")
    monkeypatch.setattr(pilot, "run", reject)
    with pytest.raises(RuntimeError, match="pre-creation"):
        pilot.main([])
    assert sorted(p.name for p in tmp_path.iterdir()) == ["user_file.txt"]
    assert sentinel.read_text() == "untouched" and pilot._created_output is None


def test_teacher_cache_rejects_changed_input_for_same_token(monkeypatch):
    batch = {"sample_id": ["same-token"]}
    cache = {"same-token": ("previous-bytes", {})}
    monkeypatch.setattr(pilot.pair, "batch_digest", lambda _: "different-bytes")
    with pytest.raises(ValueError, match="different input bytes"):
        pilot.teacher_raw(None, batch, torch.device("cpu"), cache)
