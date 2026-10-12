"""HH_261001 - Verify prospective mixed TRAIN lineage, sampling, masks and numeric-only exports."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from portable_e2e.dataset import TrainingExample
from portable_e2e.deterministic_linear_interpolation_v2 import DeterministicPathTimeDecoderV2
from scripts.e2e import train_temporal_v2_balanced_replay_20261001 as trainer


def strata():
    return dict(curved={"C": [0, 1, 2], "T": [3, 4]},
                straight={"C": [5, 6], "O": [7, 8, 9], "T": [10, 11]},
                eligible=list(range(12)), excluded=[12])


def batch():
    history = torch.zeros(4, 10, 13); history[:, -1, 1] = torch.tensor([2.484, 2.484, 8.425, 4.12])
    speed_valid = torch.zeros(4, 33, dtype=torch.bool)
    for i, count in enumerate((8, 8, 27, 14)):
        speed_valid[i, :count] = True
    xy = torch.zeros(4, 64, 2); xy[:, :, 0] = torch.arange(1, 65)[None] * .1 * history[:, -1, 1, None]
    return dict(ego_history=history, ego_history_mask=torch.ones(4, 10, dtype=torch.bool),
        route_xy=torch.tensor([[[0., 0.], [64., 0.]]]*4), route_mask=torch.ones(4, 2, dtype=torch.bool),
        spatial_valid=speed_valid.clone(), spatial_speed_valid=speed_valid,
        spatial_curvature_valid=speed_valid.clone(), spatial_curvature=torch.zeros(4, 33),
        spatial_preferred_speed=history[:, -1, 1, None].expand(4, 33).clone(),
        target_xy=xy, target_speed_mps=history[:, -1, 1, None].expand(4, 64).clone(),
        target_valid=torch.ones(4, 64, dtype=torch.bool), intent_valid=torch.zeros(4, dtype=torch.bool),
        intent_target=torch.zeros(4, dtype=torch.long), stop_distance_valid=torch.zeros(4, dtype=torch.bool),
        stop_distance_target=torch.zeros(4))


def raw():
    return dict(curvature_knots=torch.full((4, 33), .003, requires_grad=True),
                preferred_speed_knots=torch.full((4, 33), 3., requires_grad=True),
                intent_logits=torch.zeros(4, 3, requires_grad=True),
                stop_distance_raw=torch.zeros(4, requires_grad=True))


def test_exact_two_one_one_sampler_and_all_canonical_indices():
    rows = list(trainer.balanced_batches(7, 13, strata(), updates=24))
    assert all(len(r) == 4 and all(0 <= i < 7 for i in r[:2]) for r in rows)
    assert all(r[2]-7 in range(5) and r[3]-7 in range(5, 12) for r in rows)
    assert set(i for r in rows for i in r[2:]) == set(range(7, 19))
    assert 19 not in {i for r in rows for i in r}
    flat = [i for r in rows for i in r[:2]]
    assert all(set(flat[i:i+7]) == set(range(7)) for i in range(0, len(flat)-6, 7))


def test_map_round_robin_and_stratum_queues_do_not_drop_tails():
    rows = list(trainer.balanced_batches(7, 13, strata(), updates=30))
    curved = [r[2]-7 for r in rows]
    assert all(i in (0, 1, 2) for i in curved[::2])
    assert all(i in (3, 4) for i in curved[1::2])
    assert set(curved[:6:2]) == {0, 1, 2}
    straight = [r[3]-7 for r in rows]
    assert all(i in (5, 6) for i in straight[::3])
    assert all(i in (7, 8, 9) for i in straight[1::3])
    assert all(i in (10, 11) for i in straight[2::3])


def test_resume_reconstructs_exact_suffix_and_fixed3000_horizon():
    all_rows = list(trainer.balanced_batches(327, 13, strata()))
    assert len(all_rows) == 3000
    assert list(trainer.balanced_batches(327, 13, strata(), start_update=1500)) == all_rows[1500:]
    assert all_rows == list(trainer.balanced_batches(327, 13, strata()))


@pytest.mark.parametrize("mutation", ["duplicate", "out_of_range", "eligible_omission", "excluded_leak", "empty_map"])
def test_sampler_rejects_membership_leakage(mutation):
    s = strata()
    if mutation == "duplicate":s["straight"]["C"].append(1)
    if mutation == "out_of_range":s["curved"]["C"].append(13)
    if mutation == "eligible_omission":s["eligible"].append(12)
    if mutation == "excluded_leak":s["excluded"].append(5)
    if mutation == "empty_map":s["straight"]["C"] = []
    with pytest.raises(ValueError):list(trainer.balanced_batches(7, 13, s, updates=3))


@pytest.mark.parametrize("updates,start", [(3001, 0), (3000, -1), (3000, 3001), (True, 0)])
def test_sampler_rejects_unbounded_intervals(updates, start):
    with pytest.raises(ValueError):list(trainer.balanced_batches(7, 13, strata(), updates=updates, start_update=start))


def view(split, episode, token):
    return SimpleNamespace(split=split, examples=[SimpleNamespace(episode_id=episode, token=token)])


def test_no_repeated_view_train_val_or_test_admission():
    low, old, dev = view("train", "low", "a"), view("train", "old", "b"), view("val", "dev", "c")
    trainer.verify_train_disjoint(low, old, dev)
    with pytest.raises(ValueError, match="duplicate"):
        trainer.verify_train_disjoint(low, view("train", "other", "a"), dev)
    with pytest.raises(ValueError, match="episode overlap"):
        trainer.verify_train_disjoint(low, view("train", "low", "b"), dev)
    with pytest.raises(ValueError, match="episode overlap"):
        trainer.verify_train_disjoint(low, old, view("val", "old", "c"))
    with pytest.raises(ValueError, match="TRAIN and known VAL"):
        trainer.verify_train_disjoint(low, view("test", "old", "b"), dev)
    with pytest.raises(ValueError, match="TRAIN and known VAL"):
        trainer.verify_train_disjoint(low, old, view("test", "dev", "c"))


def test_curvature_strata_use_true_valid_geometry_not_navigation_command(monkeypatch):
    rows = [SimpleNamespace(features=(1., 4., 0.)+(0.,)*10, map_id="map", token=str(i)) for i in range(3)]
    def labels(row):
        return dict(spatial_valid=torch.tensor([True, True]), spatial_curvature_valid=torch.tensor([True, False]),
                    spatial_curvature=torch.tensor([.02 if row.token == "0" else .0, .2]))
    monkeypatch.setattr(trainer, "spatial_expert_labels", labels)
    s = trainer.legacy_strata(SimpleNamespace(examples=rows))
    assert s["curved"] == {"map": [0]}
    assert s["straight"] == {"map": [1, 2]}


def test_decoder_receives_only_two_real_low_speed_v0_and_causal_navigation():
    data = batch(); original = {k: v.clone() for k, v in data.items()}
    decoder = DeterministicPathTimeDecoderV2()
    class Spy:
        config = decoder.config
        def __call__(self, c, s, v0, intent, stop, support):
            assert c.shape == s.shape == (2, 33)
            assert v0.tolist() == pytest.approx([2.484, 2.484])
            assert support.tolist() == [64., 64.]
            return decoder(c, s, v0, intent, stop, support)
    decoded, low, old, loss = trainer.balanced_objective(raw(), Spy(), data)
    assert decoded["instantaneous_speed"].shape == (2, 65)
    assert loss.item() == pytest.approx(.5*low["total"].item()+.5*old["total"].item())
    loss.backward()
    assert all(torch.equal(original[k], v) for k, v in data.items())
    assert data["ego_history"][2, -1, 1].item() == pytest.approx(8.425)


def test_per_anchor_raw_masks_are_normalized_before_two_item_domain_mean():
    data = batch(); outputs = raw()
    data["spatial_speed_valid"][2] = False; data["spatial_speed_valid"][2, :2] = True
    data["spatial_speed_valid"][3] = False; data["spatial_speed_valid"][3, :20] = True
    with torch.no_grad():
        outputs["preferred_speed_knots"][2] = data["spatial_preferred_speed"][2] + 1.
        outputs["preferred_speed_knots"][3] = data["spatial_preferred_speed"][3] + 2.
    _, _, old, _ = trainer.balanced_objective(outputs, DeterministicPathTimeDecoderV2(), data)
    assert old["raw_preferred_speed"].item() == pytest.approx((.5+1.5)/2)


def test_masked_invalid_legacy_targets_do_not_change_raw_loss():
    data = batch(); outputs = raw(); decoder = DeterministicPathTimeDecoderV2()
    _, _, old, _ = trainer.balanced_objective(outputs, decoder, data)
    data["spatial_preferred_speed"][2:][~data["spatial_speed_valid"][2:]] = 999.
    data["target_xy"][2:] = 999. # HH_261001 - Legacy time labels are retained but not decoded in this experiment.
    _, _, changed, _ = trainer.balanced_objective(outputs, decoder, data)
    assert old["total"].item() == changed["total"].item()


@pytest.mark.parametrize("field", ["intent_valid", "stop_distance_valid", "initial_acceleration_valid"])
def test_new_semantic_labels_require_new_declaration(field):
    data = batch(); data[field] = torch.tensor([False, False, False, True])
    with pytest.raises(ValueError, match="separately declared"):
        trainer.balanced_objective(raw(), DeterministicPathTimeDecoderV2(), data)


def test_domain_gradient_check_does_not_populate_optimizer_gradients():
    model = torch.nn.Linear(2, 1); loss = model(torch.ones(1, 2)).square().mean()
    report = trainer.gradient_receipt(loss, model)
    assert report["all_present_parameter_gradients_finite"]
    assert report["gradient_l2_norm"] > 0
    assert all(p.grad is None for p in model.parameters())
    loss.backward()
    assert all(torch.isfinite(p.grad).all() for p in model.parameters())


def test_domain_gradient_check_rejects_nonfinite_before_update():
    model = torch.nn.Linear(2, 1); loss = model(torch.ones(1, 2)).sum()*float("inf")
    with pytest.raises(ValueError, match="nonfinite domain gradient"):
        trainer.gradient_receipt(loss, model)


def resume_document():
    return dict(schema=trainer.SCHEMA, optimizer_updates=500, examples_seen=2000,
        v4_examples_seen=1000, legacy_examples_seen=1000, parent_checkpoint_sha256=trainer.PARENT_SHA,
        parent_source_optimizer_updates=200, known_total_optimizer_updates=700, maximum_optimizer_updates=3000,
        batch_domains=["v4_joint", "v4_joint", "legacy_curved_raw", "legacy_straight_raw"],
        training_dataset_fingerprint_sha256="identity", source_sha256={}, optimizer_state_dict={}, **trainer.scope_fields())


@pytest.mark.parametrize("field,value", [("examples_seen", 1999), ("v4_examples_seen", 999),
    ("legacy_examples_seen", 999), ("parent_checkpoint_sha256", "0"*64), ("one_episode_screening", True),
    ("main_training", True), ("training_episode_count", 1), ("legacy_decoded", True),
    ("schema", "old-single-episode-schema"), ("source_sha256", {"bad": "sha"}),
    ("parent_source_optimizer_updates", 3000), ("known_total_optimizer_updates", 500),
    ("maximum_optimizer_updates", 9999), ("batch_domains", ["v4_joint"]*4)])
def test_resume_rejects_falsified_mixed_exposure_or_lineage(tmp_path, field, value):
    d = resume_document(); d[field] = value
    p = tmp_path / "state.pt"; torch.save(d, p)
    with pytest.raises(ValueError, match="mixed lineage"):
        trainer.verify_resume(p, trainer.digest(p), {}, "identity")


def test_resume_accepts_exact_mixed_scope_and_optimizer(tmp_path):
    p = tmp_path / "state.pt"; torch.save(resume_document(), p)
    assert trainer.verify_resume(p, trainer.digest(p), {}, "identity")["optimizer_updates"] == 500


def test_source_closure_is_explicit_and_keeps_live_runtime_unimported():
    snapshot = trainer.source_snapshot()
    assert all(snapshot[k] == v for k, v in trainer.FROZEN_SOURCES.items())
    assert "learning/portable_e2e/temporal_simulation_research_v2_20261001.py" not in snapshot
    assert "learning/portable_e2e/deterministic_linear_interpolation_v2.py" in snapshot
    assert "tools/e2e/train_temporal_v2_balanced_replay_20261001.py" in snapshot


def test_fresh_minimal_clone_imports_trainer_and_gpu_guard_dependencies(tmp_path):
    # HH_261001 - Copy only the declared closure; a full workspace cannot conceal a missing remote import.
    snapshot = trainer.source_snapshot()
    for name in snapshot:
        destination = tmp_path/name; destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(trainer.ROOT/name, destination)
    result = subprocess.run([sys.executable, "-B", str(tmp_path/"tools/workspace_python.py"),
        "-m", "scripts.e2e.train_temporal_v2_balanced_replay_20261001", "--", "--help"],
        cwd=tmp_path, capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert "--legacy-train" in result.stdout
    assert not (tmp_path/"learning/portable_e2e/temporal_simulation_research_v2_20261001.py").exists()


def test_recursive_closure_includes_function_local_imports_and_package_initializers():
    snapshot = trainer.source_snapshot()
    expected = {"learning/portable_e2e/__init__.py", "learning/portable_e2e/contract.py",
                "tools/e2e/train_temporal_v2_raw_bootstrap_20261001.py",
                "tools/e2e/train_temporal_v2_low_speed_joint_20261001.py"}
    assert expected <= set(snapshot)
    first = trainer.imported_source_closure(trainer.ROOT, [trainer.ROOT/"tools/e2e/train_temporal_v2_balanced_replay_20261001.py"])
    second = trainer.imported_source_closure(trainer.ROOT, [trainer.ROOT/"tools/e2e/train_temporal_v2_balanced_replay_20261001.py"])
    assert first == second and len(first) == len(set(first))


def test_export_new_mixed_schema_numeric_roundtrip_and_create_only(tmp_path):
    model = torch.nn.Linear(2, 1); model.config = SimpleNamespace(to_dict=lambda: dict(test_only=True))
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)
    loss = model(torch.ones(1, 2)).square().mean(); loss.backward(); optimizer.step()
    snapshot = trainer.source_snapshot()
    result = trainer.save_checkpoint(tmp_path, 500, model, optimizer, snapshot, "d"*64, "identity")
    meta = json.loads(Path(result["export_metadata"]).read_text())
    assert meta["schema"] == trainer.EXPORT_SCHEMA
    assert meta["one_episode_screening"] is False and meta["training_episode_count"] == 9
    assert meta["inference_loader_authorized"] is False and meta["model_promoted"] is False
    assert meta["v4_examples_seen"] == meta["legacy_examples_seen"] == 1000
    with np.load(result["export_npz"], allow_pickle=False) as arrays:
        assert set(arrays.files) == set(model.state_dict())
        assert all(arrays[k].dtype == np.float32 and np.array_equal(arrays[k], v.detach().numpy())
                   for k, v in model.state_dict().items())
    state = torch.load(result["path"], map_location="cpu")
    assert state["optimizer_state_dict"]["state"]
    before = Path(result["path"]).read_bytes()
    with pytest.raises(FileExistsError):trainer.save_checkpoint(tmp_path, 500, model, optimizer, snapshot, "d"*64, "identity")
    assert before == Path(result["path"]).read_bytes()


def test_schedule_identity_binds_inputs_and_curved_membership():
    a = trainer.training_identity(dict(v4="fp", legacy="fp2"), strata())
    b = trainer.training_identity(dict(v4="changed", legacy="fp2"), strata())
    s = strata(); s["curved"]["C"].reverse()
    assert a != b and a != trainer.training_identity(dict(v4="fp", legacy="fp2"), s)


def test_fixed_constants_and_fingerprint_lengths():
    assert trainer.MAX_UPDATES == 3000 and trainer.BATCH_SIZE == 4 and trainer.MAX_WALL_SECONDS == 2700
    assert trainer.scope_fields()["mixed_raw_joint_training"] is True
    for name in ("LOW_MANIFEST_SHA", "LOW_CONVERSION_SHA", "LOW_LOADER_FP", "LOW_TEMPORAL_FP",
                 "LEGACY_LOADER_FP", "LEGACY_TEMPORAL_FP", "DEV_LOADER_FP", "DEV_TEMPORAL_FP", "PARENT_SHA"):
        assert len(getattr(trainer, name)) == 64


def diagnostic_inputs():
    eight = dict(mean_total=.276282618)
    group = dict(count=599, raw_curvature=.1, raw_preferred_speed=.25)
    known = dict(full_fixed_eligible_denominator=599, groups=dict(all_eligible=group),
        per_episode={"right": dict(count=271, raw_curvature=.15, raw_preferred_speed=.2),
                     "straight": dict(count=328, raw_curvature=.03, raw_preferred_speed=.3)})
    v4 = dict(mean_ade_6p4s_m=.09, mean_fde_6p4s_m=.20, mean_instantaneous_speed_mae_mps=.02)
    return eight, known, v4


def test_endpoint_screens_have_fixed_thresholds_and_never_promote_features():
    eight, known, v4 = diagnostic_inputs()
    result = trainer.endpoint_screens(eight, eight, known, known, v4)
    assert result["all_diagnostic_screens_passed"] is True
    assert len(result["checks"]) == 10
    assert result["feature_approved"] is result["model_promoted"] is False
    raw8 = next(row for row in result["checks"] if row["name"] == "known_DEV8_raw_total")
    assert raw8["threshold"] == .303911


def test_endpoint_reports_speed_regression_and_episode_failure_honestly():
    eight, known, v4 = diagnostic_inputs()
    after = json.loads(json.dumps(known)); after["per_episode"]["right"]["raw_preferred_speed"] = 1.7
    result = trainer.endpoint_screens(eight, dict(mean_total=1.806), known, after, v4)
    assert result["all_diagnostic_screens_passed"] is False
    failed = [row["name"] for row in result["checks"] if not row["passed"]]
    assert "known_DEV8_raw_total" in failed and "known_DEV599/right/raw_preferred_speed" in failed


def test_endpoint_rejects_changed_known599_membership():
    eight, known, v4 = diagnostic_inputs(); changed = dict(known, full_fixed_eligible_denominator=598)
    with pytest.raises(ValueError, match="membership"):
        trainer.endpoint_screens(eight, eight, known, changed, v4)


def test_storage_reserve_fail_closed_without_creating_files(tmp_path, monkeypatch):
    required = trainer.MINIMUM_FREE_BYTES + trainer.CHECKPOINT_WRITE_RESERVE_BYTES
    monkeypatch.setattr(trainer.shutil, "disk_usage", lambda p: SimpleNamespace(free=required-1))
    with pytest.raises(ValueError, match="no CUDA/output/write"):
        trainer.storage_reserve(tmp_path)
    assert list(tmp_path.iterdir()) == []
    monkeypatch.setattr(trainer.shutil, "disk_usage", lambda p: SimpleNamespace(free=required))
    assert trainer.storage_reserve(tmp_path)["deletions_performed"] is False


def test_checkpoint_rechecks_reserve_before_any_write(tmp_path, monkeypatch):
    monkeypatch.setattr(trainer.shutil, "disk_usage", lambda p: SimpleNamespace(free=0))
    with pytest.raises(ValueError, match="reserve"):
        trainer.save_checkpoint(tmp_path, 500, None, None, {}, "sha", "fp")
    assert list(tmp_path.iterdir()) == []


def test_archived_source_bytes_are_pinned_independently_of_live_files(tmp_path):
    sources = trainer.source_snapshot()
    trainer.joint.archive_sources(tmp_path, sources)
    trainer.verify_sources(tmp_path, sources)
    target = tmp_path/"source"/"tools/e2e/train_temporal_v2_balanced_replay_20261001.py"
    # HH_261001 - Test-only corruption is confined to the temporary copied source fixture.
    target.write_bytes(b"corrupted archive")
    with pytest.raises(ValueError, match="archived source"):
        trainer.verify_sources(tmp_path, sources)
