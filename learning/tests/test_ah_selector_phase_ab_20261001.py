"""HH_261001 - Prove exact TRAIN coverage, STOP-preserving weights, and unchanged generated candidates after head-only fit."""

from types import SimpleNamespace

import pytest
import torch
from torch.utils.data.dataloader import default_collate

from portable_e2e.model import ModelConfig, PHYSICAL_STOPMIX_MODEL_ID, PerspectiveTrajectoryModel
from scripts.e2e import selector_phase_research_20261001 as phase
from scripts.e2e import run_ah_selector_phase_ab_20261001 as experiment


def test_fixed_schedule_covers_every2787_with_identical_orders_and_no_batch_duplicates():
    batches = phase.schedule(2787)
    assert len(batches) == 3000 and batches == phase.schedule(2787)
    flat = [i for batch in batches for i in batch]
    assert len(flat) == 12000 and set(flat) == set(range(2787))
    assert all(len(set(batch)) == 4 for batch in batches)
    assert min(flat.count(i) for i in range(2787)) == 4
    assert max(flat.count(i) for i in range(2787)) == 5


def test_stop_priority_exact_start_thresholds_and_no_target_mutation():
    vx = torch.tensor([0., 2., 0., .5, .5, -.6])
    target = torch.ones(6, 64)
    target[1] = .1; target[3, 2] = .6; target[4, 2] = .61
    valid = torch.ones(6, 64, dtype=torch.bool)
    oracle = torch.tensor([8, 1, 5, 5, 5, 2])
    before = [value.clone() for value in (vx, target, valid, oracle)]
    assert phase.phase_ids(vx, target, valid, oracle).tolist() == [0, 0, 1, 2, 1, 3]
    assert all(torch.equal(a, b) for a, b in zip(before, (vx, target, valid, oracle)))


def test_fixed_half_mixture_preserves_positive_stop_loss_and_costs_are_detached():
    groups = torch.tensor([0, 1, 2, 3, 3, 3, 3, 3])
    batches = [(0, 1, 2, 3), (4, 5, 6, 7)]
    weights, counts = phase.phase_weights(groups, batches)
    assert counts.tolist() == [1, 1, 1, 5]
    assert torch.allclose(weights, torch.tensor([1.5, 1.5, 1.5, .7]))
    logits = torch.zeros(8, 12, requires_grad=True)
    costs = torch.ones(8, 12)
    costs[:, 8] = 0.; costs.requires_grad_(True)
    loss = phase.selector_loss(logits, costs, weights[groups])
    expected = .5 * phase.selector_loss(logits, costs, torch.ones(8))
    expected += .5 * sum(phase.selector_loss(logits[groups == i], costs[groups == i],
                                           torch.ones(int((groups == i).sum()))) for i in range(4)) / 4
    assert torch.allclose(loss, expected)
    loss.backward()
    assert bool((logits.grad[:, 8] < 0).all()) and costs.grad is None
    with pytest.raises(ValueError, match="all TRAIN"):
        phase.phase_weights(groups, [(0, 1, 2, 3)])


def test_actual_cpu_head_fit_preserves_trunk_buffers_and_all_twelve_candidate_outputs(tmp_path, monkeypatch):
    torch.set_num_threads(2)
    cfg = ModelConfig(model_id=PHYSICAL_STOPMIX_MODEL_ID, candidate_count=12, maximum_step_m=1.,
        image_width=80, image_height=48, image_embedding=8, camera_fusion_width=8,
        ego_embedding=8, route_embedding=8, hidden_width=256, encoder_base_channels=8)
    model = PerspectiveTrajectoryModel(cfg).eval()
    initial = experiment.base._clone_state(model.state_dict())

    class Dataset:
        def __len__(self): return 4
        def __getitem__(self, i):
            ego = torch.zeros(10, 13); ego[:, 0] = 1.; ego[:, 1] = float(i); ego[:, 10] = 1.
            target_speed = torch.full((64,), float(i))
            return dict(images=torch.zeros(6, 3, 48, 80), calibration=torch.zeros(6, 16),
                ego_history=ego, ego_history_mask=torch.ones(10, dtype=torch.bool),
                route_xy=torch.stack((torch.arange(128), torch.zeros(128)), 1).float(),
                route_mask=torch.ones(128, dtype=torch.bool), target_speed_mps=target_speed,
                target_xy=torch.stack((torch.arange(1, 65) * .1 * i, torch.zeros(64)), 1),
                target_yaw_rad=torch.zeros(64), target_valid=torch.ones(64, dtype=torch.bool))

    dataset = Dataset()
    cache, proof = experiment.make_cache(model, dataset, torch.device("cpu"))
    assert proof["original_loss_sha256"] == experiment.base.digest(experiment.Path(experiment.loss_module.__file__))
    cache_hash = experiment.base.tensor_digest(cache)
    groups = torch.arange(4)
    batches = [(0, 1, 2, 3)] * 2
    weights, _ = phase.phase_weights(groups, batches)
    captured = []
    original_save = experiment.base.save_new
    def save(path, payload):
        captured.append(payload); return original_save(path, payload)
    monkeypatch.setattr(experiment.base, "save_new", save)
    monkeypatch.setattr(experiment, "STEPS", 2)
    monkeypatch.setattr(experiment, "POSITIONS", 8)
    declaration = dict(experiment.contract(), training_dataset_fingerprint_sha256="test-only", exposure_order_sha256="test-only")
    receipt = experiment.fit(experiment.ARMS[1], initial, cfg, cache, groups, weights, batches, tmp_path,
                             torch.device("cpu"), declaration, cache_hash)
    assert receipt["generator_bit_identity"] and receipt["cache_bit_identity"]
    assert (tmp_path / receipt["checkpoint"]).is_file()
    after = PerspectiveTrajectoryModel(cfg).eval(); after.load_state_dict(captured[0]["model_state_dict"])
    batch = default_collate([dataset[i] for i in range(4)])
    with torch.no_grad():
        old = model(*(batch[name] for name in experiment.previous.FIELDS))
        new = after(*(batch[name] for name in experiment.previous.FIELDS))
    assert torch.equal(old[0], new[0]) and torch.equal(old[1], new[1])
    assert not torch.equal(old[2], new[2])
    assert experiment.base.tensor_digest(phase.generator_state(initial)) == receipt["frozen_generator_state_sha256"]


def test_contract_defers_dev_and_documents_common_sampling_change():
    cfg = experiment.contract()
    assert cfg["trainable_parameters"] == 3084 and cfg["steps_per_arm"] == 3000
    assert cfg["sampling_changed_vs_previous_full_model_pair"] and cfg["no_new_oracle"]
    assert cfg["train_cache_only_before_both_finals"] and not cfg["dev_during_training"]
    root = experiment.Path(__file__).resolve().parents[2]
    assert experiment.base.digest(root / experiment.PLAN_RELATIVE) == experiment.PLAN_SHA256
