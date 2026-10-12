"""HH_260906 - Verify the causal route-ranker leakage and preservation contract."""

import inspect

import pytest
import torch

from portable_e2e import frozen_causal_route_ranker as module
from portable_e2e.contract import ContractError


@pytest.fixture(autouse=True)
def cpu_single_thread():
    """HH_260906 - Keep tiny deterministic optimizer tests inexpensive."""
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def route(turn_radians=0.0, count=module.ROUTE_POINT_COUNT):
    """HH_260906 - Build one variable-length causal route fixture."""
    result = []
    for index in range(count):
        fraction = index / (count - 1)
        angle = turn_radians * fraction
        result.append([index * torch.cos(torch.tensor(angle)).item(), index * torch.sin(torch.tensor(angle)).item()])
    return result


def raw_sample(sample_id="sample:0", command=3, turn_radians=0.0):
    """HH_260906 - Create one row with forbidden payloads that extraction must ignore."""
    return {
        "sample_id": sample_id,
        "anchor_timestamp_ns": 100,
        "episode_id": "forbidden-episode",
        "map_id": "forbidden-map",
        "camera_bundle": ["forbidden-image"],
        "labels": {"planning": {"target": "forbidden-target"}},
        "ego": {
            "timestamp_ns": 99,
            "linear_velocity_base_mps": [4.0, 0.0, 0.0],
            "linear_acceleration_base_mps2": [0.1, 0.0, 0.0],
            "future": "forbidden-future-ego",
        },
        "navigation": {
            "command": command,
            "command_timestamp_ns": 99,
            "route_source_timestamp_ns": 98,
            "route_polyline_base_m": route(turn_radians),
            "map_id": "forbidden-navigation-map",
        },
    }


def cache(split="train", count=12):
    """HH_260906 - Build a fixed-shape immutable F cache fixture."""
    generator = torch.Generator().manual_seed(121 if split == "train" else 122)
    xy = torch.zeros(count, 12, 64, 2)
    time = torch.arange(1, 65, dtype=torch.float32) * 0.1
    for candidate in range(12):
        xy[:, candidate, :, 0] = time * (candidate + 1) * 0.5
        xy[:, candidate, :, 1] = candidate * 0.02
    speed = torch.ones(count, 12, 64)
    logits = torch.randn(count, 12, generator=generator)
    logits[:, 2] = 8.0
    logits[:, 8] = 3.0
    if split == "train":
        logits[-3:, 8] = 10.0
    else:
        logits[-2:, 8] = 10.0
    return module.CachedStopmixSelectorData(
        split=split,
        sample_ids=tuple(f"{split}:{index}" for index in range(count)),
        episode_ids=tuple(f"{split}-episode-{index // 4}" for index in range(count)),
        fused=torch.randn(count, 256, generator=generator),
        candidate_xy=xy,
        candidate_speed=speed,
        target_xy=xy[:, 0],
        target_speed_mps=torch.ones(count, 64),
        target_yaw_rad=torch.zeros(count, 64),
        target_valid=torch.ones(count, 64, dtype=torch.bool),
        source_checkpoint_sha256=module.SOURCE_CHECKPOINT_SHA256,
        corpus_fingerprint_sha256="a" * 64,
        dataset_fingerprint_sha256=("b" if split == "train" else "c") * 64,
        original_logits=logits,
    )


def sidecar(data):
    """HH_260906 - Build one aligned sidecar with three TRAIN fold groups."""
    count = len(data)
    groups = (
        tuple(module.ROUTE_GROUPS[index // 4] for index in range(count))
        if data.split == "train"
        else tuple("town03_dev" for _ in range(count))
    )
    return module.CausalRouteSidecar(
        split=data.split,
        sample_ids=data.sample_ids,
        route_groups=groups,
        route_xy=torch.tensor([route() for _ in range(count)], dtype=torch.float32),
        route_mask=torch.ones(count, module.ROUTE_POINT_COUNT, dtype=torch.bool),
        ego_speed_mps=torch.full((count,), 4.0),
        ego_acceleration_mps2=torch.zeros(count),
        command=torch.full((count,), 3, dtype=torch.int64),
        straight_ranker_mask=torch.tensor(
            [index % 4 < 2 for index in range(count)], dtype=torch.bool
        ),
        route_heading_change_rad=torch.zeros(count),
    )


def test_extract_causal_row_serializes_only_current_route_and_ego_inputs():
    """HH_260906 - Prove labels, images, future state and identities do not enter extraction."""
    row = module.extract_causal_row(raw_sample())
    assert set(row) == {
        "sample_id",
        "route_xy",
        "ego_speed_mps",
        "ego_acceleration_mps2",
        "command",
        "route_heading_change_rad",
        "causality",
    }
    encoded = repr(row).lower()
    assert all(word not in encoded for word in ("label", "target", "image", "map", "episode", "future", "fused"))
    assert row["ego_speed_mps"] == 4.0
    assert row["ego_acceleration_mps2"] == 0.1
    assert row["causality"] == {
        "anchor_timestamp_ns": 100,
        "ego_timestamp_ns": 99,
        "command_timestamp_ns": 99,
        "route_source_timestamp_ns": 98,
    }


@pytest.mark.parametrize("field", ["ego", "command", "route"])
def test_extract_causal_row_rejects_future_state_or_command(field):
    """HH_260906 - Fail closed if any supposed input timestamp follows the anchor."""
    sample = raw_sample()
    if field == "ego":
        sample["ego"]["timestamp_ns"] = 101
    elif field == "command":
        sample["navigation"]["command_timestamp_ns"] = 101
    else:
        sample["navigation"]["route_source_timestamp_ns"] = 101
    with pytest.raises(ContractError, match="future"):
        module.extract_causal_row(sample)


def test_causal_turn_latch_keeps_late_lane_follow_rows_on_exact_f_fallback():
    """HH_260906 - Preserve turn fallback after route geometry becomes locally straight."""
    straight, latched, reason = module.causal_straight_decision(3, 0.8, False)
    assert (straight, latched) == (False, True)
    assert "TURN" in reason
    straight, latched, reason = module.causal_straight_decision(3, 0.0, latched)
    assert (straight, latched) == (False, True)
    assert "LATCH" in reason
    assert module.causal_straight_decision(3, 0.01, False)[:2] == (True, False)
    assert module.causal_straight_decision(3, 0.2, False)[:2] == (False, False)


def test_sidecar_requires_exact_ids_and_clones_input_tensors():
    """HH_260906 - Bind raw rows to cache IDs without retaining caller-owned storage."""
    ids = ("a", "b")
    rows = {sample_id: module.extract_causal_row(raw_sample(sample_id)) for sample_id in ids}
    groups = {sample_id: "town07_same_physical_route" for sample_id in ids}
    policy = {"a": True, "b": False}
    value = module.make_sidecar("train", ids, rows, groups, policy)
    before = value.digest()
    rows["a"]["route_xy"] = tuple(tuple(point) for point in route(0.05))
    assert value.digest() == before
    with pytest.raises(ContractError, match="cover"):
        module.make_sidecar("train", ids, {"a": rows["a"]}, groups, policy)


def test_candidate_costs_are_shared_nonnegative_and_do_not_accept_logits():
    """HH_260906 - Keep the scorer API independent of labels, fused context and F ranking."""
    count = 2
    route_xy = torch.tensor([route() for _ in range(count)], dtype=torch.float32)
    route_mask = torch.ones(count, module.ROUTE_POINT_COUNT, dtype=torch.bool)
    xy = torch.zeros(count, 6, 64, 2)
    speed = torch.zeros(count, 6, 64)
    for candidate in range(6):
        xy[:, candidate, :, 0] = torch.arange(1, 65) * 0.1
        xy[:, candidate, :, 1] = candidate * 0.2
        speed[:, candidate] = 2.0 + candidate
    costs = module.candidate_route_costs(
        route_xy,
        route_mask,
        torch.full((count,), 4.0),
        torch.zeros(count),
        xy,
        speed,
    )
    assert costs.shape == (2, 6, 4)
    assert torch.isfinite(costs).all() and (costs >= 0).all()
    assert costs[0, 0, 0] < costs[0, 5, 0]
    assert tuple(inspect.signature(module.candidate_route_costs).parameters) == (
        "route_xy",
        "route_mask",
        "ego_speed_mps",
        "ego_acceleration_mps2",
        "candidate_xy",
        "candidate_speed",
    )


@pytest.mark.parametrize("raw_count", [4, 49, 121])
def test_variable_raw_route_lengths_use_model_resampling_and_mask(raw_count):
    """HH_260906 - Match the model's 128-point resampling and exclude every padded point."""
    sample = raw_sample()
    sample["navigation"]["route_polyline_base_m"] = route(count=raw_count)
    row = module.extract_causal_row(sample)
    ids = (row["sample_id"],)
    value = module.make_sidecar(
        "train",
        ids,
        {row["sample_id"]: row},
        {row["sample_id"]: "town07_same_physical_route"},
        {row["sample_id"]: True},
    )
    valid = int(value.route_mask.sum())
    assert 2 <= valid <= module.ROUTE_POINT_COUNT
    assert value.route_mask[0, :valid].all()
    assert not value.route_mask[0, valid:].any()
    changed = value.route_xy.clone()
    changed[~value.route_mask] = 1.0e6
    xy = torch.zeros(1, 6, 64, 2)
    xy[..., 0] = torch.arange(1, 65) * 0.1
    speed = torch.ones(1, 6, 64)
    first = module.candidate_route_costs(
        value.route_xy,
        value.route_mask,
        value.ego_speed_mps,
        value.ego_acceleration_mps2,
        xy,
        speed,
    )
    second = module.candidate_route_costs(
        changed,
        value.route_mask,
        value.ego_speed_mps,
        value.ego_acceleration_mps2,
        xy,
        speed,
    )
    assert torch.equal(first, second)


def test_selection_metrics_uses_weighted_candidate_score_not_featurewise_argmin():
    """HH_260906 - Catch the invalid per-feature argmin shape from the previous draft."""
    costs = torch.full((1, 6, 4), 9.0)
    costs[0, 0] = torch.tensor([0.0, 9.0, 9.0, 9.0])
    costs[0, 1] = torch.tensor([9.0, 0.0, 9.0, 9.0])
    costs[0, 2] = torch.tensor([9.0, 9.0, 0.0, 9.0])
    costs[0, 3] = torch.tensor([9.0, 9.0, 9.0, 0.0])
    costs[0, 4] = torch.ones(4)
    scores = -costs.sum(dim=2)
    result = module._selection_metrics(
        scores,
        costs,
        torch.tensor([4]),
        torch.tensor([True]),
        torch.ones(4),
    )
    assert result["oracle_agreement_count"] == 1
    assert result["mean_weighted_alignment_regret"] == 0.0


def test_scorer_has_four_nonnegative_shared_weights_and_no_candidate_parameters():
    """HH_260906 - Reject candidate-index bias or unconstrained negative cost weights."""
    scorer = module.make_scorer()
    assert set(scorer.state_dict()) == {"raw_weights"}
    assert sum(parameter.numel() for parameter in scorer.parameters()) == 4
    assert scorer.effective_weights().tolist() == pytest.approx([1.0] * 4)
    costs = torch.rand(3, 6, 4)
    scores = scorer(costs)
    assert scores.shape == (3, 6)
    scores.sum().backward()
    assert scorer.raw_weights.grad is not None


def test_mode_stop_and_turn_fallback_rows_are_byte_exact():
    """HH_260906 - Allow changes only on confident-straight original DRIVE rows."""
    original = torch.zeros(4, 12)
    original[0, 2] = 8.0
    original[1, 3] = 8.0
    original[2, 8] = 8.0
    original[3, 1] = 8.0
    scores = torch.arange(6, dtype=torch.float32).repeat(4, 1)
    policy = torch.tensor([True, False, True, True])
    result = module.mode_and_turn_preserving_logits(original, scores, policy)
    assert not torch.equal(result[0], original[0])
    assert torch.equal(result[1], original[1])
    assert torch.equal(result[2], original[2])
    assert not torch.equal(result[3], original[3])
    proof = module.preservation_proof(original, result, policy)
    assert proof["pass"] is True
    assert proof["fallback_row_logit_mismatch_count"] == 0
    assert torch.equal(result.argmax(1) < 6, original.argmax(1) < 6)


def test_short_fit_uses_all_drive_rows_then_one_post_freeze_dev_phase(monkeypatch):
    """HH_260906 - Exercise three nonempty LOO fits before one post-freeze DEV phase."""
    train, dev = cache(), cache("val", 6)
    train_sidecar, dev_sidecar = sidecar(train), sidecar(dev)
    monkeypatch.setattr(module.h_core, "validate_pair", lambda *_args: None)
    monkeypatch.setattr(module, "TRAIN_COUNT", 12)
    monkeypatch.setattr(module, "DEV_COUNT", 6)
    monkeypatch.setattr(module, "TOTAL_COUNT", 18)
    monkeypatch.setattr(module, "EXPECTED_TRAIN_DRIVE_COUNT", 9)
    monkeypatch.setattr(module, "EXPECTED_TRAIN_STOP_COUNT", 3)
    monkeypatch.setattr(module, "FIT_STEPS", 2)
    monkeypatch.setattr(module.projected, "_validate_projection", lambda *_args: None)
    target_calls = []

    def targets(data, _function, _deadline=None):
        """HH_260906 - Record that only TRAIN constructs projected supervision targets."""
        target_calls.append(data.split)
        costs = torch.arange(12, dtype=torch.float32, device=data.fused.device)
        costs = costs.unsqueeze(0).repeat(len(data), 1)
        return costs, costs.argmin(1)

    monkeypatch.setattr(module.projected, "projected_targets", targets)
    evaluation_calls = []
    original_evaluate = module.cache_core.evaluate_logits

    def evaluate(*args, **kwargs):
        """HH_260906 - Record the single core combined DEV metric reduction."""
        evaluation_calls.append(args[0].split)
        return original_evaluate(*args, **kwargs)

    monkeypatch.setattr(module.cache_core, "evaluate_logits", evaluate)
    transfer_events = []
    original_cache_to = module.CachedStopmixSelectorData.to

    def audited_cache_to(self, device):
        """HH_260906 - Record when each sealed cache is transferred to the fit device."""
        transfer_events.append(self.split + "_cache_to")
        return original_cache_to(self, device)

    def progress(phase, row):
        """HH_260906 - Mark the final TRAIN optimization completion boundary."""
        if phase == "final_all_train" and row["step"] == 2:
            transfer_events.append("final_train_fit_complete")

    monkeypatch.setattr(module.CachedStopmixSelectorData, "to", audited_cache_to)
    before = train.digest(), dev.digest(), train_sidecar.digest(), dev_sidecar.digest()
    result = module.fit_causal_route_ranker(
        train,
        dev,
        train_sidecar,
        dev_sidecar,
        projected_costs_fn=lambda *_args, **_kwargs: None,
        projection_proof={},
        callback=progress,
    )
    report = result["report"]
    assert target_calls == ["train"]
    assert evaluation_calls == ["val"]
    partition = report["train_partition"]
    assert partition["sample_count"] == 12
    assert partition["f_original_drive_count"] == 9
    assert partition["f_original_stop_count"] == 3
    assert partition["stop_rows_used_for_loss"] == 0
    assert partition["drive_rows_used_for_loss"] == 9
    assert sum(partition["drive_count_by_physical_route_group"].values()) == 9
    folds = report["train_only_leave_one_route_group_out"]
    assert [fold["held_route_group"] for fold in folds] == list(module.ROUTE_GROUPS)
    assert all(fold["fit_drive_count"] > 0 and fold["held_drive_count"] > 0 for fold in folds)
    assert all("held_policy_straight_drive_count" in fold for fold in folds)
    assert all(fold["fit_steps"] == 2 for fold in folds)
    assert partition["final_fit_optimization_row_exposures"] == 18
    assert partition["all_fold_and_final_optimization_row_exposures"] == 54
    assert partition["stop_optimization_row_exposures"] == 0
    assert report["structure_and_budget_fixed_before_dev"] is True
    assert report["dev_post_freeze_phase_count"] == 1
    assert report["core_combined_full_split_metric_reduction_count"] == 1
    assert report[
        "dev_tensors_transferred_only_after_final_train_scorer_frozen"
    ] is True
    assert report["dev_used_for_training"] is False
    assert report["test_read_or_used"] is False
    assert all(value["pass"] for value in report["final_preservation"].values())
    assert transfer_events.index("val_cache_to") > transfer_events.index(
        "final_train_fit_complete"
    )
    assert (train.digest(), dev.digest(), train_sidecar.digest(), dev_sidecar.digest()) == before
