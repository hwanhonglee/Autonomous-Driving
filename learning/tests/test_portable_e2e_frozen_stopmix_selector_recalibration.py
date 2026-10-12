"""HH_260906 - Verify paired F-selector recalibration without dataset, GPU or TEST access."""

from dataclasses import replace
import hashlib
import json

import pytest
import torch
from torch import nn

from portable_e2e import frozen_stopmix_selector_recalibration as module
from portable_e2e.contract import ContractError


@pytest.fixture(autouse=True)
def cpu_single_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def heads():
    with torch.random.fork_rng(devices=[]):
        torch.random.default_generator.manual_seed(61)
        return nn.Linear(256, 6), nn.Linear(256, 6)


def cache(split="train", count=8, original_heads=None):
    original_heads = heads() if original_heads is None else original_heads
    generator = torch.Generator().manual_seed(81 if split == "train" else 82)
    fused = torch.randn(count, 256, generator=generator) * 0.02
    xy = torch.zeros(count, 12, 64, 2)
    xy[..., 0] = torch.arange(1, 65) * 0.1
    xy[..., 1] = torch.arange(12)[None, :, None] * 0.05
    with torch.no_grad():
        logits = torch.cat(
            [
                torch.cat(
                    (
                        original_heads[0](fused[i : i + 4]),
                        original_heads[1](fused[i : i + 4]),
                    ),
                    dim=1,
                )
                for i in range(0, count, 4)
            ]
        )
    episode_count = 4 if split == "train" else 2
    return module.CachedStopmixSelectorData(
        split=split,
        sample_ids=tuple(f"{split}:{index}" for index in range(count)),
        episode_ids=tuple(
            f"{split}-episode-{index % episode_count}" for index in range(count)
        ),
        fused=fused,
        candidate_xy=xy,
        candidate_speed=torch.ones(count, 12, 64),
        target_xy=xy[:, 0],
        target_speed_mps=torch.ones(count, 64),
        target_yaw_rad=torch.zeros(count, 64),
        target_valid=torch.ones(count, 64, dtype=torch.bool),
        source_checkpoint_sha256=module.SOURCE_CHECKPOINT_SHA256,
        corpus_fingerprint_sha256="b" * 64,
        dataset_fingerprint_sha256=("c" if split == "train" else "d") * 64,
        original_logits=logits,
    )


def fake_f_occurrences(epoch):
    rows = []
    base = 0
    for replay_index in range(module.EXTRA_PER_EPOCH):
        for _ in range(3):
            rows.append(
                {
                    "dataset_index": base,
                    "domain": "carla",
                    "sampling_stream": "base",
                    "occurrence_index": base,
                }
            )
            base += 1
        rows.append(
            {
                "dataset_index": 1000 + ((replay_index + epoch) % 73),
                "domain": "carla",
                "sampling_stream": "replay",
                "occurrence_index": replay_index,
            }
        )
    while base < module.BASE_PER_EPOCH:
        rows.append(
            {
                "dataset_index": base,
                "domain": "carla",
                "sampling_stream": "base",
                "occurrence_index": base,
            }
        )
        base += 1
    return tuple(rows)


def patch_schedule_source(monkeypatch):
    monkeypatch.setattr(module, "_validate_sampling_contract", lambda *_: None)
    monkeypatch.setattr(
        module,
        "_cohort_replay_epoch_occurrences",
        lambda _plan, _config, *, epoch: fake_f_occurrences(epoch),
    )


def test_fixed_performance_first_contract_is_not_old_stopmix_contract():
    assert module.ARMS == (
        "H0_f_right_replay",
        "H1_train_cyclic_uniform_replay",
    )
    assert (module.TRAIN_COUNT, module.DEV_COUNT) == (1475, 665)
    assert (module.EPOCHS, module.STEPS, module.SAMPLES_SEEN) == (20, 8280, 33120)
    assert (module.BASE_EXPOSURES, module.EXTRA_EXPOSURES) == (29500, 3620)
    assert (module.BATCH_SIZE, module.HEAD_PARAMETER_COUNT) == (4, 3084)
    assert (
        module.LEARNING_RATE,
        module.WEIGHT_DECAY,
        module.CE_WEIGHT,
        module.EXPECTED_REGRET_WEIGHT,
        module.MAX_GRAD_NORM,
    ) == (1e-4, 1e-4, 0.1, 0.1, 5.0)
    from portable_e2e import frozen_stopmix_selector as old

    assert (old.TRAIN_COUNT, old.VAL_COUNT, old.STEPS) == (1147, 337, 1540)


def test_clone_starts_exactly_from_f_heads_and_only_3084_parameters_train():
    drive, stop = heads()
    before = (
        module.cache_core._head_digest(drive),
        module.cache_core._head_digest(stop),
        torch.get_rng_state().clone(),
    )
    cloned = module.clone_f_heads(drive, stop)
    assert torch.equal(before[2], torch.get_rng_state())
    assert sum(parameter.numel() for parameter in cloned.parameters()) == 3084
    assert all(parameter.requires_grad for parameter in cloned.parameters())
    assert module.cache_core._head_digest(drive) == before[0]
    assert module.cache_core._head_digest(stop) == before[1]
    value = torch.randn(7, 256)
    assert torch.equal(
        cloned(value), torch.cat((drive(value), stop(value)), dim=1)
    )


def test_expected_regret_matches_native_definition_and_detaches_costs():
    logits = torch.tensor([[0.0, 1.0] + [0.0] * 10], requires_grad=True)
    costs = torch.tensor([[3.0, 1.0] + [4.0] * 10], requires_grad=True)
    actual = module.expected_regret_loss(logits, costs)
    regret = costs.detach() - costs.detach().min(dim=1, keepdim=True).values
    expected = (logits.softmax(dim=1) * regret).sum(dim=1).mean()
    assert torch.equal(actual, expected)
    actual.backward()
    assert logits.grad is not None and torch.isfinite(logits.grad).all()
    assert costs.grad is None


@pytest.mark.parametrize("fault", ["shape", "dtype", "nan"])
def test_expected_regret_rejects_invalid_inputs(fault):
    logits = torch.zeros(2, 12)
    costs = torch.ones(2, 12)
    if fault == "shape":
        costs = costs[:, :6]
    elif fault == "dtype":
        costs = costs.double()
    elif fault == "nan":
        costs[0, 0] = float("nan")
    with pytest.raises((ContractError, RuntimeError)):
        module.expected_regret_loss(logits, costs)


def test_paired_schedule_preserves_every_base_position_and_is_uniform(monkeypatch):
    patch_schedule_source(monkeypatch)
    schedule = module.paired_schedule({}, object())
    h0, h1 = (schedule["arms"][arm] for arm in module.ARMS)
    assert len(h0) == len(h1) == 20
    for left, right in zip(h0, h1):
        assert len(left) == len(right) == 1656
        assert [row["sampling_stream"] for row in left] == [
            row["sampling_stream"] for row in right
        ]
        assert [row for row in left if row["sampling_stream"] == "base"] == [
            row for row in right if row["sampling_stream"] == "base"
        ]
        for rows in (left, right):
            assert all(
                len({row["dataset_index"] for row in rows[i : i + 4]}) == 4
                for i in range(0, len(rows), 4)
            )
    counts = schedule["h1_extra_identity_counts"]
    assert set(counts) == {2, 3}
    assert counts.count(3) == 670
    assert sum(counts) == 3620
    payload = [
        {
            "epoch": epoch,
            "position": position,
            "dataset_index": row["dataset_index"],
        }
        for epoch, rows in enumerate(h0)
        for position, row in enumerate(rows)
        if row["sampling_stream"] == "base"
    ]
    expected = hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    assert schedule["base_position_order_sha256"] == expected
    assert schedule["schedule_sha256"][module.ARMS[0]] != schedule[
        "schedule_sha256"
    ][module.ARMS[1]]


def test_validate_pair_is_exact_f_train_dev_and_forbids_test_or_leak(monkeypatch):
    monkeypatch.setattr(module, "TRAIN_COUNT", 8)
    monkeypatch.setattr(module, "DEV_COUNT", 6)
    original_heads = heads()
    train, dev = cache(original_heads=original_heads), cache("val", 6, original_heads)
    module.validate_pair(train, dev)
    with pytest.raises(ContractError):
        module.validate_pair(train, replace(dev, split="test"))
    with pytest.raises(ContractError, match="leakage"):
        module.validate_pair(
            train,
            replace(dev, sample_ids=(train.sample_ids[0],) + dev.sample_ids[1:]),
        )
    with pytest.raises(ContractError, match="exact F"):
        module.validate_pair(
            train, replace(dev, source_checkpoint_sha256="e" * 64)
        )


def test_short_fit_uses_only_final_dev_evaluation_and_keeps_inputs(monkeypatch):
    monkeypatch.setattr(module, "TRAIN_COUNT", 8)
    monkeypatch.setattr(module, "DEV_COUNT", 6)
    monkeypatch.setattr(module, "EPOCHS", 2)
    monkeypatch.setattr(module, "STEPS", 4)
    monkeypatch.setattr(module, "SAMPLES_SEEN", 16)
    monkeypatch.setattr(module, "BASE_EXPOSURES", 12)
    monkeypatch.setattr(module, "EXTRA_EXPOSURES", 4)
    original_heads = heads()
    train, dev = cache(original_heads=original_heads), cache("val", 6, original_heads)
    before = (train.digest(), dev.digest())
    rows = (
        {"dataset_index": 0, "sampling_stream": "base"},
        {"dataset_index": 1, "sampling_stream": "base"},
        {"dataset_index": 2, "sampling_stream": "base"},
        {"dataset_index": 3, "sampling_stream": "replay"},
        {"dataset_index": 4, "sampling_stream": "base"},
        {"dataset_index": 5, "sampling_stream": "base"},
        {"dataset_index": 6, "sampling_stream": "base"},
        {"dataset_index": 7, "sampling_stream": "replay"},
    )
    fake_schedule = {
        "arms": {arm: (rows, rows) for arm in module.ARMS},
        "schedule_sha256": {arm: arm for arm in module.ARMS},
        "base_position_order_sha256": "base",
        "h1_extra_identity_counts": [0] * 8,
    }
    monkeypatch.setattr(module, "paired_schedule", lambda *_: fake_schedule)
    monkeypatch.setattr(module.projected, "_validate_projection", lambda *_: None)

    def targets(data, _fn, _deadline=None):
        costs = torch.arange(12, device=data.fused.device, dtype=torch.float32)
        costs = costs.unsqueeze(0).expand(len(data), -1).clone()
        return costs, costs.argmin(dim=1)

    monkeypatch.setattr(module.projected, "projected_targets", targets)
    callback = []
    result = module.fit_recalibrated_selector(
        train,
        dev,
        original_drive_head=original_heads[0],
        original_stop_head=original_heads[1],
        sampling_plan={},
        train_config=object(),
        projected_costs_fn=lambda *_a, **_k: None,
        projection_proof={},
        arm=module.ARMS[0],
        callback=callback.append,
    )
    report = result["report"]
    assert report["status"] == "HEAD_ONLY_FIT_COMPLETE_NOT_PROMOTED"
    assert report["core_full_dev_metric_evaluation_count"] == 1
    assert report["dev_evaluated_only_after_final_step"] is True
    assert report["dev_used_for_training"] is False
    assert report["dev_used_for_intermediate_selection"] is False
    assert report["test_read_or_used"] is False
    assert report["new_function_head_added"] is False
    assert report["generator_parameters_trained"] is False
    assert report["head_parameter_count"] == 3084
    assert report["final_state"] == {
        "epoch": 2,
        "next_batch_index": 0,
        "global_step": 4,
        "samples_seen": 16,
    }
    assert len(callback) == len(report["history"]) == 4
    assert (train.digest(), dev.digest()) == before
    assert result["final_logits_cpu"]["val"].shape == (6, 12)
    assert all(parameter.grad is None for head in original_heads for parameter in head.parameters())


def test_callback_failure_still_proves_original_cache_and_heads_unchanged(monkeypatch):
    monkeypatch.setattr(module, "TRAIN_COUNT", 8)
    monkeypatch.setattr(module, "DEV_COUNT", 6)
    original_heads = heads()
    train, dev = cache(original_heads=original_heads), cache("val", 6, original_heads)
    monkeypatch.setattr(module, "paired_schedule", lambda *_: {
        "arms": {arm: ((
            {"dataset_index": 0, "sampling_stream": "base"},
            {"dataset_index": 1, "sampling_stream": "base"},
            {"dataset_index": 2, "sampling_stream": "base"},
            {"dataset_index": 3, "sampling_stream": "replay"},
        ),) for arm in module.ARMS},
        "schedule_sha256": {arm: arm for arm in module.ARMS},
        "base_position_order_sha256": "base",
        "h1_extra_identity_counts": [0] * 8,
    })
    monkeypatch.setattr(module.projected, "_validate_projection", lambda *_: None)
    monkeypatch.setattr(module.projected, "projected_targets", lambda data, *_: (
        torch.arange(12, dtype=torch.float32).repeat(len(data), 1),
        torch.zeros(len(data), dtype=torch.long),
    ))
    before = (
        train.digest(),
        dev.digest(),
        module.cache_core._head_digest(original_heads[0]),
        module.cache_core._head_digest(original_heads[1]),
    )

    def fail(_row):
        raise TimeoutError("owned deadline")

    with pytest.raises(TimeoutError, match="owned deadline"):
        module.fit_recalibrated_selector(
            train,
            dev,
            original_drive_head=original_heads[0],
            original_stop_head=original_heads[1],
            sampling_plan={},
            train_config=object(),
            projected_costs_fn=lambda *_a, **_k: None,
            projection_proof={},
            arm=module.ARMS[0],
            callback=fail,
        )
    assert before == (
        train.digest(),
        dev.digest(),
        module.cache_core._head_digest(original_heads[0]),
        module.cache_core._head_digest(original_heads[1]),
    )
