"""HH_260906 - Verify the sealed I DRIVE-mode residual recovery contract."""

from dataclasses import replace

import pytest
import torch

from portable_e2e import frozen_drive_mode_residual_recovery as module
from portable_e2e.contract import ContractError


@pytest.fixture(autouse=True)
def cpu_single_thread():
    previous = torch.get_num_threads()
    torch.set_num_threads(1)
    yield
    torch.set_num_threads(previous)


def cache(split="train", count=8):
    generator = torch.Generator().manual_seed(91 if split == "train" else 92)
    fused = torch.randn(count, 256, generator=generator) * 0.02
    xy = torch.zeros(count, 12, 64, 2)
    xy[..., 0] = torch.arange(1, 65) * 0.1
    xy[..., 1] = torch.arange(12)[None, :, None] * 0.05
    speed = torch.ones(count, 12, 64)
    logits = torch.randn(count, 12, generator=generator)
    logits[0::2, 2] = 8.0
    logits[0::2, 8] = 3.0
    logits[1::2, 2] = 3.0
    logits[1::2, 8] = 8.0
    episode_count = 4 if split == "train" else 2
    return module.CachedStopmixSelectorData(
        split=split,
        sample_ids=tuple(f"{split}:{index}" for index in range(count)),
        episode_ids=tuple(
            f"{split}-episode-{index % episode_count}" for index in range(count)
        ),
        fused=fused,
        candidate_xy=xy,
        candidate_speed=speed,
        target_xy=xy[:, 0],
        target_speed_mps=torch.ones(count, 64),
        target_yaw_rad=torch.zeros(count, 64),
        target_valid=torch.ones(count, 64, dtype=torch.bool),
        source_checkpoint_sha256=module.SOURCE_CHECKPOINT_SHA256,
        corpus_fingerprint_sha256="b" * 64,
        dataset_fingerprint_sha256=("c" if split == "train" else "d") * 64,
        original_logits=logits,
    )


def short_schedule(data):
    return (
        {
            "global_step": 1,
            "epoch": 0,
            "batch_index": 0,
            "sample_indices": [0, 1, 2, 3],
            "sample_tokens": list(data.sample_ids[:4]),
            "sampling_streams": ["base"] * 4,
        },
    )


def test_fixed_contract_and_paired_initial_head_bytes():
    assert module.ARMS == (
        "I0_prior_context_residual",
        "I1_candidate_geometry_residual",
    )
    assert (module.TRAIN_COUNT, module.DEV_COUNT) == (1475, 665)
    assert (module.EPOCHS, module.STEPS, module.SAMPLES_SEEN) == (20, 8280, 33120)
    assert (module.BASE_EXPOSURES, module.EXTRA_EXPOSURES) == (29500, 3620)
    assert module.HEAD_PARAMETER_COUNT == 115457
    before = torch.get_rng_state().clone()
    heads = [module.make_head(arm) for arm in module.ARMS]
    assert torch.equal(before, torch.get_rng_state())
    assert all(sum(parameter.numel() for parameter in head.parameters()) == 115457 for head in heads)
    assert module._digest_tensors(heads[0].state_dict().items()) == module._digest_tensors(
        heads[1].state_dict().items()
    )
    assert all(torch.count_nonzero(head[-1].weight) == 0 for head in heads)
    assert all(torch.count_nonzero(head[-1].bias) == 0 for head in heads)


@pytest.mark.parametrize("arm", module.ARMS)
def test_zero_residual_has_exact_f_logit_and_selection_parity(arm):
    data = cache()
    head = module.make_head(arm)
    actual = module.score_mode_preserving_logits(
        head,
        arm,
        data.fused,
        data.candidate_xy,
        data.candidate_speed,
        data.original_logits,
    )
    assert torch.equal(actual, data.original_logits)
    assert torch.equal(actual.argmax(1), data.original_logits.argmax(1))


@pytest.mark.parametrize("arm", module.ARMS)
def test_trained_scores_never_change_mode_and_stop_rows_remain_exact(arm):
    data = cache()
    head = module.make_head(arm)
    with torch.no_grad():
        head[-1].weight.fill_(0.25)
        head[-1].bias.fill_(1.0)
    actual = module.score_mode_preserving_logits(
        head,
        arm,
        data.fused,
        data.candidate_xy,
        data.candidate_speed,
        data.original_logits,
    )
    original_selected = data.original_logits.argmax(1)
    actual_selected = actual.argmax(1)
    stop = original_selected >= 6
    assert torch.equal(stop, actual_selected >= 6)
    assert torch.equal(actual[stop], data.original_logits[stop])
    assert torch.equal(actual_selected[stop], original_selected[stop])
    assert torch.equal(actual[:, 6:], data.original_logits[:, 6:])
    proof = module.mode_preservation_proof(data.original_logits, actual)
    assert proof["pass"] is True
    assert proof["mode_change_count"] == 0
    assert proof["original_stop_row_logit_mismatch_count"] == 0


def test_i0_ablates_geometry_while_i1_consumes_it():
    data = cache()
    heads = {arm: module.make_head(arm) for arm in module.ARMS}
    with torch.no_grad():
        for head in heads.values():
            head[-1].weight.fill_(0.1)
    i0 = module.score_drive_logits(
        heads[module.ARMS[0]],
        module.ARMS[0],
        data.fused,
        data.candidate_xy,
        data.candidate_speed,
        data.original_logits,
    )
    i1 = module.score_drive_logits(
        heads[module.ARMS[1]],
        module.ARMS[1],
        data.fused,
        data.candidate_xy,
        data.candidate_speed,
        data.original_logits,
    )
    assert not torch.equal(i0, i1)


def test_expected_drive_regret_uses_six_and_detaches_costs():
    logits = torch.tensor([[0.0, 1.0, 0.0, 0.0, 0.0, 0.0]], requires_grad=True)
    costs = torch.tensor([[3.0, 1.0, 4.0, 4.0, 4.0, 4.0]], requires_grad=True)
    result = module.expected_drive_regret_loss(logits, costs)
    detached = costs.detach()
    expected = (
        logits.softmax(1)
        * (detached - detached.min(dim=1, keepdim=True).values)
    ).sum(1).mean()
    assert torch.equal(result, expected)
    result.backward()
    assert logits.grad is not None and torch.isfinite(logits.grad).all()
    assert costs.grad is None
    with pytest.raises(ContractError):
        module.expected_drive_regret_loss(torch.zeros(1, 12), torch.zeros(1, 12))


def test_validate_pair_rejects_test_or_leak(monkeypatch):
    monkeypatch.setattr(module.h_core, "TRAIN_COUNT", 8)
    monkeypatch.setattr(module.h_core, "DEV_COUNT", 6)
    train, dev = cache(), cache("val", 6)
    module.validate_pair(train, dev)
    with pytest.raises(ContractError):
        module.validate_pair(train, replace(dev, split="test"))
    with pytest.raises(ContractError, match="leakage"):
        module.validate_pair(
            train,
            replace(dev, sample_ids=(train.sample_ids[0],) + dev.sample_ids[1:]),
        )


def test_short_fit_uses_full_train_rows_but_dev_only_after_final_step(monkeypatch):
    monkeypatch.setattr(module.h_core, "TRAIN_COUNT", 8)
    monkeypatch.setattr(module.h_core, "DEV_COUNT", 6)
    monkeypatch.setattr(module, "STEPS", 1)
    monkeypatch.setattr(module, "SAMPLES_SEEN", 4)
    monkeypatch.setattr(module, "BASE_EXPOSURES", 4)
    monkeypatch.setattr(module, "EXTRA_EXPOSURES", 0)
    monkeypatch.setattr(module, "EPOCHS", 1)
    monkeypatch.setattr(module, "validate_h1_schedule", lambda rows, _train: tuple(rows))
    monkeypatch.setattr(module.projected, "_validate_projection", lambda *_: None)
    target_calls = []

    def targets(data, _function, _deadline=None):
        target_calls.append(data.split)
        costs = torch.arange(6, device=data.fused.device, dtype=torch.float32)
        costs = costs.unsqueeze(0).repeat(len(data), 1)
        return costs, costs.argmin(1)

    monkeypatch.setattr(module, "_drive_targets", targets)
    train, dev = cache(), cache("val", 6)
    before = train.digest(), dev.digest()
    callback = []
    result = module.fit_drive_mode_residual(
        train,
        dev,
        schedule=short_schedule(train),
        projected_costs_fn=lambda *_args, **_kwargs: None,
        projection_proof={},
        arm=module.ARMS[1],
        callback=callback.append,
    )
    report = result["report"]
    assert target_calls == ["train", "val", "train"]
    assert report["status"] == "DRIVE_MODE_RESIDUAL_FIT_COMPLETE_NOT_PROMOTED"
    assert report["core_full_dev_metric_evaluation_count"] == 1
    assert report["dev_evaluated_only_after_final_step"] is True
    assert report["dev_used_for_training"] is False
    assert report["test_read_or_used"] is False
    assert report["selector_replacement_diagnostic_only"] is True
    assert report["new_autonomous_function_head_added"] is False
    assert report["head_parameter_count"] == 115457
    assert len(callback) == len(report["history"]) == 1
    assert all(value["pass"] for value in report["final_mode_preservation"].values())
    assert result["final_logits_cpu"]["val"].shape == (6, 12)
    assert (train.digest(), dev.digest()) == before


def test_invalid_score_shapes_and_nonfinite_values_are_rejected():
    data = cache()
    head = module.make_head(module.ARMS[0])
    with pytest.raises(ContractError):
        module.score_drive_logits(
            head,
            module.ARMS[0],
            data.fused,
            data.candidate_xy[:, :6],
            data.candidate_speed,
            data.original_logits,
        )
    bad = data.original_logits.clone()
    bad[0, 0] = float("nan")
    with pytest.raises(ContractError):
        module.score_drive_logits(
            head,
            module.ARMS[0],
            data.fused,
            data.candidate_xy,
            data.candidate_speed,
            bad,
        )
