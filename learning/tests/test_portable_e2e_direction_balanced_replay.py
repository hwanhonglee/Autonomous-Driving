"""HH_260906 - Verify deterministic right-turn balancing without adding model heads."""

from __future__ import annotations

from collections import Counter
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

torch = pytest.importorskip("torch")

from portable_e2e.contract import ContractError
import portable_e2e.evaluate as evaluation
import portable_e2e.train as training
from test_portable_e2e_model import _example


def _fixture_plan(tmp_path: Path):
    template = _example(tmp_path, 0, episode_id="direction-balanced")
    examples = []
    for index in range(1475):
        if index < 254:
            yaw, tags, episode = 0.21, ("lane_follow", "turn", "left"), "left-train"
        elif index < 327:
            yaw, tags, episode = -0.21, ("lane_follow", "turn", "right"), "right-train"
        else:
            yaw, tags, episode = 0.0, ("lane_follow", "straight"), "straight-train"
        examples.append(
            replace(
                template,
                token=f"balanced:{index:06d}",
                episode_id=episode,
                scenario_tags=tags,
                sequence_index=index,
                anchor_timestamp_ns=1_000_000_000 + index * 100_000_000,
                target_yaw_rad=(yaw,) + (0.0,) * 63,
            )
        )
    dataset = SimpleNamespace(split="train", examples=tuple(examples))
    domains = {"carla": tuple(range(1475))}
    config = training.TrainConfig(
        seed=20260906,
        batch_size=4,
        max_steps=8280,
        checkpoint_interval=414,
        sampling_policy=training.DIRECTION_BALANCED_REPLAY_SAMPLING_POLICY,
    )
    return training._sampling_plan(domains, config, dataset=dataset), config, domains


def _canonical_sha256(value) -> str:
    import hashlib

    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def test_f_contract_balances_left_and_right_and_rotates_remainder(tmp_path: Path) -> None:
    plan, config, domains = _fixture_plan(tmp_path)
    contract = plan["cohort_replay"]

    assert plan["policy"] == "direction_balanced_cohort_replay"
    assert plan["order_algorithm"] == (
        "smooth_collision_avoiding_direction_balanced_replay_interleave_v1"
    )
    assert plan["epoch_sample_count"] == 1656
    assert plan["batches_per_epoch"] == 414
    assert plan["epoch_replayed_domain_sample_counts"] == {"carla": 181}
    assert contract["direction_membership_counts"] == {"left": 254, "right": 73}
    assert contract["dominant_direction"] == "left"
    assert contract["replay_direction"] == "right"
    assert contract["base_replay_copies_per_member"] == 2
    assert contract["rotating_extra_count"] == 35
    assert contract["replay_occurrence_count"] == 181

    extra_counts: Counter[int] = Counter()
    for epoch in range(20):
        occurrences = training._cohort_replay_epoch_occurrences(
            plan, config, epoch=epoch, domain_indices=domains
        )
        assert len(occurrences) == 1656
        assert sum(row["sampling_stream"] == "base" for row in occurrences) == 1475
        replay = [row for row in occurrences if row["sampling_stream"] == "replay"]
        assert len(replay) == 181
        replay_counts = Counter(row["dataset_index"] for row in replay)
        assert set(replay_counts) == set(range(254, 327))
        assert set(replay_counts.values()) == {2, 3}
        assert sum(value == 3 for value in replay_counts.values()) == 35
        for index, count in replay_counts.items():
            extra_counts[index] += count - 2
        for offset in range(0, len(occurrences), config.batch_size):
            batch = occurrences[offset : offset + config.batch_size]
            indices = [row["dataset_index"] for row in batch]
            assert len(indices) == len(set(indices))

    assert set(extra_counts) == set(range(254, 327))
    assert set(extra_counts.values()) == {9, 10}
    assert sum(extra_counts.values()) == 700


def test_f_cursor_and_schedule_bind_exact_20_epoch_exposure(tmp_path: Path) -> None:
    plan, config, _ = _fixture_plan(tmp_path)
    streams, domains = training._cohort_replay_cursor_counts(
        plan, config, epoch=20, next_batch_index=0
    )
    schedule = training._cohort_replay_execution_schedule(
        plan, config, epoch=20, next_batch_index=0
    )

    assert streams == {"base": 29_500, "replay": 3_620}
    assert domains == {"carla": 33_120}
    assert schedule[0] == 33_120

    rows = []
    for epoch in range(20):
        for position, occurrence in enumerate(
            training._cohort_replay_epoch_occurrences(plan, config, epoch=epoch)
        ):
            rows.append(
                {
                    "epoch": epoch,
                    "position": position,
                    "stream": occurrence["sampling_stream"],
                    "occurrence_index": occurrence["occurrence_index"],
                    "dataset_index": occurrence["dataset_index"],
                }
            )
    assert schedule == (len(rows), _canonical_sha256(rows))


def test_evaluator_accepts_f_checkpoint_and_rejects_stream_tampering(tmp_path: Path) -> None:
    plan, config, _ = _fixture_plan(tmp_path)
    payload = {
        "train_config": config.to_dict(),
        "sampling_plan": plan,
        "sampling_plan_sha256": training._canonical_sha256(plan),
        "sampling_metrics": {
            "samples_seen": 33_120,
            "domain_samples_seen": {"carla": 33_120},
            "sampling_stream_samples_seen": {"base": 29_500, "replay": 3_620},
        },
        "state": {
            "epoch": 20,
            "next_batch_index": 0,
            "global_step": 8280,
            "samples_seen": 33_120,
            "domain_samples_seen": {"carla": 33_120},
        },
    }
    assert evaluation._checkpoint_sampling_provenance(payload) == (
        training._canonical_sha256(plan),
        "direction_balanced_cohort_replay",
        {"carla": 33_120},
    )

    payload["sampling_metrics"]["sampling_stream_samples_seen"] = {
        "base": 29_501,
        "replay": 3_619,
    }
    with pytest.raises(ContractError, match="stream counts do not match"):
        evaluation._checkpoint_sampling_provenance(payload)


def test_f_rejects_yaw_sign_and_route_name_as_direction_substitutes(tmp_path: Path) -> None:
    template = _example(tmp_path, 0, episode_id="contains_right_but_not_tagged")
    examples = (
        replace(
            template,
            token="untagged",
            target_yaw_rad=(-0.4,) + (0.0,) * 63,
            scenario_tags=("lane_follow", "turn"),
        ),
        replace(
            template,
            token="left",
            target_yaw_rad=(0.4,) + (0.0,) * 63,
            scenario_tags=("lane_follow", "turn", "left"),
        ),
    )
    config = training.TrainConfig(
        sampling_policy=training.DIRECTION_BALANCED_REPLAY_SAMPLING_POLICY
    )
    with pytest.raises(ContractError, match="scenario tag"):
        training._sampling_plan(
            {"carla": (0, 1)},
            config,
            dataset=SimpleNamespace(split="train", examples=examples),
        )


def test_f_contract_tampering_is_rejected(tmp_path: Path) -> None:
    import copy

    plan, config, domains = _fixture_plan(tmp_path)
    corrupted = copy.deepcopy(plan)
    corrupted["cohort_replay"]["rotating_extra_count"] = 34
    with pytest.raises(ContractError, match="rotating_extra_count"):
        training._validate_direction_balanced_replay_sampling_plan(
            corrupted, config, domain_indices=domains
        )
