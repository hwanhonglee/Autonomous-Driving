"""HH_260906 - Verify identity-bound TRAIN-only cohort replay without a GPU."""

from __future__ import annotations

import copy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

torch = pytest.importorskip("torch")

from portable_e2e.contract import ContractError
import portable_e2e.evaluate as evaluation
import portable_e2e.train as training
from portable_e2e.losses import TrajectoryLossConfig
from portable_e2e.torch_dataset import Common10TorchDataset
from test_portable_e2e_model import _example, _small_model_config


def _canonical_sha256(value) -> str:
    return hashlib.sha256(
        json.dumps(
            value, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
    ).hexdigest()


def _fixture_plan(tmp_path: Path, *, size: int = 1475, members: int = 327):
    template = _example(tmp_path, 0, episode_id="cohort")
    examples = []
    for index in range(size):
        maximum_yaw = 0.21 if index < members else 0.2
        examples.append(
            replace(
                template,
                token=f"cohort:{index:06d}",
                sequence_index=index,
                anchor_timestamp_ns=1_000_000_000 + index * 100_000_000,
                target_yaw_rad=(maximum_yaw,) + (0.0,) * 63,
            )
        )
    dataset = SimpleNamespace(split="train", examples=tuple(examples))
    domains = {"carla": tuple(range(size))}
    config = training.TrainConfig(
        seed=20260906,
        batch_size=4,
        max_steps=9020,
        checkpoint_interval=451,
        sampling_policy=training.COHORT_REPLAY_SAMPLING_POLICY,
    )
    return training._sampling_plan(domains, config, dataset=dataset), config, domains


def test_e_contract_is_exact_deterministic_and_smooth(tmp_path: Path) -> None:
    plan, config, domains = _fixture_plan(tmp_path)
    contract = plan["cohort_replay"]
    first = training._cohort_replay_epoch_occurrences(
        plan, config, epoch=0, domain_indices=domains
    )
    repeated = training._cohort_replay_epoch_occurrences(
        plan, config, epoch=0, domain_indices=domains
    )
    next_epoch = training._cohort_replay_epoch_occurrences(
        plan, config, epoch=1, domain_indices=domains
    )

    assert plan["policy"] == "cohort_replay"
    assert plan["order_algorithm"] == (
        "smooth_proportional_cohort_replay_interleave_v1"
    )
    assert plan["dataset_domain_counts"] == {"carla": 1475}
    assert plan["epoch_replayed_domain_sample_counts"] == {"carla": 327}
    assert plan["epoch_domain_sample_counts"] == {"carla": 1802}
    assert plan["epoch_sample_count"] == 1802
    assert plan["batches_per_epoch"] == 451
    assert contract["criterion"] == {
        "split": "train",
        "field": "target_yaw_rad",
        "reduction": "max_abs_non_null",
        "comparison": "strictly_greater_than",
        "threshold_rad": 0.2,
    }
    assert contract["membership_sample_count"] == 327
    assert contract["replay_occurrence_count"] == 327
    tokens = [f"cohort:{index:06d}" for index in range(327)]
    index_tokens = [
        {"dataset_index": index, "token": token}
        for index, token in enumerate(tokens)
    ]
    assert contract["membership_sample_tokens_sha256"] == _canonical_sha256(tokens)
    assert contract["membership_index_token_sha256"] == _canonical_sha256(
        index_tokens
    )
    assert first == repeated
    assert first != next_epoch
    assert len(first) == 1802
    assert sum(row["sampling_stream"] == "base" for row in first) == 1475
    assert sum(row["sampling_stream"] == "replay" for row in first) == 327
    counts = {index: 0 for index in range(1475)}
    for row in first:
        counts[row["dataset_index"]] += 1
    assert all(counts[index] == 2 for index in range(327))
    assert all(counts[index] == 1 for index in range(327, 1475))
    for prefix_size in range(1, len(first) + 1):
        observed = sum(
            row["sampling_stream"] == "replay" for row in first[:prefix_size]
        )
        assert abs(observed - prefix_size * 327.0 / 1802.0) <= 1.0


def test_full_epoch_and_normalized_cursors_bind_same_20_epoch_schedule(
    tmp_path: Path,
) -> None:
    plan, config, _ = _fixture_plan(tmp_path)
    end_checkpoint_streams, end_checkpoint_domains = (
        training._cohort_replay_cursor_counts(
            plan, config, epoch=19, next_batch_index=451
        )
    )
    normalized_streams, normalized_domains = training._cohort_replay_cursor_counts(
        plan, config, epoch=20, next_batch_index=0
    )
    checkpoint_schedule = training._cohort_replay_execution_schedule(
        plan, config, epoch=19, next_batch_index=451
    )
    normalized_schedule = training._cohort_replay_execution_schedule(
        plan, config, epoch=20, next_batch_index=0
    )

    assert end_checkpoint_streams == normalized_streams == {
        "base": 29_500,
        "replay": 6_540,
    }
    assert end_checkpoint_domains == normalized_domains == {"carla": 36_040}
    assert checkpoint_schedule == normalized_schedule
    assert checkpoint_schedule[0] == 36_040

    # Independently materialize the experiment-scale schedule and its canonical digest.
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
    assert checkpoint_schedule == (len(rows), _canonical_sha256(rows))


@pytest.mark.parametrize(
    "mutation,error",
    (
        (
            lambda plan: plan["cohort_replay"].update(
                membership_index_token_sha256="0" * 64
            ),
            "membership_index_token_sha256",
        ),
        (
            lambda plan: plan["cohort_replay"]["replay_occurrences"][0].update(
                sample_token="foreign"
            ),
            "member does not match|replay_occurrences_sha256",
        ),
        (
            lambda plan: plan.update(epoch_sample_count=1801),
            "epoch accounting",
        ),
    ),
)
def test_plan_tampering_is_rejected(
    tmp_path: Path, mutation, error: str
) -> None:
    plan, config, domains = _fixture_plan(tmp_path, size=5, members=2)
    corrupted = copy.deepcopy(plan)
    mutation(corrupted)
    with pytest.raises(ContractError, match=error):
        training._validate_cohort_replay_sampling_plan(
            corrupted, config, domain_indices=domains
        )


def test_evaluator_revalidates_replay_exposure_and_checkpoint_cursor(
    tmp_path: Path,
) -> None:
    plan, config, _ = _fixture_plan(tmp_path)
    state = {
        "epoch": 19,
        "next_batch_index": 451,
        "global_step": 9020,
        "samples_seen": 36_040,
        "domain_samples_seen": {"carla": 36_040},
    }
    payload = {
        "train_config": config.to_dict(),
        "sampling_plan": plan,
        "sampling_plan_sha256": training._canonical_sha256(plan),
        "sampling_metrics": {
            "samples_seen": 36_040,
            "domain_samples_seen": {"carla": 36_040},
            "sampling_stream_samples_seen": {"base": 29_500, "replay": 6_540},
        },
        "state": state,
    }
    assert evaluation._checkpoint_sampling_provenance(payload) == (
        training._canonical_sha256(plan),
        "cohort_replay",
        {"carla": 36_040},
    )

    corrupted = copy.deepcopy(payload)
    corrupted["sampling_metrics"]["sampling_stream_samples_seen"] = {
        "base": 29_501,
        "replay": 6_539,
    }
    with pytest.raises(ContractError, match="stream counts do not match its cursor"):
        evaluation._checkpoint_sampling_provenance(corrupted)


def test_policy_requires_train_dataset_and_strict_threshold(tmp_path: Path) -> None:
    template = _example(tmp_path, 0, episode_id="threshold")
    values = (0.2, -0.2000001, 0.0)
    examples = tuple(
        replace(
            template,
            token=f"threshold:{index}",
            sequence_index=index,
            target_yaw_rad=(yaw,) + (0.0,) * 63,
        )
        for index, yaw in enumerate(values)
    )
    config = training.TrainConfig(
        sampling_policy=training.COHORT_REPLAY_SAMPLING_POLICY
    )
    domains = {"carla": (0, 1, 2)}
    with pytest.raises(ContractError, match="requires the TRAIN dataset"):
        training._sampling_plan(domains, config)
    plan = training._sampling_plan(
        domains,
        config,
        dataset=SimpleNamespace(split="train", examples=examples),
    )
    assert [
        row["dataset_index"]
        for row in plan["cohort_replay"]["replay_occurrences"]
    ] == [1]
    with pytest.raises(ContractError, match="only derive membership from TRAIN"):
        training._sampling_plan(
            domains,
            config,
            dataset=SimpleNamespace(split="val", examples=examples),
        )


def test_cli_default_remains_uniform_and_replay_is_explicit() -> None:
    default = training.parse_args(["dataset", "--run-dir", "run"])
    replay = training.parse_args(
        [
            "dataset",
            "--run-dir",
            "run",
            "--sampling-policy",
            "cohort_replay",
        ]
    )
    assert default.sampling_policy == training.UNIFORM_SAMPLING_POLICY
    assert replay.sampling_policy == training.COHORT_REPLAY_SAMPLING_POLICY


def test_cpu_training_records_logical_streams_and_full_epoch_schedule(
    tmp_path: Path,
) -> None:
    model_config = _small_model_config()
    examples = tuple(
        replace(
            _example(tmp_path, index, episode_id="training"),
            target_yaw_rad=((0.3 if index == 1 else 0.0),) + (0.0,) * 63,
        )
        for index in range(3)
    )
    dataset = Common10TorchDataset(examples, model_config, split="train")
    config = training.TrainConfig(
        seed=20260906,
        batch_size=2,
        learning_rate=1.0e-3,
        max_steps=2,
        checkpoint_interval=2,
        sampling_policy=training.COHORT_REPLAY_SAMPLING_POLICY,
    )
    run_dir = tmp_path / "run"
    report = training.train_model(
        dataset,
        run_dir=run_dir,
        dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        corpus_fingerprint_sha256="a" * 64,
        model_config=model_config,
        train_config=config,
        loss_config=TrajectoryLossConfig(),
        device_name="cpu",
    )
    checkpoint = torch.load(run_dir / "checkpoints/latest.pt", map_location="cpu")
    manifest = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
    history = [
        json.loads(line)
        for line in (run_dir / "metrics.jsonl").read_text(encoding="utf-8").splitlines()
    ]

    assert checkpoint["state"] == {
        "epoch": 0,
        "next_batch_index": 2,
        "global_step": 2,
        "samples_seen": 4,
        "domain_samples_seen": {"carla": 4},
    }
    assert checkpoint["sampling_metrics"]["sampling_stream_samples_seen"] == {
        "base": 3,
        "replay": 1,
    }
    assert report["state"] == manifest["state"] == {
        "epoch": 1,
        "next_batch_index": 0,
        "global_step": 2,
        "samples_seen": 4,
        "domain_samples_seen": {"carla": 4},
    }
    assert report["sampling_execution_schedule_row_count"] == 4
    assert report["sampling_execution_schedule_sha256"] == manifest[
        "sampling_execution_schedule_sha256"
    ]
    assert sum(
        row["batch_sampling_stream_counts"]["base"] for row in history
    ) == 3
    assert sum(
        row["batch_sampling_stream_counts"]["replay"] for row in history
    ) == 1
    assert history[-1]["sampling_stream_samples_seen"] == {
        "base": 3,
        "replay": 1,
    }


def test_cpu_exact_resume_matches_uninterrupted_cohort_schedule(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    model_config = _small_model_config()
    examples = tuple(
        replace(
            _example(tmp_path, index, episode_id="resume_training"),
            target_yaw_rad=((0.3 if index == 1 else 0.0),) + (0.0,) * 63,
        )
        for index in range(3)
    )
    dataset = Common10TorchDataset(examples, model_config, split="train")

    def config(max_steps: int) -> training.TrainConfig:
        return training.TrainConfig(
            seed=20260906,
            batch_size=2,
            learning_rate=1.0e-3,
            max_steps=max_steps,
            checkpoint_interval=1,
            sampling_policy=training.COHORT_REPLAY_SAMPLING_POLICY,
        )

    common = {
        "dataset_fingerprint_sha256": dataset.fingerprint_sha256,
        "corpus_fingerprint_sha256": "b" * 64,
        "model_config": model_config,
        "loss_config": TrajectoryLossConfig(),
        "device_name": "cpu",
    }
    uninterrupted_dir = tmp_path / "uninterrupted"
    resumed_dir = tmp_path / "resumed"
    uninterrupted_report = training.train_model(
        dataset,
        run_dir=uninterrupted_dir,
        train_config=config(2),
        **common,
    )
    training.train_model(
        dataset,
        run_dir=resumed_dir,
        train_config=config(1),
        **common,
    )

    # HH_260906 - The local legacy torch lacks weights_only; this test-only decoder
    # exercises trainer cursor logic without weakening production checkpoint loading.
    def test_only_read(path: Path, device: torch.device):
        payload = torch.load(path, map_location=device)
        return payload, hashlib.sha256(path.read_bytes()).hexdigest()

    monkeypatch.setattr(training, "_read_checkpoint_file", test_only_read)
    resumed_report = training.train_model(
        dataset,
        run_dir=resumed_dir,
        train_config=config(2),
        resume=True,
        **common,
    )
    uninterrupted = torch.load(
        uninterrupted_dir / "checkpoints/latest.pt", map_location="cpu"
    )
    resumed = torch.load(resumed_dir / "checkpoints/latest.pt", map_location="cpu")

    assert uninterrupted["state"] == resumed["state"]
    assert uninterrupted["sampling_metrics"] == resumed["sampling_metrics"]
    assert uninterrupted_report["sampling_execution_schedule_sha256"] == (
        resumed_report["sampling_execution_schedule_sha256"]
    )
    for name, value in uninterrupted["model_state_dict"].items():
        assert torch.equal(value, resumed["model_state_dict"][name]), name
    for parameter_id, values in uninterrupted["optimizer_state_dict"]["state"].items():
        for name, value in values.items():
            other = resumed["optimizer_state_dict"]["state"][parameter_id][name]
            assert torch.equal(value, other) if isinstance(value, torch.Tensor) else value == other
