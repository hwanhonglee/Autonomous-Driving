from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from portable_e2e import ContractError
from portable_e2e import control_flow_smoke as smoke
from portable_e2e.dataset import FEATURE_NAMES
from portable_e2e.dataset import TrainingExample
from portable_e2e.dataset import batch_indices
from portable_e2e.dataset import fingerprint_examples


def _single_target_example(token: str = "one") -> TrainingExample:
    targets: list[tuple[float, float] | None] = [(2.0, 4.0)] + [None] * 63
    return TrainingExample(
        token=token,
        episode_id="synthetic",
        features=(1.0,) + (0.0,) * (len(FEATURE_NAMES) - 1),
        targets_xy=tuple(targets),
    )


def test_deterministic_batching_and_partial_batch_semantics() -> None:
    first = batch_indices(10, batch_size=4, seed=7, epoch=2)
    second = batch_indices(10, batch_size=4, seed=7, epoch=2)
    next_epoch = batch_indices(10, batch_size=4, seed=7, epoch=3)

    assert first == second
    assert first != next_epoch
    assert sorted(index for batch in first for index in batch) == list(range(10))
    assert tuple(len(batch) for batch in first) == (4, 4, 2)
    assert tuple(len(batch) for batch in batch_indices(
        10, batch_size=4, seed=7, epoch=2, drop_last=True
    )) == (4, 4)


def test_masked_mse_skips_invalid_points_instead_of_multiplying_a_mask() -> None:
    example = _single_target_example()

    loss = smoke.masked_mse(smoke.zero_weights(), [example])

    assert loss == pytest.approx(10.0)
    invalid = TrainingExample(
        token="invalid",
        episode_id="synthetic",
        features=example.features,
        targets_xy=(None,) * 64,
    )
    with pytest.raises(ContractError, match="no valid target"):
        smoke.masked_mse(smoke.zero_weights(), [invalid])


def test_one_gradient_step_has_expected_bias_update() -> None:
    example = _single_target_example()
    config = smoke.SmokeConfig(seed=1, batch_size=1, learning_rate=0.1, max_steps=1)

    state = smoke.train_until([example], config)

    assert state.weights[0][0][0] == pytest.approx(0.2)
    assert state.weights[0][1][0] == pytest.approx(0.4)
    assert state.weights[1][0][0] == 0.0
    assert state.global_step == 1


def test_synthetic_control_flow_reduces_loss_without_ml_dependencies() -> None:
    config = smoke.SmokeConfig(max_steps=40)

    report = smoke.run_control_flow_smoke(config)

    assert report["status"] == "PIPELINE_CONTRACT_PASS"
    assert report["final_dataset_loss"] < report["initial_dataset_loss"]
    assert report["production_model_created"] is False
    assert report["images_decoded"] is False
    assert report["gpu_used"] is False
    assert report["checkpoint"] is None
    assert "torch" not in smoke.__dict__
    assert "numpy" not in smoke.__dict__


def test_atomic_checkpoint_resume_matches_uninterrupted_training(tmp_path: Path) -> None:
    examples = smoke.synthetic_examples()
    fingerprint = fingerprint_examples(examples)
    config = smoke.SmokeConfig(max_steps=20)
    uninterrupted = smoke.train_until(examples, config)
    partial = smoke.train_until(examples, config, stop_at_step=7)
    checkpoint = tmp_path / "resume.dummy-smoke.json"
    smoke.save_checkpoint(
        checkpoint,
        dataset_fingerprint_sha256=fingerprint,
        config=config,
        state=partial,
    )

    loaded = smoke.load_checkpoint(
        checkpoint,
        dataset_fingerprint_sha256=fingerprint,
        config=config,
    )
    resumed = smoke.train_until(examples, config, state=loaded)

    assert smoke.state_digest(resumed) == smoke.state_digest(uninterrupted)
    assert resumed.token_trace == uninterrupted.token_trace
    assert resumed.global_step == 20
    assert not list(tmp_path.glob(f".{checkpoint.name}.tmp.*"))


def test_checkpoint_rejects_changed_config_corruption_and_model_paths(tmp_path: Path) -> None:
    examples = smoke.synthetic_examples()
    fingerprint = fingerprint_examples(examples)
    config = smoke.SmokeConfig(max_steps=2)
    state = smoke.train_until(examples, config)
    checkpoint = tmp_path / "valid.dummy-smoke.json"
    smoke.save_checkpoint(
        checkpoint,
        dataset_fingerprint_sha256=fingerprint,
        config=config,
        state=state,
    )

    with pytest.raises(ContractError, match="immutable resume config"):
        smoke.load_checkpoint(
            checkpoint,
            dataset_fingerprint_sha256=fingerprint,
            config=smoke.SmokeConfig(max_steps=3, learning_rate=0.04),
        )
    checkpoint.write_text("{", encoding="utf-8")
    with pytest.raises(ContractError, match="cannot load"):
        smoke.load_checkpoint(
            checkpoint,
            dataset_fingerprint_sha256=fingerprint,
            config=config,
        )
    with pytest.raises(ContractError, match="model/runtime paths"):
        smoke.save_checkpoint(
            tmp_path / "data" / "ml_models" / "bad.dummy-smoke.json",
            dataset_fingerprint_sha256=fingerprint,
            config=config,
            state=state,
        )


def test_cli_style_partial_checkpoint_can_resume_to_a_larger_step_target(
    tmp_path: Path,
) -> None:
    checkpoint = tmp_path / "control_flow" / "resume.dummy-smoke.json"
    partial_config = smoke.SmokeConfig(max_steps=7)
    partial = smoke.run_control_flow_smoke(
        partial_config,
        checkpoint=checkpoint,
        stop_at_step=4,
    )

    resumed = smoke.run_control_flow_smoke(
        smoke.SmokeConfig(max_steps=20),
        checkpoint=checkpoint,
        resume=True,
    )
    uninterrupted = smoke.run_control_flow_smoke(smoke.SmokeConfig(max_steps=20))

    assert partial["status"] == "PIPELINE_CHECKPOINT_SAVED"
    assert partial["global_step"] == 4
    assert resumed["status"] == "PIPELINE_CONTRACT_PASS"
    assert resumed["state_sha256"] == uninterrupted["state_sha256"]


def test_existing_checkpoint_requires_resume_or_explicit_dummy_overwrite(
    tmp_path: Path,
) -> None:
    checkpoint = tmp_path / "control_flow" / "existing.dummy-smoke.json"
    config = smoke.SmokeConfig(max_steps=2)
    smoke.run_control_flow_smoke(config, checkpoint=checkpoint)

    with pytest.raises(ContractError, match="already exists"):
        smoke.run_control_flow_smoke(config, checkpoint=checkpoint)
    replaced = smoke.run_control_flow_smoke(
        config,
        checkpoint=checkpoint,
        overwrite_dummy_checkpoint=True,
    )
    assert replaced["status"] == "PIPELINE_CONTRACT_PASS"


def test_resume_rejects_inconsistent_cursor_and_malformed_loss(tmp_path: Path) -> None:
    examples = smoke.synthetic_examples()
    fingerprint = fingerprint_examples(examples)
    config = smoke.SmokeConfig(max_steps=10)
    checkpoint = tmp_path / "cursor.dummy-smoke.json"
    state = smoke.train_until(examples, config, stop_at_step=4)
    smoke.save_checkpoint(
        checkpoint,
        dataset_fingerprint_sha256=fingerprint,
        config=config,
        state=state,
    )
    payload = json.loads(checkpoint.read_text(encoding="utf-8"))
    payload["state"]["next_batch_index"] = 0
    checkpoint.write_text(json.dumps(payload), encoding="utf-8")
    loaded = smoke.load_checkpoint(
        checkpoint,
        dataset_fingerprint_sha256=fingerprint,
        config=config,
    )
    with pytest.raises(ContractError, match="cursor does not match"):
        smoke.train_until(examples, config, state=loaded)

    payload["state"]["first_batch_loss"] = "not-a-number"
    checkpoint.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ContractError, match="first_batch_loss"):
        smoke.load_checkpoint(
            checkpoint,
            dataset_fingerprint_sha256=fingerprint,
            config=config,
        )


@pytest.mark.parametrize(
    "config",
    [
        smoke.SmokeConfig(max_steps=0),
        smoke.SmokeConfig(max_steps=101),
        smoke.SmokeConfig(batch_size=0),
        smoke.SmokeConfig(learning_rate=math.nan),
        smoke.SmokeConfig(learning_rate=True),
        smoke.SmokeConfig(learning_rate="invalid"),  # type: ignore[arg-type]
    ],
)
def test_smoke_config_hard_limits(config: smoke.SmokeConfig) -> None:
    with pytest.raises(ContractError):
        config.validate()


def test_checkpoint_is_json_and_explicitly_nonproduction(tmp_path: Path) -> None:
    checkpoint = tmp_path / "audit.dummy-smoke.json"
    report = smoke.run_control_flow_smoke(smoke.SmokeConfig(max_steps=2), checkpoint=checkpoint)
    payload = json.loads(checkpoint.read_text(encoding="utf-8"))

    assert payload["kind"] == smoke.CHECKPOINT_KIND
    assert payload["warning"] == smoke.BANNER
    assert report["production_model_created"] is False
    assert not list(tmp_path.glob("*.pt"))
    assert not list(tmp_path.glob("*.onnx"))


def test_checkpoint_rejects_corrupt_stored_config_and_fingerprint(tmp_path: Path) -> None:
    examples = smoke.synthetic_examples()
    fingerprint = fingerprint_examples(examples)
    config = smoke.SmokeConfig(max_steps=2)
    state = smoke.train_until(examples, config)
    checkpoint = tmp_path / "audit.dummy-smoke.json"
    smoke.save_checkpoint(
        checkpoint,
        dataset_fingerprint_sha256=fingerprint,
        config=config,
        state=state,
    )
    payload = json.loads(checkpoint.read_text(encoding="utf-8"))
    payload["config_fingerprint_sha256"] = "0" * 64
    checkpoint.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ContractError, match="config fingerprint is corrupt"):
        smoke.load_checkpoint(
            checkpoint,
            dataset_fingerprint_sha256=fingerprint,
            config=config,
        )

    smoke.save_checkpoint(
        checkpoint,
        dataset_fingerprint_sha256=fingerprint,
        config=config,
        state=state,
    )
    payload = json.loads(checkpoint.read_text(encoding="utf-8"))
    payload["config"]["max_steps"] = "corrupt"
    checkpoint.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ContractError, match="max_steps"):
        smoke.load_checkpoint(
            checkpoint,
            dataset_fingerprint_sha256=fingerprint,
            config=config,
        )


def test_dummy_overwrite_flag_requires_a_checkpoint_path() -> None:
    with pytest.raises(ContractError, match="requires --checkpoint"):
        smoke.run_control_flow_smoke(
            smoke.SmokeConfig(max_steps=1),
            overwrite_dummy_checkpoint=True,
        )
