"""HH_260906 - Verify score-weight CLI forwarding and unchanged exact-resume safeguards."""

from __future__ import annotations

import inspect
from pathlib import Path
from types import SimpleNamespace

import pytest

torch = pytest.importorskip("torch")

from portable_e2e.contract import ContractError
from portable_e2e.losses import TrajectoryLossConfig
from portable_e2e.torch_dataset import Common10TorchDataset
import portable_e2e.train as training
from test_portable_e2e_model import _example, _small_model_config, _train_config


@pytest.mark.parametrize("option,expected", [(None, 0.1), ("0.5", 0.5), ("0", 0.0)])
def test_score_weight_cli_forwards_only_existing_loss_coefficient(monkeypatch, tmp_path: Path, option, expected):
    captured = {}
    loaded = SimpleNamespace(examples=(), split="train", validation_report={"dataset_fingerprint_sha256": "a" * 64})
    monkeypatch.setattr(training, "_load_model_config", lambda _: _small_model_config())
    monkeypatch.setattr(training, "load_training_examples", lambda *args, **kwargs: loaded)
    monkeypatch.setattr(training, "Common10TorchDataset", lambda *args, **kwargs: SimpleNamespace(fingerprint_sha256="b" * 64))
    def record_train(dataset, **kwargs):
        captured.update(kwargs)
        return {"status": "TRAINING_TARGET_REACHED"}
    monkeypatch.setattr(training, "train_model", record_train)
    argv = [str(tmp_path / "dataset"), "--run-dir", str(tmp_path / "run")]
    if option is not None:
        argv.extend(["--candidate-score-weight", option])

    assert training.main(argv) == 0
    expected_config = TrajectoryLossConfig().to_dict()
    expected_config["candidate_score_weight"] = expected
    assert captured["loss_config"].to_dict() == expected_config
    assert captured["resume"] is False
    assert not (tmp_path / "run").exists()


@pytest.mark.parametrize("value", ["-0.1", "nan", "inf", "-inf", "1e400"])
def test_invalid_score_weight_rejected_before_dataset_access(monkeypatch, tmp_path: Path, capsys, value: str):
    monkeypatch.setattr(training, "load_training_examples", lambda *args, **kwargs: pytest.fail("invalid coefficient reached dataset"))
    monkeypatch.setattr(training, "_load_model_config", lambda *args: pytest.fail("invalid coefficient reached model loading"))
    result = training.main(["dataset", "--run-dir", str(tmp_path / "run"), f"--candidate-score-weight={value}"])
    assert result == 2
    assert "candidate_score_weight must be a finite nonnegative number" in capsys.readouterr().err
    assert not (tmp_path / "run").exists()


@pytest.mark.parametrize("value", ["true", "false"])
def test_boolean_text_is_not_a_score_coefficient(value: str):
    with pytest.raises(SystemExit):
        training.parse_args(["dataset", "--run-dir", "run", "--candidate-score-weight", value])


def test_python_boolean_score_coefficient_remains_invalid():
    with pytest.raises(ContractError, match="finite nonnegative"):
        TrajectoryLossConfig(candidate_score_weight=True).validate()


@pytest.mark.parametrize("stored_weight,requested_weight", [(0.1, 0.5), (0.5, 0.1)])
def test_resume_rejects_changed_score_weight_before_loading_tensors(monkeypatch, tmp_path: Path, stored_weight, requested_weight):
    # HH_260906 - Check exact loss metadata even when local PyTorch cannot securely deserialize checkpoints.
    model_config, train_config = _small_model_config(), _train_config(1)
    device = torch.device("cpu")
    sampling_plan = training._sampling_plan({"carla": (0,)}, train_config)
    payload = {
        "checkpoint_id": training.CHECKPOINT_ID,
        "dataset_fingerprint_sha256": "a" * 64, "corpus_fingerprint_sha256": "b" * 64,
        "model_config": model_config.to_dict(), "model_config_sha256": training._canonical_sha256(model_config.to_dict()),
        "training_split": "train", "training_episode_ids": ["training"],
        "sampling_plan": sampling_plan, "sampling_plan_sha256": training._canonical_sha256(sampling_plan),
        "runtime_abi": training._runtime_abi(), "device_abi": training._device_abi(device),
        "train_config": train_config.to_dict(),
        "loss_config": TrajectoryLossConfig(candidate_score_weight=stored_weight).to_dict(),
    }
    monkeypatch.setattr(training, "_read_checkpoint_file", lambda *_: (payload, "c" * 64))
    with pytest.raises(ContractError, match="checkpoint loss config does not match"):
        training._load_checkpoint(tmp_path / "unused.pt", model=None, optimizer=None,
            dataset_fingerprint_sha256="a" * 64, corpus_fingerprint_sha256="b" * 64,
            model_config=model_config, train_config=train_config,
            loss_config=TrajectoryLossConfig(candidate_score_weight=requested_weight), device=device,
            training_split="train", training_episode_ids=["training"], dataset_size=1,
            domain_indices={"carla": (0,)}, sampling_plan=sampling_plan)


@pytest.mark.skipif("weights_only" not in inspect.signature(torch.load).parameters,
                    reason="exact resume requires secure weights-only loading")
def test_cli_cannot_resume_with_changed_score_weight(monkeypatch, tmp_path: Path, capsys):
    # HH_260906 - Exercise the real checkpoint guard so the new CLI cannot bypass immutable loss settings.
    config = _small_model_config()
    examples = (_example(tmp_path, 0),)
    dataset = Common10TorchDataset(examples, config, split="train")
    run_dir = tmp_path / "run"
    training.train_model(dataset, run_dir=run_dir, dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        corpus_fingerprint_sha256="a" * 64, model_config=config,
        train_config=_train_config(1), loss_config=TrajectoryLossConfig())
    checkpoint = run_dir / "checkpoints/latest.pt"
    original = checkpoint.read_bytes()
    loaded = SimpleNamespace(examples=examples, split="train", validation_report={"dataset_fingerprint_sha256": "a" * 64})
    monkeypatch.setattr(training, "_load_model_config", lambda _: config)
    monkeypatch.setattr(training, "load_training_examples", lambda *args, **kwargs: loaded)
    argv = ["dataset", "--run-dir", str(run_dir), "--resume", "--seed", "7", "--batch-size", "1",
            "--learning-rate", "0.001", "--checkpoint-interval", "1", "--max-steps", "2"]

    assert training.main([*argv, "--candidate-score-weight", "0.5"]) == 2
    assert "checkpoint loss config does not match" in capsys.readouterr().err
    assert checkpoint.read_bytes() == original
    assert training.main([*argv, "--candidate-score-weight", "0.1"]) == 0
