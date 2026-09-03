from __future__ import annotations

from dataclasses import replace
import hashlib
import inspect
import json
from pathlib import Path

from PIL import Image
import pytest

torch = pytest.importorskip("torch")

from portable_e2e import ContractError  # noqa: E402
from portable_e2e.contract import MAX_JPEG_FILE_BYTES  # noqa: E402
import portable_e2e.evaluate as evaluate_module  # noqa: E402
from portable_e2e.dataset import CALIBRATION_FEATURE_NAMES, FEATURE_NAMES  # noqa: E402
from portable_e2e.dataset import TrainingExample  # noqa: E402
from portable_e2e.evaluate import evaluate_model  # noqa: E402
from portable_e2e.losses import TrajectoryLossConfig, trajectory_loss  # noqa: E402
from portable_e2e.model import (  # noqa: E402
    ModelConfig,
    PerspectiveTrajectoryModel,
    parameter_count,
)
from portable_e2e.torch_dataset import Common10TorchDataset  # noqa: E402
import portable_e2e.train as train_module  # noqa: E402
from portable_e2e.train import (  # noqa: E402
    TrainConfig,
    _runtime_abi,
    _select_device,
    train_model,
)


HAS_SECURE_TORCH_LOAD = "weights_only" in inspect.signature(torch.load).parameters
SHARED_CORPUS_SHA256 = "a" * 64


def _small_model_config() -> ModelConfig:
    return ModelConfig(
        route_points=8,
        candidate_count=2,
        image_width=32,
        image_height=16,
        image_grid_width=2,
        image_grid_height=1,
        image_embedding=8,
        camera_fusion_width=16,
        ego_embedding=8,
        route_embedding=8,
        hidden_width=16,
        encoder_base_channels=4,
        ego_history_frames=3,
    )


def _calibration() -> tuple[tuple[float, ...], ...]:
    values = (
        0.7,
        1.2,
        0.5,
        0.5,
        1.0,
        0.0,
        0.0,
        1.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        1.5,
    )
    assert len(values) == len(CALIBRATION_FEATURE_NAMES)
    return (values,) * 6


def _example(
    root: Path, sequence_index: int, *, episode_id: str = "episode"
) -> TrainingExample:
    camera_paths = []
    camera_hashes = []
    for camera_index in range(6):
        path = root / f"{episode_id}_camera_{camera_index}.jpg"
        if not path.exists():
            Image.new("RGB", (640, 360), (20 * camera_index, 40, 80)).save(
                path, format="JPEG"
            )
        camera_paths.append(path)
        camera_hashes.append(hashlib.sha256(path.read_bytes()).hexdigest())
    positions = tuple((0.2 * (index + 1), 0.01 * sequence_index) for index in range(64))
    yaws = (0.0,) * 64
    speeds = (2.0,) * 64
    features = (1.0, float(sequence_index)) + (0.0,) * (len(FEATURE_NAMES) - 2)
    return TrainingExample(
        token=f"{episode_id}:{sequence_index:06d}",
        episode_id=episode_id,
        features=features,
        targets_xy=positions,
        target_yaw_rad=yaws,
        target_speed_mps=speeds,
        sequence_index=sequence_index,
        anchor_timestamp_ns=1_000_000_000 + sequence_index * 100_000_000,
        camera_paths=tuple(camera_paths),
        camera_sha256=tuple(camera_hashes),
        camera_calibration=_calibration(),
        route_points_base_xy_m=((0.0, 0.0), (20.0, 0.0)),
        rig_id="unit_rig",
    )


def _model_inputs(config: ModelConfig) -> dict[str, torch.Tensor]:
    return {
        "images": torch.zeros(
            1,
            config.camera_count,
            config.image_channels,
            config.image_height,
            config.image_width,
        ),
        "calibration": torch.zeros(
            1, config.camera_count, config.calibration_features
        ),
        "ego_history": torch.zeros(
            1, config.ego_history_frames, config.ego_features
        ),
        "ego_history_mask": torch.ones(
            1, config.ego_history_frames, dtype=torch.bool
        ),
        "route_xy": torch.zeros(1, config.route_points, 2),
        "route_mask": torch.ones(1, config.route_points, dtype=torch.bool),
    }


def _train_config(max_steps: int) -> TrainConfig:
    return TrainConfig(
        seed=7,
        batch_size=1,
        learning_rate=1.0e-3,
        max_steps=max_steps,
        checkpoint_interval=1,
    )


def test_model_forward_preserves_candidate_trajectory_abi() -> None:
    config = _small_model_config()
    model = PerspectiveTrajectoryModel(config)
    batch = 2
    images = torch.zeros(batch, 6, 3, 16, 32)
    calibration = torch.zeros(batch, 6, 16)
    ego_history = torch.zeros(batch, 3, len(FEATURE_NAMES))
    ego_history_mask = torch.ones(batch, 3, dtype=torch.bool)
    route_xy = torch.zeros(batch, 8, 2)
    route_mask = torch.ones(batch, 8, dtype=torch.bool)

    candidate_xy, candidate_speed, candidate_logits = model(
        images,
        calibration,
        ego_history,
        ego_history_mask,
        route_xy,
        route_mask,
    )

    assert candidate_xy.shape == (2, 2, 64, 2)
    assert candidate_speed.shape == (2, 2, 64)
    assert candidate_logits.shape == (2, 2)
    assert torch.all(candidate_speed > 0.0)
    assert parameter_count(model) > 0


def test_model_config_mapping_requires_the_complete_checkpoint_abi() -> None:
    mapping = _small_model_config().to_dict()
    assert ModelConfig.from_mapping(mapping).to_dict() == mapping

    missing = dict(mapping)
    missing.pop("image_grid_width")
    with pytest.raises(ContractError, match="missing model config fields"):
        ModelConfig.from_mapping(missing)

    extra = {**mapping, "unreviewed_field": 1}
    with pytest.raises(ContractError, match="unknown model config fields"):
        ModelConfig.from_mapping(extra)


def test_model_route_encoding_preserves_point_order() -> None:
    torch.manual_seed(20260903)
    config = _small_model_config()
    model = PerspectiveTrajectoryModel(config).eval()
    inputs = _model_inputs(config)
    route = torch.tensor(
        [[[0.0, 0.0], [1.0, 0.1], [2.0, 0.4], [3.0, 0.9],
          [4.0, 1.6], [5.0, 2.5], [6.0, 3.6], [7.0, 4.9]]]
    )

    with torch.no_grad():
        forward = model(**{**inputs, "route_xy": route})
        reverse = model(**{**inputs, "route_xy": route.flip(dims=(1,))})

    assert any(
        not torch.allclose(forward_value, reverse_value)
        for forward_value, reverse_value in zip(forward, reverse)
    )


@pytest.mark.parametrize(
    ("field", "mutate", "error", "message"),
    (
        (
            "ego_history_mask",
            lambda value: value.float(),
            ValueError,
            "torch.bool",
        ),
        (
            "route_mask",
            lambda value: value.float().masked_fill(
                torch.eye(1, value.shape[1], dtype=torch.bool), float("nan")
            ),
            ValueError,
            "torch.bool",
        ),
        (
            "route_mask",
            lambda value: torch.tensor(
                [[True, False, True, True, True, True, True, True]]
            ),
            ValueError,
            "contiguous valid prefix",
        ),
        (
            "ego_history_mask",
            lambda value: torch.tensor([[True, False, True]]),
            ValueError,
            "contiguous valid suffix",
        ),
        (
            "images",
            lambda value: value.masked_fill(
                torch.eye(value.shape[-2], value.shape[-1], dtype=torch.bool),
                float("nan"),
            ),
            FloatingPointError,
            "NaN or Inf",
        ),
        (
            "calibration",
            lambda value: value.masked_fill(value == 0, float("inf")),
            FloatingPointError,
            "NaN or Inf",
        ),
    ),
)
def test_model_rejects_invalid_masks_and_nonfinite_inputs(
    field: str,
    mutate,
    error: type[Exception],
    message: str,
) -> None:
    config = _small_model_config()
    model = PerspectiveTrajectoryModel(config)
    inputs = _model_inputs(config)
    inputs[field] = mutate(inputs[field])

    with pytest.raises(error, match=message):
        model(**inputs)


def test_masked_candidate_loss_is_finite_and_backpropagates() -> None:
    candidate_xy = torch.zeros(2, 3, 64, 2, requires_grad=True)
    candidate_speed = torch.ones(2, 3, 64, requires_grad=True)
    logits = torch.zeros(2, 3, requires_grad=True)
    target_xy = torch.ones(2, 64, 2)
    target_speed = torch.full((2, 64), 2.0)
    valid = torch.ones(2, 64, dtype=torch.bool)
    valid[0, -4:] = False

    result = trajectory_loss(
        candidate_xy,
        candidate_speed,
        logits,
        target_xy,
        target_speed,
        valid,
    )
    result["loss"].backward()

    assert torch.isfinite(result["loss"])
    assert candidate_xy.grad is not None
    assert candidate_speed.grad is not None
    assert logits.grad is not None


def test_loss_validates_prefix_mask_and_reports_yaw_and_kinematics() -> None:
    candidate_xy = torch.zeros(1, 2, 64, 2)
    candidate_speed = torch.ones(1, 2, 64)
    logits = torch.zeros(1, 2)
    target_xy = torch.zeros(1, 64, 2)
    target_speed = torch.ones(1, 64)
    target_yaw = torch.zeros(1, 64)
    valid = torch.ones(1, 64, dtype=torch.bool)
    valid[:, -4:] = False

    result = trajectory_loss(
        candidate_xy,
        candidate_speed,
        logits,
        target_xy,
        target_speed,
        valid,
        target_yaw=target_yaw,
    )

    assert torch.isfinite(result["selected_yaw_mae_rad"])
    assert torch.isfinite(result["selected_kinematic_speed_mae_mps"])
    with pytest.raises(ValueError, match="torch.bool"):
        trajectory_loss(
            candidate_xy,
            candidate_speed,
            logits,
            target_xy,
            target_speed,
            valid.float(),
        )
    nonprefix = valid.clone()
    nonprefix[:, 1] = False
    with pytest.raises(ValueError, match="contiguous valid prefix"):
        trajectory_loss(
            candidate_xy,
            candidate_speed,
            logits,
            target_xy,
            target_speed,
            nonprefix,
        )
    invalid_yaw = target_yaw.clone()
    invalid_yaw[:, 0] = float("nan")
    with pytest.raises(FloatingPointError, match="NaN or Inf"):
        trajectory_loss(
            candidate_xy,
            candidate_speed,
            logits,
            target_xy,
            target_speed,
            valid,
            target_yaw=invalid_yaw,
        )


def test_torch_dataset_decodes_images_and_builds_causal_history(tmp_path: Path) -> None:
    examples = (_example(tmp_path, 0), _example(tmp_path, 1))
    dataset = Common10TorchDataset(examples, _small_model_config())

    sample = dataset[1]

    assert sample["images"].shape == (6, 3, 16, 32)
    assert sample["calibration"].shape == (6, 16)
    assert sample["ego_history"].shape == (3, len(FEATURE_NAMES))
    assert sample["ego_history_mask"].tolist() == [False, True, True]
    assert sample["route_mask"].sum().item() == 8
    assert sample["target_valid"].all()


def test_torch_dataset_fingerprint_is_derived_from_held_examples(tmp_path: Path) -> None:
    example = _example(tmp_path, 0)
    changed = replace(example, token="different-token")

    first = Common10TorchDataset((example,), _small_model_config(), split="train")
    repeated = Common10TorchDataset((example,), _small_model_config(), split="train")
    changed_data = Common10TorchDataset((changed,), _small_model_config(), split="train")
    changed_split = Common10TorchDataset((example,), _small_model_config(), split="val")

    assert first.fingerprint_sha256 == repeated.fingerprint_sha256
    assert first.fingerprint_sha256 != changed_data.fingerprint_sha256
    assert first.fingerprint_sha256 != changed_split.fingerprint_sha256


def test_runtime_abi_records_safe_versions_and_thread_environment(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv("OMP_NUM_THREADS", "7")
    monkeypatch.setenv("MKL_NUM_THREADS", "5")
    runtime = _runtime_abi()

    assert type(runtime["torch"]) is str
    assert runtime["torch_cuda"] is None or type(runtime["torch_cuda"]) is str
    assert type(runtime["torch_num_threads"]) is int
    assert runtime["torch_num_threads"] > 0
    assert type(runtime["torch_num_interop_threads"]) is int
    assert runtime["torch_num_interop_threads"] > 0
    assert runtime["omp_num_threads"] == "7"
    assert runtime["mkl_num_threads"] == "5"


def test_evaluation_warms_up_without_counting_warmup_as_a_sample(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    model_config = _small_model_config()
    training_dataset = Common10TorchDataset(
        (_example(tmp_path, 0, episode_id="training"),),
        model_config,
        split="train",
    )
    validation_dataset = Common10TorchDataset(
        tuple(_example(tmp_path, index, episode_id="validation") for index in range(2)),
        model_config,
        split="val",
    )
    model = PerspectiveTrajectoryModel(model_config)
    payload = {
        "checkpoint_id": train_module.CHECKPOINT_ID,
        "training_split": "train",
        "training_episode_ids": ["training"],
        "dataset_fingerprint_sha256": training_dataset.fingerprint_sha256,
        "corpus_fingerprint_sha256": SHARED_CORPUS_SHA256,
        "model_config": model_config.to_dict(),
        "model_config_sha256": train_module._canonical_sha256(model_config.to_dict()),
        "loss_config": TrajectoryLossConfig().to_dict(),
        "model_state_dict": model.state_dict(),
    }
    monkeypatch.setattr(
        evaluate_module,
        "_read_checkpoint_file",
        lambda _path, _device: (payload, "b" * 64),
    )

    report = evaluate_module.evaluate_model(
        validation_dataset,
        checkpoint_path=tmp_path / "checkpoint.pt",
        output_dir=tmp_path / "evaluation",
        dataset_fingerprint_sha256=validation_dataset.fingerprint_sha256,
        corpus_fingerprint_sha256=SHARED_CORPUS_SHA256,
        evaluation_split="val",
        batch_size=2,
        render_count=0,
    )

    assert report["sample_count"] == 2
    assert report["timing"]["warmup_batches"] == 1
    assert report["timing"]["warmup_samples"] == 2
    assert report["timing"]["warmup_seconds"] > 0.0
    assert all(count == 2 for count in report["metric_counts"].values())


def test_torch_dataset_rechecks_image_hash_at_decode(tmp_path: Path) -> None:
    example = _example(tmp_path, 0)
    dataset = Common10TorchDataset((example,), _small_model_config())
    example.camera_paths[0].write_bytes(b"changed")

    with pytest.raises(ContractError, match="changed after validation"):
        dataset[0]


def test_torch_dataset_rejects_oversized_sparse_image_before_read(
    tmp_path: Path,
) -> None:
    example = _example(tmp_path, 0)
    dataset = Common10TorchDataset((example,), _small_model_config())
    with example.camera_paths[0].open("r+b") as stream:
        stream.truncate(MAX_JPEG_FILE_BYTES + 1)

    with pytest.raises(ContractError, match="size .* exceeds limit"):
        dataset[0]


def test_torch_dataset_handles_duplicate_route_points_and_rejects_degenerate_route(
    tmp_path: Path,
) -> None:
    example = _example(tmp_path, 0)
    with_duplicates = replace(
        example,
        route_points_base_xy_m=(
            (0.0, 0.0),
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 0.0),
            (4.0, 1.0),
        ),
    )
    dataset = Common10TorchDataset((with_duplicates,), _small_model_config())

    sample = dataset[0]

    assert sample["route_mask"].any()
    assert torch.isfinite(sample["route_xy"]).all()
    degenerate = replace(
        example,
        route_points_base_xy_m=((1.0, 1.0), (1.0, 1.0), (1.0, 1.0)),
    )
    with pytest.raises(ContractError, match="at least two points|nonzero distance"):
        Common10TorchDataset((degenerate,), _small_model_config())


def test_new_training_writes_checkpoint_without_loading_it(tmp_path: Path) -> None:
    examples = (_example(tmp_path, 0),)
    model_config = _small_model_config()
    dataset = Common10TorchDataset(examples, model_config, split="train")
    run_dir = tmp_path / "run"

    result = train_model(
        dataset,
        run_dir=run_dir,
        dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        corpus_fingerprint_sha256=SHARED_CORPUS_SHA256,
        model_config=model_config,
        train_config=_train_config(1),
        loss_config=TrajectoryLossConfig(),
    )

    assert result["state"]["global_step"] == 1
    assert (run_dir / "checkpoints" / "latest.pt").is_file()
    assert json.loads((run_dir / "run.json").read_text(encoding="utf-8"))[
        "status"
    ] == "TRAINING_TARGET_REACHED"
    metrics = (run_dir / "metrics.jsonl").read_text(encoding="utf-8").splitlines()
    assert [json.loads(line)["global_step"] for line in metrics] == [1]


@pytest.mark.parametrize(
    ("split", "verify_image_sha256", "message"),
    (
        ("val", True, "only accepts the train split"),
        ("train", False, "requires image SHA-256 verification"),
    ),
)
def test_training_rejects_nontrain_or_unverified_dataset(
    tmp_path: Path,
    split: str,
    verify_image_sha256: bool,
    message: str,
) -> None:
    model_config = _small_model_config()
    dataset = Common10TorchDataset(
        (_example(tmp_path, 0),),
        model_config,
        split=split,
        verify_image_sha256=verify_image_sha256,
    )

    with pytest.raises(ContractError, match=message):
        train_model(
            dataset,
            run_dir=tmp_path / f"rejected_{split}_{verify_image_sha256}",
            dataset_fingerprint_sha256=dataset.fingerprint_sha256,
            corpus_fingerprint_sha256=SHARED_CORPUS_SHA256,
            model_config=model_config,
            train_config=_train_config(1),
            loss_config=TrajectoryLossConfig(),
        )


@pytest.mark.skipif(
    not HAS_SECURE_TORCH_LOAD,
    reason="secure resume/evaluation requires torch.load(weights_only=...)",
)
def test_real_trainer_resumes_and_evaluates_disjoint_split(tmp_path: Path) -> None:
    train_examples = (
        _example(tmp_path, 0, episode_id="training"),
        _example(tmp_path, 1, episode_id="training"),
    )
    val_examples = (
        _example(tmp_path, 0, episode_id="validation"),
        _example(tmp_path, 1, episode_id="validation"),
    )
    model_config = _small_model_config()
    train_dataset = Common10TorchDataset(train_examples, model_config, split="train")
    val_dataset = Common10TorchDataset(val_examples, model_config, split="val")
    run_dir = tmp_path / "resume_run"
    assert train_dataset.fingerprint_sha256 != val_dataset.fingerprint_sha256

    first = train_model(
        train_dataset,
        run_dir=run_dir,
        dataset_fingerprint_sha256=train_dataset.fingerprint_sha256,
        corpus_fingerprint_sha256=SHARED_CORPUS_SHA256,
        model_config=model_config,
        train_config=_train_config(2),
        loss_config=TrajectoryLossConfig(),
    )
    resumed = train_model(
        train_dataset,
        run_dir=run_dir,
        dataset_fingerprint_sha256=train_dataset.fingerprint_sha256,
        corpus_fingerprint_sha256=SHARED_CORPUS_SHA256,
        model_config=model_config,
        train_config=_train_config(3),
        loss_config=TrajectoryLossConfig(),
        resume=True,
    )
    evaluation = evaluate_model(
        val_dataset,
        checkpoint_path=run_dir / "checkpoints" / "latest.pt",
        output_dir=tmp_path / "evaluation",
        dataset_fingerprint_sha256=val_dataset.fingerprint_sha256,
        corpus_fingerprint_sha256=SHARED_CORPUS_SHA256,
        evaluation_split="val",
        batch_size=1,
        render_count=1,
    )

    assert first["state"]["global_step"] == 2
    assert resumed["state"]["global_step"] == 3
    assert (run_dir / "checkpoints" / "latest.pt").is_file()
    metrics = (run_dir / "metrics.jsonl").read_text(encoding="utf-8").splitlines()
    assert [json.loads(line)["global_step"] for line in metrics] == [1, 2, 3]
    assert json.loads((run_dir / "run.json").read_text(encoding="utf-8"))["status"] == (
        "TRAINING_TARGET_REACHED"
    )
    assert evaluation["status"] == "OPEN_LOOP_EVALUATION_COMPLETE"
    assert evaluation["sample_count"] == 2
    assert evaluation["rendered_trajectory_png"] == 1
    assert len(list((tmp_path / "evaluation" / "trajectories").glob("*.png"))) == 1


@pytest.mark.skipif(
    not HAS_SECURE_TORCH_LOAD,
    reason="secure exact-resume comparison requires torch.load(weights_only=...)",
)
def test_resume_matches_uninterrupted_model_optimizer_and_rng(tmp_path: Path) -> None:
    model_config = _small_model_config()
    examples = tuple(_example(tmp_path, index) for index in range(3))
    dataset = Common10TorchDataset(examples, model_config, split="train")
    loss_config = TrajectoryLossConfig()

    train_model(
        dataset,
        run_dir=tmp_path / "uninterrupted",
        dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        corpus_fingerprint_sha256=SHARED_CORPUS_SHA256,
        model_config=model_config,
        train_config=_train_config(4),
        loss_config=loss_config,
    )
    train_model(
        dataset,
        run_dir=tmp_path / "resumed",
        dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        corpus_fingerprint_sha256=SHARED_CORPUS_SHA256,
        model_config=model_config,
        train_config=_train_config(2),
        loss_config=loss_config,
    )
    train_model(
        dataset,
        run_dir=tmp_path / "resumed",
        dataset_fingerprint_sha256=dataset.fingerprint_sha256,
        corpus_fingerprint_sha256=SHARED_CORPUS_SHA256,
        model_config=model_config,
        train_config=_train_config(4),
        loss_config=loss_config,
        resume=True,
    )

    uninterrupted = torch.load(
        tmp_path / "uninterrupted" / "checkpoints" / "latest.pt",
        map_location="cpu",
        weights_only=True,
    )
    resumed = torch.load(
        tmp_path / "resumed" / "checkpoints" / "latest.pt",
        map_location="cpu",
        weights_only=True,
    )
    assert uninterrupted["state"] == resumed["state"]
    assert torch.equal(uninterrupted["torch_rng_state"], resumed["torch_rng_state"])
    for name, value in uninterrupted["model_state_dict"].items():
        assert torch.equal(value, resumed["model_state_dict"][name]), name
    assert uninterrupted["optimizer_state_dict"].keys() == resumed[
        "optimizer_state_dict"
    ].keys()
    assert uninterrupted["optimizer_state_dict"]["param_groups"] == resumed[
        "optimizer_state_dict"
    ]["param_groups"]
    for parameter_id, state in uninterrupted["optimizer_state_dict"]["state"].items():
        for name, value in state.items():
            other = resumed["optimizer_state_dict"]["state"][parameter_id][name]
            if isinstance(value, torch.Tensor):
                assert torch.equal(value, other), (parameter_id, name)
            else:
                assert value == other


@pytest.mark.skipif(
    not HAS_SECURE_TORCH_LOAD,
    reason="secure evaluation requires torch.load(weights_only=...)",
)
def test_evaluation_rejects_episode_overlap_with_training(tmp_path: Path) -> None:
    model_config = _small_model_config()
    train_examples = (_example(tmp_path, 0, episode_id="shared_episode"),)
    train_dataset = Common10TorchDataset(train_examples, model_config, split="train")
    run_dir = tmp_path / "overlap_run"
    train_model(
        train_dataset,
        run_dir=run_dir,
        dataset_fingerprint_sha256=train_dataset.fingerprint_sha256,
        corpus_fingerprint_sha256=SHARED_CORPUS_SHA256,
        model_config=model_config,
        train_config=_train_config(1),
        loss_config=TrajectoryLossConfig(),
    )
    overlapping_val_examples = (
        replace(
            _example(tmp_path, 1, episode_id="shared_episode"),
            token="validation-overlap:000001",
        ),
    )
    val_dataset = Common10TorchDataset(
        overlapping_val_examples, model_config, split="val"
    )

    with pytest.raises(ContractError, match="episode (?:leakage|overlap)"):
        evaluate_model(
            val_dataset,
            checkpoint_path=run_dir / "checkpoints" / "latest.pt",
            output_dir=tmp_path / "overlap_evaluation",
            dataset_fingerprint_sha256=val_dataset.fingerprint_sha256,
            corpus_fingerprint_sha256=SHARED_CORPUS_SHA256,
            evaluation_split="val",
            batch_size=1,
            render_count=0,
        )


def test_gpu_device_must_be_explicit() -> None:
    with pytest.raises(ContractError, match="explicit"):
        _select_device("cuda")


@pytest.mark.parametrize(
    ("parser", "arguments"),
    (
        (train_module.parse_args, ("dataset", "--run-dir", "run")),
        (
            evaluate_module.parse_args,
            ("dataset", "--checkpoint", "checkpoint.pt", "--output-dir", "evaluation"),
        ),
    ),
)
def test_real_cli_does_not_allow_schema_validation_mode(parser, arguments) -> None:
    parsed = parser(list(arguments))
    assert not hasattr(parsed, "validation_mode")
    with pytest.raises(SystemExit):
        parser([*arguments, "--validation-mode", "schema"])


def test_evaluator_preserves_checkpoint_symlink_for_safe_loader(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    model_config = _small_model_config()
    dataset = Common10TorchDataset(
        (_example(tmp_path, 0, episode_id="validation"),),
        model_config,
        split="val",
    )
    target = tmp_path / "checkpoint-target.pt"
    target.write_bytes(b"not loaded")
    checkpoint = tmp_path / "checkpoint-link.pt"
    checkpoint.symlink_to(target)
    observed: list[Path] = []

    def reject_symlink(path: Path, device: torch.device):
        del device
        observed.append(path)
        assert path.is_absolute()
        assert path.is_symlink()
        raise ContractError("checkpoint symlink rejected")

    monkeypatch.setattr(evaluate_module, "_read_checkpoint_file", reject_symlink)
    with pytest.raises(ContractError, match="checkpoint symlink rejected"):
        evaluate_module.evaluate_model(
            dataset,
            checkpoint_path=checkpoint,
            output_dir=tmp_path / "evaluation",
            dataset_fingerprint_sha256=dataset.fingerprint_sha256,
            corpus_fingerprint_sha256="a" * 64,
            evaluation_split="val",
            batch_size=1,
            render_count=0,
        )
    assert observed == [checkpoint.absolute()]


def test_evaluate_cli_preserves_checkpoint_symlink_for_safe_loader(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    target = tmp_path / "checkpoint-target.pt"
    target.write_bytes(b"not loaded")
    checkpoint = tmp_path / "checkpoint-link.pt"
    checkpoint.symlink_to(target)
    observed: list[Path] = []

    def reject_symlink(path: Path, device: torch.device):
        del device
        observed.append(path)
        assert path.is_absolute()
        assert path.is_symlink()
        raise ContractError("checkpoint symlink rejected")

    monkeypatch.setattr(evaluate_module, "_read_checkpoint_file", reject_symlink)
    result = evaluate_module.main(
        [
            str(tmp_path / "dataset"),
            "--checkpoint",
            str(checkpoint),
            "--output-dir",
            str(tmp_path / "evaluation"),
        ]
    )
    assert result == 2
    assert observed == [checkpoint.absolute()]


def test_safe_checkpoint_loader_rejects_final_symlink(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    target = tmp_path / "checkpoint-target.pt"
    target.write_bytes(b"not a real checkpoint")
    checkpoint = tmp_path / "checkpoint-link.pt"
    checkpoint.symlink_to(target)
    load_called = False

    def fake_secure_load(stream, *, map_location=None, weights_only=True):
        del stream, map_location, weights_only
        nonlocal load_called
        load_called = True
        return {}

    monkeypatch.setattr(train_module.torch, "load", fake_secure_load)
    with pytest.raises(ContractError, match="cannot open checkpoint safely"):
        train_module._read_checkpoint_file(checkpoint, torch.device("cpu"))
    assert not load_called
