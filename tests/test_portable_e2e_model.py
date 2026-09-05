from __future__ import annotations

import copy
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
    ConvImageEncoder,
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
    root: Path,
    sequence_index: int,
    *,
    episode_id: str = "episode",
    domain: str = "carla",
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
        domain=domain,
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


def test_model_config_rejects_nondivisible_downsampled_pooling_grid_axes() -> None:
    with pytest.raises(ContractError, match="image_grid_width must exactly divide"):
        replace(_small_model_config(), image_width=33).validate()
    with pytest.raises(ContractError, match="image_grid_height must exactly divide"):
        replace(
            _small_model_config(), image_height=17, image_grid_height=3
        ).validate()


def test_fixed_grid_pool_matches_adaptive_pool_for_divisible_default_shape() -> None:
    encoder = ConvImageEncoder(
        input_channels=3,
        base_channels=4,
        output_width=8,
        grid_height=3,
        grid_width=5,
        input_height=180,
        input_width=320,
    )
    features = torch.arange(12 * 20, dtype=torch.float32).reshape(1, 1, 12, 20)
    expected = torch.nn.functional.adaptive_avg_pool2d(features, (3, 5))

    with torch.no_grad():
        encoded = encoder.features(torch.zeros(1, 3, 180, 320))
    assert encoded.shape[-2:] == (12, 20)
    assert isinstance(encoder.pool, torch.nn.AvgPool2d)
    assert torch.equal(encoder.pool(features), expected)


def test_fixed_pool_strictly_loads_the_legacy_default_state_dict() -> None:
    torch.manual_seed(20260904)
    legacy = PerspectiveTrajectoryModel()
    legacy.image_encoder.pool = torch.nn.AdaptiveAvgPool2d((3, 5))
    replacement = PerspectiveTrajectoryModel()

    incompatible = replacement.load_state_dict(legacy.state_dict(), strict=True)

    assert incompatible.missing_keys == []
    assert incompatible.unexpected_keys == []
    assert parameter_count(replacement) == parameter_count(legacy) == 1_053_278


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


def test_yaw_loss_backpropagates_finitely_through_stationary_steps() -> None:
    # HH_260906 - Cover the exact stop case produced by the physical v1 decoder.
    candidate_xy = torch.zeros(1, 2, 64, 2, requires_grad=True)
    candidate_speed = torch.zeros(1, 2, 64, requires_grad=True)
    logits = torch.zeros(1, 2, requires_grad=True)
    target_xy = torch.zeros(1, 64, 2)
    target_speed = torch.zeros(1, 64)
    target_yaw = torch.ones(1, 64)
    valid = torch.ones(1, 64, dtype=torch.bool)

    result = trajectory_loss(
        candidate_xy,
        candidate_speed,
        logits,
        target_xy,
        target_speed,
        valid,
        target_yaw=target_yaw,
    )
    result["loss"].backward()

    assert candidate_xy.grad is not None
    assert torch.isfinite(candidate_xy.grad).all()
    assert result["selected_yaw_mae_rad"].item() == pytest.approx(0.0)


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


def test_domain_balanced_epoch_is_deterministic_finite_and_without_replacement() -> None:
    domain_indices = {
        "carla": (0, 1, 2, 3, 4),
        "real": (5, 6, 7),
    }
    config = TrainConfig(
        seed=17,
        batch_size=2,
        sampling_policy=train_module.DOMAIN_BALANCED_SAMPLING_POLICY,
        domain_ratios=(("carla", 1), ("real", 1)),
    )
    plan = train_module._sampling_plan(domain_indices, config)
    first = train_module._epoch_batches(
        8,
        batch_size=config.batch_size,
        seed=config.seed,
        epoch=4,
        domain_indices=domain_indices,
        sampling_plan=plan,
    )
    repeated = train_module._epoch_batches(
        8,
        batch_size=config.batch_size,
        seed=config.seed,
        epoch=4,
        domain_indices=domain_indices,
        sampling_plan=plan,
    )
    flattened = tuple(index for batch in first for index in batch)

    assert first == repeated
    assert len(first) == 3
    assert len(flattened) == len(set(flattened)) == 6
    assert sum(index < 5 for index in flattened) == 3
    assert sum(index >= 5 for index in flattened) == 3
    for batch in first:
        assert sum(index < 5 for index in batch) == 1
        assert sum(index >= 5 for index in batch) == 1
    for prefix_size in range(1, len(flattened) + 1):
        carla_count = sum(index < 5 for index in flattened[:prefix_size])
        assert abs(carla_count - prefix_size / 2.0) <= 0.5
    assert plan == {
        "policy": train_module.DOMAIN_BALANCED_SAMPLING_POLICY,
        "order_algorithm": train_module.BALANCED_ORDER_ALGORITHM,
        "seed_derivation": "sha256_seed_epoch_named_stream_v1",
        "known_domains": ["carla", "real"],
        "domain_ratios": {"carla": 1, "real": 1},
        "replacement": False,
        "dataset_domain_counts": {"carla": 5, "real": 3},
        "epoch_domain_sample_counts": {"carla": 3, "real": 3},
        "epoch_discarded_sample_counts": {"carla": 2, "real": 0},
        "epoch_sample_count": 6,
        "batch_size": 2,
        "batches_per_epoch": 3,
    }


def test_default_uniform_epoch_is_proportionally_interleaved_and_fingerprinted() -> None:
    size = 7
    kwargs = {"batch_size": 3, "seed": 29, "epoch": 2}
    domain_indices = {"carla": (0, 1, 2, 3), "real": (4, 5, 6)}
    uniform_config = TrainConfig(seed=29, batch_size=3)
    uniform_plan = train_module._sampling_plan(domain_indices, uniform_config)
    planned = train_module._epoch_batches(
        size,
        **kwargs,
        domain_indices=domain_indices,
        sampling_plan=uniform_plan,
    )
    balanced_plan = train_module._sampling_plan(
        domain_indices,
        TrainConfig(
            seed=29,
            batch_size=3,
            sampling_policy=train_module.DOMAIN_BALANCED_SAMPLING_POLICY,
            domain_ratios=(("carla", 1), ("real", 1)),
        ),
    )

    flattened = tuple(index for batch in planned for index in batch)
    assert len(flattened) == len(set(flattened)) == size
    for prefix_size in range(1, size + 1):
        carla_count = sum(index < 4 for index in flattened[:prefix_size])
        assert train_module._weighted_prefix_domain_counts(
            uniform_plan["epoch_domain_sample_counts"],
            seed=29,
            epoch=2,
            prefix_size=prefix_size,
        ) == {
            "carla": carla_count,
            "real": prefix_size - carla_count,
        }
    assert train_module._canonical_sha256(uniform_plan) != (
        train_module._canonical_sha256(balanced_plan)
    )
    assert uniform_plan["order_algorithm"] == train_module.UNIFORM_ORDER_ALGORITHM
    assert uniform_plan["seed_derivation"] == "sha256_seed_epoch_named_stream_v1"


def test_weighted_interleave_keeps_a_two_to_one_ratio_in_each_full_batch() -> None:
    domain_indices = {
        "carla": tuple(range(8)),
        "real": tuple(range(8, 12)),
    }
    config = TrainConfig(
        seed=11,
        batch_size=3,
        sampling_policy=train_module.DOMAIN_BALANCED_SAMPLING_POLICY,
        domain_ratios=(("carla", 2), ("real", 1)),
    )
    plan = train_module._sampling_plan(domain_indices, config)
    batches = train_module._epoch_batches(
        12,
        batch_size=config.batch_size,
        seed=config.seed,
        epoch=0,
        domain_indices=domain_indices,
        sampling_plan=plan,
    )
    flattened = tuple(index for batch in batches for index in batch)

    assert len(flattened) == len(set(flattened)) == 12
    for batch in batches:
        assert sum(index < 8 for index in batch) == 2
        assert sum(index >= 8 for index in batch) == 1
    for prefix_size in range(1, len(flattened) + 1):
        carla_count = sum(index < 8 for index in flattened[:prefix_size])
        assert abs(carla_count - prefix_size * 2.0 / 3.0) <= 1.0
        assert train_module._weighted_prefix_domain_counts(
            plan["epoch_domain_sample_counts"],
            seed=config.seed,
            epoch=0,
            prefix_size=prefix_size,
        ) == {
            "carla": carla_count,
            "real": prefix_size - carla_count,
        }


def test_domain_balanced_policy_fails_closed_on_missing_unknown_or_invalid_ratio() -> None:
    balanced = TrainConfig(
        sampling_policy=train_module.DOMAIN_BALANCED_SAMPLING_POLICY,
        domain_ratios=(("carla", 1), ("real", 1)),
    )
    with pytest.raises(ContractError, match="requires both carla and real"):
        train_module._sampling_plan({"carla": (0, 1)}, balanced)
    with pytest.raises(ContractError, match="unknown domains"):
        train_module._sampling_plan(
            {"carla": (0,), "real": (1,), "unreviewed": (2,)}, balanced
        )
    with pytest.raises(ContractError, match="cannot form one"):
        train_module._sampling_plan(
            {"carla": (0,), "real": (1,)},
            TrainConfig(
                sampling_policy=train_module.DOMAIN_BALANCED_SAMPLING_POLICY,
                domain_ratios=(("carla", 2), ("real", 1)),
            ),
        )
    with pytest.raises(ContractError, match="lowest terms"):
        TrainConfig(
            sampling_policy=train_module.DOMAIN_BALANCED_SAMPLING_POLICY,
            domain_ratios=(("carla", 2), ("real", 2)),
        ).validate()
    with pytest.raises(ContractError, match="exactly carla and real"):
        TrainConfig(
            sampling_policy=train_module.DOMAIN_BALANCED_SAMPLING_POLICY,
            domain_ratios=(("carla", 1),),
        ).validate()


def test_domain_ratio_cli_is_canonical_and_rejects_duplicates() -> None:
    assert train_module._parse_domain_ratios(("real=3", "carla=7")) == (
        ("carla", 7),
        ("real", 3),
    )
    with pytest.raises(ContractError, match="duplicate"):
        train_module._parse_domain_ratios(("carla=1", "carla=2"))
    with pytest.raises(ContractError, match="unknown"):
        train_module._parse_domain_ratios(("synthetic=1",))


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
        tuple(
            _example(
                tmp_path,
                index,
                episode_id="validation",
                domain=("carla", "real")[index],
            )
            for index in range(2)
        ),
        model_config,
        split="val",
    )
    model = PerspectiveTrajectoryModel(model_config)
    train_config = TrainConfig(batch_size=1, max_steps=1, checkpoint_interval=1)
    sampling_plan = train_module._sampling_plan(
        {"carla": (0,)},
        train_config,
    )
    payload = {
        "checkpoint_id": train_module.CHECKPOINT_ID,
        "training_split": "train",
        "training_episode_ids": ["training"],
        "dataset_fingerprint_sha256": training_dataset.fingerprint_sha256,
        "corpus_fingerprint_sha256": SHARED_CORPUS_SHA256,
        "model_config": model_config.to_dict(),
        "model_config_sha256": train_module._canonical_sha256(model_config.to_dict()),
        "train_config": train_config.to_dict(),
        "loss_config": TrajectoryLossConfig().to_dict(),
        "sampling_plan": sampling_plan,
        "sampling_plan_sha256": train_module._canonical_sha256(sampling_plan),
        "sampling_metrics": {
            "samples_seen": 1,
            "domain_samples_seen": {"carla": 1},
        },
        "state": {
            "epoch": 0,
            "next_batch_index": 1,
            "global_step": 1,
            "samples_seen": 1,
            "domain_samples_seen": {"carla": 1},
        },
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
    assert report["training_sampling_plan_sha256"] == (
        train_module._canonical_sha256(sampling_plan)
    )
    assert report["training_sampling_policy"] == (
        train_module.UNIFORM_SAMPLING_POLICY
    )
    assert report["training_domain_samples_seen"] == {"carla": 1}
    assert report["domain_sample_counts"] == {"carla": 1, "real": 1}
    for domain in ("carla", "real"):
        domain_report = report["per_domain_metrics"][domain]
        assert domain_report["sample_count"] == 1
        assert set(evaluate_module.CORE_METRIC_NAMES) <= set(domain_report["metrics"])
        assert all(
            domain_report["metric_counts"][name] == 1
            for name in evaluate_module.CORE_METRIC_NAMES
        )
    for name, aggregate in report["metrics"].items():
        weighted = sum(
            domain_report["metrics"][name]
            * domain_report["metric_counts"][name]
            for domain_report in report["per_domain_metrics"].values()
            if name in domain_report["metrics"]
        ) / report["metric_counts"][name]
        assert aggregate == pytest.approx(weighted, rel=1.0e-5, abs=1.0e-6)


def test_evaluation_revalidates_checkpoint_sampling_plan_and_cursor() -> None:
    config = TrainConfig(
        seed=13,
        batch_size=2,
        max_steps=1,
        checkpoint_interval=1,
        sampling_policy=train_module.DOMAIN_BALANCED_SAMPLING_POLICY,
        domain_ratios=(("carla", 1), ("real", 1)),
    )
    plan = train_module._sampling_plan(
        {"carla": (0, 1, 2, 3), "real": (4, 5)},
        config,
    )
    payload = {
        "train_config": config.to_dict(),
        "sampling_plan": plan,
        "sampling_plan_sha256": train_module._canonical_sha256(plan),
        "sampling_metrics": {
            "samples_seen": 2,
            "domain_samples_seen": {"carla": 1, "real": 1},
        },
        "state": {
            "epoch": 0,
            "next_batch_index": 1,
            "global_step": 1,
            "samples_seen": 2,
            "domain_samples_seen": {"carla": 1, "real": 1},
        },
    }

    assert evaluate_module._checkpoint_sampling_provenance(payload)[1:] == (
        train_module.DOMAIN_BALANCED_SAMPLING_POLICY,
        {"carla": 1, "real": 1},
    )

    bad_algorithm = copy.deepcopy(payload)
    bad_algorithm["sampling_plan"]["order_algorithm"] = "random_domains_v0"
    bad_algorithm["sampling_plan_sha256"] = train_module._canonical_sha256(
        bad_algorithm["sampling_plan"]
    )
    with pytest.raises(ContractError, match="validated training config"):
        evaluate_module._checkpoint_sampling_provenance(bad_algorithm)

    unreduced_ratio = copy.deepcopy(payload)
    unreduced_ratio["train_config"]["domain_ratios"] = (
        ("carla", 2),
        ("real", 2),
    )
    with pytest.raises(ContractError, match="lowest terms"):
        evaluate_module._checkpoint_sampling_provenance(unreduced_ratio)

    bad_cursor = copy.deepcopy(payload)
    bad_cursor["state"]["next_batch_index"] = 2
    with pytest.raises(ContractError, match="configured bounds"):
        evaluate_module._checkpoint_sampling_provenance(bad_cursor)

    bad_domain_cursor = copy.deepcopy(payload)
    bad_domain_cursor["sampling_metrics"]["domain_samples_seen"] = {
        "carla": 2,
        "real": 0,
    }
    bad_domain_cursor["state"]["domain_samples_seen"] = {"carla": 2, "real": 0}
    with pytest.raises(ContractError, match="do not match its cursor"):
        evaluate_module._checkpoint_sampling_provenance(bad_domain_cursor)

    invalid_save_step = copy.deepcopy(payload)
    invalid_save_step["train_config"]["max_steps"] = 4
    invalid_save_step["train_config"]["checkpoint_interval"] = 2
    with pytest.raises(ContractError, match="valid saved step"):
        evaluate_module._checkpoint_sampling_provenance(invalid_save_step)


@pytest.mark.parametrize(
    "failure",
    (MemoryError("memory"), OverflowError("overflow"), RecursionError("depth")),
)
def test_evaluation_wraps_sampling_plan_canonicalization_resource_errors(
    monkeypatch: pytest.MonkeyPatch, failure: BaseException
) -> None:
    def fail_canonicalization(value: object) -> str:
        del value
        raise failure

    monkeypatch.setattr(evaluate_module, "_canonical_sha256", fail_canonicalization)
    with pytest.raises(ContractError, match="sampling plan is not canonical JSON"):
        evaluate_module._checkpoint_sampling_provenance(
            {
                "sampling_plan": {"policy": train_module.UNIFORM_SAMPLING_POLICY},
                "sampling_plan_sha256": "a" * 64,
            }
        )


def test_evaluation_rejects_forged_uniform_domain_exposure_without_large_allocations() -> None:
    config = TrainConfig(
        seed=13,
        batch_size=2,
        max_steps=1,
        checkpoint_interval=1,
    )
    counts = {"carla": 10**12, "real": 10**12}
    plan = train_module._sampling_plan_from_counts(counts, config)
    first_batch_counts = train_module._weighted_prefix_domain_counts(
        plan["epoch_domain_sample_counts"],
        seed=config.seed,
        epoch=0,
        prefix_size=2,
    )
    payload = {
        "train_config": config.to_dict(),
        "sampling_plan": plan,
        "sampling_plan_sha256": train_module._canonical_sha256(plan),
        "sampling_metrics": {
            "samples_seen": 2,
            "domain_samples_seen": first_batch_counts,
        },
        "state": {
            "epoch": 0,
            "next_batch_index": 1,
            "global_step": 1,
            "samples_seen": 2,
            "domain_samples_seen": first_batch_counts,
        },
    }

    assert evaluate_module._checkpoint_sampling_provenance(payload)[2] == (
        first_batch_counts
    )
    forged = copy.deepcopy(payload)
    forged_counts = {"carla": 0, "real": 2}
    if forged_counts == first_batch_counts:
        forged_counts = {"carla": 2, "real": 0}
    forged["sampling_metrics"]["domain_samples_seen"] = forged_counts
    forged["state"]["domain_samples_seen"] = forged_counts
    with pytest.raises(ContractError, match="do not match its cursor"):
        evaluate_module._checkpoint_sampling_provenance(forged)


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
    reason="secure balanced exact-resume comparison requires weights-only loading",
)
def test_domain_balanced_resume_matches_uninterrupted_and_records_counts(
    tmp_path: Path,
) -> None:
    model_config = _small_model_config()
    examples = tuple(
        _example(
            tmp_path,
            index,
            episode_id="carla_training",
            domain="carla",
        )
        for index in range(3)
    ) + tuple(
        _example(
            tmp_path,
            index,
            episode_id="real_training",
            domain="real",
        )
        for index in range(2)
    )
    dataset = Common10TorchDataset(examples, model_config, split="train")
    loss_config = TrajectoryLossConfig()

    def balanced_config(max_steps: int) -> TrainConfig:
        return TrainConfig(
            seed=7,
            batch_size=2,
            learning_rate=1.0e-3,
            max_steps=max_steps,
            checkpoint_interval=1,
            sampling_policy=train_module.DOMAIN_BALANCED_SAMPLING_POLICY,
            domain_ratios=(("carla", 1), ("real", 1)),
        )

    common = {
        "dataset_fingerprint_sha256": dataset.fingerprint_sha256,
        "corpus_fingerprint_sha256": SHARED_CORPUS_SHA256,
        "model_config": model_config,
        "loss_config": loss_config,
    }
    train_model(
        dataset,
        run_dir=tmp_path / "balanced_uninterrupted",
        train_config=balanced_config(4),
        **common,
    )
    train_model(
        dataset,
        run_dir=tmp_path / "balanced_resumed",
        train_config=balanced_config(1),
        **common,
    )
    train_model(
        dataset,
        run_dir=tmp_path / "balanced_resumed",
        train_config=balanced_config(4),
        resume=True,
        **common,
    )

    uninterrupted = torch.load(
        tmp_path / "balanced_uninterrupted" / "checkpoints" / "latest.pt",
        map_location="cpu",
        weights_only=True,
    )
    resumed = torch.load(
        tmp_path / "balanced_resumed" / "checkpoints" / "latest.pt",
        map_location="cpu",
        weights_only=True,
    )
    assert uninterrupted["state"] == resumed["state"]
    assert resumed["state"]["domain_samples_seen"] == {"carla": 4, "real": 4}
    assert resumed["sampling_metrics"] == {
        "samples_seen": 8,
        "domain_samples_seen": {"carla": 4, "real": 4},
    }
    assert resumed["sampling_plan"]["replacement"] is False
    assert resumed["sampling_plan"]["epoch_discarded_sample_counts"] == {
        "carla": 1,
        "real": 0,
    }
    for name, value in uninterrupted["model_state_dict"].items():
        assert torch.equal(value, resumed["model_state_dict"][name]), name
    metric_rows = [
        json.loads(line)
        for line in (
            tmp_path / "balanced_resumed" / "metrics.jsonl"
        ).read_text(encoding="utf-8").splitlines()
    ]
    assert metric_rows[-1]["domain_samples_seen"] == {"carla": 4, "real": 4}


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


def test_safe_checkpoint_loader_rejects_oversized_sparse_file_before_load(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    checkpoint = tmp_path / "oversized-checkpoint.pt"
    with checkpoint.open("wb") as stream:
        stream.truncate(train_module.MAX_CHECKPOINT_FILE_BYTES + 1)
    hash_called = False
    load_called = False

    def fake_sha256(*args: object, **kwargs: object):
        del args, kwargs
        nonlocal hash_called
        hash_called = True
        raise AssertionError("oversized checkpoint must not be hashed")

    def fake_secure_load(stream, *, map_location=None, weights_only=True):
        del stream, map_location, weights_only
        nonlocal load_called
        load_called = True
        return {}

    monkeypatch.setattr(train_module.hashlib, "sha256", fake_sha256)
    monkeypatch.setattr(train_module.torch, "load", fake_secure_load)
    with pytest.raises(ContractError, match="checkpoint size .* exceeds limit"):
        train_module._read_checkpoint_file(checkpoint, torch.device("cpu"))
    assert not hash_called
    assert not load_called
