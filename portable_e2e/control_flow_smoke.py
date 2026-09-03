"""Deterministic, standard-library-only training control-flow smoke.

This module validates batching, masked trajectory loss, optimizer state, and
checkpoint resume.  It never decodes images, imports a neural-network library,
or creates a deployable model artifact.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
from typing import Any, Callable, Mapping, Sequence

from .contract import ContractError
from .contract import _loads_json
from .dataset import FEATURE_NAMES
from .dataset import TrainingExample
from .dataset import batch_indices
from .dataset import fingerprint_examples
from .dataset import iter_examples


BANNER = "DUMMY CONTROL-FLOW SMOKE — NO IMAGES, NO NEURAL NETWORK, NOT DEPLOYABLE"
CHECKPOINT_KIND = "DUMMY_CPU_CONTROL_FLOW_ONLY"
CHECKPOINT_SCHEMA_ID = "autoware-e2e.portable-control-flow-checkpoint.v1"
MAX_STEPS = 100
POINT_COUNT = 64
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")


@dataclass(frozen=True)
class SmokeConfig:
    seed: int = 20260903
    batch_size: int = 6
    learning_rate: float = 0.03
    max_steps: int = 40
    drop_last: bool = False

    def validate(self) -> None:
        if isinstance(self.seed, bool) or not isinstance(self.seed, int) or self.seed < 0:
            raise ContractError("smoke seed must be a nonnegative integer")
        if (
            isinstance(self.batch_size, bool)
            or not isinstance(self.batch_size, int)
            or not 1 <= self.batch_size <= 64
        ):
            raise ContractError("smoke batch_size must be in [1, 64]")
        if (
            isinstance(self.learning_rate, bool)
            or not isinstance(self.learning_rate, (int, float))
            or not math.isfinite(float(self.learning_rate))
            or not 0.0 < float(self.learning_rate) <= 1.0
        ):
            raise ContractError("smoke learning_rate must be finite and in (0, 1]")
        if (
            isinstance(self.max_steps, bool)
            or not isinstance(self.max_steps, int)
            or not 1 <= self.max_steps <= MAX_STEPS
        ):
            raise ContractError(f"smoke max_steps must be in [1, {MAX_STEPS}]")
        if not isinstance(self.drop_last, bool):
            raise ContractError("smoke drop_last must be boolean")

    def to_dict(self) -> dict[str, Any]:
        return {
            "seed": self.seed,
            "batch_size": self.batch_size,
            "learning_rate": self.learning_rate,
            "max_steps": self.max_steps,
            "drop_last": self.drop_last,
        }

    @property
    def fingerprint_sha256(self) -> str:
        encoded = json.dumps(
            self.to_dict(), sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()

    @property
    def resume_fingerprint_sha256(self) -> str:
        immutable = {
            "seed": self.seed,
            "batch_size": self.batch_size,
            "learning_rate": self.learning_rate,
            "drop_last": self.drop_last,
        }
        encoded = json.dumps(
            immutable, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()


@dataclass
class SmokeState:
    weights: list[list[list[float]]]
    epoch: int = 0
    next_batch_index: int = 0
    global_step: int = 0
    samples_seen: int = 0
    first_batch_loss: float | None = None
    last_batch_loss: float | None = None
    token_trace: list[str] | None = None

    def __post_init__(self) -> None:
        if self.token_trace is None:
            self.token_trace = []


def zero_weights(
    point_count: int = POINT_COUNT, feature_count: int = len(FEATURE_NAMES)
) -> list[list[list[float]]]:
    return [
        [[0.0 for _ in range(feature_count)] for _ in range(2)]
        for _ in range(point_count)
    ]


def _validate_examples(examples: Sequence[TrainingExample]) -> None:
    if not examples:
        raise ContractError("control-flow smoke needs at least one example")
    tokens: set[str] = set()
    for index, example in enumerate(examples):
        context = f"examples[{index}]"
        if example.token in tokens:
            raise ContractError(f"{context} repeats token {example.token!r}")
        tokens.add(example.token)
        if len(example.features) != len(FEATURE_NAMES):
            raise ContractError(f"{context} has the wrong feature count")
        if any(not math.isfinite(value) for value in example.features):
            raise ContractError(f"{context} contains a non-finite feature")
        if len(example.targets_xy) != POINT_COUNT:
            raise ContractError(f"{context} must contain {POINT_COUNT} future points")
        if example.valid_point_count == 0:
            raise ContractError(f"{context} has no valid future target")
        for target in example.targets_xy:
            if target is not None and any(not math.isfinite(value) for value in target):
                raise ContractError(f"{context} contains a non-finite valid target")


def predict(
    weights: Sequence[Sequence[Sequence[float]]], example: TrainingExample
) -> tuple[tuple[float, float], ...]:
    return tuple(
        tuple(
            sum(weight * feature for weight, feature in zip(axis_weights, example.features))
            for axis_weights in point_weights
        )
        for point_weights in weights
    )  # type: ignore[return-value]


def masked_mse(
    weights: Sequence[Sequence[Sequence[float]]],
    examples: Sequence[TrainingExample],
) -> float:
    squared_error = 0.0
    valid_points = 0
    for example in examples:
        prediction = predict(weights, example)
        for predicted, target in zip(prediction, example.targets_xy):
            if target is None:
                continue
            squared_error += (predicted[0] - target[0]) ** 2
            squared_error += (predicted[1] - target[1]) ** 2
            valid_points += 1
    if valid_points == 0:
        raise ContractError("masked trajectory loss received no valid target")
    return squared_error / (2.0 * valid_points)


def loss_and_gradient(
    weights: Sequence[Sequence[Sequence[float]]],
    examples: Sequence[TrainingExample],
) -> tuple[float, list[list[list[float]]]]:
    gradient = zero_weights(len(weights), len(weights[0][0]))
    squared_error = 0.0
    valid_points = 0
    for example in examples:
        prediction = predict(weights, example)
        for point_index, (predicted, target) in enumerate(
            zip(prediction, example.targets_xy)
        ):
            if target is None:
                continue
            valid_points += 1
            for axis in range(2):
                error = predicted[axis] - target[axis]
                squared_error += error * error
                for feature_index, feature in enumerate(example.features):
                    gradient[point_index][axis][feature_index] += error * feature
    if valid_points == 0:
        raise ContractError("masked trajectory loss received no valid target")
    scale = 1.0 / valid_points
    for point in gradient:
        for axis in point:
            for feature_index in range(len(axis)):
                axis[feature_index] *= scale
    return squared_error / (2.0 * valid_points), gradient


def _apply_gradient(
    weights: list[list[list[float]]],
    gradient: Sequence[Sequence[Sequence[float]]],
    learning_rate: float,
) -> None:
    for point_index, point in enumerate(weights):
        for axis_index, axis in enumerate(point):
            for feature_index in range(len(axis)):
                axis[feature_index] -= (
                    learning_rate * gradient[point_index][axis_index][feature_index]
                )


def train_until(
    examples: Sequence[TrainingExample],
    config: SmokeConfig,
    *,
    state: SmokeState | None = None,
    stop_at_step: int | None = None,
    on_step: Callable[[SmokeState], None] | None = None,
) -> SmokeState:
    config.validate()
    _validate_examples(examples)
    target_step = config.max_steps if stop_at_step is None else stop_at_step
    if not 0 <= target_step <= config.max_steps:
        raise ContractError("stop_at_step must be between current zero and max_steps")
    current = state or SmokeState(zero_weights())
    _validate_state(current)
    _validate_resume_cursor(examples, config, current)
    if current.global_step > target_step:
        raise ContractError("checkpoint is already beyond the requested stop_at_step")

    while current.global_step < target_step:
        batches = batch_indices(
            len(examples),
            batch_size=config.batch_size,
            seed=config.seed,
            epoch=current.epoch,
            drop_last=config.drop_last,
        )
        if current.next_batch_index > len(batches):
            raise ContractError("checkpoint next_batch_index exceeds the epoch batch count")
        if current.next_batch_index == len(batches):
            current.epoch += 1
            current.next_batch_index = 0
            continue
        indices = batches[current.next_batch_index]
        batch = iter_examples(examples, indices)
        loss, gradient = loss_and_gradient(current.weights, batch)
        _apply_gradient(current.weights, gradient, config.learning_rate)
        if current.first_batch_loss is None:
            current.first_batch_loss = loss
        current.last_batch_loss = loss
        current.token_trace.append(",".join(example.token for example in batch))
        current.samples_seen += len(batch)
        current.global_step += 1
        current.next_batch_index += 1
        if on_step is not None:
            on_step(current)
    return current


def _validate_resume_cursor(
    examples: Sequence[TrainingExample], config: SmokeConfig, state: SmokeState
) -> None:
    batches = batch_indices(
        len(examples),
        batch_size=config.batch_size,
        seed=config.seed,
        epoch=state.epoch,
        drop_last=config.drop_last,
    )
    if state.next_batch_index > len(batches):
        raise ContractError("checkpoint next_batch_index exceeds the epoch batch count")
    expected_step = state.epoch * len(batches) + state.next_batch_index
    if state.global_step != expected_step:
        raise ContractError("checkpoint epoch/batch cursor does not match global_step")
    samples_per_epoch = sum(len(batch) for batch in batches)
    expected_samples = state.epoch * samples_per_epoch + sum(
        len(batch) for batch in batches[: state.next_batch_index]
    )
    if state.samples_seen != expected_samples:
        raise ContractError("checkpoint samples_seen does not match its batch cursor")
    expected_trace: list[str] = []
    for epoch in range(state.epoch + 1):
        epoch_batches = batch_indices(
            len(examples),
            batch_size=config.batch_size,
            seed=config.seed,
            epoch=epoch,
            drop_last=config.drop_last,
        )
        limit = len(epoch_batches) if epoch < state.epoch else state.next_batch_index
        for indices in epoch_batches[:limit]:
            expected_trace.append(",".join(examples[index].token for index in indices))
    if state.token_trace != expected_trace:
        raise ContractError("checkpoint token_trace does not match deterministic batching")


def _validate_state(state: SmokeState) -> None:
    for name, value in (
        ("epoch", state.epoch),
        ("next_batch_index", state.next_batch_index),
        ("global_step", state.global_step),
        ("samples_seen", state.samples_seen),
    ):
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ContractError(f"checkpoint {name} must be a nonnegative integer")
    if not isinstance(state.weights, list) or len(state.weights) != POINT_COUNT:
        raise ContractError(f"checkpoint weights must contain {POINT_COUNT} points")
    for point in state.weights:
        if not isinstance(point, list) or len(point) != 2:
            raise ContractError("checkpoint weight shape is invalid")
        for axis in point:
            if not isinstance(axis, list) or len(axis) != len(FEATURE_NAMES):
                raise ContractError("checkpoint weight shape is invalid")
            for value in axis:
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                ):
                    raise ContractError("checkpoint contains invalid or non-finite weights")
    for name, value in (
        ("first_batch_loss", state.first_batch_loss),
        ("last_batch_loss", state.last_batch_loss),
    ):
        if value is not None and (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            raise ContractError(f"checkpoint {name} must be finite or null")
    if not isinstance(state.token_trace, list) or any(
        not isinstance(value, str) for value in state.token_trace
    ):
        raise ContractError("checkpoint token_trace must contain strings")
    if len(state.token_trace) != state.global_step:
        raise ContractError("checkpoint token_trace length must equal global_step")
    if state.global_step > 0 and (
        state.first_batch_loss is None or state.last_batch_loss is None
    ):
        raise ContractError("checkpoint with completed steps must contain batch losses")


def _checkpoint_path(path: Path | str) -> Path:
    resolved = Path(path).expanduser().resolve()
    if not resolved.name.endswith(".dummy-smoke.json"):
        raise ContractError("control-flow checkpoint must end with .dummy-smoke.json")
    lowered = tuple(part.lower() for part in resolved.parts)
    joined = "/".join(lowered)
    forbidden_parts = {"models", "ml_models", "exports", "tensorrt", "engine", "engines"}
    if any(part in forbidden_parts for part in lowered) or "/runtime/engine/" in f"/{joined}/":
        raise ContractError("control-flow checkpoints cannot be written into model/runtime paths")
    return resolved


def _state_payload(state: SmokeState) -> dict[str, Any]:
    _validate_state(state)
    return {
        "weights": state.weights,
        "epoch": state.epoch,
        "next_batch_index": state.next_batch_index,
        "global_step": state.global_step,
        "samples_seen": state.samples_seen,
        "first_batch_loss": state.first_batch_loss,
        "last_batch_loss": state.last_batch_loss,
        "token_trace": state.token_trace,
    }


def save_checkpoint(
    path: Path | str,
    *,
    dataset_fingerprint_sha256: str,
    config: SmokeConfig,
    state: SmokeState,
) -> Path:
    output = _checkpoint_path(path)
    config.validate()
    if SHA256_PATTERN.fullmatch(dataset_fingerprint_sha256) is None:
        raise ContractError("dataset fingerprint must be SHA256")
    payload = {
        "schema_id": CHECKPOINT_SCHEMA_ID,
        "schema_version": 1,
        "kind": CHECKPOINT_KIND,
        "warning": BANNER,
        "dataset_fingerprint_sha256": dataset_fingerprint_sha256,
        "config": config.to_dict(),
        "config_fingerprint_sha256": config.fingerprint_sha256,
        "resume_fingerprint_sha256": config.resume_fingerprint_sha256,
        "state": _state_payload(state),
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.tmp.{os.getpid()}")
    encoded = (json.dumps(payload, indent=2, allow_nan=False) + "\n").encode("utf-8")
    try:
        with temporary.open("xb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, output)
        directory_fd = os.open(output.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except BaseException:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise
    return output


def _exact_mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ContractError(f"{context} must be an object")
    return value


def load_checkpoint(
    path: Path | str,
    *,
    dataset_fingerprint_sha256: str,
    config: SmokeConfig,
) -> SmokeState:
    checkpoint = _checkpoint_path(path)
    config.validate()
    try:
        checkpoint_text = checkpoint.read_text(encoding="utf-8")
    except OSError as error:
        raise ContractError(f"cannot load control-flow checkpoint: {error}") from error
    try:
        document = _loads_json(checkpoint_text, "checkpoint")
    except ContractError as error:
        raise ContractError(f"cannot load control-flow checkpoint: {error}") from error
    if document.get("schema_id") != CHECKPOINT_SCHEMA_ID or document.get("schema_version") != 1:
        raise ContractError("checkpoint schema is incompatible")
    if document.get("kind") != CHECKPOINT_KIND or document.get("warning") != BANNER:
        raise ContractError("checkpoint is not a control-flow-only artifact")
    if document.get("dataset_fingerprint_sha256") != dataset_fingerprint_sha256:
        raise ContractError("checkpoint dataset fingerprint does not match")
    stored_config = _exact_mapping(document.get("config"), "checkpoint.config")
    config_keys = {"seed", "batch_size", "learning_rate", "max_steps", "drop_last"}
    if set(stored_config) != config_keys:
        raise ContractError("checkpoint config keys are incompatible")
    stored_smoke_config = SmokeConfig(
        seed=stored_config["seed"],
        batch_size=stored_config["batch_size"],
        learning_rate=stored_config["learning_rate"],
        max_steps=stored_config["max_steps"],
        drop_last=stored_config["drop_last"],
    )
    stored_smoke_config.validate()
    if document.get("config_fingerprint_sha256") != stored_smoke_config.fingerprint_sha256:
        raise ContractError("checkpoint config fingerprint is corrupt")
    if (
        document.get("resume_fingerprint_sha256")
        != stored_smoke_config.resume_fingerprint_sha256
    ):
        raise ContractError("checkpoint resume fingerprint is corrupt")
    if stored_smoke_config.resume_fingerprint_sha256 != config.resume_fingerprint_sha256:
        raise ContractError("checkpoint immutable resume config does not match")
    raw_state = _exact_mapping(document.get("state"), "checkpoint.state")
    weights = raw_state.get("weights")
    if not isinstance(weights, list):
        raise ContractError("checkpoint.state.weights must be an array")
    state = SmokeState(
        weights=weights,
        epoch=raw_state.get("epoch"),
        next_batch_index=raw_state.get("next_batch_index"),
        global_step=raw_state.get("global_step"),
        samples_seen=raw_state.get("samples_seen"),
        first_batch_loss=raw_state.get("first_batch_loss"),
        last_batch_loss=raw_state.get("last_batch_loss"),
        token_trace=raw_state.get("token_trace"),
    )
    _validate_state(state)
    return state


def synthetic_examples(count: int = 24) -> tuple[TrainingExample, ...]:
    if not 8 <= count <= 256:
        raise ContractError("synthetic example count must be in [8, 256]")
    examples: list[TrainingExample] = []
    for index in range(count):
        speed = 0.5 + (index % 8) * 0.12
        lateral_speed = ((index % 3) - 1) * 0.03
        acceleration = ((index % 5) - 2) * 0.02
        yaw_rate = ((index % 7) - 3) * 0.01
        steering = ((index % 9) - 4) * 0.015
        command = index % 6
        one_hot = tuple(float(option == command) for option in range(6))
        features = (
            1.0,
            speed,
            lateral_speed,
            acceleration,
            0.0,
            yaw_rate,
            steering,
            *one_hot,
        )
        targets: list[tuple[float, float] | None] = []
        invalid_tail = (index % 4) * 4
        direction = {0: 1.0, 1: -1.0}.get(command, 0.0)
        for point_index in range(POINT_COUNT):
            if point_index >= POINT_COUNT - invalid_tail:
                targets.append(None)
                continue
            horizon = (point_index + 1) * 0.1
            x = speed * horizon + 0.5 * acceleration * horizon * horizon
            y = (
                lateral_speed * horizon
                + 0.35 * steering * horizon * horizon
                + 0.01 * direction * horizon
            )
            targets.append((x, y))
        examples.append(
            TrainingExample(
                token=f"synthetic:{index:04d}",
                episode_id="synthetic-control-flow-only",
                features=features,
                targets_xy=tuple(targets),
            )
        )
    return tuple(examples)


def state_digest(state: SmokeState) -> str:
    encoded = json.dumps(
        _state_payload(state), sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def run_control_flow_smoke(
    config: SmokeConfig,
    *,
    checkpoint: Path | str | None = None,
    resume: bool = False,
    stop_at_step: int | None = None,
    overwrite_dummy_checkpoint: bool = False,
) -> dict[str, Any]:
    examples = synthetic_examples()
    dataset_fingerprint = fingerprint_examples(examples)
    initial_weights = zero_weights()
    initial_loss = masked_mse(initial_weights, examples)
    state = None
    checkpoint_path = _checkpoint_path(checkpoint) if checkpoint is not None else None
    if resume and overwrite_dummy_checkpoint:
        raise ContractError("--resume and --overwrite-dummy-checkpoint are mutually exclusive")
    if overwrite_dummy_checkpoint and checkpoint_path is None:
        raise ContractError("--overwrite-dummy-checkpoint requires --checkpoint")
    if stop_at_step is not None and checkpoint_path is None:
        raise ContractError("partial --stop-at-step runs require --checkpoint")
    if checkpoint_path is not None and checkpoint_path.exists() and not resume:
        if not overwrite_dummy_checkpoint:
            raise ContractError(
                "dummy checkpoint already exists; use --resume or --overwrite-dummy-checkpoint"
            )
    if resume:
        if checkpoint_path is None:
            raise ContractError("--resume requires --checkpoint")
        state = load_checkpoint(
            checkpoint_path,
            dataset_fingerprint_sha256=dataset_fingerprint,
            config=config,
        )
    target_step = config.max_steps if stop_at_step is None else stop_at_step

    def persist(current: SmokeState) -> None:
        if checkpoint_path is not None:
            save_checkpoint(
                checkpoint_path,
                dataset_fingerprint_sha256=dataset_fingerprint,
                config=config,
                state=current,
            )

    state = train_until(
        examples,
        config,
        state=state,
        stop_at_step=target_step,
        on_step=persist,
    )
    if checkpoint_path is not None and state.global_step == target_step:
        persist(state)
    final_loss = masked_mse(state.weights, examples)
    complete = state.global_step == config.max_steps
    if complete and not final_loss < initial_loss:
        raise ContractError("control-flow optimizer did not reduce the synthetic dataset loss")
    return {
        "schema_id": "autoware-e2e.portable-control-flow-report.v1",
        "status": "PIPELINE_CONTRACT_PASS" if complete else "PIPELINE_CHECKPOINT_SAVED",
        "pipeline_contract_complete": complete,
        "warning": BANNER,
        "production_model_created": False,
        "images_decoded": False,
        "gpu_used": False,
        "optimizer_is_toy": True,
        "metric_name": "toy_masked_mse_not_ADE_FDE",
        "example_count": len(examples),
        "point_count": POINT_COUNT,
        "global_step": state.global_step,
        "initial_dataset_loss": initial_loss,
        "final_dataset_loss": final_loss,
        "state_sha256": state_digest(state),
        "checkpoint": str(checkpoint_path) if checkpoint_path is not None else None,
        "remaining_production_gates": [
            "real image decode and augmentation",
            "PyTorch neural-network training",
            "ONNX and TensorRT parity",
            "CARLA open-loop and closed-loop evaluation",
            "real-vehicle shadow and closed-course safety validation",
        ],
    }


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=SmokeConfig.max_steps)
    parser.add_argument("--batch-size", type=int, default=SmokeConfig.batch_size)
    parser.add_argument("--learning-rate", type=float, default=SmokeConfig.learning_rate)
    parser.add_argument("--seed", type=int, default=SmokeConfig.seed)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--stop-at-step", type=int)
    parser.add_argument("--overwrite-dummy-checkpoint", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    print(BANNER, file=sys.stderr)
    try:
        config = SmokeConfig(
            seed=args.seed,
            batch_size=args.batch_size,
            learning_rate=args.learning_rate,
            max_steps=args.steps,
        )
        report = run_control_flow_smoke(
            config,
            checkpoint=args.checkpoint,
            resume=args.resume,
            stop_at_step=args.stop_at_step,
            overwrite_dummy_checkpoint=args.overwrite_dummy_checkpoint,
        )
    except ContractError as error:
        print(json.dumps({"status": "FAIL", "error": str(error)}, indent=2), file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
