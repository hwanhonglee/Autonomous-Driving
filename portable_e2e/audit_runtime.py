# HH_260906 - Audit every trajectory candidate against the unchanged runtime safety gate.
"""Reproducible full-split geometry audit for portable E2E checkpoints."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import stat
import sys
import time
from typing import Any, Iterable, Mapping, Sequence

import torch
from torch import Tensor
from torch.utils.data import DataLoader

from .contract import ContractError
from .dataset import FEATURE_NAMES, load_training_examples
from .evaluate import _checkpoint_sampling_provenance
from .losses import TrajectoryLossConfig
from .model import ModelConfig, PerspectiveTrajectoryModel, parameter_count
from .runtime import _select_runtime_device
from . import runtime_contract as runtime_contract_module
from .runtime_contract import RuntimeGateConfig, validate_and_select_trajectory
from .runtime_contract import RUNTIME_GATE_ID
from .torch_dataset import Common10TorchDataset
from .train import (
    CHECKPOINT_ID,
    TrainConfig,
    _absolute_path,
    _canonical_sha256,
    _nested_tensors_are_finite,
    _read_checkpoint_file,
    _require_sha256,
    _runtime_abi,
    _seed_everything,
)


AUDIT_ID = "portable_e2e.runtime_geometry_audit.v6"
FAILURE_CODES = (
    "nonfinite_or_shape",
    "spatial",
    "speed",
    "geometric_speed",
    "speed_disagreement",
    "distance_disagreement",
    "speed_rate",
    "geometric_speed_rate",
    "stationary_drift",
    "first_distance",
    "first_behind",
    "backward_step",
    "step",
    "heading",
    "curvature",
    "lateral_acceleration",
    "extent",
    "contract",
)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _source_sha256(path: Path) -> str:
    source = _absolute_path(path)
    if source.is_symlink() or not source.is_file():
        raise ContractError("runtime audit implementation source must be a regular file")
    digest = hashlib.sha256()
    with source.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _atomic_new_json(path: Path, value: Mapping[str, Any]) -> Path:
    output = _absolute_path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.parent.is_symlink() or not output.parent.is_dir():
        raise ContractError("runtime audit output parent must be a regular directory")
    directory_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    directory_flags |= getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        directory_descriptor = os.open(output.parent, directory_flags)
    except OSError as error:
        raise ContractError(f"cannot open runtime audit output directory: {error}") from error
    temporary_name = f".{output.name}.tmp.{os.getpid()}.{secrets.token_hex(8)}"
    descriptor = -1
    linked = False
    completed = False
    payload = (
        json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")
    try:
        try:
            os.stat(output.name, dir_fd=directory_descriptor, follow_symlinks=False)
        except FileNotFoundError:
            pass
        else:
            raise ContractError(f"runtime audit output already exists: {output}")
        file_flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        file_flags |= getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
        descriptor = os.open(
            temporary_name,
            file_flags,
            0o600,
            dir_fd=directory_descriptor,
        )
        with os.fdopen(descriptor, "wb", closefd=False) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(descriptor)
        file_status = os.fstat(descriptor)
        if not stat.S_ISREG(file_status.st_mode) or file_status.st_size != len(payload):
            raise ContractError("generated runtime audit is not a complete regular file")
        # HH_260906 - Publish through a hard link so a concurrent writer cannot be replaced.
        os.link(
            temporary_name,
            output.name,
            src_dir_fd=directory_descriptor,
            dst_dir_fd=directory_descriptor,
            follow_symlinks=False,
        )
        linked = True
        os.fsync(directory_descriptor)
        os.unlink(temporary_name, dir_fd=directory_descriptor)
        temporary_name = ""
        os.fsync(directory_descriptor)
        completed = True
        return output
    except FileExistsError as error:
        raise ContractError(f"runtime audit output already exists: {output}") from error
    except ContractError:
        raise
    except (OSError, RuntimeError, ValueError) as error:
        raise ContractError(f"cannot atomically write runtime audit {output}: {error}") from error
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary_name:
            try:
                os.unlink(temporary_name, dir_fd=directory_descriptor)
            except FileNotFoundError:
                pass
        if linked and not completed:
            try:
                os.unlink(output.name, dir_fd=directory_descriptor)
            except FileNotFoundError:
                pass
        try:
            os.fsync(directory_descriptor)
        except OSError:
            pass
        os.close(directory_descriptor)


def _failure_code(error: ContractError) -> str:
    message = str(error)
    # HH_260906 - Normalize malformed numeric types with other finite/shape failures.
    ordered = (
        ("finite", "nonfinite_or_shape"),
        ("must be a number", "nonfinite_or_shape"),
        ("must contain", "nonfinite_or_shape"),
        ("must be a sequence", "nonfinite_or_shape"),
        ("spatial gate", "spatial"),
        ("geometric speed rate", "geometric_speed_rate"),
        ("geometric speed", "geometric_speed"),
        ("speed gate", "speed"),
        ("disagrees with its displacement", "speed_disagreement"),
        ("distance disagrees with integrated speed", "distance_disagreement"),
        ("speed rate", "speed_rate"),
        ("stationary trajectory exceeds", "stationary_drift"),
        ("first point is too far", "first_distance"),
        ("begins behind", "first_behind"),
        ("moves backward", "backward_step"),
        ("distance gate", "step"),
        ("heading step", "heading"),
        ("curvature", "curvature"),
        ("lateral acceleration", "lateral_acceleration"),
        ("insufficient planar extent", "extent"),
    )
    for marker, code in ordered:
        if marker in message:
            return code
    return "contract"


def _physical_failure_codes(
    xy: Sequence[Sequence[float]],
    speed: Sequence[float],
    config: RuntimeGateConfig,
    current_speed_mps: float | None = None,
) -> tuple[str, ...]:
    # HH_260906 - Reject malformed rows before empty reductions or truncated zip iteration.
    if (
        isinstance(xy, (str, bytes))
        or not isinstance(xy, Sequence)
        or len(xy) != config.future_points
        or isinstance(speed, (str, bytes))
        or not isinstance(speed, Sequence)
        or len(speed) != config.future_points
    ):
        return ("nonfinite_or_shape",)
    failures: set[str] = set()
    previous = (0.0, 0.0)
    heading_anchor = (0.0, 0.0)
    previous_heading = 0.0
    heading_path_m = 0.0
    heading_block_max_speed_mps = 0.0
    heading_block_max_geometric_speed_mps = 0.0
    heading_block_max_step_m = 0.0
    previous_heading_uncertainty_m = 0.0
    extent_m = 0.0
    integrated_speed_distance_m = 0.0
    accumulated_distance_disagreement_m = 0.0
    maximum_radius_m = 0.0
    monotonic_nonincreasing_speed = True
    previous_speed_mps = None
    if current_speed_mps is not None:
        if isinstance(current_speed_mps, bool):
            return ("nonfinite_or_shape",)
        try:
            previous_speed_mps = float(current_speed_mps)
        # HH_260906 - Classify unrepresentable audit numerics instead of aborting the audit.
        except (OverflowError, TypeError, ValueError):
            return ("nonfinite_or_shape",)
        if not math.isfinite(previous_speed_mps):
            return ("nonfinite_or_shape",)
        if not 0.0 <= previous_speed_mps <= config.maximum_speed_mps:
            return ("speed",)
    previous_geometric_speed_mps = previous_speed_mps
    for index, (point, raw_speed) in enumerate(zip(xy, speed)):
        try:
            x_m = float(point[0])
            y_m = float(point[1])
            speed_mps = float(raw_speed)
        # HH_260906 - Keep oversized or malformed point scalars fail-closed in the audit.
        except (IndexError, KeyError, OverflowError, TypeError, ValueError):
            return ("nonfinite_or_shape",)
        if not all(math.isfinite(value) for value in (x_m, y_m, speed_mps)):
            return ("nonfinite_or_shape",)
        if abs(x_m) > config.maximum_abs_x_m or abs(y_m) > config.maximum_abs_y_m:
            failures.add("spatial")
        if speed_mps < 0.0 or speed_mps > config.maximum_speed_mps:
            failures.add("speed")
        segment_entry_speed_mps = previous_speed_mps
        # HH_260906 - Mirror the runtime longitudinal speed-rate gate exactly.
        if previous_speed_mps is not None:
            # HH_260906 - Mirror strict braking detection for the bounded-stop exemption.
            if speed_mps > previous_speed_mps + 1.0e-9:
                monotonic_nonincreasing_speed = False
            speed_rate_mps2 = (
                speed_mps - previous_speed_mps
            ) / config.timestep_s
            if (
                speed_rate_mps2 > config.maximum_acceleration_mps2 + 1.0e-9
                or speed_rate_mps2 < -config.maximum_deceleration_mps2 - 1.0e-9
            ):
                failures.add("speed_rate")
        previous_speed_mps = speed_mps
        dx = x_m - previous[0]
        dy = y_m - previous[1]
        step_m = math.hypot(dx, dy)
        forward_step_m = (
            dx * math.cos(previous_heading) + dy * math.sin(previous_heading)
        )
        # HH_260906 - Mirror rejection of sub-deadband reverse motion.
        if forward_step_m < -config.maximum_backward_step_m - 1.0e-9:
            failures.add("backward_step")
        if index == 0:
            if step_m > config.maximum_first_point_distance_m:
                failures.add("first_distance")
            if x_m < config.minimum_first_point_x_m:
                failures.add("first_behind")
        if step_m > config.maximum_step_m:
            failures.add("step")
        # HH_260906 - Mirror the runtime speed-displacement consistency decision.
        geometric_speed_mps = step_m / float(config.timestep_s)
        if geometric_speed_mps > config.maximum_speed_mps + 1.0e-4:
            failures.add("geometric_speed")
        if (
            abs(geometric_speed_mps - speed_mps)
            > config.maximum_speed_disagreement_mps
        ):
            failures.add("speed_disagreement")
        segment_entry_geometric_speed_mps = previous_geometric_speed_mps
        # HH_260906 - Mirror displacement-derived acceleration without trusting speed output.
        if previous_geometric_speed_mps is not None:
            geometric_speed_rate_mps2 = (
                geometric_speed_mps - previous_geometric_speed_mps
            ) / config.timestep_s
            if (
                geometric_speed_rate_mps2
                > config.maximum_acceleration_mps2 + 1.0e-4
                or geometric_speed_rate_mps2
                < -config.maximum_deceleration_mps2 - 1.0e-4
            ):
                failures.add("geometric_speed_rate")
        previous_geometric_speed_mps = geometric_speed_mps
        extent_m += step_m
        integrated_speed_distance_m += speed_mps * config.timestep_s
        accumulated_distance_disagreement_m += abs(
            step_m - speed_mps * config.timestep_s
        )
        maximum_radius_m = max(maximum_radius_m, math.hypot(x_m, y_m))
        heading_path_m += step_m
        if segment_entry_speed_mps is not None:
            heading_block_max_speed_mps = max(
                heading_block_max_speed_mps, segment_entry_speed_mps
            )
        heading_block_max_speed_mps = max(
            heading_block_max_speed_mps, speed_mps
        )
        if segment_entry_geometric_speed_mps is not None:
            heading_block_max_geometric_speed_mps = max(
                heading_block_max_geometric_speed_mps,
                segment_entry_geometric_speed_mps,
            )
        heading_block_max_geometric_speed_mps = max(
            heading_block_max_geometric_speed_mps, geometric_speed_mps
        )
        heading_block_max_step_m = max(heading_block_max_step_m, step_m)
        # HH_260906 - Mirror the runtime bounded heading-deadband policy exactly.
        heading_dx = x_m - heading_anchor[0]
        heading_dy = y_m - heading_anchor[1]
        heading_displacement_m = math.hypot(heading_dx, heading_dy)
        if heading_displacement_m >= config.heading_minimum_step_m:
            heading = math.atan2(heading_dy, heading_dx)
            heading_delta = math.atan2(
                math.sin(heading - previous_heading),
                math.cos(heading - previous_heading),
            )
            if abs(heading_delta) > config.maximum_heading_step_rad:
                failures.add("heading")
            # HH_260906 - Mirror the runtime curvature gate for every audited candidate.
            curvature_path_m = heading_path_m + previous_heading_uncertainty_m
            implied_curvature_rad_per_m = abs(heading_delta) / curvature_path_m
            if implied_curvature_rad_per_m > (
                config.maximum_curvature_rad_per_m + 1.0e-9
            ):
                failures.add("curvature")
            lateral_speed_mps = max(
                heading_block_max_speed_mps,
                heading_block_max_geometric_speed_mps,
            )
            # HH_260906 - Mirror the speed-coupled lateral acceleration decision.
            lateral_speed_limit_mps = math.sqrt(
                (config.maximum_lateral_acceleration_mps2 + 1.0e-6)
                / implied_curvature_rad_per_m
            ) if implied_curvature_rad_per_m > 0.0 else math.inf
            if lateral_speed_mps > lateral_speed_limit_mps:
                failures.add("lateral_acceleration")
            previous_heading = heading
            heading_anchor = (x_m, y_m)
            # HH_260906 - Mirror sub-deadband heading aggregation uncertainty.
            previous_heading_uncertainty_m = max(
                0.0, heading_path_m - heading_block_max_step_m
            )
            heading_path_m = 0.0
            heading_block_max_speed_mps = 0.0
            heading_block_max_geometric_speed_mps = 0.0
            heading_block_max_step_m = 0.0
        elif heading_path_m >= config.heading_minimum_step_m:
            unresolved_motion_speed_mps = max(
                heading_block_max_speed_mps,
                heading_block_max_geometric_speed_mps,
            )
            if unresolved_motion_speed_mps > config.stationary_speed_tolerance_mps:
                failures.add("heading")
            # HH_260906 - Mirror retained unresolved travel until direction or excess resolves it.
            if heading_path_m > config.maximum_stationary_extent_m:
                failures.add("heading")
        previous = (x_m, y_m)
    maximum_selected_speed_mps = max(float(value) for value in speed)
    # HH_260906 - Mirror explicit-stop and confined-oscillation limits exactly.
    claimed_stationary_drift = (
        maximum_selected_speed_mps <= config.stationary_claim_speed_epsilon_mps
        and (
            maximum_radius_m > config.maximum_stationary_radius_m
            or extent_m > config.maximum_stationary_extent_m
        )
    )
    confined_oscillation = (
        maximum_radius_m <= config.maximum_stationary_radius_m
        and extent_m > config.maximum_stationary_extent_m
    )
    final_radius_m = math.hypot(*previous)
    low_speed_oscillation = (
        maximum_selected_speed_mps <= config.stationary_speed_tolerance_mps
        and extent_m > config.maximum_stationary_extent_m
        and final_radius_m
        < config.minimum_low_speed_progress_ratio * extent_m
    )
    # HH_260906 - Mirror shifted low-speed shuttle rejection.
    if claimed_stationary_drift or confined_oscillation or low_speed_oscillation:
        failures.add("stationary_drift")
    integrated_distance_tolerance_m = max(
        config.maximum_integrated_distance_disagreement_m,
        config.maximum_integrated_distance_disagreement_ratio
        * max(extent_m, integrated_speed_distance_m),
    )
    # HH_260906 - Mirror absolute and relative full-horizon distance consistency.
    if accumulated_distance_disagreement_m > integrated_distance_tolerance_m:
        failures.add("distance_disagreement")
    dynamically_consistent_stop = (
        monotonic_nonincreasing_speed
        and previous_speed_mps is not None
        and previous_speed_mps <= config.stationary_claim_speed_epsilon_mps
    )
    # HH_260906 - Mirror the runtime's bounded low-speed stop allowance.
    if (
        extent_m < config.minimum_planar_extent_m
        and maximum_selected_speed_mps > config.stationary_speed_tolerance_mps
        and not dynamically_consistent_stop
    ):
        failures.add("extent")
    return tuple(code for code in FAILURE_CODES if code in failures)


def audit_prediction(
    candidate_xy: Any,
    candidate_speed: Any,
    candidate_logits: Any,
    config: RuntimeGateConfig | None = None,
    *,
    current_speed_mps: float | None = None,
) -> dict[str, Any]:
    """Audit each candidate and the learned argmax without changing gate thresholds."""
    gate = RuntimeGateConfig() if config is None else config
    gate.validate()
    results: list[dict[str, Any]] = []
    for candidate_index in range(gate.candidate_count):
        forced_logits = [-1.0] * gate.candidate_count
        forced_logits[candidate_index] = 1.0
        try:
            validate_and_select_trajectory(
                candidate_xy,
                candidate_speed,
                forced_logits,
                gate,
                current_speed_mps=current_speed_mps,
            )
        except ContractError as error:
            codes: tuple[str, ...]
            try:
                codes = _physical_failure_codes(
                    candidate_xy[candidate_index],
                    candidate_speed[candidate_index],
                    gate,
                    current_speed_mps,
                )
            except (IndexError, KeyError, TypeError):
                codes = ()
            if not codes:
                codes = (_failure_code(error),)
            results.append(
                {
                    "candidate_index": candidate_index,
                    "geometry_pass": False,
                    "failure_codes": list(codes),
                }
            )
        else:
            results.append(
                {
                    "candidate_index": candidate_index,
                    "geometry_pass": True,
                    "failure_codes": [],
                }
            )

    selected_index: int | None = None
    selected_pass = False
    selected_failure_codes: list[str] = []
    try:
        selected = validate_and_select_trajectory(
            candidate_xy,
            candidate_speed,
            candidate_logits,
            gate,
            current_speed_mps=current_speed_mps,
        )
    except ContractError as error:
        try:
            parsed_logits = tuple(float(value) for value in candidate_logits)
            if len(parsed_logits) == gate.candidate_count and all(
                math.isfinite(value) for value in parsed_logits
            ):
                selected_index = max(
                    range(gate.candidate_count), key=parsed_logits.__getitem__
                )
        # HH_260906 - Treat unrepresentable selector logits as malformed audit input.
        except (OverflowError, TypeError, ValueError):
            selected_index = None
        if selected_index is not None:
            selected_failure_codes = list(results[selected_index]["failure_codes"])
        if not selected_failure_codes:
            selected_failure_codes = [_failure_code(error)]
    else:
        selected_index = selected.candidate_index
        selected_pass = True
    return {
        "candidates": results,
        "all_candidates_geometry_pass": all(
            result["geometry_pass"] for result in results
        ),
        "any_candidate_geometry_pass": any(
            result["geometry_pass"] for result in results
        ),
        "selected_candidate_index": selected_index,
        "selected_geometry_pass": selected_pass,
        "selected_failure_codes": selected_failure_codes,
    }


class _AuditAccumulator:
    def __init__(self, config: RuntimeGateConfig) -> None:
        config.validate()
        self.config = config
        self.sample_count = 0
        self.all_candidates_pass_count = 0
        self.any_candidate_pass_count = 0
        self.candidate_pass_counts = [0] * config.candidate_count
        self.candidate_failure_counts = [
            {code: 0 for code in FAILURE_CODES} for _ in range(config.candidate_count)
        ]
        self.selected_pass_count = 0
        self.selected_failure_counts = {code: 0 for code in FAILURE_CODES}
        self.selection_counts = [0] * config.candidate_count
        self.invalid_selection_count = 0

    def add(self, audit: Mapping[str, Any]) -> None:
        candidates = audit.get("candidates")
        if not isinstance(candidates, list) or len(candidates) != self.config.candidate_count:
            raise ContractError("runtime audit candidate result has the wrong ABI")
        self.sample_count += 1
        self.all_candidates_pass_count += int(
            bool(audit.get("all_candidates_geometry_pass"))
        )
        self.any_candidate_pass_count += int(
            bool(audit.get("any_candidate_geometry_pass"))
        )
        for expected_index, result in enumerate(candidates):
            if result.get("candidate_index") != expected_index:
                raise ContractError("runtime audit candidate indices are not canonical")
            passed = result.get("geometry_pass") is True
            self.candidate_pass_counts[expected_index] += int(passed)
            raw_codes = result.get("failure_codes")
            if not isinstance(raw_codes, list) or any(
                code not in FAILURE_CODES for code in raw_codes
            ):
                raise ContractError("runtime audit candidate failure code is invalid")
            for code in set(raw_codes):
                self.candidate_failure_counts[expected_index][code] += 1

        selected_index = audit.get("selected_candidate_index")
        if (
            isinstance(selected_index, bool)
            or not isinstance(selected_index, int)
            or not 0 <= selected_index < self.config.candidate_count
        ):
            self.invalid_selection_count += 1
        else:
            self.selection_counts[selected_index] += 1
        selected_pass = audit.get("selected_geometry_pass") is True
        self.selected_pass_count += int(selected_pass)
        raw_selected_codes = audit.get("selected_failure_codes")
        if not isinstance(raw_selected_codes, list) or any(
            code not in FAILURE_CODES for code in raw_selected_codes
        ):
            raise ContractError("runtime audit selected failure code is invalid")
        for code in set(raw_selected_codes):
            self.selected_failure_counts[code] += 1

    @staticmethod
    def _nonzero_counts(counts: Mapping[str, int]) -> dict[str, int]:
        return {key: counts[key] for key in FAILURE_CODES if counts.get(key, 0)}

    def report(self) -> dict[str, Any]:
        if self.sample_count <= 0:
            raise ContractError("runtime audit observed no samples")
        candidate_reports = []
        for index in range(self.config.candidate_count):
            passed = self.candidate_pass_counts[index]
            candidate_reports.append(
                {
                    "candidate_index": index,
                    "sample_count": self.sample_count,
                    "geometry_pass_count": passed,
                    "geometry_reject_count": self.sample_count - passed,
                    "geometry_pass_rate": passed / self.sample_count,
                    "failure_counts": self._nonzero_counts(
                        self.candidate_failure_counts[index]
                    ),
                }
            )
        dominant_count = max(self.selection_counts, default=0)
        dominant_index = (
            self.selection_counts.index(dominant_count) if dominant_count else None
        )
        probabilities = [count / self.sample_count for count in self.selection_counts]
        entropy = -sum(
            probability * math.log(probability)
            for probability in probabilities
            if probability > 0.0
        )
        normalizer = math.log(self.config.candidate_count)
        normalized_entropy = entropy / normalizer if normalizer > 0.0 else 0.0
        return {
            "sample_count": self.sample_count,
            "all_candidates_geometry_pass_count": self.all_candidates_pass_count,
            "all_candidates_geometry_pass_rate": (
                self.all_candidates_pass_count / self.sample_count
            ),
            "any_candidate_geometry_pass_count": self.any_candidate_pass_count,
            "any_candidate_geometry_pass_rate": (
                self.any_candidate_pass_count / self.sample_count
            ),
            "candidate_results": candidate_reports,
            "selected_result": {
                "sample_count": self.sample_count,
                "geometry_pass_count": self.selected_pass_count,
                "geometry_reject_count": self.sample_count - self.selected_pass_count,
                "geometry_pass_rate": self.selected_pass_count / self.sample_count,
                "failure_counts": self._nonzero_counts(
                    self.selected_failure_counts
                ),
            },
            "selector": {
                "selection_counts": {
                    str(index): count
                    for index, count in enumerate(self.selection_counts)
                },
                "invalid_selection_count": self.invalid_selection_count,
                "distinct_selected_candidates": sum(
                    count > 0 for count in self.selection_counts
                ),
                "dominant_candidate_index": dominant_index,
                "dominant_candidate_fraction": dominant_count / self.sample_count,
                "normalized_selection_entropy": normalized_entropy,
            },
        }


def _read_checkpoint_for_audit(
    *,
    checkpoint_path: Path,
    expected_checkpoint_sha256: str,
    corpus_fingerprint_sha256: str,
) -> tuple[Mapping[str, Any], ModelConfig, tuple[str, ...], dict[str, Any]]:
    _require_sha256(expected_checkpoint_sha256, "runtime audit checkpoint fingerprint")
    _require_sha256(corpus_fingerprint_sha256, "runtime audit corpus fingerprint")
    checkpoint = _absolute_path(checkpoint_path)
    payload, checkpoint_sha256 = _read_checkpoint_file(
        checkpoint, torch.device("cpu")
    )
    if checkpoint_sha256 != expected_checkpoint_sha256:
        raise ContractError("runtime audit checkpoint fingerprint does not match")
    if payload.get("checkpoint_id") != CHECKPOINT_ID:
        raise ContractError("runtime audit checkpoint kind is not supported")
    if payload.get("training_split") != "train":
        raise ContractError("runtime audit checkpoint was not trained on train split")
    if payload.get("corpus_fingerprint_sha256") != corpus_fingerprint_sha256:
        raise ContractError("runtime audit corpus fingerprint does not match")
    sampling_plan_sha256, sampling_policy, domain_samples_seen = (
        _checkpoint_sampling_provenance(payload)
    )
    training_dataset_fingerprint = payload.get("dataset_fingerprint_sha256")
    _require_sha256(
        training_dataset_fingerprint,
        "runtime audit training dataset fingerprint",
    )
    training_episode_ids = payload.get("training_episode_ids")
    if (
        not isinstance(training_episode_ids, list)
        or not training_episode_ids
        or any(not isinstance(value, str) or not value for value in training_episode_ids)
        or training_episode_ids != sorted(training_episode_ids)
        or len(training_episode_ids) != len(set(training_episode_ids))
    ):
        raise ContractError("runtime audit training episode IDs are invalid")
    model_value = payload.get("model_config")
    if not isinstance(model_value, Mapping):
        raise ContractError("runtime audit checkpoint model config is missing")
    model_config = ModelConfig.from_mapping(model_value)
    model_config_sha256 = _canonical_sha256(model_config.to_dict())
    if payload.get("model_config_sha256") != model_config_sha256:
        raise ContractError("runtime audit model config fingerprint does not match")
    loss_value = payload.get("loss_config")
    if not isinstance(loss_value, Mapping) or set(loss_value) != set(
        TrajectoryLossConfig().to_dict()
    ):
        raise ContractError("runtime audit loss config fields do not match")
    try:
        TrajectoryLossConfig(**dict(loss_value)).validate()
    except (ContractError, TypeError) as error:
        raise ContractError(f"runtime audit loss config is invalid: {error}") from error
    train_value = payload.get("train_config")
    state = payload.get("state")
    if not isinstance(train_value, Mapping) or not isinstance(state, Mapping):
        raise ContractError("runtime audit checkpoint completion state is missing")
    try:
        train_config = TrainConfig(**dict(train_value))
        train_config.validate()
    except (ContractError, TypeError) as error:
        raise ContractError(f"runtime audit train config is invalid: {error}") from error
    if state.get("global_step") != train_config.max_steps:
        raise ContractError("runtime audit rejects an intermediate checkpoint")

    provenance = {
        "checkpoint_sha256": checkpoint_sha256,
        "checkpoint_id": CHECKPOINT_ID,
        "model_config_sha256": model_config_sha256,
        "training_dataset_fingerprint_sha256": training_dataset_fingerprint,
        "training_episode_count": len(training_episode_ids),
        "training_sampling_plan_sha256": sampling_plan_sha256,
        "training_sampling_policy": sampling_policy,
        "training_domain_samples_seen": domain_samples_seen,
    }
    return payload, model_config, tuple(training_episode_ids), provenance


def _validate_checkpoint_and_model(
    dataset: Common10TorchDataset,
    *,
    payload: Mapping[str, Any],
    model_config: ModelConfig,
    training_episode_ids: Sequence[str],
    checkpoint_provenance: Mapping[str, Any],
    device: torch.device,
) -> tuple[PerspectiveTrajectoryModel, dict[str, Any]]:
    if model_config != dataset.config:
        raise ContractError("runtime audit dataset ABI does not match the checkpoint")
    training_dataset_fingerprint = checkpoint_provenance.get(
        "training_dataset_fingerprint_sha256"
    )
    _require_sha256(
        training_dataset_fingerprint,
        "runtime audit training dataset fingerprint",
    )
    if training_dataset_fingerprint == dataset.fingerprint_sha256:
        raise ContractError("runtime audit split fingerprint matches the training split")
    evaluation_episode_ids = sorted(
        {example.episode_id for example in dataset.examples}
    )
    overlap = sorted(set(training_episode_ids) & set(evaluation_episode_ids))
    if overlap:
        raise ContractError("runtime audit detected train/evaluation episode leakage")

    model = PerspectiveTrajectoryModel(model_config).to(device)
    try:
        model.load_state_dict(payload["model_state_dict"], strict=True)
    except (KeyError, RuntimeError, TypeError, ValueError) as error:
        raise ContractError(
            f"runtime audit checkpoint model tensors are incompatible: {error}"
        ) from error
    if not _nested_tensors_are_finite(model.state_dict()):
        raise ContractError("runtime audit checkpoint model contains NaN or Inf")
    model.requires_grad_(False)
    model.eval()
    provenance = {
        **checkpoint_provenance,
        "evaluation_episode_count": len(evaluation_episode_ids),
        "model_parameter_count": parameter_count(model),
    }
    return model, provenance


def _audit_model_batches(
    model: PerspectiveTrajectoryModel,
    batches: Iterable[Mapping[str, Any]],
    *,
    device: torch.device,
    gate_config: RuntimeGateConfig,
) -> tuple[dict[str, Any], dict[str, float]]:
    accumulator = _AuditAccumulator(gate_config)
    forward_seconds = 0.0
    wall_started = time.perf_counter()
    with torch.no_grad():
        for batch in batches:
            tensor_batch = {
                key: value.to(device) if isinstance(value, Tensor) else value
                for key, value in batch.items()
            }
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            forward_started = time.perf_counter()
            candidate_xy, candidate_speed, candidate_logits = model(
                tensor_batch["images"],
                tensor_batch["calibration"],
                tensor_batch["ego_history"],
                tensor_batch["ego_history_mask"],
                tensor_batch["route_xy"],
                tensor_batch["route_mask"],
            )
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            forward_seconds += time.perf_counter() - forward_started
            batch_size = int(tensor_batch["images"].shape[0])
            if any(
                not isinstance(value, Tensor) or int(value.shape[0]) != batch_size
                for value in (candidate_xy, candidate_speed, candidate_logits)
            ):
                raise ContractError("runtime audit model returned a wrong batch ABI")
            xy_rows = candidate_xy.detach().cpu().tolist()
            speed_rows = candidate_speed.detach().cpu().tolist()
            logit_rows = candidate_logits.detach().cpu().tolist()
            current_speed_rows = (
                tensor_batch["ego_history"][
                    :, -1, FEATURE_NAMES.index("velocity_x_mps")
                ]
                .detach()
                .cpu()
                .tolist()
            )
            for xy, speed, logits, current_speed_mps in zip(
                xy_rows, speed_rows, logit_rows, current_speed_rows
            ):
                accumulator.add(
                    audit_prediction(
                        xy,
                        speed,
                        logits,
                        gate_config,
                        current_speed_mps=float(current_speed_mps),
                    )
                )
    wall_seconds = time.perf_counter() - wall_started
    if forward_seconds <= 0.0 or wall_seconds < forward_seconds:
        raise ContractError("runtime audit timing is invalid")
    summary = accumulator.report()
    timing = {
        "audit_wall_seconds": wall_seconds,
        "model_forward_seconds": forward_seconds,
        "model_forward_ms_per_sample": (
            1000.0 * forward_seconds / summary["sample_count"]
        ),
    }
    return summary, timing


def audit_checkpoint(
    dataset_root: Path,
    *,
    checkpoint_path: Path,
    expected_checkpoint_sha256: str,
    output_json: Path,
    split: str = "val",
    device_name: str = "cpu",
    batch_size: int = 4,
) -> dict[str, Any]:
    """Audit a complete val/test split and atomically preserve the result."""
    if split not in ("val", "test"):
        raise ContractError("runtime audit split must be val or test")
    if isinstance(batch_size, bool) or not isinstance(batch_size, int) or batch_size <= 0:
        raise ContractError("runtime audit batch_size must be a positive integer")
    device = _select_runtime_device(device_name)
    _seed_everything(0, device)
    loaded = load_training_examples(
        dataset_root,
        split=split,
        mode="planning",
        check_image_hashes=True,
    )
    corpus_fingerprint = loaded.validation_report.get(
        "dataset_fingerprint_sha256"
    )
    _require_sha256(corpus_fingerprint, "runtime audit corpus fingerprint")
    checkpoint_payload, model_config, training_episode_ids, checkpoint_provenance = (
        _read_checkpoint_for_audit(
            checkpoint_path=checkpoint_path,
            expected_checkpoint_sha256=expected_checkpoint_sha256,
            corpus_fingerprint_sha256=corpus_fingerprint,
        )
    )
    dataset = Common10TorchDataset(
        loaded.examples,
        model_config,
        verify_image_sha256=True,
        split=split,
    )
    model, provenance = _validate_checkpoint_and_model(
        dataset,
        payload=checkpoint_payload,
        model_config=model_config,
        training_episode_ids=training_episode_ids,
        checkpoint_provenance=checkpoint_provenance,
        device=device,
    )
    gate = RuntimeGateConfig(
        candidate_count=model.config.candidate_count,
        future_points=model.config.future_points,
    )
    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=0,
        pin_memory=False,
    )
    summary, timing = _audit_model_batches(
        model,
        loader,
        device=device,
        gate_config=gate,
    )
    expected_domain_counts: dict[str, int] = {}
    for example in dataset.examples:
        expected_domain_counts[example.domain] = (
            expected_domain_counts.get(example.domain, 0) + 1
        )
    report = {
        "audit_id": AUDIT_ID,
        "created_at_utc": _utc_now(),
        "status": "RUNTIME_GEOMETRY_AUDIT_COMPLETE",
        "warning": "RESEARCH SHADOW AUDIT ONLY — NOT APPROVED FOR VEHICLE CONTROL",
        **provenance,
        "dataset_id": loaded.validation_report.get("dataset_id"),
        "dataset_fingerprint_sha256": dataset.fingerprint_sha256,
        "corpus_fingerprint_sha256": corpus_fingerprint,
        "evaluation_split": split,
        "domain_sample_counts": dict(sorted(expected_domain_counts.items())),
        "device": str(device),
        "batch_size": batch_size,
        "implementation": {
            "audit_runtime_sha256": _source_sha256(Path(__file__)),
            "runtime_contract_sha256": _source_sha256(
                Path(runtime_contract_module.__file__)
            ),
        },
        "runtime": _runtime_abi(),
        "gate": {
            "source": RUNTIME_GATE_ID,
            "thresholds": asdict(gate),
            "threshold_overrides": False,
        },
        "geometry": summary,
        "timing": timing,
        "vehicle_control_approved": False,
    }
    _atomic_new_json(output_json, report)
    return report


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Audit every model candidate against the fixed runtime geometry gate."
    )
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--checkpoint-sha256", required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--split", choices=("val", "test"), default="val")
    parser.add_argument("--device", choices=("cpu", "cuda:0"), default="cpu")
    parser.add_argument("--batch-size", type=int, default=4)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        report = audit_checkpoint(
            args.dataset,
            checkpoint_path=args.checkpoint,
            expected_checkpoint_sha256=args.checkpoint_sha256,
            output_json=args.output_json,
            split=args.split,
            device_name=args.device,
            batch_size=args.batch_size,
        )
    except (ContractError, FloatingPointError, OSError, RuntimeError, ValueError) as error:
        print(f"RUNTIME_AUDIT_ERROR: {error}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
