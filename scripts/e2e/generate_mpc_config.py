#!/usr/bin/env python3
"""Create a complete standard MPC parameter file with explicit plant timing."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BASE = (
    ROOT
    / "src/launcher/autoware_launch/autoware_launch/config/control"
    / "trajectory_follower/lateral/mpc.param.yaml"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-delay", type=float, required=True)
    parser.add_argument("--steer-tau", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--base", type=Path, default=DEFAULT_BASE)
    parser.add_argument("--control-period", type=float, default=0.03)
    parser.add_argument(
        "--steering-input-weight",
        type=float,
        help="override both nominal and low-curvature steering-input weights",
    )
    parser.add_argument(
        "--heading-error-squared-vel-weight",
        type=float,
        help="override both nominal and low-curvature velocity-scaled heading weights",
    )
    parser.add_argument(
        "--steer-rate-weight",
        type=float,
        help="override both nominal and low-curvature steering-rate weights",
    )
    parser.add_argument(
        "--steer-acc-weight",
        type=float,
        help="override both nominal and low-curvature steering-acceleration weights",
    )
    parser.add_argument(
        "--steering-lpf-cutoff-hz",
        type=float,
        help="override the post-MPC steering-command low-pass cutoff",
    )
    parser.add_argument(
        "--steer-rate-limit-dps",
        type=float,
        help="replace every curvature- and velocity-based steering-rate limit",
    )
    return parser.parse_args()


def validate_timing(input_delay: float, steer_tau: float, control_period: float) -> None:
    if not math.isfinite(input_delay) or input_delay < 0.0:
        raise ValueError("input delay must be finite and non-negative")
    if not math.isfinite(steer_tau) or steer_tau <= 0.0:
        raise ValueError("steer tau must be positive and finite")
    if not math.isfinite(control_period) or control_period <= 0.0:
        raise ValueError("control period must be positive and finite")
    delay_steps = round(input_delay / control_period)
    effective_delay = delay_steps * control_period
    if not math.isclose(input_delay, effective_delay, abs_tol=1.0e-9):
        raise ValueError(
            f"input delay must be a multiple of {control_period:g} s; "
            f"the controller would round {input_delay:g} to {effective_delay:g}"
        )


def validate_optional_overrides(
    steering_input_weight: float | None,
    heading_error_squared_vel_weight: float | None,
    steer_rate_weight: float | None,
    steer_acc_weight: float | None,
    steering_lpf_cutoff_hz: float | None,
    steer_rate_limit_dps: float | None,
) -> None:
    non_negative = {
        "steering input weight": steering_input_weight,
        "velocity-scaled heading error weight": heading_error_squared_vel_weight,
        "steer rate weight": steer_rate_weight,
        "steer acceleration weight": steer_acc_weight,
    }
    for label, value in non_negative.items():
        if value is not None and (not math.isfinite(value) or value < 0.0):
            raise ValueError(f"{label} must be finite and non-negative")
    positive = {
        "steering LPF cutoff": steering_lpf_cutoff_hz,
        "steer rate limit": steer_rate_limit_dps,
    }
    for label, value in positive.items():
        if value is not None and (not math.isfinite(value) or value <= 0.0):
            raise ValueError(f"{label} must be positive and finite")


def atomic_write(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        Path(temporary_name).unlink(missing_ok=True)
        raise


def generate(
    base: Path,
    output: Path,
    input_delay: float,
    steer_tau: float,
    control_period: float,
    *,
    steering_input_weight: float | None = None,
    heading_error_squared_vel_weight: float | None = None,
    steer_rate_weight: float | None = None,
    steer_acc_weight: float | None = None,
    steering_lpf_cutoff_hz: float | None = None,
    steer_rate_limit_dps: float | None = None,
) -> dict[str, object]:
    validate_timing(input_delay, steer_tau, control_period)
    validate_optional_overrides(
        steering_input_weight,
        heading_error_squared_vel_weight,
        steer_rate_weight,
        steer_acc_weight,
        steering_lpf_cutoff_hz,
        steer_rate_limit_dps,
    )
    base_bytes = base.read_bytes()
    payload = yaml.safe_load(base_bytes)
    try:
        parameters = payload["/**"]["ros__parameters"]
    except (KeyError, TypeError) as error:
        raise ValueError(f"invalid ROS parameter structure in {base}") from error
    parameters["input_delay"] = float(input_delay)
    parameters["vehicle_model_steer_tau"] = float(steer_tau)
    parameter_overrides: dict[str, object] = {
        "input_delay": float(input_delay),
        "vehicle_model_steer_tau": float(steer_tau),
    }

    uniform_weight_overrides = {
        "steering_input": steering_input_weight,
        "heading_error_squared_vel": heading_error_squared_vel_weight,
        "steer_rate": steer_rate_weight,
        "steer_acc": steer_acc_weight,
    }
    for suffix, value in uniform_weight_overrides.items():
        if value is None:
            continue
        for prefix in ("mpc_weight_", "mpc_low_curvature_weight_"):
            name = prefix + suffix
            parameters[name] = float(value)
            parameter_overrides[name] = float(value)
    if steering_lpf_cutoff_hz is not None:
        parameters["steering_lpf_cutoff_hz"] = float(steering_lpf_cutoff_hz)
        parameter_overrides["steering_lpf_cutoff_hz"] = float(
            steering_lpf_cutoff_hz
        )
    if steer_rate_limit_dps is not None:
        for name in (
            "steer_rate_lim_dps_list_by_curvature",
            "steer_rate_lim_dps_list_by_velocity",
        ):
            current = parameters.get(name)
            if not isinstance(current, list) or not current:
                raise ValueError(f"base parameter {name} must be a non-empty list")
            replacement = [float(steer_rate_limit_dps)] * len(current)
            parameters[name] = replacement
            parameter_overrides[name] = replacement

    header = (
        "# Generated by scripts/e2e/generate_mpc_config.py.\n"
        "# Do not edit; regenerate it from the pinned stock Autoware MPC file.\n"
    )
    atomic_write(output, header + yaml.safe_dump(payload, sort_keys=False))
    metadata = {
        "schema_version": 1,
        "base_file": str(base.resolve()),
        "base_sha256": hashlib.sha256(base_bytes).hexdigest(),
        "output_file": str(output.resolve()),
        "input_delay_sec": float(input_delay),
        "effective_input_delay_sec": float(input_delay),
        "vehicle_model_steer_tau_sec": float(steer_tau),
        "control_period_sec": float(control_period),
        "parameter_overrides": parameter_overrides,
    }
    atomic_write(
        output.with_suffix(output.suffix + ".metadata.json"),
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
    )
    return metadata


def main() -> int:
    args = parse_args()
    metadata = generate(
        args.base.expanduser().resolve(),
        args.output.expanduser().resolve(),
        args.input_delay,
        args.steer_tau,
        args.control_period,
        steering_input_weight=args.steering_input_weight,
        heading_error_squared_vel_weight=args.heading_error_squared_vel_weight,
        steer_rate_weight=args.steer_rate_weight,
        steer_acc_weight=args.steer_acc_weight,
        steering_lpf_cutoff_hz=args.steering_lpf_cutoff_hz,
        steer_rate_limit_dps=args.steer_rate_limit_dps,
    )
    print(json.dumps(metadata, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
