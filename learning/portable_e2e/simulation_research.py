"""HH_261001 - Isolated AH inference and control proposals for owned CARLA research."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from io import BytesIO
import hashlib
import json
import math
from pathlib import Path
import time
import zipfile

import numpy as np
import torch

from .contract import ContractError, _read_regular_file_bounded
from .dataset import FEATURE_NAMES
from .model import ModelConfig, PerspectiveTrajectoryModel, PHYSICAL_STOPMIX_MODEL_ID
from .runtime import RuntimeInputs, build_runtime_tensors
from .runtime_contract import RuntimeGateConfig, SelectedTrajectory, validate_and_select_trajectory
from .train import _read_checkpoint_file


AH_CHECKPOINT_SHA256 = "4841d4076fb49145643ab2895d87befcae9021ad02f08b8307e75f77be137d91"
CHECKPOINT_SCHEMA = "portable_e2e.near_horizon_ah.v1"
WEIGHTS_SCHEMA = "portable_e2e.simulation_research_weights.v1"
DECISION_SCHEMA = "portable_e2e.simulation_research_control.v1"
MAX_WEIGHTS_BYTES = 32 * 1024 * 1024
FIELDS = ("images", "calibration", "ego_history", "ego_history_mask", "route_xy", "route_mask")


def _require(condition, message):
    if not condition:
        raise ContractError(message)


def _pinned_bytes(path, expected, limit=MAX_WEIGHTS_BYTES):
    data = _read_regular_file_bounded(Path(path), limit, "simulation research input")
    _require(hashlib.sha256(data).hexdigest() == expected, "simulation research SHA256 mismatch")
    return data


def _model(config, state):
    """HH_261001 - Reject coercion, missing weights and any unexpected model ABI."""
    cfg = ModelConfig.from_mapping(config)
    _require(cfg.model_id == PHYSICAL_STOPMIX_MODEL_ID and cfg.candidate_count == 12
             and cfg.future_points == 64, "AH requires the twelve-candidate STOPMIX ABI")
    model = PerspectiveTrajectoryModel(cfg)
    expected = model.state_dict()
    _require(set(state) == set(expected), "research state keys differ")
    for name, value in state.items():
        _require(isinstance(value, torch.Tensor) and value.shape == expected[name].shape
                 and value.dtype == expected[name].dtype
                 and bool(torch.isfinite(value).all()), f"research state ABI differs: {name}")
    model.load_state_dict(state, strict=True)
    model.requires_grad_(False)
    model._research_source_checkpoint_sha256 = AH_CHECKPOINT_SHA256
    return model.eval()


def export_ah_weights(checkpoint, output):
    """HH_261001 - Export pinned AH on modern CPU Torch; no unsafe legacy pickle fallback."""
    checkpoint, output = Path(checkpoint), Path(output)
    metadata_path = output.with_suffix(".json")
    _require(output.suffix == ".npz" and not output.exists() and not metadata_path.exists(),
             "fresh .npz and .json export paths required")
    _pinned_bytes(checkpoint, AH_CHECKPOINT_SHA256)
    payload, actual = _read_checkpoint_file(checkpoint, torch.device("cpu"))
    _require(actual == AH_CHECKPOINT_SHA256 and payload.get("checkpoint_schema") == CHECKPOINT_SCHEMA,
             "AH checkpoint identity differs")
    model = _model(payload["model_config"], payload["model_state_dict"])
    arrays = {name: value.cpu().numpy().copy() for name, value in model.state_dict().items()}
    # HH_261001 - NPZ contains arrays only; a pinned sidecar binds source and tensor ABI.
    with output.open("xb") as stream:
        np.savez(stream, **arrays)
    weights_sha = hashlib.sha256(_read_regular_file_bounded(output, MAX_WEIGHTS_BYTES, "weights export")).hexdigest()
    metadata = dict(schema=WEIGHTS_SCHEMA, source_checkpoint_sha256=actual,
                    source_checkpoint_schema=CHECKPOINT_SCHEMA, weights_sha256=weights_sha,
                    model_config=model.config.to_dict(),
                    tensors={name: dict(shape=list(value.shape), dtype=str(value.dtype))
                             for name, value in arrays.items()},
                    simulation_only=True, vehicle_control_approved=False, model_promoted=False)
    encoded = (json.dumps(metadata, sort_keys=True, indent=2, allow_nan=False) + "\n").encode()
    with metadata_path.open("xb") as stream:
        stream.write(encoded)
    return dict(weights=str(output), metadata=str(metadata_path), weights_sha256=weights_sha,
                metadata_sha256=hashlib.sha256(encoded).hexdigest(), source_checkpoint_sha256=actual)


def load_ah_weights(weights, metadata, *, weights_sha256, metadata_sha256):
    """HH_261001 - Load data-only research weights on old local Torch with exact caller pins."""
    description = json.loads(_pinned_bytes(metadata, metadata_sha256, 1024 * 1024))
    _require(description.get("schema") == WEIGHTS_SCHEMA
             and description.get("source_checkpoint_sha256") == AH_CHECKPOINT_SHA256
             and description.get("source_checkpoint_schema") == CHECKPOINT_SCHEMA
             and description.get("weights_sha256") == weights_sha256
             and description.get("simulation_only") is True
             and description.get("vehicle_control_approved") is False,
             "research weights metadata differs")
    raw = _pinned_bytes(weights, weights_sha256)
    with zipfile.ZipFile(BytesIO(raw)) as archive:
        entries = archive.infolist()
        _require(len(entries) == len(description["tensors"])
                 and len({entry.filename for entry in entries}) == len(entries)
                 and sum(entry.file_size for entry in entries) <= MAX_WEIGHTS_BYTES,
                 "research weights archive size or duplicate keys differ")
    with np.load(BytesIO(raw), allow_pickle=False) as arrays:
        _require(set(arrays.files) == set(description["tensors"]), "research weights keys differ")
        state = {}
        for name in arrays.files:
            value = arrays[name]
            spec = description["tensors"][name]
            _require(value.dtype == np.float32 and list(value.shape) == spec["shape"]
                     and str(value.dtype) == spec["dtype"] and np.isfinite(value).all(),
                     f"research array ABI differs: {name}")
            state[name] = torch.from_numpy(value.copy())
    return _model(description["model_config"], state)


@dataclass(frozen=True)
class ControllerConfig:
    """HH_261001 - Vehicle geometry must come from the spawned actor's physics."""

    wheelbase_m: float
    maximum_steer_angle_rad: float
    speed_cap_mps: float = 2.5
    minimum_lookahead_m: float = 1.5
    lookahead_time_s: float = 0.8
    speed_preview_s: float = 0.3
    throttle_gain: float = 0.7
    brake_gain: float = 0.7
    maximum_throttle: float = 0.5

    def validate(self):
        for name, value in asdict(self).items():
            _require(not isinstance(value, bool) and isinstance(value, (int, float))
                     and math.isfinite(value) and value > 0, f"invalid controller {name}")
        _require(self.maximum_steer_angle_rad < math.pi / 2 and self.maximum_throttle <= 1
                 and self.speed_cap_mps <= RuntimeGateConfig().maximum_speed_mps
                 and self.speed_preview_s <= 6.4, "controller bounds differ")


def trajectory_control(selected: SelectedTrajectory, current_speed_mps: float, config: ControllerConfig):
    """HH_261001 - Track x-forward/y-left geometry; CARLA positive steer turns right."""
    config.validate()
    _require(math.isfinite(current_speed_mps) and current_speed_mps >= -0.1, "invalid measured speed")
    lookahead = max(config.minimum_lookahead_m, max(0., current_speed_mps) * config.lookahead_time_s)
    points = [(x, y) for x, y in selected.xy_base_m if x > 0 and math.hypot(x, y) > 1e-6]
    target = next((p for p in points if math.hypot(*p) >= lookahead), points[-1] if points else None)
    preview = min(len(selected.speed_mps) - 1, max(0, round(config.speed_preview_s / selected.timestep_s) - 1))
    desired_speed = min(config.speed_cap_mps, selected.speed_mps[preview])
    if target is None:
        _require(max(selected.speed_mps) <= 0.1, "moving plan lacks a forward pursuit point")
        steer = 0.
    else:
        curvature = 2 * target[1] / (target[0] ** 2 + target[1] ** 2)
        steer = max(-1., min(1., -math.atan(config.wheelbase_m * curvature) / config.maximum_steer_angle_rad))
    error = desired_speed - max(0., current_speed_mps)
    throttle = min(config.maximum_throttle, max(0., config.throttle_gain * error))
    brake = min(1., max(0., -config.brake_gain * error))
    if desired_speed <= 0.05:
        throttle, brake = 0., max(0.3, brake)
    return dict(throttle=throttle, brake=brake, steer=steer, reverse=False, hand_brake=False,
                desired_speed_mps=desired_speed, pursuit_xy_base_m=target,
                steering_convention="CARLA_right_positive_from_base_link_y_left")


def _json_finite(value):
    """HH_261001 - Preserve invalid raw values explicitly without producing invalid JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return "NaN" if math.isnan(value) else ("+Infinity" if value > 0 else "-Infinity")
    if isinstance(value, dict):
        return {key: _json_finite(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_finite(item) for item in value]
    return value


class SimulationResearchPolicy:
    """HH_261001 - Propose controls only; the owning simulator logs their actual application."""

    def __init__(self, model, controller: ControllerConfig, *, deadline_s=0.1):
        controller.validate()
        _require(model.config.model_id == PHYSICAL_STOPMIX_MODEL_ID and model.config.candidate_count == 12,
                 "simulation policy requires AH STOPMIX")
        _require(getattr(model, "_research_source_checkpoint_sha256", None) == AH_CHECKPOINT_SHA256,
                 "simulation policy requires source-pinned research weights")
        _require(math.isfinite(deadline_s) and 0 < deadline_s <= 0.1, "deadline must be in (0, 0.1]")
        self.model, self.controller, self.deadline_s = model.eval(), controller, deadline_s
        self.gate = RuntimeGateConfig(candidate_count=12)

    def step(self, inputs: RuntimeInputs, *, now_ns: int, ego_stamp_ns: int,
             route_stamp_ns: int, emergency_reason: str | None = None):
        start = time.perf_counter()
        decision = dict(schema=DECISION_SCHEMA, source_checkpoint_sha256=AH_CHECKPOINT_SHA256,
                        simulation_only=True, vehicle_control_approved=False, training_data_approved=False,
                        selection_policy="raw_argmax_fail_closed", learned_control_proposed=False,
                        learned_control_applied=False, control_source="emergency_brake",
                        emergency_reason=None, raw_output=None, candidate_gates=[],
                        nonfinite_raw_encoding="NaN/+Infinity/-Infinity strings",
                        controller=asdict(self.controller), selected=None,
                        control=dict(throttle=0., brake=1., steer=0., reverse=False, hand_brake=False))
        try:
            _require(emergency_reason is None, "external emergency: " + str(emergency_reason))
            stamps = [frame.timestamp_ns for frame in inputs.camera_frames] + [ego_stamp_ns, route_stamp_ns]
            _require(type(now_ns) is int and all(type(stamp) is int and 0 <= now_ns - stamp <= 150_000_000
                                               for stamp in stamps), "stale or future sensor/route timestamp")
            _require(max(stamps) - min(stamps) <= 50_000_000, "sensor/state/route skew exceeds 50 ms")
            tensors = build_runtime_tensors(inputs, self.model.config)
            with torch.no_grad():
                xy, speeds, logits = self.model(*(tensors[name] for name in FIELDS))
            decision["raw_output"] = dict(candidate_xy=xy[0].detach().cpu().tolist(),
                                          candidate_speed=speeds[0].detach().cpu().tolist(),
                                          candidate_logits=logits[0].detach().cpu().tolist())
            _require(tuple(xy.shape) == (1, 12, 64, 2) and tuple(speeds.shape) == (1, 12, 64)
                     and tuple(logits.shape) == (1, 12), "model output ABI differs")
            raw = decision["raw_output"]
            speed = float(inputs.ego_history[-1][FEATURE_NAMES.index("velocity_x_mps")])
            # HH_261001 - Audit every candidate, but never silently rerank a rejected raw winner.
            for index in range(12):
                try:
                    validate_and_select_trajectory([raw["candidate_xy"][index]],
                        [raw["candidate_speed"][index]], [raw["candidate_logits"][index]],
                        RuntimeGateConfig(candidate_count=1), current_speed_mps=speed)
                    decision["candidate_gates"].append(dict(index=index, passed=True, reason=None))
                except ContractError as error:
                    decision["candidate_gates"].append(dict(index=index, passed=False, reason=str(error)))
            selected = validate_and_select_trajectory(raw["candidate_xy"], raw["candidate_speed"],
                                                      raw["candidate_logits"], self.gate, current_speed_mps=speed)
            decision["selected"] = asdict(selected)
            control = trajectory_control(selected, speed, self.controller)
            _require(time.perf_counter() - start <= self.deadline_s, "inference/control deadline exceeded")
            decision.update(control=control, control_source="learned_trajectory_tracker",
                            learned_control_proposed=True)
        except (ContractError, ValueError, TypeError, IndexError, FloatingPointError, RuntimeError) as error:
            decision["emergency_reason"] = str(error)
        decision["total_seconds"] = time.perf_counter() - start
        return _json_finite(decision)


def application_receipt(decision, *, acknowledged: bool, applied_source: str):
    """HH_261001 - Expert fallback and unacknowledged proposals never count as learned control."""
    _require(type(acknowledged) is bool and applied_source in
             ("learned_trajectory_tracker", "expert", "emergency_brake", "none"), "invalid application receipt")
    _require(applied_source != "learned_trajectory_tracker" or decision["learned_control_proposed"],
             "cannot apply a rejected learned proposal")
    return dict(acknowledged=acknowledged, applied_source=applied_source,
                learned_control_applied=acknowledged and applied_source == "learned_trajectory_tracker",
                vehicle_control_approved=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--export-npz", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(export_ah_weights(args.checkpoint, args.export_npz), sort_keys=True))


if __name__ == "__main__":
    main()
