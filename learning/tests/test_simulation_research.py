"""HH_261001 - Exercise control direction, rejection logging and pinned data-only loading."""

from dataclasses import replace
import hashlib
import json

import numpy as np
import pytest
import torch

from portable_e2e import simulation_research as trial
from portable_e2e.contract import ContractError
from portable_e2e.model import ModelConfig, PHYSICAL_STOPMIX_MODEL_ID, PerspectiveTrajectoryModel
from portable_e2e.runtime import RuntimeCameraFrame, RuntimeInputs
from portable_e2e.runtime_contract import SelectedTrajectory


def config():
    return ModelConfig(model_id=PHYSICAL_STOPMIX_MODEL_ID, candidate_count=12,
                       route_points=8, image_width=16, image_height=16, image_grid_width=1,
                       image_grid_height=1, image_embedding=8, camera_fusion_width=8,
                       ego_embedding=8, route_embedding=8, hidden_width=16,
                       encoder_base_channels=4, maximum_step_m=1.)


def inputs():
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    state = (1., 1.) + (0.,) * 11
    return RuntimeInputs(
        camera_frames=tuple(RuntimeCameraFrame(1_000_000_000, image, "rgb8") for _ in range(6)),
        calibration=tuple(tuple(float(i) for i in range(16)) for _ in range(6)),
        ego_history=(state,) * 10, ego_history_mask=(True,) * 10,
        route_xy_base_m=((0., 0.), (10., 0.)))


def selected(side=0., speed=1.):
    return SelectedTrajectory(candidate_index=2,
                              xy_base_m=tuple(((i + 1) * 0.1, side * (i + 1) * 0.01) for i in range(64)),
                              speed_mps=(speed,) * 64, timestep_s=0.1, logit_margin=1., planar_extent_m=6.4)


def test_pursuit_carla_sign_and_real_vehicle_geometry():
    geometry = trial.ControllerConfig(wheelbase_m=2.7, maximum_steer_angle_rad=0.6)
    left = trial.trajectory_control(selected(1), 0.5, geometry)
    right = trial.trajectory_control(selected(-1), 0.5, geometry)
    assert left["steer"] < 0 < right["steer"]
    assert left["steer"] == pytest.approx(-right["steer"])
    assert trial.trajectory_control(selected(), 0.5, geometry)["steer"] == 0
    assert abs(trial.trajectory_control(selected(1), 0.5, replace(geometry, wheelbase_m=3.))["steer"]) > abs(left["steer"])


def test_speed_cap_stopping_and_exclusive_pedals():
    geometry = trial.ControllerConfig(2.7, 0.6, speed_cap_mps=2.)
    accelerate = trial.trajectory_control(selected(speed=5.), 1., geometry)
    brake = trial.trajectory_control(selected(speed=1.), 3., geometry)
    stop = trial.trajectory_control(selected(speed=0.), 0., geometry)
    assert accelerate["desired_speed_mps"] == 2.
    assert accelerate["throttle"] > 0 and accelerate["brake"] == 0
    assert brake["throttle"] == 0 and brake["brake"] > 0
    assert stop["throttle"] == 0 and stop["brake"] >= 0.3
    for geometry in (trial.ControllerConfig(0., 0.6), trial.ControllerConfig(2.7, 0.),
                     trial.ControllerConfig(2.7, 0.6, speed_cap_mps=20.)):
        with pytest.raises(ContractError):
            geometry.validate()


class FakeModel:
    _research_source_checkpoint_sha256 = trial.AH_CHECKPOINT_SHA256

    def __init__(self, invalid_winner=False, nonfinite=False):
        self.config = config()
        self.invalid_winner, self.nonfinite = invalid_winner, nonfinite
        self.calls = 0

    def eval(self):
        return self

    def __call__(self, *tensors):
        self.calls += 1
        xy = torch.zeros(1, 12, 64, 2)
        xy[:, :, :, 0] = torch.arange(1, 65) * 0.1
        speed = torch.ones(1, 12, 64)
        logits = torch.zeros(1, 12)
        logits[0, 2] = 2.
        if self.invalid_winner:
            xy[0, 2, 0, 0] = 50.
        if self.nonfinite:
            logits[0, 2] = float("nan")
        assert tensors[0].shape == (1, 6, 3, 16, 16)
        return xy, speed, logits


def step(model, monkeypatch, **kwargs):
    monkeypatch.setattr(trial.time, "perf_counter", lambda: 1.)
    policy = trial.SimulationResearchPolicy(model, trial.ControllerConfig(2.7, 0.6))
    return policy.step(inputs(), now_ns=1_000_000_000, ego_stamp_ns=1_000_000_000,
                       route_stamp_ns=1_000_000_000, **kwargs)


def test_valid_outputs_are_proposals_until_acknowledged(monkeypatch):
    result = step(FakeModel(), monkeypatch)
    assert result["learned_control_proposed"] and not result["learned_control_applied"]
    assert result["selected"]["candidate_index"] == 2
    assert len(result["raw_output"]["candidate_xy"]) == 12
    assert all(row["passed"] for row in result["candidate_gates"])
    assert not trial.application_receipt(result, acknowledged=True, applied_source="expert")["learned_control_applied"]
    assert not trial.application_receipt(result, acknowledged=False, applied_source="learned_trajectory_tracker")["learned_control_applied"]
    assert trial.application_receipt(result, acknowledged=True, applied_source="learned_trajectory_tracker")["learned_control_applied"]


def test_invalid_argmax_is_not_silently_replaced_and_raw_failure_survives(monkeypatch):
    result = step(FakeModel(invalid_winner=True), monkeypatch)
    assert not result["learned_control_proposed"] and result["control"]["brake"] == 1.
    assert result["emergency_reason"]
    assert result["raw_output"]["candidate_xy"][2][0][0] == 50.
    assert sum(row["passed"] for row in result["candidate_gates"]) == 11
    with pytest.raises(ContractError, match="rejected"):
        trial.application_receipt(result, acknowledged=True, applied_source="learned_trajectory_tracker")


def test_nonfinite_outputs_remain_strict_json_and_stop(monkeypatch):
    result = step(FakeModel(nonfinite=True), monkeypatch)
    assert not result["learned_control_proposed"]
    assert result["raw_output"]["candidate_logits"][2] == "NaN"
    json.dumps(result, allow_nan=False)


def test_stale_inputs_and_external_emergency_skip_model(monkeypatch):
    model = FakeModel()
    result = step(model, monkeypatch, emergency_reason="collision event")
    assert "collision event" in result["emergency_reason"] and model.calls == 0
    policy = trial.SimulationResearchPolicy(model, trial.ControllerConfig(2.7, 0.6))
    result = policy.step(inputs(), now_ns=1_200_000_000, ego_stamp_ns=1_200_000_000,
                         route_stamp_ns=1_200_000_000)
    assert "stale" in result["emergency_reason"] and model.calls == 0


def test_deadline_failure_retains_raw_model_evidence(monkeypatch):
    clock = iter((1., 1.2, 1.21))
    monkeypatch.setattr(trial.time, "perf_counter", lambda: next(clock))
    policy = trial.SimulationResearchPolicy(FakeModel(), trial.ControllerConfig(2.7, 0.6))
    result = policy.step(inputs(), now_ns=1_000_000_000, ego_stamp_ns=1_000_000_000,
                         route_stamp_ns=1_000_000_000)
    assert "deadline" in result["emergency_reason"] and not result["learned_control_proposed"]
    assert result["raw_output"] is not None and result["control"]["brake"] == 1.


def exported_fixture(tmp_path, monkeypatch):
    cfg = config()
    model = PerspectiveTrajectoryModel(cfg)
    checkpoint = tmp_path / "checkpoint.pt"
    checkpoint.write_bytes(b"pinned fixture")
    source_hash = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    monkeypatch.setattr(trial, "AH_CHECKPOINT_SHA256", source_hash)
    monkeypatch.setattr(trial, "_read_checkpoint_file", lambda *args:
                        (dict(checkpoint_schema=trial.CHECKPOINT_SCHEMA,
                              model_config=cfg.to_dict(), model_state_dict=model.state_dict()), source_hash))
    receipt = trial.export_ah_weights(checkpoint, tmp_path / "weights.npz")
    return model, receipt


def test_safe_npz_roundtrip_and_hash_tampering(tmp_path, monkeypatch):
    before, receipt = exported_fixture(tmp_path, monkeypatch)
    loaded = trial.load_ah_weights(receipt["weights"], receipt["metadata"],
        weights_sha256=receipt["weights_sha256"], metadata_sha256=receipt["metadata_sha256"])
    assert not loaded.training and not any(p.requires_grad for p in loaded.parameters())
    assert all(torch.equal(value, loaded.state_dict()[name]) for name, value in before.state_dict().items())
    with pytest.raises(ContractError, match="SHA256"):
        trial.load_ah_weights(receipt["weights"], receipt["metadata"],
                             weights_sha256="0" * 64, metadata_sha256="0" * 64)


def test_export_rejects_checkpoint_before_deserializing(tmp_path, monkeypatch):
    checkpoint = tmp_path / "wrong.pt"
    checkpoint.write_bytes(b"not AH")
    def forbidden(*args):
        raise AssertionError("must check expected hash before deserialization")
    monkeypatch.setattr(trial, "_read_checkpoint_file", forbidden)
    with pytest.raises(ContractError, match="SHA256"):
        trial.export_ah_weights(checkpoint, tmp_path / "weights.npz")
