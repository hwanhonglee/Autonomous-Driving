# HH_260906 - Verify live tensorization and pinned shadow inference independently of ROS.

import math
from pathlib import Path

import numpy as np
import pytest
import torch

from portable_e2e import ContractError
from portable_e2e.model import ModelConfig
from portable_e2e.runtime import CameraCalibrationMetadata
from portable_e2e.runtime import LiveCameraCalibration
from portable_e2e.runtime import PortableE2EShadowRuntime
from portable_e2e.runtime import RuntimeCameraFrame
from portable_e2e.runtime import RuntimeInputs
from portable_e2e.runtime import build_runtime_tensors
from portable_e2e.runtime import load_rig_calibration
from portable_e2e.runtime import validate_live_camera_calibration
from portable_e2e.runtime_contract import CAMERA_ORDER, RuntimeGateConfig
import portable_e2e.runtime as runtime_module


# HH_260906 - Keep the mocked speed consistent with its 0.05 m by 0.001 m path step.
FAKE_TRAJECTORY_SPEED_MPS = math.hypot(0.05, 0.001) / 0.1


def _camera_metadata(name="CAM_FRONT", model_index=0):
    intrinsic = (457.0, 0.0, 320.0, 0.0, 457.0, 180.0, 0.0, 0.0, 1.0)
    return CameraCalibrationMetadata(
        name=name,
        model_index=model_index,
        optical_frame=f"{name}/camera_optical_link",
        width_px=640,
        height_px=360,
        intrinsic_k=intrinsic,
        distortion_d=(),
        rectified=True,
    )


def _live_calibration(expected=None, **overrides):
    expected = _camera_metadata() if expected is None else expected
    intrinsic = expected.intrinsic_k
    values = {
        "camera_name": expected.name,
        "timestamp_ns": 1_000_000_000,
        "optical_frame": expected.optical_frame,
        "width_px": expected.width_px,
        "height_px": expected.height_px,
        "distortion_model": "plumb_bob",
        "distortion_d": (0.0,) * 5,
        "intrinsic_k": intrinsic,
        "rectification_r": (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        "projection_p": (
            intrinsic[0],
            intrinsic[1],
            intrinsic[2],
            0.0,
            intrinsic[3],
            intrinsic[4],
            intrinsic[5],
            0.0,
            intrinsic[6],
            intrinsic[7],
            intrinsic[8],
            0.0,
        ),
        "binning_x": 0,
        "binning_y": 0,
        "roi_x_offset": 0,
        "roi_y_offset": 0,
        "roi_width": 0,
        "roi_height": 0,
        "roi_do_rectify": False,
        **overrides,
    }
    return LiveCameraCalibration(**values)


def _small_config():
    return ModelConfig(
        route_points=8,
        image_width=16,
        image_height=16,
        image_grid_width=1,
        image_grid_height=1,
        image_embedding=8,
        camera_fusion_width=8,
        ego_embedding=8,
        route_embedding=8,
        hidden_width=16,
        encoder_base_channels=4,
        ego_history_frames=3,
        maximum_step_m=0.1,
    )


def _runtime_inputs(config):
    image = np.zeros((360, 640, 3), dtype=np.uint8)
    image[:, :, 0] = 255
    calibration = tuple(tuple(float(index) for index in range(16)) for _ in range(6))
    features_list = [0.0 for _ in range(config.ego_features)]
    features_list[1] = FAKE_TRAJECTORY_SPEED_MPS
    features = tuple(features_list)
    return RuntimeInputs(
        camera_frames=tuple(
            RuntimeCameraFrame(timestamp_ns=1_000_000_000, payload=image, encoding="rgb8")
            for _ in range(6)
        ),
        calibration=calibration,
        ego_history=(features,) * config.ego_history_frames,
        ego_history_mask=(False, True, True),
        route_xy_base_m=((0.0, 0.0), (10.0, 0.0)),
    )


def test_build_runtime_tensors_preserves_model_abi():
    config = _small_config()

    tensors = build_runtime_tensors(_runtime_inputs(config), config)

    assert tensors["images"].shape == (1, 6, 3, 16, 16)
    assert tensors["calibration"].shape == (1, 6, 16)
    assert tensors["ego_history"].shape == (1, 3, 13)
    assert tensors["ego_history_mask"].tolist() == [[False, True, True]]
    assert tensors["route_xy"].shape == (1, 8, 2)
    assert tensors["route_mask"].tolist() == [[True] * 8]
    expected_red = (1.0 - 0.485) / 0.229
    assert tensors["images"][0, 0, 0, 0, 0].item() == pytest.approx(expected_red)


def test_build_runtime_tensors_rejects_wrong_raw_shape():
    config = _small_config()
    inputs = _runtime_inputs(config)
    invalid = RuntimeInputs(
        camera_frames=(
            RuntimeCameraFrame(
                timestamp_ns=1_000_000_000,
                payload=np.zeros((24, 32, 4), dtype=np.uint8),
                encoding="rgb8",
            ),
        )
        + inputs.camera_frames[1:],
        calibration=inputs.calibration,
        ego_history=inputs.ego_history,
        ego_history_mask=inputs.ego_history_mask,
        route_xy_base_m=inputs.route_xy_base_m,
    )

    with pytest.raises(ContractError, match="source shape"):
        build_runtime_tensors(invalid, config)


def test_build_runtime_tensors_rejects_non_suffix_history_mask():
    config = _small_config()
    inputs = _runtime_inputs(config)
    invalid = RuntimeInputs(
        camera_frames=inputs.camera_frames,
        calibration=inputs.calibration,
        ego_history=inputs.ego_history,
        ego_history_mask=(True, False, True),
        route_xy_base_m=inputs.route_xy_base_m,
    )

    with pytest.raises(ContractError, match="valid suffix"):
        build_runtime_tensors(invalid, config)


@pytest.mark.parametrize("field", ("calibration", "ego_history"))
def test_build_runtime_tensors_rejects_float32_overflow(field):
    # HH_260906 - Reject extreme finite ROS values before they reach the model as infinity.
    config = _small_config()
    inputs = _runtime_inputs(config)
    calibration = [list(row) for row in inputs.calibration]
    history = [list(row) for row in inputs.ego_history]
    route = [list(row) for row in inputs.route_xy_base_m]
    if field == "calibration":
        calibration[0][0] = 1.0e308
    elif field == "ego_history":
        history[-1][2] = 1.0e308
    invalid = RuntimeInputs(
        camera_frames=inputs.camera_frames,
        calibration=tuple(tuple(row) for row in calibration),
        ego_history=tuple(tuple(row) for row in history),
        ego_history_mask=inputs.ego_history_mask,
        route_xy_base_m=tuple(tuple(row) for row in route),
    )

    with pytest.raises(ContractError, match="finite float32 range"):
        build_runtime_tensors(invalid, config)


def test_load_rig_calibration_retains_live_comparable_metadata_and_contract(
    monkeypatch, tmp_path
):
    intrinsic = [457.0, 0.0, 320.0, 0.0, 457.0, 180.0, 0.0, 0.0, 1.0]
    transform = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    rig = {
        "rig_id": "fixture-rig",
        "rectified": True,
        "cameras": [
            {
                "name": name,
                "model_index": index,
                "optical_frame": f"{name}/camera_optical_link",
                "width_px": 640,
                "height_px": 360,
                "K": intrinsic,
                "D": [],
                "T_base_from_camera": transform,
            }
            for index, name in enumerate(CAMERA_ORDER)
        ],
    }
    digest = "a" * 64
    supplied_contract = {"contract_id": "fixture"}
    validated = []
    monkeypatch.setattr(
        runtime_module, "_read_json_and_sha256", lambda path: (rig, digest)
    )
    monkeypatch.setattr(
        runtime_module, "validate_contract", lambda value: validated.append(value)
    )
    monkeypatch.setattr(runtime_module, "_validate_rig", lambda value, contract, path: None)
    monkeypatch.setattr(
        runtime_module,
        "load_contract",
        lambda: pytest.fail("explicit contract must bypass the packaged default"),
    )

    bundle = load_rig_calibration(
        tmp_path / "rig.json",
        expected_sha256=digest,
        contract=supplied_contract,
    )

    assert validated == [supplied_contract]
    assert tuple(camera.name for camera in bundle.cameras) == CAMERA_ORDER
    assert bundle.cameras[0] == _camera_metadata()
    assert bundle.rig_sha256 == digest
    assert len(bundle.features) == 6


def test_live_camera_calibration_accepts_exact_rectified_common10_metadata():
    expected = _camera_metadata()

    validate_live_camera_calibration(expected, _live_calibration(expected))
    validate_live_camera_calibration(
        expected,
        _live_calibration(
            expected,
            binning_x=1,
            binning_y=1,
            roi_width=640,
            roi_height=360,
        ),
    )


@pytest.mark.parametrize(
    ("overrides", "reason"),
    (
        ({"camera_name": "CAM_BACK"}, "name"),
        ({"optical_frame": "wrong"}, "optical frame"),
        ({"width_px": 320}, "resolution"),
        (
            {"intrinsic_k": (458.0, 0.0, 320.0, 0.0, 457.0, 180.0, 0.0, 0.0, 1.0)},
            "K does not match",
        ),
        ({"distortion_model": "equidistant"}, "distortion model"),
        ({"distortion_d": (0.1, 0.0, 0.0, 0.0, 0.0)}, "zero rectified D"),
        (
            {"rectification_r": (1.0, 0.0, 0.0, 0.0, 0.9, 0.0, 0.0, 0.0, 1.0)},
            "R must be identity",
        ),
        (
            {"projection_p": (1.0,) * 12},
            "P does not match",
        ),
        ({"binning_x": 2}, "must not use binning"),
        ({"roi_x_offset": 1}, "full image ROI"),
        ({"roi_do_rectify": True}, "already be rectified"),
    ),
)
def test_live_camera_calibration_rejects_any_live_rig_mismatch(overrides, reason):
    with pytest.raises(ContractError, match=reason):
        validate_live_camera_calibration(
            _camera_metadata(), _live_calibration(**overrides)
        )


class _FakeModel:
    def __init__(self, config):
        self.config = config
        self.loaded = False

    def to(self, device):
        self.device = device
        return self

    def load_state_dict(self, value, strict):
        assert value == {"fake": torch.tensor(1.0)}
        assert strict is True
        self.loaded = True

    def parameters(self):
        return (torch.tensor(1.0),)

    def state_dict(self):
        return {"fake": torch.tensor(1.0)}

    def requires_grad_(self, enabled):
        assert enabled is False
        return self

    def eval(self):
        return self

    def __call__(self, images, calibration, ego_history, ego_history_mask, route_xy, route_mask):
        assert self.loaded
        batch = images.shape[0]
        xy = torch.zeros(batch, 6, 64, 2)
        for index in range(64):
            xy[:, :, index, 0] = 0.05 * (index + 1)
            xy[:, :, index, 1] = 0.001 * index
        speed = torch.full((batch, 6, 64), FAKE_TRAJECTORY_SPEED_MPS)
        logits = torch.tensor([[0.0, 0.1, 0.2, 1.0, -1.0, -2.0]]).repeat(batch, 1)
        return xy, speed, logits


def _patch_runtime_bundle(
    monkeypatch,
    config,
    *,
    bundle_digest="b" * 64,
    source_checkpoint_digest="a" * 64,
    state_dtype=torch.float32,
    state_shape=(),
):
    class FakeBundle:
        path = Path("fixture.runtime.npz")
        bundle_sha256 = bundle_digest
        source_checkpoint_sha256 = source_checkpoint_digest
        model_config = config.to_dict()
        model_config_sha256 = runtime_module._canonical_sha256(config.to_dict())
        corpus_fingerprint_sha256 = "c" * 64

        @staticmethod
        def to_torch_state_dict(torch_module):
            assert torch_module is torch
            return {"fake": torch.ones(state_shape, dtype=state_dtype)}

    # HH_260906 - Exercise runtime integration without decoding executable checkpoints.
    def load_bundle(path, **pins):
        if pins["expected_bundle_sha256"] != bundle_digest:
            raise ContractError("runtime bundle fingerprint does not match")
        if pins["expected_source_checkpoint_sha256"] != source_checkpoint_digest:
            raise ContractError("runtime source checkpoint fingerprint does not match")
        assert pins["expected_model_config_sha256"] == FakeBundle.model_config_sha256
        assert pins["expected_corpus_fingerprint_sha256"] == "c" * 64
        return FakeBundle()

    monkeypatch.setattr(runtime_module, "load_runtime_weight_bundle", load_bundle)
    monkeypatch.setattr(runtime_module, "PerspectiveTrajectoryModel", _FakeModel)
    return bundle_digest, source_checkpoint_digest


@pytest.mark.parametrize("dtype", (torch.int64, torch.float64, torch.complex64))
def test_shadow_runtime_rejects_bundle_state_dtype_coercion(
    monkeypatch, tmp_path, dtype
):
    config = _small_config()
    bundle_digest, source_digest = _patch_runtime_bundle(
        monkeypatch, config, state_dtype=dtype
    )
    bundle = tmp_path / "model.runtime.npz"
    bundle.write_bytes(b"fixture")

    with pytest.raises(ContractError, match="dtype"):
        PortableE2EShadowRuntime(
            bundle,
            expected_runtime_bundle_sha256=bundle_digest,
            expected_source_checkpoint_sha256=source_digest,
            expected_corpus_fingerprint_sha256="c" * 64,
            expected_model_config_sha256=runtime_module._canonical_sha256(
                config.to_dict()
            ),
            device_name="cpu",
            allow_unapproved_research_checkpoint=True,
        )


def test_shadow_runtime_rejects_bundle_state_shape_drift(monkeypatch, tmp_path):
    config = _small_config()
    bundle_digest, source_digest = _patch_runtime_bundle(
        monkeypatch, config, state_shape=(1,)
    )
    bundle = tmp_path / "model.runtime.npz"
    bundle.write_bytes(b"fixture")

    with pytest.raises(ContractError, match="shape"):
        PortableE2EShadowRuntime(
            bundle,
            expected_runtime_bundle_sha256=bundle_digest,
            expected_source_checkpoint_sha256=source_digest,
            expected_corpus_fingerprint_sha256="c" * 64,
            expected_model_config_sha256=runtime_module._canonical_sha256(
                config.to_dict()
            ),
            device_name="cpu",
            allow_unapproved_research_checkpoint=True,
        )


def test_shadow_runtime_runs_argmax_with_pinned_checkpoint(monkeypatch, tmp_path):
    config = _small_config()
    bundle_digest, source_digest = _patch_runtime_bundle(monkeypatch, config)
    bundle = tmp_path / "model.runtime.npz"
    bundle.write_bytes(b"fixture")
    gate = RuntimeGateConfig(
        maximum_speed_mps=8.4,
        maximum_step_m=0.2,
        maximum_heading_step_rad=math.pi,
        maximum_first_point_distance_m=0.2,
    )
    runtime = PortableE2EShadowRuntime(
        bundle,
        expected_runtime_bundle_sha256=bundle_digest,
        expected_source_checkpoint_sha256=source_digest,
        expected_corpus_fingerprint_sha256="c" * 64,
        expected_model_config_sha256=runtime_module._canonical_sha256(config.to_dict()),
        device_name="cpu",
        gate_config=gate,
        allow_unapproved_research_checkpoint=True,
    )

    result = runtime.infer(_runtime_inputs(config))

    assert result.runtime_bundle_sha256 == bundle_digest
    assert result.source_checkpoint_sha256 == source_digest
    assert result.selected.candidate_index == 3
    assert result.selected.logit_margin == pytest.approx(0.8)
    assert result.forward_seconds > 0.0
    assert result.total_seconds >= result.forward_seconds


def test_shadow_runtime_rejects_checkpoint_digest_mismatch(monkeypatch, tmp_path):
    config = _small_config()
    _patch_runtime_bundle(monkeypatch, config, bundle_digest="b" * 64)
    bundle = tmp_path / "model.runtime.npz"
    bundle.write_bytes(b"fixture")

    with pytest.raises(ContractError, match="fingerprint does not match"):
        PortableE2EShadowRuntime(
            bundle,
            expected_runtime_bundle_sha256="a" * 64,
            expected_source_checkpoint_sha256="a" * 64,
            expected_corpus_fingerprint_sha256="c" * 64,
            expected_model_config_sha256=runtime_module._canonical_sha256(config.to_dict()),
            device_name="cpu",
            allow_unapproved_research_checkpoint=True,
        )


def test_shadow_runtime_requires_explicit_research_mode(monkeypatch, tmp_path):
    config = _small_config()
    bundle_digest, source_digest = _patch_runtime_bundle(monkeypatch, config)
    bundle = tmp_path / "model.runtime.npz"
    bundle.write_bytes(b"fixture")

    with pytest.raises(ContractError, match="explicit unapproved research shadow mode"):
        PortableE2EShadowRuntime(
            bundle,
            expected_runtime_bundle_sha256=bundle_digest,
            expected_source_checkpoint_sha256=source_digest,
            expected_corpus_fingerprint_sha256="c" * 64,
            expected_model_config_sha256=runtime_module._canonical_sha256(config.to_dict()),
            device_name="cpu",
        )
