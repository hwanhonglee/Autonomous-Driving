# HH_260906 - Add an auditable shadow inference runtime for the portable trajectory model.
"""Live tensorization and checkpoint inference for the portable E2E planner."""

from __future__ import annotations

from dataclasses import dataclass
from io import BytesIO
import hashlib
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Mapping, Sequence

import numpy as np
from PIL import Image
import torch
from torch import Tensor

from .contract import ContractError
from .contract import MAX_JPEG_FILE_BYTES
from .contract import _read_json_and_sha256
from .contract import _validate_rig
from .contract import load_contract
from .contract import validate_contract
from .dataset import CALIBRATION_FEATURE_NAMES, FEATURE_NAMES
from .model import ModelConfig, PerspectiveTrajectoryModel
from .runtime_contract import CAMERA_ORDER
from .runtime_contract import RuntimeGateConfig
from .runtime_contract import SelectedTrajectory
from .runtime_contract import validate_and_select_trajectory
from .runtime_contract import validate_calibration_features
from .torch_dataset import RGB_MEAN, RGB_STD, _linear_route_samples
from .train import _canonical_sha256
from .train import _configure_determinism
from .train import _nested_tensors_are_finite
from .runtime_weight_bundle import BUNDLE_ID
from .runtime_weight_bundle import _validate_torch_state_dict_abi
from .runtime_weight_bundle import load_runtime_weight_bundle


RUNTIME_ID = "portable_e2e.pytorch_shadow_runtime.v1"


@dataclass(frozen=True)
class RuntimeInputs:
    """One exact six-camera inference sample in the training ABI."""

    camera_frames: tuple["RuntimeCameraFrame", ...]
    calibration: tuple[tuple[float, ...], ...]
    ego_history: tuple[tuple[float, ...], ...]
    ego_history_mask: tuple[bool, ...]
    route_xy_base_m: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class InferenceResult:
    """Validated prediction plus timing and checkpoint provenance."""

    runtime_id: str
    runtime_bundle_sha256: str
    source_checkpoint_sha256: str
    selected: SelectedTrajectory
    forward_seconds: float
    total_seconds: float
    vehicle_control_approved: bool = False


@dataclass(frozen=True)
class RuntimeCameraFrame:
    """One source-timestamped image with an explicit source encoding."""

    timestamp_ns: int
    payload: Any
    encoding: str


@dataclass(frozen=True)
class CameraCalibrationMetadata:
    """Immutable live-comparable metadata for one camera in model order."""

    name: str
    model_index: int
    optical_frame: str
    width_px: int
    height_px: int
    intrinsic_k: tuple[float, ...]
    distortion_d: tuple[float, ...]
    rectified: bool


@dataclass(frozen=True)
class LiveCameraCalibration:
    """Immutable CameraInfo observation captured at one ROS source stamp."""

    camera_name: str
    timestamp_ns: int
    optical_frame: str
    width_px: int
    height_px: int
    distortion_model: str
    distortion_d: tuple[float, ...]
    intrinsic_k: tuple[float, ...]
    rectification_r: tuple[float, ...]
    projection_p: tuple[float, ...]
    binning_x: int
    binning_y: int
    roi_x_offset: int
    roi_y_offset: int
    roi_width: int
    roi_height: int
    roi_do_rectify: bool


@dataclass(frozen=True)
class CalibrationBundle:
    """A fully validated and hash-pinned Common10 camera rig."""

    rig_id: str
    rig_sha256: str
    features: tuple[tuple[float, ...], ...]
    cameras: tuple[CameraCalibrationMetadata, ...]


def _require_sha256(value: str, context: str) -> str:
    if not isinstance(value, str) or len(value) != 64 or any(
        character not in "0123456789abcdef" for character in value
    ):
        raise ContractError(f"{context} must be a lowercase SHA-256")
    return value


def _finite_rows(
    rows: Sequence[Sequence[float]],
    *,
    expected_rows: int,
    expected_columns: int,
    context: str,
) -> tuple[tuple[float, ...], ...]:
    if isinstance(rows, (str, bytes)) or not isinstance(rows, Sequence):
        raise ContractError(f"{context} must be a sequence")
    if len(rows) != expected_rows:
        raise ContractError(f"{context} must contain {expected_rows} rows")
    parsed: list[tuple[float, ...]] = []
    for row_index, row in enumerate(rows):
        if isinstance(row, (str, bytes)) or not isinstance(row, Sequence):
            raise ContractError(f"{context}[{row_index}] must be a sequence")
        if len(row) != expected_columns:
            raise ContractError(
                f"{context}[{row_index}] must contain {expected_columns} values"
            )
        try:
            values = tuple(float(value) for value in row)
        except (TypeError, ValueError) as error:
            raise ContractError(f"{context}[{row_index}] contains a non-number") from error
        if not all(math.isfinite(value) for value in values):
            raise ContractError(f"{context}[{row_index}] contains NaN or Inf")
        parsed.append(values)
    return tuple(parsed)


def _runtime_timestamp(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ContractError(f"{context} must be a nonnegative integer nanosecond stamp")
    return value


def _runtime_calibration_vector(
    values: Sequence[float], *, length: int, context: str
) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)) or not isinstance(values, Sequence):
        raise ContractError(f"{context} must be a sequence")
    if len(values) != length:
        raise ContractError(f"{context} must contain {length} values")
    try:
        parsed = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ContractError(f"{context} contains a non-number") from error
    if not all(math.isfinite(value) for value in parsed):
        raise ContractError(f"{context} contains NaN or Inf")
    return parsed


def _runtime_calibration_integer(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ContractError(f"{context} must be a nonnegative integer")
    return value


def _calibration_values_match(
    actual: Sequence[float], expected: Sequence[float], *, tolerance: float = 1.0e-9
) -> bool:
    return len(actual) == len(expected) and all(
        math.isclose(left, right, rel_tol=1.0e-12, abs_tol=tolerance)
        for left, right in zip(actual, expected)
    )


def validate_live_camera_calibration(
    expected: CameraCalibrationMetadata,
    observed: LiveCameraCalibration,
) -> None:
    """Reject a live CameraInfo observation that differs from the pinned rig."""
    if observed.camera_name != expected.name:
        raise ContractError(
            f"live camera calibration name must be {expected.name}, "
            f"got {observed.camera_name}"
        )
    _runtime_timestamp(
        observed.timestamp_ns, f"live camera calibration {expected.name} timestamp"
    )
    if observed.optical_frame != expected.optical_frame:
        raise ContractError(
            f"live camera calibration {expected.name} optical frame does not match the rig"
        )
    width = _runtime_calibration_integer(
        observed.width_px, f"live camera calibration {expected.name} width"
    )
    height = _runtime_calibration_integer(
        observed.height_px, f"live camera calibration {expected.name} height"
    )
    if (width, height) != (expected.width_px, expected.height_px):
        raise ContractError(
            f"live camera calibration {expected.name} resolution does not match the rig"
        )
    if (width, height) != (640, 360):
        raise ContractError(
            f"live camera calibration {expected.name} must remain exactly 640x360"
        )
    intrinsic = _runtime_calibration_vector(
        observed.intrinsic_k,
        length=9,
        context=f"live camera calibration {expected.name} K",
    )
    if not _calibration_values_match(intrinsic, expected.intrinsic_k):
        raise ContractError(
            f"live camera calibration {expected.name} K does not match the rig"
        )
    if expected.rectified is not True:
        raise ContractError(f"runtime rig camera {expected.name} must be rectified")
    if observed.distortion_model != "plumb_bob":
        raise ContractError(
            f"live camera calibration {expected.name} distortion model must be plumb_bob"
        )
    distortion = _runtime_calibration_vector(
        observed.distortion_d,
        length=5,
        context=f"live camera calibration {expected.name} D",
    )
    if any(abs(value) > 1.0e-9 for value in distortion) or any(
        abs(value) > 1.0e-9 for value in expected.distortion_d
    ):
        raise ContractError(
            f"live camera calibration {expected.name} must declare zero rectified D"
        )
    rectification = _runtime_calibration_vector(
        observed.rectification_r,
        length=9,
        context=f"live camera calibration {expected.name} R",
    )
    identity = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    if not _calibration_values_match(rectification, identity):
        raise ContractError(
            f"live camera calibration {expected.name} R must be identity"
        )
    projection = _runtime_calibration_vector(
        observed.projection_p,
        length=12,
        context=f"live camera calibration {expected.name} P",
    )
    expected_projection = (
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
    )
    if not _calibration_values_match(projection, expected_projection):
        raise ContractError(
            f"live camera calibration {expected.name} P does not match rectified K"
        )
    binning_x = _runtime_calibration_integer(
        observed.binning_x, f"live camera calibration {expected.name} binning_x"
    )
    binning_y = _runtime_calibration_integer(
        observed.binning_y, f"live camera calibration {expected.name} binning_y"
    )
    if binning_x not in (0, 1) or binning_y not in (0, 1):
        raise ContractError(
            f"live camera calibration {expected.name} must not use binning"
        )
    roi = (
        _runtime_calibration_integer(
            observed.roi_x_offset,
            f"live camera calibration {expected.name} ROI x_offset",
        ),
        _runtime_calibration_integer(
            observed.roi_y_offset,
            f"live camera calibration {expected.name} ROI y_offset",
        ),
        _runtime_calibration_integer(
            observed.roi_width, f"live camera calibration {expected.name} ROI width"
        ),
        _runtime_calibration_integer(
            observed.roi_height, f"live camera calibration {expected.name} ROI height"
        ),
    )
    if roi[:2] != (0, 0) or roi[2:] not in (
        (0, 0),
        (expected.width_px, expected.height_px),
    ):
        raise ContractError(
            f"live camera calibration {expected.name} must use the full image ROI"
        )
    if observed.roi_do_rectify is not False:
        raise ContractError(
            f"live camera calibration {expected.name} must already be rectified"
        )


def _rgb_array(frame: RuntimeCameraFrame, *, camera_name: str) -> np.ndarray:
    _runtime_timestamp(frame.timestamp_ns, f"runtime image {camera_name} timestamp")
    if frame.encoding in ("rgb8", "bgr8", "bgra8"):
        image = frame.payload
        if not isinstance(image, np.ndarray):
            raise ContractError(f"runtime image {camera_name} must be a NumPy array")
        if image.dtype != np.uint8:
            raise ContractError(f"runtime image {camera_name} must use uint8")
        expected_shape = (360, 640, 4) if frame.encoding == "bgra8" else (360, 640, 3)
        if image.shape != expected_shape:
            raise ContractError(
                f"runtime image {camera_name} must have source shape {expected_shape}"
            )
        if frame.encoding == "rgb8":
            array = image
        else:
            array = image[..., :3][..., ::-1]
    elif frame.encoding == "jpeg":
        if not isinstance(frame.payload, (bytes, bytearray, memoryview)):
            raise ContractError(
                f"runtime JPEG image {camera_name} must use a bytes-like payload"
            )
        payload = bytes(frame.payload)
        if not payload or len(payload) > MAX_JPEG_FILE_BYTES:
            raise ContractError(
                f"runtime JPEG image {camera_name} violates the bounded size gate"
            )
        try:
            with Image.open(BytesIO(payload)) as image:
                if image.format != "JPEG" or image.size != (640, 360):
                    raise ContractError(
                        f"runtime JPEG image {camera_name} must be a 640x360 JPEG"
                    )
                image.load()
                array = np.asarray(image.convert("RGB"), dtype=np.uint8)
        except ContractError:
            raise
        except (MemoryError, OSError, ValueError) as error:
            raise ContractError(
                f"cannot decode runtime JPEG image {camera_name}: {error}"
            ) from error
    else:
        raise ContractError(
            f"runtime image {camera_name} encoding must be rgb8, bgr8, bgra8, or jpeg"
        )
    return np.ascontiguousarray(array)


def load_rig_calibration(
    rig_path: Path,
    *,
    expected_sha256: str,
    contract: Mapping[str, Any] | None = None,
) -> CalibrationBundle:
    """Validate and pin one Common10 rig before creating its model features."""
    expected_digest = _require_sha256(expected_sha256, "runtime expected rig fingerprint")
    path = Path(rig_path).expanduser().absolute()
    rig, actual_digest = _read_json_and_sha256(path)
    if actual_digest != expected_digest:
        raise ContractError("runtime rig fingerprint does not match")
    validated_contract = load_contract() if contract is None else contract
    validate_contract(validated_contract)
    _validate_rig(rig, validated_contract, path)
    rows: list[tuple[float, ...]] = []
    cameras: list[CameraCalibrationMetadata] = []
    for index, camera in enumerate(rig["cameras"]):
        width = float(camera["width_px"])
        height = float(camera["height_px"])
        intrinsics = tuple(float(value) for value in camera["K"])
        distortion = tuple(float(value) for value in camera["D"])
        if len(distortion) not in (0, 5):
            raise ContractError(
                f"runtime rig camera {camera['name']} D must be empty or plumb_bob-5"
            )
        transform = tuple(float(value) for value in camera["T_base_from_camera"])
        cameras.append(
            CameraCalibrationMetadata(
                name=str(camera["name"]),
                model_index=int(camera["model_index"]),
                optical_frame=str(camera["optical_frame"]),
                width_px=int(camera["width_px"]),
                height_px=int(camera["height_px"]),
                intrinsic_k=intrinsics,
                distortion_d=distortion,
                rectified=bool(rig["rectified"]),
            )
        )
        rows.append(
            (
                intrinsics[0] / width,
                intrinsics[4] / height,
                intrinsics[2] / width,
                intrinsics[5] / height,
                *transform[:12],
            )
        )
    features = validate_calibration_features(rows)
    return CalibrationBundle(
        rig_id=str(rig["rig_id"]),
        rig_sha256=actual_digest,
        features=features,
        cameras=tuple(cameras),
    )


def build_runtime_tensors(
    inputs: RuntimeInputs,
    config: ModelConfig,
) -> dict[str, Tensor]:
    """Apply the same resize, normalization, masking, and route sampling as training."""
    config.validate()
    if len(inputs.camera_frames) != config.camera_count:
        raise ContractError(
            f"runtime images must follow {CAMERA_ORDER} and contain {config.camera_count} views"
        )
    timestamps = tuple(
        _runtime_timestamp(frame.timestamp_ns, f"runtime camera {index} timestamp")
        for index, frame in enumerate(inputs.camera_frames)
    )
    if max(timestamps) - min(timestamps) > 1_000_000:
        raise ContractError("runtime six-camera bundle exceeds the 1 ms skew gate")
    image_tensors: list[Tensor] = []
    resampling = getattr(Image, "Resampling", Image).BILINEAR
    for camera_name, frame in zip(CAMERA_ORDER, inputs.camera_frames):
        array = _rgb_array(frame, camera_name=camera_name)
        image = Image.fromarray(array, mode="RGB")
        resized = image.resize(
            (config.image_width, config.image_height), resample=resampling
        )
        resized_array = np.asarray(resized, dtype=np.float32).copy()
        tensor = torch.from_numpy(resized_array).permute(2, 0, 1).contiguous().div_(255.0)
        mean = tensor.new_tensor(RGB_MEAN).view(3, 1, 1)
        std = tensor.new_tensor(RGB_STD).view(3, 1, 1)
        image_tensors.append((tensor - mean) / std)

    calibration = validate_calibration_features(
        inputs.calibration,
        camera_count=config.camera_count,
        feature_count=len(CALIBRATION_FEATURE_NAMES),
    )
    history = _finite_rows(
        inputs.ego_history,
        expected_rows=config.ego_history_frames,
        expected_columns=len(FEATURE_NAMES),
        context="runtime ego_history",
    )
    if len(inputs.ego_history_mask) != config.ego_history_frames or any(
        not isinstance(value, bool) for value in inputs.ego_history_mask
    ):
        raise ContractError("runtime ego_history_mask has the wrong ABI")
    history_mask = tuple(inputs.ego_history_mask)
    if not history_mask[-1] or any(
        current and not following
        for current, following in zip(history_mask, history_mask[1:])
    ):
        raise ContractError("runtime ego_history_mask must be a valid suffix")

    try:
        route_points = _linear_route_samples(
            inputs.route_xy_base_m,
            limit=config.route_points,
        )
    except (TypeError, ValueError) as error:
        raise ContractError(f"runtime route is invalid: {error}") from error
    route_xy = torch.zeros(config.route_points, 2, dtype=torch.float32)
    route_mask = torch.zeros(config.route_points, dtype=torch.bool)
    route_xy[: len(route_points)] = torch.tensor(route_points, dtype=torch.float32)
    route_mask[: len(route_points)] = True
    tensors = {
        "images": torch.stack(image_tensors).unsqueeze(0),
        "calibration": torch.tensor(calibration, dtype=torch.float32).unsqueeze(0),
        "ego_history": torch.tensor(history, dtype=torch.float32).unsqueeze(0),
        "ego_history_mask": torch.tensor(history_mask, dtype=torch.bool).unsqueeze(0),
        "route_xy": route_xy.unsqueeze(0),
        "route_mask": route_mask.unsqueeze(0),
    }
    # HH_260906 - Reject finite float64 inputs that overflow during float32 tensorization.
    for name, tensor in tensors.items():
        if torch.is_floating_point(tensor) and not bool(torch.isfinite(tensor).all().item()):
            raise ContractError(f"runtime {name} is outside the finite float32 range")
    return tensors


def _select_runtime_device(requested: str) -> torch.device:
    if requested == "cpu":
        return torch.device("cpu")
    if requested != "cuda:0":
        raise ContractError("runtime device must be cpu or logical cuda:0")
    visible = tuple(
        token.strip()
        for token in os.environ.get("CUDA_VISIBLE_DEVICES", "").split(",")
        if token.strip()
    )
    if len(visible) != 1 or not visible[0].startswith("GPU-"):
        raise ContractError(
            "runtime CUDA requires one UUID-pinned CUDA_VISIBLE_DEVICES entry"
        )
    if not torch.cuda.is_available() or torch.cuda.device_count() != 1:
        raise ContractError("runtime CUDA must expose exactly one logical GPU")
    device = torch.device("cuda:0")
    _configure_determinism(device)
    return device


class PortableE2EShadowRuntime:
    """Load one pinned checkpoint and produce validated shadow trajectories."""

    def __init__(
        self,
        runtime_bundle_path: Path,
        *,
        expected_runtime_bundle_sha256: str,
        expected_source_checkpoint_sha256: str,
        expected_corpus_fingerprint_sha256: str,
        expected_model_config_sha256: str,
        device_name: str,
        gate_config: RuntimeGateConfig | None = None,
        deadline_s: float = 0.1,
        allow_unapproved_research_checkpoint: bool = False,
    ) -> None:
        if allow_unapproved_research_checkpoint is not True:
            raise ContractError(
                "runtime requires explicit unapproved research shadow mode"
            )
        expected_bundle = _require_sha256(
            expected_runtime_bundle_sha256,
            "runtime expected weight bundle fingerprint",
        )
        expected_source_checkpoint = _require_sha256(
            expected_source_checkpoint_sha256,
            "runtime expected source checkpoint fingerprint",
        )
        expected_corpus = _require_sha256(
            expected_corpus_fingerprint_sha256,
            "runtime expected corpus fingerprint",
        )
        expected_model_config = _require_sha256(
            expected_model_config_sha256,
            "runtime expected model config fingerprint",
        )
        if (
            isinstance(deadline_s, bool)
            or not isinstance(deadline_s, (int, float))
            or not math.isfinite(float(deadline_s))
            or not 0.0 < float(deadline_s) <= 0.1
        ):
            raise ContractError("runtime deadline_s must be finite and in (0, 0.1]")
        # HH_260906 - Load only the non-executable, fully pinned runtime bundle locally.
        bundle = load_runtime_weight_bundle(
            Path(runtime_bundle_path),
            expected_bundle_sha256=expected_bundle,
            expected_source_checkpoint_sha256=expected_source_checkpoint,
            expected_model_config_sha256=expected_model_config,
            expected_corpus_fingerprint_sha256=expected_corpus,
        )
        model_config = ModelConfig.from_mapping(bundle.model_config)
        model_config_sha256 = _canonical_sha256(model_config.to_dict())
        if model_config_sha256 != bundle.model_config_sha256:
            raise ContractError("runtime bundle model config fingerprint changed")
        device = _select_runtime_device(device_name)
        model = PerspectiveTrajectoryModel(model_config).to(device)
        try:
            state_dict = bundle.to_torch_state_dict(torch)
            # HH_260906 - Reject dtype coercion and shape drift before strict state loading.
            _validate_torch_state_dict_abi(
                state_dict,
                model.state_dict(),
                torch,
                context="runtime bundle model state",
            )
            model.load_state_dict(state_dict, strict=True)
        except (KeyError, RuntimeError, TypeError, ValueError) as error:
            raise ContractError(
                f"runtime bundle model tensors are incompatible: {error}"
            ) from error
        if not _nested_tensors_are_finite(model.state_dict()):
            raise ContractError("runtime checkpoint model contains NaN or Inf")
        model.requires_grad_(False)
        model.eval()
        self.runtime_bundle_path = bundle.path
        self.runtime_bundle_id = BUNDLE_ID
        self.runtime_bundle_sha256 = bundle.bundle_sha256
        self.source_checkpoint_sha256 = bundle.source_checkpoint_sha256
        self.device = device
        self.model_config = model_config
        self.model_config_sha256 = model_config_sha256
        self.corpus_fingerprint_sha256 = expected_corpus
        self.gate_config = RuntimeGateConfig() if gate_config is None else gate_config
        self.gate_config.validate()
        if (
            self.gate_config.candidate_count != model_config.candidate_count
            or self.gate_config.future_points != model_config.future_points
        ):
            raise ContractError("runtime output gate does not match the model ABI")
        self.deadline_s = float(deadline_s)
        self._model = model
        self._lock = threading.Lock()

    def infer(self, inputs: RuntimeInputs) -> InferenceResult:
        if not self._lock.acquire(blocking=False):
            raise ContractError("runtime inference is already busy")
        total_started = time.perf_counter()
        try:
            tensors = build_runtime_tensors(inputs, self.model_config)
            tensors = {name: value.to(self.device) for name, value in tensors.items()}
            if self.device.type == "cuda":
                torch.cuda.synchronize(self.device)
            forward_started = time.perf_counter()
            try:
                with torch.no_grad():
                    candidate_xy, candidate_speed, candidate_logits = self._model(
                        tensors["images"],
                        tensors["calibration"],
                        tensors["ego_history"],
                        tensors["ego_history_mask"],
                        tensors["route_xy"],
                        tensors["route_mask"],
                    )
            except (FloatingPointError, OverflowError) as error:
                # HH_260906 - Convert model arithmetic failures into fail-closed runtime status.
                raise ContractError(f"runtime model numeric failure: {error}") from error
            if self.device.type == "cuda":
                torch.cuda.synchronize(self.device)
            forward_seconds = time.perf_counter() - forward_started
            if any(
                value.shape[0] != 1 or not bool(torch.isfinite(value).all().item())
                for value in (candidate_xy, candidate_speed, candidate_logits)
            ):
                raise ContractError(
                    "runtime model output is non-finite or has a wrong batch"
                )
            selected = validate_and_select_trajectory(
                candidate_xy[0].detach().cpu().tolist(),
                candidate_speed[0].detach().cpu().tolist(),
                candidate_logits[0].detach().cpu().tolist(),
                self.gate_config,
                # HH_260906 - Bind the first planned speed to the causal current ego speed.
                current_speed_mps=float(
                    inputs.ego_history[-1][
                        FEATURE_NAMES.index("velocity_x_mps")
                    ]
                ),
            )
            total_seconds = time.perf_counter() - total_started
            if forward_seconds <= 0.0 or total_seconds < forward_seconds:
                raise ContractError("runtime inference timing is invalid")
            if total_seconds > self.deadline_s:
                raise ContractError(
                    f"runtime inference exceeded {self.deadline_s * 1000.0:.1f} ms"
                )
            return InferenceResult(
                runtime_id=RUNTIME_ID,
                runtime_bundle_sha256=self.runtime_bundle_sha256,
                source_checkpoint_sha256=self.source_checkpoint_sha256,
                selected=selected,
                forward_seconds=forward_seconds,
                total_seconds=total_seconds,
            )
        finally:
            self._lock.release()


def sha256_regular_file(path: Path) -> str:
    """Hash a regular runtime input for explicit operator pinning."""
    checkpoint = Path(path).expanduser().absolute()
    if checkpoint.is_symlink() or not checkpoint.is_file():
        raise ContractError(f"runtime input is not a regular file: {checkpoint}")
    digest = hashlib.sha256()
    with checkpoint.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()
