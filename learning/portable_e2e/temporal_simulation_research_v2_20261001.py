"""HH_261001 - Propose isolated temporal V2 cruise control with verified causal history.

This adapter is research-only. Explicit GO is a fixed cruise task, not a trained
intent prediction. Navigation provides decoder arc support, not free space or
future expert labels. Source-pinned data-only weights, real startup prefills,
the unchanged decoder/runtime gates, and the original deadline all remain
necessary; no camera-to-actuator FPS or feature approval follows from this API.
"""

from __future__ import annotations

from collections import OrderedDict
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
from .longitudinal_pi_research_20261001 import LongitudinalPIResearch
from .model import ModelConfig
from .path_time_decoder_v2 import GO, PathTimeDecoderV2
from .runtime import RuntimeInputs, build_runtime_tensors
from .runtime_contract import RuntimeGateConfig, validate_and_select_trajectory
from .simulation_research import ControllerConfig, _json_finite, trajectory_control
from .temporal_cached_encoder_v2 import (
    CachedTemporalModelV2, CameraFrameIdentityV2, TemporalCacheWindowV2)
from .temporal_dataset_v2 import TIME_OFFSETS_NS, _relative_pose
from .temporal_model_v2 import TemporalModelConfigV2


EXPORT_SCHEMA = "portable_e2e.temporal_v2_low_speed_joint_export.20261001.v1"
DETERMINISTIC_EXPORT_SCHEMA = "portable_e2e.temporal_v2_low_speed_joint_cuda_deterministic_export.20261001.v2"
PARENT_CHECKPOINT_SHA256 = "6c8bfceee5b3a2a0e332666b225769af236499baac9414294c18ccc6e9833ece"
REQUIRED_SOURCE_PINS = frozenset((
    "learning/portable_e2e/temporal_model_v2.py",
    "learning/portable_e2e/temporal_dataset_v2.py",
    "learning/portable_e2e/temporal_losses_v2.py",
    "learning/portable_e2e/path_time_decoder_v2.py",
    "tools/e2e/train_temporal_v2_low_speed_joint_20261001.py",
))
DETERMINISTIC_SOURCE_PINS = frozenset((
    "learning/portable_e2e/deterministic_linear_interpolation_v2.py",
    "tools/e2e/train_temporal_v2_low_speed_joint_cuda_deterministic_20261001.py",
))
FROZEN_DECODER_SHA256 = "d80e370d52c73591df8a593a0483e225cb4cf2595e8389a91400f68f94eea7e7"
FROZEN_TRAINER_SHA256 = "f047739b639586c5998d8d117bd6d0330940ee28c9c5eadad34ce0a57340178d"
DECISION_SCHEMA = "portable_e2e.temporal_v2_cruise_simulation_control.20261001.v1"
MAX_WEIGHTS_BYTES = 32 * 1024**2
MAX_METADATA_BYTES = 4 * 1024**2
ROOT = Path(__file__).resolve().parents[2]
CADENCE_NS, CADENCE_TOLERANCE_NS = 100_000_000, 1_000


def _require(condition, message):
    if not condition:
        raise ContractError(message)


def _sha(value):
    return isinstance(value, str) and len(value) == 64 and all(x in "0123456789abcdef" for x in value)


def _pinned(path, expected, limit):
    _require(_sha(expected), "exact caller SHA256 required")
    payload = _read_regular_file_bounded(Path(path), limit, "temporal simulation input")
    _require(hashlib.sha256(payload).hexdigest() == expected, "temporal simulation input SHA256 differs")
    return payload


def load_npz(weights, metadata, *, weights_sha256, metadata_sha256, source_root=ROOT):
    """HH_261001 - Load finite numeric tensors only; never unpickle a local checkpoint."""
    description = json.loads(_pinned(metadata, metadata_sha256, MAX_METADATA_BYTES))
    required = {"schema", "model_config", "checkpoint_sha256", "npz_sha256", "state_tensor_shapes",
                "state_tensor_dtypes", "source_sha256", "optimizer_updates", "raw_spatial_only",
                "decoder_integrated", "stop_intent_trained", "runtime_intent", "simulation_only",
                "model_promoted", "vehicle_control_approved", "parent_checkpoint_sha256",
                "parent_source_optimizer_updates", "known_total_optimizer_updates",
                "training_dataset_fingerprint_sha256", "one_episode_screening", "main_training",
                "independent_validation"}
    _require(isinstance(description, dict) and required <= set(description), "temporal export metadata fields differ")
    _require(description["schema"] in (EXPORT_SCHEMA, DETERMINISTIC_EXPORT_SCHEMA)
             and _sha(description["checkpoint_sha256"])
             and description["npz_sha256"] == weights_sha256
             and type(description["optimizer_updates"]) is int and 0 < description["optimizer_updates"] <= 3000
             and description["parent_checkpoint_sha256"] == PARENT_CHECKPOINT_SHA256
             and type(description["parent_source_optimizer_updates"]) is int
             and description["parent_source_optimizer_updates"] == 200
             and type(description["known_total_optimizer_updates"]) is int
             and description["known_total_optimizer_updates"] == 200 + description["optimizer_updates"]
             and _sha(description["training_dataset_fingerprint_sha256"])
             and description["one_episode_screening"] is True and description["main_training"] is False
             and description["independent_validation"] is False
             and description["raw_spatial_only"] is False and description["decoder_integrated"] is True
             and description["stop_intent_trained"] is False
             and description["runtime_intent"] == "explicit_cruise_GO_not_head"
             and description["simulation_only"] is True and description["model_promoted"] is False
             and description["vehicle_control_approved"] is False,
             "joint temporal export identity or research boundary differs")
    sources = description["source_sha256"]
    _require(isinstance(sources, dict) and REQUIRED_SOURCE_PINS <= set(sources),
             "joint temporal export critical source closure required")
    if description["schema"] == DETERMINISTIC_EXPORT_SCHEMA:
        revision = description.get("numerical_revision")
        _require(isinstance(revision, dict) and revision.get("kernel_id") == "fixed33_to257_align_corners_dense_gemm.20261001.v1"
                 and revision.get("strict_deterministic_algorithms") is True
                 and revision.get("tf32_matrix_multiplication") is False
                 and revision.get("finite_precision_bitwise_equivalence_claimed") is False
                 and revision.get("frozen_decoder_sha256") == FROZEN_DECODER_SHA256
                 and revision.get("frozen_training_orchestrator_sha256") == FROZEN_TRAINER_SHA256,
                 "declared deterministic numerical revision differs")
        numerical_sources = revision.get("source_sha256")
        _require(isinstance(numerical_sources, dict) and set(numerical_sources) == DETERMINISTIC_SOURCE_PINS
                 and DETERMINISTIC_SOURCE_PINS <= set(sources)
                 and all(_sha(value) and sources[name] == value for name, value in numerical_sources.items())
                 and sources["learning/portable_e2e/path_time_decoder_v2.py"] == FROZEN_DECODER_SHA256
                 and sources["tools/e2e/train_temporal_v2_low_speed_joint_20261001.py"] == FROZEN_TRAINER_SHA256,
                 "deterministic kernel/trainer source closure differs")
    else:
        _require("numerical_revision" not in description, "a numerical revision needs its explicit v2 export schema")
    root = Path(source_root).resolve(strict=True)
    for name, expected in sources.items():
        _require(isinstance(name, str) and not Path(name).is_absolute()
                 and all(part not in ("", ".", "..") for part in name.split("/")) and _sha(expected),
                 "relative regular exported source and SHA required")
        source = root / name
        _require(source.resolve(strict=True).is_relative_to(root), "exported source escapes its root")
        _pinned(source, expected, 16 * 1024**2)
    configuration = TemporalModelConfigV2.from_mapping(description["model_config"])
    model = CachedTemporalModelV2(configuration)
    expected_state = model.state_dict()
    shapes, dtypes = description["state_tensor_shapes"], description["state_tensor_dtypes"]
    _require(isinstance(shapes, dict) and isinstance(dtypes, dict)
             and set(shapes) == set(dtypes) == set(expected_state), "exported state keys differ")
    raw = _pinned(weights, weights_sha256, MAX_WEIGHTS_BYTES)
    with zipfile.ZipFile(BytesIO(raw)) as archive:
        entries = archive.infolist()
        _require(len(entries) == len(expected_state)
                 and len({entry.filename for entry in entries}) == len(entries)
                 and sum(entry.file_size for entry in entries) <= MAX_WEIGHTS_BYTES,
                 "exported NPZ size or duplicate entries differ")
    state = {}
    with np.load(BytesIO(raw), allow_pickle=False) as arrays:
        _require(set(arrays.files) == set(expected_state), "exported NPZ tensor keys differ")
        for name, target in expected_state.items():
            value = arrays[name]
            _require(value.dtype == np.float32 and np.isfinite(value).all()
                     and list(value.shape) == shapes[name] == list(target.shape)
                     and dtypes[name] == str(value.dtype) == "float32"
                     and target.dtype == torch.float32,
                     "exported NPZ tensor ABI differs: " + name)
            state[name] = torch.from_numpy(value.copy())
    model.load_state_dict(state, strict=True)
    model.requires_grad_(False)
    model.eval()
    model._research_temporal_checkpoint_sha256 = description["checkpoint_sha256"]
    model._research_temporal_export_npz_sha256 = weights_sha256
    model._research_temporal_metadata_sha256 = metadata_sha256
    model._research_temporal_export_metadata = description
    return model


@dataclass(frozen=True)
class ObservedTemporalFrameV2:
    """HH_261001 - Pair one measured sensor/state sample with its observed map pose."""

    timestamp_ns: int
    inputs: RuntimeInputs
    position_map_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    pose_timestamp_ns: int
    episode_id: str
    split: str
    rig_id: str
    source_sha256: str


def causal_route_support(route_xy, route_mask):
    """HH_261001 - Match joint training support: valid causal navigation arc, capped at 64 m."""
    _require(route_xy.ndim == 3 and route_xy.shape[-1] == 2
             and route_mask.shape == route_xy.shape[:2] and route_mask.dtype == torch.bool,
             "causal navigation shape or mask differs")
    _require(bool(torch.isfinite(route_xy).all()) and not bool((route_mask[:, 1:] & ~route_mask[:, :-1]).any())
             and bool((route_mask.sum(-1) >= 2).all()), "causal navigation finite prefix required")
    distance = torch.norm(route_xy[:, 1:] - route_xy[:, :-1], dim=-1)
    valid = route_mask[:, 1:] & route_mask[:, :-1]
    return (distance * valid.to(distance.dtype)).sum(-1).clamp(max=64.)


def measured_current_speed(tensors):
    """HH_261001 - Preserve the same base-link planar speed used by the frozen joint trainer."""
    history, valid = tensors["ego_history"], tensors["ego_history_mask"]
    _require(bool(valid[:, -1].all()), "current measured ego state is missing")
    current = torch.norm(history[:, -1, 1:3], dim=-1)
    _require(bool(torch.isfinite(current).all()) and not bool(((current < 0.) | (current > 2.5)).any()),
             "actual current speed exceeds unchanged low-speed envelope; no clipping")
    _require(bool((history[:, -1, FEATURE_NAMES.index("velocity_x_mps")] >= 0.).all()),
             "cruise research cannot hide observed reverse motion with a speed norm")
    return current


class TemporalSimulationResearchPolicyV2:
    """HH_261001 - One serial CPU causal stream with startup-only prefill and fail-closed controls."""

    def __init__(self, model, controller: ControllerConfig, *, episode_id, split, rig_id,
                 source_sha256, deadline_s=.1, longitudinal_pi=False):
        controller.validate()
        _require(isinstance(model, CachedTemporalModelV2), "strict loaded cached temporal model required")
        _require(_sha(getattr(model, "_research_temporal_checkpoint_sha256", None))
                 and _sha(getattr(model, "_research_temporal_export_npz_sha256", None))
                 and _sha(getattr(model, "_research_temporal_metadata_sha256", None)),
                 "source-pinned loaded temporal export required")
        _require(all(value.device.type == "cpu" and value.dtype == torch.float32
                     for value in model.state_dict().values()), "initial temporal live research is float32 CPU only")
        _require(controller.speed_cap_mps == 2.5 and math.isfinite(deadline_s)
                 and 0 < deadline_s <= .1 and type(longitudinal_pi) is bool,
                 "fixed temporal live speed/deadline/PI policy required")
        _require(isinstance(episode_id, str) and episode_id.strip() and isinstance(rig_id, str) and rig_id.strip()
                 and split in ("train", "val", "test") and _sha(source_sha256), "explicit live stream identities required")
        self.model, self.controller, self.deadline_s = model.eval(), controller, deadline_s
        self.identity = episode_id, split, rig_id, source_sha256
        self.decoder, self.gate = PathTimeDecoderV2(), RuntimeGateConfig(candidate_count=1)
        self.pi = LongitudinalPIResearch() if longitudinal_pi else None
        self._images = OrderedDict()
        self._last_stamp_ns = None
        self._last_prefill_ns = None
        self.prefill_records = []
        self._prefill_count = 0
        self._active = False
        self.model.reset_cache("new_live_stream_requires_prefill")

    def reset(self, reason="explicit_reset"):
        """HH_261001 - A reset loses prefill authority, not merely integral state."""
        self.model.reset_cache(reason)
        self._images.clear()
        self._last_stamp_ns = self._last_prefill_ns = None
        self._prefill_count = 0
        if self.pi is not None:
            self.pi.reset()

    def _validate_frame(self, frame):
        _require(isinstance(frame, ObservedTemporalFrameV2) and isinstance(frame.inputs, RuntimeInputs),
                 "observed temporal frame and RuntimeInputs required")
        _require(type(frame.timestamp_ns) is int and frame.timestamp_ns >= 0
                 and type(frame.pose_timestamp_ns) is int and frame.pose_timestamp_ns >= 0,
                 "observed nonnegative integer frame/pose timestamps required")
        _require((frame.episode_id, frame.split, frame.rig_id, frame.source_sha256) == self.identity,
                 "past or current frame belongs to another episode/split/rig/source")
        _require(len(frame.position_map_m) == 3 and len(frame.orientation_xyzw) == 4
                 and all(not isinstance(x, bool) and isinstance(x, (int, float)) and math.isfinite(x)
                         for x in (*frame.position_map_m, *frame.orientation_xyzw)), "finite observed pose required")
        _require(abs(sum(x*x for x in frame.orientation_xyzw) - 1.) <= 1.e-4,
                 "observed pose quaternion must be normalized")
        # HH_261001 - A historical image must never receive a later pose observation.
        _require(0 <= frame.timestamp_ns - frame.pose_timestamp_ns <= 1_000,
                 "observed pose must belong to the same causal camera/state frame")
        _require(len(frame.inputs.camera_frames) == 6 and all(
            type(camera.timestamp_ns) is int and camera.timestamp_ns == frame.timestamp_ns
            for camera in frame.inputs.camera_frames), "exact temporal six-camera source frame required")

    @staticmethod
    def _camera_digest(camera):
        digest = hashlib.sha256()
        digest.update(str(camera.encoding).encode("ascii"))
        if camera.encoding in ("rgb8", "bgr8", "bgra8"):
            channels = 4 if camera.encoding == "bgra8" else 3
            value = camera.payload
            _require(isinstance(value, np.ndarray) and value.dtype == np.uint8
                     and value.shape == (360, 640, channels), "bounded source uint8 image ABI required")
            digest.update(memoryview(np.ascontiguousarray(value)).cast("B"))
        elif camera.encoding == "jpeg":
            _require(isinstance(camera.payload, (bytes, bytearray, memoryview))
                     and 0 < len(camera.payload) <= 2 * 1024**2, "bounded source JPEG required")
            digest.update(camera.payload)
        else:
            raise ContractError("unsupported actual camera encoding")
        return digest.hexdigest()

    def build_inputs(self, frame, *, past_frames=(), now_ns, ego_stamp_ns, route_stamp_ns):
        """HH_261001 - Build only observed causal model inputs; no targets or future poses are accepted."""
        self._validate_frame(frame)
        _require(type(now_ns) is int and type(ego_stamp_ns) is int and type(route_stamp_ns) is int,
                 "integer current/ego/route stamps required")
        stamps = [frame.timestamp_ns, frame.pose_timestamp_ns, ego_stamp_ns, route_stamp_ns]
        _require(all(0 <= now_ns - stamp <= 150_000_000 for stamp in stamps)
                 and max(stamps) - min(stamps) <= 50_000_000, "stale, future or skewed current inputs")
        _require(isinstance(past_frames, (list, tuple)) and len(past_frames) <= 9,
                 "bounded actual past frame list required")
        previous = -1
        for past in past_frames:
            self._validate_frame(past)
            _require(previous < past.timestamp_ns < frame.timestamp_ns,
                     "past frames must increase strictly without duplicates or future/current copies")
            _require(past.inputs.calibration == frame.inputs.calibration,
                     "temporal camera rig calibration changed inside history")
            previous = past.timestamp_ns
        actual = tuple(past_frames) + (frame,)
        # HH_261001 - Every retained pixel identity is rehashed; mutable source arrays never earn an mtime-based hit.
        hashes = {item.timestamp_ns: tuple(self._camera_digest(camera) for camera in item.inputs.camera_frames)
                  for item in actual}
        current = build_runtime_tensors(frame.inputs, ModelConfig())
        tensors = {name: value for name, value in current.items() if name != "images"}
        groups, valid, poses, offsets, identities = [], [], [], [], []
        for offset in TIME_OFFSETS_NS:
            target = frame.timestamp_ns + offset
            found = [item for item in actual if abs(item.timestamp_ns - target) <= 3_000]
            _require(len(found) <= 1, "ambiguous real temporal slot timestamps")
            item = found[0] if found else None
            if item is None:
                groups.append(torch.zeros_like(current["images"][0]))
                valid.append(False); poses.append((0., 0., 0.)); offsets.append(offset / 1.e9); identities.append(None)
                continue
            camera_hashes = hashes[item.timestamp_ns]
            key = item.timestamp_ns, camera_hashes
            if item is frame:
                image = current["images"][0]
                self._images[key] = image.detach().clone()
            elif key in self._images:
                image = self._images[key].clone()
            else:
                image = build_runtime_tensors(item.inputs, ModelConfig())["images"][0]
                self._images[key] = image.detach().clone()
            groups.append(image)
            valid.append(True)
            poses.append(_relative_pose(frame, item))
            offsets.append((item.timestamp_ns - frame.timestamp_ns) / 1.e9)
            identities.append(CameraFrameIdentityV2(item.timestamp_ns, camera_hashes))
        self._images = OrderedDict((key, value) for key, value in self._images.items()
            if frame.timestamp_ns - 800_003_000 <= key[0] <= frame.timestamp_ns)
        while len(self._images) > 9:
            self._images.popitem(last=False)
        tensors.update(images=torch.stack(groups)[None], time_valid=torch.tensor(valid, dtype=torch.bool)[None],
                       relative_pose=torch.tensor(poses, dtype=torch.float32)[None],
                       time_offsets_s=torch.tensor(offsets, dtype=torch.float32)[None])
        window = TemporalCacheWindowV2(*self.identity, frame.timestamp_ns, tuple(identities))
        return tensors, window

    def _cadence(self, stamp):
        if self._last_stamp_ns is not None and abs(stamp - self._last_stamp_ns - CADENCE_NS) > CADENCE_TOLERANCE_NS:
            self.reset("live_camera_cadence_discontinuity")
            raise ContractError("live temporal camera-policy cadence differs from fixed 100 ms; prefill again")

    def prefill(self, frame, *, past_frames=(), now_ns, ego_stamp_ns, route_stamp_ns):
        """HH_261001 - Populate the cache from real startup inputs without proposing or applying controls."""
        _require(not self._active, "serial temporal prefill required")
        self._active = True
        started = time.perf_counter()
        try:
            self._cadence(frame.timestamp_ns)
            tensors, window = self.build_inputs(frame, past_frames=past_frames, now_ns=now_ns,
                                               ego_stamp_ns=ego_stamp_ns, route_stamp_ns=route_stamp_ns)
            with torch.no_grad():
                self.model.forward_cached(tensors, window=window)
            cache = asdict(self.model.cache_stats)
            if cache["reset_reason"] in ("model_state_changed", "episode_split_rig_or_source_changed", "clock_reversal"):
                self.reset("unexpected_prefill_scope_or_weights_change")
                raise ContractError("unexpected temporal prefill scope or model mutation")
            self._prefill_count += 1
            self._last_stamp_ns = self._last_prefill_ns = frame.timestamp_ns
            record = dict(schema=DECISION_SCHEMA + ".prefill", frame_timestamp_ns=frame.timestamp_ns,
                          total_seconds=time.perf_counter() - started, cache=cache,
                          source_camera_frames=[asdict(value) if value else None for value in window.frames],
                          prefill_count=self._prefill_count, control_proposed=False, actuator_acknowledged=False,
                          cold_timing_retained=True, inference_deadline_waived_for_control=False)
            self.prefill_records.append(record)
            return record
        except BaseException:
            self.reset("failed_live_prefill")
            raise
        finally:
            self._active = False

    def step(self, frame, *, past_frames=(), now_ns, ego_stamp_ns, route_stamp_ns, emergency_reason=None):
        """HH_261001 - Keep preprocessing, raw NN, decoder, gate and tracker inside a 100 ms deadline."""
        _require(not self._active, "serial temporal policy step required")
        self._active = True
        started = time.perf_counter()
        decision = dict(schema=DECISION_SCHEMA, source_checkpoint_sha256=self.model._research_temporal_checkpoint_sha256,
            simulation_only=True, vehicle_control_approved=False, training_data_approved=False,
            selection_policy="single_learned_temporal_path_fail_closed", runtime_intent="explicit_cruise_GO_not_head",
            stop_intent_trained=False, source_npz_sha256=self.model._research_temporal_export_npz_sha256,
            learned_control_proposed=False, learned_control_applied=False, control_source="emergency_brake",
            emergency_reason=None, raw_output=None, decoded_output=None, candidate_gates=[], selected=None,
            controller=asdict(self.controller), raw_nn_seconds=None,
            control=dict(throttle=0., brake=1., steer=0., reverse=False, hand_brake=False))
        try:
            _require(emergency_reason is None, "external emergency: " + str(emergency_reason))
            _require(self._prefill_count >= 2, "two real source-parity startup prefills required before learned control")
            self._cadence(frame.timestamp_ns)
            tensors, window = self.build_inputs(frame, past_frames=past_frames, now_ns=now_ns,
                                               ego_stamp_ns=ego_stamp_ns, route_stamp_ns=route_stamp_ns)
            nn_started = time.perf_counter()
            with torch.no_grad():
                raw = self.model.forward_cached(tensors, window=window)
                decision["raw_nn_seconds"] = time.perf_counter() - nn_started
                decision["raw_output"] = {name: value[0].cpu().tolist() for name, value in raw.items()}
                cache = asdict(self.model.cache_stats)
                decision["cache"] = cache
                _require(not cache["cold_start"] and cache["cnn_images"] <= 6
                         and cache["reset_reason"] is None,
                         "cold or reset temporal control cache requires fresh disclosed startup prefills")
                current_speed = measured_current_speed(tensors)
                support = causal_route_support(tensors["route_xy"], tensors["route_mask"])
                decoded = self.decoder(raw["curvature_knots"], raw["preferred_speed_knots"], current_speed,
                    torch.full_like(current_speed, GO, dtype=torch.long), torch.zeros_like(current_speed), support)
            decision["measured_current_speed_mps"] = float(current_speed[0])
            decision["measured_current_speed_source"] = "same_base_link_planar_norm_as_joint_training_not_actor_center_or_clipped"
            decision["legacy_base_velocity_x_mps"] = float(tensors["ego_history"][0, -1, 1])
            decision["causal_navigation_support_m"] = float(support[0])
            decision["navigation_support_is_free_space_claim"] = False
            raw_xy, raw_speed = decoded["xy"][0].cpu().tolist(), decoded["speed"][0].cpu().tolist()
            decision["decoded_output"] = dict(candidate_xy=raw_xy, candidate_speed=raw_speed, candidate_logits=[0.],
                instantaneous_speed=decoded["instantaneous_speed"][0].cpu().tolist(),
                arc_distance=decoded["arc_distance"][0].cpu().tolist(), intent=GO)
            try:
                selected = validate_and_select_trajectory(raw_xy, raw_speed, [0.], self.gate,
                                                         current_speed_mps=float(current_speed[0]))
            except ContractError as error:
                decision["candidate_gates"] = [dict(index=0, passed=False, reason=str(error))]
                raise
            decision["candidate_gates"] = [dict(index=0, passed=True, reason=None)]
            decision["selected"] = asdict(selected)
            control = trajectory_control(selected, float(current_speed[0]), self.controller)
            if self.pi is not None:
                request, audit = self.pi.update(control["desired_speed_mps"], float(current_speed[0]))
                original = dict(control)
                if request is not None:
                    control.update(request)
                decision["longitudinal_pi_research"] = dict(audit, original_normal_tracker_control=original,
                                                            proposed_normal_tracker_control=dict(control))
            _require(time.perf_counter() - started <= self.deadline_s, "temporal inference/control deadline exceeded")
            self._last_stamp_ns = frame.timestamp_ns
            decision.update(control=control, control_source="learned_trajectory_tracker", learned_control_proposed=True)
        except (ContractError, ValueError, TypeError, IndexError, FloatingPointError, RuntimeError) as error:
            decision["emergency_reason"] = str(error)
            self.reset("rejected_live_control_requires_prefill")
        finally:
            self._active = False
        decision["total_seconds"] = time.perf_counter() - started
        decision["sensor_callback_or_actuator_ACK_included"] = False
        decision["ten_hz_end_to_end_proven"] = False
        return _json_finite(decision)
