"""HH_261005 - Prepare a non-actuating GOOD6000 temporal shadow interface.

This is an OFFLINE interface-preparation core, not an authorized live ROS node.
It delegates the frozen observed-frame policy, cache, decoder, gates and 100 ms
deadline without modifying them. Ten observed camera groups provide the original
five causal image slots; missing startup slots are never invented. The original
policy also computes a tracker request internally, but no request is exposed.

The existing ROS ``make_shadow_trajectory(result, odometry, anchor_ns=...)`` can
consume ``result.selected``. Its message/TF/calibration admission, native-frame
and source binding, ROS loader authority, node/launch wiring, recorded numerical
parity and sensor-to-output latency are UNIMPLEMENTED / UNVERIFIED here. This
core does not authorize ROS publication, actuator calls, learned STOP, promotion
or P3 / RTE-01 capability approval. Scalar fixtures are not model inference.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import copy
import hashlib
import math
from pathlib import Path
import stat
import threading
from types import SimpleNamespace


SCHEMA = "portable_e2e.temporal_autoware_shadow_interface_preparation.20261005.v1"
ROOT = Path(__file__).resolve().parents[2]
GOOD_CHECKPOINT_SHA256 = "43b861c30ec9971ad5828af5f2c9f7917356ac007446433cd2cf6e627f7b4e88"
GOOD_NPZ_SHA256 = "b24a6ddd151595f8ef2a463dc8307819e9c52dd43aec3af216c76e4d646d8625"
GOOD_METADATA_SHA256 = "a4b2c35cbc782378a5464ba8daa69ff97cea3ac388567ba260d4f2934381be04"
SOURCE_PINS = {
    "learning/portable_e2e/temporal_simulation_research_v2_20261001.py": "772275dd9e452c70bd9d5364bbacb32d8bf08438ce8bf3526f7ebcf3d64884dc",
    "learning/portable_e2e/temporal_route_joint_export_research_20261002.py": "3245e5ee72b6a2bca030249b3075704eb579a0331d0b0c4d6292068e7c192bab",
    "learning/portable_e2e/temporal_route_recovery_v3_20261001.py": "41494aeb6175c7e8bb054b0c099b121e1897144681a6e3591919bc0740925a25",
    "learning/portable_e2e/runtime_contract.py": "38e993278ef84b149efc90931423cd90b1562d86c9eb1585260d50e03b2ae0d3",
    "learning/portable_e2e/temporal_cached_encoder_v2.py": "55ef7371e0a1f4ec2b9cb854282e4d51b6c406b055ec5eceb8e286737f03adff",
    "learning/portable_e2e/runtime.py": "bd1f672efe5cf571744bd4c5ef7655435dc67dddadc41c23fddddb82d721a57b",
    "learning/portable_e2e/path_time_decoder_v2.py": "d80e370d52c73591df8a593a0483e225cb4cf2595e8389a91400f68f94eea7e7",
    "learning/portable_e2e/simulation_research.py": "4a66e2d3b522272822ddb43c43edeed1ebc6a2ee6c20bd0cda6ce2b3ea6c9252",
    "learning/portable_e2e/temporal_model_v2.py": "93f709c6c229e705899fcf1778f9dd253ea867e88a6f237ad3f0fb8efdbbf435",
}
SHADOW_TOPICS = frozenset((
    "/planning/portable_e2e/shadow_trajectory", "/planning/portable_e2e/shadow_path",
    "/planning/portable_e2e/status", "/planning/portable_e2e/latency_ms",
    "/planning/portable_e2e/selected_candidate",
))
# HH_261005 - Allowlisted original diagnostic fields cannot leak nested control or tracker commands.
DIAGNOSTIC_KEYS = (
    "schema", "total_seconds", "raw_nn_seconds", "cache", "raw_output",
    "decoded_output", "candidate_gates", "emergency_reason",
    "source_camera_frames", "cold_timing_retained", "prefill_count",
    "measured_current_speed_mps", "causal_navigation_support_m",
    "navigation_support_is_free_space_claim", "sensor_callback_or_actuator_ACK_included",
    "ten_hz_end_to_end_proven",
)


class ShadowInterfaceError(ValueError):
    """HH_261005 - Reject this prepared interface without proposing fallback actuation."""


def require(value, message):
    if not value:
        raise ShadowInterfaceError(message)


def require_shadow_topic(requested, resolved):
    """HH_261005 - Reject control-topic remapping, including remaps between shadow topics."""
    require(type(requested) is str and requested in SHADOW_TOPICS and resolved == requested,
            "exact fixed research shadow topic required; no remapping")
    return requested


def verify_sources(root=ROOT):
    """HH_261005 - Check bounded canonical source bytes once, before dependency imports."""
    root = Path(root)
    require(root.is_absolute() and root.resolve(strict=True) == root, "canonical source root required")
    for name, expected in SOURCE_PINS.items():
        path = root / name
        require(path.resolve(strict=True) == path, "source symlink forbidden")
        entry = path.stat()
        require(stat.S_ISREG(entry.st_mode) and 0 < entry.st_size <= 1024**2,
                "bounded regular frozen source required")
        require(hashlib.sha256(path.read_bytes()).hexdigest() == expected, "frozen shadow dependency changed: " + name)
    return dict(SOURCE_PINS)


def _bindings():
    # HH_261005 - Importing this preparation module alone does not import Torch, ROS or any runtime.
    verify_sources()
    from . import temporal_route_joint_export_research_20261002 as loader
    from .temporal_route_recovery_v3_20261001 import RouteRecoveryTemporalModelV3
    from .temporal_simulation_research_v2_20261001 import ObservedTemporalFrameV2, TemporalSimulationResearchPolicyV2
    from .runtime_contract import SelectedTrajectory, validate_calibration_features
    return SimpleNamespace(loader=loader, Model=RouteRecoveryTemporalModelV3,
        Frame=ObservedTemporalFrameV2, Policy=TemporalSimulationResearchPolicyV2,
        Selected=SelectedTrajectory, validate_calibration=validate_calibration_features)


@dataclass(frozen=True)
class ShadowResultV1:
    """HH_261005 - Keep the existing pure trajectory converter's selected-result interface."""

    status: str
    attempt_index: int
    anchor_ns: int | None
    selected: object | None
    diagnostics: dict
    schema: str = SCHEMA
    source_checkpoint_sha256: str = GOOD_CHECKPOINT_SHA256
    source_npz_sha256: str = GOOD_NPZ_SHA256
    source_metadata_sha256: str = GOOD_METADATA_SHA256
    actuator_command_emitted: bool = False
    vehicle_control_approved: bool = False
    model_promoted: bool = False
    learned_STOP_claim: bool = False
    P3_approved: bool = False
    ROS_bindings_implemented: bool = False
    runtime_numerical_parity_verified: bool = False


class TemporalAutowareShadowPreparationV1:
    """HH_261005 - One serial stream; accept only original observed frames, never arbitrary dictionaries."""

    def __init__(self, model, controller, *, episode_id, split, rig_id, source_sha256,
                 expected_calibration):
        bindings = _bindings()
        require(type(model) is bindings.Model, "exact strict-loaded current RouteRecoveryTemporalModelV3 required")
        require((getattr(model, "_research_temporal_checkpoint_sha256", None),
                 getattr(model, "_research_temporal_export_npz_sha256", None),
                 getattr(model, "_research_temporal_metadata_sha256", None)) ==
                (GOOD_CHECKPOINT_SHA256, GOOD_NPZ_SHA256, GOOD_METADATA_SHA256), "exact GOOD6000 export required")
        binding = getattr(model, "_research_recovery_owner_binding", None)
        require(isinstance(binding, dict) and binding.get("purpose") == "offline_cpu_numeric_validation_only"
                and binding.get("simulator_control_authorized") is False
                and getattr(model, "_research_recovery_simulator_control_authorized", None) is False
                and getattr(model, "_research_recovery_numeric_loader_schema", None) == bindings.loader.SCHEMA,
                "offline strict-loader admission required; no live ROS purpose is authorized")
        bindings.loader.verify_scope(model._research_temporal_export_metadata)
        bindings.loader.verify_owner(binding, model._research_temporal_export_metadata,
                                     GOOD_NPZ_SHA256, GOOD_METADATA_SHA256)
        require(len(model.state_dict()) == 101, "complete101 state ABI required")
        self._calibration = bindings.validate_calibration(expected_calibration)
        self._policy = bindings.Policy(model, controller, episode_id=episode_id, split=split,
            rig_id=rig_id, source_sha256=source_sha256, deadline_s=.1, longitudinal_pi=False)
        self._bindings = bindings
        self._history = deque(maxlen=10)
        self._lock = threading.Lock()
        self.attempt_count = self.prefill_count = self.trajectory_count = self.rejection_count = 0
        self.last_result = None

    def reset(self, reason="shadow_interface_explicit_reset"):
        require(self._lock.acquire(blocking=False), "serial shadow reset required")
        try:
            self._reset(reason)
        finally:
            self._lock.release()

    def _reset(self, reason):
        self._policy.reset(reason)
        self._history.clear()

    def _result(self, status, frame, original):
        selected = None
        if status == "TRAJECTORY":
            payload = copy.deepcopy(original["selected"])
            for key in ("xy_base_m", "speed_mps", "heading_rad"):
                if key in payload:
                    payload[key] = tuple(tuple(point) if isinstance(point, (list, tuple)) else point
                                         for point in payload[key])
            # HH_261005 - Preserve original JSON-null single-candidate margin; do not redo or relax gates.
            selected = self._bindings.Selected(**payload)
        diagnostics = {key: copy.deepcopy(original[key]) for key in DIAGNOSTIC_KEYS if key in original}
        diagnostics.update(original_policy_gate_passed=status == "TRAJECTORY",
            learned_actuator_proposal_not_exposed=True, runtime_intent="explicit_cruise_GO_not_head",
            maximum_speed_mps=2.5, original_policy_deadline_seconds=.1,
            original_policy_timer_excludes_shadow_result_attachment=True,
            actual_TF_tree_and_ROS_admission_implemented=False,
            original_pixel_state_calibration_history_route_validation_delegated=True)
        anchor = frame.timestamp_ns if type(frame) is self._bindings.Frame and type(frame.timestamp_ns) is int else None
        result = ShadowResultV1(status, self.attempt_count, anchor, selected, diagnostics)
        self.last_result = result
        return result

    def accept(self, frame, *, now_ns, ego_stamp_ns, route_stamp_ns):
        """HH_261005 - Retain real cold prefills and failures; never publish a rejected selected trajectory."""
        require(self._lock.acquire(blocking=False), "serial shadow input required")
        self.attempt_count += 1
        try:
            require(type(frame) is self._bindings.Frame, "original ObservedTemporalFrameV2 required")
            self._policy._validate_frame(frame)
            require(frame.inputs.calibration == self._calibration, "pinned camera calibration/TF-derived rig differs")
            past = tuple(self._history)
            if len(past) == 10:
                past = past[1:]
            keywords = dict(past_frames=past, now_ns=now_ns, ego_stamp_ns=ego_stamp_ns, route_stamp_ns=route_stamp_ns)
            if self._policy._prefill_count < 2:
                original = self._policy.prefill(frame, **keywords)
                self.prefill_count += 1
                status = "PREFILL"
            else:
                original = self._policy.step(frame, **keywords)
                accepted = (original.get("learned_control_proposed") is True and original.get("selected") is not None
                    and original.get("candidate_gates") == [dict(index=0, passed=True, reason=None)]
                    and type(original.get("total_seconds")) in (int, float)
                    and math.isfinite(original["total_seconds"]) and 0 <= original["total_seconds"] <= .1)
                if not accepted:
                    self.rejection_count += 1
                    self._reset("shadow_rejected_original_policy_requires_prefill")
                    return self._result("REJECTED", frame, original)
                status = "TRAJECTORY"
            self._history.append(frame)
            result = self._result(status, frame, original)
            if status == "TRAJECTORY":
                self.trajectory_count += 1
            return result
        except (ValueError, TypeError, IndexError, KeyError, AttributeError, FloatingPointError, RuntimeError) as error:
            self.rejection_count += 1
            self._reset("shadow_rejected_input_requires_prefill")
            return self._result("REJECTED", frame, dict(emergency_reason=str(error)))
        finally:
            self._lock.release()
