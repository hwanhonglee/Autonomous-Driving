#!/usr/bin/env python3
# HH_260906 - Publish validated portable E2E predictions on an isolated shadow-only ROS topic.

from __future__ import annotations

from dataclasses import asdict
import json
import math
from pathlib import Path
import time
from typing import Any

from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_vehicle_msgs.msg import SteeringReport
from geometry_msgs.msg import AccelWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, Path as PathMessage
import numpy as np
from portable_e2e.contract import (
    CONTRACT_ID,
    ContractError,
    _canonical_route_in_base,
    _read_json_and_sha256,
    load_contract,
)
from portable_e2e.runtime import RUNTIME_ID, PortableE2EShadowRuntime
from portable_e2e.runtime import (
    LiveCameraCalibration,
    RuntimeCameraFrame,
    RuntimeInputs,
    load_rig_calibration,
    sha256_regular_file,
    validate_live_camera_calibration,
)
from portable_e2e.runtime_contract import CAMERA_ORDER, EgoHistory, ExactCameraBundle
from portable_e2e.runtime_contract import RuntimeGateConfig, RuntimeHealth
from portable_e2e.runtime_contract import RUNTIME_CONTRACT_ID, RUNTIME_GATE_ID
from portable_e2e.runtime_contract import build_ego_features, runtime_health_reasons
from portable_e2e.runtime_contract import transform_base_trajectory_to_map
from portable_e2e.runtime_weight_bundle import BUNDLE_ID
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, Int8, String

from vad_route_logic import RoutePlan


IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
CAMERA_INFO_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
OUTPUT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
SHADOW_TOPIC_PREFIX = "/planning/portable_e2e/"
SHADOW_TRAJECTORY_TOPIC = f"{SHADOW_TOPIC_PREFIX}shadow_trajectory"
SHADOW_PATH_TOPIC = f"{SHADOW_TOPIC_PREFIX}shadow_path"
SHADOW_STATUS_TOPIC = f"{SHADOW_TOPIC_PREFIX}status"
SHADOW_LATENCY_TOPIC = f"{SHADOW_TOPIC_PREFIX}latency_ms"
SHADOW_SELECTED_CANDIDATE_TOPIC = f"{SHADOW_TOPIC_PREFIX}selected_candidate"
SHADOW_ADAPTER_ID = "autoware-e2e.portable-shadow-node.v2"
SHADOW_STATUS_SCHEMA_ID = "autoware-e2e.portable-shadow-status.v2"
SHADOW_OUTPUT_TOPICS = frozenset(
    (
        SHADOW_TRAJECTORY_TOPIC,
        SHADOW_PATH_TOPIC,
        SHADOW_STATUS_TOPIC,
        SHADOW_LATENCY_TOPIC,
        SHADOW_SELECTED_CANDIDATE_TOPIC,
    )
)
COMMON10_MANEUVER_COMMANDS = frozenset((0, 1, 2, 4, 5))
SHADOW_REJECTION_STAGES = (
    "image",
    "camera_info",
    "odometry",
    "acceleration",
    "steering",
    "inference",
)


class CausalMessageBuffer:
    """Retain a bounded timestamp index and never select future state."""

    def __init__(self, capacity: int = 64) -> None:
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity <= 0:
            raise ContractError("causal message buffer capacity must be positive")
        self._capacity = capacity
        self._messages: dict[int, Any] = {}

    def push(self, stamp_ns: int, message: Any) -> None:
        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns < 0:
            raise ContractError("causal message timestamp must be nonnegative")
        self._messages[stamp_ns] = message
        while len(self._messages) > self._capacity:
            del self._messages[min(self._messages)]

    def latest_not_after(self, anchor_ns: int) -> tuple[int, Any] | None:
        if isinstance(anchor_ns, bool) or not isinstance(anchor_ns, int) or anchor_ns < 0:
            raise ContractError("causal message anchor must be nonnegative")
        eligible = [stamp_ns for stamp_ns in self._messages if stamp_ns <= anchor_ns]
        if not eligible:
            return None
        selected_stamp_ns = max(eligible)
        return selected_stamp_ns, self._messages[selected_stamp_ns]


def require_fixed_publisher_topic(publisher: Any, expected_topic: str) -> None:
    """Reject ROS remaps that could turn a shadow publisher into an authority."""
    resolved = str(getattr(publisher, "topic_name", ""))
    if expected_topic not in SHADOW_OUTPUT_TOPICS or resolved != expected_topic:
        raise RuntimeError(
            f"shadow publisher must resolve exactly to {expected_topic}, "
            f"got {resolved or '<empty>'}"
        )


def require_sha256(value: str, context: str) -> str:
    """Validate an operator-pinned lowercase SHA-256 value."""
    if not isinstance(value, str) or len(value) != 64 or any(
        character not in "0123456789abcdef" for character in value
    ):
        raise RuntimeError(f"{context} must be a lowercase SHA-256")
    return value


def require_positive_finite(value: float, context: str) -> float:
    """Validate a positive runtime-policy value before publishing provenance."""
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise RuntimeError(f"{context} must be finite and positive")
    return parsed


def classify_shadow_failure(stage: str, error: Exception) -> str:
    """Return one stable automation code without exposing exception text as an ABI."""
    # HH_260906 - Keep failure codes stable while preserving full text only as detail.
    if stage not in SHADOW_REJECTION_STAGES:
        raise RuntimeError(f"unknown shadow rejection stage: {stage}")
    message = str(error)
    ordered_codes = (
        ("runtime health gate", "runtime_health"),
        ("CameraInfo", "camera_info_contract"),
        ("camera calibration", "camera_info_contract"),
        ("causal ", "causal_input"),
        ("must use map -> base_link", "state_frame"),
        ("must use the base_link frame", "state_frame"),
        ("inference exceeded", "inference_deadline"),
        ("trajectory", "trajectory_gate"),
        ("provenance", "provenance"),
        ("bundle", "camera_bundle"),
    )
    for marker, code in ordered_codes:
        if marker in message:
            return code
    if stage == "inference":
        return "inference_contract"
    return f"{stage}_contract"


def load_pinned_route(route_path: Path, expected_sha256: str) -> tuple[RoutePlan, str]:
    """Parse the exact stable bytes whose digest is bound to runtime provenance."""
    expected = require_sha256(expected_sha256, "route_sha256")
    # HH_260906 - Read, hash, and parse one immutable byte snapshot to close route TOCTOU.
    payload, actual_sha256 = _read_json_and_sha256(route_path, "shadow route file")
    if actual_sha256 != expected:
        raise RuntimeError("route_file SHA-256 does not match route_sha256")
    # HH_260906 - Require an explicit base_link frame for security-pinned live routes.
    if not isinstance(payload, dict) or payload.get("coordinate_reference") != "base_link":
        raise RuntimeError("pinned route must explicitly declare coordinate_reference=base_link")
    return RoutePlan.from_payload(payload, route_file=route_path), actual_sha256


def build_shadow_provenance(
    *,
    runtime_id: str,
    model_id: str,
    runtime_bundle_sha256: str,
    source_checkpoint_sha256: str,
    model_config_sha256: str,
    corpus_fingerprint_sha256: str,
    rig_id: str,
    rig_sha256: str,
    contract_sha256: str,
    route_sha256: str,
    runtime_gate: RuntimeGateConfig,
    runtime_policy: dict[str, float],
    device_name: str,
    clock_mode: str,
) -> dict[str, Any]:
    if not isinstance(runtime_id, str) or not runtime_id.strip():
        raise RuntimeError("runtime_id must be nonempty")
    if not isinstance(model_id, str) or not model_id.strip():
        raise RuntimeError("model_id must be nonempty")
    if not isinstance(rig_id, str) or not rig_id.strip():
        raise RuntimeError("rig_id must be nonempty")
    runtime_gate.validate()
    expected_policy_names = {
        "deadline_s",
        "maximum_image_age_s",
        "maximum_camera_info_age_s",
        "maximum_state_age_s",
        "maximum_sensor_skew_s",
        "maximum_history_gap_s",
        "bundle_timeout_s",
        "maneuver_lookahead_m",
        "maneuver_exit_lookahead_m",
        "route_projection_backtrack_m",
        "route_projection_forward_m",
    }
    if set(runtime_policy) != expected_policy_names:
        raise RuntimeError("runtime_policy must contain the exact health policy fields")
    # HH_260906 - Bind every effective gate and deadline to each shadow result.
    pinned_runtime_policy = {
        name: require_positive_finite(runtime_policy[name], f"runtime_policy.{name}")
        for name in sorted(expected_policy_names)
    }
    # HH_260906 - Pin execution device and ROS clock semantics with every result.
    if device_name not in ("cpu", "cuda:0"):
        raise RuntimeError("device_name must be cpu or logical cuda:0")
    if clock_mode not in ("ros_sim_time", "ros_system_time"):
        raise RuntimeError("clock_mode must identify the active ROS clock")
    return {
        "adapter_id": SHADOW_ADAPTER_ID,
        "runtime_id": runtime_id,
        "model_id": model_id,
        "runtime_bundle_id": BUNDLE_ID,
        "common10_contract_id": CONTRACT_ID,
        "runtime_contract_id": RUNTIME_CONTRACT_ID,
        "runtime_gate_id": RUNTIME_GATE_ID,
        "camera_order": CAMERA_ORDER,
        "camera_bundle_policy": "exact_source_timestamp",
        "model_future_points": 64,
        "model_timestep_s": 0.1,
        "model_output_frame": "base_link_at_anchor",
        "published_trajectory_frame": "map",
        "runtime_device": device_name,
        "clock_mode": clock_mode,
        "watchdog_clock_mode": "steady_time",
        "runtime_bundle_sha256": require_sha256(
            runtime_bundle_sha256, "runtime_bundle_sha256"
        ),
        "source_checkpoint_sha256": require_sha256(
            source_checkpoint_sha256, "source_checkpoint_sha256"
        ),
        "model_config_sha256": require_sha256(
            model_config_sha256, "model_config_sha256"
        ),
        "corpus_fingerprint_sha256": require_sha256(
            corpus_fingerprint_sha256, "corpus_fingerprint_sha256"
        ),
        "rig_id": rig_id,
        "rig_sha256": require_sha256(rig_sha256, "rig_sha256"),
        "contract_sha256": require_sha256(
            contract_sha256, "contract_sha256"
        ),
        "live_camera_info_policy": "causal_nonstale_exact_common10_rig_match",
        "tf_extrinsic_parity": "UNIMPLEMENTED_BLOCKED",
        "health_scope": "runtime_inputs_except_camera_extrinsics",
        "route_sha256": require_sha256(route_sha256, "route_sha256"),
        "runtime_gate": asdict(runtime_gate),
        "runtime_policy": pinned_runtime_policy,
    }


def validate_shadow_result_provenance(
    result: Any, provenance: dict[str, Any]
) -> None:
    if result.runtime_id != provenance["runtime_id"]:
        raise ContractError("shadow result runtime provenance changed")
    if result.runtime_bundle_sha256 != provenance["runtime_bundle_sha256"]:
        raise ContractError("shadow result runtime bundle provenance changed")
    if result.source_checkpoint_sha256 != provenance["source_checkpoint_sha256"]:
        raise ContractError("shadow result source checkpoint provenance changed")
    if result.vehicle_control_approved is not False:
        raise ContractError("shadow result must remain unapproved for vehicle control")


def stamp_to_nanoseconds(stamp: Any) -> int:
    seconds = getattr(stamp, "sec", None)
    nanoseconds = getattr(stamp, "nanosec", None)
    if (
        isinstance(seconds, bool)
        or not isinstance(seconds, int)
        or seconds < 0
        or isinstance(nanoseconds, bool)
        or not isinstance(nanoseconds, int)
        or not 0 <= nanoseconds < 1_000_000_000
    ):
        raise ContractError("ROS timestamp is invalid")
    return seconds * 1_000_000_000 + nanoseconds


def common10_command_at(
    route: RoutePlan,
    progress_m: float,
    lookahead_m: float,
    exit_lookahead_m: float,
) -> int:
    values = tuple(float(value) for value in (progress_m, lookahead_m, exit_lookahead_m))
    if not all(math.isfinite(value) for value in values):
        raise ContractError("Common10 command inputs must be finite")
    progress = max(0.0, min(route.length_m, values[0]))
    current_index = len(route.points) - 1
    for index, point in enumerate(route.points):
        if point.distance_m + 1.0e-6 >= progress:
            current_index = index
            break
    current_command = int(route.points[current_index].vad_command)
    if current_command in COMMON10_MANEUVER_COMMANDS:
        limit = progress + max(0.0, values[2])
        for point in route.points[current_index:]:
            if point.distance_m > limit:
                break
            if point.vad_command not in COMMON10_MANEUVER_COMMANDS:
                return int(point.vad_command)
        return current_command
    limit = progress + max(0.0, values[1])
    for point in route.points[current_index:]:
        if point.distance_m > limit:
            break
        if point.vad_command in COMMON10_MANEUVER_COMMANDS:
            return int(point.vad_command)
    return 3


def quaternion_yaw(orientation: Any) -> float:
    values = tuple(
        float(getattr(orientation, field)) for field in ("x", "y", "z", "w")
    )
    if not all(math.isfinite(value) for value in values):
        raise ContractError("ego quaternion contains NaN or Inf")
    norm = math.sqrt(sum(value * value for value in values))
    if not 0.999 <= norm <= 1.001:
        raise ContractError("ego quaternion is not normalized")
    x, y, z, w = values
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def image_message_to_runtime_frame(
    message: Image, *, expected_frame_id: str | None = None
) -> RuntimeCameraFrame:
    if message.width != 640 or message.height != 360:
        raise ContractError("runtime ROS image must be exactly 640x360")
    if int(message.is_bigendian) != 0:
        raise ContractError("runtime ROS image must use little-endian byte order")
    if expected_frame_id is not None and message.header.frame_id != expected_frame_id:
        raise ContractError(
            f"runtime ROS image frame must be {expected_frame_id}, got {message.header.frame_id}"
        )
    channels = {"rgb8": 3, "bgr8": 3, "bgra8": 4}.get(message.encoding)
    if channels is None:
        raise ContractError("runtime ROS image encoding is not supported")
    row_bytes = int(message.width) * channels
    step = int(message.step)
    if step < row_bytes:
        raise ContractError("runtime ROS image step is shorter than one row")
    payload = np.frombuffer(message.data, dtype=np.uint8)
    expected_bytes = step * int(message.height)
    if payload.size != expected_bytes:
        raise ContractError("runtime ROS image byte count does not match its shape")
    rows = payload.reshape(int(message.height), step)
    image = rows[:, :row_bytes].reshape(int(message.height), int(message.width), channels)
    return RuntimeCameraFrame(
        timestamp_ns=stamp_to_nanoseconds(message.header.stamp),
        payload=np.array(image, copy=True, order="C"),
        encoding=str(message.encoding),
    )


def camera_info_message_to_live_calibration(
    message: CameraInfo, *, camera_name: str
) -> LiveCameraCalibration:
    """Copy one mutable ROS CameraInfo message into the immutable runtime ABI."""
    roi = message.roi
    return LiveCameraCalibration(
        camera_name=camera_name,
        timestamp_ns=stamp_to_nanoseconds(message.header.stamp),
        optical_frame=str(message.header.frame_id),
        width_px=int(message.width),
        height_px=int(message.height),
        distortion_model=str(message.distortion_model),
        distortion_d=tuple(float(value) for value in message.d),
        intrinsic_k=tuple(float(value) for value in message.k),
        rectification_r=tuple(float(value) for value in message.r),
        projection_p=tuple(float(value) for value in message.p),
        binning_x=int(message.binning_x),
        binning_y=int(message.binning_y),
        roi_x_offset=int(roi.x_offset),
        roi_y_offset=int(roi.y_offset),
        roi_width=int(roi.width),
        roi_height=int(roi.height),
        roi_do_rectify=bool(roi.do_rectify),
    )


def make_shadow_trajectory(
    result,
    odometry: Odometry,
    *,
    anchor_ns: int,
) -> Trajectory:
    if isinstance(anchor_ns, bool) or not isinstance(anchor_ns, int) or anchor_ns < 0:
        raise ContractError("shadow trajectory anchor must be nonnegative nanoseconds")
    selected = result.selected
    if (
        len(selected.xy_base_m) != 64
        or len(selected.speed_mps) != 64
        or not math.isclose(selected.timestep_s, 0.1, rel_tol=0.0, abs_tol=1.0e-12)
    ):
        raise ContractError("shadow trajectory must use the exact Common10 64x0.1s ABI")
    pose = odometry.pose.pose
    yaw = quaternion_yaw(pose.orientation)
    # HH_260906 - Reject non-finite ego values before constructing ROS output fields.
    ego_z_m = float(pose.position.z)
    previous_speed = float(odometry.twist.twist.linear.x)
    if not math.isfinite(ego_z_m) or not math.isfinite(previous_speed):
        raise ContractError("shadow trajectory ego z and longitudinal speed must be finite")
    mapped = transform_base_trajectory_to_map(
        selected,
        ego_x_m=float(pose.position.x),
        ego_y_m=float(pose.position.y),
        ego_yaw_rad=yaw,
    )
    message = Trajectory()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        anchor_ns, 1_000_000_000
    )
    message.header.frame_id = "map"
    previous_yaw = yaw
    for index, (x_m, y_m, point_yaw, speed_mps) in enumerate(mapped, start=1):
        point = TrajectoryPoint()
        point.pose.position.x = x_m
        point.pose.position.y = y_m
        point.pose.position.z = ego_z_m
        point.pose.orientation.z = math.sin(point_yaw * 0.5)
        point.pose.orientation.w = math.cos(point_yaw * 0.5)
        point.longitudinal_velocity_mps = speed_mps
        point.acceleration_mps2 = (speed_mps - previous_speed) / selected.timestep_s
        yaw_delta = math.atan2(
            math.sin(point_yaw - previous_yaw), math.cos(point_yaw - previous_yaw)
        )
        point.heading_rate_rps = yaw_delta / selected.timestep_s
        total_nanoseconds = int(round(index * selected.timestep_s * 1.0e9))
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(
            total_nanoseconds, 1_000_000_000
        )
        message.points.append(point)
        previous_speed = speed_mps
        previous_yaw = point_yaw
    return message


def make_shadow_path(trajectory: Trajectory) -> PathMessage:
    path = PathMessage()
    path.header = trajectory.header
    for point in trajectory.points:
        pose = PoseStamped()
        pose.header = trajectory.header
        pose.pose = point.pose
        path.poses.append(pose)
    return path


class PortableE2EShadowNode(Node):
    def __init__(self) -> None:
        super().__init__("portable_e2e_shadow")
        self._declare_parameters()
        self._load_parameters()
        contract_path = self._required_path("contract_file")
        expected_contract_sha256 = require_sha256(
            self._required_text("contract_sha256"), "contract_sha256"
        )
        contract_sha256_before = sha256_regular_file(contract_path)
        if contract_sha256_before != expected_contract_sha256:
            raise RuntimeError(
                "contract_file SHA-256 does not match contract_sha256"
            )
        self._route_contract = load_contract(contract_path)
        if self._route_contract.get("_sha256") != expected_contract_sha256:
            raise RuntimeError("loaded Common10 contract SHA-256 changed")
        contract_sha256_after = sha256_regular_file(contract_path)
        if contract_sha256_after != expected_contract_sha256:
            raise RuntimeError("contract_file changed while the contract was loaded")
        route_path = self._required_path("route_file")
        expected_route_sha256 = require_sha256(
            self._required_text("route_sha256"), "route_sha256"
        )
        self._route, loaded_route_sha256 = load_pinned_route(
            route_path, expected_route_sha256
        )
        self._route_points = tuple((point.x, point.y) for point in self._route.points)
        calibration = load_rig_calibration(
            self._required_path("rig_file"),
            expected_sha256=self._required_text("rig_sha256"),
            contract=self._route_contract,
        )
        if tuple(camera.name for camera in calibration.cameras) != CAMERA_ORDER:
            raise RuntimeError("runtime calibration camera order changed")
        gate = RuntimeGateConfig(
            maximum_speed_mps=self._maximum_speed_mps,
            maximum_step_m=self._maximum_step_m,
            maximum_abs_x_m=self._maximum_abs_x_m,
            maximum_abs_y_m=self._maximum_abs_y_m,
            maximum_heading_step_rad=self._maximum_heading_step_rad,
            maximum_curvature_rad_per_m=self._maximum_curvature_rad_per_m,
            maximum_lateral_acceleration_mps2=(
                self._maximum_lateral_acceleration_mps2
            ),
            heading_minimum_step_m=self._heading_minimum_step_m,
            maximum_backward_step_m=self._maximum_backward_step_m,
            maximum_speed_disagreement_mps=self._maximum_speed_disagreement_mps,
            maximum_integrated_distance_disagreement_m=(
                self._maximum_integrated_distance_disagreement_m
            ),
            maximum_integrated_distance_disagreement_ratio=(
                self._maximum_integrated_distance_disagreement_ratio
            ),
            maximum_acceleration_mps2=self._maximum_acceleration_mps2,
            maximum_deceleration_mps2=self._maximum_deceleration_mps2,
            minimum_planar_extent_m=self._minimum_planar_extent_m,
            stationary_speed_tolerance_mps=self._stationary_speed_tolerance_mps,
            stationary_claim_speed_epsilon_mps=(
                self._stationary_claim_speed_epsilon_mps
            ),
            maximum_stationary_radius_m=self._maximum_stationary_radius_m,
            maximum_stationary_extent_m=self._maximum_stationary_extent_m,
            minimum_low_speed_progress_ratio=(
                self._minimum_low_speed_progress_ratio
            ),
            maximum_first_point_distance_m=self._maximum_first_point_distance_m,
            minimum_first_point_x_m=self._minimum_first_point_x_m,
        )
        # HH_260906 - Reconstruct weights only from the pinned non-executable bundle.
        self._runtime = PortableE2EShadowRuntime(
            self._required_path("runtime_bundle_file"),
            expected_runtime_bundle_sha256=self._required_text(
                "runtime_bundle_sha256"
            ),
            expected_source_checkpoint_sha256=self._required_text(
                "source_checkpoint_sha256"
            ),
            expected_corpus_fingerprint_sha256=self._required_text(
                "corpus_fingerprint_sha256"
            ),
            expected_model_config_sha256=self._required_text(
                "model_config_sha256"
            ),
            device_name=self._device,
            gate_config=gate,
            deadline_s=self._deadline_s,
            allow_unapproved_research_checkpoint=self._research_acknowledged,
        )
        self._calibration = calibration.features
        self._camera_calibration = {
            camera.name: camera for camera in calibration.cameras
        }
        self._bundle = ExactCameraBundle()
        self._last_complete_bundle_wall_ns: int | None = None
        self._camera_info_buffers = {
            camera: CausalMessageBuffer() for camera in CAMERA_ORDER
        }
        self._history = EgoHistory(
            frames=self._runtime.model_config.ego_history_frames,
            maximum_gap_s=self._maximum_history_gap_s,
        )
        self._odometry_buffer = CausalMessageBuffer()
        self._acceleration_buffer = CausalMessageBuffer()
        self._steering_buffer = CausalMessageBuffer()
        self._progress_m = 0.0
        self._last_processed_anchor_ns = -1
        # HH_260906 - Separate anchor attempts from malformed asynchronous input events.
        self._anchor_attempt_count = 0
        self._accepted_count = 0
        self._anchor_rejected_count = 0
        self._input_event_rejected_count = 0
        self._rejection_counts_by_stage = {
            stage: 0 for stage in SHADOW_REJECTION_STAGES
        }
        self._provenance = build_shadow_provenance(
            runtime_id=RUNTIME_ID,
            model_id=self._runtime.model_config.model_id,
            runtime_bundle_sha256=self._runtime.runtime_bundle_sha256,
            source_checkpoint_sha256=self._runtime.source_checkpoint_sha256,
            model_config_sha256=self._runtime.model_config_sha256,
            corpus_fingerprint_sha256=self._runtime.corpus_fingerprint_sha256,
            rig_id=calibration.rig_id,
            rig_sha256=calibration.rig_sha256,
            contract_sha256=contract_sha256_after,
            route_sha256=loaded_route_sha256,
            runtime_gate=gate,
            runtime_policy={
                "deadline_s": self._deadline_s,
                "maximum_image_age_s": self._maximum_image_age_s,
                "maximum_camera_info_age_s": self._maximum_camera_info_age_s,
                "maximum_state_age_s": self._maximum_state_age_s,
                "maximum_sensor_skew_s": self._maximum_sensor_skew_s,
                "maximum_history_gap_s": self._maximum_history_gap_s,
                "bundle_timeout_s": self._bundle_timeout_s,
                "maneuver_lookahead_m": self._maneuver_lookahead_m,
                "maneuver_exit_lookahead_m": self._maneuver_exit_lookahead_m,
                "route_projection_backtrack_m": self._route_projection_backtrack_m,
                "route_projection_forward_m": self._route_projection_forward_m,
            },
            device_name=str(self._runtime.device),
            clock_mode=self._clock_mode,
        )
        self._last_status_state = "SHADOW_WAITING"
        self._last_status_anchor_ns = None
        self._last_status_detail: dict[str, Any] = {
            "reason": "waiting_for_exact_bundle_and_camera_info",
            "failure_code": "waiting_for_inputs",
            "tf_extrinsic_parity": "UNIMPLEMENTED_BLOCKED",
        }

        self._trajectory_publisher = self._create_fixed_publisher(
            Trajectory, SHADOW_TRAJECTORY_TOPIC, OUTPUT_QOS
        )
        self._path_publisher = self._create_fixed_publisher(
            PathMessage, SHADOW_PATH_TOPIC, OUTPUT_QOS
        )
        self._status_publisher = self._create_fixed_publisher(
            String, SHADOW_STATUS_TOPIC, OUTPUT_QOS
        )
        self._latency_publisher = self._create_fixed_publisher(
            Float32, SHADOW_LATENCY_TOPIC, OUTPUT_QOS
        )
        self._candidate_publisher = self._create_fixed_publisher(
            Int8, SHADOW_SELECTED_CANDIDATE_TOPIC, OUTPUT_QOS
        )
        for camera in CAMERA_ORDER:
            self.create_subscription(
                Image,
                f"/sensing/camera/{camera}/image_raw",
                self._image_callback(camera),
                IMAGE_QOS,
            )
            self.create_subscription(
                CameraInfo,
                f"/sensing/camera/{camera}/camera_info",
                self._camera_info_callback(camera),
                CAMERA_INFO_QOS,
            )
        self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self._on_odometry,
            STATE_QOS,
        )
        self.create_subscription(
            AccelWithCovarianceStamped,
            "/localization/acceleration",
            self._on_acceleration,
            STATE_QOS,
        )
        self.create_subscription(
            SteeringReport,
            "/vehicle/status/steering_status",
            self._on_steering,
            STATE_QOS,
        )
        # HH_260906 - Keep inference and health watchdogs alive when simulation time freezes.
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.01, self._on_timer, clock=self._steady_clock)
        self.create_timer(1.0, self._on_status_heartbeat, clock=self._steady_clock)
        self._publish_status(
            self._last_status_state,
            anchor_ns=None,
            detail=self._last_status_detail,
        )
        self.get_logger().warning(
            "Portable E2E is running in shadow-only mode; no canonical trajectory "
            "or control topic is published"
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "contract_file": "",
            "contract_sha256": "",
            "runtime_bundle_file": "",
            "runtime_bundle_sha256": "",
            "source_checkpoint_sha256": "",
            "corpus_fingerprint_sha256": "",
            "model_config_sha256": "",
            "rig_file": "",
            "rig_sha256": "",
            "route_file": "",
            "route_sha256": "",
            "device": "cpu",
            "research_acknowledged": False,
            "deadline_s": 0.1,
            "maximum_image_age_s": 0.25,
            "maximum_camera_info_age_s": 0.25,
            "maximum_state_age_s": 0.1,
            "maximum_sensor_skew_s": 0.1,
            "maximum_history_gap_s": 0.15,
            "bundle_timeout_s": 0.3,
            "maneuver_lookahead_m": 2.0,
            "maneuver_exit_lookahead_m": 2.5,
            "route_projection_backtrack_m": 3.0,
            "route_projection_forward_m": 80.0,
            "maximum_speed_mps": 8.333333333333334,
            "maximum_step_m": 1.0,
            "maximum_abs_x_m": 60.0,
            "maximum_abs_y_m": 60.0,
            "maximum_heading_step_rad": 0.75,
            "maximum_curvature_rad_per_m": 0.5,
            "maximum_lateral_acceleration_mps2": 3.0,
            "heading_minimum_step_m": 0.01,
            "maximum_backward_step_m": 0.002,
            "maximum_speed_disagreement_mps": 0.25,
            "maximum_integrated_distance_disagreement_m": 0.02,
            "maximum_integrated_distance_disagreement_ratio": 0.05,
            "maximum_acceleration_mps2": 3.0,
            "maximum_deceleration_mps2": 6.0,
            "minimum_planar_extent_m": 0.05,
            "stationary_speed_tolerance_mps": 0.1,
            "stationary_claim_speed_epsilon_mps": 1.0e-4,
            "maximum_stationary_radius_m": 0.05,
            "maximum_stationary_extent_m": 0.15,
            "minimum_low_speed_progress_ratio": 0.5,
            "maximum_first_point_distance_m": 1.0,
            "minimum_first_point_x_m": -0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        self._device = str(self.get_parameter("device").value)
        # HH_260906 - Record whether health timestamps use simulation or system time.
        self._clock_mode = (
            "ros_sim_time"
            if bool(self.get_parameter("use_sim_time").value)
            else "ros_system_time"
        )
        self._research_acknowledged = bool(
            self.get_parameter("research_acknowledged").value
        )
        numeric_names = (
            "deadline_s",
            "maximum_image_age_s",
            "maximum_camera_info_age_s",
            "maximum_state_age_s",
            "maximum_sensor_skew_s",
            "maximum_history_gap_s",
            "bundle_timeout_s",
            "maneuver_lookahead_m",
            "maneuver_exit_lookahead_m",
            "route_projection_backtrack_m",
            "route_projection_forward_m",
            "maximum_speed_mps",
            "maximum_step_m",
            "maximum_abs_x_m",
            "maximum_abs_y_m",
            "maximum_heading_step_rad",
            "maximum_curvature_rad_per_m",
            "maximum_lateral_acceleration_mps2",
            "heading_minimum_step_m",
            "maximum_backward_step_m",
            "maximum_speed_disagreement_mps",
            "maximum_integrated_distance_disagreement_m",
            "maximum_integrated_distance_disagreement_ratio",
            "maximum_acceleration_mps2",
            "maximum_deceleration_mps2",
            "minimum_planar_extent_m",
            "stationary_speed_tolerance_mps",
            "stationary_claim_speed_epsilon_mps",
            "maximum_stationary_radius_m",
            "maximum_stationary_extent_m",
            "minimum_low_speed_progress_ratio",
            "maximum_first_point_distance_m",
            "minimum_first_point_x_m",
        )
        for name in numeric_names:
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value):
                raise RuntimeError(f"{name} must be finite")
            setattr(self, f"_{name}", value)
        positive_names = set(numeric_names) - {"minimum_first_point_x_m"}
        if any(getattr(self, f"_{name}") <= 0.0 for name in positive_names):
            raise RuntimeError("shadow numeric safety parameters must be positive")

    def _required_text(self, name: str) -> str:
        value = str(self.get_parameter(name).value)
        if not value:
            raise RuntimeError(f"{name} is required")
        return value

    def _required_path(self, name: str) -> Path:
        path = Path(self._required_text(name)).expanduser().absolute()
        if path.is_symlink() or not path.is_file():
            raise RuntimeError(f"{name} must be a regular file")
        return path

    def _create_fixed_publisher(self, message_type, topic: str, qos):
        publisher = self.create_publisher(message_type, topic, qos)
        try:
            require_fixed_publisher_topic(publisher, topic)
        except RuntimeError:
            self.destroy_publisher(publisher)
            raise
        return publisher

    def _image_callback(self, camera: str):
        def callback(message: Image) -> None:
            try:
                frame = image_message_to_runtime_frame(
                    message,
                    expected_frame_id=self._camera_calibration[camera].optical_frame,
                )
                self._bundle.push(camera, frame.timestamp_ns, frame)
            except (ContractError, ValueError) as error:
                self._reject("image", error)

        return callback

    def _camera_info_callback(self, camera: str):
        def callback(message: CameraInfo) -> None:
            try:
                observed = camera_info_message_to_live_calibration(
                    message, camera_name=camera
                )
                self._camera_info_buffers[camera].push(
                    observed.timestamp_ns, observed
                )
                validate_live_camera_calibration(
                    self._camera_calibration[camera], observed
                )
            except (ContractError, TypeError, ValueError) as error:
                self._reject("camera_info", error, source=camera)

        return callback

    def _on_odometry(self, message: Odometry) -> None:
        try:
            self._odometry_buffer.push(
                stamp_to_nanoseconds(message.header.stamp), message
            )
        except ContractError as error:
            self._reject("odometry", error)

    def _on_acceleration(self, message: AccelWithCovarianceStamped) -> None:
        try:
            self._acceleration_buffer.push(
                stamp_to_nanoseconds(message.header.stamp), message
            )
        except ContractError as error:
            self._reject("acceleration", error)

    def _on_steering(self, message: SteeringReport) -> None:
        try:
            self._steering_buffer.push(stamp_to_nanoseconds(message.stamp), message)
        except ContractError as error:
            self._reject("steering", error)

    def _on_timer(self) -> None:
        complete = self._bundle.pop_latest()
        if complete is None:
            self._publish_bundle_timeout_if_needed()
            return
        anchor_ns, frames = complete
        if anchor_ns <= self._last_processed_anchor_ns:
            return
        self._last_processed_anchor_ns = anchor_ns
        self._last_complete_bundle_wall_ns = time.monotonic_ns()
        # HH_260906 - Count each fresh exact six-camera bundle exactly once.
        self._anchor_attempt_count += 1
        try:
            self._run_shadow(anchor_ns, frames)
        # HH_260906 - Keep any arithmetic failure inside the shadow rejection boundary.
        except (ArithmeticError, ContractError, RuntimeError, ValueError) as error:
            self._reject("inference", error, anchor_ns=anchor_ns)

    def _run_shadow(self, anchor_ns: int, frames: tuple[Any, ...]) -> None:
        camera_info_stamps = self._validated_camera_info(anchor_ns)
        odometry_ns, odometry = self._causal_state(
            self._odometry_buffer, anchor_ns, "odometry"
        )
        acceleration_ns, acceleration = self._causal_state(
            self._acceleration_buffer, anchor_ns, "acceleration"
        )
        steering_ns, steering = self._causal_state(
            self._steering_buffer, anchor_ns, "steering"
        )
        if odometry.header.frame_id != "map" or odometry.child_frame_id != "base_link":
            raise ContractError("odometry must use map -> base_link frames")
        if acceleration.header.frame_id != "base_link":
            raise ContractError("acceleration must use the base_link frame")
        now_ns = self.get_clock().now().nanoseconds
        reasons = runtime_health_reasons(
            RuntimeHealth(
                now_ns=now_ns,
                image_stamp_ns=anchor_ns,
                odometry_stamp_ns=odometry_ns,
                route_ready=True,
                calibration_ready=True,
            ),
            maximum_image_age_s=self._maximum_image_age_s,
            maximum_odometry_age_s=self._maximum_state_age_s,
            maximum_sensor_skew_s=self._maximum_sensor_skew_s,
        )
        if reasons:
            raise ContractError("runtime health gate: " + ",".join(reasons))
        pose = odometry.pose.pose
        projection = self._route.project(
            float(pose.position.x),
            float(pose.position.y),
            self._progress_m,
            self._route_projection_backtrack_m,
            self._route_projection_forward_m,
        )
        self._progress_m = projection.progress_m
        command = common10_command_at(
            self._route,
            self._progress_m,
            self._maneuver_lookahead_m,
            self._maneuver_exit_lookahead_m,
        )
        ego_features = build_ego_features(
            velocity_x_mps=float(odometry.twist.twist.linear.x),
            velocity_y_mps=float(odometry.twist.twist.linear.y),
            acceleration_x_mps2=float(acceleration.accel.accel.linear.x),
            acceleration_y_mps2=float(acceleration.accel.accel.linear.y),
            yaw_rate_radps=float(odometry.twist.twist.angular.z),
            steering_tire_angle_rad=float(steering.steering_tire_angle),
            command=command,
        )
        self._history.append(anchor_ns, ego_features)
        history, history_mask = self._history.padded()
        orientation = pose.orientation
        route_base, _goal_base, _anchor_arc = _canonical_route_in_base(
            self._route_points,
            position_m=(pose.position.x, pose.position.y, pose.position.z),
            orientation_xyzw=(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
            contract=self._route_contract,
            context="portable_e2e_shadow.route",
        )
        result = self._runtime.infer(
            RuntimeInputs(
                camera_frames=tuple(frames),
                calibration=self._calibration,
                ego_history=history,
                ego_history_mask=history_mask,
                route_xy_base_m=route_base,
            )
        )
        validate_shadow_result_provenance(result, self._provenance)
        trajectory = make_shadow_trajectory(
            result,
            odometry,
            anchor_ns=anchor_ns,
        )
        self._trajectory_publisher.publish(trajectory)
        self._path_publisher.publish(make_shadow_path(trajectory))
        self._latency_publisher.publish(
            Float32(data=float(result.total_seconds * 1000.0))
        )
        self._candidate_publisher.publish(
            Int8(data=int(result.selected.candidate_index))
        )
        self._accepted_count += 1
        self._publish_status(
            "SHADOW_OK",
            anchor_ns=anchor_ns,
            detail={
                "stage": "inference",
                "failure_code": "none",
                "candidate": result.selected.candidate_index,
                "latency_ms": result.total_seconds * 1000.0,
                "logit_margin": result.selected.logit_margin,
                "odometry_timestamp_ns": odometry_ns,
                "acceleration_timestamp_ns": acceleration_ns,
                "steering_timestamp_ns": steering_ns,
                "camera_info_timestamp_ns": camera_info_stamps,
                "tf_extrinsic_parity": "UNIMPLEMENTED_BLOCKED",
                "trajectory_point_count": len(trajectory.points),
            },
        )

    def _validated_camera_info(self, anchor_ns: int) -> dict[str, int]:
        stamps: dict[str, int] = {}
        for camera in CAMERA_ORDER:
            selected = self._camera_info_buffers[camera].latest_not_after(anchor_ns)
            if selected is None:
                raise ContractError(f"causal CameraInfo {camera} is missing")
            stamp_ns, observed = selected
            age_s = (anchor_ns - stamp_ns) * 1.0e-9
            if age_s > self._maximum_camera_info_age_s:
                raise ContractError(f"causal CameraInfo {camera} is stale")
            validate_live_camera_calibration(
                self._camera_calibration[camera], observed
            )
            stamps[camera] = stamp_ns
        return stamps

    def _causal_state(
        self,
        buffer: CausalMessageBuffer,
        anchor_ns: int,
        name: str,
    ) -> tuple[int, Any]:
        selected = buffer.latest_not_after(anchor_ns)
        if selected is None:
            raise ContractError(f"causal {name} is missing")
        state_ns, message = selected
        age_s = (anchor_ns - state_ns) * 1.0e-9
        if age_s > self._maximum_state_age_s:
            raise ContractError(f"causal {name} is stale")
        return state_ns, message

    def _on_status_heartbeat(self) -> None:
        if self._publish_bundle_timeout_if_needed():
            return
        self._publish_status(
            self._last_status_state,
            anchor_ns=self._last_status_anchor_ns,
            detail=self._last_status_detail,
        )

    def _publish_bundle_timeout_if_needed(self) -> bool:
        if self._last_complete_bundle_wall_ns is None:
            return False
        elapsed_s = (
            time.monotonic_ns() - self._last_complete_bundle_wall_ns
        ) * 1.0e-9
        if elapsed_s <= self._bundle_timeout_s:
            return False
        if (
            self._last_status_state == "SHADOW_WAITING"
            and self._last_status_detail.get("failure_code")
            == "camera_bundle_timeout"
        ):
            return False
        # HH_260906 - Revoke stale SHADOW_OK when exact camera bundles stop arriving.
        self._publish_status(
            "SHADOW_WAITING",
            anchor_ns=self._last_status_anchor_ns,
            detail={
                "stage": "camera_bundle",
                "failure_code": "camera_bundle_timeout",
                "reason": "no exact camera bundle arrived within the wall-time limit",
            },
        )
        return True

    def _reject(
        self,
        stage: str,
        error: Exception,
        *,
        anchor_ns: int | None = None,
        source: str | None = None,
    ) -> None:
        failure_code = classify_shadow_failure(stage, error)
        # HH_260906 - Keep asynchronous input rejects outside the anchor denominator.
        self._rejection_counts_by_stage[stage] += 1
        if stage == "inference":
            self._anchor_rejected_count += 1
        else:
            self._input_event_rejected_count += 1
        detail = {
            "stage": stage,
            "failure_code": failure_code,
            "reason": str(error),
        }
        if source is not None:
            detail["source"] = source
        self._publish_status(
            "SHADOW_REJECTED",
            anchor_ns=anchor_ns,
            detail=detail,
        )
        self.get_logger().warning(f"Portable E2E shadow rejected {stage}: {error}")

    def _publish_status(
        self,
        state: str,
        *,
        anchor_ns: int | None,
        detail: dict[str, Any],
    ) -> None:
        # HH_260906 - Fail closed if one processed anchor is absent from the outcome counts.
        if self._anchor_attempt_count != (
            self._accepted_count + self._anchor_rejected_count
        ):
            raise RuntimeError("shadow anchor accounting invariant changed")
        self._last_status_state = state
        self._last_status_anchor_ns = anchor_ns
        self._last_status_detail = dict(detail)
        bundle_age_ms = (
            None
            if self._last_complete_bundle_wall_ns is None
            else max(
                0.0,
                (time.monotonic_ns() - self._last_complete_bundle_wall_ns) * 1.0e-6,
            )
        )
        inference_inputs_healthy_now = state == "SHADOW_OK"
        # HH_260906 - Keep overall health false until live camera extrinsics are verified.
        calibration_extrinsics_verified = (
            self._provenance.get("tf_extrinsic_parity") == "VERIFIED"
        )
        payload = {
            "schema_id": SHADOW_STATUS_SCHEMA_ID,
            "state": state,
            "stage": detail.get("stage", "none"),
            "failure_code": detail.get("failure_code", "none"),
            "healthy_now": (
                inference_inputs_healthy_now and calibration_extrinsics_verified
            ),
            "inference_inputs_healthy_now": inference_inputs_healthy_now,
            "calibration_extrinsics_verified": calibration_extrinsics_verified,
            "health_scope": "full_runtime_requires_camera_extrinsic_parity",
            "anchor_timestamp_ns": anchor_ns,
            "status_timestamp_ns": int(self.get_clock().now().nanoseconds),
            "status_wall_timestamp_ns": time.time_ns(),
            "anchor_attempt_count": self._anchor_attempt_count,
            "accepted_count": self._accepted_count,
            "anchor_rejected_count": self._anchor_rejected_count,
            "input_event_rejected_count": self._input_event_rejected_count,
            "rejection_counts_by_stage": dict(self._rejection_counts_by_stage),
            "last_complete_bundle_wall_age_ms": bundle_age_ms,
            "camera_bundle_counters": self._bundle.counters(),
            "vehicle_control_approved": False,
            "output_topics": sorted(SHADOW_OUTPUT_TOPICS),
            "provenance": self._provenance,
            "detail": detail,
        }
        self._status_publisher.publish(
            String(data=json.dumps(payload, sort_keys=True, allow_nan=False))
        )


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = PortableE2EShadowNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                # HH_260906 - Treat a repeated launch SIGINT during node teardown as clean shutdown.
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except (KeyboardInterrupt, ExternalShutdownException):
                # HH_260906 - Keep shutdown idempotent when launch and rclpy signal together.
                pass


if __name__ == "__main__":
    main()
