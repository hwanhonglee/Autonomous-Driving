# HH_260906 - Verify the ROS adapter remains causal, ABI-exact, and shadow-only.

import ast
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "autoware_e2e_vad_launch/scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from portable_e2e import ContractError
from portable_e2e.runtime import CameraCalibrationMetadata, LiveCameraCalibration
from portable_e2e.runtime_contract import SelectedTrajectory
import portable_e2e_shadow_node as shadow


class _Publisher:
    def __init__(self, topic_name):
        self.topic_name = topic_name


class _StatusHarness:
    # HH_260906 - Exercise status accounting without constructing a live ROS node.
    _reject = shadow.PortableE2EShadowNode._reject
    _publish_status = shadow.PortableE2EShadowNode._publish_status
    _publish_bundle_timeout_if_needed = (
        shadow.PortableE2EShadowNode._publish_bundle_timeout_if_needed
    )

    def __init__(self):
        self._anchor_attempt_count = 0
        self._accepted_count = 0
        self._anchor_rejected_count = 0
        self._input_event_rejected_count = 0
        self._rejection_counts_by_stage = {
            stage: 0 for stage in shadow.SHADOW_REJECTION_STAGES
        }
        self._provenance = {"fixture": True}
        self._bundle_timeout_s = 0.3
        self._last_complete_bundle_wall_ns = None
        self._bundle = SimpleNamespace(
            counters=lambda: {
                "pending_bundle_count": 0,
                "dropped_stale_count": 0,
                "expired_pending_count": 0,
                "evicted_capacity_count": 0,
            }
        )
        self.messages = []
        self._status_publisher = SimpleNamespace(
            publish=lambda message: self.messages.append(message)
        )

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=123))

    def get_logger(self):
        return SimpleNamespace(warning=lambda message: None)


def _camera_metadata(name):
    return CameraCalibrationMetadata(
        name=name,
        model_index=shadow.CAMERA_ORDER.index(name),
        optical_frame=f"{name}/camera_optical_link",
        width_px=640,
        height_px=360,
        intrinsic_k=(457.0, 0.0, 320.0, 0.0, 457.0, 180.0, 0.0, 0.0, 1.0),
        distortion_d=(),
        rectified=True,
    )


def _live_calibration(name, stamp_ns):
    expected = _camera_metadata(name)
    intrinsic = expected.intrinsic_k
    return LiveCameraCalibration(
        camera_name=name,
        timestamp_ns=stamp_ns,
        optical_frame=expected.optical_frame,
        width_px=640,
        height_px=360,
        distortion_model="plumb_bob",
        distortion_d=(0.0,) * 5,
        intrinsic_k=intrinsic,
        rectification_r=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        projection_p=(
            intrinsic[0], 0.0, intrinsic[2], 0.0,
            0.0, intrinsic[4], intrinsic[5], 0.0,
            0.0, 0.0, 1.0, 0.0,
        ),
        binning_x=0,
        binning_y=0,
        roi_x_offset=0,
        roi_y_offset=0,
        roi_width=0,
        roi_height=0,
        roi_do_rectify=False,
    )


def _selected_trajectory():
    points = tuple((0.1 * (index + 1), 0.0) for index in range(64))
    return SelectedTrajectory(
        candidate_index=3,
        xy_base_m=points,
        speed_mps=(2.0,) * 64,
        timestep_s=0.1,
        logit_margin=0.75,
        planar_extent_m=6.4,
        heading_rad=(0.0,) * 64,
    )


def _route_with_commands(commands):
    points = tuple(
        SimpleNamespace(distance_m=float(index), vad_command=command)
        for index, command in enumerate(commands)
    )
    return SimpleNamespace(points=points, length_m=points[-1].distance_m)


def _runtime_policy():
    return {
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
    }


def test_shadow_output_allowlist_contains_only_fixed_isolated_topics():
    assert shadow.SHADOW_OUTPUT_TOPICS == {
        "/planning/portable_e2e/shadow_trajectory",
        "/planning/portable_e2e/shadow_path",
        "/planning/portable_e2e/status",
        "/planning/portable_e2e/latency_ms",
        "/planning/portable_e2e/selected_candidate",
    }
    assert "/planning/trajectory" not in shadow.SHADOW_OUTPUT_TOPICS
    assert all("/control/" not in topic for topic in shadow.SHADOW_OUTPUT_TOPICS)
    assert all(
        topic.startswith(shadow.SHADOW_TOPIC_PREFIX)
        for topic in shadow.SHADOW_OUTPUT_TOPICS
    )


def test_node_source_has_no_unguarded_or_authoritative_publisher():
    source_path = SCRIPTS / "portable_e2e_shadow_node.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    publisher_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_publisher"
    ]
    string_literals = {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }

    assert len(publisher_calls) == 1
    assert "/planning/trajectory" not in string_literals
    assert all(not value.startswith("/control/") for value in string_literals)


def test_status_accounting_separates_input_events_from_anchor_attempts():
    node = _StatusHarness()

    node._reject("image", ContractError("invalid image"))
    input_payload = json.loads(node.messages[-1].data)
    assert input_payload["schema_id"] == shadow.SHADOW_STATUS_SCHEMA_ID
    assert input_payload["anchor_attempt_count"] == 0
    assert input_payload["accepted_count"] == 0
    assert input_payload["anchor_rejected_count"] == 0
    assert input_payload["input_event_rejected_count"] == 1
    assert input_payload["failure_code"] == "image_contract"

    node._anchor_attempt_count = 1
    node._reject(
        "inference",
        ContractError("runtime inference exceeded 100.0 ms"),
        anchor_ns=100,
    )
    anchor_payload = json.loads(node.messages[-1].data)
    assert anchor_payload["anchor_attempt_count"] == 1
    assert anchor_payload["accepted_count"] == 0
    assert anchor_payload["anchor_rejected_count"] == 1
    assert anchor_payload["input_event_rejected_count"] == 1
    assert anchor_payload["failure_code"] == "inference_deadline"
    assert anchor_payload["rejection_counts_by_stage"]["image"] == 1
    assert anchor_payload["rejection_counts_by_stage"]["inference"] == 1


def test_failure_classifier_rejects_unknown_stage():
    with pytest.raises(RuntimeError, match="unknown shadow rejection stage"):
        shadow.classify_shadow_failure("unknown", ContractError("fixture"))


def test_status_revokes_ok_after_exact_camera_bundle_timeout(monkeypatch):
    node = _StatusHarness()
    node._last_complete_bundle_wall_ns = 1_000_000_000
    node._last_status_state = "SHADOW_OK"
    node._last_status_anchor_ns = 900_000_000
    node._last_status_detail = {"failure_code": "none"}
    monkeypatch.setattr(shadow.time, "monotonic_ns", lambda: 1_400_000_000)

    assert node._publish_bundle_timeout_if_needed() is True

    payload = json.loads(node.messages[-1].data)
    assert payload["state"] == "SHADOW_WAITING"
    assert payload["healthy_now"] is False
    assert payload["failure_code"] == "camera_bundle_timeout"
    assert payload["last_complete_bundle_wall_age_ms"] == pytest.approx(400.0)


def test_shadow_ok_does_not_claim_full_health_without_extrinsic_parity():
    # HH_260906 - Separate accepted inference inputs from unverified camera mounting geometry.
    node = _StatusHarness()
    node._provenance = {"tf_extrinsic_parity": "UNIMPLEMENTED_BLOCKED"}
    node._anchor_attempt_count = 1
    node._accepted_count = 1

    node._publish_status(
        "SHADOW_OK",
        anchor_ns=100,
        detail={"stage": "inference", "failure_code": "none"},
    )

    payload = json.loads(node.messages[-1].data)
    assert payload["inference_inputs_healthy_now"] is True
    assert payload["calibration_extrinsics_verified"] is False
    assert payload["healthy_now"] is False
    assert payload["vehicle_control_approved"] is False


def test_node_watchdogs_use_steady_clock_when_simulation_clock_freezes():
    # HH_260906 - Prevent use_sim_time from disabling the wall-time health watchdog.
    source_path = SCRIPTS / "portable_e2e_shadow_node.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    steady_clock_assignments = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Attribute)
            and target.attr == "_steady_clock"
            for target in node.targets
        )
    ]
    timer_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_timer"
    ]

    assert len(steady_clock_assignments) == 1
    assert len(timer_calls) == 2
    assert all(
        any(
            keyword.arg == "clock"
            and isinstance(keyword.value, ast.Attribute)
            and keyword.value.attr == "_steady_clock"
            for keyword in call.keywords
        )
        for call in timer_calls
    )


def test_shadow_launch_pins_route_and_cannot_remap_publishers():
    launch = ET.parse(
        ROOT / "autoware_e2e_vad_launch/launch/portable_e2e_shadow.launch.xml"
    ).getroot()
    arguments = {element.get("name") for element in launch.findall("./arg")}
    node = launch.find("./node")
    assert node is not None
    parameters = {
        element.get("name"): element.get("value")
        for element in node.findall("./param")
    }

    assert {
        "contract_file",
        "contract_sha256",
        "runtime_bundle_file",
        "runtime_bundle_sha256",
        "source_checkpoint_sha256",
        "route_sha256",
    } <= arguments
    assert parameters["contract_file"] == "$(var contract_file)"
    assert parameters["contract_sha256"] == "$(var contract_sha256)"
    assert parameters["route_sha256"] == "$(var route_sha256)"
    assert not list(node.findall(".//remap"))
    assert all(not name.startswith("output_") for name in parameters)


def test_required_contract_path_rejects_symlink(tmp_path):
    regular = tmp_path / "common10.json"
    regular.write_text("{}", encoding="utf-8")
    link = tmp_path / "common10-link.json"
    link.symlink_to(regular)
    node = SimpleNamespace(_required_text=lambda name: str(link))

    with pytest.raises(RuntimeError, match="regular file"):
        shadow.PortableE2EShadowNode._required_path(node, "contract_file")


def test_pinned_route_parses_only_the_stable_hashed_payload(monkeypatch, tmp_path):
    route_file = tmp_path / "route.json"
    route_file.write_text('{"replaced":true}', encoding="utf-8")
    payload = {
        "schema_version": 1,
        "coordinate_reference": "base_link",
        "route": [
            {
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "distance_m": 0.0,
                "vad_command": 3,
                "road_option": "LANEFOLLOW",
            },
            {
                "x": 10.0,
                "y": 0.0,
                "yaw": 0.0,
                "distance_m": 10.0,
                "vad_command": 3,
                "road_option": "LANEFOLLOW",
            },
        ],
    }
    exact_bytes = json.dumps(payload, sort_keys=True).encode("utf-8")
    digest = hashlib.sha256(exact_bytes).hexdigest()
    monkeypatch.setattr(
        shadow,
        "_read_json_and_sha256",
        lambda path, context: (payload, digest),
    )
    monkeypatch.setattr(
        shadow.RoutePlan,
        "load",
        lambda path: (_ for _ in ()).throw(AssertionError("path reopened")),
    )

    route, loaded_digest = shadow.load_pinned_route(route_file, digest)

    assert loaded_digest == digest
    assert route.points[-1].x == 10.0
    assert route.metadata["route_file"] == str(route_file)


def test_pinned_route_rejects_digest_mismatch(monkeypatch, tmp_path):
    monkeypatch.setattr(
        shadow,
        "_read_json_and_sha256",
        lambda path, context: ({}, "b" * 64),
    )

    with pytest.raises(RuntimeError, match="does not match"):
        shadow.load_pinned_route(tmp_path / "route.json", "a" * 64)


def test_pinned_route_requires_explicit_base_link_reference(monkeypatch, tmp_path):
    # HH_260906 - Reject a hash-correct live route whose coordinate frame is ambiguous.
    payload = {"schema_version": 1, "route": []}
    monkeypatch.setattr(
        shadow,
        "_read_json_and_sha256",
        lambda path, context: (payload, "a" * 64),
    )

    with pytest.raises(RuntimeError, match="explicitly declare"):
        shadow.load_pinned_route(tmp_path / "route.json", "a" * 64)


@pytest.mark.parametrize("topic", sorted(shadow.SHADOW_OUTPUT_TOPICS))
def test_fixed_publisher_guard_accepts_exact_shadow_topics(topic):
    shadow.require_fixed_publisher_topic(_Publisher(topic), topic)


@pytest.mark.parametrize(
    ("resolved", "expected"),
    (
        ("/planning/trajectory", "/planning/trajectory"),
        ("/control/command/control_cmd", "/control/command/control_cmd"),
        ("/remapped/status", shadow.SHADOW_STATUS_TOPIC),
        (shadow.SHADOW_STATUS_TOPIC, "/planning/trajectory"),
    ),
)
def test_fixed_publisher_guard_rejects_authority_and_remaps(resolved, expected):
    with pytest.raises(RuntimeError, match="must resolve exactly"):
        shadow.require_fixed_publisher_topic(_Publisher(resolved), expected)


def test_causal_message_buffer_never_selects_future_state():
    buffer = shadow.CausalMessageBuffer(capacity=3)
    buffer.push(300, "future")
    buffer.push(100, "old")
    buffer.push(200, "causal")

    assert buffer.latest_not_after(250) == (200, "causal")
    assert buffer.latest_not_after(50) is None

    buffer.push(200, "replacement")
    assert buffer.latest_not_after(250) == (200, "replacement")
    buffer.push(400, "newest")
    assert buffer.latest_not_after(150) is None


def test_ros_timestamp_conversion_is_exact_and_rejects_invalid_values():
    assert shadow.stamp_to_nanoseconds(Time(sec=12, nanosec=345)) == 12_000_000_345

    with pytest.raises(ContractError, match="timestamp is invalid"):
        shadow.stamp_to_nanoseconds(SimpleNamespace(sec=1, nanosec=1_000_000_000))
    with pytest.raises(ContractError, match="timestamp is invalid"):
        shadow.stamp_to_nanoseconds(SimpleNamespace(sec=-1, nanosec=0))


def test_common10_command_matches_collector_at_transitional_straight():
    route = _route_with_commands((2, 0, 0, 3))

    assert shadow.common10_command_at(route, 0.0, 2.0, 2.5) == 2


def test_common10_command_anticipates_and_exits_maneuvers_like_collector():
    approaching = _route_with_commands((3, 3, 1, 1))
    exiting = _route_with_commands((0, 3, 3, 3))

    assert shadow.common10_command_at(approaching, 0.0, 2.0, 0.5) == 1
    assert shadow.common10_command_at(exiting, 0.0, 0.5, 1.5) == 3


def test_bgra_image_preserves_runtime_encoding_and_owns_payload_copy():
    message = Image()
    message.header.stamp = Time(sec=2, nanosec=50)
    message.header.frame_id = "CAM_FRONT/camera_optical_link"
    message.width = 640
    message.height = 360
    message.encoding = "bgra8"
    message.is_bigendian = 0
    message.step = 640 * 4
    message.data = bytes([1, 2, 3, 4]) * (640 * 360)

    frame = shadow.image_message_to_runtime_frame(
        message,
        expected_frame_id="CAM_FRONT/camera_optical_link",
    )

    assert frame.timestamp_ns == 2_000_000_050
    assert frame.encoding == "bgra8"
    assert frame.payload.shape == (360, 640, 4)
    assert frame.payload.flags.c_contiguous
    assert frame.payload[0, 0].tolist() == [1, 2, 3, 4]
    message.data[0] = 99
    assert frame.payload[0, 0].tolist() == [1, 2, 3, 4]


def test_image_adapter_rejects_wrong_frame_and_malformed_storage():
    message = Image()
    message.header.stamp = Time(sec=1)
    message.header.frame_id = "wrong_frame"
    message.width = 640
    message.height = 360
    message.encoding = "bgr8"
    message.step = 640 * 3
    message.data = bytes(message.step * message.height)

    with pytest.raises(ContractError, match="image frame"):
        shadow.image_message_to_runtime_frame(
            message,
            expected_frame_id="CAM_FRONT/camera_optical_link",
        )

    message.header.frame_id = "CAM_FRONT/camera_optical_link"
    message.step -= 1
    with pytest.raises(ContractError, match="shorter than one row"):
        shadow.image_message_to_runtime_frame(message)


def test_camera_info_adapter_copies_exact_rectified_common10_metadata():
    message = CameraInfo()
    message.header.stamp = Time(sec=4, nanosec=25)
    message.header.frame_id = "CAM_FRONT/camera_optical_link"
    message.width = 640
    message.height = 360
    message.distortion_model = "plumb_bob"
    message.d = [0.0] * 5
    message.k = [457.0, 0.0, 320.0, 0.0, 457.0, 180.0, 0.0, 0.0, 1.0]
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.p = [
        457.0, 0.0, 320.0, 0.0,
        0.0, 457.0, 180.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]

    observed = shadow.camera_info_message_to_live_calibration(
        message, camera_name="CAM_FRONT"
    )

    assert observed == _live_calibration("CAM_FRONT", 4_000_000_025)
    message.k[0] = 1.0
    assert observed.intrinsic_k[0] == 457.0


def test_camera_info_gate_uses_reliable_qos_and_causal_nonstale_six_camera_set():
    assert shadow.CAMERA_INFO_QOS.reliability == shadow.ReliabilityPolicy.RELIABLE
    node = SimpleNamespace(
        _camera_calibration={
            name: _camera_metadata(name) for name in shadow.CAMERA_ORDER
        },
        _camera_info_buffers={
            name: shadow.CausalMessageBuffer() for name in shadow.CAMERA_ORDER
        },
        _maximum_camera_info_age_s=0.25,
    )
    for name in shadow.CAMERA_ORDER:
        observed = _live_calibration(name, 900_000_000)
        node._camera_info_buffers[name].push(observed.timestamp_ns, observed)
        future = _live_calibration(name, 1_100_000_000)
        node._camera_info_buffers[name].push(future.timestamp_ns, future)

    stamps = shadow.PortableE2EShadowNode._validated_camera_info(
        node, 1_000_000_000
    )

    assert stamps == {name: 900_000_000 for name in shadow.CAMERA_ORDER}


def test_camera_info_gate_rejects_missing_stale_and_changed_intrinsics():
    node = SimpleNamespace(
        _camera_calibration={
            name: _camera_metadata(name) for name in shadow.CAMERA_ORDER
        },
        _camera_info_buffers={
            name: shadow.CausalMessageBuffer() for name in shadow.CAMERA_ORDER
        },
        _maximum_camera_info_age_s=0.05,
    )
    with pytest.raises(ContractError, match="is missing"):
        shadow.PortableE2EShadowNode._validated_camera_info(node, 1_000_000_000)

    for name in shadow.CAMERA_ORDER:
        observed = _live_calibration(name, 900_000_000)
        node._camera_info_buffers[name].push(observed.timestamp_ns, observed)
    with pytest.raises(ContractError, match="is stale"):
        shadow.PortableE2EShadowNode._validated_camera_info(node, 1_000_000_000)

    node._maximum_camera_info_age_s = 0.25
    changed = replace(
        _live_calibration("CAM_FRONT", 950_000_000),
        intrinsic_k=(1.0, 0.0, 320.0, 0.0, 457.0, 180.0, 0.0, 0.0, 1.0),
    )
    node._camera_info_buffers["CAM_FRONT"].push(changed.timestamp_ns, changed)
    with pytest.raises(ContractError, match="K does not match"):
        shadow.PortableE2EShadowNode._validated_camera_info(node, 1_000_000_000)


def test_shadow_trajectory_has_exact_common10_horizon_and_anchor_stamp():
    odometry = Odometry()
    odometry.pose.pose.position.x = 10.0
    odometry.pose.pose.position.y = 20.0
    odometry.pose.pose.position.z = 1.0
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.x = 1.0
    result = SimpleNamespace(selected=_selected_trajectory())

    trajectory = shadow.make_shadow_trajectory(
        result,
        odometry,
        anchor_ns=1_234_567_890,
    )

    assert trajectory.header.frame_id == "map"
    assert trajectory.header.stamp.sec == 1
    assert trajectory.header.stamp.nanosec == 234_567_890
    assert len(trajectory.points) == 64
    assert trajectory.points[0].pose.position.x == pytest.approx(10.1)
    assert trajectory.points[0].pose.position.y == pytest.approx(20.0)
    assert trajectory.points[0].time_from_start.sec == 0
    assert trajectory.points[0].time_from_start.nanosec == 100_000_000
    assert trajectory.points[-1].pose.position.x == pytest.approx(16.4)
    assert trajectory.points[-1].time_from_start.sec == 6
    assert trajectory.points[-1].time_from_start.nanosec == 400_000_000

    path = shadow.make_shadow_path(trajectory)
    assert path.header == trajectory.header
    assert len(path.poses) == 64
    assert path.poses[0].header == trajectory.header
    assert path.poses[-1].pose == trajectory.points[-1].pose


def test_shadow_trajectory_holds_heading_rate_for_subcentimeter_jitter():
    odometry = Odometry()
    odometry.pose.pose.orientation.w = 1.0
    points = tuple(
        ((-1.0 if index % 2 else 1.0) * 0.001, 0.0) for index in range(64)
    )
    selected = SelectedTrajectory(
        candidate_index=0,
        xy_base_m=points,
        speed_mps=(0.02,) * 64,
        timestep_s=0.1,
        logit_margin=0.0,
        planar_extent_m=0.127,
        heading_rad=(0.0,) * 64,
    )

    trajectory = shadow.make_shadow_trajectory(
        SimpleNamespace(selected=selected),
        odometry,
        anchor_ns=1,
    )

    assert {point.heading_rate_rps for point in trajectory.points} == {0.0}


@pytest.mark.parametrize("field", ("z", "longitudinal_speed"))
def test_shadow_trajectory_rejects_nonfinite_ego_output_fields(field):
    # HH_260906 - Guarantee every copied numeric field is finite at the ROS output boundary.
    odometry = Odometry()
    odometry.pose.pose.orientation.w = 1.0
    if field == "z":
        odometry.pose.pose.position.z = math.nan
    else:
        odometry.twist.twist.linear.x = math.inf

    with pytest.raises(ContractError, match="must be finite"):
        shadow.make_shadow_trajectory(
            SimpleNamespace(selected=_selected_trajectory()),
            odometry,
            anchor_ns=1,
        )


def test_shadow_trajectory_reuses_validated_mixed_speed_headings():
    odometry = Odometry()
    odometry.pose.pose.orientation.w = 1.0
    heading = math.atan2(0.011, 0.012)
    points = ((0.009, 0.0), (0.012, 0.011)) + tuple(
        (
            0.012 + 0.1 * offset * math.cos(heading),
            0.011 + 0.1 * offset * math.sin(heading),
        )
        for offset in range(1, 63)
    )
    selected = SelectedTrajectory(
        candidate_index=0,
        xy_base_m=points,
        speed_mps=(
            0.09,
            math.hypot(0.003, 0.011) / 0.1,
            0.4,
            0.7,
            1.0,
        )
        + (1.0,) * 59,
        timestep_s=0.1,
        logit_margin=0.0,
        planar_extent_m=6.2,
        heading_rad=(0.0, heading) + (heading,) * 62,
    )

    trajectory = shadow.make_shadow_trajectory(
        SimpleNamespace(selected=selected),
        odometry,
        anchor_ns=1,
    )

    assert trajectory.points[1].heading_rate_rps == pytest.approx(heading / 0.1)
    assert max(abs(point.heading_rate_rps) for point in trajectory.points) <= 7.5


@pytest.mark.parametrize("anchor_ns", (-1, True, 1.5))
def test_shadow_trajectory_rejects_invalid_anchor(anchor_ns):
    odometry = Odometry()
    odometry.pose.pose.orientation.w = 1.0

    with pytest.raises(ContractError, match="anchor"):
        shadow.make_shadow_trajectory(
            SimpleNamespace(selected=_selected_trajectory()),
            odometry,
            anchor_ns=anchor_ns,
        )


@pytest.mark.parametrize(
    "selected",
    (
        SelectedTrajectory(
            candidate_index=0,
            xy_base_m=((0.1, 0.0),) * 63,
            speed_mps=(1.0,) * 63,
            timestep_s=0.1,
            logit_margin=0.0,
            planar_extent_m=0.1,
        ),
        SelectedTrajectory(
            candidate_index=0,
            xy_base_m=((0.1, 0.0),) * 64,
            speed_mps=(1.0,) * 64,
            timestep_s=0.2,
            logit_margin=0.0,
            planar_extent_m=0.1,
        ),
    ),
)
def test_shadow_trajectory_rejects_non_common10_output(selected):
    odometry = Odometry()
    odometry.pose.pose.orientation.w = 1.0

    with pytest.raises(ContractError, match="exact Common10"):
        shadow.make_shadow_trajectory(
            SimpleNamespace(selected=selected),
            odometry,
            anchor_ns=1,
        )


@pytest.mark.parametrize("value", ("a" * 63, "A" * 64, "g" * 64, ""))
def test_required_sha256_is_strict(value):
    with pytest.raises(RuntimeError, match="lowercase SHA-256"):
        shadow.require_sha256(value, "fixture")


def test_shadow_provenance_pins_common10_runtime_bundle_rig_and_route():
    provenance = shadow.build_shadow_provenance(
        runtime_id="runtime-v1",
        model_id="portable_e2e.perspective_trajectory.v0",
        runtime_bundle_sha256="a" * 64,
        source_checkpoint_sha256="9" * 64,
        model_config_sha256="b" * 64,
        corpus_fingerprint_sha256="c" * 64,
        rig_id="rig-v1",
        rig_sha256="d" * 64,
        contract_sha256="f" * 64,
        route_sha256="e" * 64,
        runtime_gate=shadow.RuntimeGateConfig(),
        runtime_policy=_runtime_policy(),
        device_name="cpu",
        clock_mode="ros_sim_time",
    )

    assert provenance == {
        "adapter_id": shadow.SHADOW_ADAPTER_ID,
        "runtime_id": "runtime-v1",
        "model_id": "portable_e2e.perspective_trajectory.v0",
        "runtime_bundle_id": shadow.BUNDLE_ID,
        "common10_contract_id": "common_10hz_v1",
        "runtime_contract_id": shadow.RUNTIME_CONTRACT_ID,
        "runtime_gate_id": shadow.RUNTIME_GATE_ID,
        "camera_order": shadow.CAMERA_ORDER,
        "camera_bundle_policy": "exact_source_timestamp",
        "model_future_points": 64,
        "model_timestep_s": 0.1,
        "model_output_frame": "base_link_at_anchor",
        "published_trajectory_frame": "map",
        "runtime_device": "cpu",
        "clock_mode": "ros_sim_time",
        "watchdog_clock_mode": "steady_time",
        "runtime_bundle_sha256": "a" * 64,
        "source_checkpoint_sha256": "9" * 64,
        "model_config_sha256": "b" * 64,
        "corpus_fingerprint_sha256": "c" * 64,
        "rig_id": "rig-v1",
        "rig_sha256": "d" * 64,
        "contract_sha256": "f" * 64,
        "live_camera_info_policy": "causal_nonstale_exact_common10_rig_match",
        "tf_extrinsic_parity": "UNIMPLEMENTED_BLOCKED",
        "health_scope": "runtime_inputs_except_camera_extrinsics",
        "route_sha256": "e" * 64,
        "runtime_gate": asdict(shadow.RuntimeGateConfig()),
        "runtime_policy": _runtime_policy(),
    }

    result = SimpleNamespace(
        runtime_id="runtime-v1",
        runtime_bundle_sha256="a" * 64,
        source_checkpoint_sha256="9" * 64,
        vehicle_control_approved=False,
    )
    shadow.validate_shadow_result_provenance(result, provenance)


@pytest.mark.parametrize(
    "overrides",
    (
        {"runtime_id": "wrong"},
        {"runtime_bundle_sha256": "f" * 64},
        {"source_checkpoint_sha256": "f" * 64},
        {"vehicle_control_approved": True},
    ),
)
def test_shadow_result_rejects_changed_or_control_approved_provenance(overrides):
    provenance = shadow.build_shadow_provenance(
        runtime_id="runtime-v1",
        model_id="portable_e2e.perspective_trajectory.physical.v1",
        runtime_bundle_sha256="a" * 64,
        source_checkpoint_sha256="9" * 64,
        model_config_sha256="b" * 64,
        corpus_fingerprint_sha256="c" * 64,
        rig_id="rig-v1",
        rig_sha256="d" * 64,
        contract_sha256="f" * 64,
        route_sha256="e" * 64,
        runtime_gate=shadow.RuntimeGateConfig(),
        runtime_policy=_runtime_policy(),
        device_name="cpu",
        clock_mode="ros_sim_time",
    )
    values = {
        "runtime_id": "runtime-v1",
        "runtime_bundle_sha256": "a" * 64,
        "source_checkpoint_sha256": "9" * 64,
        "vehicle_control_approved": False,
        **overrides,
    }

    with pytest.raises(ContractError, match="provenance|unapproved"):
        shadow.validate_shadow_result_provenance(SimpleNamespace(**values), provenance)


def test_main_handles_repeated_sigint_during_node_teardown(monkeypatch):
    # HH_260906 - Reproduce launch and rclpy delivering SIGINT during the same shutdown.
    events = []

    class _InterruptingNode:
        def destroy_node(self):
            events.append("destroy_node")
            raise KeyboardInterrupt

    monkeypatch.setattr(shadow.rclpy, "init", lambda: events.append("init"))
    monkeypatch.setattr(
        shadow,
        "PortableE2EShadowNode",
        lambda: _InterruptingNode(),
    )
    monkeypatch.setattr(
        shadow.rclpy,
        "spin",
        lambda node: (_ for _ in ()).throw(KeyboardInterrupt()),
    )
    monkeypatch.setattr(shadow.rclpy, "ok", lambda: True)
    monkeypatch.setattr(shadow.rclpy, "shutdown", lambda: events.append("shutdown"))

    shadow.main()

    assert events == ["init", "destroy_node", "shutdown"]
