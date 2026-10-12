#!/usr/bin/env python3

"""Read-only validation of the full Autoware + CARLA + VAD runtime graph."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
import json
import math
import os
from pathlib import Path
import re
import sys
import time
from typing import Any, Callable


ENV_SENTINEL = "AUTOWARE_E2E_FULL_VALIDATOR_ENV"


@dataclass
class CheckResult:
    section: str
    label: str
    ok: bool
    detail: str


@dataclass
class SampleState:
    total_count: int = 0
    valid_count: int = 0
    first_arrival: float | None = None
    last_arrival: float | None = None
    stamp_ns: int | None = None
    summary: str = "no message"
    last_error: str = "no message"


@dataclass(frozen=True)
class LiveTopic:
    topic: str
    label: str
    validator: Callable[[Any, argparse.Namespace], tuple[bool, str]]
    minimum: int = 2
    transient_local: bool = False
    check_sim_age: bool = False


@dataclass
class Report:
    results: list[CheckResult] = field(default_factory=list)
    nodes: list[str] = field(default_factory=list)
    samples: dict[str, dict[str, Any]] = field(default_factory=dict)

    def add(self, section: str, label: str, ok: bool, detail: str) -> None:
        self.results.append(CheckResult(section, label, ok, detail))

    @property
    def failures(self) -> list[CheckResult]:
        return [result for result in self.results if not result.ok]

    def emit(self, verbose: bool) -> None:
        current_section = None
        for result in self.results:
            if result.section != current_section:
                current_section = result.section
                print(f"\n[{current_section}]")
            if result.ok and not verbose and result.section in {"Topics", "Freshness"}:
                continue
            status = "PASS" if result.ok else "FAIL"
            print(f"  {status:<4} {result.label}: {result.detail}")

        passed = len(self.results) - len(self.failures)
        status = "PASS" if not self.failures else "FAIL"
        print(
            f"\nFull Autoware + CARLA + VAD validation: {status} "
            f"({passed}/{len(self.results)} checks passed)"
        )
        if not verbose:
            topic_passes = sum(
                result.ok
                for result in self.results
                if result.section in {"Topics", "Freshness"}
            )
            if topic_passes:
                print(
                    f"  {topic_passes} successful topic checks hidden; "
                    "use --verbose to show them."
                )

    def write_json(self, path: str) -> None:
        output = Path(path).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "success": not self.failures,
            "checks": [asdict(result) for result in self.results],
            "nodes": self.nodes,
            "samples": self.samples,
        }
        output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"Validation report: {output}")


NODE_GROUPS = (
    ("CARLA bridge", (r"^/autoware_carla_interface$",), 1),
    ("CARLA actuation conversion", (r"^/autoware_raw_vehicle_cmd_converter$",), 1),
    (
        "standard vehicle model",
        (r"/(?:robot_state_publisher|vehicle_state_publisher)$",),
        1,
    ),
    (
        "standard map loaders",
        (r"^/map/(?:pointcloud_map_loader|lanelet2_map_loader)$",),
        2,
    ),
    (
        "standard sensing",
        (
            r"vehicle_velocity_converter",
            r"/sensing/lidar/(?:crop_box_filter_self|pointcloud_relay)$",
            r"/sensing/imu/(?:imu_corrector|gyro_bias_estimator)$",
        ),
        3,
    ),
    ("CARLA truth localization", (r"^/carla_state_publisher$", r"^/twist2accel$"), 2),
    (
        "standard system",
        (
            r"^/system/duplicated_node_checker$",
            r"^/system/(?:processing_time_checker|pipeline_latency_monitor)$",
            r"^/system/.+(?:diagnostic|component)",
        ),
        3,
    ),
    ("standard AD API", (r"^/adapi/node/",), 3),
    (
        "standard control",
        (
            r"^/control/vehicle_cmd_gate$",
            r"^/control/autoware_operation_mode_transition_manager$",
            r"^/control/trajectory_follower/controller_node_exe$",
        ),
        3,
    ),
    (
        "standard mission planner",
        (
            r"^/planning/mission_planning/mission_planner$",
            r"^/planning/mission_planning/route_selector$",
        ),
        2,
    ),
    ("VAD inference", (r"^/vad_carla_tiny$",), 1),
    ("VAD AEB configuration", (r"^/vad_aeb_configurator$",), 1),
    (
        "VAD route integration",
        (r"^/vad_route_manager$", r"^/vad_standard_route_adapter$"),
        2,
    ),
    (
        "Autoware RViz UI",
        (r"/(?:rviz2|autoware_rviz|autoware_vad_rviz)(?:_\w+)?$",),
        1,
    ),
)


CAMERAS = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)


TOPIC_GROUPS = {
    "standard UI/system": (
        "/robot_description",
        "/diagnostics_graph/status",
        "/system/operation_mode/state",
        "/api/operation_mode/state",
        "/autoware/state",
        "/system/vad/aeb_configured",
    ),
    "standard map": (
        "/map/pointcloud_map",
        "/map/vector_map",
        "/map/vector_map_marker",
    ),
    "standard sensing": (
        "/sensing/lidar/top/pointcloud_before_sync",
        "/sensing/lidar/concatenated/pointcloud",
        "/sensing/imu/tamagawa/imu_raw",
        "/sensing/imu/imu_data",
        "/sensing/vehicle_velocity_converter/twist_with_covariance",
    ),
    "localization state": (
        "/api/localization/initialization_state",
        "/localization/kinematic_state",
        "/localization/acceleration",
    ),
    "VAD outputs": (
        "/planning/vad/raw_trajectory",
        "/planning/vad/candidate_trajectories",
        "/perception/object_recognition/objects",
        "/perception/vad/map_points",
    ),
    "standard mission planning": (
        "/planning/route",
        "/planning/mission_planning/route",
        "/planning/mission_planning/route_marker",
        "/planning/route_state",
    ),
    "VAD route integration": (
        "/planning/trajectory",
        "/planning/vad_route/status",
        "/planning/vad_route/cross_track_error",
        "/planning/vad_route/trajectory_correction",
        "/planning/vad_route/standard_route_aligned",
    ),
    "standard control": (
        "/control/trajectory_follower/control_cmd",
        "/control/command/control_cmd",
        "/control/command/actuation_cmd",
    ),
}


OWNERSHIP = (
    (
        "/planning/trajectory",
        "final trajectory",
        r"^/vad_route_manager$",
    ),
    (
        "/perception/object_recognition/objects",
        "canonical objects",
        r"^/vad_carla_tiny$",
    ),
    (
        "/localization/kinematic_state",
        "localization state",
        r"^/carla_state_publisher$",
    ),
    (
        "/control/command/control_cmd",
        "final control command",
        r"^/control/vehicle_cmd_gate$",
    ),
    (
        "/planning/vad/raw_trajectory",
        "raw VAD trajectory",
        r"^/vad_carla_tiny$",
    ),
    (
        "/localization/acceleration",
        "localization acceleration",
        r"^/twist2accel$",
    ),
    (
        "/control/command/actuation_cmd",
        "CARLA actuation command",
        r"^/autoware_raw_vehicle_cmd_converter$",
    ),
)


SUBSCRIBER_CONNECTIONS = (
    (
        "/planning/vad/candidate_trajectories",
        "VAD candidates -> route manager",
        (r"^/vad_route_manager$",),
        1,
    ),
    (
        "/planning/trajectory",
        "final trajectory -> standard controller",
        (r"^/control/trajectory_follower/controller_node_exe$",),
        1,
    ),
    (
        "/perception/object_recognition/objects",
        "VAD objects -> standard control safety",
        (r"^/control/(?:autonomous_emergency_braking|collision_detector)$",),
        2,
    ),
    (
        "/localization/kinematic_state",
        "truth localization -> VAD, route, and controller",
        (
            r"^/vad_carla_tiny$",
            r"^/vad_route_manager$",
            r"^/control/trajectory_follower/controller_node_exe$",
        ),
        3,
    ),
    (
        "/planning/mission_planning/route",
        "standard mission route -> VAD route adapter",
        (r"^/vad_standard_route_adapter$",),
        1,
    ),
    (
        "/control/command/control_cmd",
        "standard control -> CARLA actuation converter",
        (r"^/autoware_raw_vehicle_cmd_converter$",),
        1,
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only validation of the full Autoware shell, CARLA bridge, "
            "VAD ownership boundaries, route alignment, and live data."
        )
    )
    parser.add_argument(
        "--graph-timeout",
        type=float,
        default=30.0,
        help="wall seconds to wait for the full runtime graph",
    )
    parser.add_argument(
        "--message-timeout",
        type=float,
        default=30.0,
        help="wall seconds to collect fresh messages after graph discovery",
    )
    parser.add_argument(
        "--minimum-live-messages",
        type=int,
        default=2,
        help="minimum new messages required from each live topic",
    )
    parser.add_argument(
        "--max-wall-age",
        type=float,
        default=5.0,
        help="maximum wall-clock age of the last live message",
    )
    parser.add_argument(
        "--max-sim-age",
        type=float,
        default=3.0,
        help="maximum absolute difference from /clock for stamped core data",
    )
    parser.add_argument(
        "--max-cross-track",
        type=float,
        default=3.5,
        help="maximum allowed route cross-track error in meters",
    )
    parser.add_argument(
        "--max-trajectory-correction",
        type=float,
        default=15.0,
        help="maximum route correction applied to a VAD trajectory in meters",
    )
    parser.add_argument("--json", metavar="PATH", help="also write a machine-readable report")
    parser.add_argument("--verbose", action="store_true", help="show every successful topic check")
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="run dependency-free helper tests without sourcing ROS",
    )
    args = parser.parse_args()
    positive = (
        "graph_timeout",
        "message_timeout",
        "max_wall_age",
        "max_sim_age",
        "max_cross_track",
        "max_trajectory_correction",
    )
    for name in positive:
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive and finite")
    if args.minimum_live_messages < 1:
        parser.error("--minimum-live-messages must be at least 1")
    return args


def bootstrap_environment() -> None:
    if os.environ.get(ENV_SENTINEL) == "1":
        return
    script = Path(__file__).resolve()
    root = script.parents[2]
    env_file = root / "scripts/e2e/env.sh"
    if not env_file.is_file():
        raise RuntimeError(f"project environment script not found: {env_file}")
    command = (
        'env_file="$1"; validator="$2"; shift 2; '
        'source "$env_file" || exit $?; '
        f"export {ENV_SENTINEL}=1; "
        'exec python3 "$validator" "$@"'
    )
    os.execv(
        "/bin/bash",
        ["bash", "-c", command, "validate_full_stack", str(env_file), str(script), *sys.argv[1:]],
    )


def canonical_node(name: str, namespace: str) -> str:
    parts = [part for part in (namespace.strip("/"), name.strip("/")) if part]
    return "/" + "/".join(parts)


def endpoint_node(endpoint: Any) -> str:
    return canonical_node(endpoint.node_name, endpoint.node_namespace)


def matching_nodes(nodes: set[str], patterns: tuple[str, ...]) -> list[str]:
    return sorted(node for node in nodes if any(re.search(pattern, node) for pattern in patterns))


def finite_values(values: list[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def header_stamp_ns(message: Any) -> int | None:
    clock = getattr(message, "clock", None)
    if clock is not None:
        return int(clock.sec) * 1_000_000_000 + int(clock.nanosec)
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def validate_presence(message: Any, _args: argparse.Namespace) -> tuple[bool, str]:
    del message
    return True, "message received"


def validate_clock(message: Any, _args: argparse.Namespace) -> tuple[bool, str]:
    nanoseconds = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)
    return nanoseconds >= 0, f"sim_time={nanoseconds / 1.0e9:.3f}s"


def validate_localization_initialized(
    message: Any, _args: argparse.Namespace
) -> tuple[bool, str]:
    state = int(message.state)
    return state == 3, f"state={state} (INITIALIZED=3)"


def validate_operation_mode_ready(
    message: Any, _args: argparse.Namespace
) -> tuple[bool, str]:
    stable = not bool(message.is_in_transition)
    autonomous_available = bool(message.is_autonomous_mode_available)
    ok = stable and autonomous_available
    return (
        ok,
        f"mode={int(message.mode)}, stable={stable}, "
        f"autonomous_available={autonomous_available}, "
        f"control_enabled={bool(message.is_autoware_control_enabled)}",
    )


def validate_odometry(message: Any, _args: argparse.Namespace) -> tuple[bool, str]:
    position = message.pose.pose.position
    velocity = message.twist.twist.linear
    values = [position.x, position.y, position.z, velocity.x, velocity.y, velocity.z]
    frame = message.header.frame_id
    ok = frame == "map" and finite_values(values)
    return ok, f"frame={frame!r}, position=({position.x:.2f}, {position.y:.2f})"


def validate_acceleration(message: Any, _args: argparse.Namespace) -> tuple[bool, str]:
    linear = message.accel.accel.linear
    values = [linear.x, linear.y, linear.z]
    return finite_values(values), f"linear=({linear.x:.2f}, {linear.y:.2f}, {linear.z:.2f})"


def trajectory_points(message: Any) -> list[Any]:
    return list(getattr(message, "points", ()))


def validate_trajectory(message: Any, _args: argparse.Namespace) -> tuple[bool, str]:
    points = trajectory_points(message)
    frame = message.header.frame_id
    if not points:
        return False, f"frame={frame!r}, points=0"
    planar = [
        coordinate
        for point in (points[0], points[-1])
        for coordinate in (point.pose.position.x, point.pose.position.y)
    ]
    ok = frame == "map" and finite_values(planar)
    return ok, f"frame={frame!r}, points={len(points)}"


def validate_candidates(message: Any, _args: argparse.Namespace) -> tuple[bool, str]:
    candidates = list(getattr(message, "candidate_trajectories", ()))
    counts = [len(trajectory_points(candidate)) for candidate in candidates]
    ok = bool(counts) and all(count > 0 for count in counts)
    return ok, f"candidates={len(counts)}, points={counts}"


def validate_objects(message: Any, _args: argparse.Namespace) -> tuple[bool, str]:
    frame = message.header.frame_id
    objects = list(getattr(message, "objects", ()))
    return frame == "map", f"frame={frame!r}, objects={len(objects)}"


def validate_markers(message: Any, _args: argparse.Namespace) -> tuple[bool, str]:
    markers = list(getattr(message, "markers", ()))
    point_count = sum(len(getattr(marker, "points", ())) for marker in markers)
    ok = bool(markers) and point_count > 0
    return ok, f"markers={len(markers)}, points={point_count}"


def validate_pointcloud(message: Any, _args: argparse.Namespace) -> tuple[bool, str]:
    width = int(getattr(message, "width", 0))
    height = int(getattr(message, "height", 0))
    point_step = int(getattr(message, "point_step", 0))
    data_size = len(getattr(message, "data", ()))
    points = width * height
    ok = points > 0 and point_step > 0 and data_size >= points * point_step
    return ok, f"points={points}, point_step={point_step}, bytes={data_size}"


def validate_route_status(message: Any, _args: argparse.Namespace) -> tuple[bool, str]:
    status = str(message.data)
    ok = status in {"ready", "stopping", "goal_reached"}
    return ok, f"status={status!r}"


def validate_route_alignment(message: Any, _args: argparse.Namespace) -> tuple[bool, str]:
    aligned = bool(message.data)
    return aligned, f"aligned={aligned}"


def validate_true(message: Any, _args: argparse.Namespace) -> tuple[bool, str]:
    value = bool(message.data)
    return value, f"value={value}"


def validate_cross_track(message: Any, args: argparse.Namespace) -> tuple[bool, str]:
    value = float(message.data)
    ok = math.isfinite(value) and abs(value) <= args.max_cross_track
    return ok, f"abs_error={abs(value):.3f}m (limit={args.max_cross_track:.3f}m)"


def validate_trajectory_correction(message: Any, args: argparse.Namespace) -> tuple[bool, str]:
    value = float(message.data)
    ok = math.isfinite(value) and 0.0 <= value <= args.max_trajectory_correction
    return ok, f"correction={value:.3f}m (limit={args.max_trajectory_correction:.3f}m)"


def live_topics(args: argparse.Namespace) -> tuple[LiveTopic, ...]:
    minimum = args.minimum_live_messages
    return (
        LiveTopic("/clock", "CARLA simulation clock", validate_clock, minimum),
        LiveTopic(
            "/api/localization/initialization_state",
            "initialized standard localization state",
            validate_localization_initialized,
            1,
            transient_local=True,
        ),
        LiveTopic(
            "/api/operation_mode/state",
            "stable autonomous operation-mode availability",
            validate_operation_mode_ready,
            1,
            transient_local=True,
        ),
        LiveTopic(
            "/localization/kinematic_state",
            "CARLA truth localization",
            validate_odometry,
            minimum,
            check_sim_age=True,
        ),
        LiveTopic(
            "/localization/acceleration",
            "derived acceleration",
            validate_acceleration,
            minimum,
        ),
        LiveTopic(
            "/planning/vad/raw_trajectory",
            "raw VAD trajectory",
            validate_trajectory,
            minimum,
            check_sim_age=True,
        ),
        LiveTopic(
            "/planning/vad/candidate_trajectories",
            "VAD candidates",
            validate_candidates,
            minimum,
        ),
        LiveTopic(
            "/perception/object_recognition/objects",
            "VAD objects",
            validate_objects,
            minimum,
            check_sim_age=True,
        ),
        LiveTopic(
            "/sensing/lidar/top/pointcloud_before_sync",
            "raw CARLA LiDAR",
            validate_pointcloud,
            minimum,
            check_sim_age=True,
        ),
        LiveTopic("/perception/vad/map_points", "VAD map output", validate_markers, minimum),
        LiveTopic(
            "/planning/trajectory",
            "route-conditioned final trajectory",
            validate_trajectory,
            minimum,
            check_sim_age=True,
        ),
        LiveTopic("/planning/vad_route/status", "VAD route status", validate_route_status, minimum),
        LiveTopic(
            "/planning/vad_route/cross_track_error",
            "route cross-track alignment",
            validate_cross_track,
            minimum,
        ),
        LiveTopic(
            "/planning/vad_route/trajectory_correction",
            "VAD trajectory corridor correction",
            validate_trajectory_correction,
            minimum,
        ),
        LiveTopic(
            "/planning/vad_route/standard_route_aligned",
            "standard mission route alignment",
            validate_route_alignment,
            minimum,
            transient_local=True,
        ),
        LiveTopic(
            "/system/vad/aeb_configured",
            "standard AEB VAD-object configuration",
            validate_true,
            1,
            transient_local=True,
        ),
        LiveTopic(
            "/control/trajectory_follower/control_cmd",
            "standard trajectory follower command",
            validate_presence,
            minimum,
        ),
        LiveTopic(
            "/control/command/control_cmd",
            "standard gated control command",
            validate_presence,
            minimum,
        ),
        LiveTopic(
            "/control/command/actuation_cmd",
            "CARLA actuation command",
            validate_presence,
            minimum,
        ),
    )


def run_self_test() -> int:
    assert canonical_node("node", "/namespace/") == "/namespace/node"
    assert canonical_node("node", "/") == "/node"
    nodes = {"/map/pointcloud_map_loader", "/map/lanelet2_map_loader", "/other"}
    matches = matching_nodes(nodes, NODE_GROUPS[3][1])
    assert matches == ["/map/lanelet2_map_loader", "/map/pointcloud_map_loader"]
    assert finite_values([0.0, -1.0, 2.5])
    assert not finite_values([0.0, math.nan])

    class Value:
        data = 0.25

    args = argparse.Namespace(max_cross_track=1.0, max_trajectory_correction=2.0)
    assert validate_cross_track(Value(), args)[0]
    Value.data = 1.25
    assert not validate_cross_track(Value(), args)[0]
    initialized = type("Initialized", (), {"state": 3})()
    assert validate_localization_initialized(initialized, args)[0]
    initialized.state = 2
    assert not validate_localization_initialized(initialized, args)[0]
    operation = type(
        "Operation",
        (),
        {
            "mode": 1,
            "is_in_transition": False,
            "is_autonomous_mode_available": True,
            "is_autoware_control_enabled": False,
        },
    )()
    assert validate_operation_mode_ready(operation, args)[0]
    operation.is_in_transition = True
    assert not validate_operation_mode_ready(operation, args)[0]
    print("[PASS] validate_full_stack helper self-test")
    return 0


def graph_nodes(node: Any) -> list[str]:
    return sorted(
        canonical_node(name, namespace)
        for name, namespace in node.get_node_names_and_namespaces()
        if canonical_node(name, namespace) != "/autoware_e2e_full_stack_validator"
    )


def graph_is_ready(nodes: set[str], topics: set[str]) -> bool:
    core_nodes = {
        "/autoware_carla_interface",
        "/map/lanelet2_map_loader",
        "/control/vehicle_cmd_gate",
        "/vad_carla_tiny",
        "/vad_route_manager",
        "/vad_standard_route_adapter",
    }
    core_topics = {
        "/localization/kinematic_state",
        "/planning/vad/raw_trajectory",
        "/planning/trajectory",
        "/control/command/control_cmd",
    }
    return core_nodes <= nodes and core_topics <= topics


def wait_for_graph(node: Any, timeout: float) -> tuple[list[str], dict[str, list[str]]]:
    import rclpy

    deadline = time.monotonic() + timeout
    names: list[str] = []
    topic_types: dict[str, list[str]] = {}
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
        names = graph_nodes(node)
        topic_types = dict(node.get_topic_names_and_types())
        if graph_is_ready(set(names), set(topic_types)):
            return names, topic_types
    return names, topic_types


def check_nodes(report: Report, nodes: set[str]) -> None:
    for label, patterns, minimum in NODE_GROUPS:
        matches = matching_nodes(nodes, patterns)
        ok = len(matches) >= minimum
        detail = f"{len(matches)}/{minimum} required; " + (
            ", ".join(matches) if matches else "none"
        )
        report.add("Nodes", label, ok, detail)


def check_topic_groups(report: Report, node: Any, topic_types: dict[str, list[str]]) -> None:
    for label, topics in TOPIC_GROUPS.items():
        missing = []
        publisherless = []
        for topic in topics:
            if topic not in topic_types:
                missing.append(topic)
                continue
            if not node.get_publishers_info_by_topic(topic):
                publisherless.append(topic)
        ok = not missing and not publisherless
        details = []
        if missing:
            details.append("missing=" + ", ".join(missing))
        if publisherless:
            details.append("no publisher=" + ", ".join(publisherless))
        if not details:
            details.append(f"{len(topics)} topics have publishers")
        report.add("Topics", label, ok, "; ".join(details))

    missing = []
    bridge_missing = []
    republisher_missing = []
    vad_missing = []
    for camera in CAMERAS:
        raw_topic = f"/sensing/camera/{camera}/image_raw"
        compressed_topic = f"{raw_topic}/compressed"
        info_topic = f"/sensing/camera/{camera}/camera_info"
        for topic in (raw_topic, compressed_topic, info_topic):
            if topic not in topic_types:
                missing.append(topic)

        if raw_topic in topic_types:
            raw_publishers = {
                endpoint_node(info) for info in node.get_publishers_info_by_topic(raw_topic)
            }
            raw_subscribers = {
                endpoint_node(info) for info in node.get_subscriptions_info_by_topic(raw_topic)
            }
            if "/autoware_carla_interface" not in raw_publishers:
                bridge_missing.append(raw_topic)
            if not any(name.endswith("_republish") for name in raw_subscribers):
                republisher_missing.append(raw_topic)

        if compressed_topic in topic_types:
            compressed_publishers = {
                endpoint_node(info)
                for info in node.get_publishers_info_by_topic(compressed_topic)
            }
            compressed_subscribers = {
                endpoint_node(info)
                for info in node.get_subscriptions_info_by_topic(compressed_topic)
            }
            if not any(name.endswith("_republish") for name in compressed_publishers):
                republisher_missing.append(compressed_topic)
            if "/vad_carla_tiny" not in compressed_subscribers:
                vad_missing.append(compressed_topic)

        if info_topic in topic_types:
            info_publishers = {
                endpoint_node(info) for info in node.get_publishers_info_by_topic(info_topic)
            }
            info_subscribers = {
                endpoint_node(info) for info in node.get_subscriptions_info_by_topic(info_topic)
            }
            if "/autoware_carla_interface" not in info_publishers:
                bridge_missing.append(info_topic)
            if "/vad_carla_tiny" not in info_subscribers:
                vad_missing.append(info_topic)

    ok = not missing and not bridge_missing and not republisher_missing and not vad_missing
    details = []
    if missing:
        details.append("missing=" + ", ".join(missing))
    if bridge_missing:
        details.append("not bridge-owned=" + ", ".join(bridge_missing))
    if republisher_missing:
        details.append("compressed transport broken=" + ", ".join(republisher_missing))
    if vad_missing:
        details.append("VAD not subscribed=" + ", ".join(vad_missing))
    if not details:
        details.append("all 6 raw images pass through compressed transport to VAD; camera_info is direct")
    report.add("Topics", "six-camera VAD wiring", ok, "; ".join(details))


def check_ownership(report: Report, node: Any) -> None:
    for topic, label, owner_pattern in OWNERSHIP:
        endpoints = node.get_publishers_info_by_topic(topic)
        owners = sorted(endpoint_node(endpoint) for endpoint in endpoints)
        unique_owners = sorted(set(owners))
        ok = (
            len(endpoints) == 1
            and len(unique_owners) == 1
            and re.search(owner_pattern, unique_owners[0]) is not None
        )
        if not endpoints:
            detail = f"{topic}: no publisher"
        else:
            detail = (
                f"{topic}: endpoints={len(endpoints)}, owners="
                + ", ".join(unique_owners)
            )
        report.add("Ownership", label, ok, detail)


def check_subscriber_connections(report: Report, node: Any) -> None:
    for topic, label, patterns, minimum in SUBSCRIBER_CONNECTIONS:
        subscribers = {
            endpoint_node(endpoint)
            for endpoint in node.get_subscriptions_info_by_topic(topic)
        }
        matches = matching_nodes(subscribers, patterns)
        ok = len(matches) >= minimum
        detail = f"{topic}: {len(matches)}/{minimum} required consumers; " + (
            ", ".join(matches) if matches else "none"
        )
        report.add("Connections", label, ok, detail)


def make_callback(
    state: SampleState,
    spec: LiveTopic,
    args: argparse.Namespace,
) -> Callable[[Any], None]:
    def callback(message: Any) -> None:
        now = time.monotonic()
        state.total_count += 1
        if state.first_arrival is None:
            state.first_arrival = now
        state.last_arrival = now
        state.stamp_ns = header_stamp_ns(message)
        try:
            valid, summary = spec.validator(message, args)
        except Exception as error:  # Keep collecting in case a later message is valid.
            valid = False
            summary = f"validator error: {error}"
        state.summary = summary
        if valid:
            state.valid_count += 1
            state.last_error = ""
        else:
            state.last_error = summary

    return callback


def collect_live_samples(
    report: Report,
    node: Any,
    topic_types: dict[str, list[str]],
    args: argparse.Namespace,
) -> None:
    import rclpy
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    specs = live_topics(args)
    states = {spec.topic: SampleState() for spec in specs}
    subscriptions = []
    setup_failures: dict[str, str] = {}
    for spec in specs:
        types = topic_types.get(spec.topic, [])
        if len(types) != 1:
            setup_failures[spec.topic] = f"expected one message type, found {types}"
            continue
        try:
            message_type = get_message(types[0])
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=(
                    ReliabilityPolicy.RELIABLE
                    if spec.transient_local
                    else ReliabilityPolicy.BEST_EFFORT
                ),
                durability=(
                    DurabilityPolicy.TRANSIENT_LOCAL
                    if spec.transient_local
                    else DurabilityPolicy.VOLATILE
                ),
            )
            subscriptions.append(
                node.create_subscription(
                    message_type,
                    spec.topic,
                    make_callback(states[spec.topic], spec, args),
                    qos,
                )
            )
        except Exception as error:
            setup_failures[spec.topic] = str(error)

    deadline = time.monotonic() + args.message_timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if not setup_failures and all(
            states[spec.topic].valid_count >= spec.minimum for spec in specs
        ):
            break

    finished = time.monotonic()
    clock_state = states["/clock"]
    clock_ns = clock_state.stamp_ns

    for spec in specs:
        state = states[spec.topic]
        if spec.topic in setup_failures:
            report.add("Freshness", spec.label, False, setup_failures[spec.topic])
            continue
        age = math.inf if state.last_arrival is None else finished - state.last_arrival
        count_ok = state.valid_count >= spec.minimum
        age_ok = age <= args.max_wall_age
        sim_age = None
        sim_age_ok = True
        if spec.check_sim_age:
            if clock_ns is None or state.stamp_ns is None:
                sim_age_ok = False
            else:
                sim_age = abs(clock_ns - state.stamp_ns) / 1.0e9
                sim_age_ok = sim_age <= args.max_sim_age
        ok = count_ok and age_ok and sim_age_ok and not state.last_error
        detail = (
            f"valid={state.valid_count}/{state.total_count} "
            f"(required={spec.minimum}), wall_age={age:.2f}s, {state.summary}"
        )
        if sim_age is not None:
            detail += f", sim_age={sim_age:.3f}s"
        elif spec.check_sim_age:
            detail += ", sim_age=unavailable"
        if state.last_error:
            detail += f", last_invalid={state.last_error}"
        report.add("Freshness", spec.label, ok, detail)
        report.samples[spec.topic] = {
            "total_count": state.total_count,
            "valid_count": state.valid_count,
            "wall_age_sec": None if math.isinf(age) else age,
            "sim_age_sec": sim_age,
            "summary": state.summary,
            "last_error": state.last_error,
        }

    # Keep references alive until collection and graph inspection are complete.
    del subscriptions


def related_nodes(nodes: list[str]) -> list[str]:
    terms = ("carla", "vad", "control", "map", "mission", "adapi", "rviz")
    return [node for node in nodes if any(term in node.lower() for term in terms)]


def validate_runtime(args: argparse.Namespace) -> int:
    try:
        import rclpy
    except ImportError as error:
        print(f"[FAIL] ROS 2 Python environment is unavailable: {error}", file=sys.stderr)
        return 2

    rclpy.init(args=[])
    node = rclpy.create_node("autoware_e2e_full_stack_validator")
    report = Report()
    try:
        print(
            f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '<unset>')} - "
            "read-only graph and subscription validation; CARLA world/tick are untouched."
        )
        nodes, topic_types = wait_for_graph(node, args.graph_timeout)
        report.nodes = nodes
        if not graph_is_ready(set(nodes), set(topic_types)):
            observed = related_nodes(nodes)
            detail = (
                "full stack was not detected before timeout; start "
                "scripts/e2e/run_route_vad_full.sh ROUTE_JSON, then rerun this validator. "
                f"Observed {len(nodes)} nodes"
            )
            if observed:
                detail += ": " + ", ".join(observed[:20])
            else:
                detail += "; no related CARLA/Autoware/VAD nodes were discovered"
            report.add("Runtime", "full stack discovery", False, detail)
            report.emit(args.verbose)
            if args.json:
                report.write_json(args.json)
            return 1


        report.add(
            "Runtime",
            "full stack discovery",
            True,
            f"{len(nodes)} nodes and {len(topic_types)} topics discovered",
        )
        check_nodes(report, set(nodes))
        check_topic_groups(report, node, topic_types)
        check_ownership(report, node)
        check_subscriber_connections(report, node)
        collect_live_samples(report, node, topic_types, args)
        report.emit(args.verbose)
        if args.json:
            report.write_json(args.json)
        return 0 if not report.failures else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test()
    try:
        bootstrap_environment()
    except RuntimeError as error:
        print(f"[FAIL] {error}", file=sys.stderr)
        return 2
    return validate_runtime(args)


if __name__ == "__main__":
    raise SystemExit(main())
