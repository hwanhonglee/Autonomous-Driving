#!/usr/bin/env python3
"""Fail-closed pre-engagement runtime health probe for CARLA VAD trials.

The pure evaluation functions in this module keep wall receipt time and ROS
simulation stamps in separate clock domains.  The live probe only subscribes
to clock/camera-info and reads ROS graph metadata; it does not change publisher
QoS, advance CARLA, record a bag, or engage the vehicle.
"""

from __future__ import annotations

import argparse
import bisect
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import tempfile
import time
from typing import Any, Mapping, Sequence


CAMERAS = (
    "CAM_FRONT",
    "CAM_BACK",
    "CAM_FRONT_LEFT",
    "CAM_BACK_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
)
CAMERA_INFO_TOPICS = tuple(
    f"/sensing/camera/{camera}/camera_info" for camera in CAMERAS
)
CAMERA_IMAGE_TOPICS = tuple(
    f"/sensing/camera/{camera}/image_raw" for camera in CAMERAS
)
CLOCK_TOPIC = "/clock"
SCHEMA_VERSION = 1
PROBE_ID = "pre_engagement_runtime_health_v1"

CAMERA_TRANSPORT_PROFILE_V1 = (
    "carla_vad_camera_source_5hz_best_effort_image_v1"
)
CAMERA_TRANSPORT_PROFILE_V2 = (
    "carla_vad_camera_source_5hz_best_effort_image_v2"
)
# HH_260906 - Bind the Portable 10 Hz shadow profile to the exact loopback image graph gate.
CAMERA_TRANSPORT_PROFILE_PORTABLE_10HZ = "portable_e2e_exact_bundle_10hz_v2"
# HH_260906 - Qualify the same six-camera transport ABI without loading the Portable model.
CAMERA_TRANSPORT_PROFILE_STRICT_10HZ = "carla_vad_camera_source_10hz_strict_v2"
EXACT_CAMERA_GRAPH_PROFILES = {
    CAMERA_TRANSPORT_PROFILE_V2,
    CAMERA_TRANSPORT_PROFILE_PORTABLE_10HZ,
    CAMERA_TRANSPORT_PROFILE_STRICT_10HZ,
}
TEN_HZ_CAMERA_PROFILES = {
    CAMERA_TRANSPORT_PROFILE_PORTABLE_10HZ,
    CAMERA_TRANSPORT_PROFILE_STRICT_10HZ,
}
CAMERA_GRAPH_DISCOVERY_TIMEOUT_SECONDS = 5.0
CAMERA_GRAPH_POLL_SECONDS = 0.1
CAMERA_GRAPH_REQUIRED_CONSECUTIVE_PASSES = 3
EXPECTED_CAMERA_IMAGE_PUBLISHER = "/autoware_carla_interface"
EXPECTED_VAD_IMAGE_SUBSCRIBER = "/vad_carla_tiny"
EXPECTED_RVIZ_IMAGE_SUBSCRIBER = "/autoware_vad_rviz"

DEFAULT_WINDOW_SECONDS = 8.0
DEFAULT_TIMEOUT_SECONDS = 45.0
EVALUATION_INTERVAL_SECONDS = 1.0
BUNDLE_SETTLE_SECONDS = 0.1
BUNDLE_MATCH_TOLERANCE_SECONDS = 0.1
# HH_260906 - Keep strict 10 Hz source-stamp matching far below one 100 ms frame period.
STRICT_10HZ_BUNDLE_MATCH_TOLERANCE_SECONDS = 0.000005
REQUIRED_CONSECUTIVE_PASSES = 3

MINIMUM_RTF = 0.9
MINIMUM_CAMERA_WALL_RATE_HZ = 4.0
MINIMUM_COMPLETE_BUNDLES = 20
# HH_260906 - Require near-10 Hz camera delivery throughout each Portable eight-second gate.
PORTABLE_MINIMUM_CAMERA_WALL_RATE_HZ = 9.0
PORTABLE_MINIMUM_COMPLETE_BUNDLES = 70
MINIMUM_BUNDLE_COVERAGE_PERCENT = 99.0
MAXIMUM_BUNDLE_RECEIPT_P95_SECONDS = 0.040
# HH_260906 - Reject strict bundles unless all six source stamps agree within five microseconds.
STRICT_10HZ_MAXIMUM_BUNDLE_STAMP_SPAN_SECONDS = 0.000005
# HH_260906 - Bound both receipt and source cadence before a strict 60 kph engagement.
STRICT_10HZ_MAXIMUM_CAMERA_WALL_RATE_HZ = 11.0
STRICT_10HZ_MINIMUM_CAMERA_SOURCE_RATE_HZ = 9.5
STRICT_10HZ_MAXIMUM_CAMERA_SOURCE_RATE_HZ = 10.5
# HH_260906 - Mirror the five-microsecond upper tolerance around the 100 ms source period.
STRICT_10HZ_MINIMUM_CAMERA_SOURCE_GAP_SECONDS = 0.099995
STRICT_10HZ_MAXIMUM_CAMERA_SOURCE_GAP_SECONDS = 0.100005
PORTABLE_SHADOW_NODE = "/portable_e2e_shadow"
PORTABLE_OUTPUT_TOPICS = (
    "/planning/portable_e2e/shadow_trajectory",
    "/planning/portable_e2e/shadow_path",
    "/planning/portable_e2e/status",
    "/planning/portable_e2e/latency_ms",
    "/planning/portable_e2e/selected_candidate",
)


class RuntimeHealthError(RuntimeError):
    """Raised when the live health probe cannot safely collect evidence."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def owned_process_record(pid: int, pgid: int) -> dict[str, Any]:
    """Prove that an optional load-generating process remains in its owned PGID."""
    try:
        stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    except OSError as error:
        raise RuntimeHealthError(f"owned RViz recorder {pid} is unavailable: {error}") from error
    close = stat.rfind(")")
    fields = stat[close + 2:].split() if close >= 0 else []
    state = fields[0] if fields else None
    if state is None or state.startswith("Z"):
        raise RuntimeHealthError(
            f"owned RViz recorder {pid} has invalid process state {state!r}"
        )
    try:
        actual_pgid = os.getpgid(pid)
    except OSError as error:
        raise RuntimeHealthError(f"cannot inspect RViz recorder {pid}: {error}") from error
    if actual_pgid != pgid:
        raise RuntimeHealthError(
            "owned RViz recorder process-group mismatch: "
            f"expected={pgid} actual={actual_pgid}"
        )
    return {"pid": pid, "pgid": pgid, "process_state": state}


def atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, staged_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".staged", dir=path.parent
    )
    staged = Path(staged_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(staged, path)
    finally:
        staged.unlink(missing_ok=True)


def default_contract(window_seconds: float = DEFAULT_WINDOW_SECONDS) -> dict[str, Any]:
    """Return a fresh, JSON-serializable copy of the fixed gate contract."""
    return {
        "window_seconds": float(window_seconds),
        "evaluation_interval_seconds": EVALUATION_INTERVAL_SECONDS,
        "bundle_settle_seconds": BUNDLE_SETTLE_SECONDS,
        "bundle_match_tolerance_seconds": BUNDLE_MATCH_TOLERANCE_SECONDS,
        "required_consecutive_passes": REQUIRED_CONSECUTIVE_PASSES,
        "thresholds": {
            "minimum_rtf": MINIMUM_RTF,
            "minimum_camera_wall_rate_hz": MINIMUM_CAMERA_WALL_RATE_HZ,
            "minimum_complete_bundle_count": MINIMUM_COMPLETE_BUNDLES,
            "minimum_bundle_coverage_percent": MINIMUM_BUNDLE_COVERAGE_PERCENT,
            "maximum_bundle_receipt_p95_seconds": (
                MAXIMUM_BUNDLE_RECEIPT_P95_SECONDS
            ),
        },
        "topics": {
            "clock": CLOCK_TOPIC,
            "camera_info": list(CAMERA_INFO_TOPICS),
        },
        "clock_domains": {
            "rtf_numerator": "ROS /clock simulation time delta",
            "rtf_denominator": "monotonic wall window duration",
            "camera_rate": "monotonic wall receipt count per window",
            "bundle_receipt_span": "monotonic wall receipt span within bundle",
            "bundle_matching": "camera_info header stamp",
            "cross_domain_subtraction_used": False,
        },
    }


def apply_camera_profile_contract(
    contract: dict[str, Any], profile_id: str | None
) -> dict[str, Any]:
    """HH_260906 - Apply profile-specific cadence and synchronization gates."""
    if profile_id in TEN_HZ_CAMERA_PROFILES:
        contract["thresholds"]["minimum_camera_wall_rate_hz"] = (
            PORTABLE_MINIMUM_CAMERA_WALL_RATE_HZ
        )
        contract["thresholds"]["minimum_complete_bundle_count"] = (
            PORTABLE_MINIMUM_COMPLETE_BUNDLES
        )
    if profile_id == CAMERA_TRANSPORT_PROFILE_STRICT_10HZ:
        # HH_260906 - A one-frame-shifted stream must never qualify as a strict bundle.
        contract["strict_camera_source_integrity_required"] = True
        contract["bundle_match_tolerance_seconds"] = (
            STRICT_10HZ_BUNDLE_MATCH_TOLERANCE_SECONDS
        )
        contract["thresholds"]["maximum_bundle_stamp_span_seconds"] = (
            STRICT_10HZ_MAXIMUM_BUNDLE_STAMP_SPAN_SECONDS
        )
        # HH_260906 - Reject duplicated, accelerated, or gapped delivery before engagement.
        contract["thresholds"]["maximum_camera_wall_rate_hz"] = (
            STRICT_10HZ_MAXIMUM_CAMERA_WALL_RATE_HZ
        )
        contract["thresholds"]["minimum_camera_source_rate_hz"] = (
            STRICT_10HZ_MINIMUM_CAMERA_SOURCE_RATE_HZ
        )
        contract["thresholds"]["maximum_camera_source_rate_hz"] = (
            STRICT_10HZ_MAXIMUM_CAMERA_SOURCE_RATE_HZ
        )
        contract["thresholds"]["minimum_camera_source_gap_seconds"] = (
            STRICT_10HZ_MINIMUM_CAMERA_SOURCE_GAP_SECONDS
        )
        contract["thresholds"]["maximum_camera_source_gap_seconds"] = (
            STRICT_10HZ_MAXIMUM_CAMERA_SOURCE_GAP_SECONDS
        )
    return contract


def evaluate_portable_runtime_absence(
    nodes: Sequence[str],
    output_publishers: Mapping[str, Sequence[Mapping[str, Any]]],
) -> dict[str, Any]:
    """HH_260906 - Prove Portable node and output-publisher absence."""
    observed_nodes = sorted({str(node) for node in nodes})
    portable_nodes = [node for node in observed_nodes if node == PORTABLE_SHADOW_NODE]
    observed_publishers = {
        topic: sorted(
            (dict(endpoint) for endpoint in output_publishers.get(topic, ())),
            key=lambda endpoint: (
                str(endpoint.get("node")),
                str(endpoint.get("endpoint_gid_hex")),
            ),
        )
        for topic in PORTABLE_OUTPUT_TOPICS
    }
    published_topics = [
        topic for topic, endpoints in observed_publishers.items() if endpoints
    ]
    failures: list[dict[str, Any]] = []
    if portable_nodes:
        failures.append(
            {
                "check": "portable_shadow_node_absent",
                "actual": portable_nodes,
                "expected": [],
            }
        )
    if published_topics:
        failures.append(
            {
                "check": "portable_output_publishers_absent",
                "actual": published_topics,
                "expected": [],
            }
        )
    return {
        "status": "PASS" if not failures else "FAIL",
        "required_absent_node": PORTABLE_SHADOW_NODE,
        "required_absent_output_topics": list(PORTABLE_OUTPUT_TOPICS),
        "observed_portable_nodes": portable_nodes,
        "observed_output_publishers": observed_publishers,
        "failures": failures,
    }


def collect_portable_runtime_absence(node: Any) -> dict[str, Any]:
    """HH_260906 - Capture a graph snapshot for the non-Portable claim."""
    nodes = [
        _fully_qualified_node_name(name, namespace)
        for name, namespace in node.get_node_names_and_namespaces()
    ]
    publishers = {
        topic: [
            serialize_topic_endpoint(endpoint)
            for endpoint in node.get_publishers_info_by_topic(topic)
        ]
        for topic in PORTABLE_OUTPUT_TOPICS
    }
    report = evaluate_portable_runtime_absence(nodes, publishers)
    # HH_260906 - Label this evidence as a point sample, not continuous observation.
    report["observation_scope"] = "window_end_graph_snapshot"
    report["continuous_window_observation"] = False
    report["observed_monotonic_seconds"] = time.monotonic()
    return report


def _policy_name(value: Any) -> str:
    """Return a stable lower-case ROS QoS policy name."""
    name = getattr(value, "name", None)
    text = str(name if isinstance(name, str) and name else value)
    return text.rsplit(".", 1)[-1].strip().lower().replace(" ", "_")


def _fully_qualified_node_name(node_name: Any, node_namespace: Any) -> str:
    name = str(node_name or "").strip("/")
    namespace = str(node_namespace or "/").strip("/")
    parts = [part for part in (namespace, name) if part]
    return "/" + "/".join(parts)


def serialize_topic_endpoint(endpoint: Any) -> dict[str, Any]:
    """Convert ``TopicEndpointInfo`` into stable JSON evidence."""
    qos = endpoint.qos_profile
    endpoint_gid = getattr(endpoint, "endpoint_gid", b"")
    try:
        gid_hex = bytes(endpoint_gid).hex()
    except (TypeError, ValueError):
        gid_hex = ""
    return {
        "node": _fully_qualified_node_name(
            getattr(endpoint, "node_name", ""),
            getattr(endpoint, "node_namespace", "/"),
        ),
        "topic_type": str(getattr(endpoint, "topic_type", "")),
        "endpoint_gid_hex": gid_hex,
        "qos": {
            "reliability": _policy_name(qos.reliability),
            "durability": _policy_name(qos.durability),
            "history": _policy_name(qos.history),
            "depth": int(qos.depth),
        },
    }


def expected_camera_image_graph() -> dict[str, Any]:
    """Return the exact six-camera image graph required by bounded profiles."""
    topics: dict[str, Any] = {}
    for topic in CAMERA_IMAGE_TOPICS:
        subscribers = [EXPECTED_VAD_IMAGE_SUBSCRIBER]
        if topic == CAMERA_IMAGE_TOPICS[0]:
            subscribers.append(EXPECTED_RVIZ_IMAGE_SUBSCRIBER)
        topics[topic] = {
            "publishers": [EXPECTED_CAMERA_IMAGE_PUBLISHER],
            "subscriptions": sorted(subscribers),
        }
    return {
        "topics": topics,
        "topic_type": "sensor_msgs/msg/Image",
        "qos": {
            "reliability": "best_effort",
            "durability": "volatile",
            "history": "keep_last",
            "depth": 1,
        },
        "required_consecutive_passes": CAMERA_GRAPH_REQUIRED_CONSECUTIVE_PASSES,
    }


def evaluate_camera_image_graph(
    topics: Mapping[str, Mapping[str, Sequence[Mapping[str, Any]]]],
) -> dict[str, Any]:
    """Fail closed unless the image graph is exact and every endpoint is depth 1."""
    expected = expected_camera_image_graph()
    failures: list[dict[str, Any]] = []
    normalized_topics: dict[str, Any] = {}
    for topic, expected_topic in expected["topics"].items():
        observed_topic = topics.get(topic, {})
        publishers = sorted(
            (dict(item) for item in observed_topic.get("publishers", ())),
            key=lambda item: (str(item.get("node")), str(item.get("endpoint_gid_hex"))),
        )
        subscriptions = sorted(
            (dict(item) for item in observed_topic.get("subscriptions", ())),
            key=lambda item: (str(item.get("node")), str(item.get("endpoint_gid_hex"))),
        )
        normalized_topics[topic] = {
            "publishers": publishers,
            "subscriptions": subscriptions,
        }
        for endpoint_kind, endpoints in (
            ("publishers", publishers),
            ("subscriptions", subscriptions),
        ):
            actual_nodes = sorted(str(item.get("node")) for item in endpoints)
            expected_nodes = expected_topic[endpoint_kind]
            if actual_nodes != expected_nodes:
                failures.append(
                    {
                        "check": "endpoint_nodes",
                        "topic": topic,
                        "endpoint_kind": endpoint_kind,
                        "actual": actual_nodes,
                        "expected": expected_nodes,
                    }
                )
            for endpoint in endpoints:
                if endpoint.get("topic_type") != expected["topic_type"]:
                    failures.append(
                        {
                            "check": "endpoint_topic_type",
                            "topic": topic,
                            "endpoint_kind": endpoint_kind,
                            "node": endpoint.get("node"),
                            "actual": endpoint.get("topic_type"),
                            "expected": expected["topic_type"],
                        }
                    )
                qos = endpoint.get("qos")
                if not isinstance(qos, Mapping):
                    failures.append(
                        {
                            "check": "endpoint_qos",
                            "topic": topic,
                            "endpoint_kind": endpoint_kind,
                            "node": endpoint.get("node"),
                            "actual": qos,
                            "expected": expected["qos"],
                        }
                    )
                    continue
                for field, expected_value in expected["qos"].items():
                    actual_value = qos.get(field)
                    if actual_value != expected_value:
                        failures.append(
                            {
                                "check": f"endpoint_qos_{field}",
                                "topic": topic,
                                "endpoint_kind": endpoint_kind,
                                "node": endpoint.get("node"),
                                "actual": actual_value,
                                "expected": expected_value,
                            }
                        )
    unexpected_topics = sorted(set(topics) - set(CAMERA_IMAGE_TOPICS))
    if unexpected_topics:
        failures.append(
            {
                "check": "unexpected_topic_records",
                "actual": unexpected_topics,
                "expected": [],
            }
        )
    return {
        "status": "PASS" if not failures else "FAIL",
        "expected": expected,
        "topics": normalized_topics,
        "failures": failures,
    }


def collect_camera_image_graph(node: Any) -> dict[str, Any]:
    """Require three consecutive exact graph snapshots before cadence probing."""
    started = time.monotonic()
    deadline = started + CAMERA_GRAPH_DISCOVERY_TIMEOUT_SECONDS
    attempts: list[dict[str, Any]] = []
    consecutive_passes = 0
    maximum_consecutive_passes = 0
    final_evaluation: dict[str, Any] | None = None
    while True:
        topics = {
            topic: {
                "publishers": [
                    serialize_topic_endpoint(endpoint)
                    for endpoint in node.get_publishers_info_by_topic(topic)
                ],
                "subscriptions": [
                    serialize_topic_endpoint(endpoint)
                    for endpoint in node.get_subscriptions_info_by_topic(topic)
                ],
            }
            for topic in CAMERA_IMAGE_TOPICS
        }
        final_evaluation = evaluate_camera_image_graph(topics)
        if final_evaluation["status"] == "PASS":
            consecutive_passes += 1
        else:
            consecutive_passes = 0
        maximum_consecutive_passes = max(
            maximum_consecutive_passes, consecutive_passes
        )
        attempts.append(
            {
                "index": len(attempts),
                "elapsed_wall_seconds": time.monotonic() - started,
                "status": final_evaluation["status"],
                "failures": final_evaluation["failures"],
            }
        )
        if consecutive_passes >= CAMERA_GRAPH_REQUIRED_CONSECUTIVE_PASSES:
            break
        if time.monotonic() >= deadline:
            break
        import rclpy

        rclpy.spin_once(node, timeout_sec=CAMERA_GRAPH_POLL_SECONDS)
    assert final_evaluation is not None
    passed = consecutive_passes >= CAMERA_GRAPH_REQUIRED_CONSECUTIVE_PASSES
    return {
        "status": "PASS" if passed else "FAIL",
        "discovery_timeout_seconds": CAMERA_GRAPH_DISCOVERY_TIMEOUT_SECONDS,
        "poll_seconds": CAMERA_GRAPH_POLL_SECONDS,
        "required_consecutive_passes": CAMERA_GRAPH_REQUIRED_CONSECUTIVE_PASSES,
        "maximum_consecutive_passes": maximum_consecutive_passes,
        "attempt_count": len(attempts),
        "elapsed_wall_seconds": time.monotonic() - started,
        "attempts": attempts,
        "final": final_evaluation,
    }


def percentile(values: Sequence[float], percentile_value: float) -> float | None:
    """Return a linearly interpolated percentile without a NumPy dependency."""
    finite = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not finite:
        return None
    if not 0.0 <= percentile_value <= 100.0:
        raise ValueError("percentile must be in [0, 100]")
    position = (len(finite) - 1) * percentile_value / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return finite[lower]
    fraction = position - lower
    return finite[lower] * (1.0 - fraction) + finite[upper] * fraction


def _window_records(
    records: Sequence[Mapping[str, Any]], start: float, end: float
) -> list[Mapping[str, Any]]:
    return sorted(
        (
            record
            for record in records
            if start <= float(record["wall_time_sec"]) <= end
        ),
        key=lambda record: float(record["wall_time_sec"]),
    )


def _nearest_unused_record(
    records: Sequence[Mapping[str, Any]],
    stamps: Sequence[int],
    target_stamp: int,
    used: set[int],
    tolerance_ns: int,
) -> tuple[int, Mapping[str, Any]] | None:
    position = bisect.bisect_left(stamps, target_stamp)
    candidates: list[int] = []
    left = position - 1
    while left >= 0 and target_stamp - stamps[left] <= tolerance_ns:
        if left not in used:
            candidates.append(left)
        left -= 1
    right = position
    while right < len(stamps) and stamps[right] - target_stamp <= tolerance_ns:
        if right not in used:
            candidates.append(right)
        right += 1
    if not candidates:
        return None
    index = min(candidates, key=lambda item: abs(stamps[item] - target_stamp))
    return index, records[index]


def camera_bundle_metrics(
    camera_samples: Mapping[str, Sequence[Mapping[str, Any]]],
    window_start: float,
    window_end: float,
    match_tolerance_seconds: float = BUNDLE_MATCH_TOLERANCE_SECONDS,
    wall_edge_guard_seconds: float | None = None,
) -> dict[str, Any]:
    """Match six camera_info streams once each and measure wall receipt skew."""
    tolerance_ns = int(round(match_tolerance_seconds * 1.0e9))
    edge_guard_seconds = (
        match_tolerance_seconds
        if wall_edge_guard_seconds is None
        else wall_edge_guard_seconds
    )
    extended = {
        topic: sorted(
            (
                record
                for record in camera_samples.get(topic, ())
                if window_start - edge_guard_seconds
                <= float(record["wall_time_sec"])
                <= window_end + edge_guard_seconds
                and int(record.get("stamp_ns", 0)) > 0
            ),
            key=lambda record: int(record["stamp_ns"]),
        )
        for topic in CAMERA_INFO_TOPICS
    }
    front_records = _window_records(
        camera_samples.get(CAMERA_INFO_TOPICS[0], ()), window_start, window_end
    )
    anchors = [
        record
        for record in extended[CAMERA_INFO_TOPICS[0]]
        if window_start <= float(record["wall_time_sec"]) <= window_end
    ]
    used = {topic: set() for topic in CAMERA_INFO_TOPICS[1:]}
    stamps = {
        topic: [int(record["stamp_ns"]) for record in records]
        for topic, records in extended.items()
    }
    receipt_spans: list[float] = []
    stamp_spans: list[float] = []
    for anchor in anchors:
        bundle = [anchor]
        selected: list[tuple[str, int]] = []
        anchor_stamp = int(anchor["stamp_ns"])
        for topic in CAMERA_INFO_TOPICS[1:]:
            nearest = _nearest_unused_record(
                extended[topic],
                stamps[topic],
                anchor_stamp,
                used[topic],
                tolerance_ns,
            )
            if nearest is None:
                break
            index, record = nearest
            selected.append((topic, index))
            bundle.append(record)
        if len(bundle) != len(CAMERA_INFO_TOPICS):
            continue
        for topic, index in selected:
            used[topic].add(index)
        receipt_times = [float(record["wall_time_sec"]) for record in bundle]
        bundle_stamps = [int(record["stamp_ns"]) for record in bundle]
        receipt_spans.append(max(receipt_times) - min(receipt_times))
        stamp_spans.append((max(bundle_stamps) - min(bundle_stamps)) * 1.0e-9)

    complete_count = len(receipt_spans)
    # A zero/invalid front stamp is still a delivered frame and therefore
    # remains in the coverage denominator, but it cannot anchor a bundle.
    anchor_count = len(front_records)
    coverage = 100.0 * complete_count / anchor_count if anchor_count else 0.0
    return {
        "front_anchor_count": anchor_count,
        "complete_bundle_count": complete_count,
        "coverage_percent": coverage,
        "receipt_span_seconds": {
            "count": complete_count,
            "p95": percentile(receipt_spans, 95.0),
            "maximum": max(receipt_spans) if receipt_spans else None,
        },
        "stamp_span_seconds": {
            "count": complete_count,
            "p95": percentile(stamp_spans, 95.0),
            "maximum": max(stamp_spans) if stamp_spans else None,
        },
        "match_tolerance_seconds": match_tolerance_seconds,
        "wall_edge_guard_seconds": edge_guard_seconds,
        "matching_policy": "front_anchor_nearest_stamp_one_to_one",
    }


def strict_camera_source_integrity(
    camera_samples: Mapping[str, Sequence[Mapping[str, Any]]],
    window_start: float,
    window_end: float,
    match_tolerance_seconds: float,
    wall_edge_guard_seconds: float = BUNDLE_SETTLE_SECONDS,
) -> dict[str, Any]:
    """HH_260906 - Validate source stamps while preserving wall-window edges."""
    tolerance_ns = int(round(match_tolerance_seconds * 1.0e9))
    window_records = {
        topic: _window_records(
            camera_samples.get(topic, ()), window_start, window_end
        )
        for topic in CAMERA_INFO_TOPICS
    }
    front_window = window_records[CAMERA_INFO_TOPICS[0]]
    positive_front_stamps = [
        int(record.get("stamp_ns", 0))
        for record in front_window
        if int(record.get("stamp_ns", 0)) > 0
    ]
    lower_stamp = min(positive_front_stamps) if positive_front_stamps else None
    upper_stamp = max(positive_front_stamps) if positive_front_stamps else None

    # HH_260906 - Compare complete source-stamp ranges across serial wall-edge receipt.
    # HH_260906 - Require every in-range record to be consumed without false edge drops.
    source_records: dict[str, list[Mapping[str, Any]]] = {}
    for topic in CAMERA_INFO_TOPICS:
        if lower_stamp is None or upper_stamp is None:
            source_records[topic] = []
            continue
        source_records[topic] = sorted(
            (
                record
                for record in camera_samples.get(topic, ())
                if int(record.get("stamp_ns", 0)) > 0
                and lower_stamp - tolerance_ns
                <= int(record["stamp_ns"])
                <= upper_stamp + tolerance_ns
            ),
            key=lambda record: float(record["wall_time_sec"]),
        )

    # HH_260906 - Complete edge-straddling bundles from a bounded wall extension.
    extended_records: dict[str, list[dict[str, Any]]] = {}
    for topic in CAMERA_INFO_TOPICS:
        extended_records[topic] = sorted(
            (
                {
                    "sample_index": sample_index,
                    "wall_time_sec": float(record["wall_time_sec"]),
                    "stamp_ns": int(record.get("stamp_ns", 0)),
                }
                for sample_index, record in enumerate(
                    camera_samples.get(topic, ())
                )
                if window_start - wall_edge_guard_seconds
                <= float(record["wall_time_sec"])
                <= window_end + wall_edge_guard_seconds
            ),
            key=lambda record: int(record["stamp_ns"]),
        )
    positive_extended_records = {
        topic: [
            record
            for record in extended_records[topic]
            if int(record["stamp_ns"]) > 0
        ]
        for topic in CAMERA_INFO_TOPICS
    }
    extended_stamps = {
        topic: [int(record["stamp_ns"]) for record in records]
        for topic, records in positive_extended_records.items()
    }
    extended_used_indexes = {
        topic: set() for topic in CAMERA_INFO_TOPICS[1:]
    }
    used_sample_indexes = {topic: set() for topic in CAMERA_INFO_TOPICS}
    for anchor in positive_extended_records[CAMERA_INFO_TOPICS[0]]:
        anchor_stamp = int(anchor["stamp_ns"])
        selected: list[tuple[str, int]] = []
        for topic in CAMERA_INFO_TOPICS[1:]:
            nearest = _nearest_unused_record(
                positive_extended_records[topic],
                extended_stamps[topic],
                anchor_stamp,
                extended_used_indexes[topic],
                tolerance_ns,
            )
            if nearest is None:
                break
            index, _record = nearest
            selected.append((topic, index))
        if len(selected) != len(CAMERA_INFO_TOPICS) - 1:
            continue
        used_sample_indexes[CAMERA_INFO_TOPICS[0]].add(
            int(anchor["sample_index"])
        )
        for topic, index in selected:
            extended_used_indexes[topic].add(index)
            used_sample_indexes[topic].add(
                int(positive_extended_records[topic][index]["sample_index"])
            )

    # HH_260906 - Every receipt owned by the wall window must belong to one complete bundle.
    window_unmatched_counts: dict[str, int] = {}
    for topic in CAMERA_INFO_TOPICS:
        window_sample_indexes = {
            int(record["sample_index"])
            for record in extended_records[topic]
            if window_start <= float(record["wall_time_sec"]) <= window_end
        }
        window_unmatched_counts[topic] = len(
            window_sample_indexes - used_sample_indexes[topic]
        )

    topics: dict[str, dict[str, Any]] = {}
    for topic in CAMERA_INFO_TOPICS:
        arrival_stamps = [
            int(record.get("stamp_ns", 0)) for record in window_records[topic]
        ]
        positive_arrival_stamps = [stamp for stamp in arrival_stamps if stamp > 0]
        ranged_stamps = [int(record["stamp_ns"]) for record in source_records[topic]]
        duplicate_count = len(ranged_stamps) - len(set(ranged_stamps))
        non_increasing_count = sum(
            current <= previous
            for previous, current in zip(
                ranged_stamps, ranged_stamps[1:]
            )
        )
        ranged_deltas = [
            current - previous
            for previous, current in zip(ranged_stamps, ranged_stamps[1:])
        ]
        strictly_increasing = (
            bool(ranged_stamps)
            and len(positive_arrival_stamps) == len(arrival_stamps)
            and duplicate_count == 0
            and non_increasing_count == 0
        )
        source_rate_hz = None
        if (
            len(ranged_stamps) >= 2
            and all(delta > 0 for delta in ranged_deltas)
            and ranged_stamps[-1] > ranged_stamps[0]
        ):
            source_rate_hz = (len(ranged_stamps) - 1) * 1.0e9 / (
                ranged_stamps[-1] - ranged_stamps[0]
            )
        topics[topic] = {
            "window_record_count": len(arrival_stamps),
            "source_range_record_count": len(ranged_stamps),
            "positive_window_stamp_count": len(positive_arrival_stamps),
            "zero_or_negative_window_stamp_count": (
                len(arrival_stamps) - len(positive_arrival_stamps)
            ),
            "duplicate_positive_source_range_stamp_count": duplicate_count,
            "non_increasing_source_range_arrival_stamp_delta_count": (
                non_increasing_count
            ),
            "strictly_increasing_unique_positive_arrival_stamps": (
                strictly_increasing
            ),
            "source_rate_hz_from_span": source_rate_hz,
            "minimum_source_gap_seconds": (
                min(ranged_deltas) * 1.0e-9 if ranged_deltas else None
            ),
            "maximum_source_gap_seconds": (
                max(ranged_deltas) * 1.0e-9 if ranged_deltas else None
            ),
            "window_matched_record_count": (
                len(arrival_stamps) - window_unmatched_counts[topic]
            ),
            "window_unmatched_record_count": window_unmatched_counts[topic],
        }

    matching_records = {
        topic: sorted(
            source_records[topic], key=lambda record: int(record["stamp_ns"])
        )
        for topic in CAMERA_INFO_TOPICS
    }
    matching_stamps = {
        topic: [int(record["stamp_ns"]) for record in records]
        for topic, records in matching_records.items()
    }
    used = {topic: set() for topic in CAMERA_INFO_TOPICS[1:]}
    matched_bundle_count = 0
    bundle_stamp_spans: list[float] = []
    for anchor in matching_records[CAMERA_INFO_TOPICS[0]]:
        anchor_stamp = int(anchor["stamp_ns"])
        bundle_stamps = [anchor_stamp]
        selected: list[tuple[str, int]] = []
        for topic in CAMERA_INFO_TOPICS[1:]:
            nearest = _nearest_unused_record(
                matching_records[topic],
                matching_stamps[topic],
                anchor_stamp,
                used[topic],
                tolerance_ns,
            )
            if nearest is None:
                break
            index, record = nearest
            selected.append((topic, index))
            bundle_stamps.append(int(record["stamp_ns"]))
        if len(bundle_stamps) != len(CAMERA_INFO_TOPICS):
            continue
        for topic, index in selected:
            used[topic].add(index)
        matched_bundle_count += 1
        bundle_stamp_spans.append(
            (max(bundle_stamps) - min(bundle_stamps)) * 1.0e-9
        )

    record_counts = [len(source_records[topic]) for topic in CAMERA_INFO_TOPICS]
    record_count_parity = bool(record_counts) and len(set(record_counts)) == 1
    all_records_used_once = (
        matched_bundle_count == len(source_records[CAMERA_INFO_TOPICS[0]])
        and all(
            len(used[topic]) == len(source_records[topic])
            for topic in CAMERA_INFO_TOPICS[1:]
        )
    )
    all_topic_stamps_valid = all(
        item["strictly_increasing_unique_positive_arrival_stamps"]
        for item in topics.values()
    )
    all_window_records_used_once = all(
        count == 0 for count in window_unmatched_counts.values()
    )
    status = (
        "PASS"
        if (
            all_topic_stamps_valid
            and record_count_parity
            and all_records_used_once
            and all_window_records_used_once
        )
        else "FAIL"
    )
    return {
        "status": status,
        "matching_policy": (
            "front_source_range_nearest_stamp_one_to_one_with_wall_edge_guard"
        ),
        "match_tolerance_seconds": match_tolerance_seconds,
        "anchor_source_stamp_range_ns": {
            "minimum": lower_stamp,
            "maximum": upper_stamp,
        },
        "record_count_parity": record_count_parity,
        "all_records_used_exactly_once": all_records_used_once,
        "all_window_records_used_exactly_once": all_window_records_used_once,
        "wall_edge_guard_seconds": wall_edge_guard_seconds,
        "matched_bundle_count": matched_bundle_count,
        "maximum_bundle_stamp_span_seconds": (
            max(bundle_stamp_spans) if bundle_stamp_spans else None
        ),
        "topics": topics,
    }


def evaluate_window(
    clock_samples: Sequence[Mapping[str, Any]],
    camera_samples: Mapping[str, Sequence[Mapping[str, Any]]],
    window_start: float,
    window_end: float,
    contract: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Purely evaluate one fixed wall-time window against the health contract."""
    active_contract = dict(contract or default_contract(window_end - window_start))
    thresholds = active_contract["thresholds"]
    duration = float(window_end) - float(window_start)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("window_end must be finite and greater than window_start")

    clocks = _window_records(clock_samples, window_start, window_end)
    clock_monotonic = all(
        float(right["simulation_time_sec"])
        >= float(left["simulation_time_sec"])
        for left, right in zip(clocks, clocks[1:])
    )
    simulation_delta = (
        float(clocks[-1]["simulation_time_sec"])
        - float(clocks[0]["simulation_time_sec"])
        if len(clocks) >= 2
        else None
    )
    rtf = simulation_delta / duration if simulation_delta is not None else None

    camera_metrics: dict[str, dict[str, Any]] = {}
    for topic in CAMERA_INFO_TOPICS:
        records = _window_records(camera_samples.get(topic, ()), window_start, window_end)
        camera_metrics[topic] = {
            "message_count": len(records),
            "wall_rate_hz": len(records) / duration,
            "first_receipt_offset_seconds": (
                float(records[0]["wall_time_sec"]) - window_start if records else None
            ),
            "last_receipt_offset_seconds": (
                float(records[-1]["wall_time_sec"]) - window_start if records else None
            ),
        }

    strict_source_integrity_required = (
        active_contract.get("strict_camera_source_integrity_required") is True
    )
    bundles = camera_bundle_metrics(
        camera_samples,
        window_start,
        window_end,
        float(active_contract["bundle_match_tolerance_seconds"]),
        (
            float(active_contract["bundle_settle_seconds"])
            if strict_source_integrity_required
            else None
        ),
    )
    strict_source_integrity = None
    if strict_source_integrity_required:
        strict_source_integrity = strict_camera_source_integrity(
            camera_samples,
            window_start,
            window_end,
            float(active_contract["bundle_match_tolerance_seconds"]),
            float(active_contract["bundle_settle_seconds"]),
        )
    failures: list[dict[str, Any]] = []
    if len(clocks) < 2:
        failures.append({"check": "clock_samples", "actual": len(clocks), "minimum": 2})
    if not clock_monotonic:
        failures.append({"check": "clock_monotonic", "actual": False, "expected": True})
    if rtf is None or not math.isfinite(rtf) or rtf < float(thresholds["minimum_rtf"]):
        failures.append(
            {
                "check": "rtf",
                "actual": rtf,
                "minimum": float(thresholds["minimum_rtf"]),
            }
        )
    for topic, metrics in camera_metrics.items():
        if metrics["wall_rate_hz"] < float(thresholds["minimum_camera_wall_rate_hz"]):
            failures.append(
                {
                    "check": "camera_wall_rate_hz",
                    "topic": topic,
                    "actual": metrics["wall_rate_hz"],
                    "minimum": float(thresholds["minimum_camera_wall_rate_hz"]),
                }
            )
        maximum_wall_rate = thresholds.get("maximum_camera_wall_rate_hz")
        if (
            maximum_wall_rate is not None
            and metrics["wall_rate_hz"] > float(maximum_wall_rate)
        ):
            failures.append(
                {
                    "check": "camera_wall_rate_hz",
                    "topic": topic,
                    "actual": metrics["wall_rate_hz"],
                    "maximum": float(maximum_wall_rate),
                }
            )
    if strict_source_integrity is not None:
        # HH_260906 - Consume one unique positive source record per camera and bundle.
        # HH_260906 - Preserve arrival ordering before a strict pre-engagement PASS.
        if strict_source_integrity["status"] != "PASS":
            failures.append(
                {
                    "check": "camera_source_stamp_integrity",
                    "actual": strict_source_integrity["status"],
                    "expected": "PASS",
                }
            )
        for topic, metrics in strict_source_integrity["topics"].items():
            source_rate_hz = metrics["source_rate_hz_from_span"]
            minimum_source_rate = float(
                thresholds["minimum_camera_source_rate_hz"]
            )
            maximum_source_rate = float(
                thresholds["maximum_camera_source_rate_hz"]
            )
            if (
                source_rate_hz is None
                or not math.isfinite(float(source_rate_hz))
                or not minimum_source_rate
                <= float(source_rate_hz)
                <= maximum_source_rate
            ):
                failures.append(
                    {
                        "check": "camera_source_rate_hz",
                        "topic": topic,
                        "actual": source_rate_hz,
                        "minimum": minimum_source_rate,
                        "maximum": maximum_source_rate,
                    }
                )
            minimum_source_gap = metrics["minimum_source_gap_seconds"]
            allowed_minimum_source_gap = float(
                thresholds["minimum_camera_source_gap_seconds"]
            )
            if (
                minimum_source_gap is None
                or not math.isfinite(float(minimum_source_gap))
                or (
                    float(minimum_source_gap) < allowed_minimum_source_gap
                    and not math.isclose(
                        float(minimum_source_gap),
                        allowed_minimum_source_gap,
                        rel_tol=0.0,
                        abs_tol=1.0e-12,
                    )
                )
            ):
                failures.append(
                    {
                        "check": "camera_source_gap_seconds",
                        "topic": topic,
                        "actual": minimum_source_gap,
                        "minimum": allowed_minimum_source_gap,
                    }
                )
            maximum_source_gap = metrics["maximum_source_gap_seconds"]
            allowed_source_gap = float(
                thresholds["maximum_camera_source_gap_seconds"]
            )
            if (
                maximum_source_gap is None
                or not math.isfinite(float(maximum_source_gap))
                or (
                    float(maximum_source_gap) > allowed_source_gap
                    and not math.isclose(
                        float(maximum_source_gap),
                        allowed_source_gap,
                        rel_tol=0.0,
                        abs_tol=1.0e-12,
                    )
                )
            ):
                failures.append(
                    {
                        "check": "camera_source_gap_seconds",
                        "topic": topic,
                        "actual": maximum_source_gap,
                        "maximum": allowed_source_gap,
                    }
                )
    if bundles["complete_bundle_count"] < int(
        thresholds["minimum_complete_bundle_count"]
    ):
        failures.append(
            {
                "check": "complete_bundle_count",
                "actual": bundles["complete_bundle_count"],
                "minimum": int(thresholds["minimum_complete_bundle_count"]),
            }
        )
    if bundles["coverage_percent"] < float(
        thresholds["minimum_bundle_coverage_percent"]
    ):
        failures.append(
            {
                "check": "bundle_coverage_percent",
                "actual": bundles["coverage_percent"],
                "minimum": float(thresholds["minimum_bundle_coverage_percent"]),
            }
        )
    receipt_p95 = bundles["receipt_span_seconds"]["p95"]
    if receipt_p95 is None or receipt_p95 > float(
        thresholds["maximum_bundle_receipt_p95_seconds"]
    ):
        failures.append(
            {
                "check": "bundle_receipt_p95_seconds",
                "actual": receipt_p95,
                "maximum": float(
                    thresholds["maximum_bundle_receipt_p95_seconds"]
                ),
            }
        )
    maximum_stamp_span = thresholds.get("maximum_bundle_stamp_span_seconds")
    if maximum_stamp_span is not None:
        observed_stamp_span = bundles["stamp_span_seconds"]["maximum"]
        if (
            observed_stamp_span is None
            or not math.isfinite(float(observed_stamp_span))
            or float(observed_stamp_span) > float(maximum_stamp_span)
        ):
            failures.append(
                {
                    "check": "bundle_stamp_span_seconds",
                    "actual": observed_stamp_span,
                    "maximum": float(maximum_stamp_span),
                }
            )
    return {
        "status": "PASS" if not failures else "FAIL",
        "window": {
            "start_monotonic_seconds": float(window_start),
            "end_monotonic_seconds": float(window_end),
            "duration_seconds": duration,
        },
        "clock": {
            "message_count": len(clocks),
            "monotonic_simulation_time": clock_monotonic,
            "simulation_delta_seconds": simulation_delta,
            "rtf": rtf,
        },
        "cameras": camera_metrics,
        "minimum_observed_camera_wall_rate_hz": min(
            metrics["wall_rate_hz"] for metrics in camera_metrics.values()
        ),
        "maximum_observed_camera_wall_rate_hz": max(
            metrics["wall_rate_hz"] for metrics in camera_metrics.values()
        ),
        "bundles": bundles,
        "camera_source_stamp_integrity": strict_source_integrity,
        "failures": failures,
    }


def evaluate_runtime_health(
    windows: Sequence[Mapping[str, Any]],
    required_consecutive_passes: int = REQUIRED_CONSECUTIVE_PASSES,
) -> dict[str, Any]:
    """Purely classify a sequence and identify the first consecutive PASS run."""
    if required_consecutive_passes <= 0:
        raise ValueError("required_consecutive_passes must be positive")
    consecutive = 0
    maximum_consecutive = 0
    winning_indexes: list[int] = []
    for index, window in enumerate(windows):
        if window.get("status") == "PASS":
            consecutive += 1
        else:
            consecutive = 0
        maximum_consecutive = max(maximum_consecutive, consecutive)
        if consecutive >= required_consecutive_passes and not winning_indexes:
            winning_indexes = list(
                range(index - required_consecutive_passes + 1, index + 1)
            )
    return {
        "status": "PASS" if winning_indexes else "FAIL",
        "required_consecutive_passes": required_consecutive_passes,
        "maximum_consecutive_passes": maximum_consecutive,
        "trailing_consecutive_passes": consecutive,
        "winning_window_indexes": winning_indexes,
        "evaluated_window_count": len(windows),
    }


def _time_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def collect_live_health(
    window_seconds: float,
    timeout_seconds: float,
    contract: Mapping[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any], dict[str, Any]]:
    """Subscribe to runtime topics until three sliding windows pass or timeout."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo
    except Exception as error:  # pragma: no cover - requires sourced ROS
        raise RuntimeHealthError(
            f"ROS 2 Python modules are unavailable; source scripts/e2e/env.sh: {error}"
        ) from error

    clock_samples: list[dict[str, Any]] = []
    camera_samples: dict[str, list[dict[str, Any]]] = {
        topic: [] for topic in CAMERA_INFO_TOPICS
    }
    rclpy.init(args=[])
    node = Node("autoware_e2e_runtime_health_probe")
    subscriptions: list[Any] = []
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)

    transport = contract.get("camera_transport", {})
    require_exact_image_graph = (
        isinstance(transport, Mapping)
        and transport.get("profile_id") in EXACT_CAMERA_GRAPH_PROFILES
    )
    require_portable_runtime_absence = (
        isinstance(transport, Mapping)
        and transport.get("profile_id") == CAMERA_TRANSPORT_PROFILE_STRICT_10HZ
    )
    camera_image_graph: dict[str, Any] = {
        "status": "NOT_REQUIRED",
        "reason": "exact image endpoint graph is a bounded-profile contract",
    }

    def clock_callback(message: Any) -> None:
        clock_samples.append(
            {
                "wall_time_sec": time.monotonic(),
                "simulation_time_sec": _time_ns(message.clock) * 1.0e-9,
            }
        )

    def camera_callback(topic: str):
        def callback(message: Any) -> None:
            camera_samples[topic].append(
                {
                    "wall_time_sec": time.monotonic(),
                    "stamp_ns": _time_ns(message.header.stamp),
                }
            )

        return callback

    windows: list[dict[str, Any]] = []
    sequence: dict[str, Any] = evaluate_runtime_health(windows)
    try:
        if require_exact_image_graph:
            camera_image_graph = collect_camera_image_graph(node)
            if camera_image_graph["status"] != "PASS":
                sequence.update(
                    {
                        "timed_out": False,
                        "elapsed_wall_seconds": camera_image_graph[
                            "elapsed_wall_seconds"
                        ],
                        "failure_reason": (
                            "camera image endpoint graph contract failed"
                        ),
                    }
                )
                return windows, sequence, camera_image_graph

        subscriptions.append(
            node.create_subscription(Clock, CLOCK_TOPIC, clock_callback, qos)
        )
        for topic in CAMERA_INFO_TOPICS:
            subscriptions.append(
                node.create_subscription(CameraInfo, topic, camera_callback(topic), qos)
            )

        started = time.monotonic()
        deadline = started + timeout_seconds
        next_evaluation = started + window_seconds + float(
            contract["bundle_settle_seconds"]
        )
        while True:
            now = time.monotonic()
            while now >= next_evaluation and next_evaluation <= deadline + 1.0e-9:
                window_end = next_evaluation - float(contract["bundle_settle_seconds"])
                window = evaluate_window(
                    clock_samples,
                    camera_samples,
                    window_end - window_seconds,
                    window_end,
                    contract,
                )
                window["index"] = len(windows)
                window["window"]["start_elapsed_seconds"] = (
                    window["window"].pop("start_monotonic_seconds") - started
                )
                window["window"]["end_elapsed_seconds"] = (
                    window["window"].pop("end_monotonic_seconds") - started
                )
                if require_portable_runtime_absence:
                    portable_absence = collect_portable_runtime_absence(node)
                    portable_absence["observed_elapsed_seconds"] = (
                        portable_absence.pop("observed_monotonic_seconds") - started
                    )
                    window["portable_runtime_absence"] = portable_absence
                    if portable_absence["status"] != "PASS":
                        window["failures"].append(
                            {
                                "check": "portable_runtime_absence",
                                "actual": portable_absence["status"],
                                "expected": "PASS",
                            }
                        )
                        window["status"] = "FAIL"
                windows.append(window)
                sequence = evaluate_runtime_health(windows)
                if sequence["status"] == "PASS":
                    return (
                        windows,
                        {
                            **sequence,
                            "timed_out": False,
                            "elapsed_wall_seconds": time.monotonic() - started,
                        },
                        camera_image_graph,
                    )
                next_evaluation += float(contract["evaluation_interval_seconds"])
            if now >= deadline:
                break
            wait_seconds = min(
                0.1,
                max(0.0, next_evaluation - time.monotonic()),
                max(0.0, deadline - time.monotonic()),
            )
            rclpy.spin_once(node, timeout_sec=wait_seconds)
        sequence = evaluate_runtime_health(windows)
        return (
            windows,
            {
                **sequence,
                "timed_out": True,
                "elapsed_wall_seconds": time.monotonic() - started,
            },
            camera_image_graph,
        )
    finally:
        # Keep subscription objects alive until after the last spin.
        subscriptions.clear()
        node.destroy_node()
        rclpy.shutdown()


def validate_transport_environment(
    profile_id: str | None,
    cyclonedds_uri: str | None,
    cyclonedds_config_sha256: str | None,
) -> dict[str, Any]:
    """Bind exact transports to the hash-pinned loopback CycloneDDS environment."""
    actual = {
        "ros_localhost_only": os.environ.get("ROS_LOCALHOST_ONLY"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
        "cyclonedds_uri": os.environ.get("CYCLONEDDS_URI"),
    }
    evidence: dict[str, Any] = {
        "status": "NOT_REQUIRED",
        "profile_id": profile_id,
        "actual": actual,
        "expected": None,
        "cyclonedds_config": None,
        "failures": [],
    }
    if profile_id not in EXACT_CAMERA_GRAPH_PROFILES:
        return evidence

    expected = {
        # CycloneDDS 0.10 rejects selecting ``lo`` both here and through
        # ROS_LOCALHOST_ONLY=1.  The hash-pinned XML exclusively owns the
        # interface selection, so the generic RMW override must be disabled.
        "ros_localhost_only": "0",
        "rmw_implementation": "rmw_cyclonedds_cpp",
        "cyclonedds_uri": cyclonedds_uri,
    }
    evidence["expected"] = expected
    failures: list[dict[str, Any]] = []
    for field, expected_value in expected.items():
        if actual[field] != expected_value:
            failures.append(
                {
                    "check": field,
                    "actual": actual[field],
                    "expected": expected_value,
                }
            )

    config_record: dict[str, Any] = {
        "uri": cyclonedds_uri,
        "path": None,
        "expected_sha256": cyclonedds_config_sha256,
        "actual_sha256": None,
        "regular_file": False,
    }
    if not isinstance(cyclonedds_uri, str) or not cyclonedds_uri.startswith(
        "file:///"
    ):
        failures.append(
            {
                "check": "cyclonedds_uri_scheme",
                "actual": cyclonedds_uri,
                "expected": "canonical absolute file:/// URI",
            }
        )
    else:
        config_path = Path(cyclonedds_uri[len("file://"):])
        config_record["path"] = str(config_path)
        config_record["regular_file"] = config_path.is_file()
        normalized_path = Path(os.path.abspath(config_path))
        if not config_path.is_absolute() or normalized_path != config_path:
            failures.append(
                {
                    "check": "cyclonedds_config_canonical_path",
                    "actual": str(config_path),
                    "expected": str(normalized_path),
                }
            )
        if not config_path.is_file():
            failures.append(
                {
                    "check": "cyclonedds_config_regular_file",
                    "actual": False,
                    "expected": True,
                }
            )
        else:
            actual_sha256 = sha256_file(config_path)
            config_record["actual_sha256"] = actual_sha256
            if actual_sha256 != cyclonedds_config_sha256:
                failures.append(
                    {
                        "check": "cyclonedds_config_sha256",
                        "actual": actual_sha256,
                        "expected": cyclonedds_config_sha256,
                    }
                )
    evidence["cyclonedds_config"] = config_record
    evidence["failures"] = failures
    evidence["status"] = "PASS" if not failures else "FAIL"
    return evidence


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--window-sec", type=float, default=DEFAULT_WINDOW_SECONDS)
    parser.add_argument("--timeout-sec", type=float, default=DEFAULT_TIMEOUT_SECONDS)
    parser.add_argument("--rviz-recorder-pid", type=int)
    parser.add_argument("--rviz-recorder-pgid", type=int)
    parser.add_argument("--camera-transport-profile-id")
    parser.add_argument("--sensor-mapping-sha256")
    parser.add_argument("--vad-model-override-sha256")
    parser.add_argument("--cyclonedds-uri")
    parser.add_argument("--cyclonedds-config-sha256")
    args = parser.parse_args(argv)
    if not math.isfinite(args.window_sec) or args.window_sec <= 0.0:
        parser.error("--window-sec must be a positive finite number")
    if not math.isfinite(args.timeout_sec) or args.timeout_sec <= 0.0:
        parser.error("--timeout-sec must be a positive finite number")
    minimum_useful_timeout = (
        args.window_sec
        + BUNDLE_SETTLE_SECONDS
        + EVALUATION_INTERVAL_SECONDS * (REQUIRED_CONSECUTIVE_PASSES - 1)
    )
    if args.timeout_sec < minimum_useful_timeout:
        parser.error(
            "--timeout-sec cannot permit three evaluated windows; "
            f"minimum is {minimum_useful_timeout:g}"
        )
    if (args.rviz_recorder_pid is None) != (args.rviz_recorder_pgid is None):
        parser.error(
            "--rviz-recorder-pid and --rviz-recorder-pgid must be supplied together"
        )
    if args.rviz_recorder_pid is not None and (
        args.rviz_recorder_pid <= 1 or args.rviz_recorder_pgid <= 1
    ):
        parser.error("owned RViz recorder PID/PGID must be greater than one")
    transport_values = (
        args.camera_transport_profile_id,
        args.sensor_mapping_sha256,
        args.vad_model_override_sha256,
    )
    if any(value is not None for value in transport_values) and not all(
        value is not None for value in transport_values
    ):
        parser.error("camera transport provenance arguments must be supplied together")
    if args.camera_transport_profile_id is not None:
        if args.camera_transport_profile_id not in {
            CAMERA_TRANSPORT_PROFILE_V1,
            CAMERA_TRANSPORT_PROFILE_V2,
            CAMERA_TRANSPORT_PROFILE_PORTABLE_10HZ,
            CAMERA_TRANSPORT_PROFILE_STRICT_10HZ,
        }:
            parser.error("unsupported camera transport profile")
        for value in (
            args.sensor_mapping_sha256,
            args.vad_model_override_sha256,
        ):
            if re.fullmatch(r"[0-9a-f]{64}", value) is None:
                parser.error("camera transport provenance hashes must be SHA-256")
    cyclonedds_values = (args.cyclonedds_uri, args.cyclonedds_config_sha256)
    if args.camera_transport_profile_id in EXACT_CAMERA_GRAPH_PROFILES:
        if not all(value is not None for value in cyclonedds_values):
            parser.error(
                "exact camera transport requires CycloneDDS URI and config SHA-256"
            )
        if not args.cyclonedds_uri.startswith("file:///"):
            parser.error("--cyclonedds-uri must be an absolute file:/// URI")
        if re.fullmatch(r"[0-9a-f]{64}", args.cyclonedds_config_sha256) is None:
            parser.error("--cyclonedds-config-sha256 must be SHA-256")
    elif any(value is not None for value in cyclonedds_values):
        parser.error("CycloneDDS provenance arguments require exact camera transport")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    output = args.output.expanduser().resolve()
    contract = apply_camera_profile_contract(
        default_contract(args.window_sec), args.camera_transport_profile_id
    )
    if args.camera_transport_profile_id is not None:
        camera_transport = {
            "profile_id": args.camera_transport_profile_id,
            "camera_image_publisher_reliability": "best_effort",
            "camera_info_publisher_reliability": "reliable",
            "vad_image_subscription_reliability": "best_effort",
            "rviz_image_subscription_reliability": "best_effort",
            "sensor_mapping_sha256": args.sensor_mapping_sha256,
            "vad_model_override_sha256": args.vad_model_override_sha256,
            "probe_topics": "camera_info_only",
        }
        if args.camera_transport_profile_id in EXACT_CAMERA_GRAPH_PROFILES:
            camera_transport.update(
                {
                    "camera_image_endpoint_history": "keep_last",
                    "camera_image_endpoint_depth": 1,
                    "camera_image_endpoint_durability": "volatile",
                    "exact_camera_image_graph_required": True,
                    "cyclonedds_loopback_interface_required": True,
                    "ros_localhost_only_expected": "0",
                    "rmw_implementation": "rmw_cyclonedds_cpp",
                    "cyclonedds_uri": args.cyclonedds_uri,
                    "cyclonedds_config_sha256": (
                        args.cyclonedds_config_sha256
                    ),
                    "probe_topics": "camera_info_plus_read_only_image_graph",
                }
            )
            contract["topics"]["camera_image_graph"] = list(
                CAMERA_IMAGE_TOPICS
            )
        if args.camera_transport_profile_id in TEN_HZ_CAMERA_PROFILES:
            camera_transport.update(
                {
                    "camera_source_sensor_tick_seconds": 0.1,
                    "bridge_publish_cap_hz": 11,
                    "declared_effective_camera_rate_hz": 10.0,
                    "minimum_camera_wall_rate_hz": (
                        PORTABLE_MINIMUM_CAMERA_WALL_RATE_HZ
                    ),
                    "minimum_complete_bundle_count": (
                        PORTABLE_MINIMUM_COMPLETE_BUNDLES
                    ),
                }
            )
        if args.camera_transport_profile_id == CAMERA_TRANSPORT_PROFILE_STRICT_10HZ:
            camera_transport.update(
                {
                    "qualification_scope": (
                        "strict_six_camera_10hz_transport_runtime_only"
                    ),
                    "simulation_only": True,
                    "vehicle_behavior_validated": False,
                    "portable_model_loaded": False,
                    "portable_model_60kph_validated": False,
                    "portable_model_60kph_claim_allowed": False,
                    "portable_runtime_graph_absence_required": True,
                    "portable_shadow_node": PORTABLE_SHADOW_NODE,
                    "portable_output_topics": list(PORTABLE_OUTPUT_TOPICS),
                    "real_vehicle_ready": False,
                }
            )
        contract["camera_transport"] = camera_transport
    transport_environment = validate_transport_environment(
        args.camera_transport_profile_id,
        args.cyclonedds_uri,
        args.cyclonedds_config_sha256,
    )
    source_path = Path(__file__).resolve()
    payload: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "probe_id": PROBE_ID,
        "status": "FAIL",
        "checked_at": utc_now(),
        "finished_at": None,
        "timeout_seconds": args.timeout_sec,
        "contract": contract,
        "runtime": {
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "ros_localhost_only": os.environ.get("ROS_LOCALHOST_ONLY"),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
            "cyclonedds_uri": os.environ.get("CYCLONEDDS_URI"),
            "process_id": os.getpid(),
            "read_only_subscriber": True,
            "publisher_qos_modified": False,
            "rosbag_started": False,
            "vehicle_engaged": False,
            "rviz_recorder_required": args.rviz_recorder_pid is not None,
            "rviz_recorder_before": None,
            "rviz_recorder_after": None,
            "transport_environment": transport_environment,
        },
        "source": {
            "path": str(source_path),
            "sha256": sha256_file(source_path),
        },
        "windows": [],
        "sequence": None,
        "camera_image_graph": None,
        "error": None,
    }
    exit_status = 1
    try:
        if transport_environment["status"] == "FAIL":
            raise RuntimeHealthError(
                "camera transport environment contract failed"
            )
        if args.rviz_recorder_pid is not None:
            payload["runtime"]["rviz_recorder_before"] = owned_process_record(
                args.rviz_recorder_pid, args.rviz_recorder_pgid
            )
        windows, sequence, camera_image_graph = collect_live_health(
            args.window_sec, args.timeout_sec, contract
        )
        if (
            args.camera_transport_profile_id
            == CAMERA_TRANSPORT_PROFILE_STRICT_10HZ
            and sequence.get("status") == "PASS"
        ):
            windows_by_index = {
                window.get("index"): window
                for window in windows
                if isinstance(window, Mapping)
            }
            missing_absence = [
                index
                for index in sequence.get("winning_window_indexes", ())
                if not isinstance(windows_by_index.get(index), Mapping)
                or windows_by_index[index].get(
                    "portable_runtime_absence", {}
                ).get("status")
                != "PASS"
            ]
            if missing_absence:
                sequence = {
                    **sequence,
                    "status": "FAIL",
                    "failure_reason": (
                        "strict runtime health lacks observed Portable graph "
                        f"absence in winning windows {missing_absence}"
                    ),
                }
        payload["windows"] = windows
        payload["sequence"] = sequence
        payload["camera_image_graph"] = camera_image_graph
        if args.rviz_recorder_pid is not None:
            payload["runtime"]["rviz_recorder_after"] = owned_process_record(
                args.rviz_recorder_pid, args.rviz_recorder_pgid
            )
        if sequence["status"] == "PASS" and sequence["timed_out"] is False:
            payload["status"] = "PASS"
            exit_status = 0
        else:
            payload["error"] = sequence.get(
                "failure_reason",
                "runtime health gate timed out without three consecutive PASS windows",
            )
    except Exception as error:
        payload["error"] = str(error)
    payload["finished_at"] = utc_now()
    atomic_json(output, payload)
    if exit_status == 0:
        print(
            "RUNTIME_HEALTH_PASS "
            f"windows={payload['sequence']['winning_window_indexes']} "
            f"output={output}"
        )
    else:
        print(payload["error"], file=os.sys.stderr)
    return exit_status


if __name__ == "__main__":
    raise SystemExit(main())
