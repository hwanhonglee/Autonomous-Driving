from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
MODULE_PATH = ROOT / "scripts/e2e/probe_runtime_health.py"
TRIAL_SCRIPT = ROOT / "scripts/e2e/run_recorded_route_trial.sh"
SPEC = importlib.util.spec_from_file_location("probe_runtime_health", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
health = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(health)


def _healthy_samples(
    *,
    start: float = 100.0,
    rtf: float = 1.0,
    camera_hz: float = 5.0,
    receipt_span: float = 0.010,
) -> tuple[list[dict[str, float]], dict[str, list[dict[str, float | int]]]]:
    clock = [
        {
            "wall_time_sec": start + index * 0.1,
            "simulation_time_sec": index * 0.1 * rtf,
        }
        for index in range(81)
    ]
    count = int(8.0 * camera_hz)
    camera = {topic: [] for topic in health.CAMERA_INFO_TOPICS}
    for index in range(count):
        base = start + 0.05 + index / camera_hz
        stamp = 1_000_000_000 + int(round(index / camera_hz * 1.0e9))
        for camera_index, topic in enumerate(health.CAMERA_INFO_TOPICS):
            camera[topic].append(
                {
                    "wall_time_sec": base
                    + receipt_span * camera_index / (len(health.CAMERA_INFO_TOPICS) - 1),
                    "stamp_ns": stamp,
                }
            )
    return clock, camera


def _failure_checks(report: dict) -> set[str]:
    return {failure["check"] for failure in report["failures"]}


def _image_endpoint(node: str, *, depth: int = 1) -> dict:
    return {
        "node": node,
        "topic_type": "sensor_msgs/msg/Image",
        "endpoint_gid_hex": node.encode().hex(),
        "qos": {
            "reliability": "best_effort",
            "durability": "volatile",
            "history": "keep_last",
            "depth": depth,
        },
    }


def _exact_image_graph() -> dict:
    graph = {}
    for topic in health.CAMERA_IMAGE_TOPICS:
        subscriptions = [_image_endpoint(health.EXPECTED_VAD_IMAGE_SUBSCRIBER)]
        if topic == health.CAMERA_IMAGE_TOPICS[0]:
            subscriptions.append(
                _image_endpoint(health.EXPECTED_RVIZ_IMAGE_SUBSCRIBER)
            )
        graph[topic] = {
            "publishers": [_image_endpoint(health.EXPECTED_CAMERA_IMAGE_PUBLISHER)],
            "subscriptions": subscriptions,
        }
    return graph


def test_default_contract_is_the_fixed_fail_closed_contract() -> None:
    contract = health.default_contract()

    assert contract["window_seconds"] == 8.0
    assert contract["required_consecutive_passes"] == 3
    assert contract["thresholds"] == {
        "minimum_rtf": 0.9,
        "minimum_camera_wall_rate_hz": 4.0,
        "minimum_complete_bundle_count": 20,
        "minimum_bundle_coverage_percent": 99.0,
        "maximum_bundle_receipt_p95_seconds": 0.040,
    }
    assert contract["topics"]["clock"] == "/clock"
    assert len(contract["topics"]["camera_info"]) == 6
    assert contract["clock_domains"]["cross_domain_subtraction_used"] is False


def test_exact_depth1_camera_image_graph_passes() -> None:
    report = health.evaluate_camera_image_graph(_exact_image_graph())

    assert report["status"] == "PASS"
    assert report["failures"] == []
    assert report["expected"]["qos"] == {
        "reliability": "best_effort",
        "durability": "volatile",
        "history": "keep_last",
        "depth": 1,
    }
    front = report["expected"]["topics"][health.CAMERA_IMAGE_TOPICS[0]]
    assert front["publishers"] == ["/autoware_carla_interface"]
    assert front["subscriptions"] == ["/autoware_vad_rviz", "/vad_carla_tiny"]


def test_camera_image_graph_rejects_extra_reader_and_depth_ten() -> None:
    graph = _exact_image_graph()
    front = graph[health.CAMERA_IMAGE_TOPICS[0]]
    front["subscriptions"].append(_image_endpoint("/unexpected_remote_reader"))
    front["publishers"][0]["qos"]["depth"] = 10

    report = health.evaluate_camera_image_graph(graph)

    assert report["status"] == "FAIL"
    checks = {failure["check"] for failure in report["failures"]}
    assert "endpoint_nodes" in checks
    assert "endpoint_qos_depth" in checks


def test_camera_image_graph_rejects_missing_side_vad_reader() -> None:
    graph = _exact_image_graph()
    graph[health.CAMERA_IMAGE_TOPICS[-1]]["subscriptions"] = []

    report = health.evaluate_camera_image_graph(graph)

    assert report["status"] == "FAIL"
    failure = next(
        item
        for item in report["failures"]
        if item["topic"] == health.CAMERA_IMAGE_TOPICS[-1]
    )
    assert failure["actual"] == []
    assert failure["expected"] == ["/vad_carla_tiny"]


def test_healthy_window_passes_all_clock_camera_and_bundle_checks() -> None:
    clock, camera = _healthy_samples()

    report = health.evaluate_window(clock, camera, 100.0, 108.0)

    assert report["status"] == "PASS"
    assert report["failures"] == []
    assert report["clock"]["rtf"] == pytest.approx(1.0)
    assert report["minimum_observed_camera_wall_rate_hz"] == pytest.approx(5.0)
    assert report["bundles"]["complete_bundle_count"] == 40
    assert report["bundles"]["coverage_percent"] == pytest.approx(100.0)
    assert report["bundles"]["receipt_span_seconds"]["p95"] == pytest.approx(0.010)


def test_strict_10hz_rejects_one_frame_shifted_camera_stamps() -> None:
    clock, camera = _healthy_samples(camera_hz=10.0)
    shifted_topic = health.CAMERA_INFO_TOPICS[-1]
    for sample in camera[shifted_topic]:
        sample["stamp_ns"] += 100_000_000
    contract = health.apply_camera_profile_contract(
        health.default_contract(), health.CAMERA_TRANSPORT_PROFILE_STRICT_10HZ
    )

    report = health.evaluate_window(clock, camera, 100.0, 108.0, contract)

    assert report["status"] == "FAIL"
    assert report["bundles"]["match_tolerance_seconds"] == pytest.approx(0.000005)
    assert "bundle_coverage_percent" in _failure_checks(report)


def test_strict_10hz_gates_maximum_source_stamp_span() -> None:
    clock, camera = _healthy_samples(camera_hz=10.0)
    shifted_topic = health.CAMERA_INFO_TOPICS[-1]
    for sample in camera[shifted_topic]:
        sample["stamp_ns"] += 4_000
    contract = health.apply_camera_profile_contract(
        health.default_contract(), health.CAMERA_TRANSPORT_PROFILE_STRICT_10HZ
    )

    passing = health.evaluate_window(clock, camera, 100.0, 108.0, contract)
    contract["thresholds"]["maximum_bundle_stamp_span_seconds"] = 0.000003
    failing = health.evaluate_window(clock, camera, 100.0, 108.0, contract)

    assert passing["status"] == "PASS"
    assert passing["bundles"]["stamp_span_seconds"]["maximum"] == pytest.approx(
        0.000004
    )
    assert failing["status"] == "FAIL"
    assert "bundle_stamp_span_seconds" in _failure_checks(failing)


# HH_260906 - Prevent duplicated camera delivery from qualifying before engagement.
def test_strict_10hz_rejects_duplicate_non_front_source_stamp() -> None:
    clock, camera = _healthy_samples(camera_hz=10.0)
    topic = health.CAMERA_INFO_TOPICS[1]
    duplicate = dict(camera[topic][20])
    duplicate["wall_time_sec"] += 0.000001
    camera[topic].insert(21, duplicate)
    contract = health.apply_camera_profile_contract(
        health.default_contract(), health.CAMERA_TRANSPORT_PROFILE_STRICT_10HZ
    )

    report = health.evaluate_window(clock, camera, 100.0, 108.0, contract)

    assert report["status"] == "FAIL"
    assert "camera_source_stamp_integrity" in _failure_checks(report)
    integrity = report["camera_source_stamp_integrity"]
    assert integrity["status"] == "FAIL"
    assert integrity["record_count_parity"] is False
    assert integrity["all_records_used_exactly_once"] is False
    assert integrity["topics"][topic][
        "duplicate_positive_source_range_stamp_count"
    ] == 1


# HH_260906 - Reject a stale mid-window receipt even when its stamp is outside the front range.
def test_strict_10hz_rejects_unmatched_non_front_record_outside_front_range() -> None:
    clock, camera = _healthy_samples(camera_hz=10.0)
    topic = health.CAMERA_INFO_TOPICS[1]
    camera[topic].append(
        {
            "wall_time_sec": 104.0,
            "stamp_ns": 500_000_000,
        }
    )
    contract = health.apply_camera_profile_contract(
        health.default_contract(), health.CAMERA_TRANSPORT_PROFILE_STRICT_10HZ
    )

    report = health.evaluate_window(clock, camera, 100.0, 108.0, contract)

    assert report["status"] == "FAIL"
    assert "camera_source_stamp_integrity" in _failure_checks(report)
    integrity = report["camera_source_stamp_integrity"]
    assert integrity["all_window_records_used_exactly_once"] is False
    assert integrity["topics"][topic]["window_unmatched_record_count"] == 1


# HH_260906 - Use the wall extension to complete a legitimate trailing-edge bundle.
def test_strict_10hz_accepts_complete_bundle_straddling_trailing_wall_edge() -> None:
    clock, camera = _healthy_samples(camera_hz=10.0)
    for records in camera.values():
        for record in records:
            record["wall_time_sec"] += 0.045
    contract = health.apply_camera_profile_contract(
        health.default_contract(), health.CAMERA_TRANSPORT_PROFILE_STRICT_10HZ
    )

    report = health.evaluate_window(clock, camera, 100.0, 108.0, contract)

    assert report["status"] == "PASS"
    integrity = report["camera_source_stamp_integrity"]
    assert integrity["all_window_records_used_exactly_once"] is True
    assert all(
        item["window_unmatched_record_count"] == 0
        for item in integrity["topics"].values()
    )


# HH_260906 - Preserve callback arrival order instead of sorting away a stale frame.
def test_strict_10hz_rejects_out_of_order_source_stamp() -> None:
    clock, camera = _healthy_samples(camera_hz=10.0)
    topic = health.CAMERA_INFO_TOPICS[-1]
    camera[topic][20]["stamp_ns"], camera[topic][21]["stamp_ns"] = (
        camera[topic][21]["stamp_ns"],
        camera[topic][20]["stamp_ns"],
    )
    contract = health.apply_camera_profile_contract(
        health.default_contract(), health.CAMERA_TRANSPORT_PROFILE_STRICT_10HZ
    )

    report = health.evaluate_window(clock, camera, 100.0, 108.0, contract)

    assert report["status"] == "FAIL"
    assert "camera_source_stamp_integrity" in _failure_checks(report)
    integrity = report["camera_source_stamp_integrity"]
    assert integrity["status"] == "FAIL"
    assert integrity["topics"][topic][
        "non_increasing_source_range_arrival_stamp_delta_count"
    ] == 1
    assert integrity["topics"][topic][
        "strictly_increasing_unique_positive_arrival_stamps"
    ] is False


# HH_260906 - Enforce upper bounds so a doubled transport cannot satisfy minima alone.
def test_strict_10hz_rejects_excess_wall_and_source_cadence() -> None:
    clock, camera = _healthy_samples(camera_hz=20.0)
    contract = health.apply_camera_profile_contract(
        health.default_contract(), health.CAMERA_TRANSPORT_PROFILE_STRICT_10HZ
    )

    report = health.evaluate_window(clock, camera, 100.0, 108.0, contract)

    assert report["status"] == "FAIL"
    assert report["maximum_observed_camera_wall_rate_hz"] == pytest.approx(20.0)
    assert "camera_wall_rate_hz" in _failure_checks(report)
    assert "camera_source_rate_hz" in _failure_checks(report)


# HH_260906 - Reject a short source interval hidden by an acceptable aggregate rate.
def test_strict_10hz_rejects_compressed_source_interval() -> None:
    clock, camera = _healthy_samples(camera_hz=10.0)
    for records in camera.values():
        for record in records[40:]:
            record["stamp_ns"] -= 50_000_000
    contract = health.apply_camera_profile_contract(
        health.default_contract(), health.CAMERA_TRANSPORT_PROFILE_STRICT_10HZ
    )

    report = health.evaluate_window(clock, camera, 100.0, 108.0, contract)

    assert report["status"] == "FAIL"
    assert "camera_source_gap_seconds" in _failure_checks(report)
    for item in report["camera_source_stamp_integrity"]["topics"].values():
        assert item["source_rate_hz_from_span"] == pytest.approx(
            10.0636942675
        )
        assert item["minimum_source_gap_seconds"] == pytest.approx(0.05)


# HH_260906 - Accept both inclusive five-microsecond source-period boundaries.
@pytest.mark.parametrize("source_delta_ns", (99_995_000, 100_005_000))
def test_strict_10hz_accepts_source_period_tolerance_boundaries(
    source_delta_ns: int,
) -> None:
    clock, camera = _healthy_samples(camera_hz=10.0)
    for records in camera.values():
        for index, record in enumerate(records):
            record["stamp_ns"] = 1_000_000_000 + index * source_delta_ns
    contract = health.apply_camera_profile_contract(
        health.default_contract(), health.CAMERA_TRANSPORT_PROFILE_STRICT_10HZ
    )

    report = health.evaluate_window(clock, camera, 100.0, 108.0, contract)

    assert report["status"] == "PASS"
    for item in report["camera_source_stamp_integrity"]["topics"].values():
        assert item["minimum_source_gap_seconds"] == pytest.approx(
            source_delta_ns * 1.0e-9
        )
        assert item["maximum_source_gap_seconds"] == pytest.approx(
            source_delta_ns * 1.0e-9
        )


# HH_260906 - A serial bundle crossing a wall-window edge is not a source drop.
def test_strict_10hz_source_range_guard_accepts_complete_edge_bundle() -> None:
    clock, camera = _healthy_samples(camera_hz=10.0)
    for records in camera.values():
        for record in records:
            record["wall_time_sec"] -= 0.055
    contract = health.apply_camera_profile_contract(
        health.default_contract(), health.CAMERA_TRANSPORT_PROFILE_STRICT_10HZ
    )

    report = health.evaluate_window(clock, camera, 100.0, 108.0, contract)

    assert report["status"] == "PASS"
    integrity = report["camera_source_stamp_integrity"]
    assert integrity["status"] == "PASS"
    assert integrity["record_count_parity"] is True
    assert integrity["all_records_used_exactly_once"] is True


# HH_260906 - Keep legacy 5 Hz and Portable Common10 acceptance semantics unchanged.
def test_strict_stamp_and_upper_cadence_gates_are_not_applied_to_legacy_profiles() -> None:
    baseline = health.default_contract()
    portable = health.apply_camera_profile_contract(
        health.default_contract(), health.CAMERA_TRANSPORT_PROFILE_PORTABLE_10HZ
    )

    for contract in (baseline, portable):
        assert contract.get("strict_camera_source_integrity_required") is None
        assert "maximum_camera_wall_rate_hz" not in contract["thresholds"]
        assert "minimum_camera_source_rate_hz" not in contract["thresholds"]
        assert "maximum_camera_source_rate_hz" not in contract["thresholds"]
        assert "minimum_camera_source_gap_seconds" not in contract["thresholds"]
        assert "maximum_camera_source_gap_seconds" not in contract["thresholds"]


def test_portable_runtime_absence_rejects_node_or_output_publisher() -> None:
    empty = health.evaluate_portable_runtime_absence([], {})
    node_present = health.evaluate_portable_runtime_absence(
        [health.PORTABLE_SHADOW_NODE], {}
    )
    publisher_present = health.evaluate_portable_runtime_absence(
        [],
        {
            health.PORTABLE_OUTPUT_TOPICS[0]: [
                _image_endpoint("/renamed_portable_runtime")
            ]
        },
    )

    assert empty["status"] == "PASS"
    assert node_present["status"] == "FAIL"
    assert publisher_present["status"] == "FAIL"
    assert node_present["failures"][0]["check"] == "portable_shadow_node_absent"
    assert (
        publisher_present["failures"][0]["check"]
        == "portable_output_publishers_absent"
    )


def test_low_rtf_fails_even_when_all_camera_checks_pass() -> None:
    clock, camera = _healthy_samples(rtf=0.75)

    report = health.evaluate_window(clock, camera, 100.0, 108.0)

    assert report["status"] == "FAIL"
    assert _failure_checks(report) == {"rtf"}
    assert report["clock"]["rtf"] == pytest.approx(0.75)


def test_each_camera_must_deliver_four_wall_receipts_per_second() -> None:
    clock, camera = _healthy_samples()
    slow_topic = health.CAMERA_INFO_TOPICS[-1]
    camera[slow_topic] = camera[slow_topic][:31]

    report = health.evaluate_window(clock, camera, 100.0, 108.0)

    assert report["status"] == "FAIL"
    failures = report["failures"]
    assert any(
        failure["check"] == "camera_wall_rate_hz"
        and failure["topic"] == slow_topic
        and failure["actual"] == pytest.approx(31 / 8)
        for failure in failures
    )


def test_one_missing_camera_frame_fails_ninety_nine_percent_bundle_coverage() -> None:
    clock, camera = _healthy_samples()
    del camera[health.CAMERA_INFO_TOPICS[2]][20]

    report = health.evaluate_window(clock, camera, 100.0, 108.0)

    assert report["status"] == "FAIL"
    assert "bundle_coverage_percent" in _failure_checks(report)
    assert report["bundles"]["complete_bundle_count"] == 39
    assert report["bundles"]["coverage_percent"] == pytest.approx(97.5)


def test_zero_front_stamp_counts_as_an_incomplete_delivered_bundle() -> None:
    clock, camera = _healthy_samples()
    camera[health.CAMERA_INFO_TOPICS[0]][12]["stamp_ns"] = 0

    report = health.evaluate_window(clock, camera, 100.0, 108.0)

    assert report["status"] == "FAIL"
    assert report["bundles"]["front_anchor_count"] == 40
    assert report["bundles"]["complete_bundle_count"] == 39
    assert report["bundles"]["coverage_percent"] == pytest.approx(97.5)


def test_bundle_receipt_p95_above_forty_milliseconds_fails() -> None:
    clock, camera = _healthy_samples(receipt_span=0.050)

    report = health.evaluate_window(clock, camera, 100.0, 108.0)

    assert report["status"] == "FAIL"
    assert "bundle_receipt_p95_seconds" in _failure_checks(report)
    assert report["bundles"]["receipt_span_seconds"]["p95"] == pytest.approx(0.050)


def test_bundle_matching_is_one_to_one_and_does_not_reuse_side_frames() -> None:
    _, camera = _healthy_samples()
    side_topic = health.CAMERA_INFO_TOPICS[1]
    camera[side_topic] = camera[side_topic][::2]

    bundles = health.camera_bundle_metrics(camera, 100.0, 108.0)

    assert bundles["complete_bundle_count"] == 20
    assert bundles["coverage_percent"] == pytest.approx(50.0)


def test_three_consecutive_passes_are_required_and_failure_resets_sequence() -> None:
    windows = [
        {"status": "PASS"},
        {"status": "PASS"},
        {"status": "FAIL"},
        {"status": "PASS"},
        {"status": "PASS"},
    ]

    incomplete = health.evaluate_runtime_health(windows)
    complete = health.evaluate_runtime_health([*windows, {"status": "PASS"}])

    assert incomplete["status"] == "FAIL"
    assert incomplete["maximum_consecutive_passes"] == 2
    assert complete["status"] == "PASS"
    assert complete["winning_window_indexes"] == [3, 4, 5]


def test_timeout_path_writes_fail_json_and_returns_one(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "runtime_health.json"

    def timed_out(*_args):
        return (
            [{"status": "FAIL"}],
            {
                "status": "FAIL",
                "required_consecutive_passes": 3,
                "maximum_consecutive_passes": 0,
                "trailing_consecutive_passes": 0,
                "winning_window_indexes": [],
                "evaluated_window_count": 1,
                "timed_out": True,
                "elapsed_wall_seconds": 45.0,
            },
            {"status": "NOT_REQUIRED"},
        )

    monkeypatch.setattr(health, "collect_live_health", timed_out)

    assert health.main(["--output", str(output)]) == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["schema_version"] == 1
    assert payload["status"] == "FAIL"
    assert payload["sequence"]["timed_out"] is True
    assert "three consecutive PASS" in payload["error"]


def test_runtime_health_timeout_must_allow_three_sliding_windows() -> None:
    with pytest.raises(SystemExit):
        health.parse_args(["--output", "unused.json", "--timeout-sec", "10.0"])

    args = health.parse_args(
        ["--output", "unused.json", "--timeout-sec", "10.1"]
    )
    assert args.window_sec == 8.0
    assert args.timeout_sec == 10.1


def test_runtime_health_binds_camera_transport_provenance(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "runtime_health.json"
    mapping_sha = "a" * 64
    model_sha = "b" * 64

    monkeypatch.setattr(
        health,
        "collect_live_health",
        lambda *_: (
            [],
            {
                "status": "FAIL",
                "required_consecutive_passes": 3,
                "maximum_consecutive_passes": 0,
                "trailing_consecutive_passes": 0,
                "winning_window_indexes": [],
                "evaluated_window_count": 0,
                "timed_out": True,
                "elapsed_wall_seconds": 45.0,
            },
            {"status": "NOT_REQUIRED"},
        ),
    )

    assert health.main(
        [
            "--output",
            str(output),
            "--camera-transport-profile-id",
            "carla_vad_camera_source_5hz_best_effort_image_v1",
            "--sensor-mapping-sha256",
            mapping_sha,
            "--vad-model-override-sha256",
            model_sha,
        ]
    ) == 1
    transport = json.loads(output.read_text(encoding="utf-8"))["contract"][
        "camera_transport"
    ]
    assert transport == {
        "profile_id": "carla_vad_camera_source_5hz_best_effort_image_v1",
        "camera_image_publisher_reliability": "best_effort",
        "camera_info_publisher_reliability": "reliable",
        "vad_image_subscription_reliability": "best_effort",
        "rviz_image_subscription_reliability": "best_effort",
        "sensor_mapping_sha256": mapping_sha,
        "vad_model_override_sha256": model_sha,
        "probe_topics": "camera_info_only",
    }


def test_runtime_health_rejects_partial_camera_transport_provenance() -> None:
    with pytest.raises(SystemExit):
        health.parse_args(
            [
                "--output",
                "unused.json",
                "--camera-transport-profile-id",
                "carla_vad_camera_source_5hz_best_effort_image_v1",
            ]
        )


@pytest.mark.parametrize(
    "profile_id",
    (
        health.CAMERA_TRANSPORT_PROFILE_V2,
        health.CAMERA_TRANSPORT_PROFILE_PORTABLE_10HZ,
        health.CAMERA_TRANSPORT_PROFILE_STRICT_10HZ,
    ),
)
def test_exact_transport_requires_cyclonedds_provenance(profile_id: str) -> None:
    with pytest.raises(SystemExit):
        health.parse_args(
            [
                "--output",
                "unused.json",
                "--camera-transport-profile-id",
                profile_id,
                "--sensor-mapping-sha256",
                "a" * 64,
                "--vad-model-override-sha256",
                "b" * 64,
            ]
        )


def test_transport_v2_environment_binds_loopback_config_hash(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = tmp_path / "cyclonedds.xml"
    config.write_text("<CycloneDDS/>", encoding="utf-8")
    config_sha = health.sha256_file(config)
    uri = config.resolve().as_uri()
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "0")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv("CYCLONEDDS_URI", uri)

    report = health.validate_transport_environment(
        health.CAMERA_TRANSPORT_PROFILE_V2, uri, config_sha
    )

    assert report["status"] == "PASS"
    assert report["failures"] == []
    assert report["cyclonedds_config"]["actual_sha256"] == config_sha


def test_portable_10hz_transport_binds_exact_graph_and_rate_contract(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = tmp_path / "cyclonedds.xml"
    config.write_text("<CycloneDDS/>", encoding="utf-8")
    config_sha = health.sha256_file(config)
    uri = config.resolve().as_uri()
    output = tmp_path / "runtime_health.json"
    graph = health.evaluate_camera_image_graph(_exact_image_graph())
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "0")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv("CYCLONEDDS_URI", uri)
    monkeypatch.setattr(
        health,
        "collect_live_health",
        lambda *_: (
            [
                {
                    "index": index,
                    "status": "PASS",
                    "portable_runtime_absence": {"status": "PASS"},
                }
                for index in range(3)
            ],
            {
                "status": "PASS",
                "required_consecutive_passes": 3,
                "maximum_consecutive_passes": 3,
                "trailing_consecutive_passes": 3,
                "winning_window_indexes": [0, 1, 2],
                "evaluated_window_count": 3,
                "timed_out": False,
                "elapsed_wall_seconds": 10.1,
            },
            graph,
        ),
    )

    assert health.main(
        [
            "--output",
            str(output),
            "--camera-transport-profile-id",
            health.CAMERA_TRANSPORT_PROFILE_PORTABLE_10HZ,
            "--sensor-mapping-sha256",
            "a" * 64,
            "--vad-model-override-sha256",
            "b" * 64,
            "--cyclonedds-uri",
            uri,
            "--cyclonedds-config-sha256",
            config_sha,
        ]
    ) == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["contract"]["thresholds"]["minimum_camera_wall_rate_hz"] == 9.0
    assert payload["contract"]["thresholds"]["minimum_complete_bundle_count"] == 70
    transport = payload["contract"]["camera_transport"]
    assert transport["profile_id"] == health.CAMERA_TRANSPORT_PROFILE_PORTABLE_10HZ
    assert transport["camera_source_sensor_tick_seconds"] == 0.1
    assert transport["bridge_publish_cap_hz"] == 11
    assert transport["declared_effective_camera_rate_hz"] == 10.0
    assert transport["exact_camera_image_graph_required"] is True
    assert payload["camera_image_graph"]["status"] == "PASS"
    assert payload["runtime"]["transport_environment"]["status"] == "PASS"


def test_strict_10hz_transport_records_non_portable_qualification_boundary(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = tmp_path / "cyclonedds.xml"
    config.write_text("<CycloneDDS/>", encoding="utf-8")
    config_sha = health.sha256_file(config)
    uri = config.resolve().as_uri()
    output = tmp_path / "runtime_health.json"
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "0")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv("CYCLONEDDS_URI", uri)
    monkeypatch.setattr(
        health,
        "collect_live_health",
        lambda *_: (
            [
                {
                    "index": index,
                    "status": "PASS",
                    "portable_runtime_absence": {"status": "PASS"},
                }
                for index in range(3)
            ],
            {
                "status": "PASS",
                "required_consecutive_passes": 3,
                "maximum_consecutive_passes": 3,
                "trailing_consecutive_passes": 3,
                "winning_window_indexes": [0, 1, 2],
                "evaluated_window_count": 3,
                "timed_out": False,
                "elapsed_wall_seconds": 10.1,
            },
            health.evaluate_camera_image_graph(_exact_image_graph()),
        ),
    )

    assert health.main(
        [
            "--output",
            str(output),
            "--camera-transport-profile-id",
            health.CAMERA_TRANSPORT_PROFILE_STRICT_10HZ,
            "--sensor-mapping-sha256",
            "a" * 64,
            "--vad-model-override-sha256",
            "b" * 64,
            "--cyclonedds-uri",
            uri,
            "--cyclonedds-config-sha256",
            config_sha,
        ]
    ) == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["contract"]["thresholds"]["minimum_camera_wall_rate_hz"] == 9.0
    assert payload["contract"]["thresholds"]["minimum_complete_bundle_count"] == 70
    assert payload["contract"]["strict_camera_source_integrity_required"] is True
    assert payload["contract"]["thresholds"]["maximum_camera_wall_rate_hz"] == 11.0
    assert payload["contract"]["thresholds"]["minimum_camera_source_rate_hz"] == 9.5
    assert payload["contract"]["thresholds"]["maximum_camera_source_rate_hz"] == 10.5
    assert payload["contract"]["thresholds"][
        "minimum_camera_source_gap_seconds"
    ] == pytest.approx(0.099995)
    assert payload["contract"]["thresholds"][
        "maximum_camera_source_gap_seconds"
    ] == pytest.approx(0.100005)
    transport = payload["contract"]["camera_transport"]
    assert transport["qualification_scope"] == (
        "strict_six_camera_10hz_transport_runtime_only"
    )
    assert transport["portable_model_loaded"] is False
    assert transport["portable_model_60kph_validated"] is False
    assert transport["portable_model_60kph_claim_allowed"] is False
    assert transport["vehicle_behavior_validated"] is False
    assert transport["real_vehicle_ready"] is False
    assert transport["portable_runtime_graph_absence_required"] is True
    assert payload["contract"]["bundle_match_tolerance_seconds"] == pytest.approx(
        0.000005
    )
    assert payload["contract"]["thresholds"][
        "maximum_bundle_stamp_span_seconds"
    ] == pytest.approx(0.000005)


def test_transport_v2_environment_rejects_duplicate_localhost_override_and_hash_drift(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = tmp_path / "cyclonedds.xml"
    config.write_text("<CycloneDDS/>", encoding="utf-8")
    uri = config.resolve().as_uri()
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv("CYCLONEDDS_URI", uri)

    report = health.validate_transport_environment(
        health.CAMERA_TRANSPORT_PROFILE_V2, uri, "0" * 64
    )

    assert report["status"] == "FAIL"
    checks = {failure["check"] for failure in report["failures"]}
    assert "ros_localhost_only" in checks
    assert "cyclonedds_config_sha256" in checks


def test_transport_v2_main_records_environment_and_exact_graph_contract(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    output = tmp_path / "runtime_health.json"
    config = tmp_path / "cyclonedds.xml"
    config.write_text("<CycloneDDS/>", encoding="utf-8")
    config_sha = health.sha256_file(config)
    uri = config.resolve().as_uri()
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "0")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    monkeypatch.setenv("CYCLONEDDS_URI", uri)
    graph = health.evaluate_camera_image_graph(_exact_image_graph())
    graph.update(
        {
            "status": "PASS",
            "elapsed_wall_seconds": 0.3,
        }
    )
    monkeypatch.setattr(
        health,
        "collect_live_health",
        lambda *_: (
            [{"status": "PASS"}] * 3,
            {
                "status": "PASS",
                "required_consecutive_passes": 3,
                "maximum_consecutive_passes": 3,
                "trailing_consecutive_passes": 3,
                "winning_window_indexes": [0, 1, 2],
                "evaluated_window_count": 3,
                "timed_out": False,
                "elapsed_wall_seconds": 10.1,
            },
            graph,
        ),
    )

    assert health.main(
        [
            "--output",
            str(output),
            "--camera-transport-profile-id",
            health.CAMERA_TRANSPORT_PROFILE_V2,
            "--sensor-mapping-sha256",
            "a" * 64,
            "--vad-model-override-sha256",
            "b" * 64,
            "--cyclonedds-uri",
            uri,
            "--cyclonedds-config-sha256",
            config_sha,
        ]
    ) == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "PASS"
    assert payload["runtime"]["transport_environment"]["status"] == "PASS"
    assert payload["camera_image_graph"]["status"] == "PASS"
    transport = payload["contract"]["camera_transport"]
    assert transport["camera_image_endpoint_depth"] == 1
    assert transport["exact_camera_image_graph_required"] is True
    assert transport["cyclonedds_config_sha256"] == config_sha


def test_trial_runs_auto_gate_after_video_recorder_and_before_bag_and_engagement() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    auto_gate = 'runtime_health_gate_mode="automatic_speed_camera_source_5hz"'
    gate_call = 'python3 "${runtime_health_probe}" "${runtime_health_arguments[@]}"'
    video_start = '"${output_dir}/autoware_rviz_capture.mkv" &'
    bag_start = 'setsid scripts/e2e/record_turn_dynamics.sh "${output_dir}/bag"'
    engage_start = 'setsid scripts/e2e/route_test.sh "${route_test_arguments[@]}"'

    assert "--runtime-health-gate" in source
    assert "--runtime-health-timeout" in source
    assert auto_gate in source
    assert source.index(video_start) < source.index(gate_call)
    assert source.index(gate_call) < source.index(bag_start)
    assert source.index(gate_call) < source.index(engage_start)
    assert "RUNTIME_HEALTH_EVIDENCE_SHA256=" in source
    assert "RUNTIME_HEALTH_GATE_STATUS=" in source
    assert "RUNTIME_HEALTH_RVIZ_RECORDER_ACTIVE_DURING_PROBE=" in source
    assert "CAMERA_TRANSPORT_PROFILE_ID=" in source
    assert "--camera-transport-profile-id" in source
