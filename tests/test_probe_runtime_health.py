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


def test_transport_v2_requires_cyclonedds_provenance() -> None:
    with pytest.raises(SystemExit):
        health.parse_args(
            [
                "--output",
                "unused.json",
                "--camera-transport-profile-id",
                health.CAMERA_TRANSPORT_PROFILE_V2,
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
