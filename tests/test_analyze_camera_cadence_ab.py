from __future__ import annotations

import copy
import csv
import hashlib
import importlib.util
import io
import json
from pathlib import Path
from typing import Any, Callable

from PIL import Image
import pytest
import yaml


ROOT = Path(__file__).parents[1]
MODULE_PATH = ROOT / "scripts/e2e/analyze_camera_cadence_ab.py"
SPEC = importlib.util.spec_from_file_location("analyze_camera_cadence_ab", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
analyzer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analyzer)


CAMERAS = (
    "CAM_FRONT/camera_link",
    "CAM_FRONT_LEFT/camera_link",
    "CAM_FRONT_RIGHT/camera_link",
    "CAM_BACK/camera_link",
    "CAM_BACK_LEFT/camera_link",
    "CAM_BACK_RIGHT/camera_link",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _mapping(sensor_tick: float) -> dict[str, Any]:
    sensors: dict[str, Any] = {}
    for index, key in enumerate(CAMERAS):
        camera_id = key.split("/", 1)[0]
        sensors[key] = {
            "carla_type": "sensor.camera.rgb",
            "id": camera_id,
            "ros_config": {
                "frame_id": f"{camera_id}/camera_optical_link",
                "topic_image": f"/sensing/camera/{camera_id}/image_raw",
                "topic_info": f"/sensing/camera/{camera_id}/camera_info",
                "frequency_hz": 5,
                "qos_profile": "reliable",
            },
            "parameters": {
                "image_size_x": 640,
                "image_size_y": 360,
                "fov": 110.0 if index == 3 else 70.0,
                "sensor_tick": sensor_tick,
                "enable_postprocess_effects": False,
            },
        }
    sensors["tamagawa/imu_link"] = {
        "carla_type": "sensor.other.imu",
        "id": "imu",
        "ros_config": {
            "frame_id": "tamagawa/imu_link",
            "topic": "/sensing/imu/tamagawa/imu_raw",
            "frequency_hz": 50,
            "qos_profile": "reliable",
        },
        "parameters": {"sensor_tick": 0.0, "noise_accel_stddev_x": 0.0},
    }
    sensors["gnss_link"] = {
        "carla_type": "sensor.other.gnss",
        "id": "gnss",
        "ros_config": {
            "frame_id": "map",
            "topic": "/sensing/gnss/pose_with_covariance",
            "frequency_hz": 20,
            "qos_profile": "reliable",
        },
        "parameters": {"noise_alt_stddev": 0.0},
        "covariance": {"position_variance": 0.01, "orientation_variance": 1.0},
    }
    return {
        "default_sensor_kit_name": "carla_sensor_kit_description",
        "sensor_mappings": sensors,
        "normalization": {"strip_suffixes": ["_base_link", "_link", "/camera_link"]},
        "enabled_sensors": [*CAMERAS, "tamagawa/imu_link", "gnss_link"],
    }


def _profile(candidate: bool) -> dict[str, Any]:
    profile = {
        "id": (
            "vad_carla_30kph_camera_source_5hz_visualized_v1"
            if candidate
            else "vad_carla_30kph_visualized_v2"
        ),
        "trial_wrapper": "run_recorded_route_trial.sh",
        "wrapper_options": list(
            analyzer.EXPECTED_CANDIDATE_WRAPPER_OPTIONS
            if candidate
            else analyzer.EXPECTED_BASELINE_WRAPPER_OPTIONS
        ),
        "ready_timeout_sec": 600,
        "launch_arguments": ["carla_timeout:=60"],
        "carla_quality": "Epic",
        "carla_arguments": ["-RenderOffScreen", "-nosound"],
        "map_lifecycle": "cold_start_owned_process_group_per_trial",
        "client_map_loading_allowed": False,
        "speed_contract": {
            "profile_id": "carla_vad_30kph_v2",
            "target_speed_mps": 8.333333333333334,
            "real_vehicle_ready": False,
        },
    }
    if candidate:
        profile["camera_source_contract"] = {
            "profile_id": "carla_camera_source_5hz_ab_v1",
            "sensor_count": 6,
            "sensor_tick_sec": 0.2,
            "ros_publish_frequency_hz": 5.0,
            "maximum_stamp_gap_sec": 0.25,
        }
    return profile


def _result(candidate: bool, trial_id: str) -> dict[str, Any]:
    wall_elapsed = 20.0 if candidate else 40.0
    minimum_duration = 1.0 if trial_id == "straight" else 0.0
    return {
        "schema_version": 1,
        "success": True,
        "reason": "goal reached",
        "execution_mode": "full_stack",
        "assessment": {
            "planning_architecture": "vad_route_manager_hybrid",
            "route_completion": "PASS",
        },
        "initial": {
            "sim_time_sec": 8.0,
            "mrm_state": {"state": 1, "behavior": 1},
        },
        "final": {
            "goal_reached": True,
            "route_status": "goal_reached",
            "sim_time_sec": 18.0,
            "mrm_state": {"state": 1, "behavior": 1},
            "aeb_configured_for_vad_objects": True,
        },
        "profile_context": {
            "longitudinal_velocity_source": "explicit_simulation_nominal",
            "vad_geometry_evaluated": True,
            "vad_velocity_evaluated": False,
        },
        "speed_exposure": {
            "status": "PASS",
            "longitudinal_velocity_source": "explicit_simulation_nominal",
            "vad_geometry_evaluated": True,
            "vad_velocity_evaluated": False,
        },
        "limits": {
            "maximum_absolute_cte_m": 1.0,
            "maximum_trajectory_correction_m": 15.0,
            "maximum_lateral_acceleration_mps2": 1.8,
            "maximum_observed_speed_mps": 9.0,
            "maximum_speed_sample_gap_sec": 0.25,
            "minimum_sustained_speed_sec": minimum_duration,
        },
        "metrics": {
            "sim_elapsed_sec": 10.0,
            "wall_elapsed_sec": wall_elapsed,
            "maximum_absolute_cte_m": 0.5,
            "maximum_trajectory_correction_m": 1.0,
            "maximum_lateral_acceleration_mps2": 0.7,
            "maximum_observed_speed_mps": 7.8 if trial_id == "straight" else 4.9,
            "maximum_speed_sample_gap_sec": 0.1,
            "maximum_sustained_speed_duration_sec": 2.0 if trial_id == "straight" else 0.0,
        },
    }


def _route(trial_id: str) -> dict[str, Any]:
    scenario = "straight" if trial_id == "straight" else "left"
    return {
        "schema_version": 1,
        "town": analyzer.CANONICAL_MAP,
        "weather": "ClearNoon",
        "scenario": scenario,
        "route_length_m": 200.0 if trial_id == "straight" else 75.0,
        "start_spawn_index": 10 if trial_id == "straight" else 20,
        "goal_spawn_index": 11 if trial_id == "straight" else 21,
        "route": [
            {
                "index": 0,
                "x": 0.0,
                "y": 0.0,
                "road_option": "STRAIGHT" if trial_id == "straight" else "LEFT",
            }
        ],
    }


def _latency(candidate: bool, trial_id: str) -> dict[str, Any]:
    effective_rate = 2.5 if candidate else 1.25
    front_count = 100
    # Exercise the real whole-bag boundary allowance on one baseline trial.
    candidate_count = 99 if not candidate and trial_id == "straight" else 100
    acceptance = 100.0 * candidate_count / front_count
    front_topic = "/sensing/camera/CAM_FRONT/camera_info"
    event_rates = {
        f"/sensing/camera/{camera.split('/', 1)[0]}/camera_info": {
            "count": front_count,
            "stamp_rate_hz": 5.0,
            "stamp_period_sec": {
                "available": True,
                "count": front_count - 1,
                "mean": 0.2,
                "median": 0.2,
                "max": 0.2,
            },
        }
        for camera in CAMERAS
    }
    event_rates[front_topic].update(
        {
            "effective_receipt_rate_hz": effective_rate,
            "receipt_rate_hz": effective_rate + 0.123,
            "receipt_period_sec": {
                "available": True,
                "count": front_count - 1,
                "mean": 1.0 / effective_rate,
                "median": 1.0 / (effective_rate + 0.123),
            },
        }
    )
    return {
        "schema_version": 1,
        "selected_topics": {
            "sensor": front_topic,
            "vad_output": "/planning/vad/candidate_trajectories",
        },
        "event_rates": event_rates,
        "camera_bundle": {
            "available": True,
            "camera_count": 6,
            "front_frame_count": front_count,
            "matched_bundle_count": front_count,
            "bundle_coverage_percent": 100.0,
            "stamp_span_sec": {"available": True, "count": front_count, "max": 0.0},
        },
        "candidate_front_acceptance": {
            "available": True,
            "candidate_count": candidate_count,
            "front_count": front_count,
            "acceptance_percent": acceptance,
        },
        "data_quality_warnings": [
            "receipt rate uses whole-bag recorder time",
            "candidate/front is a count-ratio proxy",
        ],
    }


def _stack_log() -> str:
    return "\n".join(
        (
            "[vad] [INFO 1000.000] VAD frame queued: source_stamp_ns=1 assembled=1 "
            "capacity_pruned=0 superseded=0 mailbox_submitted=1 coalesced_drops=0 "
            "received_images_min=1 received_images_max=1",
            "[vad] [INFO 1000.010] VAD inference complete: source_stamp_ns=1 inference_ms=30.0 "
            "published=true published_count=1 mailbox_taken=1 coalesced_drops=0",
            "[vad] [INFO 1000.200] VAD frame queued: source_stamp_ns=2 assembled=2 "
            "capacity_pruned=0 superseded=0 mailbox_submitted=2 coalesced_drops=0 "
            "received_images_min=2 received_images_max=2",
            "[vad] [INFO 1000.210] VAD inference complete: source_stamp_ns=2 inference_ms=40.0 "
            "published=true published_count=2 mailbox_taken=2 coalesced_drops=0",
            "",
        )
    )


def _write_trial(root: Path, candidate: bool, trial_id: str) -> dict[str, Any]:
    attempt = root / "maps" / analyzer.MAP_ID / "trials" / trial_id / "attempt_001"
    attempt.mkdir(parents=True)
    route = _route(trial_id)
    _write_json(attempt / "source_route.json", route)
    _write_json(attempt / "result.json", _result(candidate, trial_id))
    _write_json(attempt / "latency/e2e_latency.json", _latency(candidate, trial_id))
    (attempt / "stack.log").write_text(_stack_log(), encoding="utf-8")
    (attempt / "recorder.log").write_text(
        "[rosbag] [INFO 1001.000] Recording...\n", encoding="utf-8"
    )

    mapping_name = (
        analyzer.CANDIDATE_MAPPING_NAME if candidate else analyzer.BASELINE_MAPPING_NAME
    )
    mapping_path = attempt / "sensor_mapping_provenance/sensor_mapping.yaml"
    mapping_path.parent.mkdir(parents=True)
    mapping_path.write_text(
        yaml.safe_dump(_mapping(0.2 if candidate else 0.0), sort_keys=False),
        encoding="utf-8",
    )
    digest = _sha256(mapping_path)
    (mapping_path.parent / "SHA256SUMS").write_text(
        f"{digest}  sensor_mapping.yaml\n", encoding="utf-8"
    )
    runtime_lines = [
        f"SENSOR_MAPPING_FILE=/installed/config/{mapping_name}",
        f"SENSOR_MAPPING_SHA256={digest}",
        "VAD_ROUTE_MANAGER_OPENBLAS_NUM_THREADS=1",
        "VAD_ROUTE_MANAGER_OMP_NUM_THREADS=1",
        "VAD_ROUTE_MANAGER_MKL_NUM_THREADS=1",
        "VAD_ROUTE_MANAGER_NUMEXPR_NUM_THREADS=1",
    ]
    if candidate:
        runtime_lines.extend(
            (
                "CAMERA_SOURCE_5HZ=true",
                "CAMERA_SOURCE_SENSOR_TICK_SEC=0.2",
                "CAMERA_ROS_PUBLISH_HZ=5.0",
            )
        )
    (attempt / "runtime.env").write_text(
        "\n".join(runtime_lines) + "\n", encoding="utf-8"
    )

    selector = (
        analyzer.CANDIDATE_SELECTOR if candidate else analyzer.BASELINE_SELECTOR
    )
    validation = {
        "schema_version": 1,
        "status": "PASS",
        "map_id": analyzer.MAP_ID,
        "trial_id": trial_id,
        "runtime_profile_selector": selector,
        "result": {
            "success": True,
            "route_completion": "PASS",
            "goal_reached": True,
        },
        "speed_contract": {"status": "PASS"},
    }
    if candidate:
        validation["camera_source_contract"] = {"status": "PASS"}
    _write_json(attempt / "matrix_validation.json", validation)
    return {
        "status": "PASS",
        "reason": "synthetic strict PASS",
        "attempt_directory": str(attempt),
        "validation": str(attempt / "matrix_validation.json"),
    }


def _write_matrix_root(root: Path, candidate: bool) -> Path:
    root.mkdir(parents=True)
    selector = (
        analyzer.CANDIDATE_SELECTOR if candidate else analyzer.BASELINE_SELECTOR
    )
    profile = _profile(candidate)
    plan = {
        "schema_version": 1,
        "matrix_id": "autoware_vad_town_straight_turn_v1",
        "runtime_profile_selector": selector,
        "runtime_profile": profile,
        "route_contract": {"weather": "ClearNoon"},
    }
    _write_json(root / "matrix_plan.json", plan)
    trials = {
        trial_id: _write_trial(root, candidate, trial_id) for trial_id in analyzer.TRIAL_IDS
    }
    status = {
        "schema_version": 1,
        "matrix_id": "autoware_vad_town_straight_turn_v1",
        "map_id": analyzer.MAP_ID,
        "canonical_name": analyzer.CANONICAL_MAP,
        "runnable": True,
        "status": "PASS",
        "stage": "complete",
        "reason": "Straight and turn full-stack trials both passed.",
        "block_code": None,
        "trials": trials,
    }
    _write_json(root / "maps" / analyzer.MAP_ID / "status.json", status)
    # A selected-map subset is intentionally INCOMPLETE at aggregate scope.
    aggregate = {
        "schema_version": 1,
        "matrix_id": "autoware_vad_town_straight_turn_v1",
        "status": "INCOMPLETE",
        "runtime_profile_selector": selector,
        "runtime_profile": profile,
        "maps": [status],
    }
    _write_json(root / "aggregate.json", aggregate)
    return root


@pytest.fixture
def matrix_pair(tmp_path: Path) -> tuple[Path, Path]:
    return (
        _write_matrix_root(tmp_path / "baseline", False),
        _write_matrix_root(tmp_path / "candidate", True),
    )


def _trial(root: Path, trial_id: str = "straight") -> Path:
    return root / "maps" / analyzer.MAP_ID / "trials" / trial_id / "attempt_001"


def _mutate_json(path: Path, mutation: Callable[[dict[str, Any]], None]) -> None:
    value = json.loads(path.read_text(encoding="utf-8"))
    mutation(value)
    _write_json(path, value)


def _rewrite_mapping(
    root: Path, mutation: Callable[[dict[str, Any]], None]
) -> None:
    for trial_id in analyzer.TRIAL_IDS:
        attempt = _trial(root, trial_id)
        path = attempt / "sensor_mapping_provenance/sensor_mapping.yaml"
        mapping = yaml.safe_load(path.read_text(encoding="utf-8"))
        mutation(mapping)
        path.write_text(yaml.safe_dump(mapping, sort_keys=True), encoding="utf-8")
        digest = _sha256(path)
        (path.parent / "SHA256SUMS").write_text(
            f"{digest}  sensor_mapping.yaml\n", encoding="utf-8"
        )
        runtime = (attempt / "runtime.env").read_text(encoding="utf-8")
        runtime = "\n".join(
            f"SENSOR_MAPPING_SHA256={digest}"
            if line.startswith("SENSOR_MAPPING_SHA256=")
            else line
            for line in runtime.splitlines()
        )
        (attempt / "runtime.env").write_text(runtime + "\n", encoding="utf-8")


def test_analyze_builds_fixed_ctrack_pair_and_computes_ratios(
    matrix_pair: tuple[Path, Path],
) -> None:
    baseline, candidate = matrix_pair

    report = analyzer.analyze(baseline, candidate)

    assert report["status"] == "PASS"
    assert report["decision"] == "VALID_DESCRIPTIVE_COMPARISON"
    assert report["trial_ids"] == ["straight", "turn"]
    assert report["arms"]["baseline"]["aggregate_status"] == "INCOMPLETE"
    assert report["arms"]["candidate"]["aggregate_status"] == "INCOMPLETE"
    for trial_id in analyzer.TRIAL_IDS:
        baseline_trial = report["arms"]["baseline"]["trials"][trial_id]
        candidate_trial = report["arms"]["candidate"]["trials"][trial_id]
        assert baseline_trial["measurement"]["rtf"] == pytest.approx(0.25)
        assert candidate_trial["measurement"]["rtf"] == pytest.approx(0.5)
        assert baseline_trial["measurement"]["front_effective_receipt_rate_hz"] == 1.25
        assert candidate_trial["measurement"]["front_effective_receipt_rate_hz"] == 2.5
        assert baseline_trial["measurement"]["receipt_efficiency_ratio"] == 1.0
        assert candidate_trial["measurement"]["receipt_efficiency_ratio"] == 1.0
        assert report["comparisons"][trial_id]["rtf"][
            "candidate_change_percent"
        ] == pytest.approx(100.0)
        assert report["route_identity"][trial_id]["payload_equal"] is True
        assert baseline_trial["source_route"]["sha256"] == candidate_trial[
            "source_route"
        ]["sha256"]
        assert baseline_trial["camera_vad"]["route_window"]["status"] == "UNAVAILABLE"
        assert baseline_trial["vad_runtime"]["inference_ms"]["mean"] == 35.0
        assert baseline_trial["vad_runtime"]["inference_ms"]["p95"] == 39.5
    straight_acceptance = report["arms"]["baseline"]["trials"]["straight"][
        "camera_vad"
    ]["candidate_front_acceptance"]
    assert straight_acceptance["acceptance_percent"] == 99.0
    assert straight_acceptance["count_delta"] == -1

    delta = report["single_variable_contract"]["sensor_mapping"]
    assert delta["status"] == "PASS"
    assert delta["camera_count"] == 6
    assert delta["other_fields_equal"] is True
    assert len(delta["semantic_changes"]) == 6
    assert [item["path"] for item in delta["semantic_changes"]] == sorted(
        f"sensor_mappings.{key}.parameters.sensor_tick" for key in CAMERAS
    )


def test_semantic_mapping_delta_allows_yaml_reformatting_but_rejects_other_drift(
    matrix_pair: tuple[Path, Path],
) -> None:
    baseline, candidate = matrix_pair
    # Rewrite with sorted keys: byte formatting changes, semantics do not.
    _rewrite_mapping(candidate, lambda _mapping: None)
    report = analyzer.analyze(baseline, candidate)
    assert report["single_variable_contract"]["sensor_mapping"][
        "other_fields_equal"
    ] is True
    assert report["arms"]["baseline"]["sensor_mapping_sha256"] != report["arms"][
        "candidate"
    ]["sensor_mapping_sha256"]

    _rewrite_mapping(
        candidate,
        lambda value: value["sensor_mappings"][CAMERAS[0]]["ros_config"].update(
            {"qos_profile": "best_effort"}
        ),
    )
    with pytest.raises(analyzer.AnalysisError, match="not exactly six"):
        analyzer.analyze(baseline, candidate)


def test_outputs_are_byte_deterministic_and_report_is_not_mutated(
    matrix_pair: tuple[Path, Path], tmp_path: Path
) -> None:
    report = analyzer.analyze(*matrix_pair)
    original = copy.deepcopy(report)

    first = analyzer.write_outputs(report, tmp_path / "first")
    second = analyzer.write_outputs(report, tmp_path / "second")

    assert report == original
    assert set(first) == {"json", "csv", "markdown", "png"}
    for label in first:
        assert first[label].read_bytes() == second[label].read_bytes()
    assert first["json"].read_text(encoding="utf-8").endswith("\n")
    payload = json.loads(first["json"].read_text(encoding="utf-8"))
    assert payload == report
    rows = list(csv.DictReader(io.StringIO(first["csv"].read_text(encoding="utf-8"))))
    assert [(row["trial_id"], row["arm"]) for row in rows] == [
        ("straight", "baseline"),
        ("straight", "candidate"),
        ("turn", "baseline"),
        ("turn", "candidate"),
    ]
    markdown = first["markdown"].read_text(encoding="utf-8")
    assert "whole rosbag" in markdown
    assert "does not reread the bag" in markdown
    assert "collision-free" not in markdown
    assert first["png"].read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    with Image.open(first["png"]) as image:
        image.verify()
    assert not list((tmp_path / "first").glob(".camera_cadence_ab.*"))


def test_cli_writes_only_fixed_output_set(
    matrix_pair: tuple[Path, Path], tmp_path: Path
) -> None:
    baseline, candidate = matrix_pair
    output = tmp_path / "cli"

    status = analyzer.main(
        [
            "--baseline-root",
            str(baseline),
            "--candidate-root",
            str(candidate),
            "--output-dir",
            str(output),
        ]
    )

    assert status == 0
    assert sorted(path.name for path in output.iterdir()) == [
        "camera_cadence_ab.csv",
        "camera_cadence_ab.json",
        "camera_cadence_ab.md",
        "camera_cadence_ab.png",
    ]


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        (
            lambda root: _mutate_json(
                root / "matrix_plan.json",
                lambda value: value.update({"runtime_profile_selector": "recommended"}),
            ),
            "candidate selector",
        ),
        (
            lambda root: _mutate_json(
                root / "maps" / analyzer.MAP_ID / "status.json",
                lambda value: value.update({"stage": "running"}),
            ),
            "not terminal",
        ),
        (
            lambda root: _mutate_json(
                _trial(root) / "result.json",
                lambda value: value["metrics"].update({"wall_elapsed_sec": 0.0}),
            ),
            "wall elapsed must be positive",
        ),
    ),
)
def test_plan_terminal_and_numeric_failures_are_closed(
    matrix_pair: tuple[Path, Path],
    mutation: Callable[[Path], None],
    message: str,
) -> None:
    baseline, candidate = matrix_pair
    mutation(candidate)
    with pytest.raises(analyzer.AnalysisError, match=message):
        analyzer.analyze(baseline, candidate)


def test_source_route_payload_mismatch_is_rejected(
    matrix_pair: tuple[Path, Path],
) -> None:
    baseline, candidate = matrix_pair
    _mutate_json(
        _trial(candidate, "turn") / "source_route.json",
        lambda value: value.update({"goal_spawn_index": 999}),
    )

    with pytest.raises(analyzer.AnalysisError, match="source_route SHA256 differs"):
        analyzer.analyze(baseline, candidate)


@pytest.mark.parametrize(
    "kind",
    (
        "coverage",
        "acceptance_two_frame_gap",
        "acceptance_below_99",
        "maximum_camera_gap",
        "formula",
    ),
)
def test_latency_integrity_and_boundary_gates_are_fail_closed(
    matrix_pair: tuple[Path, Path], kind: str
) -> None:
    baseline, candidate = matrix_pair
    path = _trial(candidate) / "latency/e2e_latency.json"

    def mutation(value: dict[str, Any]) -> None:
        front_topic = value["selected_topics"]["sensor"]
        if kind == "coverage":
            value["camera_bundle"].update(
                {
                    "front_frame_count": 1000,
                    "matched_bundle_count": 989,
                    "bundle_coverage_percent": 98.9,
                }
            )
            value["event_rates"][front_topic]["count"] = 1000
            value["candidate_front_acceptance"].update(
                {"candidate_count": 1000, "front_count": 1000, "acceptance_percent": 100.0}
            )
        elif kind == "acceptance_two_frame_gap":
            value["candidate_front_acceptance"].update(
                {"candidate_count": 98, "acceptance_percent": 98.0}
            )
        elif kind == "acceptance_below_99":
            value["candidate_front_acceptance"].update(
                {"candidate_count": 989, "front_count": 1000, "acceptance_percent": 98.9}
            )
            value["event_rates"][front_topic]["count"] = 1000
            value["camera_bundle"].update(
                {
                    "front_frame_count": 1000,
                    "matched_bundle_count": 1000,
                    "bundle_coverage_percent": 100.0,
                }
            )
        elif kind == "maximum_camera_gap":
            value["event_rates"][front_topic]["stamp_period_sec"]["max"] = 0.4
        else:
            value["event_rates"][front_topic]["effective_receipt_rate_hz"] = 9.0

    _mutate_json(path, mutation)
    with pytest.raises(analyzer.AnalysisError):
        analyzer.analyze(baseline, candidate)


@pytest.mark.parametrize(
    ("old", "new", "message"),
    (
        ("capacity_pruned=0", "capacity_pruned=1", "pruning"),
        ("superseded=0", "superseded=1", "supersession"),
        ("coalesced_drops=0", "coalesced_drops=1", "coalesced"),
        ("mailbox_taken=2", "mailbox_taken=1", "published/mailbox"),
    ),
)
def test_vad_loss_counters_are_fail_closed(
    matrix_pair: tuple[Path, Path], old: str, new: str, message: str
) -> None:
    baseline, candidate = matrix_pair
    path = _trial(candidate) / "stack.log"
    source = path.read_text(encoding="utf-8")
    if old == "mailbox_taken=2":
        source = source.replace(old, new)
    else:
        source = source.replace(old, new, 1)
    path.write_text(source, encoding="utf-8")

    with pytest.raises(analyzer.AnalysisError, match=message):
        analyzer.analyze(baseline, candidate)


def test_single_startup_supersession_before_recorder_is_classified(
    matrix_pair: tuple[Path, Path],
) -> None:
    baseline, candidate = matrix_pair
    for trial_id in analyzer.TRIAL_IDS:
        path = _trial(candidate, trial_id) / "stack.log"
        source = path.read_text(encoding="utf-8")
        source = source.replace("superseded=0", "superseded=1")
        source = source.replace(
            "received_images_min=1 received_images_max=1",
            "received_images_min=2 received_images_max=2",
        )
        source = source.replace(
            "assembled=2 capacity_pruned=0 superseded=1 mailbox_submitted=2 "
            "coalesced_drops=0 received_images_min=2 received_images_max=2",
            "assembled=2 capacity_pruned=0 superseded=1 mailbox_submitted=2 "
            "coalesced_drops=0 received_images_min=3 received_images_max=3",
        )
        path.write_text(source, encoding="utf-8")

    report = analyzer.analyze(baseline, candidate)
    for trial_id in analyzer.TRIAL_IDS:
        runtime = report["arms"]["candidate"]["trials"][trial_id]["vad_runtime"]
        assert runtime["superseded_max"] == 1
        assert runtime["superseded_classification"] == "startup_before_recorder"


def test_startup_supersession_must_precede_recorder(
    matrix_pair: tuple[Path, Path],
) -> None:
    baseline, candidate = matrix_pair
    attempt = _trial(candidate)
    source = (attempt / "stack.log").read_text(encoding="utf-8")
    source = source.replace("superseded=0", "superseded=1").replace(
        "received_images_min=1 received_images_max=1",
        "received_images_min=2 received_images_max=2",
    )
    source = source.replace(
        "assembled=2 capacity_pruned=0 superseded=1 mailbox_submitted=2 "
        "coalesced_drops=0 received_images_min=2 received_images_max=2",
        "assembled=2 capacity_pruned=0 superseded=1 mailbox_submitted=2 "
        "coalesced_drops=0 received_images_min=3 received_images_max=3",
    )
    (attempt / "stack.log").write_text(source, encoding="utf-8")
    (attempt / "recorder.log").write_text(
        "[rosbag] [INFO 999.000] Recording...\n", encoding="utf-8"
    )

    with pytest.raises(analyzer.AnalysisError, match="not proven to precede"):
        analyzer.analyze(baseline, candidate)


def test_route_manager_thread_cap_provenance_is_required(
    matrix_pair: tuple[Path, Path],
) -> None:
    baseline, candidate = matrix_pair
    runtime_path = _trial(candidate) / "runtime.env"
    runtime_path.write_text(
        runtime_path.read_text(encoding="utf-8").replace(
            "VAD_ROUTE_MANAGER_OPENBLAS_NUM_THREADS=1",
            "VAD_ROUTE_MANAGER_OPENBLAS_NUM_THREADS=24",
        ),
        encoding="utf-8",
    )

    with pytest.raises(analyzer.AnalysisError, match="OPENBLAS_NUM_THREADS=1"):
        analyzer.analyze(baseline, candidate)


def test_mapping_provenance_is_sha_bound(matrix_pair: tuple[Path, Path]) -> None:
    baseline, candidate = matrix_pair
    path = _trial(candidate) / "sensor_mapping_provenance/SHA256SUMS"
    path.write_text(f"{'0' * 64}  sensor_mapping.yaml\n", encoding="utf-8")

    with pytest.raises(analyzer.AnalysisError, match="manifest changed"):
        analyzer.analyze(baseline, candidate)


def test_cli_failure_preserves_existing_outputs(
    matrix_pair: tuple[Path, Path], tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    baseline, candidate = matrix_pair
    _mutate_json(
        _trial(candidate) / "latency/e2e_latency.json",
        lambda value: value["camera_bundle"].update(
            {"matched_bundle_count": 98, "bundle_coverage_percent": 98.0}
        ),
    )
    output = tmp_path / "existing"
    output.mkdir()
    sentinel = b"keep-existing-output\n"
    for suffix in ("json", "csv", "md", "png"):
        (output / f"camera_cadence_ab.{suffix}").write_bytes(sentinel)

    status = analyzer.main(
        [
            "--baseline-root",
            str(baseline),
            "--candidate-root",
            str(candidate),
            "--output-dir",
            str(output),
        ]
    )

    assert status == 1
    assert "validation failed" in capsys.readouterr().err
    for suffix in ("json", "csv", "md", "png"):
        assert (output / f"camera_cadence_ab.{suffix}").read_bytes() == sentinel
