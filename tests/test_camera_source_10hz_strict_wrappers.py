from __future__ import annotations

import json
from pathlib import Path
import subprocess


ROOT = Path(__file__).parents[1]
FAST = ROOT / "scripts/e2e/run_route_vad_fast.sh"
RECORDED = ROOT / "scripts/e2e/run_recorded_route_trial.sh"
PILOT = ROOT / "scripts/e2e/run_autoware_vad_60kph_pilot.sh"


def _straight_route(path: Path) -> Path:
    path.write_text(
        json.dumps({"town": "Town06", "scenario": "straight"}) + "\n",
        encoding="utf-8",
    )
    return path


def test_strict_fast_wrapper_requires_60kph_and_rejects_clock_override(
    tmp_path: Path,
) -> None:
    missing_speed = subprocess.run(
        [str(FAST), "--camera-source-10hz-strict", "unused.json"],
        check=False,
        text=True,
        capture_output=True,
    )
    route = _straight_route(tmp_path / "route.json")
    overridden_clock = subprocess.run(
        [
            str(FAST),
            "--speed-60kph-pilot",
            "--camera-source-10hz-strict",
            str(route),
            "fixed_delta_seconds:=0.1",
        ],
        check=False,
        text=True,
        capture_output=True,
    )

    assert missing_speed.returncode == 2
    assert "requires --speed-60kph-pilot" in missing_speed.stderr
    assert overridden_clock.returncode == 2
    assert "controls fixed_delta_seconds" in overridden_clock.stderr


def test_recorded_strict_wrapper_requires_visual_graph_and_rejects_sync_override(
    tmp_path: Path,
) -> None:
    headless = subprocess.run(
        [
            str(RECORDED),
            "--speed-60kph-pilot",
            "--camera-source-10hz-strict",
        ],
        check=False,
        text=True,
        capture_output=True,
    )
    route = _straight_route(tmp_path / "route.json")
    overridden_sync = subprocess.run(
        [
            str(RECORDED),
            "--speed-60kph-pilot",
            "--camera-source-10hz-strict",
            "--visualize",
            str(tmp_path / "output"),
            str(route),
            "sync_mode:=false",
        ],
        check=False,
        text=True,
        capture_output=True,
    )

    assert headless.returncode == 2
    assert "requires --visualize" in headless.stderr
    assert overridden_sync.returncode == 2
    assert "controls sync_mode" in overridden_sync.stderr


def test_strict_wrappers_pin_clock_barrier_and_do_not_launch_portable() -> None:
    fast = FAST.read_text(encoding="utf-8")
    recorded = RECORDED.read_text(encoding="utf-8")

    assert '"sync_mode:=true"' in fast
    assert '"fixed_delta_seconds:=0.05"' in fast
    assert '"camera_frame_barrier_enabled:=true"' in fast
    assert 'fast_mapping="${package_share}/config/sensor_mapping_portable_e2e_10hz.yaml"' in fast
    assert "portable_e2e_shadow.launch.xml" not in fast
    assert 'camera_contract_forward_arguments=()' in recorded
    assert '"sync_mode:=true"' in recorded
    assert '"fixed_delta_seconds:=0.05"' in recorded
    assert "strict_camera_runtime_parameters.json" in recorded
    assert "CARLA_CAMERA_DELIVERY_PATCH_REVERSE_CHECK" in recorded
    assert "CARLA_CAMERA_ENTRYPOINT_SHA256" in recorded
    assert not (
        ROOT
        / "autoware_e2e_vad_launch/config/sensor_mapping_camera_source_10hz_strict.yaml"
    ).exists()


def test_strict_recorder_acknowledges_all_camera_topics_before_measurement() -> None:
    recorded = RECORDED.read_text(encoding="utf-8")

    paused_start = recorded.index(
        'record_turn_dynamics.sh "${output_dir}/bag" --start-paused'
    )
    subscription_check = recorded.index(
        "strict_recorder_subscription_count", paused_start
    )
    paused_check = recorded.index(
        "Waiting for recording: Press SPACE to start.", subscription_check
    )
    resume = recorded.index("printf ' '", paused_check)
    resume_ack = recorded.index("Resuming recording.", resume)
    measurement_evidence = recorded.index(
        "recorder_measurement_start.json", resume_ack
    )
    route_start = recorded.index("route_evaluation_started_at=", measurement_evidence)

    assert paused_start < subscription_check < paused_check < resume < resume_ack
    assert resume_ack < measurement_evidence < route_start
    assert '"${strict_recorder_subscription_count}" == "6"' in recorded
    assert "STRICT_RECORDER_START_PAUSED=true" in recorded
    assert (
        "STRICT_RECORDER_RESUME_CONTROL=humble_rosbag2_owned_pty_space_key_v1"
        in recorded
    )
    assert "STRICT_RECORDER_MEASUREMENT_RESUME_STATUS=pass" in recorded
    assert "carla_vad_camera_source_10hz_strict_v2" in recorded
    assert '"minimum_camera_source_gap_seconds": 0.099995' in recorded
    assert 'source_integrity.get("all_window_records_used_exactly_once")' in recorded
    assert "strict_winning_indexes_valid" in recorded
    assert 'get("duration_seconds")' in recorded
    assert 'get("complete_bundle_count")' in recorded
    assert "/rosbag2_recorder/resume" not in recorded
    assert "/rosbag2_recorder/is_paused" not in recorded


def test_strict_recorder_is_monitored_through_route_completion() -> None:
    recorded = RECORDED.read_text(encoding="utf-8")

    route_loop = recorded.index('while kill -0 "${route_test_pid}"')
    recorder_watch = recorded.index("require_recorder route_evaluation", route_loop)
    route_wait = recorded.index('wait "${route_test_pid}"', recorder_watch)
    completion_watch = recorded.index("require_recorder route_completion", route_wait)
    recorder_stop = recorded.index(
        '"${recorder_pgid}" "${recorder_pid}" 30 10 3', completion_watch
    )

    assert route_loop < recorder_watch < route_wait < completion_watch < recorder_stop
    assert "recorder_alive()" in recorded
    assert "recorder_failure.log" in recorded
    assert "close_recorder_control" in recorded


def test_strict_pilot_emits_symmetric_period_and_v2_recorder_contract() -> None:
    pilot = PILOT.read_text(encoding="utf-8")

    assert "carla_six_camera_10hz_strict_transport_runtime_v2" in pilot
    assert '"minimum_camera_stamp_gap_sec": 0.099995' in pilot
    assert "maximum_gap_sec > 0.100005 + 1.0e-12" in pilot
    assert "minimum_gap_sec < 0.099995 - 1.0e-12" in pilot
    assert '"STACK_POST_SHUTDOWN_CRITICAL_PROCESS_CHECK": "pass"' in pilot
