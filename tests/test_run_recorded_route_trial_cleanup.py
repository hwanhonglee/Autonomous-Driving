from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


ROOT = Path(__file__).parents[1]
TRIAL_SCRIPT = ROOT / "scripts/e2e/run_recorded_route_trial.sh"
CLEANUP_LIBRARY = ROOT / "scripts/e2e/process_group_cleanup.sh"


def _process_state(pid: int) -> str | None:
    completed = subprocess.run(
        ["ps", "-o", "stat=", "-p", str(pid)],
        check=False,
        capture_output=True,
        text=True,
    )
    state = completed.stdout.strip()
    return state or None


def _wait_until_reaped(pid: int, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = _process_state(pid)
        if state is None or state.startswith("Z"):
            return
        time.sleep(0.05)
    raise AssertionError(f"process {pid} is still alive with state {_process_state(pid)}")


def test_cleanup_scripts_have_valid_bash_syntax() -> None:
    for script in (TRIAL_SCRIPT, CLEANUP_LIBRARY):
        subprocess.run(["bash", "-n", str(script)], check=True)


def test_trial_uses_isolated_owned_groups_and_explicit_signal_exit() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert "source scripts/e2e/process_group_cleanup.sh" in source
    assert 'stack_pgid="${stack_pid}"' in source
    assert 'recorder_pgid="${recorder_pid}"' in source
    assert 'desktop_pgid="${desktop_pid}"' in source
    assert "setsid scripts/e2e/record_turn_dynamics.sh" in source
    assert "setsid ffmpeg" in source
    assert "trap 'on_signal 130' INT" in source
    assert "trap 'on_signal 143' TERM" in source
    assert "pkill" not in source
    assert "ros2 node kill" not in source


def test_trial_preserves_recommended_profile_and_renders_animation() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert "--recommended" in source
    assert "--visualize" in source
    assert "--trajectory-stability" in source
    assert "TRAJECTORY_STABILITY_CANDIDATE=%s" in source
    assert "TIGHT_CORRIDOR_CANDIDATE=%s" in source
    assert "stack_command=(scripts/e2e/run_route_vad_fast.sh --recommended)" in source
    assert "stack_command+=(--visualize)" in source
    assert "stack_command+=(--tight-corridor)" in source
    assert "vad_carla_tiny_recommended.param.yaml" in source
    assert "sensor_mapping_vad_fast_reliable_imu.yaml" in source
    assert "VAD_IMU_ACCELERATION_ENABLED=true" in source
    assert "mpc_carla_recommended.param.yaml" in source
    assert "RECOMMENDED=%s" in source
    assert "VISUALIZE=%s" in source
    assert "CAPTURE_DESKTOP=%s" in source
    assert "CLOSED_LOOP_VALIDATION_STATE=%s" in source
    assert "TRAJECTORY_LOGIC_SHA256=%s" in source
    assert "VAD_ROUTE_MANAGER_SHA256=%s" in source
    assert "trajectory_code_provenance" in source
    assert "MPC_PARAM_SHA256=%s" in source
    assert "COMFORTABLE_DECELERATION_MPS2=%s" in source
    assert "--steering-report-mode virtual" in source
    assert "turn_path_control.gif" in source
    assert "--crop motion" in source
    assert "^data: ready$" in source
    assert "^data: fault:" in source
    assert "^data: stopping$" in source
    assert "/planning/vad/candidate_trajectories" in source
    assert "autoware_internal_planning_msgs/msg/CandidateTrajectories" in source
    assert "autoware_rviz_fullscreen.png" in source
    assert "autoware_rviz_drive.gif" in source
    assert "desktop_capture.json" in source
    assert "critical_stack_child_failure" in source
    assert "exit code .*mission_planner_container" in source
    assert "critical_process_failure.log" in source
    assert "Critical Autoware mission-planner process exited" in source
    assert "setsid scripts/e2e/route_test.sh" in source
    assert 'while kill -0 "${route_test_pid}"' in source
    assert "probe_carla_server.py" in source
    assert "carla_preflight_health.json" in source
    assert "carla_completion_health.json" in source
    assert "Pre-created matrix output directory violates" in source
    assert 'children != [output / "carla_server.log"]' in source
    assert "matrix CARLA server log is not bound" in source
    assert "require_carla_owner route_readiness" in source
    assert "require_carla_owner route_evaluation" in source
    assert "require_carla_owner route_completion" in source
    assert "socket.create_connection" not in source
    assert "exited during route evaluation" in source
    assert "exited at route completion" in source
    assert "exited before evidence finalization" in source
    assert "VSCODE_SNAP_GUI_ENV_SANITIZED=%s" in source
    assert "unset GIO_LAUNCHED_DESKTOP_FILE" in source
    assert "unset GTK_EXE_PREFIX GTK_IM_MODULE_FILE GTK_PATH XDG_DATA_HOME" in source
    assert 'export XDG_DATA_DIRS="${XDG_DATA_DIRS_VSCODE_SNAP_ORIG}"' in source
    assert "GTK_IM_MODULE/QT_IM_MODULE" in source
    assert '"capture_started_after_candidate": True' in source
    assert source.count("--no-daemon") >= 2
    assert "ready|stopping" not in source


def test_trial_records_and_enforces_guarded_speed_30_contract() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert "--speed-30kph" in source
    assert "SPEED_PROFILE_ID=%s" in source
    assert "TARGET_SPEED_MPS=%s" in source
    assert "TARGET_SPEED_KPH=30.0" in source
    assert "SPEED_LIMIT_SOURCE=explicit_simulation_profile" in source
    assert "LONGITUDINAL_SPEED_SOURCE=explicit_simulation_profile" in source
    assert "LONGITUDINAL_PID_MAX_OUT_MPS2=1.5" in source
    assert "LONGITUDINAL_PID_MAX_P_EFFORT_MPS2=1.5" in source
    assert "VAD_GEOMETRY_SOURCE=true" in source
    assert "REAL_VEHICLE_READY=false" in source
    assert 'maneuver_lookahead_m="4.0"' in source
    assert "--maneuver-lookahead-m" in source
    assert 'minimum_sustained_speed_mps="7.5"' in source
    assert 'minimum_sustained_speed_sec="1.0"' in source
    assert 'maximum_observed_speed_mps="9.0"' in source
    assert 'maximum_lateral_acceleration_limit_mps2="1.8"' in source
    assert "--min-sustained-speed" in source
    assert "--max-observed-speed" in source
    assert "--max-lateral-acceleration" in source
    assert "--max-speed-sample-gap" in source
    assert "MAXIMUM_SPEED_SAMPLE_GAP_SEC=0.25" in source
    assert "speed_profile_provenance" in source
    assert "VEHICLE_CMD_GATE_PARAM_SHA256=%s" in source
    assert "LONGITUDINAL_CONTROLLER_PARAM_SHA256=%s" in source
    assert "longitudinal_controller.param.yaml.metadata.json" in source
    assert "vehicle_cmd_gate.params.yaml" in source
    assert 'speed_exposure_mode="curvature_limited_turn"' in source
    assert "scripts/e2e/analyze_actuation_map_coverage.py" in source
    assert (
        'actuation_coverage_arguments+=(--allow-target-envelope-beyond-axis)'
        in source
    )
    assert "ACTUATION_MAP_COVERAGE_STATUS" in source
    assert "ACTUATION_MAP_TARGET_ENVELOPE_CLASSIFICATION" in source
    assert "ACTUATION_TARGET_WITHIN_MAP_VELOCITY_AXIS" in source
    assert "actuation_map_runtime_coverage.json" in source
    assert "--observed-maximum-speed-mps" in source
    assert "scripts/e2e/analyze_longitudinal_response.py" in source
    assert "longitudinal_response_analysis.log" in source
    assert "--actuation-map-coverage" in source


def test_trial_records_isolated_speed_30_control_ab_candidates() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert "--control-ab-pid-i40" in source
    assert "--control-ab-turn-preview-5m" in source
    assert "pid_carla_vad_30kph_i40_ab.param.yaml" in source
    assert 'curvature_speed_preview_m="5.0"' in source
    assert "CONTROL_AB_CANDIDATE_ID=%s" in source
    assert "CONTROL_AB_ISOLATED_SINGLE_KNOB=true" in source
    assert 'stack_command+=(--control-ab-pid-i40)' in source
    assert 'stack_command+=(--control-ab-turn-preview-5m)' in source


@pytest.mark.parametrize(
    "arguments",
    (
        ("--control-ab-pid-i40",),
        ("--control-ab-turn-preview-5m",),
        (
            "--speed-30kph",
            "--control-ab-pid-i40",
            "--control-ab-turn-preview-5m",
        ),
    ),
)
def test_trial_rejects_invalid_control_ab_selection(arguments: tuple[str, ...]) -> None:
    completed = subprocess.run(
        [str(TRIAL_SCRIPT), *arguments],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert "A/B" in completed.stderr


def test_trial_records_and_enforces_straight_speed_60_pilot_contract() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert "--speed-60kph-pilot" in source
    assert 'speed_profile_id="carla_vad_60kph_straight_pilot_v1"' in source
    assert 'target_speed_mps="16.666666666666668"' in source
    assert "TARGET_SPEED_KPH=60.0" in source
    assert 'minimum_sustained_speed_mps="15.0"' in source
    assert 'minimum_sustained_speed_sec="1.0"' in source
    assert 'maximum_observed_speed_mps="18.0"' in source
    assert 'maximum_lateral_acceleration_limit_mps2="1.2"' in source
    assert 'maximum_lateral_acceleration_mps2="1.0"' in source
    assert 'maneuver_lookahead_m="6.0"' in source
    assert 'maneuver_exit_lookahead_m="3.5"' in source
    assert 'curvature_speed_preview_m="6.0"' in source
    assert 'route_curvature_lookahead_m="40.0"' in source
    assert 'max_route_deviation_m="1.0"' in source
    assert "SPEED_60KPH_PILOT=%s" in source
    assert "SIMULATION_ONLY_EXPLORATORY=true" in source
    assert "ROUTE_SCOPE=straight_only" in source
    assert "carla_60kph_straight_pilot_v1_exploratory" in source
    assert "REAL_VEHICLE_READY=false" in source
    assert 'stack_command+=(--speed-60kph-pilot)' in source
    assert "vehicle_cmd_gate_carla_60kph_pilot.param.yaml" in source
    assert "pid_carla_vad_60kph_pilot.param.yaml" in source


def test_trial_records_and_forwards_isolated_speed_60_geometry_candidate() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert "--geometry-ab-route-corridor-0p2" in source
    assert 'geometry_ab_candidate_id="baseline_corridor_0p5"' in source
    assert 'geometry_ab_candidate_id="route_corridor_0p2"' in source
    assert "GEOMETRY_AB_CANDIDATE_ID=%s" in source
    assert "GEOMETRY_AB_ROUTE_CORRIDOR_0P2=%s" in source
    assert "GEOMETRY_AB_BEHAVIORAL_SINGLE_KNOB=true" in source
    assert "GEOMETRY_AB_PARAMETER_CHANGE_COUNT=2" in source
    assert "GEOMETRY_AB_COUPLED_PARAMETER_REASON=" in source
    assert "ROUTE_CORRIDOR_HALF_WIDTH_M=%s" in source
    assert "TURN_OUTWARD_CORRIDOR_HALF_WIDTH_M=%s" in source
    assert 'stack_command+=(--geometry-ab-route-corridor-0p2)' in source


def test_trial_rejects_geometry_candidate_without_speed_60() -> None:
    completed = subprocess.run(
        [str(TRIAL_SCRIPT), "--geometry-ab-route-corridor-0p2"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert "requires --speed-60kph-pilot" in completed.stderr


def test_trial_protects_geometry_candidate_from_trailing_override() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert "route_corridor_half_width_m:=*" in source
    assert "turn_outward_corridor_half_width_m:=*" in source


def test_trial_rejects_speed_30_and_speed_60_together() -> None:
    completed = subprocess.run(
        [str(TRIAL_SCRIPT), "--speed-30kph", "--speed-60kph-pilot"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert "mutually exclusive" in completed.stderr


def test_trial_rejects_non_straight_speed_60_route(tmp_path: Path) -> None:
    route = tmp_path / "turn.json"
    route.write_text('{"town": "Town01", "scenario": "left"}\n', encoding="utf-8")
    completed = subprocess.run(
        [
            str(TRIAL_SCRIPT),
            "--speed-60kph-pilot",
            str(tmp_path / "result"),
            str(route),
        ],
        check=False,
        capture_output=True,
        text=True,
        env={
            key: value
            for key, value in os.environ.items()
            if key != "AUTOWARE_E2E_NVIDIA_COMPAT_ROOT"
        },
    )

    assert completed.returncode == 2
    assert "requires a straight route" in completed.stderr
    assert not (tmp_path / "result").exists()


def test_visualize_is_available_to_the_full_profile() -> None:
    completed = subprocess.run(
        [str(TRIAL_SCRIPT), "--visualize"],
        check=False,
        capture_output=True,
        text=True,
        env={
            key: value
            for key, value in os.environ.items()
            if key != "AUTOWARE_E2E_NVIDIA_COMPAT_ROOT"
        },
    )

    assert completed.returncode == 2
    assert "Usage: run_recorded_route_trial.sh" in completed.stderr
    assert "--visualize requires --recommended" not in completed.stderr


def test_desktop_capture_requires_visualization() -> None:
    completed = subprocess.run(
        [str(TRIAL_SCRIPT), "--capture-desktop"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert "--capture-desktop requires --visualize" in completed.stderr


def test_desktop_dimension_probe_does_not_sigpipe_under_pipefail() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert "END {print dimensions}" in source
    assert "awk '/dimensions:/{print $2; exit}'" not in source


@pytest.mark.parametrize(
    "experimental_option",
    (
        "--smart-mpc",
        "--fp16-heads",
        "--model-override",
        "--sensor-mapping",
    ),
)
def test_recommended_rejects_experimental_modes(experimental_option: str) -> None:
    command = [str(TRIAL_SCRIPT), "--recommended", experimental_option]
    if experimental_option in {"--model-override", "--sensor-mapping"}:
        command.append("unused.yaml")

    completed = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        env={
            key: value
            for key, value in os.environ.items()
            if key != "AUTOWARE_E2E_NVIDIA_COMPAT_ROOT"
        },
    )

    assert completed.returncode != 0
    assert "--recommended cannot be combined" in completed.stderr


def test_recommended_rejects_incorrect_analysis_label(tmp_path: Path) -> None:
    completed = subprocess.run(
        [
            str(TRIAL_SCRIPT),
            "--recommended",
            "--mpc-input-delay",
            "0.24",
            str(tmp_path / "result"),
            str(ROOT / "data/routes/town01_fast_left_clear_noon.json"),
        ],
        check=False,
        capture_output=True,
        text=True,
        env={
            key: value
            for key, value in os.environ.items()
            if key != "AUTOWARE_E2E_NVIDIA_COMPAT_ROOT"
        },
    )

    assert completed.returncode != 0
    assert "--recommended fixes MPC delay at 0.12 s" in completed.stderr


def test_recommended_rejects_protected_launch_argument(tmp_path: Path) -> None:
    completed = subprocess.run(
        [
            str(TRIAL_SCRIPT),
            "--recommended",
            str(tmp_path / "result"),
            str(ROOT / "data/routes/town01_fast_left_clear_noon.json"),
            "maximum_speed_mps:=9.0",
        ],
        check=False,
        capture_output=True,
        text=True,
        env={
            key: value
            for key, value in os.environ.items()
            if key != "AUTOWARE_E2E_NVIDIA_COMPAT_ROOT"
        },
    )

    assert completed.returncode != 0
    assert "Recommended profile argument is controlled" in completed.stderr


@pytest.mark.parametrize(
    "protected",
    (
        "vehicle_cmd_gate_param_path:=/tmp/unsafe.yaml",
        "maximum_longitudinal_acceleration_mps2:=9.0",
        "candidate_timeout_sec:=60.0",
    ),
)
def test_speed_30_rejects_protected_launch_argument(
    tmp_path: Path, protected: str
) -> None:
    completed = subprocess.run(
        [
            str(TRIAL_SCRIPT),
            "--speed-30kph",
            str(tmp_path / "result"),
            str(ROOT / "data/routes/town01_fast_left_clear_noon.json"),
            protected,
        ],
        check=False,
        capture_output=True,
        text=True,
        env={
            key: value
            for key, value in os.environ.items()
            if key != "AUTOWARE_E2E_NVIDIA_COMPAT_ROOT"
        },
    )

    assert completed.returncode != 0
    assert "Recommended profile argument is controlled" in completed.stderr


@pytest.mark.parametrize("speed_option", ("--speed-30kph", "--speed-60kph-pilot"))
@pytest.mark.parametrize("experimental", ("--tight-corridor", "--trajectory-stability"))
def test_speed_profiles_reject_experimental_combinations(
    speed_option: str, experimental: str
) -> None:
    completed = subprocess.run(
        [str(TRIAL_SCRIPT), speed_option, experimental],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 2
    assert "must be screened independently" in completed.stderr


def test_cleanup_stops_group_after_original_leader_exits(tmp_path: Path) -> None:
    state_file = tmp_path / "group.txt"
    leader = subprocess.Popen(
        [
            "setsid",
            "bash",
            "-c",
            (
                "trap 'exit 0' INT; "
                "bash -c 'trap \"\" INT TERM; while :; do sleep 1; done' & "
                f"printf '%s %s\\n' \"$$\" \"$!\" > {state_file}; "
                "while :; do sleep 0.1; done"
            ),
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    child_pid: int | None = None
    try:
        deadline = time.monotonic() + 3.0
        while not state_file.exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert state_file.exists()
        group_leader_pid, child_pid = map(
            int, state_file.read_text(encoding="utf-8").split()
        )
        assert group_leader_pid == leader.pid

        os.kill(group_leader_pid, signal.SIGINT)
        leader.wait(timeout=3.0)
        assert _process_state(child_pid) is not None

        completed = subprocess.run(
            [
                "bash",
                "-c",
                (
                    f"source {CLEANUP_LIBRARY}; "
                    f"e2e_stop_owned_process_group {group_leader_pid} "
                    f"{group_leader_pid} 1 1 2"
                ),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=8.0,
        )
        assert completed.returncode == 0, completed.stderr
        _wait_until_reaped(child_pid)
    finally:
        if child_pid is not None and _process_state(child_pid) is not None:
            os.kill(child_pid, signal.SIGKILL)
        if leader.poll() is None:
            os.killpg(leader.pid, signal.SIGKILL)
            leader.wait(timeout=3.0)
