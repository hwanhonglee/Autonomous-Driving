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
    assert "sensor_mapping_vad_fast_reliable.yaml" in source
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
    assert '"capture_started_after_candidate": True' in source
    assert source.count("--no-daemon") >= 2
    assert "ready|stopping" not in source


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
