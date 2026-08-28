from __future__ import annotations

from pathlib import Path
import subprocess


ROOT = Path(__file__).parents[1]
SCRIPT = ROOT / "scripts/e2e/record_turn_dynamics.sh"


def test_record_turn_dynamics_has_valid_bash_syntax() -> None:
    subprocess.run(["bash", "-n", str(SCRIPT)], check=True)


def test_record_turn_dynamics_shell_contract() -> None:
    source = SCRIPT.read_text(encoding="utf-8")

    assert "set -euo pipefail" in source
    assert "source scripts/e2e/env.sh" in source
    assert '[[ -e "${output_bag}" || -L "${output_bag}" ]]' in source
    assert 'flock -n 9' in source
    assert 'ros2 node list --no-daemon' in source
    assert 'grep -Fxq "/rosbag2_recorder"' in source
    assert 'exec ros2 bag record --output "${output_bag}" --regex "${topic_regex}"' in source
    assert "--no-discovery" not in source
    assert "--include-unpublished-topics" not in source


def test_record_turn_dynamics_supports_full_and_minimal_topic_contracts() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    expected_topics = (
        "clock",
        "localization/kinematic_state",
        "localization/acceleration",
        "sensing/camera/CAM_FRONT/camera_info",
        "sensing/camera/CAM_BACK/camera_info",
        "sensing/camera/CAM_FRONT_LEFT/camera_info",
        "sensing/camera/CAM_BACK_LEFT/camera_info",
        "sensing/camera/CAM_FRONT_RIGHT/camera_info",
        "sensing/camera/CAM_BACK_RIGHT/camera_info",
        "planning/vad/candidate_trajectories",
        "planning/vad/raw_trajectory",
        "planning/vad_route/selected_raw_trajectory",
        "perception/vad/map_points",
        "perception/object_recognition/objects",
        "planning/vad_route/command",
        "planning/vad_route/cross_track_error",
        "planning/vad_route/trajectory_correction",
        "planning/vad_route/remaining_distance",
        "planning/vad_route/status",
        "planning/trajectory",
        "control/trajectory_follower/lateral/predicted_trajectory",
        "control/trajectory_follower/predicted_trajectory",
        "control/trajectory_follower/lateral/diagnostic",
        "control/trajectory_follower/controller_node_exe/lateral/debug/processing_time_ms",
        "control/trajectory_follower/control_cmd",
        "trajectory_follower/control_cmd",
        "control/command/control_cmd",
        "control/command/actuation_cmd",
        "vehicle/status/steering_status",
        "system/fail_safe/mrm_state",
        "api/operation_mode/state",
    )

    for topic in expected_topics:
        assert f"topic_regex+='{topic}" in source
    assert "topic_regex='^/(" in source
    assert "topic_regex+=')$'" in source
