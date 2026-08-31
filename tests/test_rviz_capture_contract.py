from __future__ import annotations

from pathlib import Path
import subprocess

import pytest
import yaml


ROOT = Path(__file__).parents[1]
FULL_RVIZ = ROOT / "autoware_e2e_vad_launch/rviz/autoware_vad_carla.rviz"
LIGHTWEIGHT_RVIZ = ROOT / "autoware_e2e_vad_launch/rviz/vad_carla.rviz"
TRIAL_SCRIPT = ROOT / "scripts/e2e/run_recorded_route_trial.sh"


def _load_config(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _named_display(value: object, name: str) -> dict:
    if isinstance(value, dict):
        if value.get("Name") == name:
            return value
        for child in value.values():
            try:
                return _named_display(child, name)
            except KeyError:
                pass
    elif isinstance(value, list):
        for child in value:
            try:
                return _named_display(child, name)
            except KeyError:
                pass
    raise KeyError(name)


@pytest.mark.parametrize("config_path", (FULL_RVIZ, LIGHTWEIGHT_RVIZ))
def test_rviz_current_view_tracks_ego_from_the_viewport_center(
    config_path: Path,
) -> None:
    config = _load_config(config_path)
    views = config["Visualization Manager"]["Views"]
    current = views["Current"]

    assert current["Class"] == "rviz_default_plugins/TopDownOrtho"
    assert current["Name"] == "Follow Ego"
    assert current["Target Frame"] == "base_link"
    assert float(current["Angle"]) == 0.0
    assert float(current["X"]) == 0.0
    assert float(current["Y"]) == 0.0
    assert float(current["Scale"]) == 10.0

    saved_follow = next(
        view for view in views["Saved"] if view.get("Name") == "Follow Ego"
    )
    for key in ("Class", "Target Frame", "Angle", "X", "Y", "Scale"):
        assert saved_follow[key] == current[key]


def test_full_capture_view_keeps_reference_final_and_vad_paths_visible() -> None:
    config = _load_config(FULL_RVIZ)
    required_topics = {
        "/planning/vad_route/reference_path",
        "/planning/vad_route/actual_path",
        "/planning/trajectory",
        "/planning/vad_route/selected_raw_trajectory",
        "/planning/vad/candidate_trajectories",
    }
    visible_topics: set[str] = set()

    def visit(value: object) -> None:
        if isinstance(value, dict):
            topic = value.get("Topic")
            topic_name = topic.get("Value") if isinstance(topic, dict) else topic
            if (
                topic_name in required_topics
                and value.get("Enabled") is True
                and value.get("Value") is True
            ):
                visible_topics.add(topic_name)
            for child in value.values():
                visit(child)
        elif isinstance(value, list):
            for child in value:
                visit(child)

    visit(config["Visualization Manager"]["Displays"])
    assert visible_topics == required_topics


def test_capture_views_disable_odometry_trails_and_deemphasize_candidates() -> None:
    full = _load_config(FULL_RVIZ)
    lightweight = _load_config(LIGHTWEIGHT_RVIZ)

    kinematic_state = _named_display(full, "Kinematic State")
    assert kinematic_state["Keep"] == 1
    assert kinematic_state["Covariance"]["Value"] is False
    assert kinematic_state["Covariance"]["Orientation"]["Value"] is False
    assert kinematic_state["Covariance"]["Position"]["Value"] is False

    vehicle_motion = _named_display(lightweight, "Vehicle Motion")
    assert vehicle_motion["Keep"] == 1

    for config in (full, lightweight):
        candidates = _named_display(config, "VAD Candidate Trajectories")
        assert float(candidates["View Path"]["Alpha"]) == pytest.approx(0.22)
        assert float(candidates["View Path"]["Width"]) == pytest.approx(0.04)


def test_desktop_still_is_selected_from_the_route_recording() -> None:
    subprocess.run(["bash", "-n", str(TRIAL_SCRIPT)], check=True)
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    candidate_index = source.index("autoware_rviz_candidate.png")
    recording_index = source.index("autoware_rviz_capture.mkv")
    representative_index = source.index(
        '"${output_dir}/autoware_rviz_fullscreen.png"', recording_index
    )
    assert candidate_index < recording_index < representative_index
    assert "ffprobe -v error -show_entries format=duration" in source
    assert '"${representative_offset_sec}" -frames:v 1' in source
    assert '"selection": "route_evaluation_midpoint"' in source
    assert '"candidate_png_file": "autoware_rviz_candidate.png"' in source


def test_desktop_capture_pins_the_owned_rviz_window_above_normal_windows() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert "for command in ffmpeg ffprobe xdpyinfo xprop xwininfo" in source
    assert "pin_rviz_capture_window()" in source
    assert "pin_rviz_capture_window\n" in source
    assert "_NET_WM_STATE_ABOVE" in source
    assert "RVIZ_CAPTURE_FOREGROUND_GUARD=rviz_above" in source


def test_capture_timestamps_are_python_isoformat_compatible() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert source.count("%Y-%m-%dT%H:%M:%S.%6NZ") == 5
    assert "%Y-%m-%dT%H:%M:%S.%NZ" not in source


def test_desktop_metadata_records_a_fail_closed_centered_view_contract() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert 'capture_launch_arguments+=("rviz_config:=${capture_rviz_runtime_config}")' in source
    assert "captured RViz config changed after preflight validation" in source
    assert '"controller": "rviz_default_plugins/TopDownOrtho"' in source
    assert '"target_frame": "base_link"' in source
    assert '"center_xy_m": [0.0, 0.0]' in source
    assert '"scale": 10.0' in source
    assert '"vehicle_centered": True' in source
    assert '"visible_path_topics": sorted(visible_path_topics)' in source
