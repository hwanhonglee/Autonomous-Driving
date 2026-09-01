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


def test_full_capture_view_embeds_the_front_camera_panel() -> None:
    config = _load_config(FULL_RVIZ)
    camera = _named_display(config["Visualization Manager"]["Displays"], "VAD Front Camera")

    assert camera["Enabled"] is True
    assert camera["Value"] is True
    assert camera["Topic"]["Value"] == "/sensing/camera/CAM_FRONT/image_raw"


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


def test_desktop_capture_selects_one_owned_rviz_window_without_desktop_side_effects() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert "for command in ffmpeg ffprobe xdpyinfo xprop xwininfo" in source
    assert "matching_owned_rviz_capture_windows()" in source
    assert "inspect_owned_rviz_capture_window()" in source
    assert "prepare_owned_rviz_capture_window()" in source
    assert "prepare_owned_rviz_capture_window\n" in source
    assert "${#matching_windows[@]} != 1" in source
    assert 'grep -Fq -- "${capture_rviz_runtime_config}"' in source
    assert '"rviz2",[[:space:]]*"rviz2"' in source
    assert "_NET_WM_PID" in source
    assert '"${observed_rviz_window_pgid}" != "${stack_pgid}"' in source
    assert '"${observed_rviz_window_map_state}" != "IsViewable"' in source
    assert "_NET_WM_STATE_MAXIMIZED_HORZ" in source
    assert "_NET_WM_STATE_MAXIMIZED_VERT" in source
    assert "_NET_WM_STATE_ABOVE" not in source
    assert "XWarpPointer" not in source
    assert "_NET_CLIENT_LIST_STACKING" not in source
    assert "xdotool" not in source
    assert '${desktop_display}+0,0' not in source


def test_owned_window_capture_is_revalidated_while_the_recorder_is_alive() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    call_index = source.index("\nprepare_owned_rviz_capture_window\n")
    candidate_wait_index = source.index(
        "if ! timeout 30 ros2 topic echo /planning/vad/candidate_trajectories",
        call_index,
    )
    candidate_png_index = source.index("autoware_rviz_candidate.png", call_index)
    assert call_index < candidate_wait_index < candidate_png_index
    assert "verify_owned_rviz_capture_window candidate_pre" in source
    assert "verify_owned_rviz_capture_window candidate_post" in source
    assert "require_desktop_recorder recording_started" in source
    assert "verify_owned_rviz_capture_window recording_started" in source
    assert "desktop_recorder_alive" in source
    assert "verify_owned_rviz_capture_window representative" in source
    assert "require_desktop_recorder representative" in source
    assert "Owned RViz geometry changed during" in source
    assert "Owned RViz PID/PGID changed or left the stack group" in source


def test_owned_window_capture_pads_to_1920x1080_without_scaling() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert "capture_output_width_px=1920" in source
    assert "capture_output_height_px=1080" in source
    assert "observed_rviz_window_width_px < 1280" in source
    assert "observed_rviz_window_height_px < 720" in source
    assert "observed_rviz_window_width_px > capture_output_width_px" in source
    assert "observed_rviz_window_height_px > capture_output_height_px" in source
    assert "capture_pad_left_px=$(((capture_output_width_px" in source
    assert "capture_pad_right_px=$((capture_output_width_px" in source
    assert "capture_pad_top_px=$(((capture_output_height_px" in source
    assert "capture_pad_bottom_px=$((capture_output_height_px" in source
    assert "color=black,setsar=1" in source
    assert source.count('-window_id "${capture_rviz_window_id_decimal}"') == 2
    assert source.count('-video_size "${capture_rviz_input_dimensions}"') == 2
    assert source.count('-i "${desktop_display}"') == 2
    assert source.count('-vf "${capture_pad_filter}"') == 2
    assert '"${DISPLAY}" "${capture_output_dimensions}"' in source
    assert '"method": "ffmpeg_x11grab_owned_window_v1"' in source
    assert '"root_capture": False' in source
    assert '"shell_surfaces_excluded": True' in source
    assert '"input_dimensions": input_dimensions' in source
    assert '"padding_px": padding_px' in source
    assert '"scaling": "none"' in source


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


def test_owned_window_capture_structurally_excludes_shell_overlays() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert "top_center_bright_fraction" not in source
    assert '"method": "owned_window_excludes_shell_surfaces_v1"' in source
    assert '"root_capture": False' in source
    assert '"shell_surfaces_excluded": True' in source
    assert '"passed": True' in source


def test_desktop_capture_uses_embedded_camera_without_external_rqt() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert "stack_command+=(--rviz-only)" in source
    assert "RVIZ_CAPTURE_CAMERA_SOURCE=rviz_embedded_vad_front_camera" in source
    assert "RVIZ_CAPTURE_EXTERNAL_CAMERA_VIEW=false" in source
    assert '"embedded_rviz_display": "VAD Front Camera"' in source
    assert (
        '"embedded_rviz_topic": "/sensing/camera/CAM_FRONT/image_raw"'
        in source
    )
    assert '"external_rqt_image_view_launched": False' in source


def test_owned_window_runtime_provenance_is_fail_closed() -> None:
    source = TRIAL_SCRIPT.read_text(encoding="utf-8")

    assert source.startswith("#!/usr/bin/env bash\nset -euo pipefail\n")
    for field in (
        "RVIZ_CAPTURE_SOURCE=ffmpeg_x11grab_owned_window_v1",
        "RVIZ_CAPTURE_ROOT=false",
        "RVIZ_CAPTURE_SHELL_SURFACES_EXCLUDED=true",
        "RVIZ_CAPTURE_SCALE_APPLIED=false",
        "RVIZ_CAPTURE_WINDOW_ID=%s",
        "RVIZ_CAPTURE_WINDOW_ID_DECIMAL=%s",
        "RVIZ_CAPTURE_WINDOW_TITLE_CONFIG_MATCH=true",
        "RVIZ_CAPTURE_WINDOW_CLASS=rviz2",
        "RVIZ_CAPTURE_WINDOW_PID=%s",
        "RVIZ_CAPTURE_WINDOW_PGID=%s",
        "RVIZ_CAPTURE_STACK_PGID=%s",
        "RVIZ_CAPTURE_WINDOW_MAP_STATE=IsViewable",
        "RVIZ_CAPTURE_INPUT_WIDTH_PX=%s",
        "RVIZ_CAPTURE_INPUT_HEIGHT_PX=%s",
        "RVIZ_CAPTURE_PAD_LEFT_PX=%s",
        "RVIZ_CAPTURE_PAD_TOP_PX=%s",
        "RVIZ_CAPTURE_PAD_RIGHT_PX=%s",
        "RVIZ_CAPTURE_PAD_BOTTOM_PX=%s",
        "RVIZ_CAPTURE_SCALING=none",
        "RVIZ_CAPTURE_GEOMETRY_STABLE=true",
    ):
        assert field in source
