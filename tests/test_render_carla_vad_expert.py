from __future__ import annotations

from pathlib import Path

from PIL import Image

from scripts.e2e import render_carla_vad_expert as renderer


def test_sample_future_world_rotates_anchor_relative_label() -> None:
    sample = {
        "pose_map_xyz_yaw": [10.0, 20.0, 0.0, 3.141592653589793 / 2.0],
        "future_expert": {"positions_xy": [[2.0, 0.0], [2.0, 1.0]]},
    }

    points = renderer._sample_future_world(sample)

    assert abs(points[1][0] - 10.0) < 1.0e-9
    assert abs(points[1][1] - 22.0) < 1.0e-9
    assert abs(points[2][0] - 9.0) < 1.0e-9
    assert abs(points[2][1] - 22.0) < 1.0e-9


def test_frame_selection_keeps_endpoints_and_is_bounded() -> None:
    samples = [{"frame": index} for index in range(1000)]

    selected = renderer._select_indices(samples, 80)

    assert selected[0] == 0
    assert selected[-1] == 999
    assert len(selected) == 80
    assert selected == sorted(set(selected))


def test_overview_prefers_middle_turn_frame() -> None:
    samples = [
        {"vad_command_0based": 3},
        {"vad_command_0based": 1},
        {"vad_command_0based": 1},
        {"vad_command_0based": 3},
    ]

    assert renderer._overview_index(samples) == 2


def test_draw_camera_creates_a_nonblank_labeled_panel(tmp_path: Path) -> None:
    image_path = tmp_path / "front.jpg"
    Image.new("RGB", (640, 360), "#2878b5").save(image_path)
    sample = {"cameras": {"CAM_FRONT": {"path": image_path.name}}}
    canvas = Image.new("RGB", (400, 250), "white")

    renderer._draw_camera(canvas, tmp_path, sample, "CAM_FRONT", (10, 10, 390, 230))

    colors = canvas.getcolors(maxcolors=1_000_000)
    assert colors is not None
    assert len(colors) > 3
