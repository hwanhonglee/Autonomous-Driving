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


def test_ego_centered_transform_keeps_vehicle_centered_and_heading_up() -> None:
    ego = (100.0, 50.0)
    transform, _ = renderer._ego_centered_transform(
        ego,
        3.141592653589793 / 2.0,
        [ego, (100.0, 90.0), (90.0, 50.0)],
        (0, 0, 400, 300),
    )

    assert transform(ego) == (200, 150)
    assert transform((100.0, 60.0))[1] < 150  # forward is up
    assert transform((90.0, 50.0))[0] < 200  # ego-left is screen-left


def test_common10_future_timing_is_derived_from_sample() -> None:
    sample = {
        "future_expert": {
            "horizons_s": [(index + 1) / 10.0 for index in range(64)],
            "positions_xy": [[float(index), 0.0] for index in range(64)],
        }
    }

    assert renderer._future_timing_summary(sample) == "10 Hz, 0.1\u20136.4 s (64 points)"
    assert renderer._future_legend_label(sample) == "future label (to 6.4 s)"


def test_legacy_future_without_timing_does_not_claim_a_rate() -> None:
    sample = {"future_expert": {"positions_xy": [[1.0, 0.0], [2.0, 0.0]]}}

    assert renderer._future_timing_summary(sample) == "timing not exported (2 points)"
    assert renderer._future_legend_label(sample) == "future label (2 points)"


def test_map_panel_draws_ego_marker_at_exact_panel_center() -> None:
    canvas = Image.new("RGB", (600, 500), "#eeeeee")
    box = (20, 20, 580, 480)
    sample = {
        "pose_map_xyz_yaw": [100.0, 10.0, 0.0, 0.0],
        "route_progress_m": 100.0,
        "future_expert": {
            "horizons_s": [0.1, 0.2, 0.3],
            "positions_xy": [[1.0, 0.0], [2.0, 0.2], [3.0, 0.5]],
        },
    }
    route = [(-1000.0, 0.0), (100.0, 10.0), (2000.0, 100.0)]
    history = [(80.0, 10.0), (90.0, 10.0), (100.0, 10.0)]
    future = renderer._sample_future_world(sample)

    renderer._draw_map_panel(canvas, box, route, history, history, future, sample)

    center = ((box[0] + box[2]) // 2, (box[1] + box[3]) // 2)
    assert canvas.getpixel(center) == (27, 31, 35)
    assert canvas.getpixel((0, 0)) == (238, 238, 238)


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
