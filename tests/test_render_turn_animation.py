from __future__ import annotations

import importlib.util
from pathlib import Path
import shutil

import numpy as np
from PIL import Image, ImageDraw
import pytest


pytest.importorskip("rosbag2_py")

MODULE_PATH = Path(__file__).parents[1] / "scripts" / "e2e" / "render_turn_animation.py"
SPEC = importlib.util.spec_from_file_location("render_turn_animation", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
renderer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(renderer)


def _record(stamp_sec: float, bag_sec: float, value: str) -> dict[str, object]:
    return {
        "stamp_ns": int(round(stamp_sec * 1.0e9)),
        "bag_ns": int(round(bag_sec * 1.0e9)),
        "value": value,
    }


def test_stamped_series_aligns_by_header_stamp_and_deduplicates_receipts() -> None:
    series = renderer.StampedSeries(
        [
            _record(2.0, 100.0, "old duplicate"),
            _record(1.0, 300.0, "first"),
            _record(2.0, 200.0, "new duplicate"),
        ]
    )

    assert [record["value"] for record in series.records] == ["first", "new duplicate"]
    assert series.latest(0.9) is None
    assert series.latest(1.5)["value"] == "first"
    assert series.latest(2.0)["value"] == "new duplicate"


def test_missing_predicted_trajectory_warns_but_is_allowed(capsys: pytest.CaptureFixture) -> None:
    assert renderer._select_predicted_records({}) == []

    warning = capsys.readouterr().err
    assert "warning: bag has no MPC predicted trajectory" in warning
    assert "rendering without the controller prediction" in warning


def test_legacy_predicted_trajectory_does_not_warn(capsys: pytest.CaptureFixture) -> None:
    legacy_topic = renderer.dynamics.PREDICTED_TOPICS[1]
    records = [{"value": "legacy"}]

    assert renderer._select_predicted_records({legacy_topic: records}) == records
    assert capsys.readouterr().err == ""


def test_motion_crop_uses_first_and_last_moving_header_stamp() -> None:
    odometry = {
        "time": np.arange(6.0),
        "speed": np.asarray([0.0, 0.0, 0.2, 0.1, 0.0, 0.0]),
        "progress": np.arange(6.0),
    }

    start, end, turn_range = renderer._select_interval(
        odometry,
        "motion",
        np.arange(6.0),
        np.full(6, 3),
        padding_sec=1.0,
        motion_threshold_mps=0.05,
        turn_margin_m=0.0,
    )

    assert (start, end) == pytest.approx((1.0, 4.0))
    assert turn_range is None


def test_turn_crop_uses_route_command_progress_range() -> None:
    route_progress = np.arange(11.0)
    route_command = np.full(11, 3)
    route_command[4:7] = 1
    odometry = {
        "time": np.arange(11.0),
        "speed": np.ones(11),
        "progress": np.arange(11.0),
    }

    start, end, turn_range = renderer._select_interval(
        odometry,
        "turn",
        route_progress,
        route_command,
        padding_sec=0.5,
        motion_threshold_mps=0.05,
        turn_margin_m=1.0,
    )

    assert turn_range == pytest.approx((3.0, 7.0))
    assert (start, end) == pytest.approx((2.5, 7.5))


def test_frame_times_are_fixed_rate_and_do_not_exceed_end() -> None:
    times = renderer._frame_times(2.0, 3.05, 5.0)

    assert times.tolist() == pytest.approx([2.0, 2.2, 2.4, 2.6, 2.8, 3.0])
    assert times[-1] <= 3.05


def test_straight_route_corridor_has_requested_half_width() -> None:
    route = np.asarray([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])

    polygon = renderer._corridor_polygon(route, 0.5)

    assert polygon[:3, 1] == pytest.approx(np.full(3, 0.5))
    assert polygon[3:, 1] == pytest.approx(np.full(3, -0.5))


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg is unavailable")
def test_encoder_writes_looping_gif_and_optional_mp4(tmp_path: Path) -> None:
    frame_dir = tmp_path / "frames"
    frame_dir.mkdir()
    for index, color in enumerate(("#1976d2", "#e67e22", "#8e44ad"), start=1):
        image = Image.new("RGB", (640, 360), "white")
        ImageDraw.Draw(image).rectangle((50 * index, 80, 50 * index + 120, 200), fill=color)
        image.save(frame_dir / f"frame_{index:05d}.png")

    output_gif = tmp_path / "replay.gif"
    output_mp4 = tmp_path / "replay.mp4"
    renderer._encode_outputs(frame_dir, 5.0, output_gif, output_mp4)

    assert output_gif.stat().st_size > 0
    assert output_mp4.stat().st_size > 0
    with Image.open(output_gif) as animation:
        assert animation.n_frames == 3
        assert animation.info["loop"] == 0
