from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from PIL import Image
import pytest

from scripts.e2e import export_carla_vad_expert as exporter


def _state(index: int, command: int = 1) -> dict[str, object]:
    time = index * 0.05
    return {
        "frame": 100 + index,
        "timestamp_s": time,
        "x": 2.0 * time,
        "y": 0.0,
        "z": 0.0,
        "yaw": 0.0,
        "velocity": {"x": 2.0, "y": 0.0},
        "acceleration": {"x": 0.0, "y": 0.0},
        "yaw_rate": 0.0,
        "command": command,
        "route_progress_m": 2.0 * time,
        "route_cte_m": 0.05,
    }


def _write_episode(
    path: Path, collision: bool = False, lane_invasion_count: int = 0
) -> None:
    path.mkdir()
    (path / "manifest.json").write_text(
        json.dumps({"episode_id": "unit", "town": "Town01", "expert": "carla.BasicAgent"}),
        encoding="utf-8",
    )
    (path / "route.json").write_text(
        json.dumps({"town": "Town01", "weather": "ClearNoon", "scenario": "right"}),
        encoding="utf-8",
    )
    states = [_state(index) for index in range(81)]
    if collision:
        states[20]["collision"] = True
    for event_index in range(lane_invasion_count):
        states[20 + event_index * 4]["lane_invasion"] = True
    (path / "states.jsonl").write_text(
        "".join(json.dumps(state) + "\n" for state in states), encoding="utf-8"
    )
    frames = []
    for state_index in range(0, 81, 4):
        image_paths = {}
        for camera in exporter.PRIVATE_TINY_CAMERA_ORDER:
            relative = Path("images") / camera / f"{100 + state_index:08d}.jpg"
            target = path / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            Image.new("RGB", (64, 36), (state_index, 10, 20)).save(target)
            image_paths[camera] = relative.as_posix()
        frames.append(
            {
                "frame": 100 + state_index,
                "timestamp_s": state_index * 0.05,
                "images": image_paths,
            }
        )
    (path / "camera_frames.jsonl").write_text(
        "".join(json.dumps(frame) + "\n" for frame in frames), encoding="utf-8"
    )


def test_relative_pose_uses_anchor_heading_and_ros_left_axis() -> None:
    anchor = (10.0, 20.0, 0.0, math.pi / 2.0)

    relative = exporter.relative_pose(anchor, {"x": 10.0, "y": 22.0, "z": 0.0, "yaw": math.pi})

    assert relative == pytest.approx([2.0, 0.0, 0.0, math.pi / 2.0])


def test_future_labels_interpolate_and_do_not_pad_episode_tail() -> None:
    states = [_state(index) for index in range(81)]
    times = [exporter.state_time(state) for state in states]

    labels = exporter.future_labels(states, times, states[0], exporter.DEFAULT_HORIZONS_S)
    tail = exporter.future_labels(states, times, states[40], exporter.DEFAULT_HORIZONS_S)

    assert labels is not None
    for actual, expected in zip(
        labels["positions_xy"],
        [[1.0, 0.0], [2.0, 0.0], [3.0, 0.0], [4.0, 0.0], [5.0, 0.0], [6.0, 0.0]],
    ):
        assert actual == pytest.approx(expected)
    for delta in labels["deltas_xy"]:
        assert delta == pytest.approx([1.0, 0.0])
    assert tail is None


def test_export_writes_validated_samples_and_explicit_camera_adapters(tmp_path: Path) -> None:
    episode = tmp_path / "episode"
    output = tmp_path / "export"
    _write_episode(episode)

    report = exporter.export_episode(episode, output)
    samples = exporter.read_jsonl(output / "samples.jsonl")

    assert report["status"] == "validated"
    assert report["sample_count"] == 6
    assert report["dropped_tail_anchor_count"] == 15
    assert report["camera_order_private_tiny"] == list(exporter.PRIVATE_TINY_CAMERA_ORDER)
    assert report["camera_order_public_b2d"] == list(exporter.PUBLIC_B2D_CAMERA_ORDER)
    assert report["lane_invasion_event_count"] == 0
    assert report["maximum_lane_invasions"] == 0
    assert len(samples) == 6
    assert samples[0]["command_one_hot"] == [0, 1, 0, 0, 0, 0]
    assert samples[0]["cameras"]["CAM_BACK"]["private_tiny_index"] == 1
    assert samples[0]["cameras"]["CAM_BACK"]["public_b2d_index"] == 3
    assert len(samples[0]["can_bus_18"]) == 18
    assert not output.with_name("export.partial").exists()


def test_export_rejects_collision_and_removes_partial_output(tmp_path: Path) -> None:
    episode = tmp_path / "episode"
    output = tmp_path / "export"
    _write_episode(episode, collision=True)

    with pytest.raises(exporter.DatasetError, match="collision"):
        exporter.export_episode(episode, output)

    assert not output.exists()
    assert not output.with_name("export.partial").exists()


def test_export_rejects_lane_invasion_by_default_and_removes_partial_output(
    tmp_path: Path,
) -> None:
    episode = tmp_path / "episode"
    output = tmp_path / "export"
    _write_episode(episode, lane_invasion_count=1)

    with pytest.raises(exporter.DatasetError, match="lane invasion.*configured maximum 0"):
        exporter.export_episode(episode, output)

    assert not output.exists()
    assert not output.with_name("export.partial").exists()


def test_export_allows_explicit_integer_lane_invasion_threshold(tmp_path: Path) -> None:
    episode = tmp_path / "episode"
    output = tmp_path / "export"
    _write_episode(episode, lane_invasion_count=2)

    report = exporter.export_episode(episode, output, maximum_lane_invasions=2)
    persisted = json.loads((output / "manifest.json").read_text(encoding="utf-8"))

    assert report["lane_invasion_event_count"] == 2
    assert report["maximum_lane_invasions"] == 2
    assert persisted["maximum_lane_invasions"] == 2


@pytest.mark.parametrize("value", ["-1", "1.5", "nan", "inf"])
def test_lane_invasion_cli_threshold_rejects_non_integer_or_negative(value: str) -> None:
    with pytest.raises(argparse.ArgumentTypeError, match="nonnegative integer"):
        exporter.parse_nonnegative_integer(value)


@pytest.mark.parametrize("value", [-1, 1.0, True])
def test_export_api_rejects_non_integer_lane_invasion_threshold(
    tmp_path: Path, value: object
) -> None:
    episode = tmp_path / "episode"
    _write_episode(episode)

    with pytest.raises(exporter.DatasetError, match="nonnegative integer"):
        exporter.export_episode(
            episode,
            tmp_path / "export",
            maximum_lane_invasions=value,  # type: ignore[arg-type]
        )


def test_camera_mapping_order_is_part_of_the_contract(tmp_path: Path) -> None:
    episode = tmp_path / "episode"
    _write_episode(episode)
    records = exporter.read_jsonl(episode / "camera_frames.jsonl")
    first = records[0]
    first["images"] = dict(reversed(list(first["images"].items())))
    (episode / "camera_frames.jsonl").write_text(
        "".join(json.dumps(record) + "\n" for record in records), encoding="utf-8"
    )

    with pytest.raises(exporter.DatasetError, match="keys/order"):
        exporter.export_episode(episode, tmp_path / "export")
