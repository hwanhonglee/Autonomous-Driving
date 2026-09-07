"""HH_260906 - Check diagnostic-only real-frame rendering without a simulator or dataset promotion."""

import json
from pathlib import Path

from PIL import Image
import pytest

from scripts.e2e import render_carla_raw_trial as render


@pytest.fixture
def trial(tmp_path):
    # HH_260906 - Synthetic images here test rendering mechanics, not recorded driving evidence.
    root = tmp_path / "trial"
    episode = root / "episode.partial"
    episode.mkdir(parents=True)
    (root / "lifecycle").mkdir()
    (root / "owner_result.json").write_text(json.dumps({"exit_code": 1, "source_bytes_unchanged_and_archived": True}))
    (root / "lifecycle/stopped.json").write_text(json.dumps({"status": "PASS"}))
    (episode / "manifest.json").write_text(json.dumps({"result": {"goal_stop_quality": {"status": "FAIL"}}}))
    (episode / "route.json").write_text(json.dumps({"town": "Town07", "route": [{"x": 0, "y": 0}, {"x": 100, "y": 0}]}))
    states, cameras = [], []
    for i in range(5):
        states.append({"frame": i, "timestamp": i * .05, "x": i * .1, "y": 0., "z": 0., "yaw": 0.,
                       "vx": 2., "vy": 0., "capture_phase": "driving", "command": 2,
                       "route_progress_m": i * .1, "route_cte_m": 0.,
                       "current_control": {"throttle": .1, "brake": 0., "steer": 0., "gear": 1}})
        if i % 2 == 0:
            images = {}
            for name in render.CAMERAS:
                relative = f"{name}_{i}.png"
                Image.new("RGB", (64, 32), "#ff0000").save(episode / relative)
                images[name] = relative
            cameras.append({"frame": i, "timestamp": i * .05, "images": images,
                            "source_timestamps": {name: i * .05 for name in render.CAMERAS}})
    for name, rows in (("states.jsonl", states), ("camera_frames.jsonl", cameras)):
        (episode / name).write_text("".join(json.dumps(row) + "\n" for row in rows))
    return root


def test_failed_trial_retains_identity_and_terminal_future_is_not_fabricated(trial):
    data = render.load_trial(trial)
    assert data["episode"].name == "episode.partial"
    sample = render.raw_sample(data, data["cameras"][0])
    assert sample["future_expert"]["positions_xy"] == [[.2, 0.], [.4, 0.]]
    assert sample["future_expert"]["horizons_s"] == [.1, .2]
    last = render.raw_sample(data, data["cameras"][-1])
    assert last["future_expert"] == {"positions_xy": []}
    assert render.render_frame(data, 2, 5, 10).size == (1600, 900)
    assert len(data["displayed_image_sha256"]) == 6
    assert data["owner"]["exit_code"] == 1
    assert not (trial / "episode").exists()


def test_full_fov_is_letterboxed_not_cropped(tmp_path):
    image = Image.new("RGB", (80, 20), "red")
    image.paste("blue", (0, 0, 10, 20))
    image.paste("green", (70, 0, 80, 20))
    image.save(tmp_path / "edges.png")
    canvas = Image.new("RGB", (80, 100), "white")
    assert render.letterbox_camera(canvas, tmp_path, "edges.png", "CAM_FRONT", (0, 0, 80, 100)) == render.digest(tmp_path / "edges.png")
    assert canvas.getpixel((1, 62)) == (0, 0, 255)
    assert canvas.getpixel((78, 62)) == (0, 128, 0)
    assert canvas.getpixel((1, 30)) == (23, 33, 43)


@pytest.mark.parametrize("change", ["duplicate", "escape", "missing_camera", "wrong_time", "nonfinite", "not_stopped", "both_outputs"])
def test_invalid_or_ambiguous_raw_sources_are_rejected(trial, change):
    episode = trial / "episode.partial"
    if change == "not_stopped":
        (trial / "lifecycle/stopped.json").write_text('{"status":"FAIL"}')
    elif change == "both_outputs":
        (trial / "episode").mkdir()
    elif change == "nonfinite":
        rows = render.view.read_jsonl(episode / "states.jsonl")
        rows[0]["timestamp"] = float("nan")
        (episode / "states.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    else:
        rows = render.view.read_jsonl(episode / "camera_frames.jsonl")
        if change == "duplicate": rows.append(rows[0])
        if change == "escape": rows[0]["images"]["CAM_FRONT"] = "../outside.jpg"
        if change == "missing_camera": rows[0]["images"].pop("CAM_FRONT")
        if change == "wrong_time": rows[0]["timestamp"] = 999.
        (episode / "camera_frames.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    with pytest.raises(ValueError):
        render.load_trial(trial)


def test_changed_repeated_frame_bytes_are_rejected(trial):
    data = render.load_trial(trial)
    render.render_frame(data, 0, 5, 10)
    Image.new("RGB", (64, 32), "blue").save(data["episode"] / data["cameras"][0]["images"]["CAM_FRONT"])
    with pytest.raises(ValueError, match="changed during rendering"):
        render.render_frame(data, 0, 5, 10)


@pytest.mark.parametrize("stride,fps", [(0, 10), (True, 10), (1, 0), (1, float("nan"))])
def test_invalid_display_rates_do_not_create_output(trial, stride, fps):
    output = trial / "new_diagnostic"
    with pytest.raises(ValueError):
        render.render(trial, output, stride, fps)
    assert not output.exists()
