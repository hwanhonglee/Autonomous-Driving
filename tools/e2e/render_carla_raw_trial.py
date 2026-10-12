#!/usr/bin/env python3
"""HH_260906 - Render real raw expert frames without converting failed trials into approved datasets."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
from pathlib import Path
import shutil
import subprocess
import tempfile

from PIL import Image, ImageDraw

from scripts.e2e import render_carla_vad_expert as view


CAMERAS = tuple(name for row in view.DISPLAY_CAMERA_GRID for name in row)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_trial(trial):
    """HH_260906 - Require a stopped owner and preserve the original complete/partial distinction."""
    trial = Path(trial).resolve()
    candidates = [trial / name for name in ("episode", "episode.partial") if (trial / name).is_dir()]
    if len(candidates) != 1:
        raise ValueError("trial must contain exactly one original episode or episode.partial")
    owner = json.loads((trial / "owner_result.json").read_text())
    stopped = json.loads((trial / "lifecycle/stopped.json").read_text())
    if stopped.get("status") != "PASS" or owner.get("source_bytes_unchanged_and_archived") is not True:
        raise ValueError("render only stopped trials with unchanged archived capture sources")
    episode = candidates[0]
    states = view.read_jsonl(episode / "states.jsonl")
    cameras = view.read_jsonl(episode / "camera_frames.jsonl")
    manifest = json.loads((episode / "manifest.json").read_text())
    route = json.loads((episode / "route.json").read_text())
    frames = {row["frame"]: index for index, row in enumerate(states)}
    if len(frames) != len(states) or len(states) < 2 or not cameras:
        raise ValueError("raw state frames must be nonempty and unique")
    if any(not all(math.isfinite(float(row[k])) for k in ("timestamp", "x", "y", "z", "yaw", "vx", "vy")) for row in states):
        raise ValueError("raw measured times and motion must be finite")
    for first, second in zip(states, states[1:]):
        if second["timestamp"] <= first["timestamp"]:
            raise ValueError("raw timestamps must increase")
    seen = set()
    for camera in cameras:
        frame = camera["frame"]
        if frame not in frames or frame in seen:
            raise ValueError("camera frame is missing from states or duplicated")
        seen.add(frame)
        if set(camera["images"]) != set(CAMERAS):
            raise ValueError("every raw anchor requires the exact six-camera rig")
        if abs(camera["timestamp"] - states[frames[frame]]["timestamp"]) > 1e-6:
            raise ValueError("camera/state time mismatch")
        for name, raw in camera["images"].items():
            relative = Path(raw)
            path = episode / relative
            if relative.is_absolute() or ".." in relative.parts or not path.resolve().is_relative_to(episode):
                raise ValueError("image path escapes the original episode")
            if not path.is_file() or abs(camera["source_timestamps"][name] - camera["timestamp"]) > 1e-6:
                raise ValueError("missing or asynchronous raw camera")
    return {"root": trial, "episode": episode, "states": states, "cameras": cameras,
            "frames": frames, "manifest": manifest, "route": route, "owner": owner,
            "source_metadata_sha256": {name: digest(episode / name) for name in ("manifest.json", "route.json", "states.jsonl", "camera_frames.jsonl")},
            "displayed_image_sha256": {}}


def raw_sample(data, camera):
    """HH_260906 - Use only measured future positions; never interpolate a missing terminal tail or predict motion."""
    index = data["frames"][camera["frame"]]
    row = data["states"][index]
    future = [r for r in data["states"][index + 2:index + 129:2]
              if r["timestamp"] - row["timestamp"] <= 6.40001]
    cosine, sine = math.cos(row["yaw"]), math.sin(row["yaw"])
    positions = []
    for item in future:
        dx, dy = item["x"] - row["x"], item["y"] - row["y"]
        positions.append([cosine * dx + sine * dy, -sine * dx + cosine * dy])
    return {"frame": row["frame"], "timestamp_ns": round(row["timestamp"] * 1e9),
            "pose_map_xyz_yaw": [row[k] for k in ("x", "y", "z", "yaw")],
            "route_progress_m": row["route_progress_m"], "route_cte_m": row["route_cte_m"],
            "vad_command_0based": row["command"], "cameras": {k: {"path": v} for k, v in camera["images"].items()},
            "future_expert": {"positions_xy": positions, **({"horizons_s": [r["timestamp"] - row["timestamp"] for r in future]} if future else {})},
            "visualization_only": True}


def letterbox_camera(canvas, episode, relative, name, box):
    """HH_260906 - Show the full recorded field of view, with padding instead of a center crop."""
    left, top, right, bottom = box
    ImageDraw.Draw(canvas).rectangle(box, fill="#17212b")
    payload = (episode / relative).read_bytes()
    with Image.open(io.BytesIO(payload)) as source:
        camera = source.convert("RGB")
        scale = min((right - left) / camera.width, (bottom - top - 26) / camera.height)
        size = (max(1, round(camera.width * scale)), max(1, round(camera.height * scale)))
        resampling = getattr(Image, "Resampling", Image).LANCZOS
        camera = camera.resize(size, resampling)
    canvas.paste(camera, (left + (right - left - size[0]) // 2, top + 26 + (bottom - top - 26 - size[1]) // 2))
    draw = ImageDraw.Draw(canvas)
    draw.text((left + 8, top + 4), name.removeprefix("CAM_"), fill="white", font=view._font(15, True))
    return hashlib.sha256(payload).hexdigest()


def render_frame(data, camera_index, stride, playback_fps):
    """HH_260906 - Label measured states and observer playback separately from autonomous-model performance."""
    camera = data["cameras"][camera_index]
    row = data["states"][data["frames"][camera["frame"]]]
    sample = raw_sample(data, camera)
    canvas = Image.new("RGB", (1600, 900), "#f3f5f7")
    draw = ImageDraw.Draw(canvas)
    quality = data["manifest"].get("result", {}).get("goal_stop_quality", {}).get("status", "UNASSESSED")
    label = "RECORDED FAILURE" if data["owner"]["exit_code"] != 0 else "CAPTURE COMPLETE / NOT ADMITTED"
    draw.text((20, 14), f"CARLA expert diagnostic | {data['route'].get('town')} | {label}", font=view._font(25, True), fill="#182026")
    elapsed = row["timestamp"] - data["states"][0]["timestamp"]
    draw.text((20, 49), f"Sim t={elapsed:.2f}s | frame {row['frame']} | {row['capture_phase']} | scalar QA {quality}", font=view._font(19), fill="#59636d")
    left_width, gap, content_top, content_bottom = 970, 10, 91, 733
    cw, ch = (left_width - 20 - 2 * gap) // 3, (content_bottom - content_top - gap) // 2
    for r, names in enumerate(view.DISPLAY_CAMERA_GRID):
        for c, name in enumerate(names):
            left, top = 20 + c * (cw + gap), content_top + r * (ch + gap)
            relative = camera["images"][name]
            source_sha = letterbox_camera(canvas, data["episode"], relative, name, (left, top, left + cw, top + ch))
            if data["displayed_image_sha256"].setdefault(relative, source_sha) != source_sha:
                raise ValueError("displayed source image changed during rendering")
    index = data["frames"][camera["frame"]]
    history = data["states"][:index + 1]
    recent = [r for r in history if r["timestamp"] >= row["timestamp"] - 4.0]
    view._draw_map_panel(canvas, (982, content_top, 1580, content_bottom), view._route_xy(data["route"]),
                         [(r["x"], r["y"]) for r in history], [(r["x"], r["y"]) for r in recent],
                         view._sample_future_world(sample), sample)
    draw = ImageDraw.Draw(canvas)
    control, goal = row["current_control"], row.get("goal_stop", {})
    acceleration = 0.0 if index == 0 else (
        math.hypot(row["vx"], row["vy"]) - math.hypot(data["states"][index - 1]["vx"], data["states"][index - 1]["vy"])) / (
            row["timestamp"] - data["states"][index - 1]["timestamp"])
    lines = [
        f"Measured {math.hypot(row['vx'], row['vy']) * 3.6:.2f} km/h | dv/dt {acceleration:+.3f} m/s2 | CTE {row['route_cte_m']:.3f} m | progress {row['route_progress_m']:.2f} m",
        f"Reported throttle {control['throttle']:.4f} | brake {control['brake']:.4f} | steer {control['steer']:+.4f} | gear {control['gear']} | state {goal.get('pilot_state', goal.get('phase', 'n/a'))}",
        f"Raw six-camera recording; full FOV. EGO CENTERED. Future trace = observed data, NOT a model prediction.",
        f"Display stride {stride} camera anchors; GIF {playback_fps:g} fps (accelerated preview). Native physics 20 Hz / cameras 10 Hz. NOT Autoware or learned control.",
    ]
    for n, line in enumerate(lines):
        draw.text((20, 755 + n * 30), line, fill="#182026" if n < 2 else "#59636d", font=view._font(18 if n < 2 else 15, n == 0))
    return canvas


def render(trial, output, stride=5, playback_fps=10.0):
    """HH_260906 - Write a new diagnostic directory and hash every displayed source frame and original metadata."""
    if type(stride) is not int or stride < 1 or not math.isfinite(playback_fps) or playback_fps <= 0:
        raise ValueError("stride and playback rate must be positive")
    data = load_trial(trial)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    indices = list(range(0, len(data["cameras"]), stride))
    if indices[-1] != len(data["cameras"]) - 1:
        indices.append(len(data["cameras"]) - 1)
    states = data["states"]
    selected = {"01_start": 0, "05_final_observation": len(data["cameras"]) - 1}
    for name, predicate in (
        ("02_measured_cruise", lambda r: math.hypot(r["vx"], r["vy"]) >= 7.8),
        ("03_coast_entry", lambda r: r.get("goal_stop", {}).get("pilot_state") == "coast_low"),
        ("04_goal_dwell", lambda r: r.get("goal_stop", {}).get("complete") is True)):
        found = [i for i, c in enumerate(data["cameras"]) if predicate(states[data["frames"][c["frame"]]])]
        if found:
            selected[name] = found[0]
    worst_index = min(range(1, len(states)), key=lambda i: (
        math.hypot(states[i]["vx"], states[i]["vy"]) - math.hypot(states[i-1]["vx"], states[i-1]["vy"])) / (states[i]["timestamp"] - states[i-1]["timestamp"]))
    selected["06_maximum_observed_deceleration"] = min(range(len(data["cameras"])), key=lambda i: abs(data["cameras"][i]["timestamp"] - states[worst_index]["timestamp"]))
    for name, index in selected.items():
        render_frame(data, index, stride, playback_fps).save(output / f"{name}.png")
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("existing ffmpeg required; no installation is performed")
    with tempfile.TemporaryDirectory(prefix="carla_raw_trial_preview_") as temporary:
        frame_dir = Path(temporary)
        for number, index in enumerate(indices):
            render_frame(data, index, stride, playback_fps).save(frame_dir / f"frame_{number:05d}.png", compress_level=1)
        subprocess.run([ffmpeg, "-hide_banner", "-loglevel", "error", "-n", "-framerate", str(playback_fps),
                        "-i", str(frame_dir / "frame_%05d.png"), "-filter_complex",
                        "[0:v]split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer",
                        "-loop", "0", str(output / "whole_recording_accelerated.gif")], check=True, timeout=180)
    sources, images = data["source_metadata_sha256"], data["displayed_image_sha256"]
    # HH_260906 - Reject changing source bytes instead of attaching post-edit hashes to previously rendered pixels.
    if any(digest(data["episode"] / name) != expected for name, expected in {**sources, **images}.items()):
        raise ValueError("original metadata or displayed image changed during rendering")
    report = {"schema": "carla_expert.raw_trial_visual_diagnostic.v1", "source_episode_name": data["episode"].name,
              "source_metadata_sha256": sources, "displayed_image_sha256": images,
              "source_renderer_sha256": digest(Path(__file__)), "source_layout_sha256": digest(Path(view.__file__)),
              "original_owner_exit_code": data["owner"]["exit_code"], "training_data_approved": False,
              "learned_model_control": False, "live_autoware_screenshot": False,
              "camera_anchor_count": len(data["cameras"]), "rendered_indices": indices,
              "png_indices": selected, "camera_stride": stride, "playback_fps": playback_fps,
              "displayed_simulation_span_seconds": data["cameras"][-1]["timestamp"] - data["cameras"][0]["timestamp"],
              "display_duration_seconds": len(indices) / playback_fps,
              "terminal_future_policy": "Only available measured states; never padded, interpolated or advertised as complete training labels.",
              "notice": "Diagnostic frame selection includes the entire recording at a stated stride; rendering does not qualify data or model performance."}
    (output / "visual_provenance.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--camera-stride", type=int, default=5)
    parser.add_argument("--playback-fps", type=float, default=10.0)
    args = parser.parse_args()
    report = render(args.trial, args.output, args.camera_stride, args.playback_fps)
    print(f"RENDERED {len(report['rendered_indices'])} diagnostic frames; original exit={report['original_owner_exit_code']}; training_data_approved=false")


if __name__ == "__main__":
    main()
