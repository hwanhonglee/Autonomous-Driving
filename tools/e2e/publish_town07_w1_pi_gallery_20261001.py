"""HH_261001 - Publish real PI comparison cameras and ego-centered measured body and plans."""

from __future__ import annotations

import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT / "storage/experiments/validation/2026-10-01/town07_w1_pi_controller_v2"
OUTPUT = ROOT / "docs/assets/validation/2026-10-01/town07_w1_pi_control_results_v1"
ARMS = ("A1_original", "B1_pi_only")
CAMERAS = ("CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT")
BG, PANEL, WHITE, MUTED = "#101722", "#192333", "#eaf0f7", "#a8b9cc"
BLUE, GREEN, PURPLE, AMBER = "#769edb", "#42d8a7", "#d3a1ed", "#ffd27e"


def require(value, reason):
    if not value:
        raise ValueError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def font(size, bold=False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/" + name, size)


def centered(points, state):
    # HH_261001 - Transform archived ROS map coordinates to the current measured rear-axle base frame.
    delta = np.asarray(points, dtype=float).reshape(-1, 2) - [state["x"], state["y"]]
    c, s = math.cos(state["yaw"]), math.sin(state["yaw"])
    return np.column_stack((delta[:, 0]*c + delta[:, 1]*s, -delta[:, 0]*s + delta[:, 1]*c))


def acknowledged_plan(commands, by_frame, state):
    # HH_261001 - A command accepted at frame N cannot be shown as an already observed effect at frame N.
    eligible = [row for row in commands if row.get("application", {}).get("learned_control_applied") is True
                and row["command_frame"] < state["frame"]]
    if not eligible:
        return None, []
    row = eligible[-1]
    origin = by_frame[row["inference_frame"]]
    c, s = math.cos(origin["yaw"]), math.sin(origin["yaw"])
    points = [(origin["x"] + c*x - s*y, origin["y"] + s*x + c*y)
              for x, y in row["selected"]["xy_base_m"]]
    return row, points


def map_panel(state, geometry, route, traveled, plan):
    panel = Image.new("RGB", (410, 595), PANEL)
    draw = ImageDraw.Draw(panel)
    draw.text((15, 12), "Measured ego at center", fill=WHITE, font=font(21, True))
    draw.text((15, 43), "Fixed +/-20 m | forward up", fill=MUTED, font=font(16))
    box = Image.new("RGB", (360, 360), "#121c29")
    pen = ImageDraw.Draw(box)

    def pixels(values):
        return [(180-9*left, 180-9*forward) for forward, left in centered(values, state)]

    for value in range(0, 361, 45):
        pen.line([(value, 0), (value, 360)], fill="#293749")
        pen.line([(0, value), (360, value)], fill="#293749")
    for values, color in ((route, BLUE), (traveled, GREEN), (plan, PURPLE)):
        if len(values) > 1:
            pen.line(pixels(values), fill=color, width=3)
    body = [(x, -y) for x, y in geometry["body_polygon_carla_xy"]]
    pen.polygon(pixels(body), fill="#9f7431", outline=AMBER)
    pen.ellipse((176, 176, 184, 184), fill=WHITE)
    pen.line([(180, 180), (180, 150)], fill=WHITE, width=2)
    panel.paste(box, (25, 76))
    for index, (text, color) in enumerate((("Blue: pinned navigation route", BLUE),
            ("Green: observed trajectory so far", GREEN), ("Purple: last ACKed model plan", PURPLE),
            ("Amber: measured vehicle footprint", AMBER), ("White: rear-axle base_link origin", WHITE))):
        draw.text((20, 455+25*index), text, fill=color, font=font(15))
    return panel


def load_arm(arm):
    run = RUN / arm
    episode = run / "episode.partial"
    states, cameras, commands = [rows(episode / name) for name in
        ("states.jsonl", "camera_frames.jsonl", "learned_control.jsonl")]
    require(len(states) == 400 and len(cameras) == 200, "exact retained twenty-second cohort required")
    by_frame = {row["frame"]: row for row in states}
    body_path = RUN / "audit_v1" / ("A_body" if arm == ARMS[0] else "B_body")
    geometry = {row["frame"]: row for row in rows(body_path / "all_native_body_sweeps.jsonl.gz")}
    require(set(geometry) == set(by_frame), "audited measured body frames differ")
    paths = [episode / name for name in ("states.jsonl", "camera_frames.jsonl", "learned_control.jsonl",
        "manifest.json", "route.json", "control_receipts.jsonl")]
    paths += [run / "owner_result.json", body_path / "summary.json", body_path / "all_native_body_sweeps.jsonl.gz"]
    for camera in cameras:
        require(camera["frame"] in by_frame and camera["timestamp"] == by_frame[camera["frame"]]["timestamp"]
                and set(camera["images"]) == set(CAMERAS), "exact camera/native snapshot binding required")
        for name in CAMERAS:
            relative = Path(camera["images"][name])
            require(not relative.is_absolute() and ".." not in relative.parts, "safe actual image path required")
            path = episode / relative
            with Image.open(path) as image:
                require(image.size == (640, 360) and image.format == "JPEG", "original image ABI differs")
                image.load()
            paths.append(path)
    return dict(arm=arm, run=run, episode=episode, states=states, cameras=cameras, commands=commands,
        by_frame=by_frame, geometry=geometry, route=json.loads((episode / "route.json").read_text())["route"],
        input_sha256={str(path.relative_to(ROOT)): sha(path) for path in paths})


def render(bundle, camera_index):
    camera = bundle["cameras"][camera_index]
    state = bundle["by_frame"][camera["frame"]]
    canvas = Image.new("RGB", (1700, 820), BG)
    draw = ImageDraw.Draw(canvas)
    draw.text((24, 17), "Real W1 learned control | " + bundle["arm"], fill=WHITE, font=font(29, True))
    draw.text((24, 60), "Recorded CARLA cameras and measured trajectory | not Autoware UI | not full-route approval",
              fill=MUTED, font=font(20))
    elapsed = state["timestamp"] - bundle["states"][0]["timestamp"]
    progress = state["route_progress_m"] - bundle["states"][0]["route_progress_m"]
    draw.text((24, 96), f"t={elapsed:.2f}s | frame={state['frame']} | measured vx={state['vx']:.3f}m/s"
              f" | progress={progress:.2f}m | CTE={state['route_cte_m']:.3f}m", fill=WHITE, font=font(21))
    for index, name in enumerate(CAMERAS):
        x, y = 24 + index % 3 * 416, 150 + index // 3 * 277
        draw.text((x, y), name, fill=WHITE, font=font(18, True))
        with Image.open(bundle["episode"] / camera["images"][name]) as image:
            # HH_261001 - Reuse the installed older Pillow API without installing or upgrading packages.
            resampling = getattr(Image, "Resampling", Image)
            image = image.convert("RGB").resize((400, 225), resampling.LANCZOS)
            canvas.paste(image, (x, y+30))
    decision, plan = acknowledged_plan(bundle["commands"], bundle["by_frame"], state)
    route = [(row["x"], row["y"]) for row in bundle["route"]]
    traveled = [(row["x"], row["y"]) for row in bundle["states"] if row["frame"] <= state["frame"]]
    panel = map_panel(state, bundle["geometry"][state["frame"]], route, traveled, plan)
    canvas.paste(panel, (1270, 145))
    control = state["current_control"]
    label = "previous expert/startup control" if decision is None else "prior acknowledged learned command"
    draw.text((24, 728), f"Observed control: throttle={control['throttle']:.3f} / brake={control['brake']:.3f}"
              f" / steer={control['steer']:.3f} | {label}", fill=WHITE, font=font(20))
    draw.text((24, 766), "Playback samples actual 10Hz saved cameras at 2Hz; startup and original termination remain included.",
              fill=MUTED, font=font(18))
    return canvas


def main():
    require(not OUTPUT.exists(), "create-only publication directory required")
    bundles = [load_arm(arm) for arm in ARMS]
    OUTPUT.mkdir(parents=True)
    artifacts = []
    for bundle in bundles:
        name = bundle["arm"]
        frames = [render(bundle, index) for index in range(0, 200, 5)]
        image_path, gif_path = OUTPUT / (name+"_ego_centered.png"), OUTPUT / (name+"_actual_cameras_2hz.gif")
        frames[28].save(image_path)
        frames[0].save(gif_path, save_all=True, append_images=frames[1:], duration=500, loop=0, optimize=False)
        require(all(sha(ROOT / path) == expected for path, expected in bundle["input_sha256"].items()),
                "original input changed during publication")
        artifacts.append(dict(arm=name, actual_native_states=400, original_camera_bundles=200,
            checked_original_jpegs=1200, displayed_gif_frames=40, playback_hz=2,
            files={path.name: sha(path) for path in (image_path, gif_path)}, input_sha256=bundle["input_sha256"]))
    with (OUTPUT / "visual_provenance.json").open("x") as stream:
        json.dump(dict(comment="HH_261001 - Publish actual images without changing raw failures or control results.",
            source_sha256=sha(Path(__file__)), arms=artifacts, autoware_ui_captured=False,
            synthetic_driving_image_generated=False, full_route_approval=False), stream, indent=2)
    print(json.dumps(dict(status="REAL_CAMERA_AND_CENTERED_BODY_MEDIA_CREATED", output=str(OUTPUT))))


if __name__ == "__main__":
    main()
