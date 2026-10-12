#!/usr/bin/env python3
"""HH_260906 - Render owned measured CARLA states around a centered ego, never synthesize a drive."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


WIDTH, HEIGHT = 1280, 720
VIEW_WIDTH = 900
EGO_X, EGO_Y = VIEW_WIDTH // 2, HEIGHT // 2
PIXELS_PER_METER = 24


def read_owned_trial(trial: Path):
    """HH_260906 - Reject a failed owner or a route different from the exact owned input."""
    owner = json.loads((trial / "owner_result.json").read_text())
    plan = json.loads((trial / "owner_plan.json").read_text())
    episode = trial / "episode"
    manifest = json.loads((episode / "manifest.json").read_text())
    route_path = episode / "route.json"
    route = json.loads(route_path.read_text())
    if (owner.get("exit_code") != 0 or not owner.get("source_bytes_unchanged_and_archived")
            or manifest.get("status") != "complete" or manifest.get("cleanup", {}).get("completed") is not True
            or hashlib.sha256(route_path.read_bytes()).hexdigest() != plan.get("route_sha256")
            or route.get("coordinate_reference") != "base_link"):
        raise ValueError("AN/AO visualization requires complete owned source-bound base_link evidence")
    states = [json.loads(line) for line in (episode / "states.jsonl").open()]
    if any(a["timestamp"] >= b["timestamp"] for a, b in zip(states, states[1:])):
        raise ValueError("state timestamps must increase")
    driving = [row for row in states if row["capture_phase"] == "driving"]
    if len(driving) < 2:
        raise ValueError("AN/AO visualization needs an actual measured drive")
    return route, driving


def fonts():
    """HH_260906 - Use installed bitmap-safe fonts, with a platform-neutral fallback."""
    path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    try:
        return (ImageFont.truetype(path, 23), ImageFont.truetype(path, 17))
    except OSError:
        return (ImageFont.load_default(), ImageFont.load_default())


def local_point(reference: dict, point: dict):
    """HH_260906 - Rotate real ROS-map XY into a forward-up, right-positive ego view."""
    dx = float(point["x"]) - float(reference["x"])
    dy = float(point["y"]) - float(reference["y"])
    yaw = float(reference["yaw"])
    forward = dx * math.cos(yaw) + dy * math.sin(yaw)
    right = dx * math.sin(yaw) - dy * math.cos(yaw)
    return round(EGO_X + right * PIXELS_PER_METER), round(EGO_Y - forward * PIXELS_PER_METER)


def render(route: dict, driving: list[dict], index: int, arm: str = "AN") -> Image.Image:
    """HH_260906 - Center only the measured base_link; blue is route and gold is observed history."""
    state = driving[index]
    image = Image.new("RGB", (WIDTH, HEIGHT), "#111827")
    draw = ImageDraw.Draw(image)
    large, small = fonts()
    draw.rectangle((0, 0, VIEW_WIDTH - 1, HEIGHT - 1), fill="#162534")
    for distance in (-10, -5, 0, 5, 10, 15, 20):
        y = EGO_Y - distance * PIXELS_PER_METER
        if 0 <= y < HEIGHT:
            draw.line((0, y, VIEW_WIDTH, y), fill="#304256", width=1)
            draw.text((12, y + 3), f"{distance:+d} m forward", fill="#8faabd", font=small)
    for distance in (-15, -10, -5, 0, 5, 10, 15):
        x = EGO_X + distance * PIXELS_PER_METER
        if 0 <= x < VIEW_WIDTH:
            draw.line((x, 0, x, HEIGHT), fill="#304256", width=1)
    reference_points = [local_point(state, point) for point in route["route"]]
    actual_points = [local_point(state, point) for point in driving[max(0, index - 140):index + 1]]
    if len(reference_points) > 1:
        draw.line(reference_points, fill="#5cc5ff", width=6, joint="curve")
    if len(actual_points) > 1:
        draw.line(actual_points, fill="#ffb84d", width=5, joint="curve")
    draw.polygon([(EGO_X - 13, EGO_Y + 20), (EGO_X + 13, EGO_Y + 20),
                  (EGO_X + 13, EGO_Y - 16), (EGO_X, EGO_Y - 25),
                  (EGO_X - 13, EGO_Y - 16)], fill="white", outline="#101820")
    draw.line((EGO_X, EGO_Y - 34, EGO_X, EGO_Y - 55), fill="white", width=4)
    draw.rectangle((VIEW_WIDTH, 0, WIDTH, HEIGHT), fill="#0d1522")
    # HH_260906 - Preserve the AN default while labeling AO's separate owned A/B truthfully.
    draw.text((VIEW_WIDTH + 22, 23), f"{arm} / OWNED CARLA", fill="white", font=large)
    draw.text((VIEW_WIDTH + 22, 58), "BasicAgent expert only", fill="#ffcc66", font=small)
    draw.text((VIEW_WIDTH + 22, 99), f"Native frame: {state['frame']}", fill="white", font=small)
    draw.text((VIEW_WIDTH + 22, 130), f"Elapsed drive: {state['timestamp'] - driving[0]['timestamp']:.1f} s",
              fill="white", font=small)
    speed = math.hypot(state["vx"], state["vy"]) * 3.6
    draw.text((VIEW_WIDTH + 22, 174), f"Speed: {speed:.2f} km/h", fill="white", font=small)
    draw.text((VIEW_WIDTH + 22, 205), f"Route progress: {state['route_progress_m']:.2f} m",
              fill="white", font=small)
    draw.text((VIEW_WIDTH + 22, 236), f"|CTE|: {abs(state['route_cte_m']):.3f} m",
              fill="white", font=small)
    draw.text((VIEW_WIDTH + 22, 282), f"Steer command: {state['next_control']['steer']:+.4f}",
              fill="white", font=small)
    guard_key = "ao_launch_steering" if arm == "AO" else "an_normal_steering"
    guard = state["goal_stop"].get(guard_key, {})
    draw.text((VIEW_WIDTH + 22, 313),
              f"{arm} normal zero: {'yes' if guard.get('applied_to_normal_command') else 'no'}",
              fill="#a3e6b6" if guard.get("applied_to_normal_command") else "white", font=small)
    draw.line((VIEW_WIDTH + 22, 371, WIDTH - 22, 371), fill="#53667b", width=1)
    draw.line((VIEW_WIDTH + 25, 403, VIEW_WIDTH + 75, 403), fill="#5cc5ff", width=6)
    draw.text((VIEW_WIDTH + 89, 391), "Catalog route", fill="white", font=small)
    draw.line((VIEW_WIDTH + 25, 440, VIEW_WIDTH + 75, 440), fill="#ffb84d", width=5)
    draw.text((VIEW_WIDTH + 89, 428), "Measured base_link", fill="white", font=small)
    draw.ellipse((VIEW_WIDTH + 39, 468, VIEW_WIDTH + 59, 488), fill="white")
    draw.text((VIEW_WIDTH + 89, 466), "Ego always centered", fill="white", font=small)
    draw.text((VIEW_WIDTH + 22, 536), f"Scale: {PIXELS_PER_METER} px / m", fill="#aab9c9", font=small)
    draw.text((VIEW_WIDTH + 22, 570), "Forward is up in each frame", fill="#aab9c9", font=small)
    draw.text((VIEW_WIDTH + 22, 626), "Measured-state replay", fill="#ffcc66", font=small)
    draw.text((VIEW_WIDTH + 22, 655), "Not Autoware GUI / live FPS", fill="#ffcc66", font=small)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", type=Path, required=True)
    parser.add_argument("--arm", choices=("AN", "AO"), default="AN")
    parser.add_argument("--png", type=Path, required=True)
    parser.add_argument("--gif", type=Path, required=True)
    args = parser.parse_args()
    route, driving = read_owned_trial(args.trial)
    turn_index = min(range(len(driving)), key=lambda i: abs(driving[i]["route_progress_m"] - 50.0))
    keyframe = render(route, driving, turn_index, args.arm)
    args.png.parent.mkdir(parents=True, exist_ok=True)
    keyframe.save(args.png)
    # HH_260906 - Sample real native states by elapsed index; playback timing is illustrative, not camera FPS.
    indexes = sorted(set(round(i * (len(driving) - 1) / 55) for i in range(56)))
    frames = [render(route, driving, index, args.arm) for index in indexes]
    frames[0].save(args.gif, save_all=True, append_images=frames[1:], duration=180, loop=0,
                   optimize=True)


if __name__ == "__main__":
    main()
