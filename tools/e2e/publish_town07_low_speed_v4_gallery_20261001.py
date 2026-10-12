#!/usr/bin/env python3
"""HH_261001 - Publish timestamp-bound visual evidence from the completed V4 capture."""
from __future__ import annotations

import gzip
import hashlib
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
EPISODE = ROOT / "storage/experiments/validation/2026-10-01/town07_low_speed_moving_train_v4/episode"
QA = ROOT / "docs/assets/validation/2026-10-01/town07_low_speed_moving_v4_qa_v1"
OUTPUT = ROOT / "docs/assets/validation/2026-10-01/town07_low_speed_moving_v4_results"
TITLE = "BasicAgent expert capture — not learned-model driving, not Autoware UI"
CAMERAS = (
    ("CAM_FRONT_LEFT", "FRONT LEFT"), ("CAM_FRONT", "FRONT"), ("CAM_FRONT_RIGHT", "FRONT RIGHT"),
    ("CAM_BACK_LEFT", "BACK LEFT"), ("CAM_BACK", "BACK"), ("CAM_BACK_RIGHT", "BACK RIGHT"),
)
BG, PANEL, WHITE, MUTED = "#101722", "#192333", "#eaf0f7", "#a8b9cc"
GREEN, BLUE, AMBER = "#42d8a7", "#769edb", "#ffd27e"
WIDTH, HEIGHT, TILE_W, TILE_H = 1700, 820, 410, 231
VIEW_HALF_M = 20.0
STEP = 5
FINAL_HOLD_MS = 100


def require(condition, message):
    # HH_261001 - Refuse publication when source identity or fixed membership differs.
    if not condition:
        raise RuntimeError(message)


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def read_rows(path):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def font(size, bold=False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}", size)


def local_xy(points, state):
    # HH_261001 - Transform stored ROS-map XY to the current rear-axle ego frame.
    delta = np.asarray(points, dtype=float) - [state["x"], state["y"]]
    c, s = math.cos(state["yaw"]), math.sin(state["yaw"])
    return np.column_stack((delta[:, 0] * c + delta[:, 1] * s,
                            -delta[:, 0] * s + delta[:, 1] * c))


def map_panel(state, native_row, route_xy, trajectory_xy):
    # HH_261001 - Fixed symmetric bounds keep the ego reference central in every frame.
    panel = Image.new("RGB", (406, 588), PANEL)
    draw = ImageDraw.Draw(panel)
    draw.text((18, 14), "Ego-centered recorded map", fill=WHITE, font=font(20, True))
    draw.text((18, 44), "Fixed ±20 m • forward up • left left", fill=MUTED, font=font(15))
    box = Image.new("RGB", (356, 356), "#121c29")
    map_draw = ImageDraw.Draw(box)
    center, scale = 178.0, 356.0 / (2.0 * VIEW_HALF_M)

    def pixels(points):
        local = local_xy(points, state)
        return [(center - left * scale, center - forward * scale) for forward, left in local]

    for offset in range(-20, 21, 5):
        pos = center + offset * scale
        map_draw.line([(pos, 0), (pos, 356)], fill="#293749", width=1)
        map_draw.line([(0, pos), (356, pos)], fill="#293749", width=1)
    route_pixels = pixels(route_xy)
    for index, (first, last) in enumerate(zip(route_pixels, route_pixels[1:])):
        if index % 2 == 0:
            map_draw.line([first, last], fill=BLUE, width=5)
    if len(trajectory_xy) > 1:
        map_draw.line(pixels(trajectory_xy), fill=GREEN, width=3)
    # HH_261001 - Reuse the audited CARLA bounding-box projection, with y flipped to ROS map.
    body = native_row["geometry"]["body_polygon_carla_xy"]
    body_pixels = pixels([(point[0], -point[1]) for point in body])
    map_draw.polygon(body_pixels, fill="#9f7431", outline=AMBER, width=2)
    map_draw.line([(center, center), (center, center - 34)], fill=WHITE, width=2)
    map_draw.polygon([(center, center - 40), (center - 5, center - 31),
                      (center + 5, center - 31)], fill=WHITE)
    map_draw.ellipse((center - 4, center - 4, center + 4, center + 4), fill=WHITE)
    panel.paste(box, (25, 76))
    draw.text((25, 443), "Dashed blue: recorded route", fill=BLUE, font=font(16))
    draw.text((25, 470), "Green: actual native trajectory so far", fill=GREEN, font=font(15))
    draw.text((25, 497), "Amber: audited vehicle body footprint", fill=AMBER, font=font(15))
    draw.text((25, 524), "White dot: rear-axle base_link at (0, 0)", fill=WHITE, font=font(15))
    draw.text((25, 551), "Body rectangle: 4.514 × 2.007 m", fill=MUTED, font=font(15))
    return panel


def render(frame, state, native_row, route_xy, past_xy, ordinal, count, start_ns):
    # HH_261001 - Every camera tile is an actual JPEG at this exact saved camera anchor.
    image = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(image)
    draw.text((24, 17), TITLE, fill=WHITE, font=font(29, True))
    draw.text((24, 62), "Town07 / V4 / six synchronized stored cameras / selected 40–120 m cruise window",
              fill=MUTED, font=font(21))
    timestamp_ns = int(round(frame["timestamp"] * 1e9))
    speed = math.hypot(state["vx"], state["vy"])
    progress = frame["moving_task_measured_route_progress_m"]
    draw.text((24, 103), f"Frame {frame['frame']}  |  selected t={(timestamp_ns-start_ns)/1e9:5.2f} s  |  "
              f"progress {progress:7.3f} m  |  planar speed {speed:.4f} m/s  |  visual {ordinal}/{count}",
              fill=WHITE, font=font(22))
    for index, (camera, label) in enumerate(CAMERAS):
        x = 24 + (index % 3) * 420
        y = 158 + (index // 3) * 282
        draw.rounded_rectangle((x, y, x + TILE_W, y + 270), radius=7, fill=PANEL)
        draw.text((x + 12, y + 8), label, fill=WHITE, font=font(17, True))
        path = (EPISODE / frame["images"][camera]).resolve(strict=True)
        require(path.is_relative_to(EPISODE.resolve()), "camera image leaves capture directory")
        with Image.open(path) as source:
            require(source.size == (640, 360), "unexpected native camera image dimensions")
            # HH_261001 - Support the workspace Pillow version without altering source pixels on disk.
            resampling = getattr(Image, "Resampling", Image)
            tile = source.convert("RGB").resize((TILE_W, TILE_H), resampling.LANCZOS)
        image.paste(tile, (x, y + 34))
    image.paste(map_panel(state, native_row, route_xy, past_xy), (1280, 150))
    draw.text((24, 738), "QA: all 327 / 327 fixed windows clear • source / ACK / pixels / body / nine future bounds",
              fill=GREEN, font=font(20, True))
    draw.text((24, 771), "Visual downsampling ≈2 Hz from 10 Hz capture • timestamp-paced playback • training_data_approved=false",
              fill=AMBER, font=font(18))
    return image


def plot_summary(selected, states, native_rows, output_path):
    # HH_261001 - Summary curves retain every selected anchor and native tick in the displayed interval.
    first_ns, last_ns = selected[0]["timestamp_ns"], selected[-1]["timestamp_ns"]
    selected_states = [states[row["frame"]] for row in selected]
    interval = [row for row in native_rows if first_ns <= row["timestamp_ns"] <= last_ns]
    native_times = [(row["timestamp_ns"] - first_ns) / 1e9 for row in interval]
    native_speeds = [row["speed_mps"] for row in interval]
    times = [(row["timestamp_ns"] - first_ns) / 1e9 for row in selected]
    progress = [row["progress_m"] for row in selected]
    speeds = [math.hypot(state["vx"], state["vy"]) for state in selected_states]
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11})
    fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    fig.suptitle(TITLE + "\nAll 327 selected anchors; measured cruise evidence", fontsize=16)
    axes[0].plot(native_times, native_speeds, color="#188466", linewidth=1,
                 label=f"All {len(interval)} native ticks in selected time interval (20 Hz)")
    axes[0].scatter(times, speeds, s=7, color="#244f88", alpha=.6,
                    label="All 327 saved camera anchors (10 Hz)")
    axes[0].axhline(2.5, color="#ae7923", linestyle="--", linewidth=1, label="Declared target cap: 2.5 m/s")
    axes[0].set_ylabel("Measured planar speed (m/s)")
    axes[0].set_ylim(min(native_speeds) - .015, 2.515)
    axes[0].legend(loc="lower right", fontsize=9)
    axes[1].plot(times, progress, color="#244f88", linewidth=2)
    axes[1].axhline(40, color="#778899", linestyle="--", linewidth=.8)
    axes[1].axhline(120, color="#778899", linestyle="--", linewidth=.8)
    axes[1].set_ylabel("Independent route progress (m)")
    axes[1].set_xlabel("Time since first selected camera anchor (s)")
    for axis in axes:
        axis.grid(alpha=.2)
        axis.set_xlim(0, times[-1])
    fig.text(.5, .025, "Native speed = hypot(base_link vx, vy); progress = saved independent nearest-segment arc. "
             "Necessary checks clear; training approval false.", ha="center", fontsize=10)
    fig.tight_layout(rect=(0, .055, 1, .91))
    fig.savefig(output_path, dpi=140)
    plt.close(fig)
    return interval, speeds


def main():
    qa = read_json(QA / "summary.json")
    audit = qa["numerical_audit"]
    require(qa["status"] == "NECESSARY_CHECKS_CLEAR_PENDING_AUTHORITY_REVIEW", "unexpected QA status")
    require(qa["training_data_approved"] is False, "publication must retain unapproved data status")
    require(qa["fixed_selected_window_count"] == qa["selected_window_count"] == 327,
            "the fixed 327-window denominator changed")
    require(qa["v4_source_evidence"]["status"] == "PASS", "declared V4 source closure must pass")
    require(all(audit["flags"].values()), "necessary QA flags must all be clear")
    sources = [EPISODE / name for name in ("manifest.json", "camera_frames.jsonl", "states.jsonl", "route.json")]
    for path in sources:
        require(sha256(path) == audit["input_sha256"][str(path)], f"QA source hash differs: {path}")
    manifest = read_json(EPISODE / "manifest.json")
    route = read_json(EPISODE / "route.json")
    camera_rows = read_rows(EPISODE / "camera_frames.jsonl")
    state_rows = read_rows(EPISODE / "states.jsonl")
    native_rows = read_rows(QA / "all_native.jsonl.gz")
    windows = read_rows(QA / "all_selected_windows.jsonl.gz")
    selected = audit["membership"]["selected"]
    require(len(windows) == len(selected) == 327 and all(row["necessary_window_checks_clear"] for row in windows),
            "every fixed selected window must be retained and clear")
    require([row["frame"] for row in windows] == [row["frame"] for row in selected], "window membership differs")
    require([row["frame"] for row in camera_rows if row.get("moving_task_selected")] ==
            [row["frame"] for row in selected], "saved selected camera membership differs")
    states = {row["frame"]: row for row in state_rows}
    natives = {row["frame"]: row for row in native_rows}
    require(len(states) == len(state_rows) and len(natives) == len(native_rows), "duplicate native frame")
    for row in selected:
        camera = camera_rows[row["camera_index"]]
        require(camera["frame"] == row["frame"] and int(round(camera["timestamp"] * 1e9)) == row["timestamp_ns"],
                "selected camera identity differs")
        require(camera["moving_task_measured_route_progress_m"] == row["progress_m"], "selected progress differs")
        require(natives[row["frame"]]["timestamp_ns"] == row["timestamp_ns"], "native timestamp binding differs")
    filenames = ("six_camera_ego_centered.png", "six_camera_ego_centered_2hz.gif", "speed_progress_summary.png", "visual_metadata.json")
    require(not OUTPUT.is_symlink(), "output must be a real directory")
    for name in filenames:
        require(not (OUTPUT / name).exists(), f"new artifact required: {name}")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    indices = list(range(0, len(selected), STEP))
    if indices[-1] != len(selected) - 1:
        indices.append(len(selected) - 1)
    sampled = [selected[index] for index in indices]
    durations = [int(round((second["timestamp_ns"] - first["timestamp_ns"]) / 1e7)) * 10
                 for first, second in zip(sampled, sampled[1:])] + [FINAL_HOLD_MS]
    require(all(value > 0 for value in durations), "GIF duration must be positive")
    route_xy = [(row["x"], row["y"]) for row in route["route"]]
    frames = []
    sample_metadata = []
    image_sources = {}
    first_ns = selected[0]["timestamp_ns"]
    snapshot_index = min(range(len(sampled)), key=lambda index: abs(sampled[index]["progress_m"] - 80.0))
    for ordinal, anchor in enumerate(sampled):
        camera = camera_rows[anchor["camera_index"]]
        state = states[anchor["frame"]]
        past_xy = [(row["x"], row["y"]) for row in state_rows if row["frame"] <= anchor["frame"]]
        frame_image = render(camera, state, natives[anchor["frame"]], route_xy, past_xy,
                             ordinal + 1, len(sampled), first_ns)
        if ordinal == snapshot_index:
            frame_image.save(OUTPUT / filenames[0])
        frames.append(frame_image.quantize(colors=192))
        for camera_name, relative in camera["images"].items():
            path = EPISODE / relative
            image_sources[str(path.relative_to(ROOT))] = sha256(path)
        sample_metadata.append({"selected_index": indices[ordinal], "camera_index": anchor["camera_index"],
                                "frame": anchor["frame"], "timestamp_ns": anchor["timestamp_ns"],
                                "elapsed_selected_sec": (anchor["timestamp_ns"] - first_ns) / 1e9,
                                "progress_m": anchor["progress_m"], "speed_mps": math.hypot(state["vx"], state["vy"]),
                                "gif_display_duration_ms": durations[ordinal], "images": camera["images"]})
    frames[0].save(OUTPUT / filenames[1], save_all=True, append_images=frames[1:],
                   duration=durations, loop=0, disposal=2, optimize=False)
    interval, anchor_speeds = plot_summary(selected, states, native_rows, OUTPUT / filenames[2])
    sources.extend(QA / name for name in ("summary.json", "all_native.jsonl.gz", "all_selected_windows.jsonl.gz"))
    native_speeds = [row["speed_mps"] for row in interval]
    geometry = manifest["capture_contract"]["actor_body_geometry"]
    # HH_261001 - Record all playback timing, source identities, and limits as reviewable evidence.
    metadata = {
        "schema": "carla.town07_low_speed_v4_actual_visual_evidence.20261001.v1",
        "title": TITLE, "producer": str(Path(__file__).relative_to(ROOT)),
        "producer_sha256": sha256(Path(__file__)),
        "episode": str(EPISODE.relative_to(ROOT)), "qa_summary": str((QA / "summary.json").relative_to(ROOT)),
        "qa_status": qa["status"], "fixed_selected_window_count": 327, "necessary_window_checks_clear_count": 327,
        "training_data_approved": False, "dataset_conversion_performed": qa["dataset_conversion_performed"],
        "source_sha256": {str(path.relative_to(ROOT)): sha256(path) for path in sources},
        "sampled_image_sha256": image_sources,
        "selected_metrics": {
            "first_frame": selected[0]["frame"], "last_frame": selected[-1]["frame"],
            "first_timestamp_ns": first_ns, "last_timestamp_ns": selected[-1]["timestamp_ns"],
            "source_timestamp_span_sec": (selected[-1]["timestamp_ns"] - first_ns) / 1e9,
            "first_progress_m": selected[0]["progress_m"], "last_progress_m": selected[-1]["progress_m"],
            "camera_anchor_speed_mps": {"count": len(anchor_speeds), "minimum": min(anchor_speeds),
                                        "mean": float(np.mean(anchor_speeds)), "maximum": max(anchor_speeds)},
            "native_speed_in_selected_time_interval_mps": {"count": len(native_speeds), "minimum": min(native_speeds),
                                                           "mean": float(np.mean(native_speeds)), "maximum": max(native_speeds)},
            "speed_definition": "hypot(recorded ROS rear-axle base_link vx, vy); not XY interval-average speed",
            "progress_definition": "saved independent nearest-route-segment planar arc; no reconstruction or trimming",
        },
        "playback": {
            "actual_image_frames": len(sampled), "actual_camera_jpegs": len(image_sources),
            "native_camera_hz": 10.0, "nominal_visual_sampling_hz": 2.0, "selected_anchor_stride": STEP,
            "selection": "Every fifth selected anchor, plus final selected anchor; whole fixed 40–120m span",
            "timing": "Each GIF delay equals the next retained source timestamp gap rounded to GIF 10ms units",
            "final_frame_hold_ms": FINAL_HOLD_MS, "gif_total_duration_sec": sum(durations) / 1000.0,
            "source_timestamp_span_sec": (sampled[-1]["timestamp_ns"] - sampled[0]["timestamp_ns"]) / 1e9,
            "visual_downsampling_only": True, "synthetic_camera_motion": False,
            "ten_hz_playback_claim": False, "runtime_latency_claim": False, "loop": True,
        },
        "viewport": {
            "reference": "current recorded ROS rear-axle base_link", "ego_reference_local_xy_m": [0.0, 0.0],
            "forward_bounds_m": [-VIEW_HALF_M, VIEW_HALF_M], "left_bounds_m": [-VIEW_HALF_M, VIEW_HALF_M],
            "forward_up": True, "fixed_scale": True,
            "trajectory": "actual saved 20Hz rear-axle trajectory through current frame",
            "body": "QA all_native geometry.body_polygon_carla_xy, converted CARLA y to ROS-map -y",
            "body_geometry": geometry,
        },
        "snapshot": sample_metadata[snapshot_index], "samples": sample_metadata,
        "limitations": [
            "One BasicAgent expert capture; no learned model control or Autoware UI was recorded.",
            "All 327 overlapping necessary-check windows are one same-corridor episode, not independent successes.",
            "Clear necessary checks do not approve training data, conversion, model promotion, vehicle control or route completion.",
            "Visual playback is downsampled actual simulator cameras, not a 10Hz playback or runtime-latency measurement.",
            "The 32.6s retained timestamp span is followed by a documented 0.1s final-frame hold in each GIF loop.",
        ],
        "artifacts": {name: {"sha256": sha256(OUTPUT / name), "bytes": (OUTPUT / name).stat().st_size}
                      for name in filenames[:3]},
    }
    (OUTPUT / filenames[3]).write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(OUTPUT), "selected_metrics": metadata["selected_metrics"],
                      "playback": metadata["playback"], "artifacts": metadata["artifacts"]}, indent=2))


if __name__ == "__main__":
    main()
