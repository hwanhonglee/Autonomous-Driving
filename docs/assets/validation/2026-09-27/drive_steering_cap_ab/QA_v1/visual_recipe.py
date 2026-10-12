#!/usr/bin/env python3
"""Reproduce supplementary B BasicAgent visuals from byte-pinned raw inputs only.

HH_260906: These images are offline diagnostics, not admission or GUI evidence.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[6]
QA = Path(__file__).resolve().parent
EPISODE = ROOT / "storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_steer045_trial_v1/episode"
PLOT = QA / "topdown_raw_turn_peak.png"
CONTACT = QA / "raw_six_camera_first_middle_last.png"
MANIFEST = QA / "visual_manifest.json"
CAMERAS = ("CAM_FRONT", "CAM_BACK", "CAM_FRONT_LEFT", "CAM_BACK_LEFT",
           "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT")


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def lines(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream]


def main() -> None:
    assert not any(path.exists() for path in (PLOT, CONTACT, MANIFEST)), "fresh visuals required"
    report_path = QA / "summary.json"
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["status"] == "FAIL_CLOSED" and report["dataset_admitted"] is False
    assert report["failed_checks"] == ["full_body_in_intended_lane"]
    source = report["source_binding"]["b_raw_metadata"]
    for name in ("route.json", "states.jsonl", "camera_frames.jsonl"):
        assert sha(EPISODE / name) == source["episode/" + name]["sha256"], "raw source changed"
    route = json.loads((EPISODE / "route.json").read_text(encoding="utf-8"))["route"]
    states = lines(EPISODE / "states.jsonl")
    cameras = lines(EPISODE / "camera_frames.jsonl")
    state_by_frame = {row["frame"]: row for row in states}
    window = report["window"]
    first, last = window["first_selected"]["frame"], window["last_selected"]["frame"]
    selected_cameras = [row for row in cameras if first <= row["frame"] <= last]
    assert len(selected_cameras) == window["selected_anchor_count"] == 360
    assert selected_cameras[0]["frame"] == first and selected_cameras[-1]["frame"] == last
    selected_states = [row for row in states if first <= row["frame"] <= last]

    # HH_260906: Choose the already reported maximum tick, without moving the QA threshold.
    peak = report["future"]["original_nine_metrics"]["metrics"]["xy_curvature"]["maximum"]
    anchor_frame = int(peak["sample_id"].split(":")[1])
    anchor = state_by_frame[anchor_frame]
    target = min(states, key=lambda row: abs(round(row["timestamp"] * 1e9) - peak["target_timestamp_ns"]))
    assert anchor_frame == 2753 and target["frame"] == 2757
    assert abs(round(target["timestamp"] * 1e9) - peak["target_timestamp_ns"]) <= 5
    ox, oy = anchor["x"], anchor["y"]

    def relative(rows: list[dict]) -> tuple[list[float], list[float]]:
        return [row["x"] - ox for row in rows], [row["y"] - oy for row in rows]

    route_window = [point for point in route if 30. <= point["distance_m"] <= 120.]
    near = [state_by_frame[frame] for frame in (2753, 2755, 2757, 2759)]
    fig, ax = plt.subplots(figsize=(8.5, 8.5), dpi=150)
    ax.plot(*relative(route), color="#b6bec8", linewidth=1.5, label="Original route (all)")
    ax.plot(*relative(route_window), color="#2584bd", linewidth=3,
            label="Original route, 30–120 m")
    ax.plot(*relative(selected_states), color="#238d63", linewidth=2,
            label="Measured base_link, selected interval")
    ax.plot(*relative(near), color="#b150b6", linewidth=3, marker="o", markersize=4,
            label="Measured 0.1 s steps near B peak")
    ax.scatter([0], [0], marker="s", s=100, color="#1c5347", edgecolor="white",
               linewidth=1, label=f"Ego at anchor {anchor_frame}", zorder=7)
    ax.scatter([target["x"] - ox], [target["y"] - oy], marker="X", s=150,
               color="#d44136", edgecolor="white", linewidth=1,
               label=f"Peak tick, frame {target['frame']}", zorder=8)
    ax.arrow(0, 0, 4 * math.cos(anchor["yaw"]), 4 * math.sin(anchor["yaw"]),
             head_width=.6, head_length=.8, color="#1c5347", linewidth=2,
             length_includes_head=True, zorder=6)
    ax.set_xlim(-18, 18)
    ax.set_ylim(-18, 18)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("East of anchor base_link (m)")
    ax.set_ylabel("North of anchor base_link (m)")
    ax.set_title("Town01 RIGHT, B normalized steering cap 0.45\n"
                 "Offline raw-coordinate diagnostic, CARLA BasicAgent expert")
    ax.grid(alpha=.2)
    ax.legend(loc="upper left", fontsize=7, framealpha=.95)
    ax.text(.02, .02, "No intended-lane body-containment proof", transform=ax.transAxes,
            fontsize=8, color="#7e2830",
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": .9})
    fig.tight_layout()
    fig.savefig(PLOT)
    plt.close(fig)

    # HH_260906: Fixed first/middle/last selected anchors; preserve all 640×360 pixels.
    indices = (0, (len(selected_cameras) - 1) // 2, len(selected_cameras) - 1)
    chosen = [selected_cameras[index] for index in indices]
    audit = {row["path"]: row for row in lines(QA / "jpeg_audit.jsonl")}
    canvas = Image.new("RGB", (640 * 3, (360 + 26) * len(chosen) * 2), "white")
    draw = ImageDraw.Draw(canvas)
    inputs = []
    for group, camera in enumerate(chosen):
        assert tuple(camera["camera_order"]) == CAMERAS
        for number, name in enumerate(CAMERAS):
            relative_path = camera["images"][name]
            source_path = EPISODE / relative_path
            digest = sha(source_path)
            assert audit[relative_path]["sha256"] == digest and audit[relative_path]["decoded"]
            with Image.open(source_path) as raw_image:
                raw_image.load()
                assert raw_image.size == (640, 360) and raw_image.mode == "RGB"
                native = raw_image.copy()
            column, row = number % 3, group * 2 + number // 3
            x, y = 640 * column, (360 + 26) * row
            draw.text((x + 5, y + 5), f"B frame {camera['frame']}  {name}", fill="black")
            canvas.paste(native, (x, y + 26))
            inputs.append({"frame": camera["frame"], "camera": name,
                           "path": str(source_path.relative_to(ROOT)), "sha256": digest})
    canvas.save(CONTACT)
    MANIFEST.write_text(json.dumps({
        "schema": "carla.town01_steer045_ab_visual.v1",
        "kind": "offline_raw_evidence_not_carla_or_autoware_gui",
        "scope": "CARLA BasicAgent expert B; not learned driving, full-body lane proof, or admission",
        "fixed_selection": "All 360 selected anchors retained; camera sample first/middle/last only",
        "camera_selection_indices": indices, "camera_frames": [row["frame"] for row in chosen],
        "contact_layout": "three columns × two rows per anchor; native 640×360, no crop or resize",
        "curvature_peak_anchor_frame": anchor_frame, "peak_target_frame": target["frame"],
        "peak_curvature_per_m": peak["value"], "unchanged_limit_per_m": .2,
        "summary_sha256": sha(report_path), "route_sha256": sha(EPISODE / "route.json"),
        "states_sha256": sha(EPISODE / "states.jsonl"),
        "camera_frames_sha256": sha(EPISODE / "camera_frames.jsonl"),
        "recipe_sha256": sha(Path(__file__)), "topdown_sha256": sha(PLOT),
        "contact_sha256": sha(CONTACT), "source_images": inputs,
    }, indent=2, allow_nan=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
