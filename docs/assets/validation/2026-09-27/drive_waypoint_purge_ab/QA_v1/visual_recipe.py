#!/usr/bin/env python3
"""HH_260906 - Reproduce B/C BasicAgent visuals from byte-pinned raw records."""

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
BASE = ROOT / "storage/experiments/training/2026-09-27"
EPISODE_B = BASE / "drive_long_route_town01_right_s0002_p06_9kph_steer045_trial_v1/episode"
EPISODE_C = BASE / "drive_long_route_town01_right_s0002_p06_9kph_steer045_purge15_trial_v1/episode"
REPORT_B = ROOT / "docs/assets/validation/2026-09-27/drive_steering_cap_ab/QA_v1/summary.json"
PLOT = QA / "bc_topdown_raw_turn.png"
CONTACT = QA / "C_raw_six_camera_first_middle_last.png"
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
    assert report["status"] == "NECESSARY_CHECKS_CLEAR_NOT_ADMITTED"
    assert report["dataset_admitted"] is False and report["lane_gate"]["status"] == "PASS"
    source_c = report["source_binding"]["c_raw_metadata"]
    source_b = report["source_binding"]["b"]["raw_metadata_sha256"]
    assert sha(REPORT_B) == report["source_binding"]["b"]["summary_sha256"]
    report_b = json.loads(REPORT_B.read_text(encoding="utf-8"))
    for name in ("route.json", "states.jsonl", "camera_frames.jsonl"):
        assert sha(EPISODE_C / name) == source_c["episode/" + name]["sha256"]
        assert sha(EPISODE_B / name) == source_b["episode/" + name]
    assert sha(EPISODE_B / "route.json") == sha(EPISODE_C / "route.json")
    route = json.loads((EPISODE_C / "route.json").read_text(encoding="utf-8"))["route"]
    states_b = lines(EPISODE_B / "states.jsonl")
    states_c = lines(EPISODE_C / "states.jsonl")
    cameras_c = lines(EPISODE_C / "camera_frames.jsonl")
    window_b, window_c = report_b["window"], report["window"]
    first_b, last_b = window_b["first_selected"]["frame"], window_b["last_selected"]["frame"]
    first_c, last_c = window_c["first_selected"]["frame"], window_c["last_selected"]["frame"]
    selected_b = [state for state in states_b if first_b <= state["frame"] <= last_b]
    selected_c = [state for state in states_c if first_c <= state["frame"] <= last_c]
    selected_cameras = [row for row in cameras_c if first_c <= row["frame"] <= last_c]
    assert len(selected_cameras) == window_c["selected_anchor_count"] == 363
    assert selected_cameras[0]["frame"] == first_c and selected_cameras[-1]["frame"] == last_c
    assert len(selected_b) == report["comparison"]["b"]["selected_native_state_count"]
    assert len(selected_c) == report["comparison"]["c"]["selected_native_state_count"]

    # HH_260906 - Center on C's already reported maximum curvature witness, not a handpicked frame.
    peak = report["future"]["original_nine_metrics"]["metrics"]["xy_curvature"]["maximum"]
    anchor_frame = int(peak["sample_id"].split(":")[1])
    state_by_frame = {row["frame"]: row for row in states_c}
    anchor = state_by_frame[anchor_frame]
    target = min(states_c, key=lambda row: abs(round(row["timestamp"] * 1e9) - peak["target_timestamp_ns"]))
    assert anchor_frame == 2829 and target["frame"] == 2833
    assert abs(round(target["timestamp"] * 1e9) - peak["target_timestamp_ns"]) <= 5
    ox, oy = anchor["x"], anchor["y"]

    def relative(rows: list[dict]) -> tuple[list[float], list[float]]:
        return [row["x"] - ox for row in rows], [row["y"] - oy for row in rows]

    route_window = [point for point in route if 30. <= point["distance_m"] <= 120.]
    fig, ax = plt.subplots(figsize=(8.5, 8.5), dpi=150)
    ax.plot(*relative(route), color="#b9bec6", linewidth=1.5, label="Original route center (all)")
    ax.plot(*relative(route_window), color="#287fbd", linewidth=3,
            label="Original route center, 30–120 m")
    ax.plot(*relative(selected_b), color="#bb6243", linewidth=2, label="B measured base_link")
    ax.plot(*relative(selected_c), color="#248f62", linewidth=2, label="C measured base_link")
    near = [state_by_frame[frame] for frame in (2829, 2831, 2833, 2835)]
    ax.plot(*relative(near), color="#a94ab4", linewidth=2.5, marker="o", markersize=4,
            label="C measured 0.1 s steps near peak")
    ax.scatter([0], [0], marker="s", s=100, color="#185143", edgecolor="white", linewidth=1,
               label=f"C ego at anchor {anchor_frame}", zorder=7)
    ax.scatter([target["x"] - ox], [target["y"] - oy], marker="X", s=150,
               color="#cb3f39", edgecolor="white", linewidth=1,
               label=f"C peak tick {target['frame']}", zorder=8)
    ax.arrow(0, 0, 4 * math.cos(anchor["yaw"]), 4 * math.sin(anchor["yaw"]),
             head_width=.6, head_length=.8, color="#185143", linewidth=2,
             length_includes_head=True, zorder=6)
    ax.set_xlim(-18, 18)
    ax.set_ylim(-18, 18)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("East of C anchor base_link (m)")
    ax.set_ylabel("North of C anchor base_link (m)")
    ax.set_title("Town01 RIGHT: BasicAgent expert B/C measured paths\n"
                 "Offline raw-coordinate diagnostic, not Autoware/learned driving")
    ax.grid(alpha=.2)
    ax.legend(loc="upper left", fontsize=7, framealpha=.95)
    ax.text(.02, .02, "Route centerline shown; lane boundaries not drawn", transform=ax.transAxes,
            fontsize=8, color="#67352d",
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": .9})
    fig.tight_layout()
    fig.savefig(PLOT)
    plt.close(fig)

    # HH_260906 - Fixed first/middle/last C anchors; no pixel crop or resize.
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
            source_path = EPISODE_C / relative_path
            digest = sha(source_path)
            assert audit[relative_path]["sha256"] == digest and audit[relative_path]["decoded"]
            with Image.open(source_path) as raw_image:
                raw_image.load()
                assert raw_image.size == (640, 360) and raw_image.mode == "RGB"
                native = raw_image.copy()
            column, row = number % 3, group * 2 + number // 3
            x, y = 640 * column, (360 + 26) * row
            draw.text((x + 5, y + 5), f"C frame {camera['frame']}  {name}", fill="black")
            canvas.paste(native, (x, y + 26))
            inputs.append({"frame": camera["frame"], "camera": name,
                           "path": str(source_path.relative_to(ROOT)), "sha256": digest})
    canvas.save(CONTACT)
    MANIFEST.write_text(json.dumps({
        "schema": "carla.town01_purge15_bc_visual.v1",
        "kind": "offline_raw_evidence_not_carla_or_autoware_gui",
        "scope": "CARLA BasicAgent expert B/C same-site diagnostic; not lane proof, learned driving, or admission",
        "fixed_selection": "All 363 C selected anchors retained; camera sample first/middle/last only",
        "camera_selection_indices": indices, "C_camera_frames": [row["frame"] for row in chosen],
        "contact_layout": "three columns × two rows per anchor; native 640×360, no crop or resize",
        "C_curvature_peak_anchor_frame": anchor_frame, "C_peak_target_frame": target["frame"],
        "C_peak_curvature_per_m": peak["value"], "unchanged_limit_per_m": .2,
        "summary_sha256": sha(report_path), "B_summary_sha256": sha(REPORT_B),
        "route_sha256": sha(EPISODE_C / "route.json"),
        "B_states_sha256": sha(EPISODE_B / "states.jsonl"),
        "C_states_sha256": sha(EPISODE_C / "states.jsonl"),
        "C_camera_frames_sha256": sha(EPISODE_C / "camera_frames.jsonl"),
        "recipe_sha256": sha(Path(__file__)), "topdown_sha256": sha(PLOT),
        "contact_sha256": sha(CONTACT), "source_images": inputs,
    }, indent=2, allow_nan=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
