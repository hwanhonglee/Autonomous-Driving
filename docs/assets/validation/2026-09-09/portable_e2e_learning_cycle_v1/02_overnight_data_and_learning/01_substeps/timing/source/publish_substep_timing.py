#!/usr/bin/env python3
"""HH_260906 - Publish exact timing evidence and independently bind immutable substep visuals without importing model code."""

from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image, ImageChops

ROOT = Path(__file__).resolve().parents[3]
BASE = ROOT / "artifacts/training/2026-09-09"
PUBLIC = ROOT / "docs/assets/validation/2026-09-09/portable_e2e_learning_cycle_v1/02_overnight_data_and_learning/01_substeps"
EVIDENCE, OUTPUT = PUBLIC / "evidence", PUBLIC / "timing"
TIMING = BASE / "substep_ab_wall_timing_v1"
SUMMARY_SHA = "bace5d8654471e9efc64501ee9a797a187489b855229e5f2c9a30e091cc67d32"
CHECKSUM_SHA = "fcfc2f274bb5323060812f82f804fcf1c5eb7ff1e1bb085dd9aa085f88d7e668"
CASE_IDS = ("A_reference_10ms/run_001", "B_fine_5ms/run_001", "A_reference_10ms/run_002", "B_fine_5ms/run_002")
CAMERA_GRID = (("CAM_FRONT_LEFT", "CAM_FRONT", "CAM_FRONT_RIGHT"), ("CAM_BACK_LEFT", "CAM_BACK", "CAM_BACK_RIGHT"))


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def checked(path, pins, expected=None):
    path = Path(path)
    if not path.is_file() or any(p.is_symlink() for p in (path, *path.parents)) or not path.resolve().is_relative_to(ROOT):
        raise ValueError("unsafe source file")
    raw = path.read_bytes()
    relative, value = path.relative_to(ROOT).as_posix(), {"sha256": sha(raw), "size_bytes": len(raw)}
    if (expected is not None and value["sha256"] != expected) or (relative in pins and pins[relative] != value):
        raise ValueError("changed source: " + relative)
    pins[relative] = value
    return raw


def write(path, raw):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(raw)


def write_json(path, value):
    write(path, (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode())


def manifest_check(root, pins):
    raw = checked(root / "SHA256SUMS", pins)
    entries = {}
    for line in raw.decode().splitlines():
        digest, name = line.split("  ", 1)
        if name == "SHA256SUMS" or name in entries or Path(name).is_absolute() or ".." in Path(name).parts:
            raise ValueError("invalid checksum manifest entry")
        checked(root / name, pins, digest)
        entries[name] = digest
    actual = {p.relative_to(root).as_posix() for p in root.rglob("*") if p.is_file() and p.name != "SHA256SUMS" and "__pycache__" not in p.parts}
    if actual != set(entries):
        raise ValueError("checksum inventory is incomplete")
    return entries


def camera_tile_proof(canvas, episode, camera, pins):
    """HH_260906 - Independently reconstruct only the uncropped six image rectangles and compare all published PNG camera pixels."""
    matches = 0
    for r, names in enumerate(CAMERA_GRID):
        for c, name in enumerate(names):
            left, top, width, height = 20 + c * 320, 91 + r * 326, 310, 316
            with Image.open(io.BytesIO(checked(episode / camera["images"][name], pins))) as original:
                source = original.convert("RGB")
                scale = min(width / source.width, (height - 26) / source.height)
                size = (max(1, round(source.width * scale)), max(1, round(source.height * scale)))
                resized = source.resize(size, getattr(Image, "Resampling", Image).LANCZOS)
            x, y = left + (width - size[0]) // 2, top + 26 + (height - 26 - size[1]) // 2
            if ImageChops.difference(canvas.crop((x, y, x + size[0], y + size[1])), resized).getbbox() is not None:
                raise ValueError("published camera rectangle differs from exact full-FOV resize")
            matches += 1
    return matches


def verify_visuals(pins, raw_data):
    checksum_entries = manifest_check(EVIDENCE, pins)
    publication = json.loads(checked(EVIDENCE / "publication_manifest.json", pins))
    if publication["audit_sha256"] != "a6b2f7117dbddd33ee14aaef79add93658b611fffc008ce5e4570ce2d2cc289b" or publication["all_four_attempts_retained"] is not True:
        raise ValueError("publication audit/case denominator mismatch")
    for entry in publication["files"]:
        public = checked(EVIDENCE / entry["published_path"], pins, entry["sha256"])
        raw = checked(BASE / entry["raw_source_path"], pins, entry["raw_source_sha256"])
        if len(public) != entry["size_bytes"] or len(raw) != entry["raw_source_size_bytes"]:
            raise ValueError("publication byte count mismatch")
        if entry["representation"] == "original_bytes" and public != raw:
            raise ValueError("original published visual changed")
    results, png_count, camera_tiles = [], 0, 0
    for case_id in CASE_IDS:
        data = raw_data[case_id]
        episode, states, cameras = data["episode"], data["states"], data["cameras"]
        state_by_frame = {row["frame"]: row for row in states}
        image_sources, per_case = set(), {"case_id": case_id, "png_files": [], "metadata_provenance_count": 0}
        for group, filename in (("visuals", "visual_provenance.json"), ("milestones", "provenance.json")):
            directory = EVIDENCE / group / case_id
            proof = json.loads(checked(directory / filename, pins))
            if any(proof[k] is not False for k in ("training_data_approved", "learned_model_control", "live_autoware_screenshot")):
                raise ValueError("visual overclaims admission/control")
            raw_root = BASE / ("substep_ab_visuals_v1" if group == "visuals" else "substep_ab_milestones_v1") / case_id
            original = json.loads(checked(raw_root / filename, pins, proof["raw_source_sha256"]))
            derivative = {k: v for k, v in proof.items() if k not in ("publication_notice", "raw_source_sha256")}
            if derivative != original:
                raise ValueError("visual metadata derivative altered original fields")
            for path, digest in proof["source_metadata_sha256"].items():
                checked(episode / path, pins, digest)
            for path, digest in proof["displayed_image_sha256"].items():
                checked(episode / path, pins, digest)
                image_sources.add(path)
            checked(ROOT / "scripts/e2e/render_carla_raw_trial.py", pins, proof["source_renderer_sha256"])
            checked(ROOT / "scripts/e2e/render_carla_vad_expert.py", pins, proof["source_layout_sha256"])
            if group == "milestones":
                checked(BASE / "render_substep_milestones.py", pins, proof["source_script_sha256"])
                observed = [state_by_frame[c["frame"]] for c in cameras]
                length = data["manifest"]["result"]["route_length_m"]
                expected = {
                    "01_first_observed_13p68kph_cruise": next(i for i, row in enumerate(observed) if math.hypot(row["vx"], row["vy"]) >= 3.8),
                    "02_first_half_route_crossing": next(i for i, row in enumerate(observed) if row["route_progress_m"] >= length / 2),
                    "03_maximum_absolute_reported_steering": max(range(len(observed)), key=lambda i: abs(observed[i]["current_control"]["steer"]))}
                selections = proof["png_camera_indices"]
                if selections != expected or proof["png_state_frames"] != {k: cameras[i]["frame"] for k, i in selections.items()}:
                    raise ValueError("post-collection milestone selection mismatch")
            else:
                selections = proof["png_indices"]
                expected_indices = sorted(set(range(0, len(cameras), 5)) | {len(cameras) - 1})
                if proof["rendered_indices"] != expected_indices or proof["camera_anchor_count"] != len(cameras) or proof["playback_fps"] != 10 or proof["camera_stride"] != 5:
                    raise ValueError("GIF sampling denominator changed")
                gif_path = directory / "whole_recording_accelerated.gif"
                with Image.open(io.BytesIO(checked(gif_path, pins))) as image:
                    count = image.n_frames
                    if count != (195 if case_id.startswith("A_") else 204) or count != len(expected_indices):
                        raise ValueError("GIF frame omission")
                    for index in range(count):
                        image.seek(index); image.load()
                        if image.size != (1600, 900) or image.info.get("duration") != 100:
                            raise ValueError("GIF dimension/duration mismatch")
                per_case.update(gif_frame_count=count, gif_duration_seconds=count / 10, gif_indices=expected_indices)
            for name, index in selections.items():
                if not 0 <= index < len(cameras):
                    raise ValueError("PNG selection outside camera ledger")
                path = directory / (name + ".png")
                with Image.open(io.BytesIO(checked(path, pins))) as image:
                    image.load()
                    if image.size != (1600, 900):
                        raise ValueError("PNG dimensions changed")
                    camera_tiles += camera_tile_proof(image.convert("RGB"), episode, cameras[index], pins)
                per_case["png_files"].append({"path": path.relative_to(EVIDENCE).as_posix(), "camera_index": index, "frame": cameras[index]["frame"]})
                png_count += 1
            per_case["metadata_provenance_count"] += 1
        per_case["unique_displayed_original_jpeg_count"] = len(image_sources)
        results.append(per_case)
    if png_count != 30 or camera_tiles != 180:
        raise ValueError("expected all thirty PNGs / 180 full-FOV camera rectangles")
    return {"schema": "carla_expert.substep_visual_independent_verification.v1", "status": "PASS_NOT_ADMITTED",
        "evidence_regular_file_count": len(checksum_entries) + 1, "verified_checksum_payload_count": len(checksum_entries),
        "png_count": png_count, "gif_count": 4, "gif_frame_count": sum(c["gif_frame_count"] for c in results),
        "decoded_gif_dimensions": [1600, 900], "decoded_png_dimensions": [1600, 900],
        "full_fov_png_camera_rectangles_pixel_identical": camera_tiles, "cases": results,
        "all_source_hashes_rechecked": True, "existing_visuals_modified": False, "dataset_admission": False,
        "notice": "All published visual bytes match private originals. Eight metadata records, selected raw image hashes and thirty PNG six-camera full-FOV rectangles are checked independently. GIF frames are decoded/count/timing checked, not independently re-rendered. Ego-centered map geometry is source-bound, not pixel-level independently recomputed. No driving quality or model approval follows."}


def plot_native(data):
    figure, axes = plt.subplots(3, 1, figsize=(16, 9), sharex=True)
    for i, (case_id, rows) in enumerate(data.items()):
        states = rows["states"]; t = [r["timestamp"] - states[0]["timestamp"] for r in states]
        speed = [math.hypot(r["vx"], r["vy"]) for r in states]
        rates = [(b - a) / (tb - ta) for a, b, ta, tb in zip(speed, speed[1:], t, t[1:])]
        style = dict(color=("#1769aa" if i % 2 == 0 else "#dc5f19"), linestyle=("-" if i < 2 else "--"), alpha=.9, linewidth=1.4, label=("A1", "B1", "A2", "B2")[i])
        axes[0].plot(t, [v * 3.6 for v in speed], **style)
        axes[1].plot(t[1:], rates, **style)
        axes[2].plot(t, [r["goal_stop"]["remaining_route_arc_m"] for r in states], **style)
    axes[0].set_ylabel("Native speed (km/h)"); axes[0].legend(ncol=4, loc="upper right")
    axes[1].set_ylabel("Native delta-speed / dt (m/s²)")
    for bound in (-2.9, 2.9): axes[1].axhline(bound, color="#963f42", linestyle=":", linewidth=1)
    axes[2].set_ylabel("Remaining route arc (m)"); axes[2].set_xlabel("Recorded simulation time since first native state (s)")
    for axis in axes: axis.grid(alpha=.22)
    figure.suptitle("All native states retained: 10 ms reference vs 5 ms numerical substeps", fontsize=18)
    figure.text(.5, .015, "Same initial-condition repetitions overlap. Both B captures fail goal stopping; all four remain NOT ADMITTED. Dotted lines: unchanged ±2.9 m/s² decoder bounds.", ha="center", fontsize=10)
    figure.tight_layout(rect=(0, .035, 1, .95))
    return figure


def plot_wall(data, summary):
    figure, axes = plt.subplots(2, 1, figsize=(16, 9))
    for i, (case_id, value) in enumerate(data.items()):
        rows = [r for r in value["timing"] if r["camera_recorded"]]
        t = [r["sim_timestamp"] - value["states"][0]["timestamp"] for r in rows[1:]]
        gaps = [(b["camera_recorded_wall_ns"] - a["camera_recorded_wall_ns"]) / 1e6 for a, b in zip(rows, rows[1:])]
        axes[0].plot(t, gaps, color=("#1769aa" if i % 2 == 0 else "#dc5f19"), linestyle=("-" if i < 2 else "--"), alpha=.65, linewidth=1, label=("A1", "B1", "A2", "B2")[i])
    axes[0].set_ylabel("Camera completion gap (wall ms)"); axes[0].set_xlabel("Recorded simulation time (s)"); axes[0].legend(ncol=4)
    bottom = [0.] * 4
    colors = ("#5581b5", "#e49b3d", "#79a778", "#a46bb2", "#cc6573", "#a9a9a9")
    names = ("world_tick_snapshot", "observation_control", "camera_queue_wait", "jpeg_encode_write", "control_rpc", "unattributed")
    for name, color in zip(names, colors):
        values = [100 * (c["unattributed_fraction_of_in_tick_total"] if name == "unattributed" else c["stage_fraction_of_in_tick_total"][name]) for c in summary["cases"]]
        axes[1].bar(("A1", "B1", "A2", "B2"), values, bottom=bottom, color=color, label=name)
        bottom = [a + b for a, b in zip(bottom, values)]
    axes[1].set_ylabel("Share of summed in-tick duration (%)"); axes[1].set_ylim(0, 105); axes[1].legend(ncol=3, loc="upper center", bbox_to_anchor=(.5, -.12), fontsize=10)
    for axis in axes: axis.grid(alpha=.2, axis="y")
    figure.suptitle("Offline file completion is not display FPS or learned inference", fontsize=18)
    figure.text(.5, .015, "~35 Hz file completion at ~3.5× simulation speed. Stage fractions exclude inter-tick gaps/persistence/setup; world_tick_snapshot is not isolated physics time.", ha="center", fontsize=10)
    figure.tight_layout(rect=(0, .055, 1, .95))
    return figure


def main():
    if OUTPUT.exists(): raise ValueError("refuse existing publication directory")
    pins = {}; source = checked(Path(__file__).resolve(), pins)
    summary_raw = checked(TIMING / "summary.json", pins, SUMMARY_SHA)
    checked(TIMING / "SHA256SUMS", pins, CHECKSUM_SHA); manifest_check(TIMING, pins)
    summary = json.loads(summary_raw)
    if summary["case_count"] != 4 or tuple(c["case_id"] for c in summary["cases"]) != CASE_IDS or summary["dataset_admission"] is not False:
        raise ValueError("timing summary case/approval mismatch")
    data = {}
    for c in summary["cases"]:
        trial = BASE / "substep_ab_v1" / c["case_id"]
        episode = trial / ("episode" if c["capture_status"] == "complete" else "episode.partial")
        read = lambda name, lines=False: [json.loads(x) for x in checked(episode / name, pins).splitlines()] if lines else json.loads(checked(episode / name, pins))
        data[c["case_id"]] = dict(episode=episode, states=read("states.jsonl", True), cameras=read("camera_frames.jsonl", True), manifest=read("manifest.json"), timing=read("wall_timing.jsonl", True))
    for path, entry in summary["input_manifest"].items():
        checked(ROOT / path, pins, entry["sha256"])
    verification = verify_visuals(pins, data)
    OUTPUT.mkdir(parents=True, exist_ok=False)
    write(OUTPUT / "summary.json", summary_raw)
    write(OUTPUT / "source/publish_substep_timing.py", source)
    write(OUTPUT / "source/reconstruct_wall_timing.py", checked(TIMING / "source/reconstruct_wall_timing.py", pins))
    for name, figure in (("01_all_native_speed_acceleration_goal.png", plot_native(data)), ("02_actual_wall_gaps_and_in_tick_shares.png", plot_wall(data, summary))):
        with (OUTPUT / name).open("xb") as stream: figure.savefig(stream, format="png", dpi=100)
        plt.close(figure)
    for path, entry in list(pins.items()): checked(ROOT / path, pins, entry["sha256"])
    verification.update(verified_at_utc=datetime.now(timezone.utc).isoformat(), input_source_manifest=pins, verification_script_sha256=sha(source))
    write_json(OUTPUT / "visual_verification.json", verification)
    proof = {"schema": "carla_expert.substep_timing_publication.v1", "comment": "HH_260906 - Exact timing summary and immutable visual verification, with measured plots only.",
        "summary_representation": "original_bytes", "summary_raw_sha256": SUMMARY_SHA, "original_summary_manifest_sha256": CHECKSUM_SHA,
        "script_sha256": sha(source), "matplotlib_version": matplotlib.__version__, "matplotlib_init_sha256": sha(Path(matplotlib.__file__).read_bytes()),
        "all_inputs_unchanged_before_after": True, "current_model_imported": "portable_e2e.model" in sys.modules,
        "dataset_admission": False, "existing_evidence_modified": False, "gui_fps_measured": False, "learned_inference_measured": False,
        "source_snapshot_notice": "Source files preserve original bytes; execute from their original artifact paths in the recorded repository layout, not directly from this public snapshot folder.",
        "files": [{"path": p.relative_to(OUTPUT).as_posix(), "sha256": sha(p.read_bytes()), "size_bytes": p.stat().st_size} for p in sorted(OUTPUT.rglob("*")) if p.is_file()]}
    if proof["current_model_imported"]: raise ValueError("unexpected model import")
    write_json(OUTPUT / "publication_manifest.json", proof)
    for path in OUTPUT.rglob("*.json"):
        if re.search(r"/home/[^/]+|/root/|\b(?:\d{1,3}\.){3}\d{1,3}\b", path.read_text()):
            raise ValueError("public account path or IP found")
    paths = sorted(p for p in OUTPUT.rglob("*") if p.is_file())
    write(OUTPUT / "SHA256SUMS", "".join(sha(p.read_bytes()) + "  " + p.relative_to(OUTPUT).as_posix() + "\n" for p in paths).encode())
    print(json.dumps({"status": "PUBLISHED_NOT_ADMITTED", "png_verified": verification["png_count"], "gif_verified": verification["gif_count"], "camera_tiles_verified": verification["full_fov_png_camera_rectangles_pixel_identical"], "file_count": len(paths) + 1}))


if __name__ == "__main__":
    main()
