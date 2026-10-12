"""HH_260906 - Audit one byte-pinned Town05 right-turn sign DEV scout without training."""

from __future__ import annotations

import argparse
import importlib.util
import io
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[2]
OWNER = ROOT / "storage/experiments/training/2026-09-27/map_town05_right_sign_dev_scout_epic_v1"
ROUTE = ROOT / (
    "storage/experiments/validation/2026-09-01/"
    "autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/"
    "town05_opt/catalog/turn/routes/town05_opt/right/town05_opt_right_s0000_p00.json"
)
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/map_town05_sign_dev_scout"
ROUTE_SHA256 = "2f7a25e8ace753bfdd412ae7ef65710a001d1f4b1ecd5765f3ee7b021ced29e3"
FRAMES = 240
FRAME_WH = (640, 360)
MINIMUM_RETAINED = 120
MINIMUM_OBJECTS = 3
MINIMUM_QUALIFYING_FRAMES = 24
MINIMUM_OBJECT_PIXELS = 5


def _load_common():
    """HH_260906 - Reuse frozen native-frame hashing and all-prior-position retention rules."""
    path = ROOT / "tools/e2e/audit_map_generalization_town03.py"
    spec = importlib.util.spec_from_file_location("_town03_map_audit_helpers_t05_right", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Town03 native-frame audit helpers unavailable")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


common = _load_common()


def sign_bin(pixels: int) -> str:
    """HH_260906 - Bin whole-frame oracle tag-8 pixels, never independent sign objects."""
    common.require(isinstance(pixels, int) and pixels >= 0, "invalid tag-8 frame total")
    if pixels == 0:
        return "0"
    if pixels <= 4:
        return "1-4"
    if pixels <= 64:
        return "5-64"
    return ">=65"


def histogram(totals: list[int]) -> dict[str, int]:
    bins = {label: 0 for label in ("0", "1-4", "5-64", ">=65")}
    for total in totals:
        bins[sign_bin(total)] += 1
    return bins


def preliminary_status(retained_count: int, upper_bound_qualifying_frames: int) -> str:
    """HH_260906 - A whole-frame count below five disproves any qualifying object in that frame."""
    if retained_count < MINIMUM_RETAINED:
        return "REJECTED_INSUFFICIENT_DISTINCT_POSITIONS"
    if upper_bound_qualifying_frames < MINIMUM_QUALIFYING_FRAMES:
        return "REJECTED_SIGN_CAPABLE_DEV"
    return "PENDING_RGB_OBJECT_REVIEW_NOT_ADMITTED"


def audit_owner(owner: Path = OWNER, route_path: Path = ROUTE,
                expected_sha: str = ROUTE_SHA256) -> dict:
    """HH_260906 - Recheck exact route, Epic owner, source archive and CARLA lifecycle."""
    require, digest, read_json = common.require, common.digest, common.read_json
    source = owner / "mapping"
    require(owner.is_dir() and source.is_dir() and not owner.is_symlink()
            and not source.is_symlink(), "missing or unsafe owned Town05 source")
    require(route_path.is_file() and not route_path.is_symlink()
            and digest(route_path) == expected_sha, "Town05 right route bytes differ")
    route = read_json(route_path)
    require(route.get("town") == "Town05_Opt" and route.get("scenario") == "right"
            and route.get("weather") == "ClearNoon"
            and route.get("coordinate_reference") == "base_link"
            and len(route.get("route", [])) >= 2, "Town05 right route identity differs")
    paths = {key: owner / relative for key, relative in (
        ("plan", "owner_plan.json"), ("started", "owner_started.json"),
        ("result", "owner_result.json"), ("summary", "mapping/summary.json"))}
    require(all(path.is_file() and not path.is_symlink() for path in paths.values()),
            "missing or unsafe owner receipt")
    plan, started, result, summary = (read_json(paths[key]) for key in
                                      ("plan", "started", "result", "summary"))
    require(Path(plan["route_path"]).resolve() == route_path.resolve()
            and plan["route_sha256"] == started["route_sha256"]
            == summary["route_sha256"] == expected_sha,
            "owner route path or SHA differs")
    require(plan["map"] == started["map"] == summary["town"] == "Town05_Opt"
            and plan["quality"] == started["quality"] == "Epic"
            and plan["capture_mode"] == result["capture_mode"] == "mapping-probe"
            and plan.get("source_bytes_archived") is True
            and plan.get("bounds_source_bytes_archived") is True
            and result.get("source_bytes_unchanged_and_archived") is True
            and result["exit_code"] == 0
            and summary["status"] == "COMPLETE_ORACLE_PLUMBING_ONLY"
            and summary["research_split_intent"] == "town05_sign_dev_scout_only"
            and summary["frame_count"] == FRAMES
            and summary["physics_driving"] is True
            and summary["motion_mode"] == "basic-agent"
            and summary["controller"] == "CARLA BasicAgent"
            and summary["ignores_traffic_lights"] is True
            and all(value.get("learned_model_control") is False
                    for value in (plan, started, result, summary))
            and plan.get("vehicle_control_approved") is False
            and result.get("vehicle_control_approved") is False
            and summary["oracle_pose"] is True
            and summary["oracle_depth"] is True
            and summary["oracle_semantics"] is True
            and summary["collision_events"] == 0
            and summary["live_replay_exact_match"] is True
            and not summary["cleanup_errors"]
            and summary["hd_map_approved"] is False
            and {int(tag): label for tag, label in summary["label_table"].items()}
            == common.TAG_TO_LABEL,
            "Town05 right physical owner proof failed")
    argv = plan["collector_argv"]
    require(isinstance(argv, list) and len(argv) >= 2
            and Path(argv[0]).resolve() == source.resolve()
            and Path(argv[1]).resolve() == route_path.resolve()
            and common.option_value(argv, "--host") == plan["host"] == "127.0.0.1"
            and int(common.option_value(argv, "--port")) == plan["port"] == started["port"]
            and common.option_value(argv, "--frames") == str(FRAMES)
            and common.option_value(argv, "--motion-mode") == "basic-agent"
            and argv.count("--ignore-traffic-lights") == 1
            and argv.count("--town05-sign-dev-scout") == 1
            and len([option for option in argv if option.startswith("--town")]) == 1,
            "Town05 right collector scope differs")
    lifecycle_sha = {}
    generation = None
    for stage in ("ready", "after_capture", "stopped"):
        path = owner / "lifecycle" / f"{stage}.json"
        require(path.is_file() and not path.is_symlink(), f"missing lifecycle {stage}")
        event = read_json(path)
        require(event["stage"] == stage and event["status"] == "PASS"
                and event["expected_map"] == "Town05_Opt"
                and event["owner_pid"] == started["server_pid"]
                and event["owner_pgid"] == started["server_pgid"]
                and event["host"] == "127.0.0.1" and event["port"] == plan["port"]
                and (stage == "stopped" or event["active_map_basename"] == "Town05_Opt")
                and (stage != "stopped" or event.get("port_released") is True)
                and (generation is None or event["generation_id"] == generation),
                f"CARLA lifecycle failed at {stage}")
        generation = event["generation_id"]
        lifecycle_sha[stage] = digest(path)
    require(digest(owner / "server.log") == read_json(owner / "lifecycle/stopped.json")
            ["server_log"]["sha256"], "final CARLA server log differs")
    source_hashes = plan["source_sha256"]
    checks = result["source_checks"]
    require(isinstance(source_hashes, dict) and bool(source_hashes)
            and set(checks) == set(source_hashes)
            and all(checks.values())
            and all(result.get("external_source_checks", {}).values()),
            "source archive checks incomplete")
    for relative, expected in source_hashes.items():
        part = Path(relative)
        archive = owner / "provenance" / part
        require(not part.is_absolute() and ".." not in part.parts
                and archive.is_file() and not archive.is_symlink()
                and archive.resolve().is_relative_to((owner / "provenance").resolve())
                and digest(archive) == expected,
                f"archived source differs: {relative}")
    return {
        "owner_plan_sha256": digest(paths["plan"]),
        "owner_started_sha256": digest(paths["started"]),
        "owner_result_sha256": digest(paths["result"]),
        "mapping_summary_sha256": digest(paths["summary"]),
        "lifecycle_sha256": lifecycle_sha,
        "archived_source_count": len(source_hashes),
        "route_sha256": expected_sha,
        "town": "Town05_Opt", "quality": "Epic", "exit_code": 0,
        "travel_m": summary["travel_m"],
        "agent_goal_reached": summary["agent_goal_reached"],
        "collision_events": 0, "live_replay_exact_match": True,
        "port_released_after_stop": True,
    }


def build_summary(owner: Path = OWNER, route_path: Path = ROUTE) -> dict:
    """HH_260906 - Decode all source pairs and reject a sign gate using strict upper bounds."""
    require = common.require
    provenance = audit_owner(owner, route_path)
    source = owner / "mapping"
    frame_dir = source / "frames"
    require(frame_dir.is_dir() and not frame_dir.is_symlink(), "native frame directory missing")
    require(len(list(frame_dir.glob("*.png"))) == FRAMES
            and len(list(frame_dir.glob("*.npz"))) == FRAMES,
            "native RGB/sensor pair count differs")
    frames = common.audit_frames(source, expected_frames=FRAMES, frame_wh=FRAME_WH)
    records = common.read_json(source / "records.json")
    require(len(records) == FRAMES, "native record count differs")
    for row in frames:
        with Image.open(source / (row["stem"] + ".png")) as rgb:
            decoded = np.asarray(rgb)
            require(decoded.shape == (360, 640, 3) and decoded.dtype == np.uint8,
                    f"native full-FOV RGB decode differs: {row['stem']}")
        with np.load(source / (row["stem"] + ".npz"), allow_pickle=False) as sensor:
            require(set(sensor.files) == {"depth", "semantic", "world_from_camera"}
                    and sensor["depth"].shape == (360, 640, 4)
                    and sensor["depth"].dtype == np.uint8
                    and sensor["world_from_camera"].shape == (4, 4)
                    and np.isfinite(sensor["world_from_camera"]).all(),
                    f"native synchronized depth/pose differs: {row['stem']}")
    saved_map_path = source / "map.json"
    require(saved_map_path.is_file() and not saved_map_path.is_symlink(),
            "saved replay map missing")
    saved_map = common.read_json(saved_map_path)
    capture = common.read_json(source / "summary.json")
    require(saved_map.get("schema") == "portable_e2e.semantic_evidence.v1"
            and saved_map.get("frame_id") == "carla_ue:Town05_Opt"
            and saved_map.get("source") == "oracle_sensor"
            and saved_map.get("hd_map_approved") is False
            and len(saved_map.get("cells", [])) == capture["observed_cells"]
            and math.isclose(saved_map["timestamp_s"], records[-1]["timestamp_s"],
                             rel_tol=0.0, abs_tol=1e-6),
            "saved live/replay map evidence differs")
    cadence_errors = [abs((frames[index]["timestamp_s"]
                           - frames[index - 1]["timestamp_s"]) - .1)
                      for index in range(1, FRAMES)]
    require(all(math.isfinite(value) and value <= .001 for value in cadence_errors),
            "native 10 Hz cadence differs")
    kept, _ = common.retain_indices(frames, minimum_m=.25)
    kept_set = set(kept)
    sign_index = common.LABELS.index("traffic_sign")
    sign_totals = [row["class_pixels"][sign_index] for row in frames]
    retained_totals = [sign_totals[index] for index in kept]
    raw_bins, retained_bins = histogram(sign_totals), histogram(retained_totals)
    require(sum(raw_bins.values()) == FRAMES
            and sum(retained_bins.values()) == len(kept), "tag-8 histogram lost frames")
    # HH_260906 - A physical sign object's pixels are a subset of all tag-8 pixels in its frame.
    upper_bound_qualifying_frames = sum(total >= MINIMUM_OBJECT_PIXELS
                                        for total in retained_totals)
    status = preliminary_status(len(kept), upper_bound_qualifying_frames)
    chosen = [kept[0], max(kept, key=lambda index: (sign_totals[index], -index)), kept[-1]]
    require(len(set(chosen)) == 3, "three distinct retained full-FOV scenes required")
    impossible_qualifying_object = max(retained_totals, default=0) < MINIMUM_OBJECT_PIXELS
    return {
        "schema": "portable_e2e.map_town05_sign_dev_scout_audit.v1",
        "status": status,
        "scope": "pinned Town05_Opt right p00 Epic raw DEV scout; no model inference/training/admission",
        "provenance": provenance,
        "saved_replay_map_sha256": common.digest(saved_map_path),
        "native_frame_wh": list(FRAME_WH),
        "raw_frames": FRAMES,
        "retained_frames_0p25m": len(kept),
        "retention_rule": "keep frame iff >=0.25m from every earlier retained Town05_Opt ego XY",
        "minimum_retained": MINIMUM_RETAINED,
        "retained_gate_passed": len(kept) >= MINIMUM_RETAINED,
        "max_cadence_error_s": max(cadence_errors, default=0.0),
        "labels": list(common.LABELS),
        "raw_class_pixels": common.add_pixels(frames),
        "retained_class_pixels": common.add_pixels([frames[index] for index in kept]),
        "sign_measure": "CARLA tag 8 pixels over the entire native semantic frame, not distinct objects",
        "raw_sign_frame_bins": raw_bins,
        "retained_sign_frame_bins": retained_bins,
        "raw_tag8_pixel_sum": sum(sign_totals),
        "retained_tag8_pixel_sum": sum(retained_totals),
        "raw_max_whole_frame_tag8_pixels": max(sign_totals, default=0),
        "retained_max_whole_frame_tag8_pixels": max(retained_totals, default=0),
        "minimum_rgb_confirmed_distinct_sign_objects": MINIMUM_OBJECTS,
        "minimum_rgb_confirmed_object_pixels": MINIMUM_OBJECT_PIXELS,
        "minimum_retained_frames_with_qualifying_object": MINIMUM_QUALIFYING_FRAMES,
        "retained_qualifying_frame_upper_bound": upper_bound_qualifying_frames,
        "qualifying_rgb_confirmed_distinct_object_count": 0 if impossible_qualifying_object else None,
        "all_rgb_sign_objects_exhaustively_counted": False,
        "rgb_object_review": (
            "No >=5px tag-8 object can exist: even the entire frame has fewer than five tag-8 pixels. "
            "Full-FOV first/maximum/last views were checked; independent physical sign count at any scale "
            "is not claimed."
            if impossible_qualifying_object else
            "Possible >=5px frames require independent full-FOV RGB object tracing before admission."
        ),
        "sign_dev_gate_passed": False,
        "visual_stems": [frames[index]["stem"] for index in chosen],
        "retained_stems": [frames[index]["stem"] for index in kept],
        "frames": [{**row, "tag8_pixels": sign_totals[index],
                    "retained_0p25m": index in kept_set}
                   for index, row in enumerate(frames)],
        "dataset_admitted": False, "model_predictions_made": False,
        "model_trained": False, "vehicle_control_approved": False,
        "hd_map_approved": False,
    }


def preview(summary: dict, owner: Path = OWNER) -> Image.Image:
    """HH_260906 - Preserve native full FOV for RGB, semantic oracle and tag-8 overlay."""
    common.require(summary["native_frame_wh"] == [640, 360], "native full FOV required")
    width, height = FRAME_WH
    output = Image.new("RGB", (3 * width + 20, 72 + 3 * 398 + 40), (247, 249, 252))
    draw = ImageDraw.Draw(output)
    draw.text((15, 12), "Town05_Opt RIGHT Epic DEV scout | RGB / oracle / tag-8 overlay",
              fill=(25, 37, 49), font=common.font(21))
    draw.text((15, 43), "Each panel retains the native 640x360 camera field of view.",
              fill=(65, 76, 88), font=common.font(16))
    for index, stem in enumerate(summary["visual_stems"]):
        with Image.open(owner / "mapping" / (stem + ".png")) as raw:
            rgb = raw.convert("RGB")
        with np.load(owner / "mapping" / (stem + ".npz"), allow_pickle=False) as sensor:
            tags = sensor["semantic"][..., 2].copy()
        palette = np.empty((height, width, 3), dtype=np.uint8)
        palette[:] = common.COLORS[0]
        for tag, label in common.TAG_TO_LABEL.items():
            palette[tags == tag] = common.COLORS[common.LABELS.index(label)]
        overlay = np.asarray(rgb).copy()
        overlay[tags == 8] = np.array([255, 0, 200], dtype=np.uint8)
        y = 72 + index * 398
        draw.text((15, y), f"{stem} | whole-frame tag 8 = {int(np.count_nonzero(tags == 8))} px",
                  fill=(25, 37, 49), font=common.font(17))
        output.paste(rgb, (0, y + 26))
        output.paste(Image.fromarray(palette), (width, y + 26))
        output.paste(Image.fromarray(overlay), (2 * width, y + 26))
    draw.text((15, output.height - 34),
              "Magenta = CARLA oracle tag 8; not a model prediction or verified sign object.",
              fill=(65, 76, 88), font=common.font(16))
    return output


def _report_bytes(summary: dict) -> bytes:
    return (json.dumps(summary, indent=2, ensure_ascii=False) + "\n").encode("utf-8")


def _picture_bytes(picture: Image.Image) -> bytes:
    buffer = io.BytesIO()
    picture.save(buffer, format="PNG")
    return buffer.getvalue()


def write_or_check(summary: dict, picture: Image.Image, *, write: bool = False,
                   output: Path = OUTPUT, owner: Path = OWNER) -> None:
    """HH_260906 - Publish new evidence once, then verify the immutable report and PNG pixels."""
    common.require(output.is_dir() and not output.is_symlink(), "report folder missing or unsafe")
    common.require(not output.resolve().is_relative_to(owner.resolve()),
                   "report output overlaps original CARLA capture")
    report_path = output / "summary.json"
    picture_path = output / "town05_right_rgb_oracle_full_fov.png"
    if write:
        common.require(not report_path.exists() and not picture_path.exists(),
                       "audit artifact already exists")
        report_path.write_bytes(_report_bytes(summary))
        picture_path.write_bytes(_picture_bytes(picture))
        return
    common.require(report_path.is_file() and not report_path.is_symlink()
                   and report_path.read_bytes() == _report_bytes(summary),
                   "published audit JSON differs")
    common.require(picture_path.is_file() and not picture_path.is_symlink()
                   and picture_path.read_bytes() == _picture_bytes(picture),
                   "published full-FOV PNG differs")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true", help="write fresh report and preview once")
    args = parser.parse_args()
    summary = build_summary()
    write_or_check(summary, preview(summary), write=args.write)
    print(json.dumps({"status": summary["status"],
                      "retained_frames_0p25m": summary["retained_frames_0p25m"],
                      "retained_sign_frame_bins": summary["retained_sign_frame_bins"],
                      "retained_qualifying_frame_upper_bound":
                          summary["retained_qualifying_frame_upper_bound"]},
                     ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
