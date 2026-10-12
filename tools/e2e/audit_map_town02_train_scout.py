"""HH_260906 - Audit the pinned Town02_Opt sign scout without training or admission."""

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
OWNER = ROOT / "storage/experiments/training/2026-09-27/map_town02_sign_train_scout_epic_v1"
ROUTE = ROOT / ("storage/experiments/validation/2026-09-01/"
                "autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/"
                "town02_opt/catalog/straight/routes/town02_opt/straight/"
                "town02_opt_straight_s0000_p00.json")
ROUTE_SHA256 = "520fc0236e32d0664bc125a15f45601e4ef020cf1e3f1db888332af0e188f08b"
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/map_town02_train_scout"
REPORT_NAME = "summary.json"
PICTURE_NAME = "town02_rgb_oracle_full_fov.png"
FRAME_COUNT = 240
FRAME_WH = (640, 360)
MINIMUM_RETAINED = 120
MINIMUM_TINY_SIGN_FRAMES = 24


def _load_common():
    """HH_260906 - Share the native sensor decode and all-prior-position retention rule."""
    path = ROOT / "tools/e2e/audit_map_generalization_town03.py"
    spec = importlib.util.spec_from_file_location("_town03_map_audit_helpers", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Town03 source-audit helpers unavailable")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


common = _load_common()


def sign_bin(pixels: int) -> str:
    """HH_260906 - Bin the total tag-8 pixel count across one entire native frame."""
    common.require(isinstance(pixels, int) and pixels >= 0, "invalid tag-8 pixel total")
    if pixels == 0:
        return "0"
    if pixels <= 4:
        return "1-4"
    if pixels <= 64:
        return "5-64"
    return ">=65"


def frame_histogram(pixels: list[int]) -> dict[str, int]:
    bins = {label: 0 for label in ("0", "1-4", "5-64", ">=65")}
    for total in pixels:
        bins[sign_bin(total)] += 1
    return bins


def qualification_status(retained_count: int, retained_bins: dict[str, int]) -> str:
    """HH_260906 - Keep both preregistered thresholds fixed at their inclusive boundaries."""
    if retained_count < MINIMUM_RETAINED:
        return "REJECTED_INSUFFICIENT_DISTINCT_POSITIONS"
    if retained_bins["1-4"] < MINIMUM_TINY_SIGN_FRAMES:
        return "REJECTED_SIGN_RICH_TRAIN_CANDIDATE"
    return "SCOUT_SIGN_GATE_PASS_NOT_ADMITTED"


def audit_owner(owner: Path = OWNER, route_path: Path = ROUTE, *,
                route_sha256: str = ROUTE_SHA256,
                research_split_intent: str = "town02_sign_train_scout_only",
                collector_scope: str = "--town02-sign-train-scout") -> dict:
    """HH_260906 - Bind the owner, route, source archive, world and lifecycle receipts."""
    require, digest, read_json = common.require, common.digest, common.read_json
    source = owner / "mapping"
    require(owner.is_dir() and source.is_dir() and not owner.is_symlink()
            and not source.is_symlink(), "missing or unsafe Town02 owned source")
    require(route_path.is_file() and not route_path.is_symlink()
            and digest(route_path) == route_sha256, "Town02 pinned route bytes differ")
    route = read_json(route_path)
    require(route.get("town") == "Town02_Opt" and route.get("scenario") == "straight"
            and route.get("weather") == "ClearNoon"
            and route.get("coordinate_reference") == "base_link"
            and len(route.get("route", [])) >= 2, "Town02 route identity differs")
    paths = {key: owner / relative for key, relative in (
        ("plan", "owner_plan.json"), ("started", "owner_started.json"),
        ("result", "owner_result.json"), ("summary", "mapping/summary.json"))}
    require(all(path.is_file() and not path.is_symlink() for path in paths.values()),
            "missing or unsafe owner receipt")
    plan, started, result, summary = (read_json(paths[key]) for key in
                                      ("plan", "started", "result", "summary"))
    require(Path(plan["route_path"]).resolve() == route_path.resolve()
            and plan["route_sha256"] == started["route_sha256"]
            == summary["route_sha256"] == route_sha256,
            "owner route SHA or path differs")
    require(plan["map"] == started["map"] == summary["town"] == "Town02_Opt"
            and plan["quality"] == started["quality"] == "Epic"
            and plan["capture_mode"] == result["capture_mode"] == "mapping-probe"
            and plan.get("source_bytes_archived") is True
            and plan.get("bounds_source_bytes_archived") is True
            and result.get("source_bytes_unchanged_and_archived") is True
            and result["exit_code"] == 0
            and summary["status"] == "COMPLETE_ORACLE_PLUMBING_ONLY"
            and summary["research_split_intent"] == research_split_intent
            and summary["frame_count"] == FRAME_COUNT
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
            "Town02 physical owner proof failed")
    argv = plan["collector_argv"]
    require(isinstance(argv, list) and len(argv) >= 2
            and Path(argv[0]).resolve() == source.resolve()
            and Path(argv[1]).resolve() == route_path.resolve()
            and common.option_value(argv, "--host") == plan["host"] == "127.0.0.1"
            and int(common.option_value(argv, "--port")) == plan["port"] == started["port"]
            and common.option_value(argv, "--frames") == str(FRAME_COUNT)
            and common.option_value(argv, "--motion-mode") == "basic-agent"
            and argv.count("--ignore-traffic-lights") == 1
            and argv.count(collector_scope) == 1
            and len([option for option in argv if option.startswith("--town")]) == 1,
            "Town02 collector scope differs")
    lifecycle_sha = {}
    generation = None
    for stage in ("ready", "after_capture", "stopped"):
        path = owner / "lifecycle" / f"{stage}.json"
        require(path.is_file() and not path.is_symlink(), f"missing lifecycle {stage}")
        event = read_json(path)
        require(event["stage"] == stage and event["status"] == "PASS"
                and event["expected_map"] == "Town02_Opt"
                and event["owner_pid"] == started["server_pid"]
                and event["owner_pgid"] == started["server_pgid"]
                and event["host"] == "127.0.0.1" and event["port"] == plan["port"]
                and (stage == "stopped" or event["active_map_basename"] == "Town02_Opt")
                and (stage != "stopped" or event.get("port_released") is True)
                and (generation is None or event["generation_id"] == generation),
                f"Town02 CARLA lifecycle failed at {stage}")
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
            "owner source checks incomplete")
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
        "route_sha256": route_sha256,
        "town": "Town02_Opt", "quality": "Epic", "exit_code": 0,
        "travel_m": summary["travel_m"],
        "agent_goal_reached": summary["agent_goal_reached"],
        "collision_events": 0, "live_replay_exact_match": True,
        "port_released_after_stop": True,
    }


def build_summary(owner: Path = OWNER, route_path: Path = ROUTE, *,
                  route_sha256: str = ROUTE_SHA256,
                  research_split_intent: str = "town02_sign_train_scout_only",
                  collector_scope: str = "--town02-sign-train-scout") -> dict:
    """HH_260906 - Decode all raw pairs and apply the preregistered whole-frame sign gate."""
    require = common.require
    provenance = audit_owner(owner, route_path, route_sha256=route_sha256,
                             research_split_intent=research_split_intent,
                             collector_scope=collector_scope)
    source = owner / "mapping"
    frame_dir = source / "frames"
    require(frame_dir.is_dir() and not frame_dir.is_symlink(), "frame source directory missing")
    require(len(list(frame_dir.glob("*.png"))) == FRAME_COUNT
            and len(list(frame_dir.glob("*.npz"))) == FRAME_COUNT,
            "240 native RGB/semantic pair files required")
    frames = common.audit_frames(source, expected_frames=FRAME_COUNT, frame_wh=FRAME_WH)
    records = common.read_json(source / "records.json")
    require(len(records) == FRAME_COUNT, "record count differs")
    for row in frames:
        with Image.open(source / (row["stem"] + ".png")) as rgb:
            decoded = np.asarray(rgb)
            require(decoded.shape == (360, 640, 3) and decoded.dtype == np.uint8,
                    f"native RGB full decode differs: {row['stem']}")
        path = source / (row["stem"] + ".npz")
        with np.load(path, allow_pickle=False) as sensor:
            require(set(sensor.files) == {"depth", "semantic", "world_from_camera"}
                    and sensor["depth"].shape == (360, 640, 4)
                    and sensor["depth"].dtype == np.uint8
                    and sensor["world_from_camera"].shape == (4, 4)
                    and np.isfinite(sensor["world_from_camera"]).all(),
                    f"native synchronized sensor payload differs: {row['stem']}")
    map_path = source / "map.json"
    require(map_path.is_file() and not map_path.is_symlink(), "saved replay map missing")
    saved_map = common.read_json(map_path)
    collection = common.read_json(source / "summary.json")
    require(saved_map.get("schema") == "portable_e2e.semantic_evidence.v1"
            and saved_map.get("frame_id") == "carla_ue:Town02_Opt"
            and saved_map.get("source") == "oracle_sensor"
            and saved_map.get("hd_map_approved") is False
            and len(saved_map.get("cells", [])) == collection["observed_cells"]
            and math.isclose(saved_map["timestamp_s"], records[-1]["timestamp_s"],
                             rel_tol=0.0, abs_tol=1e-6),
            "saved live/replay map evidence differs")
    cadence_errors = [abs((frames[index]["timestamp_s"]
                           - frames[index - 1]["timestamp_s"]) - 0.1)
                      for index in range(1, FRAME_COUNT)]
    require(all(math.isfinite(value) and value <= 0.001 for value in cadence_errors),
            "native 10 Hz cadence differs")
    kept, _ = common.retain_indices(frames, minimum_m=0.25)
    kept_set = set(kept)
    sign_index = common.LABELS.index("traffic_sign")
    sign_totals = [row["class_pixels"][sign_index] for row in frames]
    retained_totals = [sign_totals[index] for index in kept]
    raw_bins = frame_histogram(sign_totals)
    retained_bins = frame_histogram(retained_totals)
    retained_gate = len(kept) >= MINIMUM_RETAINED
    sign_gate = retained_bins["1-4"] >= MINIMUM_TINY_SIGN_FRAMES
    status = qualification_status(len(kept), retained_bins)
    require(sum(raw_bins.values()) == FRAME_COUNT
            and sum(retained_bins.values()) == len(kept), "sign histogram lost frames")
    return {
        "schema": "portable_e2e.map_town02_sign_train_scout_audit.v1",
        "status": status,
        "scope": "pinned Town02_Opt Epic raw scout; no model inference, training or dataset admission",
        "provenance": provenance,
        "saved_replay_map_sha256": common.digest(map_path),
        "native_frame_wh": list(FRAME_WH),
        "raw_frames": FRAME_COUNT,
        "retained_frames_0p25m": len(kept),
        "retention_rule": "keep frame iff >=0.25m from every earlier retained Town02_Opt ego XY",
        "minimum_retained": MINIMUM_RETAINED,
        "retained_gate_passed": retained_gate,
        "minimum_retained_1_to_4_sign_frames": MINIMUM_TINY_SIGN_FRAMES,
        "small_sign_gate_passed": sign_gate,
        "sign_measure": "CARLA tag 8 pixels in the entire 640x360 semantic frame",
        "raw_sign_frame_bins": raw_bins,
        "retained_sign_frame_bins": retained_bins,
        "raw_tag8_pixel_sum": sum(sign_totals),
        "retained_tag8_pixel_sum": sum(retained_totals),
        "raw_1_to_4_stems": [frames[index]["stem"] for index, total
                             in enumerate(sign_totals) if 1 <= total <= 4],
        "retained_1_to_4_stems": [frames[index]["stem"] for index in kept
                                  if 1 <= sign_totals[index] <= 4],
        "max_cadence_error_s": max(cadence_errors, default=0.0),
        "retained_stems": [frames[index]["stem"] for index in kept],
        "frames": [{**row, "tag8_pixels": sign_totals[index],
                    "retained_0p25m": index in kept_set}
                   for index, row in enumerate(frames)],
        "dataset_admitted": False, "model_predictions_made": False,
        "model_trained": False, "vehicle_control_approved": False,
        "hd_map_approved": False,
    }


def preview(summary: dict, owner: Path = OWNER, *,
            chosen: tuple[str, str, str] = ("frames/0006", "frames/0190", "frames/0239"),
            title: str = "Town02_Opt Epic scout") -> Image.Image:
    """HH_260906 - Show native full-FOV RGB, oracle tags and tag-8 overlay only."""
    common.require(summary["native_frame_wh"] == [640, 360], "native full FOV required")
    kept = set(summary["retained_stems"])
    common.require(all(stem in kept for stem in chosen), "visual examples must be retained")
    width, height = FRAME_WH
    output = Image.new("RGB", (3 * width + 20, 72 + 3 * 398 + 40), (247, 249, 252))
    draw = ImageDraw.Draw(output)
    draw.text((15, 12), f"{title} | original RGB / CARLA oracle / tag-8 overlay",
              fill=(25, 37, 49), font=common.font(21))
    draw.text((15, 43), "All panels preserve the original 640x360 camera field of view.",
              fill=(65, 76, 88), font=common.font(16))
    for index, stem in enumerate(chosen):
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
        count = int(np.count_nonzero(tags == 8))
        draw.text((15, y), f"{stem} | whole-frame tag 8 = {count} px | retained 0.25m",
                  fill=(25, 37, 49), font=common.font(17))
        output.paste(rgb, (0, y + 26))
        output.paste(Image.fromarray(palette), (width, y + 26))
        output.paste(Image.fromarray(overlay), (2 * width, y + 26))
    draw.text((15, output.height - 34),
              "Magenta overlay marks CARLA tag 8 exactly; it is not a model prediction.",
              fill=(65, 76, 88), font=common.font(16))
    return output


def report_bytes(summary: dict) -> bytes:
    return (json.dumps(summary, indent=2, ensure_ascii=False) + "\n").encode("utf-8")


def picture_bytes(picture: Image.Image) -> bytes:
    buffer = io.BytesIO()
    picture.save(buffer, format="PNG")
    return buffer.getvalue()


def write_or_check(summary: dict, picture: Image.Image, output: Path = OUTPUT,
                   *, write: bool = False, source_owner: Path = OWNER,
                   report_name: str = REPORT_NAME, picture_name: str = PICTURE_NAME) -> None:
    """HH_260906 - Write fresh evidence once or independently recheck JSON and PNG pixels."""
    common.require(output.is_dir() and not output.is_symlink(), "audit output folder missing")
    common.require(not output.resolve().is_relative_to(source_owner.resolve()),
                   "audit output overlaps immutable source")
    report_path, picture_path = output / report_name, output / picture_name
    if write:
        common.require(not report_path.exists() and not picture_path.exists(),
                       "audit artifact already exists")
        report_path.write_bytes(report_bytes(summary))
        picture_path.write_bytes(picture_bytes(picture))
        return
    common.require(report_path.is_file() and not report_path.is_symlink()
                   and report_path.read_bytes() == report_bytes(summary),
                   "published audit JSON differs")
    common.require(picture_path.is_file() and not picture_path.is_symlink(),
                   "published audit PNG missing")
    with Image.open(picture_path) as published:
        common.require(published.mode == "RGB" and published.size == picture.size
                       and np.array_equal(np.asarray(published), np.asarray(picture)),
                       "published audit PNG pixels differ")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true", help="write fresh report and preview once")
    args = parser.parse_args()
    summary = build_summary()
    write_or_check(summary, preview(summary), write=args.write)
    print(json.dumps({"status": summary["status"],
                      "retained_frames_0p25m": summary["retained_frames_0p25m"],
                      "retained_sign_frame_bins": summary["retained_sign_frame_bins"]},
                     ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
