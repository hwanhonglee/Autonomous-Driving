"""HH_260906 - Audit the pinned Town05_Opt Epic DEV source without model inference."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[2]
OWNER = ROOT / "storage/experiments/training/2026-09-27/map_town05_independent_dev_capture_epic_v2"
ROUTE = ROOT / ("storage/experiments/validation/2026-09-01/"
                "autoware_vad_town_matrix_30kph_v16_owned_window_final/maps/"
                "town05_opt/catalog/straight/routes/town05_opt/straight/"
                "town05_opt_straight_s0000_p00.json")
ROUTE_SHA256 = "efb8ccca533f25cae54b2eb7e03274c15795211488128185334ff2bc55f4744c"
OUTPUT = ROOT / "docs/assets/validation/2026-09-27/map_town05_dev_capture"
FRAMES = 240
FRAME_WH = (640, 360)


def _load_common():
    """HH_260906 - Reuse the verified Town03 pixel and position audit, not its TRAIN routing."""
    path = ROOT / "tools/e2e/audit_map_generalization_town03.py"
    spec = importlib.util.spec_from_file_location("_town03_map_audit_helpers", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Town03 map audit helpers unavailable")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


common = _load_common()


def audit_owner(owner: Path = OWNER, route_path: Path = ROUTE,
                expected_sha: str = ROUTE_SHA256, expected_frames: int = FRAMES) -> dict:
    """HH_260906 - Verify Epic owner, exact route, source archive and three lifecycle receipts."""
    require, digest, read_json = common.require, common.digest, common.read_json
    source = owner / "mapping"
    require(owner.is_dir() and not owner.is_symlink() and source.is_dir() and not source.is_symlink(),
            "missing or unsafe owned capture")
    require(route_path.is_file() and not route_path.is_symlink() and digest(route_path) == expected_sha,
            "pinned route bytes differ")
    route = read_json(route_path)
    require(route.get("town") == "Town05_Opt" and route.get("scenario") == "straight"
            and route.get("coordinate_reference") == "base_link"
            and len(route.get("route", [])) >= 2, "route schema or town differs")
    paths = {key: owner / relative for key, relative in (
        ("plan", "owner_plan.json"), ("started", "owner_started.json"),
        ("result", "owner_result.json"), ("summary", "mapping/summary.json"))}
    evidence = {key: read_json(path) for key, path in paths.items()}
    plan, started, result, summary = (evidence[key] for key in ("plan", "started", "result", "summary"))
    require(Path(plan["route_path"]).resolve() == route_path.resolve()
            and plan["route_sha256"] == started["route_sha256"] == summary["route_sha256"] == expected_sha,
            "owner route identity differs")
    require(plan["map"] == started["map"] == summary["town"] == "Town05_Opt"
            and plan["quality"] == started["quality"] == "Epic"
            and plan["capture_mode"] == result["capture_mode"] == "mapping-probe"
            and plan.get("source_bytes_archived") is True
            and result.get("source_bytes_unchanged_and_archived") is True
            and result.get("exit_code") == 0
            and summary["status"] == "COMPLETE_ORACLE_PLUMBING_ONLY"
            and summary.get("research_split_intent") == "town05_independent_dev_candidate"
            and summary["frame_count"] == expected_frames
            and summary["physics_driving"] is True and summary["motion_mode"] == "basic-agent"
            and plan.get("learned_model_control") is False
            and started.get("learned_model_control") is False
            and result.get("learned_model_control") is False
            and summary.get("learned_model_control") is False
            and summary["collision_events"] == 0
            and summary["live_replay_exact_match"] is True
            and not summary["cleanup_errors"]
            and summary["ignores_traffic_lights"] is True
            and summary["oracle_pose"] is True and summary["oracle_semantics"] is True
            and summary.get("hd_map_approved") is False
            and {int(tag): label for tag, label in summary["label_table"].items()} == common.TAG_TO_LABEL,
            "physical Epic DEV owner proof failed")
    argv = plan["collector_argv"]
    require(isinstance(argv, list) and len(argv) >= 2
            and Path(argv[0]).resolve() == source.resolve()
            and Path(argv[1]).resolve() == route_path.resolve()
            and common.option_value(argv, "--host") == plan["host"] == "127.0.0.1"
            and int(common.option_value(argv, "--port")) == plan["port"] == started["port"]
            and common.option_value(argv, "--frames") == str(expected_frames)
            and common.option_value(argv, "--motion-mode") == "basic-agent"
            and argv.count("--ignore-traffic-lights") == 1
            and argv.count("--town05-generalization-dev") == 1
            and len([option for option in argv if option.startswith("--town")]) == 1,
            "collector DEV scope differs")
    lifecycle_sha = {}
    for stage in ("ready", "after_capture", "stopped"):
        path = owner / "lifecycle" / f"{stage}.json"
        event = read_json(path)
        require(event["stage"] == stage and event["status"] == "PASS"
                and event["expected_map"] == "Town05_Opt"
                and event["owner_pid"] == started["server_pid"]
                and event["owner_pgid"] == started["server_pgid"]
                and event["host"] == "127.0.0.1" and event["port"] == plan["port"]
                and (stage == "stopped" or event.get("active_map_basename") == "Town05_Opt")
                and (stage != "stopped" or event.get("port_released") is True),
                f"CARLA lifecycle failed at {stage}")
        lifecycle_sha[stage] = digest(path)
    source_hashes = plan["source_sha256"]
    source_checks = result["source_checks"]
    require(isinstance(source_hashes, dict) and bool(source_hashes)
            and set(source_checks) == set(source_hashes)
            and all(source_checks.values())
            and all(result.get("external_source_checks", {}).values()),
            "source archive check incomplete")
    for relative, expected in source_hashes.items():
        relative_path = Path(relative)
        archive = owner / "provenance" / relative_path
        require(not relative_path.is_absolute() and ".." not in relative_path.parts
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
        "quality": "Epic", "town": "Town05_Opt", "exit_code": 0,
        "collision_events": 0, "live_replay_exact_match": True,
        "port_released_after_stop": True,
        "travel_m": summary["travel_m"],
        "agent_goal_reached": summary["agent_goal_reached"],
    }


def build_summary(owner: Path = OWNER, route_path: Path = ROUTE,
                  expected_sha: str = ROUTE_SHA256, expected_frames: int = FRAMES,
                  frame_wh: tuple[int, int] = FRAME_WH,
                  minimum_retained: int = 120) -> dict:
    """HH_260906 - Decode all native pairs and count seven oracle classes on retained frames."""
    require = common.require
    provenance = audit_owner(owner, route_path, expected_sha, expected_frames)
    source = owner / "mapping"
    frame_dir = source / "frames"
    require(frame_dir.is_dir() and not frame_dir.is_symlink(), "native frame directory missing")
    require(len(list(frame_dir.glob("*.png"))) == expected_frames
            and len(list(frame_dir.glob("*.npz"))) == expected_frames,
            "native pair file count differs")
    frames = common.audit_frames(source, expected_frames=expected_frames, frame_wh=frame_wh)
    cadence_error = [abs((frames[index]["timestamp_s"] - frames[index - 1]["timestamp_s"]) - .1)
                     for index in range(1, len(frames))]
    require(all(math.isfinite(value) for value in cadence_error)
            and max(cadence_error, default=0.0) <= 0.001,
            "native 10 Hz source cadence differs")
    kept, _ = common.retain_indices(frames)
    retained = [frames[index] for index in kept]
    passed = len(retained) >= minimum_retained
    sign_index = common.LABELS.index("traffic_sign")
    return {
        "schema": "portable_e2e.map_town05_dev_capture_audit.v1",
        "status": "CAPTURE_AUDIT_PASS_NOT_ADMITTED" if passed else "REJECTED_INSUFFICIENT_DISTINCT_POSITIONS",
        "scope": "pinned Town05_Opt Epic mapping source; no model prediction or training",
        "provenance": provenance,
        "native_frame_wh": list(frame_wh),
        "labels": list(common.LABELS),
        "raw_frames": len(frames),
        "retained_frames_0p25m": len(retained),
        "retention_rule": "first frame then >=0.25m from every retained Town05_Opt ego XY",
        "minimum_retained": minimum_retained,
        "retained_gate_passed": passed,
        "raw_class_pixels": common.add_pixels(frames),
        "retained_class_pixels": common.add_pixels(retained),
        "raw_sign_positive_frames": sum(row["class_pixels"][sign_index] > 0 for row in frames),
        "retained_sign_positive_frames": sum(row["class_pixels"][sign_index] > 0 for row in retained),
        "max_cadence_error_s": max(cadence_error, default=0.0),
        "retained_stems": [row["stem"] for row in retained],
        "frames": frames,
        "dataset_admitted": False, "model_predictions_made": False,
        "model_trained": False, "hd_map_approved": False,
    }


def preview(summary: dict, owner: Path = OWNER) -> Image.Image:
    """HH_260906 - Show only source RGB and oracle semantic masks at full camera FOV."""
    common.require(summary["native_frame_wh"] == [640, 360], "preview requires native full FOV")
    kept = summary["retained_stems"]
    common.require(bool(kept), "no retained preview frames")
    picked = [kept[index] for index in (0, len(kept) // 2, len(kept) - 1)]
    output = Image.new("RGB", (1300, 55 + 3 * 390 + 48), (247, 249, 252))
    draw = ImageDraw.Draw(output)
    draw.text((15, 12), "Town05_Opt Epic DEV source | RGB vs CARLA oracle labels",
              fill=(25, 37, 49), font=common.font(22))
    for index, stem in enumerate(picked):
        with Image.open(owner / "mapping" / (stem + ".png")) as raw:
            rgb = raw.convert("RGB")
        with np.load(owner / "mapping" / (stem + ".npz"), allow_pickle=False) as sensor:
            tags = sensor["semantic"][..., 2]
            canvas = np.empty((360, 640, 3), dtype=np.uint8)
            canvas[:] = common.COLORS[0]
            for tag, label in common.TAG_TO_LABEL.items():
                canvas[tags == tag] = common.COLORS[common.LABELS.index(label)]
        y = 55 + index * 390
        draw.text((15, y), f"{stem} | original 640x360; no model prediction",
                  fill=(25, 37, 49), font=common.font(16))
        output.paste(rgb, (10, y + 26))
        output.paste(Image.fromarray(canvas), (650, y + 26))
    draw.text((15, output.height - 36),
              "BasicAgent source; oracle is audit truth only. No learned control or HD-map approval.",
              fill=(66, 78, 90), font=common.font(16))
    return output


def write_artifacts(summary: dict, picture: Image.Image, output: Path = OUTPUT,
                    owner: Path = OWNER) -> None:
    """HH_260906 - Write only fresh audit artifacts, never source or any existing plan."""
    common.require(output.is_dir() and not output.is_symlink(), "audit report directory missing/unsafe")
    common.require(not output.resolve().is_relative_to(owner.resolve()), "output overlaps source")
    json_path = output / "epic_v2_audit.json"
    png_path = output / "epic_v2_rgb_oracle_full_fov.png"
    common.require(not json_path.exists() and not png_path.exists(), "audit artifact already exists")
    picture.save(png_path)
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    summary = build_summary()
    if not summary["retained_gate_passed"]:
        raise SystemExit("Town05_Opt Epic v2 rejected: fewer than 120 retained source frames")
    write_artifacts(summary, preview(summary), args.output)
    print(json.dumps({"status": summary["status"],
                      "retained_frames_0p25m": summary["retained_frames_0p25m"],
                      "output": str(args.output)}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
