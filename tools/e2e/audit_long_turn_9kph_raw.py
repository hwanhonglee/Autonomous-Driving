#!/usr/bin/env python3
"""Read-only, fixed-scope raw QA for the one Town01 s0002_p06 9 km/h turn.

Run through tools/workspace_python.py. This audit never creates training samples,
changes source evidence, or treats the straight moving-task/stop auditors as judges.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re

from PIL import Image, ImageDraw

from scripts.e2e import audit_carla_acknowledged_control as ack
from scripts.e2e import audit_carla_raw_pre_admission as raw
from scripts.e2e import audit_portable_target_feasibility as targets


ROOT = Path(__file__).resolve().parents[2]
TRIAL = ROOT / "storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_trial_v1"
PLAN = ROOT / "docs/assets/validation/2026-09-27/drive_long_route_p06_capture/PLAN.md"
OUTPUT = PLAN.parent / "QA_v3"
ROUTE = ROOT / "storage/experiments/training/2026-09-27/drive_long_route_town01_train_catalog_v4/routes/town01/right/town01_right_s0002_p06.json"
MAPPING = ROOT / "autonomy/autoware_e2e_vad_launch/config/sensor_mapping_vad_fast_reliable.yaml"
CALIBRATION = ROOT / "autonomy/src/launcher/autoware_launch/sensor_kit/carla_sensor_kit_launch/carla_sensor_kit_description/config/sensor_kit_calibration.yaml"
PINS = {
    "preregistered_plan": (PLAN, "03482e12a8fea09c416b086ac3efbb18f353a187d1c391096a247d1c1166547a"),
    "route": (ROUTE, "49d0c147e83ced265b9e954aacb7bad340970b87a42a060cbfcf1f7625387142"),
    "owner_wrapper": (ROOT / "tools/e2e/run_owned_carla_expert_trial.sh", "0723336ac4b7649004e5cc2f4bfce95e7c0f12caf941a03839096d04cd9f507b"),
    "collector": (ROOT / "tools/e2e/collect_carla_vad_expert.py", "d22af16f051ee7835f41363d85934a3060ed9bd6dab61b727a8458ed7c30a08a"),
    "mapping": (MAPPING, "9aaff2befed7ad12376b2e04bbdd51bd1808a3bafe39d87a6f6b241dbcca3136"),
    "calibration": (CALIBRATION, "5022cd1de5b48e9c824b6f2f8c59991fa665eeaf7a7bafd084c88acdb65e4bea"),
    "common10_contract": (ROOT / "learning/portable_e2e/config/common_10hz_v1.contract.json", "6f1a7a82abe39a0cc16b42df2b191d2cdae735243a22e8f8cb47bf11faaa4642"),
}
REQUIRED_OPTIONS = {
    "--target-speed-kmh": "9", "--max-duration-sec": "180", "--physics-hz": "20",
    "--capture-hz": "10", "--stationary-warmup-sec": "3.5", "--stationary-tail-sec": "6.5",
    "--control-transport": "acknowledged_batch", "--agent-initialization": "after_bootstrap",
    "--weather": "ClearNoon", "--seed": "2026092702", "--vehicle-type": "vehicle.toyota.prius",
    "--spawn-z-offset-m": "2", "--wheelbase-m": "2.85", "--goal-stop-profile": "disabled",
}
DT_NATIVE_NS = 50_000_000
DT_CAMERA_NS = 100_000_000
FUTURE_NS = 6_400_000_000
SPEED_MIN_MPS = 0.5
MONOTONE_TOLERANCE_M = 1e-6


class AuditError(ValueError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AuditError(message)


def sha(path: Path) -> str:
    require(path.is_file() and not path.is_symlink(), f"regular file required: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite(value: object, label: str) -> float:
    require(type(value) in (int, float) and math.isfinite(value), f"finite {label} required")
    return float(value)


def timestamp_ns(row: dict) -> int:
    return raw.adapter._timestamp_ns(row["timestamp"], "Town01 native timestamp")


def project(route_points: list[dict], row: dict) -> tuple[float, float, int]:
    """Nearest XY segment against the original route arc, independent of saved progress."""
    x, y = finite(row.get("x"), "base_link x"), finite(row.get("y"), "base_link y")
    best = None
    for index, (left, right) in enumerate(zip(route_points, route_points[1:])):
        x0, y0 = finite(left.get("x"), "route x"), finite(left.get("y"), "route y")
        dx, dy = finite(right.get("x"), "route x") - x0, finite(right.get("y"), "route y") - y0
        squared = dx * dx + dy * dy
        if squared <= 1e-12:
            continue
        fraction = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / squared))
        error = math.hypot(x - x0 - fraction * dx, y - y0 - fraction * dy)
        arc = finite(left.get("distance_m"), "route arc") + fraction * (
            finite(right.get("distance_m"), "route arc") - finite(left.get("distance_m"), "route arc"))
        candidate = (error, -arc, index)
        if best is None or candidate < best:
            best = candidate
    require(best is not None, "route has no projectable segment")
    return -best[1], best[0], best[2]


def verified_turn_window(route: dict, route_source_bytes: bytes | None,
                         expected_route_sha256: str | None, window_report_bytes: bytes | None,
                         expected_window_report_sha256: str | None) -> dict:
    """HH_260906 - Bind a future turn-centered window to independently pinned source bytes."""
    supplied = (route_source_bytes, expected_route_sha256, window_report_bytes,
                expected_window_report_sha256)
    require(all(value is not None for value in supplied), "all four window source/pin inputs required")
    require(type(route_source_bytes) is bytes and type(window_report_bytes) is bytes,
            "raw route and static window report must be byte strings")
    require(all(type(pin) is str and re.fullmatch(r"[0-9a-f]{64}", pin) for pin in
                (expected_route_sha256, expected_window_report_sha256)),
            "independent lowercase SHA256 pins required")
    route_digest = hashlib.sha256(route_source_bytes).hexdigest()
    report_digest = hashlib.sha256(window_report_bytes).hexdigest()
    require(route_digest == expected_route_sha256 and report_digest == expected_window_report_sha256,
            "route or window report differs from independent SHA256 pin")
    try:
        serialized_route = json.loads(route_source_bytes)
        report = json.loads(window_report_bytes)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise AuditError(f"pinned window input is not JSON: {error}") from error
    require(type(serialized_route) is dict and serialized_route == route,
            "in-memory route differs from pinned raw route bytes")
    require(route.get("scenario") == "right" and route.get("coordinate_reference") == "base_link",
            "pinned turn window requires a right/base_link route")
    require(type(report) is dict and report.get("schema") == "portable_e2e.turn_centered_subwindow.v1" and
            report.get("status") == "STATIC_WINDOW_CANDIDATE" and report.get("candidate_count") == 1 and
            report.get("route_sha256") == route_digest and type(report.get("selected")) is dict,
            "pinned static selector did not approve exactly one window for this route")
    policy = report.get("window_policy", {})
    require(type(policy) is dict and policy.get("approach_before_right_command_m") == 30.0 and
            policy.get("moving_anchor_window_m") == 90.0 and
            policy.get("post_right_command_inside_window_m") == 30.0 and
            policy.get("route_lead_before_window_m") == 30.0 and
            policy.get("9kph_6p4s_future_distance_m") == 16.0 and
            policy.get("future_clearance_guard_m") == 10.0 and
            policy.get("previous_maneuver_guard_m") == 15.0,
            "pinned static selector policy differs from fixed 9 km/h turn contract")
    selected = report["selected"]
    start = finite(selected.get("window_start_m"), "window start")
    end = finite(selected.get("window_end_m"), "window end")
    require(start >= 30.0 and abs(end - start - 90.0) <= 1e-6,
            "fixed 90 m window with 30 m route lead required")
    points = route.get("route")
    require(type(points) is list and len(points) > 2, "route polyline missing")
    require(abs(finite(points[0].get("distance_m"), "first route arc")) <= 1e-6,
            "pinned route progress must begin at zero")
    first, last = selected.get("right_block_start_index"), selected.get("right_block_end_index")
    require(type(first) is int and type(last) is int and 0 < first <= last < len(points) - 1,
            "complete indexed right-turn block required")
    require(all(point.get("road_option") == "RIGHT" for point in points[first:last + 1]) and
            points[first - 1].get("road_option") != "RIGHT" and
            points[last + 1].get("road_option") != "RIGHT",
            "selected indices do not cover the complete right-turn command block")
    boundary_start = 0.5 * (finite(points[first - 1].get("distance_m"), "pre-turn arc") +
                            finite(points[first].get("distance_m"), "first turn arc"))
    boundary_end = 0.5 * (finite(points[last].get("distance_m"), "last turn arc") +
                          finite(points[last + 1].get("distance_m"), "post-turn arc"))
    require(abs(finite(selected.get("right_block_start_m"), "turn start boundary") - boundary_start) <= 1e-6 and
            abs(finite(selected.get("right_block_end_m"), "turn end boundary") - boundary_end) <= 1e-6 and
            abs(start + 30.0 - boundary_start) <= 1e-6 and boundary_end <= end - 30.0 + 1e-6,
            "window does not preserve the fixed 30 m approach/post-turn context")
    route_length = finite(route.get("route_length_m"), "route length")
    require(abs(route_length - finite(points[-1].get("distance_m"), "last route arc")) <= 1e-6 and
            abs(route_length - finite(selected.get("route_length_m"), "selected route length")) <= 1e-6 and
            end + 16.0 + 10.0 <= route_length + 1e-6,
            "window lacks the complete 9 km/h future distance and 10 m route guard")
    # HH_260906 - Independently recheck that no other maneuver enters the protected window/future.
    maneuvers = {"LEFT", "RIGHT", "STRAIGHT", "CHANGELANELEFT", "CHANGELANERIGHT"}
    blocks = []
    for index, point in enumerate(points):
        option = point.get("road_option")
        require(option in maneuvers | {"LANEFOLLOW"}, "unknown road option in pinned route")
        if option not in maneuvers:
            continue
        if blocks and blocks[-1]["option"] == option and blocks[-1]["last"] == index - 1:
            blocks[-1]["last"] = index
        else:
            blocks.append({"option": option, "first": index, "last": index})
    require(not any(block["option"] in {"LEFT", "CHANGELANELEFT", "CHANGELANERIGHT"}
                    for block in blocks), "unsupported left turn or lane change on right-route search")
    for block in blocks:
        if block["first"] == first and block["last"] == last and block["option"] == "RIGHT":
            continue
        left_index, right_index = block["first"], block["last"]
        left_arc = finite(points[left_index].get("distance_m"), "other maneuver start arc")
        right_arc = finite(points[right_index].get("distance_m"), "other maneuver end arc")
        if left_index > 0:
            left_arc = 0.5 * (finite(points[left_index - 1].get("distance_m"), "pre-maneuver arc") + left_arc)
        if right_index + 1 < len(points):
            right_arc = 0.5 * (right_arc + finite(points[right_index + 1].get("distance_m"), "post-maneuver arc"))
        require(right_arc < start - 15.0 - 1e-9 or left_arc > end + 26.0 + 1e-9,
                "another maneuver intersects protected turn window/future")
    return {"start_m": start, "end_m": end, "turn_start_index": first, "turn_end_index": last,
            "route_sha256": route_digest, "window_report_sha256": report_digest}


def select_window(route: dict, states: list[dict], cameras: list[dict], *,
                  route_source_bytes: bytes | None = None, expected_route_sha256: str | None = None,
                  window_report_bytes: bytes | None = None,
                  expected_window_report_sha256: str | None = None) -> tuple[dict, list[dict], list[dict]]:
    """Select every eligible anchor once; first native progress beyond the end latches closure."""
    dynamic = any(value is not None for value in (route_source_bytes, expected_route_sha256,
                                                   window_report_bytes, expected_window_report_sha256))
    spec = verified_turn_window(route, route_source_bytes, expected_route_sha256,
                                window_report_bytes, expected_window_report_sha256) if dynamic else None
    start_m, end_m = (spec["start_m"], spec["end_m"]) if spec else (30.0, 120.0)
    points = route["route"]
    require(type(points) is list and len(points) > 2, "route polyline missing")
    previous_arc = -math.inf
    for point in points:
        arc = finite(point.get("distance_m"), "route arc")
        require(arc >= previous_arc, "route arc reversed")
        previous_arc = arc
    cameras_by_frame = {row["frame"]: row for row in cameras}
    require(len(cameras_by_frame) == len(cameras), "duplicate camera frame")
    projections, selected, closed = [], [], None
    driving_projection_drops = []
    previous_driving = None
    for state in states:
        frame = state["frame"]
        progress, error, segment = project(points, state)
        item = {"frame": frame, "timestamp_ns": timestamp_ns(state), "capture_phase": state["capture_phase"],
                "progress_m": progress, "cross_track_m": error, "segment_index": segment,
                "recorded_progress_m": finite(state.get("route_progress_m"), "saved route progress")}
        projections.append(item)
        if state["capture_phase"] == "driving":
            if previous_driving is not None and progress < previous_driving["progress_m"] - MONOTONE_TOLERANCE_M:
                driving_projection_drops.append({"from_frame": previous_driving["frame"], "to_frame": frame,
                    "drop_m": previous_driving["progress_m"] - progress})
            previous_driving = item
            if closed is None and progress > end_m:
                closed = {"frame": frame, "timestamp_ns": item["timestamp_ns"], "progress_m": progress}
        if frame in cameras_by_frame and state["capture_phase"] == "driving" and closed is None and start_m <= progress <= end_m:
            selected.append({**item, "speed_mps": math.hypot(finite(state.get("vx"), "vx"), finite(state.get("vy"), "vy")),
                             "command": state.get("command")})
    selected_frames = {row["frame"] for row in selected}
    selected_cameras = [camera for camera in cameras if camera["frame"] in selected_frames]
    require([r["frame"] for r in selected] == [r["frame"] for r in selected_cameras],
            "camera order differs from native selection")
    native_window = [row for row in projections if selected and selected[0]["frame"] <= row["frame"] <= selected[-1]["frame"]]
    states_by_frame = {row["frame"]: row for row in states}
    native_speeds = [math.hypot(finite(states_by_frame[row["frame"]].get("vx"), "vx"),
                                finite(states_by_frame[row["frame"]].get("vy"), "vy")) for row in native_window]
    def continuous(rows: list[dict], frame_step: int, time_step: int) -> bool:
        return bool(rows) and all(second["frame"] == first["frame"] + frame_step and
            abs(second["timestamp_ns"] - first["timestamp_ns"] - time_step) <= 500
            for first, second in zip(rows, rows[1:]))
    turn = route.get("physical_turn_preflight", {}).get("selected_block", {}) if spec is None else {}
    start_index, end_index = (turn.get("start_index"), turn.get("end_index")) if spec is None else (
        spec["turn_start_index"], spec["turn_end_index"])
    require(type(start_index) is int and type(end_index) is int and 0 <= start_index <= end_index < len(points),
            "fixed route turn block missing")
    require(all(point.get("road_option") == "RIGHT" for point in points[start_index:end_index + 1]),
            "turn block is not RIGHT")
    start_arc = finite(points[start_index]["distance_m"], "turn start arc")
    end_arc = finite(points[end_index]["distance_m"], "turn end arc")
    turn_states = [states_by_frame[row["frame"]] for row in native_window if start_arc <= row["progress_m"] <= end_arc]
    turn_commands = sum(row.get("command") == 1 for row in turn_states)
    if turn_states:
        yaw_change = math.atan2(math.sin(turn_states[-1]["yaw"] - turn_states[0]["yaw"]),
                                math.cos(turn_states[-1]["yaw"] - turn_states[0]["yaw"]))
    else:
        yaw_change = None
    selected_drops = [{"from_frame": left["frame"], "to_frame": right["frame"],
                       "drop_m": left["progress_m"] - right["progress_m"]}
                      for left, right in zip(native_window, native_window[1:])
                      if right["progress_m"] < left["progress_m"] - MONOTONE_TOLERANCE_M]
    span_ns = selected[-1]["timestamp_ns"] - selected[0]["timestamp_ns"] if selected else None
    flags = {
        "nonempty_fixed_selection": bool(selected),
        ("first_over_120_closes_selection" if spec is None else "first_over_window_end_closes_selection"):
            closed is not None and bool(selected) and selected[-1]["frame"] < closed["frame"],
        "selected_span_at_least_30s": span_ns is not None and span_ns >= 30_000_000_000,
        "selected_camera_10hz_continuous": continuous(selected, 2, DT_CAMERA_NS),
        "selected_native_20hz_continuous": continuous(native_window, 1, DT_NATIVE_NS),
        "selected_native_speed_all_over_0p5_mps": bool(native_speeds) and min(native_speeds) > SPEED_MIN_MPS,
        "selected_native_progress_monotone": bool(native_window) and not selected_drops,
        "selected_turn_entry_exit": bool(selected) and selected[0]["progress_m"] < start_arc and selected[-1]["progress_m"] > end_arc,
        "selected_right_command_present": turn_commands > 0,
        "observed_clockwise_turn_at_least_45deg": yaw_change is not None and yaw_change <= -math.pi / 4,
    }
    summary = {"flags": flags, "selected_anchor_count": len(selected), "selected_span_ns": span_ns,
        "first_selected": selected[0] if selected else None, "last_selected": selected[-1] if selected else None,
        ("first_progress_over_120_m" if spec is None else "first_progress_over_window_end_m"): closed,
        "minimum_selected_native_speed_mps": min(native_speeds) if native_speeds else None,
        "turn_start_arc_m": start_arc, "turn_end_arc_m": end_arc, "turn_native_state_count": len(turn_states),
        "turn_command_1_state_count": turn_commands, "turn_signed_yaw_change_rad": yaw_change,
        "selected_native_progress_drops": selected_drops, "full_driving_progress_drop_count": len(driving_projection_drops),
        "full_driving_progress_drop_examples": driving_projection_drops[:20],
        "selection_rule": ("all driving camera anchors at independently projected 30–120 m until first native >120 m; no post hoc exclusions"
                           if spec is None else
                           f"all driving camera anchors at independently projected {start_m:.6f}–{end_m:.6f} m "
                           f"until first native >{end_m:.6f} m; no post hoc exclusions")}
    if spec is not None:
        summary["pinned_window"] = spec
    return summary, selected_cameras, projections


def summarize_selected_futures(rows: list[dict], selected: list[dict], tail_start_ns: int) -> dict:
    require(len(rows) == len(selected), "future denominator differs from fixed selected cameras")
    results = [row["diagnostic"] for row in rows if "diagnostic" in row]
    metrics = targets.summarize_samples(results, 64)
    failures = {name: item["violating_anchor_count"] for name, item in metrics["metrics"].items()}
    require(set(failures) == set(targets.METRICS) and len(failures) == 9, "original nine bounds absent")
    incomplete = [row["frame"] for row in rows if row.get("valid_points") != 64 or "diagnostic" not in row]
    late = [row["frame"] for row in rows if row["anchor_timestamp_ns"] + FUTURE_NS >= tail_start_ns]
    tail_steps = [row["frame"] for row in rows if "diagnostic" in row and
                  any(step["phase"] != "pre_tail" for step in row["diagnostic"]["steps"])]
    return {"flags": {"all_selected_have_64_actual_future_points": bool(rows) and not incomplete,
            "all_selected_64_points_before_brake_tail": bool(rows) and not late and not tail_steps,
            "all_original_nine_motion_bounds_clear": bool(rows) and not any(failures.values())},
        "selected_anchor_count": len(rows), "incomplete_anchor_frames": incomplete,
        "future_reaches_brake_tail_frames": sorted(set(late + tail_steps)),
        "original_nine_violating_anchor_counts": failures, "original_nine_metrics": metrics}


def source_checks(ledger: dict) -> tuple[dict, dict]:
    preregistered = {}
    for name, (path, expected) in PINS.items():
        actual = sha(path)
        # Execution sources are authenticated from the owner's immutable archive below.
        # A later checkout edit cannot retroactively change the captured process.
        if name not in ("collector", "owner_wrapper"):
            require(actual == expected, f"preregistered {name} SHA256 changed")
        preregistered[name] = {"path": str(path.relative_to(ROOT)), "expected_capture_sha256": expected,
                                "current_checkout_sha256": actual,
                                "current_checkout_matches_capture": actual == expected}
    plan = raw.read_metadata(TRIAL, "owner_plan.json", ledger)
    owner = raw.read_metadata(TRIAL, "owner_result.json", ledger)
    require(owner.get("exit_code") == 0 and owner.get("capture_mode") == "expert" and
            owner.get("source_bytes_unchanged_and_archived") is True and
            owner.get("learned_model_control") is False and owner.get("vehicle_control_approved") is False,
            "owner result does not prove completed expert-only capture")
    require(plan.get("route_sha256") == PINS["route"][1] and plan.get("map") == "Town01" and
            plan.get("quality") == "Epic" and plan.get("port") == 2100 and
            plan.get("collector_wall_timeout_sec") == 300 and plan.get("capture_mode") == "expert" and
            plan.get("source_bytes_archived") is True and plan.get("bounds_source_bytes_archived") is True,
            "owner plan differs from preregistered scope")
    argv = plan.get("collector_argv")
    require(type(argv) is list and len(argv) >= 2 and argv[0] == str(TRIAL / "episode") and
            argv[1] == str(ROUTE) and "--moving-task-plan" not in argv and
            "--moving-task-plan-sha256" not in argv, "collector command differs from one original capture")
    for option, expected in REQUIRED_OPTIONS.items():
        require(argv.count(option) == 1 and argv[argv.index(option) + 1] == expected,
                f"collector option differs: {option}")
    require(argv.count("--mapping") == argv.count("--calibration") == 1 and
            argv[argv.index("--mapping") + 1] == str(MAPPING.relative_to(ROOT)) and
            argv[argv.index("--calibration") + 1] == str(CALIBRATION.relative_to(ROOT)),
            "sensor source arguments differ")
    source_pins = plan.get("source_sha256")
    require(type(source_pins) is dict and set(source_pins) == set(owner.get("source_checks", {})) and
            all(owner["source_checks"].values()), "owner archived-source after-check incomplete")
    require(source_pins["scripts/e2e/run_owned_carla_expert_trial.sh"] == PINS["owner_wrapper"][1] and
            source_pins["scripts/e2e/collect_carla_vad_expert.py"] == PINS["collector"][1],
            "preregistered worker source differs from owner plan")
    for relative, expected in source_pins.items():
        data = raw.control_audit.checked_bytes(TRIAL, "provenance/" + relative, ledger)
        require(hashlib.sha256(data).hexdigest() == expected, f"archived source mismatch: {relative}")
    for stage in ("ready", "after_capture", "stopped"):
        receipt = raw.read_metadata(TRIAL, f"lifecycle/{stage}.json", ledger)
        require(receipt.get("stage") == stage and receipt.get("status") == "PASS" and
                receipt.get("expected_map") == "Town01" and
                receipt.get("mode") == ("stopped" if stage == "stopped" else "running"),
                f"owned {stage} lifecycle failed")
        if stage == "stopped":
            require(receipt.get("port_released") is True, "owned server port was not released")
    return plan, {"owner": owner, "preregistered": preregistered, "source_archive_sha256": source_pins}


def audit() -> tuple[dict, dict]:
    ledger = {}
    plan, sources = source_checks(ledger)
    require((TRIAL / "episode").is_dir() and not (TRIAL / "episode.partial").exists(),
            "one successful final episode required")
    manifest = raw.read_metadata(TRIAL, "episode/manifest.json", ledger)
    route = raw.read_metadata(TRIAL, "episode/route.json", ledger)
    states = raw.read_metadata(TRIAL, "episode/states.jsonl", ledger, lines=True)
    cameras = raw.read_metadata(TRIAL, "episode/camera_frames.jsonl", ledger, lines=True)
    receipts = raw.read_metadata(TRIAL, "episode/control_receipts.jsonl", ledger, lines=True)
    require(ledger["episode/route.json"]["sha256"] == PINS["route"][1] and
            manifest.get("status") == "complete" and manifest.get("cleanup", {}).get("completed") is True,
            "final native route/manifest incomplete")
    capture = manifest["capture_contract"]
    require(capture.get("physics_hz") == 20 and capture.get("camera_hz") == 10 and
            capture.get("target_speed_kmh") == 9 and capture.get("maximum_total_duration_sec") == 180 and
            capture.get("seed") == 2026092702 and capture.get("spawn_z_offset_m") == 2 and
            capture.get("control_transport", {}).get("mode") == "acknowledged_batch" and
            capture.get("agent_initialization", {}).get("mode") == "after_bootstrap" and
            manifest["coordinate_contract"].get("wheelbase_m") == 2.85,
            "native capture contract differs from owner command")
    provenance = manifest["provenance"]
    for name in ("route", "mapping", "calibration", "collector"):
        require(provenance[name + "_sha256"] == PINS[name][1], f"native {name} provenance differs")
    timeline = raw.measured_timeline(states)
    require(len(states) == manifest["result"]["state_count"] and
            len(cameras) == manifest["result"]["camera_anchor_count"], "native counts differ from manifest")
    transport = ack.analyze_transport(states, receipts, capture["control_transport"]["bootstrap_observation"])
    camera_ack = ack.analyze_camera_alignment(states, cameras)
    selection, selected_cameras, projections = select_window(route, states, cameras)
    pixels, image_rows, camera_rows = raw.camera_audit(TRIAL / "episode", manifest, cameras, timeline)
    selected_frames = {camera["frame"] for camera in selected_cameras}
    selected_camera_rows = [row for row in camera_rows if row["frame"] in selected_frames]
    _, future_rows = raw.measured_future_audit(timeline, selected_cameras, route, selected_camera_rows)
    tail_start_ns = next((timestamp_ns(row) for row in states if row["capture_phase"] == "stationary_tail"), None)
    require(tail_start_ns is not None, "brake tail is absent")
    future = summarize_selected_futures(future_rows, selected_cameras, tail_start_ns)
    events = {name: sum(len(row[name]) for row in states) for name in ("collision", "lane_invasion")}
    selected_event_frames = {name: [row["frame"] for row in states if row["frame"] in selected_frames and row[name]]
                             for name in events}
    checks = {**selection["flags"], **future["flags"],
        "six_rgb_pixels_and_camera_state_aligned": pixels["all_pixel_and_metadata_checks_clear"] and camera_ack["status"] == "PASS",
        "acknowledged_native_state_alignment": transport["status"] == "PASS",
        "collision_and_lane_sensor_clear": events == {"collision": 0, "lane_invasion": 0},
        "full_body_in_intended_lane": False,
    }
    unsupported = {"full_body_in_intended_lane":
        "Current raw state lacks a measured vehicle footprint and an offline CARLA OpenDRIVE intended-lane polygon bound to this exact capture. "
        "The older Prius measurement does not prove current actor-asset identity; Lanelet2 IDs cannot certify the route's OpenDRIVE lane identity. "
        "No CARLA map API/server was used, so full-body containment is unverified and fails closed."}
    for relative, pin in ledger.items():
        require(sha(TRIAL / relative) == pin["sha256"], f"raw metadata changed during audit: {relative}")
    for row in image_rows:
        require(sha(TRIAL / "episode" / row["path"]) == row["sha256"], "raw image changed during audit")
    helper_files = [Path(__file__), Path(raw.__file__), Path(ack.__file__), Path(targets.__file__)]
    helper_hashes = {str(path.relative_to(ROOT)): sha(path) for path in helper_files}
    report = {"schema": "carla.town01_long_right_9kph_raw_qa.v1", "status": "FAIL_CLOSED" if not all(checks.values()) else "PASS",
        "dataset_admitted": all(checks.values()), "training_started": False, "learned_control": False,
        "one_owned_attempt": str(TRIAL.relative_to(ROOT)), "preregistered_plan_sha256": PINS["preregistered_plan"][1],
        "checks": checks, "failed_checks": [name for name, passed in checks.items() if not passed],
        "unsupported_checks": unsupported, "window": selection, "future": future,
        "transport": transport, "camera_ack": camera_ack, "pixels": pixels,
        "events": events, "selected_event_frames": selected_event_frames,
        "native_result": {"goal_reached": manifest["result"].get("goal_reached"),
                          "termination_reason": manifest["result"].get("termination_reason"),
                          "capture_status": manifest.get("status"), "owner_exit_code": sources["owner"]["exit_code"]},
        "sources": {"preregistered": sources["preregistered"],
                    "owner_source_archive_sha256": sources["source_archive_sha256"],
                    "raw_metadata": ledger, "qa_implementation_sha256": helper_hashes},
        "limits": ["One BasicAgent expert capture, not learned driving or an Autoware GUI trial.",
                   "ACK proves recorded command acceptance/readback alignment, not physical actuation timing.",
                   "Nine original bounds are necessary discrete label screens, not sufficient route safety proof."]}
    companions = {"native_projection.jsonl": projections, "selected_future.jsonl": future_rows,
                  "jpeg_audit.jsonl": image_rows}
    return report, {"companions": companions, "selected_cameras": selected_cameras,
                    "implementation_hashes": helper_hashes}


def write_json(path: Path, value: object) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def write_jsonl(path: Path, rows: list[dict]) -> None:
    with path.open("x", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n")


def visual_sample(output: Path, selected_cameras: list[dict]) -> dict:
    """First/middle/last fixed selected anchors; all six full 640x360 camera pixels."""
    if not selected_cameras:
        return {"status": "NO_SELECTED_ANCHOR"}
    indices = sorted({0, (len(selected_cameras) - 1) // 2, len(selected_cameras) - 1})
    chosen = [selected_cameras[index] for index in indices]
    order = tuple(raw.contract.CAMERA_ORDER)
    canvas = Image.new("RGB", (640 * 3, (360 + 26) * len(chosen) * 2), "white")
    draw = ImageDraw.Draw(canvas)
    inputs = []
    for group, camera in enumerate(chosen):
        for number, name in enumerate(order):
            path = TRIAL / "episode" / camera["images"][name]
            with Image.open(path) as original:
                original.load()
                require(original.size == (640, 360) and original.mode == "RGB", "visual source pixels differ")
                image = original.copy()
            column, row = number % 3, group * 2 + number // 3
            x, y = 640 * column, (360 + 26) * row
            draw.text((x + 5, y + 5), f"frame {camera['frame']}  {name}", fill="black")
            canvas.paste(image, (x, y + 26))
            inputs.append({"frame": camera["frame"], "camera": name,
                           "path": str(path.relative_to(ROOT)), "sha256": sha(path)})
    path = output / "raw_six_camera_first_middle_last.png"
    canvas.save(path)
    return {"status": "PASS", "selection_indices": indices, "frames": [row["frame"] for row in chosen],
            "layout": "three columns × two rows per anchor; full native pixel frames without crop or resize",
            "path": path.name, "sha256": sha(path), "source_images": inputs}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args(argv)
    require(args.output.absolute() == OUTPUT and not OUTPUT.exists() and not OUTPUT.is_symlink(),
            "only fresh preregistered QA_v1 output outside raw capture is allowed")
    report = None
    companions = {}
    try:
        report, companions = audit()
    except Exception as error:
        report = {"schema": "carla.town01_long_right_9kph_raw_qa.v1", "status": "FAIL_CLOSED",
            "dataset_admitted": False, "training_started": False, "learned_control": False,
            "fatal_check": type(error).__name__, "fatal_detail": str(error),
            "one_owned_attempt": str(TRIAL.relative_to(ROOT)),
            "preregistered_plan_sha256": PINS["preregistered_plan"][1]}
    OUTPUT.mkdir(parents=True, exist_ok=False)
    if companions:
        for name, rows in companions["companions"].items():
            write_jsonl(OUTPUT / name, rows)
        # Pixels are sampled only after the full source and raw byte checks in audit().
        report["visual_sample"] = visual_sample(OUTPUT, companions["selected_cameras"])
    write_json(OUTPUT / "summary.json", report)
    with (OUTPUT / "QA.md").open("x", encoding="utf-8") as stream:
        stream.write("# Town01 s0002_p06 9 km/h raw QA\n\n")
        stream.write(f"Status: **{report['status']}**. Dataset admitted: **{str(report['dataset_admitted']).lower()}**. "
                     "Training started: **false**. Learned control: **false**.\n\n")
        stream.write("One preregistered BasicAgent expert capture. The selection contains every 30–120 m driving camera "
                     "anchor before the first native progress above 120 m.\n\n")
        for name in report.get("failed_checks", []):
            stream.write(f"- Failed: `{name}`\n")
        if "fatal_detail" in report:
            stream.write(f"- Fatal check: `{report['fatal_check']}` — {report['fatal_detail']}\n")
        for name, detail in report.get("unsupported_checks", {}).items():
            stream.write(f"- Unsupported: `{name}` — {detail}\n")
        stream.write("\nFull hashes, per-anchor futures, progress, and JPEG checks are in the companion files. "
                     "This is not an Autoware GUI or learned-driving result.\n")
    print(json.dumps({"status": report["status"], "dataset_admitted": report["dataset_admitted"],
                      "output": str(OUTPUT), "failed_checks": report.get("failed_checks", []),
                      "fatal_check": report.get("fatal_check")}, allow_nan=False))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
