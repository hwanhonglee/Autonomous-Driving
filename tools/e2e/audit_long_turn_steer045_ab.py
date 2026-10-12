#!/usr/bin/env python3
"""Read-only, fail-closed one-attempt Town01 BasicAgent steering-cap A/B raw audit.

Run with tools/workspace_python.py after B owner_result and lifecycle completion.
This compares expert captures; it never admits data or starts learned control.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from scripts.e2e import audit_long_turn_9kph_raw as prior


ROOT = Path(__file__).resolve().parents[2]
TRIAL_B = ROOT / "storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_steer045_trial_v1"
TRIAL_A = prior.TRIAL
PLAN = ROOT / "docs/assets/validation/2026-09-27/drive_steering_cap_ab/PLAN.md"
OUTPUT = PLAN.parent / "QA_v1"
REPORT_A = ROOT / "docs/assets/validation/2026-09-27/drive_long_route_p06_capture/QA_v3/summary.json"
PLAN_SHA = "63ec412458aa6bf3919c43ee47ab2892e9239b52c36a81636dc6dbb0bc63e077"
REPORT_A_SHA = "fb38f38d3a188fa88a18e70072c91786acea9459a6e7bd1ac086945228d5ce0c"
ROUTE_SHA = prior.PINS["route"][1]
COLLECTOR_B_SHA = "5dd1654afe2c8357db1c1b44c81bef180ce675c727d4bc021483e73ce07ba24e"
WRAPPER_SHA = prior.PINS["owner_wrapper"][1]
CAMERA_ORDER = tuple(prior.raw.contract.CAMERA_ORDER)


def require(condition: bool, message: str) -> None:
    prior.require(condition, message)


def sha(path: Path) -> str:
    return prior.sha(path)


def one_factor_arguments(argv_a: list[str], argv_b: list[str]) -> dict:
    """Only the output directory and the explicit normalized steering cap may differ."""
    require(type(argv_a) is list and type(argv_b) is list and len(argv_a) > 2 and len(argv_b) > 2,
            "complete A/B collector argv required")
    require(argv_a[0] == str(TRIAL_A / "episode") and argv_b[0] == str(TRIAL_B / "episode") and
            argv_a[1] == argv_b[1] == str(prior.ROUTE), "A/B episode or route argument differs")
    option = "--basic-agent-max-steering"
    require(argv_a.count(option) == 0 and argv_b.count(option) == 1 and
            argv_b[argv_b.index(option) + 1] == "0.45", "only B may opt in to max-steering 0.45")
    reduced_b = argv_b[:]
    location = reduced_b.index(option)
    del reduced_b[location:location + 2]
    require(argv_a[1:] == reduced_b[1:], "A/B collector command has another changed option")
    return {"a_effective_normalized_max_steering": 0.8, "b_explicit_normalized_max_steering": 0.45,
            "all_other_collector_arguments_equal": True}


def one_factor_agent(a: dict, b: dict) -> dict:
    """Check effective BasicAgent options, not only a printed CLI switch."""
    require(a.get("normalized_max_steering") == 0.8 and b.get("normalized_max_steering") == 0.45 and
            a.get("effective_opt_dict", {}).get("max_steering") == 0.8 and
            b.get("effective_opt_dict", {}).get("max_steering") == 0.45 and
            a.get("collector_cli", {}).get("--basic-agent-max-steering") == 0.8 and
            b.get("collector_cli", {}).get("--basic-agent-max-steering") == 0.45,
            "effective normalized steering cap differs from preregistration")
    stripped_a, stripped_b = copy.deepcopy(a), copy.deepcopy(b)
    for item in (stripped_a, stripped_b):
        item.pop("normalized_max_steering")
        item["effective_opt_dict"].pop("max_steering")
        item["collector_cli"].pop("--basic-agent-max-steering")
    require(stripped_a == stripped_b, "another BasicAgent configuration factor changed")
    return {"a_normalized_cap": 0.8, "b_normalized_cap": 0.45,
            "all_other_effective_agent_options_equal": True,
            "measured_tire_angle_is_not_normalized_command": True}


def closed_owner(ledger: dict) -> tuple[dict, dict, dict]:
    """Gate access to any B episode bytes on a completed owner and lifecycle."""
    require(sha(PLAN) == PLAN_SHA, "preregistered B plan changed")
    require(sha(prior.ROUTE) == ROUTE_SHA, "preregistered route changed")
    require(REPORT_A.is_file() and sha(REPORT_A) == REPORT_A_SHA, "sealed A QA summary changed")
    require((TRIAL_B / "owner_result.json").is_file(), "B owner_result is not final")
    owner = prior.raw.read_metadata(TRIAL_B, "owner_result.json", ledger)
    lifecycle = {}
    for stage in ("ready", "after_capture", "stopped"):
        path = TRIAL_B / "lifecycle" / (stage + ".json")
        require(path.is_file(), f"B lifecycle {stage} is not final")
        receipt = prior.raw.read_metadata(TRIAL_B, f"lifecycle/{stage}.json", ledger)
        lifecycle[stage] = {"status": receipt.get("status"), "stage": receipt.get("stage"),
                            "expected_map": receipt.get("expected_map"), "mode": receipt.get("mode"),
                            "port_released": receipt.get("port_released")}
    require(all(item["status"] == "PASS" and item["stage"] == stage and item["expected_map"] == "Town01" and
                item["mode"] == ("stopped" if stage == "stopped" else "running")
                for stage, item in lifecycle.items()) and lifecycle["stopped"]["port_released"] is True,
            "B owned lifecycle failed or server remains active")
    require(owner.get("capture_mode") == "expert" and owner.get("learned_model_control") is False and
            owner.get("vehicle_control_approved") is False, "B owner is not expert-only")
    return owner, lifecycle, json.loads(REPORT_A.read_text(encoding="utf-8"))


def verify_sources(owner: dict, ledger: dict, report_a: dict) -> tuple[dict, dict, dict]:
    plan_b = prior.raw.read_metadata(TRIAL_B, "owner_plan.json", ledger)
    plan_a = prior.raw.read_metadata(TRIAL_A, "owner_plan.json", {})
    require(sha(TRIAL_A / "owner_plan.json") == report_a["sources"]["raw_metadata"]["owner_plan.json"]["sha256"],
            "A owner plan changed after QA")
    require(owner.get("exit_code") == 0 and owner.get("source_bytes_unchanged_and_archived") is True and
            plan_b.get("source_bytes_archived") is True and plan_b.get("bounds_source_bytes_archived") is True,
            "B owner did not complete with archived execution sources")
    require(plan_b.get("capture_mode") == "expert" and plan_b.get("map") == "Town01" and
            plan_b.get("quality") == "Epic" and plan_b.get("route_sha256") == ROUTE_SHA and
            plan_b.get("port") == 2100 and plan_b.get("collector_wall_timeout_sec") == 300,
            "B owner plan differs from preregistered scope")
    arguments = one_factor_arguments(plan_a["collector_argv"], plan_b["collector_argv"])
    source_pins = plan_b.get("source_sha256")
    require(type(source_pins) is dict and set(source_pins) == set(owner.get("source_checks", {})) and
            all(value is True for value in owner["source_checks"].values()), "B archived-source after-check incomplete")
    require(source_pins["scripts/e2e/run_owned_carla_expert_trial.sh"] == WRAPPER_SHA and
            source_pins["scripts/e2e/collect_carla_vad_expert.py"] == COLLECTOR_B_SHA and
            plan_a["source_sha256"]["scripts/e2e/collect_carla_vad_expert.py"] == prior.PINS["collector"][1],
            "A/B collector source identity differs from declaration")
    for relative, expected in source_pins.items():
        data = prior.raw.control_audit.checked_bytes(TRIAL_B, "provenance/" + relative, ledger)
        require(hashlib.sha256(data).hexdigest() == expected, f"B archived execution source changed: {relative}")
    require(source_pins["portable_e2e/model.py"] == plan_a["source_sha256"]["portable_e2e/model.py"] and
            source_pins["portable_e2e/runtime_contract.py"] == plan_a["source_sha256"]["portable_e2e/runtime_contract.py"],
            "A/B original motion-bound source differs")
    return plan_b, arguments, {"b_archived_source_sha256": source_pins,
        "a_archived_collector_sha256": prior.PINS["collector"][1],
        "b_archived_collector_sha256": COLLECTOR_B_SHA,
        "collector_instrumentation_bytes_differ": True,
        "b_current_collector_sha256": sha(ROOT / "tools/e2e/collect_carla_vad_expert.py")}


def validate_geometry(trial: Path, manifest: dict, receipts: list[dict], ledger: dict) -> dict:
    """Authenticate the captured box/map bytes without claiming lane containment."""
    capture = manifest["capture_contract"]
    body = capture.get("actor_body_geometry", {})
    map_record = capture.get("active_map_opendrive", {})
    require(body.get("status") == "RECORDED" and body.get("source") == "carla.Vehicle.bounding_box" and
            body.get("actor_type_id") == "vehicle.toyota.prius" and
            body.get("reference_frame") == "CARLA actor-local; not ROS rear-axle base_link" and
            body.get("lane_body_containment_verified") is False,
            "B actor footprint provenance missing or contradictory")
    actor_id = body.get("actor_id")
    require(type(actor_id) is int and actor_id > 0 and receipts and
            all(row.get("actor_id") == actor_id and row.get("response_actor_id") == actor_id for row in receipts),
            "B actor footprint is not bound to ACK actor identity")
    construction = capture.get("agent_initialization", {}).get("construction", {})
    require(construction.get("before", {}).get("actor_id") == actor_id and
            construction.get("after", {}).get("actor_id") == actor_id,
            "B actor footprint is not bound to BasicAgent actor identity")
    box = body.get("bounding_box", {})
    for component in ("location", "extent", "rotation"):
        values = box.get(component)
        require(type(values) is dict and set(values) == {"x", "y", "z"} if component != "rotation"
                else type(values) is dict and set(values) == {"roll", "pitch", "yaw"},
                f"B actor box {component} is incomplete")
        for value in values.values():
            require(type(value) in (int, float) and math.isfinite(value) and
                    (component != "extent" or value > 0), "B actor box has invalid scalar")
    require(map_record.get("status") == "RECORDED" and map_record.get("source") == "carla.Map.to_opendrive" and
            map_record.get("map_name") == "Carla/Maps/Town01" and
            map_record.get("file") == manifest.get("files", {}).get("active_map_opendrive") ==
            "active_map_opendrive.xodr" and map_record.get("lane_body_containment_verified") is False,
            "B active-map OpenDRIVE provenance missing or contradictory")
    payload = prior.raw.control_audit.checked_bytes(trial, "episode/active_map_opendrive.xodr", ledger)
    require(len(payload) == map_record.get("size_bytes") and
            hashlib.sha256(payload).hexdigest() == map_record.get("sha256") and
            ET.fromstring(payload).tag == "OpenDRIVE", "B active-map OpenDRIVE bytes differ from manifest")
    return {"status": "BBOX_AND_ACTIVE_MAP_ARCHIVE_VERIFIED_NOT_LANE_PASS", "actor_id": actor_id,
            "actor_type_id": body["actor_type_id"], "bounding_box": box,
            "map_sha256": map_record["sha256"], "map_size_bytes": len(payload),
            "body_containment_verified": False,
            "limits": "Raw actor box and same-world OpenDRIVE are bound to this episode, but exact intended-lane body containment and continuous sweep are not independently evaluated."}


def verify_static_manifest(a: dict, b: dict) -> dict:
    ac, bc = a["capture_contract"], b["capture_contract"]
    for key in ("physics_hz", "camera_hz", "camera_interval_ticks", "camera_order", "image_encoding",
                "jpeg_quality", "target_speed_kmh", "maximum_total_duration_sec", "capture_phase_schedule",
                "seed", "spawn_z_offset_m", "command_lookahead_m", "command_exit_lookahead_m",
                "goal_tolerance_m", "stopped_brake_steering_suppressed", "client_map_loading_allowed"):
        require(ac.get(key) == bc.get(key), f"A/B static capture setting changed: {key}")
    require(a.get("cameras") == b.get("cameras") and
            a["coordinate_contract"] == b["coordinate_contract"],
            "A/B camera rig or coordinate contract changed")
    for key in ("town", "weather", "vehicle_type", "client_map_loading_allowed", "client_map_loading_performed",
                "capture_world_settings"):
        require(a["runtime"].get(key) == b["runtime"].get(key), f"A/B runtime factor changed: {key}")
    factor = one_factor_agent(ac["basic_agent_control"], bc["basic_agent_control"])
    require(a["runtime"]["basic_agent_control"]["effective_opt_dict"]["max_steering"] == 0.8 and
            b["runtime"]["basic_agent_control"]["effective_opt_dict"]["max_steering"] == 0.45,
            "runtime effective BasicAgent steering cap differs")
    return factor


def measured_metrics(states: list[dict], projections: list[dict], window: dict,
                     future: dict, manifest: dict) -> dict:
    first, last = window["first_selected"], window["last_selected"]
    require(first is not None and last is not None, "no selected window to compare")
    native = [row for row in states if first["frame"] <= row["frame"] <= last["frame"]]
    projected = [row for row in projections if first["frame"] <= row["frame"] <= last["frame"]]
    require(native and len(native) == len(projected), "selected native denominator differs")
    speeds = [math.hypot(prior.finite(row["vx"], "vx"), prior.finite(row["vy"], "vy")) for row in native]
    cte = [prior.finite(row["cross_track_m"], "independent cross-track error") for row in projected]
    saved_cte = [abs(prior.finite(row["route_cte_m"], "recorded CTE")) for row in native]
    normalized_steer = [abs(prior.finite(row["current_control"]["steer"], "normalized steer")) for row in native]
    tire_rad = [abs(prior.finite(row["steering_tire_angle_rad"], "measured tire angle")) for row in native]
    curvature = future["original_nine_metrics"]["metrics"]["xy_curvature"]
    return {"selected_camera_anchors": window["selected_anchor_count"],
        "selected_span_s": window["selected_span_ns"] / 1e9,
        "selected_native_state_count": len(native), "minimum_speed_mps": min(speeds),
        "maximum_speed_mps": max(speeds), "mean_speed_mps": sum(speeds) / len(speeds),
        "maximum_independent_cte_m": max(cte), "maximum_recorded_cte_m": max(saved_cte),
        "maximum_abs_normalized_current_steer": max(normalized_steer),
        "maximum_abs_measured_tire_angle_rad": max(tire_rad),
        "turn_start_progress_m": window["turn_start_arc_m"], "turn_end_progress_m": window["turn_end_arc_m"],
        "turn_native_state_count": window["turn_native_state_count"],
        "turn_right_command_state_count": window["turn_command_1_state_count"],
        "turn_signed_yaw_change_rad": window["turn_signed_yaw_change_rad"],
        "goal_reached": manifest["result"].get("goal_reached"),
        "maximum_xy_curvature_per_m": curvature["maximum"]["value"] if curvature["maximum"] else None,
        "xy_curvature_violating_anchor_count": curvature["violating_anchor_count"],
        "xy_curvature_violating_step_count": curvature["violating_step_count"],
        "xy_curvature_unique_100ms_violation_ticks": curvature["unique_episode_relative_100ms_violation_tick_count"]}


def reference_a(report: dict) -> tuple[dict, dict, dict]:
    require(report.get("schema") == "carla.town01_long_right_9kph_raw_qa.v1" and
            report.get("status") == "FAIL_CLOSED" and report.get("dataset_admitted") is False and
            set(report.get("failed_checks", [])) == {"all_original_nine_motion_bounds_clear", "full_body_in_intended_lane"},
            "A QA is not the completed fixed-window failure result")
    a_ledger = report["sources"]["raw_metadata"]
    for name in ("owner_plan.json", "owner_result.json", "episode/manifest.json", "episode/route.json",
                 "episode/states.jsonl", "episode/camera_frames.jsonl"):
        require(sha(TRIAL_A / name) == a_ledger[name]["sha256"], f"A source changed after QA: {name}")
    manifest = prior.raw.read_metadata(TRIAL_A, "episode/manifest.json", {})
    route = prior.raw.read_metadata(TRIAL_A, "episode/route.json", {})
    states = prior.raw.read_metadata(TRIAL_A, "episode/states.jsonl", {}, lines=True)
    cameras = prior.raw.read_metadata(TRIAL_A, "episode/camera_frames.jsonl", {}, lines=True)
    window, _, projections = prior.select_window(route, states, cameras)
    recorded = report["window"]
    require(window["selected_anchor_count"] == recorded["selected_anchor_count"] and
            window["first_selected"]["frame"] == recorded["first_selected"]["frame"] and
            window["last_selected"]["frame"] == recorded["last_selected"]["frame"] and
            window["turn_signed_yaw_change_rad"] == recorded["turn_signed_yaw_change_rad"],
            "A fixed selection replay differs from sealed QA")
    return (manifest, measured_metrics(states, projections, window, report["future"], manifest),
            {"summary_sha256": REPORT_A_SHA,
             "raw_metadata_sha256": {name: a_ledger[name]["sha256"] for name in a_ledger}})


def audit_complete(owner: dict, lifecycle: dict, report_a: dict, ledger: dict) -> tuple[dict, dict]:
    plan_b, argv_factor, sources = verify_sources(owner, ledger, report_a)
    require((TRIAL_B / "episode").is_dir() and not (TRIAL_B / "episode.partial").exists(),
            "B completed owner has no single final episode")
    manifest = prior.raw.read_metadata(TRIAL_B, "episode/manifest.json", ledger)
    route = prior.raw.read_metadata(TRIAL_B, "episode/route.json", ledger)
    states = prior.raw.read_metadata(TRIAL_B, "episode/states.jsonl", ledger, lines=True)
    cameras = prior.raw.read_metadata(TRIAL_B, "episode/camera_frames.jsonl", ledger, lines=True)
    receipts = prior.raw.read_metadata(TRIAL_B, "episode/control_receipts.jsonl", ledger, lines=True)
    require(manifest.get("status") == "complete" and manifest.get("cleanup", {}).get("completed") is True and
            ledger["episode/route.json"]["sha256"] == ROUTE_SHA and
            manifest["result"].get("state_count") == len(states) and
            manifest["result"].get("camera_anchor_count") == len(cameras),
            "B native episode is incomplete")
    provenance = manifest["provenance"]
    for name in ("route", "mapping", "calibration", "collector"):
        expected = COLLECTOR_B_SHA if name == "collector" else prior.PINS[name][1]
        require(provenance[name + "_sha256"] == expected, f"B native provenance differs: {name}")
    manifest_a, metrics_a, sources_a = reference_a(report_a)
    factor = verify_static_manifest(manifest_a, manifest)
    timeline = prior.raw.measured_timeline(states)
    transport = prior.ack.analyze_transport(states, receipts,
        manifest["capture_contract"]["control_transport"]["bootstrap_observation"])
    camera_ack = prior.ack.analyze_camera_alignment(states, cameras)
    geometry = validate_geometry(TRIAL_B, manifest, receipts, ledger)
    window, selected_cameras, projections = prior.select_window(route, states, cameras)
    pixels, image_rows, camera_rows = prior.raw.camera_audit(TRIAL_B / "episode", manifest, cameras, timeline)
    selected_frames = {camera["frame"] for camera in selected_cameras}
    selected_camera_rows = [row for row in camera_rows if row["frame"] in selected_frames]
    _, future_rows = prior.raw.measured_future_audit(timeline, selected_cameras, route, selected_camera_rows)
    tail_start = next((prior.timestamp_ns(row) for row in states if row["capture_phase"] == "stationary_tail"), None)
    require(tail_start is not None, "B brake tail absent")
    future = prior.summarize_selected_futures(future_rows, selected_cameras, tail_start)
    events = {name: sum(len(row[name]) for row in states) for name in ("collision", "lane_invasion")}
    checks = {**window["flags"], **future["flags"],
        "six_rgb_pixels_and_camera_state_aligned": pixels["all_pixel_and_metadata_checks_clear"] and camera_ack["status"] == "PASS",
        "acknowledged_native_state_alignment": transport["status"] == "PASS",
        "collision_and_lane_sensor_clear": events == {"collision": 0, "lane_invasion": 0},
        "goal_reached": manifest["result"].get("goal_reached") is True,
        "actor_bbox_and_live_map_provenance": geometry["status"] == "BBOX_AND_ACTIVE_MAP_ARCHIVE_VERIFIED_NOT_LANE_PASS",
        "full_body_in_intended_lane": False}
    metrics_b = measured_metrics(states, projections, window, future, manifest)
    for relative, pin in ledger.items():
        require(sha(TRIAL_B / relative) == pin["sha256"], f"B raw metadata changed during audit: {relative}")
    for row in image_rows:
        require(sha(TRIAL_B / "episode" / row["path"]) == row["sha256"], "B JPEG changed during audit")
    comparison = {"a": metrics_a, "b": metrics_b,
        "delta_b_minus_a": {key: metrics_b[key] - metrics_a[key] for key in
            ("selected_camera_anchors", "selected_span_s", "maximum_independent_cte_m",
             "maximum_recorded_cte_m", "maximum_abs_normalized_current_steer",
             "maximum_abs_measured_tire_angle_rad", "maximum_xy_curvature_per_m",
             "xy_curvature_violating_anchor_count", "xy_curvature_unique_100ms_violation_ticks")},
        "interpretation_limit": "One same-site BasicAgent expert A/B with a new B collector instrumentation revision; not independent statistical replication or learned-control evidence."}
    report = {"schema": "carla.town01_long_right_steer045_ab_raw_qa.v1",
        "status": "FAIL_CLOSED" if not all(checks.values()) else "NECESSARY_CHECKS_CLEAR_NOT_ADMITTED",
        "dataset_admitted": False, "training_started": False, "learned_control": False,
        "all_b_necessary_checks_clear": all(checks.values()),
        "failed_checks": [name for name, passed in checks.items() if not passed], "checks": checks,
        "preregistered_plan_sha256": PLAN_SHA,
        "owner": {"exit_code": owner["exit_code"], "lifecycle": lifecycle},
        "one_factor_collector_argv": argv_factor, "one_factor_effective_agent": factor,
        "source_binding": {**sources, "a": sources_a, "b_raw_metadata": ledger,
                           "b_qa_script_sha256": sha(Path(__file__)),
                           "reused_a_qa_script_sha256": sha(Path(prior.__file__))},
        "geometry_provenance": geometry,
        "unsupported_checks": {"full_body_in_intended_lane":
            "Recorded actor bbox and same-world OpenDRIVE are authentic inputs. This CPU-only audit does not reconstruct an independently verified intended OpenDRIVE lane polygon or continuously sweep this exact body through it. Sensor zero events are insufficient."},
        "window": window, "future": future, "transport": transport, "camera_ack": camera_ack,
        "pixels": pixels, "events": events, "comparison": comparison,
        "limits": ["Normalized BasicAgent steering command and measured tire angle are different units.",
                   "The B collector differs from A in geometry instrumentation; this is a one-attempt diagnostic.",
                   "No CARLA API/server, GPU, SSH, training or dataset conversion was used by this audit."]}
    return report, {"native_projection.jsonl": projections, "selected_future.jsonl": future_rows,
                    "jpeg_audit.jsonl": image_rows}


def audit() -> tuple[dict, dict]:
    ledger = {}
    owner, lifecycle, report_a = closed_owner(ledger)
    if owner.get("exit_code") != 0:
        return {"schema": "carla.town01_long_right_steer045_ab_raw_qa.v1",
            "status": "COLLECTION_FAILED", "dataset_admitted": False, "training_started": False,
            "learned_control": False, "b_metrics": None, "comparison": None,
            "owner": {"exit_code": owner.get("exit_code"), "lifecycle": lifecycle},
            "preregistered_plan_sha256": PLAN_SHA,
            "reason": "B owner failed; no B episode bytes were opened or inferred.",
            "source_binding": {"owner_result_sha256": ledger["owner_result.json"]["sha256"]}}, {}
    return audit_complete(owner, lifecycle, report_a, ledger)


def write_report(report: dict, companions: dict) -> None:
    require(not OUTPUT.exists() and not OUTPUT.is_symlink(), "fresh B QA_v1 output required")
    OUTPUT.mkdir(parents=True, exist_ok=False)
    for name, rows in companions.items():
        prior.write_jsonl(OUTPUT / name, rows)
    prior.write_json(OUTPUT / "summary.json", report)
    with (OUTPUT / "QA.md").open("x", encoding="utf-8") as stream:
        stream.write("# Town01 right-turn steering cap A/B raw QA\n\n")
        stream.write(f"B status: **{report['status']}**. Dataset admitted: **false**. "
                     "Training started: **false**. Learned control: **false**.\n\n")
        if report.get("comparison") is None:
            stream.write("The B owner failed or the audit could not establish complete evidence. "
                         "No B metrics or A/B effect are claimed.\n")
            if report.get("reason"):
                stream.write(report["reason"] + "\n")
            if report.get("fatal_detail"):
                stream.write(f"Fatal check: {report['fatal_detail']}\n")
            return
        a, b = report["comparison"]["a"], report["comparison"]["b"]
        stream.write("Both arms use the same pinned Town01 route and effective BasicAgent settings except "
                     "the normalized steering cap, 0.80 → 0.45. B's collector adds geometry recording; "
                     "this is one same-site expert-control diagnostic.\n\n")
        stream.write("| Raw measure | A | B |\n| --- | ---: | ---: |\n")
        for label, key in (("Selected camera anchors", "selected_camera_anchors"),
                           ("Selected span (s)", "selected_span_s"),
                           ("Peak XY curvature (rad/m)", "maximum_xy_curvature_per_m"),
                           ("Curvature violating anchors", "xy_curvature_violating_anchor_count"),
                           ("Unique violating 100 ms ticks", "xy_curvature_unique_100ms_violation_ticks"),
                           ("Maximum independent CTE (m)", "maximum_independent_cte_m"),
                           ("Maximum normalized current steer", "maximum_abs_normalized_current_steer"),
                           ("Maximum measured tire angle (rad)", "maximum_abs_measured_tire_angle_rad"),
                           ("Turn signed yaw change (rad)", "turn_signed_yaw_change_rad"),
                           ("Goal reached", "goal_reached")):
            stream.write(f"| {label} | {a[key]} | {b[key]} |\n")
        stream.write("\nB failed checks: " + ", ".join(report["failed_checks"]) + ".\n\n")
        stream.write("The current actor bbox and active OpenDRIVE file are byte-verified, but exact "
                     "full-body intended-lane containment remains unsupported and fails closed. "
                     "The nine original motion bounds are unchanged; no violating anchors or ticks are dropped. "
                     "ACK and RGB outcomes are in `summary.json`; per-anchor futures, native progress and JPEG "
                     "checks are in the companion JSONL files. This is not an Autoware GUI or learned-driving result.\n")


def main() -> int:
    require((TRIAL_B / "owner_result.json").is_file() and
            (TRIAL_B / "lifecycle/stopped.json").is_file(),
            "B owner is still running; QA output was not created")
    try:
        report, companions = audit()
    except Exception as error:
        report, companions = {"schema": "carla.town01_long_right_steer045_ab_raw_qa.v1",
            "status": "FAIL_CLOSED", "dataset_admitted": False, "training_started": False,
            "learned_control": False, "b_metrics": None, "comparison": None,
            "preregistered_plan_sha256": PLAN_SHA, "fatal_check": type(error).__name__,
            "fatal_detail": str(error)}, {}
    write_report(report, companions)
    print(json.dumps({"status": report["status"], "dataset_admitted": False,
                      "output": str(OUTPUT), "failed_checks": report.get("failed_checks", []),
                      "fatal_check": report.get("fatal_check")}))
    return 0 if report["status"] == "NECESSARY_CHECKS_CLEAR_NOT_ADMITTED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
