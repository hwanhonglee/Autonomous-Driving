#!/usr/bin/env python3
"""HH_260906 - Read-only, fail-closed Town01 BasicAgent waypoint-purge B/C audit.

The C owner and all lifecycle receipts must finish before any C episode is read.
This same-site expert diagnostic never admits data or starts learned driving.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path

from scripts.e2e import audit_future_actor_lane_containment as lane
from scripts.e2e import audit_long_turn_9kph_raw as prior
from scripts.e2e import audit_long_turn_steer045_ab as ab


ROOT = Path(__file__).resolve().parents[2]
TRIAL_B = ab.TRIAL_B
TRIAL_C = ROOT / "storage/experiments/training/2026-09-27/drive_long_route_town01_right_s0002_p06_9kph_steer045_purge15_trial_v1"
EPISODE_C = TRIAL_C / "episode"
PLAN = ROOT / "docs/assets/validation/2026-09-27/drive_waypoint_purge_ab/PLAN.md"
OUTPUT = PLAN.parent / "QA_v1"
REPORT_B = ab.OUTPUT / "summary.json"
LANE_REPORT_B = ROOT / "docs/assets/validation/2026-09-27/drive_capture_geometry_provenance/B_lane_gate_30_120.json"
PLAN_SHA = "c5946590a902a28c4b09da6d52d3505dbd910b126214d0650866dcaec6e3b803"
REPORT_B_SHA = "6ced9f6307c067f4e3a7441a07e906217cec38060ff283b9d8ef5cf225a56c9e"
LANE_REPORT_B_SHA = "f561b0dcc6b67e6b3eb1b9c866ecc26daed0b09532b30b6219fa4ede8d8253ef"
LANE_CODE_SHA = "75d73449c8ffb17e27d1ddb419bd064a58ab052043b2a7b0ada69de8a65fa57d"
COLLECTOR_SHA = ab.COLLECTOR_B_SHA
ROUTE_SHA = ab.ROUTE_SHA


def require(condition: bool, message: str) -> None:
    prior.require(condition, message)


def sha(path: Path) -> str:
    return prior.sha(path)


def one_factor_arguments(argv_b: list[str], argv_c: list[str]) -> dict:
    """HH_260906 - Only episode path and explicit 1.5 m purge base may differ."""
    require(type(argv_b) is list and type(argv_c) is list and len(argv_b) > 2 and len(argv_c) > 2,
            "complete B/C collector argv required")
    require(argv_b[0] == str(TRIAL_B / "episode") and argv_c[0] == str(EPISODE_C) and
            argv_b[1] == argv_c[1] == str(prior.ROUTE), "B/C output or pinned route differs")
    option = "--basic-agent-base-min-distance-m"
    require(argv_b.count(option) == 0 and argv_c.count(option) == 1,
            "only C may opt in to the purge-base change")
    location = argv_c.index(option)
    require(location + 1 < len(argv_c) and argv_c[location + 1] == "1.5",
            "C purge-base argument is not preregistered 1.5 m")
    reduced = argv_c[:]
    del reduced[location:location + 2]
    require(argv_b[1:] == reduced[1:], "another B/C collector argument changed")
    return {"b_base_min_distance_m": 3.0, "c_base_min_distance_m": 1.5,
            "all_other_collector_arguments_equal": True}


def one_factor_agent(b: dict, c: dict) -> dict:
    """HH_260906 - Require the injected BasicAgent option and unchanged peers."""
    require(b.get("effective_opt_dict", {}).get("base_min_distance") == 3.0 and
            c.get("effective_opt_dict", {}).get("base_min_distance") == 1.5 and
            b.get("waypoint_purge_lookahead", {}).get("base_min_distance_m") == 3.0 and
            c.get("waypoint_purge_lookahead", {}).get("base_min_distance_m") == 1.5 and
            b.get("collector_cli", {}).get("--basic-agent-base-min-distance-m") == 3.0 and
            c.get("collector_cli", {}).get("--basic-agent-base-min-distance-m") == 1.5 and
            b.get("normalized_max_steering") == c.get("normalized_max_steering") == 0.45,
            "effective B/C purge base or fixed steering cap differs")
    left, right = copy.deepcopy(b), copy.deepcopy(c)
    for item in (left, right):
        item["effective_opt_dict"].pop("base_min_distance")
        item["waypoint_purge_lookahead"].pop("base_min_distance_m")
        item["collector_cli"].pop("--basic-agent-base-min-distance-m")
    require(left == right, "another BasicAgent effective configuration factor changed")
    return {"b_base_min_distance_m": 3.0, "c_base_min_distance_m": 1.5,
            "b_nominal_purge_at_9kmh_m": 4.25, "c_nominal_purge_at_9kmh_m": 2.75,
            "all_other_effective_agent_options_equal": True}


def closed_owner(ledger: dict) -> tuple[dict, dict, dict, dict]:
    """HH_260906 - Check final owner before opening any C episode bytes."""
    require(sha(PLAN) == PLAN_SHA, "preregistered C plan changed")
    require(sha(prior.ROUTE) == ROUTE_SHA, "preregistered route changed")
    require(sha(Path(lane.__file__)) == LANE_CODE_SHA, "bound lane-gate implementation changed")
    require(REPORT_B.is_file() and sha(REPORT_B) == REPORT_B_SHA, "B sealed raw QA changed")
    require(LANE_REPORT_B.is_file() and sha(LANE_REPORT_B) == LANE_REPORT_B_SHA,
            "B sealed lane report changed")
    require((TRIAL_C / "owner_result.json").is_file(), "C owner_result is not final")
    owner = prior.raw.read_metadata(TRIAL_C, "owner_result.json", ledger)
    lifecycle = {}
    for stage in ("ready", "after_capture", "stopped"):
        path = TRIAL_C / "lifecycle" / (stage + ".json")
        require(path.is_file(), f"C lifecycle {stage} is not final")
        receipt = prior.raw.read_metadata(TRIAL_C, f"lifecycle/{stage}.json", ledger)
        lifecycle[stage] = {key: receipt.get(key) for key in
                            ("status", "stage", "expected_map", "mode", "port_released")}
    require(all(item["status"] == "PASS" and item["stage"] == stage and
                item["expected_map"] == "Town01" and
                item["mode"] == ("stopped" if stage == "stopped" else "running")
                for stage, item in lifecycle.items()) and lifecycle["stopped"]["port_released"] is True,
            "C owned lifecycle failed or server remains active")
    require(owner.get("capture_mode") == "expert" and owner.get("learned_model_control") is False and
            owner.get("vehicle_control_approved") is False, "C owner is not expert-only")
    return owner, lifecycle, json.loads(REPORT_B.read_text(encoding="utf-8")), \
        json.loads(LANE_REPORT_B.read_text(encoding="utf-8"))


def verify_sources(owner: dict, ledger: dict, report_b: dict) -> tuple[dict, dict, dict]:
    plan_c = prior.raw.read_metadata(TRIAL_C, "owner_plan.json", ledger)
    plan_b = prior.raw.read_metadata(TRIAL_B, "owner_plan.json", {})
    pinned_b = report_b["source_binding"]["b_raw_metadata"]
    require(sha(TRIAL_B / "owner_plan.json") == pinned_b["owner_plan.json"]["sha256"],
            "B owner plan changed after sealed QA")
    require(owner.get("exit_code") == 0 and owner.get("source_bytes_unchanged_and_archived") is True and
            plan_c.get("source_bytes_archived") is True and
            plan_c.get("bounds_source_bytes_archived") is True,
            "C owner did not complete with archived execution sources")
    require(plan_c.get("capture_mode") == "expert" and plan_c.get("map") == "Town01" and
            plan_c.get("quality") == "Epic" and plan_c.get("route_sha256") == ROUTE_SHA and
            plan_c.get("port") == 2100 and plan_c.get("collector_wall_timeout_sec") == 300,
            "C owner plan differs from preregistered scope")
    args = one_factor_arguments(plan_b["collector_argv"], plan_c["collector_argv"])
    source_pins = plan_c.get("source_sha256")
    require(type(source_pins) is dict and source_pins == plan_b.get("source_sha256") and
            source_pins.get("scripts/e2e/collect_carla_vad_expert.py") == COLLECTOR_SHA and
            source_pins.get("scripts/e2e/run_owned_carla_expert_trial.sh") == ab.WRAPPER_SHA and
            set(source_pins) == set(owner.get("source_checks", {})) and
            all(value is True for value in owner["source_checks"].values()),
            "C archived sources or after-capture source checks differ")
    for relative, expected in source_pins.items():
        data = prior.raw.control_audit.checked_bytes(TRIAL_C, "provenance/" + relative, ledger)
        require(hashlib.sha256(data).hexdigest() == expected,
                f"C archived execution source changed: {relative}")
    return plan_c, args, {"b_archived_source_sha256": plan_b["source_sha256"],
                          "c_archived_source_sha256": source_pins,
                          "collector_implementation_equal": True}


def verify_static_manifest(b: dict, c: dict) -> dict:
    bc, cc = b["capture_contract"], c["capture_contract"]
    for key in ("physics_hz", "camera_hz", "camera_interval_ticks", "camera_order", "image_encoding",
                "jpeg_quality", "target_speed_kmh", "maximum_total_duration_sec", "capture_phase_schedule",
                "seed", "spawn_z_offset_m", "command_lookahead_m", "command_exit_lookahead_m",
                "goal_tolerance_m", "stopped_brake_steering_suppressed", "client_map_loading_allowed"):
        require(bc.get(key) == cc.get(key), f"B/C static capture setting changed: {key}")
    require(b.get("cameras") == c.get("cameras") and
            b.get("coordinate_contract") == c.get("coordinate_contract"),
            "B/C camera rig or coordinate contract changed")
    for key in ("town", "weather", "vehicle_type", "client_map_loading_allowed",
                "client_map_loading_performed", "capture_world_settings"):
        require(b["runtime"].get(key) == c["runtime"].get(key),
                f"B/C runtime factor changed: {key}")
    factor = one_factor_agent(bc["basic_agent_control"], cc["basic_agent_control"])
    runtime_b, runtime_c = copy.deepcopy(b["runtime"]["basic_agent_control"]), \
                           copy.deepcopy(c["runtime"]["basic_agent_control"])
    require(runtime_b["effective_opt_dict"].pop("base_min_distance") == 3.0 and
            runtime_c["effective_opt_dict"].pop("base_min_distance") == 1.5 and
            runtime_b == runtime_c, "another runtime BasicAgent factor changed")
    require(bc["active_map_opendrive"]["sha256"] == cc["active_map_opendrive"]["sha256"],
            "B/C archived active map differs")
    return factor


def reference_b(report: dict, lane_b: dict) -> tuple[dict, dict, dict]:
    require(report.get("schema") == "carla.town01_long_right_steer045_ab_raw_qa.v1" and
            report.get("status") == "FAIL_CLOSED" and report.get("dataset_admitted") is False and
            report.get("failed_checks") == ["full_body_in_intended_lane"],
            "B QA is not the sealed fixed-window result")
    require(lane_b.get("status") == "FAIL" and lane_b.get("training_data_approved") is False and
            lane_b.get("lane_sample_m") == lane.LANE_SAMPLE_M and
            lane_b.get("lane_inset_m") == lane.LANE_INSET_M and
            lane_b.get("authorized_seam_gap_close_m") == lane.SEAM_GAP_CLOSE_M,
            "B lane baseline differs from C gate")
    pinned = report["source_binding"]["b_raw_metadata"]
    for name in ("owner_plan.json", "owner_result.json", "episode/manifest.json", "episode/route.json",
                 "episode/states.jsonl", "episode/camera_frames.jsonl", "episode/control_receipts.jsonl",
                 "episode/active_map_opendrive.xodr"):
        require(sha(TRIAL_B / name) == pinned[name]["sha256"],
                f"B raw metadata changed after sealed QA: {name}")
    manifest = prior.raw.read_metadata(TRIAL_B, "episode/manifest.json", {})
    route = prior.raw.read_metadata(TRIAL_B, "episode/route.json", {})
    states = prior.raw.read_metadata(TRIAL_B, "episode/states.jsonl", {}, lines=True)
    cameras = prior.raw.read_metadata(TRIAL_B, "episode/camera_frames.jsonl", {}, lines=True)
    window, _, projections = prior.select_window(route, states, cameras)
    require(window["selected_anchor_count"] == report["window"]["selected_anchor_count"] and
            window["first_selected"]["frame"] == report["window"]["first_selected"]["frame"] and
            window["last_selected"]["frame"] == report["window"]["last_selected"]["frame"],
            "B fixed selection replay differs from sealed QA")
    metrics = ab.measured_metrics(states, projections, window, report["future"], manifest)
    require(metrics == report["comparison"]["b"], "B measured metrics differ from sealed QA")
    for key, expected in lane_b["source_sha256"].items():
        target = (ROOT / "tools/e2e/collect_carla_vad_expert.py" if key == "collector"
                  else TRIAL_B / "episode" / {"manifest": "manifest.json", "route": "route.json",
                                              "states": "states.jsonl",
                                              "active_map_opendrive": "active_map_opendrive.xodr"}[key])
        require(sha(target) == expected, f"B lane input changed: {key}")
    return manifest, metrics, {"summary_sha256": REPORT_B_SHA,
                               "lane_report_sha256": LANE_REPORT_B_SHA,
                               "raw_metadata_sha256": {key: value["sha256"] for key, value in pinned.items()}}


def run_lane_gate() -> dict:
    """HH_260906 - Same immutable implementation and 30–120 m parameters as B."""
    try:
        return lane.audit_episode(EPISODE_C, start_m=30.0, end_m=120.0)
    except Exception as error:
        return {"status": "UNSUPPORTED", "reason": f"{type(error).__name__}: {error}",
                "training_data_approved": False}


def audit_complete(owner: dict, lifecycle: dict, report_b: dict,
                   lane_b: dict, ledger: dict) -> tuple[dict, dict]:
    _, argv_factor, sources = verify_sources(owner, ledger, report_b)
    require(EPISODE_C.is_dir() and not (TRIAL_C / "episode.partial").exists(),
            "C completed owner has no single final episode")
    manifest = prior.raw.read_metadata(TRIAL_C, "episode/manifest.json", ledger)
    route = prior.raw.read_metadata(TRIAL_C, "episode/route.json", ledger)
    states = prior.raw.read_metadata(TRIAL_C, "episode/states.jsonl", ledger, lines=True)
    cameras = prior.raw.read_metadata(TRIAL_C, "episode/camera_frames.jsonl", ledger, lines=True)
    receipts = prior.raw.read_metadata(TRIAL_C, "episode/control_receipts.jsonl", ledger, lines=True)
    require(manifest.get("status") == "complete" and manifest.get("cleanup", {}).get("completed") is True and
            ledger["episode/route.json"]["sha256"] == ROUTE_SHA and
            manifest["result"].get("state_count") == len(states) and
            manifest["result"].get("camera_anchor_count") == len(cameras),
            "C native episode is incomplete")
    for name in ("route", "mapping", "calibration", "collector"):
        expected = COLLECTOR_SHA if name == "collector" else prior.PINS[name][1]
        require(manifest["provenance"][name + "_sha256"] == expected,
                f"C native provenance differs: {name}")
    manifest_b, metrics_b, sources_b = reference_b(report_b, lane_b)
    factor = verify_static_manifest(manifest_b, manifest)
    timeline = prior.raw.measured_timeline(states)
    transport = prior.ack.analyze_transport(states, receipts,
        manifest["capture_contract"]["control_transport"]["bootstrap_observation"])
    camera_ack = prior.ack.analyze_camera_alignment(states, cameras)
    geometry = ab.validate_geometry(TRIAL_C, manifest, receipts, ledger)
    window, selected_cameras, projections = prior.select_window(route, states, cameras)
    pixels, image_rows, camera_rows = prior.raw.camera_audit(EPISODE_C, manifest, cameras, timeline)
    selected_frames = {camera["frame"] for camera in selected_cameras}
    selected_camera_rows = [row for row in camera_rows if row["frame"] in selected_frames]
    _, future_rows = prior.raw.measured_future_audit(
        timeline, selected_cameras, route, selected_camera_rows)
    tail_start = next((prior.timestamp_ns(row) for row in states
                       if row["capture_phase"] == "stationary_tail"), None)
    require(tail_start is not None, "C brake tail absent")
    future = prior.summarize_selected_futures(future_rows, selected_cameras, tail_start)
    events = {name: sum(len(row[name]) for row in states) for name in ("collision", "lane_invasion")}
    for relative, pin in ledger.items():
        require(sha(TRIAL_C / relative) == pin["sha256"],
                f"C raw metadata changed during audit: {relative}")
    for row in image_rows:
        require(sha(EPISODE_C / row["path"]) == row["sha256"], "C JPEG changed during audit")
    lane_c = run_lane_gate()
    require(lane_c.get("training_data_approved") is False,
            "lane tool unexpectedly approved training data")
    if lane_c["status"] in ("PASS", "FAIL"):
        require(lane_c["source_sha256"]["manifest"] == ledger["episode/manifest.json"]["sha256"] and
                lane_c["source_sha256"]["route"] == ledger["episode/route.json"]["sha256"] and
                lane_c["source_sha256"]["states"] == ledger["episode/states.jsonl"]["sha256"] and
                lane_c["source_sha256"]["collector"] == COLLECTOR_SHA and
                lane_c["source_sha256"]["active_map_opendrive"] == geometry["map_sha256"] and
                lane_c["lane_sample_m"] == lane_b["lane_sample_m"] and
                lane_c["lane_inset_m"] == lane_b["lane_inset_m"] and
                lane_c["authorized_seam_gap_close_m"] == lane_b["authorized_seam_gap_close_m"],
                "C lane run does not match pinned raw/map/parameters")
    checks = {**window["flags"], **future["flags"],
        "six_rgb_pixels_and_camera_state_aligned": pixels["all_pixel_and_metadata_checks_clear"] and
            camera_ack["status"] == "PASS",
        "acknowledged_native_state_alignment": transport["status"] == "PASS",
        "collision_and_lane_sensor_clear": events == {"collision": 0, "lane_invasion": 0},
        "goal_reached": manifest["result"].get("goal_reached") is True,
        "actor_bbox_and_live_map_provenance":
            geometry["status"] == "BBOX_AND_ACTIVE_MAP_ARCHIVE_VERIFIED_NOT_LANE_PASS",
        "full_body_intended_lane_sampled_and_bounded_sweep": lane_c["status"] == "PASS"}
    metrics_c = ab.measured_metrics(states, projections, window, future, manifest)
    lane_keys = ("frames_checked", "sweeps_checked", "body_raw_sampled_lane_violation_count",
                 "body_inset_10cm_violation_count", "sweep_raw_sampled_lane_violation_count",
                 "sweep_inset_10cm_violation_count", "maximum_body_raw_sampled_lane_outside_m2",
                 "maximum_body_outside_m2", "maximum_sweep_raw_sampled_lane_outside_m2",
                 "maximum_sweep_outside_m2")
    comparison = {"b": metrics_b, "c": metrics_c,
        "delta_c_minus_b": {key: metrics_c[key] - metrics_b[key] for key in
            ("selected_camera_anchors", "selected_span_s", "minimum_speed_mps", "mean_speed_mps",
             "maximum_independent_cte_m", "maximum_recorded_cte_m",
             "maximum_abs_normalized_current_steer", "maximum_abs_measured_tire_angle_rad",
             "turn_signed_yaw_change_rad", "maximum_xy_curvature_per_m",
             "xy_curvature_violating_anchor_count", "xy_curvature_unique_100ms_violation_ticks")},
        "lane": {"b": {key: lane_b.get(key) for key in lane_keys},
                 "c": {key: lane_c.get(key) for key in lane_keys},
                 "status_b": lane_b["status"], "status_c": lane_c["status"]},
        "interpretation_limit": "One same-site BasicAgent expert B/C. The lane gate uses CARLA actor-pose projection, so its 30–120 m frame range can differ from the base_link camera-anchor selection. Not an independent training site or learned-control result."}
    report = {"schema": "carla.town01_right_purge15_bc_raw_qa.v1",
        "status": "FAIL_CLOSED" if not all(checks.values()) else "NECESSARY_CHECKS_CLEAR_NOT_ADMITTED",
        "dataset_admitted": False, "training_started": False, "learned_control": False,
        "failed_checks": [name for name, passed in checks.items() if not passed], "checks": checks,
        "preregistered_plan_sha256": PLAN_SHA,
        "owner": {"exit_code": owner["exit_code"], "lifecycle": lifecycle},
        "one_factor_collector_argv": argv_factor, "one_factor_effective_agent": factor,
        "source_binding": {**sources, "b": sources_b, "c_raw_metadata": ledger,
                           "c_qa_script_sha256": sha(Path(__file__)),
                           "reused_a_qa_script_sha256": sha(Path(prior.__file__)),
                           "reused_b_qa_script_sha256": sha(Path(ab.__file__)),
                           "lane_gate_script_sha256": LANE_CODE_SHA},
        "geometry_provenance": geometry, "window": window, "future": future,
        "transport": transport, "camera_ack": camera_ack, "pixels": pixels,
        "events": events, "lane_gate": lane_c, "comparison": comparison,
        "limits": ["Same-site expert one-factor comparison; not independent training diversity.",
                   "Raw and 10 cm inset sampled-lane areas are separate from sensor lane-invasion events.",
                   "Lane gate bounds sampled 20 Hz bodies and snapshot-linear sweeps; exact physical continuous time is not proven.",
                   "No CARLA server, GPU, SSH, training or dataset conversion is performed by this audit."]}
    return report, {"native_projection.jsonl": projections,
                    "selected_future.jsonl": future_rows, "jpeg_audit.jsonl": image_rows}


def audit() -> tuple[dict, dict]:
    ledger = {}
    owner, lifecycle, report_b, lane_b = closed_owner(ledger)
    if owner.get("exit_code") != 0:
        return {"schema": "carla.town01_right_purge15_bc_raw_qa.v1",
            "status": "COLLECTION_FAILED", "dataset_admitted": False,
            "training_started": False, "learned_control": False,
            "comparison": None, "c_metrics": None, "lane_gate": None,
            "owner": {"exit_code": owner.get("exit_code"), "lifecycle": lifecycle},
            "preregistered_plan_sha256": PLAN_SHA,
            "reason": "C owner failed; no C episode bytes were opened or inferred.",
            "source_binding": {"owner_result_sha256": ledger["owner_result.json"]["sha256"]}}, {}
    return audit_complete(owner, lifecycle, report_b, lane_b, ledger)


def write_report(report: dict, companions: dict) -> None:
    require(not OUTPUT.exists() and not OUTPUT.is_symlink(), "fresh C QA_v1 output required")
    OUTPUT.mkdir(parents=True, exist_ok=False)
    for name, rows in companions.items():
        prior.write_jsonl(OUTPUT / name, rows)
    prior.write_json(OUTPUT / "summary.json", report)
    if report.get("lane_gate") is not None:
        prior.write_json(OUTPUT / "C_lane_gate_30_120.json", report["lane_gate"])
    with (OUTPUT / "QA.md").open("x", encoding="utf-8") as stream:
        stream.write("# Town01 right-turn BasicAgent waypoint purge B/C raw QA\n\n")
        stream.write(f"C status: **{report['status']}**. Dataset admitted: **false**. "
                     "Training started: **false**. Learned control: **false**.\n\n")
        if report.get("comparison") is None:
            stream.write("C collection failed or the audit could not establish complete evidence. "
                         "No C metrics or B/C effect are claimed.\n")
            if report.get("reason"):
                stream.write(report["reason"] + "\n")
            if report.get("fatal_detail"):
                stream.write(f"Fatal check: {report['fatal_detail']}\n")
            return
        b, c = report["comparison"]["b"], report["comparison"]["c"]
        stream.write("Both arms use one pinned Town01 route and BasicAgent expert settings except "
                     "`base_min_distance` 3.0 → 1.5 m; normalized steering cap remains 0.45. "
                     "This is one same-site expert-control diagnostic, not a new training site.\n\n")
        stream.write("| Raw measure | B | C |\n| --- | ---: | ---: |\n")
        for label, key in (("Selected camera anchors", "selected_camera_anchors"),
                           ("Selected span (s)", "selected_span_s"),
                           ("Peak XY curvature (/m)", "maximum_xy_curvature_per_m"),
                           ("Curvature violating anchors", "xy_curvature_violating_anchor_count"),
                           ("Unique violating 100 ms ticks", "xy_curvature_unique_100ms_violation_ticks"),
                           ("Maximum independent CTE (m)", "maximum_independent_cte_m"),
                           ("Minimum selected speed (m/s)", "minimum_speed_mps"),
                           ("Maximum normalized current steer", "maximum_abs_normalized_current_steer"),
                           ("Maximum measured tire angle (rad)", "maximum_abs_measured_tire_angle_rad"),
                           ("Turn signed yaw change (rad)", "turn_signed_yaw_change_rad"),
                           ("Goal reached", "goal_reached")):
            stream.write(f"| {label} | {b[key]} | {c[key]} |\n")
        lane_result = report["comparison"]["lane"]
        stream.write("\nThe separate lane gate projects the recorded CARLA actor pose onto the same "
                     "active OpenDRIVE route. Its frame range may differ from the base_link camera "
                     "anchor window. It tests full actor polygons at 20 Hz and bounded "
                     "snapshot-linear sweeps against the sampled raw lane and a 10 cm inset.\n\n")
        stream.write("| Lane measure | B | C |\n| --- | ---: | ---: |\n")
        for label, key in (("Lane gate status", "status"),
                           ("Frames checked", "frames_checked"),
                           ("Raw outside bodies", "body_raw_sampled_lane_violation_count"),
                           ("Inset outside bodies", "body_inset_10cm_violation_count"),
                           ("Raw outside sweeps", "sweep_raw_sampled_lane_violation_count"),
                           ("Inset outside sweeps", "sweep_inset_10cm_violation_count"),
                           ("Worst raw body outside (m²)", "maximum_body_raw_sampled_lane_outside_m2"),
                           ("Worst inset body outside (m²)", "maximum_body_outside_m2"),
                           ("Worst raw sweep outside (m²)", "maximum_sweep_raw_sampled_lane_outside_m2"),
                           ("Worst inset sweep outside (m²)", "maximum_sweep_outside_m2")):
            values = (lane_result["status_b"], lane_result["status_c"]) if key == "status" else \
                     (lane_result["b"][key], lane_result["c"][key])
            stream.write(f"| {label} | {values[0]} | {values[1]} |\n")
        stream.write("\nC failed checks: " + (", ".join(report["failed_checks"]) or "none") + ".\n\n")
        stream.write("All selected anchors, each real 64-step future and the unchanged original nine "
                     "motion bounds are retained in the companion JSONL/summary. "
                     "The lane gate does not prove exact physical continuous time, and no result "
                     "here admits data or proves learned/Autoware driving.\n")


def main() -> int:
    require((TRIAL_C / "owner_result.json").is_file() and
            (TRIAL_C / "lifecycle/stopped.json").is_file(),
            "C owner is still running; no QA output created")
    try:
        report, companions = audit()
    except Exception as error:
        report, companions = {"schema": "carla.town01_right_purge15_bc_raw_qa.v1",
            "status": "FAIL_CLOSED", "dataset_admitted": False,
            "training_started": False, "learned_control": False,
            "comparison": None, "c_metrics": None, "lane_gate": None,
            "preregistered_plan_sha256": PLAN_SHA,
            "fatal_check": type(error).__name__, "fatal_detail": str(error)}, {}
    write_report(report, companions)
    print(json.dumps({"status": report["status"], "dataset_admitted": False,
                      "output": str(OUTPUT), "failed_checks": report.get("failed_checks", []),
                      "fatal_check": report.get("fatal_check")}))
    return 0 if report["status"] == "NECESSARY_CHECKS_CLEAR_NOT_ADMITTED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
